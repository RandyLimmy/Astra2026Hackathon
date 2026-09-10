"""Integration regressions for operator predictions, exports and retained-state runs."""
import argparse
import contextlib
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from simulator.platforms import catalog, operator


class PlatformOperatorTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)

    def args(self, **kwargs):
        values = dict(scenario='drone_hover', output=self.root / 'result', config=None,
                      duration=.1, timestep=None, fault_at=None, probe='hover',
                      frames=False, camera='chase', fps=12)
        values.update(kwargs)
        return argparse.Namespace(**values)

    def test_prediction_file_is_locked_before_actual_constructor(self):
        args = self.args()
        original = catalog.create
        calls = []

        def create(preset, values=None, *, healthy=False):
            calls.append(healthy)
            if not healthy:
                prediction = (args.output / 'prediction.json').read_bytes()
                digest = (args.output / 'prediction.sha256').read_text().strip()
                self.assertEqual(hashlib.sha256(prediction).hexdigest(), digest)
            return original(preset, values, healthy=healthy)

        with patch.object(catalog, 'create', side_effect=create), contextlib.redirect_stdout(io.StringIO()):
            operator.compare(args)
        self.assertEqual(calls, [True, False])
        report = json.loads((args.output / 'comparison.json').read_text())
        self.assertLess(report['max_position_error_m'], 1e-10)
        self.assertEqual(report['prediction_sha256'], hashlib.sha256(
            (args.output / 'prediction.json').read_bytes()).hexdigest())
        with self.assertRaises(FileExistsError), contextlib.redirect_stdout(io.StringIO()):
            operator.compare(args)

    def test_run_exports_only_observable_fields(self):
        sim = catalog.create('drone_wind', {'duration': .1, 'fault_at': .02, 'probe': 'hover'})
        summary, trace = operator.execute(sim, self.root / 'run')
        self.assertGreater(len(trace['time']), 2)
        for path in (self.root / 'run/public').glob('*.json*'):
            text = path.read_text()
            for private in ('"fault"', '"fault_at"', '"events"', '"voltage_ratio"', '"rotor_effectiveness"'):
                self.assertNotIn(private, text)
        private = json.loads((self.root / 'run/private/summary.json').read_text())
        self.assertEqual(private['config']['fault'], 'wind')
        self.assertEqual(summary['public'], json.loads((self.root / 'run/public/summary.json').read_text()))

    def test_retained_trial_can_be_recorded_with_monotonic_clock(self):
        sim = catalog.create('quadruped_joint_weakness', {'duration': .06, 'fault_at': .02})
        operator.execute(sim, self.root / 'first')
        old_time = sim.elapsed
        before = sim.model.actuator_gainprm.copy()
        sim.reset_trial()
        self.assertFalse(sim.finished)
        operator.execute(sim, self.root / 'second')
        rows = [json.loads(line) for line in (self.root / 'second/public/observations.jsonl').read_text().splitlines()]
        self.assertGreaterEqual(rows[0]['time'], old_time)
        self.assertGreater(rows[-1]['time'], old_time)
        np.testing.assert_array_equal(before, sim.model.actuator_gainprm)

    def test_nominal_export_compiles_without_source_assets(self):
        args = self.args(scenario='car_tire_pressure')
        with contextlib.redirect_stdout(io.StringIO()):
            operator.export(args)
        exported = mujoco.MjModel.from_xml_path(str(args.output / 'model.xml'))
        reference = catalog.create('car_tire_pressure', healthy=True)
        np.testing.assert_allclose(exported.body_mass, reference.model.body_mass)
        for key in ('fault', 'fault_at', 'pressure_radius_scale'):
            self.assertNotIn(key, json.loads((args.output / 'observation-example.json').read_text()))
        with self.assertRaises(FileExistsError):
            operator.export(args)

    def test_new_platform_rejects_legacy_physics_overrides(self):
        with self.assertRaisesRegex(ValueError, 'legacy car'):
            operator.overrides(self.args(initial_speed=20))
        with self.assertRaisesRegex(ValueError, 'Unknown'):
            catalog.create('drone_hover', {'unknown_parameter': 1})


if __name__ == '__main__':
    unittest.main()
