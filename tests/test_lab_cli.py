"""Intervention, prediction-lock, and recording-boundary integration tests."""

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

from simulator import lab
from simulator import __main__ as simulator_cli


class LabTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)

    def controls(self, value, duration=4.0):
        path = self.root / "controls.json"
        path.write_text(json.dumps(value))
        return lab.load_controls(path, duration)

    def test_schedule_validation(self):
        for value in ({}, [], [[1, {"left": 1}]], [[0, {}]], [[0, []]],
                      [[True, {"left": 1}]], [[float("nan"), {"left": 1}]],
                      [[0, {"left": float("inf")}]],
                      [[0, {"left": 1}], [0, {"left": 0}]],
                      [[0, {"left": 1}], [4, {"left": 0}]]):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.controls(value)
        self.assertEqual(self.controls([[0, {"left": .2}], [1.1, {"left": 0}]]),
                         [(0, {"left": .2}), (1.1, {"left": 0})])
        self.assertIsNone(lab.load_controls(None, 4))

    def test_prediction_exists_before_actual_run_and_hashes_exact_file(self):
        output = self.root / "comparison"
        original = lab.record_trial
        original_create = lab.create
        calls = []

        def create_after_lock(scenario, overrides=None, *, healthy=False):
            if not healthy:
                self.assertTrue((output / "prediction.json").is_file())
                self.assertTrue((output / "prediction.sha256").is_file())
            return original_create(scenario, overrides, healthy=healthy)

        def inspect_order(sim, path, **options):
            if path.name == "actual":
                self.assertTrue((output / "prediction.json").is_file())
                self.assertTrue((output / "prediction.sha256").is_file())
            calls.append(path.name)
            return original(sim, path, **options)

        schedule = self.controls([[0, {"left": .25, "right": .25}],
                                  [.4, {"left": 0, "right": 0}]])
        with (patch.object(lab, "record_trial", side_effect=inspect_order),
              patch.object(lab, "create", side_effect=create_after_lock)):
            report = lab.compare("warehouse_healthy", {"duration": .8}, output, controls=schedule)
        self.assertEqual(calls, ["nominal", "actual"])
        digest = hashlib.sha256((output / "prediction.json").read_bytes()).hexdigest()
        self.assertEqual(report["prediction_sha256"], digest)
        self.assertEqual((output / "prediction.sha256").read_text().strip(), digest)
        self.assertEqual(report["comparison"], "fixed command schedule")
        self.assertLess(report["position_rmse_m"], 1e-10)
        self.assertGreater(report["common_samples"], 20)
        original_bytes = (output / "prediction.json").read_bytes()
        with self.assertRaises(FileExistsError):
            lab.compare("warehouse_healthy", {"duration": .8}, output)
        self.assertEqual((output / "prediction.json").read_bytes(), original_bytes)

    def test_recorded_commands_follow_schedule_and_separate_private_data(self):
        schedule = self.controls([[0, {"left": .1, "right": .1}],
                                  [.2, {"left": .3, "right": .1}]])
        sim = lab.create("warehouse_healthy", {"duration": .5})
        output = self.root / "run"
        lab.record_trial(sim, output, controls=schedule)
        rows = [json.loads(line) for line in (output / "public/observations.jsonl").read_text().splitlines()]
        times = [row["time"] for row in rows]
        self.assertTrue(all(b > a for a, b in zip(times, times[1:])))
        self.assertGreater(len(rows), 20)
        early = next(row for row in rows if .09 < row["time"] < .11)
        late = next(row for row in rows if .29 < row["time"] < .31)
        self.assertEqual(early["command"], {"left": .1, "right": .1})
        self.assertEqual(late["command"], {"left": .3, "right": .1})
        for path in (output / "public").glob("*.json*"):
            content = path.read_text()
            for private_name in ('"fault"', '"fault_at"', '"events"', '"interventions"',
                                 '"motor_scale"', '"added_mass"'):
                self.assertNotIn(private_name, content)
        diagnostic = json.loads((output / "private/summary.json").read_text())
        self.assertEqual(diagnostic["interventions"], [[0, {"left": .1, "right": .1}],
                                                     [.2, {"left": .3, "right": .1}]])

    def test_export_is_healthy_and_reloadable(self):
        for scenario in ("warehouse_battery", "drone_rotor_loss"):
            with self.subTest(scenario=scenario):
                output = self.root / scenario
                lab.export(scenario, output)
                exported = mujoco.MjModel.from_xml_path(str(output / "model.xml"))
                expected = lab.create(scenario, healthy=True)
                np.testing.assert_allclose(exported.body_mass, expected.model.body_mass)
                np.testing.assert_allclose(exported.geom_friction, expected.model.geom_friction)
                self.assertEqual({p.name for p in output.iterdir()},
                                 {"model.xml", "observation-example.json", "README.md"})
                with self.assertRaises(FileExistsError):
                    lab.export(scenario, output)

    def test_cli_config_override_and_invalid_arguments(self):
        config = self.root / "config.json"
        config.write_text(json.dumps({"config": {"duration": .9}}))
        output = self.root / "cli"
        with contextlib.redirect_stdout(io.StringIO()):
            lab.main(["run", "drone_hover", "--config", str(config),
                      "--duration", ".3", "--output", str(output)])
        summary = json.loads((output / "private/summary.json").read_text())
        self.assertEqual(summary["config"]["duration"], .3)
        for args in (["run", "missing"], ["run", "drone_hover", "--duration", "nan"],
                     ["run", "warehouse_healthy", "--fps", "0"],
                     ["run", "drone_hover", "--output", str(output)]):
            with self.subTest(args=args), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as caught:
                    lab.main(args)
                self.assertEqual(caught.exception.code, 2)

    def test_shared_cli_keeps_platform_compare_and_both_export_contracts(self):
        output = self.root / "shared-cli"
        with contextlib.redirect_stdout(io.StringIO()):
            simulator_cli.main(["compare", "warehouse_healthy", "--duration", ".2",
                                "--output", str(output)])
        report = json.loads((output / "comparison.json").read_text())
        self.assertEqual(report["actual"]["platform"], "warehouse")
        self.assertEqual(report["position_rmse_m"], 0)
        for command in ("export-task", "export-baseline"):
            self.assertEqual(simulator_cli._parser().parse_args([command, "destination"]).action, command)
        parsed = simulator_cli._parser().parse_args(["export-platform", "drone_hover", "destination"])
        self.assertEqual(parsed.action, "export-platform")
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as caught:
            simulator_cli.main(["compare", "warehouse_healthy", "--candidate", "candidate/actuator.py"])
        self.assertEqual(caught.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
