import assert from 'node:assert/strict';
import test from 'node:test';
import { frameAt, replayFrames, trialRows, validManifest, validTrace } from './replay.js';

const config = { speed_mps: 22, preparation_cycles: 2, wait_s: 0, wall_distance_m: null };
const summary = { stopping_distance: 62.4, trial_duration: 7, stopped: true, collision: false, censored: false };
const record = { config, summary, source_sha256: 'frozen-candidate' };
const selected = { case_id: 'reserved_2', config, candidate: record, reference: { config, summary } };
const manifest = { kind: 'mujoco_rendered_replay', metrics_match: true, config, summary, source_sha256: 'frozen-candidate' };

test('only traces for the selected scenario, model, and saved result are accepted', () => {
  const trace = { case_id: 'reserved_2', track: 'candidate', config, summary, observations: [] };
  assert.equal(validTrace(trace, selected, 'candidate'), true);
  assert.equal(validTrace({ ...trace, case_id: 'reserved_1' }, selected, 'candidate'), false);
  assert.equal(validTrace({ ...trace, track: 'original' }, selected, 'candidate'), false);
  assert.equal(validTrace({ ...trace, config: { ...config, wait_s: 30 } }, selected, 'candidate'), false);
  assert.equal(validTrace({ ...trace, summary: { ...summary, stopping_distance: 50 } }, selected, 'candidate'), false);
});

test('replays require verified outcomes and the selected frozen model', () => {
  assert.equal(validManifest(manifest, selected, 'candidate'), true);
  assert.equal(validManifest({ ...manifest, source_sha256: null }, selected, 'reference'), true);
  assert.equal(validManifest(manifest, selected, 'reference'), false);
  assert.equal(validManifest({ ...manifest, source_sha256: 'other-model' }, selected, 'candidate'), false);
  assert.equal(validManifest({ ...manifest, metrics_match: false }, selected, 'candidate'), false);
  assert.equal(validManifest({ ...manifest, config: { ...config, preparation_cycles: 0 } }, selected, 'candidate'), false);
  assert.equal(validManifest({ ...manifest, summary: { ...summary, stopped: false } }, selected, 'candidate'), false);
});

test('telemetry uses trial time, physical velocity magnitude, and preserves position-only samples', () => {
  const rows = trialRows({ observations: [
    { phase: 'preparation', phase_time: 0, velocity: [99, 0, 0], front_x: 99 },
    { phase: 'trial', phase_time: 1, front_x: 7 },
    { phase: 'trial', phase_time: 0, time: 40, velocity: [3, 4, 0], front_x: 2 },
    { phase: 'trial', phase_time: -1, velocity: [99, 0, 0] },
    { phase: 'trial', phase_time: 2, velocity: [NaN, 0, 0] },
  ] });
  assert.deepEqual(rows, [{ t: 0, speed: 5, x: 2 }, { t: 1, speed: undefined, x: 7 }]);
});

test('seeking chooses the recorded frame at or before trial time, within the selected media directory', () => {
  const frames = replayFrames({ frames: [
    { t_s: 1, file: 'frames/frame_000002.jpg' },
    { t_s: 0, file: 'frames/frame_000000.jpg' },
    { t_s: 0.5, file: 'frames/frame_000001.jpg' },
    { t_s: 0.75, file: '../other-case/frame.jpg' },
    { t_s: NaN, file: 'frames/invalid.jpg' },
  ] });
  assert.equal(frames.length, 3);
  assert.equal(frameAt(frames, 0).t_s, 0);
  assert.equal(frameAt(frames, 0.9).t_s, 0.5);
  assert.equal(frameAt(frames, 1).t_s, 1);
  assert.equal(frameAt(frames, 9).t_s, 1);
  assert.equal(frameAt([], 0), null);
});
