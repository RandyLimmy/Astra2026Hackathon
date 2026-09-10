import assert from 'node:assert/strict';
import test from 'node:test';
import { recordedFrames, clipDuration, physicalOutcome, replayFor } from './paired.js';
import { frameAt } from './replay.js';

test('paired playback uses real elapsed time and retains short clips at their last recorded frame', () => {
  const replay = { duration_s: 10, frames: [
    { t_s: 2, url: '/api/runs/astra/platform-media/physics/run_1/frames/f2.jpg' },
    { t_s: 0, url: '/api/runs/astra/platform-media/physics/run_1/frames/f0.jpg' },
    { t_s: 1, url: 'https://unrelated.example/frame.jpg' },
    { t_s: -1, url: '/api/invalid.jpg' },
  ] };
  const frames = recordedFrames(replay);
  assert.equal(frames.length, 2);
  assert.equal(clipDuration(replay), 10);
  assert.equal(frameAt(frames, 1).t_s, 0);
  assert.equal(frameAt(frames, 9).t_s, 2);
  assert.equal(clipDuration({ duration_s: 10, frames: [] }), 0);
});

test('physical outcomes preserve unknowns and partial success without promoting prediction accuracy', () => {
  assert.equal(physicalOutcome({ predictive_success: true }), 'Not verified');
  assert.equal(physicalOutcome({ goal_achieved: false, cases: [{ goal_achieved: true }, { goal_achieved: false }] }), 'Partial success');
  assert.equal(physicalOutcome({ goal_achieved: true }), 'Goal achieved');
  assert.equal(physicalOutcome({ goal_achieved: false }), 'Goal not achieved');
  assert.equal(physicalOutcome(null, true), 'Verification pending');
});

test('the matched original replay comes from fresh verification rather than an earlier diagnostic', () => {
  const initial = { id: 'early', kind: 'before', probe: 'hover', record_path: 'physics/early/record.json' };
  const final = { id: 'verified', kind: 'before', probe: 'hover', record_path: 'broker/verification/final/record.json' };
  assert.equal(replayFor({ story: { replays: [initial, final] } }, 'before', 'hover'), final);
  assert.equal(replayFor({ story: { replays: [initial] } }, 'after', 'hover'), null);
});
