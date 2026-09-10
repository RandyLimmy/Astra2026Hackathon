import assert from 'node:assert/strict';
import test from 'node:test';
import { TASKS, batchComparisonId, isCurrentTask, taskCatalog, recordedFrames, clipDuration, outcomePresentation, physicalOutcome, replayFor } from './paired.js';
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
  assert.equal(physicalOutcome({ goal_achieved: true }), 'Task completed');
  assert.equal(physicalOutcome({ goal_achieved: false }), 'Task failed');
  assert.equal(physicalOutcome({ goal_achieved: false, partial_success: true }), 'Partial success');
  assert.equal(physicalOutcome({ goal_achieved: false, aggregate: { partial_success: true } }), 'Partial success');
  assert.equal(physicalOutcome(null, true), 'Verification pending');
});

test('verified task status is primary while a recorded timeout and contact-step failure remain explicit', () => {
  const result = outcomePresentation({ outcome: 'mission_timeout', metrics: { in_flight_body_contacts: 9 } },
    { goal_achieved: false, partial_success: true });
  assert.equal(result.label, 'Partial success');
  assert.equal(result.termination, 'mission timeout');
  assert.equal(result.unmetCriterion, 'In-flight contact criterion unmet: 9 recorded contact steps.');
  assert.equal(outcomePresentation({ outcome: 'mission_complete' }, null).label, 'Not verified');
});

test('a completed scoring recheck displays current verified success without importing the old failure', () => {
  const reassessment = { original_outcome: { goal_achieved: false, partial_success: true }, original_termination: 'mission_timeout' };
  const result = outcomePresentation({ outcome: 'mission_complete', metrics: { in_flight_body_contacts: 0 } },
    { goal_achieved: true, partial_success: false });
  assert.equal(result.label, 'Task completed');
  assert.equal(result.termination, '');
  assert.equal(result.unmetCriterion, '');
  assert.equal(physicalOutcome(reassessment.original_outcome), 'Partial success');
});

test('the testing catalog and recordings contain only the four approved task identities', () => {
  const catalog = taskCatalog([
    { id: 'legacy', scenarios: [{ id: 'car_demo' }] },
    ...TASKS.map(task => ({ id: task.platform, scenarios: [{ id: `${task.platform}_demo` }, { id: task.scenario }] })),
  ]);
  assert.deepEqual(catalog.map(item => item.default_scenario), TASKS.map(item => item.scenario));
  assert.ok(catalog.every(item => item.scenarios.length === 1));
  for (const task of TASKS) {
    assert.equal(isCurrentTask(task), true);
    assert.equal(isCurrentTask({ manifest: task }), true);
  }
  assert.equal(isCurrentTask({ platform: 'drone', scenario: 'drone_rotor_loss' }), false);
  assert.equal(isCurrentTask({ platform: 'car', scenario: 'car_demo' }), false);
  assert.equal(isCurrentTask({ platform: 'quadruped', scenario: 'drone_delivery_imbalance' }), false);
  assert.equal(batchComparisonId({ comparisons: { drone: 'batch-drone', warehouse: { id: 'batch-warehouse' } } }, 'drone'), 'batch-drone');
  assert.equal(batchComparisonId(null, 'quadruped'), null);
});

test('the matched original replay comes from fresh verification rather than an earlier diagnostic', () => {
  const initial = { id: 'early', kind: 'before', probe: 'hover', record_path: 'physics/early/record.json' };
  const final = { id: 'verified', kind: 'before', probe: 'hover', record_path: 'broker/verification/final/record.json' };
  assert.equal(replayFor({ story: { replays: [initial, final] } }, 'before', 'hover'), final);
  assert.equal(replayFor({ story: { replays: [initial] } }, 'after', 'hover'), null);
});
