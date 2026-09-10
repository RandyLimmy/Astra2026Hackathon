import test from 'node:test';
import assert from 'node:assert/strict';
import { advanceTime, attemptLabel, clockLabel, eventTime, failureWindow, firstFailure, frameIndexAt, seekTime, stepFrame } from './scenario-playback.mjs';

const frames = [{ t_s: 0 }, { t_s: 0.1 }, { t_s: 0.2 }, { t_s: 0.3 }];

test('recorded frames hold until their next timestamp and at completion', () => {
  assert.equal(frameIndexAt([], 0), -1);
  assert.equal(frameIndexAt(frames, -1), 0);
  assert.equal(frameIndexAt(frames, 0.19), 1);
  assert.equal(frameIndexAt(frames, 0.2), 2);
  assert.equal(frameIndexAt(frames, 9), 3);
});

test('single-frame stepping works at a frame and between frames, without wrapping', () => {
  assert.equal(stepFrame(frames, 0.2, -1), 0.1);
  assert.equal(stepFrame(frames, 0.25, -1), 0.2);
  assert.equal(stepFrame(frames, 0.25, 1), 0.3);
  assert.equal(stepFrame(frames, 0, -1), 0);
  assert.equal(stepFrame(frames, 0.3, 1), 0.3);
});

test('an explicitly marked failure takes precedence over named event fallbacks', () => {
  const events = [{ time: 2, event: 'stumble' }, { time: 6, event: 'impact', is_failure: true }, { time: 4, event: 'lost_balance', is_failure: true }];
  assert.equal(eventTime(firstFailure(events)), 4);
  assert.equal(eventTime(firstFailure([{ time: 5, event: 'impact' }, { t_s: 3, event: 'support_loss' }])), 3);
  assert.equal(firstFailure([{ time: 0, event: 'start' }]), null);
});

test('inspection shows two seconds before and one second after, clipped to the recording', () => {
  assert.deepEqual(failureWindow({ time: 4 }, 12), { start: 2, end: 5 });
  assert.deepEqual(failureWindow({ time: 0.5 }, 12), { start: 0, end: 1.5 });
  assert.deepEqual(failureWindow({ time: 11.5 }, 12), { start: 9.5, end: 12 });
  assert.equal(failureWindow(null, 12), null);
});

test('playback holds its final frame, whereas inspection wraps with elapsed time preserved', () => {
  assert.deepEqual(advanceTime(9.9, 0.4, 10), { time: 10, finished: true });
  assert.deepEqual(advanceTime(10, 1, 10), { time: 10, finished: true });
  assert.deepEqual(advanceTime(4.75, 0.5, 10, { start: 2, end: 5 }), { time: 2.25, finished: false });
  assert.deepEqual(advanceTime(4.75, 6.5, 10, { start: 2, end: 5 }), { time: 2.25, finished: false });
  assert.equal(clockLabel(62.25), '01:02.25');
});

test('a range input rounded to millisecond precision still reaches the exact recording end', () => {
  const duration = 16.00000000000201;
  assert.equal(seekTime(16, duration), duration);
  assert.equal(seekTime(15.9995, duration), duration);
  assert.equal(seekTime(15.998, duration), 15.998);
  assert.equal(seekTime(-1, duration), 0);
  assert.equal(seekTime(17, duration), duration);
});

test('recording labels distinguish original, developer, candidate, and unknown provenance', () => {
  assert.equal(attemptLabel('original_attempt'), 'Original attempt');
  assert.equal(attemptLabel('developer_control'), 'Developer-authored reference');
  assert.equal(attemptLabel('candidate_attempt'), 'Candidate attempt');
  assert.equal(attemptLabel(undefined), 'Recorded attempt');
});
