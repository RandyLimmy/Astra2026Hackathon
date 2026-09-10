export function frameIndexAt(frames, time) {
  if (!frames.length) return -1;
  let low = 0;
  let high = frames.length - 1;
  while (low <= high) {
    const middle = Math.floor((low + high) / 2);
    if (frames[middle].t_s <= time + 1e-8) low = middle + 1;
    else high = middle - 1;
  }
  return Math.max(0, high);
}

export function stepFrame(frames, time, direction) {
  if (!frames.length) return 0;
  const index = frameIndexAt(frames, time);
  const betweenFrames = time > frames[index].t_s + 1e-8;
  const next = direction < 0 && betweenFrames ? index : index + direction;
  return frames[Math.max(0, Math.min(frames.length - 1, next))].t_s;
}

export function eventTime(event) {
  return Number.isFinite(event.time) ? event.time : event.t_s;
}

export function firstFailure(events) {
  const ordered = events.filter(event => Number.isFinite(eventTime(event))).slice()
    .sort((a, b) => eventTime(a) - eventTime(b));
  return ordered.find(event => event.is_failure === true)
    || ordered.find(event => /fall|impact|support_loss|lost_balance|body_contact|stumble/.test(event.event || ''))
    || null;
}

export function failureWindow(event, duration) {
  if (!event || !(duration > 0)) return null;
  const center = Math.max(0, Math.min(duration, eventTime(event)));
  if (!Number.isFinite(center)) return null;
  return { start: Math.max(0, center - 2), end: Math.min(duration, center + 1) };
}

export function advanceTime(time, delta, duration, loop = null) {
  const next = time + Math.max(0, delta);
  if (loop && loop.end > loop.start) {
    const length = loop.end - loop.start;
    return { time: next >= loop.end ? loop.start + (next - loop.start) % length : Math.max(loop.start, next), finished: false };
  }
  return { time: Math.max(0, Math.min(duration, next)), finished: next >= duration };
}

export function seekTime(time, duration, precision = 0.001) {
  const bounded = Math.max(0, Math.min(duration, time));
  return duration - bounded <= precision ? duration : bounded;
}

export function attemptLabel(provenance) {
  if (provenance === 'developer_control') return 'Developer-authored reference';
  if (provenance === 'candidate_attempt') return 'Candidate attempt';
  if (provenance === 'original_attempt' || provenance === 'failure_demonstration') return 'Original attempt';
  return 'Recorded attempt';
}

export function clockLabel(time) {
  const value = Math.max(0, Number.isFinite(time) ? time : 0);
  return `${String(Math.floor(value / 60)).padStart(2, '0')}:${(value % 60).toFixed(2).padStart(5, '0')}`;
}
