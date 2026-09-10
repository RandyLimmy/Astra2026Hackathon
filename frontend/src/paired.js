import { numeric } from './api.js';

export const AGENTS = [{ id: 'astra', label: 'Astra' }, { id: 'sol', label: 'Sol' }];
export const comparisonPath = id => `/api/comparisons/${encodeURIComponent(id)}`;
export const readable = value => typeof value === 'string' ? value.replaceAll('_', ' ') : '';
export const formatValue = value => value == null ? 'Not recorded' : typeof value === 'object' ? JSON.stringify(value) : String(value);
export const count = value => numeric(value) ? value.toLocaleString() : '—';
export const yesNo = value => value === true ? 'Yes' : value === false ? 'No' : 'Not verified';
export const goalOutcome = value => value === true ? 'Goal achieved' : value === false ? 'Goal not achieved' : 'Not verified';
export function physicalOutcome(verification, active = false) {
  if (verification?.goal_achieved === true) return 'Goal achieved';
  const outcomes = (verification?.cases ?? []).map(item => item.goal_achieved);
  if (verification?.goal_achieved === false && outcomes.some(value => value === true)) return 'Partial success';
  if (verification?.goal_achieved === false) return 'Goal not achieved';
  return active ? 'Verification pending' : 'Not verified';
}
export const summaryOf = run => run?.summary ?? {};
export const metadataOf = run => summaryOf(run).metadata ?? summaryOf(run);
export function durationLabel(value) {
  if (!numeric(value)) return '—';
  return value >= 60 ? `${Math.floor(value / 60)}m ${Math.round(value % 60)}s` : `${value.toFixed(1)} s`;
}
export function comparisonLabel(record) {
  const source = record.manifest ?? record;
  const date = source.start_at || source.created_at || record.start_at;
  const parsed = date && new Date(date);
  return `${readable(source.scenario || record.scenario || record.id)}${parsed && !Number.isNaN(parsed.valueOf()) ? ` · ${parsed.toLocaleString([], { month: 'short', day: 'numeric', hour: 'numeric', minute: '2-digit' })}` : ''}${record.active ? ' · running' : ''}`;
}
export function recordedFrames(replay) {
  return (Array.isArray(replay?.frames) ? replay.frames : []).filter(frame => numeric(frame?.t_s)
    && frame.t_s >= 0 && typeof frame.url === 'string' && frame.url.startsWith('/api/')
    && !/[\\\r\n]/.test(frame.url)).sort((a, b) => a.t_s - b.t_s);
}
export function clipDuration(replay) {
  const frames = recordedFrames(replay);
  return frames.length ? Math.max(frames.at(-1).t_s, numeric(replay.duration_s) ? replay.duration_s : 0) : 0;
}
export function replayFor(run, kind, probe) {
  const candidates = (run?.story?.replays ?? []).filter(replay => replay.kind === kind && (!probe || replay.probe === probe));
  return candidates.find(replay => replay.record_path?.startsWith('broker/verification/')) ?? candidates[0] ?? null;
}
export function actionStatus(run) {
  const action = run?.story?.action_summary;
  if (action?.status) return readable(action.status);
  if (action?.change_applied === true) return 'Change applied; verification pending';
  if (action?.fix_attempted === true) return 'Fix attempted';
  if (action?.fix_attempted === false && run && !summaryOf(run).active) return 'No fix attempted';
  return summaryOf(run).active ? 'Investigation in progress' : 'Waiting for an investigation';
}
