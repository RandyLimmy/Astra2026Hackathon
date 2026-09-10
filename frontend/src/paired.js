import { numeric } from './api.js';

export const AGENTS = [{ id: 'astra', label: 'Astra' }, { id: 'sol', label: 'Sol' }];
export const TASKS = [
  { platform: 'quadruped', scenario: 'quadruped_gait_failure', label: 'Robot dog' },
  { platform: 'drone', scenario: 'drone_delivery_imbalance', label: 'Drone' },
  { platform: 'warehouse', scenario: 'warehouse_curve_demo', label: 'Warehouse train' },
  { platform: 'car', scenario: 'car_auto_brake_failure', label: 'Car braking' },
];
export function isCurrentTask(record) {
  const source = record?.manifest ?? record;
  return TASKS.some(task => task.scenario === source?.scenario && task.platform === source?.platform);
}
export function taskCatalog(platforms = []) {
  return TASKS.flatMap(task => {
    const platform = platforms.find(item => item.id === task.platform);
    const scenario = platform?.scenarios?.find(item => item.id === task.scenario);
    return platform && scenario ? [{ ...platform, label: task.label, default_scenario: task.scenario, scenarios: [scenario] }] : [];
  });
}
export function batchComparisonId(batch, platform) {
  const entry = batch?.comparisons?.[platform];
  return typeof entry === 'string' ? entry : entry?.id || entry?.comparison_id || null;
}
export const comparisonPath = id => `/api/comparisons/${encodeURIComponent(id)}`;
export const readable = value => typeof value === 'string' ? value.replaceAll('_', ' ') : '';
export const formatValue = value => value == null ? 'Not recorded' : typeof value === 'object' ? JSON.stringify(value) : String(value);
export const count = value => numeric(value) ? value.toLocaleString() : '—';
export const yesNo = value => value === true ? 'Yes' : value === false ? 'No' : 'Not verified';
export const goalOutcome = value => value === true ? 'Task completed' : value === false ? 'Task failed' : 'Not verified';
export function physicalOutcome(verification, active = false) {
  if (verification?.goal_achieved === true) return 'Task completed';
  const outcomes = (verification?.cases ?? []).map(item => item.goal_achieved);
  if (verification?.goal_achieved === false && (verification.partial_success === true
    || verification.aggregate?.partial_success === true || outcomes.some(value => value === true))) return 'Partial success';
  if (verification?.goal_achieved === false) return 'Task failed';
  return active ? 'Verification pending' : 'Not verified';
}
export function outcomePresentation(summary, verification, active = false) {
  const raw = summary?.outcome;
  const contacts = summary?.metrics?.in_flight_body_contacts;
  return {
    label: physicalOutcome(verification, active),
    termination: raw && !['mission_complete', 'goal_achieved', 'task_complete'].includes(raw) ? readable(raw) : '',
    unmetCriterion: verification?.goal_achieved === false && Number.isInteger(contacts) && contacts > 0
      ? `In-flight contact criterion unmet: ${contacts.toLocaleString()} recorded contact step${contacts === 1 ? '' : 's'}.` : '',
  };
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
  return candidates.find(replay => replay.record_path?.startsWith('broker/verification/')) ?? candidates.at(-1) ?? null;
}
export function actionStatus(run) {
  const action = run?.story?.action_summary;
  if (action?.status) return readable(action.status);
  if (action?.change_applied === true) return 'Change applied; verification pending';
  if (action?.fix_attempted === true) return 'Fix attempted';
  if (action?.fix_attempted === false && run && !summaryOf(run).active) return 'No fix attempted';
  return summaryOf(run).active ? 'Investigation in progress' : 'Waiting for an investigation';
}
