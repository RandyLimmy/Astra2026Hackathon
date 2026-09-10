import { getSummary, numeric } from './api.js';

const OUTCOME_FIELDS = ['stopped', 'collision', 'censored', 'lane_departure', 'stopping_distance',
  'trial_duration', 'impact_speed', 'final_front_x', 'final_speed'];

function sameValue(actual, expected) {
  if (numeric(actual) && numeric(expected)) {
    return Math.abs(actual - expected) <= Math.max(1e-7, Math.abs(expected) * 1e-8);
  }
  return actual === expected;
}

export function matchesSavedRecord(value, caseData, track) {
  if (!value || !caseData?.[track]) return false;
  const config = caseData[track].config ?? caseData.config ?? {};
  const summary = getSummary(caseData[track]);
  return Object.entries(config).every(([key, expected]) => sameValue(value.config?.[key], expected))
    && OUTCOME_FIELDS.filter(key => key in summary).every(key => sameValue(value.summary?.[key], summary[key]));
}

export function validTrace(value, caseData, track) {
  return value?.case_id === caseData?.case_id && value?.track === track
    && Array.isArray(value?.observations) && matchesSavedRecord(value, caseData, track);
}

export function validManifest(value, caseData, track) {
  return value?.kind === 'mujoco_rendered_replay' && value?.metrics_match === true
    && matchesSavedRecord(value, caseData, track)
    && (track === 'reference' ? value.source_sha256 === null
      : Boolean(caseData?.[track]?.source_sha256) && value.source_sha256 === caseData[track].source_sha256);
}

export function trialRows(trace) {
  const rows = Array.isArray(trace?.observations) ? trace.observations : [];
  return rows.filter(row => row?.phase === 'trial' && numeric(row.phase_time) && row.phase_time >= 0).map(row => ({
    t: row.phase_time,
    speed: Array.isArray(row.velocity) && row.velocity.length === 3 && row.velocity.every(numeric)
      ? Math.hypot(...row.velocity) : row.speed_mps,
    x: row.front_x,
  })).filter(row => numeric(row.speed) || numeric(row.x)).sort((a, b) => a.t - b.t);
}

export function replayFrames(manifest) {
  return (Array.isArray(manifest?.frames) ? manifest.frames : []).filter(frame => numeric(frame?.t_s)
    && frame.t_s >= 0 && /^frames\/[A-Za-z0-9_-]+\.(jpg|jpeg|png|webp)$/.test(frame.file || frame.url || ''))
    .sort((a, b) => a.t_s - b.t_s);
}

export function frameAt(frames, time) {
  let left = 0, right = frames.length - 1;
  while (left <= right) {
    const middle = Math.floor((left + right) / 2);
    if (frames[middle].t_s <= time) left = middle + 1;
    else right = middle - 1;
  }
  return frames[Math.max(0, right)] ?? null;
}
