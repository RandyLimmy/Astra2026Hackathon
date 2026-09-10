export async function request(path, options = {}) {
  const response = await fetch(path, {
    ...options,
    headers: { ...(options.body ? { 'Content-Type': 'application/json' } : {}), ...options.headers },
  });
  let body;
  try {
    body = await response.json();
  } catch {
    throw new Error(response.ok ? 'The server returned an unreadable response.' : `Request failed (${response.status}).`);
  }
  if (!response.ok) throw new Error(body.error || `Request failed (${response.status}).`);
  return body;
}

export const runPath = id => `/api/runs/${encodeURIComponent(id)}`;

export function getSummary(value) {
  return value?.summary ?? value ?? {};
}

export function numeric(value) {
  return typeof value === 'number' && Number.isFinite(value);
}

export function meters(value, digits = 2) {
  return numeric(value) ? `${value.toFixed(digits)} m` : '—';
}

export function modelName(value) {
  if (value?.includes('astra')) return 'Astra';
  if (value?.includes('sol')) return 'Sol';
  return value || 'Model not recorded';
}

export function runLabel(run) {
  const metadata = run.metadata ?? {};
  const name = `${modelName(metadata.model)} / ${metadata.reasoning_effort || 'effort not recorded'}`;
  const date = metadata.start_at && new Date(metadata.start_at);
  const time = date && !Number.isNaN(date.valueOf())
    ? date.toLocaleString([], { month: 'short', day: 'numeric', hour: 'numeric', minute: '2-digit' }) : run.id;
  return `${name} — ${time}`;
}

export function caseName(caseData, index = 0) {
  const config = caseData.config ?? {};
  const cycles = config.preparation_cycles;
  const wait = config.wait_s;
  if (!numeric(cycles)) return caseData.case_id || `Case ${index + 1}`;
  let label = cycles === 0 ? 'Fresh brakes' : `${cycles} braking ${cycles === 1 ? 'cycle' : 'cycles'}`;
  if (wait > 0) label += ` + ${wait} s rest`;
  return label;
}

export function elapsedLabel(metadata = {}) {
  let seconds = metadata.duration_s;
  if (!numeric(seconds) && metadata.start_at && metadata.end_at) {
    seconds = (Date.parse(metadata.end_at) - Date.parse(metadata.start_at)) / 1000;
  }
  if (!numeric(seconds)) return null;
  return seconds >= 60 ? `${Math.floor(seconds / 60)}m ${Math.round(seconds % 60)}s` : `${Math.round(seconds)}s`;
}

export function verdict(aggregate, status) {
  if (aggregate?.predictive_success === true) return { text: 'Passed', className: 'text-pass' };
  if (aggregate?.predictive_success === false) return { text: 'Failed', className: 'text-fail' };
  if (['api_error', 'error', 'evaluation_error', 'interrupted'].includes(status)) return { text: 'Incomplete', className: 'text-fail' };
  if (['running', 'starting', 'evaluating'].includes(status)) return { text: status === 'evaluating' ? 'Evaluating' : 'In progress', className: '' };
  return { text: aggregate ? 'Not fully scored' : 'Not evaluated', className: 'muted' };
}
