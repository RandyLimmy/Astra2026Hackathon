import { useState } from 'react';
import { AGENTS, actionStatus, formatValue, metadataOf, readable, summaryOf } from '../paired.js';
import { Chevron } from './Icons.jsx';
import { SectionTitle } from './OriginalScenario.jsx';

const STAGE_LABELS = { attempted: 'Action attempted', applied: 'Change applied', unchanged: 'Controller unchanged', rejected: 'Action rejected', verified: 'Check completed', failed: 'Check failed', proposed: 'Proposed only', completed: 'Completed' };
const KIND_LABELS = { controller_edit: 'Controller change', repair: 'Physical intervention', model_edit: 'Predictive model change', inspection: 'Inspection', experiment: 'Task attempt', verification: 'Task verification', prediction: 'Prediction experiment', submission: 'Submission' };

export function DataDetails({ label, value, diff = false }) {
  const [open, setOpen] = useState(false);
  if (value == null) return null;
  const text = typeof value === 'string' ? value : JSON.stringify(value, null, 2);
  return <details className="data-details" onToggle={event => setOpen(event.currentTarget.open)}><summary>{label}<Chevron size={13} /></summary>{open && <pre>{diff ? text.split('\n').map((line, i) => <span key={i} className={line.startsWith('+') && !line.startsWith('+++') ? 'diff-added' : line.startsWith('-') && !line.startsWith('---') ? 'diff-removed' : ''}>{line}{'\n'}</span>) : text}</pre>}</details>;
}

function Checkpoint({ checkpoint, index }) {
  const [open, setOpen] = useState(['controller_edit', 'repair', 'model_edit'].includes(checkpoint.kind));
  const failed = ['rejected', 'failed'].includes(checkpoint.stage);
  return <li className={`checkpoint ${open ? 'checkpoint-open' : ''}`}>
    <span className="checkpoint-number">{index + 1}</span>
    <details open={open} onToggle={event => setOpen(event.currentTarget.open)}><summary><span>{checkpoint.title || readable(checkpoint.tool)}<small className={failed ? 'text-fail' : ''}>{STAGE_LABELS[checkpoint.stage] || readable(checkpoint.stage)}</small></span><Chevron /></summary>
      {open && <div className="checkpoint-content"><dl>
        <dt>Type</dt><dd>{KIND_LABELS[checkpoint.kind] || readable(checkpoint.kind)}</dd>
        {checkpoint.action && <><dt>Action</dt><dd>{readable(checkpoint.action)}</dd></>}
        {checkpoint.target && <><dt>Target</dt><dd className="literal-value">{checkpoint.target}</dd></>}
        {checkpoint.reason && <><dt>Stated reason</dt><dd>{checkpoint.reason}</dd></>}
        {checkpoint.expected_effect && <><dt>Expected effect</dt><dd>{checkpoint.expected_effect}</dd></>}
        <dt>Status</dt><dd className={failed ? 'text-fail' : ''}>{STAGE_LABELS[checkpoint.stage] || readable(checkpoint.stage)}{checkpoint.stage === 'applied' && ' · outcome requires verification'}</dd>
      </dl>
      {checkpoint.changes?.length > 0 && <div className="table-scroll"><table className="changes-table"><caption>Exact recorded adjustments</caption><thead><tr><th>Parameter</th><th>Before</th><th>After</th></tr></thead><tbody>{checkpoint.changes.map((change, i) => <tr key={i}><td>{change.parameter}</td><td>{formatValue(change.before)}</td><td>{formatValue(change.after)}</td></tr>)}</tbody></table></div>}
      {checkpoint.source_diff && <DataDetails label={checkpoint.kind === 'controller_edit' ? 'Exact controller diff' : 'Exact source diff'} value={checkpoint.source_diff} diff />}
      {checkpoint.error && <p className="checkpoint-error text-fail">{typeof checkpoint.error === 'string' ? checkpoint.error : formatValue(checkpoint.error)}</p>}
      <DataDetails label="Recorded tool result" value={checkpoint.result} />
      {checkpoint.timestamp && <p className="footnote">Recorded {new Date(checkpoint.timestamp).toLocaleTimeString()}</p>}
      </div>}
    </details>
  </li>;
}

function AgentTrack({ agent, run, profile, comparisonActive }) {
  const summary = summaryOf(run);
  const metadata = metadataOf(run);
  const checkpoints = run?.story?.checkpoints ?? [];
  const active = summary.active || ['running', 'starting', 'evaluating'].includes(summary.status ?? metadata.status);
  const status = summary.status || metadata.status;
  const error = run?.error || summary.error || (['api_error', 'error', 'evaluation_error', 'interrupted'].includes(status) ? run?.story?.events?.findLast(event => event.type === 'error')?.message : null);
  const explanation = run?.story?.explanations ?? [];
  return <article className={`agent-track ${agent.id}`}>
    <div className="agent-heading"><h3><span className="agent-symbol" aria-hidden="true">{agent.id === 'astra' ? '✦' : '✣'}</span>{agent.label}<span className="model-effort">/ {metadata.reasoning_effort || profile?.reasoning_effort || 'max'}</span></h3><span className={`track-status ${active ? 'is-running' : ''}`}>{active ? 'Running' : readable(status) || (comparisonActive ? 'Starting' : 'Ready')}</span></div>
    <p className="agent-action-status" role="status">{actionStatus(run)}</p>
    {error && <p className="track-error" role="alert">{typeof error === 'string' ? error : error.message || 'The investigation could not complete.'}</p>}
    {checkpoints.length ? <ol className="checkpoint-list">{checkpoints.map((checkpoint, index) => <Checkpoint key={checkpoint.id || `${checkpoint.tool}-${index}`} checkpoint={checkpoint} index={index} />)}</ol> : <div className="track-empty"><strong>{active || comparisonActive ? 'Preparing the investigation' : 'No changes recorded yet'}</strong><p>{active || comparisonActive ? 'Actions and exact adjustments appear here as this model works.' : `${agent.label} will inspect the scenario, attempt justified fixes, and check the result.`}</p></div>}
    {explanation.length > 0 && <div className="recorded-explanations"><h4>Recorded statements</h4><p className="footnote">Model statements are separate from executed actions.</p>{explanation.map((entry, index) => <DataDetails key={index} label={`Statement ${index + 1}`} value={entry.text} />)}</div>}
    {run?.story?.source?.diff && <DataDetails label="Complete controller diff" value={run.story.source.diff} diff />}
    {summary.latest_status && <p className="footnote">{summary.latest_status}</p>}
    {metadata.stop_reason && <p className="footnote">Stopped: {readable(metadata.stop_reason)}</p>}
  </article>;
}

export default function AgentTracks({ comparison, profiles = {} }) {
  return <section className="story-section" aria-labelledby="changes-heading"><SectionTitle number="02"><span id="changes-heading">What changed</span></SectionTitle><div className="agent-columns">{AGENTS.map(agent => <AgentTrack key={`${comparison?.id || 'new'}-${agent.id}`} agent={agent} run={comparison?.runs?.[agent.id]} profile={profiles[agent.id]} comparisonActive={comparison?.active} />)}</div><p className="section-note">Each checkpoint records the exact controller adjustment, its stated reason, and the measured result. A proposed fix, an applied edit, and a completed task have separate statuses.</p></section>;
}
