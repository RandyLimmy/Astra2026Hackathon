import { useState } from 'react';
import { numeric } from '../api.js';
import { AGENTS, clipDuration, count, durationLabel, formatValue, metadataOf, outcomePresentation, physicalOutcome, readable, replayFor, summaryOf, yesNo } from '../paired.js';
import { usePlayback } from '../pairedHooks.js';
import { DataDetails } from './AgentTracks.jsx';
import { SectionTitle } from './OriginalScenario.jsx';
import { PlaybackControls, RecordedFrame } from './RecordedPlayback.jsx';

const outcomeClass = value => value === true ? 'text-pass' : value === false ? 'text-fail' : 'muted';
function metricValue(value) {
  if (numeric(value)) return Number.isInteger(value) ? count(value) : value.toFixed(3);
  if (typeof value === 'boolean') return yesNo(value);
  return value == null ? '—' : formatValue(value);
}
function costLabel(run) {
  const metadata = metadataOf(run);
  const cost = run?.story?.metrics?.cost ?? metadata.cost;
  const value = cost?.total_usd ?? cost?.estimated_usd ?? run?.story?.metrics?.estimated_cost_usd ?? metadata.estimated_cost_usd ?? (numeric(cost) ? cost : null);
  return numeric(value) ? `$${value.toFixed(4)}` : 'Not recorded';
}

function ReassessmentNote({ agent, reassessment }) {
  if (!reassessment) return null;
  const date = reassessment.evaluated_at && new Date(reassessment.evaluated_at);
  const original = physicalOutcome(reassessment.original_outcome);
  return <aside className="reassessment-note" role="note" aria-label={`${agent} controller reassessment`}>
    <strong>Controller rechecked after scoring fix — no new model call</strong>
    {reassessment.reason && <p>{reassessment.reason}</p>}
    <p>Previous recorded outcome: {original}{reassessment.original_termination ? ` (${readable(reassessment.original_termination)})` : ''}. The original recording is preserved.</p>
    {date && !Number.isNaN(date.valueOf()) && <time dateTime={reassessment.evaluated_at}>Rechecked {date.toLocaleString()}</time>}
  </aside>;
}

function MatchedResults({ comparison, probe }) {
  const runs = comparison?.runs ?? {};
  const original = replayFor(runs.astra, 'before', probe) || replayFor(runs.sol, 'before', probe);
  const clips = [{ id: 'original', label: 'Original', replay: original }, ...AGENTS.map(agent => ({ ...agent, replay: replayFor(runs[agent.id], 'after', probe) }))];
  const playback = usePlayback(Math.max(0, ...clips.map(clip => clipDuration(clip.replay))));
  const selectedCase = agent => runs[agent]?.story?.verification?.cases?.find(item => item.probe === probe);
  const stats = agent => runs[agent]?.story?.metrics ?? {};
  const actions = agent => runs[agent]?.story?.action_summary ?? {};
  const metadata = agent => metadataOf(runs[agent]);
  const baselineResult = original?.summary?.task_complete ?? original?.summary?.goal_achieved;
  const rows = [
    ['Original task completion', baselineResult != null ? physicalOutcome({ goal_achieved: baselineResult }) : '—', ...AGENTS.map(agent => physicalOutcome(runs[agent.id]?.story?.verification, summaryOf(runs[agent.id]).active))],
    ['Controller changes applied', '—', ...AGENTS.map(agent => count(actions(agent.id).controller_edits_applied))],
    ['Controller changes attempted', '—', ...AGENTS.map(agent => count(actions(agent.id).controller_edit_attempts))],
    ['Failed or rejected attempts', '—', ...AGENTS.map(agent => count(actions(agent.id).failed_attempts))],
    ['Investigation time', '—', ...AGENTS.map(agent => durationLabel(stats(agent.id).duration_s ?? metadata(agent.id).duration_s))],
    ['Tool calls', '—', ...AGENTS.map(agent => count(stats(agent.id).tool_calls ?? summaryOf(runs[agent.id]).tool_calls))],
    ['API requests', '—', ...AGENTS.map(agent => count(stats(agent.id).api_requests ?? summaryOf(runs[agent.id]).api_requests))],
    ['Total tokens', '—', ...AGENTS.map(agent => count((stats(agent.id).usage ?? metadata(agent.id).usage)?.total_tokens))],
    ['Estimated cost', '—', ...AGENTS.map(agent => costLabel(runs[agent.id]))],
  ];
  const metricNames = [...new Set(clips.flatMap(clip => Object.keys(clip.replay?.summary?.metrics ?? {})))];
  return <>
    <div className="result-players">{clips.map(clip => {
      const isOriginal = clip.id === 'original';
      const run = runs[clip.id];
      const verification = selectedCase(clip.id) ?? run?.story?.verification;
      const goal = isOriginal ? baselineResult : verification?.goal_achieved;
      const outcome = outcomePresentation(clip.replay?.summary, verification, summaryOf(run).active);
      return <article key={clip.id} className={`result-player ${clip.id}`}><h3>{clip.label}</h3><RecordedFrame replay={clip.replay} time={playback.time} label={`${clip.label} ${readable(probe)}`} pending={!isOriginal && summaryOf(run).active} compact />
        <div className="clip-outcome"><strong>{isOriginal ? 'Outcome' : 'Verified outcome'}</strong><span className={outcomeClass(goal)}>{isOriginal ? readable(clip.replay?.summary?.outcome) || 'Not recorded' : outcome.label}</span></div>
        {!isOriginal && outcome.termination && <p className="clip-detail">Recorded termination: {outcome.termination}.</p>}
        {!isOriginal && outcome.unmetCriterion && <p className="clip-detail text-fail">{outcome.unmetCriterion}</p>}
        {numeric(clip.replay?.actual_duration_s) && <p className="clip-detail">Recorded {clip.replay.actual_duration_s.toFixed(1)} s{numeric(clip.replay.duration_s) ? ` · ${clip.replay.duration_s.toFixed(1)} s task window` : ''}</p>}
        {!isOriginal && <ReassessmentNote agent={clip.label} reassessment={run?.story?.reassessment} />}
      </article>;
    })}</div>
    <PlaybackControls playback={playback} label="Synchronized replays" />
    <p className="playback-note">Same original task and elapsed scenario time. Each clip holds its final frame when it ends.</p>
    <div className="table-scroll"><table className="comparison-table"><caption className="visually-hidden">Recorded model performance</caption><thead><tr><th scope="col">Comparison</th><th scope="col">Original</th><th scope="col">Astra</th><th scope="col">Sol</th></tr></thead><tbody>{rows.map(([label, ...values]) => <tr key={label}><th scope="row">{label}</th>{values.map((value, index) => <td key={index}>{value}</td>)}</tr>)}</tbody></table></div>
    {metricNames.length > 0 && <details className="metrics-details"><summary>Task measurements</summary><div className="table-scroll"><table className="comparison-table"><thead><tr><th>Recorded measurement</th>{clips.map(clip => <th key={clip.id}>{clip.label}</th>)}</tr></thead><tbody>{metricNames.map(name => <tr key={name}><th scope="row">{readable(name)}</th>{clips.map(clip => <td key={clip.id}>{metricValue(clip.replay?.summary?.metrics?.[name])}</td>)}</tr>)}</tbody></table></div></details>}
    {AGENTS.map(agent => {
      const verification = runs[agent.id]?.story?.verification;
      const applied = runs[agent.id]?.story?.checkpoints?.filter(checkpoint => checkpoint.stage === 'applied' && checkpoint.kind === 'controller_edit') ?? [];
      const adjustments = applied.flatMap(checkpoint => checkpoint.changes ?? []).map(change => `${change.parameter}: ${formatValue(change.before)} → ${formatValue(change.after)}`);
      return verification?.status === 'completed' && <p key={agent.id} className="result-explanation"><strong>{agent.label}:</strong> {applied.length ? `${adjustments.length ? adjustments.join('; ') : `${applied.length} controller change${applied.length === 1 ? '' : 's'} applied`}. ${physicalOutcome(verification)} on the original task.` : `No applied controller changes were recorded. ${physicalOutcome(verification)}.`}</p>;
    })}
  </>;
}

export default function OutcomeComparison({ comparison }) {
  const [selectedProbe, setSelectedProbe] = useState('');
  const probes = [...new Set(AGENTS.flatMap(agent => (comparison?.runs?.[agent.id]?.story?.replays ?? []).filter(replay => ['before', 'after'].includes(replay.kind)).map(replay => replay.probe).filter(Boolean)))];
  const probe = probes.includes(selectedProbe) ? selectedProbe : probes[0] || '';
  const manifest = comparison?.manifest;
  const valid = comparison?.comparison_valid ?? manifest?.comparison_valid;
  const warnings = comparison?.warnings ?? manifest?.warnings ?? [];
  const rechecked = AGENTS.filter(agent => comparison?.runs?.[agent.id]?.story?.reassessment).map(agent => agent.label);
  return <section className="story-section" aria-labelledby="result-heading"><SectionTitle number="03" extra={probes.length > 1 ? <label className="probe-control">Recorded task<select aria-label="Recorded task" value={probe} onChange={event => setSelectedProbe(event.target.value)}>{probes.map(value => <option key={value} value={value}>{readable(value)}</option>)}</select></label> : <span className="task-verification-label">Full original task</span>}><span id="result-heading">The result</span></SectionTitle>
    {comparison && <div className={`comparison-integrity ${valid === false ? 'integrity-failed' : ''}`} role="status"><strong>{valid === true ? rechecked.length ? 'Original comparison conditions matched' : 'Recorded comparison conditions matched' : valid === false ? 'Comparison conditions did not match' : 'Comparison conditions are awaiting verification'}</strong><p>{valid === false ? 'These outcomes are preserved, but this pair cannot establish a fair model performance difference.' : valid === true ? 'Both runs used the same recorded starting scenario, instructions, tools, and budgets.' : 'The final protocol and starting-condition checks appear after both investigations finish.'}</p>{rechecked.length > 0 && <p>Displayed outcomes for {rechecked.join(' and ')} were subsequently rechecked after a scoring fix using the same frozen controllers. No new model calls were made; the original comparison and results are preserved.</p>}{warnings.length > 0 && <ul>{warnings.map((warning, index) => <li key={index}>{typeof warning === 'string' ? warning : formatValue(warning)}</li>)}</ul>}</div>}
    <MatchedResults key={`${comparison?.id || 'new'}-${probe}`} comparison={comparison} probe={probe} />
    <p className="section-note">Each model gets a separate simulation of the same starting scenario. One recorded pair shows this experiment’s outcome; repeated matched runs are needed to establish a consistent performance difference.</p>
    <DataDetails label="Recorded comparison conditions and budgets" value={manifest} />
  </section>;
}
