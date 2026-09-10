import { elapsedLabel, meters, modelName, runLabel, verdict } from '../api.js';

function submission(run) {
  if (run.metadata?.agent_submitted === true) return 'Agent submitted';
  if (run.metadata?.source_hash || run.metadata?.frozen_source_file) return 'Host frozen';
  return run.active || ['running', 'starting', 'evaluating'].includes(run.status) ? 'In progress' : '—';
}

function CompareTable({ runs, selectedId, onSelect }) {
  return <div className="table-scroll"><table className="comparison-table"><thead><tr><th>Model</th><th>Reasoning</th><th>Mean error</th><th>Criteria</th><th>Submission</th></tr></thead><tbody>{runs.map(run => {
    const check = verdict(run.aggregate, run.status);
    return <tr key={run.id} className={selectedId === run.id ? 'selected-row' : ''}>
      <td><button className="run-row-button" type="button" onClick={() => onSelect(run.id)} title={runLabel(run)}>{modelName(run.metadata?.model)}</button><span className="row-meta">{run.metadata?.api_requests ?? run.api_requests ?? 0} requests{elapsedLabel(run.metadata) ? ` · ${elapsedLabel(run.metadata)}` : ''}</span></td>
      <td>{run.metadata?.reasoning_effort || '—'}</td><td>{meters(run.aggregate?.candidate_mae_m)}</td><td className={check.className}>{check.text}</td><td>{submission(run)}</td>
    </tr>;
  })}</tbody></table></div>;
}

export default function Comparison({ runs, selectedId, onSelect, showPrompts }) {
  const groups = new Map();
  const other = [];
  for (const run of runs) {
    const fingerprint = run.metadata?.protocol_fingerprint;
    if (fingerprint) groups.set(fingerprint, [...(groups.get(fingerprint) ?? []), run]);
    else other.push(run);
  }
  return <section className="comparison-panel" aria-labelledby="comparison-title">
    <h2 id="comparison-title">Model comparison</h2>
    {!runs.length ? <div className="context-empty">No experiments have been recorded yet. Start a run to build a comparison.</div> : <>
      {[...groups.entries()].map(([fingerprint, group], index) => <div className="comparison-group" key={fingerprint}>
        <h3>{groups.size > 1 ? `Experiment setup ${index + 1}` : 'Current experiment setup'}</h3>
        <CompareTable runs={group} selectedId={selectedId} onSelect={onSelect} />
        {group.length > 1 && <p className="inline-note">These runs share the same recorded experiment setup; model and reasoning settings are shown above.</p>}
      </div>)}
      {other.length > 0 && <div className="comparison-group">{groups.size > 0 && <h3>Earlier runs · separate from the matched comparison</h3>}<CompareTable runs={other} selectedId={selectedId} onSelect={onSelect} /></div>}
      <p className="comparison-note">Synthetic experiments. Lower mean error does not mean every prediction passed. Select a model row to inspect that run.</p>
    </>}
    <div className="footer-rule"><button className="text-button" type="button" onClick={showPrompts}>View exact prompts and source</button><span>RealityPatch · local experiment viewer</span></div>
  </section>;
}
