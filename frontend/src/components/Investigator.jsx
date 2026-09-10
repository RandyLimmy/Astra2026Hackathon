import { useMemo, useState } from 'react';
import { Chevron } from './Icons.jsx';

const TOOL_NAMES = {
  inspect_model: 'Inspect model', run_experiment: 'Run experiment', observe_run: 'Inspect trajectory',
  patch_model: 'Patch component', run_model: 'Check prediction', run_regression_suite: 'Check development cases',
  submit_prediction: 'Submit prediction',
};

function activityRows(events) {
  const rows = [];
  for (const event of events) {
    if (event.type === 'tool_result') {
      const pending = rows.findLast(row => row.type === 'tool_call' && row.name === event.name && !row.result);
      if (pending) { pending.result = event.result; continue; }
    }
    if (['tool_call', 'assistant_message', 'reasoning_summary', 'status', 'error', 'tool_result'].includes(event.type)) {
      rows.push({ ...event });
    }
  }
  return rows;
}

function JsonDetails({ label, value }) {
  const [open, setOpen] = useState(false);
  return <details className="payload-details" onToggle={event => setOpen(event.currentTarget.open)}><summary>{label}<Chevron size={14} /></summary>{open && <pre>{typeof value === 'string' ? value : JSON.stringify(value, null, 2)}</pre>}</details>;
}

function ActivityItem({ event, initiallyOpen = false }) {
  const [open, setOpen] = useState(initiallyOpen);
  if (event.type === 'status' || event.type === 'error') return <div className={`activity-status ${event.type === 'error' ? 'text-fail' : ''}`}>{event.message}</div>;
  if (event.type === 'assistant_message' || event.type === 'reasoning_summary') return <div className="activity-message"><div className="activity-label">{event.type === 'assistant_message' ? 'Explanation' : 'Brief reasoning summary'}</div><p>{event.text}</p></div>;
  const description = event.arguments?.hypothesis || event.arguments?.rationale;
  const failed = event.result?.ok === false || event.result?.error;
  return <details className="activity-tool" open={open} onToggle={event => setOpen(event.currentTarget.open)}>
    <summary><span>{TOOL_NAMES[event.name] || event.name || 'Tool result'}{failed && <small className="text-fail"> · rejected</small>}</span><Chevron /></summary>
    {open && <div className="activity-tool-content">
      {description && <p>{description}</p>}
      {event.arguments?.expected_observation && <p className="muted"><strong>Expected:</strong> {event.arguments.expected_observation}</p>}
      {event.result?.error && <p className="text-fail">{typeof event.result.error === 'string' ? event.result.error : event.result.error.message || 'The operation could not complete.'}</p>}
      {event.arguments && <JsonDetails label="Tool arguments" value={event.arguments} />}
      {event.result && <JsonDetails label="Recorded result" value={event.result} />}
      {!event.result && <span className="muted">Waiting for the recorded result…</span>}
    </div>}
  </details>;
}

function SourceView({ artifacts }) {
  const [view, setView] = useState('diff');
  const source = view === 'diff' ? artifacts.source_diff : view === 'original' ? artifacts.original_source : artifacts.frozen_source || artifacts.current_source;
  return <div className="source-view">
    <label className="context-selector">Source<select aria-label="Source view" value={view} onChange={event => setView(event.target.value)}><option value="diff">Source diff</option><option value="candidate">{artifacts.frozen_source ? 'Frozen candidate' : 'Current candidate'}</option><option value="original">Original component</option></select></label>
    {source ? <pre className="source-code">{view === 'diff' ? source.split('\n').map((line, index) => <span key={index} className={line.startsWith('+') && !line.startsWith('+++') ? 'diff-added' : line.startsWith('-') && !line.startsWith('---') ? 'diff-removed' : ''}>{line}{'\n'}</span>) : source}</pre> : <div className="context-empty">{view === 'diff' ? 'No source diff has been recorded yet.' : 'This source snapshot is not available yet.'}</div>}
  </div>;
}

function PromptView({ artifacts }) {
  const [view, setView] = useState('system_prompt');
  const value = artifacts[view];
  return <div>
    <label className="context-selector">Exact prompt<select aria-label="Prompt view" value={view} onChange={event => setView(event.target.value)}><option value="system_prompt">System message</option><option value="task_prompt">Task and initial evidence</option><option value="tools">Tool definitions</option></select></label>
    <p className="inline-note">The saved text sent in this run.</p>
    {value ? <pre className="prompt-text">{typeof value === 'string' ? value : JSON.stringify(value, null, 2)}</pre> : <div className="context-empty">Prompt records will appear after initial evidence is prepared.</div>}
  </div>;
}

export default function Investigator({ run, tab, setTab }) {
  const rows = useMemo(() => activityRows(run?.events ?? []), [run?.events]);
  const artifacts = run?.artifacts ?? {};
  return <aside className="investigator-panel" aria-labelledby="investigator-title">
    <h2 id="investigator-title">Investigator</h2>
    <div className="tabs" role="tablist" aria-label="Investigation context">{['activity', 'code', 'prompts'].map(item => <button key={item} id={`tab-${item}`} type="button" role="tab" aria-selected={tab === item} aria-controls={`panel-${item}`} className={tab === item ? 'selected' : ''} onClick={() => setTab(item)}>{item[0].toUpperCase() + item.slice(1)}</button>)}</div>
    <div className="context-content" id={`panel-${tab}`} role="tabpanel" aria-labelledby={`tab-${tab}`}>
      {tab === 'activity' && (rows.length ? <div className="activity-list">{rows.map((event, index) => <ActivityItem key={`${event.timestamp}-${event.type}-${index}`} event={event} initiallyOpen={index === rows.findIndex(row => row.type === 'tool_call')} />)}</div> : <div className="context-empty">{run?.active ? 'Preparing the initial experiments. Investigator activity will appear here as it runs.' : 'No investigator activity has been recorded for this run.'}</div>)}
      {tab === 'code' && <SourceView artifacts={artifacts} />}
      {tab === 'prompts' && <PromptView artifacts={artifacts} />}
    </div>
  </aside>;
}
