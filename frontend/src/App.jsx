import { useEffect, useState } from 'react';
import { elapsedLabel, request, runLabel, verdict } from './api.js';
import { useRun, useRuns } from './hooks.js';
import Replay from './components/Replay.jsx';
import Investigator from './components/Investigator.jsx';
import Comparison from './components/Comparison.jsx';
import ScenarioReplay from './components/ScenarioReplay.jsx';

function initialSelection() {
  return new URL(window.location.href).searchParams.get('run') || '';
}

export default function App() {
  if (window.__SCENARIO_REPLAY__ || new URL(window.location.href).searchParams.get('view') === 'scenarios') {
    return <ScenarioReplay onBack={() => window.location.assign(window.location.pathname)} />;
  }
  return <InvestigationApp />;
}

function InvestigationApp() {
  const listing = useRuns();
  const [selectedId, setSelectedId] = useState(initialSelection);
  const detail = useRun(selectedId);
  const [selectedCase, setSelectedCase] = useState('');
  const [profile, setProfile] = useState('sol-high');
  const [starting, setStarting] = useState(false);
  const [startError, setStartError] = useState('');
  const [tab, setTab] = useState('activity');
  const [view, setView] = useState('scenario');
  const run = detail.run;
  const activeId = listing.active_run_id || listing.runs.find(item => item.active)?.id;

  useEffect(() => {
    if (listing.runs.length && !selectedId) {
      // A completed run with actual results gives first-time visitors something
      // to inspect; active runs remain available immediately in the selector.
      setSelectedId(listing.runs.find(item => item.aggregate)?.id || listing.runs[0].id);
    }
  }, [listing.runs, selectedId]);

  useEffect(() => {
    const url = new URL(window.location.href);
    if (selectedId) url.searchParams.set('run', selectedId);
    else url.searchParams.delete('run');
    window.history.replaceState({}, '', url);
    setSelectedCase('');
  }, [selectedId]);

  useEffect(() => {
    const cases = (run?.evaluation?.cases ?? []).filter(item => typeof item.case_id === 'string');
    if (cases.length && !cases.some(item => item.case_id === selectedCase)) {
      setSelectedCase(cases.find(item => item.config?.preparation_cycles > 0 && !item.config?.wait_s)?.case_id || cases[0].case_id);
    }
  }, [run?.evaluation?.cases, selectedCase]);

  async function startRun() {
    if (starting || activeId || listing.loading) return;
    setStarting(true); setStartError('');
    try {
      const result = await request('/api/runs', { method: 'POST', body: JSON.stringify({ profile }) });
      setSelectedId(result.id); setTab('activity'); setView('investigation');
      await listing.refresh();
    } catch (failure) {
      setStartError(failure.message);
    } finally {
      setStarting(false);
    }
  }

  function showPrompts() {
    setTab('prompts'); setView('investigation');
  }

  const latest = run?.status === 'completed'
    ? `Completed · prediction criteria ${verdict(run.aggregate || run.evaluation?.aggregate, run.status).text.toLowerCase()}${run.media_status === 'rendering' || run.media_status === 'queued' ? ' · preparing replay frames' : ''}`
    : run?.latest_status || (run?.active ? 'Investigation is running. Activity updates automatically.' : '');
  const requests = run?.api_requests ?? run?.metadata?.api_requests;
  const error = startError || listing.error || detail.error;
  const mergedRuns = listing.runs.map(item => item.id === run?.id ? { ...item, metadata: run.metadata, aggregate: run.aggregate || run.evaluation?.aggregate, status: run.status } : item);
  return <>
    <header className="app-header">
      <a className="brand" href={window.location.pathname} aria-label="RealityPatch home">RealityPatch</a>
      <a className="text-button" href="?view=scenarios">Failure demos</a>
      <select className="run-selector" aria-label="Recorded run" value={selectedId} onChange={event => setSelectedId(event.target.value)} disabled={!listing.runs.length}>
        {!listing.runs.length && <option value="">{listing.loading ? 'Loading experiments…' : 'No recorded runs'}</option>}
        {selectedId && !listing.runs.some(item => item.id === selectedId) && <option value={selectedId}>{selectedId}</option>}
        {listing.runs.map(item => <option key={item.id} value={item.id}>{runLabel(item)}{item.active ? ' · running' : ''}</option>)}
      </select>
      <div className="start-controls"><select aria-label="New investigation profile" value={profile} onChange={event => setProfile(event.target.value)} disabled={starting}><option value="astra-xhigh">Astra / extra high</option><option value="sol-high">Sol / high</option></select><button className="primary start-button" type="button" disabled={starting || listing.loading || Boolean(activeId) || Boolean(listing.error)} onClick={startRun} title={activeId ? 'An investigation is already active' : 'Start a bounded investigation using the configured local API key'}>{starting ? 'Starting…' : activeId ? 'Run active' : 'Start run'}</button></div>
    </header>
    {error && <div className="error-banner" role="alert"><span>{error}</span><button type="button" className="text-button" onClick={() => { setStartError(''); listing.refresh(); }}>Retry connection</button></div>}
    {(latest || requests !== undefined || detail.loading) && <div className="run-status" role="status"><span>{detail.loading && !run ? 'Loading experiment…' : latest || `Recorded run · ${run?.status || 'status unavailable'}`}</span><span>{requests !== undefined ? `${requests} API requests` : ''}{run?.tool_calls !== undefined ? ` · ${run.tool_calls} tool calls` : ''}{elapsedLabel(run?.metadata) ? ` · ${elapsedLabel(run.metadata)}` : ''}</span></div>}
    <main>
      <nav className="workspace-nav" aria-label="Dashboard views">
        {[['scenario', 'Scenario'], ['investigation', 'Investigation'], ['comparison', 'Compare runs']].map(([id, label]) =>
          <button key={id} type="button" aria-pressed={view === id} className={view === id ? 'selected' : ''} onClick={() => setView(id)}>{label}</button>)}
      </nav>
      <div className="workspace" aria-busy={detail.loading}>
        {view === 'scenario' && <Replay run={run} selectedCase={selectedCase} onSelectCase={setSelectedCase} />}
        {view === 'investigation' && <Investigator key={run?.id || selectedId} run={run} tab={tab} setTab={setTab} />}
        {view === 'comparison' && <Comparison runs={mergedRuns} selectedId={selectedId} onSelect={id => { setSelectedId(id); setView('scenario'); }} showPrompts={showPrompts} />}
      </div>
    </main>
  </>;
}
