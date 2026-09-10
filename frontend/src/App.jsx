import { useEffect, useRef, useState } from 'react';
import { request } from './api.js';
import { comparisonLabel, readable } from './paired.js';
import { useComparison, useComparisons, useScenarios } from './pairedHooks.js';
import OriginalScenario from './components/OriginalScenario.jsx';
import AgentTracks from './components/AgentTracks.jsx';
import OutcomeComparison from './components/OutcomeComparison.jsx';
import { PlayIcon } from './components/Icons.jsx';
import ScenarioReplay from './components/ScenarioReplay.jsx';

export default function App() {
  if (window.__SCENARIO_REPLAY__ || new URL(window.location.href).searchParams.get('view') === 'scenarios') {
    return <ScenarioReplay onBack={() => window.location.assign(window.location.pathname)} />;
  }
  return <InvestigationApp />;
}

function InvestigationApp() {
  const catalog = useScenarios();
  const listing = useComparisons();
  const [selectedId, setSelectedId] = useState('');
  const [platformId, setPlatformId] = useState('drone');
  const [scenarioId, setScenarioId] = useState('');
  const [starting, setStarting] = useState(false);
  const [startError, setStartError] = useState('');
  const initialized = useRef(false);
  const syncedId = useRef('');
  const detail = useComparison(selectedId);
  const platforms = catalog.data?.platforms ?? [];
  const comparisons = listing.data?.comparisons ?? [];
  const comparison = detail.data;
  const platform = platforms.find(item => item.id === platformId) || platforms[0];
  const scenarios = platform?.scenarios ?? [];
  const scenario = scenarios.find(item => item.id === scenarioId) || scenarios.find(item => item.id === platform?.default_scenario) || scenarios[0];
  const activeId = listing.data?.active_comparison_id;
  const error = startError || catalog.error || listing.error || detail.error;

  useEffect(() => {
    if (listing.loading || !listing.data || initialized.current) return;
    initialized.current = true;
    if (comparisons.length) setSelectedId(activeId || comparisons[0].id);
  }, [listing.loading, listing.data, comparisons, activeId]);
  useEffect(() => {
    if (!comparison || syncedId.current === comparison.id) return;
    syncedId.current = comparison.id;
    const record = comparison.manifest ?? comparison;
    if (record.platform) setPlatformId(record.platform);
    if (record.scenario) setScenarioId(record.scenario);
  }, [comparison]);
  function choosePlatform(value) { setPlatformId(value); setScenarioId(''); setSelectedId(''); syncedId.current = ''; }
  function chooseScenario(value) { setScenarioId(value); setSelectedId(''); syncedId.current = ''; }
  async function startPair() {
    if (!platform || !scenario || starting || activeId) return;
    setStarting(true); setStartError('');
    try {
      const result = await request('/api/comparisons', { method: 'POST', body: JSON.stringify({ platform: platform.id, scenario: scenario.id }) });
      const id = result.id || result.comparison_id || result.comparison?.id;
      if (!id) throw new Error('The server started a request but returned no comparison identifier. Refresh the recordings before trying again.');
      setSelectedId(id); listing.refresh();
    } catch (failure) { setStartError(failure.message); }
    finally { setStarting(false); }
  }
  function retry() { setStartError(''); catalog.refresh(); listing.refresh(); detail.refresh(); }
  const profiles = catalog.data?.profiles ?? comparison?.manifest?.profiles ?? {};
  return <>
    <header className="app-header"><a className="brand" href={window.location.pathname}>RealityPatch</a>
      <div className="scenario-controls"><label>Platform<select aria-label="Platform" value={platform?.id || ''} onChange={event => choosePlatform(event.target.value)} disabled={!platforms.length || starting}>{!platforms.length && <option value="">{catalog.loading ? 'Loading…' : 'Unavailable'}</option>}{platforms.map(item => <option key={item.id} value={item.id}>{item.label}</option>)}</select></label>
        <label>Preset<select aria-label="Scenario preset" value={scenario?.id || ''} onChange={event => chooseScenario(event.target.value)} disabled={!scenarios.length || starting}>{!scenarios.length && <option value="">No presets available</option>}{scenarios.map(item => <option key={item.id} value={item.id}>{item.label}</option>)}</select></label>
      </div>
      <button className="primary start-button" type="button" disabled={starting || !scenario || Boolean(activeId) || catalog.loading || listing.loading || Boolean(catalog.error || listing.error)} onClick={startPair}><PlayIcon />{starting ? 'Starting…' : activeId ? 'Pair running' : 'Run Astra + Sol'}</button>
      <label className="recording-control">Recording<select aria-label="Recorded comparison" value={selectedId} onChange={event => { setSelectedId(event.target.value); syncedId.current = ''; }}><option value="">{listing.loading ? 'Loading recordings…' : 'New comparison'}</option>{selectedId && !comparisons.some(item => item.id === selectedId) && <option value={selectedId}>{selectedId}</option>}{comparisons.map(item => <option key={item.id} value={item.id}>{comparisonLabel(item)}</option>)}</select></label>
    </header>
    {error && <div className="error-banner" role="alert"><span>{error}</span><button className="text-button" type="button" onClick={retry}>Retry connection</button></div>}
    <main><div className="page-intro"><h1>One scenario. Two approaches.</h1><p>Original behavior, exact changes, measured results.</p><a className="failure-demos-link" href="?view=scenarios">Browse failure demos →</a></div>
      {(comparison || detail.loading || activeId) && <div className="comparison-status" role="status"><span>{detail.loading && !comparison ? 'Loading the recorded comparison…' : comparison ? `${scenario?.label || readable(comparison.manifest?.scenario)} · ${comparison.active ? 'Both investigations update independently' : readable(comparison.status) || 'Recorded comparison'}` : 'A comparison is running.'}</span>{activeId && selectedId !== activeId && <button className="text-button" type="button" onClick={() => setSelectedId(activeId)}>View active comparison</button>}</div>}
      <div aria-busy={detail.loading}><OriginalScenario comparison={comparison} platform={platform} scenario={scenario} /><AgentTracks comparison={comparison} profiles={profiles} /><OutcomeComparison comparison={comparison} /></div>
      <footer>New car · Drone · Robot dog<span>Two independent investigations. Every result backed by a recording.</span></footer>
    </main>
  </>;
}
