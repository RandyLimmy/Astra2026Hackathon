import { useEffect, useRef, useState } from 'react';
import { request } from './api.js';
import { AGENTS, TASKS, batchComparisonId, comparisonLabel, isCurrentTask, readable, taskCatalog } from './paired.js';
import { useBatches, useComparison, useComparisons, useScenarios } from './pairedHooks.js';
import OriginalScenario from './components/OriginalScenario.jsx';
import AgentTracks from './components/AgentTracks.jsx';
import OutcomeComparison from './components/OutcomeComparison.jsx';
import BatchOverview from './components/BatchOverview.jsx';
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
  const batches = useBatches();
  const [selectedId, setSelectedId] = useState('');
  const [platformId, setPlatformId] = useState('quadruped');
  const [startedBatch, setStartedBatch] = useState(null);
  const [starting, setStarting] = useState(false);
  const [startError, setStartError] = useState('');
  const followedBatchId = useRef('');
  const detail = useComparison(selectedId);
  const platforms = taskCatalog(catalog.data?.platforms);
  const comparisons = (listing.data?.comparisons ?? []).filter(isCurrentTask);
  const batchList = batches.data?.batches ?? [];
  const activeBatch = batchList.find(item => item.id === batches.data?.active_batch_id || item.active || ['running', 'starting'].includes(item.status));
  const batch = activeBatch || batchList.find(item => item.id === startedBatch?.id) || startedBatch || batchList[0];
  const active = Boolean(activeBatch || comparisons.some(item => item.active)
    || (startedBatch && !batchList.some(item => item.id === startedBatch.id)));
  const platform = platforms.find(item => item.id === platformId) || platforms[0];
  const scenario = platform?.scenarios[0];
  const recordings = comparisons.filter(item => (item.manifest ?? item).platform === platform?.id);
  const comparison = detail.data && isCurrentTask(detail.data) ? detail.data : null;
  const error = startError || catalog.error || listing.error || batches.error || detail.error;

  useEffect(() => {
    const id = batchComparisonId(activeBatch, platformId);
    if (!activeBatch?.id || !id || followedBatchId.current === activeBatch.id) return;
    // Follow a newly launched batch once, including launches from another tab
    // or the CLI. Later polls preserve an explicit recording/preview selection.
    followedBatchId.current = activeBatch.id;
    setSelectedId(id);
  }, [activeBatch, platformId]);

  function choosePlatform(value, id = null) {
    setPlatformId(value);
    // With no live batch, selecting a task opens its current original preview.
    // Historical incidents remain selectable through batch cards/recordings.
    setSelectedId(id || batchComparisonId(activeBatch, value) || '');
  }
  async function startBatch() {
    if (starting || active || platforms.length !== TASKS.length) return;
    setStarting(true); setStartError('');
    try {
      const result = await request('/api/batches', { method: 'POST', body: '{}' });
      if (!result.id || !batchComparisonId(result, platform.id)) {
        throw new Error('The server returned no complete batch identifier. Refresh the recordings before trying again.');
      }
      setStartedBatch(result);
      setSelectedId(batchComparisonId(result, platform.id));
      listing.refresh(); batches.refresh();
    } catch (failure) { setStartError(failure.message); }
    finally { setStarting(false); }
  }
  function retry() { setStartError(''); catalog.refresh(); listing.refresh(); batches.refresh(); detail.refresh(); }
  const profiles = catalog.data?.profiles ?? comparison?.manifest?.profiles ?? {};
  return <>
    <header className="app-header"><a className="brand" href={window.location.pathname}>RealityPatch</a>
      <div className="scenario-controls"><label>Scenario<select aria-label="Scenario" value={platform?.id || ''} onChange={event => choosePlatform(event.target.value)} disabled={!platforms.length || starting}>
        {!platforms.length && <option value="">{catalog.loading ? 'Loading…' : 'Unavailable'}</option>}
        {platforms.map(item => <option key={item.id} value={item.id}>{item.label}</option>)}
      </select></label></div>
      <button className="primary start-button" type="button" disabled={starting || platforms.length !== TASKS.length || active || catalog.loading || listing.loading || batches.loading || Boolean(catalog.error || listing.error || batches.error)} onClick={startBatch}><PlayIcon />{starting ? `Starting all ${TASKS.length}…` : active ? 'Batch running · Astra + Sol' : `Run all ${TASKS.length} · Astra + Sol`}</button>
      <label className="recording-control">Recording<select aria-label="Recorded comparison" value={selectedId} onChange={event => setSelectedId(event.target.value)}><option value="">Original preview</option>{selectedId && !recordings.some(item => item.id === selectedId) && <option value={selectedId}>Current comparison</option>}{recordings.map(item => <option key={item.id} value={item.id}>{comparisonLabel(item)}</option>)}</select></label>
    </header>
    {error && <div className="error-banner" role="alert"><span>{error}</span><button className="text-button" type="button" onClick={retry}>Retry connection</button></div>}
    <main><div className="page-intro"><h1>{TASKS.length} scenarios. {AGENTS.length} approaches.</h1><p>The original failure, each controller change, and the measured result.</p><span className="parallel-note">{new Intl.ListFormat('en', { style: 'long', type: 'conjunction' }).format(TASKS.map(task => task.label))} run together. Astra and GPT-5.6 Sol each use max reasoning.</span></div>
      <BatchOverview batch={batch} comparisons={comparisons} selectedId={selectedId} platformId={platform?.id} onSelect={choosePlatform} />
      {(comparison || detail.loading) && <div className="comparison-status" role="status"><span>{detail.loading && !comparison ? 'Loading the recorded comparison…' : `${platform?.label} · ${comparison?.active ? 'Both investigations update independently' : readable(comparison?.status) || 'Recorded comparison'}`}</span></div>}
      <div aria-busy={detail.loading}><OriginalScenario comparison={comparison} platform={platform} scenario={scenario} /><AgentTracks comparison={comparison} profiles={profiles} /><OutcomeComparison comparison={comparison} /></div>
      <footer>{TASKS.map(task => task.label).join(' · ')}<span>{TASKS.length * AGENTS.length} independent investigations per batch. Results come from recorded task runs.</span><a href="?view=scenarios">Scenario archive →</a></footer>
    </main>
  </>;
}
