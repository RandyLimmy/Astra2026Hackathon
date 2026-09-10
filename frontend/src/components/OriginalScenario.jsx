import RecordedPlayback from './RecordedPlayback.jsx';
import { replayFor, readable } from '../paired.js';

export function SectionTitle({ number, children, extra }) {
  return <div className="section-heading"><h2><span>{number}</span>{children}</h2>{extra}</div>;
}

export default function OriginalScenario({ comparison, platform, scenario }) {
  const runs = comparison?.runs ?? {};
  const replay = replayFor(runs.astra, 'incident') || replayFor(runs.sol, 'incident')
    || replayFor(platform?.preview, 'incident') || replayFor(platform?.preview, 'before');
  const story = runs.astra?.story || runs.sol?.story || platform?.preview?.story;
  const goal = story?.goal ?? platform?.goal;
  const title = typeof goal === 'string' ? goal : goal?.title || goal?.description;
  const summary = replay?.summary || platform?.preview?.summary;
  const criteria = Array.isArray(goal?.criteria) ? goal.criteria : [];
  return <section className="story-section" aria-labelledby="original-heading">
    <SectionTitle number="01"><span id="original-heading">The original scenario</span></SectionTitle>
    <div className="original-layout">
      <RecordedPlayback key={`${platform?.id}-${comparison?.id || 'preview'}-${replay?.id || 'pending'}`} replay={replay} label="Original scenario" pending={comparison?.active || platform?.preview?.status === 'recording'} />
      <div className="scenario-context"><div><h3>Goal</h3><p>{title || 'Choose a scenario to inspect its original task.'}</p>{criteria.length > 0 && <details className="text-details"><summary>Success criteria</summary><ul>{criteria.map((criterion, index) => <li key={index}>{typeof criterion === 'string' ? criterion : JSON.stringify(criterion)}</li>)}</ul></details>}</div>
        <div><h3>Recorded outcome</h3><p className={summary?.safe === false ? 'text-fail' : ''}>{summary?.outcome ? readable(summary.outcome) : comparison?.active ? 'Preparing the original scenario…' : 'Awaiting a recorded run.'}</p>{!summary && scenario?.description && <p className="secondary-note">{scenario.description}</p>}</div>
        {replay && <p className="footnote">Original controller · {replay.duration_s?.toFixed?.(1) ?? '—'} s task window</p>}
      </div>
    </div>
  </section>;
}
