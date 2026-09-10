import { useEffect, useMemo, useRef, useState } from 'react';
import { advanceTime, attemptLabel, clockLabel, eventTime, failureWindow, firstFailure, frameIndexAt, seekTime, stepFrame } from './scenario-playback.mjs';
import '../scenario-replay.css';

const DEFAULT_SCENARIOS = [
  { id: 'quadruped_gait_failure', title: 'Dog · loss of balance', description: 'A faster walk exposes a coordination failure.', objective: 'Walk forward along the marked strip, then increase speed without falling.', status: 'missing' },
  { id: 'drone_delivery_imbalance', title: 'Drone · delivery imbalance', description: 'An uneven load turns a delivery into a crash.', objective: 'Carry the parcel from A to B, deliver it, and return to A.', status: 'missing' },
];

const OUTCOME_METRICS = [
  ['distance_before_speed_transition_m', 'Walk before speed-up', ' m'],
  ['forward_progress_before_impact', 'Travel before impact', ' m'],
  ['impact_speed', 'Impact speed', ' m/s'],
  ['max_tilt_deg', 'Maximum tilt', '°'],
  ['max_tilt_degrees', 'Maximum tilt', '°'],
  ['body_contact_duration_s', 'Body contact', ' s'],
  ['parcel_delivered', 'Parcel delivered', ''],
  ['fall_time', 'Fall time', ' s'],
];

function Icon({ name, size = 18 }) {
  const paths = {
    play: <path d="m9 5 11 7-11 7Z" />,
    pause: <><path d="M8 5v14M16 5v14" /></>,
    restart: <><path d="M4 10a8 8 0 1 1 1 7M4 4v6h6" /></>,
    previous: <><path d="M6 5v14m12-14-9 7 9 7Z" /></>,
    next: <><path d="M18 5v14M6 5l9 7-9 7Z" /></>,
    loop: <><path d="m18 2 4 4-4 4M2 11V9a3 3 0 0 1 3-3h17M6 22l-4-4 4-4m16-1v2a3 3 0 0 1-3 3H2" /></>,
    arrow: <path d="m9 5-7 7 7 7M2 12h20" />,
    dog: <><path d="m3 8 3 4h9l2-7h3l2 4-4 2v7m-3-6 1 7M7 12l-2 7m5-7 1 7" /></>,
    car: <><path d="m4 10 2-5h12l2 5m-17 0h18v9h-3v-3H6v3H3Zm4 3h1m8 0h1" /></>,
    drone: <><path d="m6 6 12 12M6 18 18 6M10 10h4v4h-4Zm0 7v4h5v-4" /><ellipse cx="5" cy="5" rx="4" ry="2" /><ellipse cx="19" cy="5" rx="4" ry="2" /><ellipse cx="5" cy="17" rx="4" ry="2" /><ellipse cx="19" cy="17" rx="4" ry="2" /></>,
  };
  return <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.7" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">{paths[name]}</svg>;
}

const humanize = value => String(value || '').replaceAll('_', ' ').replace(/^./, value => value.toUpperCase());
const scenarioIcon = id => id.startsWith('car_') ? 'car' : id.startsWith('quadruped_') ? 'dog' : 'drone';

async function readJson(url, signal) {
  const response = await fetch(url, { signal, headers: { Accept: 'application/json' } });
  if (!response.ok) throw new Error(`Recording request failed (${response.status}).`);
  return response.json();
}

function metricValue(value) {
  if (typeof value === 'boolean') return value ? 'Yes' : 'No';
  if (typeof value === 'number') return Number.isInteger(value) ? String(value) : value.toFixed(2);
  return humanize(value);
}

export default function ScenarioReplay({ onBack, offlineManifest = window.__SCENARIO_REPLAY__ }) {
  const offlineScenario = useMemo(() => offlineManifest ? {
    ...DEFAULT_SCENARIOS.find(item => item.id === offlineManifest.scenario_id),
    id: offlineManifest.scenario_id,
    title: offlineManifest.title,
    objective: offlineManifest.objective,
    ...(offlineManifest.provenance === 'developer_control' ? { description: 'Developer-authored reference correction.' }
      : offlineManifest.provenance === 'candidate_attempt' ? { description: 'Recorded candidate attempt.' } : {}),
    status: 'ready',
  } : null, [offlineManifest]);
  const [scenarios, setScenarios] = useState(() => offlineScenario ? [offlineScenario] : DEFAULT_SCENARIOS);
  const [selectedId, setSelectedId] = useState(() => offlineScenario?.id || DEFAULT_SCENARIOS[0].id);
  const [reload, setReload] = useState(0);
  const [listLoading, setListLoading] = useState(!offlineManifest);
  const [listError, setListError] = useState('');
  const [recording, setRecording] = useState(null);
  const [mediaLoading, setMediaLoading] = useState(false);
  const [mediaError, setMediaError] = useState('');
  const [failedFrame, setFailedFrame] = useState('');
  const [camera, setCamera] = useState('side');
  const [time, setTime] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [loop, setLoop] = useState(null);
  const timeRef = useRef(0);
  const frameSrcRef = useRef('');
  const selected = scenarios.find(item => item.id === selectedId) || scenarios[0];
  // Tag each response with its scenario so a selection change cannot briefly
  // display frames or evidence from the previously selected recording.
  const manifest = recording?.scenarioId === selectedId ? recording.manifest : null;
  const manifestUrl = manifest ? recording.url : '';

  useEffect(() => {
    if (offlineScenario) {
      setScenarios([offlineScenario]); setSelectedId(offlineScenario.id);
      setListLoading(false); setListError('');
      return;
    }
    const controller = new AbortController();
    setListLoading(true); setListError('');
    readJson('/api/scenarios', controller.signal).then(value => {
      if (controller.signal.aborted) return;
      if (!Array.isArray(value.scenarios)) throw new Error('Scenario listing is unavailable.');
      setScenarios(value.scenarios.length ? value.scenarios : DEFAULT_SCENARIOS);
    }).catch(error => {
      if (!controller.signal.aborted) setListError(error.message || 'Could not load recordings.');
    }).finally(() => { if (!controller.signal.aborted) setListLoading(false); });
    return () => controller.abort();
  }, [reload, offlineScenario]);

  useEffect(() => {
    const controller = new AbortController();
    setRecording(null); setMediaError(''); setFailedFrame('');
    setPlaying(false); setLoop(null); setTime(0); timeRef.current = 0;
    setMediaLoading(false);
    if (listLoading || (!offlineManifest && (!selected?.manifest_url || selected.status !== 'ready'))) return () => controller.abort();
    // Standalone exports embed their manifest and load images beside replay.html.
    // Avoid fetch entirely so file:// playback works without a server or CORS.
    const url = offlineManifest ? window.location.href : new URL(selected.manifest_url, window.location.origin).href;
    setMediaLoading(true);
    const source = offlineManifest ? Promise.resolve(offlineManifest) : readJson(url, controller.signal);
    source.then(value => {
      if (controller.signal.aborted) return;
      if (value.schema_version !== 1 || value.scenario_id !== selectedId || !Array.isArray(value.frames) || !Array.isArray(value.cameras)) {
        throw new Error('This recording has an unsupported or incomplete manifest.');
      }
      const frames = value.frames.filter(frame => Number.isFinite(frame.t_s) && frame.views && frame.t_s >= 0).slice().sort((a, b) => a.t_s - b.t_s);
      const cameras = value.cameras.filter(view => view.id && frames.some(frame => typeof frame.views[view.id] === 'string'));
      if (!frames.length || !cameras.length) throw new Error('This recording does not contain playable camera frames yet.');
      setRecording({ scenarioId: selectedId, url, manifest: { ...value, frames, cameras } });
      setCamera(current => cameras.some(view => view.id === current) ? current : cameras[0].id);
    }).catch(error => {
      if (!controller.signal.aborted) setMediaError(error.message || 'Could not load this recording.');
    }).finally(() => { if (!controller.signal.aborted) setMediaLoading(false); });
    return () => controller.abort();
  }, [selectedId, selected?.manifest_url, selected?.status, listLoading, reload, offlineManifest]);

  const frames = manifest?.frames || [];
  const duration = Math.max(Number.isFinite(manifest?.duration_s) ? manifest.duration_s : 0, frames.at(-1)?.t_s || 0);
  const frameIndex = frameIndexAt(frames, time);
  const frame = frames[frameIndex];
  const taskPresentation = frame?.presentation || (time >= duration ? manifest?.conclusion : null);
  const framePath = frame?.views?.[camera];
  const frameSrc = framePath && manifestUrl ? new URL(framePath, manifestUrl).href : '';
  frameSrcRef.current = frameSrc;
  const imageFailed = Boolean(failedFrame && failedFrame === frameSrc);
  const events = useMemo(() => (manifest?.events || []).filter(event => Number.isFinite(eventTime(event))).slice().sort((a, b) => eventTime(a) - eventTime(b)), [manifest]);
  const samples = useMemo(() => (manifest?.samples || []).filter(sample => Number.isFinite(sample.t_s)).slice().sort((a, b) => a.t_s - b.t_s), [manifest]);
  const sample = samples[frameIndexAt(samples, time)];
  const footContacts = selectedId === DEFAULT_SCENARIOS[0].id && sample?.foot_contacts;
  const isReference = manifest?.provenance === 'developer_control';
  const isCandidate = manifest?.provenance === 'candidate_attempt';
  const failure = firstFailure(events);
  const inspection = failureWindow(failure, duration);
  const currentEvent = events.filter(event => eventTime(event) <= time + 1e-8).at(-1);
  const observationSpeed = Array.isArray(sample?.velocity) && sample.velocity.length >= 3 && sample.velocity.every(Number.isFinite)
    ? Math.hypot(...sample.velocity).toFixed(2) : null;
  const observationTilt = sample?.body_tilt_deg ?? sample?.tilt_degrees;
  const summary = Object.entries(manifest?.summary || {}).filter(([, value]) => ['number', 'boolean', 'string'].includes(typeof value)).slice(0, 6);
  const outcomeMetrics = OUTCOME_METRICS.filter(([key]) => typeof manifest?.summary?.metrics?.[key] === 'boolean' || Number.isFinite(manifest?.summary?.metrics?.[key])).slice(0, 4);
  const loading = listLoading || mediaLoading;
  const usable = Boolean(manifest && frames.length && duration > 0);

  useEffect(() => {
    if (!playing || !usable) return;
    let animation;
    let previous = performance.now();
    const tick = now => {
      const delta = (now - previous) / 1000 * speed;
      previous = now;
      const next = advanceTime(timeRef.current, delta, duration, loop);
      timeRef.current = next.time;
      setTime(next.time);
      if (next.finished) { setPlaying(false); return; }
      animation = requestAnimationFrame(tick);
    };
    animation = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(animation);
  }, [playing, usable, speed, duration, loop]);

  function seek(next, keepPlaying = false) {
    const value = seekTime(next, duration);
    timeRef.current = value; setTime(value); setPlaying(keepPlaying); setFailedFrame('');
  }

  function jump(next) { setLoop(null); seek(next); }

  function togglePlay() {
    if (!playing && time >= duration) seek(loop?.start || 0);
    setPlaying(value => !value);
  }

  function toggleInspection() {
    if (loop) { setLoop(null); setPlaying(false); return; }
    if (!inspection) return;
    setLoop(inspection); setSpeed(0.25); seek(inspection.start, true);
  }

  function refresh() {
    setPlaying(false); setRecording(null); setFailedFrame(''); setReload(value => value + 1);
  }

  return <div className="scenario-lab">
    <header className="scenario-header">
      {offlineManifest ? <span className="scenario-brand">RealityPatch<span> / </span><span>Saved replay</span></span>
        : <a className="scenario-brand" href="/">RealityPatch<span> / </span><span>Failure lab</span></a>}
      {!offlineManifest && (onBack ? <button type="button" className="scenario-button scenario-back" onClick={onBack}><Icon name="arrow" /> Investigations</button>
        : <a className="scenario-back-link" href="/?view=investigations">Investigations <span aria-hidden="true">↗</span></a>)}
    </header>
    <main className="scenario-main">
      <div className="scenario-heading">
        <div><p className="scenario-eyebrow">Recorded simulations</p><h1>{isReference || isCandidate ? 'Inspect the recorded motion.' : 'See exactly where it goes wrong.'}</h1><p className="scenario-intro">{isReference ? 'A developer-authored reference correction, ready to replay and inspect.' : isCandidate ? 'A recorded candidate attempt, ready to replay and inspect.' : 'Physical failures. Every moment ready to replay and inspect.'}</p></div>
        <span className="scenario-repair-status"><span /> Astra repair: <strong>Not run</strong></span>
      </div>

      <nav className="scenario-cards" aria-label="Failure scenarios">
        {scenarios.map((item, index) => <button type="button" key={item.id} className={`scenario-card ${selectedId === item.id ? 'is-selected' : ''}`} aria-pressed={selectedId === item.id} onClick={() => { setPlaying(false); setSelectedId(item.id); }}>
          <span className="scenario-card-icon"><Icon name={scenarioIcon(item.id)} size={28} /></span>
          <span className="scenario-card-copy"><span className="scenario-card-number">Scenario 0{index + 1}</span><strong>{item.title}</strong><span>{item.description}</span></span>
          <span className={`scenario-ready ${item.status === 'ready' ? 'is-ready' : ''}`}><i />{listLoading ? 'Loading…' : item.status === 'ready' ? 'Replay ready' : 'Recording unavailable'}</span>
        </button>)}
      </nav>

      {listError && <div className="scenario-error" role="alert"><span>{listError}</span><button className="scenario-button" type="button" onClick={refresh}>Reload recordings</button></div>}

      <section className="scenario-workspace" aria-label={`${selected?.title || 'Scenario'} replay`}>
        <div className="scenario-player-column">
          <div className="scenario-objective"><span>Objective</span><p>{manifest?.objective || selected?.objective}</p></div>
          <div className="scenario-player">
            <div className="scenario-player-top"><span className={`scenario-attempt ${isReference ? 'is-reference' : ''}`}><i />{manifest ? attemptLabel(manifest.provenance) : loading ? 'Loading recording…' : 'Recorded attempt'}</span>
              <div className="scenario-cameras" role="group" aria-label="Camera view">
              {(manifest?.cameras || [{ id: 'side', label: 'Side' }, { id: 'overview', label: 'Overview' }]).map(view => <button type="button" key={view.id} aria-pressed={camera === view.id} disabled={!manifest} onClick={() => { setCamera(view.id); setFailedFrame(''); }}>{view.label || humanize(view.id)}</button>)}
            </div></div>
            <div className={`scenario-viewport ${frameSrc && !imageFailed ? 'has-frame' : ''}`}>
              {frameSrc && !imageFailed ? <img src={frameSrc} alt={`${selected?.title}, ${humanize(camera)} camera, ${time.toFixed(2)} seconds`} draggable="false" onError={() => {
                if (frameSrcRef.current === frameSrc) { setFailedFrame(frameSrc); setPlaying(false); }
              }} /> : <div className="scenario-empty" role="status"><span className="scenario-empty-icon"><Icon name={selectedId === DEFAULT_SCENARIOS[0].id ? 'dog' : 'drone'} size={42} /></span><h2>{loading ? 'Loading recorded motion…' : imageFailed ? 'This frame could not be loaded' : 'No playable recording yet'}</h2><p>{mediaError || (imageFailed ? 'Try another moment or camera, or reload the saved recording.' : loading ? 'Preparing synchronized camera views and the event timeline.' : 'The simulation recording is not available. Reload once the saved frames are ready.')}</p>{!loading && <button className="scenario-button" type="button" onClick={refresh}><Icon name="restart" />Reload recordings</button>}</div>}
              {manifest && frameSrc && !imageFailed && <><div className="scenario-frame-label"><span>{loop ? 'Inspecting failure · loop' : time >= duration ? 'End of recording' : playing ? 'Playing' : 'Paused'}</span><span>{clockLabel(time)}</span></div><span className="scenario-frame-phase">{humanize(sample?.phase || currentEvent?.label || currentEvent?.event || 'Recorded motion')}</span></>}
            </div>

              {taskPresentation && frameSrc && !imageFailed && <div className={`scenario-task-result ${/FAILED|INCOMPLETE|OFF COURSE/i.test(taskPresentation.status) ? 'is-failed' : ''} ${time >= duration ? 'is-ended' : ''}`} role="status"><strong>{taskPresentation.status}</strong><span>{taskPresentation.detail}</span></div>}
            <div className="scenario-timeline">
              <div className="scenario-timeline-top"><span>{loop ? `Inspection window ${clockLabel(loop.start)}–${clockLabel(loop.end)}` : 'Recorded timeline'}</span><span>{clockLabel(time)} <span className="scenario-time-total">/ {clockLabel(duration)}</span></span></div>
              <div className="scenario-scrubber"><input type="range" aria-label="Scenario replay time" min="0" max={duration || 1} step="0.001" value={time} disabled={!usable} onChange={event => jump(Number(event.target.value))} style={{ '--progress': `${duration ? time / duration * 100 : 0}%` }} />
                <div className="scenario-timeline-markers">{events.map((event, index) => <button type="button" key={`${event.event}-${index}`} aria-label={`Jump to ${event.label || humanize(event.event)} at ${eventTime(event).toFixed(2)} seconds`} title={`${event.label || humanize(event.event)} · ${eventTime(event).toFixed(2)} s`} className={event === failure || event.is_failure ? 'is-failure' : ''} style={{ left: `${duration ? Math.max(0, Math.min(100, eventTime(event) / duration * 100)) : 0}%` }} onClick={() => jump(eventTime(event))} />)}</div>
              </div>
              <div className="scenario-controls"><div className="scenario-transport"><button className="scenario-icon-button" type="button" aria-label="Restart replay" title="Restart replay" disabled={!usable} onClick={() => { setLoop(null); seek(0, true); }}><Icon name="restart" /></button><button className="scenario-icon-button" type="button" aria-label="Previous frame" title="Previous frame" disabled={!usable || time <= frames[0]?.t_s} onClick={() => jump(stepFrame(frames, time, -1))}><Icon name="previous" /></button><button className="scenario-play" type="button" aria-label={playing ? 'Pause replay' : time >= duration && usable ? 'Replay from start' : 'Play replay'} disabled={!usable || imageFailed} onClick={togglePlay}><Icon name={playing ? 'pause' : 'play'} size={21} /></button><button className="scenario-icon-button" type="button" aria-label="Next frame" title="Next frame" disabled={!usable || time >= frames.at(-1)?.t_s} onClick={() => jump(stepFrame(frames, time, 1))}><Icon name="next" /></button></div>
                <button className={`scenario-inspect ${loop ? 'is-active' : ''}`} type="button" aria-pressed={Boolean(loop)} disabled={!inspection} onClick={toggleInspection}><Icon name="loop" size={16} />{loop ? 'Exit inspection' : 'Inspect failure'}</button>
                <label className="scenario-speed"><span>Speed</span><select aria-label="Scenario playback speed" value={speed} disabled={!usable} onChange={event => setSpeed(Number(event.target.value))}>{[0.25, 0.5, 1, 2].map(value => <option key={value} value={value}>{value}×</option>)}</select></label>
              </div>
            </div>
          </div>
          <div className="scenario-caption"><span>{manifest ? `Saved simulation · ${frames.length} frames per camera · ${manifest.fps || '—'} fps` : 'Saved simulation playback'}</span><span>{manifest ? `Frame ${frameIndex + 1} / ${frames.length}` : 'Playback opens paused'}</span></div>
          {manifest?.effects?.enabled && <p className="scenario-effect-note">Visual effect: {manifest.effects.description || 'Impact-triggered crash effect.'}</p>}
        </div>

        <aside className="scenario-evidence" aria-label="Recorded evidence">
          <div className="scenario-evidence-heading"><p className="scenario-eyebrow">{isReference || isCandidate ? 'Follow the motion' : 'Follow the failure'}</p><h2>Key moments</h2><p>Jump to an event to examine the motion.</p></div>
          <ol className="scenario-events">{events.length ? events.map((event, index) => <li key={`${event.event}-${index}`} className={`${currentEvent === event ? 'is-current' : ''} ${event === failure || event.is_failure ? 'is-failure' : ''}`}><button type="button" onClick={() => jump(eventTime(event))} aria-current={currentEvent === event ? 'step' : undefined}><span className="scenario-event-dot" /><span><strong>{event.label || humanize(event.event)}</strong><small>{clockLabel(eventTime(event))}{event === failure ? ' · First failure' : ''}</small></span><span className="scenario-event-arrow" aria-hidden="true">↗</span></button></li>) : <li className="scenario-events-empty">{loading ? 'Loading saved events…' : 'Events appear with the recording.'}</li>}</ol>
          <div className="scenario-observation"><div className="scenario-observation-heading"><h3>At this moment</h3><span>{clockLabel(time)}</span></div><dl><div><dt>Phase</dt><dd>{sample?.phase ? humanize(sample.phase) : '—'}</dd></div>{Number.isFinite(sample?.task_requested_speed_mps ?? sample?.requested_speed_mps) && <div><dt>Requested speed</dt><dd>{(sample.task_requested_speed_mps ?? sample.requested_speed_mps).toFixed(2)} m/s</dd></div>}<div><dt>Speed</dt><dd>{observationSpeed === null ? '—' : `${observationSpeed} m/s`}</dd></div>{Number.isFinite(observationTilt) && <div><dt>Body tilt</dt><dd>{observationTilt.toFixed(1)}°</dd></div>}{selectedId.startsWith('car_') ? <div><dt>{Number.isFinite(sample?.bumper_clearance) ? 'Barrier clearance' : 'Lane error'}</dt><dd>{Number.isFinite(sample?.bumper_clearance ?? sample?.lateral_error) ? `${(sample.bumper_clearance ?? sample.lateral_error).toFixed(2)} m` : '—'}</dd></div> : <div><dt>Height</dt><dd>{Number.isFinite(sample?.position?.[2]) ? `${sample.position[2].toFixed(2)} m` : '—'}</dd></div>}</dl></div>
          {footContacts && <div className="scenario-foot-contacts"><h3>Ground contact</h3><div className="scenario-contact-feet" role="list" aria-label="Recorded foot contacts">{[['FL', 'Front left'], ['FR', 'Front right'], ['RL', 'Rear left'], ['RR', 'Rear right']].map(([id, name]) => <span role="listitem" key={id} className={footContacts[id] === true ? 'is-contact' : ''} aria-label={`${name}: ${footContacts[id] === true ? 'ground contact' : footContacts[id] === false ? 'no ground contact' : 'not recorded'}`}><i aria-hidden="true" /><strong>{id}</strong><small>{footContacts[id] === true ? 'On' : footContacts[id] === false ? 'Off' : '—'}</small></span>)}</div></div>}
          {(summary.length > 0 || outcomeMetrics.length > 0) && <details className="scenario-outcome"><summary>Recorded outcome <span aria-hidden="true">+</span></summary><dl>{summary.map(([key, value]) => <div key={key}><dt>{humanize(key)}</dt><dd>{metricValue(value)}</dd></div>)}{outcomeMetrics.map(([key, label, unit]) => <div key={`metric-${key}`}><dt>{label}</dt><dd>{metricValue(manifest.summary.metrics[key])}{unit}</dd></div>)}</dl></details>}
          <div className="scenario-evidence-note"><span className="scenario-note-dot" />{isReference ? 'This correction is a developer-authored reference. Astra repair has not been run.' : isCandidate ? 'This is a saved candidate attempt. Astra repair has not been run.' : 'Replay uses saved simulation data. Astra repair has not been run.'}</div>
        </aside>
      </section>
    </main>
  </div>;
}
