import { useEffect, useMemo, useState } from 'react';
import { caseName, getSummary, meters, numeric, request, runPath } from '../api.js';
import { frameAt, replayFrames, validManifest, validTrace } from '../replay.js';
import { PlayIcon } from './Icons.jsx';
import SpeedChart, { ChartLegend, TRACKS, trialRows } from './SpeedChart.jsx';

export default function Replay(props) {
  const [track, setTrack] = useState('reference');
  // Discard the previous scenario's traces and playback before painting a new selection.
  return <ScenarioReplay key={JSON.stringify([props.run?.id, props.selectedCase])} {...props} track={track} setTrack={setTrack} />;
}

function ScenarioReplay({ run, selectedCase, onSelectCase, track, setTrack }) {
  const recordedCases = run?.evaluation?.cases ?? [];
  const cases = recordedCases.filter(item => typeof item.case_id === 'string' && numeric(item.config?.speed_mps));
  const unsupportedReplay = run?.metadata?.kind === 'platform_investigation'
    || run?.evaluation?.kind === 'platform_repair_verification' || recordedCases.length > 0 && !cases.length;
  const caseData = cases.find(item => item.case_id === selectedCase);
  const [traces, setTraces] = useState({});
  const [traceLoading, setTraceLoading] = useState(false);
  const [traceError, setTraceError] = useState('');
  const [media, setMedia] = useState(null);
  const [imageError, setImageError] = useState('');
  const [mediaError, setMediaError] = useState('');
  const [mediaLoading, setMediaLoading] = useState(false);
  const [time, setTime] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [quantity, setQuantity] = useState('position');
  const mediaItem = run?.media?.find(item => item.case_id === selectedCase && item.track === track);
  const manifestUrl = mediaItem?.manifest_url;
  const caseSignature = JSON.stringify(caseData ?? null);
  const manifest = media?.url === manifestUrl && media?.caseSignature === caseSignature ? media.value : null;

  useEffect(() => {
    setTraces({}); setTraceError(''); setTraceLoading(false);
    if (!run?.id || !caseData) return;
    const selected = JSON.parse(caseSignature);
    let cancelled = false;
    let timer;
    const controller = new AbortController();
    setTraceLoading(true);
    async function refresh() {
      const expectedTracks = TRACKS.filter(item => selected[item.id]);
      const results = await Promise.allSettled(expectedTracks.map(async item => {
        const value = await request(`${runPath(run.id)}/trace?case=${encodeURIComponent(selectedCase)}&track=${item.id}`,
          { signal: controller.signal });
        if (!validTrace(value, selected, item.id)) throw new Error('Trace does not match this scenario.');
        return [item.id, value];
      }));
      if (cancelled) return;
      const fulfilled = results.filter(item => item.status === 'fulfilled').map(item => item.value);
      setTraces(Object.fromEntries(fulfilled));
      setTraceError(fulfilled.length === 0 ? 'Verified trajectories are not available for this scenario yet.'
        : fulfilled.length < TRACKS.length ? 'Some model traces are unavailable; showing verified records only.' : '');
      setTraceLoading(false);
      // Artifacts can arrive after the run details, or be briefly unreadable while written.
      if (fulfilled.length < expectedTracks.length) timer = setTimeout(refresh, 3000);
    }
    refresh();
    return () => { cancelled = true; controller.abort(); clearTimeout(timer); };
  }, [run?.id, selectedCase, caseSignature]);

  useEffect(() => {
    setMedia(null); setImageError(''); setMediaError(''); setMediaLoading(false);
    if (!manifestUrl || !caseData) return;
    const selected = JSON.parse(caseSignature);
    let cancelled = false;
    const controller = new AbortController();
    setMediaLoading(true);
    request(manifestUrl, { signal: controller.signal }).then(value => {
      if (!validManifest(value, selected, track) || !replayFrames(value).length) {
        throw new Error('Replay frames could not be verified for this scenario and model.');
      }
      if (!cancelled) setMedia({ url: manifestUrl, caseSignature, value });
    }).catch(failure => { if (!cancelled) setMediaError(failure.message); })
      .finally(() => { if (!cancelled) setMediaLoading(false); });
    return () => { cancelled = true; controller.abort(); };
  }, [manifestUrl, caseSignature, track]);

  const frames = useMemo(() => replayFrames(manifest), [manifest]);
  const duration = useMemo(() => Math.max(0, manifest?.duration_s || 0,
    ...trialRows(traces[track]).map(row => row.t)), [traces, manifest, track]);
  const frame = frames.length ? frameAt(frames, time) : null;
  const frameSrc = frame && manifestUrl ? new URL(frame.url || frame.file, new URL(manifestUrl, window.location.origin)).href : '';
  const hasFrames = Boolean(frameSrc && imageError !== manifestUrl);
  const summary = getSummary(caseData?.[track]);
  const config = caseData?.config ?? {};
  const trackLabel = TRACKS.find(item => item.id === track)?.label;
  const hasTraces = Object.values(traces).some(value => trialRows(value).length);

  useEffect(() => {
    if (!playing || duration <= 0) return;
    let handle;
    let previous = performance.now();
    function advance(now) {
      const delta = (now - previous) / 1000 * speed;
      previous = now;
      setTime(current => {
        const next = Math.min(duration, current + delta);
        if (next >= duration) setPlaying(false);
        return next;
      });
      handle = requestAnimationFrame(advance);
    }
    handle = requestAnimationFrame(advance);
    return () => cancelAnimationFrame(handle);
  }, [playing, speed, duration]);

  const seek = next => { setTime(Math.max(0, Math.min(duration, next))); setPlaying(false); };
  return <section className="replay-panel" aria-labelledby="replay-title">
    <h1 id="replay-title">Scenario replay</h1>
    <div className="replay-selectors">
      <label>Scenario<select value={selectedCase || ''} onChange={event => onSelectCase(event.target.value)} disabled={!cases.length}>
        {!cases.length && <option value="">No recorded cases yet</option>}
        {cases.map((item, index) => <option key={item.case_id} value={item.case_id}>{caseName(item, index)}</option>)}
      </select></label>
      <label>Replay model<select value={track} onChange={event => { setTrack(event.target.value); setTime(0); setPlaying(false); }} disabled={!caseData}>
        <option value="reference">Reference</option><option value="candidate">Candidate</option><option value="original">Original model</option>
      </select></label>
    </div>
    {caseData && <div className="case-context">
      {numeric(config.speed_mps) && <span>{config.speed_mps} m/s</span>}
      {numeric(config.brake_strength) && <span>{Math.round(config.brake_strength * 100)}% brake</span>}
      {numeric(config.preparation_cycles) && <span>{config.preparation_cycles} preparation {config.preparation_cycles === 1 ? 'cycle' : 'cycles'}</span>}
      {numeric(config.wait_s) && <span>{config.wait_s} s rest</span>}
      {'wall_distance_m' in config && <span>{config.wall_distance_m === null ? 'No wall' : `${config.wall_distance_m} m wall gap`}</span>}
    </div>}
    {!hasFrames && hasTraces && <div className="telemetry-controls">
      <div className="quantity-toggle" aria-label="Telemetry measurement">{['position', 'speed'].map(value => <button type="button" key={value} className={quantity === value ? 'is-selected' : ''} aria-pressed={quantity === value} onClick={() => setQuantity(value)}>{value === 'position' ? 'Position' : 'Speed'}</button>)}</div>
      <ChartLegend traces={traces} selectedTrack={track} />
    </div>}
    <div className={`replay-media ${hasFrames ? '' : 'replay-media-telemetry'}`}>
      {hasFrames ? <img key={manifestUrl} src={frameSrc} alt={`${trackLabel} MuJoCo replay for ${caseName(caseData)} at ${frame.t_s.toFixed(2)} seconds`} onError={() => setImageError(manifestUrl)} />
        : caseData && hasTraces ? <SpeedChart traces={traces} time={time} onSeek={seek} large quantity={quantity} selectedTrack={track} />
          : <div className="empty-message"><strong>{unsupportedReplay ? 'Replay is not available for this experiment type' : traceLoading || mediaLoading ? 'Loading recorded motion…' : run?.active ? 'The investigation is running' : 'No replay available yet'}</strong><p>{unsupportedReplay ? 'This viewer supports recorded car braking scenarios. Open Investigation to inspect this run’s evidence.' : run?.active ? 'Follow the investigator as it tests and edits the component. Reserved replays appear after predictions are frozen and evaluated.' : 'Select a completed run or start an investigation to inspect recorded car motion.'}</p></div>}
    </div>
    <div className="media-caption">
      <span>{hasFrames ? `${trackLabel} · verified MuJoCo rerender` : traceLoading ? 'Loading telemetry…' : hasTraces ? mediaLoading ? 'Loading replay frames; showing recorded telemetry' : `${trackLabel} · recorded telemetry` : 'Synthetic experiment'}</span>
      {caseData && <span>{summary.censored ? 'Stopping distance censored' : numeric(summary.stopping_distance) ? `Stop: ${meters(summary.stopping_distance)}` : 'Stop not recorded'}</span>}
    </div>
    <div className="playback-controls">
      <button className="primary play-button" type="button" disabled={!duration} aria-label={playing ? 'Pause replay' : time >= duration && duration > 0 ? 'Replay from start' : 'Play replay'} onClick={() => {
        if (!playing && time >= duration) setTime(0);
        setPlaying(current => !current);
      }}><PlayIcon paused={!playing} /></button>
      <input aria-label="Replay time" type="range" min="0" max={duration || 1} step="0.01" value={time} disabled={!duration} onChange={event => seek(Number(event.target.value))} style={{ '--progress': `${duration ? time / duration * 100 : 0}%` }} />
      <span className="playback-time">{time.toFixed(1)} s / {duration.toFixed(1)} s</span>
      <label className="speed-control">Speed<select aria-label="Replay speed" value={speed} onChange={event => setSpeed(Number(event.target.value))}><option value="0.5">0.5x</option><option value="1">1x</option><option value="2">2x</option></select></label>
    </div>
    {traceError && <p className="inline-note" role="status">{traceError}</p>}
    {(mediaError || imageError === manifestUrl && manifestUrl) && <p className="inline-note" role="status">Replay frames unavailable; showing recorded telemetry.</p>}
    {hasTraces && !traces[track] && <p className="inline-note" role="status">{trackLabel} telemetry is unavailable. Other models are shown for comparison.</p>}
    {hasFrames && <details className="telemetry-details"><summary>Speed over time</summary>
      <ChartLegend traces={traces} selectedTrack={track} />
      <SpeedChart traces={traces} time={time} onSeek={seek} selectedTrack={track} />
    </details>}
    {caseData && <div className="case-metrics" aria-label="Selected case stopping predictions">
      {TRACKS.map(item => <span key={item.id}><i style={{ background: item.color }} />{item.label}: <strong>{getSummary(caseData[item.id]).censored ? 'censored' : meters(getSummary(caseData[item.id]).stopping_distance)}</strong></span>)}
      {numeric(caseData.candidate_error_m) && <span>Candidate error: <strong>{meters(caseData.candidate_error_m)}</strong></span>}
      {numeric(caseData.distance_tolerance_m) && <span>Distance tolerance: <strong>{meters(caseData.distance_tolerance_m)}</strong><strong className={caseData.within_tolerance === false ? 'text-fail' : caseData.within_tolerance === true ? 'text-pass' : ''}>{caseData.within_tolerance === true ? 'Passed' : caseData.within_tolerance === false ? 'Failed' : 'Not scored'}</strong></span>}
    </div>}
  </section>;
}
