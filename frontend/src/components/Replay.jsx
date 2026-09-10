import { useEffect, useMemo, useState } from 'react';
import { caseName, getSummary, meters, numeric, request, runPath } from '../api.js';
import { PlayIcon } from './Icons.jsx';
import SpeedChart, { ChartLegend, TRACKS, trialRows } from './SpeedChart.jsx';

function frameAt(frames, time) {
  let left = 0, right = frames.length - 1;
  while (left <= right) {
    const middle = Math.floor((left + right) / 2);
    if (frames[middle].t_s <= time) left = middle + 1;
    else right = middle - 1;
  }
  return frames[Math.max(0, right)];
}

export default function Replay({ run, selectedCase, onSelectCase }) {
  const cases = run?.evaluation?.cases ?? [];
  const caseData = cases.find(item => item.case_id === selectedCase);
  const [track, setTrack] = useState('reference');
  const [traces, setTraces] = useState({});
  const [traceLoading, setTraceLoading] = useState(false);
  const [traceError, setTraceError] = useState('');
  const [manifest, setManifest] = useState(null);
  const [imageError, setImageError] = useState(false);
  const [mediaLoading, setMediaLoading] = useState(false);
  const [time, setTime] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const mediaItem = run?.media?.find(item => item.case_id === selectedCase && item.track === track);
  const manifestUrl = mediaItem?.manifest_url;

  useEffect(() => {
    setTime(0); setPlaying(false); setTraces({}); setTraceError('');
    if (!run?.id || !selectedCase) return;
    let cancelled = false;
    const controller = new AbortController();
    setTraceLoading(true);
    Promise.allSettled(TRACKS.map(async item => [item.id, await request(
      `${runPath(run.id)}/trace?case=${encodeURIComponent(selectedCase)}&track=${item.id}`,
      { signal: controller.signal },
    )])).then(results => {
      if (cancelled) return;
      const fulfilled = results.filter(item => item.status === 'fulfilled').map(item => item.value);
      setTraces(Object.fromEntries(fulfilled));
      if (fulfilled.length === 0) setTraceError('Recorded trajectories are not available for this case yet.');
      else if (fulfilled.length < 3) setTraceError('Some model traces are unavailable; the chart shows the records that exist.');
      setTraceLoading(false);
    });
    return () => { cancelled = true; controller.abort(); };
  }, [run?.id, selectedCase]);

  useEffect(() => {
    setManifest(null); setImageError(false); setMediaLoading(false);
    if (!manifestUrl) return;
    let cancelled = false;
    const controller = new AbortController();
    setMediaLoading(true);
    request(manifestUrl, { signal: controller.signal }).then(value => {
      if (!cancelled) setManifest(value);
    }).catch(() => {}).finally(() => { if (!cancelled) setMediaLoading(false); });
    return () => { cancelled = true; controller.abort(); };
  }, [manifestUrl]);

  const frames = useMemo(() => (manifest?.frames ?? []).filter(frame => numeric(frame.t_s)), [manifest]);
  const duration = useMemo(() => Math.max(0, manifest?.duration_s || 0,
    ...Object.values(traces).flatMap(value => trialRows(value).map(row => row.t))), [traces, manifest]);
  const frame = frames.length ? frameAt(frames, time) : null;
  const frameSrc = frame && manifestUrl ? new URL(frame.url || frame.file, new URL(manifestUrl, window.location.origin)).href : '';
  const hasFrames = Boolean(frameSrc && !imageError);
  const summary = getSummary(caseData?.[track]);
  const config = caseData?.config ?? {};

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
    <h1 id="replay-title">Car replay</h1>
    <div className="replay-selectors">
      <label>Case<select value={selectedCase || ''} onChange={event => onSelectCase(event.target.value)} disabled={!cases.length}>
        {!cases.length && <option value="">No recorded cases yet</option>}
        {cases.map((item, index) => <option key={item.case_id} value={item.case_id}>{caseName(item, index)}</option>)}
      </select></label>
      <label>Track<select value={track} onChange={event => setTrack(event.target.value)} disabled={!caseData}>
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
    <div className={`replay-media ${hasFrames ? '' : 'replay-media-telemetry'}`}>
      {hasFrames ? <img src={frameSrc} alt={`${track} MuJoCo replay for ${caseName(caseData)} at ${time.toFixed(1)} seconds`} onError={() => setImageError(true)} />
        : caseData && Object.keys(traces).length ? <SpeedChart traces={traces} time={time} onSeek={seek} large quantity="position" />
          : <div className="empty-message"><strong>{traceLoading || mediaLoading ? 'Loading recorded motion…' : run?.active ? 'The investigation is running' : 'No replay available yet'}</strong><p>{run?.active ? 'Follow the investigator as it tests and edits the component. Reserved replays appear after predictions are frozen and evaluated.' : 'Select a completed run or start an investigation to inspect recorded car motion.'}</p></div>}
    </div>
    <div className="media-caption">
      <span>{hasFrames ? 'Recorded MuJoCo replay' : traceLoading ? 'Loading telemetry…' : Object.keys(traces).length ? mediaLoading ? 'Loading replay frames; showing recorded telemetry' : 'Recorded telemetry · replay frames unavailable' : 'Synthetic experiment'}</span>
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
    <div className="chart-heading"><h2>Speed over time</h2><ChartLegend traces={traces} /></div>
    <SpeedChart traces={traces} time={time} onSeek={seek} />
    {caseData && <div className="case-metrics" aria-label="Selected case stopping predictions">
      {TRACKS.map(item => <span key={item.id}><i style={{ background: item.color }} />{item.label}: <strong>{getSummary(caseData[item.id]).censored ? 'censored' : meters(getSummary(caseData[item.id]).stopping_distance)}</strong></span>)}
      {numeric(caseData.candidate_error_m) && <span>Candidate error: <strong>{meters(caseData.candidate_error_m)}</strong></span>}
      {numeric(caseData.distance_tolerance_m) && <span>Distance tolerance: <strong>{meters(caseData.distance_tolerance_m)}</strong><strong className={caseData.within_tolerance === false ? 'text-fail' : caseData.within_tolerance === true ? 'text-pass' : ''}>{caseData.within_tolerance === true ? 'Passed' : caseData.within_tolerance === false ? 'Failed' : 'Not scored'}</strong></span>}
    </div>}
  </section>;
}
