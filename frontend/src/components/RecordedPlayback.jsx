import { useMemo, useState } from 'react';
import { frameAt } from '../replay.js';
import { clipDuration, recordedFrames } from '../paired.js';
import { usePlayback } from '../pairedHooks.js';
import { PlayIcon } from './Icons.jsx';

export function PlaybackControls({ playback, label = 'Replay' }) {
  const { duration, time, playing, toggle, seek, speed, setSpeed } = playback;
  return <div className="playback-controls">
    <button className="play-button" type="button" disabled={!duration} aria-label={`${playing ? 'Pause' : time >= duration && duration > 0 ? 'Restart' : 'Play'} ${label.toLowerCase()}`} onClick={toggle}><PlayIcon paused={!playing} /></button>
    <span className="playback-time">{time.toFixed(1)} / {duration.toFixed(1)} s</span>
    <input aria-label={`${label} time`} type="range" min="0" max={duration || 1} step="0.01" value={time} disabled={!duration} onChange={event => seek(Number(event.target.value))} style={{ '--progress': `${duration ? time / duration * 100 : 0}%` }} />
    <select aria-label={`${label} speed`} className="speed-select" value={speed} onChange={event => setSpeed(Number(event.target.value))}><option value="0.5">0.5×</option><option value="1">1×</option><option value="2">2×</option></select>
  </div>;
}

export function RecordedFrame({ replay, time = 0, label, pending = false, compact = false }) {
  const frames = useMemo(() => recordedFrames(replay), [replay]);
  const frame = frameAt(frames, time);
  const [broken, setBroken] = useState(null);
  const hasFrame = frame && broken !== frame.url;
  return <div className={`recorded-frame ${compact ? 'compact-frame' : ''}`}>
    {hasFrame ? <img src={frame.url} alt={`${label}, recorded at ${frame.t_s.toFixed(2)} seconds`} onError={() => setBroken(frame.url)} /> : <div className="media-empty"><strong>{broken ? 'Recording frame unavailable' : pending ? 'Recording in progress' : 'No recording yet'}</strong><p>{broken ? 'The saved frame could not be loaded.' : pending ? 'Recorded motion appears as the investigation progresses.' : 'Real scenario footage will appear here after a run.'}</p></div>}
    {hasFrame && <span className="frame-time">{frame.t_s.toFixed(1)} s{time > frames.at(-1).t_s ? ' · clip ended' : ''}</span>}
  </div>;
}

export default function RecordedPlayback({ replay, label, pending }) {
  const playback = usePlayback(clipDuration(replay));
  return <div className="single-playback"><RecordedFrame replay={replay} time={playback.time} label={label} pending={pending} /><PlaybackControls playback={playback} label={label} /></div>;
}
