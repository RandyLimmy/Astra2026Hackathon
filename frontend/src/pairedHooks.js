import { useCallback, useEffect, useRef, useState } from 'react';
import { request } from './api.js';
import { comparisonPath } from './paired.js';

function useResource(path, interval = 0) {
  const [state, setState] = useState({ path, data: null, loading: Boolean(path), error: '' });
  const [revision, setRevision] = useState(0);
  const refresh = useCallback(() => setRevision(current => current + 1), []);
  useEffect(() => {
    let cancelled = false;
    let timer;
    const controller = new AbortController();
    setState(current => ({ path, data: current.path === path ? current.data : null, loading: Boolean(path), error: '' }));
    if (!path) return () => controller.abort();
    async function poll() {
      try {
        const data = await request(path, { signal: controller.signal });
        if (!cancelled) setState({ path, data, loading: false, error: '' });
      } catch (error) {
        if (!cancelled && error.name !== 'AbortError') setState(current => ({ ...current, loading: false, error: error.message }));
      } finally {
        if (!cancelled && interval) timer = setTimeout(poll, interval);
      }
    }
    poll();
    return () => { cancelled = true; clearTimeout(timer); controller.abort(); };
  }, [path, interval, revision]);
  return { ...(state.path === path ? state : { data: null, loading: Boolean(path), error: '' }), refresh };
}
export const useScenarios = () => useResource('/api/scenarios');
export const useComparisons = () => useResource('/api/comparisons', 3000);
export const useComparison = id => useResource(id ? comparisonPath(id) : null, 2500);

export function usePlayback(duration) {
  const [time, setTime] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const timeRef = useRef(0);
  useEffect(() => {
    if (!playing || duration <= 0) return;
    let frame;
    let previous = performance.now();
    function advance(now) {
      const next = Math.min(duration, timeRef.current + (now - previous) / 1000 * speed);
      previous = now;
      timeRef.current = next;
      setTime(next);
      if (next >= duration) setPlaying(false);
      else frame = requestAnimationFrame(advance);
    }
    frame = requestAnimationFrame(advance);
    return () => cancelAnimationFrame(frame);
  }, [playing, duration, speed]);
  const seek = next => { timeRef.current = Math.max(0, Math.min(duration, next)); setTime(timeRef.current); setPlaying(false); };
  const toggle = () => { if (timeRef.current >= duration) seek(0); setPlaying(current => !current); };
  return { time: Math.min(time, duration), playing, speed, setSpeed, seek, toggle, duration };
}
