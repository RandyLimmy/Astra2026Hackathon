import { useCallback, useEffect, useRef, useState } from 'react';
import { request, runPath } from './api.js';

export function useRuns() {
  const [data, setData] = useState({ runs: [], active_run_id: null });
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState('');
  const latestRequest = useRef(0);
  const refresh = useCallback(async () => {
    const requestId = ++latestRequest.current;
    try {
      const next = await request('/api/runs');
      if (requestId !== latestRequest.current) return;
      setData(next);
      setError('');
    } catch (failure) {
      if (requestId === latestRequest.current) setError(failure.message);
    } finally {
      if (requestId === latestRequest.current) setLoading(false);
    }
  }, []);
  useEffect(() => {
    let cancelled = false;
    let timer;
    async function poll() {
      await refresh();
      if (!cancelled) timer = setTimeout(poll, 3000);
    }
    poll();
    return () => { cancelled = true; ++latestRequest.current; clearTimeout(timer); };
  }, [refresh]);
  return { ...data, loading, error, refresh };
}

export function useRun(id) {
  const [state, setState] = useState({ id, run: null, loading: Boolean(id), error: '' });
  useEffect(() => {
    setState({ id, run: null, loading: Boolean(id), error: '' });
    if (!id) return;
    let cancelled = false;
    let timer;
    const controller = new AbortController();
    async function refresh() {
      try {
        const next = await request(runPath(id), { signal: controller.signal });
        if (next.id !== id) throw new Error('The server returned a different experiment.');
        if (!cancelled) setState({ id, run: next, loading: false, error: '' });
      } catch (failure) {
        if (!cancelled && failure.name !== 'AbortError') {
          setState(current => ({ ...current, loading: false, error: failure.message }));
        }
      } finally {
        // Serial polling prevents a slow older response from replacing newer run state.
        if (!cancelled) timer = setTimeout(refresh, 3000);
      }
    }
    refresh();
    return () => { cancelled = true; controller.abort(); clearTimeout(timer); };
  }, [id]);
  // Effects run after render, so hide the previous run immediately on selection.
  return state.id === id ? state : { run: null, loading: Boolean(id), error: '' };
}
