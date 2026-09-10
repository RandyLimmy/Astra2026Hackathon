import { useCallback, useEffect, useState } from 'react';
import { request, runPath } from './api.js';

export function useRuns() {
  const [data, setData] = useState({ runs: [], active_run_id: null });
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState('');
  const refresh = useCallback(async () => {
    try {
      const next = await request('/api/runs');
      setData(next);
      setError('');
    } catch (failure) {
      setError(failure.message);
    } finally {
      setLoading(false);
    }
  }, []);
  useEffect(() => {
    refresh();
    const timer = setInterval(refresh, 3000);
    return () => clearInterval(timer);
  }, [refresh]);
  return { ...data, loading, error, refresh };
}

export function useRun(id) {
  const [run, setRun] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  useEffect(() => {
    setRun(null);
    setError('');
    if (!id) return;
    let cancelled = false;
    const controller = new AbortController();
    setLoading(true);
    async function refresh() {
      try {
        const next = await request(runPath(id), { signal: controller.signal });
        if (!cancelled) { setRun(next); setError(''); }
      } catch (failure) {
        if (!cancelled && failure.name !== 'AbortError') setError(failure.message);
      } finally {
        if (!cancelled) setLoading(false);
      }
    }
    refresh();
    const timer = setInterval(refresh, 3000);
    return () => { cancelled = true; controller.abort(); clearInterval(timer); };
  }, [id]);
  return { run, loading, error };
}
