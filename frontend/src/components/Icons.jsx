export function Chevron({ size = 16, ...props }) {
  return <svg width={size} height={size} viewBox="0 0 20 20" fill="none" aria-hidden="true" {...props}><path d="m5 7.5 5 5 5-5" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round" /></svg>;
}

export function PlayIcon({ paused = true }) {
  return <svg width="18" height="18" viewBox="0 0 20 20" fill="currentColor" aria-hidden="true">{paused ? <path d="M5 3.5a.7.7 0 0 1 1.05-.6l10 6.5a.7.7 0 0 1 0 1.2l-10 6.5a.7.7 0 0 1-1.05-.6Z" /> : <><rect x="4" y="3" width="4" height="14" rx=".6" /><rect x="12" y="3" width="4" height="14" rx=".6" /></>}</svg>;
}
