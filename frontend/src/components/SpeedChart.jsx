import { useLayoutEffect, useMemo, useRef, useState } from 'react';
import { numeric } from '../api.js';

export const TRACKS = [
  { id: 'candidate', label: 'Candidate', color: '#0864ee' },
  { id: 'reference', label: 'Reference', color: '#7b8797' },
  { id: 'original', label: 'Original', color: '#bf9460' },
];

export function trialRows(trace) {
  const rows = trace?.observations ?? [];
  return rows.filter(row => row.phase === 'trial' && numeric(row.phase_time)).map(row => ({
    t: row.phase_time,
    speed: Array.isArray(row.velocity) ? Math.hypot(...row.velocity) : row.speed_mps,
    x: row.front_x,
  })).filter(row => numeric(row.speed));
}

export default function SpeedChart({ traces, time = 0, onSeek, large = false, quantity = 'speed' }) {
  const containerRef = useRef(null);
  const [size, setSize] = useState({ width: 900, height: large ? 290 : 192 });
  useLayoutEffect(() => {
    const container = containerRef.current;
    const measure = () => {
      const bounds = container.getBoundingClientRect();
      const next = { width: Math.max(280, Math.round(bounds.width)), height: Math.max(150, Math.round(bounds.height)) };
      setSize(current => current.width === next.width && current.height === next.height ? current : next);
    };
    measure();
    const observer = new ResizeObserver(measure);
    observer.observe(container);
    return () => observer.disconnect();
  }, []);
  const series = useMemo(() => TRACKS.map(track => ({ ...track, rows: trialRows(traces[track.id]) })), [traces]);
  const points = series.flatMap(track => track.rows);
  const containerClass = `speed-chart-container ${large ? 'speed-chart-container-large' : ''}`;
  if (!points.length) return <div ref={containerRef} className={containerClass}><div className="chart-empty">Recorded telemetry will appear when a trial is available.</div></div>;
  const width = size.width, height = large ? size.height : 192;
  const margin = { left: 48, right: 14, top: 12, bottom: 38 };
  const chartWidth = width - margin.left - margin.right;
  const chartHeight = height - margin.top - margin.bottom;
  const maxTime = Math.max(0.1, ...points.map(point => point.t));
  const value = point => quantity === 'position' ? point.x : point.speed;
  const peakSpeed = Math.max(...points.map(value).filter(numeric));
  const speedStep = quantity === 'position' ? peakSpeed > 80 ? 50 : peakSpeed > 20 ? 20 : 5 : peakSpeed > 20 ? 10 : 5;
  const maxSpeed = Math.max(speedStep, Math.ceil(peakSpeed / speedStep) * speedStep);
  const x = value => margin.left + value / maxTime * chartWidth;
  const y = value => margin.top + chartHeight - value / maxSpeed * chartHeight;
  const xTickCount = width < 400 ? 4 : width < 600 ? 5 : 6;
  const xTicks = Array.from({ length: xTickCount }, (_, index) => maxTime * index / (xTickCount - 1));
  const yTicks = Array.from({ length: maxSpeed / speedStep + 1 }, (_, index) => index * speedStep);
  return <div ref={containerRef} className={containerClass}><svg className="speed-chart" viewBox={`0 0 ${width} ${height}`} role="img" aria-label={`Actual recorded ${quantity} over trial time for the original model, candidate, and synthetic reference`} onClick={event => {
    if (!onSeek) return;
    const bounds = event.currentTarget.getBoundingClientRect();
    onSeek(Math.max(0, Math.min(maxTime, ((event.clientX - bounds.left) / bounds.width * width - margin.left) / chartWidth * maxTime)));
  }}>
    {xTicks.map(tick => <g key={tick}><line x1={x(tick)} y1={margin.top} x2={x(tick)} y2={height - margin.bottom} className="chart-grid" /><text x={x(tick)} y={height - margin.bottom + 19} textAnchor="middle">{tick.toFixed(tick < 10 ? 1 : 0)}</text></g>)}
    {yTicks.map(tick => <g key={tick}><line x1={margin.left} y1={y(tick)} x2={width - margin.right} y2={y(tick)} className="chart-grid" /><text x={margin.left - 10} y={y(tick) + 4} textAnchor="end">{tick.toFixed(0)}</text></g>)}
    {series.filter(track => track.rows.length).map(track => <path key={track.id} d={track.rows.filter(point => numeric(value(point))).map((point, index) => `${index ? 'L' : 'M'}${x(point.t).toFixed(2)},${y(value(point)).toFixed(2)}`).join(' ')} stroke={track.color} strokeWidth={track.id === 'candidate' ? 2.6 : 1.8} fill="none" strokeDasharray={track.id === 'original' ? '5 4' : undefined} />)}
    <line x1={x(Math.min(time, maxTime))} x2={x(Math.min(time, maxTime))} y1={margin.top} y2={height - margin.bottom} stroke="#111827" strokeWidth="1" strokeDasharray="3 4" opacity=".5" />
    <text x={width / 2} y={height - 4} textAnchor="middle">Time (s)</text>
    <text transform={`translate(13 ${margin.top + chartHeight / 2}) rotate(-90)`} textAnchor="middle">{quantity === 'position' ? 'Front position (m)' : 'Speed (m/s)'}</text>
  </svg></div>;
}

export function ChartLegend({ traces }) {
  return <div className="chart-legend">{TRACKS.filter(track => trialRows(traces[track.id]).length).map(track => <span key={track.id}><i style={{ background: track.color }} />{track.label}</span>)}</div>;
}
