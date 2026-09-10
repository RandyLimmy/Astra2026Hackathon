import { AGENTS, TASKS, batchComparisonId, readable } from '../paired.js';

export default function BatchOverview({ batch, comparisons, selectedId, platformId, onSelect }) {
  return <section className="batch-overview" aria-label={`${TASKS.length}-scenario batch status`}><div className="batch-heading"><strong>{batch ? 'Batch progress' : `Ready to compare all ${TASKS.length}`}</strong><span>{batch ? readable(batch.status) || 'Preparing' : `${TASKS.length} scenarios × ${AGENTS.length} models · parallel calls`}</span></div><div className="batch-tasks" style={{ '--task-count': TASKS.length }}>{TASKS.map(task => {
    const pairId = batchComparisonId(batch, task.platform);
    const comparison = comparisons.find(item => item.id === pairId);
    const selected = pairId ? selectedId === pairId : task.platform === platformId;
    return <button key={task.platform} className={`batch-task ${selected ? 'selected' : ''}`} type="button" aria-pressed={selected} onClick={() => onSelect(task.platform, pairId)}>
      <strong>{task.label}</strong><span className="batch-agents">{AGENTS.map(agent => {
        const state = comparison?.run_statuses?.[agent.id];
        const label = state?.active ? 'Running' : readable(state?.status) || (pairId ? readable(comparison?.status) || 'Starting' : 'Ready');
        return <span key={agent.id} className={agent.id}><i aria-hidden="true" />{agent.label}<span>{label}</span></span>;
      })}</span>
    </button>;
  })}</div></section>;
}
