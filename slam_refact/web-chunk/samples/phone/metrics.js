// In-memory metrics of the phone page. Nothing is stored or sent: the JSON leaves the page only when the user clicks the download button.
// It carries counters, intervals and status transitions only - no camera image, no IMU/DeviceMotion sample.
export const percentile = (sorted, p) => (sorted.length ? sorted[Math.min(sorted.length - 1, Math.floor(p * sorted.length))] : null);

export const summarize = (values) => {
  const s = values.slice().sort((a, b) => a - b);
  return { n: s.length, p50: percentile(s, 0.5), p95: percentile(s, 0.95), p99: percentile(s, 0.99), max: s.length ? s[s.length - 1] : null };
};

const CAP = 20000;  // per series, so a long session cannot grow without bound

export function createMetrics(now = () => performance.now()) {
  const m = {
    startedAt: null, firstNormalAt: null, lastUpdateAt: null,
    updates: 0, nullUpdates: 0, resets: 0, hitTests: 0, hitMisses: 0, placed: 0,
    intervalsPre: [], intervalsPost: [], worldPoints: [], transitions: [], events: [], imageEvents: { found: 0, updated: 0, lost: 0 }, errors: 0,
  };
  const push = (arr, v) => { if (arr.length < CAP) arr.push(v); };
  return {
    start() { m.startedAt = now(); },
    update(reality, nPoints) {
      const t = now();
      if (!reality) { m.nullUpdates++; return; }
      m.updates++;
      if (m.lastUpdateAt != null) push(m.firstNormalAt == null ? m.intervalsPre : m.intervalsPost, t - m.lastUpdateAt);
      m.lastUpdateAt = t;
      if (m.firstNormalAt == null && reality.trackingStatus === 'NORMAL') m.firstNormalAt = t;
      push(m.worldPoints, nPoints);
    },
    transition(status, reason) { push(m.transitions, { tMs: m.startedAt == null ? null : Math.round(now() - m.startedAt), status, reason }); },
    event(name, detail) { push(m.events, { tMs: m.startedAt == null ? null : Math.round(now() - m.startedAt), name, ...detail }); },
    image(kind) { m.imageEvents[kind]++; },
    hit(ok) { m.hitTests++; if (!ok) m.hitMisses++; },
    placed() { m.placed++; },
    error() { m.errors++; },
    snapshot(extra = {}) {
      return {
        schema: 'slam-refact-phone-metrics-v1',
        note: 'Counters and intervals only. No camera image and no IMU sample. Produced by the user clicking the download button. Interval unit: ms between pipeline onUpdate calls with a reality result (not engine time).',
        label: 'page-observed (rAF/onUpdate cadence), device and input kind are recorded by the user, not by this page',
        ...extra,
        updates: m.updates, nullUpdates: m.nullUpdates, hitTests: m.hitTests, hitMisses: m.hitMisses, placed: m.placed,
        imageEvents: m.imageEvents, errors: m.errors,
        timeToFirstNormalMs: m.firstNormalAt != null && m.startedAt != null ? Math.round(m.firstNormalAt - m.startedAt) : null,
        updateIntervalMs: { beforeFirstNormal: summarize(m.intervalsPre), afterFirstNormal: summarize(m.intervalsPost) },
        worldPoints: summarize(m.worldPoints),
        transitions: m.transitions, events: m.events,
      };
    },
  };
}
