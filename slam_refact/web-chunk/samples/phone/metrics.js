// In-memory metrics of the phone page. Nothing is stored or sent: the JSON leaves the page only when the user clicks the download button.
// It carries counters, intervals and status transitions only - no camera image, no IMU/DeviceMotion sample.
export const percentile = (sorted, p) => (sorted.length ? sorted[Math.min(sorted.length - 1, Math.floor(p * sorted.length))] : null);

export const summarize = (values) => {
  const s = values.slice().sort((a, b) => a - b);
  return { n: s.length, p50: percentile(s, 0.5), p95: percentile(s, 0.95), p99: percentile(s, 0.99), max: s.length ? s[s.length - 1] : null };
};

const CAP = 20000;  // per series, so a long session cannot grow without bound
export const POSE_LOG_CAP = 40000;  // rows of the opt-in pose log (?record=1): about 20 minutes at 30 Hz
const NOTE = 'Counters and intervals only. No camera image and no IMU sample. Produced by the user clicking the download button. Interval unit: ms between pipeline onUpdate calls with a reality result (not engine time).';
const NOTE_RECORD = ' Recording was on (?record=1): poseLog holds one pose row per onUpdate (pose, status, point count, timestamps; no image, no raw IMU) and receipt holds the camera/device/build identification; both stay in memory until this download.';
const NOTE_TRACKER = ' Tracker diagnostics were on (&diag=1): each row also holds the engine tracker state of the pose source frame and three numbers derived from the DeviceMotion events of that frame (event count, peak gyro magnitude in deg/s, peak linear-acceleration magnitude in m/s^2); no raw IMU sample is stored.';

const num = (v) => (typeof v === 'number' && Number.isFinite(v) ? v : null);
const xyz = (v) => (v ? { x: num(v.x), y: num(v.y), z: num(v.z) } : null);

// What the pose log numbers mean (read by tools/harness/eval/strict_accuracy.py; nothing here is a calibration or a metric claim).
export const POSE_LOG_META = Object.freeze({
  rotation: 'XR8 reality.rotation as an {x, y, z, w} object (quaternion, camera-to-world)',
  position: 'XR8 world, right-handed, y up; responsive units (origin height = 1 unit unless scale: absolute), NOT metres',
  tPerfMs: 'performance.now() at the onUpdate callback (page clock, not exposure time)',
  videoTimeS: 'frameStartResult.videoTime = video.currentTime of the uploaded frame (media clock)',
  frameTimeS: 'frameStartResult.frameTime = pipeline rAF performance.now()/1e3',
  sourceFrameIndex: 'chunk index of the camera frame the published pose was computed from (responses lag one frame); null if unknown. Updated only on fresh responses',
  repeatFrame: 'frameStartResult.repeatFrame: no new camera upload in this callback (NOT the engine pose.repeated flag)',
  source: 'chunk sourceTiming of this callback (fields of build/xr-accuracy-research-20261030/i2-m0/fields.md): the pose source frame, its staged times and freshness; null when delivery diagnostics are off. Pair source.sourceVideoTimeS with the pose, not videoTimeS',
  timeVsSource: 'videoTimeS / frameTimeS belong to the frame uploaded in this callback, NOT to sourceFrameIndex: do not pair them as one frame (off by one)',
});

// The sourceTiming scalars a row keeps (the chunk getter already returns a detached copy; this keeps the row small and fixed).
const SOURCE_TIMING_KEYS = ['kind', 'hasPose', 'fresh', 'engineFresh', 'newEngineResponse', 'repeated', 'publicationRepeated', 'sourceMetadataFound',
  'sourceFrameIndex', 'sessionEpoch', 'currentSessionEpoch', 'sourceVideoTimeS', 'sourceFrameTimeS', 'sourceStageTimeNanos', 'sourceTimeNanos',
  'callbackVideoTimeS', 'callbackFrameTimeS', 'engineReturnedAtMs'];
// The chunk's diag.trackerDiag scalars a row keeps (&diag=1; fixed key list, so a row cannot grow with the chunk object).
export const TRACKER_DIAG_KEYS = Object.freeze(['session', 'sourceFrameIndex', 'mapTrackerState', 'frameTrackerOk', 'ok', 'lossState', 'lostFrames', 'frames',
  'mapReset', 'keyframeCreated', 'numKeyframes', 'groundMatches', 'keypoints', 'pointsTriangulated', 'pointsGround', 'pointsSky', 'pointsDepth', 'pointsCloud',
  'inputFound', 'motionEvents', 'gyroPeakDegS', 'linAccPeakMs2', 'totalResponses', 'totalMapResets', 'totalKeyframesCreated', 'totalMapTrackerNotOk', 'totalInputMissing']);
export const pickTrackerDiag = (td) => {
  if (!td || typeof td !== 'object') return null;
  const out = {};
  for (const k of TRACKER_DIAG_KEYS) { const v = td[k]; out[k] = typeof v === 'number' ? num(v) : typeof v === 'boolean' ? v : null; }
  return out;
};
export const POSE_LOG_META_TRACKER = Object.freeze({
  tracker: 'chunk diag.trackerDiag sampled at this callback, stored once per tracked frame: engine tracker step of (tracker.session, tracker.sourceFrameIndex) (mapTrackerState 1 = map tracked, mapReset = the map was discarded in that frame, keyframeCreated, numKeyframes, points by source) and the DeviceMotion events the engine took for that frame (motionEvents, gyroPeakDegS, linAccPeakMs2). total* are counters since the chunk started: totalResponses rising by more than 1 between two stored values means a response was not sampled',
  trackerNull: 'tracker is null when this callback brought no new tracked frame (same session and sourceFrameIndex as the last stored value), before the first tracked frame, or when the chunk does not record (engine Worker / frameScheduler path)',
  trackerSession: 'tracker.session numbers the chunk session; a rebuilt session (rotation, re-attach) restarts sourceFrameIndex at 0, so a frame is identified by the pair',
});

export const pickSourceTiming = (st) => {
  if (!st || typeof st !== 'object') return null;
  const out = {};
  for (const k of SOURCE_TIMING_KEYS) { const v = st[k]; out[k] = typeof v === 'number' ? num(v) : typeof v === 'boolean' || typeof v === 'string' ? v : null; }
  return out;
};

// Pose log of ?record=1. Rows beyond the cap are not stored but always counted (droppedRows), so a long session is visibly truncated, never silently.
// overheadMs is the time this recorder itself spent building and storing a row (timed with the same clock as the page), so its cost is on record.
export function createPoseRecorder(now = () => performance.now(), cap = POSE_LOG_CAP, trackerDiag = false) {
  const rows = [], overhead = [];
  let droppedRows = 0, lastSession = null, lastSource = null;
  // One stored copy per tracked frame: a callback that samples the same (session, sourceFrameIndex) again stores null.
  const newTrackerSample = (td) => {
    if (!td || typeof td !== 'object' || (td.session === lastSession && td.sourceFrameIndex === lastSource)) return null;
    lastSession = td.session; lastSource = td.sourceFrameIndex;
    return pickTrackerDiag(td);
  };
  return {
    // frameStart is xr.js's frameStartResult: videoTime (video.currentTime, s) and frameTime (performance.now()/1e3, s) when the SDK gives them;
    // repeatFrame marks a callback without a new camera frame. sourceFrameIndex is the chunk's index of the frame the pose was computed from
    // (window.__slamRefactChunk.stats.lastSourceFrameIndex), so repeated / held poses are told apart from fresh ones offline.
    add(reality, frameStart, sourceFrameIndex = null, sourceTiming = null, tracker = null) {
      if (rows.length >= cap) { droppedRows++; return; }
      const t0 = now();
      const p = reality.position, q = reality.rotation;
      const row = {
        tPerfMs: t0, videoTimeS: num(frameStart && frameStart.videoTime), frameTimeS: num(frameStart && frameStart.frameTime),
        repeatFrame: frameStart && typeof frameStart.repeatFrame === 'boolean' ? frameStart.repeatFrame : null, sourceFrameIndex: num(sourceFrameIndex),
        source: pickSourceTiming(sourceTiming),
        status: reality.trackingStatus ?? null, reason: reality.trackingReason ?? null,
        position: xyz(p), rotation: q ? { ...xyz(q), w: num(q.w) } : null,
        worldPoints: Array.isArray(reality.worldPoints) ? reality.worldPoints.length : 0,
      };
      if (trackerDiag) row.tracker = newTrackerSample(tracker);  // &diag=1 only: without it the row is exactly the v3 row
      rows.push(row);
      overhead.push(now() - t0);  // one sample per stored row, so it is bounded by cap as well
    },
    counts: () => ({ rows: rows.length, droppedRows }),
    snapshot(nullUpdates = 0) {
      if (trackerDiag) return { schema: 'slam-refact-xr-poselog-v4', meta: { ...POSE_LOG_META, ...POSE_LOG_META_TRACKER }, rows, droppedRows, cap, nullUpdates, overheadMs: summarize(overhead) };
      return { schema: 'slam-refact-xr-poselog-v3', meta: POSE_LOG_META, rows, droppedRows, cap, nullUpdates, overheadMs: summarize(overhead) };
    },
  };
}

const sha256Hex = async (subtle, text) => {
  const d = await subtle.digest('SHA-256', new TextEncoder().encode(String(text)));
  return Array.from(new Uint8Array(d), (b) => b.toString(16).padStart(2, '0')).join('');
};
const jsonCopy = (v) => (v === undefined ? null : JSON.parse(JSON.stringify(v)));
const guard = async (fn) => { try { return await fn(); } catch (e) { return { unavailable: String(e && e.message || e) }; } };

// What the engine reports it actually runs with (glue getEffectiveConfig, kept by the chunk at session.configurationReadback.effectiveConfig
// when the chunk exposes it). sentAccuracy = the accuracy object the page passed (null in default mode): flags has one entry per sent key
// {sent, effective, ignored}; allProductFlagsActive is true only when every sent flag was read back with the same value and none is in
// ignoredKeys (k1KeepMapActive is the same question for k1KeepMap alone). An engine build that does not know a key is never reported as running it.
// Everything else is { available: false, reason }.
export function readEngineConfigReadback(chunk, sentAccuracy = null) {
  const eff = chunk && chunk.session && chunk.session.configurationReadback && chunk.session.configurationReadback.effectiveConfig;
  if (!eff || typeof eff !== 'object') return { available: false, reason: 'the chunk does not expose the engine getEffectiveConfig read-back' };
  if (eff.available === false) return { available: false, reason: String(eff.reason ?? 'unavailable') };  // engine-worker / engine without getEffectiveConfig
  const ignoredKeys = Array.isArray(eff.ignoredKeys) ? eff.ignoredKeys.map(String) : null;
  const k1 = eff.accuracy && typeof eff.accuracy.k1KeepMap === 'boolean' ? eff.accuracy.k1KeepMap : null;
  const flags = {};
  for (const [k, sent] of Object.entries(sentAccuracy || {})) {
    flags[k] = { sent, effective: eff.accuracy && k in eff.accuracy ? eff.accuracy[k] : null, ignored: ignoredKeys === null ? null : ignoredKeys.includes(`accuracy.${k}`) };
  }
  const keys = Object.keys(flags);
  const allProductFlagsActive = keys.length > 0 && ignoredKeys !== null && keys.every((k) => flags[k].effective === flags[k].sent && flags[k].ignored === false);
  return { available: true, k1KeepMap: k1, ignoredKeys, k1KeepMapActive: k1 === true && ignoredKeys !== null && !ignoredKeys.includes('accuracy.k1KeepMap'), flags, allProductFlagsActive };
}

// engineModeEffective: the requested mode name only when the read-back shows every sent flag effective and not ignored; 'default' when nothing was
// sent; otherwise 'requested-not-effective' (also while the read-back is unavailable: never claim a flag the engine did not confirm).
export const effectiveMode = (opt, readback) => (!opt.accuracy ? 'default' : readback && readback.allProductFlagsActive === true ? opt.mode : 'requested-not-effective');

// Counts and the p50/p95/p99 of the per-frame timing arrays; the arrays themselves (up to tens of thousands of entries) are not copied.
const TIMING_KEYS = ['stageMs', 'readMs', 'engineMs'];
export function summarizeChunkStats(chunk) {
  if (!chunk || typeof chunk !== 'object') return null;
  const out = {};
  for (const [k, v] of Object.entries(chunk.stats || {})) {
    if (Array.isArray(v)) out[k] = TIMING_KEYS.includes(k) ? summarize(v.filter((x) => typeof x === 'number')) : { n: v.length };
    else out[k] = jsonCopy(v);
  }
  return { stats: out, limits: jsonCopy(chunk.limits), error: jsonCopy(chunk.error), warningsCount: Array.isArray(chunk.warnings) ? chunk.warnings.length : null };
}

// Identification of what produced a pose log: camera stream, device, build. Every part is read separately and reports why it is missing
// (unavailable: reason) instead of failing the whole receipt. The raw deviceId/groupId never appear: only their SHA-256.
export async function buildReceipt(env) {
  const { track = null, win = {}, nav = {}, scr = {}, canvas = null, fetchFn = null, subtle = null, nowMs = Date.now() } = env;
  const hash = async (v) => (v == null ? null : subtle ? sha256Hex(subtle, v) : { unavailable: 'crypto.subtle is not available (insecure context)' });
  const fetchJson = async (url) => {
    if (!fetchFn) return { unavailable: 'fetch is not available' };
    return guard(async () => {
      const r = await fetchFn(url, { cache: 'no-store' });
      if (!r.ok) return { unavailable: `HTTP ${r.status}` };
      return r.json();
    });
  };
  const pick = (o, keys) => (o && o.unavailable === undefined ? Object.fromEntries(keys.map((k) => [k, o[k] ?? null])) : o);
  const camera = track ? await guard(async () => {
    const { deviceId, groupId, ...settings } = jsonCopy(track.getSettings ? track.getSettings() : {});
    return { settings, label: track.label ?? null, deviceIdSha256: await hash(deviceId), groupIdSha256: await hash(groupId), readyState: track.readyState ?? null };
  }) : { unavailable: 'no camera track seen (no hasStream/hasVideo camera status yet)' };
  const deploy = await fetchJson('../../deploy.json');
  const manifest = await fetchJson('../../engine/slam-refact-engine/manifest.json');
  const chunk = win.__slamRefactChunk;
  const orient = scr.orientation;
  return {
    capturedAtMs: nowMs, tPerfMs: typeof win.performance?.now === 'function' ? win.performance.now() : null,
    camera, cameraResolution: await guard(() => jsonCopy(win.XrCameraResolution)),
    deploy: pick(deploy, ['schema', 'engine', 'xr_js', 'bundle_sha256']),
    engineManifest: manifest && manifest.unavailable === undefined ? { schema: manifest.schema ?? null, artifacts: jsonCopy(manifest.artifacts) } : manifest,
    session: await guard(() => jsonCopy(chunk && chunk.session)),
    chunkStats: await guard(() => summarizeChunkStats(chunk)),
    delivery: await guard(() => jsonCopy(chunk && chunk.delivery)),  // full bounded delivery ledger (null when off); read twice per session only
    userAgent: nav.userAgent ?? null,
    userAgentData: nav.userAgentData && nav.userAgentData.getHighEntropyValues
      ? await guard(async () => jsonCopy(await nav.userAgentData.getHighEntropyValues(['model', 'platformVersion']))) : { unavailable: 'navigator.userAgentData is not available' },
    screen: { orientation: orient ? { type: orient.type, angle: orient.angle } : null, width: scr.width ?? null, height: scr.height ?? null, innerWidth: win.innerWidth ?? null, innerHeight: win.innerHeight ?? null },
    devicePixelRatio: win.devicePixelRatio ?? null,
    canvas: canvas ? { width: canvas.width, height: canvas.height } : null,
  };
}

export function createMetrics(now = () => performance.now(), options = {}) {
  const trackerDiag = !!(options.record && options.trackerDiag);
  const recorder = options.record ? createPoseRecorder(now, options.poseLogCap, trackerDiag) : null;
  let receiptAtStart = null;
  const m = {
    startedAt: null, firstNormalAt: null, lastUpdateAt: null,
    updates: 0, nullUpdates: 0, resets: 0, hitTests: 0, hitMisses: 0, placed: 0,
    intervalsPre: [], intervalsPost: [], worldPoints: [], transitions: [], events: [], imageEvents: { found: 0, updated: 0, lost: 0 }, errors: 0,
  };
  const push = (arr, v) => { if (arr.length < CAP) arr.push(v); };
  return {
    start() { m.startedAt = now(); },
    update(reality, nPoints, frameStart, sourceFrameIndex = null, sourceTiming = null, tracker = null) {
      const t = now();
      if (!reality) { m.nullUpdates++; return; }
      m.updates++;
      if (recorder) recorder.add(reality, frameStart, sourceFrameIndex, sourceTiming, tracker);
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
    recording: !!recorder,
    poseLogCounts: () => (recorder ? recorder.counts() : null),
    setReceiptAtStart(r) { receiptAtStart = r; },
    snapshot(extra = {}, receiptAtDownload = null) {
      const snap = {
        schema: 'slam-refact-phone-metrics-v1',
        note: recorder ? NOTE + NOTE_RECORD + (trackerDiag ? NOTE_TRACKER : '') : NOTE,
        label: 'page-observed (rAF/onUpdate cadence), device and input kind are recorded by the user, not by this page',
        ...extra,
        updates: m.updates, nullUpdates: m.nullUpdates, hitTests: m.hitTests, hitMisses: m.hitMisses, placed: m.placed,
        imageEvents: m.imageEvents, errors: m.errors,
        timeToFirstNormalMs: m.firstNormalAt != null && m.startedAt != null ? Math.round(m.firstNormalAt - m.startedAt) : null,
        updateIntervalMs: { beforeFirstNormal: summarize(m.intervalsPre), afterFirstNormal: summarize(m.intervalsPost) },
        worldPoints: summarize(m.worldPoints),
        transitions: m.transitions, events: m.events,
      };
      if (recorder) { snap.poseLog = recorder.snapshot(m.nullUpdates); snap.receipt = { atStart: receiptAtStart, atDownload: receiptAtDownload }; }
      return snap;
    },
  };
}
