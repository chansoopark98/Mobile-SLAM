// Main-thread side of the opt-in engine Worker (R1). xr-slam.js imports this module dynamically and only when the mode is requested
// (`?engineWorker=1` or SLAM_REFACT_CHUNK_OPTIONS.engineWorker), so the default chunk neither loads nor depends on it. The `capture=worker`
// Worker of the measurement page (camera capture / recorder) is a different thing and is not touched.
//
// What runs where: the WASM engine and every SlamRefactGlue live in the Worker (engine-worker.js). Here `Module.SlamRefactGlue` is a
// facade with the surface xr-slam.js uses on the engine glue:
//   ordered fire-and-forget  configure, updateCameraProjectionMatrix, recenter, pushDeviceMotionEvent, pushDeviceOrientation, delete
//   answered from the last result   getWorldPoints, getFramesUntilRecenter, getRequestedRois (always empty: image targets are refused)
//   answered from a prefetched layout   pyramidLayout (see requestLayout)
//   xrHitTest: answers [] (a hit test needs the engine state synchronously; counted, one warning)
//   setImageTargets: throws (the ROI request/draw loop needs the engine's answer within the same frame)
// Frames travel with submitFrame(): the pyramid atlas is a Transferable (zero copy; buffers come back with the result and are reused),
// the IMU events staged with the frame and the orientation ride in the same message, so the engine sees the main-thread call order.
// Results are queued here and drained by the chunk in onProcessCpu (takeResults); a result is therefore at least one onProcessCpu later
// than the call that submitted it (the pose answer lags the main-thread mode by the frames in flight).
//
// Limits recorded in diag.engineWorker by xr-slam.js: no frameScheduler (B1prime), no createGlue test hook, no gpuCheck/roiCheck,
// no image targets, hitTest answers [], a Worker that dies after the load is reported (errorCallback) and not recovered.

export const ENGINE_WORKER_PARAM = 'engineWorker';
export const DEFAULT_MAX_IN_FLIGHT = 4;

// Default OFF. ON for options.engineWorker === true / {enabled: true}, or ?engineWorker=1 in the page URL.
export function engineWorkerRequested(options, search = globalThis.location?.search ?? '') {
  const o = options?.engineWorker;
  if (o === true || o?.enabled === true) return true;
  if (o === false || o?.enabled === false) return false;
  try { return new URLSearchParams(search).get(ENGINE_WORKER_PARAM) === '1'; } catch (_) { return false; }
}

// Options the Worker mode cannot honour: the chunk keeps the main-thread engine and says why (diag.engineWorker.fallback).
export function engineWorkerUnsupportedReason(options) {
  if (options?.frameScheduler?.enabled === true) return 'frameScheduler (B1prime) is not supported with engineWorker';
  if (options?.createGlue) return 'the createGlue test hook cannot run in a Worker';
  if (options?.gpuCheck || options?.roiCheck) return 'gpuCheck / roiCheck need the engine on the main thread';
  return null;
}

const workerError = (message, code, cause) => Object.assign(new Error(message), { code, ...(cause ? { cause } : {}) });
const rebuildError = (d, loaded) => Object.assign(new Error(d.message), { ...(d.code ? { code: d.code } : {}), ...(loaded ? { loaded } : {}) });
const layoutKey = (w, h, rotation) => `${w}x${h}@${rotation}`;
const copyAxes = (o, keys) => (o && typeof o === 'object' ? Object.fromEntries(keys.map((k) => [k, o[k]])) : o);
// DeviceMotionEvent sub-objects are not structured-cloneable (WebIDL interfaces): the three sensor objects are copied field by field.
const plainMotion = (e) => ({ acceleration: copyAxes(e.acceleration, ['x', 'y', 'z']),
  accelerationIncludingGravity: copyAxes(e.accelerationIncludingGravity, ['x', 'y', 'z']),
  rotationRate: copyAxes(e.rotationRate, ['alpha', 'beta', 'gamma']), interval: e.interval });

// worker: object with postMessage(message, transfer), onmessage, onerror (a real Worker, or a test transport).
export function createEngineWorkerClient(worker, { moduleArgs = {}, maxInFlight = DEFAULT_MAX_IN_FLIGHT, warn = () => {}, onFatal = () => {} } = {}) {
  const stats = { framesSubmitted: 0, resultsReceived: 0, backpressureSkips: 0, opErrors: 0, hitTestMisses: 0, imageTargetRefusals: 0,
    roiRequests: 0, layoutProbes: 0, layoutMismatches: 0, buffersAllocated: 0, buffersReused: 0, buffersDropped: 0, rttMs: [], workerEngineMs: [], submitMs: [], transitMs: [] };
  const layouts = new Map(), probes = new Map(), glues = new Map();
  const pool = [];
  let nextId = 0, nextGlueId = 0, fatal = null, compileFeatures = null;
  let onLoaded = null, loadComplete = false;

  // A failure before the load completes only rejects the load (the chunk then falls back to the main-thread engine, which is not an
  // error of the page). After the load, a failure is reported exactly once, through onFatal; the engine abort message is the same event
  // and is not also sent to moduleArgs.onAbort.
  const fail = (error) => {
    if (fatal) return;
    fatal = error;
    for (const p of probes.values()) p.reject(error);
    probes.clear();
    onLoaded?.reject(error);
    if (loadComplete) onFatal(error);
  };
  worker.onerror = (event) => fail(workerError(`[XR] slam_refact engine Worker failed: ${event && (event.message || event.error?.message) || 'script error'}`, 'engine-worker-error'));
  worker.onmessageerror = () => fail(workerError('[XR] slam_refact engine Worker sent a message that could not be deserialized', 'engine-worker-error'));

  worker.onmessage = (event) => {
    const m = event.data;
    switch (m.type) {
      case 'log': (m.err ? moduleArgs.printErr : moduleArgs.print)?.(m.text); break;
      case 'abort': fail(workerError(`[XR] slam_refact engine aborted in the Worker: ${m.message}`, 'engine-worker-abort')); break;
      case 'loaded': loadComplete = true; onLoaded?.resolve(m); break;
      case 'load-error': onLoaded?.reject(rebuildError(m.error, m.loaded)); break;
      case 'probed': { const p = probes.get(m.id); probes.delete(m.id); if (!p) break; stats.layoutProbes++; layouts.set(p.key, m.layout); p.resolve(m.layout); break; }
      case 'probe-error': { const p = probes.get(m.id); probes.delete(m.id); p?.reject(rebuildError(m.error)); break; }
      case 'created': {
        const g = glues.get(m.glueId);
        if (!g) break;
        g.captureIntrinsics = m.captureIntrinsics;
        if (JSON.stringify(m.layout) !== JSON.stringify(layouts.get(g.key))) {
          stats.layoutMismatches++;
          g.fail(workerError('[XR] slam_refact engine Worker: the layout of the created glue differs from the probed layout the session was built with', 'engine-worker-layout-mismatch'));
        }
        break;
      }
      case 'create-error': glues.get(m.glueId)?.fail(rebuildError(m.error)); break;
      case 'op-error': { stats.opErrors++; const g = glues.get(m.glueId); g?.errors.push(`${m.name}: ${m.error.message}`); warn(`[XR] slam_refact engine Worker: ${m.name} failed: ${m.error.message}`); break; }
      case 'frame-result': {
        const g = glues.get(m.glueId);
        if (m.atlas && pool.length < maxInFlight + 2) pool.push(new Uint8Array(m.atlas));
        if (!g) break;
        g.inflight--;
        stats.resultsReceived++;
        const sub = g.submitted.get(m.seq);
        g.submitted.delete(m.seq);
        if (sub) {
          const now = performance.now();
          stats.rttMs.push(now - sub.at); stats.workerEngineMs.push(m.engineMs);
          stats.transitMs.push(performance.timeOrigin + now - m.workerSentEpoch);
        }
        if (m.error) g.results.push({ seq: m.seq, error: rebuildError(m.error) });
        else {
          g.world = m.world; g.framesUntilRecenter = m.framesUntilRecenter;
          if (m.roiCount) { stats.roiRequests++; warn('[XR] slam_refact engine Worker: the engine requested image-target ROIs, which the Worker mode does not draw (image targets are unsupported with engineWorker)'); }
          g.results.push({ seq: m.seq, pose: m.pose, world: m.world, engineMs: m.engineMs });
        }
        break;
      }
      default: break;
    }
  };

  const post = (message, transfer = []) => { if (fatal) throw fatal; worker.postMessage(message, transfer); };

  class WorkerGlue {
    constructor(config) {
      this.id = ++nextGlueId;
      this.key = layoutKey(config.captureWidth, config.captureHeight, config.rotationDeg ?? 0);
      this.world = new Float32Array(0);
      this.framesUntilRecenter = config.framesUntilRecenter ?? 5;
      this.inflight = 0; this.results = []; this.submitted = new Map(); this.errors = []; this.captureIntrinsics = null;
      this.failure = null; this.seq = 0; this.deleted = false;
      glues.set(this.id, this);
      post({ type: 'create', glueId: this.id, config: { ...config } });
    }
    fail(error) { this.failure = error; }
    // A failure of this glue or of the whole Worker; the chunk throws it from the frame path (reported once, answer null).
    get fatal() { return this.failure ?? fatal; }
    pyramidLayout() {
      const layout = layouts.get(this.key);
      if (!layout) throw workerError(`[XR] slam_refact engine Worker: no pyramid layout for ${this.key} (requestLayout must resolve before the session is built)`, 'engine-worker-layout-missing');
      return layout;
    }
    getRequestedRois() { return []; }
    getWorldPoints() { return this.world; }
    getFramesUntilRecenter() { return this.framesUntilRecenter; }
    xrHitTest() {
      stats.hitTestMisses++;
      warn('[XR] slam_refact chunk: hitTest answers no hit with engineWorker (the engine runs in a Worker; a synchronous query is not possible)');
      return [];
    }
    setImageTargets() {
      stats.imageTargetRefusals++;
      throw new Error('[XR] image targets are not supported with engineWorker (the ROI request and draw loop needs the engine answer within the frame)');
    }
    unloadImageTargets() {}
    clearImageTargets() {}
    configure(...args) { this._op('configure', args); }
    updateCameraProjectionMatrix(...args) { this._op('updateCameraProjectionMatrix', args); }
    recenter() { this._op('recenter', []); }
    pushDeviceMotionEvent(timestampNanos, event) { this._op('pushDeviceMotionEvent', [timestampNanos, plainMotion(event)]); }
    pushDeviceOrientation(...args) { this._op('pushDeviceOrientation', args); }
    _op(name, args) { if (this.deleted) return; post({ type: 'op', glueId: this.id, name, args }); }
    delete() {
      if (this.deleted) return;
      this.deleted = true;
      if (!fatal) post({ type: 'delete', glueId: this.id });
      glues.delete(this.id);
    }
    // A buffer for the next pyramid readback: one the Worker handed back, else a new one.
    acquireAtlas(byteLength) {
      // Buffers smaller than the request (left over from a smaller capture) can never serve it or any later request of this size: drop them.
      for (let i = pool.length - 1; i >= 0; i--) if (pool[i].byteLength < byteLength) { pool.splice(i, 1); stats.buffersDropped++; }
      if (pool.length) { stats.buffersReused++; return pool.shift(); }
      stats.buffersAllocated++;
      return new Uint8Array(byteLength);
    }
    // frame: {motion: [[timestampNanos, event]], orientation: [t, alpha, beta, gamma], timeNanos, videoNs, atlas (the Uint8Array
    // view from the readback), size, rows}. The atlas buffer is transferred: the caller must not touch it afterwards.
    submitFrame(frame) {
      const t0 = performance.now();
      const seq = ++this.seq;
      const buffer = frame.atlas.buffer;
      post({ type: 'frame', glueId: this.id, seq, motion: frame.motion, orientation: frame.orientation, timeNanos: frame.timeNanos,
        videoNs: frame.videoNs, atlas: buffer, size: frame.size, rows: frame.rows }, [buffer]);
      this.inflight++;
      this.submitted.set(seq, { at: t0 });
      stats.framesSubmitted++;
      stats.submitMs.push(performance.now() - t0);
      return seq;
    }
    takeResults() { const r = this.results; this.results = []; return r; }
  }

  const module = { SlamRefactGlue: Object.assign(WorkerGlue, {
    getCompileFeatures: () => compileFeatures,
  }) };

  return {
    Module: module,
    stats,
    maxInFlight,
    plainMotion,
    get fatal() { return fatal; },
    hasLayout: (w, h, rotation) => layouts.has(layoutKey(w, h, rotation)),
    // Prefetches the pyramid layout of a capture (the chunk needs it synchronously when it builds a session).
    requestLayout(w, h, rotation) {
      const key = layoutKey(w, h, rotation);
      if (layouts.has(key)) return Promise.resolve(layouts.get(key));
      if (fatal) return Promise.reject(fatal);
      const id = ++nextId;
      return new Promise((resolve, reject) => {
        probes.set(id, { key, resolve, reject });
        post({ type: 'probe', id, config: { captureWidth: w, captureHeight: h, rotationDeg: rotation, manufacturer: '', model: '' } });
      });
    },
    load(baseUrl) {
      return new Promise((resolve, reject) => {
        onLoaded = { resolve, reject };
        post({ type: 'load', baseUrl: String(baseUrl) });
      }).then((m) => { compileFeatures = m.compileFeatures; onLoaded = null; return m.loaded; });
    },
    terminate() { try { worker.terminate?.(); } catch (_) { /* already gone */ } },
  };
}

// Starts the Worker, loads the engine in it and returns {Module, loaded, worker}: the shape of engine-loader's loadEngine plus the client.
// Throws (code 'engine-worker-unavailable' when no Worker can be started; the classified engine error, hash mismatch included, when the
// engine fails in the Worker) - xr-slam.js then loads the engine on the main thread and records why.
export async function loadEngineInWorker(baseUrl, moduleArgs = {}, { createWorker, workerUrl, maxInFlight, warn, onFatal } = {}) {
  let worker;
  try {
    worker = createWorker ? createWorker(workerUrl)
      : new Worker(workerUrl ?? new URL('./engine-worker.js', import.meta.url), { type: 'module', name: 'slam-refact-engine' });
  } catch (error) {
    throw workerError(`[XR] slam_refact engine Worker could not be started: ${error && error.message || error}`, 'engine-worker-unavailable', error);
  }
  const client = createEngineWorkerClient(worker, { moduleArgs, maxInFlight, warn, onFatal });
  const t0 = performance.now();
  try {
    const loaded = await client.load(baseUrl);
    loaded.workerLoadMs = performance.now() - t0;
    return { Module: client.Module, loaded, worker: client };
  } catch (error) {
    client.terminate();
    throw error;
  }
}
