// slam_refact drop-in for the reference runtime `xr-slam.js` chunk (full-parity PRD US-007..US-009, contract FULL-2,
// docs/spec/07-xr-slam-chunk.md).
//
// The deployed xr.js loads this ES module with `import(base + "xr-slam.js")`, calls
// `ResourceUrls.setBaseUrl(base)` and `await XrControllerFactory(runConfig, xrccWithContextPromise,
// logCallback, errorCallback)`, and installs the result as XR8.XrController. The JS logic below follows the
// deployed chunk (SDK = assets/references/xr_reference/analysis-runtime/standalone/xr-slam.js, sha256 d5b059d8…,
// factory `AB` at offset ~5,494,000) and the public tracking-controller.ts
// (ARC:reality/app/xr/js/src/tracking-controller.ts): state variables, configure / updateCameraProjectionMatrix /
// recenter / hitTest, sensor queue, events and callback order. The original's C++ side (_c8EmAsm_* with capnp
// messages) is replaced by the embind SlamRefactGlue of slam_refact (GlueRunner = tracking-controller.cc) and the
// GL pyramid by ./gr8-compute-pyramid.js on XR8's compute context. No capnp, no runtime dependency.
//
// Frame timing (original c8EmAsm_stageFrame / c8EmAsm_processStagedFrame):
//   onProcessGpu(N): sensor queue + orientation taken (setEventQueue), pyramid of N drawn, readback started.
//   onProcessCpu(N): pyramid of N read (fence + pixel buffer), IMU of N pushed, glue.processFramePyramid(N):
//                    GlueRunner stages N and tracks N-1 (its 1-frame pipeline = processPipeline's
//                    pipeline.size() >= 2) -> the response is frame N-1 with N-1's camera texture, as in the original.
//
// Video size / orientation changes (RM-4a, original c8EmAsm_engineOrientationChange / engineVideoSizeChange =
// engineInit again + resetOriginToLastPose): the session (glue + pyramid) is destroyed and rebuilt for the new capture
// (portrait or landscape, rotation 0 / 90 / 180 / 270 and their negatives, FULL-8 / US-023); a capture the engine cannot take
// (a rotation that is not one of those, e.g. NaN or 45) leaves world tracking inactive:
// one console.warn per episode, no errorCallback (xr.js hands errorCallback to the onException modules and the
// XRExtras RuntimeError module stops the session), and onProcessCpu keeps answering with the last pose, tracking
// status LIMITED and the live camera texture (RM-4a2, spec 07 section 3). No pipeline callback throws for this: xr.js
// runs onProcessCpu inside the pre-render try block, and one throw there skips onUpdate and the camera texture
// upload of every later frame.
import { ENGINE_DIR, loadEngine } from './engine-loader.js';
import { Gr8ComputePyramid, OwnedDisplaySnapshots } from './gr8-compute-pyramid.js';
import { dispatchImageEvents } from './image-events.js';
import { createImageTargets } from './image-targets.js';
import { createSensors, pushMotion } from './sensors.js';

// ---- ResourceUrls (SDK offset ~5,489,300 `fQ`) ----
let baseUrl_ = null;
const getBaseUrl = () => {
  if (baseUrl_ === null) throw new Error('[XR] Base URL has not been set yet.');
  return baseUrl_;
};
const resolveLocal = (p) => () => getBaseUrl() + p;
const resolveUnsupported = (p) => () => { throw new Error(`[XR] Resource "${p}" is not supported in this environment.`); };
const ResourceUrls = {
  resolveFaceModel: resolveLocal('resources/face-model.tflite'),
  resolveFaceMeshModel: resolveLocal('resources/face-mesh-model.tflite'),
  resolveFaceEarModel: resolveLocal('resources/face-ear-model.tflite'),
  resolveSemanticsWorker: resolveLocal('resources/semantics-worker.js'),
  resolveSemanticsModel: resolveLocal('resources/semantics-model.tflite'),
  resolveMediaWorker: resolveLocal('resources/media-worker.js'),
  resolvePoweredLogo: resolveLocal('resources/powered-by.svg'),
  resolveDracoWorker: resolveUnsupported('draco-worker-l5sniji5.js'),
  resolveDracoWrapper: resolveUnsupported('draco_wasm_wrapper-l325u8do.js'),
  resolveDomTabletFrameGlb: resolveLocal('resources/dom-tablet-frame.glb'),
  resolveDomTabletButtonGlb: resolveLocal('resources/dom-tablet-button.glb'),
  setBaseUrl: (url) => {
    if (!url) throw new Error('[XR] Base URL cannot be empty.');
    baseUrl_ = url;
  },
  getBaseUrl,
};

// factory.ts singleton (SDK `Yg`).
const singleton = (fn) => {
  let result = null;
  return (...args) => (result || (result = fn(...args)), result);
};

// The deprecated `points` field of the onProcessCpu answer (one shared object: it never has content).
const EMPTY_POINTS = { getLength: () => 0, get: (i) => { throw new Error(`get(${i}) exceeds length 0`); } };

// tracking-controller.ts:138-160.
const initFakeRealityTexture = (GLctx) => {
  const blackPixel = new Uint8Array([45, 46, 67, 255]);
  const restoreTex = GLctx.getParameter(GLctx.TEXTURE_BINDING_2D);
  const texture = GLctx.createTexture();
  GLctx.bindTexture(GLctx.TEXTURE_2D, texture);
  GLctx.texImage2D(GLctx.TEXTURE_2D, 0, GLctx.RGBA, 1, 1, 0, GLctx.RGBA, GLctx.UNSIGNED_BYTE, blackPixel);
  GLctx.bindTexture(GLctx.TEXTURE_2D, restoreTex);
  return texture;
};

// configure() keys of the deployed chunk whose engine side slam_refact does not have (data recorder, maps / area
// targets). They are reported, not emulated (spec 07 §3). imageTargetData is supported (image-targets.js).
const UNSUPPORTED_KEYS = ['mapSrcUrl', 'enableJpgEncoding', 'enableDataRecorder', 'enableAreaTargets'];

// Pixel test of GlRealityFrame pyramid level 3, channel 0 (pixel-transforms.cc:2929 isAllZeroes).
function levelIsAllZeroes(atlas, size, level) {
  for (let y = level.r; y < level.r + level.h; y++) {
    let nonZero = 0;
    for (let x = level.c, k = (y * size + x) * 4; x < level.c + level.w; x++, k += 4) nonZero |= atlas[k];
    if (nonZero) return false;
  }
  return true;
}

function compareAtlas(gpu, cpu, size, rows) {
  const n = size * rows * 4;
  let equal = 0, maxAbs = 0;
  for (let i = 0; i < n; i++) {
    const d = Math.abs(gpu[i] - cpu[i]);
    if (d === 0) equal++; else if (d > maxAbs) maxAbs = d;
  }
  return { identical: equal === n, equalByteRatio: equal / n, maxAbsDiff: maxAbs };
}

// roiCheck (diagnostics, test hook): the R plane of every slot of the ROIs that were drawn into this atlas, with the ROI list.
function captureRoiSlots(s, atlas, rois, stageIndex) {
  const size = s.layout.textureSize;
  return { stageIndex, rois: rois.map((r) => ({ slot: r.slot, source: r.source, name: r.name, warp: Array.from(r.warp) })),
    slots: rois.map((r) => {
      const l = s.layout.roiSlots[r.slot];
      const plane = new Uint8Array(l.w * l.h);
      for (let y = 0; y < l.h; y++) for (let x = 0; x < l.w; x++) plane[y * l.w + x] = atlas[((l.r + y) * size + l.c + x) * 4];
      return { slot: r.slot, w: l.w, h: l.h, r: Array.from(plane) };
    }) };
}

// P1-I1 is constructor-only and opt-in. Identity is not a calibration profile or an accuracy switch.
function deviceIdentityOptions(value) {
  if (!value || value.enabled !== true) return null;
  const timeoutMs = value.timeoutMs ?? 1000;
  if (!Number.isFinite(timeoutMs) || timeoutMs < 1 || timeoutMs > 5000) {
    throw new Error('[XR] deviceIdentity.timeoutMs must be between 1 and 5000');
  }
  let fallback = null;
  if (value.fallback !== undefined) {
    const input = value.fallback;
    if (!input || typeof input.model !== 'string' || !input.model.trim()) {
      throw new Error('[XR] deviceIdentity.fallback.model must be a non-empty string');
    }
    const text = (name, def = '') => {
      const v = input[name] ?? def;
      if (typeof v !== 'string' || v.length > 128 || /[\x00-\x1f\x7f]/.test(v)) {
        throw new Error(`[XR] deviceIdentity.fallback.${name} must be a string of at most 128 printable characters`);
      }
      return v.trim();
    };
    fallback = { model: text('model'), manufacturer: text('manufacturer'), profileId: text('profileId'), source: 'explicit-fallback' };
  }
  return { timeoutMs, fallback };
}

async function resolveDeviceIdentity({ timeoutMs, fallback }) {
  let timer;
  let highEntropyStatus = 'unavailable';
  const expired = Symbol('timeout');
  try {
    const uaData = globalThis.navigator?.userAgentData;
    if (typeof uaData?.getHighEntropyValues === 'function') {
      const result = await Promise.race([
        Promise.resolve().then(() => uaData.getHighEntropyValues(['model'])),
        new Promise((resolve) => { timer = setTimeout(() => resolve(expired), timeoutMs); }),
      ]);
      if (result === expired) highEntropyStatus = 'timed-out';
      else if (typeof result?.model === 'string' && result.model.trim()
          && result.model.length <= 128 && !/[\x00-\x1f\x7f]/.test(result.model)) {
        // UA-CH supplies model, not manufacturer. Let the engine's existing model resolver use it.
        return { selected: { manufacturer: '', model: result.model.trim(), source: 'userAgentData.model' }, highEntropyStatus: 'available' };
      } else highEntropyStatus = 'empty-or-invalid-model';
    }
  } catch (_) {
    highEntropyStatus = 'rejected';
  } finally { clearTimeout(timer); }
  return { selected: fallback, highEntropyStatus };
}

const XrControllerFactory = singleton((runConfig, xrccPromise, logCallback, errorCallback) => {
  const options = (typeof window !== 'undefined' && window.SLAM_REFACT_CHUNK_OPTIONS) || {};
  // FX-E3 (opt-in): validated here, before the engine load starts, so a malformed value leaves nothing running.
  const trustedMessageOrigins = options.trustedMessageOrigins;
  if (trustedMessageOrigins !== undefined && (!Array.isArray(trustedMessageOrigins) || !trustedMessageOrigins.every((o) => typeof o === 'string'))) {
    throw new Error('[XR] SLAM_REFACT_CHUNK_OPTIONS.trustedMessageOrigins must be an array of origin strings');
  }
  const schedulerRequested = options.frameScheduler?.enabled === true && options.frameScheduler.mode === 'gpu-task-owned-v1';
  let schedulerEnabled = schedulerRequested;
  let schedulerFallbackReason = null;
  const identityOptions = deviceIdentityOptions(options.deviceIdentity);  // value copy before any await
  let resolvedIdentity = null;
  const deliveryOptions = options.deliveryDiagnostics?.enabled === true || schedulerEnabled
    ? { enabled: true, maxEntries: Number.isInteger(options.deliveryDiagnostics?.maxEntries)
      && options.deliveryDiagnostics.maxEntries >= 1 && options.deliveryDiagnostics.maxEntries <= 1024
      ? options.deliveryDiagnostics.maxEntries : 256 } : null;
  const responseEntries = [];
  let responseNext = 0, responseTotal = 0, publication = null, nextSessionEpoch = 0, lastPublishedSource = null;
  const diag = window.__slamRefactChunk = {
    chunk: 'slam_refact', engine: null, error: null, session: null, gpuCheck: [], roiCheck: [], warnings: [], sessionLog: [],
    stats: { stagedFrames: 0, processedFrames: 0, freshResponses: 0, droppedStaged: 0, motionEventsPushed: 0,
      motionEventsSkipped: 0, responseLag: {}, lastSourceFrameIndex: null, stageMs: [], readMs: [], engineMs: [],
      sessionsCreated: 0, sessionsDestroyed: 0, sessionsRejected: 0, originResets: 0, sizeMismatchFrames: 0, engineImageEventsIgnored: 0, roiPaddedFrames: 0,
      stageErrors: 0, processErrors: 0, samplesDropped: 0, warningsDropped: 0, inactiveAnswers: 0, gapAnswers: 0, contextLostFrames: 0 },
    limits: null,
  };
  // Diagnostic arrays are bounded: the per-frame timing arrays keep the newest diagMaxSamples (default 20000,
  // about 11 minutes at 30 fps; trimmed in batches once they reach twice that), everything else 100 entries.
  const MAX_SAMPLES = Math.max(1, options.diagMaxSamples | 0 || 20000);
  const MAX_ENTRIES = 100;
  diag.limits = { maxSamples: MAX_SAMPLES, maxEntries: MAX_ENTRIES };
  const pushSample = (arr, value) => {
    arr.push(value);
    if (arr.length > 2 * MAX_SAMPLES) {
      const drop = arr.length - MAX_SAMPLES;
      arr.splice(0, drop);
      diag.stats.samplesDropped += drop;
    }
  };
  const pushEntry = (arr, value) => {
    if (arr.length < MAX_ENTRIES) arr.push(value); else diag.stats.warningsDropped++;
  };
  const warnOnce = (() => {
    const seen = new Set();
    return (msg) => { if (seen.has(msg)) return; seen.add(msg); pushEntry(diag.warnings, msg); console.warn(msg); };
  })();
  // errorCallback for failures that must not throw out of a pipeline callback (each distinct message once).
  const reportOnce = (() => {
    const seen = new Set();
    return (where, error) => {
      const message = `${where}: ${error && error.message || error}`;
      if (seen.has(message)) return;
      if (seen.size >= MAX_ENTRIES) { diag.stats.warningsDropped++; return; }
      seen.add(message);
      pushEntry(diag.warnings, message);
      errorCallback(error instanceof Error ? error : new Error(String(error)));
    };
  })();

  // A-0e (opt-in, default OFF, SLAM_REFACT_CHUNK_OPTIONS.trackerDiagnostics = {enabled: true}): the engine's own tracker state of every
  // tracked frame (pose.tracker of the response: map tracker state, map reset, keyframe created, keyframe / point counts) joined with what
  // the engine was given for that frame (DeviceMotion events pushed with it, their peak gyro / linear-acceleration magnitude - the two
  // quantities the engine's extreme-motion and keyframe rules compare, map_tracker.cc EventExceedsLimits / map_builder.cc AnyGyroAbove).
  // diag.trackerDiag is a latest-only scalar copy (like diag.sourceTiming); the page samples it once per callback. Counters (total*) make
  // a callback that missed a response visible. Nothing is passed to the engine and no engine call is added: the chunk only reads fields
  // the response already carries. Recorded only by the main-thread frame path (processStagedFrame): the engine Worker and the
  // frameScheduler path do not record. `session` numbers the chunk session (a rebuilt session restarts sourceFrameIndex at 0).
  const trackerDiagEnabled = options.trackerDiagnostics?.enabled === true;
  let trackerDiagLatest = null;
  const trackerDiagTotals = { totalResponses: 0, totalMapResets: 0, totalKeyframesCreated: 0, totalMapTrackerNotOk: 0, totalInputMissing: 0 };
  if (trackerDiagEnabled) {
    Object.defineProperty(diag, 'trackerDiag', { enumerable: true, get: () => (trackerDiagLatest ? { ...trackerDiagLatest, ...trackerDiagTotals } : null) });
    if (schedulerRequested) warnOnce('[XR] slam_refact chunk: trackerDiagnostics does not record on the frameScheduler path (only the main-thread frame path records)');
  }
  const magnitude3 = (v, a, b, c) => (v ? Math.hypot(Number(v[a]), Number(v[b]), Number(v[c])) : null);
  // The DeviceMotion events the engine took since its last staged frame, as its rules see them (deg/s and m/s^2 magnitudes; a NaN
  // component gives NaN -> not counted as a peak). One event accepted by the glue is added at a time; the sum is taken when the frame call
  // returns, so events that stayed in the engine queue because the frame call threw are counted with the frame that gets them.
  const addTrackerMotion = (s, e) => {
    const acc = s.trackerMotion ?? (s.trackerMotion = { motionEvents: 0, gyroPeakDegS: null, linAccPeakMs2: null });
    acc.motionEvents++;
    const g = magnitude3(e.rotationRate, 'alpha', 'beta', 'gamma'), a = magnitude3(e.acceleration, 'x', 'y', 'z');
    if (Number.isFinite(g) && (acc.gyroPeakDegS === null || g > acc.gyroPeakDegS)) acc.gyroPeakDegS = g;
    if (Number.isFinite(a) && (acc.linAccPeakMs2 === null || a > acc.linAccPeakMs2)) acc.linAccPeakMs2 = a;
  };
  // stageIndex = the glue's index of the frame staged in this call; the response (pose) is the tracker step of pose.sourceFrameIndex.
  const noteTrackerDiag = (s, stageIndex, motion, pose) => {
    const session = s.trackerSession ?? (s.trackerSession = diag.stats.sessionsCreated);
    if (trackerDiagLatest && trackerDiagLatest.session !== session) trackerDiagLatest = null;  // never show another session's frame as current
    const inputs = s.trackerInputs ?? (s.trackerInputs = new Map());
    inputs.set(stageIndex, motion ?? { motionEvents: 0, gyroPeakDegS: null, linAccPeakMs2: null });
    while (inputs.size > 64) inputs.delete(inputs.keys().next().value);
    const t = pose.tracker;
    if (pose.repeated || !t) return;  // no new tracker step in this response
    const src = t.pointsBySource || {}, inp = inputs.get(pose.sourceFrameIndex) ?? null;
    trackerDiagTotals.totalResponses++;
    if (t.mapReset) trackerDiagTotals.totalMapResets++;
    if (t.keyframeCreated) trackerDiagTotals.totalKeyframesCreated++;
    if (t.mapTrackerState !== 1) trackerDiagTotals.totalMapTrackerNotOk++;
    if (!inp) trackerDiagTotals.totalInputMissing++;
    trackerDiagLatest = { session, sourceFrameIndex: pose.sourceFrameIndex, mapTrackerState: t.mapTrackerState, frameTrackerOk: t.frameTrackerOk, ok: t.ok,
      lossState: t.lossState, lostFrames: t.lostFrames, frames: t.frames, mapReset: t.mapReset, keyframeCreated: t.keyframeCreated,
      numKeyframes: t.numKeyframes, groundMatches: t.groundMatches, keypoints: pose.keypoints ?? null,
      pointsTriangulated: src.triangulated ?? null, pointsGround: src.ground ?? null, pointsSky: src.sky ?? null, pointsDepth: src.depth ?? null, pointsCloud: src.cloud ?? null,
      inputFound: !!inp, motionEvents: inp ? inp.motionEvents : null, gyroPeakDegS: inp ? inp.gyroPeakDegS : null, linAccPeakMs2: inp ? inp.linAccPeakMs2 : null };
    for (const k of inputs.keys()) if (k < pose.sourceFrameIndex) inputs.delete(k);
  };

  // xrccPromise is not used: the original only needs XRCC to share its Emscripten GL texture table
  // (scc.GL.textures = xrcc.GL.textures); this chunk uses XR8's computeTexture objects directly.
  let engine_ = null;  // {Module, loaded}; the original's `scc_`
  let ew_ = null;      // R1: the engine Worker client when the engine runs in a Worker (opt-in), else null
  // Measurement option (default OFF, SLAM_REFACT_CHUNK_OPTIONS.frameDoneMarks === true), same code in the main-thread and the Worker mode:
  //   performance.mark('sr-frame-start:<n>')  at the FIRST main-thread work of the chunk for a camera frame: the entry of onProcessGpu, before
  //                                           anything is staged or submitted. n = the frame's ordinal: 0, 1, 2, ... over the non-repeat frames the
  //                                           chunk is called with, unless the page names the next one with diag.setFrameOrdinal(n) (the harness
  //                                           numbers the recorded frames 1, 2, 3, ... and the frames it feeds before the recording apart).
  //   performance.mark('sr-frame-done:<n>')   once ALL main-thread handling of the engine response to frame n has returned. The response to frame
  //                                           n is what processing frame n returns (the pose of the frame before it, fresh, or a repeat of the
  //                                           previous pose during the startup recenter): every processed frame has exactly one, including the
  //                                           last one, so no extra frame is needed to complete it. The response is applied inside onProcessCpu
  //                                           (the Worker mode: when the Worker's answer is drained, one callback later); XR8 then runs the other
  //                                           modules' onUpdate / onRender synchronously in the same loop callback, so the mark is queued as a
  //                                           microtask, which runs when that whole callback has returned. Nothing for the frame is scheduled
  //                                           after the mark.
  // Without the option: one boolean test per frame / response, no counter, no frame field, no performance.mark.
  const frameDoneMarks = options.frameDoneMarks === true && typeof globalThis.performance?.mark === 'function';
  const doneQueue = [];
  let doneScheduled = false, frameOrdinal_ = 0;
  if (frameDoneMarks) {
    diag.setFrameOrdinal = (n) => { frameOrdinal_ = n; };
    diag.resultSeqs = [];  // the ordinals whose results were handled since the page last drained this list (the harness attributes its update callbacks)
  }
  const markFrameStart = () => {
    const n = frameOrdinal_++;
    globalThis.performance.mark(`sr-frame-start:${n}`);
    return n;
  };
  const noteFrameDone = (f) => {
    if (!frameDoneMarks || f.seq === undefined) return;
    diag.resultSeqs.push(f.seq);
    doneQueue.push(f.seq);
    if (doneScheduled) return;
    doneScheduled = true;
    queueMicrotask(() => {
      doneScheduled = false;
      for (const m of doneQueue.splice(0)) globalThis.performance.mark(`sr-frame-done:${m}`);
    });
  };
  // R1 (opt-in, default OFF): the engine in a dedicated Worker. Nothing of it runs, and engine-worker-client.js is not imported, unless
  // requested by SLAM_REFACT_CHUNK_OPTIONS.engineWorker (true / {enabled: true}) or ?engineWorker=1. Distinct from ?capture=worker.
  const engineWorkerAsked = options.engineWorker === true || options.engineWorker?.enabled === true
    || (options.engineWorker !== false && options.engineWorker?.enabled !== false
      && (() => { try { return new URLSearchParams(globalThis.location?.search ?? '').get('engineWorker') === '1'; } catch (_) { return false; } })());
  if (trackerDiagEnabled && engineWorkerAsked) warnOnce('[XR] slam_refact chunk: trackerDiagnostics records only when the engine runs on the main thread; trackerDiag stays null while the engine Worker is active');
  if (engineWorkerAsked) diag.engineWorker = { requested: true, active: false, fallback: null, stats: null, compileFeatures: null,
    maxInFlight: Math.max(1, options.engineWorker?.maxInFlight | 0 || 4), pendingBuilds: 0, deferredBuilds: 0,
    engineMsMeaning: 'main-thread cost of submitting the frame (stats.workerEngineMs is the Worker-side engine time)',
    limits: ['no frameScheduler', 'no createGlue hook', 'no gpuCheck/roiCheck', 'image targets refused', 'hitTest answers no hit',
      'pose answers lag the main-thread mode by the frames in flight', 'framesUntilRecenter carried over a rebuild lags by the frames in flight'] };
  const loadMainEngine = () => loadEngine(new URL(ENGINE_DIR, import.meta.url),
    { print: logCallback(false), printErr: logCallback(true), onAbort: errorCallback });
  // Worker first; any failure (no Worker, script error, engine/WASM failure with its classified code) is recorded and the engine is
  // loaded on the main thread exactly as without the option. If that fails too, its error is the one the chunk reports.
  const loadWorkerEngine = async () => {
    const unsupported = options.frameScheduler?.enabled === true ? 'frameScheduler (B1prime) is not supported with engineWorker'
      : options.createGlue ? 'the createGlue test hook cannot run in a Worker'
        : options.gpuCheck || options.roiCheck ? 'gpuCheck / roiCheck need the engine on the main thread' : null;
    try {
      if (unsupported) throw Object.assign(new Error(`[XR] engineWorker not used: ${unsupported}`), { code: 'engine-worker-unsupported-option' });
      const client = await import('./engine-worker-client.js');
      return await client.loadEngineInWorker(new URL(ENGINE_DIR, import.meta.url),
        { print: logCallback(false), printErr: logCallback(true), onAbort: errorCallback },
        { maxInFlight: diag.engineWorker.maxInFlight, warn: (m) => warnOnce(m), onFatal: (error) => reportOnce('process', error),  // same key as the frame path's throw of it: one report
          createWorker: options.engineWorker?.createWorker, workerUrl: options.engineWorker?.workerUrl });
    } catch (error) {
      diag.engineWorker.fallback = { code: error && error.code || 'engine-worker-failed', message: String(error && error.message || error) };
      warnOnce(`[XR] slam_refact engineWorker unavailable (${diag.engineWorker.fallback.code}); the engine runs on the main thread: ${diag.engineWorker.fallback.message}`);
      return loadMainEngine();
    }
  };
  const enginePromise = (engineWorkerAsked ? loadWorkerEngine() : loadMainEngine())
    .then((r) => {
      engine_ = r;
      if (r.worker) {
        ew_ = r.worker;
        Object.assign(diag.engineWorker, { active: true, stats: ew_.stats, compileFeatures: r.Module.SlamRefactGlue.getCompileFeatures() });
      }
      // A test hook that replaces the engine glue must not leave the evidence "engine = manifest bytes" looking untouched.
      r.loaded.glueReplaced = !!options.createGlue;
      diag.engine = r.loaded;
      return r;
    },
      (error) => { diag.error = String(error && error.message || error); diag.engineLoad = error.loaded ?? null; throw error; });

  // No UA access, new promise or extra wait in the default parity arm. In the opt-in arm the
  // controller is not exposed until this bounded request settles; late UA answers cannot change it.
  const identityPromise = identityOptions ? resolveDeviceIdentity(identityOptions).then((result) => {
    resolvedIdentity = result;
    diag.deviceIdentity = { enabled: true, timeoutMs: identityOptions.timeoutMs,
      highEntropyStatus: result.highEntropyStatus, selected: result.selected ? { ...result.selected } : null };
  }) : null;

  const sensors = createSensors(() => !engine_ || !window.XR8 || window.XR8.isPaused(), deliveryOptions);
  // Per-frame recorder path: copy only the latest scalar receipt, not the full history rings.
  const latestSourceTiming = () => publication ? { ...publication } : null;
  Object.defineProperty(diag, 'sourceTiming', { enumerable: true, get: latestSourceTiming });
  if (deliveryOptions) Object.defineProperty(diag, 'delivery', { enumerable: true, get: () => ({
    schema: 'xr-passive-delivery-v1', mode: 'passive', enabled: true, physicalExposureVerified: false,
    sourceTimeMeaning: 'WASM frame-time argument, not physical exposure',
    publishTimeMeaning: 'not sampled here; recorder callback arrival is separate',
    sourceTiming: latestSourceTiming(),
    responses: { total: responseTotal, overwritten: Math.max(0, responseTotal - deliveryOptions.maxEntries),
      entries: (responseTotal < deliveryOptions.maxEntries ? responseEntries
        : responseEntries.slice(responseNext).concat(responseEntries.slice(0, responseNext))).map((e) => ({ ...e })) },
    sensors: sensors.delivery.snapshot(),
    stages: { stageMs: diag.stats.stageMs.slice(-deliveryOptions.maxEntries),
      readMs: diag.stats.readMs.slice(-deliveryOptions.maxEntries), engineMs: diag.stats.engineMs.slice(-deliveryOptions.maxEntries) },
    sourceMetadata: { entries: session_?.delivery?.sourceFrames.size ?? 0, evicted: session_?.delivery?.evicted ?? 0 },
    maxSourceFrames: 64, maxEntries: deliveryOptions.maxEntries,
  }) });

  // B1′ owns one staged packet and one CPU mailbox. Native work runs only in a real task.
  // The legacy path never constructs this state, its MessageChannel, or its GL allocations.
  let schedulerEpoch = 0, schedulerControlledBytes = 0, schedulerPeakBytes = 0;
  let schedulerSnapshotSlots = 0, schedulerSnapshotBytes = 0, schedulerSnapshotPeakBytes = 0;
  let schedulerRebuilding = false, activePublication = null;
  // D1's frozen frame schedule is 30 Hz; D2's longest synthetic task delay is 250 ms.
  // 32 GPU ticks allow 1067 ms at 30 Hz (533 ms at 60 Hz, 267 ms at 120 Hz).
  // Higher refresh rates are unverified. This recovery limit does not change D2's gate.
  const MAX_STALLED_GPU_TICKS = 32;
  let retireStallTicks = 0, retireWatchdogReported = false;
  let glErrorStreak = 0;  // consecutive frames dropped for a preexisting GL error
  const retiredSnapshotPools = [];
  const MAX_DISCARD_MOTION_SEQS = 128;
  const schedulerCounts = { admitted: 0, primingAcked: 0, processed: 0, publications: 0,
    droppedBusy: 0, droppedCapacity: 0, droppedTargetWarmup: 0, discardedAdmissions: 0,
    discardedReadyReceipts: 0, preAdmissionDiscards: 0, acceptedMotionSeqOverflow: 0,
    lifecycleResets: 0, waits: 0, barrierResets: 0, errors: 0, errorCallbackErrors: 0, watchdogResets: 0,
    droppedGlError: 0, hitTests: 0, hitMisses: 0,
    partialDispatchCount: 0, startupDiscardedAdmissions: 0, staleTasks: 0, queryRejected: 0 };
  const schedulerFailures = [];
  const schedulerDiscards = [];
  const reportSchedulerError = (where, error) => {
    try { reportOnce(where, error); }
    catch (callbackError) {
      // An application error handler cannot be allowed to skip native recovery
      // or escape from an XR8 GPU/CPU/MessageChannel callback.
      schedulerCounts.errorCallbackErrors++;
      pushEntry(schedulerFailures, { where: 'scheduler-error-callback', source: where,
        message: String(callbackError?.message || callbackError),
        epoch: session_?.scheduler ? { ...session_.scheduler.epoch } : null });
    }
  };
  const schedulerCurrent = (s) => session_ === s && !!s?.scheduler && !s.scheduler.disposed;
  const schedulerBusyReason = () => {
    const q = session_?.scheduler;
    return schedulerRebuilding ? 'rebuilding' : !q ? 'no-session' : q.nativeCallDepth ? 'native-call' : q.nativeAdvanced ? 'native-unpublished'
      : q.publishing ? 'publishing' : q.queryNeedsFreshState ? 'await-first-publication' : null;
  };
  const schedulerDispatch = (framework, name, detail) => {
    const tx = activePublication;
    if (tx && !schedulerCurrent(tx.session)) throw new Error('[XR] B1prime stale publication');
    if (tx) tx.dispatches++;
    framework.dispatchEvent(name, detail);
    if (tx && !schedulerCurrent(tx.session)) throw new Error('[XR] B1prime publication epoch changed in callback');
  };
  const makeScheduler = (s) => {
    const q = { epoch: { sessionEpoch: s.delivery.epoch, schedulerEpoch: ++schedulerEpoch }, captureSeq: 0,
      pending: null, ready: null, nativeAdvanced: false, nativeCallDepth: 0, publishing: false,
      queryNeedsFreshState: true, expectNativePriming: true, admissions: new Map(), displayKey: null,
      pollsThisTick: 0, taskQueued: false, disposed: false, controlledBytes: 0, peakBytes: 0,
      tokens: new Set(), lastAdmission: null, lastSource: null, reentrantError: null,
      busyState: null, busyTicks: 0 };
    q.reserveBytes = (kind, bytes) => {
      if (!Number.isSafeInteger(bytes) || bytes < 0 || schedulerControlledBytes + bytes > 64 * 1024 * 1024
        || (kind === 'draw-snapshot' && schedulerSnapshotSlots >= 4)) return null;
      const token = { kind, bytes }; q.tokens.add(token); q.controlledBytes += bytes; schedulerControlledBytes += bytes;
      if (kind === 'draw-snapshot') {
        schedulerSnapshotSlots++; schedulerSnapshotBytes += bytes;
        schedulerSnapshotPeakBytes = Math.max(schedulerSnapshotPeakBytes, schedulerSnapshotBytes);
      }
      q.peakBytes = Math.max(q.peakBytes, q.controlledBytes);
      schedulerPeakBytes = Math.max(schedulerPeakBytes, schedulerControlledBytes); return token;
    };
    q.releaseBytes = (token) => { if (q.tokens.delete(token)) {
      q.controlledBytes -= token.bytes; schedulerControlledBytes -= token.bytes;
      if (token.kind === 'draw-snapshot') { schedulerSnapshotSlots--; schedulerSnapshotBytes -= token.bytes; }
    } };
    const atlasBytes = s.layout.textureSize * Math.max(s.layout.rows, s.layout.roiAtlasRows || 0) * 4;
    // Reserve the maximum owned atlas plus baseline PBO/read view conservatively before new allocations.
    q.buffersToken = q.reserveBytes('atlas+pbo+read-view+control-reserve', 3 * atlasBytes + 1024 * 1024);
    if (!q.buffersToken) throw new Error('[XR] B1prime 64MiB capacity exceeded');
    try {
      q.atlas = new Uint8Array(atlasBytes);
      q.snapshots = new OwnedDisplaySnapshots(drawCtx_, { maxSlots: 4, reserveBytes: q.reserveBytes, releaseBytes: q.releaseBytes });
      q.channel = new MessageChannel();
      q.channel.port1.onmessage = () => {
        q.taskQueued = false;
        if (!schedulerCurrent(s)) { schedulerCounts.staleTasks++; return; }
        schedulerTask(s);
      };
      // Node contract fixtures must not keep the process alive solely for an idle scheduler.
      q.channel.port1.unref?.(); q.channel.port2.unref?.();
      return q;
    } catch (error) { q.snapshots?.dispose(); q.releaseBytes(q.buffersToken); throw error; }
  };
  const queueSchedulerTask = (s) => {
    const q = s.scheduler;
    if (schedulerCurrent(s) && q.pending && !q.taskQueued && q.pollsThisTick < 2) {
      q.taskQueued = true; q.channel.port2.postMessage(0);
    }
  };
  const schedulerNative = (s, fn) => {
    const q = s.scheduler; q.nativeCalls = (q.nativeCalls || 0) + 1; q.nativeCallDepth++;
    try {
      const value = fn();
      if (q.reentrantError) throw q.reentrantError;
      return value;
    } finally { q.nativeCallDepth--; }
  };
  const disposeScheduler = (s, reason = 'session-discard') => {
    const q = s.scheduler;
    if (!q || q.disposed) return;
    if (q.nativeCallDepth) { q.reentrantError = new Error('[XR] B1prime cannot destroy an executing native call'); throw q.reentrantError; }
    q.disposed = true;
    q.channel.port1.close(); q.channel.port2.close();
    if (q.nativeAdvanced && q.pending && ![...q.admissions.values()].includes(q.pending)) {
      schedulerCounts.preAdmissionDiscards++;
      pushEntry(schedulerDiscards, { reason, kind: 'unknown-native-admission',
        captureSeq: q.pending.snapshotKey?.captureSeq ?? null, sourceVideoTimeS: q.pending.videoTime,
        acceptedMotionSeqs: q.pending.acceptedMotionSeqs?.slice() ?? [],
        acceptedMotionSeqOverflow: q.pending.acceptedMotionSeqOverflow ?? 0, epoch: { ...q.epoch } });
    }
    for (const [index, frame] of q.admissions) if (index !== q.lastSource) {
      schedulerCounts.discardedAdmissions++;
      pushEntry(schedulerDiscards, { reason, admissionFrameIndex: index, captureSeq: frame.snapshotKey?.captureSeq ?? null,
        sourceVideoTimeS: frame.videoTime, acceptedMotionSeqs: frame.acceptedMotionSeqs?.slice() ?? [],
        acceptedMotionSeqOverflow: frame.acceptedMotionSeqOverflow ?? 0, epoch: { ...q.epoch } });
    }
    if (q.ready?.kind === 'processed') {
      schedulerCounts.discardedReadyReceipts++;
      pushEntry(schedulerDiscards, { reason, kind: 'unpublished-processed',
        sourceFrameIndex: q.ready.descriptor.sourceTiming.sourceFrameIndex, epoch: { ...q.epoch } });
    }
    sensors.delivery?.drop(q.pending?.events || [], 'scheduler-epoch-discard');
    q.pending = null; q.ready = null; q.admissions.clear();
    if (q.displayKey && !drawCtx_?.isContextLost?.()) {
      // The renderer may still hold the committed texture during this CPU callback.
      // Reject old-epoch lookups now, then free its GL allocation after a post-render fence.
      q.snapshots.invalidateEpoch(q.epoch, 'scheduler-disposed');
      retiredSnapshotPools.push(q.snapshots);
    } else q.snapshots.dispose();
    q.atlas = null; q.releaseBytes(q.buffersToken);
  };
  const drainRetiredSnapshotPools = (final = false) => {
    for (let i = retiredSnapshotPools.length - 1; i >= 0; i--) {
      const pool = retiredSnapshotPools[i];
      try {
        if (!pool.gl.isContextLost?.()) {
          if (final) {
            // Teardown has no later render callback; finish the draw stream before deletion.
            pool.gl.finish();
            pool.dispose();
            retiredSnapshotPools.splice(i, 1);
            continue;
          }
          pool.fenceRetiredAfterRender();
          pool.pollRetired();
          if (pool.stats().pendingRetire) continue;
        }
        pool.dispose();
        retiredSnapshotPools.splice(i, 1);
      } catch (error) {
        schedulerCounts.errors++;
        reportSchedulerError('scheduler-retired-snapshot', error);
        if (final) {
          // Stop has no later post-render callback. Release logical ownership even if
          // the driver cannot confirm the fence; preserve the failure diagnostic.
          try { pool.dispose(); } catch (disposeError) {
            schedulerCounts.errors++;
            reportSchedulerError('scheduler-retired-snapshot-dispose', disposeError);
          }
          retiredSnapshotPools.splice(i, 1);
        }
      }
    }
  };
  const schedulerBarrier = (reason) => {
    const s = session_;
    if (!schedulerEnabled || !s) return;
    if (schedulerRebuilding || s.scheduler?.nativeCallDepth) {
      const error = new Error('[XR] B1prime mutator is busy during native call or rebuild');
      if (s.scheduler) s.scheduler.reentrantError = error;
      throw error;
    }
    const geometry = { videoWidth: s.videoWidth, videoHeight: s.videoHeight, rotation: diag.session.rotation, computeCtx: computeCtx_ };
    schedulerRebuilding = true; schedulerCounts.barrierResets++;
    try {
      // Only committed state may seed a rebuild. Never copy s.last from an unpublished transaction.
      if (s.scheduler.pending && !s.scheduler.nativeAdvanced) {
        sensors.carryForStage(s.scheduler.pending.events); s.scheduler.pending = null; s.pending = null;
      }
      destroySession(`barrier:${reason}`); heldDesc_ = null; lastExtrinsic_ = null; intrinsics_ = null;
      publication = null; lastPublishedSource = null;
      if (!computeContextLost() && !drawCtx_?.isContextLost?.()) {
        try {
          session_ = createSession({ ...geometry, coords: { origin: origin_, facing: facing_ }, framesUntilRecenter: startupFramesLeft_ });
        } catch (error) {
          session_ = null;
          schedulerCounts.errors++;
          pushEntry(schedulerFailures, { where: 'scheduler-barrier-create', message: String(error?.message || error),
            epoch: { ...s.scheduler.epoch }, nativeAdvanced: false, partialDispatchCount: 0 });
          reportSchedulerError('scheduler-barrier-create', error);
        }
      }
      pushEntry(diag.sessionLog, { event: 'scheduler-barrier', reason });
    } finally { schedulerRebuilding = false; }
  };
  const schedulerFailure = (s, where, error) => {
    schedulerCounts.errors++;
    pushEntry(schedulerFailures, { where, message: String(error?.message || error), epoch: { ...s.scheduler.epoch },
      nativeAdvanced: s.scheduler.nativeAdvanced, partialDispatchCount: activePublication?.dispatches || 0 });
    schedulerCounts.partialDispatchCount += activePublication?.dispatches || 0;
    // Report the original fault before rebuilding. The callback can itself change epochs.
    reportSchedulerError(where, error);
    if (schedulerCurrent(s)) {
      // Only events that never entered native can be carried. Settled/uncertain entries cannot be replayed.
      if (!s.scheduler.nativeAdvanced && s.scheduler.pending) {
        sensors.carryForStage(s.scheduler.pending.events); s.scheduler.pending = null; s.pending = null;
      }
      try { schedulerBarrier(where); }
      catch (barrierError) { schedulerCounts.errors++; reportSchedulerError('scheduler-barrier', barrierError); }
    }
  };
  const schedulerBusyWatchdog = (s) => {
    const q = s.scheduler;
    const state = q.ready ? 'ready' : q.nativeAdvanced ? 'native-advanced' : 'pending';
    if (q.busyState !== state) { q.busyState = state; q.busyTicks = 0; }
    if (++q.busyTicks < MAX_STALLED_GPU_TICKS) return false;
    schedulerCounts.watchdogResets++;
    schedulerFailure(s, 'scheduler-watchdog', new Error(`[XR] B1prime ${state} stalled for ${MAX_STALLED_GPU_TICKS} GPU ticks`));
    return true;
  };
  const schedulerRetireWatchdog = (s) => {
    const pendingRetire = s.scheduler.snapshots.stats().pendingRetire
      + retiredSnapshotPools.reduce((n, pool) => n + pool.stats().pendingRetire, 0);
    if (schedulerSnapshotSlots < 4 || pendingRetire === 0) {
      retireStallTicks = 0; retireWatchdogReported = false; return false;
    }
    if (retireWatchdogReported || ++retireStallTicks < MAX_STALLED_GPU_TICKS) return false;
    // A barrier may invalidate native history, but it cannot free an unsignaled GL fence.
    // Latch the warning until a fence signals to avoid a rebuild loop on permanent GPU stalls.
    retireWatchdogReported = true; schedulerCounts.watchdogResets++;
    schedulerFailure(s, 'scheduler-retire-watchdog', new Error(`[XR] B1prime retired snapshots blocked all four slots for ${MAX_STALLED_GPU_TICKS} GPU ticks`));
    return true;
  };
  const schedulerTask = (s) => {
    const q = s.scheduler, f = q.pending;
    if (!f || q.ready || !schedulerCurrent(s)) return;
    try {
      q.pollsThisTick++;
      if (!s.pyramid.isReadReady()) { schedulerCounts.waits++; queueSchedulerTask(s); return; }
      const t0 = performance.now(), view = s.pyramid.readReady(), rows = s.pyramid.lastReadRows;
      if (!view || view.byteLength > q.atlas.byteLength) throw new Error('[XR] B1prime invalid atlas read');
      q.atlas.set(view); const atlas = q.atlas.subarray(0, view.byteLength);
      pushSample(diag.stats.readMs, performance.now() - t0);
      // Before the first native mutator, including push and callback re-entry from native.
      q.nativeAdvanced = true; q.queryNeedsFreshState = true;
      sensors.delivery?.process(f.events);
      for (const e of f.events) {
        let accepted;
        try { accepted = schedulerNative(s, () => pushMotion(s.glue, e)); }
        catch (error) { sensors.delivery?.uncertain(e); throw error; }
        if (accepted) {
          sensors.delivery?.pushed(e);
          const seq = sensors.delivery?.seqOf(e);
          if (seq !== null && seq !== undefined) {
            const receipts = f.acceptedMotionSeqs ||= [];
            if (receipts.length < MAX_DISCARD_MOTION_SEQS) receipts.push(seq);
            else { f.acceptedMotionSeqOverflow = (f.acceptedMotionSeqOverflow ?? 0) + 1; schedulerCounts.acceptedMotionSeqOverflow++; }
          }
          diag.stats.motionEventsPushed++;
        }
        else { sensors.delivery?.drop([e], 'invalid-interval'); diag.stats.motionEventsSkipped++; }
      }
      schedulerNative(s, () => s.glue.pushDeviceOrientation(f.timeNanos, Number(f.alpha), Number(f.beta), Number(f.gamma)));
      const beforeStartup = schedulerNative(s, () => s.glue.getFramesUntilRecenter());
      const t1 = performance.now();
      const engineInputTime = f.frameTime ? f.frameTime * 1e9 : f.timeNanos;
      const videoTimeNanos = f.videoTime * 1e9;
      const pose = schedulerNative(s, () => s.glue.processFramePyramid(engineInputTime,
        videoTimeNanos, atlas, s.layout.textureSize, rows));
      const index = pose.videoFrameIndex;
      if (!Number.isInteger(index) || index < 0 || q.admissions.has(index)) throw new Error('[XR] B1prime invalid native admission index');
      // Record native admission before every fallible getter/receipt assembly operation.
      if (!q.snapshots.retain(f.snapshotKey, 'enginePrevious')) throw new Error('[XR] B1prime incoming snapshot expired');
      q.admissions.set(index, f); q.lastAdmission = index;
      const inputTimeNanos = engineInputTime;
      s.delivery.sourceFrames.set(index, { sourceVideoTimeS: f.videoTime, sourceFrameTimeS: f.frameTime ?? null,
        sourceTimeNanosInput: inputTimeNanos, sourceTimeNanos: Math.trunc(inputTimeNanos),
        sourceStageTimeNanos: f.timeNanos, sessionEpoch: q.epoch.sessionEpoch });
      while (s.delivery.sourceFrames.size > 64) { s.delivery.sourceFrames.delete(s.delivery.sourceFrames.keys().next().value); s.delivery.evicted++; }
      s.delivery.responses++;
      const engineReturnedAtMs = performance.now();
      pushSample(diag.stats.engineMs, engineReturnedAtMs - t1); diag.stats.processedFrames++;
      const afterStartup = schedulerNative(s, () => s.glue.getFramesUntilRecenter());
      const hasStep = Object.hasOwn(pose, 'stageMs');
      if (q.expectNativePriming && pose.repeated === true && !hasStep) {
        q.expectNativePriming = false;
        q.ready = { kind: 'nativePriming', incoming: f, admissionFrameIndex: index };
        return;
      }
      if (pose.repeated || !hasStep) throw new Error('[XR] B1prime unexplained native no-step response');
      q.expectNativePriming = false;
      const source = q.admissions.get(pose.sourceFrameIndex);
      if (!source || !q.snapshots.textureForPublication(source.snapshotKey, q.epoch)) throw new Error('[XR] B1prime missing native source snapshot');
      if (!q.snapshots.retain(source.snapshotKey, 'task')) throw new Error('[XR] B1prime source lease expired');
      const startupClear = beforeStartup === 1 && afterStartup === 0;
      if (startupClear) { q.expectNativePriming = true; schedulerCounts.startupDiscardedAdmissions++; }
      const flat = schedulerNative(s, () => s.glue.getWorldPoints());
      const points = [];
      for (let k = 0; k < flat.length; k += 5) points.push({ id: flat[k], confidence: flat[k + 1], position: { x: flat[k + 2], y: flat[k + 3], z: flat[k + 4] } });
      const timing = { sourceFrameIndex: pose.sourceFrameIndex, sourceVideoTimeS: source.videoTime,
        sourceFrameTimeS: source.frameTime ?? null, sourceTimeNanosInput: source.frameTime ? source.frameTime * 1e9 : source.timeNanos,
        sourceTimeNanos: Math.trunc(source.frameTime ? source.frameTime * 1e9 : source.timeNanos),
        sourceStageTimeNanos: source.timeNanos, sessionEpoch: q.epoch.sessionEpoch, sourceMetadataFound: true,
        repeated: false, engineReturnedAtMs };
      s.delivery.sourceTiming = timing;
      diag.stats.lastSourceFrameIndex = pose.sourceFrameIndex;
      const lag = index - pose.sourceFrameIndex; diag.stats.responseLag[lag] = (diag.stats.responseLag[lag] || 0) + 1;
      const descriptor = { position: Array.from(pose.position), rotationWxyz: Array.from(pose.rotationWxyz), intrinsics: Array.from(pose.intrinsics),
        lighting: pose.lighting ? { exposure: pose.lighting.exposure, temperature: pose.lighting.temperature } : null,
        status: pose.status, reason: pose.reason, worldPoints: points, detectedImages: structuredClone(pose.detectedImages || []),
        realityTexture: q.snapshots.textureForPublication(source.snapshotKey, q.epoch), sourceTiming: timing };
      for (const [key, value] of q.admissions) {
        if (key !== index || startupClear) { q.snapshots.release(value.snapshotKey, 'enginePrevious'); q.admissions.delete(key); }
      }
      if (source !== f) q.snapshots.release(f.snapshotKey, 'task');
      q.ready = { kind: 'processed', descriptor, sourceKey: source.snapshotKey, startupClear, pose, source };
      schedulerCounts.processed++;
    } catch (error) {
      if (q.nativeAdvanced) sensors.delivery?.drop(f.events, 'scheduler-process-error');
      schedulerFailure(s, 'scheduler-task', error);
    }
  };
  const schedulerGpu = ({ framework, frameStartResult: frame }) => {
    const s = session_, q = s?.scheduler;
    if (!q || runConfig.paused || cameraProcessingDisabled_) { drainRetiredSnapshotPools(); return; }
    q.pollsThisTick = 0;
    if (checkContext(s) || drawCtx_?.isContextLost?.()) {
      schedulerFailure(s, 'scheduler-context', new Error('[XR] B1prime context lost')); return;
    }
    try {
    if (q.pending || q.ready || q.nativeAdvanced || schedulerRebuilding) {
      if (!frame.repeatFrame) schedulerCounts.droppedBusy++;
      drainRetiredSnapshotPools();
      q.snapshots.fenceRetiredAfterRender(); q.snapshots.pollRetired();
      if (schedulerRetireWatchdog(s) || schedulerBusyWatchdog(s)) return;
      queueSchedulerTask(s); return;
    }
    if (frame.repeatFrame) { drainRetiredSnapshotPools(); q.snapshots.fenceRetiredAfterRender(); q.snapshots.pollRetired();
      schedulerRetireWatchdog(s); return; }
    } catch (error) { schedulerFailure(s, 'scheduler-housekeeping', error); return; }
    try {
      maybeDispatchConfigChange(framework);
      if (!schedulerCurrent(s)) return;
    } catch (error) { schedulerFailure(s, 'scheduler-before-stage', error); return; }
    if (!schedulerCurrent(s)) return;
    if (frame.textureWidth !== s.videoWidth || frame.textureHeight !== s.videoHeight) { diag.stats.sizeMismatchFrames++; return; }
    if (checkContext(s) || drawCtx_?.isContextLost?.()) return;
    // Match B0's orientation → time → sensor latch, before added GL work.
    const { alpha, beta, gamma } = lastDevicePose_;
    const timeNanos = 1e3 * window.performance.now() * 1e3;
    const events = sensors.takeForStage() || [];
    const f = { ...frame, alpha, beta, gamma, timeNanos, events };
    try {
      q.busyState = null; q.busyTicks = 0;
      f.latchedOnly = true; q.pending = f;
      drainRetiredSnapshotPools();
      imageTargets.pump();
      if (!schedulerCurrent(s)) return;
      // Finish bounded target replay while the native FIFO is known empty. Admitting a pixel
      // here would force another barrier for the next replay item and restart its queue forever.
      if (diag.imageTargets?.replayPending() > 0) {
        schedulerCounts.droppedTargetWarmup++; sensors.carryForStage(events); q.pending = null; return;
      }
      f.latchedOnly = false;
      q.snapshots.fenceRetiredAfterRender(); q.snapshots.pollRetired();
      if (schedulerRetireWatchdog(s)) return;
      const input = f.frameTime ? f.frameTime * 1e9 : timeNanos;
      f.snapshotKey = q.snapshots.capture({ sourceDrawTexture: frame.cameraTexture, width: frame.textureWidth, height: frame.textureHeight,
        meta: { epoch: q.epoch, captureSeq: q.captureSeq++, rotation: frame.orientation,
          sourceVideoTimeS: frame.videoTime, sourceFrameTimeS: frame.frameTime ?? null,
          sourceTimeNanosInput: input, sourceTimeNanos: Math.trunc(input), sourceStageTimeNanos: timeNanos, roiRevision: q.epoch.schedulerEpoch } });
      if (!f.snapshotKey) {
        schedulerCounts.droppedCapacity++; sensors.carryForStage(events); q.pending = null;
        if (s.pending === f) s.pending = null;
        return;
      }
      glErrorStreak = 0;
      q.snapshots.retain(f.snapshotKey, 'task'); q.snapshots.release(f.snapshotKey, 'gpuPending');
      q.pending = f; stageFrame(s, f); schedulerCounts.admitted++; queueSchedulerTask(s);
    } catch (error) {
      if (error?.code === 'preexisting-gl-error' && schedulerCurrent(s) && !f.snapshotKey) {
        // The flag belongs to earlier draw work. It was consumed by capture's one
        // preflight getError; this frame has no owned pixels or native admission.
        schedulerCounts.droppedGlError++;
        sensors.carryForStage(events); q.pending = null; s.pending = null;
        // A persistent flag fails every frame: record and report the first of a consecutive run only
        // (reportOnce also dedups by message); the counter above still counts every dropped frame.
        if (glErrorStreak++ === 0) {
          pushEntry(schedulerFailures, { where: 'scheduler-capture-gl-preexisting', message: error.message,
            epoch: { ...q.epoch }, nativeAdvanced: false, partialDispatchCount: 0 });
          reportSchedulerError('scheduler-capture-gl-preexisting', error);
        }
        return;
      }
      q.pending ||= f; schedulerFailure(s, 'scheduler-stage', error);
    }
  };
  const schedulerRepeat = (s, framework) => {
    if (!s.lastDesc) return null;
    if (!s.scheduler.snapshots.textureForPublication(s.scheduler.displayKey, s.scheduler.epoch)) {
      schedulerFailure(s, 'scheduler-repeat-snapshot', new Error('[XR] B1prime committed snapshot unavailable')); return null;
    }
    activePublication = { session: s, dispatches: 0 }; s.scheduler.publishing = true;
    try { return finishResult(framework, resultFromDesc(s.lastDesc), s.lastDesc.sourceTiming, 'callback-repeat'); }
    catch (error) { schedulerFailure(s, 'scheduler-repeat-publication', error); return null; }
    finally { s.scheduler.publishing = false; activePublication = null; }
  };
  const schedulerCpu = (framework, frameStartResult) => {
    const s = session_, q = s?.scheduler;
    if (!q) return !session_ && !computeContextLost() && !drawCtx_?.isContextLost?.()
      ? inactiveAnswer(framework, frameStartResult) : null;
    if (runConfig.paused) return null;
    if (checkContext(s) || drawCtx_?.isContextLost?.()) {
      schedulerFailure(s, 'scheduler-cpu-context', new Error('[XR] B1prime context lost before publication')); return null;
    }
    const ready = q.ready;
    if (!ready) return schedulerRepeat(s, framework);
    if (publication) { publication.newEngineResponse = true; publication.engineFresh = ready.kind === 'processed'; }
    if (ready.kind === 'nativePriming') {
      if (!q.snapshots.textureForPublication(ready.incoming.snapshotKey, q.epoch)) {
        schedulerFailure(s, 'scheduler-ack', new Error('[XR] B1prime priming snapshot expired')); return null;
      }
      q.snapshots.release(ready.incoming.snapshotKey, 'task');
      q.ready = null; q.pending = null; s.pending = null; q.nativeAdvanced = false;
      schedulerCounts.primingAcked++;
      return schedulerRepeat(s, framework);
    }
    q.publishing = true; activePublication = { session: s, dispatches: 0 };
    try {
      maybeDispatchConfigChange(framework);
      const texture = q.snapshots.textureForPublication(ready.sourceKey, q.epoch);
      if (!texture || !schedulerCurrent(s)) throw new Error('[XR] B1prime stale publication snapshot');
      ready.descriptor.realityTexture = texture;
      const result = finishResult(framework, resultFromDesc(ready.descriptor), ready.descriptor.sourceTiming, 'engine-response');
      if (!schedulerCurrent(s)) throw new Error('[XR] B1prime stale publication commit');
      q.snapshots.retain(ready.sourceKey, 'display');
      if (q.displayKey && q.displayKey !== ready.sourceKey) q.snapshots.release(q.displayKey, 'display');
      q.displayKey = ready.sourceKey; q.snapshots.release(ready.sourceKey, 'task');
      s.lastDesc = ready.descriptor; s.last = ready.pose; s.lastWorldPoints = ready.descriptor.worldPoints; s.hasResponse = true;
      q.lastSource = ready.descriptor.sourceTiming.sourceFrameIndex;
      q.ready = null; q.pending = null; s.pending = null; q.nativeAdvanced = false;
      q.queryNeedsFreshState = ready.startupClear;
      diag.stats.freshResponses++; schedulerCounts.publications++;
      return result;
    } catch (error) { schedulerFailure(s, 'scheduler-publication', error); return null; }
    finally { q.publishing = false; activePublication = null; }
  };
  const schedulerCommand = (reason, effective, action) => {
    if (!schedulerEnabled) return action();
    if (schedulerRebuilding || session_?.scheduler.nativeCallDepth) {
      const error = new Error('[XR] B1prime mutator busy'); if (session_?.scheduler) session_.scheduler.reentrantError = error; throw error;
    }
    if (!session_) return action();
    const outerPublication = activePublication; activePublication = null;
    const before = session_, callsBefore = before.scheduler.nativeCalls || 0;
    try {
      if (effective) schedulerBarrier(reason);
      try { return action(); }
      catch (error) {
        if (session_ && (effective || before.scheduler.reentrantError || (before.scheduler.nativeCalls || 0) !== callsBefore)) schedulerBarrier(`${reason}-partial-error`);
        throw error;
      }
    } finally { activePublication = outerPublication; }
  };
  if (schedulerEnabled) Object.defineProperty(diag, 'scheduler', { enumerable: true, get: () => {
    const q = session_?.scheduler;
    return { enabled: schedulerEnabled, mode: schedulerEnabled ? 'gpu-task-owned-v1' : 'legacy-fallback',
      fallbackReason: schedulerFallbackReason, watchdogTicks: MAX_STALLED_GPU_TICKS,
      pending: !!q?.pending, ready: !!q?.ready,
      readyKind: q?.ready?.kind ?? null, pollsThisTick: q?.pollsThisTick ?? 0, taskQueued: !!q?.taskQueued,
      nativeAdvanced: !!q?.nativeAdvanced, queryBusyReason: schedulerEnabled ? schedulerBusyReason() : null,
      epoch: q ? { ...q.epoch } : null,
      counters: { ...schedulerCounts }, lastAdmission: q?.lastAdmission ?? null, lastSource: q?.lastSource ?? null,
      memory: { controlledBytes: schedulerControlledBytes, peakBytes: schedulerPeakBytes,
        wasmHeapBytes: engine_?.Module?.HEAPU8?.byteLength ?? null, totalSdkHeapVerified: false, driverBytes: null },
      snapshots: { ...(q?.snapshots.stats() || {}), uniqueSlots: schedulerSnapshotSlots,
        activeUniqueSlots: q?.snapshots.stats().uniqueSlots ?? 0,
        retiredUniqueSlots: retiredSnapshotPools.reduce((n, pool) => n + pool.stats().uniqueSlots, 0),
        retiredPools: retiredSnapshotPools.length,
        pendingRetire: (q?.snapshots.stats().pendingRetire ?? 0) + retiredSnapshotPools.reduce((n, pool) => n + pool.stats().pendingRetire, 0),
        controlledBytes: schedulerSnapshotBytes, peakBytes: schedulerSnapshotPeakBytes,
        driverBytes: null }, failures: schedulerFailures.map((e) => ({ ...e, epoch: { ...e.epoch } })),
      discards: schedulerDiscards.map((e) => ({ ...e, epoch: { ...e.epoch },
        acceptedMotionSeqs: e.acceptedMotionSeqs?.slice() })) };
  } });

  // JS state (SDK factory locals; tracking-controller.ts:256-306).
  let isSimulator_ = false;
  let fakeSimulatorExtrinsic_ = { position: { x: 0, y: 0, z: 0 }, rotation: { w: 1, x: 0, y: 0, z: 0 } };
  let lastDevicePose_ = { timeStamp: 0, alpha: 0, beta: 0, gamma: 0 };
  let lastGeolocation_ = { latitude: 0, longitude: 0, accuracy: 0 };
  let absoluteScale_ = false;          // a
  let enableLighting_ = false;         // h
  let enableWorldPoints_ = false;      // S
  let disableWorldTracking_ = false;   // R
  let leftHandedAxes_ = false;         // L
  let mirroredDisplay_ = false;        // Y
  let notifyOthersOfConfigChange_ = false;  // H
  let trackingStatus_ = 'UNSPECIFIED';  // t
  let trackingReason_ = 'UNSPECIFIED';  // e
  let cameraProcessingDisabled_ = false;  // r
  let scale_ = 1;                      // d
  let origin_ = { x: 0, y: 2, z: 0 };  // P
  let facing_ = { w: 1, x: 0, y: 0, z: 0 };  // l
  let recenterOrigin_ = { x: 0, y: 2, z: 0 };  // p
  let recenterFacing_ = { w: 1, x: 0, y: 0, z: 0 };  // f
  const cam_ = { pixelRectWidth: 0, pixelRectHeight: 0, nearClipPlane: 0.01, farClipPlane: 1000 };  // n
  let projectWayspots_ = [];           // V (used only by VPS, which the standalone build never enables)
  const detectedImageMap_ = {};        // T
  const imageTargetCache_ = {};        // u
  let appResourcesLoaded_ = false;     // IA
  let sessionStarted_ = false;         // AA
  let framework_ = null;               // gA
  let drawCtx_ = null;                 // CA
  let computeCtx_ = null;              // o / cA
  let fakeRealityTexture_ = null;      // v
  let intrinsics_ = null;              // DA
  let engineVpsEventsDisabled_ = false;  // EA
  let toCheckAllZeros_ = null;         // w (window._c8.checkAllZeroes at the first attach)
  let session_ = null;                 // engineInit state (glue + pyramid)
  let cameraSessionAttached_ = false;  // onSessionAttach ran with camera processing: size / orientation changes rebuild the session
  let lastExtrinsic_ = null;           // pose of the last response of a destroyed session (resetOriginToLastPose)
  let heldDesc_ = null;                // the last response of a destroyed session (the original keeps window._c8.reality across engineInit)
  let inactiveWarned_ = false;         // the unsupported-capture warning was given in this episode
  // C++ configuredCoordinateSystem (tracking-controller.cc): the last coordinate system configureXr sent. rebuildEngine
  // (recenter, engineInit) applies it; resetOriginToLastPose changes only the live engine coordinate system, not this one.
  let configured_ = { origin: { x: 0, y: 2, z: 0 }, facing: { w: 1, x: 0, y: 0, z: 0 }, scale: 1 };
  let dataRecorderRequested_ = false;  // configure({enableDataRecorder}) (iA); the recorder itself is not implemented
  // The original keeps framesUntilRecenter in the EngineData singleton: engineInit again (size / orientation change) does not
  // arm it again, engineCleanup (detach) does. This is the count the last destroyed glue had left; null = a fresh start (5).
  let startupFramesLeft_ = null;
  let heldInactive_ = false;           // heldDesc_ was served as an inactive answer since it was taken (RM-4c m-B)
  // A rebuild that was postponed because the compute context was lost: {kind, videoWidth, videoHeight, orientation, computeCtx}.
  // It is carried out by onProcessCpu as soon as the context is back, so a context that returns without a later size change
  // does not leave the world tracking off for good (RM-4d, review m3). Not set for an unsupported capture or a failed build.
  let deferredRebuild_ = null;
  const imageTargets = createImageTargets({
    options, diag, pushEntry, getComputeCtx: () => computeCtx_, getGlue: () => (session_ ? (session_.imageGlue || session_.glue) : null),
  });

  const maybeDispatchConfigChange = (framework, recenter) => {
    if ((!notifyOthersOfConfigChange_ && !recenter) || !framework) return;
    const callbackSession = session_;
    schedulerDispatch(framework, 'cameraconfigured', {
      mirroredDisplay: mirroredDisplay_, origin: { position: origin_, rotation: facing_ }, recenter: !!recenter,
    });
    if (schedulerEnabled && callbackSession !== session_) throw new Error('[XR] B1prime config callback changed epoch');
    notifyOthersOfConfigChange_ = false;
  };

  const updateDeviceOrientation = ({ timeStamp, alpha, beta, gamma }) => { lastDevicePose_ = { timeStamp, alpha, beta, gamma }; };
  const updateDeviceGeolocation = ({ coords }) => {
    lastGeolocation_ = { latitude: coords.latitude, longitude: coords.longitude, accuracy: coords.accuracy };
  };
  const addWindowListener = (name, cb, useCapture = true) => window.addEventListener(name, cb, useCapture);
  const removeWindowListener = (name, cb, useCapture = true) => window.removeEventListener(name, cb, useCapture);
  const addMotionOrientationListeners = () => {
    addWindowListener('deviceorientation', updateDeviceOrientation);
    addWindowListener('devicemotion', sensors.updateDeviceMotion);
  };

  // ---- engineInit (c8EmAsm_engineInit) and the C++-side configuration of GlueRunner ----
  // The chunk keeps the original JS state; GlueRunner keeps the same JS mirror and the configured
  // coordinate system / mask (spec 06 §3.2). Every configureXr-triggering call is forwarded with the same
  // arguments, so both sides stay equal.
  const glueConfigure = (glue) => glue.configure({ enableWorldPoints: enableWorldPoints_, enableLighting: enableLighting_,
    leftHandedAxes: leftHandedAxes_, mirroredDisplay: mirroredDisplay_ });

  // The engine takes any capture size and the rotations the original maps to a device orientation (FULL-8, US-023; spec 07 section 3 /
  // 11.3; R16 is lifted). null when the capture is supported. Rotation is XR8's `rotation` argument in degrees.
  const SUPPORTED_ROTATIONS = [0, 90, 180, 270, -90, -180, -270];
  const unsupportedCapture = (videoWidth, videoHeight, rotation) => (!SUPPORTED_ROTATIONS.includes(rotation)
    ? `slam_refact chunk supports rotation 0, 90, 180, 270 (and the negatives) only (got ${videoWidth}x${videoHeight} rotation ${rotation})` : null);

  const sameCoords = (a, b) => a.origin.x === b.origin.x && a.origin.y === b.origin.y && a.origin.z === b.origin.z
    && a.facing.w === b.facing.w && a.facing.x === b.facing.x && a.facing.y === b.facing.y && a.facing.z === b.facing.z;

  // A-1a: validates options.engineConfig (only `intrinsics` and `accuracy` are accepted; null / undefined values are skipped) and copies
  // it into the glue constructor config. Returns the applied keys. A malformed value is a `[XR]` error, never silently dropped.
  const ENGINE_CONFIG_KEYS = ['intrinsics', 'accuracy'];
  const isPlainObject = (v) => v !== null && typeof v === 'object' && !Array.isArray(v);
  const applyEngineConfig = (glueConfig, engineConfig) => {
    if (!isPlainObject(engineConfig)) throw new Error('[XR] SLAM_REFACT_CHUNK_OPTIONS.engineConfig must be an object {intrinsics, accuracy}');
    const unknown = Object.keys(engineConfig).filter((k) => !ENGINE_CONFIG_KEYS.includes(k));
    if (unknown.length) throw new Error(`[XR] SLAM_REFACT_CHUNK_OPTIONS.engineConfig: unsupported key(s) ${unknown.join(', ')} (supported: ${ENGINE_CONFIG_KEYS.join(', ')})`);
    const { intrinsics, accuracy } = engineConfig;
    const applied = [];
    if (intrinsics !== undefined && intrinsics !== null) {
      if (!Array.isArray(intrinsics) || intrinsics.length !== 4 || !intrinsics.every((x) => typeof x === 'number' && Number.isFinite(x))) {
        throw new Error('[XR] SLAM_REFACT_CHUNK_OPTIONS.engineConfig.intrinsics must be [fx, fy, cx, cy], four finite numbers');
      }
      glueConfig.intrinsics = [...intrinsics];
      applied.push('intrinsics');
    }
    if (accuracy !== undefined && accuracy !== null) {
      if (!isPlainObject(accuracy)) throw new Error('[XR] SLAM_REFACT_CHUNK_OPTIONS.engineConfig.accuracy must be an object');
      try { glueConfig.accuracy = structuredClone(accuracy); } catch (error) {
        throw new Error(`[XR] SLAM_REFACT_CHUNK_OPTIONS.engineConfig.accuracy cannot be copied: ${error && error.message || error}`);
      }
      applied.push('accuracy');
    }
    return applied;
  };

  // coords: the coordinate system the engine starts with. engineInit -> rebuildEngine applies configuredCoordinateSystem
  // (C), so the first attach and a video size change start with `configured_`; an orientation change starts with the JS state
  // (P, l) because the configureXr that follows it sends P. When C differs from P the glue's live system differs from the JS
  // state (`liveStale`): the next configureXr / recenter re-sends the right one first (resendJsCoords, recenter).
  // framesUntilRecenter: the count the previous glue had left (null = the original's fresh 5).
  const createSession = ({ videoWidth, videoHeight, rotation, computeCtx, coords, framesUntilRecenter = null }) => {
    const unsupported = unsupportedCapture(videoWidth, videoHeight, rotation);
    if (unsupported) throw new Error(`[XR] ${unsupported}`);
    if (!computeCtx || typeof computeCtx.fenceSync !== 'function') {
      throw new Error('[XR] slam_refact chunk needs XR8.run({webgl2: true}) (WebGL2 compute context)');
    }
    const { Module } = engine_;
    const device = resolvedIdentity?.selected ?? window.XR8.XrDevice.deviceEstimate();
    const deviceSource = resolvedIdentity?.selected?.source ?? 'XR8.XrDevice.deviceEstimate';
    const glueConfig = {
      captureWidth: videoWidth, captureHeight: videoHeight, rotationDeg: rotation,
      manufacturer: device.manufacturer || '', model: device.model || '',
      estimateScale: absoluteScale_, disableVio: disableWorldTracking_,
      leftHandedAxes: leftHandedAxes_, mirroredDisplay: mirroredDisplay_,
    };
    if (framesUntilRecenter !== null) glueConfig.framesUntilRecenter = framesUntilRecenter;
    // A-1a (opt-in, default OFF): SLAM_REFACT_CHUNK_OPTIONS.engineConfig = {intrinsics: [fx, fy, cx, cy], accuracy: {...}} goes to the
    // glue constructor unchanged (the engine binding validates the values). Without the option glueConfig is exactly as before.
    const engineConfigKeys = options.engineConfig === undefined ? null : applyEngineConfig(glueConfig, options.engineConfig);
    const sentGlueConfig = engineConfigKeys ? structuredClone(glueConfig) : { ...glueConfig };  // diagnostic copy; never passed back to the engine
    // options.createGlue is a test hook (chunk-contract.test.mjs wraps or replaces the engine glue). Whether a replaced glue was
    // used is decided here, for every session, not once at engine load (RM-4c m-F): once true it stays true.
    const createGlue = options.createGlue;
    if (createGlue) engine_.loaded.glueReplaced = true;
    const glue = createGlue ? createGlue(Module, glueConfig) : new Module.SlamRefactGlue(glueConfig);
    let pyramid;
    const wasRebuilding = schedulerRebuilding;
    if (schedulerEnabled) schedulerRebuilding = true;
    const imageGlue = schedulerEnabled ? new Proxy(glue, { get(target, name) {
      const value = target[name];
      if (typeof value !== 'function') return value;
      return (...args) => {
        const current = session_;
        if (!current || schedulerRebuilding) return value.apply(target, args);
        if (current.nativeGlue !== glue) return current.glue[name](...args);
        if (/^(get|has|is)/.test(String(name))) return current.glue[name](...args);
        const q = current.scheduler;
        const effective = !!((q.pending && !q.pending.latchedOnly) || q.ready || q.nativeAdvanced || q.lastAdmission !== null);
        return schedulerCommand('image-target', effective, () => session_.glue[name](...args));
      };
    } }) : glue;
    try {
      // Mirror = chunk state: recenter point (p, f) first, then the current origin / facing (P, l) and cam.
      glue.updateCameraProjectionMatrix({ origin: { ...recenterOrigin_ }, facing: { ...recenterFacing_ }, updateRecenterPoint: true });
      glueConfigure(glue);
      glue.updateCameraProjectionMatrix({ cam: { ...cam_ }, origin: { ...coords.origin }, facing: { ...coords.facing }, updateRecenterPoint: false });
      if (!absoluteScale_ && scale_ !== origin_.y) {
        warnOnce(`[XR] slam_refact chunk: scale ${scale_} != origin.y ${origin_.y} (updateCameraProjectionMatrix not called); the engine glue uses scale = origin.y`);
      }
      const layout = glue.pyramidLayout();
      pyramid = new Gr8ComputePyramid(computeCtx, layout, { cameraWidth: videoWidth, cameraHeight: videoHeight, rotation, warn: warnOnce });
      // targets loaded before this glue existed (size / orientation change, or loaded while no session existed); never throws. The ones that
      // are being detected right now go into the engine first (RM-4e, review A-5), so a rebuild does not make them lose and find again.
      imageTargets.attach(imageGlue, framework_, Object.keys(detectedImageMap_));
      // gpuCheck compares with SlamRefactGlue.cpuPyramidAtlas, which builds with rotation 0 only: a session with another rotation is not checked
      // (the comparison would report the rotation, not a defect); UNVERIFIED for rotation != 0 (docs/design/pkg-a-landscape-report.md).
      const gpuCheck = options.gpuCheck && rotation === 0
        ? { every: Math.max(1, options.gpuCheck.every | 0 || 10), left: Math.max(0, options.gpuCheck.frames | 0 || 30) } : null;
      const roiCheck = options.roiCheck
        ? { every: Math.max(1, options.roiCheck.every | 0 || 1), left: Math.max(0, options.roiCheck.frames | 0 || 20) } : null;
      // Old WASM has neither getter. In particular getIntrinsic() is a display projection,
      // not capture-space K, and a build label/hash alone cannot establish compiled features.
      const readback = { captureIntrinsics: null, captureIntrinsicsSource: 'unavailable',
        captureIntrinsicsConvention: 'engine-capture-before-processing',
        compileFeatures: { loopClose: 'unknown' }, compileFeaturesSource: 'unavailable', errors: [] };
      try {
        if (typeof glue.getCaptureIntrinsics === 'function') {
          readback.captureIntrinsics = { ...glue.getCaptureIntrinsics() };
          readback.captureIntrinsicsSource = 'SlamRefactGlue.getCaptureIntrinsics';
        }
        if (typeof Module.SlamRefactGlue.getCompileFeatures === 'function') {
          const features = Module.SlamRefactGlue.getCompileFeatures();
          if (typeof features.loopClose === 'boolean') {
            readback.compileFeatures.loopClose = features.loopClose;
            readback.compileFeaturesSource = 'SlamRefactGlue.getCompileFeatures';
          }
        }
      } catch (error) {
        readback.errors.push(String(error && error.message || error));  // diagnostics must not fail tracking
      }
      // What the engine reports it actually runs with (glue getEffectiveConfig: accuracy flags, ignoredKeys), one call per engine construction;
      // a plain-data copy, so a rebuilt session never shows the previous engine's config. The engine Worker would need a protocol message
      // for it, so that path says so instead of guessing.
      try {
        readback.effectiveConfig = ew_ ? { available: false, reason: 'engine-worker' }
          : typeof glue.getEffectiveConfig === 'function' ? structuredClone(glue.getEffectiveConfig())
            : { available: false, reason: 'engine has no getEffectiveConfig' };
      } catch (error) {
        readback.effectiveConfig = { available: false, reason: String(error && error.message || error) };
      }
      diag.session = { videoWidth, videoHeight, rotation,
        device: { manufacturer: sentGlueConfig.manufacturer, model: sentGlueConfig.model, source: deviceSource },
        glueConfig: sentGlueConfig,
        ...(engineConfigKeys ? { engineConfig: { keys: engineConfigKeys, transport: ew_ ? 'engine-worker' : 'main-thread' } } : {}),
        engineManifest: { jsSha256: engine_.loaded.manifest.jsSha256, wasmSha256: engine_.loaded.manifest.wasmSha256,
          matchesLoadedBytes: engine_.loaded.matchesManifest, glueReplaced: engine_.loaded.glueReplaced },
        configurationReadback: readback,
        layout: { textureSize: layout.textureSize, rows: layout.rows }, renderer: rendererInfo(computeCtx), readback: pyramid.stats };
      diag.stats.sessionsCreated++;
      const delivery = deliveryOptions ? { epoch: ++nextSessionEpoch, sourceFrames: new Map(), sourceTiming: null, responses: 0, evicted: 0 } : null;
      if (delivery) diag.session.sessionEpoch = delivery.epoch;
      pushEntry(diag.sessionLog, { event: 'created', videoWidth, videoHeight, rotation });
      const nativeGlue = glue;
      const guardedGlue = schedulerEnabled ? new Proxy(glue, { get(target, name) {
        const value = target[name];
        return typeof value === 'function' ? (...args) => {
          const current = session_;
          return current?.nativeGlue === nativeGlue && current.scheduler
            ? schedulerNative(current, () => value.apply(target, args)) : value.apply(target, args);
        } : value;
      } }) : glue;
      const result = { glue: guardedGlue, nativeGlue, imageGlue, pyramid, layout, videoWidth, videoHeight, pending: null, textures: new Map(), delivery,
        last: null, lastWorldPoints: [], activeTexture: null, hasResponse: false, gpuCheck, roiCheck, prevAllZeroes: null,
        lastDesc: null, liveStale: !sameCoords(coords, { origin: origin_, facing: facing_ }), contextLost: false };
      if (schedulerEnabled) result.scheduler = makeScheduler(result);
      return result;
    } catch (error) {
      if (pyramid) pyramid.dispose();
      glue.delete();
      throw error;
    } finally { if (schedulerEnabled) schedulerRebuilding = wasRebuilding; }
  };

  const rendererInfo = (gl) => {
    const dbg = gl.getExtension('WEBGL_debug_renderer_info');
    return { renderer: gl.getParameter(dbg ? dbg.UNMASKED_RENDERER_WEBGL : gl.RENDERER), version: gl.getParameter(gl.VERSION) };
  };

  const destroySession = (reason = 'session-discard') => {
    ewBuildToken_++;  // R1: cancels a Worker-mode build that waits for its layout (no-op otherwise)
    if (!session_) return;
    const s = session_;
    if (schedulerEnabled) disposeScheduler(s, reason);
    session_ = null;
    sensors.delivery?.drop(s.pending?.events || [], 'session-discard');
    if (schedulerEnabled && s.lastDesc) {
      lastExtrinsic_ = { position: s.lastDesc.position.slice(), rotationWxyz: s.lastDesc.rotationWxyz.slice() };
    }
    if (s.hasResponse && s.last && !schedulerEnabled) {
      lastExtrinsic_ = { position: Array.from(s.last.position), rotationWxyz: Array.from(s.last.rotationWxyz) };
    }
    // Only a committed descriptor may supply the LIMITED answer while an ON
    // lifecycle has no session. The answer replaces its retired texture with
    // this callback's live camera texture; scheduler barriers clear heldDesc_.
    if (s.lastDesc) { heldDesc_ = s.lastDesc; heldInactive_ = false; }
    try { startupFramesLeft_ = s.glue.getFramesUntilRecenter(); } catch (error) { pushEntry(diag.warnings, `getFramesUntilRecenter: ${error}`); }
    try { s.pyramid.dispose(); } catch (error) { pushEntry(diag.warnings, `pyramid dispose: ${error}`); }
    try { s.glue.delete(); } catch (error) { pushEntry(diag.warnings, `glue delete: ${error}`); }
    diag.stats.sessionsDestroyed++;
    pushEntry(diag.sessionLog, { event: 'destroyed', videoWidth: s.videoWidth, videoHeight: s.videoHeight });
  };

  // c8EmAsm_stageFrame.
  const stageFrame = (s, frame) => {
    if (s.pending) {
      // The previous staged frame was never processed (onProcessCpu skipped it): its IMU events stay
      // queued for this frame and its pyramid is replaced (spec 07 §2.4).
      frame.events = s.pending.events.concat(frame.events);
      diag.stats.droppedStaged++;
    }
    const t0 = performance.now();
    // The ROIs the engine wants in this draw (image targets): the list after the last tracked frame, as in the original
    // (stageFrame draws with the engine's nextRois, whatever targets are loaded right now). The engine is always asked (RM-4d,
    // review RM-4c B-1): its list is fixed by the last processed frame, so it can still hold ROIs after the app cleared or replaced
    // the targets between onProcessCpu and onProcessGpu; skipping the call then drew too few rows and processFramePyramid threw on
    // every frame for good. Without any target the list is empty and the draw / readback are the old ones, byte for byte.
    frame.rois = s.glue.getRequestedRois();
    s.pyramid.stage(frame.computeTexture, frame.rois);
    pushSample(diag.stats.stageMs, performance.now() - t0);
    if (s.gpuCheck && s.gpuCheck.left > 0 && diag.stats.stagedFrames % s.gpuCheck.every === 0) {
      frame.checkRgba = s.pyramid.readTextureRgba(frame.computeTexture, frame.textureWidth, frame.textureHeight);
    }
    diag.stats.stagedFrames++;
    s.pending = frame;
  };

  // ---- engine Worker mode (R1, opt-in; ew_ is null otherwise and none of this runs) ----
  // The frame is read back into a buffer the Worker handed back earlier, transferred with its IMU events and orientation, and the
  // engine answers later: results are applied here in submission order at the next onProcessCpu (so the pose answer lags the
  // main-thread mode by the frames in flight). The post-processing of a result is the tail of processStagedFrame below.
  const applyWorkerResult = (s, r) => {
    const ctx = s.ewFrames.get(r.seq);
    s.ewFrames.delete(r.seq);
    if (r.error) {
      diag.stats.processErrors++;
      reportOnce('process', r.error);
      return;
    }
    const { f, allZeroes } = ctx;
    const pose = r.pose;
    const engineReturnedAtMs = performance.now();
    diag.stats.processedFrames++;
    const stageIndex = pose.videoFrameIndex;
    noteFrameDone(f);  // measurement option only
    if (s.delivery) {
      const d = s.delivery;
      const inputTimeNanos = f.frameTime ? f.frameTime * 1e9 : f.timeNanos;
      d.sourceFrames.set(stageIndex, { sourceVideoTimeS: f.videoTime, sourceFrameTimeS: f.frameTime ?? null,
        sourceTimeNanos: Math.trunc(inputTimeNanos), sourceTimeNanosInput: inputTimeNanos,
        sourceStageTimeNanos: f.timeNanos, sessionEpoch: d.epoch });
      while (d.sourceFrames.size > 64) { d.sourceFrames.delete(d.sourceFrames.keys().next().value); d.evicted++; }
      const source = !pose.repeated || s.hasResponse ? d.sourceFrames.get(pose.sourceFrameIndex) : null;
      d.sourceTiming = { sourceFrameIndex: pose.sourceFrameIndex, sourceVideoTimeS: null, sourceFrameTimeS: null,
        sourceTimeNanos: null, sourceTimeNanosInput: null, sourceStageTimeNanos: null,
        sessionEpoch: d.epoch, ...source, sourceMetadataFound: !!source, repeated: !!pose.repeated, engineReturnedAtMs };
      d.responses++;
      if (publication) { publication.newEngineResponse = true; publication.engineFresh = !pose.repeated; }
    }
    s.textures.set(stageIndex, f.computeTexture);
    while (s.textures.size > 32) s.textures.delete(s.textures.keys().next().value);
    if (!pose.repeated) {
      s.hasResponse = true;
      diag.stats.freshResponses++;
      const lag = stageIndex - pose.sourceFrameIndex;
      diag.stats.lastSourceFrameIndex = pose.sourceFrameIndex;
      diag.stats.responseLag[lag] = (diag.stats.responseLag[lag] || 0) + 1;
      s.activeTexture = s.textures.get(pose.sourceFrameIndex) ?? null;
      for (const k of s.textures.keys()) if (k < pose.sourceFrameIndex) s.textures.delete(k);
      if (toCheckAllZeros_ && s.prevAllZeroes !== null) {
        window._c8.isAllZeroes = s.prevAllZeroes ? 1 : 0;
        window._c8.wasEverNonZero = !s.prevAllZeroes || window._c8.wasEverNonZero;
      }
    }
    s.prevAllZeroes = allZeroes;
    s.last = pose;
    if (!pose.repeated && pose.imageEvents && pose.imageEvents.length) diag.stats.engineImageEventsIgnored += pose.imageEvents.length;
    const flat = r.world;
    const points = new Array(flat.length / 5);
    for (let i = 0, k = 0; i < points.length; i++, k += 5) {
      points[i] = { id: flat[k], confidence: flat[k + 1], position: { x: flat[k + 2], y: flat[k + 3], z: flat[k + 4] } };
    }
    s.lastWorldPoints = points;
  };

  const drainWorkerResults = (s) => {
    if (!s.ewFrames) return;
    for (const r of s.glue.takeResults()) applyWorkerResult(s, r);
  };

  const processStagedFrameWorker = (s) => {
    s.ewFrames ??= new Map();
    drainWorkerResults(s);
    const f = s.pending;
    if (!f) return;
    const g = s.glue;
    if (g.fatal) throw g.fatal;
    // The Worker is behind: keep the staged frame (the next stageFrame replaces it and carries its IMU events, droppedStaged), like a
    // skipped onProcessCpu. Never reached while the Worker keeps up.
    if (g.inflight >= ew_.maxInFlight) { ew_.stats.backpressureSkips++; return; }
    s.pending = null;
    const t0 = performance.now();
    const size = s.layout.textureSize;
    const dst = g.acquireAtlas(size * Math.max(s.layout.rows, s.layout.roiAtlasRows | 0) * 4);
    const atlas = s.pyramid.read(dst);
    if (!atlas) throw new Error('[XR] slam_refact chunk: no staged pyramid to read');
    const rows = s.pyramid.lastReadRows;
    const t1 = performance.now();
    pushSample(diag.stats.readMs, t1 - t0);
    const motion = [];
    const sink = { pushDeviceMotionEvent: (timestampNanos, event) => motion.push([timestampNanos, ew_.plainMotion(event)]) };
    for (const e of f.events || []) {
      let accepted;
      try { accepted = pushMotion(sink, e); }
      catch (error) { sensors.delivery?.uncertain(e); throw error; }
      if (accepted) {
        diag.stats.motionEventsPushed++;
        sensors.delivery?.pushed(e);
      } else {
        diag.stats.motionEventsSkipped++;
        sensors.delivery?.drop([e], 'invalid-interval');
        warnOnce('[XR] slam_refact chunk: a DeviceMotion event with an interval outside the 64-bit nanosecond range was skipped (the engine rejects such an event with a JS error; the original throws a RangeError from setEventQueue)');
      }
    }
    const allZeroes = levelIsAllZeroes(atlas, size, s.layout.levels[3]);  // before the transfer detaches the atlas
    const engineTime = f.frameTime ? f.frameTime * 1e9 : f.timeNanos;  // the engine time argument of the main-thread mode (CHK-1)
    const videoNs = f.videoTime * 1e9;
    const seq = g.submitFrame({ motion, orientation: [f.timeNanos, Number(f.alpha), Number(f.beta), Number(f.gamma)],
      timeNanos: engineTime, videoNs, atlas, size, rows });
    s.ewFrames.set(seq, { f, allZeroes });
    pushSample(diag.stats.engineMs, performance.now() - t1);
  };

  // The pyramid layout of a capture is needed synchronously by createSession; in Worker mode it is fetched from the Worker first
  // and the session is built when it arrives (a few ms; frames in between get the usual no-session answer). The token cancels a
  // pending build when the session is destroyed, rebuilt or detached meanwhile. Returns true when the build was deferred.
  let ewBuildToken_ = 0;
  const ewDefer = (videoWidth, videoHeight, rotation, run) => {
    if (!ew_ || ew_.hasLayout(videoWidth, videoHeight, rotation) || unsupportedCapture(videoWidth, videoHeight, rotation)) return false;
    const token = ++ewBuildToken_;
    diag.engineWorker.deferredBuilds++;
    pushEntry(diag.sessionLog, { event: 'layout-pending', videoWidth, videoHeight, rotation });
    ew_.requestLayout(videoWidth, videoHeight, rotation).then(() => {
      if (token !== ewBuildToken_) return;
      try { run(); } catch (error) { session_ = null; errorCallback(error instanceof Error ? error : new Error(String(error))); }
    }, (error) => {
      // a dead Worker was already reported by onFatal under the same key; reportOnce keeps it one report
      if (token === ewBuildToken_) reportOnce('process', error);
    });
    return true;
  };

  // c8EmAsm_processStagedFrame: read the staged pyramid, push the IMU staged with it, let the glue stage it and
  // track the previous one.
  const processStagedFrame = (s) => {
    if (ew_) return processStagedFrameWorker(s);
    const f = s.pending;
    if (!f) return;
    s.pending = null;
    const t0 = performance.now();
    let atlas = s.pyramid.read();
    let rows = s.pyramid.lastReadRows;
    const t1 = performance.now();
    pushSample(diag.stats.readMs, t1 - t0);
    for (const e of f.events || []) {
      let accepted;
      try { accepted = pushMotion(s.glue, e); }
      catch (error) { sensors.delivery?.uncertain(e); throw error; }
      if (accepted) {
        diag.stats.motionEventsPushed++;
        sensors.delivery?.pushed(e);
        if (trackerDiagEnabled) addTrackerMotion(s, e);  // A-0e, opt-in: only events the engine took
      } else {
        diag.stats.motionEventsSkipped++;
        sensors.delivery?.drop([e], 'invalid-interval');
        warnOnce('[XR] slam_refact chunk: a DeviceMotion event with an interval outside the 64-bit nanosecond range was skipped (the engine rejects such an event with a JS error; the original throws a RangeError from setEventQueue)');
      }
    }
    s.glue.pushDeviceOrientation(f.timeNanos, Number(f.alpha), Number(f.beta), Number(f.gamma));
    // atlas holds textureSize x rows RGBA pixels: rows = layout.rows, or roiAtlasRows when this frame was drawn with ROIs
    // (Gr8ComputePyramid). If the engine's list changed after this frame was staged it now wants ROI slots this draw did not
    // have (only a processed frame changes the list, so this is a guard, not a normal path); the original's slots of a draw
    // without ROIs were cleared, so the missing rows are zeros and the pyramid rows are kept.
    // A throw here (binding validation) happens before the glue stages the frame, but after this frame's IMU events were pushed:
    // they stay in the engine queue and go on with the next frame (spec 06 section 6.3), the frame itself is not staged.
    // Like stageFrame this asks the engine whatever the JS side has loaded (RM-4d, review RM-4c B-1).
    if (rows < s.layout.roiAtlasRows && s.glue.getRequestedRois().length) {
      const padded = new Uint8Array(s.layout.textureSize * s.layout.roiAtlasRows * 4);
      padded.set(atlas);
      atlas = padded;
      rows = s.layout.roiAtlasRows;
      diag.stats.roiPaddedFrames++;
    }
    // The engine's time argument is the time func 647 picks for TrackingSensorFrame.timeNanos: frameTimestampNanos (= trunc(frameTime * 1e9),
    // tracking-controller.cc:664 stageFrame, camera-loop.ts:324 frameTime = performance.now() / 1000) when it is non-zero, else
    // timestampNanos (the stage time `timeNanos`). Passing the stage time always made every frame's engine time differ from the original's
    // (CHK-1, spec 07 section 12).
    const pose = s.glue.processFramePyramid(f.frameTime ? f.frameTime * 1e9 : f.timeNanos, f.videoTime * 1e9, atlas, s.layout.textureSize, rows);
    const engineReturnedAtMs = performance.now();
    // A-0e, opt-in: the engine has taken the pushed events with this frame, so the summary is taken here (a later throw must not count them again).
    const trackerMotion = trackerDiagEnabled ? s.trackerMotion ?? null : null;
    if (trackerDiagEnabled) s.trackerMotion = null;
    pushSample(diag.stats.engineMs, engineReturnedAtMs - t1);
    diag.stats.processedFrames++;
    // The glue's own stage index of this frame is the key (RM-4a2: a counter kept here drifted for good after one
    // transient exception); bounded, because a glue that never answers fresh would only add entries.
    const stageIndex = pose.videoFrameIndex;
    noteFrameDone(f);  // measurement option only
    if (s.delivery) {
      const d = s.delivery;
      const inputTimeNanos = f.frameTime ? f.frameTime * 1e9 : f.timeNanos;
      d.sourceFrames.set(stageIndex, { sourceVideoTimeS: f.videoTime, sourceFrameTimeS: f.frameTime ?? null,
        sourceTimeNanos: Math.trunc(inputTimeNanos), sourceTimeNanosInput: inputTimeNanos,
        sourceStageTimeNanos: f.timeNanos, sessionEpoch: d.epoch });
      while (d.sourceFrames.size > 64) { d.sourceFrames.delete(d.sourceFrames.keys().next().value); d.evicted++; }
      const source = !pose.repeated || s.hasResponse ? d.sourceFrames.get(pose.sourceFrameIndex) : null;
      d.sourceTiming = { sourceFrameIndex: pose.sourceFrameIndex, sourceVideoTimeS: null, sourceFrameTimeS: null,
        sourceTimeNanos: null, sourceTimeNanosInput: null, sourceStageTimeNanos: null,
        sessionEpoch: d.epoch, ...source, sourceMetadataFound: !!source, repeated: !!pose.repeated, engineReturnedAtMs };
      d.responses++;
      if (publication) { publication.newEngineResponse = true; publication.engineFresh = !pose.repeated; }
    }
    s.textures.set(stageIndex, f.computeTexture);
    while (s.textures.size > 32) s.textures.delete(s.textures.keys().next().value);
    // checkAllZeroes runs on the pyramid of the frame tracked in this call (the previous staged frame).
    const allZeroes = levelIsAllZeroes(atlas, s.layout.textureSize, s.layout.levels[3]);
    if (!pose.repeated) {
      s.hasResponse = true;
      diag.stats.freshResponses++;
      const lag = stageIndex - pose.sourceFrameIndex;  // 1 = the previous staged frame (spec 07 C8-2)
      diag.stats.lastSourceFrameIndex = pose.sourceFrameIndex;
      diag.stats.responseLag[lag] = (diag.stats.responseLag[lag] || 0) + 1;
      s.activeTexture = s.textures.get(pose.sourceFrameIndex) ?? null;
      for (const k of s.textures.keys()) if (k < pose.sourceFrameIndex) s.textures.delete(k);
      if (toCheckAllZeros_ && s.prevAllZeroes !== null) {
        window._c8.isAllZeroes = s.prevAllZeroes ? 1 : 0;
        window._c8.wasEverNonZero = !s.prevAllZeroes || window._c8.wasEverNonZero;
      }
    }
    s.prevAllZeroes = allZeroes;
    s.last = pose;
    if (trackerDiagEnabled) noteTrackerDiag(s, stageIndex, trackerMotion, pose);  // A-0e, opt-in; after the frame's own bookkeeping
    // The engine's own imageEvents (C++ ImageEventTracker) are not dispatched: the JS diff over the built detectedImages is the
    // one event source (image-targets.js header).
    if (!pose.repeated && pose.imageEvents && pose.imageEvents.length) diag.stats.engineImageEventsIgnored += pose.imageEvents.length;
    if (s.roiCheck && s.roiCheck.left > 0 && f.rois && f.rois.length && stageIndex % s.roiCheck.every === 0) {
      diag.roiCheck.push(captureRoiSlots(s, atlas, f.rois, stageIndex));
      s.roiCheck.left--;
    }
    const flat = s.glue.getWorldPoints();
    const points = new Array(flat.length / 5);
    for (let i = 0, k = 0; i < points.length; i++, k += 5) {
      points[i] = { id: flat[k], confidence: flat[k + 1], position: { x: flat[k + 2], y: flat[k + 3], z: flat[k + 4] } };
    }
    s.lastWorldPoints = points;
    if (f.checkRgba) {
      const { SlamRefactGlue } = engine_.Module;
      const size = s.layout.textureSize, rows = s.layout.rows;
      const cpu = SlamRefactGlue.cpuPyramidAtlas(f.checkRgba, f.textureWidth, f.textureHeight, 'swiftshader-approx');
      const ieee = SlamRefactGlue.cpuPyramidAtlas(f.checkRgba, f.textureWidth, f.textureHeight, 'ieee');
      const cmp = (c) => (c.size === size && c.rows >= rows ? compareAtlas(atlas, c.rgba, size, rows) : { identical: false, sizeMismatch: true });
      diag.gpuCheck.push({ stageIndex, size, rows, cpuSize: cpu.size, cpuRows: cpu.rows,
        swiftshaderApprox: cmp(cpu), ieee: cmp(ieee) });
      s.gpuCheck.left--;
    }
  };

  const registerComputeCtx = (computeCtx) => {
    if (computeCtx_ || !computeCtx) return;
    computeCtx_ = computeCtx;
  };

  // A lost WebGL context: every GL object of the session is gone. Report it once, stop calling the engine and answer null
  // (the textures and the pose of the last response would only look alive). A rebuild of the session (size / orientation
  // change) clears it, e.g. after the context came back.
  const checkContext = (s) => {
    if (!s.contextLost && s.pyramid.isContextLost()) {
      s.contextLost = true;
      reportOnce('context', new Error('[XR] slam_refact chunk: the WebGL compute context was lost; world tracking stops until the session is rebuilt (XR8 does not restore the context)'));
    }
    if (s.contextLost) diag.stats.contextLostFrames++;
    return s.contextLost;
  };

  const onProcessGpu = ({ framework, frameStartResult }) => {
    if (schedulerEnabled) return schedulerGpu({ framework, frameStartResult });
    const frameSeq = frameDoneMarks && !frameStartResult.repeatFrame && !cameraProcessingDisabled_ ? markFrameStart() : -1;  // first work of the frame
    const { computeTexture, textureWidth, textureHeight, orientation, repeatFrame, videoTime, frameTime } = frameStartResult;
    if (repeatFrame || cameraProcessingDisabled_) return;
    maybeDispatchConfigChange(framework);
    // A frame whose camera texture is not the size the session was built for arrives before (or without) the
    // onVideoSizeChange / onDeviceOrientationChange that rebuilds the session: it is dropped, its IMU events stay queued.
    if (session_ && (textureWidth !== session_.videoWidth || textureHeight !== session_.videoHeight)) {
      diag.stats.sizeMismatchFrames++;
      warnOnce(`[XR] slam_refact chunk: camera texture ${textureWidth}x${textureHeight} differs from the session ${session_.videoWidth}x${session_.videoHeight}; frames are dropped until onVideoSizeChange / onDeviceOrientationChange rebuilds it`);
      return;
    }
    const { alpha, beta, gamma } = lastDevicePose_;
    const timeNanos = 1e3 * window.performance.now() * 1e3;
    const events = sensors.takeForStage() || [];
    if (!session_ || checkContext(session_)) {
      sensors.delivery?.drop(events, session_ ? 'context-lost' : 'no-session');
      return;
    }
    try {
      stageFrame(session_, { computeTexture, textureWidth, textureHeight, orientation, alpha, beta, gamma,
        videoTime, frameTime, timeNanos, events, ...(frameDoneMarks ? { seq: frameSeq } : {}) });
    } catch (error) {
      // xr.js runs processGpu inside post-render's try block; a throw would skip its frame bookkeeping.
      diag.stats.stageErrors++;
      sensors.delivery?.drop(events, 'stage-error');
      reportOnce('stage', error);
    }
    imageTargets.pump();  // a pending replay of the loaded targets into the rebuilt glue goes on, one time budget per frame
  };

  // The answer of onProcessCpu (tracking-controller.ts:756-790, SDK `YA`) from a response description `d`
  // {position, rotationWxyz, intrinsics, lighting, status, reason, worldPoints, realityTexture, detectedImages (engine
  // objects)}. Every path (fresh engine
  // response, the previous response repeated while a rebuilt pipeline fills, inactive capture) builds its answer here, so
  // the trackingstatus / image diffs and the simulator extrinsic apply to all of them. `d` is never handed out.
  const resultFromDesc = (d) => {
    const [w, x, y, z] = d.rotationWxyz;
    let lighting;
    if (enableLighting_) lighting = d.lighting ? { exposure: d.lighting.exposure, temperature: d.lighting.temperature }
      : { exposure: 0, temperature: 0 };  // capnp defaults of a response computed without the lighting mask
    return {
      rotation: { x, y, z, w }, position: { x: d.position[0], y: d.position[1], z: d.position[2] },
      intrinsics: d.intrinsics.slice(), realityTexture: d.realityTexture, trackingStatus: d.status, trackingReason: d.reason,
      worldPoints: d.reason !== 'INITIALIZING' ? d.worldPoints.map((p) => ({ ...p, position: { ...p.position } })) : [],
      lighting, detectedImages: imageTargets.build(d.detectedImages), points: EMPTY_POINTS, meanRGB: 0,
    };
  };

  const finishResult = (framework, result, timing = null, kind = 'held') => {
    intrinsics_ = result.intrinsics;
    if (engineVpsEventsDisabled_) {
      result.rotation = fakeSimulatorExtrinsic_.rotation;
      result.position = fakeSimulatorExtrinsic_.position;
    }
    // Image targets: result.detectedImages was built from the engine's detections (image-targets.js). This diff
    // (image-events.js) is the only dispatcher of imagefound / imageupdated / imagelost; the engine's own imageEvents are
    // counted and dropped in processStagedFrame, so no change fires twice (RM-4b, spec 07 section 4).
    dispatchImageEvents(detectedImageMap_, result.detectedImages, (name, detail) => schedulerDispatch(framework, name, detail));
    if (result.trackingStatus !== trackingStatus_ || result.trackingReason !== trackingReason_) {
      trackingStatus_ = result.trackingStatus;
      trackingReason_ = result.trackingReason;
      schedulerDispatch(framework, 'trackingstatus', { status: result.trackingStatus, reason: result.trackingReason });
    }
    if (activePublication && !schedulerCurrent(activePublication.session)) throw new Error('[XR] B1prime stale publication');
    // Commit only after every fallible callback above succeeds. A source computed during an
    // earlier failed callback can be published for the first time by a later repeat callback.
    if (publication) {
      const sameSource = !!(timing && lastPublishedSource && timing.sessionEpoch === lastPublishedSource.epoch
        && timing.sourceFrameIndex === lastPublishedSource.index);
      const isFresh = !!(timing?.sourceMetadataFound && !timing.repeated
        && timing.sessionEpoch === session_?.delivery?.epoch && !sameSource);
      Object.assign(publication, timing || {}, { hasPose: true,
        currentSessionEpoch: session_?.delivery?.epoch ?? null, kind, fresh: isFresh, publicationRepeated: sameSource });
      if (timing?.sourceMetadataFound) lastPublishedSource = { epoch: timing.sessionEpoch, index: timing.sourceFrameIndex };
    }
    return result;
  };

  // No session: the capture is not one the engine takes (unsupported rotation) or the rebuild failed. The last pose stays, tracking is LIMITED,
  // and the camera shows the live frame of this callback (cameraTexture is the draw-context texture of the frame; an
  // app that draws reality.realityTexture, e.g. A-Frame, would otherwise get a dark loading texture or a frozen picture).
  // Without any earlier response there is nothing to hold: null, as before the first response of a session.
  // Image targets are not seen while inactive (the engine is not running): detectedImages is empty, so a target that was
  // detected is reported lost (imagelost) and found again after the way back.
  const inactiveAnswer = (framework, frameStartResult) => {
    if (!heldDesc_) return null;
    diag.stats.inactiveAnswers++;
    heldInactive_ = true;
    return finishResult(framework, {
      ...resultFromDesc(heldDesc_), realityTexture: frameStartResult.cameraTexture,
      trackingStatus: 'LIMITED', trackingReason: 'UNSPECIFIED', worldPoints: [], detectedImages: [],
    }, heldDesc_.sourceTiming, 'inactive-held');
  };

  // The compute context is lost right now (RM-4c m-E). A rebuilt session needs working GL, a lost one cannot answer.
  const computeContextLost = (gl = computeCtx_) => !!(gl && typeof gl.isContextLost === 'function' && gl.isContextLost());

  const onProcessCpu = ({ framework, frameStartResult }) => {
    if (deliveryOptions) {
      publication = { hasPose: false, sourceFrameIndex: null, sourceVideoTimeS: null, sourceFrameTimeS: null,
        sourceTimeNanos: null, sourceTimeNanosInput: null, sourceStageTimeNanos: null,
        sessionEpoch: null, currentSessionEpoch: session_?.delivery?.epoch ?? null,
        repeated: null, fresh: false, publicationRepeated: false, newEngineResponse: false, engineFresh: false,
        sourceMetadataFound: false, kind: 'no-result',
        callbackVideoTimeS: frameStartResult.videoTime ?? null, callbackFrameTimeS: frameStartResult.frameTime ?? null };
      responseEntries[responseNext] = publication;
      responseNext = (responseNext + 1) % deliveryOptions.maxEntries;
      responseTotal++;
    }
    const { repeatFrame } = frameStartResult;
    if (cameraProcessingDisabled_) {
      fakeRealityTexture_ = fakeRealityTexture_ || initFakeRealityTexture(drawCtx_);
      return { realityTexture: fakeRealityTexture_ };
    }
    if (runConfig.paused) return null;
    if (schedulerEnabled) {
      const before = session_;
      try {
        maybeDispatchConfigChange(framework);
        if (before && before !== session_) return null;
        if (!session_ && deferredRebuild_ && !computeContextLost(deferredRebuild_.computeCtx || computeCtx_)) {
          const d = deferredRebuild_; deferredRebuild_ = null; buildSession(d.kind, d);
        }
        return schedulerCpu(framework, frameStartResult);
      } catch (error) {
        if (before?.scheduler) schedulerFailure(before, 'scheduler-cpu-config', error);
        else reportSchedulerError('scheduler-cpu-config', error);
        return null;
      }
    }
    maybeDispatchConfigChange(framework);
    // A rebuild that waited for the context to come back (reinitSession) is made now.
    if (!session_ && deferredRebuild_ && !computeContextLost(deferredRebuild_.computeCtx || computeCtx_)) {
      const d = deferredRebuild_;
      deferredRebuild_ = null;
      pushEntry(diag.sessionLog, { event: 'resumed', kind: d.kind, reason: 'context back' });
      buildSession(d.kind, d);
    }
    // No session and a lost context (a size change arrived before the context came back): the one report was already made
    // when the session saw the loss; the answer is the same null as with a session, not a frozen LIMITED pose.
    if (!session_) return computeContextLost(deferredRebuild_ && deferredRebuild_.computeCtx || computeCtx_) ? null : inactiveAnswer(framework, frameStartResult);
    if (publication) publication.currentSessionEpoch = session_.delivery?.epoch ?? null;
    if (checkContext(session_)) return null;
    const responsesBefore = session_.delivery?.responses;
    if (!repeatFrame) {
      // xr.js runs processCpu inside pre-render's try block; a throw would skip onUpdate and the camera texture
      // upload of this and every later frame. Report the failure and answer "no response" instead.
      const pendingEvents = session_.pending?.events || [];
      sensors.delivery?.process(pendingEvents);
      try {
        processStagedFrame(session_);
      } catch (error) {
        diag.stats.processErrors++;
        sensors.delivery?.drop(pendingEvents, 'process-error');
        reportOnce('process', error);
        return null;
      }
    } else if (ew_ && session_.ewFrames) {
      // Worker mode: a repeated callback has no new frame to submit but still takes the results that arrived since.
      try { drainWorkerResults(session_); } catch (error) { diag.stats.processErrors++; reportOnce('process', error); return null; }
    }
    const s = session_;
    if (!s.hasResponse || !s.activeTexture) {
      // The new pipeline has not produced a response yet: the original answers from window._c8.reality, which survives
      // engineInit, i.e. with the previous response (stale pose and texture, consistent with each other).
      const d = s.lastDesc || heldDesc_;
      if (!d) return null;
      // After an inactive episode heldDesc_ is the pose from before the rotation, with a texture of the old size: the gap is
      // answered like the inactive frames (RM-4c m-B); NORMAL comes with the first fresh response of the new session.
      if (!s.lastDesc && heldInactive_) return inactiveAnswer(framework, frameStartResult);
      diag.stats.gapAnswers++;
      return finishResult(framework, resultFromDesc(d), d.sourceTiming, 'held-gap');
    }
    const pose = s.last;
    s.lastDesc = {
      position: Array.from(pose.position), rotationWxyz: Array.from(pose.rotationWxyz), intrinsics: Array.from(pose.intrinsics),
      lighting: pose.lighting ? { exposure: pose.lighting.exposure, temperature: pose.lighting.temperature } : null,
      status: pose.status, reason: pose.reason, worldPoints: s.lastWorldPoints,
      detectedImages: pose.detectedImages || [],
      realityTexture: window.XR8.drawTexForComputeTex(s.activeTexture),
      sourceTiming: s.delivery?.sourceTiming,
    };
    const newResponse = s.delivery && s.delivery.responses !== responsesBefore;
    return finishResult(framework, resultFromDesc(s.lastDesc), s.lastDesc.sourceTiming,
      newResponse ? 'engine-response' : 'callback-repeat');
  };

  const onStart = ({ computeCtx, canvasWidth, canvasHeight }) => {
    maybeDispatchConfigChange(framework_);
    registerComputeCtx(computeCtx);
    if (!cam_.pixelRectWidth) cam_.pixelRectWidth = canvasWidth;
    if (!cam_.pixelRectHeight) cam_.pixelRectHeight = canvasHeight;
  };

  const onSessionAttach = ({ videoWidth, videoHeight, canvasWidth, canvasHeight, rotation, computeCtx, framework,
    GLctx, sessionAttributes }) => {
    registerComputeCtx(computeCtx);
    if (toCheckAllZeros_ === null) toCheckAllZeros_ = window._c8.checkAllZeroes;
    cameraProcessingDisabled_ = !sessionAttributes.fillsCameraTexture;
    framework_ = framework;
    notifyOthersOfConfigChange_ = true;
    maybeDispatchConfigChange(framework_);
    sessionStarted_ = true;
    if (!cam_.pixelRectWidth) cam_.pixelRectWidth = canvasWidth;
    if (!cam_.pixelRectHeight) cam_.pixelRectHeight = canvasHeight;
    drawCtx_ = GLctx;
    cameraSessionAttached_ = !cameraProcessingDisabled_;
    if (!cameraProcessingDisabled_) {
      // engineInit: the startup recenter count is 5 on the first run and after a detach (engineCleanup, onSessionDetach); an
      // attach that follows an attach without a detach continues the count like any engineInit again.
      destroySession('session-attach');
      heldInactive_ = false;
      deferredRebuild_ = null;
      if (schedulerEnabled && !(GLctx && typeof GLctx.copyTexSubImage2D === 'function'
        && typeof GLctx.fenceSync === 'function' && typeof GLctx.texStorage2D === 'function')) {
        schedulerFallbackReason = 'WebGL1 draw context has no WebGL2 owned-snapshot copy/fence support';
        drainRetiredSnapshotPools(true);
        schedulerEnabled = false;
        heldDesc_ = null;  // A new legacy session cannot publish the retired ON texture.
        warnOnce(`[XR] B1prime OFF fallback: ${schedulerFallbackReason}`);
      }
      if (schedulerRequested) {
        try {
          session_ = createSession({ videoWidth, videoHeight, rotation, computeCtx: computeCtx_, coords: configured_, framesUntilRecenter: startupFramesLeft_ });
        } catch (error) {
          // createSession already released its glue/pyramid. Leave no session (inactive answers, no throw out of the
          // XR8 attach callback), record the failure, and finish the attach below (simulator listeners, flags).
          // A later size/orientation change, or detach + attach, builds a session again.
          session_ = null;
          schedulerCounts.errors++;
          pushEntry(schedulerFailures, { where: 'scheduler-attach', message: String(error?.message || error),
            epoch: null, nativeAdvanced: false, partialDispatchCount: 0 });
          pushEntry(diag.sessionLog, { event: 'scheduler-attach-failed' });
          reportSchedulerError('scheduler-attach', error);
        }
      } else if (!ewDefer(videoWidth, videoHeight, rotation, () => {
        session_ = createSession({ videoWidth, videoHeight, rotation, computeCtx: computeCtx_, coords: configured_, framesUntilRecenter: startupFramesLeft_ });
      })) session_ = createSession({ videoWidth, videoHeight, rotation, computeCtx: computeCtx_, coords: configured_, framesUntilRecenter: startupFramesLeft_ });
      inactiveWarned_ = false;
    }
    isSimulator_ = sessionAttributes.isSimulator === true;
    if (isSimulator_) {
      removeWindowListener('deviceorientation', updateDeviceOrientation);
      removeWindowListener('devicemotion', sensors.updateDeviceMotion);
    }
  };

  const onSessionDetach = () => {
    cameraProcessingDisabled_ = false;
    cameraSessionAttached_ = false;
    destroySession('stop');
    if (schedulerEnabled) drainRetiredSnapshotPools(true);
    startupFramesLeft_ = null;  // engineCleanup: the next engineInit starts with a fresh count
    heldInactive_ = false;
    deferredRebuild_ = null;
    lastExtrinsic_ = null;
    heldDesc_ = null;
    inactiveWarned_ = false;
    configured_ = { origin: { x: 0, y: 2, z: 0 }, facing: { w: 1, x: 0, y: 0, z: 0 }, scale: 1 };
    computeCtx_ = null;
    addMotionOrientationListeners();
    lastDevicePose_.alpha = 0;
    lastDevicePose_.beta = 0;
    lastDevicePose_.gamma = 0;
    lastGeolocation_.latitude = 0;
    lastGeolocation_.longitude = 0;
    lastGeolocation_.accuracy = 0;
    if (fakeRealityTexture_) {
      drawCtx_.deleteTexture(fakeRealityTexture_);
      fakeRealityTexture_ = null;
    }
    drawCtx_ = null;
    scale_ = 1;
    origin_.x = 0; origin_.y = 2; origin_.z = 0;
    facing_.w = 1; facing_.x = 0; facing_.y = 0; facing_.z = 0;
    cam_.pixelRectWidth = 0;
    cam_.pixelRectHeight = 0;
    cam_.nearClipPlane = 0.01;
    cam_.farClipPlane = 1000;
    trackingStatus_ = 'UNSPECIFIED';
    trackingReason_ = 'UNSPECIFIED';
    Object.keys(detectedImageMap_).forEach((k) => delete detectedImageMap_[k]);
    Object.keys(imageTargetCache_).forEach((k) => delete imageTargetCache_[k]);
    imageTargets.onSessionDetach();
    sessionStarted_ = false;
    appResourcesLoaded_ = false;
    framework_ = null;
    window._c8.isAllZeroes = null;
    window._c8.wasEverNonZero = null;
  };

  const onDetach = () => {
    if (schedulerEnabled) drainRetiredSnapshotPools(true);
    absoluteScale_ = false;
    enableLighting_ = false;
    enableWorldPoints_ = false;
    disableWorldTracking_ = false;
    leftHandedAxes_ = false;
    mirroredDisplay_ = false;
    projectWayspots_ = [];
    imageTargets.onDetach();
  };

  const onAppResourcesLoaded = ({ imageTargets: appImageTargets, framework }) => {
    if (appResourcesLoaded_) return;
    maybeDispatchConfigChange(framework);
    appResourcesLoaded_ = true;
    framework_ = framework;
    framework.dispatchEvent('locationscanning', { locations: [] });
    framework.dispatchEvent('projectwayspotscanning', { wayspots: [] });
    if (appImageTargets) appImageTargets.forEach((t) => { imageTargetCache_[t.name] = t; });
    imageTargets.onAppResourcesLoaded(framework);
  };

  // The glue has one coordinate system (coords_) where the original has two: the configured one (what rebuildEngine applies)
  // and the live one of the engine. After a video size change the live origin is the last pose (resetOriginToLastPose, below)
  // and `liveStale` marks that the glue's live system differs from the JS state (P, l); the original's next configureXr sends P
  // again, so every such call re-sends it first.
  const resendJsCoords = (s) => {
    if (!s.liveStale) return;
    s.glue.updateCameraProjectionMatrix({ origin: { ...origin_ }, facing: { ...facing_ }, updateRecenterPoint: false });
    s.liveStale = false;
  };

  // configureXr (SDK `fA`): what the original sends to c8EmAsm_configureXr is forwarded to the glue mirror. The coordinate
  // system (origin, facing, scale) is sent when all three are truthy and becomes the configured one.
  const configureXr = ({ cam, origin, facing, scale }) => {
    notifyOthersOfConfigChange_ = true;
    const sendsCoords = !!(origin && facing && scale);
    if (sendsCoords) configured_ = { origin: { ...origin }, facing: { ...facing }, scale };
    if (!session_) return;
    if (sendsCoords) resendJsCoords(session_);
    if (cam) glueConfigure(session_.glue);
  };

  // XrController.configure (SDK `nA`, spec 06 §3.1): same order; a throw leaves the earlier changes.
  const configureImpl = (args) => {
    if (!args) return;
    // chunk-only refusal (more than 32 targets) before any state changes (RM-4d, review m1)
    if (args.imageTargetData !== undefined) imageTargets.precheck(args.imageTargetData, { cameraProcessingDisabled: cameraProcessingDisabled_ });
    if (args.enableFeatureSet !== undefined) enableWorldPoints_ = !!args.enableFeatureSet;
    if (args.enableWorldPoints !== undefined) enableWorldPoints_ = !!args.enableWorldPoints;
    if (args.scale !== undefined) {
      if (sessionStarted_) throw new Error('[XR] Scale can only be changed before calling XR8.run()');
      absoluteScale_ = args.scale === 'absolute';
    }
    if (args.enableLighting !== undefined) enableLighting_ = !!args.enableLighting;
    if (args.leftHandedAxes !== undefined) leftHandedAxes_ = !!args.leftHandedAxes;
    if (args.mirroredDisplay !== undefined) {
      notifyOthersOfConfigChange_ = notifyOthersOfConfigChange_ || mirroredDisplay_ !== !!args.mirroredDisplay;
      mirroredDisplay_ = !!args.mirroredDisplay;
    }
    if (args.enableVps !== undefined) {
      if (sessionStarted_) throw new Error('[XR] VPS can only be changed before calling XR8.run()');
      if (args.enableVps) console.error('[XR] VPS is not supported in standalone mode.');
    }
    if (args.disableWorldTracking !== undefined) {
      if (sessionStarted_) throw new Error('[XR] World tracking can only be changed before calling XR8.run()');
      disableWorldTracking_ = !!args.disableWorldTracking;
    }
    if (absoluteScale_ && disableWorldTracking_) {
      throw new Error('[XR] Unable to perform metric scale estimation if world tracking is disabled.');
    }
    if (args.projectWayspots !== undefined) {
      if (sessionStarted_) throw new Error('[XR] Project wayspots can only be changed before calling XR8.run()');
      if (!Array.isArray(args.projectWayspots)) throw new Error('[XR] Expected array for arg: projectWayspots');
      args.projectWayspots.forEach((w) => {
        if (typeof w !== 'string') throw new Error(`[XR] Project Wayspot name should be type 'string' but '${w}' is type '${typeof w}'`);
      });
      projectWayspots_ = args.projectWayspots;
    }
    if (args.imageTargetData !== undefined) {
      imageTargets.configure(args.imageTargetData, { cameraProcessingDisabled: cameraProcessingDisabled_, appResourcesLoaded: appResourcesLoaded_, framework: framework_ });
    } else if (args.imageTargets !== undefined) {
      console.warn('[XR] imageTargets is deprecated, please use imageTargetData instead.');
    }
    if (args.enableDataRecorder !== undefined) dataRecorderRequested_ = !!args.enableDataRecorder;
    // mapSrcUrl: the original stores whatever it is given and sends it to the engine only when it is truthy (tracking-controller.ts:1275-1279,
    // deployed `mapSrcUrl:x`), and A-Frame's xrweb passes its schema default "" on every configure (aframe/common.ts:46, xr-web-component.ts:136).
    // An empty string asks for no map, so it is not reported (RM-4b); every other listed key is reported as soon as it is given.
    const unsupported = UNSUPPORTED_KEYS.filter((k) => (k === 'mapSrcUrl' ? !!args[k] : args[k] !== undefined));
    if (unsupported.length) warnOnce(`[XR] slam_refact chunk: configure options not supported by this engine: ${unsupported.join(', ')}`);
    configureXr({ cam: cam_, origin: origin_, facing: facing_, scale: scale_ });
  };

  // XrController.updateCameraProjectionMatrix (SDK `xA`).
  const updateCameraProjectionMatrixImpl = ({ cam, origin, facing, updateRecenterPoint = true }) => {
    notifyOthersOfConfigChange_ = true;
    if (cam) Object.assign(cam_, cam);
    if (origin) {
      origin_ = { x: origin.x, y: origin.y, z: origin.z };
      if (updateRecenterPoint) recenterOrigin_ = { ...origin_ };
    }
    if (facing) {
      facing_ = { w: facing.w, x: facing.x, y: facing.y, z: facing.z };
      if (updateRecenterPoint) recenterFacing_ = { ...facing_ };
    }
    if (absoluteScale_) {
      origin_.y = 1;
      scale_ = 1;
    } else {
      scale_ = origin_.y;
    }
    if (session_) {
      resendJsCoords(session_);
      const u = { updateRecenterPoint };
      if (cam) u.cam = cam;
      if (origin) u.origin = origin;
      if (facing) u.facing = facing;
      session_.glue.updateCameraProjectionMatrix(u);
    }
    configureXr({ cam: cam_, origin: origin_, facing: facing_, scale: scale_ });
  };

  // Rotation / video size changes re-run engineInit in the original (tracking-controller.ts:1410-1431:
  // onDeviceOrientationChange -> maybeDispatchConfigChange, c8EmAsm_engineOrientationChange, configureXr;
  // onVideoSizeChange -> maybeDispatchConfigChange, c8EmAsm_engineVideoSizeChange;
  // tracking-controller.cc:679-692: both = c8EmAsm_engineInit(w, h, rotation) + resetOriginToLastPose()).
  // Here the session is destroyed and rebuilt (new SlamRefactGlue = engineInit). A capture the engine cannot take (an unsupported
  // rotation value) leaves the session inactive: one warn per episode, no errorCallback (header note, spec 07 section 3);
  // nothing here throws. Differences, spec 07 section 3: the new glue repeats the startup recenter (5 frames at the
  // configured origin and one more core rebuild; the original's framesUntilRecenter is not re-armed by engineInit);
  // the origin reset is skipped with scale: 'absolute' (the glue mirror cannot express scale != 1 there).
  //
  // resetOriginToLastPose (cc:413-436) sets the LIVE engine coordinate system to the last response's pose and leaves the
  // configured one (configuredCoordinateSystem, `configured_` here) and the JS state (P, l) alone; recenter() and the next
  // configureXr bring the live system back (see resendJsCoords and recenter).
  const resetOriginToLastPose = (s) => {
    const e = lastExtrinsic_;
    if (!e || absoluteScale_) return;
    const [px, py, pz] = e.position;
    const [w, x, y, z] = e.rotationWxyz;
    // y > 1e-3: origin = pose, scale = pose.y (= origin.y in the mirror); else the configured origin height stays.
    s.glue.updateCameraProjectionMatrix({ origin: { x: px, y: py > 1e-3 ? py : configured_.origin.y, z: pz }, facing: { w, x, y, z }, updateRecenterPoint: false });
    diag.stats.originResets++;
  };

  const warnInactive = () => {
    if (inactiveWarned_) return;
    inactiveWarned_ = true;
    const message = '[XR] slam_refact chunk supports rotation 0, 90, 180, 270 (and the negatives) only: world tracking is inactive '
      + '(last pose held, tracking status LIMITED, camera feed kept) until the capture has a supported rotation again';
    pushEntry(diag.warnings, message);
    console.warn(message);
  };

  // Builds the session for a size / orientation change (or a postponed one) and brings the live coordinate system to where the
  // original's engineInit would have it. A failed build is reported (errorCallback: a real failure) and leaves no session.
  const buildSession = (kind, args) => {
    const { videoWidth, videoHeight, orientation, computeCtx } = args;
    if (ewDefer(videoWidth, videoHeight, orientation, () => buildSession(kind, args))) return;
    try {
      // engineInit -> rebuildEngine applies the configured coordinate system; an orientation change then sends P (configureXr).
      // The startup recenter count continues where the destroyed glue was (engineInit does not arm it again).
      session_ = createSession({ videoWidth, videoHeight, rotation: orientation, computeCtx: computeCtx || computeCtx_,
        coords: kind === 'videoSize' ? configured_ : { origin: origin_, facing: facing_ }, framesUntilRecenter: startupFramesLeft_ });
    } catch (error) {
      session_ = null;
      errorCallback(error instanceof Error ? error : new Error(String(error)));
      return;
    }
    inactiveWarned_ = false;
    if (kind === 'videoSize') {
      resetOriginToLastPose(session_);
      session_.liveStale = true;  // live system = configured / last pose, JS state may differ (e.g. after recenter())
    } else {
      configureXr({ cam: cam_, origin: origin_, facing: facing_, scale: scale_ });
    }
  };

  const reinitSession = (kind, args) => {
    const { videoWidth, videoHeight, orientation, computeCtx } = args;
    if (schedulerEnabled && session_?.scheduler.nativeCallDepth) {
      const error = new Error('[XR] B1prime lifecycle mutator busy'); session_.scheduler.reentrantError = error; throw error;
    }
    maybeDispatchConfigChange(framework_);
    if (!cameraSessionAttached_) return;
    if (schedulerEnabled && session_) schedulerCounts.lifecycleResets++;
    destroySession(kind);
    deferredRebuild_ = null;
    const unsupported = unsupportedCapture(videoWidth, videoHeight, orientation);
    if (unsupported) {
      diag.stats.sessionsRejected++;
      pushEntry(diag.sessionLog, { event: 'rejected', kind, videoWidth, videoHeight, rotation: orientation });
      warnInactive();
      return;
    }
    // The compute context is still lost: a session cannot be built (its GL init would fail with a second report). The build is
    // postponed (deferredRebuild_) and made by onProcessCpu when the context is back (RM-4c m-E, RM-4d m3).
    if (computeContextLost(computeCtx || computeCtx_)) {
      deferredRebuild_ = { kind, videoWidth, videoHeight, orientation, computeCtx };
      pushEntry(diag.sessionLog, { event: 'deferred', kind, reason: 'context lost' });
      return;
    }
    buildSession(kind, args);
  };

  const onDeviceOrientationChange = (args) => reinitSession('orientation', args);
  const onVideoSizeChange = (args) => reinitSession('videoSize', args);

  const onCanvasSizeChangeImpl = ({ canvasWidth, canvasHeight }) => {
    maybeDispatchConfigChange(framework_);
    cam_.pixelRectWidth = canvasWidth;
    cam_.pixelRectHeight = canvasHeight;
    if (session_) session_.glue.updateCameraProjectionMatrix({ cam: { pixelRectWidth: canvasWidth, pixelRectHeight: canvasHeight }, updateRecenterPoint: false });
    configureXr({ cam: cam_ });
  };

  const onCameraStatusChange = ({ status, cameraConfig }) => {
    sessionStarted_ = true;
    if (status !== 'requesting') return;
    maybeDispatchConfigChange(framework_);
    const { direction } = cameraConfig;
    if (direction !== window.XR8.XrConfig.camera().BACK && !disableWorldTracking_) {
      throw Object.assign(new Error('[XR] World tracking is only supported on the back camera'), {
        type: 'configuration', source: 'reality', err: 'slam-front-camera-unsupported',
      });
    }
  };

  const requiredPermissions = () => {
    const permissions = window.XR8.XrPermissions.permissions();
    const pl = [permissions.CAMERA];
    if (!disableWorldTracking_ && !cameraProcessingDisabled_) {
      pl.push(permissions.DEVICE_ORIENTATION);
      pl.push(permissions.DEVICE_MOTION);
    }
    return pl;
  };

  const onBeforeSessionInitialize = ({ sessionAttributes }) => {
    cameraProcessingDisabled_ = !sessionAttributes.fillsCameraTexture;
    if (!sessionAttributes.fillsCameraTexture && imageTargets.hasConfigured()) {
      throw new Error('[XR] Image Targets require a session that can provide a camera texture');
    }
    if (sessionAttributes.controlsCamera && disableWorldTracking_) {
      throw new Error('[XR] Disabled world tracking requires a session without camera control');
    }
    if (!disableWorldTracking_) {
      const mobileOnlyRunConfig = { ...runConfig, allowedDevices: window.XR8.XrConfig.device().MOBILE };
      if (sessionAttributes.fillsCameraTexture && !window.XR8.XrDevice.isDeviceBrowserCompatible(mobileOnlyRunConfig)) {
        throw new Error('[XR] Reality with camera on non-mobile devices requires disableWorldTracking');
      }
    }
  };

  const pipelineModule = () => ({
    name: 'reality',
    onAppResourcesLoaded,
    onBeforeSessionInitialize,
    onStart,
    onSessionAttach,
    onSessionDetach,
    onDetach,
    onProcessGpu,
    onProcessCpu,
    onDeviceOrientationChange,
    onCanvasSizeChange,
    onVideoSizeChange,
    onCameraStatusChange,
    requiredPermissions,
  });

  // XrController.recenter (SDK `XA`): P = p, l = f, rebuild; no configureXr.
  const recenterImpl = () => {
    origin_ = { ...recenterOrigin_ };
    facing_ = { ...recenterFacing_ };
    // No session (inactive capture, failed rebuild): the original keeps tracking and rebuildEngine restarts at the configured
    // coordinate system, so the pose of the last response must not be used to reset the origin of the next session (RM-4c m-A).
    if (!session_) lastExtrinsic_ = null;
    if (session_) {
      const s = session_;
      if (s.liveStale) {
        // rebuildEngine applies the configured coordinate system, not the origin resetOriginToLastPose moved the live one to.
        s.glue.updateCameraProjectionMatrix({ origin: { ...configured_.origin }, facing: { ...configured_.facing }, updateRecenterPoint: false });
        s.liveStale = false;
      }
      s.glue.recenter();
    }
    maybeDispatchConfigChange(framework_, true);
  };

  const configure = (args) => {
    if (!schedulerEnabled) return configureImpl(args);
    if (args?.imageTargetData !== undefined) imageTargets.precheck(args.imageTargetData, { cameraProcessingDisabled: cameraProcessingDisabled_ });
    // Predict only the JS prefix that the existing validation order can actually reach.
    // In an active session scale/VPS/world-tracking/wayspot validation throws before later keys.
    let nextWorld = enableWorldPoints_;
    if (args?.enableFeatureSet !== undefined) nextWorld = !!args.enableFeatureSet;
    if (args?.enableWorldPoints !== undefined) nextWorld = !!args.enableWorldPoints;
    let effective = nextWorld !== enableWorldPoints_;
    if (!(sessionStarted_ && args?.scale !== undefined)) {
      const scalars = { enableLighting: enableLighting_, leftHandedAxes: leftHandedAxes_, mirroredDisplay: mirroredDisplay_ };
      effective ||= Object.keys(scalars).some((k) => args?.[k] !== undefined && !!args[k] !== scalars[k]);
      if (!(sessionStarted_ && (args?.enableVps !== undefined || args?.disableWorldTracking !== undefined || args?.projectWayspots !== undefined))) {
        // configureXr is unreachable when an earlier runtime-only validation throws.
        effective ||= !!(args && session_?.liveStale && scale_);
        effective ||= args?.imageTargetData !== undefined || (args?.enableDataRecorder !== undefined && !!args.enableDataRecorder !== dataRecorderRequested_);
      }
    }
    return schedulerCommand('configure', effective, () => configureImpl(args));
  };
  const updateCameraProjectionMatrix = (args) => {
    if (!schedulerEnabled) return updateCameraProjectionMatrixImpl(args);
    const effective = Object.entries(args.cam || {}).some(([k, v]) => cam_[k] !== v)
      || Object.entries(args.origin || {}).some(([k, v]) => origin_[k] !== v || (args.updateRecenterPoint !== false && recenterOrigin_[k] !== v))
      || Object.entries(args.facing || {}).some(([k, v]) => facing_[k] !== v || (args.updateRecenterPoint !== false && recenterFacing_[k] !== v));
    return schedulerCommand('projection', effective || !!session_?.liveStale, () => updateCameraProjectionMatrixImpl(args));
  };
  const recenter = () => schedulerCommand('recenter', true, recenterImpl);
  const onCanvasSizeChange = (args) => {
    if (!schedulerEnabled) return onCanvasSizeChangeImpl(args);
    return schedulerCommand('canvas-size', args.canvasWidth !== cam_.pixelRectWidth
      || args.canvasHeight !== cam_.pixelRectHeight, () => onCanvasSizeChangeImpl(args));
  };

  const hitTestTypeEnum = {
    UNSPECIFIED: 'UNSPECIFIED', FEATURE_POINT: 'FEATURE_POINT', ESTIMATED_SURFACE: 'ESTIMATED_SURFACE', DETECTED_SURFACE: 'DETECTED_SURFACE',
  };
  // XrController.hitTest (SDK `zA` -> c8EmAsm_query): unknown type names stay UNSPECIFIED.
  const hitTest = (x, y, includedTypes = []) => {
    if (schedulerEnabled) schedulerCounts.hitTests++;
    const miss = () => { if (schedulerEnabled) schedulerCounts.hitMisses++; return []; };
    if (!engine_) return miss();
    if (!session_) return miss();
    if (schedulerEnabled && (computeContextLost() || drawCtx_?.isContextLost?.())) {
      schedulerFailure(session_, 'scheduler-query-context', new Error('[XR] B1prime context lost before query')); return miss();
    }
    if (schedulerEnabled && schedulerBusyReason()) { schedulerCounts.queryRejected++; return miss(); }
    if (!schedulerEnabled) return session_.glue.xrHitTest(x, y, includedTypes && includedTypes.length ? includedTypes : []);
    const s = session_;
    try {
      const hits = s.glue.xrHitTest(x, y, includedTypes && includedTypes.length ? includedTypes : []);
      if (!hits?.length) schedulerCounts.hitMisses++;
      return hits;
    } catch (error) { schedulerFailure(s, 'scheduler-query', error); return miss(); }
  };
  hitTest.type = hitTestTypeEnum;

  const dispatchToFramework = (name, detail) => { framework_.dispatchEvent(name, detail); };
  // FX-E3: a message whose data is not an object is ignored (event.data can be null or a string). Opt-in, default OFF:
  // SLAM_REFACT_CHUNK_OPTIONS.trustedMessageOrigins (array of origin strings) drops messages from other origins and counts them in
  // diag.trustedMessageOrigins.dropped. Unset = every origin is accepted, as in the original simulator hook.
  if (trustedMessageOrigins) diag.trustedMessageOrigins = { origins: [...trustedMessageOrigins], dropped: 0 };
  addWindowListener('message', (event) => {
    if (event.data === null || typeof event.data !== 'object') return;
    if (trustedMessageOrigins && !trustedMessageOrigins.includes(event.origin)) { diag.trustedMessageOrigins.dropped++; return; }
    if (event.data.deviceOrientation8w) window.dispatchEvent(new CustomEvent('deviceorientation8w', { detail: event.data.deviceOrientation8w }));
    if (event.data.deviceMotion8w) window.dispatchEvent(new CustomEvent('devicemotion8w', { detail: event.data.deviceMotion8w }));
    if (event.data.gps8w) window.dispatchEvent(new CustomEvent('gps8w', { detail: event.data.gps8w }));
  }, false);
  addWindowListener('deviceorientation8w', ({ detail }) => updateDeviceOrientation(detail));
  addWindowListener('devicemotion8w', ({ detail }) => sensors.updateDeviceMotion(detail));
  addWindowListener('extrinsic8w', ({ detail }) => { fakeSimulatorExtrinsic_ = { position: detail.position, rotation: detail.rotation }; });
  addWindowListener('gps8w', ({ detail }) => updateDeviceGeolocation({ coords: detail }));
  addWindowListener('locationscanning8w', () => dispatchToFramework('locationscanning', { locations: [] }));
  addWindowListener('locationfound8w', ({ detail }) => dispatchToFramework('locationfound', detail));
  addWindowListener('locationlost8w', ({ detail }) => dispatchToFramework('locationlost', detail));
  addWindowListener('meshfound8w', ({ detail }) => dispatchToFramework('meshfound', detail));
  addWindowListener('meshlost8w', ({ detail }) => dispatchToFramework('meshlost', detail));
  addMotionOrientationListeners();

  const updateEnableEngineVpsEvents = (enable) => { engineVpsEventsDisabled_ = !enable; };
  const getEnableEngineVpsEvents = () => !engineVpsEventsDisabled_;
  const getIntrinsic = () => intrinsics_;

  // Data recorder (original KA / FA / sA, tracking-controller.ts:323-351). xr.js never calls these, apps may. The
  // recorder itself needs the capnp LogRecord format (not implemented, spec 07 section 3), so what is reproduced
  // is the original behaviour of a recorder that was not enabled; with enableDataRecorder requested the
  // chunk says that it cannot record instead of claiming a recording.
  const startDatarecorder = () => {
    if (dataRecorderRequested_) {
      warnOnce('[XR] slam_refact chunk: the data recorder is not supported by this engine; no recording was started.');
      return;
    }
    console.warn('[XR] Data Recorder not enabled. Enable it first before starting.');
  };
  const stopDatarecorder = () => undefined;
  const getIsDataRecording = () => false;

  const ready = identityPromise ? Promise.all([enginePromise, identityPromise]) : enginePromise;
  return ready.then(() => ({
    pipelineModule,
    configure,
    recenter,
    hitTest,
    updateCameraProjectionMatrix,
    updateEnableEngineVpsEvents,
    getEnableEngineVpsEvents,
    getIntrinsic,
    getIsDataRecording,
    startDatarecorder,
    stopDatarecorder,
    xrController: () => ({
      cameraPipelineModule: pipelineModule,
      configure,
      recenter,
      updateCameraProjectionMatrix,
      updateCameraOrigin: updateCameraProjectionMatrix,
    }),
  }));
});

export { ResourceUrls, XrControllerFactory };
