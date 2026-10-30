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
import { Gr8ComputePyramid } from './gr8-compute-pyramid.js';
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

const XrControllerFactory = singleton((runConfig, xrccPromise, logCallback, errorCallback) => {
  const options = (typeof window !== 'undefined' && window.SLAM_REFACT_CHUNK_OPTIONS) || {};
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

  // xrccPromise is not used: the original only needs XRCC to share its Emscripten GL texture table
  // (scc.GL.textures = xrcc.GL.textures); this chunk uses XR8's computeTexture objects directly.
  let engine_ = null;  // {Module, loaded}; the original's `scc_`
  const enginePromise = loadEngine(new URL(ENGINE_DIR, import.meta.url),
    { print: logCallback(false), printErr: logCallback(true), onAbort: errorCallback })
    .then((r) => {
      engine_ = r;
      // A test hook that replaces the engine glue must not leave the evidence "engine = manifest bytes" looking untouched.
      r.loaded.glueReplaced = !!options.createGlue;
      diag.engine = r.loaded;
      return r;
    },
      (error) => { diag.error = String(error && error.message || error); diag.engineLoad = error.loaded ?? null; throw error; });

  const sensors = createSensors(() => !engine_ || !window.XR8 || window.XR8.isPaused());

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
    options, diag, pushEntry, getComputeCtx: () => computeCtx_, getGlue: () => (session_ ? session_.glue : null),
  });

  const maybeDispatchConfigChange = (framework, recenter) => {
    if ((!notifyOthersOfConfigChange_ && !recenter) || !framework) return;
    framework.dispatchEvent('cameraconfigured', {
      mirroredDisplay: mirroredDisplay_, origin: { position: origin_, rotation: facing_ }, recenter: !!recenter,
    });
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
    const device = window.XR8.XrDevice.deviceEstimate();
    const glueConfig = {
      captureWidth: videoWidth, captureHeight: videoHeight, rotationDeg: rotation,
      manufacturer: device.manufacturer || '', model: device.model || '',
      estimateScale: absoluteScale_, disableVio: disableWorldTracking_,
      leftHandedAxes: leftHandedAxes_, mirroredDisplay: mirroredDisplay_,
    };
    if (framesUntilRecenter !== null) glueConfig.framesUntilRecenter = framesUntilRecenter;
    // options.createGlue is a test hook (chunk-contract.test.mjs wraps or replaces the engine glue). Whether a replaced glue was
    // used is decided here, for every session, not once at engine load (RM-4c m-F): once true it stays true.
    const createGlue = options.createGlue;
    if (createGlue) engine_.loaded.glueReplaced = true;
    const glue = createGlue ? createGlue(Module, glueConfig) : new Module.SlamRefactGlue(glueConfig);
    let pyramid;
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
      imageTargets.attach(glue, framework_, Object.keys(detectedImageMap_));
      // gpuCheck compares with SlamRefactGlue.cpuPyramidAtlas, which builds with rotation 0 only: a session with another rotation is not checked
      // (the comparison would report the rotation, not a defect); UNVERIFIED for rotation != 0 (docs/design/pkg-a-landscape-report.md).
      const gpuCheck = options.gpuCheck && rotation === 0
        ? { every: Math.max(1, options.gpuCheck.every | 0 || 10), left: Math.max(0, options.gpuCheck.frames | 0 || 30) } : null;
      const roiCheck = options.roiCheck
        ? { every: Math.max(1, options.roiCheck.every | 0 || 1), left: Math.max(0, options.roiCheck.frames | 0 || 20) } : null;
      diag.session = { videoWidth, videoHeight, rotation, device: { manufacturer: device.manufacturer, model: device.model },
        layout: { textureSize: layout.textureSize, rows: layout.rows }, renderer: rendererInfo(computeCtx), readback: pyramid.stats };
      diag.stats.sessionsCreated++;
      pushEntry(diag.sessionLog, { event: 'created', videoWidth, videoHeight, rotation });
      return { glue, pyramid, layout, videoWidth, videoHeight, pending: null, textures: new Map(),
        last: null, lastWorldPoints: [], activeTexture: null, hasResponse: false, gpuCheck, roiCheck, prevAllZeroes: null,
        lastDesc: null, liveStale: !sameCoords(coords, { origin: origin_, facing: facing_ }), contextLost: false };
    } catch (error) {
      if (pyramid) pyramid.dispose();
      glue.delete();
      throw error;
    }
  };

  const rendererInfo = (gl) => {
    const dbg = gl.getExtension('WEBGL_debug_renderer_info');
    return { renderer: gl.getParameter(dbg ? dbg.UNMASKED_RENDERER_WEBGL : gl.RENDERER), version: gl.getParameter(gl.VERSION) };
  };

  const destroySession = () => {
    if (!session_) return;
    const s = session_;
    session_ = null;
    if (s.hasResponse && s.last) {
      lastExtrinsic_ = { position: Array.from(s.last.position), rotationWxyz: Array.from(s.last.rotationWxyz) };
    }
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

  // c8EmAsm_processStagedFrame: read the staged pyramid, push the IMU staged with it, let the glue stage it and
  // track the previous one.
  const processStagedFrame = (s) => {
    const f = s.pending;
    if (!f) return;
    s.pending = null;
    const t0 = performance.now();
    let atlas = s.pyramid.read();
    let rows = s.pyramid.lastReadRows;
    const t1 = performance.now();
    pushSample(diag.stats.readMs, t1 - t0);
    for (const e of f.events || []) {
      if (pushMotion(s.glue, e)) {
        diag.stats.motionEventsPushed++;
      } else {
        diag.stats.motionEventsSkipped++;
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
    pushSample(diag.stats.engineMs, performance.now() - t1);
    diag.stats.processedFrames++;
    // The glue's own stage index of this frame is the key (RM-4a2: a counter kept here drifted for good after one
    // transient exception); bounded, because a glue that never answers fresh would only add entries.
    const stageIndex = pose.videoFrameIndex;
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
    if (!session_ || checkContext(session_)) return;
    try {
      stageFrame(session_, { computeTexture, textureWidth, textureHeight, orientation, alpha, beta, gamma,
        videoTime, frameTime, timeNanos, events });
    } catch (error) {
      // xr.js runs processGpu inside post-render's try block; a throw would skip its frame bookkeeping.
      diag.stats.stageErrors++;
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

  const finishResult = (framework, result) => {
    intrinsics_ = result.intrinsics;
    if (engineVpsEventsDisabled_) {
      result.rotation = fakeSimulatorExtrinsic_.rotation;
      result.position = fakeSimulatorExtrinsic_.position;
    }
    // Image targets: result.detectedImages was built from the engine's detections (image-targets.js). This diff
    // (image-events.js) is the only dispatcher of imagefound / imageupdated / imagelost; the engine's own imageEvents are
    // counted and dropped in processStagedFrame, so no change fires twice (RM-4b, spec 07 section 4).
    dispatchImageEvents(detectedImageMap_, result.detectedImages, (name, detail) => framework.dispatchEvent(name, detail));
    if (result.trackingStatus !== trackingStatus_ || result.trackingReason !== trackingReason_) {
      trackingStatus_ = result.trackingStatus;
      trackingReason_ = result.trackingReason;
      framework.dispatchEvent('trackingstatus', { status: result.trackingStatus, reason: result.trackingReason });
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
    });
  };

  // The compute context is lost right now (RM-4c m-E). A rebuilt session needs working GL, a lost one cannot answer.
  const computeContextLost = (gl = computeCtx_) => !!(gl && typeof gl.isContextLost === 'function' && gl.isContextLost());

  const onProcessCpu = ({ framework, frameStartResult }) => {
    const { repeatFrame } = frameStartResult;
    if (cameraProcessingDisabled_) {
      fakeRealityTexture_ = fakeRealityTexture_ || initFakeRealityTexture(drawCtx_);
      return { realityTexture: fakeRealityTexture_ };
    }
    if (runConfig.paused) return null;
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
    if (checkContext(session_)) return null;
    if (!repeatFrame) {
      // xr.js runs processCpu inside pre-render's try block; a throw would skip onUpdate and the camera texture
      // upload of this and every later frame. Report the failure and answer "no response" instead.
      try {
        processStagedFrame(session_);
      } catch (error) {
        diag.stats.processErrors++;
        reportOnce('process', error);
        return null;
      }
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
      return finishResult(framework, resultFromDesc(d));
    }
    const pose = s.last;
    s.lastDesc = {
      position: Array.from(pose.position), rotationWxyz: Array.from(pose.rotationWxyz), intrinsics: Array.from(pose.intrinsics),
      lighting: pose.lighting ? { exposure: pose.lighting.exposure, temperature: pose.lighting.temperature } : null,
      status: pose.status, reason: pose.reason, worldPoints: s.lastWorldPoints,
      detectedImages: pose.detectedImages || [],
      realityTexture: window.XR8.drawTexForComputeTex(s.activeTexture),
    };
    return finishResult(framework, resultFromDesc(s.lastDesc));
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
      destroySession();
      heldInactive_ = false;
      deferredRebuild_ = null;
      session_ = createSession({ videoWidth, videoHeight, rotation, computeCtx: computeCtx_, coords: configured_, framesUntilRecenter: startupFramesLeft_ });
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
    destroySession();
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
  const configure = (args) => {
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
  const updateCameraProjectionMatrix = ({ cam, origin, facing, updateRecenterPoint = true }) => {
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
  const buildSession = (kind, { videoWidth, videoHeight, orientation, computeCtx }) => {
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
    maybeDispatchConfigChange(framework_);
    if (!cameraSessionAttached_) return;
    destroySession();
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

  const onCanvasSizeChange = ({ canvasWidth, canvasHeight }) => {
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
  const recenter = () => {
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

  const hitTestTypeEnum = {
    UNSPECIFIED: 'UNSPECIFIED', FEATURE_POINT: 'FEATURE_POINT', ESTIMATED_SURFACE: 'ESTIMATED_SURFACE', DETECTED_SURFACE: 'DETECTED_SURFACE',
  };
  // XrController.hitTest (SDK `zA` -> c8EmAsm_query): unknown type names stay UNSPECIFIED.
  const hitTest = (x, y, includedTypes = []) => {
    if (!engine_) return [];
    if (!session_) return [];
    return session_.glue.xrHitTest(x, y, includedTypes && includedTypes.length ? includedTypes : []);
  };
  hitTest.type = hitTestTypeEnum;

  const dispatchToFramework = (name, detail) => { framework_.dispatchEvent(name, detail); };
  addWindowListener('message', (event) => {
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

  return enginePromise.then(() => ({
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
