// Image targets of the slam_refact xr-slam chunk (full-parity PRD US-013 wiring, RM-4c / RM-4b JS, spec 07 section 4).
//
// JS side of the deployed chunk (SDK = assets/references/xr_reference/analysis-runtime/standalone/xr-slam.js; public
// ARC:reality/app/xr/js/src/tracking-controller.ts:62-130, 700-760, 795-1000, 1110-1200, 1312-1330): the
// `imageTargetData` loading queue, the `imageloading` / `imagescanning` events, the target metadata kept next to the
// engine (`targetMetadata_`), the DetectedImageTarget objects built from the engine's detections, and the found /
// updated / lost diff. The engine side is SlamRefactGlue.setImageTargets / unloadImageTargets and
// getPose().detectedImages (C++ ImageTargetSet + detection, contract FULL-3).
//
// ONE EVENT SOURCE (RM-4b, review M9). The engine also reports `imageEvents` (C++ ImageEventTracker, a diff of the same
// rule). This module does NOT dispatch them: the original keeps `detectedImageMap_` in the JS closure, which survives
// engineInit (size / orientation changes, here: a new SlamRefactGlue with a fresh C++ tracker), and the JS diff compares
// the complete DetectedImageTarget (metadata, properties, geometry), so only the JS diff reproduces the original across
// session rebuilds and filters names whose metadata is gone. The engine events are counted
// (diag.stats.engineImageEventsIgnored) and dropped, so a change is dispatched exactly once.
import { dispatchImageEvents } from './image-events.js';

// ImageTargetType of reality/shared/engine/image-targets.ts:6-9 (xrhomeName = stored, apiName = returned to apps).
const TYPES = {
  PLANAR: { xrhomeName: 'PLANAR', apiName: 'FLAT' },
  CYLINDER: { xrhomeName: 'CYLINDER', apiName: 'CYLINDRICAL' },
  CONICAL: { xrhomeName: 'CONICAL', apiName: 'CONICAL' },
  UNSPECIFIED: { xrhomeName: 'UNSPECIFIED', apiName: 'UNSPECIFIED' },
};
const typeFromXrhome = (s) => (s === 'PLANAR' ? TYPES.PLANAR : s === 'CYLINDER' ? TYPES.CYLINDER : s === 'CONICAL' ? TYPES.CONICAL : TYPES.UNSPECIFIED);

// Curved target geometry (full-parity PRD US-030, spec 07 section 11). The engine reports the capnp `CurvyGeometry` of reality.capnp
// (@0..@8, nine floats) with every detection (getPose().detectedImages[].curvyGeometry, wasm/bindings.cc DetectedImage). The original JS
// never shows those nine fields: it keeps five of them under other names (PUB tracking-controller.ts:913-920, deployed SDK/xr-slam.js
// `A.geometry={height:g.getHeight(),radiusTop:g.getTopRadius(),...}`):
//   geometry = { height: getHeight(), radiusTop: getTopRadius(), radiusBottom: getBottomRadius(),
//                arcLengthRadians: getArcLengthRadians(), arcStartRadians: getArcStartRadians() }
// and does not forward curvyCircumferenceTop / Bottom, curvySideLength, targetCircumferenceTop (the fields "provided by the client"). The
// key order is the order of that object literal. The result is merged into every DetectedImageTarget with Object.assign
// (tracking-controller.ts:726) and sits in `geometry` of the imagescanning target list.
export function curvyGeometryToJs(g) {
  return {
    height: g.height,
    radiusTop: g.topRadius,
    radiusBottom: g.bottomRadius,
    arcLengthRadians: g.arcLengthRadians,
    arcStartRadians: g.arcStartRadians,
  };
}

// The engine accepts at most 32 active targets (ImageTargetSet kMaxActiveImageTargets; the original's limit is the
// xrhome config MAXIMUM_ACTIVE_IMAGE_TARGETS, which a standalone page does not have).
export const MAX_ACTIVE_IMAGE_TARGETS = 32;

// tracking-controller.ts:100-112.
const metadataFromImageTarget = (it) => {
  let { metadata } = it;
  if (it.userMetadataIsJson) {
    try { metadata = JSON.parse(metadata); } catch (e) { /* keep the string */ }
  }
  return metadata;
};

// tracking-controller.ts:114-136 (deployed `_Q`).
const extractTargetMetadata = (it) => {
  const type = typeFromXrhome(it.type);
  const targetMetadata = { name: it.name, type: type.apiName, metadata: metadataFromImageTarget(it), properties: null };
  let xrMetadata = {};
  if (type === TYPES.PLANAR || type === TYPES.UNSPECIFIED) {
    xrMetadata.moveable = it.moveable;
    xrMetadata.physicalWidthInMeters = it.physicalWidthInMeters;
  }
  xrMetadata = { ...xrMetadata, ...it.properties };
  targetMetadata.properties = xrMetadata;
  return targetMetadata;
};

// The original reads the image with texImage2D(img) on the compute context and the engine takes the texture's R
// channel (a luminance image is gray, R = G = B). Same upload here, read back through a scratch framebuffer; GL state
// the upload touches is restored.
export function readLuminanceFromImage(gl, img) {
  const w = img.naturalWidth, h = img.naturalHeight;
  if (!(w > 0 && h > 0)) throw new Error('image has no pixels');
  const saved = {
    active: gl.getParameter(gl.ACTIVE_TEXTURE), fb: gl.getParameter(gl.FRAMEBUFFER_BINDING), pack: gl.getParameter(gl.PIXEL_PACK_BUFFER_BINDING),
    packAlign: gl.getParameter(gl.PACK_ALIGNMENT),
  };
  gl.activeTexture(gl.TEXTURE0);
  const savedTex = gl.getParameter(gl.TEXTURE_BINDING_2D);
  const tex = gl.createTexture(), fbo = gl.createFramebuffer();
  try {
    gl.bindTexture(gl.TEXTURE_2D, tex);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, img);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, tex, 0);
    if (gl.checkFramebufferStatus(gl.FRAMEBUFFER) !== gl.FRAMEBUFFER_COMPLETE) throw new Error('image upload framebuffer incomplete');
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, null);
    gl.pixelStorei(gl.PACK_ALIGNMENT, 4);
    const rgba = new Uint8Array(w * h * 4);
    gl.readPixels(0, 0, w, h, gl.RGBA, gl.UNSIGNED_BYTE, rgba);
    const luminance = new Uint8Array(w * h);
    for (let i = 0; i < luminance.length; i++) luminance[i] = rgba[i * 4];
    return { width: w, height: h, luminance };
  } finally {
    gl.bindFramebuffer(gl.FRAMEBUFFER, saved.fb);
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, saved.pack);
    gl.pixelStorei(gl.PACK_ALIGNMENT, saved.packAlign);
    gl.bindTexture(gl.TEXTURE_2D, savedTex);
    gl.activeTexture(saved.active);
    gl.deleteFramebuffer(fbo);
    gl.deleteTexture(tex);
  }
}

// loadRemoteImageUrl (tracking-controller.ts:62-71): a detached <img crossorigin="anonymous">.
const loadRemoteImage = (url) => new Promise((resolve, reject) => {
  const img = document.createElement('img');
  img.onload = () => resolve(img);
  img.onerror = () => reject(new Error(`image failed to load: ${url}`));
  img.setAttribute('crossorigin', 'anonymous');
  img.setAttribute('src', url);
});

const defaultNextFrame = () => new Promise((resolve) => {
  if (typeof window.requestAnimationFrame === 'function') window.requestAnimationFrame(() => resolve());
  else setTimeout(resolve, 0);
});

// deps: {options (SLAM_REFACT_CHUNK_OPTIONS), diag, warnOnce, getComputeCtx(), getFramework(), getGlue() -> current
// session glue or null, onEngineError(error, what)}.
export function createImageTargets(deps) {
  const { options, diag, getComputeCtx, getGlue } = deps;
  let configured_ = null;           // configuredImageTargetData_
  let targetMetadata_ = [];         // targetMetadata_ (processed metadata + geometry once loaded)
  let currentTargetUrls_ = [];      // currentTargetUrls_ (imagePath of targets that are loaded in the engine)
  const imagesToLoad_ = [];         // imagesToLoad_ (load queue of ids)
  let imgId_ = 0;
  let batchId_ = 0;                 // imageBatchLoadId_
  // name -> {name, imagePath, json, luminance, width, height}: what the engine was given; replayed into every new glue.
  const loaded_ = new Map();
  const nextFrame = options.nextFrame || defaultNextFrame;
  let pending_ = 0;
  let idleWaiters_ = [];
  const state = diag.imageTargets = {
    whenIdle: () => (pending_ === 0 ? Promise.resolve() : new Promise((resolve) => idleWaiters_.push(resolve))),
    loadErrors: 0, loadedNames: () => Array.from(loaded_.keys()),
  };
  const settle = () => { if (pending_ === 0) { const w = idleWaiters_; idleWaiters_ = []; w.forEach((fn) => fn()); } };
  const loadLuminance = options.loadLuminance
    || (async (url) => readLuminanceFromImage(getComputeCtx(), await loadRemoteImage(url)));
  // Replay of loaded_ into a new glue: {glue, framework, pending: [names not given to the engine yet]} (attach / pump).
  let replay_ = null;
  const attachBudgetMs = options.attachBudgetMs ?? 8;
  state.replayPending = () => (replay_ ? replay_.pending.length : 0);

  // What the engine of the current glue is given: every loaded target except the ones the replay has not reached yet (the
  // engine's setImageTargets takes the complete list and unloads what is missing from it).
  const inputs = () => Array.from(loaded_.values()).filter((t) => !(replay_ && replay_.pending.includes(t.name)))
    .map((t) => ({ json: t.json, luminance: t.luminance, width: t.width, height: t.height }));
  const removeId = (id) => { const i = imagesToLoad_.indexOf(id); if (i > -1) imagesToLoad_.splice(i, 1); };
  const scanningIfDone = (framework) => {
    if (framework && imagesToLoad_.length === 0) framework.dispatchEvent('imagescanning', { imageTargets: targetMetadata_ });
  };

  const loadFailed = (target, error, framework, where) => {
    state.loadErrors++;
    const message = `[XR] Image target "${target.name}" ${where}: ${error && error.message || error}`;
    console.error(message);
    deps.pushEntry(diag.warnings, message);
    targetMetadata_ = targetMetadata_.filter((it) => it.name !== target.name);
    scanningIfDone(framework);
  };

  // A loaded target the engine refuses (JSON error, curvy = US-014, ...) - found when it is given to a glue, e.g. a target that
  // was loaded while no session existed. Only that target is dropped.
  const rejectLoaded = (entry, error, framework) => {
    loaded_.delete(entry.name);
    currentTargetUrls_ = currentTargetUrls_.filter((u) => u !== entry.imagePath);
    loadFailed({ name: entry.name }, error, framework, 'was rejected by the engine');
  };

  // readImageDetectionTarget (tracking-controller.ts:797-930) for one target whose image load is `promise`.
  const readTarget = async (id, target, promise, framework, batch) => {
    let image;
    try {
      image = await promise;
    } catch (error) {
      if (batch === batchId_) { removeId(id); loadFailed(target, error, framework, 'failed to load'); } else removeId(id);
      return;
    }
    if (batch !== batchId_) { removeId(id); return; }
    while (imagesToLoad_[0] !== id) {  // wait until id is first in the queue
      await nextFrame();
      if (batch !== batchId_) { removeId(id); return; }
    }
    await nextFrame();  // the original processes the uploaded texture in the next animation frame
    if (batch !== batchId_) { imagesToLoad_.shift(); return; }
    const { properties } = target;
    const entry = { name: target.name, imagePath: target.imagePath, json: JSON.stringify(target), luminance: image.luminance,
      width: image.width, height: image.height };
    loaded_.set(entry.name, entry);
    try {
      const glue = getGlue();
      if (glue) glue.setImageTargets(inputs());
    } catch (error) {
      loaded_.delete(entry.name);
      imagesToLoad_.shift();
      loadFailed(target, error, framework, 'was rejected by the engine');
      return;
    }
    currentTargetUrls_.push(target.imagePath);
    imagesToLoad_.shift();
    const itMetadata = targetMetadata_.find((o) => o.name === target.name);
    // The original sets the five curved fields here for CYLINDER / CONICAL targets (tracking-controller.ts:906-921) and the flat
    // ones for every other type. An engine without getImageTargetGeometry reports a curved target's geometry only with a detection
    // (US-030, spec 07 section 11), so there a curved target has no `geometry` until build() sees its first detection.
    const curved = target.type === TYPES.CYLINDER.xrhomeName || target.type === TYPES.CONICAL.xrhomeName;
    if (itMetadata && curved) {
      // US-030 / D-1: an engine with getImageTargetGeometry (FULL-9 and later builds) reports the load-time CurvyGeometry, as the
      // original reads it right after the load. An engine without it keeps the first-detection path of build().
      const glue = getGlue();
      const geometry = glue && typeof glue.getImageTargetGeometry === 'function' ? glue.getImageTargetGeometry(target.name) : undefined;
      if (geometry) itMetadata.geometry = curvyGeometryToJs(geometry);
    }
    if (itMetadata && !curved) {
      itMetadata.geometry = {
        scaledWidth: properties.isRotated ? 1.0 : (properties.width / properties.height),
        scaledHeight: properties.isRotated ? (properties.width / properties.height) : 1.0,
      };
    }
    scanningIfDone(framework);
  };

  // unloadImageTargets. SOURCE (RM-4d, was UNVERIFIED): the original's setActiveImageTargets passes the imagePath URLs
  // (tracking-controller.ts:1142-1144, unloadImageTargets :1109-1126) and c8EmAsm_unloadDetectionImages erases
  // detectionImages_ by name (tracking-controller.cc:509-511; the insert at :483 is by name), so for a target whose imagePath
  // differs from its name the original's unload is a no-op: the target keeps being detected (its events keep firing; the JS
  // metadata is only dropped by the next load). The chunk unloads by name, the names of the loaded entries with these URLs.
  const unloadUrls = (urls) => {
    if (!urls.length) return;
    const names = [];
    for (const [name, entry] of loaded_) if (urls.includes(entry.imagePath)) names.push(name);
    names.forEach((n) => loaded_.delete(n));
    const glue = getGlue();
    if (glue && names.length) glue.unloadImageTargets(names);
  };

  // setActiveImageTargets (tracking-controller.ts:1127-1177).
  const setActive = (imageTargets, framework) => {
    const batch = ++batchId_;
    const targetsToLoad = imageTargets.filter((x) => !currentTargetUrls_.some((obj) => obj === x.imagePath));
    const targetUrlsToUnload = currentTargetUrls_.filter((obj) => !imageTargets.some((obj2) => obj === obj2.imagePath));
    unloadUrls(targetUrlsToUnload);
    const existingTargets = imageTargets.filter((obj) => !targetsToLoad.some((obj2) => obj.imagePath === obj2.imagePath));
    const existingTargetNames = existingTargets.map((a) => a.name);
    currentTargetUrls_ = existingTargets.map((a) => a.imagePath);
    if (!targetsToLoad.length) return;
    targetMetadata_ = targetMetadata_.filter((it) => existingTargetNames.includes(it.name));
    targetMetadata_.push(...targetsToLoad.map(extractTargetMetadata));
    framework.dispatchEvent('imageloading', { imageTargets: targetMetadata_ });
    const batchItems = targetsToLoad.map((it) => {
      const id = imgId_++;
      imagesToLoad_.push(id);
      const promise = Promise.resolve().then(() => loadLuminance(it.imagePath, it));
      promise.catch(() => {});  // reported by readTarget
      return { id, it, promise };
    });
    pending_++;
    Promise.all(batchItems.map(({ id, it, promise }) => readTarget(id, it, promise, framework, batch)))
      .catch((error) => deps.pushEntry(diag.warnings, `image target load: ${error && error.message || error}`))
      .finally(() => { pending_--; settle(); });
  };

  // Pump of the replay: `budget` ms, at least one target.
  const pumpReplay = (glue, budget) => {
    const q = replay_;
    if (!q || q.glue !== glue) return;
    if (budget === Infinity) {
      // everything in one engine call; when the engine refuses the list (it validates before it changes anything) the targets
      // go one by one below, so only the refused ones are dropped
      const names = q.pending.slice();
      q.pending.length = 0;
      try {
        glue.setImageTargets(inputs());
        if (replay_ === q) replay_ = null;
        return;
      } catch (error) {
        q.pending.push(...names);
      }
    }
    const t0 = performance.now();
    while (q.pending.length) {
      const name = q.pending.shift();
      const entry = loaded_.get(name);
      if (!entry) continue;  // unloaded again meanwhile
      try {
        glue.setImageTargets(inputs());
      } catch (error) {
        rejectLoaded(entry, error, q.framework);
      }
      if (performance.now() - t0 >= budget) break;
    }
    if (!q.pending.length && replay_ === q) replay_ = null;
  };

  // The chunk-only limit (the original has none; xrhome's MAXIMUM_ACTIVE_IMAGE_TARGETS is 32) is checked BEFORE configure()
  // changes anything else, so a refused call leaves the JS state and the glue mirror alone (RM-4d, review m1: it used to throw
  // after enableLighting etc. had been applied and before configureXr forwarded them).
  const precheck = (data, { cameraProcessingDisabled }) => {
    if (Array.isArray(data) && !cameraProcessingDisabled && data.length > MAX_ACTIVE_IMAGE_TARGETS) {
      throw new Error(`[XR] Too many active image targets: ${data.length} > ${MAX_ACTIVE_IMAGE_TARGETS}`);
    }
  };

  return {
    precheck,
    // configure({imageTargetData}) (tracking-controller.ts:1312-1326); `cameraProcessingDisabled` and `appResourcesLoaded`
    // are the chunk's flags, `framework` the one of the last attach.
    configure(data, { cameraProcessingDisabled, appResourcesLoaded, framework }) {
      if (!Array.isArray(data)) throw new Error('[XR] Expected array for arg: imageTargetData');
      if (cameraProcessingDisabled) {
        console.warn('[XR] Image Targets are not supported in the current session.');
        return;
      }
      precheck(data, { cameraProcessingDisabled });
      configured_ = data;
      if (appResourcesLoaded) setActive(configured_, framework);
    },
    hasConfigured: () => !!(configured_ && configured_.length),
    onAppResourcesLoaded(framework) {
      if (configured_) setActive(configured_, framework);
    },
    // Targets in the engine of a new glue (engineInit again keeps the original's detectionImages_; a new glue starts empty).
    // Never throws (see the header): the replay starts here with the first target(s) and goes on in pump().
    // `detecting`: the names the app sees detected right now; they are replayed first (the rest keep their load order), so the
    // frames of the replay do not make a tracked target disappear and appear again (RM-4e, review A-5).
    attach(glue, framework = null, detecting = []) {
      const names = Array.from(loaded_.keys());
      const first = names.filter((n) => detecting.includes(n));
      replay_ = loaded_.size ? { glue, framework, pending: [...first, ...names.filter((n) => !detecting.includes(n))] } : null;
      if (replay_) pumpReplay(glue, attachBudgetMs);
    },
    // One step of a pending replay into the current glue (the chunk calls it once per staged frame).
    pump() {
      if (replay_) pumpReplay(getGlue(), attachBudgetMs);
    },
    hasLoaded: () => loaded_.size > 0,
    // DetectedImageTarget objects of one engine response (tracking-controller.ts:695-731).
    build(engineImages) {
      return (engineImages || []).reduce((o, msg) => {
        const stored = targetMetadata_.find((it) => it.name === msg.name);
        // A target that was just unloaded may still fire tracking events until it is fully unloaded.
        if (stored != null) {
          const image = {
            name: stored.name, metadata: stored.metadata,
            position: { x: msg.position.x, y: msg.position.y, z: msg.position.z },
            rotation: { x: msg.rotation.x, y: msg.rotation.y, z: msg.rotation.z, w: msg.rotation.w },
            scale: msg.scale,
            type: msg.type,
            properties: {
              width: stored.properties.width, height: stored.properties.height, top: stored.properties.top, left: stored.properties.left,
              originalWidth: stored.properties.originalWidth, originalHeight: stored.properties.originalHeight,
              isRotated: stored.properties.isRotated, staticOrientation: stored.properties.staticOrientation,
            },
          };
          // Curved target: the geometry of the original's load-time CurvyGeometry buffer, which the engine reports with each
          // detection; it is constant per target, so the first detection stores it and the answers are the original's.
          if (stored.geometry === undefined && msg.curvyGeometry) stored.geometry = curvyGeometryToJs(msg.curvyGeometry);
          Object.assign(image, stored.geometry);
          o.push(image);
        }
        return o;
      }, []);
    },
    dispatch: dispatchImageEvents,
    // onSessionDetach (tracking-controller.ts:1070-1076): halt loading of the current batch.
    onSessionDetach() { imgId_ = 0; batchId_++; replay_ = null; },
    // onDetach (:1109-1115). configuredImageTargetData_ stays, as in the original (the next run loads it again).
    onDetach() { imagesToLoad_.length = 0; targetMetadata_.length = 0; currentTargetUrls_.length = 0; loaded_.clear(); replay_ = null; },
  };
}
