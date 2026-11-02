// Engine side of the opt-in engine Worker (R1). Loaded only when the chunk runs with `?engineWorker=1` or
// SLAM_REFACT_CHUNK_OPTIONS.engineWorker (see engine-worker-client.js); the default chunk never imports it.
//
// One module Worker owns the WASM engine (engine-loader.js: same hash-checked bytes as the main thread would execute) and
// every SlamRefactGlue of the chunk. The main thread sends ordered messages; this side executes them in arrival order, so
// the engine sees the same call sequence as in the main-thread mode:
//   load            {baseUrl}                                  -> loaded {loaded, compileFeatures} | load-error
//   probe           {id, config}                               -> probed {id, layout} | probe-error     (pyramid layout of a capture, glue discarded)
//   create          {glueId, config}                           -> created {glueId, layout, captureIntrinsics} | create-error
//   op              {glueId, name, args}                       -> (nothing) | op-error                  (configure, updateCameraProjectionMatrix, recenter, ...)
//   frame           {glueId, seq, motion, orientation, ...}    -> frame-result                           (IMU events, orientation, processFramePyramid, world points)
//   delete          {glueId}
// Only structured-cloneable data crosses the boundary: engine answers are converted with toPlain() (typed arrays copied, embind
// objects flattened). The pyramid atlas travels as a Transferable ArrayBuffer and is sent back with the result so the main thread
// can reuse it (no per-frame allocation in steady state).
import { loadEngine } from './engine-loader.js';

// Copies an engine answer into structured-cloneable plain data.
export function toPlain(value) {
  if (value === null || typeof value !== 'object') return typeof value === 'function' ? undefined : value;
  if (ArrayBuffer.isView(value)) return value.slice();
  if (Array.isArray(value)) return value.map(toPlain);
  const out = {};
  for (const key of Object.keys(value)) {
    const v = toPlain(value[key]);
    if (v !== undefined || typeof value[key] !== 'function') out[key] = v;
  }
  return out;
}

const describe = (error) => ({ message: String(error && error.message || error), name: error && error.name || 'Error', code: error && error.code });

// post(message, transferList) sends to the main thread. deps.loadEngine is replaceable for tests.
export function createEngineWorkerCore({ post, deps = {} }) {
  const load = deps.loadEngine ?? loadEngine;
  const now = deps.now ?? (() => performance.now());
  let Module = null;
  const glues = new Map();

  const handlers = {
    async load(msg) {
      try {
        const r = await load(new URL(msg.baseUrl), {
          print: (text) => post({ type: 'log', err: false, text: String(text) }),
          printErr: (text) => post({ type: 'log', err: true, text: String(text) }),
          onAbort: (what) => post({ type: 'abort', message: String(what) }),
        });
        Module = r.Module;
        let compileFeatures = null;
        try { compileFeatures = typeof Module.SlamRefactGlue.getCompileFeatures === 'function' ? toPlain(Module.SlamRefactGlue.getCompileFeatures()) : null; }
        catch (_) { compileFeatures = null; }
        post({ type: 'loaded', loaded: toPlain(r.loaded), compileFeatures });
      } catch (error) {
        post({ type: 'load-error', error: describe(error), loaded: error && error.loaded ? toPlain(error.loaded) : null });
      }
    },
    // The pyramid layout depends on the capture (size, rotation) and is needed synchronously by the chunk when it builds a session.
    // A probe glue is built from the capture alone and discarded; the real glue's layout is compared with it when it is created.
    probe(msg) {
      let glue = null;
      try {
        glue = new Module.SlamRefactGlue(msg.config);
        post({ type: 'probed', id: msg.id, layout: toPlain(glue.pyramidLayout()) });
      } catch (error) {
        post({ type: 'probe-error', id: msg.id, error: describe(error) });
      } finally { try { glue?.delete(); } catch (_) { /* probe only */ } }
    },
    create(msg) {
      try {
        const glue = new Module.SlamRefactGlue(msg.config);
        glues.set(msg.glueId, glue);
        let captureIntrinsics = null;
        try { if (typeof glue.getCaptureIntrinsics === 'function') captureIntrinsics = toPlain(glue.getCaptureIntrinsics()); } catch (_) { captureIntrinsics = null; }
        post({ type: 'created', glueId: msg.glueId, layout: toPlain(glue.pyramidLayout()), captureIntrinsics });
      } catch (error) {
        post({ type: 'create-error', glueId: msg.glueId, error: describe(error) });
      }
    },
    op(msg) {
      const glue = glues.get(msg.glueId);
      try {
        if (!glue) throw new Error(`no glue ${msg.glueId}`);
        glue[msg.name](...msg.args);
      } catch (error) {
        post({ type: 'op-error', glueId: msg.glueId, name: msg.name, error: describe(error) });
      }
    },
    frame(msg) {
      const glue = glues.get(msg.glueId);
      const t0 = now();
      let pose = null, world = null, framesUntilRecenter = null, roiCount = 0, error = null;
      try {
        if (!glue) throw new Error(`no glue ${msg.glueId}`);
        // Same order as the main-thread mode: the IMU staged with this frame, the orientation, then the frame.
        for (const [timestampNanos, event] of msg.motion) glue.pushDeviceMotionEvent(timestampNanos, event);
        glue.pushDeviceOrientation(...msg.orientation);
        const atlas = new Uint8Array(msg.atlas, 0, msg.size * msg.rows * 4);
        pose = toPlain(glue.processFramePyramid(msg.timeNanos, msg.videoNs, atlas, msg.size, msg.rows));
        world = toPlain(glue.getWorldPoints());
        framesUntilRecenter = glue.getFramesUntilRecenter();
        roiCount = glue.getRequestedRois().length;
      } catch (e) {
        error = describe(e);
      }
      post({ type: 'frame-result', glueId: msg.glueId, seq: msg.seq, pose, world, framesUntilRecenter, roiCount, error,
        engineMs: now() - t0, workerSentEpoch: (deps.epoch ?? (() => performance.timeOrigin + performance.now()))(), atlas: msg.atlas }, [msg.atlas]);
    },
    delete(msg) {
      const glue = glues.get(msg.glueId);
      glues.delete(msg.glueId);
      try { glue?.delete(); } catch (_) { /* a failed delete leaves nothing to report to */ }
    },
  };

  return {
    handle(msg) {
      const fn = handlers[msg.type];
      if (!fn) { post({ type: 'op-error', glueId: null, name: String(msg.type), error: { message: `unknown message ${msg.type}`, name: 'Error' } }); return undefined; }
      return fn(msg);
    },
    glueCount: () => glues.size,
  };
}

// Entry point inside a real Worker (not executed when the module is imported by a test).
if (typeof WorkerGlobalScope !== 'undefined' && typeof self !== 'undefined' && self instanceof WorkerGlobalScope) {
  const core = createEngineWorkerCore({ post: (message, transfer) => self.postMessage(message, transfer || []) });
  self.onmessage = (event) => { core.handle(event.data); };
}
