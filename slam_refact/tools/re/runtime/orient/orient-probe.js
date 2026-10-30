/* US-023/US-024 page-side probe for the ORIGINAL xr-slam Wasm (run-orient.mjs). Read-only: the Wasm bytes, imports
 * and SDK files are untouched (except with an explicit --probe-wasm build, see run-orient.mjs); the exports object
 * handed back to the Emscripten glue is a copy in which some c8EmAsm_* exports record their arguments first.
 *
 * Recorded (window.__orient):
 *   calls   [seq, tag, performance.now(), args, extra]
 *             'I' engineInit / 'O' engineOrientationChange / 'V' engineVideoSizeChange (captureW, captureH, rotation)
 *             'D' initDeviceInfo   extra = base64 of the capnp DeviceInfo message at (ptr, len)
 *             'Q' setEventQueue    extra = base64 of the capnp RequestPose message
 *             'S' stageFrame       args = the 17 stageFrame arguments
 *             'P' processStagedFrame (extra = 1-based count)
 *   reads   GL pyramid readbacks (>= 1024*1024*4 bytes): {call, callName, at, capture}
 *   captures[k]  Uint8Array: first `rows` rows of the 1024-wide RGBA atlas of readback k (while started and the
 *                stage count is >= fromStage)
 *   probe647    (with a probe wasm) copies of func 647 exits, read through the exported global sr_probe_647
 * Same GL readback rule as spec_frontend/capture_window.mjs (getBufferSubData into the PBO destination). */
// Bit-exact channel (US-023/024 v2): a float is shipped to the host as the hex of its float32 bit pattern, never as a JSON
// number (JSON.stringify(-0) === "0" loses the sign bit of a zero; orient-us02x-gate.v1 review B-1).
window.__orientHex = (v) => { const f = new Float32Array(1); new Float32Array(f.buffer)[0] = v; return new Uint32Array(f.buffer)[0].toString(16).padStart(8, '0'); };
// func 647 exit probe (instrument_647_wat.py): [pCount, total, calls[ {count, events[ [kind, timeNs, intervalNs, xHex, yHex, zHex] ]} ]]
// of the 647 calls since the last read; the call counter is cleared after every read. The float fields are read as raw uint32.
window.__orientReadProbe647 = (C, exports, memory, pCount) => {
  const ptr = exports.sr_probe_647.value;
  if (!ptr) return;
  const dv = new DataView(memory.buffer);
  const total = dv.getUint32(ptr, true);
  if (total === 0) return;
  const hex = (o) => dv.getUint32(o, true).toString(16).padStart(8, '0');
  const calls = [];
  for (let k = 0; k < Math.min(total, 16); k++) {
    const slot = ptr + 16 + k * 10256;
    const count = dv.getUint32(slot, true);
    const events = [];
    if (count <= 256) {
      for (let j = 0; j < count; j++) {
        const e = slot + 16 + j * 40;
        events.push([dv.getInt32(e, true), Number(dv.getBigInt64(e + 8, true)), Number(dv.getBigInt64(e + 16, true)), hex(e + 24), hex(e + 28), hex(e + 32)]);
      }
    }
    calls.push({count, events});
  }
  (C.probe647 ??= []).push([pCount, total, calls]);
  dv.setUint32(ptr, 0, true);
};
// What the host collects from the page: the page-runtime state with the reality.intrinsics values as bit patterns.
window.__orientCollect = () => ({frames: window.__probe.frames, epochWallMs: window.__probe.epochWallMs, events: window.__probe.events,
  intrinsicsChanges: window.__probe.intrinsicsChanges.map(c => ({textureWidth: c.textureWidth, textureHeight: c.textureHeight, orientation: c.orientation,
    videoTime: c.videoTime, bits: c.intrinsics.map(window.__orientHex)})),
  errors: window.__probe.errors, rows: window.__probe.rows, imuDispatched: window.__probe.imuDispatched, cameraRequests: window.__probe.cameraRequests});
window.__orientInit = ({targetSha256, maxCaptures, rows, fromStage, probeWasmUrl, screenAngle, windowOrientation, platform}) => {
  const C = window.__orient = {target: targetSha256, calls: [], reads: [], captures: [], seq: 0, stages: 0, current: null,
    error: null, wrapped: false, glInfo: null, probe647: null, probeWasm: null, pCount: 0};
  if (screenAngle !== null && screenAngle !== undefined) {
    try { Object.defineProperty(screen.orientation, 'angle', {get: () => screenAngle, configurable: true}); } catch (e) { C.error = 'angle: ' + e; }
    try { Object.defineProperty(window, 'orientation', {get: () => windowOrientation, configurable: true}); } catch (e) { C.error = 'orientation: ' + e; }
  }
  if (platform) { try { Object.defineProperty(navigator, 'platform', {get: () => platform, configurable: true}); } catch (e) { C.error = 'platform: ' + e; } }
  const nativeInstantiate = WebAssembly.instantiate.bind(WebAssembly);
  const nativeCompile = WebAssembly.compile.bind(WebAssembly);
  const hex = b => [...new Uint8Array(b)].map(x => x.toString(16).padStart(2, '0')).join('');
  const b64 = (memory, ptr, len) => {
    if (!memory || ptr < 0 || len < 0 || ptr + len > memory.buffer.byteLength) return null;
    const u8 = new Uint8Array(memory.buffer, ptr, len);
    let s = '';
    for (let k = 0; k < u8.length; k += 0x8000) s += String.fromCharCode.apply(null, u8.subarray(k, k + 0x8000));
    return btoa(s);
  };
  const tags = {c8EmAsm_stageFrame: 'S', c8EmAsm_setEventQueue: 'Q', c8EmAsm_processStagedFrame: 'P', c8EmAsm_engineInit: 'I',
    c8EmAsm_engineOrientationChange: 'O', c8EmAsm_engineVideoSizeChange: 'V', c8EmAsm_initDeviceInfo: 'D'};
  const wrapExports = exports => {
    const memory = exports.memory;
    const o = {};
    for (const [name, fn] of Object.entries(exports)) {
      const tag = tags[name];
      if (typeof fn !== 'function' || !tag) { o[name] = fn; continue; }
      o[name] = function (...args) {
        const seq = ++C.seq;
        if (tag === 'S' && window.__probe?.started) C.stages++;
        let extra = null;
        try {
          if (tag === 'Q' || tag === 'D') extra = b64(memory, args[0], args[1]);
          else if (tag === 'P') extra = ++C.pCount;
          C.calls.push([seq, tag, performance.now(), tag === 'Q' || tag === 'D' ? [args[0], args[1]] : args, extra]);
        } catch (e) { if (!C.error) C.error = String(e && e.stack || e); }
        const saved = C.current; C.current = {seq, name};
        try { return fn.apply(this, args); } finally {
          C.current = saved;
          if (tag === 'P' && exports.sr_probe_647) { try { window.__orientReadProbe647(C, exports, memory, C.pCount); } catch (e) { C.error ??= 'probe647: ' + e; } }
        }
      };
    }
    o.__memory = memory;
    return o;
  };
  const instantiateTarget = async (module, imports) => {
    const inst = await nativeInstantiate(module, imports);
    C.wrapped = true;
    return {module, instance: {exports: wrapExports(inst.exports)}};
  };
  const handle = async (bytes, imports) => {
    const digest = hex(await crypto.subtle.digest('SHA-256', bytes));
    if (digest !== targetSha256) { const module = await nativeCompile(bytes); return {module, instance: await nativeInstantiate(module, imports)}; }
    let useBytes = bytes;
    if (probeWasmUrl) {
      useBytes = await (await fetch(probeWasmUrl)).arrayBuffer();
      C.probeWasm = {url: probeWasmUrl, sha256: hex(await crypto.subtle.digest('SHA-256', useBytes)), bytes: useBytes.byteLength};
    }
    return instantiateTarget(await nativeCompile(useBytes), imports);
  };
  WebAssembly.instantiate = async (input, imports) => {
    if (input instanceof WebAssembly.Module) return nativeInstantiate(input, imports);
    const bytes = ArrayBuffer.isView(input) ? input.buffer.slice(input.byteOffset, input.byteOffset + input.byteLength) : input;
    try { return await handle(bytes, imports); } catch (e) { C.error = String(e); throw e; }
  };
  WebAssembly.instantiateStreaming = async (response, imports) => {
    const bytes = await (await response).arrayBuffer();
    try { return await handle(bytes, imports); } catch (e) { C.error = String(e); throw e; }
  };
  const proto = WebGL2RenderingContext.prototype;
  const origGet = proto.getBufferSubData;
  proto.getBufferSubData = function (target, srcOffset, dst, ...rest) {
    const r = origGet.call(this, target, srcOffset, dst, ...rest);
    if (dst && dst.length >= 1024 * 1024 * 4) {
      const ev = {kind: 'getBufferSubData', length: dst.length, call: C.current ? C.current.seq : null,
        callName: C.current ? C.current.name : null, at: performance.now(), capture: null};
      if (window.__probe?.started && C.stages - 1 >= fromStage && C.captures.length < maxCaptures) {
        ev.capture = C.captures.length;
        C.captures.push(dst.slice(0, rows * 1024 * 4));
      }
      C.reads.push(ev);
    }
    return r;
  };
  const origRead = proto.readPixels;
  proto.readPixels = function (x, y, w, h, format, type, dst, ...rest) {
    const r = origRead.call(this, x, y, w, h, format, type, dst, ...rest);
    if (w === 1024 && h === 1024) {
      const ev = {kind: typeof dst === 'number' ? 'readPixelsToPackBuffer' : 'readPixels', call: C.current ? C.current.seq : null,
        callName: C.current ? C.current.name : null, at: performance.now(), capture: null};
      if (typeof dst !== 'number' && dst && dst.length >= 1024 * 1024 * 4 && window.__probe?.started && C.stages - 1 >= fromStage && C.captures.length < maxCaptures) {
        ev.capture = C.captures.length;
        C.captures.push(dst.slice(0, rows * 1024 * 4));
      }
      C.reads.push(ev);
      if (!C.glInfo) {
        const ext = this.getExtension('WEBGL_debug_renderer_info');
        C.glInfo = {version: this.getParameter(this.VERSION), renderer: ext ? this.getParameter(ext.UNMASKED_RENDERER_WEBGL) : null,
          shadingLanguage: this.getParameter(this.SHADING_LANGUAGE_VERSION)};
      }
    }
    return r;
  };
};
