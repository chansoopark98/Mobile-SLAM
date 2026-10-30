/* Reader for the instrumented ORIGINAL xr-slam.wasm (instrument_it_wat.py). Injected with page.addInitScript AFTER memprobe.js (and
 * input-recorder.js), so it is the outermost wrapper of c8EmAsm_processStagedFrame: after memprobe's own sample it reads the log buffer behind the
 * exported global `sr_probe_it` (read-only except resetting the slot count) and the shared RandomNumbers state, and appends one record per call.
 * Without an instrumented module (no `sr_probe_it` export) it records nothing.
 *
 * Record {seq, count, rngPtr, slots:[{tag,a,b,c,state?}], post}:
 *   seq     1-based processStagedFrame call count (= memprobe.js seq)
 *   slots   in call order; tag 3 = func 784 entry (state before the frame's first RNG use), tag 2 = func 992 entry (+ a = found after, b = RNG index after),
 *           tag 1 = func 1022 (a = return, b = n rays, c = non-zero inliers), tag 4 = func 1025 (a = return, b = n points, c = non-zero inliers);
 *           state = base64 of the 2500 bytes of the mt19937 (x[624] u32 LE, then the libc++ index u32) for tags 2 and 3
 *   post    base64 mt19937 state at the end of the call (pointer = the last func-784 RNG pointer), null before the first func-784 call */
window.__itProbeInit = () => {
  const P = window.__srItProbe = {records: [], seq: 0, error: null, wrapped: false};
  const nativeInstantiate = WebAssembly.instantiate.bind(WebAssembly);
  const nativeStreaming = WebAssembly.instantiateStreaming ? WebAssembly.instantiateStreaming.bind(WebAssembly) : null;
  const b64 = u8 => { let s = ''; for (let k = 0; k < u8.length; k += 0x8000) s += String.fromCharCode.apply(null, u8.subarray(k, k + 0x8000)); return btoa(s); };
  const sample = (exports, seq) => {
    const base = exports.sr_probe_it.value >>> 0;
    if (!base) { P.records.push({seq, count: 0, rngPtr: 0, slots: [], post: null}); return; }
    const buf = exports.memory.buffer, u8 = new Uint8Array(buf), dv = new DataView(buf);
    const count = dv.getUint32(base, true), rngPtr = dv.getUint32(base + 4, true), slots = [];
    for (let k = 0; k < Math.min(count, 12); k++) {
      const s = base + 64 + k * 2560, tag = dv.getUint32(s, true);
      const slot = {tag, a: dv.getUint32(s + 4, true), b: dv.getUint32(s + 8, true), c: dv.getUint32(s + 12, true)};
      if (tag === 2 || tag === 3) slot.state = b64(u8.subarray(s + 32, s + 32 + 2500));
      if (tag === 1 || tag === 5 || tag === 6 || tag === 7) slot.raw = b64(u8.subarray(s + 32, s + 32 + 360));  // payload: see instrument_it_wat.py
      slots.push(slot);
    }
    dv.setUint32(base, 0, true);
    P.records.push({seq, count, rngPtr, slots, post: rngPtr ? b64(u8.subarray(rngPtr, rngPtr + 2500)) : null});
  };
  const wrapExports = exports => {
    if (!exports || !exports.sr_probe_it || !exports.memory || typeof exports.c8EmAsm_processStagedFrame !== 'function') return exports;
    P.wrapped = true;
    const out = {...exports}, fn = exports.c8EmAsm_processStagedFrame;
    out.c8EmAsm_processStagedFrame = function (...args) {
      try { return fn.apply(this, args); }
      finally { try { sample(exports, ++P.seq); } catch (error) { if (!P.error) P.error = String(error && error.stack || error); } }
    };
    return out;
  };
  const fix = r => {
    if (r && r.instance && r.instance.exports) return {...r, instance: {exports: wrapExports(r.instance.exports)}};
    if (r && r.exports) return {exports: wrapExports(r.exports)};
    return r;
  };
  WebAssembly.instantiate = async (input, imports) => fix(await nativeInstantiate(input, imports));
  if (nativeStreaming) WebAssembly.instantiateStreaming = async (response, imports) => fix(await nativeStreaming(response, imports));
};
