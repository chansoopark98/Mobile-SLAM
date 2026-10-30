/* Read-only INPUT recorder for the ORIGINAL xr-slam Wasm (run-reference.mjs --record-inputs, thresholds-registration
 * Addendum 6 item 4).  Same mechanism as spec_frontend/capture_window.mjs (minus the GL readbacks): the Wasm bytes,
 * imports and SDK files are untouched; only the exports object handed back to the Emscripten glue is a plain copy in
 * which a few c8EmAsm_* exports are wrapped to record their arguments before calling the original.
 *
 * Recorded per call (window.__inputRec.calls, in call order):
 *   [seq, 'S', performance.now(), args]   c8EmAsm_stageFrame(texture, w, h, rotation, alpha, beta, gamma, qw, qx, qy, qz,
 *                                          videoTimeS, frameTimeS, timeNanos, latitude, longitude, accuracy)
 *   [seq, 'Q', performance.now(), [ptr, len], base64]  c8EmAsm_setEventQueue: the capnp message bytes the glue wrote at
 *                                          ptr (read from linear memory before the call; the glue frees it afterwards)
 *   [seq, 'P', performance.now(), args, pCount]  c8EmAsm_processStagedFrame (pCount = 1-based call count; equals the
 *                                          memprobe.js record seq because both wrap every call of the same export)
 *   [seq, 'I'|'O'|'V', performance.now(), args]  engineInit / engineOrientationChange / engineVideoSizeChange
 *
 * Install BEFORE memprobe.js (run-reference.mjs does): memprobe compiles the bytes itself and instantiates the Module
 * through this wrapper, so both see every call (memprobe outermost on processStagedFrame). */
window.__inputRecInit = ({targetSha256, maxCalls = 400000}) => {
  const R = window.__inputRec = {target: targetSha256, wrapped: false, error: null, calls: [], seq: 0, pCount: 0, dropped: 0};
  const nativeInstantiate = WebAssembly.instantiate.bind(WebAssembly);
  const nativeCompile = WebAssembly.compile.bind(WebAssembly);
  const hex = b => [...new Uint8Array(b)].map(x => x.toString(16).padStart(2, '0')).join('');
  const tags = {c8EmAsm_stageFrame: 'S', c8EmAsm_setEventQueue: 'Q', c8EmAsm_processStagedFrame: 'P', c8EmAsm_engineInit: 'I',
    c8EmAsm_engineOrientationChange: 'O', c8EmAsm_engineVideoSizeChange: 'V'};
  const push = rec => { if (R.calls.length < maxCalls) R.calls.push(rec); else R.dropped++; };
  const wrapExports = exports => {
    const memory = exports.memory;
    const o = {};
    for (const [name, fn] of Object.entries(exports)) {
      const tag = tags[name];
      if (typeof fn !== 'function' || !tag) { o[name] = fn; continue; }
      o[name] = function (...args) {
        try {
          const seq = ++R.seq, at = performance.now();
          if (tag === 'Q') {
            const [ptr, len] = args;
            let b64 = null;
            if (memory && ptr >= 0 && len >= 0 && ptr + len <= memory.buffer.byteLength) {
              const u8 = new Uint8Array(memory.buffer, ptr, len);
              let s = '';
              for (let k = 0; k < u8.length; k += 0x8000) s += String.fromCharCode.apply(null, u8.subarray(k, k + 0x8000));
              b64 = btoa(s);
            }
            push([seq, tag, at, [ptr, len], b64]);
          } else if (tag === 'P') push([seq, tag, at, args, ++R.pCount]);
          else push([seq, tag, at, args]);
        } catch (error) { if (!R.error) R.error = String(error && error.stack || error); }
        return fn.apply(this, args);
      };
    }
    return o;
  };
  const isTarget = module => WebAssembly.Module.exports(module).some(e => e.name === 'c8EmAsm_stageFrame');
  const handle = async (bytes, imports) => {
    const digest = hex(await crypto.subtle.digest('SHA-256', bytes));
    const module = await nativeCompile(bytes);
    const instance = await nativeInstantiate(module, imports);
    if (digest !== targetSha256) return {module, instance};
    R.wrapped = true;
    return {module, instance: {exports: wrapExports(instance.exports)}};
  };
  WebAssembly.instantiate = async (input, imports) => {
    if (input instanceof WebAssembly.Module) {
      // memprobe.js path: it verified the sha256 of these bytes and hands the compiled Module here.
      if (!isTarget(input)) return nativeInstantiate(input, imports);
      R.wrapped = true;
      const inst = await nativeInstantiate(input, imports);
      return {exports: wrapExports(inst.exports)};
    }
    const bytes = ArrayBuffer.isView(input) ? input.buffer.slice(input.byteOffset, input.byteOffset + input.byteLength) : input;
    try { return await handle(bytes, imports); } catch (error) { R.error = String(error); throw error; }
  };
  WebAssembly.instantiateStreaming = async (response, imports) => {
    const bytes = await (await response).arrayBuffer();
    try { return await handle(bytes, imports); } catch (error) { R.error = String(error); throw error; }
  };
};
