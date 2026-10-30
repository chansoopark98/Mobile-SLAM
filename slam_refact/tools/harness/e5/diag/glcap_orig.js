/* GL readback + draw-op capture for the ORIGINAL xr-slam (pixel-level ROI diagnosis).  Init script, injected after memprobe.js / itprobe_orig.js.
 *  - wraps the exports c8EmAsm_stageFrame / c8EmAsm_processStagedFrame (context: which export call a GL call happens in; counters stageCalls, processCalls)
 *  - WebGL2 getBufferSubData with a >= 4 MiB destination = the pyramid atlas readback the CPU tracker consumes (wasm heap view): the first `rows`
 *    rows (RGBA, 1024 wide, row 0 = GL row 0) of up to `maxCaptures` readbacks are copied (window.__glCap.captures) and logged (events)
 *  - GL state/draw ops (viewport, scissor, framebuffer/program/texture binds, uniforms by name, draws, clears) are logged while captures < maxCaptures,
 *    to recover the ROI quad matrices (uniform `mvp`) the original actually drew with.  Read-only: every call is forwarded unchanged. */
window.__glCapInit = ({maxCaptures = 12, rows = 1024, maxOps = 400000} = {}) => {
  const C = window.__glCap = {captures: [], events: [], ops: [], stageCalls: 0, processCalls: 0, current: null, error: null, wrapped: false};
  const nativeInstantiate = WebAssembly.instantiate.bind(WebAssembly);
  const nativeStreaming = WebAssembly.instantiateStreaming ? WebAssembly.instantiateStreaming.bind(WebAssembly) : null;
  const wrapExports = exports => {
    if (!exports || typeof exports.c8EmAsm_stageFrame !== 'function') return exports;
    C.wrapped = true;
    const out = {...exports};
    for (const name of ['c8EmAsm_stageFrame', 'c8EmAsm_processStagedFrame']) {
      const fn = exports[name];
      out[name] = function (...args) {
        if (name === 'c8EmAsm_stageFrame') C.stageCalls++; else C.processCalls++;
        const saved = C.current; C.current = name === 'c8EmAsm_stageFrame' ? 'S' + C.stageCalls : 'P' + C.processCalls;
        try { return fn.apply(this, args); } finally { C.current = saved; }
      };
    }
    return out;
  };
  const fix = r => {
    if (r && r.instance && r.instance.exports) return {...r, instance: {exports: wrapExports(r.instance.exports)}};
    if (r && r.exports) return {exports: wrapExports(r.exports)};
    return r;
  };
  WebAssembly.instantiate = async (input, imports) => fix(await nativeInstantiate(input, imports));
  if (nativeStreaming) WebAssembly.instantiateStreaming = async (response, imports) => fix(await nativeStreaming(response, imports));
  const names = new WeakMap();   // WebGLUniformLocation -> name
  const logging = () => C.captures.length < maxCaptures && C.ops.length < maxOps;
  const log = (...a) => { if (logging()) C.ops.push([C.current, ...a]); };
  const arr = v => (v && v.length !== undefined ? Array.from(v) : v);
  for (const proto of [WebGL2RenderingContext.prototype, WebGLRenderingContext.prototype]) {
    const wrap = (name, fn) => { const orig = proto[name]; if (!orig) return; proto[name] = function (...a) { fn.call(this, a); return orig.apply(this, a); }; };
    const origLoc = proto.getUniformLocation;
    proto.getUniformLocation = function (p, n) { const l = origLoc.call(this, p, n); if (l) names.set(l, n); return l; };
    wrap('viewport', a => log('viewport', ...a));
    wrap('scissor', a => log('scissor', ...a));
    wrap('enable', a => log('enable', a[0])); wrap('disable', a => log('disable', a[0]));
    wrap('bindFramebuffer', a => log('bindFramebuffer', a[0], a[1] ? 'fb' : null));
    wrap('useProgram', a => log('useProgram', a[0] ? 'prog' : null));
    wrap('clearColor', a => log('clearColor', ...a)); wrap('clear', a => log('clear', a[0]));
    wrap('activeTexture', a => log('activeTexture', a[0]));
    wrap('bindTexture', a => log('bindTexture', a[0], a[1] ? 'tex' : null));
    wrap('texImage2D', a => log('texImage2D', a[0], a[1], a[3], a[4]));
    wrap('drawArrays', a => log('drawArrays', ...a)); wrap('drawElements', a => log('drawElements', ...a));
    for (const n of ['uniform1i', 'uniform1f', 'uniform2f', 'uniform3f', 'uniform4f', 'uniform2i']) wrap(n, a => log(n, names.get(a[0]) || '?', ...a.slice(1)));
    for (const n of ['uniform1fv', 'uniform2fv', 'uniform3fv', 'uniform4fv']) wrap(n, a => log(n, names.get(a[0]) || '?', arr(a[1])));
    wrap('uniformMatrix4fv', a => log('uniformMatrix4fv', names.get(a[0]) || '?', a[1], arr(a[2])));
    wrap('uniformMatrix3fv', a => log('uniformMatrix3fv', names.get(a[0]) || '?', a[1], arr(a[2])));
  }
  const proto = WebGL2RenderingContext.prototype;
  const origGet = proto.getBufferSubData;
  proto.getBufferSubData = function (target, srcOffset, dst, ...rest) {
    const r = origGet.call(this, target, srcOffset, dst, ...rest);
    if (dst && dst.length >= 1024 * 1024 * 4) {
      const ev = {call: C.current, stage: C.stageCalls, process: C.processCalls, at: performance.now(), capture: null, opsBefore: C.ops.length};
      if (C.captures.length < maxCaptures) { ev.capture = C.captures.length; C.captures.push(dst.slice(0, rows * 1024 * 4)); }
      C.events.push(ev);
      log('READBACK', ev.capture);
    }
    return r;
  };
};
