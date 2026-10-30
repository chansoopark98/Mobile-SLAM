/* In-page JS-boundary instrumentation for the xr-slam Wasm module (lane C, optional deliverable 4).
 * Injected with page.addInitScript before any SDK script.  SDK bytes on disk and the Wasm bytes are unchanged:
 * the module is compiled from the original bytes, its import functions are wrapped with counters before
 * instantiation, and the exports object handed back to the Emscripten glue is a plain object whose functions
 * wrap the originals.  Only the module whose sha256 equals `targetSha256` is wrapped; others pass through.
 * Hot-path cost: one closure call + counter per boundary crossing; c8EmAsm_* exports are timed per call. */
window.__wasmWrapInit = ({targetSha256, maxCallLog}) => {
  const W = window.__wasmWrap = {modules: [], target: null, exportCounts: {}, exportMs: {}, importCounts: {},
    calls: [], frameImports: [], frameSeq: 0, error: null};
  const nativeInstantiate = WebAssembly.instantiate.bind(WebAssembly);
  const nativeCompile = WebAssembly.compile.bind(WebAssembly);
  const hex = buffer => [...new Uint8Array(buffer)].map(x => x.toString(16).padStart(2, '0')).join('');
  const sha256 = async bytes => hex(await crypto.subtle.digest('SHA-256', bytes));
  let current = null;  // import counts within the current processStagedFrame call
  const wrapImports = (module, imports) => {
    const descriptors = WebAssembly.Module.imports(module);
    const wrapped = {};
    for (const [moduleName, table] of Object.entries(imports)) wrapped[moduleName] = {...table};
    let funcIndex = 0;
    const indexOf = {};
    for (const d of descriptors) {
      if (d.kind !== 'function') continue;
      const key = `${d.module}.${d.name}`;
      indexOf[key] = funcIndex++;
      const original = imports[d.module]?.[d.name];
      if (typeof original !== 'function') continue;
      W.importCounts[key] = 0;
      wrapped[d.module][d.name] = function (...args) {
        W.importCounts[key]++;
        if (current) current[key] = (current[key] || 0) + 1;
        return original.apply(this, args);
      };
    }
    return {wrapped, importFunctionIndex: indexOf, importFunctionCount: funcIndex};
  };
  const wrapExports = exports => {
    const out = {};
    for (const [name, value] of Object.entries(exports)) {
      if (typeof value !== 'function') { out[name] = value; continue; }
      W.exportCounts[name] = 0; W.exportMs[name] = 0;
      const timed = name.startsWith('c8EmAsm_');
      const isFrame = name === 'c8EmAsm_processStagedFrame';
      out[name] = timed ? function (...args) {
        W.exportCounts[name]++;
        let saved = null;
        if (isFrame) { saved = current; current = {}; W.frameSeq++; }
        const start = performance.now();
        try { return value.apply(this, args); }
        finally {
          const elapsed = performance.now() - start;
          W.exportMs[name] += elapsed;
          if (W.calls.length < maxCallLog) W.calls.push([name, +start.toFixed(3), +elapsed.toFixed(3), W.frameSeq]);
          if (isFrame) { W.frameImports.push([W.frameSeq, current]); current = saved; }
        }
      } : function (...args) { W.exportCounts[name]++; return value.apply(this, args); };
    }
    return out;
  };
  const handle = async (bytes, imports) => {
    const digest = await sha256(bytes);
    const module = await nativeCompile(bytes);
    const record = {sha256: digest, bytes: bytes.byteLength, exports: WebAssembly.Module.exports(module).length,
      imports: WebAssembly.Module.imports(module).length, wrapped: digest === targetSha256};
    W.modules.push(record);
    if (!record.wrapped) return {module, instance: await nativeInstantiate(module, imports)};
    const {wrapped, importFunctionIndex, importFunctionCount} = wrapImports(module, imports);
    W.target = {sha256: digest, importFunctionIndex, importFunctionCount,
      exportNames: WebAssembly.Module.exports(module).map(e => `${e.kind}:${e.name}`)};
    const instance = await nativeInstantiate(module, wrapped);
    return {module, instance: {exports: wrapExports(instance.exports)}};
  };
  WebAssembly.instantiate = async (input, imports) => {
    if (input instanceof WebAssembly.Module) return nativeInstantiate(input, imports);
    const bytes = ArrayBuffer.isView(input) ? input.buffer.slice(input.byteOffset, input.byteOffset + input.byteLength) : input;
    try { return await handle(bytes, imports); } catch (error) { W.error = String(error); throw error; }
  };
  WebAssembly.instantiateStreaming = async (response, imports) => {
    const bytes = await (await response).arrayBuffer();
    try { return await handle(bytes, imports); } catch (error) { W.error = String(error); throw error; }
  };
};
