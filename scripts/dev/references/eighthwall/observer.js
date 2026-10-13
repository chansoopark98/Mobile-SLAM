// Observer-only wrapper: original Wasm bytes and numerical functions remain intact.
window.__probe = {ready:false,started:false,done:false,errors:[],events:[],modules:[],calls:[],callCounts:{},rows:[],frames:[],imu:[],cameraRequests:[]};
window.addEventListener('error', e => window.__probe.errors.push({kind:'window',message:e.message,stack:e.error?.stack}));
window.addEventListener('unhandledrejection', e => window.__probe.errors.push({kind:'promise',message:String(e.reason),stack:e.reason?.stack}));
(() => {
  const nativeInstantiate = WebAssembly.instantiate.bind(WebAssembly);
  const hash = async bytes => [...new Uint8Array(await crypto.subtle.digest('SHA-256', bytes))].map(x=>x.toString(16).padStart(2,'0')).join('');
  const observe = async (result, bytes) => {
    const instance = result.instance || result;
    const module = result.module || null;
    const identity = {sha256:bytes ? await hash(bytes) : null, exports:Object.keys(instance.exports),
      imports:module ? WebAssembly.Module.imports(module) : null, initialMemoryBytes:instance.exports.memory?.buffer.byteLength ?? null};
    window.__probe.modules.push(identity);
    const exports = {};
    for (const [name, value] of Object.entries(instance.exports)) {
      if (typeof value !== 'function' || !name.startsWith('c8EmAsm_')) { exports[name]=value; continue; }
      exports[name] = (...args) => {
        const key=identity.sha256+'/'+name;
        window.__probe.callCounts[key]=(window.__probe.callCounts[key]||0)+1;
        if(!/^c8EmAsm_(stageFrame|processStagedFrame|engineInit|configureXr|setEventQueue|prepareYuvBuffer|recenter|query|engineCleanup|initDeviceInfo|engineOrientationChange|engineVideoSizeChange|setDisablePixelBuffer)$/.test(name))return value(...args);
        const start = performance.now();
        const row = {name,moduleSha256:identity.sha256,args,startedWallMs:start,sourcePublication:window.__probe.frames.at(-1)?.index ?? null};
        try { const result=value(...args);row.completed=true;return result; }
        catch(error) { row.error=String(error);throw error; }
        finally { row.elapsedObserverWallMs=performance.now()-start;window.__probe.calls.push(row); }
      };
    }
    const observed = {exports};
    return result.instance ? {...result, instance:observed} : observed;
  };
  WebAssembly.instantiate = async (input, imports) => {
    const bytes = input instanceof WebAssembly.Module ? null : input;
    return observe(await nativeInstantiate(input, imports), bytes);
  };
  WebAssembly.instantiateStreaming = async (response, imports) => {
    const bytes = await (await response).arrayBuffer();
    return observe(await nativeInstantiate(bytes, imports), bytes);
  };
})();
