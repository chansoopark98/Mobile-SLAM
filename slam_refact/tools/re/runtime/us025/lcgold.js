/* US-025 LC-G: golden dumps from the ORIGINAL xr-slam Wasm (generated runner run_lcgold.mjs --lcgold; read-only).
 *
 * Same mechanism as memprobe.js / us026/config-poke.js: the Wasm bytes, imports and SDK files are untouched. Installed
 * LAST, it calls the previously installed WebAssembly.instantiate (memprobe -> input-recorder -> native) and wraps
 * c8EmAsm_processStagedFrame of the exports it gets back, so it is the outermost wrapper (outside config-poke.js when
 * --poke is on). The root VioTracker address and the per-call sequence number come from the memprobe record of the same
 * call (memprobe is innermost, so its record exists when this wrapper's post-call code runs). Memory is only READ.
 *
 * One event per processStagedFrame call that created a keyframe (MapBuilderOutput[1] != 0, b+6601), i.e. exactly the calls
 * in which MapBuilder::update can run the loop close block (G936:4159-5754) and in which AddKeyframe (func 654) wrote the
 * keyframe's global descriptor (func 1263) into WorldMap+0x184 + slot*0x20.  Layout of one event (JSON line, blobs are
 * base64 here and split into blobs.bin by the runner): see docs/design/us025-lcgold-golden.md section 2.
 *
 * Optional hook mode (hooks = true, only with the instrumented module of instrument_lc_wat.py): the module calls the
 * existing import env.emscripten_memcpy_big with a magic destination 0xFFF1xxxx (never a valid address) and two arguments;
 * this script answers it (returns 0) and reads linear memory AT THAT MOMENT (inside the Wasm call, memory is consistent).
 * The wrapped import is the only thing changed in the import object; the hook ids are listed in HOOKS below. */
window.__lcGoldInit = ({targetSha256, hooks = false, maxEvents = 4000, maxHookRecords = 20000}) => {
  const KF = 984;
  const G = window.__lcGold = {events: [], hooks: [], hookLog: [], error: null, target: null, modules: [], seq: 0, wrapped: false,
    finalSnapshot: null, hookMode: hooks, memprobeFields: null};
  const prevInstantiate = WebAssembly.instantiate;
  const hex = buffer => [...new Uint8Array(buffer)].map(x => x.toString(16).padStart(2, '0')).join('');
  let memory = null, buf = null, u8 = null, u32 = null, i32 = null, f32 = null;
  const views = () => {
    if (buf !== memory.buffer) { buf = memory.buffer; u8 = new Uint8Array(buf); u32 = new Uint32Array(buf); i32 = new Int32Array(buf); f32 = new Float32Array(buf); }
  };
  const inMem = p => p >= 0x10000 && p + 8 <= buf.byteLength && (p & 3) === 0;
  const i64lo = p => u32[p >> 2] + i32[(p >> 2) + 1] * 4294967296;
  const b64 = bytes => {
    let s = '';
    for (let i = 0; i < bytes.length; i += 0x8000) s += String.fromCharCode.apply(null, bytes.subarray(i, i + 0x8000));
    return btoa(s);
  };
  // Range guard: after the original's own undefined behaviour (U3) vector pointers can be garbage; a bad range is reported as a string instead of looping/allocating.
  const okRange = (p, n) => Number.isFinite(p) && Number.isFinite(n) && n >= 0 && n <= (8 << 20) && p >= 0 && p + n <= buf.byteLength;
  const blob = (p, n) => (okRange(p, n) ? 'b64:' + b64(u8.subarray(p, p + n)) : 'range-error:' + p + ':' + n);  // the runner replaces every 'b64:' string by a blobs.bin reference
  const fnv = (p, n) => {  // FNV-1a 64, split 32-bit arithmetic (as config-poke.js)
    if (!okRange(p, n)) return 'range-error:' + p + ':' + n;
    let lo = 0x84222325, hi = 0xcbf29ce4;
    for (let i = p, e = p + n; i < e; i++) {
      const l = (lo ^ u8[i]) >>> 0, q = l * 435, qlo = q % 4294967296;
      hi = (hi * 435 + (q - qlo) / 4294967296 + l * 256) % 4294967296; lo = qlo;
    }
    return hi.toString(16).padStart(8, '0') + lo.toString(16).padStart(8, '0');
  };
  // libc++ std::map / std::set header: begin node at +0, end node at +4 (its left child = root), size at +8.
  const treeNodes = (h, limit = 4096) => {
    // Bounded in-order walk. After the original's own undefined behaviour (U3) the red-black pointers can form cycles, so EVERY loop is bounded.
    const out = [], end = h + 4;
    let n = u32[h >> 2], steps = 0;
    const maxSteps = 16 * limit;
    while (n && n !== end && inMem(n) && out.length < limit && steps < maxSteps) {
      out.push(n);
      const r = u32[(n + 4) >> 2];
      if (r) {
        n = r;
        while (steps++ < maxSteps && inMem(n) && u32[n >> 2]) n = u32[n >> 2];
      } else {
        let p = u32[(n + 8) >> 2];
        while (steps++ < maxSteps && p && inMem(p) && u32[p >> 2] !== n) { n = p; p = u32[(n + 8) >> 2]; }
        n = p;
      }
      steps++;
    }
    if (steps >= maxSteps || out.length >= limit) out.truncated = true;
    return out;
  };
  const rootOf = () => {
    const mp = window.__memProbe, last = mp && mp.records.length ? mp.records[mp.records.length - 1] : null;
    return last ? {seq: last[0], root: last[4], rec: last} : {seq: null, root: 0, rec: null};
  };
  const ringWords = mb => Array.from(u32.subarray((mb + 0xa28) >> 2, ((mb + 0xa28) >> 2) + 6));
  const handleVec = (b, e) => (inMem(b) && e >= b && e - b <= (1 << 20) ? Array.from(u32.subarray(b >> 2, e >> 2)) : []);
  // Per-keyframe detail used by success events: pts_ handles (FramePoint 0x50 B, handle {index,id} at +0xc) and worldPtIndices_.
  const kfDetail = kf => {
    const pb = u32[(kf + 0x14) >> 2], pe = u32[(kf + 0x18) >> 2];
    const n = (pe >= pb && pe - pb <= 0x50 * 4096) ? (pe - pb) / 0x50 : 0;   // guard: garbage pointers after the original's own UB
    const handles = new Uint32Array(Math.max(0, n) * 2);
    for (let i = 0; i < n; i++) { handles[2 * i] = u32[(pb + 0x50 * i + 0xc) >> 2]; handles[2 * i + 1] = u32[(pb + 0x50 * i + 0x10) >> 2]; }
    const wpi = treeNodes(kf + 0xa4).map(x => u32[(x + 16) >> 2]);
    return {nPts: n, ptHandles: 'b64:' + b64(new Uint8Array(handles.buffer)), wpiSize: u32[(kf + 0xac) >> 2], wpi, wpiWalk: wpi.length,
      extrinsic: blob(kf + 0x14c, 0x84)};
  };
  const covis = kf => treeNodes(kf + 0x98).map(x => [u32[(x + 16) >> 2], u32[(x + 20) >> 2], u32[(x + 32) >> 2]]);
  const snapshotWm = (mb, wm) => {
    const kb = u32[(wm + 0xc) >> 2], ke = u32[(wm + 0x10) >> 2], nSlotsRaw = (ke - kb) / KF;
    const nSlots = (Number.isInteger(nSlotsRaw) && nSlotsRaw >= 0 && nSlotsRaw <= 4096) ? nSlotsRaw : 0;   // guard
    const db = u32[(wm + 0x184) >> 2], de = u32[(wm + 0x188) >> 2];
    const hb = u32[(wm + 0x190) >> 2], he = u32[(wm + 0x194) >> 2];
    const slots = [];
    for (let s = 0; s < nSlots; s++) {
      const kf = kb + s * KF, ob = u32[(kf + 0x2c) >> 2], oe = u32[(kf + 0x30) >> 2];
      slots.push({s, id: u32[(kf + 0x10) >> 2], kseq: u32[kf >> 2], created: i64lo(kf + 8), orbN: (oe - ob) >> 5,
        orbFnv: u32[(kf + 0x10) >> 2] ? fnv(ob, oe - ob) : null});
    }
    return {mb, wm, kb, nSlots, nDesc: (de - db) >> 5, nHandles: (he - hb) >> 3, desc: blob(db, de - db),
      handles: handleVec(hb, he), slots, newest: [u32[(wm + 0x1a8) >> 2], u32[(wm + 0x1ac) >> 2]], freeList: (u32[(wm + 0x1c) >> 2] - u32[(wm + 0x18) >> 2]) / 4,
      lastKeyframeFrame: i64lo(wm + 0x1a0)};
  };
  let pre = null;
  const fieldIdx = name => G.memprobeFields ? G.memprobeFields.indexOf(name) : -1;
  const sampleEvent = (seq, preState) => {
    views();
    const {root: b, rec} = rootOf();
    if (!b || !rec || rec[0] !== seq) { G.events.push({seq, skipped: 'no memprobe record/root', root: b}); return; }
    if (!G.memprobeFields && window.__memProbe) G.memprobeFields = window.__memProbe.fields.slice();
    const created = u8[b + 6601];
    if (!created) return;
    if (G.events.length >= maxEvents) return;
    const wmP = u32[(b + 7232) >> 2];
    if (!inMem(wmP)) { G.events.push({seq, skipped: 'no wm'}); return; }
    const s = snapshotWm(u32[(b + 6596) >> 2], wmP);
    const ni = s.newest[0], kfN = s.kb + ni * KF;
    const ev = {seq, root: b, wm: s.wm, mb: s.mb, lcEnabled: u8[606264], frameCount: i64lo(s.mb + 0xa10),
      mapReset: u8[b + 6600], keyframeCreated: created, outLcOk: u8[b + 6628],
      nSlots: s.nSlots, nDesc: s.nDesc, nHandles: s.nHandles, freeList: s.freeList, newest: s.newest, lastKeyframeFrame: s.lastKeyframeFrame,
      nLive: s.slots.filter(x => x.id !== 0).length, slots: s.slots, handles: s.handles,
      ringPre: preState ? preState.ring : null, ringPost: ringWords(s.mb), desc: s.desc,
      outLcMat: blob(b + 6632, 0x84), newestExtrinsic: ni < s.nSlots ? blob(kfN + 0x14c, 0x84) : null};
    if (ni < s.nSlots) {
      const ob = u32[(kfN + 0x2c) >> 2], oe = u32[(kfN + 0x30) >> 2];
      ev.newestOrb = {n: (oe - ob) >> 5, bytes: blob(ob, oe - ob), fnv: fnv(ob, oe - ob),
        headerWords: Array.from(u32.subarray((kfN + 0x2c) >> 2, ((kfN + 0x68) >> 2) + 1)), kfId: u32[(kfN + 0x10) >> 2]};
      ev.newestAgate = blob(u32[(s.wm + 0x184) >> 2] + ni * 32, 32);
    }
    ev.covis = s.slots.filter(x => x.id !== 0).map(x => [x.s, covis(s.kb + x.s * KF)]);
    if (rec && G.memprobeFields) {
      const mp = {};
      for (const nm of ['mbFrameCount', 'kfSlots', 'kfFree', 'kfLive', 'pts1', 'pts2', 'pts3', 'pts4', 'pts5', 'newGround', 'newSky', 'outNewPoints']) { const k = fieldIdx(nm); if (k >= 0) mp[nm] = rec[k]; }
      ev.memprobe = mp;
    }
    if (ev.outLcOk) {  // success: candidate = ring[(cursor + 2) % 3] (cursor was advanced after the pick)
      const r = ev.ringPost, cursor = r[5], candId = r[2 + ((cursor + 2) % 3)];
      const cs = s.slots.find(x => x.id === candId && candId !== 0);
      ev.cand = cs ? {slot: cs.s, id: candId} : {slot: null, id: candId};
      ev.post = {newest: kfDetail(kfN), cand: cs ? kfDetail(s.kb + cs.s * KF) : null};
    }
    G.events.push(ev);
  };
  const preSample = () => {
    views();
    const {root: b} = rootOf();
    let ring = null;
    if (b && inMem(b)) { const mb = u32[(b + 6596) >> 2]; if (inMem(mb)) ring = ringWords(mb); }
    return {ring, root: b};
  };
  // ------------------------------------------------------------------------------------------------ hooks (instrumented module)
  const HOOKS = G.hookIds = {};
  const hookLog = (id, rec) => { if (G.hooks.length < maxHookRecords) G.hooks.push({seq: G.seq, id, ...rec}); };
  let lcFrame = 0, lcWm = 0, lcMb = 0, saved = {};
  const hookHandler = (id, a, b) => {  // a, b: u32 values passed by the instrumented module
    views();
    const h = HOOKS[id];
    if (h) h(a, b);
  };
  window.__lcGoldRegisterHooks = table => { Object.assign(HOOKS, table(
    {views, get u8() { return u8; }, get u32() { return u32; }, get i32() { return i32; }, get f32() { return f32; },
     blob, fnv, treeNodes, inMem, i64lo, hookLog, G, saved, state: {get lcFrame() { return lcFrame; }, set lcFrame(v) { lcFrame = v; },
       get lcWm() { return lcWm; }, set lcWm(v) { lcWm = v; }, get lcMb() { return lcMb; }, set lcMb(v) { lcMb = v; }},
     kfDetail, covis, snapshotWm, ringWords})); };
  // Import wrapping (the only change to the import object): env.abort is wrapped to LOG the call (seq, JS stack with the Wasm function indices, hook state)
  // and then call the original; in hook mode env.emscripten_memcpy_big additionally answers the magic-destination hook calls.
  const wrapImports = imports => {
    if (!imports || !imports.env) return imports;
    const env = Object.assign({}, imports.env);
    const origAbort = imports.env.abort;
    if (typeof origAbort === 'function') {
      env.abort = function (...args) {
        try {
          (G.aborts = G.aborts || []).push({seq: G.seq, stack: String(new Error().stack).split('\n').slice(0, 14).map(s => s.trim()).filter(s => /wasm-function|abort/.test(s)),
            lcFrame, lcWm});
        } catch (error) { /* logging must never change behaviour */ }
        return origAbort.apply(this, args);
      };
    }
    if (hooks) {
      const orig = imports.env.emscripten_memcpy_big;
      env.emscripten_memcpy_big = function (dest, src, num) {
        if (((dest >>> 16) & 0xffff) === 0xfff1) { try { hookHandler(dest & 0xffff, src >>> 0, num >>> 0); } catch (error) { if (!G.error) G.error = 'hook: ' + String(error && error.stack || error); } return 0; }
        return orig.call(this, dest, src, num);
      };
    }
    const out = Object.assign({}, imports, {env});
    if (imports.wasi_snapshot_preview1 === imports.env) out.wasi_snapshot_preview1 = env;
    return out;
  };
  // ------------------------------------------------------------------------------------------------ wrapper
  const wrapExports = exports => {
    const out = {};
    for (const [name, value] of Object.entries(exports)) {
      if (name === 'memory') memory = value;
      if (name !== 'c8EmAsm_processStagedFrame' || typeof value !== 'function') { out[name] = value; continue; }
      out[name] = function (...args) {
        const seq = ++G.seq;
        lcFrame = 0; lcWm = 0; lcMb = 0; for (const k of Object.keys(saved)) delete saved[k];
        let preState = null;
        try { preState = preSample(); } catch (error) { if (!G.error) G.error = String(error && error.stack || error); }
        try { return value.apply(this, args); }
        catch (error) {  // log the exception that leaves the Wasm call (abort() RuntimeError or a trap such as `memory access out of bounds`), then rethrow it unchanged
          try { (G.throws = G.throws || []).push({seq, name: error && error.name, message: String(error && error.message || error).slice(0, 200),
            stack: String(error && error.stack || '').split('\n').map(s => s.trim()).filter(s => /wasm-function/.test(s)).slice(0, 12), lcFrame, lcWm}); } catch (e2) { /* logging only */ }
          throw error;
        }
        finally { try { sampleEvent(seq, preState); } catch (error) { if (!G.error) G.error = String(error && error.stack || error); } }
      };
    }
    // Final snapshot of every live keyframe (agate, ORB store fnv), taken by the runner before the page is closed.
    return out;
  };
  window.__lcGoldFinish = () => {
    try {
      views();
      const {root: b} = rootOf();
      if (!b || !inMem(b)) return null;
      const wm = u32[(b + 7232) >> 2];
      if (!inMem(wm)) return null;
      const s = snapshotWm(u32[(b + 6596) >> 2], wm);
      G.finalSnapshot = {root: b, wm, nSlots: s.nSlots, nDesc: s.nDesc, newest: s.newest, slots: s.slots, handles: s.handles, desc: s.desc,
        orb: s.slots.filter(x => x.id !== 0).map(x => { const kf = s.kb + x.s * KF, ob = u32[(kf + 0x2c) >> 2], oe = u32[(kf + 0x30) >> 2]; return [x.s, x.id, (oe - ob) >> 5, blob(ob, oe - ob)]; })};
    } catch (error) { if (!G.error) G.error = String(error && error.stack || error); }
    return null;
  };
  const wrapResult = (res, verified) => {
    if (!verified) return res;
    G.wrapped = true;
    if (res && res.instance) return {module: res.module, instance: {exports: wrapExports(res.instance.exports)}};
    return {exports: wrapExports(res.exports)};
  };
  const viaBytes = async (bytes, imports) => {
    const digest = hex(await crypto.subtle.digest('SHA-256', bytes));
    const verified = digest === targetSha256;
    G.modules.push({sha256: digest, bytes: bytes.byteLength, wrapped: verified});
    if (verified) G.target = {sha256: digest};
    return wrapResult(await prevInstantiate.call(WebAssembly, bytes, verified ? wrapImports(imports) : imports), verified);
  };
  WebAssembly.instantiate = async (input, imports) => {
    if (input instanceof WebAssembly.Module) { G.modules.push({sha256: null, wrapped: false, note: 'Module input (unverified), not wrapped'}); return prevInstantiate.call(WebAssembly, input, imports); }
    const bytes = ArrayBuffer.isView(input) ? input.buffer.slice(input.byteOffset, input.byteOffset + input.byteLength) : input;
    try { return await viaBytes(bytes, imports); } catch (error) { G.error = String(error); throw error; }
  };
  WebAssembly.instantiateStreaming = async (response, imports) => {
    const bytes = await (await response).arrayBuffer();
    try { return await viaBytes(bytes, imports); } catch (error) { G.error = String(error); throw error; }
  };
};
