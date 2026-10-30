/* US-026 S6: config poke for the ORIGINAL xr-slam Wasm (run-reference.mjs --poke, design us026-design.md §2.1-§2.4).
 *
 * Same mechanism as memprobe.js / input-recorder.js: the Wasm bytes, imports and SDK files are untouched. Installed
 * LAST (after input-recorder.js and memprobe.js), it does not compile anything: it calls the previously installed
 * WebAssembly.instantiate (memprobe -> input-recorder -> native) and wraps c8EmAsm_processStagedFrame of the exports
 * object that call returns, so it is the outermost wrapper.
 *
 * Per processStagedFrame call (seq = 1-based call count; equals the memprobe record seq because both wrap every call):
 *   pre-call   read [PV, ba value, ba cache, mse value, mse cache]; then write each requested key (u8 @606188 for
 *              `ba`, i32 @606476 for `mse`). Getter cache versions are never written.
 *   post-call  (after memprobe sampled) read the same five words again and the observation fields below. A
 *              requested key whose post value differs from the written value is a `gap` for that call
 *              (the getter re-read Params inside the call). Memory is only READ after the call.
 *
 * Allowlisted keys (no other address is ever written):
 *   ba   MapBuilder.bundleAdjustmentIterativeEnabled  u8  @606188  values 0|1   cache version @527080 (getter 651)
 *   mse  MetricScaleEstimator.scaleEstimationMethod   i32 @606476  values 1..5  cache version @527084 (getter 637)
 *   lc   MapBuilder.loopCloseEnabled                  u8  @606264  values 0|1   cache version @527080 (getter 651, shared
 *        with ba; US-025 LC-0). Only when `lc` is requested, pre/post each get one extra trailing word (the lc value)
 *        and prePostLayout ends with 'lc'; without it every output is unchanged.
 *   PV   Params version i32 @613472 (read only); baLatch u8 @608328 (func1145 one-shot init flag, read only).
 *
 * Observation fields (read only; b = VioTracker root taken from the memprobe record of the same seq):
 *   kfLive, kfNewest     live keyframe slots (wm = *(b+7232), vector<Keyframe 984 B> at wm+0xc, live when +0x10 != 0),
 *                        newest keyframe index (wm+0x1a8)
 *   kfPoseHash           FNV-1a 64 over the forward extrinsic (kf+332, 16 f32) of every live keyframe in slot order
 *   kfPoseChanged        live keyframes other than the newest whose (slot, kf seq +0x0) existed at the previous call of
 *                        the same WorldMap and whose forward extrinsic bits changed
 *   kfPoseMaxDt          max |delta translation| (extrinsic words 12..14) over those keyframes (0 when none)
 *   pts1Live, ptsHash1   type-1 pool (wm+0x24, Point 72 B, live when state +0x18 != 2): live count, FNV-1a 64 over
 *                        [slot, position x,y,z,w (+0x8..+0x14)] of live points
 *   pts1Changed          live type-1 points whose (slot, creation seq +0x0) existed at the previous call of the same
 *                        WorldMap and whose position bits changed
 *   mseOk                MSE = b+8 signature for this root: f32 @MSE+2752 == 1.0 and @MSE+2792 == 1.0 (the two
 *                        RollingWindowSum windows = movementCheckPeriodInSec, constant after construction); the
 *                        design §2.2 check (scale +2824 == 1.0, min |a| +2808 == FLT_MAX) is recorded in the header for
 *                        the first sample of every root (it is order dependent: +2808 changes with the first event)
 *   mseScale, mseResidual, mseSum, mseHave, mseActive   MSE +2824 f32, +2828 f32, +2816 f32, +2821 u8, +2812 u8
 *   mseKfHash, mseImuHash  FNV-1a 64 (mse_precision_golden.mjs convention): fnv(x[9] @MSE+16+4) + fnv(P[81] @MSE+16+76),
 *                        fnv(x[6] @MSE+1744+124) + fnv(P[36] @MSE+1744+172)
 *   vtLatch, vtLatchedScale  VioTracker latch u8 @b+2840 and latched scale f32 @b+2844 (spec 05 §1.1, DCMP:85671-85673)
 * MSE fields are null when mseOk is false. */
window.__configPokeInit = ({targetSha256, keys = {}, maxRecords = 200000}) => {
  const TABLE = {
    ba: {name: 'MapBuilder.bundleAdjustmentIterativeEnabled', addr: 606188, type: 'u8', cache: 527080, def: 0,
         ok: v => v === 0 || v === 1},
    mse: {name: 'MetricScaleEstimator.scaleEstimationMethod', addr: 606476, type: 'i32', cache: 527084, def: 1,
          ok: v => Number.isInteger(v) && v >= 1 && v <= 5},
    lc: {name: 'MapBuilder.loopCloseEnabled', addr: 606264, type: 'u8', cache: 527080, def: 0,
         ok: v => v === 0 || v === 1},
  };
  const PV = 613472, BA_LATCH = 608328;
  const FIELDS = ['seq', 'callMs', 'root', 'pre', 'post', 'gap', 'baLatch', 'kfLive', 'kfNewest', 'kfPoseHash',
    'kfPoseChanged', 'kfPoseMaxDt', 'pts1Live', 'ptsHash1', 'pts1Changed', 'mseOk', 'mseScale', 'mseResidual', 'mseSum',
    'mseHave', 'mseActive', 'mseKfHash', 'mseImuHash', 'vtLatch', 'vtLatchedScale'];
  const writes = [];
  const K = window.__configPoke = {fields: FIELDS, prePostLayout: ['PV', 'ba', 'baCache', 'mse', 'mseCache'],
    writes, records: [], seq: 0, target: null, modules: [], error: null, mseCheck: [], wrapped: false};
  for (const [k, v] of Object.entries(keys)) {
    const t = TABLE[k];
    if (!t || !t.ok(v)) { K.error = 'refused poke key/value ' + k + '=' + v; return; }
    writes.push({key: k, name: t.name, addr: t.addr, type: t.type, value: v, cacheAddr: t.cache, default: t.def});
  }
  const prevInstantiate = WebAssembly.instantiate;
  const hex = buffer => [...new Uint8Array(buffer)].map(x => x.toString(16).padStart(2, '0')).join('');
  let memory = null, buf = null, u8 = null, u32 = null, i32 = null, f32 = null;
  const views = () => {
    if (buf !== memory.buffer) { buf = memory.buffer; u8 = new Uint8Array(buf); u32 = new Uint32Array(buf); i32 = new Int32Array(buf); f32 = new Float32Array(buf); }
  };
  const inMem = p => p >= 0x10000 && p + 8 <= buf.byteLength && (p & 3) === 0;
  // FNV-1a 64 over bytes, split 32-bit arithmetic (same value as the BigInt version in mse_precision_golden.mjs).
  const fnvStart = () => [0x84222325, 0xcbf29ce4];
  const fnvBytes = (h, start, len) => {
    let lo = h[0], hi = h[1];
    for (let i = start, e = start + len; i < e; i++) {
      const l = (lo ^ u8[i]) >>> 0;
      const p = l * 435;
      const plo = p % 4294967296;
      hi = (hi * 435 + (p - plo) / 4294967296 + l * 256) % 4294967296;
      lo = plo;
    }
    h[0] = lo; h[1] = hi;
    return h;
  };
  const fnvU32 = (h, v) => {  // little-endian bytes of one u32 word (not in memory)
    let lo = h[0], hi = h[1];
    for (let s = 0; s < 32; s += 8) {
      const l = (lo ^ ((v >>> s) & 255)) >>> 0;
      const p = l * 435;
      const plo = p % 4294967296;
      hi = (hi * 435 + (p - plo) / 4294967296 + l * 256) % 4294967296;
      lo = plo;
    }
    h[0] = lo; h[1] = hi;
    return h;
  };
  const fnvHex = h => h[1].toString(16).padStart(8, '0') + h[0].toString(16).padStart(8, '0');
  const fnvMem = (p, nFloats) => fnvHex(fnvBytes(fnvStart(), p, 4 * nFloats));
  const withLc = writes.some(w => w.key === 'lc');
  if (withLc) K.prePostLayout.push('lc');
  const readKeys = () => {
    const a = [i32[PV >> 2], u8[TABLE.ba.addr], i32[TABLE.ba.cache >> 2], i32[TABLE.mse.addr >> 2], i32[TABLE.mse.cache >> 2]];
    if (withLc) a.push(u8[TABLE.lc.addr]);
    return a;
  };
  const doWrites = () => {
    for (const w of writes) { if (w.type === 'u8') u8[w.addr] = w.value; else i32[w.addr >> 2] = w.value; }
  };
  let prevWm = 0, prevKf = new Map(), prevPts = new Map(), lastRoot = 0;
  const observe = (r, b) => {
    r[6] = u8[BA_LATCH];
    if (!b || !inMem(b)) return;
    const wm = u32[(b + 7232) >> 2];
    if (inMem(wm)) {
      if (wm !== prevWm) { prevKf = new Map(); prevPts = new Map(); prevWm = wm; }
      const kb = u32[(wm + 0xc) >> 2], ke = u32[(wm + 0x10) >> 2], ni = u32[(wm + 0x1a8) >> 2];
      const h = fnvStart(), kf = new Map();
      let live = 0, changed = 0, maxDt = 0;
      for (let p = kb, s = 0; p < ke; p += 984, s++) {
        if (u32[(p + 0x10) >> 2] === 0) continue;
        live++;
        fnvBytes(h, p + 332, 64);
        const w = u32.slice((p + 332) >> 2, ((p + 332) >> 2) + 16), seqv = u32[p >> 2];
        kf.set(s, [seqv, w]);
        const old = prevKf.get(s);
        if (s !== ni && old && old[0] === seqv && old[1].some((x, j) => x !== w[j])) {
          changed++;
          const t = f32.subarray((p + 332 + 48) >> 2, ((p + 332 + 48) >> 2) + 3);
          const o = new Float32Array(old[1].buffer, 48, 3);
          maxDt = Math.max(maxDt, Math.hypot(t[0] - o[0], t[1] - o[1], t[2] - o[2]));
        }
      }
      r[7] = live; r[8] = ni; r[9] = fnvHex(h); r[10] = changed; r[11] = maxDt;
      prevKf = kf;
      const base = wm + 0x24, pb = u32[base >> 2], pe = u32[(base + 4) >> 2];
      const hp = fnvStart(), pts = new Map();
      let plive = 0, pchanged = 0;
      for (let p = pb, s = 0; p < pe; p += 72, s++) {
        if (i32[(p + 0x18) >> 2] === 2) continue;
        plive++;
        fnvU32(hp, s);
        fnvBytes(hp, p + 8, 16);
        const q = (p + 8) >> 2, v = [u32[p >> 2], u32[q], u32[q + 1], u32[q + 2], u32[q + 3]];
        pts.set(s, v);
        const old = prevPts.get(s);
        if (old && old[0] === v[0] && (old[1] !== v[1] || old[2] !== v[2] || old[3] !== v[3] || old[4] !== v[4])) pchanged++;
      }
      r[12] = plive; r[13] = fnvHex(hp); r[14] = pchanged;
      prevPts = pts;
    }
    const m = b + 8;
    const ok = f32[(m + 2752) >> 2] === 1 && f32[(m + 2792) >> 2] === 1;
    if (b !== lastRoot) {
      lastRoot = b;
      if (K.mseCheck.length < 100) K.mseCheck.push({seq: r[0], root: b, windows: ok, scaleIs1: f32[(m + 2824) >> 2] === 1,
        minLinIsFltMax: u32[(m + 2808) >> 2] === 0x7f7fffff});
    }
    r[15] = ok;
    if (ok) {
      r[16] = f32[(m + 2824) >> 2]; r[17] = f32[(m + 2828) >> 2]; r[18] = f32[(m + 2816) >> 2];
      r[19] = u8[m + 2821]; r[20] = u8[m + 2812];
      r[21] = fnvMem(m + 16 + 4, 9) + fnvMem(m + 16 + 76, 81);
      r[22] = fnvMem(m + 1744 + 124, 6) + fnvMem(m + 1744 + 172, 36);
    }
    r[23] = u8[b + 2840]; r[24] = f32[(b + 2844) >> 2];
  };
  const wrapExports = exports => {
    const out = {};
    for (const [name, value] of Object.entries(exports)) {
      if (name === 'memory') memory = value;
      if (name !== 'c8EmAsm_processStagedFrame' || typeof value !== 'function') { out[name] = value; continue; }
      out[name] = function (...args) {
        const seq = ++K.seq;
        const r = new Array(FIELDS.length).fill(null);
        r[0] = seq;
        try { views(); r[3] = readKeys(); doWrites(); } catch (error) { if (!K.error) K.error = String(error && error.stack || error); }
        const start = performance.now();
        try { return value.apply(this, args); }
        finally {
          r[1] = +(performance.now() - start).toFixed(3);
          try {
            views();
            r[4] = readKeys();
            r[5] = writes.filter(w => (w.type === 'u8' ? u8[w.addr] : i32[w.addr >> 2]) !== w.value).map(w => w.key);
            const mp = window.__memProbe, last = mp && mp.records.length ? mp.records[mp.records.length - 1] : null;
            r[2] = last && last[0] === seq ? last[4] : null;  // memprobe root of the same call (0 = no valid root)
            observe(r, r[2]);
          } catch (error) { if (!K.error) K.error = String(error && error.stack || error); }
          if (K.records.length < maxRecords) K.records.push(r);
        }
      };
    }
    return out;
  };
  const wrapResult = (res, verified) => {
    if (!verified) return res;
    K.wrapped = true;
    if (res && res.instance) return {module: res.module, instance: {exports: wrapExports(res.instance.exports)}};
    return {exports: wrapExports(res.exports)};
  };
  const viaBytes = async (bytes, imports) => {
    const digest = hex(await crypto.subtle.digest('SHA-256', bytes));
    const verified = digest === targetSha256;
    K.modules.push({sha256: digest, bytes: bytes.byteLength, wrapped: verified});
    if (verified) K.target = {sha256: digest};
    return wrapResult(await prevInstantiate.call(WebAssembly, bytes, imports), verified);
  };
  WebAssembly.instantiate = async (input, imports) => {
    if (input instanceof WebAssembly.Module) {  // no bytes to verify: never poke through this path
      K.modules.push({sha256: null, wrapped: false, note: 'Module input (unverified), not wrapped'});
      return prevInstantiate.call(WebAssembly, input, imports);
    }
    const bytes = ArrayBuffer.isView(input) ? input.buffer.slice(input.byteOffset, input.byteOffset + input.byteLength) : input;
    try { return await viaBytes(bytes, imports); } catch (error) { K.error = String(error); throw error; }
  };
  // Streaming: read the bytes once and go through the previous instantiate (memprobe's streaming path does the same).
  WebAssembly.instantiateStreaming = async (response, imports) => {
    const bytes = await (await response).arrayBuffer();
    try { return await viaBytes(bytes, imports); } catch (error) { K.error = String(error); throw error; }
  };
};
