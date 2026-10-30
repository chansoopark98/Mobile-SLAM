/* US-025 LC-G hook handlers for the instrumented module of instrument_lc_wat.py (read-only; installed after lcgold.js, hook mode only).
 * Each handler runs INSIDE a Wasm call (the module called the import env.emscripten_memcpy_big with a magic destination), so linear memory is in the
 * exact state of that program point. Handlers only read memory and append records to window.__lcGold.hooks (via api.hookLog).
 * Ids and places: instrument_lc_wat.py docstring. Record kinds written here:
 *   lc-start      state at the start of the loop-close block (G936:4159 onward): ring, slots, descriptor vector, handles, covisibility, newest keyframe detail, pools
 *   u2-dist       the distance vector malloc(n_desc*12) as the selection loop will read it (U2: the tail beyond the live count is uninitialised heap)
 *   u2-loop       newest index and loop count of the selection loop
 *   u2-result     selection result (best distance bits, candidate index/id, found flag)
 *   1128-in/out   local match: input keyframes (newest N: pts_/store, candidate K: pts_/store/key point indices/intrinsics) and the PointMatch vector
 *   1146-in/out   PnP: observations, world points, initial pose, params; output pose and return value (only calls whose pose pointer lies in 936's frame)
 *   660-in/out    correction matrix: P_old (A), P_new (B) and C = A.inv * B
 *   655-in/out    setKeyframeExtrinsic: handle, pose argument, newest keyframe extrinsic before/after
 *   658-in, 1124-in  associate / mergePoints arguments (loop-close window only)
 *   unlink-iter      U3: every iteration of the unlink loop (node, value k) before getPoint(pts[k]); unlink-pre/post: the node before and after the associate(null) call that erases it
 *   lc-mid        state after the unlink loop, before the inner adjustPoints (1127): newest and candidate keyframe detail, pools
 * Blobs are 'b64:' strings (the runner moves them to blobs.bin). */
window.__lcHooksTable = api => {
  const {state, saved, hookLog, blob, fnv, treeNodes, inMem, i64lo, kfDetail, covis, snapshotWm, ringWords} = api;
  const KF = 984;
  const inFrame = p => state.lcFrame !== 0 && p >= state.lcFrame && p < state.lcFrame + 1840;
  const f32bits = bits => { const dv = new DataView(new ArrayBuffer(4)); dv.setUint32(0, bits >>> 0, true); return dv.getFloat32(0, true); };
  const slotById = (wm, id) => {
    const u32 = api.u32, kb = u32[(wm + 0xc) >> 2], ke = u32[(wm + 0x10) >> 2];
    for (let p = kb, s = 0; p < ke; p += KF, s++) if (id !== 0 && u32[(p + 0x10) >> 2] === id) return s;
    return -1;
  };
  const pools = wm => {
    const u32 = api.u32, i32 = api.i32, out = [];
    for (let k = 1; k <= 5; k++) {
      const base = wm + 0x24 + (k - 1) * 0x24, b = u32[base >> 2], e = u32[(base + 4) >> 2];
      let live = 0, tenured = 0;
      for (let p = b; p < e; p += 72) { const s = i32[(p + 0x18) >> 2]; if (s !== 2) live++; if (s === 1) tenured++; }
      out.push([(e - b) / 72, k === 5 ? 0 : (u32[(base + 0x1c) >> 2] - u32[(base + 0x18) >> 2]) / 4, live, tenured]);
    }
    return out;
  };
  const vecBlob = (h, stride, cap = 8 << 20) => {  // std::vector header at h -> blob of the elements, [count, blob]
    const u32 = api.u32, b = u32[h >> 2], e = u32[(h + 4) >> 2];
    if (!inMem(b) && b !== e) return [null, null];
    if (e < b || e - b > cap) return [(e - b) / stride, null];
    return [(e - b) / stride, blob(b, e - b)];
  };
  const kfFull = (wm, slot) => {  // one keyframe's raw matcher inputs
    const u32 = api.u32, kb = u32[(wm + 0xc) >> 2], kf = kb + slot * KF;
    const [nPts, pts] = vecBlob(kf + 0x14, 0x50);
    const [nOrb, orb] = vecBlob(kf + 0x2c, 32);
    const [nKp, kp] = vecBlob(kf + 0x68, 1);
    return {slot, id: u32[(kf + 0x10) >> 2], kseq: u32[kf >> 2], nPts, pts, nOrb, orb, kpBytes: nKp, kp, extrinsic: blob(kf + 0x14c, 0x84),
      intrinsicsWords: Array.from(u32.subarray((kf + 0xb0) >> 2, ((kf + 0xc8) >> 2))), excludedEdgeWord: u32[(kf + 0x3a8) >> 2]};
  };
  const words = (p, n) => Array.from(api.u32.subarray(p >> 2, (p >> 2) + n));
  const rec = (kind, r) => hookLog(kind, r);
  return {
    0x01: (a, b) => { state.lcFrame = a; state.lcWm = b; },
    0x02: a => {
      state.lcMb = a;
      const wm = state.lcWm, u32 = api.u32;
      const s = snapshotWm(a, wm);
      const ni = s.newest[0];
      rec('lc-start', {frame: state.lcFrame, wm, mb: a, ring: ringWords(a), nSlots: s.nSlots, nDesc: s.nDesc, nHandles: s.nHandles, freeList: s.freeList,
        newest: s.newest, slots: s.slots, handles: s.handles, desc: s.desc, pools: pools(wm),
        covis: s.slots.filter(x => x.id !== 0).map(x => [x.s, covis(s.kb + x.s * KF)]),
        newestDetail: ni < s.nSlots ? kfDetail(s.kb + ni * KF) : null});
    },
    0x10: (a, b) => {
      saved.dist = a; saved.nDesc = b;
      rec('u2-dist', {distPtr: a, nDesc: b, dist: blob(a, b * 12), chunkSizeWord: api.u32[(a - 4) >> 2], prev16: blob(a - 16, 16)});
    },
    0x11: (a, b) => rec('u2-loop', {newestIdx: a, loopCount: b}),
    0x12: (a, b) => { saved.bestBits = a; saved.candIdx = b; },
    0x13: (a, b) => {
      saved.candId = a; saved.found = b;
      rec('u2-result', {bestBits: saved.bestBits >>> 0, best: f32bits(saved.bestBits), candIdx: saved.candIdx, candId: a, found: b});
    },
    // ---- 1128 (local match)
    0x40: (a, b) => { saved.m = [a, b]; },
    0x41: (a, b) => { saved.m.push(a, b); },
    0x42: (a, b) => {
      saved.m.push(a);
      const [p0, p1, p2, p3, p4] = saved.m;
      saved.lc1128 = inFrame(p3) || inFrame(p4);
      if (!saved.lc1128) return;
      const wm = state.lcWm, u32 = api.u32;
      const kfN = p1 - 0x14, ni = (kfN - u32[(wm + 0xc) >> 2]) / KF;
      const cs = slotById(wm, saved.candId);
      rec('1128-in', {args: saved.m.slice(), matcherWords: words(p0, 32), newestSlot: ni, newest: kfFull(wm, ni), candSlot: cs, cand: cs >= 0 ? kfFull(wm, cs) : null,
        p2IsNewestStore: p2 === kfN + 0x2c});
    },
    0x31: () => {
      if (!saved.lc1128) return;
      const [, , , p3, p4] = saved.m;
      const [n, m] = vecBlob(p3, 20);
      rec('1128-out', {nMatches: n, matches: m, p4Words: words(p4, 16)});
    },
    // ---- 1146 (PnP)
    0x43: (a, b) => { saved.p = [a, b]; },
    0x44: (a, b) => { saved.p.push(a, b); },
    0x45: (a, b) => {
      saved.p.push(a, b);
      const [p0, p1, p2, p3, p4, p5] = saved.p;
      saved.lc1146 = inFrame(p3) || inFrame(p4);
      if (!saved.lc1146) return;
      const [nObs, obs] = vecBlob(p0, 24), [nWp, wp] = vecBlob(p1, 16);
      rec('1146-in', {args: saved.p.slice(), nObs, obs, nWp, wp, lastTwoWords: words(p2, 6), pose: blob(p3, 0x84), paramsWords: words(p4, 8), p5});
    },
    0x32: ret => {
      if (!saved.lc1146) return;
      rec('1146-out', {ret, pose: blob(saved.p[3], 0x84)});
      saved.lc1146 = false;
    },
    // ---- 660 (correction)
    0x46: (a, b) => { saved.c = [a, b]; },
    0x47: a => {
      saved.c.push(a);
      saved.lc660 = inFrame(saved.c[0]);
      if (!saved.lc660) return;
      rec('660-in', {args: saved.c.slice(), A: blob(saved.c[1], 0x84), B: blob(a, 0x84)});
    },
    0x33: () => { if (saved.lc660) { rec('660-out', {C: blob(saved.c[0], 0x84)}); saved.lc660 = false; } },
    // ---- 655 (setKeyframeExtrinsic, loop-close window)
    0x48: (a, b) => { saved.s = [a, b]; },
    0x49: a => {
      saved.s.push(a);
      saved.lc655 = inFrame(saved.s[1]);
      if (!saved.lc655) return;
      const u32 = api.u32, wm = state.lcWm;
      const hIdx = u32[saved.s[1] >> 2], hId = u32[(saved.s[1] + 4) >> 2];
      const kb = u32[(wm + 0xc) >> 2], kf = kb + hIdx * KF;
      saved.lc655Kf = kf;
      rec('655-in', {args: saved.s.slice(), handle: [hIdx, hId], pose: blob(a, 0x84), extrinsicBefore: blob(kf + 0x14c, 0x84), pools: pools(wm),
        newestDetail: kfDetail(kf)});
    },
    0x34: () => { if (saved.lc655) { rec('655-out', {extrinsicAfter: blob(saved.lc655Kf + 0x14c, 0x84), pools: pools(state.lcWm)}); saved.lc655 = false; } },
    // ---- 658 / 1124 (association, merging)
    0x4A: (a, b) => { saved.a = [a, b]; },
    0x4B: (a, b) => { saved.a.push(a, b); },
    0x4C: a => {
      saved.a.push(a);
      if (state.lcFrame === 0) return;
      const u32 = api.u32, hp = saved.a[1];
      rec('658-in', {wm: saved.a[0], handlePtr: hp, handle: inMem(hp) ? [u32[hp >> 2], u32[(hp + 4) >> 2]] : null, obsIdx: saved.a[2], kf: [saved.a[3], a]});
    },
    0x4D: (a, b) => { saved.g = [a, b]; },
    0x4E: (a, b) => { saved.g.push(a, b); },
    0x4F: a => {
      saved.g.push(a);
      if (state.lcFrame === 0) return;
      rec('1124-in', {wm: saved.g[0], src: [saved.g[1], saved.g[2]], dst: [saved.g[3], a]});
    },
    0x50: () => { if (state.lcFrame !== 0) rec('1127-in', {afterUnlink: !!saved.unlinkDone}); },
    // ---- U3
    0x39: (node, k) => {
      saved.unlinkNode = node;
      rec('unlink-pre', {node, k, nodeWords: words(node, 10), rootOfSetWords: null});
    },
    0x3A: (node, k) => {
      rec('unlink-post', {node, k, nodeWords: words(node, 10), right: api.u32[(node + 4) >> 2], parent: api.u32[(node + 8) >> 2]});
    },
    0x3C: (node, k) => {  // every unlink-loop iteration, before getPoint(pts[k]): the visited std::set node and its value
      const u32 = api.u32;
      saved.unlinkIter = (saved.unlinkIter || 0) + 1;
      if (saved.unlinkIter <= 400) rec('unlink-iter', {iter: saved.unlinkIter, node, k, nodeInMem: inMem(node), nodeWords: inMem(node) ? words(node, 10) : null});
      // DIAGNOSTIC GUARD (hook builds only): the original loop walks freed memory (U3) and can in principle never terminate; stop it with a JS exception so the
      // run ends and the records survive. The exception is logged in header.throws with message 'lcgold unlink-loop guard'.
      if (saved.unlinkIter > 5000) throw new Error('lcgold unlink-loop guard (more than 5000 iterations)');
    },
    0x3B: () => {
      saved.unlinkDone = true;
      const wm = state.lcWm, u32 = api.u32, kb = u32[(wm + 0xc) >> 2];
      const ni = u32[(wm + 0x1a8) >> 2], cs = slotById(wm, saved.candId);
      rec('lc-mid', {newest: kfDetail(kb + ni * KF), cand: cs >= 0 ? kfDetail(kb + cs * KF) : null, candSlot: cs, pools: pools(wm)});
    },
  };
};
