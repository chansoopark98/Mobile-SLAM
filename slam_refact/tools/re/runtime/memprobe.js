/* Read-only memory probe for the ORIGINAL xr-slam Wasm (spec 03 §8.4).  Injected with page.addInitScript
 * before any SDK script (run-reference.mjs --memprobe).  The Wasm bytes and imports are untouched: the module is
 * compiled from the original bytes and instantiated with the original import object; only the exports object
 * handed back to the Emscripten glue is a plain copy in which c8EmAsm_processStagedFrame is wrapped.  After each
 * call returns, linear memory is read (never written) and one compact record is appended.
 *
 * Root discovery: words equal to the c8::VioTracker vtable address point 0x250b0 (RTTI, spec 03 §10) are scanned
 * in linear memory; a candidate b is kept only if the pointer chain validates (MapBuilder *(b+6596), MapTracker
 * *(b+6460), WorldMap *(b+7232) inside memory, MapBuilder frame_count_ small and non-negative).  The root is
 * re-validated before every sample (one load) and re-scanned when it fails (engine re-creation at the start
 * recenter frees the old object, whose vtable word is overwritten by the destructor / allocator).
 *
 * Record layout (array, see FIELDS; record[0] = processStagedFrame sequence number, 1-based). */
window.__memProbeInit = ({targetSha256, vtable = 0x250b0, maxRecords = 200000, scanFrom = 0x10000, detailUntilSeq = 0,
                         probeWasmUrl = null, paths = false, lc = false}) => {
  // probeWasmUrl (diagnostic, capture_window --probe-wasm): compile this instrumented build of the original instead of
  // the original bytes (same functions; func 1146 additionally copies its inputs to the buffer behind the exported
  // global sr_probe_1146, see fix-lane instr/patch_wat.py). Recorded per record as 'p1146' (first 4 calls per frame).
  const FIELDS = [
    'seq', 'tMs', 'callMs', 'probeMs', 'root', 'rootScans',
    'mbFrameCount', 'mbLastResetFrame', 'mbLastStateChangeFrame', 'mbState',
    'outMapReset', 'outKeyframeCreated', 'outNewPoints',
    'trkState', 'trkFrameTrackerOk', 'trkConsecutiveOk', 'trkCandidates', 'trkSky', 'frmWorldMatches',
    'wm', 'kfSlots', 'kfFree', 'kfLive', 'wmKfSeq', 'wmLastKeyframeFrame', 'newestKfIndex', 'newestKfCreatedFrame',
    'pts1', 'pts2', 'pts3', 'pts4', 'pts5',            // [slots, free, live(state!=2), tenured(state==1)] per type
    'newGround', 'newSky',                              // keyframe frames only: type-2/3 slots whose creation seq == newest keyframe seq,
                                                        // [created (any state), still live (state != 2) at the end of the frame]
    'fftAll', 'fftGround', 'fftSky', 'fftLastOk', 'fftHeight', 'fftCalls',
  ];
  // Optional unreachable-path fields (paths = true, run-reference.mjs --memprobe-paths; off by default so the default
  // header and records stay byte-identical).  Appended after the fields above (docs/reports/re-unreachable-paths.md):
  //   mtRelocPending   u8  MapTracker+0x12ac  reloc_kf_pending_: 664 copies MapBuilderOutput[0x18] (dcmp 57108),
  //                        926 enters path A when != 0 (dcmp 100425), clears it on success (dcmp 101038)
  //   mtRelocKf        [u32, u32] MapTracker+0x12a4  keyframe id 664 copies from MapBuilderOutput+0x10
  //   mtKfCreated      u8  MapTracker+0x12b0  positive control: 664 sets it to 1 when MapBuilderOutput[1] != 0
  //   outRelocKf       u8  VioTracker+6624 (MapBuilderOutput+0x18), outRelocKfId [u32, u32] +6616 (+0x10)
  //   vtIncreasePts    u8  VioTracker+6208  increase_frame_points_ (784 reads it, dcmp 82471: nKeypoints = 1000)
  //   vpsMode          u16 VioTracker+2980  vps_mode_ (XREngine+2996; every setter above is inside vps_mode_ == 1)
  //   engineOk         *(holder = *(530516)) + 16 == VioTracker (holder chain used by func509)
  //   holderVps        [u8 holder+2200, +2201, +2202] (stageFrame / func509 VPS capture flags)
  //   mbCfg            MapBuilder settings bytes (func651) at 606224, 606248, 606256, 606264, 606272, 606284, 606285
  //                    (relocToKfWithDepthPoints), 606286, 606292
  //   frmPoints        points 784 added to this frame's TrackerInput *(VioTracker+6456): vector<FramePoint 0x50> at +0
  //                    (913 clear / 914 reserve / addImagePixelPoint loop, dcmp 82490-82532); null if not whole strides
  //   vtCfgPts         [i32 @531848, i32 @531852] = VioTracker.numIncreasedFramePoints / cloudMapRelocNumPoints
  //                    (784 loads 0[132962] / 0[132963] into DetectionConfig.nKeypoints, dcmp 82473 / 82479)
  const PATH_FIELDS = ['mtRelocPending', 'mtRelocKf', 'mtKfCreated', 'outRelocKf', 'outRelocKfId', 'vtIncreasePts',
    'vpsMode', 'engineOk', 'holderVps', 'mbCfg', 'frmPoints', 'vtCfgPts'];
  const MB_CFG_ADDRS = [606224, 606248, 606256, 606264, 606272, 606284, 606285, 606286, 606292];
  const base = FIELDS.length;
  if (paths) FIELDS.push(...PATH_FIELDS);
  // Optional loop-close fields (lc = true, run-reference.mjs --memprobe-lc, US-025 LC-0; off by default so the default
  // header and records stay byte-identical).  Appended after the fields above (and after PATH_FIELDS when both are on).
  // Read-only; loop close (G936:4159-5754) is gated by MapBuilder.loopCloseEnabled, off in the default params:
  //   outLcOk          u8   VioTracker+6628 = MapBuilderOutput+0x1c (loop close succeeded this call; out->Clear() zeroes it)
  //   outLcMat         null while MapBuilderOutput+0x20 HMatrix (forward 16 f32 + inverse 16 f32) is the identity pair,
  //                    else its 32 words as hex bit patterns
  //   mbLcRing         MapBuilder+0xa28..+0xa47 as 8 u32 words (+0xa28 u64, +0xa30..+0xa3c ring[3]+cursor, spec 03 s2.1)
  //   wmKfDescCount    (WorldMap+0x188 - +0x184) >> 5: entries of the per-keyframe global descriptor vector<32 B>
  //                    (written by AddKeyframe func 654 via func 1263 in every configuration)
  //   lcKfSlots        keyframe slots (WorldMap+0xc vector<Keyframe 984 B>, free slots included; G936:4196 compares this)
  const LC_FIELDS = ['outLcOk', 'outLcMat', 'mbLcRing', 'wmKfDescCount', 'lcKfSlots'];
  const lcBase = FIELDS.length;
  if (lc) FIELDS.push(...LC_FIELDS);
  // Optional exact-value detail (precision lane, off by default): for seq <= detailUntilSeq the record gets two
  // extra trailing entries (not in FIELDS): 'mtPose' = MapTracker pose_ (*(b+6460)+0x1010, 16 f32 bit patterns,
  // column-major, 926 param1+0x404), 'mtDelta' (frame_delta_ +0x11fc, 16 bit patterns, then FFT height bits) 'mtSubset' (subset_obs / subset world points of the last trackPoints, all_obs count), 'vtRaw' (VioTracker
  // words b+7400..b+7760: pose history / MotionPrediction inputs) and 'points' = per pool k=1..3 the slots [index, creation seq (+0x0),
  // id (+0x4), state (+0x18), position x,y,z,w (+0x8..+0x14, f32 bit patterns; 657 stores the HPoint3 at Point+8),
  // tracking hits (+0x34), tracking count (+0x38)], then 'mtTrack' (inverse blocks and projected frame, see below).
  const P = window.__memProbe = {fields: FIELDS, records: [], seq: 0, target: null, error: null, modules: [],
    scans: [], vtable, rootHistory: []};
  const nativeInstantiate = WebAssembly.instantiate.bind(WebAssembly);
  const nativeCompile = WebAssembly.compile.bind(WebAssembly);
  const hex = buffer => [...new Uint8Array(buffer)].map(x => x.toString(16).padStart(2, '0')).join('');
  let memory = null, buf = null, u8 = null, u32 = null, i32 = null, f32 = null, root = 0, rootScans = 0, probe1146 = null;
  const views = () => {
    if (buf !== memory.buffer) { buf = memory.buffer; u8 = new Uint8Array(buf); u32 = new Uint32Array(buf); i32 = new Int32Array(buf); f32 = new Float32Array(buf); }
  };
  const inMem = p => p >= scanFrom && p + 8 <= buf.byteLength && (p & 3) === 0;
  const i64lo = p => {  // int64 read as double; exact below 2^53
    const lo = u32[p >> 2], hi = i32[(p >> 2) + 1];
    return hi * 4294967296 + lo;
  };
  const valid = b => {
    if (!inMem(b) || u32[b >> 2] !== vtable) return false;
    const mb = u32[(b + 6596) >> 2], mt = u32[(b + 6460) >> 2], wm = u32[(b + 7232) >> 2];
    if (!inMem(mb) || !inMem(mt) || !(wm === 0 || inMem(wm))) return false;
    const fc = i64lo(mb + 0xa10);
    return fc >= 0 && fc < 1e7;
  };
  const scan = seq => {
    const t0 = performance.now();
    const n = u32.length, cands = [];
    for (let k = scanFrom >> 2; k < n; k++) if (u32[k] === vtable) cands.push(k << 2);
    const ok = cands.filter(valid);
    rootScans++;
    const rec = {seq, ms: +(performance.now() - t0).toFixed(3), memBytes: buf.byteLength, candidates: cands.length, valid: ok.slice(0, 8)};
    if (P.scans.length < 1000) P.scans.push(rec);
    return ok.length ? ok[ok.length - 1] : 0;
  };
  const pool = (wm, k) => {  // k = 1..5, base wm+0x24+(k-1)*0x24: vector<Point 0x48> @+0, side vector @+0xc, free list @+0x18
    const base = wm + 0x24 + (k - 1) * 0x24;
    const b = u32[base >> 2], e = u32[(base + 4) >> 2];
    const slots = (e - b) / 72;
    const free = k === 5 ? 0 : (u32[(base + 0x1c) >> 2] - u32[(base + 0x18) >> 2]) / 4;
    let live = 0, tenured = 0;
    for (let p = b; p < e; p += 72) { const s = i32[(p + 0x18) >> 2]; if (s !== 2) live++; if (s === 1) tenured++; }
    return [slots, free, live, tenured];
  };
  const countNew = (wm, k, seqv) => {
    const base = wm + 0x24 + (k - 1) * 0x24;
    const b = u32[base >> 2], e = u32[(base + 4) >> 2];
    let n = 0, live = 0;
    for (let p = b; p < e; p += 72) {
      if (u32[p >> 2] !== seqv || i32[(p + 0x40) >> 2] !== k) continue;
      n++; if (i32[(p + 0x18) >> 2] !== 2) live++;
    }
    return [n, live];
  };
  const vecLen = (p, stride) => (u32[(p + 4) >> 2] - u32[p >> 2]) / stride;
  const sample = (seq, tMs, callMs) => {
    const t0 = performance.now();
    views();
    if (!valid(root)) { const prev = root; root = scan(seq); if (root !== prev) P.rootHistory.push([seq, root]); }
    const r = new Array(FIELDS.length).fill(null);
    r[0] = seq; r[1] = +tMs.toFixed(3); r[2] = +callMs.toFixed(3); r[4] = root; r[5] = rootScans;
    if (root) {
      const b = root, mb = u32[(b + 6596) >> 2], trk = b + 6768, out = b + 6600;
      r[6] = i64lo(mb + 0xa10); r[7] = i64lo(mb + 0xa08); r[8] = i64lo(mb + 0xa18); r[9] = i32[(mb + 0xa20) >> 2];
      r[10] = u8[out]; r[11] = u8[out + 1]; r[12] = vecLen(out + 4, 8);
      r[13] = i32[(trk + 0x84) >> 2]; r[14] = u8[trk + 0x88]; r[15] = i64lo(trk + 0x90);
      r[16] = vecLen(trk + 0x98, 4); r[17] = vecLen(trk + 0xa4, 4);
      const frm = u32[(b + 6456) >> 2];
      r[18] = inMem(frm) ? u32[(frm + 0x98) >> 2] : null;
      const wm = u32[(b + 7232) >> 2];
      r[19] = wm;
      if (inMem(wm)) {
        const kb = u32[(wm + 0xc) >> 2], ke = u32[(wm + 0x10) >> 2];
        r[20] = (ke - kb) / 984; r[21] = vecLen(wm + 0x18, 4);
        let live = 0;
        for (let p = kb; p < ke; p += 984) if (u32[(p + 0x10) >> 2] !== 0) live++;
        r[22] = live; r[23] = u32[(wm + 8) >> 2]; r[24] = i64lo(wm + 0x1a0);
        const ni = u32[(wm + 0x1a8) >> 2];
        r[25] = ni;
        const kf = kb + ni * 984;
        const newestOk = ni < r[20] && u32[(kf + 0x10) >> 2] !== 0;
        r[26] = newestOk ? i64lo(kf + 8) : null;
        for (let k = 1; k <= 5; k++) r[26 + k] = pool(wm, k);
        if (r[11] && newestOk) { const s = u32[kf >> 2]; r[32] = countNew(wm, 2, s); r[33] = countNew(wm, 3, s); }
      }
      const mt = u32[(b + 6460) >> 2];
      if (inMem(mt)) {
        const f = mt + 0x88;
        r[34] = vecLen(f + 0xf4c, 20); r[35] = vecLen(f + 0xf58, 20); r[36] = vecLen(f + 0xf64, 20);
        r[37] = u8[f + 0xf3c]; r[38] = +f32[(f + 0xb50) >> 2].toFixed(6); r[39] = i32[(f + 0xf48) >> 2];
      }
      if (paths) {
        if (inMem(mt)) {
          r[base] = u8[mt + 0x12ac]; r[base + 1] = [u32[(mt + 0x12a4) >> 2], u32[(mt + 0x12a8) >> 2]];
          r[base + 2] = u8[mt + 0x12b0];
        }
        r[base + 3] = u8[b + 6624]; r[base + 4] = [u32[(b + 6616) >> 2], u32[(b + 6620) >> 2]];
        r[base + 5] = u8[b + 6208]; r[base + 6] = u32[(b + 2980) >> 2] & 0xffff;
        const holder = u32[530516 >> 2];
        if (inMem(holder)) {
          r[base + 7] = u32[holder >> 2] + 16 === b ? 1 : 0;
          r[base + 8] = [u8[holder + 2200], u8[holder + 2201], u8[holder + 2202]];
        }
        r[base + 9] = MB_CFG_ADDRS.map(a => u8[a]);
        if (inMem(frm)) {
          const pb = u32[frm >> 2], pe = u32[(frm + 4) >> 2];
          r[base + 10] = inMem(pb) && pe >= pb && (pe - pb) % 0x50 === 0 ? (pe - pb) / 0x50 : null;
        }
        r[base + 11] = [i32[531848 >> 2], i32[531852 >> 2]];
      }
      if (lc) {
        r[lcBase] = u8[b + 6628];
        const m = (b + 6632) >> 2, ident = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
        let isIdentity = true;
        for (let j = 0; j < 32 && isIdentity; j++) if (f32[m + j] !== ident[j & 15]) isIdentity = false;
        r[lcBase + 1] = isIdentity ? null : Array.from(u32.subarray(m, m + 32), v => (v >>> 0).toString(16).padStart(8, '0'));
        r[lcBase + 2] = Array.from(u32.subarray((mb + 0xa28) >> 2, ((mb + 0xa28) >> 2) + 8));
        if (inMem(wm)) {
          r[lcBase + 3] = (u32[(wm + 0x188) >> 2] - u32[(wm + 0x184) >> 2]) >>> 5;
          r[lcBase + 4] = (u32[(wm + 0x10) >> 2] - u32[(wm + 0xc) >> 2]) / 984;
        }
      }
    }
    if (root && seq <= detailUntilSeq) {
      const hx = v => (v >>> 0).toString(16).padStart(8, '0');
      const mt = u32[(root + 6460) >> 2];
      r.push(inMem(mt) ? Array.from(u32.subarray((mt + 0x1010) >> 2, ((mt + 0x1010) >> 2) + 16), hx) : null);
      // frame_delta_ (926 param1+0x47f, the FFT / inertial delta composed this frame) and FFT height (f+0xb50) bits.
      r.push(inMem(mt) ? [...Array.from(u32.subarray((mt + 0x11fc) >> 2, ((mt + 0x11fc) >> 2) + 16), hx), hx(u32[(mt + 0x88 + 0xb50) >> 2])] : null);
      // Last trackPoints result moved in by 1260: subset_obs (+0x12e4, ObservedPoint 24 B = ray x,y,w, scale i32,
      // descriptorDist, weight) and subset world points (+0x11c8, HPoint3 16 B), as bit patterns; all_obs count (+0x12b4).
      const vecWords = (p, words) => {
        const b = u32[p >> 2], e = u32[(p + 4) >> 2];
        return inMem(b) && e >= b ? Array.from(u32.subarray(b >> 2, e >> 2), hx) : [];
      };
      r.push(inMem(mt) ? {obs: vecWords(mt + 0x12e4), wp: vecWords(mt + 0x11c8), allObs: vecLen(mt + 0x12b4, 24),
        allObsWords: vecWords(mt + 0x12b4)} : null);
      // VioTracker words b+7400 .. b+7760 (pose history hist_[0..1], MotionPrediction inputs read by 784 at
      // +7636..+7732), raw bit patterns.
      r.push(Array.from(u32.subarray((root + 7400) >> 2, (root + 7760) >> 2), hx));
      const wm = u32[(root + 7232) >> 2];
      const pts = [];
      if (inMem(wm)) {
        for (let k = 1; k <= 3; k++) {
          const base = wm + 0x24 + (k - 1) * 0x24;
          const b = u32[base >> 2], e = u32[(base + 4) >> 2], list = [];
          for (let p = b, j = 0; p < e; p += 72, j++) {
            list.push([j, u32[p >> 2], u32[(p + 4) >> 2], i32[(p + 0x18) >> 2], ...Array.from(u32.subarray((p + 8) >> 2, ((p + 8) >> 2) + 4), hx),
                       u32[(p + 0x34) >> 2], u32[(p + 0x38) >> 2]]);
          }
          pts.push(list);
        }
      }
      r.push(pts);
      // 'mtTrack' (precision lane): the inverse blocks and the projected frame of the last trackPoints (1272).
      //   mtPoseInv  MapTracker pose_ inverse (+0x1010+0x40, 16 words)
      //   trPose     MapTrackResult pose (+0x1320, forward 16 words then inverse 16 words), trResidual (+0x13a4)
      //   projExt    projected FrameWithPoints (FWP = +0x1320+0x160) extrinsic_: forward FWP+336..+400 (1272 reads
      //              it at result+688 for all_world_points), inverse FWP+400..+464 (1056 reads it at
      //              this+376 with this = FWP+24 for the projection), u8 flags FWP+464/+465
      //   lastIds    last_tracked_ids_ (+0x11d4) [index, uid]
  //   proj       projected points pts_ (FWP+0x18 begin/end, FramePoint stride 0x50, 1056 layout):
      //              [id lo (+0x0c), id hi (+0x10), depth (+0x14), x_ (+0x18), y_ (+0x1c)] as bit patterns
      if (inMem(mt)) {
        const words = (p, n) => Array.from(u32.subarray(p >> 2, (p >> 2) + n), hx);
        const tr = mt + 0x1320, fwp = tr + 0x160;
        const pb = u32[(fwp + 0x18) >> 2], pe = u32[(fwp + 0x1c) >> 2], proj = [];
        if (inMem(pb) && pe >= pb && (pe - pb) % 0x50 === 0) {
          for (let p = pb; p < pe; p += 0x50) proj.push(words(p + 0x0c, 5));
        }
        // lastIds: MapTracker last_tracked_ids_ (+0x11d4, 1260 moves result+0x90 here; the next frame's first
        // trackPoints projects these ids in this order), [index, uid] per WorldMapPointId (8 B).
        const lb = u32[(mt + 0x11d4) >> 2], le = u32[(mt + 0x11d8) >> 2], lastIds = [];
        if (inMem(lb) && le >= lb) for (let p = lb; p < le; p += 8) lastIds.push([u32[p >> 2], hx(u32[(p + 4) >> 2])]);
        // kfPoses: per live keyframe slot (wm+0xc, 984 B, live when +0x10 != 0) [slot, kf seq (+0x0), forward
        // extrinsic +332..+396 and inverse +396..+460 (1549 reads the forward columns at +332..+380)].
        const kfPoses = [];
        const wmk = u32[(root + 7232) >> 2];
        if (inMem(wmk)) {
          const kb = u32[(wmk + 0xc) >> 2], ke = u32[(wmk + 0x10) >> 2];
          for (let p = kb, s = 0; p < ke; p += 984, s++) if (u32[(p + 0x10) >> 2] !== 0) kfPoses.push([s, u32[p >> 2], ...words(p + 332, 32)]);
        }
        // tracked: MapTracker tracked bimap by_frame (+0x12cc, libc++ std::map<u32, WorldMapPointId>: +0 begin,
        // +4 root, +8 size; node +0 left, +4 right, +16 key, +20 index, +24 uid), in key order. 1273 keeps the first
        // insert per frame index, so this holds the first trackPoints pass's matches of the frame.
        // idRng: MapBuilder id generator (G641: libc++ mt19937 at *(b+6596)+0x10 seeded with one getentropy() word;
        // x[624] then the index at +2496). libc++ regenerates x[i] in place per draw, so x[index..] still hold the
        // seeding values: [index, x[index], x[index+1]] lets the seed be recovered by inverting the init recurrence.
        const mbp = u32[(root + 6596) >> 2];
        let idRng = null;
        if (inMem(mbp)) {
          const ix = u32[(mbp + 0x10 + 2496) >> 2];
          idRng = ix < 623 ? [ix, u32[(mbp + 0x10 + 4 * ix) >> 2], u32[(mbp + 0x10 + 4 * ix + 4) >> 2]] : [ix, null, null];
        }
        const tracked = [];
        const tm = mt + 0x12cc, stack = [];
        let node = u32[(tm + 4) >> 2], guard = 0;
        while ((stack.length || node) && guard++ < 100000) {
          if (node && inMem(node)) { stack.push(node); node = u32[node >> 2]; continue; }
          node = stack.pop();
          tracked.push([u32[(node + 16) >> 2], u32[(node + 20) >> 2]]);
          node = u32[(node + 4) >> 2];
        }
        r.push({mtPoseInv: words(mt + 0x1010 + 0x40, 16), trPose: words(tr, 32), trResidual: hx(u32[(tr + 0x84) >> 2]),
          projExt: [...words(fwp + 336, 32), u8[fwp + 464], u8[fwp + 465]], proj, lastIds, kfPoses,
          tracked, trackedSize: u32[(tm + 8) >> 2], idRng});
      } else {
        r.push(null);
      }
    }
    if (root && seq <= detailUntilSeq && probe1146) {
      const hx = v => (v >>> 0).toString(16).padStart(8, '0');
      const base = probe1146.value >>> 0, calls = [];
      if (base) {
        const count = u32[base >> 2];
        for (let k = 0; k < Math.min(count, 4); k++) {
          const s = base + 64 + k * 8192, n = u32[s >> 2];
          calls.push({n, pose: Array.from(u32.subarray((s + 4) >> 2, (s + 4 + 132) >> 2), hx),
            obs: Array.from(u32.subarray((s + 136) >> 2, (s + 136 + n * 24) >> 2), hx),
            wp: Array.from(u32.subarray((s + 136 + n * 24) >> 2, (s + 136 + n * 40) >> 2), hx)});
        }
        u32[base >> 2] = 0;
        r.push({calls: count, p1146: calls});
      } else {
        r.push({calls: 0, p1146: []});
      }
    }
    r[3] = +(performance.now() - t0).toFixed(3);
    if (P.records.length < maxRecords) P.records.push(r);
  };
  const wrapExports = exports => {
    const out = {};
    for (const [name, value] of Object.entries(exports)) {
      if (name === 'memory') memory = value;
      if (name === 'sr_probe_1146') probe1146 = value;
      if (name !== 'c8EmAsm_processStagedFrame' || typeof value !== 'function') { out[name] = value; continue; }
      out[name] = function (...args) {
        const start = performance.now();
        try { return value.apply(this, args); }
        finally {
          const end = performance.now();
          P.seq++;
          try { sample(P.seq, end, end - start); } catch (error) { if (!P.error) P.error = String(error && error.stack || error); }
        }
      };
    }
    return out;
  };
  const handle = async (bytes, imports) => {
    const digest = hex(await crypto.subtle.digest('SHA-256', bytes));
    let compileBytes = bytes;
    const record = {sha256: digest, bytes: bytes.byteLength, wrapped: digest === targetSha256};
    if (record.wrapped && probeWasmUrl) {
      compileBytes = await (await fetch(probeWasmUrl)).arrayBuffer();
      record.probeSha256 = hex(await crypto.subtle.digest('SHA-256', compileBytes));
    }
    const module = await nativeCompile(compileBytes);
    P.modules.push(record);
    const instance = await nativeInstantiate(module, imports);
    if (!record.wrapped) return {module, instance};
    P.target = {sha256: digest};
    return {module, instance: {exports: wrapExports(instance.exports)}};
  };
  WebAssembly.instantiate = async (input, imports) => {
    if (input instanceof WebAssembly.Module) return nativeInstantiate(input, imports);
    const bytes = ArrayBuffer.isView(input) ? input.buffer.slice(input.byteOffset, input.byteOffset + input.byteLength) : input;
    try { return await handle(bytes, imports); } catch (error) { P.error = String(error); throw error; }
  };
  WebAssembly.instantiateStreaming = async (response, imports) => {
    const bytes = await (await response).arrayBuffer();
    try { return await handle(bytes, imports); } catch (error) { P.error = String(error); throw error; }
  };
};
