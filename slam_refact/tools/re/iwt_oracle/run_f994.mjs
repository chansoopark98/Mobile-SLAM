// US-015 RE oracle: call the closed image-world-tracker (xr-slam.wasm func994) from Node on synthetic
// inputs and print its outputs as JSON. No server, no browser, no network: the module copy made by
// export_func.py (func994 exported as "f994") is instantiated with stub imports; only the imports
// the call reaches are implemented (time, abort, memcpy_big, invoke_* trampolines). Any other
// import throws, so a silent fake can not hide a reached path.
//
//   node run_f994.mjs <xr-slam-f994.wasm> <cases.json> > <outputs.json>
//
// Memory layout of the arguments (offsets read from the 994 decompile; see
// slam_refact/docs/spec/11-image-world-tracker.md section 9):
//   a  out HMatrix (0x84 B: m[16] col-major, inv[16], u8 noInvert, u8 invertFailed)
//   b  ImageWorldTrackerState (0xd0 B): f32 scale @0, HMatrix worldPose @4, vectors (begin,end,cap)
//      @0x88 rays HPoint2(12 B), @0x94 searchPtsInCamera HPoint3(16 B), @0xa0 visible u32,
//      @0xac worldPts HPoint3, @0xb8 ids WorldMapPointId(8 B: u32 slot, u32 id), @0xc4 targetPts HPoint3
//   c  WorldMap: point pools vector<WorldPoint 0x48> at +0x24/+0x48/+0x6c/+0x90/+0xb4
//      (WorldPoint: +4 u32 id, +8 HPoint3 position, +0x40 i32 source)
//   d  {f32 scale @0, f32 offset[3] @4, std::map<u32, WorldMapPointId> @0x14 (begin, root, size)}
//   e  DetectionImage: +0xf0 targetK (c8_PixelPinholeCameraModel 24 B), +0x21c CurvyImageGeometry,
//      +0x26c i32 type (2 = CURVY)
//   f  target view pose HMatrix, g  tam HMatrix
//   h  FrameWithPoints: vector<FramePoint 0x50> @0 (ray x,y @+0x18), vector<HPoint2> pixels @0xc,
//      intrinsic @0x9c
//   i  f32 scale guess
import fs from 'node:fs';

const [wasmPath, casesPath] = process.argv.slice(2);
const bytes = fs.readFileSync(wasmPath);
const module = new WebAssembly.Module(bytes);
let inst = null;
const reached = {};
const table = () => inst.exports.__indirect_function_table;
const mem = () => new DataView(inst.exports.memory.buffer);

const impl = {
  'env.emscripten_get_now': () => performance.now(),
  'env.emscripten_date_now': () => Date.now(),
  'env.abort': () => { throw new Error('wasm abort'); },
  'env.emscripten_memcpy_big': (d, s, n) => {
    new Uint8Array(inst.exports.memory.buffer).copyWithin(d, s, s + n);
  },
  'env.emscripten_resize_heap': () => 0,
  'env._emscripten_throw_longjmp': () => { throw new Error('longjmp'); },
  // Static constructors read the (empty) environment.
  'wasi_snapshot_preview1.environ_sizes_get': (pcount, psize) => {
    mem().setUint32(pcount, 0, true);
    mem().setUint32(psize, 0, true);
    return 0;
  },
  'wasi_snapshot_preview1.environ_get': () => 0,
  // Ceres (miniglog) stamps its warning lines with the local time; only the log text uses them.
  'env._tzset_js': (ptz, pdst, pstd, pdstname) => {
    for (const p of [ptz, pdst]) mem().setInt32(p, 0, true);
    for (const p of [pstd, pdstname]) if (p) mem().setUint32(p, 0, true);
  },
  'env._localtime_js': (t, ptm) => { new Uint8Array(inst.exports.memory.buffer, ptm, 44).fill(0); },
  'wasi_snapshot_preview1.fd_write': (fd, iov, iovcnt, pnum) => {
    const dv = mem();
    let n = 0;
    let text = '';
    for (let k = 0; k < iovcnt; k++) {
      const p = dv.getUint32(iov + 8 * k, true);
      const l = dv.getUint32(iov + 8 * k + 4, true);
      text += Buffer.from(new Uint8Array(inst.exports.memory.buffer, p, l)).toString('latin1');
      n += l;
    }
    process.stderr.write(`[wasm fd ${fd}] ${text}`);
    dv.setUint32(pnum, n, true);
    return 0;
  },
};
for (const sig of ['vi', 'ii', 'iiii', 'viiii', 'vii', 'viii']) {
  impl[`env.invoke_${sig}`] = (idx, ...args) => {
    const sp = inst.exports.stackSave();
    try {
      return table().get(idx)(...args);
    } catch (e) {
      inst.exports.stackRestore(sp);
      throw e;  // exceptions / longjmp are not expected on the 994 path
    }
  };
}
const imports = {};
for (const d of WebAssembly.Module.imports(module)) {
  if (d.kind !== 'function') throw new Error(`unexpected import kind ${d.kind}`);
  const key = `${d.module}.${d.name}`;
  imports[d.module] ??= {};
  imports[d.module][d.name] = (...args) => {
    reached[key] = (reached[key] || 0) + 1;
    if (impl[key]) return impl[key](...args);
    throw new Error(`unimplemented import reached: ${key}`);
  };
}
inst = new WebAssembly.Instance(module, imports);
const X = inst.exports;
X.emscripten_stack_init();
X.__wasm_call_ctors();

const malloc = (n) => {
  const p = X.malloc(Math.max(n, 4));
  new Uint8Array(X.memory.buffer, p, Math.max(n, 4)).fill(0);
  return p;
};
const f32 = (p, v) => mem().setFloat32(p, v, true);
const u32 = (p, v) => mem().setUint32(p, v >>> 0, true);
const i32 = (p, v) => mem().setInt32(p, v, true);
const rf32 = (p) => mem().getFloat32(p, true);
const ru32 = (p) => mem().getUint32(p, true);
const ru8 = (p) => mem().getUint8(p);

function vec(p, elemSize, items, write) {  // libc++ vector: begin, end, cap
  if (!items.length) return;
  const b = malloc(elemSize * items.length);
  items.forEach((it, k) => write(b + elemSize * k, it));
  u32(p, b);
  u32(p + 4, b + elemSize * items.length);
  u32(p + 8, b + elemSize * items.length);
}
function readVec(p, elemSize, read) {
  const b = ru32(p);
  const e = ru32(p + 4);
  const out = [];
  for (let q = b; q < e; q += elemSize) out.push(read(q));
  return out;
}
function writeHMatrix(p, h) {
  h.m.forEach((v, k) => f32(p + 4 * k, v));
  h.inv.forEach((v, k) => f32(p + 64 + 4 * k, v));
  mem().setUint8(p + 128, h.noInvert ? 1 : 0);
  mem().setUint8(p + 129, h.invertFailed ? 1 : 0);
}
function readHMatrix(p) {
  return {
    m: Array.from({length: 16}, (_, k) => rf32(p + 4 * k)),
    inv: Array.from({length: 16}, (_, k) => rf32(p + 64 + 4 * k)),
    noInvert: ru8(p + 128), invertFailed: ru8(p + 129),
  };
}
function writeK(p, k) {
  i32(p, k[0]); i32(p + 4, k[1]);
  f32(p + 8, k[2]); f32(p + 12, k[3]); f32(p + 16, k[4]); f32(p + 20, k[5]);
}
const hp3 = (q) => [rf32(q), rf32(q + 4), rf32(q + 8), rf32(q + 12)];

// libc++ std::map<u32, WorldMapPointId>: header {begin_node*, end_node.left (root), size}; node 28 B:
// left, right, parent, u8 is_black, key @16, value @20 (slot, id). Built as a sorted chain (each
// node is the right child of the previous one): a valid binary search tree, which is all the
// lookup in 994 (lower_bound walk) reads.
function writeMap(hdr, entries) {
  const endNode = hdr + 4;
  u32(hdr, endNode);
  u32(hdr + 4, 0);
  u32(hdr + 8, 0);
  const sorted = [...entries].sort((x, y) => x[0] - y[0]);
  let parent = endNode;
  let first = 0;
  sorted.forEach(([key, slot, id], k) => {
    const n = malloc(28);
    u32(n + 8, parent);
    mem().setUint8(n + 12, 1);
    u32(n + 16, key);
    u32(n + 20, slot);
    u32(n + 24, id);
    if (k === 0) { u32(endNode, n); first = n; } else { u32(parent + 4, n); }
    parent = n;
  });
  if (first) u32(hdr, first);
  u32(hdr + 8, sorted.length);
}

const cases = JSON.parse(fs.readFileSync(casesPath, 'utf8')).cases;
const results = [];
for (const c of cases) {
  const out = malloc(0x84);
  const st = malloc(0xd0);
  f32(st, c.state.scale);
  writeHMatrix(st + 4, c.state.world_pose);
  const map = malloc(0x1a0);
  const pools = [0x24, 0x48, 0x6c, 0x90, 0xb4];
  for (let pi = 0; pi < 5; pi++) {
    const pts = c.map_pools[pi] || [];
    vec(map + pools[pi], 0x48, pts, (q, pt) => {
      u32(q + 4, pt.id);
      pt.pos.forEach((v, k) => f32(q + 8 + 4 * k, v));
      i32(q + 0x40, pt.source);
    });
  }
  const norm = malloc(0x20);
  f32(norm, c.norm.scale);
  c.norm.offset.forEach((v, k) => f32(norm + 4 + 4 * k, v));
  writeMap(norm + 0x14, c.tracked);
  const im = malloc(0x280);
  writeK(im + 0xf0, c.image.target_k);
  i32(im + 0x26c, c.image.type);
  if (c.image.geom) {
    const g = c.image.geom;
    f32(im + 0x21c, g.radius); f32(im + 0x220, g.height);
    f32(im + 0x224, g.left); f32(im + 0x228, g.right); f32(im + 0x22c, g.top); f32(im + 0x230, g.bottom);
    i32(im + 0x234, g.src_rows); i32(im + 0x238, g.src_cols);
    mem().setUint8(im + 0x23c, g.is_cone ? 1 : 0); f32(im + 0x240, g.radius_bottom);
  }
  const pose = malloc(0x84);
  writeHMatrix(pose, c.target_pose);
  const tam = malloc(0x84);
  writeHMatrix(tam, c.tam);
  const fr = malloc(0xc0);
  vec(fr, 0x50, c.frame.rays, (q, r) => { f32(q + 0x18, r[0]); f32(q + 0x1c, r[1]); });
  vec(fr + 0xc, 12, c.frame.pixels, (q, r) => { f32(q, r[0]); f32(q + 4, r[1]); f32(q + 8, r[2] ?? 1); });
  writeK(fr + 0x9c, c.frame.cam_k);
  let error = null;
  try {
    X.f994(out, st, map, norm, im, pose, tam, fr, Math.fround(c.scale_guess));
  } catch (e) {
    error = String(e && e.message || e);
  }
  results.push({
    name: c.name, error,
    out: error ? null : readHMatrix(out),
    state: error ? null : {
      scale: rf32(st), world_pose: readHMatrix(st + 4),
      rays: readVec(st + 0x88, 12, (q) => [rf32(q), rf32(q + 4), rf32(q + 8)]),
      search_pts_in_camera: readVec(st + 0x94, 16, hp3),
      visible: readVec(st + 0xa0, 4, ru32),
      world_pts: readVec(st + 0xac, 16, hp3),
      ids: readVec(st + 0xb8, 8, (q) => [ru32(q), ru32(q + 4)]),
      target_pts: readVec(st + 0xc4, 16, hp3),
    },
  });
}
process.stdout.write(JSON.stringify({wasm_sha256_note: 'see export_func.py output', reached_imports: reached, results}, null, 1) + '\n');
