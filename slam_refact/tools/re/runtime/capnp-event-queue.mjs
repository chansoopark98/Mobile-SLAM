// Minimal capnp reader for the event-queue message the reference runtime glue hands to c8EmAsm_setEventQueue
// (xr-slam.js: initRoot(hA).initEventQueue(n); per element setKind / setTimestampNanos / setEventTimestampNanos /
// setIntervalNanos / getValue().setX/Y/Z).  Generic single-/multi-segment wire format; the field layout is resolved
// by FIELD_LAYOUT below (determined from recorded bytes, see decodeEventQueueRaw for the generic dump).
const WORD = 8;

function segments(buf) {
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  const n = dv.getUint32(0, true) + 1;
  let off = 4 + 4 * n;
  if (off % 8) off += 4;
  const segs = [];
  for (let k = 0; k < n; k++) {
    const words = dv.getUint32(4 + 4 * k, true);
    segs.push({start: off, words});
    off += words * WORD;
  }
  if (off > buf.byteLength) throw new Error('truncated capnp message');
  return {dv, segs};
}

// Resolve a pointer at byte position p (absolute) in segment s; returns {kind, s, at, ...} with absolute byte offsets.
function readPointer(m, s, p) {
  const lo = m.dv.getUint32(p, true), hi = m.dv.getUint32(p + 4, true);
  if (lo === 0 && hi === 0) return null;
  const kind = lo & 3;
  if (kind === 2) {  // far pointer
    const landing = lo >>> 3, seg = hi, double = (lo >>> 2) & 1;
    if (double) throw new Error('double-far pointer not supported');
    return readPointer(m, seg, m.segs[seg].start + landing * WORD);
  }
  const offset = (lo | 0) >> 2;
  const at = p + WORD + offset * WORD;
  if (kind === 0) return {kind: 'struct', s, at, dataWords: hi & 0xffff, ptrs: hi >>> 16};
  if (kind === 1) {
    const elemSize = hi & 7, count = hi >>> 3;
    if (elemSize === 7) {
      const tlo = m.dv.getUint32(at, true), thi = m.dv.getUint32(at + 4, true);
      return {kind: 'composite', s, at: at + WORD, n: (tlo | 0) >> 2, dataWords: thi & 0xffff, ptrs: thi >>> 16, words: count};
    }
    return {kind: 'list', s, at, elemSize, count};
  }
  throw new Error('unsupported pointer kind ' + kind);
}

const hexWord = (dv, p) => dv.getBigUint64(p, true).toString(16).padStart(16, '0');

// Generic dump: root struct -> first list pointer -> elements {data: [u64 hex], ptrs: [{data: [u64 hex]}]}.
export function decodeEventQueueRaw(bytes) {
  const buf = bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes);
  const m = segments(buf);
  const root = readPointer(m, 0, m.segs[0].start);
  if (!root || root.kind !== 'struct') throw new Error('root is not a struct');
  let list = null, listPtrIndex = null;
  for (let k = 0; k < root.ptrs && !list; k++) {
    const q = readPointer(m, root.s, root.at + (root.dataWords + k) * WORD);
    if (q) { list = q; listPtrIndex = k; }
  }
  if (!list) return {root: {dataWords: root.dataWords, ptrs: root.ptrs}, listPtrIndex: null, elements: []};
  if (list.kind !== 'composite') throw new Error('event queue is not a composite list');
  const stride = (list.dataWords + list.ptrs) * WORD;
  const elements = [];
  for (let e = 0; e < list.n; e++) {
    const base = list.at + e * stride;
    const data = [];
    for (let w = 0; w < list.dataWords; w++) data.push(hexWord(m.dv, base + w * WORD));
    const ptrs = [];
    for (let k = 0; k < list.ptrs; k++) {
      const q = readPointer(m, list.s, base + (list.dataWords + k) * WORD);
      if (!q) { ptrs.push(null); continue; }
      const d = [];
      if (q.kind === 'struct') for (let w = 0; w < q.dataWords; w++) d.push(hexWord(m.dv, q.at + w * WORD));
      ptrs.push({kind: q.kind, dataWords: q.dataWords, ptrs: q.ptrs, data: d, at: q.at});
    }
    elements.push({base, data, ptrs});
  }
  return {root: {dataWords: root.dataWords, ptrs: root.ptrs}, listPtrIndex, elementDataWords: list.dataWords, elementPtrs: list.ptrs, elements, m};
}

// Field layout, VERIFIED from recorded bytes (dev run flip6-s3-lowtex/rindev1, 2026-10-16): element = 4 data words +
// 1 pointer; word 0 low u16 = kind (PositionalSensorKind; 1 ACCELEROMETER, 2 GYROSCOPE, 4 LINEAR_ACCELERATION as seen),
// words 1..3 = timestampNanos, eventTimestampNanos, intervalNanos (int64); pointer 0 = value struct, float32 x, y, z at
// data bytes 0, 4, 8 (e.g. accelIncludingGravity (0.3, 8.0, 6.0) of the record).
export function decodeEventQueue(bytes) {
  const raw = decodeEventQueueRaw(bytes);
  const dv = raw.m ? raw.m.dv : null;
  const out = [];
  for (const e of raw.elements) {
    const b = e.base;
    const v = e.ptrs[0];
    const num = p => Number(dv.getBigInt64(p, true));
    out.push({kind: dv.getUint16(b, true), timestampNanos: num(b + 8), eventTimestampNanos: num(b + 16), intervalNanos: num(b + 24),
      x: v ? dv.getFloat32(v.at, true) : 0, y: v ? dv.getFloat32(v.at + 4, true) : 0, z: v ? dv.getFloat32(v.at + 8, true) : 0});
  }
  return out;
}
