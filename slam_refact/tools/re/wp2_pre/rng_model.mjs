// Reference model of the FrameFrameTracker RNG usage in the original xr-slam.wasm (WP2-PRE item 1,
// spec S4 U6). Pure JavaScript, no dependencies. Every function here mirrors one decoded routine:
//
//   Mt19937            std::mt19937 (libc++ layout: uint32 x[624], uint32 i at +2496).
//                      Seeding loop = func 640 (MapTracker ctor) at MapTracker+0x1e0 (FFT+0x158).
//   uid32(g, a, b)     libc++ uniform_int_distribution<32-bit>::operator()(g, param(a, b)), closed
//                      [a, b], independent_bits_engine rejection sampling = func 805 (inline param)
//                      and func 895 (param_type&). Full range (b - a + 1 == 0 mod 2^32) returns raw.
//   nextUnsignedInt(g) c8::RandomNumbers::uniform32i(rng) with param (0, 0xffffffff) stored at
//                      RandomNumbers+2524 = raw mt19937 output (func 895, Rp == 0 path).
//   shuffle(g, arr)    libc++ std::__shuffle: forward Fisher-Yates, i = uid32(0, d) with
//                      d = n-1, n-2, ..., 1; swap(a[k], a[k+i]) only when i != 0 (funcs 926/1266/1279
//                      loops around call 805). difference_type is int32 on wasm32.
//   fftDepthPairs(n)   1266 "depth-egomotion": all (i<j), i outer, then shuffle.
//   groundPairs(n)     1279 "propose-homography" pair list (RobustHomographyScratchSpace.hSampleInds).
//   groundSamples(n)   1279 point-sample index list (ptSampleInds); first min(n,100) are used.
//
// CLI: node rng_model.mjs --vectors OUT.json   writes deterministic test vectors.

export class Mt19937 {
  constructor(seed = 5489) {
    this.x = new Uint32Array(624);
    this.i = 0;
    this.x[0] = seed >>> 0;
    for (let k = 1; k < 624; k++) {
      const p = this.x[k - 1];
      this.x[k] = (Math.imul(1812433253, (p ^ (p >>> 30)) >>> 0) + k) >>> 0;
    }
    this.draws = 0;
  }

  next() {
    const i = this.i;
    const j = (i + 1) % 624;
    const y = ((this.x[i] & 0x80000000) | (this.x[j] & 0x7ffffffe)) >>> 0;
    let z = (this.x[(i + 397) % 624] ^ (y >>> 1) ^ ((this.x[j] & 1) ? 0x9908b0df : 0)) >>> 0;
    this.x[i] = z;
    this.i = j;
    z ^= z >>> 11;
    z = (z ^ ((z << 7) & 0x9d2c5680)) >>> 0;
    z = (z ^ ((z << 15) & 0xefc60000)) >>> 0;
    z = (z ^ (z >>> 18)) >>> 0;
    this.draws++;
    return z;
  }

  clone() {
    const c = Object.create(Mt19937.prototype);
    c.x = new Uint32Array(this.x);
    c.i = this.i;
    c.draws = this.draws;
    return c;
  }
}

const clz32 = Math.clz32;

// libc++ uniform_int_distribution<T> with 32-bit T (unsigned arithmetic), closed range [a, b].
export function uid32(g, a, b) {
  const rp = ((b >>> 0) - (a >>> 0) + 1) >>> 0;
  if (rp === 1) return a;
  if (rp === 0) return g.next();
  // w = number of bits needed; ++w when rp is not a power of two.
  let w = 32 - clz32(rp) - 1;
  if ((rp & (0xffffffff >>> (32 - w))) !== 0) w++;
  // __independent_bits_engine with one 32-bit word: mask = ~0u >> (32 - w).
  const mask = w === 32 ? 0xffffffff : (0xffffffff >>> (32 - w));
  let u;
  do {
    u = (g.next() & mask) >>> 0;
  } while (u >= rp);
  return (u + a) | 0;
}

export const nextUnsignedInt = (g) => g.next();

// libc++ std::shuffle (forward). arr is mutated; returns arr.
export function shuffle(g, arr) {
  const n = arr.length;
  if (n < 2) return arr;
  for (let k = 0, d = n - 1; d > 0; k++, d--) {
    const i = uid32(g, 0, d);
    if (i !== 0) {
      const t = arr[k];
      arr[k] = arr[k + i];
      arr[k + i] = t;
    }
  }
  return arr;
}

export function allPairs(n) {
  const out = [];
  for (let i = 0; i + 1 < n; i++) for (let j = i + 1; j < n; j++) out.push([i, j]);
  return out;
}

// 1266: pairs over all depth matches, shuffled. The tracker then iterates them in this order.
export function fftDepthPairs(g, n) {
  return shuffle(g, allPairs(n));
}

export const kGroundMaxPairs = 73;      // G1279:283-287
export const kGroundMinPoints = 15;     // homography.cc MIN_POINTS_FOR_ROBUST_HOMOGRAPHY
export const kGroundMaxSamples = 100;   // homography.cc MAX_POINT_SAMPLES_FOR_ROBUST_HOMOGRAPHY

// 1279 pair list. Returns {pairs, path, attempts}.
export function groundPairs(g, n) {
  if (n < kGroundMinPoints) return { pairs: [], path: 'none', attempts: 0 };
  const total = ((n - 1) * n) >>> 1;          // u32 arithmetic in the binary
  const nH = Math.min(total, kGroundMaxPairs);
  if (total <= 5 * nH) {
    const p = shuffle(g, allPairs(n));
    return { pairs: p.slice(0, nH), path: 'enumerate-shuffle', attempts: 0 };
  }
  const set = new Map();
  let attempts = 0;
  while (set.size < nH) {
    const r1 = (g.next() % n) >>> 0;          // i32.rem_u
    const r2 = (g.next() % n) >>> 0;
    attempts++;
    if (r1 === r2) continue;
    const lo = Math.min(r1, r2), hi = Math.max(r1, r2);
    set.set(lo * 65536 + hi, [lo, hi]);
  }
  // std::set<pair<size_t,size_t>> iteration order = lexicographic (lo, hi).
  const pairs = [...set.entries()].sort((a, b) => a[0] - b[0]).map((e) => e[1]);
  return { pairs, path: 'random-set', attempts };
}

// 1279 point samples: identity order when n <= 100, else shuffle(0..n-1) and keep the first 100.
export function groundSamples(g, n) {
  if (n < kGroundMinPoints) return { used: [], shuffled: null };
  if (n <= kGroundMaxSamples) return { used: [...Array(n).keys()], shuffled: null };
  const idx = shuffle(g, [...Array(n).keys()]);
  return { used: idx.slice(0, kGroundMaxSamples), shuffled: idx };
}

// Full 1279 RNG consumption for one call (pairs first, then samples).
export function groundCall(g, n) {
  const p = groundPairs(g, n);
  const s = groundSamples(g, n);
  return { ...p, samples: s.used, sampleShuffle: s.shuffled };
}

// ---------- test vectors ----------
export function makeVectors() {
  const v = { seed: 5489, notes: 'Generated by slam_refact/tools/re/wp2_pre/rng_model.mjs' };
  let g = new Mt19937();
  v.mt_first10 = Array.from({ length: 10 }, () => g.next());
  g = new Mt19937();
  for (let k = 0; k < 9999; k++) g.next();
  v.mt_10000th = g.next();                  // std: 4123659995
  g = new Mt19937();
  v.uid32_cases = [];
  for (const [a, b] of [[0, 1], [0, 2], [0, 6], [0, 7], [0, 8], [0, 72], [0, 99], [0, 364], [0, 4949], [5, 9], [0, 0x7fffffff]]) {
    const draws0 = g.draws;
    const out = Array.from({ length: 8 }, () => uid32(g, a, b));
    v.uid32_cases.push({ a, b, out, raw_draws: g.draws - draws0 });
  }
  g = new Mt19937();
  v.shuffle_cases = [1, 2, 3, 5, 10, 37].map((n) => {
    const d0 = g.draws;
    const out = shuffle(g, [...Array(n).keys()]);
    return { n, out, raw_draws: g.draws - d0 };
  });
  g = new Mt19937();
  v.fft_depth_pairs = [10, 12].map((n) => {
    const d0 = g.draws;
    const p = fftDepthPairs(g, n);
    return { n, first12: p.slice(0, 12), count: p.length, raw_draws: g.draws - d0 };
  });
  v.ground_calls = [];
  for (const n of [14, 15, 20, 27, 28, 40, 100, 101, 150]) {
    const gg = new Mt19937();
    const r = groundCall(gg, n);
    v.ground_calls.push({ n, path: r.path, attempts: r.attempts, pairs: r.pairs, samples_first10: r.samples.slice(0, 10),
      n_samples: r.samples.length, raw_draws: gg.draws, mt_index_after: gg.i });
  }
  return v;
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const a = process.argv.slice(2);
  if (a.length !== 2 || a[0] !== '--vectors') throw Error('Usage: node rng_model.mjs --vectors OUT.json');
  const fs = await import('node:fs');
  if (fs.existsSync(a[1])) throw Error('Output exists; preserving evidence');
  const v = makeVectors();
  if (v.mt_10000th !== 4123659995) throw Error('mt19937 self-check failed (10000th output)');
  fs.writeFileSync(a[1], JSON.stringify(v, null, 1));
  console.log(`wrote ${a[1]} (mt19937 10000th = ${v.mt_10000th}, OK)`);
}
