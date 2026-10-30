// node --test slam_refact/web-chunk/tests/or1.test.mjs
// Unit tests of the OR-1 replay harness (slam_refact/web-chunk/tools/or1/): the canonical forms, the comparators, the bundle derivation, the fake
// clock and the mutants. No engine, no GPU, no network; the engine-level runs are `or1.mjs selftest` (evidence build/.../evidence/OR-1/).
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import { pathToFileURL } from 'node:url';

// SLAM_REFACT_OR1_TOOLS_DIR: a modified copy of tools/or1 (tests/run-or1-mutants.mjs shows that these tests fail on it)
const toolsDir = process.env.SLAM_REFACT_OR1_TOOLS_DIR ?? path.join(import.meta.dirname, '../tools/or1');
const load = (f) => import(pathToFileURL(path.join(toolsDir, f)).href);
const { canon, diffPaths, f64hex, hexToF64, sha256, ulpDistance } = await load('canon.mjs');
const { compareAnswers, compareInputs, compareOutputs, quatAngleDeg } = await load('compare.mjs');
const { derive, readBundle } = await load('bundle.mjs');
const { CHUNK_MUTANTS, NATIVE_MUTANTS, makeChunkMutant, makeNativeMutant } = await load('mutants.mjs');
const { fakeGl, msForNs, secForNs } = await load('players.mjs');

const repo = path.resolve(import.meta.dirname, '../../..');
const scratch = path.join(repo, 'build/slam-refact-full-20261019/evidence/OR-1', `unit-tmp-${process.pid}`);
fs.mkdirSync(scratch, { recursive: true });
let n = 0;
const fresh = (tag) => path.join(scratch, `${tag}-${process.pid}-${++n}`);

test('OR1-C1: canonical forms are bit exact (signed zero, NaN payload kept apart from numbers, one ulp visible)', () => {
  assert.equal(f64hex(1), '3ff0000000000000');
  assert.notEqual(f64hex(0), f64hex(-0));
  assert.equal(hexToF64(f64hex(Math.PI)), Math.PI);
  assert.notEqual(f64hex(1), f64hex(1 + Number.EPSILON));
  assert.deepEqual(canon({ b: [1, 2], a: new Uint8Array([1, 2, 3]), c: 'x', d: null }), { a: `u8:3:${sha256(new Uint8Array([1, 2, 3]))}`, b: [`f:${f64hex(1)}`, `f:${f64hex(2)}`], c: 'x', d: null });
  assert.deepEqual(diffPaths(canon({ a: { b: [1, 2] } }), canon({ a: { b: [1, 3] } })), ['a.b.1']);
  assert.deepEqual(diffPaths(canon({ a: 1 }), canon({ a: 1 })), []);
  assert.deepEqual(diffPaths(canon({ a: 1 }), canon({ a: 1, extra: 2 })), ['extra']);
  assert.equal(ulpDistance(1, 1), 0);
  assert.equal(ulpDistance(1, 1 + Number.EPSILON), 1);
  assert.equal(ulpDistance(0, -0), 0);
  assert.equal(ulpDistance(Number.NaN, 1), Infinity);
});

test('OR1-C2: quaternion angle is exactly 0 for equal unit quaternions and 90 degrees for a quarter turn about z', () => {
  const q = [0.9727064967155457, 0.2319677472114563, 0.0013331040972843766, 0.005590083077549934];
  assert.equal(quatAngleDeg(q, q) < 1e-12, true, 'conj(q) * q is the identity up to rounding noise of the product');
  const a = [1, 0, 0, 0], b = [Math.SQRT1_2, 0, 0, Math.SQRT1_2];
  assert.ok(Math.abs(quatAngleDeg(a, b) - 90) < 1e-12);
  assert.ok(Math.abs(quatAngleDeg(a, [-b[0], -b[1], -b[2], -b[3]]) - 90) < 1e-12, 'sign of the quaternion does not matter');
});

const row = (index, over = {}) => ({ index, fresh: true, raw: { position: [0, 2, index * 1e-3], rotationWxyz: [1, 0, 0, 0], status: 'NORMAL', reason: 'UNSPECIFIED',
  repeated: false, sourceFrameIndex: index, worldPoints: index, ...over }, pose: canon({ position: [0, 2, index * 1e-3], k: index }) });
const rows = (k) => Array.from({ length: k }, (_, i) => row(i));
const f32Next = (x) => { const b = new Float32Array([x]); new Uint32Array(b.buffer)[0] += 1; return b[0]; };

test('OR1-C3: compareOutputs sees one float32 ulp, one bit of a status, a missing frame and a full-pose difference; identical rows are identical', () => {
  const base = rows(10);
  assert.equal(compareOutputs(base, rows(10), { full: true }).identical, true);
  const withUlp = rows(10); withUlp[4].raw.position[2] = f32Next(withUlp[4].raw.position[2]);
  let c = compareOutputs(withUlp, base);
  assert.equal(c.identical, false); assert.equal(c.firstDiff.index, 4); assert.deepEqual(c.firstDiff.fields, ['position']); assert.equal(c.maxUlp > 0, true);
  const st = rows(10); st[2].raw.status = 'LIMITED';
  assert.deepEqual(compareOutputs(st, base).firstDiff.fields, ['status']);
  const rep = rows(10); rep[3].raw.repeated = true;
  assert.deepEqual(compareOutputs(rep, base).firstDiff.fields, ['repeated']);
  assert.equal(compareOutputs(rows(9), base).identical, false, 'a missing frame');
  assert.equal(compareOutputs(base, rows(9)).identical, false, 'an extra frame');
  const other = rows(10); other[5].pose = canon({ position: [0, 2, 0.005], k: 99 });
  assert.equal(compareOutputs(other, base).identical, true, 'headline fields only without full');
  c = compareOutputs(other, base, { full: true });
  assert.equal(c.identical, false); assert.deepEqual(c.firstDiff.fields, ['k']);
  const dropped = rows(10); dropped[6] = { index: 6, fresh: false };
  assert.equal(compareOutputs(dropped, base).identical, false, 'fresh vs not fresh');
  const zero = rows(10); zero[1].raw.position[0] = 0; const negZero = rows(10); negZero[1].raw.position[0] = -0;
  assert.deepEqual(compareOutputs(negZero, zero).firstDiff.fields, ['position'], '-0 and +0 are different bits');
  const nan = rows(10); nan[1].raw.position[0] = Number.NaN;
  assert.deepEqual(compareOutputs(nan, zero).firstDiff.fields, ['position'], 'NaN against a number');
  const rot = rows(10); rot[7].raw.rotationWxyz = [f32Next(1), 0, 0, 0];
  assert.deepEqual(compareOutputs(rot, base).firstDiff.fields, ['rotationWxyz']);
});

test('OR1-C4: compareInputs compares per-call records in order, including motion lists and the atlas digest', () => {
  const frame = (i, over = {}) => ({ index: i, timeNs: 1e9 + i, videoTimeNs: 2e9 + i, size: 1024, rowsGiven: 650, atlasPrefixSha256: sha256(`a${i}`), atlasPrefixBytes: 99,
    motion: [{ t: 5, aig: [0, 9.8, 0], lin: null, rate: [1, 2, 3], interval: 16.6 }], orientation: { a: 1, b: 2, g: 3 }, orientationTimeNs: 7, ...over });
  const A = [frame(0), frame(1), frame(2)];
  assert.equal(compareInputs(A, [frame(0), frame(1), frame(2)]).identical, true);
  assert.equal(compareInputs(A, [frame(0), frame(1, { orientationTimeNs: 99, rowsGiven: 660 }), frame(2)]).identical, true, 'orientation time and rows given are information, not inputs');
  for (const [what, b, path_] of [
    ['time', frame(1, { timeNs: 1e9 + 2 }), 'timeNs'], ['video time', frame(1, { videoTimeNs: 1 }), 'videoTimeNs'], ['atlas', frame(1, { atlasPrefixSha256: sha256('x') }), 'atlasPrefixSha256'],
    ['orientation', frame(1, { orientation: { a: 2, b: 1, g: 3 } }), 'orientation.a'],
    ['motion value', frame(1, { motion: [{ t: 5, aig: [0, 9.8000000000000001 + Number.EPSILON * 16, 0], lin: null, rate: [1, 2, 3], interval: 16.6 }] }), 'motion.0.aig.1'],
    ['motion count', frame(1, { motion: [] }), 'motion.0'], ['lin', frame(1, { motion: [{ t: 5, aig: [0, 9.8, 0], lin: [0, 0, 0], rate: [1, 2, 3], interval: 16.6 }] }), 'motion.0.lin'],
  ]) {
    const c = compareInputs(A, [frame(0), b, frame(2)]);
    assert.equal(c.identical, false, what);
    assert.ok(c.first.paths.some((p) => p === path_ || p.startsWith(`${path_}.`)), `${what}: ${c.first.paths}`);
  }
  assert.equal(compareInputs(A, A.slice(0, 2)).identical, false, 'a missing call');
});

test('OR1-C5: compareAnswers maps the engine pose to the answer shape (x y z w) and finds a swapped component', () => {
  const r = [row(1), row(2)];
  const ans = (x) => ({ index: x.index, answer: canon({ position: { x: x.raw.position[0], y: x.raw.position[1], z: x.raw.position[2] },
    rotation: { x: x.raw.rotationWxyz[1], y: x.raw.rotationWxyz[2], z: x.raw.rotationWxyz[3], w: x.raw.rotationWxyz[0] }, trackingStatus: x.raw.status,
    trackingReason: x.raw.reason, worldPoints: x.raw.worldPoints }) });
  assert.equal(compareAnswers([{ index: 0, answer: null }, ans(r[0]), ans(r[1])], r).identical, true);
  const bad = ans(r[1]); bad.answer.rotation = { ...bad.answer.rotation, x: bad.answer.rotation.w };
  assert.equal(compareAnswers([ans(r[0]), bad], r).identical, false);
});

test('OR1-C6: the fake clock returns a double whose chunk product truncates to the requested integer (exact when a double exists)', () => {
  let state = 20261020 >>> 0;
  const next = () => { state ^= state << 13; state >>>= 0; state ^= state >>> 17; state ^= state << 5; state >>>= 0; return state; };
  let exact = 0, total = 0;
  for (let i = 0; i < 4000; i++) {
    const ns = 986955000 + (next() % 4e9) * 1000 + (i % 7) * 100 + (next() % 3);
    const m = msForNs(ns), v = secForNs(ns);
    assert.equal(m.truncMatches, true, `ms ${ns}`); assert.equal(v.truncMatches, true, `s ${ns}`);
    assert.equal(Math.trunc(1e3 * m.value * 1e3), ns); assert.equal(Math.trunc(v.value * 1e9), ns);
    if (m.exact) assert.equal(1e3 * m.value * 1e3, ns);
    total++; if (m.exact) exact++;
  }
  assert.ok(exact > total * 0.5, `exact for ${exact}/${total}`);
  // an independent search (bit increments) for a neighbouring double whose product is exactly ns: the fake clock must find one whenever it exists
  const bits = new BigInt64Array(1), dbl = new Float64Array(bits.buffer);
  const hasExactNeighbour = (c, ok) => { dbl[0] = c; const b0 = bits[0]; for (let k = -4096n; k <= 4096n; k++) { bits[0] = b0 + k; if (ok(dbl[0])) return true; } return false; };
  let neighbourOnly = 0;
  for (let i = 0; i < 300; i++) {
    const ns = 1013300000 + i * 33333 + (i % 11);
    const wantMs = hasExactNeighbour(ns / 1e6, (x) => 1e3 * x * 1e3 === ns), wantS = hasExactNeighbour(ns / 1e9, (x) => x * 1e9 === ns);
    assert.equal(msForNs(ns).exact, wantMs, `ms ${ns}`); assert.equal(secForNs(ns).exact, wantS, `s ${ns}`);
    if (wantMs && 1e3 * (ns / 1e6) * 1e3 !== ns) neighbourOnly++;
  }
  assert.ok(neighbourOnly > 0, 'the test data contain values whose only exact doubles are neighbours of ns / 1e6');
});

test('OR1-C7: the fake GL answers the pixel pack buffer read with the current atlas and refuses a read without one', () => {
  const state = { atlas: null };
  const gl = fakeGl(state);
  assert.throws(() => gl.getBufferSubData(gl.PIXEL_PACK_BUFFER, 0, new Uint8Array(4)), /no atlas set/);
  state.atlas = new Uint8Array([1, 2, 3, 4, 5, 6]);
  const dst = new Uint8Array(4);
  gl.getBufferSubData(gl.PIXEL_PACK_BUFFER, 0, dst);
  assert.deepEqual(Array.from(dst), [1, 2, 3, 4]);
  assert.throws(() => gl.getBufferSubData(gl.PIXEL_PACK_BUFFER, 0, new Uint8Array(9)), /< readback/);
  assert.equal(gl.clientWaitSync({}, 0, 0), gl.ALREADY_SIGNALED);
  assert.equal(gl.checkFramebufferStatus(gl.FRAMEBUFFER), gl.FRAMEBUFFER_COMPLETE);
});

// ---- bundle derivation on a tiny synthetic bundle ----
function syntheticBundle(dir) {
  fs.mkdirSync(dir, { recursive: true });
  fs.writeFileSync(path.join(dir, 'engine-config.tsv'), ['scenario\tx', 'capture_width\t480', 'capture_height\t640', 'rotation_deg\t0', 'manufacturer\t', 'model\tK', 'estimate_scale\t0',
    'disable_vio\t0', 'pyramid_model\tswiftshader-approx', 'origin\t0 2 0', 'facing_wxyz\t1 0 0 0', 'left_handed_axes\t0', 'mirrored_display\t0', 'world_point_sample_every\t50',
    'id_generator_seed\t123'].map((l) => `${l}\n`).join(''));
  const sensor = (kind, t) => [kind, t, 0, 9.8, 0, '-', '-', '-', 1, 2, 3, 16.6, 10, 20, 30].join('\t');
  fs.writeFileSync(path.join(dir, 'sensors.tsv'), [sensor('bootstrap', 90), sensor('run', 100), sensor('run', 150), sensor('run', 250), sensor('run', 400)].map((l) => `${l}\n`).join(''));
  const frame = (kind, i, t, proc, pyr, ft) => [kind, i, t, t, t / 1e9, proc, `/img/${i}.pgm`, pyr, ...(ft === undefined ? [] : [ft])].join('\t');
  fs.writeFileSync(path.join(dir, 'frames.tsv'), [frame('bootstrap', -1, 90, 1, 'a0.gz', 80), frame('frame', 0, 200, 1, 'a1.gz', 190), frame('frame', 1, 300, 1, '-', 290), frame('frame', 2, 450, 0, 'a3.gz', 440),
    frame('frame', 3, 500, 1, 'a4.gz', 490)].map((l) => `${l}\n`).join(''));
}

test('OR1-B1: derive drops the id seed, keeps the frame-time column, disables processed frames without an atlas, truncates, and is deterministic', () => {
  const src = fresh('src'); syntheticBundle(src);
  const a = derive(src, fresh('d'), { frames: 3, sourceLabel: 'synthetic' });
  assert.deepEqual(a.droppedConfigKeys, ['id_generator_seed']);
  assert.equal(a.frameTimeColumn, true);
  assert.deepEqual(a.framesWithoutAtlasDisabled, [1]);
  assert.equal(a.frames, 3); assert.equal(a.truncatedTo, 3); assert.equal(a.source, 'synthetic');
  const b = readBundle(a.outDir);
  assert.equal(b.config.has('id_generator_seed'), false);
  assert.deepEqual(b.frames.map((f) => [f.index, f.process, f.frameTimeNs]), [[-1, true, 80], [0, true, 190], [1, false, 290], [2, false, 440]], 'column 9 (func 647 time) is part of the recorded input');
  assert.equal(b.sensors.length, 5, 'the bootstrap row and every run row up to the time of the last kept frame (450)');
  const again = derive(src, fresh('d'), { frames: 3, sourceLabel: 'synthetic' });
  assert.deepEqual(again.derivedSha256, a.derivedSha256);
  assert.throws(() => derive(src, a.outDir), /refusing to overwrite/);
  // the source is read-only: unchanged
  assert.equal(readBundle(src).config.get('id_generator_seed'), '123');
  assert.equal(readBundle(src).frames[1].frameTimeNs, 190);
});

test('OR1-M1: every chunk mutant pattern occurs exactly once in the current chunk files and changes exactly one file; native mutants change their bundle', () => {
  const chunk = path.join(repo, 'slam_refact/web-chunk');
  const baseline = chunk;
  for (const [id] of CHUNK_MUTANTS) {
    const out = fresh('mut');
    const dir = makeChunkMutant(id, baseline, out);
    const changed = fs.readdirSync(dir).filter((f) => f.endsWith('.js') && fs.readFileSync(path.join(dir, f), 'utf8') !== fs.readFileSync(path.join(baseline, f), 'utf8'));
    assert.equal(changed.length, 1, `${id}: changed ${changed.join(',')}`);
  }
  assert.throws(() => makeChunkMutant('nope', chunk, fresh('mut')), /unknown chunk mutant/);
  // a pattern that is not (or is twice) in the file must stop the run instead of producing an unmodified "mutant"
  const stale = fresh('stale'); fs.mkdirSync(stale, { recursive: true });
  for (const f of fs.readdirSync(chunk).filter((x) => x.endsWith('.js'))) fs.copyFileSync(path.join(chunk, f), path.join(stale, f));
  const sensors = fs.readFileSync(path.join(stale, 'sensors.js'), 'utf8');
  fs.writeFileSync(path.join(stale, 'sensors.js'), sensors.replace('const staged = eventQueue.slice();', 'const staged = eventQueue.slice(0);'));
  assert.throws(() => makeChunkMutant('X2-first-motion-event-lost', stale, fresh('mut')), /pattern occurs 0 times/);
  fs.writeFileSync(path.join(stale, 'sensors.js'), `${sensors}\n// const staged = eventQueue.slice();\n`);
  assert.throws(() => makeChunkMutant('X2-first-motion-event-lost', stale, fresh('mut')), /pattern occurs 2 times/);
  const src = fresh('src2'); syntheticBundle(src);
  const base = derive(src, fresh('d2'), {});
  for (const [id] of NATIVE_MUTANTS) {
    const dir = makeNativeMutant(id, base.outDir, fresh('nm'));
    const diff = ['sensors.tsv', 'frames.tsv'].filter((f) => fs.readFileSync(path.join(dir, f), 'utf8') !== fs.readFileSync(path.join(base.outDir, f), 'utf8'));
    assert.equal(diff.length, 1, `${id}: ${diff}`);
  }
});

test.after(() => { fs.rmSync(scratch, { recursive: true, force: true }); });
