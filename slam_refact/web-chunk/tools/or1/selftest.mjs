// OR-1 self-test on DEVELOPMENT data only: null-runs (no defect: every gate must say identical) and mutation runs (a defect: a gate must
// say different). Nothing here reads the registered gate data (hold.mjs lists it; the dev sources below are not in that list).
//   node slam_refact/web-chunk/tools/or1/or1.mjs selftest --out build/slam-refact-full-20261019/evidence/OR-1/selftest-<label>
import fs from 'node:fs';
import path from 'node:path';
import { derive } from './bundle.mjs';
import { runAll, evaluate, summarize } from './run.mjs';
import { CHUNK_MUTANTS, NATIVE_MUTANTS, makeChunkMutant, makeNativeMutant } from './mutants.mjs';
import { compareOutputs } from './compare.mjs';
import { canon, f64hex } from './canon.mjs';

export const DEV_SOURCES = [
  { name: 'D1-flip6-s2-pmr3a', src: 'build/slam-refact-20261016/fix/round2/bundles-seed/flip6-s2/pmr3a', frames: 300, chunkMutants: 'all' },
  { name: 'D2-tum-room1-full-pmr3a', src: 'build/slam-refact-20261016/fix/round2/bundles-seed/tum-room1-full/pmr3a', frames: 300,
    chunkMutants: ['X0-noop-comment', 'X1-orientation-alpha-beta-swapped', 'X3-frame-time-plus-1ns', 'X5-atlas-one-bit-flipped', 'X6-answer-rotation-x-y-swapped', 'X8-one-frame-not-staged', 'X10-acceleration-fields-swapped', 'X11-engine-time-is-stage-time'] },
];
export const PAIR = 'build/slam-refact-or1-20261020/pair.json';

const flagged = (g) => ({ 'IN-CW': g['IN-CW']?.identical === false, 'OUT-CW': g['OUT-CW']?.identical === false, ANS: g.ANS?.identical === false,
  NW: g.NW?.identical === false, CN: g.CN?.identical === false });
const anyChunkGate = (f) => f['IN-CW'] || f['OUT-CW'] || f.ANS;

// one-ulp (float32) / one-bit changes of recorded output rows: the resolution of the output comparators
function f32Next(x) { const b = new Float32Array([x]); const u = new Uint32Array(b.buffer); u[0] += x >= 0 ? 1 : -1; return b[0]; }
function rowMutations(rows) {
  const r = rows.find((x) => x.fresh && x.raw.position.some((v) => v !== 0)) ?? rows.find((x) => x.fresh);
  const clone = () => rows.map((x) => (x.fresh ? { ...x, raw: { ...x.raw, position: [...x.raw.position], rotationWxyz: [...x.raw.rotationWxyz] }, pose: x.pose } : x));
  const out = [];
  const mutate = (id, fn) => { const m = clone(); const t = m.find((x) => x.index === r.index); fn(t); const c = compareOutputs(m, rows); out.push({ id, index: r.index, detected: !c.identical, firstDiff: c.firstDiff }); };
  mutate('position-x-one-float32-ulp', (t) => { t.raw.position[0] = f32Next(t.raw.position[0]); });
  mutate('rotation-w-one-float32-ulp', (t) => { t.raw.rotationWxyz[0] = f32Next(t.raw.rotationWxyz[0]); });
  mutate('position-z-one-double-ulp', (t) => { const b = new Float64Array([t.raw.position[2]]); new BigUint64Array(b.buffer)[0] += 1n; t.raw.position[2] = b[0]; });
  mutate('status-string', (t) => { t.raw.status = t.raw.status === 'NORMAL' ? 'LIMITED' : 'NORMAL'; });
  mutate('repeated-flag', (t) => { t.raw.repeated = !t.raw.repeated; });
  mutate('sourceFrameIndex-plus-1', (t) => { t.raw.sourceFrameIndex += 1; });
  return out;
}

export async function selftest({ root, out }) {
  if (fs.existsSync(out)) throw new Error(`refusing to overwrite ${out}`);
  fs.mkdirSync(out, { recursive: true });
  const pair = JSON.parse(fs.readFileSync(path.join(root, PAIR), 'utf8'));
  const engineDir = path.join(root, path.dirname(pair.wasm.file));
  const chunkSrc = path.join(root, 'slam_refact/web-chunk');
  const result = { schema: 'or1-selftest-v1', at: new Date().toISOString(), pair, datasets: {}, summary: {} };
  let mutantsRun = 0, mutantsDetected = 0, equivalentFalseAlarms = 0, nullFailures = 0;
  for (const ds of DEV_SOURCES) {
    const dir = path.join(out, ds.name);
    const d = derive(path.join(root, ds.src), path.join(dir, 'bundle'), { frames: ds.frames, sourceLabel: ds.src });
    const common = { bundleDir: path.join(dir, 'bundle'), engineDir, nativeBin: pair.native_replay.file, repoRoot: root, chunkDir: chunkSrc };
    const entry = { derived: { frames: d.frames, sensorRows: d.sensorRows, droppedConfigKeys: d.droppedConfigKeys, framesWithoutAtlasDisabled: d.framesWithoutAtlasDisabled } };
    // ---- null-run: all three arms, two repetitions each
    const nullRun = await runAll({ ...common, outDir: path.join(dir, 'null'), reps: 2, arms: ['W', 'C', 'N'] });
    const g0 = evaluate(nullRun);
    entry.null = { verdict: g0.verdict, summary: summarize(g0), inputStats: g0.inputStats, wallMs: g0.wallMs, clock: g0.clock, NW: { maxPositionDelta: g0.NW.maxPositionDelta, maxUlp: g0.NW.maxUlp }, chunkErrors: g0.chunkErrors };
    if (!Object.values(g0.verdict).every((v) => v === true)) nullFailures++;
    const W = nullRun.runs.W, N = nullRun.runs.N, C = nullRun.runs.C;
    entry.rowMutations = rowMutations(W[0].rows);
    // ---- chunk mutants (arm C replaced; W and N reused from the null-run)
    entry.chunkMutants = {};
    const ids = ds.chunkMutants === 'all' ? CHUNK_MUTANTS.map((m) => m[0]) : ds.chunkMutants;
    for (const id of ids) {
      const chunkDir = makeChunkMutant(id, chunkSrc, path.join(dir, 'mutants'));
      const r = await runAll({ ...common, chunkDir, outDir: path.join(dir, 'mut-' + id), reps: 1, arms: ['C'] });
      const g = evaluate({ runs: { W: [W[0]], N: [N[0]], C: r.runs.C } });
      const f = flagged(g);
      const equivalent = id === 'X0-noop-comment';
      entry.chunkMutants[id] = { models: CHUNK_MUTANTS.find((m) => m[0] === id)[4], flagged: f, detectedByChunkGate: anyChunkGate(f), calls: [g['IN-CW'].countA, g['IN-CW'].countB],
        inputMismatchedCalls: g['IN-CW'].mismatches, outputMismatchedFrames: g['OUT-CW'].mismatchedFrames, answerMismatches: g.ANS.mismatches, firstOutputDiff: g['OUT-CW'].firstDiff,
        outputFrames: g['OUT-CW'].frames, maxPositionDelta: g['OUT-CW'].maxPositionDelta, maxRotationDeltaDeg: g['OUT-CW'].maxRotationDeltaDeg };
      if (equivalent) { if (anyChunkGate(f) || f.CN) equivalentFalseAlarms++; } else { mutantsRun++; if (anyChunkGate(f)) mutantsDetected++; }
    }
    // ---- native-side mutants (arm N replaced; W and C reused)
    entry.nativeMutants = {};
    for (const [id, what] of NATIVE_MUTANTS) {
      const nb = makeNativeMutant(id, path.join(dir, 'bundle'), path.join(dir, 'nmutants'));
      const r = await runAll({ ...common, nativeBundleDir: nb, outDir: path.join(dir, 'nmut-' + id), reps: 1, arms: ['N'] });
      const g = evaluate({ runs: { W: [W[0]], C: [C[0]], N: r.runs.N } });
      entry.nativeMutants[id] = { models: what, NW: g.NW.identical === false, CN: g.CN.identical === false, firstDiff: g.NW.firstDiff, maxPositionDelta: g.NW.maxPositionDelta, maxRotationDeltaDeg: g.NW.maxRotationDeltaDeg,
        mismatchedFrames: g.NW.mismatchedFrames };
    }
    result.datasets[ds.name] = entry;
    fs.writeFileSync(path.join(out, 'partial-' + ds.name + '.json'), `${JSON.stringify(entry, null, 1)}\n`);
  }
  const rowAll = Object.values(result.datasets).flatMap((e) => e.rowMutations);
  result.summary = { nullFailures, chunkMutantsRun: mutantsRun, chunkMutantsDetectedByAChunkGate: mutantsDetected, equivalentMutantFalseAlarms: equivalentFalseAlarms,
    rowMutations: rowAll.length, rowMutationsDetected: rowAll.filter((x) => x.detected).length,
    nativeMutantsDetectedByNW: Object.values(result.datasets).flatMap((e) => Object.values(e.nativeMutants)).filter((x) => x.NW).length,
    nativeMutants: Object.values(result.datasets).flatMap((e) => Object.values(e.nativeMutants)).length,
    passes: nullFailures === 0 && mutantsDetected === mutantsRun && equivalentFalseAlarms === 0 && rowAll.every((x) => x.detected) };
  fs.writeFileSync(path.join(out, 'selftest.json'), `${JSON.stringify(result, null, 1)}\n`);
  console.log(JSON.stringify(result.summary, null, 1));
  process.exitCode = result.summary.passes ? 0 : 1;
  return result;
}
