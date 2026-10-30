// or1-chunk-replay-gate.v4: the null-run and mutation evidence of the registration (protocol sections 3 and 4), on DEVELOPMENT data only.
//   node selftest-v4.mjs --pair <pair.json> --out <NEW dir under build/> [--chunk-dir <dir>] [--rotation-frames 30] [--portrait-frames 300] [--skip-portrait]
// The evidence the manifest cites (`null_run`, `mutation`) is the run of this tool on the F3 pair (frozen engine, frozen chunk); a run on another pair is a development
// smoke and is not the registration's evidence. Nothing here is the registered gate data: the portrait datasets are the development bundles D1 / D2 of the v3 self-test,
// the rotation datasets are the first N frames of the four orient recordings that the gate also runs (a PREFIX of the gate data: the gate's power rests on the mutation
// detection below, not on the novelty of its data, and the registration says so).
//
// null-run: W, C and N, two repetitions each, on every dataset: every comparison must say identical (the measurement noise floor of an exact comparison is 0).
// mutants (a defect injected into a COPY of the chunk, never the repo file): the v3 list on the portrait data, and the rotation list (RX*) on the rotation data. A mutant
// is detected when the gate would not PASS: a graded comparison differs (FAIL), or the chunk arm cannot be played at all (a crash, which the gate reports INVALID, never PASS).
// The list marks which mutants are in the gate's claim; the ones outside it (the GL-side rotation uniforms, which the fake GL cannot see) are reported and must be NOT detected
// or detected without anyone relying on it: they are not counted.
import fs from 'node:fs';
import path from 'node:path';
import { parseArgs } from 'node:util';
import { derive } from '../../../web-chunk/tools/or1/bundle.mjs';
import { runAll, evaluate, summarize } from '../../../web-chunk/tools/or1/run.mjs';
import { CHUNK_MUTANTS, NATIVE_MUTANTS, makeChunkMutant, makeNativeMutant } from '../../../web-chunk/tools/or1/mutants.mjs';
import { compareOutputs } from '../../../web-chunk/tools/or1/compare.mjs';
import { prepare } from './prepare-orient-bundle.mjs';
import { SESSION_KEYS } from './judge.mjs';

const root = path.resolve(import.meta.dirname, '../../../..');
const REC = 'build/slam-refact-full-20261019/orient/original/flip6-s2';
export const PORTRAIT_DEV = [
  { name: 'D1-flip6-s2-pmr3a', src: 'build/slam-refact-20261016/fix/round2/bundles-seed/flip6-s2/pmr3a', mutants: 'all' },
  { name: 'D2-tum-room1-full-pmr3a', src: 'build/slam-refact-20261016/fix/round2/bundles-seed/tum-room1-full/pmr3a',
    mutants: ['X0-noop-comment', 'X1-orientation-alpha-beta-swapped', 'X3-frame-time-plus-1ns', 'X5-atlas-one-bit-flipped', 'X6-answer-rotation-x-y-swapped', 'X8-one-frame-not-staged', 'X10-acceleration-fields-swapped', 'X11-engine-time-is-stage-time'] },
];
// rotation data: the orient recordings (name, directory, the rotation the original passed to engineInit)
export const ROTATION_DEV = [['T0-orient-o0', 'v2-b-o0', 0], ['T90-orient-o90', 'v2-b-o90', 90], ['T180-orient-o180', 'v2-b-o180', 180], ['T270-orient-o270', 'v2-b-o270', -90]];

// the rotation mutants: [id, file, from, to, what it models, inGateClaim, applies to rotations]. `from` occurs exactly once in the file.
export const ROTATION_MUTANTS = [
  ['RX1-glue-rotation-zero', 'xr-slam.js', 'captureWidth: videoWidth, captureHeight: videoHeight, rotationDeg: rotation,', 'captureWidth: videoWidth, captureHeight: videoHeight, rotationDeg: 0,',
    'the session builds its glue without the capture rotation', true],
  ['RX2-capture-size-swapped', 'xr-slam.js', 'captureWidth: videoWidth, captureHeight: videoHeight, rotationDeg: rotation,', 'captureWidth: videoHeight, captureHeight: videoWidth, rotationDeg: rotation,',
    'width and height exchanged in the glue config', true],
  ['RX3-glue-rotation-sign-flipped', 'xr-slam.js', 'captureWidth: videoWidth, captureHeight: videoHeight, rotationDeg: rotation,', 'captureWidth: videoWidth, captureHeight: videoHeight, rotationDeg: -rotation,',
    'the rotation sign lost on its way to the engine', true],
  ['RX4-attach-rotation-zero', 'xr-slam.js', 'createSession({ videoWidth, videoHeight, rotation, computeCtx: computeCtx_, coords: configured_, framesUntilRecenter: startupFramesLeft_ })',
    'createSession({ videoWidth, videoHeight, rotation: 0, computeCtx: computeCtx_, coords: configured_, framesUntilRecenter: startupFramesLeft_ })', 'onSessionAttach forgets the rotation', true],
  ['RX5-rotation-90-refused', 'xr-slam.js', 'const SUPPORTED_ROTATIONS = [0, 90, 180, 270, -90, -180, -270];', 'const SUPPORTED_ROTATIONS = [0, 180, 270, -90, -180, -270];', 'rotation 90 not in the supported set', true],
  ['RX6-rotation-minus-90-refused', 'xr-slam.js', 'const SUPPORTED_ROTATIONS = [0, 90, 180, 270, -90, -180, -270];', 'const SUPPORTED_ROTATIONS = [0, 90, 180, 270, -180, -270];', 'rotation -90 (a 270 degree device orientation) not supported', true],
  ['RX7-rotation-180-refused', 'xr-slam.js', 'const SUPPORTED_ROTATIONS = [0, 90, 180, 270, -90, -180, -270];', 'const SUPPORTED_ROTATIONS = [0, 90, 270, -90, -180, -270];', 'rotation 180 not supported', true],
  ['RX8-landscape-refused', 'xr-slam.js', '(!SUPPORTED_ROTATIONS.includes(rotation)', '(!SUPPORTED_ROTATIONS.includes(rotation) || videoWidth > videoHeight', 'the portrait-only guard (R16) back', true],
  ['RX11-one-frame-not-staged-20th', 'xr-slam.js', 'if (repeatFrame || cameraProcessingDisabled_) return;\n    maybeDispatchConfigChange(framework);',
    'if (repeatFrame || cameraProcessingDisabled_) return;\n    if (diag.stats.stagedFrames % 20 === 19 && !diag.or1Skipped) { diag.or1Skipped = true; return; }\n    maybeDispatchConfigChange(framework);',
    'one frame lost on the way to the engine (the 20th: the v3 mutant X8 drops the 50th, which a 30 frame prefix does not reach)', true],
  ['RX9-pyramid-rotation-zero', 'xr-slam.js', 'cameraWidth: videoWidth, cameraHeight: videoHeight, rotation, warn: warnOnce', 'cameraWidth: videoWidth, cameraHeight: videoHeight, rotation: 0, warn: warnOnce',
    'the GL pyramid is told rotation 0 (GL-side uniforms: the fake GL returns the recorded atlas whatever is drawn)', false],
  ['RX10-shader-rotation-pre-pkga', 'gr8-compute-pyramid.js', '(l.rotated ? this.rotation - 90 : this.rotation)', '(l.rotated ? -90 : 0)', 'the shader rotation uniform of the portrait-only chunk (GL side, outside the claim)', false],
];

const flagged = (g) => {
  const ses = SESSION_KEYS.filter((k) => JSON.stringify(g.glueConfig?.C?.[k]) !== JSON.stringify(g.glueConfig?.W?.[k]));
  return { 'IN-CW': g['IN-CW']?.identical === false, 'OUT-CW': g['OUT-CW']?.identical === false, ANS: g.C6 ? !g.C6.pass : g.ANS?.identical === false, C7: g.C7 ? !g.C7.pass : false,
    C8: g.C8 ? !g.C8.pass : false, SES: ses.length > 0, NW: g.NW?.identical === false, CN: g.CN?.identical === false };
};
const chunkGate = (f) => f['IN-CW'] || f['OUT-CW'] || f.ANS || f.C7 || f.C8 || f.SES;

function patchedChunk(id, [, file, from, to], srcDir, outDir) {
  const dir = path.join(outDir, id, 'chunk');
  fs.mkdirSync(dir, { recursive: true });
  for (const f of fs.readdirSync(srcDir).filter((x) => x.endsWith('.js'))) fs.copyFileSync(path.join(srcDir, f), path.join(dir, f));
  const text = fs.readFileSync(path.join(dir, file), 'utf8');
  const n = text.split(from).length - 1;
  if (n !== 1) throw new Error(`${id}: pattern occurs ${n} times in ${file}`);
  fs.writeFileSync(path.join(dir, file), text.replace(from, () => to));
  return dir;
}

function f32Next(x) { const b = new Float32Array([x]); new Uint32Array(b.buffer)[0] += x >= 0 ? 1 : -1; return b[0]; }
function rowMutations(rows) {
  const r = rows.find((x) => x.fresh && x.raw.position.some((v) => v !== 0)) ?? rows.find((x) => x.fresh);
  const clone = () => rows.map((x) => (x.fresh ? { ...x, raw: { ...x.raw, position: [...x.raw.position], rotationWxyz: [...x.raw.rotationWxyz] }, pose: x.pose } : x));
  const out = [];
  const mutate = (id, fn) => { const m = clone(); fn(m.find((x) => x.index === r.index)); const c = compareOutputs(m, rows); out.push({ id, index: r.index, detected: !c.identical }); };
  mutate('position-x-one-float32-ulp', (t) => { t.raw.position[0] = f32Next(t.raw.position[0]); });
  mutate('rotation-w-one-float32-ulp', (t) => { t.raw.rotationWxyz[0] = f32Next(t.raw.rotationWxyz[0]); });
  mutate('position-z-one-double-ulp', (t) => { const b = new Float64Array([t.raw.position[2]]); new BigUint64Array(b.buffer)[0] += 1n; t.raw.position[2] = b[0]; });
  mutate('status-string', (t) => { t.raw.status = t.raw.status === 'NORMAL' ? 'LIMITED' : 'NORMAL'; });
  mutate('repeated-flag', (t) => { t.raw.repeated = !t.raw.repeated; });
  mutate('sourceFrameIndex-plus-1', (t) => { t.raw.sourceFrameIndex += 1; });
  return out;
}

export async function selftest({ pairFile, out, chunkDir, rotationFrames = 30, portraitFrames = 300, skipPortrait = false, log = () => {} }) {
  if (fs.existsSync(out)) throw new Error(`refusing to overwrite ${out}`);
  fs.mkdirSync(out, { recursive: true });
  const pair = JSON.parse(fs.readFileSync(path.resolve(root, pairFile), 'utf8'));
  const engineDir = path.join(root, path.dirname(pair.wasm.file));
  const common = { engineDir, nativeBin: pair.native_replay.file, repoRoot: root, chunkDir };
  const result = { schema: 'or1-v4-selftest-v1', at: new Date().toISOString(), pair, chunkDir: path.relative(root, chunkDir), rotationFrames, portraitFrames, datasets: {}, summary: {} };
  let nullFailures = 0, run = 0, detected = 0, equivalentFalseAlarms = 0;
  const outOfClass = [];
  const sets = [];
  if (!skipPortrait) for (const d of PORTRAIT_DEV) sets.push({ ...d, family: 'portrait', build: (dir) => derive(path.join(root, d.src), path.join(dir, 'bundle'), { frames: portraitFrames, sourceLabel: d.src }) });
  for (const [name, rec, rot] of ROTATION_DEV) {
    sets.push({ name, family: 'rotation', rotation: rot, mutants: 'rotation', build: (dir) => {
      const src = path.join(dir, 'src');
      prepare({ rec: path.join(root, REC, rec), out: src, repoRoot: root });
      return derive(src, path.join(dir, 'bundle'), { frames: rotationFrames, sourceLabel: path.relative(root, src) });
    } });
  }
  for (const ds of sets) {
    const dir = path.join(out, ds.name);
    fs.mkdirSync(dir, { recursive: true });
    const d = ds.build(dir);
    const c = { ...common, bundleDir: path.join(dir, 'bundle') };
    const entry = { family: ds.family, derived: { frames: d.frames, sensorRows: d.sensorRows, droppedConfigKeys: d.droppedConfigKeys, framesWithoutAtlasDisabled: d.framesWithoutAtlasDisabled } };
    log(`${ds.name}: null-run`);
    const nullRun = await runAll({ ...c, outDir: path.join(dir, 'null'), reps: 2, arms: ['W', 'C', 'N'] });
    const g0 = evaluate(nullRun);
    const ses0 = SESSION_KEYS.filter((k) => JSON.stringify(g0.glueConfig.C[k]) !== JSON.stringify(g0.glueConfig.W[k]));
    entry.null = { verdict: g0.verdict, summary: summarize(g0), inputStats: g0.inputStats, sessionKeysDifferent: ses0, NW: { maxPositionDelta: g0.NW.maxPositionDelta, maxUlp: g0.NW.maxUlp }, chunkErrors: g0.chunkErrors, clock: g0.clock };
    if (!Object.values(g0.verdict).every((v) => v === true) || ses0.length) nullFailures++;
    const { W, N, C } = nullRun.runs;
    entry.rowMutations = rowMutations(W[0].rows);
    entry.chunkMutants = {};
    const list = ds.mutants === 'rotation' ? ROTATION_MUTANTS.map((m) => ({ id: m[0], patch: m, claim: m[5], what: m[4] }))
      : (ds.mutants === 'all' ? CHUNK_MUTANTS.map((m) => m[0]) : ds.mutants).map((id) => ({ id, claim: true, what: CHUNK_MUTANTS.find((m) => m[0] === id)[4], v3: true }));
    // rotation datasets also get the generic IMU / time / atlas mutants of v3 once (on the 90 degree dataset)
    if (ds.mutants === 'rotation' && ds.rotation === 90) {
      for (const id of ['X0-noop-comment', 'X1-orientation-alpha-beta-swapped', 'X2-first-motion-event-lost', 'X3-frame-time-plus-1ns', 'X5-atlas-one-bit-flipped', 'X11-engine-time-is-stage-time']) {
        list.push({ id, claim: true, what: CHUNK_MUTANTS.find((m) => m[0] === id)[4], v3: true });
      }
    }
    for (const m of list) {
      log(`${ds.name}: mutant ${m.id}`);
      const mdir = m.v3 ? makeChunkMutant(m.id, chunkDir, path.join(dir, 'mutants')) : patchedChunk(m.id, m.patch, chunkDir, path.join(dir, 'mutants'));
      let res;
      try {
        const r = await runAll({ ...c, chunkDir: mdir, outDir: path.join(dir, `mut-${m.id}`), reps: 1, arms: ['C'] });
        const g = evaluate({ runs: { W: [W[0]], N: [N[0]], C: r.runs.C }, ctx: { bundle: nullRun.ctx.bundle } });
        const f = flagged(g);
        res = { models: m.what, inGateClaim: m.claim, outcome: chunkGate(f) ? 'FAIL' : 'NOT DETECTED', flagged: f, calls: [g['IN-CW'].countA, g['IN-CW'].countB], inputMismatchedCalls: g['IN-CW'].mismatches,
          outputMismatchedFrames: g['OUT-CW'].mismatchedFrames, answerMismatches: g.ANS.mismatches, firstOutputDiff: g['OUT-CW'].firstDiff, chunkErrors: g.chunkErrors };
      } catch (error) {
        res = { models: m.what, inGateClaim: m.claim, outcome: 'INVALID (the chunk arm could not be played)', error: String(error?.message ?? error).slice(0, 300) };
      }
      entry.chunkMutants[m.id] = res;
    }
    entry.nativeMutants = {};
    if (ds.family === 'rotation' && ds.rotation === 90) {
      for (const [id, what] of NATIVE_MUTANTS) {
        const nb = makeNativeMutant(id, path.join(dir, 'bundle'), path.join(dir, 'nmutants'));
        const r = await runAll({ ...c, nativeBundleDir: nb, outDir: path.join(dir, `nmut-${id}`), reps: 1, arms: ['N'] });
        const g = evaluate({ runs: { W: [W[0]], C: [C[0]], N: r.runs.N } });
        entry.nativeMutants[id] = { models: what, NW: g.NW.identical === false, CN: g.CN.identical === false, mismatchedFrames: g.NW.mismatchedFrames };
      }
    }
    result.datasets[ds.name] = entry;
    fs.writeFileSync(path.join(out, `partial-${ds.name}.json`), `${JSON.stringify(entry, null, 1)}\n`);
  }
  // a mutant is counted per id over all datasets it ran on: detected when at least one dataset did not PASS it
  const byId = {};
  for (const [dsName, e] of Object.entries(result.datasets)) for (const [id, r] of Object.entries(e.chunkMutants)) (byId[id] ??= { inGateClaim: r.inGateClaim, models: r.models, outcomes: {} }).outcomes[dsName] = r.outcome;
  for (const [id, r] of Object.entries(byId)) {
    r.detected = Object.values(r.outcomes).some((o) => o !== 'NOT DETECTED');
    if (id === 'X0-noop-comment') { if (r.detected) equivalentFalseAlarms++; } else if (r.inGateClaim) { run++; if (r.detected) detected++; } else outOfClass.push({ id, detected: r.detected, models: r.models });
  }
  const rowAll = Object.values(result.datasets).flatMap((e) => e.rowMutations);
  const nat = Object.values(result.datasets).flatMap((e) => Object.values(e.nativeMutants));
  result.mutantsById = byId;
  result.summary = { nullFailures, chunkMutantsRun: run, chunkMutantsDetectedByAChunkGate: detected, equivalentMutantFalseAlarms: equivalentFalseAlarms,
    undetectedInClaim: Object.entries(byId).filter(([id, r]) => id !== 'X0-noop-comment' && r.inGateClaim && !r.detected).map(([id]) => id), outOfClaim: outOfClass,
    rowMutations: rowAll.length, rowMutationsDetected: rowAll.filter((x) => x.detected).length, nativeMutants: nat.length, nativeMutantsDetectedByNW: nat.filter((x) => x.NW).length,
    passes: nullFailures === 0 && detected === run && equivalentFalseAlarms === 0 && rowAll.every((x) => x.detected) };
  fs.writeFileSync(path.join(out, 'selftest.json'), `${JSON.stringify(result, null, 1)}\n`);
  return result;
}

const isMain = process.argv[1] && path.resolve(process.argv[1]) === import.meta.filename;
if (isMain) {
  const { values } = parseArgs({ options: { pair: { type: 'string' }, out: { type: 'string' }, 'chunk-dir': { type: 'string' }, 'rotation-frames': { type: 'string' }, 'portrait-frames': { type: 'string' }, 'skip-portrait': { type: 'boolean' } } });
  try {
    const out = path.resolve(root, values.out ?? '');
    if (!values.out || !out.startsWith(path.join(root, 'build') + path.sep)) throw new Error('--out must be a new directory under build/');
    const r = await selftest({ pairFile: values.pair, out, chunkDir: path.resolve(root, values['chunk-dir'] ?? 'slam_refact/web-chunk'), rotationFrames: Number(values['rotation-frames'] ?? 30),
      portraitFrames: Number(values['portrait-frames'] ?? 300), skipPortrait: values['skip-portrait'] === true, log: (m) => console.error(m) });
    console.log(JSON.stringify(r.summary, null, 1));
    process.exitCode = r.summary.passes ? 0 : 1;
  } catch (error) {
    console.error(String(error?.stack ?? error));
    process.exitCode = 2;
  }
}
