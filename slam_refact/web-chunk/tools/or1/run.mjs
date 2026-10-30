// OR-1 orchestration: play one derived bundle through the arms and compare them. Library (no CLI); or1.mjs is the command line.
//
// Gates computed here (registered in the OR-1 gate file; every one is a bit comparison, tolerance 0):
//   IN-CW   per-call engine inputs of the chunk arm C == those of the direct WASM arm W
//   OUT-CW  engine outputs (the complete pose object without the wall-clock key) of C == W, every frame
//   ANS     what the chunk answered the app == the engine pose of the same call
//   REP     repetitions of one arm are identical (determinism of the arm itself)
//   NW      headline outputs of N (native) == W, every frame           (cross-platform: libm / compiler differ; its rule is in the gate file)
//   CN      headline outputs of C == N                                  (follows from OUT-CW and NW; reported for the end-to-end statement)
// v3 (OR-1 v2 pre-review C4, C6-C9; first-run review V-4 / V-15): the verdicts the review made conditions are computed here and stored in the report,
// not by hand afterwards:
//   C4  per bundle: r = share of LIMITED (not INITIALIZING) among the fresh, not repeated rows of W, k = status transitions between consecutive such rows;
//       the bundle is degenerate when r < C4_R_MIN or k < C4_K_MIN (a floor set after the original ratios were public). aggregateGate() applies the
//       record rule (a record is valid when one of its bundles is not degenerate; fewer than two valid records: insufficient discrimination).
//   C6  ANS is not vacuous: identical, nullAnswers <= 1, nullAnswers + compared == answers, compared == OUT-CW.frames, no chunk error
//   C7  the fake clock lost no integer ns (clockLost == 0, videoLost == 0)
//   C8  call counts: IN-CW countA == countB == expected.calls and OUT-CW.frames == expected.frames (from the bundle: 1 + processed frames / processed frames)
//   C9  no exclusion (degenerate bundle, harness) removes a mismatch: any bundle with IN-CW, OUT-CW, ANS (C6), REP, C7 or C8 failing fails the gate
import fs from 'node:fs';
import path from 'node:path';
import { readBundle } from './bundle.mjs';
import { playDirect, playChunk } from './players.mjs';
import { runNative } from './native.mjs';
import { compareInputs, compareOutputs, compareAnswers } from './compare.mjs';
import { canon, diffPaths, sha256 } from './canon.mjs';

const stat = (rows) => {
  const fresh = rows.filter((r) => r.fresh);
  const count = (k) => fresh.reduce((m, r) => { m[r.raw[k]] = (m[r.raw[k]] ?? 0) + 1; return m; }, {});
  const pos = fresh.map((r) => r.raw.position), q = new Set(fresh.map((r) => r.raw.rotationWxyz.join(',')));
  const range = (i) => (pos.length ? [Math.min(...pos.map((p) => p[i])), Math.max(...pos.map((p) => p[i]))] : null);
  return { rows: rows.length, fresh: fresh.length, status: count('status'), reason: count('reason'), repeated: fresh.filter((r) => r.raw.repeated).length,
    distinctRotations: q.size, positionRange: [0, 1, 2].map(range) };
};

// result of an arm -> the part that is stored/compared
export async function playArm(arm, ctx, rep) {
  const t0 = performance.now();
  let r;
  if (arm === 'W') r = await playDirect({ engineDir: ctx.engineDir, bundle: ctx.bundle, repoRoot: ctx.repoRoot });
  else if (arm === 'C') r = await playChunk({ chunkDir: ctx.chunkDir, engineDir: ctx.engineDir, bundle: ctx.bundle, repoRoot: ctx.repoRoot });
  else if (arm === 'N') {
    const out = path.join(ctx.outDir, `native-${rep}`);
    const n = runNative({ bin: ctx.nativeBin, bundleDir: ctx.nativeBundleDir ?? ctx.bundle.dir, outDir: out, repoRoot: ctx.repoRoot });
    r = { arm: 'N', rows: n.rows, traceSha256: n.traceSha256, nativeMeta: { processedFrames: n.meta.processedFrames, droppedFrames: n.meta.droppedFrames, contractVersion: n.meta.contractVersion } };
  } else throw new Error(`unknown arm ${arm}`);
  r.wallMs = performance.now() - t0;
  return r;
}

// ctx: {bundleDir, nativeBundleDir?, engineDir, nativeBin, chunkDir, repoRoot, outDir, reps, arms}
export async function runAll(ctx) {
  if (fs.existsSync(ctx.outDir)) throw new Error(`refusing to overwrite ${ctx.outDir}`);
  fs.mkdirSync(ctx.outDir, { recursive: true });
  ctx.bundle = readBundle(ctx.bundleDir);
  const runs = {};
  for (const arm of ctx.arms) {
    runs[arm] = [];
    for (let rep = 0; rep < ctx.reps; rep++) runs[arm].push(await playArm(arm, ctx, rep));
  }
  return { ctx, runs };
}

export const C4_R_MIN = 0.10;
export const C4_K_MIN = 2;

// C4 of one bundle from the rows of W (first repetition). null when W did not run.
export function degeneracy(rows) {
  if (!rows) return null;
  const used = rows.filter((r) => r.fresh && !r.raw.repeated);
  const limited = used.filter((r) => r.raw.status === 'LIMITED' && r.raw.reason !== 'INITIALIZING').length;
  let k = 0;
  for (let i = 1; i < used.length; i++) if (used[i].raw.status !== used[i - 1].raw.status) k++;
  const r = used.length ? limited / used.length : 0;
  return { rows: used.length, limited, r, k, rMin: C4_R_MIN, kMin: C4_K_MIN, degenerate: r < C4_R_MIN || k < C4_K_MIN };
}

// C6 / C7 / C8 of one bundle. `expected` = {calls, frames} counted from the bundle (null: the bundle was not given, C8 is not evaluated).
export function conditions(g, expected) {
  const ans = g.ANS;
  const c6 = ans ? (() => {
    const reasons = [];
    if (!ans.identical) reasons.push('ANS not identical');
    if (!(ans.nullAnswers <= 1)) reasons.push(`nullAnswers ${ans.nullAnswers} > 1`);
    if (ans.nullAnswers + ans.compared !== ans.answers) reasons.push('nullAnswers + compared != answers');
    if (g['OUT-CW'] && ans.compared !== g['OUT-CW'].frames) reasons.push(`compared ${ans.compared} != OUT-CW.frames ${g['OUT-CW'].frames}`);
    if (g.chunkErrors.length) reasons.push(`${g.chunkErrors.length} chunk error(s)`);
    return { pass: reasons.length === 0, reasons };
  })() : null;
  const c7 = g.clock ? { pass: g.clock.clockLost === 0 && g.clock.videoLost === 0, clockLost: g.clock.clockLost, videoLost: g.clock.videoLost, clockInexact: g.clock.clock, videoInexact: g.clock.video } : null;
  const c8 = expected && g['IN-CW'] && g['OUT-CW'] ? (() => {
    const reasons = [];
    if (g['IN-CW'].countA !== expected.calls || g['IN-CW'].countB !== expected.calls) reasons.push(`IN-CW calls C ${g['IN-CW'].countA} / W ${g['IN-CW'].countB}, expected ${expected.calls}`);
    if (g['OUT-CW'].frames !== expected.frames) reasons.push(`OUT-CW frames ${g['OUT-CW'].frames}, expected ${expected.frames}`);
    return { pass: reasons.length === 0, expected, reasons };
  })() : null;
  return { C6: c6, C7: c7, C8: c8 };
}

// The comparisons of one set of runs. `arms[X][i]` is repetition i of arm X.
export function expectedCounts(bundle) {
  const processed = bundle.frames.slice(1).filter((f) => f.process).length;
  return { calls: 1 + processed, frames: processed };
}

export function evaluate({ runs, ctx = null, expected = ctx?.bundle ? expectedCounts(ctx.bundle) : null }) {
  const g = {};
  const W = runs.W?.[0], C = runs.C?.[0], N = runs.N?.[0];
  if (C && W) {
    g['IN-CW'] = compareInputs(C.frames, W.frames);
    g['OUT-CW'] = compareOutputs(C.rows, W.rows, { full: true });
    g.layoutRows = { C: C.layout.rows, W: W.layout.rows };
  }
  if (C) {
    g.ANS = compareAnswers(C.answers, C.rows);
    g.chunkErrors = C.errors.map((e) => String(e.message ?? e));
    g.clock = C.inexact;
  }
  g.REP = {};
  for (const arm of Object.keys(runs)) {
    const rs = runs[arm];
    if (rs.length < 2) continue;
    // N: the native trace also holds wall-clock stage timings (srStageMs), so repetitions are compared on the output fields, not on the file hash
    g.REP[arm] = rs.slice(1).map((r, i) => {
      const o = compareOutputs(r.rows, rs[0].rows, { full: arm !== 'N' });
      const inp = arm === 'N' ? { identical: true, mismatches: 0 } : compareInputs(r.frames, rs[0].frames);
      return { rep: i + 1, identical: o.identical && inp.identical, outputs: { frames: o.frames, identicalFrames: o.identicalFrames, firstDiff: o.firstDiff },
        inputs: { identical: inp.identical, mismatches: inp.mismatches } };
    });
  }
  if (W && N) g.NW = compareOutputs(N.rows, W.rows);
  if (C && N) g.CN = compareOutputs(C.rows, N.rows);
  g.inputStats = (W ?? C ?? N) ? stat((W ?? C ?? N).rows) : null;
  g.wallMs = Object.fromEntries(Object.entries(runs).map(([a, rs]) => [a, rs.map((r) => Math.round(r.wallMs))]));
  if (C && W) g.glueConfig = { W: W.glueConfig ?? null, C: C.glueConfig ?? null, diffPaths: W.glueConfig && C.glueConfig ? diffPaths(W.glueConfig, C.glueConfig) : null, note: 'REPORT (V-11): recorded, not compared by IN-CW' };
  g.C4 = degeneracy(W?.rows);
  Object.assign(g, conditions(g, expected));
  g.verdict = {
    'IN-CW': g['IN-CW']?.identical, 'OUT-CW': g['OUT-CW']?.identical, ANS: g.C6 ? g.C6.pass : g.ANS?.identical,
    REP: Object.keys(g.REP).length ? Object.values(g.REP).flat().every((r) => r.identical) : undefined, NW: g.NW?.identical, CN: g.CN?.identical,
    C7: g.C7?.pass, C8: g.C8?.pass,
  };
  return g;
}

// The GATE-graded part of one bundle's verdict (NW and CN are REPORT, C4 only removes PASS evidence): true / false / undefined (not evaluated)
export const gradedPass = (v) => {
  const graded = [v['IN-CW'], v['OUT-CW'], v.ANS, v.REP, v.C7, v.C8];
  return graded.some((x) => x === false) ? false : graded.every((x) => x === true) ? true : undefined;
};

// The gate over its bundles: bundles = [{name, record, g}] (g = evaluate() result of the registered run, with reps >= 2 so REP exists). C9: any graded
// mismatch fails the gate, degenerate bundles included. C4: a record is valid when at least one of its bundles is not degenerate; fewer than two valid
// records is "insufficient discrimination" and no PASS wording may be used.
export function aggregateGate(bundles) {
  const fails = [], unevaluated = [];
  const perBundle = bundles.map(({ name, record, g }) => {
    const pass = gradedPass(g.verdict);
    if (pass === false) fails.push({ bundle: name, failed: Object.entries(g.verdict).filter(([k, v]) => ['IN-CW', 'OUT-CW', 'ANS', 'REP', 'C7', 'C8'].includes(k) && v === false).map(([k]) => k) });
    if (pass === undefined) unevaluated.push(name);
    return { name, record, graded: pass, degenerate: g.C4?.degenerate ?? null, r: g.C4?.r ?? null, k: g.C4?.k ?? null };
  });
  const records = {};
  for (const b of perBundle) {
    records[b.record] ??= { bundles: [], valid: false };
    records[b.record].bundles.push(b.name);
    if (b.degenerate === false) records[b.record].valid = true;
  }
  const validRecords = Object.values(records).filter((x) => x.valid).length;
  const verdict = fails.length ? 'FAIL' : unevaluated.length ? 'UNEVALUATED' : validRecords < 2 ? 'INSUFFICIENT' : 'PASS';
  return { verdict, fails, unevaluated, validRecords, records, perBundle,
    note: 'NW and CN are REPORT and not in this verdict; INSUFFICIENT = fewer than two valid records (discrimination too low, no PASS wording); the C5 sentence rules are for the result document' };
}

export const summarize = (g) => Object.entries(g.verdict).map(([k, v]) => `${k}: ${v === undefined ? '-' : v ? 'identical' : 'DIFFERENT'}`).join('  ');
