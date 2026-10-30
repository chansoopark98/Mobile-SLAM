// or1-chunk-replay-gate.v4: the judge. It reads the report.json of every registered scenario and the registered expectations in the manifest
// (manifest.registered_run.scenarios) and prints the verdict. Every rule below is fixed by the registration document (section 6) BEFORE any first real
// run; nothing is a tolerance (the comparisons are bit comparisons, protocol 11.1 determinism class).
//
//   node judge.mjs --manifest <manifest.json> --out-root <dir holding <scenario>/report.json> [--run-status <run-status.tsv>]
//   exit 0 PASS   1 FAIL (the chunk and the direct engine calls disagreed, or an arm was not deterministic, in a valid measurement)
//        4 INVALID (the measurement is incomplete or not a valid one: a report is missing or malformed, the apparatus failed its own check, the input was
//          degenerate, a registered scenario did not run: never a PASS and never a FAIL)
//        5 JUDGE ERROR (this program crashed; it says nothing about the engine or the chunk)
//
// Review defects of the earlier gate tools this judge closes (reviews/final-a-review-harness.md): M-2 (a verdict with k = 1 evaluated scenarios and the rest
// INVALID was PASS: here every registered scenario counts, the denominator is the registered number and INVALID / MISSING are never dropped), M-3 (a tool crash was
// exit 1 = FAIL: here a crash is 5 and a missing report is INVALID), M-7 (no lower bound on the compared frames: the registered call and frame counts are required
// exactly and a floor on fresh engine responses and on distinct rotations removes PASS evidence from a degenerate scenario).
import fs from 'node:fs';
import path from 'node:path';
import { parseArgs } from 'node:util';
import { canon } from '../../../web-chunk/tools/or1/canon.mjs';
import { designationPins } from '../../../web-chunk/tools/or1/hold.mjs';

export const EXIT = { PASS: 0, FAIL: 1, INVALID: 4, JUDGE_ERROR: 5 };
export const GATE_ID = 'or1-chunk-replay-gate';
export const GATE_VERSION = 4;
// the glue-config keys the chunk's session must have equal to the direct arm's (W builds the glue from the bundle; C from the chunk's own session)
export const SESSION_KEYS = ['captureWidth', 'captureHeight', 'rotationDeg', 'manufacturer', 'model', 'estimateScale', 'disableVio', 'leftHandedAxes', 'mirroredDisplay'];

const isInt = (x) => Number.isInteger(x);
const isNum = (x) => typeof x === 'number' && Number.isFinite(x);

// ---- one scenario ----
// spec: manifest.registered_run.scenarios[i]; report: parsed report.json or null; ctx: {beforeReviewSha256}
// returns {status: PASS | PASS_NO_EVIDENCE | FAIL | INVALID | MISSING, reasons: [], checks: {}}
export function judgeScenario(spec, report, ctx = {}) {
  const invalid = [], failed = [], notes = [], checks = {};
  const out = (status) => ({ name: spec.name, family: spec.family, record: spec.record, status, reasons: [...invalid, ...failed, ...notes], invalid, failed, notes, checks });
  if (report === null || report === undefined) { invalid.push('no report.json for this registered scenario'); return out('MISSING'); }
  const g = report.gates, meta = report.meta;
  // V0: it is a report of THIS gate and THIS scenario under THIS registration
  if (!g || typeof g !== 'object' || !meta || typeof meta !== 'object') { invalid.push('report.json has no meta / gates'); return out('INVALID'); }
  const [gateId, gateVersion] = [ctx.gateId ?? GATE_ID, ctx.gateVersion ?? GATE_VERSION];
  if (meta.gate?.id !== gateId || meta.gate?.version !== gateVersion) invalid.push(`report is of ${meta.gate?.id} v${meta.gate?.version}, not ${gateId} v${gateVersion}`);
  if (meta.gate?.datum !== spec.name) invalid.push(`report names the datum ${meta.gate?.datum}, expected ${spec.name}`);
  if (meta.kind !== 'gate') invalid.push(`report kind ${meta.kind}, expected gate`);
  if (meta.arms !== 'WCN') invalid.push(`arms ${meta.arms}, expected WCN`);
  if (!(meta.reps >= 3)) invalid.push(`repetitions ${meta.reps}, expected >= 3`);
  if (ctx.beforeReviewSha256 && meta.gate?.manifest?.beforeReviewSha256 !== ctx.beforeReviewSha256) invalid.push('the report was produced under another manifest (manifest_before_review_sha256 differs)');
  for (const key of ['IN-CW', 'OUT-CW', 'ANS', 'REP', 'clock', 'inputStats', 'glueConfig', 'C4', 'chunkErrors']) if (g[key] === undefined) invalid.push(`gates.${key} is missing`);
  if (!report.rowsSha256 || typeof report.rowsSha256 !== 'object') invalid.push('rowsSha256 is missing');
  if (invalid.length) return out('INVALID');

  const exp = spec.expected;
  // V1: the apparatus did its own job. The direct arm W is the oracle: if W did not play the registered number of calls the measurement is not valid.
  checks.wCalls = g['IN-CW'].countB === exp.calls;
  if (!checks.wCalls) invalid.push(`direct arm W played ${g['IN-CW'].countB} calls, registered ${exp.calls}`);
  const want = { captureWidth: canon(spec.capture.width), captureHeight: canon(spec.capture.height), rotationDeg: canon(spec.capture.rotation) };
  const gw = g.glueConfig?.W, gc = g.glueConfig?.C;
  if (!gw || typeof gw !== 'object' || !gc || typeof gc !== 'object') invalid.push('gates.glueConfig has no W / C');
  else {
    for (const k of Object.keys(want)) if (JSON.stringify(gw[k]) !== JSON.stringify(want[k])) invalid.push(`direct arm W built its glue with ${k} ${JSON.stringify(gw[k])}, registered ${JSON.stringify(want[k])}`);
  }
  if (invalid.length) return out('INVALID');

  // V2: the graded comparisons (strict: only `true` / `0` / the registered number passes; undefined is not a pass)
  const inCw = g['IN-CW'], outCw = g['OUT-CW'], ans = g.ANS;
  checks['IN-CW'] = inCw.identical === true && inCw.mismatches === 0 && inCw.countA === exp.calls && inCw.countB === exp.calls;
  if (!checks['IN-CW']) failed.push(`IN-CW: identical ${inCw.identical}, mismatches ${inCw.mismatches}, calls C ${inCw.countA} / W ${inCw.countB} (registered ${exp.calls}), first ${JSON.stringify(inCw.first)}`);
  checks['OUT-CW'] = outCw.identical === true && outCw.mismatchedFrames === 0 && outCw.frames === exp.frames && outCw.identicalFrames === exp.frames;
  if (!checks['OUT-CW']) failed.push(`OUT-CW: identical ${outCw.identical}, mismatched frames ${outCw.mismatchedFrames}, frames ${outCw.frames} identical ${outCw.identicalFrames} (registered ${exp.frames}), first ${JSON.stringify(outCw.firstDiff)}`);
  checks.ANS = ans.identical === true && ans.mismatches === 0 && isInt(ans.nullAnswers) && ans.nullAnswers <= 1 && ans.nullAnswers + ans.compared === ans.answers
    && ans.compared === exp.frames && ans.answers === exp.calls && Array.isArray(g.chunkErrors) && g.chunkErrors.length === 0;
  if (!checks.ANS) failed.push(`ANS: identical ${ans.identical}, mismatches ${ans.mismatches}, null ${ans.nullAnswers}, compared ${ans.compared}, answers ${ans.answers} (registered ${exp.frames} / ${exp.calls}), chunk errors ${JSON.stringify(g.chunkErrors)}`);
  checks.C7 = g.clock.clockLost === 0 && g.clock.videoLost === 0;
  if (!checks.C7) failed.push(`C7: the fake clock lost integer nanoseconds (clockLost ${g.clock.clockLost}, videoLost ${g.clock.videoLost})`);
  const rep = g.REP ?? {};
  const repOk = (arm) => Array.isArray(rep[arm]) && rep[arm].length >= 2 && rep[arm].every((r) => r.identical === true);
  const shaOk = (arm) => Array.isArray(report.rowsSha256[arm]) && report.rowsSha256[arm].length >= 3 && new Set(report.rowsSha256[arm]).size === 1;
  checks.REP = repOk('W') && repOk('C') && repOk('N') && shaOk('W') && shaOk('C') && shaOk('N');
  if (!checks.REP) failed.push(`REP: W ${repOk('W')}/${shaOk('W')}, C ${repOk('C')}/${shaOk('C')}, N ${repOk('N')}/${shaOk('N')} (repetition identical / rows sha256 equal over the repetitions)`);
  // SES: the glue the chunk built has the capture size, rotation and device of the direct arm (and so of the registered scenario)
  const sesBad = SESSION_KEYS.filter((k) => JSON.stringify(gc[k]) !== JSON.stringify(gw[k]));
  checks.SES = sesBad.length === 0;
  if (!checks.SES) failed.push(`SES: the chunk's glue config differs from the direct arm's in ${sesBad.join(', ')} (C ${JSON.stringify(sesBad.map((k) => gc[k]))}, W ${JSON.stringify(sesBad.map((k) => gw[k]))})`);
  // the reports of the arms must agree with themselves: equal rows over W and C when OUT-CW says identical
  checks.consistent = !(outCw.identical === true && report.rowsSha256.W?.[0] !== report.rowsSha256.C?.[0]);
  if (!checks.consistent) invalid.push('internally inconsistent report: OUT-CW is identical but the rows sha256 of W and C differ');
  if (invalid.length) return out('INVALID');
  if (failed.length) return out('FAIL');

  // V3: input degeneracy removes PASS evidence; it never removes a FAIL (v3 C9)
  const st = g.inputStats, fl = spec.floors ?? {};
  if (!isNum(fl.min_fresh_nonrepeated)) { invalid.push('the registration has no floor min_fresh_nonrepeated for this scenario'); return out('INVALID'); }
  const freshNonRepeated = st.fresh - st.repeated;
  checks.floorFresh = isNum(freshNonRepeated) && freshNonRepeated >= fl.min_fresh_nonrepeated;
  if (!checks.floorFresh) notes.push(`degenerate: fresh non-repeated engine responses ${freshNonRepeated} < registered floor ${fl.min_fresh_nonrepeated}`);
  if (fl.min_distinct_rotations !== null && fl.min_distinct_rotations !== undefined) {
    checks.floorRotations = isNum(st.distinctRotations) && st.distinctRotations >= fl.min_distinct_rotations;
    if (!checks.floorRotations) notes.push(`degenerate: distinct output rotations ${st.distinctRotations} < registered floor ${fl.min_distinct_rotations}`);
  }
  if (fl.c4) {
    checks.c4 = g.C4 && g.C4.degenerate === false && isNum(g.C4.r) && g.C4.r >= fl.c4.r_min && isInt(g.C4.k) && g.C4.k >= fl.c4.k_min;
    if (!checks.c4) notes.push(`degenerate (C4): LIMITED share r ${g.C4?.r}, status transitions k ${g.C4?.k}; registered floor r >= ${fl.c4.r_min}, k >= ${fl.c4.k_min}`);
  }
  const degenerate = Object.entries(checks).some(([k, v]) => ['floorFresh', 'floorRotations', 'c4'].includes(k) && v === false);
  return out(degenerate ? 'PASS_NO_EVIDENCE' : 'PASS');
}

// ---- the gate over its registered scenarios ----
// reports: {name: report | null}. Every registered scenario is counted; INVALID / MISSING are never dropped from the denominator.
export function judgeGate(scenarios, reports, ctx = {}) {
  const per = scenarios.map((s) => judgeScenario(s, reports[s.name] ?? null, ctx));
  const failures = per.filter((p) => p.status === 'FAIL');
  const invalids = per.filter((p) => p.status === 'INVALID' || p.status === 'MISSING');
  const records = {};
  for (const p of per) {
    const r = (records[`${p.family}:${p.record}`] ??= { family: p.family, record: p.record, scenarios: [], valid: false });
    r.scenarios.push(p.name);
    if (p.status === 'PASS') r.valid = true;
  }
  const rotation = per.filter((p) => p.family === 'rotation');
  const portraitRecords = Object.values(records).filter((r) => r.family === 'portrait');
  const need = ctx.kMin ?? {};
  const rotationPass = rotation.filter((p) => p.status === 'PASS').length;
  const portraitValid = portraitRecords.filter((r) => r.valid).length;
  const counts = { registered: scenarios.length, evaluated: per.filter((p) => p.status !== 'MISSING').length, pass: per.filter((p) => p.status === 'PASS').length,
    passNoEvidence: per.filter((p) => p.status === 'PASS_NO_EVIDENCE').length, fail: failures.length, invalid: per.filter((p) => p.status === 'INVALID').length,
    missing: per.filter((p) => p.status === 'MISSING').length, rotationPass, rotationRegistered: rotation.length, portraitValidRecords: portraitValid, portraitRecords: portraitRecords.length };
  const reasons = [];
  let verdict;
  if (failures.length) verdict = 'FAIL';
  else if (invalids.length) { verdict = 'INVALID'; reasons.push(`${invalids.length} of ${scenarios.length} registered scenarios are INVALID / MISSING: ${invalids.map((p) => p.name).join(', ')}`); }
  else {
    const kRot = need.rotationScenarios ?? rotation.length, kRec = need.portraitRecords ?? portraitRecords.length;
    if (rotationPass < kRot) reasons.push(`rotation scenarios with PASS evidence ${rotationPass} < registered ${kRot}`);
    if (portraitValid < kRec) reasons.push(`portrait records with PASS evidence ${portraitValid} < registered ${kRec}`);
    if (counts.pass + counts.passNoEvidence !== scenarios.length) reasons.push('not every registered scenario passed');
    verdict = reasons.length ? 'INVALID' : 'PASS';
  }
  return { verdict, exit: EXIT[verdict], counts, reasons, records, perScenario: per };
}

// ---- CLI ----
export function readReports(scenarios, outRoot) {
  const reports = {};
  for (const s of scenarios) {
    const f = path.join(outRoot, s.name, 'report.json');
    if (!fs.existsSync(f)) { reports[s.name] = null; continue; }
    try { reports[s.name] = JSON.parse(fs.readFileSync(f, 'utf8')); } catch (error) { reports[s.name] = { __unreadable: String(error.message) }; }
  }
  return reports;
}

export function runCli(argv) {
  const { values } = parseArgs({ args: argv, options: { manifest: { type: 'string' }, 'out-root': { type: 'string' }, 'run-status': { type: 'string' } } });
  if (!values.manifest || !values['out-root']) throw new Error('usage: judge.mjs --manifest <manifest.json> --out-root <dir> [--run-status <file>]');
  const manifest = JSON.parse(fs.readFileSync(values.manifest, 'utf8'));
  const scenarios = manifest.registered_run?.scenarios;
  if (!Array.isArray(scenarios) || !scenarios.length) throw new Error('manifest.registered_run.scenarios is missing');
  for (const s of scenarios) for (const k of ['name', 'family', 'record', 'capture', 'expected', 'floors']) if (!s[k]) throw new Error(`registered scenario ${s.name ?? '?'}: ${k} is missing`);
  const reports = readReports(scenarios, values['out-root']);
  const ctx = { beforeReviewSha256: designationPins(values.manifest).manifest_before_review_sha256, kMin: manifest.registered_run.k_min ?? {}, gateId: manifest.gate_id, gateVersion: manifest.version };
  const result = judgeGate(scenarios, reports, ctx);
  if (values['run-status'] && fs.existsSync(values['run-status'])) result.runStatus = fs.readFileSync(values['run-status'], 'utf8').trim().split('\n');
  return result;
}

const isMain = process.argv[1] && path.resolve(process.argv[1]) === import.meta.filename;
if (isMain) {
  try {
    const result = runCli(process.argv.slice(2));
    console.log(JSON.stringify(result, null, 1));
    console.log(`verdict: ${result.verdict} (exit ${result.exit}; PASS ${result.counts.pass}, PASS_NO_EVIDENCE ${result.counts.passNoEvidence}, FAIL ${result.counts.fail}, INVALID ${result.counts.invalid}, MISSING ${result.counts.missing} of ${result.counts.registered} registered)`);
    process.exitCode = result.exit;
  } catch (error) {
    console.error(`JUDGE ERROR (not a verdict about the engine or the chunk): ${String(error?.stack ?? error)}`);
    process.exitCode = EXIT.JUDGE_ERROR;
  }
}
