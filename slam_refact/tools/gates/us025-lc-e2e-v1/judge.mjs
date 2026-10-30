// us025-lc-e2e-gate.v1: the decision rules that sit ABOVE the comparator (slam_refact/tools/harness/us025/compare_us025.py).
// Pure functions, no I/O except the CLI at the bottom. Registered in the gate document us025-lc-e2e-gate-v1-registration.md (sections 5-7).
//
//   node judge.mjs verdict --manifest <manifest.json> --reports <report.json>,...   exit codes below
//   node judge.mjs constants                                                        print JUDGE_CONSTANTS
//
// The comparator decides per original run and arm (ON / OFF-versus-ON / NULL). This file decides what a run, a scenario and the gate are.
// Nothing here relaxes a comparator verdict. Constants are declared before any gate result (the manifest records them and the runner holds on a mismatch).
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

// ---- constants (declared before the first real run) ------------------------------------------------------------------------------
export const SCENARIO_RUNS = {   // original runs per registered scenario: lc=1 runs (ON / OFF-vs-ON arms) and lc=0 runs (NULL arm). Fixed, no early stop, no re-run.
  'flip6-s3-lowtex': { lc1: 16, lc0: 2 },
  'flip6-s2': { lc1: 8, lc0: 2 },
  'tum-room1-full': { lc1: 4, lc0: 2 },
  'tum-room2-full': { lc1: 4, lc0: 2 },
  'tum-room4-full': { lc1: 4, lc0: 2 },
};
export const JUDGE_CONSTANTS = {
  K_MIN: 3,                    // evaluated ON runs (over all scenarios) needed for PASS; fewer = INSUFFICIENT (final-a-review-harness M-2)
  MAX_INFRA_LC1_FRACTION: 0.5, // INFRA lc=1 runs (the original SDK hung or failed) above this fraction of ALL planned lc=1 runs: the gate verdict is INFRA, no PASS
  MIN_NULL_ZERO: 1,            // usable NULL-arm runs (bit identical) a scenario needs before any of its lc=1 runs may count as EVALUATED
  MIN_ATTEMPTS_COMPARED: 5,    // floors that keep a degenerate comparison from passing (final-a-review-harness M-7, the analogue of the co-detection floor):
  MIN_UNDECLARED_PICKS: 2,     //   attempts compared in the ON arm, candidate picks compared that are not declared-U2, trace frames before F.
  MIN_TRACE_FRAMES: 30,        //   Far below every value seen in the dev run (13 / 4 / 131); a degeneracy guard, not a tuned threshold.
  RULE_ENGINE_ON_EXIT: 'US025-M1',   // engine ON replay abnormal while the OFF replay of the same run exits 0
  RULE_ENGINE_EXIT: 'US025-M2',      // any other abnormal exit of an engine replay (ON, OFF or NULL): FAIL, unless the cause is the harness (ENOSPC)
};
export const VERDICT_EXIT = { PASS: 0, FAIL: 1, BLOCKED: 4, UNEVALUATED: 4, INFRA: 6, INSUFFICIENT: 7, INVALID: 8 };

const PASSV = new Set(['PASS', 'PASS_DECLARED']);
const check = (res, name) => (res?.checks ?? []).find((c) => c.name === name);

// ---- engine replay exit rules (US025-M1, US025-M2) --------------------------------------------------------------------------------------
// An engine replay is deterministic, so an abnormal exit of ANY engine replay (ON, OFF or NULL arm) is a defect of the engine, not of the original SDK:
// it is a FAIL, never INFRA (prereview C2, final-a-review-harness M-3). The only exception is a cause that lies in the harness: the replay log shows
// ENOSPC / "No space left on device" (`harness: true`); SIGKILL is retried once by the runner before it is judged. replays[arm] = {exit, harness?}.
const abnormal = (r) => !!r && (r.exit === null || (typeof r.exit === 'number' && r.exit !== 0));
export const engineOnExitRule = (replays) => abnormal(replays?.on) && !replays.on.harness && replays?.off?.exit === 0;   // US025-M1 (the subset ON abnormal, OFF clean)
export function engineReplayVerdict(replays) {
  const bad = ['on', 'off', 'null'].filter((a) => abnormal(replays?.[a]));
  const engine = bad.filter((a) => !replays[a].harness);
  if (engine.length) return { kind: 'FAIL', rule: engineOnExitRule(replays) ? JUDGE_CONSTANTS.RULE_ENGINE_ON_EXIT : JUDGE_CONSTANTS.RULE_ENGINE_EXIT, arms: engine };
  if (bad.length) return { kind: 'INFRA', arms: bad };
  return null;
}

// ---- one lc=1 run ------------------------------------------------------------------------------------------------------------------------
// run = { name, infra: string|null, replays: {on:{exit}, off:{exit}}, on: <compare result>|null, off: <compare result>|null }
// where a compare result is {verdict, reason?, checks:[{name, compared, mismatch, declaredU2}]} or {verdict:'TOOL_ERROR', reason}.
export function classifyLc1Run(run) {
  const reasons = [];
  if (run.infra) return { name: run.name, class: 'INFRA', reasons: [run.infra] };
  const on = run.on;
  const off = run.off;
  const ev = engineReplayVerdict(run.replays);
  if (ev?.kind === 'FAIL') {
    return { name: run.name, class: 'FAIL', rule: ev.rule, reasons: [`engine replay exited abnormally (${ev.arms.map((a) => `${a} ${run.replays[a].exit}`).join(', ')})`] };
  }
  if (ev?.kind === 'INFRA') return { name: run.name, class: 'INFRA', reasons: [`engine replay failed for a harness reason (${ev.arms.join(', ')}: no space left)`] };
  if (!on || !off) return { name: run.name, class: 'TOOL_ERROR', reasons: ['comparator produced no result for the ON or OFF arm'] };
  if (on.verdict === 'FAIL' || off.verdict === 'FAIL') {
    return { name: run.name, class: 'FAIL', reasons: [`ON ${on.verdict}${on.reason ? `: ${on.reason}` : ''}`, `OFF ${off.verdict}${off.reason ? `: ${off.reason}` : ''}`] };
  }
  if (on.verdict === 'TOOL_ERROR' || off.verdict === 'TOOL_ERROR') {
    return { name: run.name, class: 'TOOL_ERROR', reasons: [`ON ${on.verdict} ${on.reason ?? ''}`, `OFF ${off.verdict} ${off.reason ?? ''}`] };
  }
  if (on.verdict === 'INVALID' || off.verdict === 'INVALID') {
    return { name: run.name, class: 'INVALID', reasons: [`ON ${on.verdict}${on.reason ? `: ${on.reason}` : ''}`, `OFF ${off.verdict}${off.reason ? `: ${off.reason}` : ''}`] };
  }
  if (on.verdict === 'BLOCKED') {
    if (off.verdict !== 'BLOCKED') return { name: run.name, class: 'INVALID', reasons: [`ON BLOCKED (no original success) but OFF ${off.verdict}: inconsistent`] };
    return { name: run.name, class: 'NO_TRIGGER', reasons: [on.reason ?? 'the original lc=1 run never succeeded in loop close'] };
  }
  if (!PASSV.has(on.verdict)) return { name: run.name, class: 'INVALID', reasons: [`unrecognised ON verdict ${on.verdict}`] };
  if (off.verdict === 'BLOCKED') return { name: run.name, class: 'INSENSITIVE', reasons: ['OFF-versus-ON arm is bit-identical over the whole run: the comparison cannot show a reaction to the trigger'] };
  if (off.verdict !== 'PASS') return { name: run.name, class: 'INVALID', reasons: [`unrecognised OFF verdict ${off.verdict}`] };
  // floors (degenerate comparison guards)
  const att = check(on, 'attemptFrameSet');
  const pick = check(on, 'candidateId');
  const tr = check(on, 'traceAndWorldPointsBeforeSuccess');
  const succ = check(on, 'successCall');
  if (!(att && att.compared >= JUDGE_CONSTANTS.MIN_ATTEMPTS_COMPARED)) reasons.push(`attempts compared ${att?.compared ?? 'none'} < ${JUDGE_CONSTANTS.MIN_ATTEMPTS_COMPARED}`);
  if (!(pick && pick.compared - (pick.declaredU2 ?? 0) >= JUDGE_CONSTANTS.MIN_UNDECLARED_PICKS)) reasons.push(`undeclared candidate picks compared ${pick ? pick.compared - (pick.declaredU2 ?? 0) : 'none'} < ${JUDGE_CONSTANTS.MIN_UNDECLARED_PICKS}`);
  if (!(tr && tr.compared >= JUDGE_CONSTANTS.MIN_TRACE_FRAMES)) reasons.push(`trace frames compared before F ${tr?.compared ?? 'none'} < ${JUDGE_CONSTANTS.MIN_TRACE_FRAMES}`);
  if (!(succ && succ.compared === 1)) reasons.push('success call not compared');
  if (reasons.length) return { name: run.name, class: 'INVALID', reasons: [`comparison below the registered floors: ${reasons.join('; ')}`] };
  const declared = (on.checks ?? []).reduce((n, c) => n + (c.declaredU2 ?? 0), 0);
  return { name: run.name, class: 'EVALUATED', verdict: on.verdict, declaredU2: declared, selfCandidate: on.detail?.selfCandidate === true, reasons: [] };
}

// ---- one lc=0 run (NULL arm) -------------------------------------------------------------------------------------------------------------
// run = { name, infra, null: <compare result>|null }
export function classifyLc0Run(run) {
  if (run.infra) return { name: run.name, class: 'INFRA', reasons: [run.infra] };
  const ev = engineReplayVerdict({ null: run.replay });
  if (ev?.kind === 'FAIL') return { name: run.name, class: 'FAIL', rule: JUDGE_CONSTANTS.RULE_ENGINE_EXIT, reasons: [`engine NULL replay exited abnormally (${run.replay.exit})`] };
  if (ev?.kind === 'INFRA') return { name: run.name, class: 'INFRA', reasons: ['engine NULL replay failed for a harness reason (no space left)'] };
  const n = run.null;
  if (!n) return { name: run.name, class: 'TOOL_ERROR', reasons: ['comparator produced no result for the NULL arm'] };
  if (n.verdict === 'PASS') {
    const c = check(n, 'traceAndWorldPointsWholeRun');
    if (!(c && c.compared >= JUDGE_CONSTANTS.MIN_TRACE_FRAMES)) return { name: run.name, class: 'INVALID', reasons: [`NULL arm compared ${c?.compared ?? 'none'} frames < ${JUDGE_CONSTANTS.MIN_TRACE_FRAMES}`] };
    return { name: run.name, class: 'NULL_ZERO', reasons: [] };
  }
  if (n.verdict === 'FAIL') return { name: run.name, class: 'FAIL', reasons: [`NULL arm FAIL: the engine with loop close OFF is not bit-identical to the original lc=0 run (a nonzero measurement floor or an OFF-path deviation)${n.reason ? `: ${n.reason}` : ''}`] };
  if (n.verdict === 'TOOL_ERROR') return { name: run.name, class: 'TOOL_ERROR', reasons: [`NULL ${n.reason ?? ''}`] };
  return { name: run.name, class: 'INVALID', reasons: [`NULL arm ${n.verdict}${n.reason ? `: ${n.reason}` : ''}`] };
}

// ---- one scenario -------------------------------------------------------------------------------------------------------------------------
// input = { scenario, lc1: [classified lc=1 runs], lc0: [classified lc=0 runs], planned: {lc1, lc0}, infraFailure: string|null }
export function classifyScenario({ scenario, lc1, lc0, planned, infraFailure = null }) {
  const count = (arr, cls) => arr.filter((r) => r.class === cls).length;
  const out = {
    scenario, planned,
    lc1: { runs: lc1.length, EVALUATED: count(lc1, 'EVALUATED'), NO_TRIGGER: count(lc1, 'NO_TRIGGER'), INSENSITIVE: count(lc1, 'INSENSITIVE'), INFRA: count(lc1, 'INFRA'),
      FAIL: count(lc1, 'FAIL'), INVALID: count(lc1, 'INVALID'), TOOL_ERROR: count(lc1, 'TOOL_ERROR') },
    lc0: { runs: lc0.length, NULL_ZERO: count(lc0, 'NULL_ZERO'), INFRA: count(lc0, 'INFRA'), FAIL: count(lc0, 'FAIL'), INVALID: count(lc0, 'INVALID'), TOOL_ERROR: count(lc0, 'TOOL_ERROR') },
    declaredU2: lc1.filter((r) => r.class === 'EVALUATED').reduce((n, r) => n + (r.declaredU2 ?? 0), 0),
    selfCandidate: lc1.filter((r) => r.class === 'EVALUATED' && r.selfCandidate).length,
    reasons: [],
  };
  const failing = [...lc1, ...lc0].filter((r) => r.class === 'FAIL');
  const tool = [...lc1, ...lc0].filter((r) => r.class === 'TOOL_ERROR');
  const invalid = [...lc1, ...lc0].filter((r) => r.class === 'INVALID');
  const note = (list) => list.map((r) => `${r.name}: ${r.reasons.join(' | ')}`);
  if (failing.length) return { ...out, class: 'FAIL', verdict: 'FAIL', reasons: note(failing) };
  if (infraFailure) return { ...out, class: 'INFRA', reasons: [`infrastructure failure: ${infraFailure}`] };
  if (tool.length) return { ...out, class: 'INFRA', reasons: note(tool) };
  if (lc1.length !== planned.lc1 || lc0.length !== planned.lc0) return { ...out, class: 'INVALID', reasons: [`runs recorded ${lc1.length}+${lc0.length}, planned ${planned.lc1}+${planned.lc0}`] };
  if (invalid.length) return { ...out, class: 'INVALID', reasons: note(invalid) };
  if (out.lc1.EVALUATED > 0) {
    // an evaluated scenario needs its measurement floor: at least MIN_NULL_ZERO usable NULL-arm runs
    if (out.lc0.NULL_ZERO < JUDGE_CONSTANTS.MIN_NULL_ZERO) return { ...out, class: 'INVALID', reasons: [`NULL arm: ${out.lc0.NULL_ZERO} usable lc=0 runs < ${JUDGE_CONSTANTS.MIN_NULL_ZERO} (INFRA ${out.lc0.INFRA})`] };
    return { ...out, class: 'EVALUATED', verdict: 'PASS' };
  }
  return { ...out, class: 'BLOCKED', reasons: [`no usable lc=1 run reached a verified loop close success (no trigger ${out.lc1.NO_TRIGGER}, insensitive ${out.lc1.INSENSITIVE}, infra ${out.lc1.INFRA} of ${planned.lc1})`] };
}

// ---- the gate ------------------------------------------------------------------------------------------------------------------------------
export function aggregate(manifest, reports) {
  const names = (manifest.registered_run?.scenarios ?? []).map((s) => s.name);
  const by = {};
  for (const r of reports) {
    if (r.gate?.key !== 'lc' || r.kind !== 'gate') throw new Error(`report of ${r.gate?.key}/${r.kind} given to the lc gate verdict`);
    if (!names.includes(r.scenario)) throw new Error(`${r.scenario} is not a registered scenario`);
    if (by[r.scenario]) throw new Error(`two reports for ${r.scenario}`);
    by[r.scenario] = r.result;
  }
  const out = { gate: 'us025-lc-e2e-gate', version: 1, n_independent_registered: manifest.independent_samples?.n ?? null, K_MIN: JUDGE_CONSTANTS.K_MIN, scenarios: {}, unevaluated: [] };
  for (const n of names) { if (!by[n]) out.unevaluated.push(n); out.scenarios[n] = by[n] ?? { class: 'NOT-RUN' }; }
  const cls = (c) => Object.entries(out.scenarios).filter(([, r]) => r.class === c).map(([n, r]) => ({ scenario: n, reasons: r.reasons }));
  const planned1 = Object.values(out.scenarios).reduce((n, r) => n + (r.planned?.lc1 ?? 0), 0);
  out.infra_lc1_runs = Object.values(out.scenarios).reduce((n, r) => n + (r.lc1?.INFRA ?? 0), 0);
  out.planned_lc1_runs = planned1;
  out.infra_runs_too_many = planned1 > 0 && out.infra_lc1_runs > JUDGE_CONSTANTS.MAX_INFRA_LC1_FRACTION * planned1;
  out.fail = cls('FAIL');
  out.infra = cls('INFRA');
  out.invalid = cls('INVALID');
  out.blocked = cls('BLOCKED');
  const ev = Object.entries(out.scenarios).filter(([, r]) => r.class === 'EVALUATED');
  out.k_runs = ev.reduce((n, [, r]) => n + (r.lc1?.EVALUATED ?? 0), 0);
  out.k_scenarios = ev.length;
  out.declaredU2 = ev.reduce((n, [, r]) => n + (r.declaredU2 ?? 0), 0);
  out.selfCandidateRuns = ev.reduce((n, [, r]) => n + (r.selfCandidate ?? 0), 0);
  out.infraByScenario = Object.fromEntries(Object.entries(out.scenarios).map(([n, r]) => [n, r.lc1?.INFRA ?? null]));
  if (out.fail.length) out.verdict = 'FAIL';
  else if (out.infra.length || out.infra_runs_too_many) out.verdict = 'INFRA';
  else if (out.invalid.length) out.verdict = 'INVALID';
  else if (out.unevaluated.length) out.verdict = 'UNEVALUATED';
  else if (out.k_runs === 0) out.verdict = 'BLOCKED';
  else if (out.k_runs < JUDGE_CONSTANTS.K_MIN) out.verdict = 'INSUFFICIENT';
  else out.verdict = 'PASS';
  const allSelf = out.k_runs > 0 && out.selfCandidateRuns === out.k_runs;
  out.claim = out.verdict === 'PASS'
    ? `PASS: ${out.k_runs} original lc=1 runs of ${out.k_scenarios} scenario(s) agree with the engine up to the first loop close success (N2: nothing after it is compared)${out.k_scenarios < 2 ? '; ONE dataset only (thin sample)' : ''}`
      + `; ${out.selfCandidateRuns} of ${out.k_runs} successes were SELF-candidates (N1)${allSelf ? ': the non-self-candidate PnP / merge path and the 664 consumption of the correction are UNVERIFIED' : ''}`
      + `; ${out.infra_lc1_runs} of ${out.planned_lc1_runs} lc=1 runs were INFRA (per scenario ${JSON.stringify(out.infraByScenario)}; original SDK hang/failure, no comparison exists)`
      + '; lc=1 runs without an original success were not compared on the engine ON side; engine source-level mutants were not built (indirect comparator-mutant evidence only)'
    : `${out.verdict}: no PASS claim`;
  return out;
}

function cli() {
  const [cmd, ...rest] = process.argv.slice(2);
  const arg = (k) => { const i = rest.indexOf(k); return i < 0 ? null : rest[i + 1]; };
  if (cmd === 'constants') { console.log(JSON.stringify({ JUDGE_CONSTANTS, SCENARIO_RUNS, VERDICT_EXIT }, null, 1)); return 0; }
  if (cmd === 'verdict') {
    const manifest = JSON.parse(fs.readFileSync(arg('--manifest'), 'utf8'));
    const reports = (arg('--reports') ?? '').split(',').filter(Boolean).map((f) => JSON.parse(fs.readFileSync(f, 'utf8')));
    const agg = aggregate(manifest, reports);
    console.log(JSON.stringify(agg, null, 1));
    return VERDICT_EXIT[agg.verdict];
  }
  console.error('usage: judge.mjs verdict --manifest <m> --reports <a,b,...> | constants');
  return 2;
}
if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) process.exitCode = cli();
