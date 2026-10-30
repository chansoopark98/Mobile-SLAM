// us025-lc-e2e-gate.v2 registration evidence (DEV, read-only on the v1 gate output): re-judges the v1 first-run data with the v2 rules (judge.mjs of this directory).
//   node rejudge_v1_data.mjs --v1-root <.../us025-lc-e2e-gate-v1/us025-lc-e2e-gate> --full-dir <dir with <scenario>-<run>.json from compare_full_us025.py> --out <new json>
// Inputs per run are the STORED v1 comparator outputs (compare/<run>-on.json, -off.json) and the v1 report (replay exit codes, INFRA reasons, lc=0 classes); the
// NO_TRIGGER runs additionally need the whole-run comparison result (compare_full_us025.py on the stored v1 replay, see the registration section 2). Nothing is
// re-recorded and no v1 file is written. This is an exposure computation, NOT a gate result: v2 judges only runs recorded under v2.
import fs from 'node:fs';
import path from 'node:path';
import { parseArgs } from 'node:util';
import { aggregate, classifyLc1Run, classifyScenario, JUDGE_CONSTANTS, SCENARIO_RUNS } from './judge.mjs';

const { values } = parseArgs({ options: { 'v1-root': { type: 'string' }, 'full-dir': { type: 'string' }, out: { type: 'string' } } });
const rd = (f) => JSON.parse(fs.readFileSync(f, 'utf8'));
const compact = (res) => (res ? { verdict: res.verdict, reason: res.reason ?? null, checks: (res.checks ?? []).map((c) => ({ name: c.name, compared: c.compared ?? null, mismatch: c.mismatch ?? null,
  declaredU2: c.declaredU2 ?? null })), detail: res.detail ?? null } : null);
const names = Object.keys(SCENARIO_RUNS);
const reports = [];
const perRun = [];
for (const s of names) {
  const rep = rd(path.join(values['v1-root'], s, 'report.json'));
  const lc1 = [];
  for (const v1 of rep.classified.lc1) {
    const rec = rep.runs[v1.name];
    if (v1.class === 'INFRA') { lc1.push(classifyLc1Run({ name: v1.name, infra: rec.infra })); perRun.push({ scenario: s, run: v1.name, v1: v1.class, v2: 'INFRA' }); continue; }
    const cd = path.join(values['v1-root'], s, 'compare');
    const on = rd(path.join(cd, `${v1.name}-on.json`));
    const off = rd(path.join(cd, `${v1.name}-off.json`));
    const fullFile = path.join(values['full-dir'], `${s}-${v1.name}.json`);
    const full = on.verdict === 'BLOCKED' && fs.existsSync(fullFile) ? rd(fullFile) : null;
    const c = classifyLc1Run({ name: v1.name, infra: null, replays: { on: rec.replays.on, off: rec.replays.off }, on: compact(on), off: compact(off), full: compact(full) });
    lc1.push(c);
    perRun.push({ scenario: s, run: v1.name, v1: v1.class, v2: c.class, reasons: c.reasons });
  }
  const result = classifyScenario({ scenario: s, lc1, lc0: rep.classified.lc0, planned: rep.planned, infraFailure: rep.infraFailure });
  reports.push({ gate: { key: 'lc' }, kind: 'gate', scenario: s, result, v1Result: rep.result });
}
const manifest = { registered_run: { scenarios: names.map((name) => ({ name })) }, independent_samples: { n: 1 } };
const agg = aggregate(manifest, reports);
const changed = perRun.filter((r) => r.v1 !== r.v2);
const out = { schema: 'us025-lc-e2e-gate-v2-rejudge-of-v1-data', note: 'exposure computation on the v1 first-run data with the v2 rules; NOT a v2 gate result', judgeConstants: JUDGE_CONSTANTS,
  v1Verdict: 'INVALID (run.sh exit 8)', v2RulesOnV1Data: { verdict: agg.verdict, k_runs: agg.k_runs, k_scenarios: agg.k_scenarios, thin_lc1_runs: agg.thin_lc1_runs, reached_success_lc1_runs: agg.reached_success_lc1_runs, usable_lc1_runs: agg.usable_lc1_runs,
    thin_fraction_of_reached: agg.reached_success_lc1_runs ? agg.thin_lc1_runs / agg.reached_success_lc1_runs : null, thin_fraction_of_usable: agg.usable_lc1_runs ? agg.thin_lc1_runs / agg.usable_lc1_runs : null,
    wholeRunDeclaredPicks: agg.wholeRunDeclaredPicks, wholeRunDeclaredRing: agg.wholeRunDeclaredRing, thinRunsByScenario: agg.thinRunsByScenario, infra_lc1_runs: agg.infra_lc1_runs, wholeRunCheckedNoTrigger: agg.wholeRunCheckedNoTrigger,
    scenarios: Object.fromEntries(Object.entries(agg.scenarios).map(([n, r]) => [n, { class: r.class, lc1: r.lc1 }])), claim: agg.claim },
  runsWhoseClassChanged: changed, runs: perRun };
fs.writeFileSync(values.out, `${JSON.stringify(out, null, 1)}\n`, { flag: 'wx' });
console.log(JSON.stringify({ verdict: agg.verdict, k_runs: agg.k_runs, k_scenarios: agg.k_scenarios, thin: agg.thin_lc1_runs, reached: agg.reached_success_lc1_runs, usable: agg.usable_lc1_runs, changed: changed.map((r) => `${r.scenario}/${r.run} ${r.v1}->${r.v2}`) }, null, 1));
