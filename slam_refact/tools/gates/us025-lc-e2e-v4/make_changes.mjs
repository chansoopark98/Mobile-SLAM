// us025-lc-e2e-gate.v4: numeric diff v3 -> v4 of the decision constants, run counts and the registered configuration (protocol 13.2: statements about "tightened / loosened /
// unchanged" come from this output, not from prose). READ-ONLY on both gate directories. FORK of ../us025-lc-e2e-v3/make_changes.mjs (v2 -> v3): the comparison sides moved by one version.
// D-V3-LOCK: the expected result is that no decision constant, run count or verdict exit changes and that the configuration changes only in the execution keys (slots, CPU sets, L_SLOT, jobs, phases).
//   node make_changes.mjs --out <new json>
import fs from 'node:fs';
import { parseArgs } from 'node:util';
import * as J1 from '../us025-lc-e2e-v3/judge.mjs';
import * as G1 from '../us025-lc-e2e-v3/gate_lc.mjs';
import * as J2 from './judge.mjs';
import * as G2 from './gate_lc.mjs';

const { values } = parseArgs({ options: { out: { type: 'string' } } });
const flat = (o, p = '') => Object.entries(o ?? {}).flatMap(([k, v]) => (v && typeof v === 'object' && !Array.isArray(v) ? flat(v, `${p}${k}.`) : [[`${p}${k}`, v]]));
const diff = (a, b) => {
  const A = Object.fromEntries(flat(a));
  const B = Object.fromEntries(flat(b));
  return Object.keys({ ...A, ...B }).sort().map((k) => ({ key: k, v3: A[k] ?? null, v4: B[k] ?? null, change: !(k in A) ? 'added' : !(k in B) ? 'removed' : JSON.stringify(A[k]) === JSON.stringify(B[k]) ? 'unchanged'
    : typeof A[k] === 'number' && typeof B[k] === 'number' ? (B[k] > A[k] ? 'raised' : 'lowered') : 'changed' }));
};
const cfg = (G) => { const c = G.configRecord(); return c; };
const out = {
  schema: 'us025-lc-e2e-gate-v4-changes-v1',
  judgeConstants: diff(J1.JUDGE_CONSTANTS, J2.JUDGE_CONSTANTS),
  scenarioRuns: diff(J1.SCENARIO_RUNS, J2.SCENARIO_RUNS),
  verdictExit: diff(J1.VERDICT_EXIT, J2.VERDICT_EXIT),
  configuration: diff(cfg(G1), cfg(G2)),
  gate: { id: [G1.GATE_ID, G2.GATE_ID], version: [G1.GATE_VERSION, G2.GATE_VERSION], pinDir: [G1.PIN_DIR, G2.PIN_DIR], replayBin: [G1.REPLAY_BIN, G2.REPLAY_BIN], outDefault: [G1.OUT_DEFAULT, G2.OUT_DEFAULT] },
  pinnedFiles: { v3Only: G1.PINNED_FILES().filter((f) => !G2.PINNED_FILES().includes(f)), v4Only: G2.PINNED_FILES().filter((f) => !G1.PINNED_FILES().includes(f)) },
};
out.summary = {
  changedOrAdded: [...out.judgeConstants, ...out.scenarioRuns, ...out.verdictExit, ...out.configuration].filter((r) => r.change !== 'unchanged').map((r) => `${r.key}: ${r.change} (${JSON.stringify(r.v3)} -> ${JSON.stringify(r.v4)})`),
};
fs.writeFileSync(values.out, `${JSON.stringify(out, null, 1)}\n`, { flag: 'wx' });
console.log(JSON.stringify(out.summary, null, 1), JSON.stringify(out.gate), JSON.stringify(out.pinnedFiles));
