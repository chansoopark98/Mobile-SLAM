// Registration manifest of us026-ba-e2e-gate.v1 / us026-mse-e2e-gate.v1 (protocol §6, §13.3; sr-held-design §3.2).
//   node make_e2e_manifest.mjs --gate ba|mse --prereg-dir <dir> --static <json> [--check]
// Writes <prereg-dir>/<gate id>.v1.manifest.json exclusively (never overwrites) from
//   * the static part (<json>: classification, tiers, tolerance, resolution, null_run, mutation, independent_samples, rules, ...;
//     written by the registration author), and
//   * the live part computed here: file / sha256 of the gate document, created_at (`date -Is`), pinned.{files, executables, trees,
//     versions}, held.{kinds, pins, inputs, seed}, registered_run.{scenarios (dataset and base bundle tree sha256, base sensors sha256),
//     config, decision_constants}.
// `review` is {} and `first_real_run_at` null: the shared device refuses the manifest until the coordinator fills the review fields.
// --check: print what would be written, write nothing.
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { parseArgs } from 'node:util';
import { GATES, livePins, root, scenarioRecord, sha256File } from './gate_e2e.mjs';

const { values } = parseArgs({ options: { gate: { type: 'string' }, 'prereg-dir': { type: 'string' }, static: { type: 'string' }, check: { type: 'boolean', default: false } } });
const g = values.gate;
const cfg = GATES[g];
if (!cfg) throw new Error('--gate ba|mse');
const pre = path.resolve(root, values['prereg-dir']);
const st = JSON.parse(fs.readFileSync(path.resolve(root, values.static), 'utf8'));
const name = `${cfg.gateId}.v${cfg.version}`;
const md = path.join(pre, `${name}.md`);
const out = path.join(pre, `${name}.manifest.json`);
const scenarios = st.registered_scenarios.map((s) => scenarioRecord(s));
for (const s of scenarios) if (cfg.devScenarios.includes(s.name)) throw new Error(`${s.name} is a dev scenario`);
const live = livePins(g);
for (const [k, v] of Object.entries({ ...live.files, ...live.executables })) if (!/^[0-9a-f]{64}$/.test(v)) throw new Error(`pin ${k} unavailable`);
const manifest = {
  gate_id: cfg.gateId,
  version: cfg.version,
  supersedes: null,
  file: `${name}.md`,
  sha256: sha256File(md),
  created_at: spawnSync('date', ['-Is'], { encoding: 'utf8' }).stdout.trim(),
  registered_by: st.registered_by,
  git_head: spawnSync('git', ['rev-parse', 'HEAD'], { cwd: root, encoding: 'utf8' }).stdout.trim(),
  tree_sha: null,
  snapshot: st.snapshot,
  ...st.fields,
  registered_run: { scenarios, dev_scenarios: cfg.devScenarios, config: live.config, decision_constants: live.decision_constants, command_file: st.command_file,
    replay_binary: st.replay_binary },
  pinned: { files: live.files, executables: live.executables, trees: live.trees, versions: live.versions },
  pins_note: 'sha256 of files and executables pin which bytes were present, not which bytes executed; shared libraries, GPU/SwiftShader data, the kernel and the CPU are not covered (protocol 12.6). The hold is an accident-prevention device, not an authentication.',
  held: { kinds: scenarios.map((s) => s.name), pins: live.held_pins, inputs: Object.fromEntries(scenarios.map((s) => [s.name, s.dataset_tree_sha256])), seed: false },
  first_run_marker: `${path.relative(root, pre)}/${name}.first-run.<scenario>.marker (one per registered scenario, created exclusively by its run; absent at registration)`,
  review: {},
  first_real_run_at: null,
};
const text = `${JSON.stringify(manifest, null, 1)}\n`;
if (values.check) { process.stdout.write(text); process.exit(0); }
fs.writeFileSync(out, text, { flag: 'wx' });
console.log(JSON.stringify({ out: path.relative(root, out), sha256: sha256File(out), scenarios: scenarios.map((s) => s.name), pins: Object.keys(live.held_pins).length }));
