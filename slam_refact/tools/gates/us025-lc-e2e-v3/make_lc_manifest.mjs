// Registration manifest of us025-lc-e2e-gate.v2 (protocol §6, §13.3; sr-held-design §3.2). FORK of us026-ba-e2e-v2/make_e2e_manifest.mjs plus the engine pin.
//   node make_lc_manifest.mjs --prereg-dir <dir> --static <json> --engine-commit <40-hex commit | PENDING-PIN> [--replay-binary <path>] [--check]
// * --engine-commit PENDING-PIN (coordinator message D-G10: kContractVersion 9 -> 10 lands in a separate lane): engine_pin.commit and
//   engine_pin.replay_binary_sha256 are the text PENDING-PIN, the replay binary is not pinned, and the file written is <gate id>.v2.manifest.pending.json
//   (NOT the name the runner reads, so a PENDING manifest can never be used for a run; gate_lc.mjs also refuses PENDING-PIN with HELD).
// * --engine-commit <sha>: after build_pin.sh <sha>. The commit must be an ancestor of HEAD, the replay binary must exist, and engine_pin records the
//   non-test engine files changed since 0a5c5c4 (the commit that added loop close), the binary sha256 and the pin record's sha256. The manifest is written
//   exclusively (never overwritten) as <gate id>.v2.manifest.json.
// `review` is {} and `first_real_run_at` null: the shared device refuses the manifest until the coordinator fills the review fields.
// --check: print what would be written, write nothing.
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { parseArgs } from 'node:util';
import { ENGINE_BASE_COMMIT, GATE_ID, GATE_VERSION, PENDING, PIN_DIR, REPLAY_BIN, DEV_SCENARIOS, gitReal, livePins, root, scenarioRecord, sha256File } from './gate_lc.mjs';
import { SCENARIO_RUNS } from './judge.mjs';

const { values } = parseArgs({ options: { 'prereg-dir': { type: 'string' }, static: { type: 'string' }, 'engine-commit': { type: 'string' }, 'replay-binary': { type: 'string' }, check: { type: 'boolean', default: false } } });
const ec = values['engine-commit'];
if (!ec) throw new Error('--engine-commit <40-hex commit | PENDING-PIN> is required');
const pending = ec === PENDING;
if (!pending && !/^[0-9a-f]{40}$/.test(ec)) throw new Error('--engine-commit must be a 40-hex commit or PENDING-PIN');
const pre = path.resolve(root, values['prereg-dir']);
const st = JSON.parse(fs.readFileSync(path.resolve(root, values.static), 'utf8'));
const name = `${GATE_ID}.v${GATE_VERSION}`;
const md = path.join(pre, `${name}.md`);
const out = path.join(pre, pending ? `${name}.manifest.pending.json` : `${name}.manifest.json`);
const scenarios = Object.keys(SCENARIO_RUNS).map((s) => scenarioRecord(s));
for (const s of scenarios) if (DEV_SCENARIOS.includes(s.name)) throw new Error(`${s.name} is a dev scenario`);
const live = livePins({ withBinary: !pending });
for (const [k, v] of Object.entries({ ...live.files, ...live.executables })) if (!/^[0-9a-f]{64}$/.test(v)) throw new Error(`pin ${k} unavailable`);

const base = gitReal('rev-parse', `${ENGINE_BASE_COMMIT}^{commit}`);
if (base.status !== 0) throw new Error(`cannot resolve ${ENGINE_BASE_COMMIT}`);
const enginePin = { commit: pending ? PENDING : ec, replay_binary: REPLAY_BIN, replay_binary_sha256: PENDING, pin_record: null, pin_record_sha256: null,
  base_commit: base.out, base_commit_meaning: 'the commit that added loop close behind SLAM_REFACT_LOOP_CLOSE (0a5c5c4)', engine_diff_since_base: null, engine_sources_unchanged_since_base: null,
  build: 'build_pin.sh <commit>: git archive snapshot, Release, -Werror, SLAM_REFACT_LOOP_CLOSE=ON, jobs 2; link with the lc6a --wrap capture (lc6a_replay_main.cc)',
  rule: 'the runner holds unless HEAD has no non-test engine change since engine_pin.commit and the replay binary sha256 equals engine_pin.replay_binary_sha256' };
if (!pending) {
  if (gitReal('merge-base', '--is-ancestor', ec, 'HEAD').status !== 0) throw new Error(`${ec} is not an ancestor of HEAD`);
  const bin = path.resolve(root, values['replay-binary'] ?? REPLAY_BIN);
  if (path.relative(root, bin) !== REPLAY_BIN) throw new Error(`--replay-binary must be ${REPLAY_BIN} (the path the runner uses)`);
  enginePin.replay_binary_sha256 = sha256File(bin);
  const rec = path.resolve(root, PIN_DIR, 'pin.json');
  if (fs.existsSync(rec)) {
    const pj = JSON.parse(fs.readFileSync(rec, 'utf8'));
    if (pj.commit !== ec) throw new Error(`pin.json is for ${pj.commit}, not ${ec}`);
    if (pj.binary?.sha256 !== enginePin.replay_binary_sha256) throw new Error('pin.json binary sha256 differs from the binary');
    enginePin.pin_record = path.relative(root, rec);
    enginePin.pin_record_sha256 = sha256File(rec);
  } else throw new Error(`${path.relative(root, rec)} missing: run build_pin.sh ${ec} first`);
}
const diffRef = pending ? 'HEAD' : ec;
const d = gitReal('diff', '--name-only', base.out, diffRef, '--', 'slam_refact/engine', ':(exclude)slam_refact/engine/tests');
if (d.status !== 0) throw new Error(`git diff failed: ${d.err}`);
enginePin.engine_diff_since_base = d.out ? d.out.split('\n') : [];
enginePin.engine_sources_unchanged_since_base = enginePin.engine_diff_since_base.length === 0;
enginePin.engine_diff_ref = pending ? 'HEAD at registration (the D-G10 contract version change is NOT in it yet)' : ec;

const manifest = {
  gate_id: GATE_ID,
  version: GATE_VERSION,
  supersedes: st.supersedes ?? null,
  file: `${name}.md`,
  sha256: sha256File(md),
  created_at: spawnSync('date', ['-Is'], { encoding: 'utf8' }).stdout.trim(),
  registered_by: st.registered_by,
  git_head: gitReal('rev-parse', 'HEAD').out,
  tree_sha: null,
  snapshot: st.snapshot,
  ...st.fields,
  engine_pin: enginePin,
  registered_run: { scenarios, dev_scenarios: DEV_SCENARIOS, config: live.config, decision_constants: live.decision_constants, command_file: st.command_file },
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
console.log(JSON.stringify({ out: path.relative(root, out), sha256: sha256File(out), pending, scenarios: scenarios.map((s) => s.name), pins: Object.keys(live.held_pins).length }));
