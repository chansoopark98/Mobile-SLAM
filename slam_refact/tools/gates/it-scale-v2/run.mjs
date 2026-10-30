// it-scale-gate.v3 first-run driver (this directory is named it-scale-v2; v2 was withdrawn before any run) (US-015 criterion 2', E5): original-SDK A/B on two synthetic sequences derived from the coordinator seed.
//
//   node run.mjs --prereg DIR --gate it-scale-gate.v3 --env-var NAME --engine HELD_sr_e5_replay --tolerances tolerances.json --out NEWDIR [--check]
//
// --check   side-effect-free sr_held check (no marker, no seed read, nothing generated).
// Real run: sr_held heldGate (designation sha in $NAME, manifest/review/designation pins, held pins, one-run marker O_EXCL, seed handed out
// only after the marker) -> scene.py x2 (seeds derived from the coordinator seed, section 3 of the gate file) -> e5_pipeline.py per sequence
// (K=3 original SDK runs in headless Chromium on an ephemeral 127.0.0.1 server with external requests aborted, exact-input bundles, engine arms
// 0 and 1) -> premises -> e5_gate_rule.py judge -> result.json + SHA256SUMS.
// Exit codes: 0 all judged items PASS; 1 some judged item FAIL; 3 HELD (no marker: not a run); 4 INVALID after the marker (premise or tool failure);
// 2 usage. An accident guard, not an authentication (protocol 12.6). Development tool, desktop synthetic input only.
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { checkRelease, heldGate, sha256 } from '../../prereg/sr_held.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const repo = path.resolve(here, '../../../..');
const argv = process.argv.slice(2);
const args = {};
for (let i = 0; i < argv.length; i++) {
  const a = argv[i];
  if (a === '--check') args.check = true;
  else if (['--prereg', '--gate', '--env-var', '--engine', '--tolerances', '--out'].includes(a)) args[a.slice(2)] = argv[++i];
  else { console.error(`unknown argument ${a}`); process.exit(2); }
}
for (const k of ['prereg', 'gate', 'env-var', 'engine', 'tolerances']) if (!args[k]) { console.error(`usage: run.mjs --prereg DIR --gate ID.vN --env-var NAME --engine BIN --tolerances FILE --out NEWDIR [--check] (missing --${k})`); process.exit(2); }
if (!args.check && !args.out) { console.error('--out NEWDIR is required for a real run'); process.exit(2); }
const m = /^(.+)\.v([1-9][0-9]{0,5})$/.exec(args.gate);
if (!m) { console.error('--gate must be <gate_id>.v<N>'); process.exit(2); }
const [gateId, version] = [m[1], Number(m[2])];

// the pinned file set (manifest held.pins keys are the basenames): the tool chain, the original SDK files the harness serves, the engine binary, the tolerances.
const PIN_REL = [
  'slam_refact/tools/harness/e5/scene.py', 'slam_refact/tools/harness/e5/run_original.mjs', 'slam_refact/tools/harness/e5/e5_image_hook.js',
  'slam_refact/tools/harness/e5/e5_pipeline.py', 'slam_refact/tools/harness/e5/e5_extract.py', 'slam_refact/tools/harness/e5/e5_compare.py',
  'slam_refact/tools/harness/e5/prepare_bundle.py', 'slam_refact/tools/harness/prepare_inputs.py', 'slam_refact/tools/harness/divergence/make_variant.py',
  'slam_refact/tools/gates/it-scale-v2/e5_gate_rule.py', 'slam_refact/tools/gates/it-scale-v2/run.mjs', 'slam_refact/tools/gates/it-scale-v2/run.sh',
  'slam_refact/tools/prereg/sr_held.mjs',
  'slam_refact/tools/re/runtime/page-runtime.js', 'slam_refact/tools/re/runtime/memprobe.js', 'slam_refact/tools/re/runtime/input-recorder.js',
  'slam_refact/tools/re/runtime/capnp-event-queue.mjs', 'scripts/dev/references/xr_reference/sensor_adapter.mjs', 'scripts/dev/replay-android-input.mjs',
  'assets/references/xr_reference/analysis-runtime/standalone/xr.js', 'assets/references/xr_reference/analysis-runtime/standalone/xr-slam.js',
];
const py = spawnSync('python3', ['-I', '-c', 'import sys;print(sys.executable)'], { encoding: 'utf8' }).stdout.trim();
const pinPaths = [process.execPath, py, ...PIN_REL.map((p) => path.join(repo, p)), args.engine, args.tolerances];
const real = (p) => { try { return fs.realpathSync(p); } catch { console.error(`HELD [PIN_UNREADABLE]: ${p}`); process.exit(3); } };
const rq = { preregDir: path.resolve(args.prereg), gateId, version, kind: 'main', env: process.env[args['env-var']] ?? '', pins: pinPaths.map(real), wantSeed: true };

if (args.check) {
  const o = checkRelease(rq);
  console.log(o.log.join('\n'));
  console.log(o.ok ? 'RELEASED (check only: no marker created, nothing generated)' : `HELD [${o.code}]: ${o.why}`);
  process.exit(o.ok ? 0 : 3);
}
if (fs.existsSync(path.resolve(args.out))) { console.error(`precondition: ${args.out} exists (a new directory is required; nothing created)`); process.exit(2); }
const held = heldGate(rq);
console.log(held.log.join('\n'));  // never contains the seed
if (!held.ok) { console.error(`HELD [${held.code}]: ${held.why}`); process.exit(3); }

const invalid = (why) => { console.error(`INVALID (after the marker): ${why}`); process.exit(4); };
const out = path.resolve(args.out);
try {
  fs.mkdirSync(out, { recursive: false });
  const seed = BigInt(held.seed);
  const seeds = [Number(seed % 2147483647n) || 1, Number((seed * 2654435761n + 12345n) % 2147483647n) || 2];
  const names = ['seq-a', 'seq-b'];
  const verdicts = [];
  const log = [];
  for (let n = 0; n < 2; n++) {
    const seqDir = path.join(out, names[n]);
    let r = spawnSync('python3', ['-I', path.join(repo, 'slam_refact/tools/harness/e5/scene.py'), '--out', seqDir, '--seed', String(seeds[n])], { encoding: 'utf8' });
    if (r.status !== 0) invalid(`scene.py ${names[n]}: ${r.stderr}`);
    const pipeDir = path.join(out, `pipe-${names[n]}`);
    r = spawnSync('python3', ['-I', path.join(repo, 'slam_refact/tools/harness/e5/e5_pipeline.py'), '--seq', seqDir, '--out', pipeDir, '--engine', args.engine, '--k', '3', '--arms', '0,1'],
      { encoding: 'utf8', maxBuffer: 1 << 26 });
    log.push(`[E5] ${names[n]} pipeline rc=${r.status}\n${r.stdout}${r.stderr}`);
    if (r.status !== 0) invalid(`pipeline ${names[n]} rc=${r.status}: ${r.stderr}`);
    const M = JSON.parse(fs.readFileSync(path.join(pipeDir, 'metrics.json'), 'utf8'));
    // premises (gate file section 5): the original found the target in every run, GT has >=2 'in' runs and >=2 'out' runs, one of them >= 60 frames, originals processed >= 85 % of the frames
    const prem = [];
    for (const [k, o] of Object.entries(M.original)) {
      if (!(o.events.some((e) => e[0] === 'found') && o.detected >= 10)) prem.push(`P1 ${names[n]} ${k}: no found event / detected < 10`);
      if (o.processed < 0.85 * 420) prem.push(`P3 ${names[n]} ${k}: processed ${o.processed} < 85 %`);
    }
    const ins = M.gt_runs.filter((x) => x[0] === 'in').length;
    const outs = M.gt_runs.filter((x) => x[0] === 'out');
    const longOut = outs.filter((x) => x[2] - x[1] + 1 >= 60).length;
    if (ins < 2 || outs.length < 2 || longOut < 1) prem.push(`P2 ${names[n]}: GT runs in=${ins} out=${outs.length} longOut(>=60)=${longOut}`);
    if (prem.length) invalid(prem.join('; '));
    const vp = path.join(out, `verdict-${names[n]}.json`);
    r = spawnSync('python3', ['-I', path.join(repo, 'slam_refact/tools/gates/it-scale-v2/e5_gate_rule.py'), 'judge', '--metrics', path.join(pipeDir, 'metrics.json'),
      '--tol', path.resolve(args.tolerances), '--gt', path.join(seqDir, 'gt.json'), '--out', vp], { encoding: 'utf8' });
    if (r.status !== 0) invalid(`judge ${names[n]}: ${r.stderr}`);
    const vj = JSON.parse(fs.readFileSync(vp, 'utf8'));
    verdicts.push({ name: names[n], all_pass: vj.all_pass, all_pass_by_arm: vj.all_pass_by_arm });
  }
  const allPass = verdicts.every((v) => v.all_pass);
  const byArm = {};
  for (const v of verdicts) for (const [arm, ok] of Object.entries(v.all_pass_by_arm)) byArm[arm] = (byArm[arm] ?? true) && ok;
  fs.writeFileSync(path.join(out, 'result.json'), JSON.stringify({ schema: 'e5-gate-result-v1', gate: args.gate, verdicts, all_pass: allPass, all_pass_by_arm: byArm,
    scope: 'desktop headless SwiftShader, synthetic E5 room, original reference runtime SDK vs engine (exact-input bundles), not device performance' }, null, 1) + '\n', { flag: 'wx' });
  fs.writeFileSync(path.join(out, 'run.log'), log.join('\n'), { flag: 'wx' });
  const files = [];
  const walk = (d) => { for (const e of fs.readdirSync(d, { withFileTypes: true })) { const p = path.join(d, e.name); e.isDirectory() ? walk(p) : files.push(p); } };
  walk(out);
  files.sort();
  fs.writeFileSync(path.join(out, 'SHA256SUMS'), files.map((f) => `${sha256(fs.readFileSync(f))}  ${path.relative(out, f)}\n`).join(''), { flag: 'wx' });
  console.log(`[E5-GATE] ${allPass ? 'ALL PASS' : 'NOT ALL PASS'} by arm ${JSON.stringify(byArm)} ${JSON.stringify(verdicts)}`);
  process.exit(allPass ? 0 : 1);
} catch (e) {
  invalid(String(e && e.stack ? e.stack : e));
}
