// it-scale-gate.v5 first-run driver (v5 = v4 + the OUTPUT-PATH fix only: --out must be under build/slam-refact-pkgh-20261022, which run_original.mjs requires;
// plus a marker-free --smoke mode that runs ONE dev-seed sequence through this very code path). Derived from it-scale-v4/run.mjs; the rule, controls and
// REPORT scripts are the unchanged it-scale-v4 files.  v4 first run: INVALID, run_original.mjs refused the --out path, nothing measured.
// it-scale-gate.v4 first-run driver (US-015 criterion 2', E5): original-SDK A/B on THREE synthetic sequences derived from the coordinator seed,
// FIVE original SDK runs per sequence (the in-run original-vs-original null), the engine on every run's exact-input bundle for both
// world-tracker ON (the GATE arm; arm 0, a pre-fix engine and cross pairs are REPORT only), positive-control engine runs, and the v4 rule (tolerance = 3 x in-run null; gate file it-scale-gate.v4.md section 5).
//
//   node run.mjs --prereg DIR --gate it-scale-gate.v4 --env-var NAME --engine HELD_sr_e5_replay --prefix-engine HELD_sr_e5_replay_prefix --out NEWDIR [--check]
//   node run.mjs --prereg DIR --gate it-scale-gate.v5 --engine BIN --prefix-engine BIN --smoke --smoke-seed N --out NEWDIR   (NO marker, NO coordinator seed; N must be in manifest held.dev_seeds)
//   node run.mjs --engine HELD_sr_e5_replay --prefix-engine HELD_sr_e5_replay_prefix --print-pins     (side-effect free: the pin set {basename: sha256})
//
// --check   side-effect-free sr_held check (no marker, no seed read, nothing generated).
// Real run: sr_held heldGate (designation sha in $NAME, manifest/review/designation pins, held pins, one-run marker O_EXCL, seed handed out
// only after the marker) -> per sequence: scene.py (seed = sha256 derivation of the coordinator seed, gate file section 3.2) -> e5_pipeline.py
// --k 5 --arms 1 -> v4_report_runs.py (REPORT only) -> premises P1-P3 -> v4_controls.py -> e5_gate_rule_v4.py judge -> after the loop e5_gate_rule_v4.py final -> result.json + SHA256SUMS.
// Exit codes: 0 PASS; 1 FAIL; 5 INSENSITIVE (all checks passed, a positive control was not detected: no PASS may be claimed);
// 4 INVALID after the marker (premise or tool failure, no FAIL anywhere); 3 HELD (no marker: not a run); 2 usage; 6 lane-precheck (run.sh).
// FAIL-first: a FAIL of a valid sequence stands even if another sequence is INVALID.  A premise failure (P1-P3, P5) marks THAT sequence INVALID
// and the loop continues; a tool failure (non-zero exit of scene/pipeline/controls/judge) marks the sequence INVALID and STOPS the loop.
// An accident guard, not an authentication (protocol 12.6).  Development tool, desktop synthetic input only.
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { fileURLToPath } from 'node:url';
import { checkRelease, heldGate, sha256 } from '../../prereg/sr_held.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const repo = path.resolve(here, '../../../..');
const argv = process.argv.slice(2);
const args = {};
for (let i = 0; i < argv.length; i++) {
  const a = argv[i];
  if (a === '--check') args.check = true;
  else if (a === '--print-pins') args.printPins = true;
  else if (a === '--smoke') args.smoke = true;
  else if (['--prereg', '--gate', '--env-var', '--engine', '--prefix-engine', '--out', '--smoke-seed'].includes(a)) args[a.slice(2)] = argv[++i];
  else { console.error(`unknown argument ${a}`); process.exit(2); }
}
for (const k of args.printPins ? ['engine', 'prefix-engine'] : args.smoke ? ['prereg', 'gate', 'engine', 'prefix-engine', 'smoke-seed'] : ['prereg', 'gate', 'env-var', 'engine', 'prefix-engine']) if (!args[k]) { console.error(`usage: run.mjs --prereg DIR --gate ID.vN --env-var NAME --engine BIN --out NEWDIR [--check] (missing --${k})`); process.exit(2); }
if (!args.check && !args.printPins && !args.out) { console.error('--out NEWDIR is required for a real run'); process.exit(2); }
const m = /^(.+)\.v([1-9][0-9]{0,5})$/.exec(args.gate ?? 'x.v1');
if (!m) { console.error('--gate must be <gate_id>.v<N>'); process.exit(2); }
const [gateId, version] = [m[1], Number(m[2])];
// v5 fix: run_original.mjs (pinned package-H harness file) refuses any output root outside this directory (run_original.mjs:93-94,97). A real run and a smoke
// must therefore put --out (and with it every pipeline directory) below it; checked BEFORE any marker (usage error 2). v4 put --out elsewhere and was INVALID.
const OUT_ROOT = path.join(repo, 'build/slam-refact-pkgh-20261022');
if (!args.check && !args.printPins) {
  const o = path.resolve(args.out ?? '');
  if (!args.out || !o.startsWith(OUT_ROOT + path.sep)) { console.error(`precondition: --out must be a new directory under ${OUT_ROOT} (run_original.mjs refuses other output roots); refused before any marker`); process.exit(2); }
}
if (args.smoke && args.check) { console.error('--smoke and --check are exclusive'); process.exit(2); }

const N_ORIG = 5;
const FRAMES = 420;   // = scene.py default 28 s x 15 fps (scene.py is invoked without --frames/--fps/--duration)
// the pinned file set (manifest held.pins keys are the basenames): the tool chain, the original SDK files the harness serves, the engine binary.
const PIN_REL = [
  'slam_refact/tools/harness/e5/scene.py', 'slam_refact/tools/harness/e5/run_original.mjs', 'slam_refact/tools/harness/e5/e5_image_hook.js',
  'slam_refact/tools/harness/e5/e5_pipeline.py', 'slam_refact/tools/harness/e5/e5_extract.py', 'slam_refact/tools/harness/e5/e5_compare.py',
  'slam_refact/tools/harness/e5/prepare_bundle.py', 'slam_refact/tools/harness/prepare_inputs.py', 'slam_refact/tools/harness/divergence/make_variant.py',
  'slam_refact/tools/gates/it-scale-v4/e5_gate_rule_v4.py', 'slam_refact/tools/gates/it-scale-v4/v4_controls.py', 'slam_refact/tools/gates/it-scale-v4/v4_report_runs.py',
  'slam_refact/tools/gates/it-scale-v5/run.mjs', 'slam_refact/tools/gates/it-scale-v5/run.sh', 'slam_refact/tools/gates/it-scale-v5/smoke.sh',
  'slam_refact/tools/prereg/sr_held.mjs',
  'slam_refact/tools/re/runtime/page-runtime.js', 'slam_refact/tools/re/runtime/memprobe.js', 'slam_refact/tools/re/runtime/input-recorder.js',
  'slam_refact/tools/re/runtime/capnp-event-queue.mjs', 'scripts/dev/references/xr_reference/sensor_adapter.mjs', 'scripts/dev/replay-android-input.mjs',
  'assets/references/xr_reference/analysis-runtime/standalone/xr.js', 'assets/references/xr_reference/analysis-runtime/standalone/xr-slam.js',
];
// The browser the original SDK runs in is part of the measurement (the engine's ROI model reproduces SwiftShader's float32 draw): the harness
// launches /usr/bin/google-chrome unless BROWSER_EXECUTABLE is set. A different BROWSER_EXECUTABLE is refused BEFORE the marker (usage error 2);
// the variable is removed for the children, and Chrome + its SwiftShader libraries are pinned by sha256 below.
const CHROME = '/usr/bin/google-chrome';
if (process.env.BROWSER_EXECUTABLE) {
  let ok = false;
  try { ok = fs.realpathSync(process.env.BROWSER_EXECUTABLE) === fs.realpathSync(CHROME); } catch { ok = false; }
  if (!ok) { console.error(`precondition: BROWSER_EXECUTABLE=${process.env.BROWSER_EXECUTABLE} is not ${CHROME}; refused before any marker`); process.exit(2); }
  delete process.env.BROWSER_EXECUTABLE;
}
const chromeDir = path.dirname(fs.realpathSync(CHROME));
const BROWSER_PINS = [CHROME, path.join(chromeDir, 'chrome'), path.join(chromeDir, 'libvk_swiftshader.so'), path.join(chromeDir, 'libEGL.so'),
  path.join(chromeDir, 'libGLESv2.so'), path.join(chromeDir, 'vk_swiftshader_icd.json')];
const py = spawnSync('python3', ['-I', '-c', 'import sys;print(sys.executable)'], { encoding: 'utf8' }).stdout.trim();
const pinPaths = [process.execPath, py, ...PIN_REL.map((p) => path.join(repo, p)), ...BROWSER_PINS, args.engine, args['prefix-engine']];
const real = (p) => { try { return fs.realpathSync(p); } catch { console.error(`HELD [PIN_UNREADABLE]: ${p}`); process.exit(3); } };
if (args.printPins) {   // side-effect free: the pin set as {basename: sha256} (used by make_manifest.py)
  const o = {};
  for (const p of pinPaths.map(real)) o[path.basename(p)] = sha256(fs.readFileSync(p));
  console.log(JSON.stringify(o, null, 1));
  process.exit(0);
}
const rq = { preregDir: path.resolve(args.prereg ?? '.'), gateId, version, kind: 'main', env: process.env[args['env-var'] ?? ''] ?? '', pins: pinPaths.map(real), wantSeed: true };

if (args.check) {
  const o = checkRelease(rq);
  console.log(o.log.join('\n'));
  console.log(o.ok ? 'RELEASED (check only: no marker created, nothing generated)' : `HELD [${o.code}]: ${o.why}`);
  process.exit(o.ok ? 0 : 3);
}
if (fs.existsSync(path.resolve(args.out))) { console.error(`precondition: ${args.out} exists (a new directory is required; nothing created)`); process.exit(2); }
// smoke: no sr_held call at all (no marker, no coordinator seed read); the sequence seed must be a declared DEV seed of the manifest (held.dev_seeds)
let smokeSeed = null;
if (args.smoke) {
  let dev = [];
  try { dev = JSON.parse(fs.readFileSync(path.join(path.resolve(args.prereg), `${gateId}.v${version}.manifest.json`), 'utf8')).held.dev_seeds ?? []; } catch { dev = []; }
  if (!/^[0-9]+$/.test(args['smoke-seed']) || !dev.includes(args['smoke-seed'])) { console.error(`precondition: --smoke-seed ${args['smoke-seed']} is not in the manifest held.dev_seeds ${JSON.stringify(dev)}; refused`); process.exit(2); }
  smokeSeed = Number(args['smoke-seed']);
}
const held = args.smoke ? { ok: true, log: ['[SMOKE] no sr_held, no marker, no coordinator seed; dev seed ' + smokeSeed], seed: null } : heldGate(rq);
console.log(held.log.join('\n'));  // never contains the seed
if (!held.ok) { console.error(`HELD [${held.code}]: ${held.why}`); process.exit(3); }

const out = path.resolve(args.out);
const names = args.smoke ? ['seq-smoke'] : ['seq-a', 'seq-b', 'seq-c'];
// seed_n = (first 8 bytes of sha256("<gate_id>.v<N>:<S>:<n>") as an unsigned integer) mod 2147483647, 0 -> n + 1 (gate file section 3.2)
const deriveSeed = (S, n) => {
  const d = createHash('sha256').update(`${gateId}.v${version}:${S}:${n}`).digest();
  return Number(d.readBigUInt64BE(0) % 2147483647n) || n + 1;
};
const py4 = (script, ...a) => spawnSync('python3', ['-I', script, ...a], { encoding: 'utf8', maxBuffer: 1 << 26 });
const rule = path.join(repo, 'slam_refact/tools/gates/it-scale-v4/e5_gate_rule_v4.py');
const verdictFiles = [];
const log = [];
let stopped = null;
try {
  fs.mkdirSync(out, { recursive: false });
  const S = held.seed;
  const seeds = args.smoke ? [smokeSeed] : names.map((_, n) => deriveSeed(S, n + 1));
  if (new Set(seeds).size !== seeds.length) throw new Error('derived sequence seeds are not distinct');
  for (let n = 0; n < names.length && stopped === null; n++) {
    const name = names[n];
    const seqDir = path.join(out, name);
    const pipeDir = path.join(out, `pipe-${name}`);
    const vp = path.join(out, `verdict-${name}.json`);
    const markInvalid = (why) => { fs.writeFileSync(vp, JSON.stringify({ invalid: `${name}: ${why}` }), { flag: 'wx' }); verdictFiles.push(vp); log.push(`[E5] ${name} INVALID: ${why}`); };
    const toolFail = (why) => { markInvalid(why); stopped = why; };
    let r = py4(path.join(repo, 'slam_refact/tools/harness/e5/scene.py'), '--out', seqDir, '--seed', String(seeds[n]));
    if (r.status !== 0) { toolFail(`scene.py rc=${r.status}: ${r.stderr}`); break; }
    r = py4(path.join(repo, 'slam_refact/tools/harness/e5/e5_pipeline.py'), '--seq', seqDir, '--out', pipeDir, '--engine', args.engine, '--k', String(N_ORIG), '--arms', '1');
    log.push(`[E5] ${name} pipeline rc=${r.status}\n${r.stdout}${r.stderr}`);
    if (r.status !== 0) { toolFail(`pipeline rc=${r.status}: ${r.stderr}`); break; }
    const metricsPath = path.join(pipeDir, 'metrics.json');
    const M = JSON.parse(fs.readFileSync(metricsPath, 'utf8'));
    // REPORT-only runs (arm 0, pre-fix engine control, cross pairs): a failure is logged and never makes the sequence INVALID
    const reportPath = path.join(pipeDir, 'report.json');
    r = py4(path.join(repo, 'slam_refact/tools/gates/it-scale-v4/v4_report_runs.py'), '--pipe', pipeDir, '--seq', seqDir, '--engine', args.engine, '--prefix-engine', args['prefix-engine'], '--out', reportPath);
    log.push(`[E5] ${name} report-runs rc=${r.status}\n${r.stdout}${r.stderr}`);
    const controlsPath = path.join(pipeDir, 'controls.json');
    r = py4(path.join(repo, 'slam_refact/tools/gates/it-scale-v4/v4_controls.py'), '--pipe', pipeDir, '--seq', seqDir, '--engine', args.engine, '--out', controlsPath);
    log.push(`[E5] ${name} controls rc=${r.status}\n${r.stdout}${r.stderr}`);
    if (r.status !== 0) { toolFail(`controls rc=${r.status}: ${r.stderr}`); break; }
    // premises (gate file section 6): P1 every original found the target and detected >= 10 frames, P3 originals processed >= 85 % of the frames,
    // P2 GT has >= 2 'in' runs and >= 2 'out' runs, one of them >= 60 frames.  (P5, the anchor premise, is in the rule: it raises INVALID.)
    const prem = [];
    for (const [k, o] of Object.entries(M.original)) {
      if (!(o.events.some((e) => e[0] === 'found') && o.detected >= 10)) prem.push(`P1 ${k}: no found event / detected < 10`);
      if (o.processed < 0.85 * FRAMES) prem.push(`P3 ${k}: processed ${o.processed} < 85 %`);
    }
    if (Object.keys(M.original).length !== N_ORIG) prem.push(`P1: ${Object.keys(M.original).length} original runs, expected ${N_ORIG}`);
    const ins = M.gt_runs.filter((x) => x[0] === 'in').length;
    const outs = M.gt_runs.filter((x) => x[0] === 'out');
    const longOut = outs.filter((x) => x[2] - x[1] + 1 >= 60).length;
    if (ins < 2 || outs.length < 2 || longOut < 1) prem.push(`P2: GT runs in=${ins} out=${outs.length} longOut(>=60)=${longOut}`);
    if (prem.length) { markInvalid(prem.join('; ')); continue; }
    r = py4(rule, 'judge', '--metrics', metricsPath, '--gt', path.join(seqDir, 'gt.json'), '--controls', controlsPath, '--out', vp, ...(fs.existsSync(reportPath) ? ['--report', reportPath] : []));
    log.push(`[E5] ${name} judge rc=${r.status}: ${r.stdout}${r.stderr}`);
    if (r.status === 4) { markInvalid(r.stdout.trim()); continue; }          // P5 or an inconsistent metrics file raised Invalid in the rule
    if (![0, 1, 5].includes(r.status)) { toolFail(`judge rc=${r.status}: ${r.stderr}`); break; }
    verdictFiles.push(vp);
  }
} catch (e) {
  stopped = String(e && e.stack ? e.stack : e);
  log.push(`[E5] exception after the marker: ${stopped}`);
}
if (args.smoke) {   // smoke: report which stages ran; the verdict of the dev sequence is information only (no final(), 1 of 3 sequences)
  fs.writeFileSync(path.join(out, 'run.log'), log.join('\n'), { flag: 'wx' });
  let v = null;
  try { v = JSON.parse(fs.readFileSync(verdictFiles[0], 'utf8')); } catch { v = null; }
  const complete = v !== null && 'status' in v && !stopped;   // every stage up to and including the rule's judge ran (the judged status itself is information only)
  const summary = { smoke: true, seed: smokeSeed, out, stopped, sequence_verdict: v === null ? null : ('status' in v ? v.status : v), stages_logged: log.map((l) => l.split('\n')[0].slice(0, 80)) };
  fs.writeFileSync(path.join(out, 'smoke-summary.json'), JSON.stringify(summary, null, 1), { flag: 'wx' });
  console.log(`[E5-SMOKE] ${complete ? 'COMPLETE' : 'INCOMPLETE'} ${JSON.stringify(summary.sequence_verdict).slice(0, 200)}`);
  process.exit(complete ? 0 : 4);
}
if (verdictFiles.length === 0) { console.error(`INVALID (after the marker, no verdict): ${stopped}`); process.exit(4); }
const resultPath = path.join(out, 'result.json');
const f = py4(rule, 'final', '--verdicts', ...verdictFiles, '--out', resultPath);
log.push(`[E5] final rc=${f.status}: ${f.stdout}${f.stderr}`);
fs.writeFileSync(path.join(out, 'run.log'), log.join('\n'), { flag: 'wx' });
const files = [];
const walk = (d) => { for (const e of fs.readdirSync(d, { withFileTypes: true })) { const p = path.join(d, e.name); e.isDirectory() ? walk(p) : files.push(p); } };
walk(out);
files.sort();
fs.writeFileSync(path.join(out, 'SHA256SUMS'), files.map((x) => `${sha256(fs.readFileSync(x))}  ${path.relative(out, x)}\n`).join(''), { flag: 'wx' });
console.log(`[E5-GATE] ${f.stdout.trim()}${stopped ? ` (stopped: ${stopped.slice(0, 200)})` : ''}`);
process.exit([0, 1, 4, 5].includes(f.status) ? f.status : 4);
