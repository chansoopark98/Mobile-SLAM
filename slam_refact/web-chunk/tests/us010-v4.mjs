// US-010 samples v4: the harness part that is not a measurement (code only; the registration, the first run and the pin list are the coordinator's, after the
// freeze). Four things the v3 closure audit (B) and the v3 post-run review (U10-1, U10-2, U10-5) asked for:
//   check-manifest  the registered run is refused unless the manifest says mutation.detected === true and null_run.result === 'PASS' BEFORE the first run
//                   (v3 ran with mutation.detected null; an addendum cannot change it, protocol 6)
//   freeze          the CORE of the registered run - the chunk, the samples, the measuring and driving scripts, this file, the gate-common helpers, the replay
//                   driver tools/gates/us010-samples-v4/run.sh - and the external bytes (node, Chromium binary and wrapper, the SDK xr.js, three.js, A-Frame,
//                   the engine files, any --extra file such as the capture index) are summed into ONE pin list, written ONCE (exclusive create, read-only; an
//                   existing file is refused: v3 rewrote its SHA256SUMS after the registered runs, review U10-2). The replay driver and its sha256 are named in it.
//   verify          recompute and compare: every pinned file, every external entry, and files that appeared in the core roots after the freeze. Run before
//                   every group of registered runs and after the last one.
// GRADE (do not skip): this harness is the same author's code and oracle as the samples it measures. US-010 v4 is a GATE only if the world-lock criterion has an
// ORACLE INDEPENDENT of this code's author (plan r4b D2 / Fable M-8); with the same author's oracle the result is REPORT. Nothing in this file decides the grade
// (the registration does); it only keeps a REPORT honest. The held device (first-run marker, designation) is NOT wired here: a REPORT run needs none, a GATE
// run would call tools/gate-common/held.mjs around run-sample-smoke.mjs as the OR-1 and rm4c harnesses do.
//   node slam_refact/web-chunk/tests/us010-v4.mjs check-manifest --manifest <file>
//   node slam_refact/web-chunk/tests/us010-v4.mjs freeze --out <new pin list under build/> --manifest <file> --engine-dir <dir> [--snapshot-dir <new dir>] [--extra <file>]...
//   node slam_refact/web-chunk/tests/us010-v4.mjs verify --pins <pin list> --engine-dir <dir>
import { spawnSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';
import { parseArgs } from 'node:util';
import { fileURLToPath } from 'node:url';
import { AFRAME_DIR, SDK_DIR, THREE_DIR } from '../samples/serve.mjs';
import { copyInto, makeReadOnly, writeSums } from '../tools/gate-common/freeze.mjs';

export const REPLAY_DRIVER = 'slam_refact/tools/gates/us010-samples-v4/run.sh';
// [directory, file name pattern, recursive]: the core of the registered run. Whatever matches is pinned at the freeze; a match that appears later is reported by verify.
export const CORE_ROOTS = [
  ['slam_refact/web-chunk', /^[^/]+\.js$/, false],
  ['slam_refact/web-chunk/samples', /./, true],
  ['slam_refact/web-chunk/tests', /^(run-sample-smoke|run-sample-ablation|run-exec-model|run-us010b-series|run-pose-nullrun|summarize-us010b|us010-v4|run-us010-v4-mutants)\.mjs$|^sample-(smoke-criteria|timing|fixture)\.mjs$|^sample-input\.js$|^sample-smoke-criteria\.test\.mjs$|^us010-v4\.test\.mjs$/, false],
  ['slam_refact/web-chunk/tests/worker-replay', /\.js$/, false],
  ['slam_refact/web-chunk/tests/fake-chunk', /\.js$/, false],
  ['slam_refact/web-chunk/tools/gate-common', /\.mjs$/, false],
  ['slam_refact/tools/gates/us010-samples-v4', /./, false],
];

export const sha256File = (file) => createHash('sha256').update(fs.readFileSync(file)).digest('hex');
const sha256Of = (file) => { try { return sha256File(file); } catch { return null; } };

function walk(dir, re, recursive, rel = '') {
  const out = [];
  let names;
  try { names = fs.readdirSync(path.join(dir, rel)); } catch { return out; }
  for (const name of names.sort()) {
    const r = rel ? `${rel}/${name}` : name;
    const st = fs.lstatSync(path.join(dir, r));
    if (st.isDirectory()) { if (recursive) out.push(...walk(dir, re, recursive, r)); } else if (st.isFile() && re.test(name)) out.push(r);
  }
  return out;
}

// the core files (repository-relative, sorted)
export const coreFiles = (repo, roots = CORE_ROOTS) => roots.flatMap(([dir, re, recursive]) => walk(path.join(repo, dir), re, recursive).map((f) => `${dir}/${f}`)).sort();

function run(cmd, args) {
  const r = spawnSync(cmd, args, { encoding: 'utf8' });
  return r.status === 0 ? r.stdout.trim() : null;
}

// the external bytes of a registered run, each with its path; null where a file cannot be read (verify then reports a mismatch against a recorded sha256)
export function externalPins({ engineDir, extra = [], browserExe = process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', dirs = {} }) {
  const { sdkDir = SDK_DIR, threeDir = THREE_DIR, aframeDir = AFRAME_DIR } = dirs;
  const pins = {};
  const add = (name, file) => { pins[name] = { file, sha256: file ? sha256Of(file) : null }; };
  add('node', fs.realpathSync(process.execPath));
  let real = null;
  try { real = fs.realpathSync(browserExe); } catch { /* recorded as unreadable */ }
  const wrapper = real && fs.readFileSync(real).subarray(0, 2).toString() === '#!';
  add('browser_binary', real && (wrapper ? path.join(path.dirname(real), 'chrome') : real));
  add('browser_wrapper', wrapper ? real : null);
  add('sdk_xr_js', path.join(sdkDir, 'xr.js'));
  add('three_module', path.join(threeDir, 'three.module.min.js'));
  add('three_core', path.join(threeDir, 'three.core.min.js'));
  add('aframe', path.join(aframeDir, 'aframe-v1.5.0.min.js'));
  for (const f of ['manifest.json', 'slam_refact_engine.js', 'slam_refact_engine.wasm']) add(`engine/${f}`, path.join(engineDir, f));
  extra.forEach((f, i) => add(`extra/${i}:${path.basename(f)}`, path.resolve(f)));
  pins.versions = { node: process.version, browser: (/(\d+\.\d+\.\d+\.\d+)/.exec(run(browserExe, ['--version']) ?? '') || [])[1] ?? null };
  return pins;
}

// the registered run is refused unless the manifest records a detected mutation and a passing null-run before the first run
export function checkManifest(manifestFile) {
  let m;
  try { m = JSON.parse(fs.readFileSync(manifestFile, 'utf8')); } catch (error) { return { ok: false, checks: [{ id: 'manifest', ok: false, detail: `unreadable: ${error.message}` }] }; }
  const checks = [
    { id: 'mutation.detected', ok: m.mutation?.detected === true, detail: `mutation.detected is ${JSON.stringify(m.mutation?.detected ?? null)} (must be true; an addendum cannot change it)` },
    { id: 'null_run.result', ok: m.null_run?.result === 'PASS', detail: `null_run.result is ${JSON.stringify(m.null_run?.result ?? null)} (must be "PASS")` },
    { id: 'first_real_run_at', ok: m.first_real_run_at === null || m.first_real_run_at === undefined, detail: `first_real_run_at is ${JSON.stringify(m.first_real_run_at ?? null)} (must still be empty)` },
  ];
  return { ok: checks.every((c) => c.ok), checks, manifestSha256: sha256File(manifestFile) };
}

// Writes the pin list ONCE. Throws (nothing written) when the manifest check fails, the list exists, or the replay driver is not among the core files.
export function freeze({ repo, out, manifestFile, engineDir, extra = [], snapshotDir = null, now = new Date(), roots = CORE_ROOTS, driver = REPLAY_DRIVER, ext = {} }) {
  const m = checkManifest(manifestFile);
  if (!m.ok) throw new Error(`refused: ${m.checks.filter((c) => !c.ok).map((c) => c.detail).join('; ')}`);
  if (fs.existsSync(out)) throw new Error(`refused: ${out} exists - the pin list is written once, a change is a new freeze under a new name`);
  if (snapshotDir && fs.existsSync(snapshotDir)) throw new Error(`refused: ${snapshotDir} exists - the snapshot is written once (nothing was written)`);
  const files = coreFiles(repo, roots);
  if (!files.includes(driver)) throw new Error(`refused: the replay driver ${driver} is not among the core files (it must exist before the freeze so that its sha256 is pinned)`);
  const record = {
    schema: 'us010-v4-pins-v1', at: now.toISOString(), manifest: { file: path.relative(repo, manifestFile), sha256: m.manifestSha256 },
    replayDriver: { file: driver, sha256: sha256File(path.join(repo, driver)) },
    files: Object.fromEntries(files.map((f) => [f, sha256File(path.join(repo, f))])), external: externalPins({ engineDir, extra, ...ext }),
    note: 'written once (exclusive create, read-only). A byte that differs later is reported by `verify`; a changed pin means a new freeze under a new name, not a rewrite.',
  };
  if (Object.values(record.external).some((e) => e && 'sha256' in e && e.sha256 === null)) throw new Error('refused: an external file is unreadable, nothing is pinned');
  fs.mkdirSync(path.dirname(out), { recursive: true });
  fs.writeFileSync(out, `${JSON.stringify(record, null, 1)}\n`, { flag: 'wx', mode: 0o444 });
  if (snapshotDir) {  // copies of the core files + SHA256SUMS, written once, read-only (v3: rewritten after the registered runs)
    fs.mkdirSync(snapshotDir, { recursive: false });
    for (const f of files) copyInto(snapshotDir, path.dirname(f), [path.join(repo, f)]);
    record.snapshot = { dir: path.relative(repo, snapshotDir), sums: writeSums(snapshotDir), readOnlyEntries: makeReadOnly(snapshotDir) };
  }
  return record;
}

// Compares the live tree with a pin list. mismatched: pinned files whose bytes differ; missing: pinned files that are gone; added: files in the core roots that were not
// pinned; external: entries whose path or sha256 differ. ok only when all four are empty.
export function verify({ repo, pinsFile, engineDir, roots = CORE_ROOTS, ext = {} }) {
  const rec = JSON.parse(fs.readFileSync(pinsFile, 'utf8'));
  const mismatched = [], missing = [];
  for (const [f, sha] of Object.entries(rec.files)) {
    const live = sha256Of(path.join(repo, f));
    if (live === null) missing.push(f); else if (live !== sha) mismatched.push({ file: f, pinned: sha, live });
  }
  const added = coreFiles(repo, roots).filter((f) => !(f in rec.files));
  const extra = Object.keys(rec.external).filter((k) => k.startsWith('extra/')).map((k) => rec.external[k].file);
  const now = externalPins({ engineDir, extra, ...ext });
  const external = [];
  for (const [k, e] of Object.entries(rec.external)) {
    if (k === 'versions') { for (const [v, val] of Object.entries(e)) if (now.versions[v] !== val) external.push({ entry: `versions.${v}`, pinned: val, live: now.versions[v] }); continue; }
    if (!(k in now) || now[k].sha256 !== e.sha256 || now[k].file !== e.file) external.push({ entry: k, pinned: e.sha256, live: now[k]?.sha256 ?? null, file: e.file, liveFile: now[k]?.file ?? null });
  }
  const driverNow = sha256Of(path.join(repo, rec.replayDriver.file));
  const driver = driverNow === rec.replayDriver.sha256 ? [] : [{ file: rec.replayDriver.file, pinned: rec.replayDriver.sha256, live: driverNow }];
  return { ok: !mismatched.length && !missing.length && !added.length && !external.length && !driver.length, mismatched, missing, added, external, replayDriver: driver, pinsSha256: sha256File(pinsFile) };
}

const isMain = process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url);
if (isMain) {
  const repo = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../..');
  const [cmd, ...rest] = process.argv.slice(2);
  const { values } = parseArgs({ args: rest, options: { manifest: { type: 'string' }, out: { type: 'string' }, 'engine-dir': { type: 'string' }, 'snapshot-dir': { type: 'string' },
    extra: { type: 'string', multiple: true }, pins: { type: 'string' } } });
  const under = (p) => { const a = path.resolve(repo, p); if (!a.startsWith(path.join(repo, 'build') + path.sep)) throw new Error(`${p}: must be under build/`); return a; };
  try {
    if (cmd === 'check-manifest') {
      const r = checkManifest(path.resolve(repo, values.manifest));
      console.log(JSON.stringify(r, null, 1));
      process.exitCode = r.ok ? 0 : 3;
    } else if (cmd === 'freeze') {
      if (!values['engine-dir']) throw new Error('--engine-dir is required');
      const r = freeze({ repo, out: under(values.out), manifestFile: path.resolve(repo, values.manifest), engineDir: path.resolve(repo, values['engine-dir']),
        extra: (values.extra ?? []).map((f) => path.resolve(repo, f)), snapshotDir: values['snapshot-dir'] ? under(values['snapshot-dir']) : null });
      console.log(JSON.stringify({ out: values.out, files: Object.keys(r.files).length, replayDriver: r.replayDriver, pinsSha256: sha256File(under(values.out)), snapshot: r.snapshot ?? null }, null, 1));
    } else if (cmd === 'verify') {
      if (!values['engine-dir']) throw new Error('--engine-dir is required');
      const r = verify({ repo, pinsFile: path.resolve(repo, values.pins), engineDir: path.resolve(repo, values['engine-dir']) });
      console.log(JSON.stringify(r, null, 1));
      process.exitCode = r.ok ? 0 : 3;
    } else {
      console.error('usage: us010-v4.mjs check-manifest|freeze|verify (see the header)');
      process.exitCode = 2;
    }
  } catch (error) {
    console.error(String(error.message));
    process.exitCode = String(error.message).startsWith('refused:') ? 3 : 1;
  }
}
