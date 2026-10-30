// First-real-run hold of the RM-4c/RM-4d/RM-4e chunk gate (protocol section 12.5: the integrity of the first real run is checked by code,
// not by procedure). Used by run-chunk-gl.mjs before the engine or the browser produce anything; unit-tested by chunk-hold.test.mjs.
//
// v5: the generic conditions are the shared device slam_refact/tools/prereg/sr_held.mjs (docs/design/sr-held-design.md), through
// tools/gate-common/held.mjs: manifest identity and shape, created_at < review.at < now, first_real_run_at null, the gate / review / designation
// file sha256, the environment value = the designation file's sha256, the designation's three pins, the pinned files (held.pins by base name, exactly
// the presented set), the coordinator seed policy (held.seed) and the one-run marker `<gate>.v<N>.first-run.<kind>.marker` (O_EXCL, kinds roi / it).
// NOTHING about the gate is a constant of this file any more (v4 pinned the id, the version, the paths and the sequence seed in the code): the gate
// id, version, preregistration directory and the NAME of the environment variable are arguments; the pinned engine directory, the registered
// sequence length and (when the seed is not held) the registered seed come from the registered manifest (registered_run), and the sha256 of the
// pinned files are values of the manifest, never of the code. The code lists WHICH files are pinned (CHUNK_FILES ... below).
//
// WHAT THIS IS NOT (RM-4e, review G-5): it is an accident-prevention device, not an authentication. Whoever can write the preregistration
// directory and set the environment variable can write a designation file, the review fields and a manifest that satisfy every check below
// (the v3 review demonstrated it), and the marker can be deleted by hand. It stops a run that is not what the gate registered by
// mistake: another seed, a changed file, a second run, a mutant copy. The sha256 of the executables is a pin of "which bytes were present",
// not proof of what executed (the shared libraries, ICU data, GPU driver / SwiftShader build and the kernel are not covered).
//
// Gate data = the runs whose result the gate judges: `--mode roi --kind gate` (SEED_GL camera) and the registered sequence of `--mode it`
// (`--kind gate`, or, when the manifest registers a seed, `--seed <that seed>` with a target). For those runs every condition below must hold,
// otherwise the run is refused ("HELD: <check or CODE>: ...") before any measurement. Null-runs and smoke data (other kinds / seeds / --no-targets)
// are not held.
//
//   1  args       seed is a plain integer (013 and 13 are the same seed), no --mutate, no --chunk-dir (the repo chunk is served), the pinned
//                 engine directory of the manifest (no other --engine-dir), the registered number of frames, every --expect-chunk-sha256 equal
//   2  device     sr_held CheckRelease with the presented pins: chunk, device, test, gate-common files, the three engine files, sr_held.mjs
//                 and the node executable (gate id / version / manifest / review / env / designation / gate file / pins / seed policy)
//   3  pins       held.pins is the flat form of manifest.pinned (checked first); manifest.pinned (the readable record) equals the live files group by group; the node / python /
//                 numpy / Pillow / playwright / Chromium versions AND the sha256 of the node, Chromium and python executables and of the
//                 playwright, numpy and Pillow package trees match manifest.pinned.environment (every key of both)
//   4  browser    (after launch, before the page runs) the Chromium version equals manifest.pinned.environment.browser
//   5  first-run  (RM-4e, review G-3) the marker of this kind of gate data (roi / it) does not exist; claimFirstRun then creates it exclusively
//                 BEFORE the browser starts and nothing removes it. A second gate run of the same kind is refused for good; a repeat after an
//                 infrastructure failure needs a new gate version (gate v4 section 5).
import fs from 'node:fs/promises';
import { closeSync, existsSync, openSync, readFileSync, readSync, readdirSync, readlinkSync, realpathSync } from 'node:fs';
import path from 'node:path';
import { createHash } from 'node:crypto';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { SR_HELD_FILE, claim, flatPins, gateArgs, gateMdPath, manifestPath, releaseCheck, request, sr } from '../tools/gate-common/held.mjs';

export const ENGINE_FILES = ['slam_refact_engine.js', 'slam_refact_engine.wasm', 'manifest.json'];
export const CHUNK_FILES = ['xr-slam.js', 'engine-loader.js', 'gr8-compute-pyramid.js', 'gr8-shaders.js', 'image-events.js', 'image-targets.js', 'sensors.js']
  .map((f) => `slam_refact/web-chunk/${f}`);
export const DEVICE_FILES = [
  'slam_refact/web-chunk/tests/run-chunk-gl.mjs', 'slam_refact/web-chunk/tests/chunk-hold.mjs', 'slam_refact/web-chunk/tests/gl-driver.js',
  'slam_refact/web-chunk/tools/roi_gl_compare.py', 'slam_refact/tools/image_target/roi_reference.py', 'slam_refact/tools/image_target/render_synthetic.py',
];
// The measuring files of the node tier, the Python tier and GL-R (RM-4e, review G-6): their results are judged by the gate, so the bytes are pinned.
export const TEST_FILES = [
  'slam_refact/web-chunk/tests/chunk-contract.test.mjs', 'slam_refact/web-chunk/tests/chunk-hold.test.mjs', 'slam_refact/web-chunk/tests/test_chunk_tools.py',
  'slam_refact/web-chunk/tests/run-chunk-smoke.mjs', 'slam_refact/web-chunk/tests/mutants/run-mutants.py', 'slam_refact/web-chunk/tests/mutants/run-hold-mutants.py',
  'slam_refact/web-chunk/tools/record-provenance.mjs',
  'slam_refact/tools/re/runtime/run-chunk-ab.mjs', 'slam_refact/tools/re/runtime/chunk_ab_report.py',
];
// The shared adapter the hold runs through (the device itself, sr_held.mjs, is pinned as `held_device`).
export const GATE_COMMON_FILES = ['held.mjs', 'runlog.mjs', 'freeze.mjs'].map((f) => `slam_refact/web-chunk/tools/gate-common/${f}`);
// The two kinds of gate data: the roi camera (`--mode roi --kind gate`) and the registered sequence (`--mode it`).
export const gateDataKind = (values) => (values.mode === 'roi' ? 'roi' : 'it');
export const KINDS = ['roi', 'it'];

export const sha256 = (bytes) => createHash('sha256').update(bytes).digest('hex');
export const sha256File = async (file) => sha256(await fs.readFile(file));

const sortKeys = (v) => (Array.isArray(v) ? v.map(sortKeys) : v && typeof v === 'object'
  ? Object.fromEntries(Object.keys(v).sort().map((k) => [k, sortKeys(v[k])])) : v);

// The manifest without the fields the coordinator fills in at review time and the runner at the first run, in the form the designation pins
// (sr_held print-pin; `held` and `pinned` are inside it).
export const manifestBeforeReviewSha256 = (manifest) => sr.canonManifestSha(sr.parseJson(Buffer.from(JSON.stringify(manifest))).json);

// A seed is a plain integer; anything else is refused instead of being taken for "some other seed" (013, 13 and +13 are one seed).
export function parseSeed(text) {
  if (typeof text !== 'string' || !/^\s*[+-]?\d+\s*$/.test(text)) throw new Error(`bad --seed ${JSON.stringify(text)}: a plain integer is required`);
  return Number(text);
}

// `registered` = manifest.registered_run ({seed: integer | null, frames}) when a manifest is at hand, else null.
export function isGateData({ mode, kind, seed, noTargets }, registered = null) {
  if (mode === 'roi') return kind === 'gate';
  if (kind === 'gate') return !noTargets;
  return registered?.seed != null && !noTargets && parseSeed(seed) === registered.seed;
}

const held = (id, detail) => new Error(`HELD: ${id}: ${detail}`);

function run(cmd, args) {
  const r = spawnSync(cmd, args, { encoding: 'utf8' });
  return r.status === 0 ? r.stdout.trim() : `unavailable (${cmd} exit ${r.status})`;
}

// sha256 of a (large) file without holding it in memory; 'unavailable' when it cannot be read.
export function fileSha256Sync(file) {
  let fd;
  try {
    fd = openSync(file, 'r');
    const hash = createHash('sha256'), buf = Buffer.allocUnsafe(1 << 20);
    for (let n = readSync(fd, buf, 0, buf.length, null); n > 0; n = readSync(fd, buf, 0, buf.length, null)) hash.update(buf.subarray(0, n));
    return hash.digest('hex');
  } catch (error) { return 'unavailable'; } finally { if (fd !== undefined) closeSync(fd); }
}

// sha256 over the sorted (relative path, file sha256) list of a package directory; compiled caches (__pycache__, *.pyc) are not part of it.
export function treeSha256(dir) {
  const lines = [];
  const walk = (d, rel) => {
    for (const e of readdirSync(d, { withFileTypes: true }).sort((a, b) => (a.name < b.name ? -1 : a.name > b.name ? 1 : 0))) {
      const r = rel ? `${rel}/${e.name}` : e.name;
      if (e.name === '__pycache__' || e.name.endsWith('.pyc')) continue;
      const full = path.join(d, e.name);
      if (e.isDirectory()) walk(full, r);
      else if (e.isSymbolicLink()) lines.push(`${r}\0symlink:${readlinkSync(full)}`);
      else if (e.isFile()) lines.push(`${r}\0${fileSha256Sync(full)}`);
    }
  };
  try { walk(dir, ''); } catch (error) { return 'unavailable'; }
  return sha256(lines.join('\n'));
}

const pythonLine = (code) => run('python3', ['-I', '-c', code]);

// Versions of the tools the device uses, and (RM-4e, review G-5 / protocol 12.5 (e)) the sha256 of the executables and package trees where
// they can be read: node, the Chromium binary (and the wrapper script when the chrome command is one), python, playwright(-core), numpy, Pillow.
// The browser version comes from the executable (and is compared with browser.version() at launch).
export function liveEnvironment(root) {
  const browserExe = process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome';
  const browser = (/(\d+\.\d+\.\d+\.\d+)/.exec(run(browserExe, ['--version'])) || [])[1] ?? 'unavailable';
  let playwright = 'unavailable', browserWrapper = 'unavailable', browserBinary = 'unavailable', playwrightTree = 'unavailable';
  const modules = path.join(root, 'scripts/dev/node_modules');
  try {
    playwright = JSON.parse(readFileSync(path.join(modules, 'playwright/package.json'), 'utf8')).version;
    playwrightTree = sha256(`playwright=${treeSha256(path.join(modules, 'playwright'))}\nplaywright-core=${treeSha256(path.join(modules, 'playwright-core'))}`);
  } catch (error) { /* stays unavailable */ }
  try {
    const real = realpathSync(browserExe);
    // the chrome command is often a shell wrapper that starts `chrome` in the same directory
    const wrapper = readFileSync(real).subarray(0, 2).toString() === '#!';
    browserBinary = fileSha256Sync(wrapper ? path.join(path.dirname(real), 'chrome') : real);
    browserWrapper = wrapper ? fileSha256Sync(real) : 'not a wrapper';
  } catch (error) { /* stays unavailable */ }
  const treeOf = (module) => {
    const dir = pythonLine(`import ${module},os;print(os.path.dirname(${module}.__file__))`);
    return dir.startsWith('unavailable') ? dir : treeSha256(dir);
  };
  const pythonExe = pythonLine('import sys;print(sys.executable)');
  return {
    node: process.version, node_sha256: fileSha256Sync(realpathSync(process.execPath)),
    browser, browser_binary_sha256: browserBinary, browser_wrapper_sha256: browserWrapper,
    playwright, playwright_tree_sha256: playwrightTree,
    python: pythonLine('import sys;print(sys.version.split()[0])'), python_sha256: pythonExe.startsWith('unavailable') ? pythonExe : fileSha256Sync(realpathSync(pythonExe)),
    numpy: pythonLine('import numpy;print(numpy.__version__)'), numpy_tree_sha256: treeOf('numpy'),
    pillow: pythonLine('import PIL;print(PIL.__version__)'), pillow_tree_sha256: treeOf('PIL'),
  };
}

async function shaMap(root, rels) {
  const out = {};
  for (const rel of rels) out[rel] = await sha256File(path.join(root, rel));
  return out;
}

// The pins as they are now (what the manifest's `pinned` must equal): `node chunk-hold.mjs --print-pins --engine-dir <dir>`. The engine directory is an
// argument (v4 pinned it in the code); the device pin is the file this process loaded.
export async function livePins(root, environment = liveEnvironment(root), engineDirRel = null) {
  const engine = { dir: engineDirRel };
  if (engineDirRel) for (const f of ENGINE_FILES) engine[f] = await sha256File(path.join(root, engineDirRel, f));
  return { engine, chunk: await shaMap(root, CHUNK_FILES), device: await shaMap(root, DEVICE_FILES), tests: await shaMap(root, TEST_FILES),
    gate_common: await shaMap(root, GATE_COMMON_FILES), held_device: { 'sr_held.mjs': await sha256File(SR_HELD_FILE) }, environment };
}

// Every pinned file as [absolute path, sha256] and held.pins, the flat base-name form of the device. The node executable is pinned by its environment entry.
export function pinnedFiles(root, pinned) {
  const files = [];
  for (const group of ['engine', 'chunk', 'device', 'tests', 'gate_common']) {
    for (const [rel, sha] of Object.entries(pinned[group] ?? {})) {
      if (group === 'engine' && rel === 'dir') continue;
      files.push([path.join(root, group === 'engine' ? path.join(pinned.engine.dir, rel) : rel), sha]);
    }
  }
  for (const [name, sha] of Object.entries(pinned.held_device ?? {})) files.push([name === 'sr_held.mjs' ? SR_HELD_FILE : path.join(path.dirname(SR_HELD_FILE), name), sha]);
  if (pinned.environment?.node_sha256) files.push([process.execPath, pinned.environment.node_sha256]);
  return files;
}
export const heldPinsOf = (root, pinned) => flatPins(pinnedFiles(root, pinned));

const realpathOrNull = async (p) => { try { return await fs.realpath(p); } catch (error) { return null; } };

// values: the parsed command line {mode, kind, seed, noTargets, mutate, frames, chunkDir, engineDir, expectChunk[]} (raw strings as typed, undefined when absent).
// gate: {gateId, version, preregDir, designationEnv} (gateArgs()) - null for runs that are not held and need no manifest.
// Returns {gateData, checks, ...} - checks is the list written at the head of meta.json - or throws a HELD error.
export async function checkHold({ root, gate, values, env = process.env, environment = null, now = new Date() }) {
  const manifestFile = gate ? manifestPath(gate) : null;
  let manifest = null;
  if (manifestFile && existsSync(manifestFile)) {
    try { manifest = JSON.parse(await fs.readFile(manifestFile, 'utf8')); } catch (error) { if (values.kind === 'gate') throw held('manifest', `${path.relative(root, manifestFile)} is not JSON: ${error.message}`); }
  }
  const registered = manifest?.registered_run ?? null;
  const gateData = isGateData(values, registered);
  const checks = [];
  const ok = (id, detail) => checks.push({ id, ok: true, detail });
  if (!gateData) return { gateData, checks: [{ id: 'not-gate-data', ok: true, detail: 'null-run or smoke data: not held' }] };
  if (!gate) throw held('args', 'gate data needs --prereg-dir --gate-id --gate-version --designation-env');
  if (!manifest) throw held('manifest', `cannot read ${path.relative(root, manifestFile)}`);

  // 1 args
  if (values.mutate) throw held('args', 'mutations run on smoke data only; a gate-data run with --mutate is refused');
  if (values.chunkDir !== undefined) throw held('args', '--chunk-dir is refused for gate data: the repo chunk is served and its files are sha256-checked');
  if (values.mode === 'it') {
    if (values.noTargets) throw held('args', 'internal: --no-targets is not gate data');
    if (!Number.isInteger(registered?.frames)) throw held('args', 'manifest.registered_run.frames (the registered sequence length) is missing');
    if (Number(values.frames) !== registered.frames || !/^\s*\d+\s*$/.test(String(values.frames))) throw held('args', `the registered sequence has ${registered.frames} frames (got ${values.frames})`);
    if (registered.seed != null && parseSeed(values.seed) !== registered.seed) throw held('args', `the registered seed is ${registered.seed} (got ${values.seed})`);
  }
  const pinnedDir = manifest.pinned?.engine?.dir;
  const wantEngine = pinnedDir ? await realpathOrNull(path.join(root, pinnedDir)) : null;
  const gotEngine = await realpathOrNull(path.resolve(root, values.engineDir ?? ''));
  if (!wantEngine || !gotEngine || wantEngine !== gotEngine) throw held('args', `--engine-dir must be the pinned engine (${pinnedDir})`);
  // the caller (the coordinator's run script) states which chunk bytes it expects: name=sha256 for any chunk file; a difference is refused before the device runs
  for (const spec of values.expectChunk ?? []) {
    const m = /^([A-Za-z0-9._-]+)=([0-9a-f]{64})$/.exec(spec);
    if (!m || !CHUNK_FILES.some((f) => path.basename(f) === m[1])) throw held('args', `--expect-chunk-sha256 ${spec}: name=<64 hex> of a chunk file is required`);
    const live = await sha256File(path.join(root, 'slam_refact/web-chunk', m[1])).catch(() => null);
    if (live !== m[2]) throw held('args', `chunk file ${m[1]} has sha256 ${live}, the caller expected ${m[2]}`);
  }
  ok('args', 'plain-integer seed, no mutation, no --chunk-dir, pinned engine directory, registered length, expected chunk bytes');

  // 2 the device: manifest, review, designation, env, pins, seed policy (marker is claimed later)
  const pinned = manifest.pinned;
  if (!pinned || !pinned.engine || !pinned.chunk || !pinned.device || !pinned.environment || !pinned.tests || !pinned.gate_common || !pinned.held_device) throw held('pins', 'manifest.pinned is incomplete');
  const kind = gateDataKind(values);
  const files = pinnedFiles(root, pinned);
  // the readable record (pinned) and the device's registration (held.pins) are one fact in two forms
  let flat;
  try { flat = heldPinsOf(root, pinned); } catch (error) { throw held('pins', error.message); }
  if (JSON.stringify(sortKeys(flat)) !== JSON.stringify(sortKeys(manifest.held?.pins ?? null))) throw held('pins', 'manifest.held.pins is not the flat form of manifest.pinned');
  const wantSeed = manifest.held?.seed === true;
  if (wantSeed && registered.seed != null) throw held('seed', 'held.seed is true but registered_run.seed is a value: the seed would not be held');
  if (!wantSeed && values.mode === 'it' && registered.seed == null) throw held('seed', 'neither a held coordinator seed nor a registered seed');
  const rq = request(gate, { kind, env: env[gate.designationEnv], pins: files.map(([f]) => f), wantSeed, now });
  const outcome = releaseCheck(rq);
  ok('device', `sr_held: ${outcome.log.length} lines (designation ${outcome.designationSha256})`);

  // 3 pins: the readable record equals the live files group by group, held.pins is its flat form, the environment entries are all equal
  const live = await livePins(root, environment ?? liveEnvironment(root), pinned.engine.dir);
  for (const group of ['engine', 'chunk', 'device', 'tests', 'gate_common', 'held_device']) {
    const want = pinned[group], got = live[group];
    const keys = new Set([...Object.keys(want), ...Object.keys(got)]);
    for (const k of keys) if (want[k] !== got[k]) throw held('pins', `${group} ${k}: ${got[k]} != pinned ${want[k]}`);
  }
  for (const key of new Set([...Object.keys(pinned.environment), ...Object.keys(live.environment)])) {
    if (live.environment[key] !== pinned.environment[key]) throw held('pins', `environment ${key}: ${live.environment[key]} != pinned ${pinned.environment[key]}`);
  }
  ok('pins', `${Object.keys(live.chunk).length} chunk, ${Object.keys(live.device).length} device, ${Object.keys(live.tests).length} test, ${Object.keys(live.gate_common).length} gate-common, 3 engine files, `
    + `${Object.keys(live.environment).length} environment entries (versions and executable / package sha256)`);

  // 5 first-run: one marker per kind of gate data, created exclusively by claimFirstRun (the device checks it only when it claims)
  const markerRel = path.relative(root, path.join(gate.preregDir, sr.markerName(rq)));
  if (existsSync(path.join(root, markerRel))) throw held('first-run', `${markerRel} exists: this kind of gate data was already run (or claimed); a repeat needs a new gate version`);
  ok('first-run', `no ${markerRel}`);
  return { gateData, checks, live, pinnedBrowser: pinned.environment.browser, designationFile: manifest.review?.designation_file, markerRel, rq, gate, manifest,
    seedHeld: wantSeed, registered, log: outcome.log };
}

// The first real run of this kind of gate data is claimed: the marker is created exclusively (flag wx), so two runners racing for it cannot both
// start, and nothing in the code removes it. Called by run-chunk-gl.mjs after checkHold and before the output directory / the browser exist.
// Returns null for data that is not held, else {marker, seed} (seed: the coordinator's seed, only when the manifest holds it, else null).
export async function claimFirstRun({ root, hold }) {
  if (!hold.gateData) return null;
  const outcome = claim(hold.rq);
  hold.log = outcome.log;
  return { marker: path.relative(root, outcome.marker), seed: hold.seedHeld ? outcome.seed : null, log: outcome.log };
}

// After launch: the browser that really runs equals the pinned one.
export function checkBrowser(version, pinnedBrowser) {
  if (version !== pinnedBrowser) throw held('browser', `Chromium ${version} != pinned ${pinnedBrowser}`);
  return { id: 'browser', ok: true, detail: version };
}

export { gateArgs, gateMdPath };

// `node chunk-hold.mjs --print-pins --engine-dir <engine dir relative to the repository root> [--manifest <file>]`: the live pins in the shape of manifest.pinned and
// held.pins, for the registration step; with --manifest also the designation values of that manifest (sr_held print-pin form).
if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url) && process.argv.includes('--print-pins')) {
  const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../..');
  const arg = (name) => { const i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : undefined; };
  const engineDir = arg('--engine-dir');
  if (!engineDir) throw new Error('--engine-dir <dir> is required (nothing is defaulted)');
  const pinned = await livePins(root, liveEnvironment(root), engineDir);
  console.log(JSON.stringify({ pinned, held_pins: heldPinsOf(root, pinned) }, null, 1));
  if (arg('--manifest')) {
    const manifest = JSON.parse(await fs.readFile(path.resolve(root, arg('--manifest')), 'utf8'));
    console.log(`manifest before review sha256 (designation pinned.manifest_before_review_sha256): ${manifestBeforeReviewSha256(manifest)}`);
  }
}
