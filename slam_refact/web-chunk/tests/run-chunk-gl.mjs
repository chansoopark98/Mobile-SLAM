// Chromium (SwiftShader) measurements of the RM-4c gate rm4c-chunk-gate.v1.md G-GL. One headless Chromium, one ephemeral
// 127.0.0.1 server, no external request (blocked and counted), outputs only under build/.
//
//   node slam_refact/web-chunk/tests/run-chunk-gl.mjs --mode roi --kind smoke|const|ramp|gate [--mutate MF|MZ|MH] --out <new dir> --engine-dir <dir>
//        [--chunk-dir <copy of slam_refact/web-chunk>]
//   node slam_refact/web-chunk/tests/run-chunk-gl.mjs --mode it --seed 14 [--no-targets] [--frames 132] --out <new dir> --engine-dir <dir>
//        [--chunk-dir <copy>]
//   GATE ARGUMENTS (gate data; optional for smoke data and null-runs): --prereg-dir <dir> --gate-id <id> --gate-version <N> --designation-env <NAME>
//   --precheck (every check of the hold, no marker, no output, no browser: the dry run), --run-log <new file under build/, outside --out> (required for gate data), --expect-chunk-sha256 <chunk file>=<sha256> (repeatable; the freeze record's
//   chunk bytes), --mutation-evidence <summary json> (repeatable; recorded with its sha256 in meta.json p3)
//
// mode roi: Gr8ComputePyramid draws the four oracle warps into the ROI slots; roi_gl_compare.py compares the slot pixels with
//           the independent numpy oracle (GL-A1..A4).
// mode it:  the real XrControllerFactory + pyramid + engine process a synthetic 480x640 sequence (render_synthetic.py, seed N)
//           with one flat image target loaded through the default image loader, next to the engine CPU path (GL-E1..E4).
//
// FIRST REAL RUN HOLD (gate v5 section 5, protocol section 12.5; chunk-hold.mjs over the shared sr_held device, unit-tested by chunk-hold.test.mjs): the gate
// data (--mode roi --kind gate, the registered sequence of --mode it) is refused ("HELD: <check or CODE>: ...", before the browser starts) unless the registered
// manifest records the review, the coordinator's designation file is pinned by the environment variable NAMED by --designation-env + manifest, and the gate file,
// the measurement device, the chunk files, the pinned engine and the tool versions are the registered ones. The gate id / version / directory, the engine
// directory, the registered seed and sequence length are arguments or values of the registered manifest, not constants of this file (v4 had them in the
// code). --mutate and --chunk-dir are refused for gate data (copies and mutants run on smoke data only); the seed is a plain integer (013 = 13), or the
// coordinator's held seed (returned by the claim only). Null-runs (const, ramp, --no-targets) and smoke data (--kind smoke, other seeds) are not held.
// A gate-data run claims its first-run marker (one file per kind, exclusive create) before the output directory and the browser exist: a second run of the
// same kind is refused (G-3). After a gate-data run the output tree gets SHA256SUMS and is made read-only, and the run log (outside the tree) records the
// command, the names of the steering environment variables, every device check line and the marker.
import fs from 'node:fs/promises';
import http from 'node:http';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { parseArgs } from 'node:util';
import { chromium } from '../../../scripts/dev/node_modules/playwright/index.mjs';
import { createHash } from 'node:crypto';
import { checkBrowser, checkHold, claimFirstRun, gateArgs, livePins, parseSeed } from './chunk-hold.mjs';
import { RunLog, envRecord } from '../tools/gate-common/runlog.mjs';
import { copyInto, makeReadOnly, readOnlyFiles, sha256File, writeSums } from '../tools/gate-common/freeze.mjs';
import { gateMdPath, manifestPath } from '../tools/gate-common/held.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(here, '../../..');
const { values } = parseArgs({ options: {
  mode: { type: 'string' }, kind: { type: 'string', default: 'smoke' }, mutate: { type: 'string' }, seed: { type: 'string', default: '14' },
  'no-targets': { type: 'boolean', default: false }, frames: { type: 'string', default: '132' }, out: { type: 'string' },
  'chunk-dir': { type: 'string' }, 'engine-dir': { type: 'string' },
  'prereg-dir': { type: 'string' }, 'gate-id': { type: 'string' }, 'gate-version': { type: 'string' }, 'designation-env': { type: 'string' }, 'run-log': { type: 'string' },
  'expect-chunk-sha256': { type: 'string', multiple: true }, 'mutation-evidence': { type: 'string', multiple: true }, precheck: { type: 'boolean', default: false },
} });
if (!values['engine-dir']) throw new Error('--engine-dir <dir> is required (nothing is defaulted: the gate data run must name the registered engine directory)');
if (!['roi', 'it'].includes(values.mode)) throw new Error('--mode roi|it');
if (!['gate', 'smoke', 'const', 'ramp'].includes(values.kind)) throw new Error('--kind gate|smoke|const|ramp');
const out = path.resolve(root, values.out || '');
if (!values.out || !out.startsWith(path.join(root, 'build') + path.sep)) throw new Error('--out <new dir under build/> required');
// the gate arguments are all-or-nothing: a gate-data run needs them, smoke data and null-runs run without
const gateGiven = ['prereg-dir', 'gate-id', 'gate-version', 'designation-env'].some((k) => values[k]);
const gate = gateGiven || values.kind === 'gate' ? gateArgs(values, root) : null;
const log = values['run-log'] && !values.precheck ? new RunLog(path.resolve(root, values['run-log']), { out }) : null;
log?.write('start', { argv: process.argv.slice(2), node: process.version, mode: values.mode, kind: values.kind, gate: gate && { id: gate.gateId, version: gate.version, designationEnvName: gate.designationEnv },
  env: envRecord(process.env, gate ? [gate.designationEnv] : []) });
// The hold first: nothing is created, no browser or engine runs, when it refuses.
let hold, claimed;
try {
  hold = await checkHold({ root, gate, values: { mode: values.mode, kind: values.kind, seed: values.seed, noTargets: values['no-targets'], mutate: values.mutate,
    frames: values.frames, chunkDir: values['chunk-dir'], engineDir: values['engine-dir'], expectChunk: values['expect-chunk-sha256'] } });
  if (values.precheck) {  // --precheck: every check of the hold, no marker, no output, no browser (the dry run of a registered command)
    console.log(JSON.stringify({ precheck: true, gateData: hold.gateData, checks: hold.checks, deviceLog: hold.log ?? null }, null, 1));
    process.exit(0);
  }
  if (hold.gateData && !log) throw new Error('--run-log <new file> is required for gate data (outside --out)');
  if (await fs.stat(out).then(() => true, () => false)) throw new Error(`--out ${values.out} exists`);
  // A gate-data run claims its marker before anything exists (exclusive create); null-runs and smoke data claim nothing.
  log?.write('hold-checks', { gateData: hold.gateData, checks: hold.checks, deviceLog: hold.log ?? null });
  claimed = await claimFirstRun({ root, hold });
  if (claimed) log?.write('first-run-claimed', { marker: claimed.marker });
} catch (error) {
  if (log && !log.closed) { log.write(String(error.message).startsWith('HELD:') ? 'held' : 'error', { message: error.message }); log.close(String(error.message).startsWith('HELD:') ? 'HELD' : 'ERROR'); }
  throw error;
}
// the seed of the registered sequence: the coordinator's held seed (returned by the claim only), else the plain integer given
const seed = claimed?.seed ?? String(parseSeed(values.seed));
await fs.mkdir(out, { recursive: false });
const chunkDir = path.resolve(root, values['chunk-dir'] || 'slam_refact/web-chunk');
const engineDir = path.resolve(root, values['engine-dir']);
const repoChunk = chunkDir === path.join(root, 'slam_refact/web-chunk');

const py = (args) => {
  const r = spawnSync('python3', ['-I', ...args], { cwd: root, encoding: 'utf8' });
  if (r.status !== 0) throw new Error(`python3 ${args.join(' ')} failed: ${r.stderr || r.stdout}`);
  return r.stdout;
};

const shaOf = async (file) => createHash('sha256').update(await fs.readFile(file)).digest('hex');
const inputs = {};  // protocol 12.5 (f): the sha256 of the inputs of this run
let job, served = {};
if (values.mode === 'roi') {
  py(['slam_refact/web-chunk/tools/roi_gl_compare.py', 'prepare', '--out', out, '--kind', values.kind, ...(values.mutate ? ['--mutate', values.mutate] : [])]);
  job = JSON.parse(await fs.readFile(path.join(out, 'job.json'), 'utf8'));
  inputs.camera_pgm = await shaOf(path.join(out, 'camera.pgm'));
  inputs.job_json = await shaOf(path.join(out, 'job.json'));
  inputs.expected_json = await shaOf(path.join(out, 'expected.json'));
} else {
  const data = path.join(out, 'data');
  py(['slam_refact/tools/image_target/render_synthetic.py', '--out', data, '--seed', seed, '--trajectory', 'registered']);
  // The default image loader decodes a browser image: the target luminance as a (lossless, gray) PNG made from the PGM.
  py(['-c', `from PIL import Image; Image.open(r'${path.join(data, 'synth-flat_luminance.pgm')}').save(r'${path.join(data, 'synth-flat_luminance.png')}')`]);
  const target = JSON.parse(await fs.readFile(path.join(data, 'synth-flat.json'), 'utf8'));
  target.imagePath = '/data/synth-flat_luminance.png';
  const frames = Number(values.frames);
  job = { width: 480, height: 640, frameCount: frames, roiCheckFrames: 40, ...(values['no-targets'] ? {} : { imageTargetData: [target] }) };
  served.dataDir = data;
  const frameShas = [];
  for (let i = 0; i < frames; i++) frameShas.push(await shaOf(path.join(data, 'frames', `${String(i).padStart(4, '0')}.pgm`)));
  inputs.frames = { count: frames, sha256_of_sha256_list: createHash('sha256').update(frameShas.join('\n')).digest('hex'), first: frameShas[0], last: frameShas.at(-1) };
  inputs.target_luminance_pgm = await shaOf(path.join(data, 'synth-flat_luminance.pgm'));
  inputs.target_luminance_png = await shaOf(path.join(data, 'synth-flat_luminance.png'));
  inputs.target_json = await shaOf(path.join(data, 'synth-flat.json'));
}

const mime = (f) => (f.endsWith('.js') || f.endsWith('.mjs') ? 'text/javascript' : f.endsWith('.json') ? 'application/json' : f.endsWith('.wasm') ? 'application/wasm'
  : f.endsWith('.png') ? 'image/png' : 'application/octet-stream');
const CHUNK_FILES = new Set(['xr-slam.js', 'engine-loader.js', 'gr8-compute-pyramid.js', 'gr8-shaders.js', 'image-events.js', 'image-targets.js', 'sensors.js']);
const html = '<!doctype html><meta charset="utf-8"><title>rm4c gl</title><script type="module" src="/driver.js"></script><body></body>';
const blocked = [], requests = [], consoleLines = [], pageErrors = [];
const server = http.createServer(async (req, res) => {
  try {
    const p = new URL(req.url, 'http://x').pathname;
    let file;
    if (p === '/' ) { res.writeHead(200, { 'Content-Type': 'text/html' }); res.end(html); return; }
    if (p === '/driver.js') file = path.join(here, 'gl-driver.js');
    else if (p.startsWith('/chunk/slam-refact-engine/')) {
      const name = p.slice('/chunk/slam-refact-engine/'.length);
      if (!['manifest.json', 'slam_refact_engine.js', 'slam_refact_engine.wasm'].includes(name)) throw new Error('unserved');
      file = path.join(engineDir, name);
    } else if (p.startsWith('/chunk/')) {
      const name = p.slice('/chunk/'.length);
      if (!CHUNK_FILES.has(name)) throw new Error('unserved');
      file = path.join(chunkDir, name);
    } else if (p.startsWith('/data/') && served.dataDir) {
      const f = path.resolve(served.dataDir, p.slice('/data/'.length));
      if (!f.startsWith(served.dataDir + path.sep)) throw new Error('unserved');
      file = f;
    } else throw new Error('unserved');
    res.writeHead(200, { 'Content-Type': mime(file), 'Cache-Control': 'no-store' });
    res.end(await fs.readFile(file));
  } catch (e) { res.writeHead(404); res.end(String(e)); }
});
await new Promise((r) => server.listen(0, '127.0.0.1', r));
const origin = `http://127.0.0.1:${server.address().port}`;
const browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true, args: ['--enable-unsafe-swiftshader'] });
const report = { hold: { gateData: hold.gateData, checks: hold.checks, firstRunMarker: claimed?.marker ?? null, deviceLog: hold.log ?? null, seedHeld: hold.seedHeld ?? false,
    gate: gate && { id: gate.gateId, version: gate.version, designationEnvName: gate.designationEnv } }, inputs,
  pinsAtRun: hold.live ?? await livePins(root, undefined, path.relative(root, engineDir)), mode: values.mode, argv: process.argv.slice(2), startedAt: new Date().toISOString(), browser: browser.version(),
  chunkDir: path.relative(root, chunkDir), engineDir: path.relative(root, engineDir), repoChunk, mutate: values.mutate ?? null, kind: values.kind, seed, noTargets: values['no-targets'], origin };
let status = 'FAILED';
try {
  if (hold.gateData) report.hold.checks.push(checkBrowser(browser.version(), hold.pinnedBrowser));
  const page = await (await browser.newContext({ viewport: { width: 480, height: 640 } })).newPage();
  await page.route('**/*', (route) => {
    const url = route.request().url();
    if (!url.startsWith(origin + '/') && !url.startsWith('data:') && !url.startsWith('blob:')) { blocked.push(url); return route.abort('blockedbyclient'); }
    return route.continue();
  });
  page.on('response', (r) => requests.push({ url: r.url().replace(origin, ''), status: r.status() }));
  page.on('console', (m) => { if (consoleLines.length < 300) consoleLines.push({ type: m.type(), text: m.text().slice(0, 300) }); });
  page.on('pageerror', (e) => pageErrors.push({ message: e.message, stack: e.stack }));
  await page.goto(origin + '/');
  await page.waitForFunction(() => window.__driverReady === true, null, { timeout: 60000 });
  const fn = values.mode === 'roi' ? 'runRoi' : 'runIt';
  const result = await page.evaluate(([f, j]) => window[f](j), [fn, job]);
  if (values.mode === 'roi') {
    await fs.writeFile(path.join(out, 'gl_result.json'), JSON.stringify(result));
    py(['slam_refact/web-chunk/tools/roi_gl_compare.py', 'compare', '--dir', out]);
    report.metrics = JSON.parse(await fs.readFile(path.join(out, 'metrics.json'), 'utf8')).worst;
    report.verdict = JSON.parse(await fs.readFile(path.join(out, 'metrics.json'), 'utf8')).verdict;
  } else {
    await fs.writeFile(path.join(out, 'it_result.json'), JSON.stringify(result));
    report.summary = { renderer: result.renderer, pngRoundTrip: result.pngRoundTrip ?? null,
      chunkEvents: result.chunk.events.filter((e) => /^image(found|updated|lost)$/.test(e.name)).map((e) => [e.name, e.source, e.k]),
      cpuEvents: result.cpu.events.map((e) => [e.name, e.source, e.k]), errors: result.chunk.errors, warnings: result.chunk.warnings,
      readback: result.chunk.readback, stats: result.chunk.stats, imageTargets: result.chunk.imageTargets,
      imageLoadingEvents: result.chunk.events.filter((e) => /^image(loading|scanning)$/.test(e.name)).length };
  }
  status = 'COMPLETED';
} catch (error) {
  report.error = { message: error.message, stack: error.stack };
} finally {
  await browser.close();
  await new Promise((r) => server.close(r));
  report.status = status;
  report.blockedExternal = blocked;
  report.pageErrors = pageErrors;
  report.console = consoleLines;
  report.requests = requests.length;
  report.finishedAt = new Date().toISOString();
  // P3 (result document: full-parity plan r3 section 7): what the result document needs and only the harness can state. The mutation matrix and the
  // undetected classes come from the mutation runs named by --mutation-evidence (their sha256 is recorded here); nothing here is a verdict.
  report.p3 = {
    schema: 'rm4c-p3-inputs-v1', gate: report.hold.gate, mode: values.mode, kind: values.kind, gateData: hold.gateData,
    n_independent_inputs: 1,
    input_unit: values.mode === 'roi' ? 'one fixed GL camera image (SEED_GL), 4 oracle warps' : `one synthetic sequence (seed ${seed}, ${values.frames} frames, ${values['no-targets'] ? 'no' : 'one'} flat image target)`,
    runs_of_this_kind_in_this_registration: hold.gateData ? 1 : null,
    counts: { pageErrors: pageErrors.length, blockedExternal: blocked.length, requests: requests.length, consoleLines: consoleLines.length },
    chunk_bytes_expected_by_caller: values['expect-chunk-sha256'] ?? [],
    mutation_evidence: (values['mutation-evidence'] ?? []).map((f) => { try { return { file: f, sha256: sha256File(path.resolve(root, f)) }; } catch (error) { return { file: f, sha256: null, error: error.message }; } }),
    not_covered: ['real device, real GPU, real camera (this is headless Chromium + SwiftShader)', 'one input per kind: no estimate of input-to-input variation'],
  };
  await fs.writeFile(path.join(out, 'meta.json'), JSON.stringify(report, null, 1) + '\n');
  if (hold.gateData) {  // evidence hygiene: copies of the registered files, SHA256SUMS over the tree, then read-only (the run log is outside the tree)
    const review = hold.manifest.review ?? {};
    const registered = [gateMdPath(gate), manifestPath(gate), path.join(gate.preregDir, review.designation_file), path.join(gate.preregDir, review.file), path.join(root, claimed.marker)];
    const copies = copyInto(out, 'registered', registered);
    const originals = readOnlyFiles([gateMdPath(gate), path.join(gate.preregDir, review.designation_file), path.join(gate.preregDir, review.file),
      ...(hold.seedHeld ? [path.join(gate.preregDir, `${gate.gateId}.v${gate.version}.coordinator-seed.txt`)] : [])]);
    report.frozen = { copies, originalsReadOnly: originals, sums: writeSums(out), readOnlyEntries: makeReadOnly(out) };
    log?.write('finished', { status, out: path.relative(root, out), frozen: report.frozen, verdict: report.verdict ?? null });
  }
  if (log && !log.closed) log.close(status);
}
console.log(JSON.stringify({ status: report.status, out: path.relative(root, out), verdict: report.verdict, metrics: report.metrics, error: report.error?.message,
  blocked: blocked.length, pageErrors: pageErrors.length, summary: report.summary && { events: report.summary.chunkEvents, cpu: report.summary.cpuEvents, png: report.summary.pngRoundTrip } }, null, 1));
process.exitCode = report.status === 'COMPLETED' ? 0 : 1;
