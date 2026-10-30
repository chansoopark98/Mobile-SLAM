// node --test slam_refact/web-chunk/tests/chunk-hold.test.mjs
// Unit test of the first-real-run hold (chunk-hold.mjs over the shared device tools/prereg/sr_held.mjs, protocol section 12.5): a synthetic tree (small
// fake files, a registered manifest, a designation file) is built under build/, a pass with everything in order is shown, then each bypass attempt is made
// alone and must be refused for the expected reason ("HELD: <check or CODE>"). No browser, no engine. The gate (id, version, preregistration directory), the
// NAME of the environment variable and the pinned engine directory are arguments of every call: nothing here depends on a constant of the tool (v5).
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test, { after } from 'node:test';
import * as sr from '../../tools/prereg/sr_held.mjs';
import {
  CHUNK_FILES, DEVICE_FILES, ENGINE_FILES, GATE_COMMON_FILES, TEST_FILES,
  checkBrowser, checkHold, claimFirstRun, fileSha256Sync, gateArgs, gateDataKind, heldPinsOf, isGateData, liveEnvironment, livePins, manifestBeforeReviewSha256, parseSeed,
  pinnedFiles, sha256, sha256File, treeSha256,
} from './chunk-hold.mjs';

const repo = path.resolve(import.meta.dirname, '../../..');
const scratch = path.join(repo, 'build/slam-refact-full-chunk-test-tmp');
fs.mkdirSync(scratch, { recursive: true });
const made = [];
after(() => { for (const dir of made) fs.rmSync(dir, { recursive: true, force: true }); });
const GATE_ID = 'rm4c-test-gate', VERSION = 5, ENV_NAME = 'SLAM_REFACT_RM4C_TEST_V5_DESIGNATION_SHA256', ENGINE_DIR = 'build/test-lane/engine-pinned';
// versions and (RM-4e) the sha256 of the executables / package trees: every key is compared. The node executable is presented to the device, so its sha256 is the real one.
const ENV = { node: 'v0.0.0', node_sha256: fileSha256Sync(fs.realpathSync(process.execPath)), browser: '1.2.3.4', browser_binary_sha256: '2'.repeat(64), browser_wrapper_sha256: '3'.repeat(64),
  playwright: '9.9.9', playwright_tree_sha256: '4'.repeat(64), python: '3.0.0', python_sha256: '5'.repeat(64), numpy: '1.0.0', numpy_tree_sha256: '6'.repeat(64),
  pillow: '2.0.0', pillow_tree_sha256: '7'.repeat(64) };
const NOW = new Date('2026-10-20T05:00:00+09:00');

// A consistent synthetic tree + the values of a gate-data run that passes. heldSeed: the manifest holds the coordinator seed (registered_run.seed null).
async function fixture({ heldSeed = false } = {}) {
  const root = fs.mkdtempSync(path.join(scratch, 'hold-'));
  made.push(root);
  const put = (rel, text) => { fs.mkdirSync(path.dirname(path.join(root, rel)), { recursive: true }); fs.writeFileSync(path.join(root, rel), text); };
  for (const rel of [...DEVICE_FILES, ...CHUNK_FILES, ...TEST_FILES, ...GATE_COMMON_FILES]) put(rel, `// fake ${rel}\n`);
  for (const f of ENGINE_FILES) put(path.join(ENGINE_DIR, f), `fake engine ${f}`);
  const gate = { gateId: GATE_ID, version: VERSION, preregDir: path.join(root, 'prereg'), designationEnv: ENV_NAME };
  const gateRel = `prereg/${GATE_ID}.v${VERSION}.md`, reviewRel = 'prereg/review.md', designationRel = 'prereg/designation.json';
  const manifestRel = `prereg/${GATE_ID}.v${VERSION}.manifest.json`, seedRel = `prereg/${GATE_ID}.v${VERSION}.coordinator-seed.txt`;
  put(gateRel, '# gate v5 (fake)\n');
  put(reviewRel, '# review (fake)\n');
  const pinned = await livePins(root, { ...ENV }, ENGINE_DIR);  // a copy: the tests below change the manifest's pins
  pinned.held_device['sr_held.mjs'] = await sha256File(path.join(repo, 'slam_refact/tools/prereg/sr_held.mjs'));
  const manifest = {
    gate_id: GATE_ID, version: VERSION, file: path.basename(gateRel), sha256: sha256(fs.readFileSync(path.join(root, gateRel))),
    created_at: '2026-10-20T03:00:00+09:00', first_real_run_at: null,
    registered_run: { seed: heldSeed ? null : 13, frames: 132 }, tolerance: { GL_A2n_level: 25 },
    pinned,
    held: { kinds: ['roi', 'it'], pins: heldPinsOf(root, pinned), inputs: {}, seed: heldSeed, dev_seeds: ['14'], sealed_file_sha256: [] },
    review: {},
  };
  const api = {
    root, manifest, gate, gateRel, reviewRel, designationRel, manifestRel, seedRel,
    write() { put(manifestRel, JSON.stringify(manifest, null, 1)); },
    put,
    // the coordinator's act: designation file + review fields in the manifest
    review({ at = '2026-10-20T04:00:00+09:00', permitted = true, mutateDesignation } = {}) {
      const designation = { gate: `${GATE_ID}.v${VERSION}`, first_real_run_permitted: permitted,
        pinned: { gate_md_sha256: manifest.sha256, manifest_before_review_sha256: manifestBeforeReviewSha256(manifest), review_file_sha256: sha256(fs.readFileSync(path.join(root, reviewRel))) } };
      if (mutateDesignation) mutateDesignation(designation);
      put(designationRel, JSON.stringify(designation));
      manifest.review = { reviewer: 'independent reviewer (fake)', at, file: path.basename(reviewRel), sha256: sha256(fs.readFileSync(path.join(root, reviewRel))),
        designation_file: path.basename(designationRel), designation_file_sha256: sha256(fs.readFileSync(path.join(root, designationRel))) };
      api.write();
      return manifest.review.designation_file_sha256;
    },
  };
  api.write();
  return api;
}

const gateRoi = (over = {}) => ({ mode: 'roi', kind: 'gate', seed: '14', noTargets: false, mutate: undefined, frames: '132', chunkDir: undefined, engineDir: ENGINE_DIR, ...over });
const gateIt = (over = {}) => ({ mode: 'it', kind: 'smoke', seed: '13', noTargets: false, mutate: undefined, frames: '132', chunkDir: undefined, engineDir: ENGINE_DIR, ...over });
const call = (f, values, extra = {}) => checkHold({ root: f.root, gate: f.gate, values, env: { [ENV_NAME]: f.sha }, environment: ENV, now: NOW, ...extra });
const refuses = async (f, values, code, extra) => {
  await assert.rejects(call(f, values, extra), (e) => new RegExp(`^HELD: (${code}):`).test(e.message), `${code}: ${JSON.stringify(values)}`);
};
async function reviewed(opts) { const f = await fixture(opts); f.sha = f.review(); return f; }

test('what is gate data: the roi gate camera, and the registered sequence in any spelling of its seed', () => {
  const registered = { seed: 13, frames: 132 };
  assert.equal(isGateData(gateRoi(), registered), true);
  assert.equal(isGateData(gateRoi({ kind: 'smoke' }), registered), false);
  for (const kind of ['const', 'ramp']) assert.equal(isGateData(gateRoi({ kind }), registered), false);
  for (const seed of ['13', '013', ' 13', '+13', '0013 ']) assert.equal(isGateData(gateIt({ seed }), registered), true, `seed ${JSON.stringify(seed)}`);
  assert.equal(isGateData(gateIt({ seed: '14' }), registered), false);
  assert.equal(isGateData(gateIt({ seed: '13', noTargets: true }), registered), false, 'the no-target null-run on the gate sequence is allowed');
  assert.equal(isGateData(gateIt({ kind: 'gate', seed: undefined }), { seed: null, frames: 132 }), true, 'a held seed: the kind decides');
  assert.equal(isGateData(gateIt({ kind: 'gate', noTargets: true }), registered), false);
  assert.equal(isGateData(gateIt({ seed: '13' }), null), false, 'without a manifest nothing is registered, so nothing is gate data (v4 had the seed 13 in the code)');
  assert.equal(isGateData(gateIt({ seed: '13' }), { seed: null, frames: 132 }), false);
  for (const bad of ['abc', '', '13.0', '0x0d', '1e1', '13a', ' ', '-']) assert.throws(() => parseSeed(bad), /bad --seed/, `seed ${JSON.stringify(bad)}`);
  assert.throws(() => isGateData(gateIt({ seed: '1.3e1' }), registered), /bad --seed/);
  assert.equal(gateDataKind({ mode: 'roi' }), 'roi'); assert.equal(gateDataKind({ mode: 'it' }), 'it');
});

test('null-runs and smoke data are not held, with no manifest at all and no gate arguments', async () => {
  const root = fs.mkdtempSync(path.join(scratch, 'hold-empty-'));
  made.push(root);
  for (const values of [gateRoi({ kind: 'smoke' }), gateRoi({ kind: 'const' }), gateRoi({ kind: 'ramp', mutate: 'MF' }), gateIt({ seed: '14' }), gateIt({ seed: '13', noTargets: true }), gateIt({ seed: '14', chunkDir: 'somewhere' })]) {
    const r = await checkHold({ root, gate: null, values, env: {}, environment: ENV });
    assert.equal(r.gateData, false);
  }
  await assert.rejects(checkHold({ root, gate: null, values: gateRoi(), env: {}, environment: ENV }), /^Error: HELD: args: gate data needs --prereg-dir/);
});

test('everything in order: the gate-data runs pass and say what was checked', async () => {
  const f = await reviewed();
  for (const values of [gateRoi(), gateIt(), gateIt({ seed: '013' }), gateIt({ seed: ' 13' }), gateIt({ kind: 'gate' })]) {
    const r = await call(f, values);
    assert.equal(r.gateData, true);
    assert.deepEqual(r.checks.map((c) => c.id), ['args', 'device', 'pins', 'first-run']);
    assert.ok(r.checks.every((c) => c.ok));
    assert.equal(r.pinnedBrowser, ENV.browser);
    assert.ok(r.log.some((l) => l.includes('pin chunk-hold.mjs') || l.includes('pin xr-slam.js')), 'the device log lists the pinned files');
  }
  assert.deepEqual(checkBrowser('1.2.3.4', ENV.browser), { id: 'browser', ok: true, detail: '1.2.3.4' });
  assert.throws(() => checkBrowser('9.9.9.9', ENV.browser), /HELD: browser:/);
});

test('no review recorded, or recorded in the wrong order: held', async () => {
  const f = await fixture();
  f.sha = 'a'.repeat(64);
  await refuses(f, gateRoi(), 'MANIFEST_INVALID');             // review {} : the review is not recorded
  await refuses(f, gateIt({ seed: '013' }), 'MANIFEST_INVALID');
  const g = await fixture();
  g.sha = g.review({ at: '2026-10-20T02:00:00+09:00' });  // before created_at
  await refuses(g, gateRoi(), 'ORDER');
  const h = await fixture();
  h.sha = h.review({ at: '2026-10-21T00:00:00+09:00' });  // after now
  await refuses(h, gateRoi(), 'ORDER');
  const i = await fixture();
  i.sha = i.review();
  i.manifest.review.reviewer = '';
  i.write();
  await refuses(i, gateRoi(), 'MANIFEST_INVALID');
  const j = await fixture();
  j.sha = j.review();
  j.put(j.reviewRel, '# a different review text\n');  // the review file is not the recorded one
  await refuses(j, gateRoi(), 'REVIEW_FILE_SHA');
  const k = await fixture();
  k.sha = k.review();
  delete k.manifest.review.sha256;
  k.write();
  await refuses(k, gateRoi(), 'MANIFEST_INVALID');
  const l = await fixture();
  l.sha = l.review();
  l.manifest.review.designation_file = '../../outside.json';
  l.write();
  await refuses(l, gateRoi(), 'NAME_UNSAFE');
});

test('the designation env value is read by the NAME given as an argument: missing, wrong, not the file, or "non-empty" is not enough', async () => {
  const f = await reviewed();
  await assert.rejects(checkHold({ root: f.root, gate: f.gate, values: gateRoi(), env: {}, environment: ENV, now: NOW }), /HELD: ENV_MISSING:/);
  await refuses({ ...f, sha: '1' }, gateRoi(), 'ENV_MISMATCH');
  await refuses({ ...f, sha: 'b'.repeat(64) }, gateRoi(), 'ENV_MISMATCH');
  // the right value under another name is unset: the name is an argument, not a constant of the tool
  await assert.rejects(checkHold({ root: f.root, gate: { ...f.gate, designationEnv: 'SLAM_REFACT_RM4C_OTHER_SHA256' }, values: gateRoi(), env: { [ENV_NAME]: f.sha }, environment: ENV, now: NOW }), /HELD: ENV_MISSING:/);
  // manifest and env agree with each other but not with the file
  const g = await fixture();
  g.review();
  const fake = 'c'.repeat(64);
  g.manifest.review.designation_file_sha256 = fake;
  g.write();
  await refuses({ ...g, sha: fake }, gateRoi(), 'DESIGNATION_SHA');
  // the designation file changed after the coordinator pinned its hash
  const h = await reviewed();
  h.put(h.designationRel, JSON.stringify({ gate: `${GATE_ID}.v${VERSION}`, first_real_run_permitted: true }));
  await refuses(h, gateRoi(), 'DESIGNATION_SHA');
});

test('the designation file must name this gate and pin the manifest, the gate file and the review', async () => {
  const wrong = async (mutateDesignation, code) => {
    const f = await fixture();
    f.sha = f.review({ mutateDesignation });
    await refuses(f, gateRoi(), code);
  };
  await wrong((d) => { d.first_real_run_permitted = false; }, 'DESIGNATION_NOT_PERMITTED');
  await wrong((d) => { delete d.first_real_run_permitted; }, 'DESIGNATION_NOT_PERMITTED');
  await wrong((d) => { d.gate = 'rm4c-chunk-gate-v2'; }, 'DESIGNATION_GATE');
  await wrong((d) => { d.gate = `${GATE_ID}.v${VERSION}0`; }, 'DESIGNATION_GATE');
  await wrong((d) => { d.pinned.manifest_before_review_sha256 = '0'.repeat(64); }, 'DESIGNATION_PIN');
  await wrong((d) => { d.pinned.review_file_sha256 = '0'.repeat(64); }, 'DESIGNATION_PIN');
  await wrong((d) => { d.pinned.gate_md_sha256 = '0'.repeat(64); }, 'DESIGNATION_PIN');
  // a manifest edited after designation (any field of the core, e.g. a loosened tolerance) is not the pinned one
  const f = await fixture();
  f.sha = f.review();
  f.manifest.tolerance = { GL_A2n_level: 30 };
  f.write();
  await refuses(f, gateRoi(), 'DESIGNATION_PIN');
  // while review / first_real_run_at are not part of the pinned form: filling them does not invalidate the designation (that is the order of events)
  const g = await fixture();
  g.sha = g.review();
  assert.equal(manifestBeforeReviewSha256({ ...g.manifest, review: null, first_real_run_at: 'x' }), manifestBeforeReviewSha256(g.manifest));
});

test('the gate file, the measurement device, the chunk and the engine are the registered bytes', async () => {
  const f = await reviewed();
  f.put(f.gateRel, '# gate v5 (edited after registration)\n');
  await refuses(f, gateRoi(), 'GATE_SHA_MISMATCH');
  for (const rel of [...DEVICE_FILES, ...CHUNK_FILES, ...TEST_FILES, ...GATE_COMMON_FILES]) {
    const g = await reviewed();
    g.put(rel, `// edited ${rel}\n`);
    await refuses(g, gateRoi(), 'PIN_MISMATCH');
  }
  for (const file of ENGINE_FILES) {
    const g = await reviewed();
    g.put(path.join(ENGINE_DIR, file), `tampered ${file}`);
    await refuses(g, gateIt(), 'PIN_MISMATCH');
  }
  // every environment entry counts, the versions and the sha256 of the node / Chromium / python executables and the package trees (G-5)
  for (const key of Object.keys(ENV)) {
    const g = await reviewed();
    await refuses(g, gateRoi(), 'pins', { environment: { ...ENV, [key]: 'other' } });
  }
  const noKey = await reviewed();  // an entry the live environment lacks is a difference too
  const { node_sha256: omitted, ...fewer } = ENV;
  await refuses(noKey, gateRoi(), 'pins', { environment: fewer });
  // and an entry the manifest does not pin (dropped from `pinned` before the designation, so the designation still matches) is one as well
  const unpinned = await fixture();
  delete unpinned.manifest.pinned.environment.browser_binary_sha256;
  unpinned.sha = unpinned.review();
  await refuses(unpinned, gateRoi(), 'pins');
  for (const group of ['tests', 'gate_common', 'held_device']) {  // a group of the record may not be missing: it is refused, not read as "nothing to compare"
    const g = await fixture();
    delete g.manifest.pinned[group];
    g.sha = g.review();
    await assert.rejects(call(g, gateRoi()), /^Error: HELD: pins: manifest\.pinned is incomplete/, group);
  }
  // an extra file in the pinned groups is a difference too (a removed pin cannot hide a changed file)
  const h = await reviewed();
  delete h.manifest.pinned.device[DEVICE_FILES[0]];
  h.write();
  h.sha = h.review();
  await refuses(h, gateRoi(), 'pins|PIN_NOT_PRESENTED');  // the device registers the file (held.pins) but it is no longer presented
  // the record and the device's flat form must agree
  const i = await fixture();
  i.manifest.held.pins['chunk-hold.mjs'] = '0'.repeat(64);
  i.sha = i.review();
  await refuses(i, gateRoi(), 'pins');
  const j = await fixture();
  delete j.manifest.held.pins['xr-slam.js'];
  j.sha = j.review();
  await refuses(j, gateRoi(), 'pins');
});

test('--chunk-dir, --mutate, another --engine-dir, a short sequence, a wrong seed, other chunk bytes than the caller expects, a second run, another gate version: refused for gate data', async () => {
  const f = await reviewed();
  await refuses(f, gateRoi({ chunkDir: 'build/copy-of-the-chunk' }), 'args');
  await refuses(f, gateIt({ chunkDir: 'slam_refact/web-chunk' }), 'args');  // even the repo path spelled out
  await refuses(f, gateRoi({ mutate: 'MF' }), 'args');
  await refuses(f, gateIt({ mutate: 'O1' }), 'args');
  await refuses(f, gateIt({ frames: '131' }), 'args');
  await refuses(f, gateIt({ frames: '132abc' }), 'args');
  await refuses(f, gateIt({ kind: 'gate', seed: '14' }), 'args');  // the registered seed is 13
  // the caller states the chunk bytes it expects (the freeze record): another value is refused, a right one passes
  const xr = await sha256File(path.join(f.root, CHUNK_FILES[0]));
  await refuses(f, gateRoi({ expectChunk: [`xr-slam.js=${'0'.repeat(64)}`] }), 'args');
  await refuses(f, gateRoi({ expectChunk: ['xr-slam.js=abc'] }), 'args');
  await refuses(f, gateRoi({ expectChunk: [`not-a-chunk-file.js=${xr}`] }), 'args');
  f.put('slam_refact/web-chunk/other.js', '// a file in the chunk directory that is not one of the pinned chunk files\n');
  await assert.rejects(call(f, gateRoi({ expectChunk: [`other.js=${await sha256File(path.join(f.root, 'slam_refact/web-chunk/other.js'))}`] })), /^Error: HELD: args: --expect-chunk-sha256 other\.js=.* of a chunk file is required/);
  await assert.doesNotReject(call(f, gateRoi({ expectChunk: [`xr-slam.js=${xr}`] })));
  // a copy of the pinned engine in another place, with identical bytes
  const copy = path.join(f.root, 'build/engine-copy');
  fs.mkdirSync(copy, { recursive: true });
  for (const file of ENGINE_FILES) fs.copyFileSync(path.join(f.root, ENGINE_DIR, file), path.join(copy, file));
  await refuses(f, gateRoi({ engineDir: 'build/engine-copy' }), 'args');
  await refuses(f, gateRoi({ engineDir: undefined }), 'args');
  const g = await reviewed();
  g.manifest.first_real_run_at = '2026-10-20T06:00:00+09:00';
  g.write();
  await refuses(g, gateRoi(), 'ALREADY_RUN');
  const h = await reviewed();
  h.manifest.version = 2;
  h.write();
  await refuses(h, gateRoi(), 'MANIFEST_IDENTITY');
  const i = await reviewed();
  fs.rmSync(path.join(i.root, i.manifestRel));
  await refuses(i, gateRoi({ kind: 'gate' }), 'args|manifest');
});

test('the live pins are what the manifest compares: a repo-side read of the current device, chunk, gate-common and engine files (the engine directory is an argument)', async () => {
  const engineDir = 'build/slam-refact-full-20261019/chunk/rm4d-20261020/engine-pinned';  // any existing engine directory: it is the argument
  const pins = await livePins(repo, ENV, engineDir);
  assert.deepEqual(Object.keys(pins.device), DEVICE_FILES);
  assert.deepEqual(Object.keys(pins.chunk), CHUNK_FILES);
  assert.deepEqual(Object.keys(pins.tests), TEST_FILES);
  assert.deepEqual(Object.keys(pins.gate_common), GATE_COMMON_FILES);
  for (const rel of TEST_FILES) assert.equal(pins.tests[rel], await sha256File(path.join(repo, rel)), rel);
  assert.equal(pins.engine.dir, engineDir, 'the engine directory is the argument');
  assert.deepEqual(Object.keys(pins.engine), ['dir', ...ENGINE_FILES]);
  assert.equal(pins.device['slam_refact/web-chunk/tests/chunk-hold.mjs'], await sha256File(path.join(repo, 'slam_refact/web-chunk/tests/chunk-hold.mjs')));
  assert.equal(pins.held_device['sr_held.mjs'], await sha256File(path.join(repo, 'slam_refact/tools/prereg/sr_held.mjs')));
  // every pinned file has a unique base name (the device registers pins by base name) - the real lists, not the synthetic tree
  const flat = heldPinsOf(repo, pins);
  assert.ok('chunk-hold.mjs' in flat && 'xr-slam.js' in flat && 'sr_held.mjs' in flat && 'node' in flat);
  assert.equal(Object.keys(flat).length, DEVICE_FILES.length + CHUNK_FILES.length + TEST_FILES.length + GATE_COMMON_FILES.length + ENGINE_FILES.length + 1 + 1, 'files + 3 engine files + sr_held + node');
});

// ---- RM-4e (v3 review G-3, G-4, G-5) ----

test('RM4E-H1 (G-3): the first real run of a kind of gate data is claimed once; a second run of the same kind is refused, in any spelling', async () => {
  const f = await reviewed();
  const hold = await call(f, gateRoi());
  const markerRel = path.relative(f.root, path.join(f.gate.preregDir, `${GATE_ID}.v${VERSION}.first-run.roi.marker`));
  assert.equal(hold.markerRel, markerRel);
  assert.ok(!fs.existsSync(path.join(f.root, hold.markerRel)), 'checkHold only checks, it does not create');
  const claimed = await claimFirstRun({ root: f.root, hold });
  assert.equal(claimed.marker, markerRel);
  assert.equal(claimed.seed, null, 'no held seed in this registration');
  assert.match(fs.readFileSync(path.join(f.root, markerRel), 'utf8'), /"kind":"roi"/);
  assert.equal(fs.statSync(path.join(f.root, markerRel)).mode & 0o222, 0, 'the marker is read-only');
  // the same kind again: refused by the check and (racing runners) by the exclusive create
  await refuses(f, gateRoi(), 'first-run');
  await assert.rejects(claimFirstRun({ root: f.root, hold }), /HELD: MARKER_EXISTS:/);
  // the other kind is independent, and every spelling of seed 13 is the same kind
  const itHold = await call(f, gateIt());
  assert.match(itHold.markerRel, /\.first-run\.it\.marker$/);
  await claimFirstRun({ root: f.root, hold: itHold });
  for (const seed of ['13', '013', ' 13', '+13']) await refuses(f, gateIt({ seed }), 'first-run');
  await refuses(f, gateRoi(), 'first-run');
  // with the markers in place the manifest is still the registered one (first_real_run_at null): the marker, not the manifest, stops the repeat
  assert.equal(f.manifest.first_real_run_at, null);
});

test('RM4E-H2 (G-3): null-runs and smoke data claim nothing; two runners racing for the marker: exactly one starts', async () => {
  const f = await reviewed();
  for (const values of [gateRoi({ kind: 'smoke' }), gateIt({ seed: '14' }), gateIt({ seed: '13', noTargets: true })]) {
    const hold = await call(f, values);
    assert.equal(await claimFirstRun({ root: f.root, hold }), null);
  }
  assert.deepEqual(fs.readdirSync(path.join(f.root, 'prereg')).filter((n) => n.includes('first-run')), []);
  const hold = await call(f, gateRoi());
  const results = await Promise.allSettled([1, 2, 3].map(() => claimFirstRun({ root: f.root, hold })));
  assert.equal(results.filter((r) => r.status === 'fulfilled').length, 1);
  assert.ok(results.filter((r) => r.status === 'rejected').every((r) => /HELD: MARKER_EXISTS:/.test(r.reason.message)));
});

test('RM4E-H3 (G-4, reviewer mutant HM1): files and manifest.pinned changed together after the designation are not the pinned manifest', async () => {
  const f = await reviewed();
  await call(f, gateRoi());  // the designation holds as designated
  const core = manifestBeforeReviewSha256(f.manifest);
  assert.notEqual(manifestBeforeReviewSha256({ ...f.manifest, pinned: { ...f.manifest.pinned, device: {} } }), core, 'pinned is part of the pinned form');
  assert.notEqual(manifestBeforeReviewSha256({ ...f.manifest, pinned: { ...f.manifest.pinned, environment: { ...ENV, node_sha256: 'e'.repeat(64) } } }), core);
  assert.notEqual(manifestBeforeReviewSha256({ ...f.manifest, held: { ...f.manifest.held, pins: {} } }), core, 'held is part of the pinned form');
  // the executor swaps a measurement file AND re-pins it in the manifest: the live bytes match the manifest, the designation does not
  f.put(DEVICE_FILES[3], '// a different comparison script\n');
  f.manifest.pinned = await livePins(f.root, ENV, ENGINE_DIR);
  f.manifest.pinned.held_device['sr_held.mjs'] = await sha256File(path.join(repo, 'slam_refact/tools/prereg/sr_held.mjs'));
  f.manifest.held.pins = heldPinsOf(f.root, f.manifest.pinned);
  f.write();
  await refuses(f, gateRoi(), 'DESIGNATION_PIN');
  // the same for a test file and for the pinned environment
  const g = await reviewed();
  g.put(TEST_FILES[0], '// a test that always passes\n');
  g.manifest.pinned = await livePins(g.root, ENV, ENGINE_DIR);
  g.manifest.pinned.held_device['sr_held.mjs'] = await sha256File(path.join(repo, 'slam_refact/tools/prereg/sr_held.mjs'));
  g.manifest.held.pins = heldPinsOf(g.root, g.manifest.pinned);
  g.write();
  await refuses(g, gateIt(), 'DESIGNATION_PIN');
  const h = await reviewed();
  h.manifest.pinned.environment = { ...ENV, browser_binary_sha256: '9'.repeat(64) };
  h.write();
  await refuses(h, gateRoi(), 'DESIGNATION_PIN');
});

test('RM4E-H4 (reviewer mutant HM9): the gate file must be a bare name in the preregistration directory even when its sha256 matches everything', async () => {
  const f = await fixture();
  const outside = 'build/outside-the-prereg-dir/gate.md';
  f.put(outside, '# a gate file kept elsewhere\n');
  f.manifest.file = outside;
  f.manifest.sha256 = sha256(fs.readFileSync(path.join(f.root, outside)));
  f.sha = f.review();
  await refuses(f, gateRoi(), 'NAME_UNSAFE');
});

test('RM4E-H5 (G-5): the environment identifies executables and package trees by sha256; the tree hash ignores compiled caches only', async () => {
  const env = liveEnvironment(repo);
  for (const key of ['node', 'node_sha256', 'browser', 'browser_binary_sha256', 'playwright', 'playwright_tree_sha256', 'python', 'python_sha256', 'numpy',
    'numpy_tree_sha256', 'pillow', 'pillow_tree_sha256']) {
    assert.equal(typeof env[key], 'string', key);
    assert.ok(env[key] && !env[key].startsWith('unavailable'), `${key}: ${env[key]}`);
  }
  for (const key of ['node_sha256', 'browser_binary_sha256', 'playwright_tree_sha256', 'python_sha256', 'numpy_tree_sha256', 'pillow_tree_sha256']) {
    assert.match(env[key], /^[0-9a-f]{64}$/, key);
  }
  assert.equal(env.node_sha256, fileSha256Sync(fs.realpathSync(process.execPath)));
  // fileSha256Sync reads in blocks: the same digest as the whole-file hash across a block boundary
  const dir = fs.mkdtempSync(path.join(scratch, 'tree-'));
  made.push(dir);
  const big = Buffer.alloc((1 << 20) * 2 + 17, 7);
  fs.writeFileSync(path.join(dir, 'big.bin'), big);
  assert.equal(fileSha256Sync(path.join(dir, 'big.bin')), sha256(big));
  assert.equal(fileSha256Sync(path.join(dir, 'missing')), 'unavailable');
  // tree hash: a changed byte, an added file and a renamed file change it; __pycache__ / *.pyc do not
  fs.mkdirSync(path.join(dir, 'pkg/sub'), { recursive: true });
  fs.writeFileSync(path.join(dir, 'pkg/a.py'), 'a');
  fs.writeFileSync(path.join(dir, 'pkg/sub/b.py'), 'b');
  const base = treeSha256(path.join(dir, 'pkg'));
  assert.match(base, /^[0-9a-f]{64}$/);
  fs.mkdirSync(path.join(dir, 'pkg/__pycache__'));
  fs.writeFileSync(path.join(dir, 'pkg/__pycache__/a.cpython-313.pyc'), 'cache');
  fs.writeFileSync(path.join(dir, 'pkg/sub/c.pyc'), 'cache');
  assert.equal(treeSha256(path.join(dir, 'pkg')), base);
  fs.writeFileSync(path.join(dir, 'pkg/sub/b.py'), 'B');
  assert.notEqual(treeSha256(path.join(dir, 'pkg')), base);
  fs.writeFileSync(path.join(dir, 'pkg/sub/b.py'), 'b');
  fs.renameSync(path.join(dir, 'pkg/a.py'), path.join(dir, 'pkg/z.py'));
  assert.notEqual(treeSha256(path.join(dir, 'pkg')), base);
  assert.equal(treeSha256(path.join(dir, 'no-such-dir')), 'unavailable');
});

// ---- v5: nothing of the gate is in the code; the coordinator's seed ----

test('RM4V5-1: the gate arguments are required and validated (id, version, preregistration directory, the NAME of the environment variable)', () => {
  const ok = { 'prereg-dir': 'pre', 'gate-id': 'rm4c-chunk-gate', 'gate-version': '5', 'designation-env': 'SLAM_REFACT_RM4C_V5_DESIGNATION_SHA256' };
  assert.deepEqual(gateArgs(ok, '/r'), { gateId: 'rm4c-chunk-gate', version: 5, preregDir: '/r/pre', designationEnv: 'SLAM_REFACT_RM4C_V5_DESIGNATION_SHA256' });
  for (const k of Object.keys(ok)) assert.throws(() => gateArgs({ ...ok, [k]: undefined }, '/r'), new RegExp(`--${k}`));
  assert.throws(() => gateArgs({ ...ok, 'gate-version': '5.0' }, '/r'), /positive integer/);
  assert.throws(() => gateArgs({ ...ok, 'designation-env': 'lowercase_name' }, '/r'), /environment variable NAME/);
});

test('RM4V5-2: the registered manifest decides the pinned engine directory, the sequence length and the seed (v4 had them in the code); a held seed is returned only by the claim', async () => {
  // another engine directory and another length in the manifest are the registered ones
  const f = await fixture();
  f.manifest.registered_run = { seed: 21, frames: 90 };
  f.sha = f.review();
  await refuses(f, gateIt({ kind: 'gate', seed: '13', frames: '132' }), 'args');
  await assert.doesNotReject(call(f, gateIt({ kind: 'gate', seed: '21', frames: '90' })));
  assert.equal((await call(f, gateIt({ seed: '21', frames: '90' }))).gateData, true, 'the registered seed makes a smoke-kind run gate data');
  assert.equal((await call(f, gateIt({ seed: '13', frames: '90' }))).gateData, false, 'seed 13 is not special any more');
  // a registration without the sequence length, or without any seed (neither held nor registered), cannot be run
  const nolen = await fixture();
  nolen.manifest.registered_run = { seed: 13 };
  nolen.sha = nolen.review();
  await assert.rejects(call(nolen, gateIt({ kind: 'gate' })), /^Error: HELD: args: manifest\.registered_run\.frames .* is missing/);
  const noseed = await fixture();
  noseed.manifest.registered_run = { seed: null, frames: 132 };
  noseed.sha = noseed.review();
  await assert.rejects(call(noseed, gateIt({ kind: 'gate' })), /^Error: HELD: seed: neither a held coordinator seed nor a registered seed/);
  // a held seed: the manifest says so, the seed file exists, the seed is returned by claimFirstRun and only there
  const h = await fixture({ heldSeed: true });
  h.put(h.seedRel, '918273645 coordinator seed (fake)\n');
  h.sha = h.review();
  const hold = await call(h, gateIt({ kind: 'gate', seed: undefined }));
  assert.equal(hold.seedHeld, true);
  assert.ok(!JSON.stringify(hold).includes('918273645'), 'the check does not carry the seed');
  const claimed = await claimFirstRun({ root: h.root, hold });
  assert.equal(claimed.seed, '918273645');
  assert.ok(!claimed.log.join('\n').includes('918273645'), 'the log has the sha256 of the seed file, not the value');
  // a seed file that is a seed the executor ran before the review is refused, and so is a missing one; a registered seed together with a held one
  const dev = await fixture({ heldSeed: true });
  dev.put(dev.seedRel, '14 a development seed\n');
  dev.sha = dev.review();
  await refuses(dev, gateIt({ kind: 'gate', seed: undefined }), 'SEED_FORBIDDEN');
  const none = await reviewed({ heldSeed: true });
  await refuses(none, gateIt({ kind: 'gate', seed: undefined }), 'SEED_MISSING');
  const both = await fixture({ heldSeed: true });
  both.put(both.seedRel, '5 seed\n');
  both.manifest.registered_run.seed = 13;
  both.sha = both.review();
  await refuses(both, gateIt({ kind: 'gate' }), 'seed');
});

test('RM4V5-3: manifestBeforeReviewSha256 is the value of the shared device (print-pin) for the same manifest', async () => {
  const f = await reviewed();
  assert.equal(manifestBeforeReviewSha256(f.manifest), sr.canonManifestSha(sr.parseJson(fs.readFileSync(path.join(f.root, f.manifestRel))).json));
  assert.ok(pinnedFiles(f.root, f.manifest.pinned).some(([p]) => p === process.execPath));
});
