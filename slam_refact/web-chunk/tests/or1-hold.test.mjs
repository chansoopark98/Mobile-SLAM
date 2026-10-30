// node --test slam_refact/web-chunk/tests/or1-hold.test.mjs
// Unit tests of the first-real-run hold of the OR-1 gate (tools/or1/hold.mjs over the shared device tools/prereg/sr_held.mjs; protocol 12.5): a synthetic
// repository root under build/ with a registered manifest (the sr_held shape: held.kinds / pins / inputs), a coordinator designation, a review file,
// pinned files and one registered bundle. Every defect must give "HELD: <CODE or check>:". The gate (id, version, preregistration directory) and the
// NAME of the environment variable are arguments of every call: nothing here depends on a constant of the tool (v2 review V-1, V-8).
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import { pathToFileURL } from 'node:url';

// SLAM_REFACT_OR1_TOOLS_DIR: a modified copy of tools/or1 (tests/run-or1-mutants.mjs shows that these tests fail on it); gate-common and tools/prereg
// are its siblings in the same layout as the repository
const toolsDir = process.env.SLAM_REFACT_OR1_TOOLS_DIR ?? path.join(import.meta.dirname, '../tools/or1');
const load = (...p) => import(pathToFileURL(path.join(toolsDir, ...p)).href);
const { assertNotGateData, checkHold, claimFirstRun, designationPins, fileSetSha256, heldPinsOf, liveEnvironment, pinnedFiles, sha256File } = await load('hold.mjs');
const { SR_HELD_FILE, sr } = await load('../gate-common/held.mjs');

const repo = path.resolve(import.meta.dirname, '../../..');
const base = path.join(repo, 'build/slam-refact-full-20261019/evidence/OR-1', `unit-tmp-hold-${process.pid}`);
fs.mkdirSync(base, { recursive: true });
let counter = 0;

const GATE_ID = 'or1-test-gate', VERSION = 3, ENV_NAME = 'SLAM_REFACT_OR1_TEST_V3_DESIGNATION_SHA256';
const DERIVED_JSON = { source: 'src/bundleA', sourceSha256: { 'engine-config.tsv': 'a'.repeat(64), 'sensors.tsv': 'b'.repeat(64), 'frames.tsv': 'c'.repeat(64) },
  derivedSha256: { 'engine-config.tsv': 'd'.repeat(64), 'sensors.tsv': 'e'.repeat(64), 'frames.tsv': 'f'.repeat(64) } };
const BUNDLE = 'bundles/A/bundle';

const write = (root, rel, text) => { const f = path.join(root, rel); fs.mkdirSync(path.dirname(f), { recursive: true }); fs.writeFileSync(f, text); return f; };
const readJson = (file) => JSON.parse(fs.readFileSync(file, 'utf8'));

// returns {root, gate, values, derivedMeta, env, now}; `edit` hooks change one thing at a precise point
function setup(edit = {}) {
  const root = path.join(base, `r${process.pid}-${++counter}`);
  const gate = { gateId: GATE_ID, version: VERSION, preregDir: path.join(root, 'prereg'), designationEnv: ENV_NAME };
  const sections = {
    harness: { 'slam_refact/web-chunk/tools/or1/run.mjs': 'run', 'slam_refact/web-chunk/tools/or1/build-pair.sh': 'sh' },
    gate_common: { 'slam_refact/web-chunk/tools/gate-common/freeze.mjs': 'freeze' },
    chunk: { 'slam_refact/web-chunk/xr-slam.js': 'chunk' },
    tests: { 'slam_refact/web-chunk/tests/or1.test.mjs': 'test' },
    docs: { 'slam_refact/docs/spec/07-xr-slam-chunk.md': 'spec' },
    pair: { 'pair/pair.json': '{}', 'pair/snapshot-sources.sha256': 'sums' },
    native: { 'native/replay': 'bin' },
    runner: { 'slam_refact/tools/gates/or1-test/run.sh': 'runner' },
  };
  for (const entries of Object.values(sections)) for (const [rel, text] of Object.entries(entries)) write(root, rel, text);
  for (const f of ['slam_refact_engine.js', 'slam_refact_engine.wasm', 'manifest.json']) write(root, `eng/${f}`, `engine ${f}`);
  write(root, 'atlas/a0.gz', 'atlas0'); write(root, 'atlas/a1.gz', 'atlas1');
  write(root, `${BUNDLE}/engine-config.tsv`, 'k\tv\n'); write(root, `${BUNDLE}/sensors.tsv`, 's\n'); write(root, `${BUNDLE}/frames.tsv`, 'f\n');
  write(root, `${BUNDLE}/or1-derived.json`, `${JSON.stringify(DERIVED_JSON)}\n`);
  if (edit.files) edit.files(root);
  const gateMd = write(root, `prereg/${GATE_ID}.v${VERSION}.md`, '# gate\n');
  const review = write(root, 'prereg/review-text.md', 'review text\n');
  const atlasSet = fileSetSha256(root, ['atlas/a0.gz', 'atlas/a1.gz']);
  const pinned = { engine: { dir: 'eng' }, environment: liveEnvironment(), held_device: { 'sr_held.mjs': sha256File(SR_HELD_FILE) } };
  for (const [section, entries] of Object.entries(sections)) pinned[section] = Object.fromEntries(Object.keys(entries).map((rel) => [rel, sha256File(path.join(root, rel))]));
  for (const f of ['slam_refact_engine.js', 'slam_refact_engine.wasm', 'manifest.json']) pinned.engine[f] = sha256File(path.join(root, 'eng', f));
  pinned.gate_data = [{ name: 'A', record: 'R1', source: DERIVED_JSON.source, source_sha256: DERIVED_JSON.sourceSha256, derived_sha256: DERIVED_JSON.derivedSha256, derived_dir: BUNDLE, atlas_set_sha256: atlasSet }];
  const manifest = {
    gate_id: GATE_ID, version: VERSION, file: path.basename(gateMd), sha256: sha256File(gateMd), created_at: '2026-10-20T10:00:00+09:00',
    registered_run: { arms: 'WCN', repetitions: 3 }, pinned,
    held: { kinds: ['A'], pins: heldPinsOf(root, pinned), inputs: { A: sr.treeSha(path.join(root, BUNDLE)).sha }, seed: false, dev_seeds: [], sealed_file_sha256: [] },
    review: {}, first_real_run_at: null,
  };
  if (edit.manifest) edit.manifest(manifest, root);
  const manifestFile = path.join(gate.preregDir, `${GATE_ID}.v${VERSION}.manifest.json`);
  fs.writeFileSync(manifestFile, `${JSON.stringify(manifest, null, 1)}\n`);
  const designationFile = path.join(gate.preregDir, 'designation.json');
  const pins = designationPins(manifestFile);
  const designation = { gate: `${GATE_ID}.v${VERSION}`, first_real_run_permitted: true,
    pinned: { gate_md_sha256: pins.gate_md_sha256, manifest_before_review_sha256: pins.manifest_before_review_sha256, review_file_sha256: sha256File(review) } };
  if (edit.designation) edit.designation(designation, root);
  fs.writeFileSync(designationFile, `${JSON.stringify(designation, null, 1)}\n`);
  manifest.review = { reviewer: 'independent reviewer designated by the coordinator', at: '2026-10-20T11:00:00+09:00', file: path.basename(review), sha256: sha256File(review),
    designation_file: path.basename(designationFile), designation_file_sha256: sha256File(designationFile) };
  if (edit.review) edit.review(manifest.review, root);
  fs.writeFileSync(manifestFile, `${JSON.stringify(manifest, null, 1)}\n`);
  if (edit.after) edit.after(root, manifest, gate);
  const values = { kind: 'gate', chunkDir: undefined, nativeBundle: undefined, engineDir: 'eng', nativeBin: 'native/replay', arms: 'WCN', reps: 3, atlasSetSha256: atlasSet, bundle: BUNDLE, ...(edit.values ?? {}) };
  const env = { [ENV_NAME]: fs.existsSync(designationFile) ? sha256File(designationFile) : 'missing', ...(edit.env ?? {}) };
  return { root, gate: edit.gate ? edit.gate(gate) : gate, values, derivedMeta: { ...DERIVED_JSON, ...(edit.derived ?? {}) }, env, now: new Date('2026-10-20T12:00:00+09:00') };
}
const held = (edit, code) => {
  const s = setup(edit);
  assert.throws(() => checkHold(s), (e) => e.message.startsWith(`HELD: ${code}:`), `expected HELD: ${code}`);
  return s;
};
const manifestOf = (root, name = `${GATE_ID}.v${VERSION}.manifest.json`) => path.join(root, 'prereg', name);
const edited = (root, fn) => { const f = manifestOf(root); const j = readJson(f); fn(j); fs.writeFileSync(f, `${JSON.stringify(j, null, 1)}\n`); };

test('OR1H-1: a complete registration passes every check and names the datum, the designation and the checks it made', () => {
  const s = setup();
  const h = checkHold(s);
  assert.equal(h.manifest.gate_id, GATE_ID);
  assert.equal(h.datum.name, 'A');
  assert.equal(h.designationSha, env0(s));
  assert.ok(h.log.some((l) => l.includes('pin node sha256')) && h.log.some((l) => l.includes('pin sr_held.mjs')) && h.log.some((l) => l.includes('inputs A tree sha256')), 'the device log lists pins and data');
});
const env0 = (s) => s.env[ENV_NAME];

test('OR1H-2: arguments that are not the registered run are held (kind, chunk copy, --native-bundle, engine, native binary, arms, repetitions)', () => {
  held({ values: { kind: 'dev' } }, 'args');
  held({ values: { chunkDir: 'build/some-mutant-copy/chunk' } }, 'args');
  held({ values: { nativeBundle: 'build/other-bundle' } }, 'args');  // V-3
  held({ values: { engineDir: 'other-eng' } }, 'args');
  held({ values: { nativeBin: 'native/other' } }, 'args');
  held({ values: { arms: 'WC' } }, 'args');
  held({ values: { reps: 2 } }, 'args');
  assert.doesNotThrow(() => checkHold(setup({ values: { chunkDir: 'slam_refact/web-chunk' } })), 'the repo chunk named explicitly is fine');
});

test('OR1H-3: manifest problems are held (missing, not JSON, other gate / version, already run)', () => {
  held({ after: (root) => fs.rmSync(manifestOf(root)) }, 'manifest');
  held({ after: (root) => fs.writeFileSync(manifestOf(root), '{ not json') }, 'manifest');
  held({ gate: (g) => ({ ...g, version: VERSION + 1 }) }, 'manifest');  // no manifest of that version
  held({ after: (root) => { fs.copyFileSync(manifestOf(root), manifestOf(root, `${GATE_ID}.v${VERSION + 1}.manifest.json`)); }, gate: (g) => ({ ...g, version: VERSION + 1 }) }, 'MANIFEST_IDENTITY');
  held({ after: (root) => edited(root, (m) => { m.first_real_run_at = '2026-10-20T11:30:00+09:00'; }) }, 'ALREADY_RUN');
  held({ after: (root) => edited(root, (m) => { m.held.kinds = ['A', 'A']; }) }, 'MANIFEST_INVALID');
});

test('OR1H-4: no review, an out-of-order review, a changed or missing review file are held', () => {
  held({ after: (root) => edited(root, (m) => { m.review = {}; }) }, 'MANIFEST_INVALID');
  held({ review: (r) => { r.at = null; } }, 'REVIEW_MISSING');
  held({ review: (r) => { r.at = '2026-10-20T09:00:00+09:00'; } }, 'ORDER');
  held({ review: (r) => { r.at = '2026-10-20T13:00:00+09:00'; } }, 'ORDER');
  held({ after: (root) => fs.appendFileSync(path.join(root, 'prereg/review-text.md'), 'edited after the review\n') }, 'REVIEW_FILE_SHA');
  held({ after: (root) => fs.rmSync(path.join(root, 'prereg/review-text.md')) }, 'REVIEW_FILE_MISSING');
  held({ after: (root) => fs.rmSync(path.join(root, 'prereg/designation.json')) }, 'DESIGNATION_MISSING');
  held({ review: (r) => { r.designation_file = '../designation.json'; } }, 'NAME_UNSAFE');
});

test('OR1H-5: the environment variable is read by the NAME given as an argument and must be the designation file sha256 (unset, empty, another value, another name)', () => {
  held({ env: { [ENV_NAME]: undefined } }, 'ENV_MISSING');
  held({ env: { [ENV_NAME]: '' } }, 'ENV_MISSING');
  held({ env: { [ENV_NAME]: 'x'.repeat(64) } }, 'ENV_MISMATCH');
  held({ env: { [ENV_NAME]: '1' } }, 'ENV_MISMATCH');
  // V-1: v2 read a name fixed in the code (…_V1_…) that the registered command did not export. Now the argument decides: the right value under another name is unset.
  const s = setup();
  assert.throws(() => checkHold({ ...s, gate: { ...s.gate, designationEnv: 'SLAM_REFACT_OR1_OTHER_NAME_SHA256' } }), (e) => e.message.startsWith('HELD: ENV_MISSING:'));
  assert.doesNotThrow(() => checkHold({ ...s, gate: { ...s.gate, designationEnv: 'SLAM_REFACT_OR1_OTHER_NAME_SHA256' }, env: { SLAM_REFACT_OR1_OTHER_NAME_SHA256: s.env[ENV_NAME] } }));
});

test('OR1H-6: the designation must name this gate and version, permit the run and pin the gate file, the review file and the manifest before the review', () => {
  held({ designation: (d) => { d.gate = 'rm4c-chunk-gate.v4'; } }, 'DESIGNATION_GATE');
  held({ designation: (d) => { d.gate = `${GATE_ID}.v${VERSION}0`; } }, 'DESIGNATION_GATE');
  held({ designation: (d) => { d.first_real_run_permitted = false; } }, 'DESIGNATION_NOT_PERMITTED');
  held({ designation: (d) => { d.pinned.gate_md_sha256 = '0'.repeat(64); } }, 'DESIGNATION_PIN');
  held({ designation: (d) => { d.pinned.manifest_before_review_sha256 = '0'.repeat(64); } }, 'DESIGNATION_PIN');
  held({ designation: (d) => { d.pinned.review_file_sha256 = '0'.repeat(64); } }, 'DESIGNATION_PIN');
  // the manifest changed after the designation pinned it (a repetition edit after the review; the designation sha is re-recorded so that only the manifest pin catches it)
  held({ after: (root) => { edited(root, (m) => { m.registered_run.repetitions = 1; });
    const d = path.join(root, 'prereg/designation.json'); edited(root, (m) => { m.review.designation_file_sha256 = sha256File(d); }); } }, 'DESIGNATION_PIN');
});

test('OR1H-7: the gate file must be the registered one, in the preregistration directory under its name', () => {
  held({ after: (root) => fs.appendFileSync(path.join(root, `prereg/${GATE_ID}.v${VERSION}.md`), 'changed after registration\n') }, 'GATE_SHA_MISMATCH');
  held({ after: (root) => edited(root, (m) => { m.file = 'other.md'; }) }, 'NAME_UNSAFE');
});

test('OR1H-8: every pinned file is compared (harness, gate-common, chunk, tests, docs, pair, engine files, native binary, runner, the node executable)', () => {
  for (const rel of ['slam_refact/web-chunk/tools/or1/run.mjs', 'slam_refact/web-chunk/tools/or1/build-pair.sh', 'slam_refact/web-chunk/tools/gate-common/freeze.mjs', 'slam_refact/web-chunk/xr-slam.js',
    'slam_refact/web-chunk/tests/or1.test.mjs', 'slam_refact/docs/spec/07-xr-slam-chunk.md', 'pair/pair.json', 'pair/snapshot-sources.sha256', 'eng/slam_refact_engine.js',
    'eng/slam_refact_engine.wasm', 'eng/manifest.json', 'native/replay', 'slam_refact/tools/gates/or1-test/run.sh']) {
    held({ after: (root) => fs.appendFileSync(path.join(root, rel), ' changed') }, 'PIN_MISMATCH');
  }
  const { root } = setup();
  const live = liveEnvironment();
  assert.equal(live.node_sha256, sha256File(process.execPath), 'the live pin is the sha256 of the node executable');
  assert.equal(live.node, process.version);
  const pinned = readJson(manifestOf(root)).pinned;
  assert.ok(pinnedFiles(root, pinned).some(([f]) => f === process.execPath), 'the node executable is presented');
  assert.ok(pinnedFiles(root, pinned).some(([f]) => f === SR_HELD_FILE), 'the device that was loaded is the device that is presented');
  held({ manifest: (m) => { m.pinned.environment.node = 'v0.0.0'; m.held.pins = heldPinsOf(process.cwd(), m.pinned); } }, 'pins');
  held({ manifest: (m) => { m.pinned.environment.node_sha256 = '0'.repeat(64); m.held.pins = heldPinsOf(process.cwd(), m.pinned); } }, 'PIN_MISMATCH');
  held({ manifest: (m) => { m.pinned.held_device['sr_held.mjs'] = '0'.repeat(64); m.held.pins = heldPinsOf(process.cwd(), m.pinned); } }, 'PIN_MISMATCH');
});

test('OR1H-9: a file next to the pinned ones that is not pinned is held; the record (pinned) must be the flat form the device holds (held.pins)', () => {
  held({ after: (root) => write(root, 'slam_refact/web-chunk/tools/or1/extra.mjs', 'x') }, 'pins');
  held({ after: (root) => write(root, 'slam_refact/web-chunk/tools/or1/extra.sh', 'x') }, 'pins');
  held({ after: (root) => write(root, 'slam_refact/web-chunk/tools/gate-common/extra.mjs', 'x') }, 'pins');
  held({ after: (root) => write(root, 'slam_refact/web-chunk/new-chunk-file.js', 'x') }, 'pins');
  held({ manifest: (m) => { m.held.pins['run.mjs'] = '0'.repeat(64); } }, 'pins');
  held({ manifest: (m) => { delete m.held.pins['run.mjs']; } }, 'pins');
  held({ manifest: (m) => { m.pinned.chunk['slam_refact/web-chunk/not-there.js'] = 'a'.repeat(64); } }, 'pins');
});

test('OR1H-10: only registered gate data runs: another bundle, a different source, another path (V-2), an edited derived file, other atlas files', () => {
  held({ derived: { source: 'src/bundleB' } }, 'data');
  held({ derived: { sourceSha256: { ...DERIVED_JSON.sourceSha256, 'sensors.tsv': '0'.repeat(64) } } }, 'data');
  // V-2: a copy of the registered bundle at another path is not the registered directory
  held({ files: (root) => { for (const f of ['engine-config.tsv', 'sensors.tsv', 'frames.tsv', 'or1-derived.json']) write(root, `copy/${f}`, fs.readFileSync(path.join(root, BUNDLE, f), 'utf8')); }, values: { bundle: 'copy' } }, 'data');
  // V-2: v2 compared the sha256 values WRITTEN in or1-derived.json; frames.tsv edited with or1-derived.json unchanged must now be caught by the live tree sha256
  held({ after: (root) => fs.appendFileSync(path.join(root, BUNDLE, 'frames.tsv'), 'frame\t9\n') }, 'INPUT_MISMATCH');
  held({ after: (root) => fs.appendFileSync(path.join(root, BUNDLE, 'sensors.tsv'), 'row\n') }, 'INPUT_MISMATCH');
  held({ after: (root) => write(root, `${BUNDLE}/extra.tsv`, 'x') }, 'INPUT_MISMATCH');
  held({ values: { atlasSetSha256: '0'.repeat(64) } }, 'data');
});

test('OR1H-11: the first run of each registered bundle is claimed exactly once, by an exclusive read-only marker; a marker of another bundle does not hold this one', () => {
  const s = setup();
  const h = checkHold(s);
  const markerPath = path.join(s.root, 'prereg', `${GATE_ID}.v${VERSION}.first-run.A.marker`);
  assert.equal(fs.existsSync(markerPath), false, 'checkHold creates nothing (it is the precheck)');
  const marker = claimFirstRun({ hold: h });
  assert.equal(marker, markerPath);
  assert.equal(fs.statSync(marker).mode & 0o222, 0, 'the marker is read-only');
  assert.match(fs.readFileSync(marker, 'utf8'), /"kind":"A"/);
  assert.throws(() => checkHold(s), (e) => e.message.startsWith('HELD: first-run:'), 'the precheck of a claimed bundle says so');
  assert.throws(() => claimFirstRun({ hold: h }), (e) => e.message.startsWith('HELD: MARKER_EXISTS:'));
  held({ after: (root) => write(root, `prereg/${GATE_ID}.v${VERSION}.first-run.A.marker`, '{}') }, 'first-run');
  assert.doesNotThrow(() => checkHold(setup({ after: (root) => write(root, `prereg/${GATE_ID}.v${VERSION}.first-run.B.marker`, '{}') })));
  // the manifest of the run says it was audited later: the audit sees the marker (sr.audit) once the coordinator recorded first_real_run_at
  edited(s.root, (m) => { m.first_real_run_at = '2026-10-20T12:30:00+09:00'; });
  const a = sr.audit({ preregDir: s.gate.preregDir, gateId: GATE_ID, version: VERSION, kind: 'A' });
  assert.equal(a.ok, true, `${a.code} ${a.why}`);
});

test('OR1H-12: a run that is not a gate run refuses the registered gate data (by the content of the derived bundle and by its source sha256) and reads others', () => {
  const s = setup();
  assert.throws(() => assertNotGateData({ root: s.root, gate: s.gate, derivedMeta: s.derivedMeta, bundleDir: BUNDLE }), /SEALED_DATA/);
  // an edited copy of the derived bundle has another tree sha256 but the same source: refused by the source sha256
  write(s.root, 'dev/engine-config.tsv', 'other\n'); write(s.root, 'dev/sensors.tsv', 'other\n');
  assert.throws(() => assertNotGateData({ root: s.root, gate: s.gate, derivedMeta: s.derivedMeta, bundleDir: 'dev' }), /registered gate data/);
  assert.doesNotThrow(() => assertNotGateData({ root: s.root, gate: s.gate, derivedMeta: { source: 'dev', sourceSha256: { 'sensors.tsv': '1'.repeat(64) } }, bundleDir: 'dev' }));
  const empty = { root: path.join(base, `r${process.pid}-empty`), gate: { ...s.gate, preregDir: path.join(base, `r${process.pid}-empty/prereg`) } };
  fs.mkdirSync(empty.gate.preregDir, { recursive: true });
  write(empty.root, 'dev/sensors.tsv', 'x\n');
  assert.doesNotThrow(() => assertNotGateData({ ...empty, derivedMeta: s.derivedMeta, bundleDir: 'dev' }), 'no manifest yet: nothing is registered');
});

test('OR1H-13: the designation pins printed by `or1.mjs pin` are the ones the device compares (print-pin of the shared device)', () => {
  const s = setup();
  const pins = designationPins(manifestOf(s.root));
  assert.equal(pins.manifest_before_review_sha256, sr.canonManifestSha(sr.parseJson(fs.readFileSync(manifestOf(s.root))).json));
  assert.equal(pins.gate_md_sha256, sha256File(path.join(s.root, 'prereg', `${GATE_ID}.v${VERSION}.md`)));
});

test.after(() => { fs.rmSync(base, { recursive: true, force: true }); });
