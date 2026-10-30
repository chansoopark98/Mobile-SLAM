// node --test slam_refact/web-chunk/tests/us010-v4.test.mjs
// Unit tests of the US-010 v4 harness code (tests/us010-v4.mjs and the v4 expectations of sample-smoke-criteria.mjs): the manifest check before the first run, the
// pin list written once, verify (changed / missing / added / external / replay driver), and the M6 expectation of v4 (written from the v3 observation, labelled as such).
// No browser, no server: a synthetic repository tree under build/.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import { CONTROL_EXPECTATIONS, EXPECTED_FAIL, EXPECTED_PASS, controlOutcome } from './sample-smoke-criteria.mjs';
import { REPLAY_DRIVER, checkManifest, coreFiles, freeze, sha256File, verify } from './us010-v4.mjs';
import { makeWritable } from '../tools/gate-common/freeze.mjs';

const repo = path.resolve(import.meta.dirname, '../../..');
const base = path.join(repo, 'build/slam-refact-full-c1-chunk', `us010-v4-unit-${process.pid}`);
fs.rmSync(base, { recursive: true, force: true });
fs.mkdirSync(base, { recursive: true });
test.after(() => { makeWritable(base); fs.rmSync(base, { recursive: true, force: true }); });
let n = 0;

const put = (root, rel, text) => { const f = path.join(root, rel); fs.mkdirSync(path.dirname(f), { recursive: true }); fs.writeFileSync(f, text); return f; };
const goodManifest = { gate_id: 'us010-samples', version: 4, mutation: { detected: true }, null_run: { result: 'PASS' }, first_real_run_at: null };

// a tree of the shape CORE_ROOTS describes + the external files of a registered run
function tree(manifest = goodManifest) {
  const root = path.join(base, `t${++n}`);
  const core = {
    'slam_refact/web-chunk/xr-slam.js': 'chunk', 'slam_refact/web-chunk/sensors.js': 'sensors',
    'slam_refact/web-chunk/samples/serve.mjs': 'serve', 'slam_refact/web-chunk/samples/threejs/app.js': 'three app', 'slam_refact/web-chunk/samples/common/probe.js': 'probe',
    'slam_refact/web-chunk/tests/run-sample-smoke.mjs': 'smoke', 'slam_refact/web-chunk/tests/sample-smoke-criteria.mjs': 'criteria', 'slam_refact/web-chunk/tests/us010-v4.mjs': 'v4',
    'slam_refact/web-chunk/tests/worker-replay/replay-core.js': 'replay core', 'slam_refact/web-chunk/tests/fake-chunk/xr-slam.js': 'fake',
    'slam_refact/web-chunk/tools/gate-common/freeze.mjs': 'freeze', [REPLAY_DRIVER]: '#!/usr/bin/env bash\necho replay\n',
  };
  for (const [rel, text] of Object.entries(core)) put(root, rel, text);
  put(root, 'slam_refact/web-chunk/tests/not-core.test.mjs', 'not pinned');           // does not match a core root
  put(root, 'ext/sdk/xr.js', 'sdk'); put(root, 'ext/three/three.module.min.js', 'three'); put(root, 'ext/three/three.core.min.js', 'core'); put(root, 'ext/aframe/aframe-v1.5.0.min.js', 'aframe');
  for (const f of ['manifest.json', 'slam_refact_engine.js', 'slam_refact_engine.wasm']) put(root, `engine/${f}`, `engine ${f}`);
  put(root, 'browser/chrome', 'chromium binary'); put(root, 'browser/google-chrome', '#!/bin/sh\nexec chrome\n');
  put(root, 'capture/index.json', '{}');
  const manifestFile = put(root, 'prereg/us010-samples.v4.manifest.json', JSON.stringify(manifest));
  const ext = { browserExe: path.join(root, 'browser/google-chrome'), dirs: { sdkDir: path.join(root, 'ext/sdk'), threeDir: path.join(root, 'ext/three'), aframeDir: path.join(root, 'ext/aframe') } };
  return { root, manifestFile, ext, engineDir: path.join(root, 'engine'), out: path.join(root, 'pins/pins.json'), extra: [path.join(root, 'capture/index.json')] };
}
const args = (t, over = {}) => ({ repo: t.root, out: t.out, manifestFile: t.manifestFile, engineDir: t.engineDir, extra: t.extra, ext: t.ext, now: new Date('2026-10-20T12:00:00Z'), ...over });

test('US10V4-1: the manifest check refuses the first run unless mutation.detected is true and the null-run passed (v3 ran with mutation.detected null)', () => {
  const ok = (m) => { const t = tree(m); return checkManifest(t.manifestFile); };
  assert.equal(ok(goodManifest).ok, true);
  assert.equal(ok({ ...goodManifest, mutation: { detected: null } }).ok, false, 'null, as in v3');
  assert.equal(ok({ ...goodManifest, mutation: { detected: false } }).ok, false);
  assert.equal(ok({ ...goodManifest, mutation: { detected: 'true' } }).ok, false, 'a string is not true');
  assert.equal(ok({ ...goodManifest, mutation: undefined }).ok, false);
  assert.equal(ok({ ...goodManifest, null_run: { result: 'FAIL' } }).ok, false);
  assert.equal(ok({ ...goodManifest, null_run: undefined }).ok, false);
  assert.equal(ok({ ...goodManifest, first_real_run_at: '2026-10-20T05:53:11.5Z' }).ok, false, 'a run is on record already');
  const r = ok({ ...goodManifest, mutation: { detected: null } });
  assert.deepEqual(r.checks.filter((c) => !c.ok).map((c) => c.id), ['mutation.detected']);
  assert.equal(checkManifest(path.join(base, 'no-such.json')).ok, false);
});

test('US10V4-2: freeze writes the pin list once - core files, external bytes, the replay driver with its sha256 - read-only, and refuses a second write (v3 rewrote its SHA256SUMS)', () => {
  const t = tree();
  const rec = freeze(args(t));
  assert.deepEqual(Object.keys(rec.files), coreFiles(t.root), 'sorted core files');
  assert.ok(!('slam_refact/web-chunk/tests/not-core.test.mjs' in rec.files));
  assert.equal(rec.replayDriver.file, REPLAY_DRIVER);
  assert.equal(rec.replayDriver.sha256, sha256File(path.join(t.root, REPLAY_DRIVER)));
  assert.equal(rec.files[REPLAY_DRIVER], rec.replayDriver.sha256);
  for (const k of ['node', 'browser_binary', 'browser_wrapper', 'sdk_xr_js', 'three_module', 'three_core', 'aframe', 'engine/manifest.json', 'engine/slam_refact_engine.wasm', 'extra/0:index.json']) {
    assert.match(rec.external[k].sha256, /^[0-9a-f]{64}$/, k);
  }
  assert.equal(rec.external.browser_binary.sha256, sha256File(path.join(t.root, 'browser/chrome')), 'the wrapper points at the binary next to it');
  assert.equal(rec.external.browser_wrapper.sha256, sha256File(path.join(t.root, 'browser/google-chrome')));
  assert.equal(rec.manifest.sha256, sha256File(t.manifestFile));
  const onDisk = JSON.parse(fs.readFileSync(t.out, 'utf8'));
  assert.deepEqual(onDisk.files, rec.files);
  assert.equal(fs.statSync(t.out).mode & 0o222, 0, 'read-only');
  assert.throws(() => fs.writeFileSync(t.out, 'x'), (e) => e.code === 'EACCES');
  const before = sha256File(t.out);
  assert.throws(() => freeze(args(t)), /refused: .* exists - the pin list is written once/);
  assert.equal(sha256File(t.out), before, 'the first list is untouched');
});

test('US10V4-3: freeze refuses (and writes nothing) when mutation.detected is not true, when the replay driver is absent, or when an external file cannot be read', () => {
  const a = tree({ ...goodManifest, mutation: { detected: null } });
  assert.throws(() => freeze(args(a)), /refused: mutation\.detected is null/);
  assert.equal(fs.existsSync(a.out), false);
  const b = tree();
  fs.rmSync(path.join(b.root, REPLAY_DRIVER));
  assert.throws(() => freeze(args(b)), /replay driver .* is not among the core files/);
  assert.equal(fs.existsSync(b.out), false);
  const c = tree();
  fs.rmSync(path.join(c.root, 'ext/sdk/xr.js'));
  assert.throws(() => freeze(args(c)), /an external file is unreadable/);
  assert.equal(fs.existsSync(c.out), false);
  const d = tree();
  assert.throws(() => freeze(args(d, { extra: [path.join(d.root, 'capture/missing.json')] })), /an external file is unreadable/);
});

test('US10V4-4: verify reports a changed core file, a missing one, a file added to the core roots, a changed external byte, a changed extra and a changed replay driver', () => {
  const t = tree();
  freeze(args(t));
  const v = (over = {}) => verify({ repo: t.root, pinsFile: t.out, engineDir: t.engineDir, ext: t.ext, ...over });
  assert.equal(v().ok, true, 'as frozen');
  const change = (rel, text, fn) => { const f = path.join(t.root, rel); const old = fs.readFileSync(f); fs.writeFileSync(f, text); try { fn(); } finally { fs.writeFileSync(f, old); } };
  change('slam_refact/web-chunk/samples/threejs/app.js', 'edited', () => { const r = v(); assert.equal(r.ok, false); assert.deepEqual(r.mismatched.map((m) => m.file), ['slam_refact/web-chunk/samples/threejs/app.js']); });
  change('slam_refact/web-chunk/xr-slam.js', 'edited', () => assert.equal(v().mismatched.length, 1));
  change('slam_refact/web-chunk/tests/worker-replay/replay-core.js', 'edited', () => assert.equal(v().ok, false));
  change('slam_refact/web-chunk/tools/gate-common/freeze.mjs', 'edited', () => assert.equal(v().ok, false));
  change(REPLAY_DRIVER, '#!/usr/bin/env bash\necho replay with another loop\n', () => { const r = v(); assert.equal(r.ok, false); assert.equal(r.replayDriver.length, 1); assert.equal(r.mismatched.length, 1); });
  for (const rel of ['ext/sdk/xr.js', 'ext/three/three.core.min.js', 'ext/aframe/aframe-v1.5.0.min.js', 'engine/slam_refact_engine.wasm', 'browser/chrome', 'browser/google-chrome', 'capture/index.json']) {
    change(rel, rel.endsWith('google-chrome') ? '#!/bin/sh\nexec chrome --changed\n' : 'edited', () => { const r = v(); assert.equal(r.ok, false, rel); assert.equal(r.external.length, 1, rel); });
  }
  // missing
  const gone = path.join(t.root, 'slam_refact/web-chunk/sensors.js');
  const keep = fs.readFileSync(gone);
  fs.rmSync(gone);
  assert.deepEqual(v().missing, ['slam_refact/web-chunk/sensors.js']);
  fs.writeFileSync(gone, keep);
  // added: a new sample file, a new chunk file, a new gate-common file and a new test of the core regex appear after the freeze
  for (const rel of ['slam_refact/web-chunk/samples/aframe/app.js', 'slam_refact/web-chunk/extra-chunk-file.js', 'slam_refact/web-chunk/tools/gate-common/extra.mjs', 'slam_refact/web-chunk/tests/run-sample-ablation.mjs']) {
    const f = put(t.root, rel, 'new');
    assert.deepEqual(v().added, [rel], rel);
    fs.rmSync(f);
  }
  put(t.root, 'slam_refact/web-chunk/tests/unrelated-new-file.mjs', 'not in the core regex');
  assert.equal(v().ok, true, 'a file outside the core roots is not a core change');
  assert.equal(v().ok, true, 'and everything is back as frozen');
});

test('US10V4-5: the snapshot of the core files is written once with SHA256SUMS and read-only', () => {
  const t = tree();
  const snap = path.join(t.root, 'snapshot');
  const rec = freeze(args(t, { snapshotDir: snap }));
  assert.equal(rec.snapshot.sums.files, Object.keys(rec.files).length);
  for (const l of fs.readFileSync(path.join(snap, 'SHA256SUMS'), 'utf8').trim().split('\n')) { const [sha, f] = l.split('  '); assert.equal(sha256File(path.join(snap, f)), sha, f); }
  assert.equal(fs.statSync(path.join(snap, 'SHA256SUMS')).mode & 0o222, 0);
  assert.throws(() => fs.writeFileSync(path.join(snap, 'new'), 'x'), (e) => e.code === 'EACCES');
  // an existing snapshot directory (even an empty one) is refused BEFORE anything is written, so no second pin list appears
  const second = path.join(t.root, 'pins/second.json');
  assert.throws(() => freeze(args(t, { out: second, snapshotDir: snap })), /refused: .*snapshot is written once \(nothing was written\)/);
  const empty = path.join(t.root, 'empty-snapshot'); fs.mkdirSync(empty);
  assert.throws(() => freeze(args(t, { out: second, snapshotDir: empty })), /snapshot is written once/);
  assert.equal(fs.existsSync(second), false, 'no pin list was written');
});

test('US10V4-6: the M6 expectation of v4 is the v3 observation, registered for the new version; v3 is unchanged; the outcome is judged against the chosen expectations', () => {
  assert.deepEqual(CONTROL_EXPECTATIONS.v3.fail, EXPECTED_FAIL); assert.deepEqual(CONTROL_EXPECTATIONS.v3.pass, EXPECTED_PASS);
  assert.deepEqual(EXPECTED_FAIL.M6, [], 'v3 as registered (the expectation the result contradicted)');
  assert.deepEqual(CONTROL_EXPECTATIONS.v4.fail.M6, ['S10', 'S10b']);
  assert.deepEqual(CONTROL_EXPECTATIONS.v4.pass.M6, ['S7', 'S8', 'S9']);
  assert.match(CONTROL_EXPECTATIONS.v4.basis.M6, /v3 reg2 observation/);
  for (const c of ['N0', 'M1', 'M2', 'M3', 'M4', 'M5']) assert.deepEqual(CONTROL_EXPECTATIONS.v4.fail[c], EXPECTED_FAIL[c], `${c} is as in v3`);
  const checks = (failing) => Object.fromEntries(['S1', 'S2', 'S3', 'S4', 'S5', 'S6', 'S7', 'S8', 'S9', 'S10', 'S10b', 'S11', 'S12', 'S13', 'S14'].map((k) => [k, { pass: !failing.includes(k) }]));
  const outcome = (failing, expectations = CONTROL_EXPECTATIONS.v4) => controlOutcome({ checks: checks(failing), failed: failing, expectations, control: 'M6' });
  // what v3 observed: three.js FAIL S10 S10b S13, A-Frame FAIL S10 S10b - both detect under v4, and neither would under v3
  assert.equal(outcome(['S10', 'S10b', 'S13']).controlDetected, true);
  assert.equal(outcome(['S10', 'S10b']).controlDetected, true);
  assert.equal(outcome(['S10', 'S10b', 'S13'], CONTROL_EXPECTATIONS.v3).controlDetected, false, 'v3: the expectation (nothing fails) was missed');
  assert.equal(outcome(['S10']).controlDetected, false); assert.deepEqual(outcome(['S10']).controlMissed, ['S10b']);
  assert.equal(outcome([]).controlDetected, false, 'a pose-only scale error that no check sees is a miss, not a detection');
  assert.equal(outcome(['S10', 'S10b', 'S7']).controlDetected, false, 'S7 (self-consistency) must stay PASS: it is not a chunk detector');
  assert.deepEqual(outcome(['S10', 'S10b', 'S7']).controlPassViolations, ['S7']);
  assert.equal(controlOutcome({ checks: checks([]), failed: [], expectations: CONTROL_EXPECTATIONS.v4, control: null }).controlDetected, null, 'no control, nothing expected');
});
