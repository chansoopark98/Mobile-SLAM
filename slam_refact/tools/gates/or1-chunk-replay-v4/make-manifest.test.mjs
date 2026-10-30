// node --test slam_refact/tools/gates/or1-chunk-replay-v4/make-manifest.test.mjs
// The v4 registration tool and the registered command, on a MADE-UP gate (ids or1-v4-e2e-a / or1-v4-e2e-b, never the real gate's preregistration directory, marker or
// environment variable) and on DEVELOPMENT data (12 frame prefixes of D1, D2 and the orient recording flip6-s2 v2-b-o90).
//   MM-1  the DRAFT of the real registered scenarios: derived and hashed, nothing played; the expected counts, capture sizes and rotations; the four portrait scenarios are the
//         v3 data (sha256 of source, derived bundle and atlas set equal to the v3 manifest when it is present); every F3 field is PENDING-F3; refuses to overwrite
//   MM-2  refusals of the final manifest: no pair, a selftest that did not pass, a gate document that is not the registration document
//   MM-3  end to end through run.sh (a copy whose only difference is the FREEZE_PATHS constant): dry run, PRECHECK_* recorded / refused, dirty tree refused, real run, markers,
//         run-status, judge exit 0, second run HELD, audit
//   MM-4  an infrastructure failure after the first scenario: the run stops, the judge reports the rest MISSING (INVALID), the exit code is 6 and never 1
// Skipped when the OR-1 engine pair, the development bundles or the recording are absent.
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import { pathToFileURL } from 'node:url';

const repo = path.resolve(import.meta.dirname, '../../../..');
const gateDir = path.join(repo, 'slam_refact/tools/gates/or1-chunk-replay-v4');
const tools = path.join(repo, 'slam_refact/web-chunk/tools');
const or1 = path.join(tools, 'or1/or1.mjs');
const PAIR = 'build/slam-refact-or1-20261020/pair.json';
const DEV = [['P1', 'build/slam-refact-20261016/fix/round2/bundles-seed/flip6-s2/pmr3a', 'rec-p1'], ['P2', 'build/slam-refact-20261016/fix/round2/bundles-seed/tum-room1-full/pmr3a', 'rec-p2']];
const ORIENT = 'build/slam-refact-full-20261019/orient/original/flip6-s2/v2-b-o90';
const available = fs.existsSync(path.join(repo, PAIR)) && DEV.every(([, src]) => fs.existsSync(path.join(repo, src, 'frames.tsv'))) && fs.existsSync(path.join(repo, ORIENT, 'tsv/stages.tsv'));
const mm = await import(pathToFileURL(path.join(gateDir, 'make-manifest.mjs')).href);
const { designationPins, sha256File } = await import(pathToFileURL(path.join(tools, 'or1/hold.mjs')).href);
const freeze = await import(pathToFileURL(path.join(tools, 'gate-common/freeze.mjs')).href);

const E = path.join(repo, 'build/slam-refact-g4-20261025', `e2e-${process.pid}`);
const rel = (p) => path.relative(repo, p);
const put = (p, text) => { fs.mkdirSync(path.dirname(p), { recursive: true }); fs.writeFileSync(p, text); return p; };
const cleanEnv = () => { const env = { ...process.env }; for (const k of Object.keys(env)) if (k.startsWith('PRECHECK_') || k.startsWith('SLAM_REFACT_OR1_V4') || k.startsWith('OR1_')) delete env[k]; return env; };

// ---- MM-1 ----
test('MM-1: the draft of the registered scenarios: counts, sizes, rotations, the v3 data unchanged, every F3 field PENDING-F3, no overwrite', { timeout: 300000 }, () => {
  const D = path.join(E, 'mm1');
  const prereg = path.join(D, 'prereg');
  put(path.join(prereg, 'or1-chunk-replay-gate.v4.md'), 'gate document (test copy)\n');
  const r = mm.buildV4Manifest({ preregDir: rel(prereg), scenariosFile: 'slam_refact/tools/gates/or1-chunk-replay-v4/scenarios.json', derivedOut: rel(path.join(D, 'derived')), pairFile: mm.PENDING,
    requireDoc: false, now: '2026-10-25T00:00:00+09:00', gitHead: 'x' });
  assert.equal(r.mode, 'draft');
  assert.match(path.basename(r.manifestFile), /\.manifest\.draft\.json$/);
  assert.equal(fs.existsSync(path.join(prereg, 'or1-chunk-replay-gate.v4.manifest.json')), false, 'a draft is not a manifest');
  const m = r.manifest;
  assert.equal(m.status, 'DRAFT-PENDING-F3');
  for (const k of ['snapshot', 'null_run', 'mutation', 'held', 'engine_provenance']) assert.equal(m[k], mm.PENDING, k);
  for (const k of ['harness', 'gate_common', 'held_device', 'tests', 'chunk', 'docs', 'pair', 'engine', 'native', 'runner', 'environment']) assert.equal(m.pinned[k], mm.PENDING, k);
  const got = m.registered_run.scenarios.map((s) => [s.name, s.family, s.expected.calls, s.expected.frames, s.capture.width, s.capture.height, s.capture.rotation]);
  assert.deepEqual(got, [
    ['G1-flip6-s3-lowtex-pmr3a', 'portrait', 381, 380, 480, 640, 0], ['G2-flip6-s3-lowtex-pmr4a', 'portrait', 381, 380, 480, 640, 0],
    ['G3-tum-room1-lowtex-pmr3b', 'portrait', 480, 479, 240, 320, 0], ['G4-tum-room1-lowtex-pmr4a', 'portrait', 480, 479, 240, 320, 0],
    ['O0-flip6-s2-orient-o0', 'rotation', 61, 60, 480, 640, 0], ['O90-flip6-s2-orient-o90', 'rotation', 61, 60, 640, 480, 90],
    ['O180-flip6-s2-orient-o180', 'rotation', 61, 60, 480, 640, 180], ['O270-flip6-s2-orient-o270', 'rotation', 61, 60, 640, 480, -90]]);
  assert.deepEqual(m.registered_run.scenarios.map((s) => s.floors.min_fresh_nonrepeated), [342, 342, 432, 432, 48, 48, 48, 48]);
  assert.deepEqual(m.registered_run.k_min, { rotationScenarios: 4, portraitRecords: 2 });
  assert.equal(m.independent_samples.n, 3);
  assert.equal(m.tolerance.value, 0); assert.equal(m.tolerance.determinism_class_gate, true);
  // the portrait scenarios are the v3 data: same source, derived bundle and atlas set
  const v3File = path.join(repo, 'build/slam-refact-full-20261019/preregistration/or1-chunk-replay-gate.v3.manifest.json');
  if (fs.existsSync(v3File)) {
    const v3 = Object.fromEntries(JSON.parse(fs.readFileSync(v3File, 'utf8')).pinned.gate_data.map((d) => [d.name, d]));
    for (const d of m.pinned.gate_data.filter((x) => x.family === 'portrait')) {
      assert.deepEqual(d.source_sha256, v3[d.name].source_sha256, d.name); assert.deepEqual(d.derived_sha256, v3[d.name].derived_sha256, d.name);
      assert.equal(d.atlas_set_sha256, v3[d.name].atlas_set_sha256, d.name);
    }
  }
  assert.ok(m.pinned.gate_data.filter((x) => x.family === 'rotation').every((d) => d.source_recording && d.frames_without_atlas_disabled.length === 0));
  assert.match(m.data_reuse.statement, /first 25 frames/);
  assert.throws(() => mm.buildV4Manifest({ preregDir: rel(prereg), scenariosFile: 'slam_refact/tools/gates/or1-chunk-replay-v4/scenarios.json', derivedOut: rel(path.join(D, 'derived2')), pairFile: mm.PENDING, requireDoc: false }), /refusing to overwrite/);
});

// ---- the synthetic registration used by MM-2..MM-4 ----
function register(id) {
  const D = path.join(E, id);
  const prereg = path.join(D, 'prereg');
  const gate = { gateId: id, version: 4, preregDir: prereg };
  const ENV = `SLAM_REFACT_OR1_V4_E2E_${id.slice(-1).toUpperCase()}_DESIGNATION_SHA256`;
  for (const [name, src] of DEV) {
    const lines = (f) => fs.readFileSync(path.join(repo, src, f), 'utf8').split('\n').filter(Boolean);
    const fl = lines('frames.tsv').slice(0, 13);
    const lastNs = Number(fl.at(-1).split('\t')[2]);
    put(path.join(D, 'srcs', name, 'frames.tsv'), `${fl.join('\n')}\n`);
    put(path.join(D, 'srcs', name, 'sensors.tsv'), `${lines('sensors.tsv').filter((l) => { const c = l.split('\t'); return c[0] === 'bootstrap' || Number(c[1]) <= lastNs; }).join('\n')}\n`);
    put(path.join(D, 'srcs', name, 'engine-config.tsv'), `${lines('engine-config.tsv').join('\n')}\n`);
  }
  const floors = { c4: null, min_fresh_nonrepeated: 5, min_distinct_rotations: 3 };
  const spec = { floors: { portrait: { ...floors, min_distinct_rotations: null }, rotation: floors }, k_min: { rotationScenarios: 1, portraitRecords: 2 },
    scenarios: [...DEV.map(([name, src, record]) => ({ name, family: 'portrait', record, kind: 'bundle', src: rel(path.join(D, 'srcs', name)) })),
      { name: 'R1', family: 'rotation', record: 'rec-r', kind: 'orient', src: ORIENT, frames: 12 }] };
  put(path.join(D, 'scenarios.json'), JSON.stringify(spec));
  put(path.join(prereg, `${id}.v4.md`), `# ${id} v4 (synthetic registration of development data)\n`);
  put(path.join(D, 'ev/selftest/selftest.json'), JSON.stringify({ datasets: { E: {} }, summary: { nullFailures: 0, passes: true, chunkMutantsRun: 1, chunkMutantsDetectedByAChunkGate: 1, equivalentMutantFalseAlarms: 0, rowMutationsDetected: 1, rowMutations: 1, outOfClaim: [] } }));
  put(path.join(D, 'ev/judge/summary.json'), JSON.stringify({ nullRunPassed: true, survivors: [], detected: 1, mutants: 1 }));
  put(path.join(D, 'ev/tool/summary.json'), JSON.stringify({ survivors: [], detected: 1, mutants: 1 }));
  const common = { preregDir: rel(prereg), scenariosFile: rel(path.join(D, 'scenarios.json')), derivedOut: rel(path.join(D, 'derived')), gateId: id, version: 4, requireDoc: false,
    tests: ['slam_refact/web-chunk/tests/or1.test.mjs'], runnerFiles: ['slam_refact/tools/gates/or1-chunk-replay-v4/run.sh', 'slam_refact/tools/gates/or1-chunk-replay-v4/judge.mjs'] };
  return { D, prereg, gate, ENV, spec, common, evidence: { selftestDir: rel(path.join(D, 'ev/selftest')), judgeMutantsDir: rel(path.join(D, 'ev/judge')), toolMutantsDir: rel(path.join(D, 'ev/tool')) } };
}
function reviewAndDesignate(R, manifestFile) {
  const review = put(path.join(R.prereg, 'review.md'), 'review (synthetic)\n');
  const designation = put(path.join(R.prereg, 'designation.json'), JSON.stringify({ gate: `${R.gate.gateId}.v4`, first_real_run_permitted: true, pinned: { ...designationPins(manifestFile), review_file_sha256: sha256File(review) } }));
  const m = JSON.parse(fs.readFileSync(manifestFile, 'utf8'));
  m.review = { reviewer: 'synthetic', at: new Date(Date.parse(m.created_at) + 1).toISOString(), file: 'review.md', sha256: sha256File(review), designation_file: 'designation.json', designation_file_sha256: sha256File(designation) };
  fs.writeFileSync(manifestFile, JSON.stringify(m, null, 1));
  return { designation, env: { [R.ENV]: sha256File(designation) } };
}
// a copy of run.sh whose only difference is the FREEZE_PATHS constant (the registered file's constant names the repository's freeze paths, and the working tree is dirty
// while the lane works); a probe directory under the gate directory stands in for them
function runShCopy(R, tag) {
  const probe = path.join(gateDir, `zz-freeze-probe-${process.pid}-${tag}`);
  const text = fs.readFileSync(path.join(gateDir, 'run.sh'), 'utf8');
  const m = text.match(/^FREEZE_PATHS="[^"]*"$/m);
  assert.ok(m, 'FREEZE_PATHS constant not found');
  const file = put(path.join(R.D, `run-${tag}.sh`), text.replace(m[0], `FREEZE_PATHS="${rel(probe)}"`));
  return { file, probe };
}
const stub = () => put(path.join(E, 'precheck-stub.sh'), '#!/usr/bin/env bash\necho "lane precheck stub: pass"\nexit 0\n');
const sh = (R, copy, args, env = {}) => spawnSync('bash', [copy.file, ...args], { cwd: repo, encoding: 'utf8', timeout: 900000, env: { ...cleanEnv(), OR1_GATE_ID: R.gate.gateId, OR1_GATE_VERSION: '4', OR1_DESIGNATION_ENV: R.ENV, OR1_PREREG_DIR: rel(R.prereg),
  OR1_ENGINE_DIR: 'build/slam-refact-or1-20261020/wasm/wasm', OR1_NATIVE_BIN: 'build/slam-refact-or1-20261020/native/harness/slam_refact_replay', OR1_OUT_ROOT: rel(path.join(R.D, 'out')), OR1_PRECHECK: stub(), ...env } });

// ---- MM-2 ----
test('MM-2: the final manifest needs the pair and the evidence; a self-test that did not pass, a missing pair file and a gate document that is not the registration document are refused', { skip: !available && 'dev data / pair absent', timeout: 300000 }, () => {
  const R = register('or1-v4-e2e-m');
  assert.throws(() => mm.buildV4Manifest({ ...R.common, pairFile: PAIR }), /--selftest, --judge-mutants and --tool-mutants are required/);
  put(path.join(R.D, 'ev/selftest/selftest.json'), JSON.stringify({ datasets: {}, summary: { passes: false, nullFailures: 0 } }));
  fs.rmSync(path.join(R.D, 'derived'), { recursive: true, force: true });
  assert.throws(() => mm.buildV4Manifest({ ...R.common, pairFile: PAIR, ...R.evidence }), /the self-test did not pass/);
  fs.rmSync(path.join(R.D, 'derived'), { recursive: true, force: true });
  assert.throws(() => mm.buildV4Manifest({ ...R.common, pairFile: 'build/nonexistent-pair.json', ...R.evidence, requireDoc: true }), /is not a byte copy of/);
});

// ---- MM-3 ----
test('MM-3: register, designate, precheck, run.sh (dry run, PRECHECK_*, dirty tree, real run), judge PASS, second run HELD, audit', { skip: !available && 'dev data / pair absent', timeout: 1800000 }, () => {
  const R = register('or1-v4-e2e-a');
  const { manifest, manifestFile } = mm.buildV4Manifest({ ...R.common, pairFile: PAIR, ...R.evidence });
  assert.equal(manifest.registered_run.scenarios.length, 3);
  assert.deepEqual(manifest.registered_run.scenarios.map((s) => [s.name, s.capture.rotation]), [['P1', 0], ['P2', 0], ['R1', 90]]);
  assert.equal(manifest.pinned.gate_data[2].frames, 12);
  assert.deepEqual(Object.keys(manifest.held.inputs), ['P1', 'P2', 'R1']);
  assert.equal(manifest.review && Object.keys(manifest.review).length, 0); assert.equal(manifest.first_real_run_at, null);
  const { env } = reviewAndDesignate(R, manifestFile);
  const copy = runShCopy(R, 'a');
  try {
    // dry run: every HELD check, no marker, no output tree; the log is read-only and summed
    const dry = sh(R, copy, ['--dry-run'], env);
    assert.equal(dry.status, 0, dry.stdout + dry.stderr);
    assert.match(dry.stdout, /registered scenarios:\n\s+P1 [\s\S]*P2 [\s\S]*R1 /);
    assert.equal((dry.stdout.match(/RELEASED \(precheck: no marker created/g) ?? []).length, 3);
    assert.match(dry.stdout, /dry run: stopping before the first real run/);
    assert.match(dry.stdout, /PRECHECK_\* variables: none/);
    assert.equal(fs.existsSync(path.join(R.D, 'out')), false); assert.equal(fs.readdirSync(R.prereg).filter((f) => f.endsWith('.marker')).length, 0);
    const dryLog = fs.readdirSync(`${R.D}/out.logs`).find((f) => f.endsWith('.log'));
    assert.equal(fs.statSync(path.join(`${R.D}/out.logs`, dryLog)).mode & 0o222, 0, 'the log is read-only');
    // M-4: the PRECHECK_* variables are recorded by a dry run and refused by a real run (no log, no marker)
    const rec = sh(R, copy, ['--dry-run'], { ...env, PRECHECK_MAX_DISK_PCT: '99' });
    assert.equal(rec.status, 0, rec.stdout);
    assert.match(rec.stdout, /PRECHECK_MAX_DISK_PCT=99/);
    const refused = sh(R, copy, [], { ...env, PRECHECK_MAX_DISK_PCT: '99' });
    assert.equal(refused.status, 2); assert.match(refused.stderr, /PRECHECK_\* is set in the environment/);
    assert.equal(fs.existsSync(path.join(R.D, 'out')), false);
    // without the designation variable: HELD (exit 3) by the first precheck
    const held = sh(R, copy, ['--dry-run'], { [R.ENV]: '' });
    assert.equal(held.status, 3); assert.match(held.stdout, /HELD or failed for P1; no marker was created/);
    // M-5: a dirty freeze path refuses a real run before any marker
    fs.mkdirSync(copy.probe, { recursive: true }); fs.writeFileSync(path.join(copy.probe, 'dirty.txt'), 'x\n');
    const dirty = sh(R, copy, [], env);
    assert.equal(dirty.status, 2, dirty.stdout); assert.match(dirty.stdout, /git status \(freeze paths .*\) is NOT empty/); assert.match(dirty.stdout, /a real run needs an empty freeze-path git status/);
    assert.equal(fs.readdirSync(R.prereg).filter((f) => f.endsWith('.marker')).length, 0);
    fs.rmSync(copy.probe, { recursive: true, force: true });
    // the real run (a refused real run leaves its log, and a gate log is never reused: the rerun names a new log directory)
    assert.ok(fs.existsSync(path.join(`${R.D}/out.logs`, 'run.sh.log')), 'the refused real run left its log');
    const real = sh(R, copy, [], { ...env, OR1_LOG_DIR: rel(path.join(R.D, 'real.logs')) });
    assert.equal(real.status, 0, real.stdout.slice(-3000) + real.stderr);
    assert.match(real.stdout, /verdict: PASS \(exit 0; PASS 3,/);
    const status = fs.readFileSync(`${R.D}/real.logs/run-status.tsv`, 'utf8').trim().split('\n').map((l) => l.split('\t'));
    assert.deepEqual(status, [['scenario', 'exit_code', 'marker', 'report'], ['P1', '0', 'yes', 'yes'], ['P2', '0', 'yes', 'yes'], ['R1', '0', 'yes', 'yes']]);
    for (const n of ['P1', 'P2', 'R1']) {
      assert.ok(fs.existsSync(path.join(R.D, 'out', n, 'report.json'))); assert.ok(fs.existsSync(path.join(R.prereg, `${R.gate.gateId}.v4.first-run.${n}.marker`)));
    }
    const realLog = path.join(`${R.D}/real.logs`, 'run.sh.log');
    assert.equal(fs.statSync(realLog).mode & 0o222, 0); assert.ok(fs.existsSync(`${realLog}.sha256`));
    // the second real run is held by the markers at the first precheck
    const again = sh(R, copy, [], { ...env, OR1_LOG_DIR: rel(path.join(R.D, 'again.logs')) });
    assert.equal(again.status, 3); assert.match(again.stdout, /HELD or failed for P1/);
    // the judge alone, on the frozen outputs: the same verdict, and a judge crash is not a FAIL
    const j = spawnSync(process.execPath, [path.join(gateDir, 'judge.mjs'), '--manifest', manifestFile, '--out-root', path.join(R.D, 'out')], { encoding: 'utf8' });
    assert.equal(j.status, 0, j.stdout.slice(-2000)); assert.match(j.stdout, /verdict: PASS/);
    assert.equal(spawnSync(process.execPath, [path.join(gateDir, 'judge.mjs'), '--manifest', path.join(R.D, 'nonexistent.json'), '--out-root', path.join(R.D, 'out')], { encoding: 'utf8' }).status, 5);
    // audit after the coordinator records first_real_run_at
    const m2 = JSON.parse(fs.readFileSync(manifestFile, 'utf8')); m2.first_real_run_at = new Date().toISOString(); fs.writeFileSync(manifestFile, JSON.stringify(m2, null, 1));
    const audit = spawnSync(process.execPath, [or1, 'audit', '--prereg-dir', rel(R.prereg), '--gate-id', R.gate.gateId, '--gate-version', '4'], { cwd: repo, encoding: 'utf8' });
    assert.equal(audit.status, 0, audit.stdout + audit.stderr);
    assert.match(audit.stdout, /P1: AUDIT OK[\s\S]*P2: AUDIT OK[\s\S]*R1: AUDIT OK/);
  } finally { fs.rmSync(copy.probe, { recursive: true, force: true }); }
});

// ---- MM-4 ----
test('MM-4: a scenario whose process fails after the first one: the run stops, run-status says so, the judge reports the rest MISSING, the exit code is 6 (INVALID), never 1', { skip: !available && 'dev data / pair absent', timeout: 1800000 }, () => {
  const R = register('or1-v4-e2e-b');
  const { manifestFile } = mm.buildV4Manifest({ ...R.common, pairFile: PAIR, ...R.evidence });
  const { env } = reviewAndDesignate(R, manifestFile);
  const copy = runShCopy(R, 'b');
  try {
    fs.mkdirSync(path.join(R.D, 'out', 'P2'), { recursive: true });  // `or1.mjs run` refuses an existing output directory: P1 runs, P2 fails (before its marker)
    const real = sh(R, copy, [], env);
    assert.equal(real.status, 6, real.stdout.slice(-3000));
    assert.match(real.stdout, /infrastructure failure at P2 \(exit 1, marker no, report no\)/);
    assert.match(real.stdout, /verdict: INVALID/);
    assert.match(real.stdout, /gate verdict exit code 4/);
    const status = fs.readFileSync(`${R.D}/out.logs/run-status.tsv`, 'utf8').trim().split('\n').map((l) => l.split('\t'));
    assert.deepEqual(status, [['scenario', 'exit_code', 'marker', 'report'], ['P1', '0', 'yes', 'yes'], ['P2', '1', 'no', 'no']]);
    assert.equal(fs.existsSync(path.join(R.prereg, `${R.gate.gateId}.v4.first-run.R1.marker`)), false, 'the scenario after the failure was not started');
    const j = spawnSync(process.execPath, [path.join(gateDir, 'judge.mjs'), '--manifest', manifestFile, '--out-root', path.join(R.D, 'out')], { encoding: 'utf8' });
    assert.equal(j.status, 4); assert.match(j.stdout, /"missing": 2/);
  } finally { fs.rmSync(copy.probe, { recursive: true, force: true }); }
});

test.after(() => { freeze.makeWritable(E); fs.rmSync(E, { recursive: true, force: true }); });
