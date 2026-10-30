// node --test slam_refact/web-chunk/tests/or1-e2e.test.mjs
// End to end through the real command line (tools/or1/or1.mjs) with a SYNTHETIC registration of DEVELOPMENT data: a gate of the made-up id
// `or1-e2e-gate` (version 1) is registered by make-gate-manifest on two short dev bundles (never the registered gate data), reviewed and designated by this
// test, then precheck -> run (gate kind, three arms, three repetitions, the real harness, the real chunk, the pinned pair engine and native binary) ->
// second run refused -> verdict -> audit. It shows that the pieces agree (hold, registration tool, outputs, freeze, run log); it is not a gate result and
// it does not touch the real gate's marker, manifest or environment variable. Skipped when the OR-1 engine pair or the development bundles are absent.
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import { pathToFileURL } from 'node:url';

const repo = path.resolve(import.meta.dirname, '../../..');
const tools = path.join(repo, 'slam_refact/web-chunk/tools');
const or1 = path.join(tools, 'or1/or1.mjs');
const PAIR = 'build/slam-refact-or1-20261020/pair.json';
const DEV = [['E1', 'build/slam-refact-20261016/fix/round2/bundles-seed/flip6-s2/pmr3a', 30, 'R1'], ['E2', 'build/slam-refact-20261016/fix/round2/bundles-seed/tum-room1-full/pmr3a', 90, 'R2']];
const available = fs.existsSync(path.join(repo, PAIR)) && DEV.every(([, src]) => fs.existsSync(path.join(repo, src, 'frames.tsv')));
const { buildManifest } = await import(pathToFileURL(path.join(tools, 'or1/make-gate-manifest.mjs')).href);
const { designationPins, sha256File } = await import(pathToFileURL(path.join(tools, 'or1/hold.mjs')).href);
const freeze = await import(pathToFileURL(path.join(tools, 'gate-common/freeze.mjs')).href);

const E = path.join(repo, 'build/slam-refact-full-c1-chunk', `e2e-${process.pid}`);
const rel = (p) => path.relative(repo, p);
const ENV = 'SLAM_REFACT_OR1_E2E_DESIGNATION_SHA256';
const gateArgv = ['--prereg-dir', rel(path.join(E, 'prereg')), '--gate-id', 'or1-e2e-gate', '--gate-version', '1', '--designation-env', ENV];
const cli = (args, env = {}) => spawnSync(process.execPath, [or1, ...args], { cwd: repo, encoding: 'utf8', env: { ...process.env, ...env } });
const bundleArgs = (name, extra = []) => ['--bundle', `${rel(E)}/derived/${name}/bundle`, '--engine-dir', 'build/slam-refact-or1-20261020/wasm/wasm',
  '--native-bin', 'build/slam-refact-or1-20261020/native/harness/slam_refact_replay', '--arms', 'WCN', '--reps', '3', '--kind', 'gate', ...extra];

test('OR1E-1: register, designate, precheck, run each bundle once, refuse the second run, summarise the gate, audit', { skip: !available && 'the OR-1 pair or the development bundles are not in this tree', timeout: 600000 }, () => {
  fs.rmSync(E, { recursive: true, force: true });
  const put = (p, text) => { fs.mkdirSync(path.dirname(p), { recursive: true }); fs.writeFileSync(p, text); return p; };
  // short development sources: the first N frames of the development bundles (never the registered gate data)
  for (const [name, src, frames] of DEV) {
    const lines = (f) => fs.readFileSync(path.join(repo, src, f), 'utf8').split('\n').filter(Boolean);
    const fl = lines('frames.tsv').slice(0, frames + 1);
    const lastNs = Number(fl.at(-1).split('\t')[2]);
    put(path.join(E, 'srcs', name, 'frames.tsv'), `${fl.join('\n')}\n`);
    put(path.join(E, 'srcs', name, 'sensors.tsv'), `${lines('sensors.tsv').filter((l) => { const c = l.split('\t'); return c[0] === 'bootstrap' || Number(c[1]) <= lastNs; }).join('\n')}\n`);
    put(path.join(E, 'srcs', name, 'engine-config.tsv'), `${lines('engine-config.tsv').join('\n')}\n`);
  }
  const prereg = path.join(E, 'prereg');
  const gate = { gateId: 'or1-e2e-gate', version: 1, preregDir: prereg, designationEnv: ENV };
  put(path.join(prereg, 'or1-e2e-gate.v1.md'), '# or1-e2e-gate v1 (synthetic registration of development data)\n');
  put(path.join(E, 'ev/selftest/selftest.json'), JSON.stringify({ datasets: { E: {} }, summary: { nullFailures: 0, passes: true, chunkMutantsDetectedByAChunkGate: 1, chunkMutantsRun: 1, equivalentMutantFalseAlarms: 0, rowMutationsDetected: 1, rowMutations: 1 } }));
  put(path.join(E, 'ev/tool/summary.json'), JSON.stringify({ survivors: [], detected: 1, mutants: 1 }));
  const { manifestFile } = buildManifest({ root: repo, gate, gateDataSpec: DEV.map(([name, , , record]) => ({ name, src: rel(path.join(E, 'srcs', name)), record })), pairFile: PAIR,
    derivedOut: `${rel(E)}/derived`, selftestDir: rel(path.join(E, 'ev/selftest')), toolMutantsDir: rel(path.join(E, 'ev/tool')), classification: 'e2e', tests: ['or1.test.mjs'] });
  const review = put(path.join(prereg, 'review.md'), 'review (synthetic)\n');
  const designation = put(path.join(prereg, 'designation.json'), JSON.stringify({ gate: 'or1-e2e-gate.v1', first_real_run_permitted: true,
    pinned: { ...designationPins(manifestFile), review_file_sha256: sha256File(review) } }));
  const m = JSON.parse(fs.readFileSync(manifestFile, 'utf8'));
  m.review = { reviewer: 'synthetic', at: new Date(Date.parse(m.created_at) + 1).toISOString(), file: 'review.md', sha256: sha256File(review), designation_file: 'designation.json', designation_file_sha256: sha256File(designation) };
  fs.writeFileSync(manifestFile, JSON.stringify(m, null, 1));
  const env = { [ENV]: sha256File(designation) };

  // no designation in the environment: held before anything is created
  const noenv = cli(['run', ...gateArgv, ...bundleArgs('E1'), '--out', `${rel(E)}/out/E1`, '--run-log', `${rel(E)}/logs/E1.log`], { [ENV]: '' });
  assert.equal(noenv.status, 3, noenv.stderr);
  assert.match(noenv.stderr, /^HELD: ENV_MISSING:/);
  assert.equal(fs.existsSync(path.join(E, 'out')), false, 'nothing created');
  assert.equal(fs.existsSync(path.join(prereg, 'or1-e2e-gate.v1.first-run.E1.marker')), false);
  assert.match(fs.readFileSync(path.join(E, 'logs/E1.log'), 'utf8'), /"event":"held"/, 'the refusal is in the run log');

  // dry run: every HELD check, no marker
  const pre = cli(['precheck', ...gateArgv, ...bundleArgs('E1')], env);
  assert.equal(pre.status, 0, pre.stderr);
  assert.match(pre.stdout, /RELEASED \(precheck: no marker created/);
  assert.equal(fs.existsSync(path.join(prereg, 'or1-e2e-gate.v1.first-run.E1.marker')), false);
  // --native-bundle is refused for a gate run (V-3), --chunk-dir too
  assert.match(cli(['precheck', ...gateArgv, ...bundleArgs('E1', ['--native-bundle', `${rel(E)}/derived/E1/bundle`])], env).stderr, /^HELD: args: --native-bundle/);

  // the registered command's dry run (tools/gates/or1-chunk-replay-v3/run.sh --dry-run) against this synthetic gate: prechecks only, no marker, no output tree
  const stub = put(path.join(E, 'precheck-stub.sh'), '#!/usr/bin/env bash\necho "lane precheck stub: pass"\nexit 0\n');
  const dry = spawnSync('bash', [path.join(repo, 'slam_refact/tools/gates/or1-chunk-replay-v3/run.sh'), '--dry-run'], { cwd: repo, encoding: 'utf8', env: { ...process.env, ...env,
    OR1_GATE_ID: 'or1-e2e-gate', OR1_GATE_VERSION: '1', OR1_DESIGNATION_ENV: ENV, OR1_PREREG_DIR: rel(prereg), OR1_ENGINE_DIR: 'build/slam-refact-or1-20261020/wasm/wasm',
    OR1_NATIVE_BIN: 'build/slam-refact-or1-20261020/native/harness/slam_refact_replay', OR1_OUT_ROOT: `${rel(E)}/out-sh`, OR1_PRECHECK: stub } });
  assert.equal(dry.status, 0, dry.stdout + dry.stderr);
  assert.match(dry.stdout, /registered bundles:\n\s+E1 /);
  assert.equal((dry.stdout.match(/RELEASED \(precheck: no marker created/g) ?? []).length, 2, 'both bundles prechecked');
  assert.match(dry.stdout, /dry run: stopping before the first real run/);
  assert.match(dry.stdout, /## EXIT 0/);
  assert.equal(fs.existsSync(path.join(E, 'out-sh')), false, 'no output tree');
  assert.equal(fs.readdirSync(prereg).filter((f) => f.endsWith('.marker')).length, 0, 'no marker');
  const dryLog = fs.readdirSync(`${E}/out-sh.logs`).find((f) => f.endsWith('.log'));
  assert.equal(fs.statSync(path.join(`${E}/out-sh.logs`, dryLog)).mode & 0o222, 0, 'the run.sh log is read-only');
  assert.ok(fs.existsSync(path.join(`${E}/out-sh.logs`, `${dryLog}.sha256`)));
  // without the designation variable the dry run is held by the first precheck
  const heldDry = spawnSync('bash', [path.join(repo, 'slam_refact/tools/gates/or1-chunk-replay-v3/run.sh'), '--dry-run'], { cwd: repo, encoding: 'utf8', env: { ...process.env, [ENV]: '',
    OR1_GATE_ID: 'or1-e2e-gate', OR1_GATE_VERSION: '1', OR1_DESIGNATION_ENV: ENV, OR1_PREREG_DIR: rel(prereg), OR1_ENGINE_DIR: 'build/slam-refact-or1-20261020/wasm/wasm',
    OR1_NATIVE_BIN: 'build/slam-refact-or1-20261020/native/harness/slam_refact_replay', OR1_OUT_ROOT: `${rel(E)}/out-sh2`, OR1_PRECHECK: stub } });
  assert.equal(heldDry.status, 3);
  assert.match(heldDry.stdout, /HELD or failed for E1; no marker was created/);

  const reports = [];
  for (const [name] of DEV) {
    const out = `${rel(E)}/out/${name}`, log = `${rel(E)}/logs/${name}-run.log`;
    const r = cli(['run', ...gateArgv, ...bundleArgs(name), '--out', out, '--run-log', log], env);
    assert.equal(r.status, 0, r.stderr + r.stdout);
    reports.push(path.join(out, 'report.json'));
    const dir = path.join(repo, out);
    const report = JSON.parse(fs.readFileSync(path.join(dir, 'report.json'), 'utf8'));
    // F-1: the raw inputs and answers are on disk with their sha256
    for (const file of ['inputs-W-0.json', 'inputs-C-2.json', 'answers-C-0.json', 'rows-N-1.json']) assert.ok(fs.existsSync(path.join(dir, file)), file);
    const all = { ...report.files.rows, ...report.files.inputs, ...report.files.answers };
    assert.equal(Object.keys(all).length, 3 * 3 + 2 * 3 + 3);  // rows W C N x3, inputs W C x3, answers C x3
    for (const [file, sha] of Object.entries(all)) assert.equal(sha256File(path.join(dir, file)), sha, file);
    // V-8: the report names the gate file of THIS version and the device
    assert.equal(report.meta.gate.version, 1);
    assert.equal(report.meta.gate.gateMd.sha256, sha256File(path.join(prereg, 'or1-e2e-gate.v1.md')));
    assert.equal(report.meta.gate.designation.sha256, sha256File(designation));
    assert.equal(report.meta.gate.review.sha256, sha256File(review));
    assert.equal(report.meta.gate.datum, name);
    assert.deepEqual(report.meta.gate.dataReuse, { observed_in_previous_version: false });
    assert.equal(report.meta.gate.designationEnvName, ENV);
    // the verdict the review made conditions of
    assert.ok(report.gates.C4 && typeof report.gates.C4.r === 'number');
    for (const key of ['IN-CW', 'OUT-CW', 'ANS', 'REP', 'C7', 'C8', 'NW', 'CN']) assert.equal(report.gates.verdict[key], true, `${name} ${key}: ${JSON.stringify(report.gates.verdict)}`);
    assert.equal(report.gates.C6.pass, true);
    assert.ok(Array.isArray(report.gates.glueConfig.diffPaths));
    // F-10: SHA256SUMS, registered copies, read-only
    const sums = fs.readFileSync(path.join(dir, 'SHA256SUMS'), 'utf8').trim().split('\n');
    assert.ok(sums.length >= 30);
    for (const l of sums) { const [sha, f] = l.split('  '); assert.equal(sha256File(path.join(dir, f)), sha, f); }
    for (const f of ['or1-e2e-gate.v1.md', 'or1-e2e-gate.v1.manifest.json', 'designation.json', 'review.md', `or1-e2e-gate.v1.first-run.${name}.marker`]) assert.ok(fs.existsSync(path.join(dir, 'registered', f)), f);
    assert.equal(fs.statSync(path.join(dir, 'report.json')).mode & 0o222, 0);
    assert.equal(fs.statSync(dir).mode & 0o222, 0);
    assert.equal(fs.statSync(path.join(prereg, 'designation.json')).mode & 0o222, 0, 'the registered originals are read-only after the run');
    assert.equal(fs.statSync(path.join(prereg, `or1-e2e-gate.v1.first-run.${name}.marker`)).mode & 0o222, 0);
    // F-9: the log outside the output directory
    const events = fs.readFileSync(path.join(repo, log), 'utf8').trim().split('\n').map((l) => JSON.parse(l));
    assert.deepEqual(events.map((e) => e.event), ['start', 'hold-checks', 'first-run-claimed', 'finished', 'end']);
    assert.ok(events[0].env[ENV].set && !JSON.stringify(events[0]).includes(env[ENV]) || events[0].env[ENV].valueSha256 !== env[ENV], 'the value of the variable is hashed, not written');
    assert.ok(events[1].lines.some((l) => l.includes('designation_file')) && events[1].lines.some((l) => l.includes('pin sr_held.mjs')));
    assert.equal(events.at(-1).status, 'OK');
  }
  // a second run of the same bundle: held (marker), nothing new
  const again = cli(['run', ...gateArgv, ...bundleArgs('E1'), '--out', `${rel(E)}/out/E1-again`, '--run-log', `${rel(E)}/logs/E1-again.log`], env);
  assert.equal(again.status, 3);
  assert.match(again.stderr, /^HELD: first-run:/);
  assert.equal(fs.existsSync(path.join(E, 'out/E1-again')), false);

  // the gate over its bundles
  const v = cli(['verdict', '--reports', reports.join(','), '--manifest', rel(manifestFile)], env);
  const agg = JSON.parse(v.stdout);
  assert.deepEqual(agg.fails, []);
  assert.ok(['PASS', 'INSUFFICIENT'].includes(agg.verdict), v.stdout);
  assert.equal(v.status, agg.verdict === 'PASS' ? 0 : 4);
  assert.deepEqual(Object.keys(agg.records).sort(), ['R1', 'R2']);

  // audit: needs first_real_run_at (the coordinator records it after the runs)
  assert.equal(cli(['audit', '--prereg-dir', gateArgv[1], '--gate-id', 'or1-e2e-gate', '--gate-version', '1']).status, 3);
  const m2 = JSON.parse(fs.readFileSync(manifestFile, 'utf8'));
  m2.first_real_run_at = new Date().toISOString();
  fs.writeFileSync(manifestFile, JSON.stringify(m2, null, 1));
  const audit = cli(['audit', '--prereg-dir', gateArgv[1], '--gate-id', 'or1-e2e-gate', '--gate-version', '1']);
  assert.equal(audit.status, 0, audit.stdout + audit.stderr);
  assert.match(audit.stdout, /E1: AUDIT OK[\s\S]*E2: AUDIT OK/);
});

test.after(() => { freeze.makeWritable(E); fs.rmSync(E, { recursive: true, force: true }); });
