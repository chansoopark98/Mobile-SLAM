// Mutation check of the OR-1 harness tests (tests/or1.test.mjs, tests/or1-hold.test.mjs, tests/or1-v3.test.mjs, tests/chk1-engine-time.test.mjs): every mutant
// is a modified COPY of slam_refact/web-chunk/tools/{or1,gate-common} (with the shared device tools/prereg/sr_held.mjs) under build/, in the layout of the
// repository so that the relative imports resolve; the repo files are never touched. The tests must pass on the unmodified copy (null-run) and fail on every
// mutant. The mutated device itself (sr_held.mjs) has its own mutation run (tools/prereg/run_sr_held_mutants.py).
//   node slam_refact/web-chunk/tests/run-or1-mutants.mjs --out build/<lane>/or1-tool-mutants [--only ID,ID]
// Not part of the gate pins' execution: it is a development check whose summary the registration tool reads (make-gate-manifest --tool-mutants <dir>).
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { parseArgs } from 'node:util';

const repo = path.resolve(import.meta.dirname, '../../..');
const { values } = parseArgs({ options: { out: { type: 'string' }, only: { type: 'string' }, 'check-patterns': { type: 'boolean', default: false } } });
const out = path.resolve(repo, values.out ?? '');
if (!values['check-patterns'] && (!values.out || !out.startsWith(path.join(repo, 'build') + path.sep))) throw new Error('--out must be a path under build/');
if (!values['check-patterns'] && fs.existsSync(out)) throw new Error(`refusing to overwrite ${out}`);
const toolsSrc = path.join(repo, 'slam_refact/web-chunk/tools');
const srHeld = path.join(repo, 'slam_refact/tools/prereg/sr_held.mjs');
const tests = ['or1.test.mjs', 'or1-hold.test.mjs', 'or1-v3.test.mjs', 'chk1-engine-time.test.mjs'].map((f) => path.join(repo, 'slam_refact/web-chunk/tests', f));

// [id, file (relative to the copy of tools/or1), from, to]: `from` must occur exactly once
const M = [
  // hold.mjs (the OR-1 extras around the shared device)
  ['H1-kind-not-checked', 'hold.mjs', "if (values.kind !== 'gate') throw", 'if (false) throw'],
  ['H2-chunk-copy-allowed', 'hold.mjs', 'if (values.chunkDir && path.resolve', 'if (false && path.resolve'],
  ['H3-native-bundle-allowed', 'hold.mjs', 'if (values.nativeBundle) throw held', 'if (false) throw held'],
  ['H4-engine-dir-not-checked', 'hold.mjs', "if (path.resolve(root, values.engineDir ?? '') !== path.resolve(root, pinned.engine?.dir ?? '/nonexistent')) throw", 'if (false) throw'],
  ['H5-native-bin-not-checked', 'hold.mjs', "if (values.nativeBin && path.resolve(root, values.nativeBin) !== path.resolve(root, Object.keys(pinned.native ?? {})[0] ?? '/nonexistent')) throw", 'if (false) throw'],
  ['H6-arms-not-checked', 'hold.mjs', "if ([...(values.arms ?? '')].sort().join('') !== [...(reg.arms ?? 'CNW')].sort().join('')) throw", 'if (false) throw'],
  ['H7-repetitions-not-checked', 'hold.mjs', 'if (!(Number(values.reps) >= (reg.repetitions ?? 3))) throw', 'if (false) throw'],
  ['H8-source-bundle-not-compared', 'hold.mjs', 'if (JSON.stringify(sortKeys(datum.source_sha256)) !== JSON.stringify(sortKeys(derivedMeta.sourceSha256))) throw', 'if (false) throw'],
  ['H9-derived-path-not-compared', 'hold.mjs', "if (bundleDir !== path.resolve(root, datum.derived_dir ?? '/nonexistent')) throw", 'if (false) throw'],
  ['H10-pinned-set-vs-directory-not-compared', 'hold.mjs', "if (JSON.stringify(live) !== JSON.stringify(want)) throw held('pins'", "if (false) throw held('pins'"],
  ['H11-node-version-not-compared', 'hold.mjs', 'if (pinned.environment?.node !== process.version) throw', 'if (false) throw'],
  ['H12-held-pins-vs-pinned-not-compared', 'hold.mjs', 'if (JSON.stringify(sortKeys(flat)) !== JSON.stringify(sortKeys(manifest.held?.pins ?? null))) throw', 'if (false) throw'],
  ['H13-atlas-set-not-compared', 'hold.mjs', 'if (values.atlasSetSha256 !== datum.atlas_set_sha256) throw', 'if (false) throw'],
  ['H14-marker-not-prechecked', 'hold.mjs', "if (fs.existsSync(marker)) throw held('first-run'", "if (false) throw held('first-run'"],
  ['H15-env-name-fixed-in-code', 'hold.mjs', 'env: env[gate.designationEnv]', "env: env['SLAM_REFACT_OR1_V1_DESIGNATION_SHA256']"],
  ['H16-node-executable-not-presented', 'hold.mjs', "if (pinned?.environment?.node_sha256) files.push([process.execPath, pinned.environment.node_sha256]);", ''],
  ['H17-gate-common-directory-not-listed', 'hold.mjs', "['gate_common', gateCommonDir, /\\.mjs$/], ", ''],
  ['H18-sealed-data-not-refused', 'hold.mjs', 'refuseSealedData(gate, path.resolve(root, bundleDir));', ''],
  ['H19-source-sha-not-refused-in-dev-runs', 'hold.mjs', 'if (JSON.stringify(sortKeys(d.source_sha256)) === JSON.stringify(sortKeys(derivedMeta.sourceSha256))) {', 'if (false) {'],
  ['H20-claim-skips-the-device', 'hold.mjs', 'const outcome = claim(hold.rq);', 'const outcome = hold.outcome;'],
  ['H21-device-pin-not-presented', 'hold.mjs', "for (const [name, sha] of Object.entries(pinned?.held_device ?? {})) files.push([name === 'sr_held.mjs' ? SR_HELD_FILE : path.join(path.dirname(SR_HELD_FILE), name), sha]);", ''],
  ['H22-engine-files-not-presented', 'hold.mjs', "files.push([path.join(root, section === 'engine' ? path.join(entries.dir, rel) : rel), entries[rel]]);", ''],
  // gate-common
  ['G1-gate-version-unchecked', '../gate-common/held.mjs', 'if (!Number.isInteger(version) || version < 1 ||', 'if (false ||'],
  ['G2-env-name-unchecked', '../gate-common/held.mjs', "if (!/^[A-Z][A-Z0-9_]{2,}$/.test(values['designation-env'])) throw", 'if (false) throw'],
  ['G3-missing-gate-argument-defaulted', '../gate-common/held.mjs', "if (missing.length) throw new Error", 'if (false) throw new Error'],
  ['G4-refusal-code-dropped', '../gate-common/held.mjs', 'new Error(`HELD: ${outcome.code}: ${', 'new Error(`HELD: x: ${'],
  ['G5-release-check-ignored', '../gate-common/held.mjs', 'const o = sr.checkRelease(rq);\n  if (!o.ok) throw heldError(o);', 'const o = sr.checkRelease(rq);'],
  ['G6-pin-base-name-collision-allowed', '../gate-common/held.mjs', 'if (base in out) throw new Error', 'if (false) throw new Error'],
  ['L1-run-log-overwritable', '../gate-common/runlog.mjs', "{ flag: 'wx', mode: 0o644 }", "{ flag: 'w', mode: 0o644 }"],
  ['L2-run-log-inside-output-allowed', '../gate-common/runlog.mjs', "if (!rel.startsWith('..') && !path.isAbsolute(rel)) throw", 'if (false) throw'],
  ['L3-env-value-recorded', '../gate-common/runlog.mjs', "valueSha256: createHash('sha256').update(String(env[k] ?? '')).digest('hex')", 'valueSha256: String(env[k])'],
  ['L4-run-log-stays-writable', '../gate-common/runlog.mjs', 'fs.chmodSync(this.file, 0o444);', ''],
  ['F1-sums-overwritable', '../gate-common/freeze.mjs', "fs.writeFileSync(path.join(dir, name), text, { flag: 'wx' });", "fs.writeFileSync(path.join(dir, name), text, { flag: 'w' });"],
  ['F2-files-stay-writable', '../gate-common/freeze.mjs', 'fs.chmodSync(p, 0o444);', 'fs.chmodSync(p, 0o644);'],
  ['F3-directories-stay-writable', '../gate-common/freeze.mjs', 'fs.chmodSync(d, 0o555);', ''],
  ['F4-copy-overwrites', '../gate-common/freeze.mjs', 'fs.copyFileSync(f, dest, fs.constants.COPYFILE_EXCL);', 'fs.copyFileSync(f, dest);'],
  ['F5-sums-skip-subdirectories', '../gate-common/freeze.mjs', 'if (st.isDirectory()) out.push(...listFiles(dir, r));', 'if (st.isDirectory()) continue;'],
  // run.mjs (C4, C6-C9, the gate)
  ['R1-degenerate-by-r-only', 'run.mjs', 'degenerate: r < C4_R_MIN || k < C4_K_MIN', 'degenerate: r < C4_R_MIN'],
  ['R2-floor-r-changed', 'run.mjs', 'export const C4_R_MIN = 0.10;', 'export const C4_R_MIN = 0.05;'],
  ['R3-repeated-rows-used', 'run.mjs', 'const used = rows.filter((r) => r.fresh && !r.raw.repeated);', 'const used = rows.filter((r) => r.fresh);'],
  ['R4-initializing-counted-as-limited', 'run.mjs', " && r.raw.reason !== 'INITIALIZING').length;", ').length;'],
  ['R5-null-answers-unbounded', 'run.mjs', 'if (!(ans.nullAnswers <= 1))', 'if (false)'],
  ['R6-compared-vs-frames-unchecked', 'run.mjs', "if (g['OUT-CW'] && ans.compared !== g['OUT-CW'].frames)", 'if (false)'],
  ['R7-chunk-errors-unchecked', 'run.mjs', 'if (g.chunkErrors.length) reasons.push', 'if (false) reasons.push'],
  ['R8-clock-loss-unchecked', 'run.mjs', 'pass: g.clock.clockLost === 0 && g.clock.videoLost === 0', 'pass: true'],
  ['R9-call-count-unchecked', 'run.mjs', "if (g['IN-CW'].countA !== expected.calls || g['IN-CW'].countB !== expected.calls)", 'if (false)'],
  ['R10-ans-verdict-without-c6', 'run.mjs', 'ANS: g.C6 ? g.C6.pass : g.ANS?.identical', 'ANS: g.ANS?.identical'],
  ['R11-degenerate-bundle-makes-record-valid', 'run.mjs', 'if (b.degenerate === false) records[b.record].valid = true;', 'records[b.record].valid = true;'],
  ['R12-one-valid-record-enough', 'run.mjs', 'validRecords < 2 ?', 'validRecords < 1 ?'],
  ['R13-c8-not-graded', 'run.mjs', "const graded = [v['IN-CW'], v['OUT-CW'], v.ANS, v.REP, v.C7, v.C8];", "const graded = [v['IN-CW'], v['OUT-CW'], v.ANS, v.REP, v.C7];"],
  ['R14-expected-calls-without-bootstrap', 'run.mjs', 'return { calls: 1 + processed, frames: processed };', 'return { calls: processed, frames: processed };'],
  ['R15-mismatch-in-degenerate-bundle-excused', 'run.mjs', 'if (pass === false) fails.push', 'if (pass === false && !g.C4?.degenerate) fails.push'],
  ['R16-nw-in-the-gate', 'run.mjs', "const graded = [v['IN-CW'], v['OUT-CW'], v.ANS, v.REP, v.C7, v.C8];", "const graded = [v['IN-CW'], v['OUT-CW'], v.ANS, v.REP, v.C7, v.C8, v.NW];"],
  // outputs.mjs
  ['O1-inputs-not-written', 'outputs.mjs', 'if (r.frames) {', 'if (false) {'],
  ['O2-answers-not-written', 'outputs.mjs', 'if (r.answers) files.answers', 'if (false) files.answers'],
  ['O3-canonical-form-not-stored', 'outputs.mjs', 'canonical: comparableFrame(f)', 'canonical: null'],
  ['O4-report-overwritable', 'outputs.mjs', "`${JSON.stringify(report, null, 1)}\\n`, { flag: 'wx' }", "`${JSON.stringify(report, null, 1)}\\n`, { flag: 'w' }"],
  ['O5-rows-sha-is-the-file-sha', 'outputs.mjs', 'sha256(JSON.stringify(r.rows))', "sha256(`${JSON.stringify(r.rows)}\\n`)"],
  ['O6-files-sha-not-recorded', 'outputs.mjs', 'const fileSha = (file) => sha256(fs.readFileSync(file));', "const fileSha = () => 'x';"],
  // make-gate-manifest.mjs
  ['M1-held-pins-empty', 'make-gate-manifest.mjs', 'pins: heldPins,', 'pins: {},'],
  ['M2-review-not-empty', 'make-gate-manifest.mjs', 'review: {}, first_real_run_at: null,', 'review: null, first_real_run_at: null,'],
  ['M3-mutation-detected-unchecked', 'make-gate-manifest.mjs', 'detected: selftest.summary.passes === true && tool.survivors.length === 0,', 'detected: true,'],
  ['M4-snapshot-sums-wrong-path', 'make-gate-manifest.mjs', "sums.push(`${sha256File(gateMd)}  ${base}.md\\n`);", 'sums.push(`${sha256File(gateMd)}  ${path.relative(root, gateMd)}\\n`);'],
  ['M6-data-reuse-not-disclosed', 'make-gate-manifest.mjs', 'const dataReuse = supersedes && supersedes.first_real_run_at_in_previous', 'const dataReuse = false && supersedes && supersedes.first_real_run_at_in_previous'],
  ['M5-gate-data-record-dropped', 'make-gate-manifest.mjs', 'return { name: g.name, record: g.record,', 'return { name: g.name,'],
  // compare.mjs
  ['K1-repeated-flag-not-compared', 'compare.mjs', "for (const k of ['status', 'reason', 'repeated', 'sourceFrameIndex', 'worldPoints'])", "for (const k of ['status', 'reason', 'sourceFrameIndex', 'worldPoints'])"],
  ['K2-bits-compared-as-numbers', 'compare.mjs', 'const sameBits = (a, b) => f64hex(a) === f64hex(b);', 'const sameBits = (a, b) => a === b;'],
  ['K3-full-pose-ignored', 'compare.mjs', 'if (full && a.pose && b.pose)', 'if (false && a.pose && b.pose)'],
  ['K4-rotation-not-compared', 'compare.mjs', "if (!arrBits(ra.rotationWxyz, rb.rotationWxyz)) fields.push('rotationWxyz');", ''],
  ['K5-extra-frames-ignored', 'compare.mjs', 'identical: identical === frames && rowsA.length === rowsB.length', 'identical: identical === frames'],
  ['K6-input-paths-ignored', 'compare.mjs', 'const paths = a && b ? diffPaths(a, b) : [', 'const paths = a && b ? [] : ['],
  ['K7-answer-rotation-order', 'compare.mjs', 'rotation: { x: p.rotationWxyz[1], y: p.rotationWxyz[2], z: p.rotationWxyz[3], w: p.rotationWxyz[0] }', 'rotation: { x: p.rotationWxyz[0], y: p.rotationWxyz[1], z: p.rotationWxyz[2], w: p.rotationWxyz[3] }'],
  ['K8-quaternion-angle-by-acos', 'compare.mjs', 'return 2 * Math.atan2(Math.hypot(x, y, z), Math.abs(w)) * 180 / Math.PI;', 'return 2 * Math.acos(Math.min(1, Math.abs(w))) * 180 / Math.PI;'],
  // canon.mjs
  ['C1-numbers-by-text', 'canon.mjs', "if (typeof v === 'number') return `f:${f64hex(v)}`;", "if (typeof v === 'number') return `f:${v}`;"],
  ['C2-bytes-by-length', 'canon.mjs', "return `u8:${v.length}:${sha256(v)}`;", "return `u8:${v.length}`;"],
  ['C3-ulp-distance-zero', 'canon.mjs', 'const d = ord(a) - ord(b);\n  return Number(d < 0n ? -d : d);', 'return 0;'],
  ['C4-diff-paths-blind', 'canon.mjs', 'if (JSON.stringify(a) === JSON.stringify(b)) return out;', 'return out;'],
  // bundle.mjs
  ['B1-id-seed-kept', 'bundle.mjs', "if (k === 'id_generator_seed') { dropped.push(k); return false; } return true;", 'return true;'],
  ['B2-frame-time-column-dropped', 'bundle.mjs', 'const c = [...f.raw];  // all columns:', 'const c = f.raw.slice(0, 8);  // all columns:'],
  ['B3-frames-without-atlas-kept', 'bundle.mjs', "if (!f.bootstrap && f.process && !f.pyramid) { c[5] = '0'; noAtlas.push(f.index); }", ''],
  ['B4-not-truncated', 'bundle.mjs', 'maxFrames > 0 ? src.frames.slice(1, 1 + maxFrames) : src.frames.slice(1)', 'src.frames.slice(1)'],
  ['B5-source-overwritable', 'bundle.mjs', "if (fs.existsSync(outDir)) throw new Error(`refusing to overwrite ${outDir}`);", ''],
  // players.mjs (fake clock, the C driver)
  ['P1-clock-not-searched', 'players.mjs', 'const exact = searchDoubles(c, (x) => 1e3 * x * 1e3 === ns);', 'const exact = ok0(c, ns) ? c : null;'],
  ['P2-readback-without-atlas-allowed', 'players.mjs', "if (!atlas) throw new Error('or1 fake GL: no atlas set for this readback');", 'if (!atlas) return;'],
  ['P3-column-9-zero-gives-video-time', 'players.mjs', ': { value: 0, exact: true, truncMatches: true };', ': v;'],
  // mutants.mjs
  ['T1-chunk-mutant-pattern-unchecked', 'mutants.mjs', 'if (n !== 1) throw new Error(`${id}: pattern occurs ${n} times in ${file}`);', ''],
];
// P1 needs a helper in the copy
const PRE = { 'players.mjs': '\nconst ok0 = (c, ns) => 1e3 * c * 1e3 === ns;\n' };

// --check-patterns: every pattern occurs exactly once in the current tools (no copy, no test run)
if (values['check-patterns']) {
  let bad = 0;
  for (const [id, file, from] of M) {
    const sub = file.startsWith('../') ? path.join(toolsSrc, file.slice(3)) : path.join(toolsSrc, 'or1', file);
    const n = fs.readFileSync(sub, 'utf8').split(from).length - 1;
    if (n !== 1) { console.log(`${id}: pattern occurs ${n} times in ${file}`); bad++; }
  }
  console.log(bad ? `${bad} bad pattern(s)` : `all ${M.length} patterns occur exactly once`);
  process.exit(bad ? 1 : 0);
}
fs.mkdirSync(out, { recursive: true });
// <out>/<id>/web-chunk/tools/or1 + gate-common, <out>/<id>/tools/prereg/sr_held.mjs: the relative imports of the repository resolve inside the copy
const copyTools = (id) => {
  const base = path.join(out, id);
  for (const sub of ['or1', 'gate-common']) {
    const d = path.join(base, 'web-chunk/tools', sub);
    fs.mkdirSync(d, { recursive: true });
    for (const f of fs.readdirSync(path.join(toolsSrc, sub))) if (/\.(mjs|sh)$/.test(f)) fs.copyFileSync(path.join(toolsSrc, sub, f), path.join(d, f));
  }
  fs.mkdirSync(path.join(base, 'tools/prereg'), { recursive: true });
  fs.copyFileSync(srHeld, path.join(base, 'tools/prereg/sr_held.mjs'));
  return path.join(base, 'web-chunk/tools/or1');
};
const run = (id, dir) => {
  const r = spawnSync(process.execPath, ['--test', ...tests], { env: { ...process.env, SLAM_REFACT_OR1_TOOLS_DIR: dir }, encoding: 'utf8' });
  fs.writeFileSync(path.join(out, `${id}.log`), r.stdout + r.stderr);
  return { id, exit: r.status, failed: [...r.stdout.matchAll(/^not ok \d+ - (\S+)/gm)].map((m) => m[1].replace(/:$/, '')) };
};
const only = values.only ? new Set(values.only.split(',')) : null;
const results = [];
results.push({ ...run('NULL', copyTools('NULL')), expected: 'pass' });
for (const [id, file, from, to] of M) {
  if (only && !only.has(id)) continue;
  const dir = copyTools(id);
  const target = path.join(dir, file);
  const text = fs.readFileSync(target, 'utf8');
  const n = text.split(from).length - 1;
  if (n !== 1) throw new Error(`${id}: pattern occurs ${n} times in ${file}`);
  fs.writeFileSync(target, text.replace(from, () => to) + (PRE[file] && id.startsWith('P1') ? PRE[file] : ''));
  results.push({ ...run(id, dir), expected: 'fail' });
}
const ok = results[0].exit === 0 && results.slice(1).every((r) => r.exit !== 0 && r.failed.length > 0);
const summary = { schema: 'or1-tool-mutants-v2', at: new Date().toISOString(), nullRunPasses: results[0].exit === 0, mutants: results.length - 1,
  detected: results.slice(1).filter((r) => r.exit !== 0 && r.failed.length > 0).length, survivors: results.slice(1).filter((r) => !(r.exit !== 0 && r.failed.length > 0)).map((r) => r.id), results };
fs.writeFileSync(path.join(out, 'summary.json'), `${JSON.stringify(summary, null, 1)}\n`);
for (const r of results) console.log(`${r.id.padEnd(46)} exit ${r.exit} ${r.expected === 'pass' ? '(null-run)' : `failed: ${r.failed.join(', ')}`}`);
console.log(ok ? `OK: null-run passes, ${summary.detected}/${summary.mutants} mutants detected` : `FAILED: survivors ${summary.survivors.join(', ')}`);
process.exitCode = ok ? 0 : 1;
