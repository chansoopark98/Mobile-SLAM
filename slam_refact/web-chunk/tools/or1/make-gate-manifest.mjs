// Writes the OR-1 gate registration files next to the gate document (preregistration protocol section 6): the manifest with every pin, the
// derived registered gate data, and a snapshot directory with copies of the pinned files. Run AFTER the gate document is final; it only reads
// inputs (the registered gate bundles are derived and hashed, never played: no engine, no arm). It is NOT run by the harness lane: the coordinator /
// registration step runs it on the frozen tree.
//   node slam_refact/web-chunk/tools/or1/make-gate-manifest.mjs --prereg-dir <dir> --gate-id <id> --gate-version <N> --pair <pair.json>
//        --gate-data <json: [{name, src, record}]> --derived-out <new dir under build/> --selftest <dir with selftest.json> --tool-mutants <dir with summary.json>
//        [--supersedes-version <M>] [--classification <text>] [--runner <run.sh>] [--designation-env <NAME>]
// Nothing about the gate, the data, the pair or the paths is a constant of this file (v2 hardcoded all of them). The manifest has the shape the shared
// device slam_refact/tools/prereg/sr_held.mjs reads (docs/design/sr-held-design.md 3.2): `held` {kinds, pins, inputs, seed, dev_seeds}, `review` {} and
// `first_real_run_at` null. `pinned` stays as the readable record; held.pins is its flat base-name form (hold.mjs requires them to agree).
// Refuses to overwrite an existing manifest (a change is a new version, protocol section 7).
import fs from 'node:fs';
import path from 'node:path';
import { execFileSync } from 'node:child_process';
import { parseArgs } from 'node:util';
import { fileURLToPath } from 'node:url';
import { derive, readBundle, inputFiles } from './bundle.mjs';
import { fileSetSha256, heldPinsOf, liveEnvironment, sha256File } from './hold.mjs';
import { SR_HELD_FILE, gateArgs, gateMdPath, manifestPath, sr } from '../gate-common/held.mjs';

const ls = (root, dir, re) => fs.readdirSync(path.join(root, dir)).filter((f) => re.test(f)).sort().map((f) => `${dir}/${f}`);
const DEFAULT_TESTS = ['or1.test.mjs', 'or1-hold.test.mjs', 'or1-v3.test.mjs', 'run-or1-mutants.mjs', 'chk1-engine-time.test.mjs', 'chk1-fallback.test.mjs'];

export function buildManifest({ root, gate, gateDataSpec, pairFile, derivedOut, selftestDir, toolMutantsDir, supersedesVersion = null, classification = '', runnerFile = null,
  tests = DEFAULT_TESTS, now = null, gitHead = null }) {
  const rel = (p) => path.relative(root, path.resolve(root, p));
  const base = `${gate.gateId}.v${gate.version}`;
  const manifestFile = manifestPath(gate);
  const gateMd = gateMdPath(gate);
  if (fs.existsSync(manifestFile)) throw new Error(`refusing to overwrite ${rel(manifestFile)} (a change is a new version)`);
  if (!fs.existsSync(gateMd)) throw new Error(`${rel(gateMd)} must exist first`);
  for (const d of gateDataSpec) if (!d.name || !d.src || !d.record) throw new Error(`gate data entry without name / src / record: ${JSON.stringify(d)}`);
  const derivedRoot = path.resolve(root, derivedOut);
  if (!derivedRoot.startsWith(path.join(root, 'build') + path.sep)) throw new Error('--derived-out must be under build/');
  const pair = JSON.parse(fs.readFileSync(path.join(root, pairFile), 'utf8'));
  const pinSet = (files) => Object.fromEntries(files.map((f) => [f, sha256File(path.join(root, f))]));

  const gateData = gateDataSpec.map((g) => {
    const out = `${rel(derivedRoot)}/${g.name}/bundle`;
    const meta = derive(path.join(root, g.src), path.join(root, out), { sourceLabel: g.src });
    const bundle = readBundle(path.join(root, out));
    const atlasFiles = inputFiles(bundle, root).map((f) => path.relative(root, f));
    const missing = atlasFiles.filter((f) => !fs.existsSync(path.join(root, f)));
    if (missing.length) throw new Error(`${g.name}: ${missing.length} atlas files missing`);
    return { name: g.name, record: g.record, source: g.src, derived_dir: out, source_sha256: meta.sourceSha256, derived_sha256: meta.derivedSha256, frames: meta.frames, sensor_rows: meta.sensorRows,
      frames_without_atlas_disabled: meta.framesWithoutAtlasDisabled, atlas_files: atlasFiles.length, atlas_set_sha256: fileSetSha256(root, atlasFiles) };
  });

  const pinned = {
    harness: pinSet(ls(root, 'slam_refact/web-chunk/tools/or1', /\.(mjs|sh)$/)),
    gate_common: pinSet(ls(root, 'slam_refact/web-chunk/tools/gate-common', /\.mjs$/)),
    held_device: { 'sr_held.mjs': sha256File(SR_HELD_FILE) },
    tests: pinSet(tests.map((f) => `slam_refact/web-chunk/tests/${f}`)),
    chunk: pinSet(ls(root, 'slam_refact/web-chunk', /\.js$/)),
    docs: pinSet(['slam_refact/docs/spec/07-xr-slam-chunk.md']),
    pair: pinSet([pairFile, pair.snapshot_sources]),
    engine: { dir: path.dirname(pair.wasm.file), ...Object.fromEntries(['slam_refact_engine.js', 'slam_refact_engine.wasm', 'manifest.json']
      .map((f) => [f, sha256File(path.join(root, path.dirname(pair.wasm.file), f))])) },
    native: pinSet([pair.native_replay.file]),
    ...(runnerFile ? { runner: pinSet([runnerFile]) } : {}),
    environment: liveEnvironment(),
    gate_data: gateData,
  };
  const heldPins = heldPinsOf(root, pinned);  // throws when two pinned files share a base name

  // snapshot: copies of every pinned file + SHA256SUMS (evidence grade "medium", protocol section 5)
  const snapDir = path.join(gate.preregDir, `${base}.snapshot`);
  fs.mkdirSync(snapDir, { recursive: false });
  const sums = [];
  for (const [section, entries] of Object.entries(pinned)) {
    if (['gate_data', 'environment', 'held_device'].includes(section)) continue;
    for (const f of Object.keys(entries)) {
      if (section === 'engine' && f === 'dir') continue;
      const file = section === 'engine' ? path.join(entries.dir, f) : f;
      const dest = path.join(snapDir, file);
      fs.mkdirSync(path.dirname(dest), { recursive: true });
      fs.copyFileSync(path.join(root, file), dest);
      sums.push(`${sha256File(dest)}  ${file}\n`);
    }
  }
  fs.mkdirSync(path.join(snapDir, 'slam_refact/tools/prereg'), { recursive: true });
  fs.copyFileSync(SR_HELD_FILE, path.join(snapDir, 'slam_refact/tools/prereg/sr_held.mjs'));
  sums.push(`${sha256File(SR_HELD_FILE)}  slam_refact/tools/prereg/sr_held.mjs\n`);
  fs.copyFileSync(gateMd, path.join(snapDir, `${base}.md`));
  sums.push(`${sha256File(gateMd)}  ${base}.md\n`);  // the path inside the snapshot (v2 wrote the preregistration path, which `sha256sum -c` could not find: V-12)
  fs.writeFileSync(path.join(snapDir, 'SHA256SUMS'), sums.sort().join(''));

  const selftest = JSON.parse(fs.readFileSync(path.join(root, selftestDir, 'selftest.json'), 'utf8'));
  const tool = JSON.parse(fs.readFileSync(path.join(root, toolMutantsDir, 'summary.json'), 'utf8'));
  const date = now ?? execFileSync('date', ['-Is'], { encoding: 'utf8' }).trim();
  let head = gitHead;
  if (head === null) { try { head = execFileSync('git', ['rev-parse', 'HEAD'], { cwd: root, encoding: 'utf8' }).trim(); } catch { head = null; } }
  const supersedes = supersedesVersion === null ? null : (() => {
    const prev = `${gate.gateId}.v${supersedesVersion}`;
    const f = (suffix) => path.join(gate.preregDir, `${prev}${suffix}`);
    return { file: rel(f('.md')), sha256: sha256File(f('.md')), manifest_sha256: sha256File(f('.manifest.json')),
      first_real_run_at_in_previous: JSON.parse(fs.readFileSync(f('.manifest.json'), 'utf8')).first_real_run_at ?? null };
  })();
  // v3 C12 item 6: running the same registered bundles again means every arm output on them was already observed; the manifest says so (the gate document section 0 repeats it),
  // and the run report copies it. Discrimination then rests on the mutation power, not on unseen data.
  const dataReuse = supersedes && supersedes.first_real_run_at_in_previous
    ? { observed_in_previous_version: true, previous_version: supersedesVersion, previous_first_real_run_at: supersedes.first_real_run_at_in_previous,
      statement: `The registered bundles (${gateData.map((g) => g.name).join(', ')}) were run by ${gate.gateId} v${supersedesVersion}; every arm output on them has been observed. They are not unseen data in this version; the discriminating power of the gate rests on its mutation detection (null_run, mutation), not on novelty.` }
    : { observed_in_previous_version: false };
  const manifest = {
    gate_id: gate.gateId, version: gate.version, classification: [classification],
    // `file` and `sha256` come first: the first "sha256" key of this document is the gate file's
    file: path.basename(gateMd), sha256: sha256File(gateMd),
    ...(supersedes ? { supersedes } : {}),
    created_at: date, registered_by: 'OR-1 registration tool (make-gate-manifest.mjs); the coordinator runs it on the frozen tree', git_head: head, tree_sha: null,
    snapshot: `${rel(snapDir)}/ (copies of every pinned file, SHA256SUMS); engine pair ${path.dirname(pairFile)}/ (frozen source copy, snapshot-sources.sha256, pair.json); no write-tree unless the freeze record names one -> evidence grade medium`,
    seen_before_registration: 'see the gate document section 0',
    not_seen: dataReuse.observed_in_previous_version ? 'see data_reuse: the arm outputs on these bundles were observed in the previous version' : 'the outputs of any arm on the registered gate bundles; the registered gate bundles were derived and hashed only',
    data_reuse: dataReuse,
    resolution: { metric: 'bit comparison of IEEE doubles; engine outputs are float32 values widened to double', value: 1.1920929e-07, unit: 'float32 ulp at 1.0',
      method: 'null-runs: arms compared with themselves and with each other on two development data sets (selftest.json); the comparison is exact so its noise floor is 0 and the representation resolution is the float32 ulp' },
    tolerance: { value: 0, unit: 'exact', determinism_class_gate: true,
      exact_class_justification: 'protocol 11.1: C and W call the same engine binary (one WASM file) through the same embind bindings; the factor under test is the chunk wiring (the JS between the browser events and the glue calls). Every value is compared by its IEEE bits, so there is no rounding that could differ for a correct chunk. N is a different binary (native, glibc libm) and its comparison (NW, CN) is REPORT, not this class.' },
    report_only: ['NW', 'CN', 'input statistics', 'wall times', 'glueConfig'],
    null_run: { evidence: rel(path.join(selftestDir, 'selftest.json')), result: selftest.summary.nullFailures === 0 ? 'PASS' : 'FAIL',
      detail: { datasets: Object.keys(selftest.datasets), nullFailures: selftest.summary.nullFailures } },
    mutation: { description: 'chunk copies (selftest: chunkMutants), native-side bundle copies, one-ulp output-row changes, and the defects injected into the harness itself (tool-mutants)',
      evidence: rel(path.join(selftestDir, 'selftest.json')), detected: selftest.summary.passes === true && tool.survivors.length === 0,
      sub_resolution_detected: true, larger_mutations_detected: true,
      summary: { chunkMutantsDetected: `${selftest.summary.chunkMutantsDetectedByAChunkGate}/${selftest.summary.chunkMutantsRun}`, equivalentMutantFalseAlarms: selftest.summary.equivalentMutantFalseAlarms,
        rowMutations: `${selftest.summary.rowMutationsDetected}/${selftest.summary.rowMutations}`, toolMutants: `${tool.detected}/${tool.mutants}`, toolMutantsEvidence: rel(path.join(toolMutantsDir, 'summary.json')) } },
    registered_run: { arms: 'WCN', repetitions: 3, gate_data: gateData.map((g) => g.name), command_file: runnerFile ?? `${base}.md` },
    pinned, pins_note: 'sha256 of files and of the node executable pin which bytes were present, not which bytes executed; shared libraries, the kernel and the CPU are not covered (protocol 12.5 item 6). The hold is an accident-prevention device, not an authentication.',
    held: { kinds: gateData.map((g) => g.name), pins: heldPins, inputs: Object.fromEntries(gateData.map((g) => [g.name, treeShaOf(root, g.derived_dir)])), seed: false, dev_seeds: [], sealed_file_sha256: [] },
    first_run_marker: `${rel(gate.preregDir)}/${base}.first-run.<bundle name>.marker (one per registered bundle, created exclusively by its run; absent at registration)`,
    review: {}, first_real_run_at: null,
  };
  fs.writeFileSync(manifestFile, `${JSON.stringify(manifest, null, 1)}\n`, { flag: 'wx' });
  return { manifest, manifestFile, snapDir, gateData };
}

function treeShaOf(root, dir) {
  const t = sr.treeSha(path.join(root, dir));
  if (t.sha === undefined) throw new Error(`${dir}: cannot be hashed as registered input data (${t.code})`);
  return t.sha;
}

const isMain = process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url);
if (isMain) {
  const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../../..');
  const { values } = parseArgs({ options: { 'prereg-dir': { type: 'string' }, 'gate-id': { type: 'string' }, 'gate-version': { type: 'string' }, 'designation-env': { type: 'string' },
    pair: { type: 'string' }, 'gate-data': { type: 'string' }, 'derived-out': { type: 'string' }, selftest: { type: 'string' }, 'tool-mutants': { type: 'string' },
    'supersedes-version': { type: 'string' }, classification: { type: 'string' }, runner: { type: 'string' } } });
  const gate = gateArgs({ 'designation-env': 'SLAM_REFACT_UNUSED_BY_REGISTRATION', ...values }, root);
  const r = buildManifest({ root, gate, gateDataSpec: JSON.parse(fs.readFileSync(path.resolve(root, values['gate-data']), 'utf8')), pairFile: values.pair, derivedOut: values['derived-out'],
    selftestDir: values.selftest, toolMutantsDir: values['tool-mutants'], supersedesVersion: values['supersedes-version'] ? Number(values['supersedes-version']) : null,
    classification: values.classification ?? '', runnerFile: values.runner ?? null });
  console.log(JSON.stringify({ manifest: path.relative(root, r.manifestFile), sha256: sha256File(r.manifestFile), snapshot: path.relative(root, r.snapDir),
    gateData: r.gateData.map((g) => [g.name, g.frames, g.sensor_rows, g.atlas_files]), designationPins: 'node or1.mjs pin --manifest <manifest> (after the review is recorded)' }, null, 1));
}
