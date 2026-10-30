// or1-chunk-replay-gate.v4: the registration tool. It writes the manifest next to the gate document (protocol section 6): the registered scenarios with their registered
// expectations (calls, frames, capture size, rotation, floors), the derived bundles and their hashes, and the pins of every file the run uses.
//
//   node make-manifest.mjs --prereg-dir <dir> --scenarios <scenarios.json> --derived-out <NEW dir under build/> --pair PENDING-F3
//        -> DRAFT: <prereg>/or1-chunk-replay-gate.v4.manifest.draft.json. The data part is real (the scenarios are derived and hashed, nothing is played); every field that
//           depends on the F3 freeze (engine pair, native binary, chunk / harness pins, environment, selftest and mutation evidence) is the string "PENDING-F3". A draft is
//           not a manifest: the hold device cannot read it and the run refuses it.
//   node make-manifest.mjs --prereg-dir <dir> --scenarios <scenarios.json> --derived-out <NEW dir> --pair <pair.json> --selftest <dir with selftest.json>
//        --judge-mutants <dir with summary.json> --tool-mutants <dir with summary.json>
//        -> the manifest or1-chunk-replay-gate.v4.manifest.json (shape of the v3 manifest, the shared device sr_held.mjs reads it), `review` {} and first_real_run_at null.
//           Run by the coordinator on the FROZEN tree, after the gate document <prereg>/or1-chunk-replay-gate.v4.md was copied byte for byte from
//           slam_refact/docs/design/or1-chunk-replay-gate-v4-registration.md. It refuses to overwrite a manifest, refuses a pair whose files are missing, a selftest that did not pass.
// Nothing in this file is run by the lane that wrote it on registered data against the real preregistration directory (the tests use a made-up gate id and development data).
import fs from 'node:fs';
import path from 'node:path';
import { execFileSync } from 'node:child_process';
import { parseArgs } from 'node:util';
import { derive, readBundle, inputFiles } from '../../../web-chunk/tools/or1/bundle.mjs';
import { expectedCounts, C4_R_MIN, C4_K_MIN } from '../../../web-chunk/tools/or1/run.mjs';
import { fileSetSha256, heldPinsOf, liveEnvironment, sha256File } from '../../../web-chunk/tools/or1/hold.mjs';
import { SR_HELD_FILE, gateMdPath, manifestPath, sr } from '../../../web-chunk/tools/gate-common/held.mjs';
import { prepare } from './prepare-orient-bundle.mjs';

const here = import.meta.dirname;
const root = path.resolve(here, '../../../..');
export const PENDING = 'PENDING-F3';
export const GATE_ID = 'or1-chunk-replay-gate';
export const GATE_VERSION = 4;
const REGISTRATION_DOC = 'slam_refact/docs/design/or1-chunk-replay-gate-v4-registration.md';
const rel = (p) => path.relative(root, path.resolve(root, p));
const ls = (dir, re) => fs.readdirSync(path.join(root, dir)).filter((f) => re.test(f)).sort().map((f) => `${dir}/${f}`);

export const DEFAULT_TESTS = ['or1.test.mjs', 'or1-hold.test.mjs', 'or1-v3.test.mjs', 'or1-rotation.test.mjs', 'run-or1-mutants.mjs', 'chk1-engine-time.test.mjs', 'chk1-fallback.test.mjs']
  .map((f) => `slam_refact/web-chunk/tests/${f}`).concat(['judge.test.mjs', 'prepare-orient-bundle.test.mjs', 'make-manifest.test.mjs'].map((f) => `slam_refact/tools/gates/or1-chunk-replay-v4/${f}`));
export const RUNNER_FILES = ['run.sh', 'judge.mjs', 'scenarios.json', 'prepare-orient-bundle.mjs', 'selftest-v4.mjs', 'make-manifest.mjs', 'compat-check.mjs', 'judge-mutants.mjs', 'players-mutants.mjs']
  .map((f) => `slam_refact/tools/gates/or1-chunk-replay-v4/${f}`);

// the data part of the manifest: derive every scenario, read what the judge must hold the runs to
export function buildScenarios({ spec, derivedRoot, prepareFn = prepare }) {
  const names = spec.scenarios.map((s) => s.name);
  if (new Set(names).size !== names.length) throw new Error('scenario names are not unique');
  const registered = [], gateData = [];
  for (const s of spec.scenarios) {
    for (const k of ['name', 'family', 'record', 'kind', 'src']) if (!s[k]) throw new Error(`scenario ${s.name ?? '?'}: ${k} is missing`);
    if (!['portrait', 'rotation'].includes(s.family)) throw new Error(`scenario ${s.name}: family ${s.family}`);
    const base = path.join(derivedRoot, s.name);
    let src = s.src;
    if (s.kind === 'orient') {
      src = rel(path.join(base, 'src'));
      prepareFn({ rec: path.resolve(root, s.src), out: path.join(root, src), repoRoot: root });
    } else if (s.kind !== 'bundle') throw new Error(`scenario ${s.name}: kind ${s.kind}`);
    const out = rel(path.join(base, 'bundle'));
    const meta = derive(path.join(root, src), path.join(root, out), { sourceLabel: src, frames: s.frames ?? 0 });  // `frames` (a prefix) exists for the tests; the registered scenarios have none
    const bundle = readBundle(path.join(root, out));
    const atlasFiles = inputFiles(bundle, root).map((f) => path.relative(root, f));
    const missing = atlasFiles.filter((f) => !fs.existsSync(path.join(root, f)));
    if (missing.length) throw new Error(`${s.name}: ${missing.length} atlas files are missing`);
    const num = (k) => Number(bundle.config.get(k));
    const expected = expectedCounts(bundle);
    const fl = spec.floors[s.family];
    const floors = { c4: fl.c4 ?? null, min_fresh_nonrepeated: fl.min_fresh_nonrepeated ?? Math.ceil(fl.min_fresh_nonrepeated_fraction * expected.frames), min_distinct_rotations: fl.min_distinct_rotations ?? null };
    if (floors.c4 && (floors.c4.r_min !== C4_R_MIN || floors.c4.k_min !== C4_K_MIN)) throw new Error(`${s.name}: the C4 floor differs from the v3 constants (${C4_R_MIN}, ${C4_K_MIN})`);
    if (s.family === 'rotation' && [num('capture_width'), num('capture_height'), num('rotation_deg')].some((x) => !Number.isFinite(x))) throw new Error(`${s.name}: capture size / rotation missing`);
    registered.push({ name: s.name, family: s.family, record: s.record, capture: { width: num('capture_width'), height: num('capture_height'), rotation: num('rotation_deg') }, expected, floors });
    gateData.push({ name: s.name, record: s.record, family: s.family, kind: s.kind, source: src, source_recording: s.kind === 'orient' ? s.src : null, derived_dir: out, source_sha256: meta.sourceSha256,
      derived_sha256: meta.derivedSha256, frames: meta.frames, sensor_rows: meta.sensorRows, frames_without_atlas_disabled: meta.framesWithoutAtlasDisabled, atlas_files: atlasFiles.length,
      atlas_set_sha256: fileSetSha256(root, atlasFiles) });
  }
  return { registered, gateData };
}

const INDEPENDENT = { n: 3, derivation: 'The independent unit is the RECORD: flip6-s3-lowtex (G1, G2), tum-room1-lowtex (G3, G4) and flip6-s2-orient (O0, O90, O180, O270) = 3. The two bundles of a portrait record are timing variants of one recording; '
  + 'the four orient bundles are ONE recorded sensor and image stream (flip6-s2, input frames 6-67) in four orientations, recorded by four separate runs of the original SDK (the stream differs only by the rotation of the images and the device orientation values): '
  + 'they are not counted as independent of each other. Neither are the frames of one bundle, the 3 repetitions or the 3 arms. A sentence about rotation 90 / 180 / 270 rests on ONE recorded stream; a sentence such as "all 8 scenarios pass" must say that they are variants of 3 records.' };

export function buildV4Manifest({ preregDir, scenariosFile, derivedOut, pairFile, selftestDir = null, judgeMutantsDir = null, toolMutantsDir = null, designationEnv = 'SLAM_REFACT_OR1_V4_DESIGNATION_SHA256',
  gateId = GATE_ID, version = GATE_VERSION, tests = DEFAULT_TESTS, runnerFiles = RUNNER_FILES, now = null, gitHead = null, prepareFn = prepare, requireDoc = true }) {
  const gate = { gateId, version, preregDir: path.resolve(root, preregDir), designationEnv };
  const draft = pairFile === PENDING;
  const base = `${gateId}.v${version}`;
  const manifestFile = draft ? path.join(gate.preregDir, `${base}.manifest.draft.json`) : manifestPath(gate);
  const gateMd = gateMdPath(gate);
  if (fs.existsSync(manifestFile)) throw new Error(`refusing to overwrite ${rel(manifestFile)} (a change is a new version)`);
  if (!fs.existsSync(gateMd)) throw new Error(`${rel(gateMd)} must exist first (a byte copy of ${REGISTRATION_DOC})`);
  if (requireDoc && !fs.existsSync(path.join(root, REGISTRATION_DOC))) throw new Error(`${REGISTRATION_DOC} does not exist`);
  if (requireDoc && sha256File(gateMd) !== sha256File(path.join(root, REGISTRATION_DOC))) throw new Error(`${rel(gateMd)} is not a byte copy of ${REGISTRATION_DOC}`);
  const derivedRoot = path.resolve(root, derivedOut);
  if (!derivedRoot.startsWith(path.join(root, 'build') + path.sep)) throw new Error('--derived-out must be under build/');
  const spec = JSON.parse(fs.readFileSync(path.resolve(root, scenariosFile), 'utf8'));
  const { registered, gateData } = buildScenarios({ spec, derivedRoot, prepareFn });
  const date = now ?? execFileSync('date', ['-Is'], { encoding: 'utf8' }).trim();
  let head = gitHead;
  if (head === null) { try { head = execFileSync('git', ['rev-parse', 'HEAD'], { cwd: root, encoding: 'utf8' }).trim(); } catch { head = null; } }
  const v3 = path.join(gate.preregDir, 'or1-chunk-replay-gate.v3');
  const supersedes = fs.existsSync(`${v3}.md`) && fs.existsSync(`${v3}.manifest.json`)
    ? { file: rel(`${v3}.md`), sha256: sha256File(`${v3}.md`), manifest_sha256: sha256File(`${v3}.manifest.json`), first_real_run_at_in_previous: JSON.parse(fs.readFileSync(`${v3}.manifest.json`, 'utf8')).first_real_run_at ?? null } : null;
  const dataReuse = { observed_in_previous_version: true, previous_version: 3, previous_first_real_run_at: supersedes?.first_real_run_at_in_previous ?? null,
    portrait_scenarios: gateData.filter((g) => g.family === 'portrait').map((g) => g.name),
    rotation_scenarios: gateData.filter((g) => g.family === 'rotation').map((g) => g.name),
    statement: 'The four portrait scenarios (G1-G4) are the registered bundles of or1-chunk-replay-gate v3: every arm output on them was observed with the v3 frozen engine and the v3 chunk bytes. They are run again on the F3 chunk bytes as a regression of the portrait path; they are not unseen data. '
      + 'The four orient scenarios (O0, O90, O180, O270) were never played through the chunk or through any OR-1 arm before this gate, EXCEPT that the author of the harness played the first 25 frames of each with all three arms as a development smoke (all comparisons identical) and that the engine-level orient gate '
      + '(orient-us023-gate v3: pyramid P, intrinsics K, sensors S, pose R) used the same recordings. The discriminating power of the gate rests on its mutation detection (null_run, mutation), not on novelty.' };
  const common = {
    gate_id: gateId, version, classification: ['extension: rotated / landscape scenarios (OR-1 criterion 5b) and re-run of the portrait scenarios of v3 on the chunk bytes after PKG-A (rotation 90 / 180 / 270 support); new judge (INVALID aggregation, tool crash is not a FAIL, registered counts); made before any result of this gate'],
    file: path.basename(gateMd), sha256: sha256File(gateMd), ...(supersedes ? { supersedes } : {}),
    created_at: date, registered_by: 'OR-1 v4 registration tool (make-manifest.mjs); the coordinator runs it on the frozen tree', git_head: head, tree_sha: null,
    seen_before_registration: 'see the gate document section 0', not_seen: 'see data_reuse', data_reuse: dataReuse,
    resolution: { metric: 'bit comparison of IEEE doubles; engine outputs are float32 values widened to double', value: 1.1920929e-07, unit: 'float32 ulp at 1.0',
      method: 'null-runs: arms compared with themselves and with each other on the development data sets (selftest.json); the comparison is exact so its noise floor is 0 and the representation resolution is the float32 ulp' },
    tolerance: { value: 0, unit: 'exact', determinism_class_gate: true,
      exact_class_justification: 'protocol 11.1: C and W call the same engine binary (one WASM file) through the same embind bindings; the factor under test is the chunk wiring (the JS between the browser events and the glue calls), now for any capture rotation. Every value is compared by its IEEE bits, so there is no rounding that could differ for a correct chunk. N is a different binary (native, glibc libm) and its comparison (NW, CN) is REPORT, not this class.' },
    report_only: ['NW', 'CN', 'input statistics', 'wall times', 'glueConfig keys outside the session keys'],
    independent_samples: INDEPENDENT,
    registered_run: { arms: 'WCN', repetitions: 3, gate_data: gateData.map((g) => g.name), command_file: 'slam_refact/tools/gates/or1-chunk-replay-v4/run.sh', judge: 'slam_refact/tools/gates/or1-chunk-replay-v4/judge.mjs',
      scenarios: registered, k_min: spec.k_min },
    first_real_run_at_rule: 'After the last scenario run, the coordinator writes first_real_run_at = the birth time (stat --format=%w) of the EARLIEST first-run marker file, floored to whole seconds, ISO with time zone; so first_real_run_at <= every marker birth (gate document section 10).',
    coordinator_seed: 'none (this gate has no held seed; inputs are the registered recorded bundles)',
    first_run_marker: `${rel(gate.preregDir)}/${base}.first-run.<scenario name>.marker (one per registered scenario, created exclusively by its run; absent at registration)`,
  };
  if (draft) {
    const draftManifest = { ...common, status: 'DRAFT-PENDING-F3', not_a_manifest: 'The hold device cannot read this file. The fields below that depend on the F3 freeze are the string PENDING-F3; make-manifest.mjs --pair <F3 pair.json> writes the manifest.',
      snapshot: PENDING, null_run: PENDING, mutation: PENDING, pinned: { harness: PENDING, gate_common: PENDING, held_device: PENDING, tests: PENDING, chunk: PENDING, docs: PENDING, pair: PENDING, engine: PENDING, native: PENDING, runner: PENDING, environment: PENDING, gate_data: gateData },
      held: PENDING, engine_provenance: PENDING, review: {}, first_real_run_at: null };
    fs.writeFileSync(manifestFile, `${JSON.stringify(draftManifest, null, 1)}\n`, { flag: 'wx' });
    return { manifest: draftManifest, manifestFile, mode: 'draft' };
  }

  // ---- final: everything is read from the frozen tree ----
  for (const d of [selftestDir, judgeMutantsDir, toolMutantsDir]) if (!d) throw new Error('--selftest, --judge-mutants and --tool-mutants are required for a manifest');
  const pair = JSON.parse(fs.readFileSync(path.resolve(root, pairFile), 'utf8'));
  const selftest = JSON.parse(fs.readFileSync(path.join(root, selftestDir, 'selftest.json'), 'utf8'));
  const judgeMut = JSON.parse(fs.readFileSync(path.join(root, judgeMutantsDir, 'summary.json'), 'utf8'));
  const toolMut = JSON.parse(fs.readFileSync(path.join(root, toolMutantsDir, 'summary.json'), 'utf8'));
  if (selftest.summary?.passes !== true) throw new Error('the self-test did not pass (summary.passes): a gate whose mutation detection failed is not registered');
  if (!judgeMut.nullRunPassed || judgeMut.survivors.length) throw new Error('the judge mutation run did not pass');
  if (toolMut.survivors.length) throw new Error('the harness tool mutation run has survivors');
  const pinSet = (files) => Object.fromEntries(files.map((f) => [f, sha256File(path.join(root, f))]));
  const pinned = {
    harness: pinSet(ls('slam_refact/web-chunk/tools/or1', /\.(mjs|sh)$/)),
    gate_common: pinSet(ls('slam_refact/web-chunk/tools/gate-common', /\.mjs$/)),
    held_device: { 'sr_held.mjs': sha256File(SR_HELD_FILE) },
    tests: pinSet(tests),
    chunk: pinSet(ls('slam_refact/web-chunk', /\.js$/)),
    docs: pinSet(['slam_refact/docs/spec/07-xr-slam-chunk.md']),
    pair: pinSet([pairFile, pair.snapshot_sources]),
    engine: { dir: path.dirname(pair.wasm.file), ...Object.fromEntries(['slam_refact_engine.js', 'slam_refact_engine.wasm', 'manifest.json'].map((f) => [f, sha256File(path.join(root, path.dirname(pair.wasm.file), f))])) },
    native: pinSet([pair.native_replay.file]),
    runner: pinSet(runnerFiles),
    environment: liveEnvironment(),
    gate_data: gateData,
  };
  const heldPins = heldPinsOf(root, pinned);  // throws when two pinned files share a base name
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
  sums.push(`${sha256File(gateMd)}  ${base}.md\n`);
  fs.writeFileSync(path.join(snapDir, 'SHA256SUMS'), sums.sort().join(''));
  const s = selftest.summary;
  const manifest = {
    ...common,
    snapshot: `${rel(snapDir)}/ (copies of every pinned file, SHA256SUMS); engine pair ${path.dirname(pairFile)}/ (frozen source copy, snapshot-sources.sha256, pair.json); no write-tree unless the freeze record names one -> evidence grade medium`,
    null_run: { evidence: rel(path.join(selftestDir, 'selftest.json')), result: s.nullFailures === 0 ? 'PASS' : 'FAIL', detail: { datasets: Object.keys(selftest.datasets), nullFailures: s.nullFailures } },
    mutation: { description: 'chunk copies (selftest: v3 chunk mutants on the portrait data, rotation mutants RX* on the orient data), native-side bundle copies, one-ulp output-row changes, the defects injected into the judge (judge mutants) and into the harness tools (tool mutants)',
      evidence: rel(path.join(selftestDir, 'selftest.json')), detected: s.passes === true && judgeMut.survivors.length === 0 && toolMut.survivors.length === 0,
      sub_resolution_detected: s.rowMutations > 0 && s.rowMutationsDetected === s.rowMutations, larger_mutations_detected: s.chunkMutantsDetectedByAChunkGate === s.chunkMutantsRun,
      summary: { chunkMutantsDetected: `${s.chunkMutantsDetectedByAChunkGate}/${s.chunkMutantsRun}`, outOfClaim: s.outOfClaim, equivalentMutantFalseAlarms: s.equivalentMutantFalseAlarms, rowMutations: `${s.rowMutationsDetected}/${s.rowMutations}`,
        judgeMutants: `${judgeMut.detected}/${judgeMut.mutants}`, judgeMutantsEvidence: rel(path.join(judgeMutantsDir, 'summary.json')), toolMutants: `${toolMut.detected}/${toolMut.mutants}`, toolMutantsEvidence: rel(path.join(toolMutantsDir, 'summary.json')) } },
    pinned, pins_note: 'sha256 of files and of the node executable pin which bytes were present, not which bytes executed; shared libraries, the kernel and the CPU are not covered (protocol 12.5 item 6). The hold is an accident-prevention device, not an authentication.',
    held: { kinds: gateData.map((g) => g.name), pins: heldPins, inputs: Object.fromEntries(gateData.map((g) => {
      const t = sr.treeSha(path.join(root, g.derived_dir));
      if (t.sha === undefined) throw new Error(`${g.derived_dir}: cannot be hashed as registered input data (${t.code})`);
      return [g.name, t.sha];
    })), seed: false, dev_seeds: [], sealed_file_sha256: [] },
    engine_provenance: { freeze_record: 'PENDING-F3: the coordinator writes the FREEZE record and the pair provenance here before the review request', pair_json: pairFile },
    review: {}, first_real_run_at: null,
  };
  fs.writeFileSync(manifestFile, `${JSON.stringify(manifest, null, 1)}\n`, { flag: 'wx' });
  return { manifest, manifestFile, mode: 'final', snapDir };
}

const isMain = process.argv[1] && path.resolve(process.argv[1]) === import.meta.filename;
if (isMain) {
  const { values } = parseArgs({ options: { 'prereg-dir': { type: 'string' }, scenarios: { type: 'string' }, 'derived-out': { type: 'string' }, pair: { type: 'string' }, selftest: { type: 'string' },
    'judge-mutants': { type: 'string' }, 'tool-mutants': { type: 'string' }, 'designation-env': { type: 'string' } } });
  try {
    for (const k of ['prereg-dir', 'scenarios', 'derived-out', 'pair']) if (!values[k]) throw new Error(`--${k} is required (--pair PENDING-F3 writes the draft)`);
    const r = buildV4Manifest({ preregDir: values['prereg-dir'], scenariosFile: values.scenarios, derivedOut: values['derived-out'], pairFile: values.pair, selftestDir: values.selftest ?? null,
      judgeMutantsDir: values['judge-mutants'] ?? null, toolMutantsDir: values['tool-mutants'] ?? null, ...(values['designation-env'] ? { designationEnv: values['designation-env'] } : {}) });
    console.log(JSON.stringify({ mode: r.mode, manifest: rel(r.manifestFile), sha256: sha256File(r.manifestFile), scenarios: r.manifest.registered_run.scenarios.map((s) => [s.name, s.expected.calls, s.expected.frames, s.capture.width, s.capture.height, s.capture.rotation]),
      next: r.mode === 'draft' ? 'F3: rerun with --pair <pair.json> --selftest --judge-mutants --tool-mutants' : 'node slam_refact/web-chunk/tools/or1/or1.mjs pin --manifest <manifest> (after the review is recorded)' }, null, 1));
  } catch (error) {
    console.error(String(error?.message ?? error));
    process.exitCode = 1;
  }
}
