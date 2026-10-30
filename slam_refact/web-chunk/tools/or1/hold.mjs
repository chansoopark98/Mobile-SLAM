// First-real-run hold of the OR-1 gate (preregistration-protocol.md section 12.5: the integrity of the first real run is checked by code, not by
// procedure). `or1.mjs run --kind gate` calls checkHold() before it reads any registered gate input, starts an arm or writes anything.
//
// v3: the generic conditions are the shared device slam_refact/tools/prereg/sr_held.mjs (docs/design/sr-held-design.md; through ../gate-common/held.mjs):
// manifest identity and shape, created_at < review.at < now, first_real_run_at null, gate / review / designation file sha256, the environment value =
// the designation file's sha256, the designation's three pins (gate file, review file, manifest before review), the pinned files (held.pins, by base
// name, the set must be exactly the presented one), the registered data (held.inputs[kind] = tree sha256 of the derived bundle directory) and the
// one-run marker `<gate>.v<N>.first-run.<kind>.marker` (O_EXCL, one per registered bundle). Nothing about the gate is a constant of this file: the gate
// id, version, preregistration directory and the NAME of the environment variable are arguments (v2 pinned them in the code, review V-1/V-8).
//
// OR-1 adds around it (all before the marker): the arguments (kind gate, the repo chunk, no --native-bundle (V-3), the pinned engine directory and
// native binary, the three arms, repetitions >= the registered number), the set of harness and chunk files equals the pinned set (an unpinned file
// next to the pinned ones is refused), manifest.pinned (the readable record) equals held.pins (the device's flat form), the derived bundle is the
// registered directory (V-2: its live tree sha256 is held.inputs, its path is datum.derived_dir) and the atlas files hash to the registered set.
//
// WHAT THIS IS NOT: an accident-prevention device, not an authentication (protocol 12.6). Whoever can write the preregistration directory and set
// the environment variable can write a designation file, the review fields and a manifest that satisfy every check; the marker can be deleted by
// hand. It stops a run that is not what the gate registered, by mistake: another bundle, a changed file, another engine, a mutant copy of the
// chunk, a second run.
import fs from 'node:fs';
import path from 'node:path';
import { createHash } from 'node:crypto';
import { SR_HELD_FILE, claim, flatPins, gateMdPath, manifestPath, refuseSealedData, releaseCheck, request, sr } from '../gate-common/held.mjs';

export const sha256 = (bytes) => createHash('sha256').update(bytes).digest('hex');
export const sha256File = (file) => sha256(fs.readFileSync(file));
const sortKeys = (v) => (Array.isArray(v) ? v.map(sortKeys) : v && typeof v === 'object'
  ? Object.fromEntries(Object.keys(v).sort().map((k) => [k, sortKeys(v[k])])) : v);

// "<relative path> <sha256>" lines of the files (sorted), as one sha256: the registered form of a set of input files
export const fileSetSha256 = (root, files) => sha256(files.map((f) => `${f} ${sha256File(path.join(root, f))}\n`).sort().join(''));

const held = (check, why) => new Error(`HELD: ${check}: ${why}`);

// manifest.pinned sections that are path -> sha256 maps of files (engine paths are relative to engine.dir; held_device is keyed by base name)
const FILE_SECTIONS = ['harness', 'gate_common', 'chunk', 'tests', 'docs', 'pair', 'engine', 'native', 'runner'];

// Every pinned file of a manifest as [absolute path, sha256]. The device pin is the file this process loaded (SR_HELD_FILE), the node executable is
// the running one. `root` resolves the relative paths of the manifest.
export function pinnedFiles(root, pinned) {
  const files = [];
  for (const section of FILE_SECTIONS) {
    const entries = pinned?.[section] ?? {};
    for (const rel of Object.keys(entries)) {
      if (section === 'engine' && rel === 'dir') continue;
      files.push([path.join(root, section === 'engine' ? path.join(entries.dir, rel) : rel), entries[rel]]);
    }
  }
  for (const [name, sha] of Object.entries(pinned?.held_device ?? {})) files.push([name === 'sr_held.mjs' ? SR_HELD_FILE : path.join(path.dirname(SR_HELD_FILE), name), sha]);
  if (pinned?.environment?.node_sha256) files.push([process.execPath, pinned.environment.node_sha256]);
  return files;
}

// the environment pin: the node version and the sha256 of the running node executable (the device pins the executable by base name `node`)
export const liveEnvironment = () => ({ node: process.version, node_sha256: sha256File(process.execPath) });

// held.pins as the registration tool writes it (and as the hold requires it): the flat base-name form of manifest.pinned
export const heldPinsOf = (root, pinned) => flatPins(pinnedFiles(root, pinned));

// live listing of a directory: the files a section may contain
const listing = (dir, re) => (fs.existsSync(dir) ? fs.readdirSync(dir).filter((f) => re.test(f)).sort() : []);

export function checkHold({ root, gate, values, derivedMeta, env = process.env, now = new Date(), expectedChunkDir = 'slam_refact/web-chunk',
  harnessDir = 'slam_refact/web-chunk/tools/or1', gateCommonDir = 'slam_refact/web-chunk/tools/gate-common' }) {
  // 1 args that need no manifest
  if (values.kind !== 'gate') throw held('args', '--kind gate is the only kind this check serves');
  if (values.chunkDir && path.resolve(root, values.chunkDir) !== path.resolve(root, expectedChunkDir)) throw held('args', `--chunk-dir ${values.chunkDir}: a modified chunk copy is not the registered chunk`);
  if (values.nativeBundle) throw held('args', `--native-bundle ${values.nativeBundle}: the native arm reads the registered bundle itself (V-3)`);
  // 2 the manifest the device is going to read (plain JSON here; the device parses it strictly and refuses a malformed one below)
  const manifestFile = manifestPath(gate);
  if (!fs.existsSync(manifestFile)) throw held('manifest', `${path.relative(root, manifestFile)} does not exist`);
  let manifest;
  try { manifest = JSON.parse(fs.readFileSync(manifestFile, 'utf8')); } catch (error) { throw held('manifest', `not JSON: ${error.message}`); }
  const reg = manifest.registered_run ?? {};
  const pinned = manifest.pinned ?? {};
  if (path.resolve(root, values.engineDir ?? '') !== path.resolve(root, pinned.engine?.dir ?? '/nonexistent')) throw held('args', `--engine-dir ${values.engineDir} is not the pinned engine directory`);
  if (values.nativeBin && path.resolve(root, values.nativeBin) !== path.resolve(root, Object.keys(pinned.native ?? {})[0] ?? '/nonexistent')) throw held('args', `--native-bin ${values.nativeBin} is not the pinned native binary`);
  if ([...(values.arms ?? '')].sort().join('') !== [...(reg.arms ?? 'CNW')].sort().join('')) throw held('args', `arms ${values.arms} (registered ${reg.arms})`);
  if (!(Number(values.reps) >= (reg.repetitions ?? 3))) throw held('args', `repetitions ${values.reps} < registered ${reg.repetitions}`);
  // 3 the registered datum: this derived bundle, at the registered path
  const bundleDir = path.resolve(root, values.bundle ?? '');
  const datum = (pinned.gate_data ?? []).find((d) => d.derived_sha256 && derivedMeta && d.source === derivedMeta.source);
  if (!datum) throw held('data', `${derivedMeta?.source} is not registered gate data`);
  if (JSON.stringify(sortKeys(datum.source_sha256)) !== JSON.stringify(sortKeys(derivedMeta.sourceSha256))) throw held('data', 'the source bundle differs from the registered one');
  if (bundleDir !== path.resolve(root, datum.derived_dir ?? '/nonexistent')) throw held('data', `the bundle ${values.bundle} is not the registered derived directory ${datum.derived_dir} (V-2)`);
  // 4 the files the run uses are exactly the pinned ones (the device then compares every sha256)
  for (const [section, dir, re] of [['harness', harnessDir, /\.(mjs|sh)$/], ['gate_common', gateCommonDir, /\.mjs$/], ['chunk', expectedChunkDir, /\.js$/]]) {
    const live = listing(path.resolve(root, dir), re).map((f) => path.posix.join(dir, f)).sort();
    const want = Object.keys(pinned[section] ?? {}).sort();
    if (JSON.stringify(live) !== JSON.stringify(want)) throw held('pins', `${section}: the files in ${dir} (${live.length}) are not the pinned set (${want.length})`);
  }
  if (pinned.environment?.node !== process.version) throw held('pins', `node ${process.version} is not the pinned ${pinned.environment?.node}`);
  const files = pinnedFiles(root, pinned);
  let flat;
  try { flat = flatPins(files); } catch (error) { throw held('pins', error.message); }
  if (JSON.stringify(sortKeys(flat)) !== JSON.stringify(sortKeys(manifest.held?.pins ?? null))) throw held('pins', 'manifest.held.pins is not the flat form of manifest.pinned');
  // 5 the device: manifest, review, designation, env, pins, data tree, (marker is claimed later)
  const rq = request(gate, { kind: datum.name, env: env[gate.designationEnv], pins: files.map(([f]) => f), dataDir: bundleDir, now });
  const outcome = releaseCheck(rq);
  // 6 the atlas files outside the bundle directory
  if (values.atlasSetSha256 !== datum.atlas_set_sha256) throw held('data', 'the atlas files differ from the registered set');
  // 7 the device checks the marker only when it claims it; the precheck (and the order of the claim) says so earlier
  const marker = path.join(gate.preregDir, sr.markerName(rq));
  if (fs.existsSync(marker)) throw held('first-run', `${path.relative(root, marker)} exists: the first real run of ${datum.name} was already taken`);
  return { manifest, datum, designationSha: outcome.designationSha256, outcome, rq, log: outcome.log, gate };
}

// Creates the marker exclusively (the device checks everything once more); after this the run has started.
export function claimFirstRun({ hold }) {
  const outcome = claim(hold.rq);
  hold.outcome = outcome;
  hold.log = outcome.log;
  return outcome.marker;
}

// A run that is not a gate run must not read registered gate data (the data stays unseen until the first real run): refused by the content of the
// derived bundle (tree sha256 = held.inputs of some kind, the device's refuseSealed) and by the source bundle's sha256 (pinned.gate_data).
export function assertNotGateData({ root, gate, derivedMeta, bundleDir }) {
  refuseSealedData(gate, path.resolve(root, bundleDir));
  const manifestFile = manifestPath(gate);
  if (!fs.existsSync(manifestFile)) return { registered: false };
  const manifest = JSON.parse(fs.readFileSync(manifestFile, 'utf8'));
  for (const d of manifest.pinned?.gate_data ?? []) {
    if (JSON.stringify(sortKeys(d.source_sha256)) === JSON.stringify(sortKeys(derivedMeta.sourceSha256))) {
      throw new Error(`refused: ${derivedMeta.source} is registered gate data of ${gate.gateId} v${gate.version}; it is only run by the held gate run`);
    }
  }
  return { registered: true };
}

// The values a designation file pins for this manifest (what `or1.mjs pin` prints).
export function designationPins(manifestFileOrText) {
  const bytes = fs.existsSync(manifestFileOrText) ? fs.readFileSync(manifestFileOrText) : Buffer.from(manifestFileOrText);
  const parsed = sr.parseJson(bytes);
  if (parsed.err !== undefined) throw new Error(`manifest is not strict JSON: ${parsed.err}`);
  return { gate_md_sha256: JSON.parse(bytes.toString('utf8')).sha256, manifest_before_review_sha256: sr.canonManifestSha(parsed.json) };
}

// The registered files of a run that its report names by sha256 (V-8: the gate file is the one of THIS version, not a constant).
export function registeredFiles(gate, manifest) {
  const abs = (name) => path.join(gate.preregDir, name);
  const review = manifest.review ?? {};
  return { gateMd: gateMdPath(gate), manifest: manifestPath(gate), designation: abs(review.designation_file ?? ''), review: abs(review.file ?? '') };
}
