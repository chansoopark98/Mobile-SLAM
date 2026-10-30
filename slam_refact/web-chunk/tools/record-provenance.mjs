// Provenance record of one chunk test / measurement run (RM-4c m-G): which engine WASM and which chunk sources a result
// belongs to. The RM-4a2 results named no engine build, and the engine sources changed while that gate ran.
//
//   node slam_refact/web-chunk/tools/record-provenance.mjs --engine-dir build/slam-refact-full-chunk-wasm/wasm \
//     [--out build/.../provenance.json]        (prints the JSON when --out is omitted; refuses to overwrite)
//
// Records: sha256 of the engine js / wasm that are actually in --engine-dir, whether they equal the manifest (exit code 1
// when they do not), the manifest sha256 and its build block, sha256 of every file of slam_refact/web-chunk (chunk code,
// tests, tools), git head and the porcelain status of slam_refact/engine and slam_refact/web-chunk.
//
// Engine sources (RM-4d, review RM-4c m5): the engine source state AT BUILD TIME is what the build provenance
// (scripts/dev/build-provenance.py, run right after the build; named in the manifest's build.provenance with its sha256) holds
// as `source_files` / `source_manifest_sha256`. This tool takes that record, checks that its sha256 is the one the manifest
// names, and compares it with the tree as it is NOW (`engineSources.buildScope`: files changed / added / removed since the
// build, same file set as build-provenance.py). `engineSources.atRecord` (every file of slam_refact/engine except caches, one
// sha256 over the sorted `sha256  path` lines) is the state at the moment of recording and is labelled that way: it is not
// the build's. Nothing is written outside build/.
import fs from 'node:fs/promises';
import path from 'node:path';
import { createHash } from 'node:crypto';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { parseArgs } from 'node:util';

const root = process.env.SLAM_REFACT_ROOT ? path.resolve(process.env.SLAM_REFACT_ROOT) : path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../..');
const { values } = parseArgs({ options: { 'engine-dir': { type: 'string' }, out: { type: 'string' } } });
if (!values['engine-dir']) throw new Error('--engine-dir required');
const engineDir = path.resolve(root, values['engine-dir']);
if (!engineDir.startsWith(path.join(root, 'build') + path.sep)) throw new Error('--engine-dir must be under build/');
const sha = (bytes) => createHash('sha256').update(bytes).digest('hex');
const git = (...args) => execFileSync('git', args, { cwd: root, encoding: 'utf8' }).trim();

async function walk(dir, skip, out = []) {
  for (const entry of await fs.readdir(dir, { withFileTypes: true })) {
    const full = path.join(dir, entry.name);
    if (skip.has(entry.name)) continue;
    if (entry.isDirectory()) await walk(full, skip, out);
    else if (entry.isFile()) out.push(full);
  }
  return out;
}
const fileShas = async (dir, skip) => {
  const files = (await walk(dir, skip)).sort();
  const lines = [];
  for (const f of files) lines.push(`${sha(await fs.readFile(f))}  ${path.relative(root, f)}`);
  return lines;
};

// The file set of scripts/dev/build-provenance.py `record`: four named files plus every file under src, include, cmake.
async function buildScopeShas(srcRoot) {
  const rel = new Map();
  for (const f of ['CMakeLists.txt', 'wasm/CMakeLists.txt', 'wasm/vio_bindings.cpp', 'tests/audit_dataset_replay.cpp']) {
    const full = path.join(srcRoot, f);
    try { rel.set(f, sha(await fs.readFile(full))); } catch (error) { if (error.code !== 'ENOENT') throw error; }
  }
  for (const d of ['src', 'include', 'cmake']) {
    try {
      for (const full of await walk(path.join(srcRoot, d), new Set())) rel.set(path.relative(srcRoot, full).split(path.sep).join('/'), sha(await fs.readFile(full)));
    } catch (error) { if (error.code !== 'ENOENT') throw error; }
  }
  return rel;
}

const manifestBytes = await fs.readFile(path.join(engineDir, 'manifest.json'));
const manifest = JSON.parse(manifestBytes);
const actual = {};
for (const key of ['js', 'wasm']) {
  const bytes = await fs.readFile(path.join(engineDir, manifest.artifacts[key].file));
  actual[key] = { file: manifest.artifacts[key].file, sha256: sha(bytes), bytes: bytes.length, manifestSha256: manifest.artifacts[key].sha256 };
}
const matchesManifest = actual.js.sha256 === actual.js.manifestSha256 && actual.wasm.sha256 === actual.wasm.manifestSha256;
const skip = new Set(['__pycache__', 'node_modules', '.pytest_cache']);
const chunkLines = await fileShas(path.join(root, 'slam_refact/web-chunk'), skip);
const engineLines = await fileShas(path.join(root, 'slam_refact/engine'), skip);
let atBuild = null, buildScope = null;
const provenanceRef = manifest.build && manifest.build.provenance;
if (provenanceRef && provenanceRef.file) {
  const provenanceBytes = await fs.readFile(path.resolve(root, provenanceRef.file));
  const provenance = JSON.parse(provenanceBytes);
  const sourceRoot = path.resolve(root, 'slam_refact/engine');
  const now = await buildScopeShas(sourceRoot);
  const then = new Map(Object.entries(provenance.source_files ?? {}));
  const changed = [...then].filter(([f, h]) => now.has(f) && now.get(f) !== h).map(([f]) => f);
  const removed = [...then].filter(([f]) => !now.has(f)).map(([f]) => f);
  const added = [...now].filter(([f]) => !then.has(f)).map(([f]) => f);
  atBuild = { provenanceFile: provenanceRef.file, provenanceSha256: sha(provenanceBytes), manifestNamesThisProvenance: sha(provenanceBytes) === provenanceRef.sha256,
    sourceManifestSha256: provenance.source_manifest_sha256 ?? null, files: then.size };
  buildScope = { changedSinceBuild: changed, addedSinceBuild: added, removedSinceBuild: removed, clean: !changed.length && !added.length && !removed.length };
}
const record = {
  schema: 'slam-refact-chunk-provenance-v2',
  createdAt: new Date().toISOString(),
  engine: { dir: path.relative(root, engineDir), js: actual.js, wasm: actual.wasm, matchesManifest, manifestSha256: sha(manifestBytes), manifestBuild: manifest.build ?? null },
  engineSources: { atBuild, buildScope, atRecord: { files: engineLines.length, sha256: sha(engineLines.join('\n')) } },
  chunk: { files: Object.fromEntries(chunkLines.map((l) => { const [h, f] = l.split('  '); return [f, h]; })) },
  git: { head: git('rev-parse', 'HEAD'), engineStatusSha256: sha(git('status', '--porcelain', '--', 'slam_refact/engine')),
    chunkStatusSha256: sha(git('status', '--porcelain', '--', 'slam_refact/web-chunk')) },
};
const text = JSON.stringify(record, null, 1) + '\n';
if (values.out) {
  const out = path.resolve(root, values.out);
  if (!out.startsWith(path.join(root, 'build') + path.sep)) throw new Error('--out must be under build/');
  await fs.writeFile(out, text, { flag: 'wx' });
  console.log(JSON.stringify({ out: path.relative(root, out), wasm: actual.wasm.sha256, matchesManifest }));
} else {
  process.stdout.write(text);
}
process.exitCode = matchesManifest ? 0 : 1;
