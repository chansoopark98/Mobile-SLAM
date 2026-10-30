// Writes the engine manifest the slam_refact xr-slam chunk verifies before running the engine
// (engine-loader.js): sha256 and size of slam_refact_engine.{js,wasm} in a WASM build directory, plus the
// git context and the build-provenance.py record when given. Output stays under build/.
//   node slam_refact/web-chunk/tools/make-manifest.mjs --engine-dir build/slam-refact-full-chunk-wasm/wasm \
//     [--provenance build/slam-refact-full-20261019/chunk/wasm-provenance.json]
import fs from 'node:fs/promises';
import path from 'node:path';
import { createHash } from 'node:crypto';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { parseArgs } from 'node:util';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../..');
const { values } = parseArgs({ options: { 'engine-dir': { type: 'string' }, provenance: { type: 'string' } } });
if (!values['engine-dir']) throw new Error('--engine-dir required');
const dir = path.resolve(root, values['engine-dir']);
if (!dir.startsWith(path.join(root, 'build') + path.sep)) throw new Error('--engine-dir must be under build/');
const sha = (b) => createHash('sha256').update(b).digest('hex');
const artifact = async (file) => {
  const bytes = await fs.readFile(path.join(dir, file));
  return { file, sha256: sha(bytes), bytes: bytes.length };
};
const git = (...args) => execFileSync('git', args, { cwd: root, encoding: 'utf8' }).trim();
const manifest = {
  schema: 'slam-refact-chunk-engine-v1',
  engine: 'slam_refact',
  artifacts: { js: await artifact('slam_refact_engine.js'), wasm: await artifact('slam_refact_engine.wasm') },
  build: {
    dir: path.relative(root, dir),
    gitHead: git('rev-parse', 'HEAD'),
    engineDirty: git('status', '--porcelain', '--', 'slam_refact/engine') !== '',
    provenance: values.provenance ? { file: values.provenance, sha256: sha(await fs.readFile(path.resolve(root, values.provenance))) } : null,
  },
  createdAt: new Date().toISOString(),
};
const out = path.join(dir, 'manifest.json');
await fs.writeFile(out, JSON.stringify(manifest, null, 1) + '\n');
console.log(JSON.stringify({ out: path.relative(root, out), js: manifest.artifacts.js.sha256, wasm: manifest.artifacts.wasm.sha256 }));
