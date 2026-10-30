// node --test slam_refact/web-chunk/tests/rm4b-residual.test.mjs
// RM-4b residual (full-parity plan r2 section 4, spec 07 section 11): configure() reports the options this engine cannot honour, but an
// empty mapSrcUrl is not one of them. A-Frame's xrweb component passes its schema default `mapSrcUrl: ""` with every configure call
// (aframe/common.ts:46, xr-web-component.ts:130-137); the original keeps the value and sends a map URL to the engine only when it is
// truthy (tracking-controller.ts:1275-1279).
//   SLAM_REFACT_CHUNK_DIR / SLAM_REFACT_CHUNK_ENGINE_DIR as in chunk-contract.test.mjs
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';
import test from 'node:test';

const repo = path.resolve(import.meta.dirname, '../../..');
const chunkDir = process.env.SLAM_REFACT_CHUNK_DIR ?? path.join(repo, 'slam_refact/web-chunk');
const engineDir = process.env.SLAM_REFACT_CHUNK_ENGINE_DIR ?? path.join(repo, 'build/slam-refact-full-20261019/chunk/rm4d-20261020/engine-pinned');

// the smallest browser surface xr-slam.js needs to load and answer configure() (the same shape as chunk-contract.test.mjs installGlobals)
globalThis.window = globalThis;
globalThis.addEventListener = () => {};
globalThis.removeEventListener = () => {};
globalThis._c8 = { checkAllZeroes: 0 };
globalThis.XR8 = { isPaused: () => false, XrDevice: { deviceEstimate: () => ({}), isDeviceBrowserCompatible: () => true },
  XrConfig: { camera: () => ({ BACK: 'back' }), device: () => ({ MOBILE: 'mobile' }) }, XrPermissions: { permissions: () => ({}) } };
const engineUrlDir = new URL('slam-refact-engine/', pathToFileURL(chunkDir + '/')).href;
globalThis.fetch = async (url) => {
  const href = String(url);
  if (!href.startsWith(engineUrlDir)) throw new Error(`unexpected fetch ${href}`);
  const bytes = fs.readFileSync(path.join(engineDir, href.slice(engineUrlDir.length)));
  return { ok: true, status: 200, arrayBuffer: async () => bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) };
};
globalThis.Blob = class { constructor(parts) { this.parts = parts; } };
URL.createObjectURL = (blob) => `data:text/javascript;base64,${Buffer.concat(blob.parts.map((p) => Buffer.from(typeof p === 'string' ? p : new Uint8Array(p)))).toString('base64')}`;
URL.revokeObjectURL = () => {};
globalThis.require = createRequire(path.join(engineDir, 'slam_refact_engine.js'));
globalThis.__dirname = engineDir;

// a fresh factory (module instance) per test: it keeps per-instance warn-once state
async function controller(tag) {
  const mod = await import(`${pathToFileURL(path.join(chunkDir, 'xr-slam.js')).href}?rm4b=${tag}`);
  return mod.XrControllerFactory({ paused: false }, Promise.resolve({}), () => () => {}, () => {});
}
async function warningsOf(tag, args) {
  const ctrl = await controller(tag);
  const warns = [], errors = [];
  const w0 = console.warn, e0 = console.error;
  console.warn = (...a) => warns.push(a.join(' '));
  console.error = (...a) => errors.push(a.join(' '));
  try { ctrl.configure(args); } finally { console.warn = w0; console.error = e0; }
  return { warns, errors, recorded: globalThis.__slamRefactChunk.warnings.slice() };
}
const NOT_SUPPORTED = /configure options not supported by this engine: (.*)$/;

test('RM4B-1: the A-Frame xrweb configure call (mapSrcUrl "") produces no warning and no error', async () => {
  const r = await warningsOf('xrweb', { disableWorldTracking: false, scale: 'responsive', enableVps: false, projectWayspots: [], mapSrcUrl: '' });
  assert.deepEqual(r.warns, []);
  assert.deepEqual(r.errors, []);
  assert.deepEqual(r.recorded, []);
});

test('RM4B-2: a non-empty mapSrcUrl is still reported, by name, once', async () => {
  const r = await warningsOf('map', { mapSrcUrl: 'https://example.invalid/map.bin' });
  assert.equal(r.warns.length, 1, r.warns.join('|'));
  assert.equal(r.warns[0].match(NOT_SUPPORTED)[1], 'mapSrcUrl');
  assert.deepEqual(r.recorded, r.warns);
});

test('RM4B-3: the other options this engine cannot honour are reported as before, with or without an empty mapSrcUrl', async () => {
  const r = await warningsOf('others', { mapSrcUrl: '', enableJpgEncoding: true, enableDataRecorder: false, enableAreaTargets: true });
  assert.equal(r.warns.filter((w) => NOT_SUPPORTED.test(w)).length, 1);
  assert.equal(r.warns.find((w) => NOT_SUPPORTED.test(w)).match(NOT_SUPPORTED)[1], 'enableJpgEncoding, enableDataRecorder, enableAreaTargets');
});
