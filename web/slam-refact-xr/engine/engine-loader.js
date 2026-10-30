// Engine bytes of the slam_refact xr-slam chunk: slam_refact_engine.{js,wasm} (Emscripten MODULARIZE,
// EXPORT_NAME=SlamRefactEngine, not SINGLE_FILE) next to a manifest written by
// slam_refact/web-chunk/tools/make-manifest.mjs. Same rule as the measurement page worker
// (web/js/slam-refact-worker.js loadEngine): the sha256 of the fetched bytes must equal the manifest,
// and the bytes that were hashed are the bytes executed (a blob module of the JS plus one fixed export
// line, and wasmBinary = the hashed .wasm bytes). Nothing runs when a hash differs.
//
// What the hash check is and is not (RM-4a review of the bypass paths):
//  - It guarantees that the executed bytes are the bytes that were hashed (no second fetch, the Blob copies
//    jsBuf, the wasm bytes are the hashed buffer) and that the js / wasm pair matches the manifest. It does NOT
//    authenticate the manifest: the manifest is fetched from the same directory, so whoever can replace the
//    engine files can replace the manifest too (a build-time pin of the manifest sha256 in the chunk would be
//    needed for that; not done, spec 07 section 3).
//  - The manifest `file` fields are validated before anything is fetched: a relative path of plain segments
//    inside the engine directory (no '..', no absolute path, no scheme, no backslash), so an absolute or
//    cross-origin URL can no longer be made "valid" by a manifest that carries its own hash.
//  - Every fetch uses redirect: 'error' (a redirect of the manifest or an engine file fails the load).
//  - `cache: 'no-store'` makes every page load download the ~3.5 MB wasm again. Kept on purpose (the hashed
//    bytes must be the fresh bytes of this load); recorded as a limitation in spec 07 section 3.

async function sha256Hex(buf) {
  const d = new Uint8Array(await crypto.subtle.digest('SHA-256', buf));
  return Array.from(d, (b) => b.toString(16).padStart(2, '0')).join('');
}

async function fetchBytes(url) {
  // redirect: 'error' (RM-4a2): the manifest and the engine files must come from the URLs this loader built; the hash only
  // proves the bytes equal the manifest, so a redirect to another origin would let that origin choose both.
  const r = await fetch(url, { cache: 'no-store', credentials: 'same-origin', redirect: 'error' });
  if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
  return r.arrayBuffer();
}

export const ENGINE_DIR = 'slam-refact-engine/';
export const MANIFEST_SCHEMA = 'slam-refact-chunk-engine-v1';

// A segment starts with a letter, digit or '_' (so '.', '..' and hidden names are impossible) and contains only
// [A-Za-z0-9_.-] (no ':' scheme, no backslash, no percent escapes).
const FILE_SEGMENT = /^[A-Za-z0-9_][A-Za-z0-9_.-]*$/;
const SHA256_HEX = /^[0-9a-f]{64}$/;

// Returns the artifact's absolute URL inside baseUrl, or throws without fetching anything.
export function resolveManifestFile(baseUrl, entry, what) {
  const file = entry && entry.file;
  if (typeof file !== 'string' || file.length === 0 || file.length > 128 || !file.split('/').every((seg) => FILE_SEGMENT.test(seg))) {
    throw new Error(`[XR] slam_refact engine manifest: ${what}.file must be a relative path inside the engine directory`);
  }
  if (typeof entry.sha256 !== 'string' || !SHA256_HEX.test(entry.sha256)) {
    throw new Error(`[XR] slam_refact engine manifest: ${what}.sha256 must be 64 lowercase hex digits`);
  }
  const url = new URL(file, baseUrl);
  if (!url.href.startsWith(new URL(baseUrl).href)) throw new Error(`[XR] slam_refact engine manifest: ${what}.file leaves the engine directory`);
  return url;
}

// baseUrl: directory URL of the engine files (ends with '/'). moduleArgs: Emscripten Module overrides
// (print, printErr, onAbort). Returns {Module, loaded}; throws with error.loaded on a hash mismatch.
export async function loadEngine(baseUrl, moduleArgs = {}) {
  const manifest = JSON.parse(new TextDecoder().decode(await fetchBytes(new URL('manifest.json', baseUrl))));
  if (manifest.schema !== MANIFEST_SCHEMA) throw new Error(`engine manifest schema ${manifest.schema} != ${MANIFEST_SCHEMA}`);
  const { js, wasm } = manifest.artifacts ?? {};
  const jsUrl = resolveManifestFile(baseUrl, js, 'artifacts.js'), wasmUrl = resolveManifestFile(baseUrl, wasm, 'artifacts.wasm');
  const [jsBuf, wasmBuf] = await Promise.all([fetchBytes(jsUrl), fetchBytes(wasmUrl)]);
  const loaded = { jsSha256: await sha256Hex(jsBuf), wasmSha256: await sha256Hex(wasmBuf), jsBytes: jsBuf.byteLength,
    wasmBytes: wasmBuf.byteLength, manifest: { jsSha256: js.sha256, wasmSha256: wasm.sha256, build: manifest.build ?? null } };
  loaded.matchesManifest = loaded.jsSha256 === js.sha256 && loaded.wasmSha256 === wasm.sha256;
  if (!loaded.matchesManifest) {
    throw Object.assign(new Error('[XR] slam_refact engine bytes do not match the chunk engine manifest'), { loaded });
  }
  const blobUrl = URL.createObjectURL(new Blob([jsBuf, '\nexport default SlamRefactEngine;\n'], { type: 'text/javascript' }));
  let factory;
  try { factory = (await import(blobUrl)).default; } finally { URL.revokeObjectURL(blobUrl); }
  const t0 = performance.now();
  const Module = await factory({ ...moduleArgs, wasmBinary: new Uint8Array(wasmBuf) });
  loaded.instantiateMs = performance.now() - t0;
  return { Module, loaded };
}
