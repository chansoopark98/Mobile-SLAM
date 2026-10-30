// Read-only static server for the slam_refact XR8 chunk samples (US-010). Node built-ins only.
//
// Nothing is copied: the deployed SDK, the chunk, the engine, three.js and A-Frame are served from where they live, through this fixed
// path map (the samples use these absolute paths):
//   /engine/xr.js                              web/slam-refact-xr/engine/xr.js   (copied SDK)
//   /engine/<chunk file>.js                    slam_refact/web-chunk/<file>   (xr-slam.js, engine-loader.js, ... the 7 chunk files)
//   /engine/slam-refact-engine/<engine file>   <engine dir>/{manifest.json,slam_refact_engine.js,slam_refact_engine.wasm}
//   /three/<file>                              web/slam-refact-xr/three/{three.module.min.js,three.core.min.js}
//   /aframe/aframe-v1.5.0.min.js               web/slam-refact-xr/aframe/aframe-v1.5.0.min.js
//   /samples/...                               slam_refact/web-chunk/samples/...
// xr.js imports `xr-slam.js` from its own directory, so the chunk must be served as /engine/xr-slam.js next to /engine/xr.js.
//
//   node slam_refact/web-chunk/samples/serve.mjs --engine-dir <engine dir> [--port 8080] [--host 127.0.0.1]
// Open http://127.0.0.1:<port>/samples/threejs/index.html or /samples/aframe/index.html (the page asks for the camera and motion
// sensors; a real phone needs HTTPS, which this script does not provide, and starting a server is the user's job).
// Only GET/HEAD of the files above; every other path is 404. Responses are `Cache-Control: no-store`.
import fs from 'node:fs/promises';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

export const CHUNK_FILES = ['xr-slam.js', 'engine-loader.js', 'gr8-compute-pyramid.js', 'gr8-shaders.js', 'image-events.js', 'image-targets.js', 'sensors.js'];
export const ENGINE_FILES = ['manifest.json', 'slam_refact_engine.js', 'slam_refact_engine.wasm'];
const here = path.dirname(fileURLToPath(import.meta.url));
export const REPO_ROOT = path.resolve(here, '../../..');
export const SDK_DIR = path.join(REPO_ROOT, 'web/slam-refact-xr/engine');
export const THREE_DIR = path.join(REPO_ROOT, 'web/slam-refact-xr/three');
export const AFRAME_DIR = path.join(REPO_ROOT, 'web/slam-refact-xr/aframe');
export const DEFAULT_ENGINE_DIR = path.join(REPO_ROOT, 'web/slam-refact-xr/engine/slam-refact-engine');
const TYPES = { '.js': 'text/javascript', '.mjs': 'text/javascript', '.json': 'application/json', '.wasm': 'application/wasm', '.html': 'text/html; charset=utf-8',
  '.css': 'text/css', '.png': 'image/png', '.jpg': 'image/jpeg' };

// Resolves a request path to a file path, or null. `dirs.chunk` and `dirs.engine` default to the repository's.
export function resolveSamplePath(pathname, dirs = {}) {
  const chunkDir = dirs.chunk ?? path.join(REPO_ROOT, 'slam_refact/web-chunk');
  const engineDir = dirs.engine ?? DEFAULT_ENGINE_DIR;
  const samplesDir = dirs.samples ?? path.join(REPO_ROOT, 'slam_refact/web-chunk/samples');
  const inside = (base, rel) => {
    const file = path.resolve(base, rel);
    return file.startsWith(base + path.sep) ? file : null;
  };
  let m;
  if (pathname === '/engine/xr.js') return path.join(SDK_DIR, 'xr.js');
  if ((m = /^\/engine\/slam-refact-engine\/([A-Za-z0-9_.]+)$/.exec(pathname))) return ENGINE_FILES.includes(m[1]) ? path.join(engineDir, m[1]) : null;
  if ((m = /^\/engine\/([A-Za-z0-9_.-]+)$/.exec(pathname))) return CHUNK_FILES.includes(m[1]) ? path.join(chunkDir, m[1]) : null;
  if ((m = /^\/three\/([A-Za-z0-9_.]+)$/.exec(pathname))) return ['three.module.min.js', 'three.core.min.js'].includes(m[1]) ? path.join(THREE_DIR, m[1]) : null;
  if (pathname === '/aframe/aframe-v1.5.0.min.js') return path.join(AFRAME_DIR, 'aframe-v1.5.0.min.js');
  if (pathname.startsWith('/samples/')) {
    const rel = pathname.slice('/samples/'.length);
    if (!rel || rel.includes('\0') || rel.split('/').some((seg) => seg === '..' || seg.startsWith('.'))) return null;
    return inside(samplesDir, rel.endsWith('/') ? rel + 'index.html' : rel);
  }
  return null;
}

// extra(pathname) -> {body, type} | null: routes of the caller (the smoke harness serves its fixture and fake camera frames there).
export function createSampleHandler({ dirs = {}, extra = null } = {}) {
  return async (req, res) => {
    try {
      if (req.method !== 'GET' && req.method !== 'HEAD') { res.writeHead(405); res.end('method not allowed'); return; }
      const pathname = new URL(req.url, 'http://localhost').pathname;
      let body, type;
      const e = extra ? await extra(pathname) : null;
      if (e) { body = e.body; type = e.type; } else {
        const file = resolveSamplePath(pathname, dirs);
        if (!file) { res.writeHead(404); res.end('not found'); return; }
        body = await fs.readFile(file);
        type = TYPES[path.extname(file)] ?? 'application/octet-stream';
      }
      res.writeHead(200, { 'Content-Type': type, 'Cache-Control': 'no-store' });
      res.end(req.method === 'HEAD' ? undefined : body);
    } catch (error) {
      res.writeHead(error.code === 'ENOENT' ? 404 : 500); res.end(String(error.message ?? error));
    }
  };
}

export async function startSampleServer({ host = '127.0.0.1', port = 0, dirs = {}, extra = null } = {}) {
  const server = http.createServer(createSampleHandler({ dirs, extra }));
  await new Promise((resolve, reject) => { server.once('error', reject); server.listen(port, host, resolve); });
  return { server, origin: `http://${host}:${server.address().port}`, close: () => new Promise((resolve) => server.close(resolve)) };
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const { parseArgs } = await import('node:util');
  const { values } = parseArgs({ options: { 'engine-dir': { type: 'string', default: DEFAULT_ENGINE_DIR }, port: { type: 'string', default: '8080' }, host: { type: 'string', default: '127.0.0.1' } } });
  const { origin } = await startSampleServer({ host: values.host, port: Number(values.port), dirs: { engine: path.resolve(values['engine-dir']) } });
  console.log(`serving ${origin}/samples/threejs/index.html and ${origin}/samples/aframe/index.html (Ctrl-C to stop)`);
}
