#!/usr/bin/env node
// Web smoke runner (US-012, architecture.md §7.4): serves this page, the WASM engine
// (build/slam-refact-wasm/wasm) and a short TUM clip from a prepared replay bundle on a one-off
// 127.0.0.1 server with a random port, drives headless Chromium through Playwright
// (scripts/dev/node_modules/playwright), aborts every request that is not to that server, and
// writes build/slam-refact-20261015/web-smoke/<label>/result.json (never overwritten).
// web/ and user servers are not touched.
//
//   node slam_refact/web-smoke/run-smoke.mjs --label <new label> [--frames 40] [--bundle <dir>]
//        [--native-trace <trace.jsonl>]
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { createRequire } from 'node:module';
import { readBundle, readPgm } from '../engine/tests/wasm/bundle_io.mjs';
import { poseDiff } from '../engine/tests/wasm/bundle_replay.mjs';

const here = import.meta.dirname;
const repo = path.resolve(here, '../..');
const E = path.join(repo, 'build/slam-refact-20261015');
const args = Object.fromEntries(process.argv.slice(2).reduce((acc, a, i, all) =>
  (a.startsWith('--') ? [...acc, [a.slice(2), all[i + 1]]] : acc), []));
if (!args.label || !/^[a-z0-9-]+$/.test(args.label)) throw new Error('--label <[a-z0-9-]+> required');
const maxFrames = Number(args.frames ?? 40);
const bundleDir = args.bundle ?? path.join(E, 'harness/bundles/tum-room1-full/wp5b-r2');
const nativeTrace = args['native-trace'] ?? path.join(E, 'engine-runs/tum-room1-full/wp5b-r2/trace.jsonl');
const wasmDir = process.env.SLAM_REFACT_WASM_DIR ?? path.join(repo, 'build/slam-refact-wasm/wasm');
const outDir = path.join(E, 'web-smoke', args.label);
if (fs.existsSync(outDir)) throw new Error(`refusing to overwrite ${outDir}`);

// Clip: bootstrap + first maxFrames input frames, sensors up to the last frame time.
const bundle = readBundle(bundleDir);
const frames = bundle.frames.slice(0, maxFrames + 1);
const lastT = frames.at(-1).timeNs;
const imagePaths = [];
const clipFrames = frames.map((f) => {
  let k = imagePaths.indexOf(f.image);
  if (k < 0) { imagePaths.push(f.image); k = imagePaths.length - 1; }
  return { ...f, image: undefined, imageIndex: k };
});
const first = readPgm(imagePaths[0]);
const clip = {
  width: first.width, height: first.height, maxFrames,
  bundle: { config: bundle.config, sensors: bundle.sensors.filter((s) => s.t <= lastT), frames: clipFrames },
};

const types = { '.html': 'text/html', '.mjs': 'text/javascript', '.js': 'text/javascript',
  '.wasm': 'application/wasm', '.json': 'application/json' };
const files = {
  '/': path.join(here, 'index.html'),
  '/smoke.mjs': path.join(here, 'smoke.mjs'),
  '/shared/bundle_replay.mjs': path.join(repo, 'slam_refact/engine/tests/wasm/bundle_replay.mjs'),
  '/engine/slam_refact_engine.js': path.join(wasmDir, 'slam_refact_engine.js'),
  '/engine/slam_refact_engine.wasm': path.join(wasmDir, 'slam_refact_engine.wasm'),
};
const server = http.createServer((req, res) => {
  const url = new URL(req.url, 'http://127.0.0.1');
  let body = null, type = 'application/octet-stream';
  if (files[url.pathname]) {
    body = fs.readFileSync(files[url.pathname]);
    type = types[path.extname(files[url.pathname])] ?? type;
  } else if (url.pathname === '/clip/clip.json') {
    body = JSON.stringify(clip); type = 'application/json';
  } else if (url.pathname.startsWith('/clip/img/')) {
    const k = Number(url.pathname.slice('/clip/img/'.length));
    if (Number.isInteger(k) && k >= 0 && k < imagePaths.length) body = Buffer.from(readPgm(imagePaths[k]).pixels);
  }
  if (body === null) { res.writeHead(404); res.end(); return; }
  res.writeHead(200, { 'content-type': type, 'cache-control': 'no-store' });
  res.end(body);
});
await new Promise((r) => server.listen(0, '127.0.0.1', r));
const origin = `http://127.0.0.1:${server.address().port}`;

const require = createRequire(path.join(repo, 'scripts/dev/package.json'));
const { chromium } = require('playwright');
const browser = await chromium.launch({ headless: true });
const blocked = [];
let result;
try {
  const page = await browser.newPage();
  await page.route('**/*', (route) => {
    if (route.request().url().startsWith(origin + '/')) return route.continue();
    blocked.push(route.request().url());
    return route.abort();
  });
  const consoleErrors = [];
  page.on('console', (m) => { if (m.type() === 'error') consoleErrors.push(m.text()); });
  await page.goto(origin + '/');
  await page.waitForFunction(() => window.__smokeResult !== undefined, null, { timeout: 600000 });
  const r = await page.evaluate(() => window.__smokeResult);
  if (!r.ok) throw new Error(r.error);
  const native = fs.readFileSync(nativeTrace, 'utf8').split('\n').filter((l) => l).map((l) => JSON.parse(l));
  const fresh = r.rows.filter((x) => x.fresh);
  const diffs = fresh.map((x) => poseDiff(x.pose, { position: native[x.i].p, rotationWxyz: native[x.i].q_wxyz }));
  const finite = fresh.every((x) => [...x.pose.position, ...x.pose.rotationWxyz].every(Number.isFinite));
  const p5 = r.rows.slice(0, 5).every((x) => x.pose && x.pose.position.join() === '0,2,0');
  const wall = fresh.map((x) => x.wallMs).sort((a, b) => a - b);
  const pct = (p) => wall[Math.max(0, Math.ceil(p / 100 * wall.length) - 1)];
  result = {
    schema: 'slam-refact-web-smoke-v1', label: args.label, origin, userAgent: r.userAgent,
    browser: browser.version(), wasmDir, bundleDir, nativeTrace, frames: r.rows.length, fresh: fresh.length,
    checks: { finitePoses: finite, p5FramesZeroToFourAtOrigin: p5, frame5Repeated: r.rows[5]?.pose?.repeated === true,
      statuses: [...new Set(fresh.map((x) => x.pose.status))], externalRequestsBlocked: blocked.length,
      consoleErrors },
    vsNative: { rotMaxDeg: Math.max(...diffs.map((d) => d.rot)), posMax: Math.max(...diffs.map((d) => d.pos)),
      exactFrames: diffs.filter((d) => d.rot === 0 && d.pos === 0).length },
    wallMsPerFrame: { p50: pct(50), p95: pct(95), p99: pct(99), note: 'headless desktop Chromium, not a phone' },
    totalMs: r.totalMs,
  };
  result.pass = finite && p5 && result.checks.frame5Repeated && consoleErrors.length === 0;
} finally {
  await browser.close();
  server.close();
}
fs.mkdirSync(outDir, { recursive: true });
fs.writeFileSync(path.join(outDir, 'result.json'), JSON.stringify(result, null, 1) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ pass: result.pass, out: path.join(outDir, 'result.json'), vsNative: result.vsNative, wall: result.wallMsPerFrame }));
process.exit(result.pass ? 0 : 1);
