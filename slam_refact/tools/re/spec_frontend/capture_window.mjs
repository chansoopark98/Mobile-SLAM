// Divergence lane: WINDOW variant of capture_pyramid.mjs (same instrumentation and input path).
// Differences: (1) pyramid readbacks are kept only from the --from-stage-th stageFrame call on
// (stage call 1 = bootstrap, so stage s draws roughly input frame s-2 minus dropped frames; the exact
// frame map is computed afterwards from videoTime as in capture_pyramid.mjs); (2) no camera-texture
// captures; (3) the original's own per-frame publication (window.__probe.rows, the rows
// run-reference.mjs turns into trace.jsonl) is saved as trace.jsonl in the run-reference schema, so this
// capture run is its own pose reference with known pyramids and known sensor binding.
// Output: build/slam-refact-20261016/divergence/capture/<scenario>/<label>/ (never overwritten).
//   node slam_refact/tools/re/spec_frontend/capture_window.mjs --scenario tum-room1-full --label w1 \
//        --max-frames 170 --from-stage 110 --max-captures 60 --rows 660
//
// Original header of capture_pyramid.mjs follows.
// Spec lane S1: capture the GPU pyramid readback of the ORIGINAL reference runtime engine (xr.js + xr-slam.js,
// bytes unmodified) running headless (Chrome + SwiftShader), with the same input path as lane C
// (slam_refact/tools/re/runtime/page-runtime.js is served as-is, not modified).
//
// Instrumentation (init script, page side only):
//  - WebAssembly.instantiate(Streaming) wrapper (same technique as lane C wasm-wrap.js) that wraps the
//    xr-slam exports c8EmAsm_stageFrame / c8EmAsm_processStagedFrame / c8EmAsm_engineInit to record their
//    arguments and call order.
//  - WebGL2RenderingContext.getBufferSubData / readPixels wrappers that record every pyramid readback
//    (which export call it happens in) and copy the first <rows> rows of up to <max-captures> readbacks.
//
//   node slam_refact/tools/re/spec_frontend/capture_pyramid.mjs --scenario tum-room1-full --label c1 \
//        --max-frames 40 --max-captures 24 --rows 660
//
// Output: build/slam-refact-20261015/spec/frontend/capture/<scenario>/<label>/
//   meta.json  (environment, stageFrame args per call, readback events, capture -> input frame map)
//   cap-<k>.rgba.gz (raw RGBA, 1024 x rows, row 0 = GL row 0)
import fs from 'node:fs/promises';
import path from 'node:path';
import http from 'node:http';
import zlib from 'node:zlib';
import os from 'node:os';
import {createHash} from 'node:crypto';
import {fileURLToPath} from 'node:url';
import {parseArgs} from 'node:util';
import {chromium} from '../../../../scripts/dev/node_modules/playwright/index.mjs';
import {parsePGM} from '../../../../scripts/dev/replay-android-input.mjs';
import {buildSensorPackets} from '../../../../scripts/dev/references/xr_reference/sensor_adapter.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
// Main checkout: datasets, assets and node_modules are gitignored and live only there.
const root = path.resolve(here, '../../../..');
const laneC = path.resolve(here, '../runtime');
const engineDir = path.join(root, 'assets/references/xr_reference/analysis-runtime/standalone');
const XR_SLAM_WASM_SHA256 = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422';
const UA = {
  pixel7: 'Mozilla/5.0 (Linux; Android 13; Pixel 7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/140.0.0.0 Mobile Safari/537.36',
  reduced: 'Mozilla/5.0 (Linux; Android 10; K) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/140.0.0.0 Mobile Safari/537.36',
};
const {values} = parseArgs({options: {
  scenario: {type: 'string'}, label: {type: 'string'},
  'max-frames': {type: 'string', default: '40'}, 'max-captures': {type: 'string', default: '24'},
  rows: {type: 'string', default: '660'}, 'from-stage': {type: 'string', default: '1'},
  memprobe: {type: 'boolean', default: false}, 'wp-every': {type: 'string', default: '0'},
  // --memprobe-detail N: memprobe.js detailUntilSeq (MapTracker pose and point pool positions, exact bits) for seq <= N.
  'memprobe-detail': {type: 'string', default: '0'},
  // --probe-wasm <path>: DIAGNOSTIC ONLY. memprobe compiles this instrumented build of the original instead of its
  // bytes (fix lane: func 1146 copies its inputs, see memprobe.js probeWasmUrl). The run is then not an original run.
  'probe-wasm': {type: 'string'},
}});
const scenarios = JSON.parse(await fs.readFile(path.join(laneC, 'scenarios.json'), 'utf8'));
const scenario = scenarios[values.scenario];
if (!scenario) throw new Error('--scenario one of ' + Object.keys(scenarios).join('|'));
if (!/^[a-z0-9-]+$/.test(values.label || '')) throw new Error('--label [a-z0-9-]+ required (new label per attempt)');
const out = path.join(root, 'build/slam-refact-20261016/divergence/capture', values.scenario, values.label);
await fs.mkdir(path.dirname(out), {recursive: true});
await fs.mkdir(out, {recursive: false});
const sha = bytes => createHash('sha256').update(bytes).digest('hex');

async function loadDataset() {  // same rules as lane C run-reference.mjs loadDataset()
  const dir = path.resolve(root, scenario.dataset);
  if (scenario.kind === 'tum') {
    const input = JSON.parse(await fs.readFile(path.join(dir, 'input.json')));
    return {dir, width: input.config.width, height: input.config.height,
      frames: input.frames.map(f => ({index: f.index, t: f.physicalTimestamp, file: path.join(dir, 'mav0/cam0/data', f.file), sha256: f.sha256})),
      imu: input.imu};
  }
  const record = JSON.parse(await fs.readFile(path.join(dir, 'record.json')));
  const rec = record.recording;
  const firstFrame = rec.frames[0].timestamp;
  const t0 = Math.max(...rec.imu.filter(r => r[0] <= firstFrame).map(r => r[0]));
  const lastFrame = rec.frames.at(-1).timestamp;
  return {dir, width: rec.width, height: rec.height,
    frames: rec.frames.map((f, i) => ({index: i, t: +(f.timestamp - t0).toFixed(9), file: path.join(dir, 'mav0/cam0/data', f.file), sha256: f.sha256})),
    imu: rec.imu.filter(r => r[0] >= t0 && r[0] <= lastFrame + 0.1).map(r => [+(r[0] - t0).toFixed(9), ...r.slice(1)])};
}

const initScript = ({targetSha256, maxCaptures, rows, fromStage}) => {
  const C = window.__pyrCap = {target: targetSha256, calls: [], reads: [], captures: [], seq: 0, stages: 0, current: null, error: null,
    wrapped: false, glInfo: null};
  const nativeInstantiate = WebAssembly.instantiate.bind(WebAssembly);
  const nativeCompile = WebAssembly.compile.bind(WebAssembly);
  const hex = b => [...new Uint8Array(b)].map(x => x.toString(16).padStart(2, '0')).join('');
  const wrapExports = exports => {
    const o = {};
    for (const [name, fn] of Object.entries(exports)) {
      if (typeof fn !== 'function' || !['c8EmAsm_stageFrame', 'c8EmAsm_processStagedFrame', 'c8EmAsm_engineInit',
        'c8EmAsm_setEventQueue'].includes(name)) { o[name] = fn; continue; }
      o[name] = function (...args) {
        const seq = ++C.seq;
        if (name === 'c8EmAsm_stageFrame' && window.__probe?.started) C.stages++;
        const rec = [seq, name, performance.now(), name === 'c8EmAsm_setEventQueue' ? [args[1]] : args];
        C.calls.push(rec);
        const saved = C.current; C.current = {seq, name};
        try { return fn.apply(this, args); } finally { C.current = saved; }
      };
    }
    return o;
  };
  const handle = async (bytes, imports) => {
    const digest = hex(await crypto.subtle.digest('SHA-256', bytes));
    const module = await nativeCompile(bytes);
    if (digest !== targetSha256) return {module, instance: await nativeInstantiate(module, imports)};
    C.wrapped = true;
    const instance = await nativeInstantiate(module, imports);
    return {module, instance: {exports: wrapExports(instance.exports)}};
  };
  WebAssembly.instantiate = async (input, imports) => {
    if (input instanceof WebAssembly.Module) {
      // --memprobe: memprobe.js (installed after this script) compiles the original bytes itself and hands the
      // Module here; wrap it when it is the xr-slam module (its export list), so both probes see every call.
      const names = WebAssembly.Module.exports(input).map(e => e.name);
      if (!names.includes('c8EmAsm_stageFrame')) return nativeInstantiate(input, imports);
      C.wrapped = true;
      const inst = await nativeInstantiate(input, imports);
      return {exports: wrapExports(inst.exports)};
    }
    const bytes = ArrayBuffer.isView(input) ? input.buffer.slice(input.byteOffset, input.byteOffset + input.byteLength) : input;
    try { return await handle(bytes, imports); } catch (e) { C.error = String(e); throw e; }
  };
  WebAssembly.instantiateStreaming = async (response, imports) => {
    const bytes = await (await response).arrayBuffer();
    try { return await handle(bytes, imports); } catch (e) { C.error = String(e); throw e; }
  };
  const proto = WebGL2RenderingContext.prototype;
  const origGet = proto.getBufferSubData;
  proto.getBufferSubData = function (target, srcOffset, dst, ...rest) {
    const r = origGet.call(this, target, srcOffset, dst, ...rest);
    if (dst && dst.length >= 1024 * 1024 * 4) {
      const ev = {kind: 'getBufferSubData', length: dst.length, call: C.current ? C.current.seq : null,
        callName: C.current ? C.current.name : null, at: performance.now(), capture: null};
      if (window.__probe?.started && C.stages - 1 >= fromStage && C.captures.length < maxCaptures) {
        ev.capture = C.captures.length;
        C.captures.push(dst.slice(0, rows * 1024 * 4));
      }
      C.reads.push(ev);
    }
    return r;
  };
  // Camera texture uploads (texImage2D from a video element / VideoFrame). The uploaded texture is read
  // back through a temporary framebuffer; all touched bindings are restored.
  C.cameraUploads = [];
  C.cameraCaptures = [];
  const origTexImage2D = proto.texImage2D;
  const origReadPixels = proto.readPixels;
  proto.texImage2D = function (...a) {
    const r = origTexImage2D.apply(this, a);
    const src = a.length === 6 ? a[5] : null;
    const isVideo = src && (src instanceof HTMLVideoElement || (typeof VideoFrame !== 'undefined' && src instanceof VideoFrame));
    if (isVideo) {
      const w = src.videoWidth || src.displayWidth, h = src.videoHeight || src.displayHeight;
      const ev = {afterCall: C.seq, at: performance.now(), w, h, capture: null};
      if (false) {
        const gl = this;
        const tex = gl.getParameter(gl.TEXTURE_BINDING_2D);
        const prevFb = gl.getParameter(gl.FRAMEBUFFER_BINDING);
        const prevPack = gl.getParameter(gl.PIXEL_PACK_BUFFER_BINDING);
        const prevAlign = gl.getParameter(gl.PACK_ALIGNMENT);
        const fb = gl.createFramebuffer();
        gl.bindFramebuffer(gl.FRAMEBUFFER, fb);
        gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, tex, 0);
        gl.bindBuffer(gl.PIXEL_PACK_BUFFER, null);
        gl.pixelStorei(gl.PACK_ALIGNMENT, 4);
        const arr = new Uint8Array(w * h * 4);
        origReadPixels.call(gl, 0, 0, w, h, gl.RGBA, gl.UNSIGNED_BYTE, arr);
        gl.pixelStorei(gl.PACK_ALIGNMENT, prevAlign);
        gl.bindBuffer(gl.PIXEL_PACK_BUFFER, prevPack);
        gl.bindFramebuffer(gl.FRAMEBUFFER, prevFb);
        gl.deleteFramebuffer(fb);
        ev.capture = C.cameraCaptures.length;
        C.cameraCaptures.push(arr);
      }
      C.cameraUploads.push(ev);
    }
    return r;
  };
  const origRead = proto.readPixels;
  proto.readPixels = function (x, y, w, h, format, type, dst, ...rest) {
    const r = origRead.call(this, x, y, w, h, format, type, dst, ...rest);
    if (w === 1024 && h === 1024) {
      C.reads.push({kind: typeof dst === 'number' ? 'readPixelsToPackBuffer' : 'readPixels', call: C.current ? C.current.seq : null,
        callName: C.current ? C.current.name : null, at: performance.now()});
      if (!C.glInfo) {
        const ext = this.getExtension('WEBGL_debug_renderer_info');
        C.glInfo = {version: this.getParameter(this.VERSION), renderer: ext ? this.getParameter(ext.UNMASKED_RENDERER_WEBGL) : null,
          shadingLanguage: this.getParameter(this.SHADING_LANGUAGE_VERSION),
          highpFloatFragment: (() => { const p = this.getShaderPrecisionFormat(this.FRAGMENT_SHADER, this.HIGH_FLOAT); return p && [p.rangeMin, p.rangeMax, p.precision]; })(),
          mediumpFloatFragment: (() => { const p = this.getShaderPrecisionFormat(this.FRAGMENT_SHADER, this.MEDIUM_FLOAT); return p && [p.rangeMin, p.rangeMax, p.precision]; })()};
      }
    }
    return r;
  };
};

const report = {schema: 'slam-refact-spec-frontend-pyramid-capture-v1', scenario: values.scenario, label: values.label, argv: process.argv,
  startedAt: new Date().toISOString(), node: process.version, loadAvgAtStart: os.loadavg(), blockedExternal: [], pageErrors: []};
let server, browser;
try {
  const data = await loadDataset();
  const frames = data.frames.slice(0, Number(values['max-frames']));
  const imu = data.imu.filter(r => r[0] <= frames.at(-1).t + 0.05);
  const sensors = buildSensorPackets(imu);
  const dts = imu.slice(1).map((r, i) => r[0] - imu[i][0]).sort((a, b) => a - b);
  const intervalMs = +(dts[Math.floor(dts.length / 2)] * 1000).toFixed(3);
  const frameDts = frames.slice(1).map((f, i) => f.t - frames[i].t).sort((a, b) => a - b);
  const fixture = {config: {width: data.width, height: data.height}, nominalFps: Math.round(1 / frameDts[Math.floor(frameDts.length / 2)]),
    scale: scenario.scale || 'responsive', worldPointSampleEvery: Number(values['wp-every']),
    frames: frames.map(f => ({index: f.index, physicalTimestamp: f.t})), sensorPackets: sensors.packets.map(p => ({...p, intervalMs}))};
  const stateScript = `window.__probe={ready:false,started:false,done:false,errors:[],events:[],rows:[],frames:[],imuDispatched:0,cameraRequests:[],intrinsicsChanges:[],worldPointSamples:[]};`;
  const html = `<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>pyramid capture</title><script src="/state.js"></script><script src="/page-runtime.js" defer></script><script src="/engine/xr.js" async data-preload-chunks="slam"></script></head><body style="margin:0"><button id="start" style="position:absolute">start</button><canvas id="view" width="${data.width}" height="${data.height}"></canvas></body></html>`;
  // --memprobe needs the page-runtime.js / memprobe.js of the main checkout (probeSeq callback column).
  const runtimeDir = values.memprobe ? path.join(root, 'slam_refact/tools/re/runtime') : laneC;
  const runtimeBytes = await fs.readFile(path.join(runtimeDir, 'page-runtime.js'));
  report.sourceHashes = {'page-runtime.js(laneC)': sha(runtimeBytes), 'capture_pyramid.mjs': sha(await fs.readFile(fileURLToPath(import.meta.url)))};
  const served = {'index.html': Buffer.from(html), 'state.js': Buffer.from(stateScript), 'page-runtime.js': runtimeBytes};
  report.pixelHashMismatches = [];
  server = http.createServer(async (req, res) => {
    try {
      const pathname = new URL(req.url, 'http://localhost').pathname;
      let bytes, type = 'application/octet-stream';
      if (pathname === '/fixture.json') { bytes = Buffer.from(JSON.stringify(fixture)); type = 'application/json'; }
      else if (/^\/gray\/\d+$/.test(pathname)) {
        const f = frames[Number(pathname.split('/').at(-1))];
        bytes = parsePGM(await fs.readFile(f.file), data.width, data.height);
        if (sha(bytes) !== f.sha256) report.pixelHashMismatches.push(f.index);
      } else if (pathname === '/' || Object.hasOwn(served, pathname.slice(1))) {
        const name = pathname === '/' ? 'index.html' : pathname.slice(1);
        bytes = served[name]; type = name.endsWith('.html') ? 'text/html' : 'text/javascript';
      } else if (pathname.startsWith('/engine/')) {
        const file = path.resolve(engineDir, pathname.slice(8));
        if (!file.startsWith(engineDir + path.sep)) throw new Error('unserved path');
        bytes = await fs.readFile(file); type = file.endsWith('.js') ? 'text/javascript' : 'application/octet-stream';
      } else throw new Error('unserved path');
      res.writeHead(200, {'Content-Type': type, 'Cache-Control': 'no-store'}); res.end(bytes);
    } catch (error) { res.writeHead(404); res.end(String(error)); }
  });
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
  const origin = `http://127.0.0.1:${server.address().port}`;
  const args = ['--enable-unsafe-swiftshader', '--autoplay-policy=no-user-gesture-required', '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream'];
  browser = await chromium.launch({executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true, args});
  report.browserVersion = browser.version();
  const context = await browser.newContext({viewport: {width: data.width, height: data.height}, permissions: ['camera', 'microphone'],
    isMobile: true, hasTouch: true, deviceScaleFactor: 1, userAgent: UA[scenario.ua || 'pixel7']});
  const page = await context.newPage();
  await page.addInitScript(initScript, {targetSha256: XR_SLAM_WASM_SHA256, maxCaptures: Number(values['max-captures']), rows: Number(values.rows),
    fromStage: Number(values['from-stage'])});
  let probeWasmBytes = null;
  if (values.memprobe) {
    const probeSource = await fs.readFile(path.join(runtimeDir, 'memprobe.js'), 'utf8');
    report.sourceHashes['memprobe.js(main)'] = sha(Buffer.from(probeSource));
    await page.addInitScript(`${probeSource}\nwindow.__memProbeInit(${JSON.stringify({targetSha256: XR_SLAM_WASM_SHA256, detailUntilSeq: Number(values['memprobe-detail']),
      probeWasmUrl: values['probe-wasm'] ? '/__probe/xr-probe.wasm' : null})});`);
    if (values['probe-wasm']) {
      const probeBytes = await fs.readFile(values['probe-wasm']);
      report.probeWasm = {path: values['probe-wasm'], sha256: sha(probeBytes)};
      probeWasmBytes = probeBytes;
    }
  }
  await page.route('**/*', route => {
    const url = route.request().url();
    if (probeWasmBytes && url === origin + '/__probe/xr-probe.wasm') {
      return route.fulfill({status: 200, contentType: 'application/wasm', body: probeWasmBytes});
    }
    if (!url.startsWith(origin + '/') && !url.startsWith('data:') && !url.startsWith('blob:')) { report.blockedExternal.push(url); return route.abort('blockedbyclient'); }
    return route.continue();
  });
  page.on('pageerror', e => report.pageErrors.push({message: e.message}));
  await page.goto(origin + '/index.html');
  await page.waitForFunction(() => window.__probe?.ready, null, {timeout: 60000});
  await page.locator('#start').click();
  try { await page.waitForFunction(() => window.__probe?.done, null, {timeout: Math.round(frames.at(-1).t * 1000) + 120000, polling: 500}); }
  catch (error) { report.waitError = String(error); }
  const probe = await page.evaluate(() => ({frames: window.__probe.frames, epochWallMs: window.__probe.epochWallMs, events: window.__probe.events.filter(e => e.name === 'onStart'),
    deviceEstimate: window.__probe.deviceEstimate, intrinsicsChanges: window.__probe.intrinsicsChanges, errors: window.__probe.errors}));
  const cap = await page.evaluate(() => {
    const C = window.__pyrCap;
    return {wrapped: C.wrapped, error: C.error, glInfo: C.glInfo, calls: C.calls, reads: C.reads, ncaptures: C.captures.length,
      cameraUploads: C.cameraUploads, ncamera: C.cameraCaptures.length};
  });
  // Map: captured readback inside stageFrame call S reads the pyramid drawn by the previous stageFrame call.
  const byUs = new Map(probe.frames.map(([index, us]) => [us, index]));
  const stageCalls = cap.calls.filter(c => c[1] === 'c8EmAsm_stageFrame');
  const frameOfStage = s => {
    const us = Math.round(s[3][11] * 1e6);
    const idx = byUs.get(us) ?? byUs.get(us - 1) ?? byUs.get(us + 1);
    return idx === undefined ? null : idx;
  };
  const captures = [];
  for (const ev of cap.reads.filter(r => r.capture !== null && r.capture !== undefined)) {
    const k = stageCalls.findIndex(s => s[0] === ev.call);
    const prev = k > 0 ? stageCalls[k - 1] : null;
    captures.push({capture: ev.capture, readInCall: ev.call, readInCallName: ev.callName,
      pyramidDrawnInStageCall: prev ? prev[0] : null, inputFrame: prev ? frameOfStage(prev) : null,
      stageArgs: prev ? {texture: prev[3][0], width: prev[3][1], height: prev[3][2], rotation: prev[3][3],
        alphaBetaGamma: prev[3].slice(4, 7), qwxyz: prev[3].slice(7, 11), videoTimeSeconds: prev[3][11],
        frameTimeSeconds: prev[3][12], timeNanos: prev[3][13]} : null});
  }
  for (let i = 0; i < cap.ncaptures; i++) {
    const b64 = await page.evaluate(i => {
      const a = window.__pyrCap.captures[i];
      let s = '';
      for (let o = 0; o < a.length; o += 0x8000) s += String.fromCharCode.apply(null, a.subarray(o, o + 0x8000));
      return btoa(s);
    }, i);
    const raw = Buffer.from(b64, 'base64');
    await fs.writeFile(path.join(out, `cap-${String(i).padStart(3, '0')}.rgba.gz`), zlib.gzipSync(raw, {level: 6, mtime: 0}));
    captures[i].rawSha256 = sha(raw);
    captures[i].rawBytes = raw.length;
  }
  const cameraCaptures = [];
  for (const up of cap.cameraUploads.filter(u => u.capture !== null)) {
    const stage = stageCalls.find(s => s[0] > up.afterCall) || null;
    const b64 = await page.evaluate(i => {
      const a = window.__pyrCap.cameraCaptures[i];
      let s = '';
      for (let o = 0; o < a.length; o += 0x8000) s += String.fromCharCode.apply(null, a.subarray(o, o + 0x8000));
      return btoa(s);
    }, up.capture);
    const raw = Buffer.from(b64, 'base64');
    await fs.writeFile(path.join(out, `cam-${String(up.capture).padStart(3, '0')}.rgba.gz`), zlib.gzipSync(raw, {level: 6, mtime: 0}));
    cameraCaptures.push({capture: up.capture, w: up.w, h: up.h, uploadAfterCall: up.afterCall, usedByStageCall: stage ? stage[0] : null,
      inputFrame: stage ? frameOfStage(stage) : null, rawSha256: sha(raw)});
  }
  try { await page.evaluate(() => window.__finishProbe?.()); } catch (error) { report.cleanupError = String(error); }
  Object.assign(report, {cameraUploads: cap.cameraUploads.length, cameraCaptures,input: {dataset: data.dir, width: data.width, height: data.height, frames: frames.length}, probe, wrapped: cap.wrapped,
    wrapError: cap.error, glInfo: cap.glInfo, rows: Number(values.rows),
    callOrder: cap.calls.map(c => [c[0], c[1], +c[2].toFixed(3), c[1] === 'c8EmAsm_stageFrame' ? null : c[3]]),
    stageCalls: stageCalls.map(s => ({call: s[0], at: +s[2].toFixed(3), args: s[3], inputFrame: frameOfStage(s)})),
    reads: cap.reads, captures});
  const rowsRaw = await page.evaluate(() => window.__probe.rows);
  {
    const epoch = probe.epochWallMs;
    const byUs2 = new Map(probe.frames.map(([index, us, published, accepted]) => [us, {index, published}]));
    const fresh = new Map();
    for (const r of rowsRaw) {
      const us = Math.round(r[1] * 1e6);
      const m = byUs2.get(us) || byUs2.get(us - 1) || byUs2.get(us + 1);
      if (!r[2] && m && !fresh.has(m.index)) fresh.set(m.index, r);
    }
    const lines = frames.map(f => {
      const r = fresh.get(f.index);
      const ok = r && r[3] !== null && [5, 6, 7, 8, 9, 10, 11].every(k => Number.isFinite(r[k]));
      return JSON.stringify({i: f.index, t: f.t, fresh: !!r, callbackMs: r ? +(r[0] - epoch).toFixed(3) : null,
        status: r ? r[3] : null, reason: r ? r[4] : null, p: ok ? [r[5], r[6], r[7]] : null, q_wxyz: ok ? [r[8], r[9], r[10], r[11]] : null,
        worldPoints: r ? r[12] : null});
    });
    await fs.writeFile(path.join(out, 'trace.jsonl'), lines.join('\n') + '\n');
    await fs.writeFile(path.join(out, 'callbacks-raw.json'), JSON.stringify(rowsRaw));
    if (Number(values['wp-every']) > 0) {
      // Same file/format as run-reference.mjs: published world points of every Nth fresh callback.
      const samples = await page.evaluate(() => window.__probe.worldPointSamples);
      const byUs3 = new Map(probe.frames.map(([index, us]) => [us, index]));
      await fs.writeFile(path.join(out, 'worldpoints-sampled.jsonl'), samples.map(sm => JSON.stringify({
        frameIndex: byUs3.get(Math.round(sm.videoTime * 1e6)) ?? byUs3.get(Math.round(sm.videoTime * 1e6) - 1) ?? byUs3.get(Math.round(sm.videoTime * 1e6) + 1) ?? null,
        ...sm})).join('\n') + '\n');
    }
    if (values.memprobe) {
      // Same association as run-reference.mjs: the first fresh callback of frame i carries the seq of the last
      // processStagedFrame before it (row column 22 = probeSeq); that record gets i.
      const mp = await page.evaluate(() => ({fields: window.__memProbe.fields, records: window.__memProbe.records,
        error: window.__memProbe.error, scans: window.__memProbe.scans, rootHistory: window.__memProbe.rootHistory}));
      const seqToFrame = new Map();
      for (const [i, r] of fresh) if (r[22] != null && !seqToFrame.has(r[22])) seqToFrame.set(r[22], i);
      const tOf = new Map(frames.map(f => [f.index, f.t]));
      const lines = [JSON.stringify({schema: 'slam-refact-xr_reference-memprobe-v1', scenario: values.scenario, label: values.label,
        fields: mp.fields, error: mp.error, scans: mp.scans, rootHistory: mp.rootHistory, records: mp.records.length,
        association: 'capture_window.mjs: i = frame whose first fresh callback has probeSeq == seq'})];
      for (const rec of mp.records) {
        const i = seqToFrame.has(rec[0]) ? seqToFrame.get(rec[0]) : null;
        const detail = rec.length > mp.fields.length ? {mtPose: rec[mp.fields.length], mtDelta: rec[mp.fields.length + 1], mtSubset: rec[mp.fields.length + 2], vtRaw: rec[mp.fields.length + 3], points: rec[mp.fields.length + 4], mtTrack: rec[mp.fields.length + 5], probe: rec[mp.fields.length + 6]} : {};
        lines.push(JSON.stringify({i, t: i === null ? null : tOf.get(i), ...Object.fromEntries(mp.fields.map((k, j) => [k, rec[j] ?? null])), ...detail}));
      }
      await fs.writeFile(path.join(out, 'memprobe.jsonl.gz'), zlib.gzipSync(lines.join('\n') + '\n', {level: 9, mtime: 0}));
      report.memprobe = {records: mp.records.length, associated: seqToFrame.size, error: mp.error};
    }
    report.traceFresh = fresh.size;
  }
  report.status = cap.wrapped && captures.length ? 'COMPLETED' : 'INCOMPLETE';
} catch (error) {
  report.status = 'FAILED'; report.error = {message: error.message, stack: error.stack};
} finally {
  if (browser) await browser.close();
  if (server) await new Promise(resolve => server.close(resolve));
  report.finishedAt = new Date().toISOString();
  report.loadAvgAtEnd = os.loadavg();
  await fs.writeFile(path.join(out, 'meta.json'), JSON.stringify(report, null, 1) + '\n');
  console.log(JSON.stringify({status: report.status, out: path.relative(root, out), captures: report.captures?.length, error: report.error?.message,
    waitError: report.waitError, glInfo: report.glInfo}, null, 1));
  process.exitCode = report.status === 'COMPLETED' ? 0 : 1;
}
