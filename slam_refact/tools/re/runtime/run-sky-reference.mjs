#!/usr/bin/env node
// Original reference runtime sky runtime (standalone xr.js + xr-slam.js + semantics-worker.js, original WASM
// and semantics model) on synthetic or recorded frames in headless Chromium, recording what
// slam_refact's sky pipeline needs for an A/B comparison (US-020; docs/spec/09-sky.md §6):
//   - the camera texture of every non-repeat processed frame (cam/cam-%04d.rgba), read back from
//     the rendering context before LayersController resizes it -> exact input of the closed loop;
//   - the 144x256 RGBA image LayersController hands to the semantics worker (invoke/invoke-%04d.img.rgba),
//     the worker's answer (float32 sky channel, invoke-%04d.data.f32), the rotation and position
//     LayersController sent with it, and the worker's own percentage (invoke-%04d.json):
//     the exact input and output of the model, i.e. the open-loop pair;
//   - the invokeData (resize result, per processed frame) the layers module computed (resized/);
//   - the layers.sky output texture after every non-repeat update (sky/sky-%04d.rgba, r g b a) with
//     the response intrinsics / rotation / position and percentage (updates in result.json);
//   - every layerscontroller.* event with its detail.
// Device orientation is injected deterministically: this script's capture module runs BEFORE the
// layers module and dispatches window 'deviceorientation' for processed frame k (alpha/beta/gamma of
// meta.orientations[k]) synchronously, so the layers module reads exactly that value.
// No network: only a 127.0.0.1 temporary server owned by this script; other origins are blocked.
// Nothing is written outside <out-root>/<label>/ (default
// build/slam-refact-full-20261019/evidence/US-020/reference/<label>/).
//
// Usage: node run-sky-reference.mjs --frames <dir> --label <new label> [--frame-ms 250]
//          [--sky-config '{"invertLayerMask":false,"edgeSmoothness":0.5}'] [--tail-ms 3000]
//          [--out-root <dir>] [--near 0.01] [--far 1000] [--mirrored 0|1]

import fs from 'node:fs/promises';
import http from 'node:http';
import path from 'node:path';
import {createHash} from 'node:crypto';
import {fileURLToPath} from 'node:url';
import {parseArgs} from 'node:util';
import {chromium} from '../../../../scripts/dev/node_modules/playwright/index.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(here, '../../../..');
const {values} = parseArgs({options: {
  frames: {type: 'string'},
  label: {type: 'string'},
  'frame-ms': {type: 'string', default: '250'},
  'tail-ms': {type: 'string', default: '3000'},
  'sky-config': {type: 'string', default: '{"invertLayerMask":false,"edgeSmoothness":0.5}'},
  'out-root': {type: 'string'},
  near: {type: 'string', default: ''},
  far: {type: 'string', default: ''},
  mirrored: {type: 'string', default: '0'},
  // 1: Android Chrome user agent with an unknown model ("K": default intrinsics 506.4104) and a
  // mobile portrait context, so XR8 reports screen orientation 0 (desktop browsers get 90, see
  // camera-loop.ts windowOrientation). 0: plain desktop Chrome (orientation 90).
  mobile: {type: 'string', default: '1'},
}});
if (!values.frames) throw new Error('--frames <dir> required');
if (!/^[a-z0-9-]+$/.test(values.label || '')) throw new Error('--label required (new, [a-z0-9-])');
const framesDir = path.resolve(values.frames);
const outRoot = values['out-root'] ? path.resolve(values['out-root'])
  : path.join(root, 'build/slam-refact-full-20261019/evidence/US-020/reference');
const out = path.join(outRoot, values.label);
await fs.mkdir(outRoot, {recursive: true});
await fs.mkdir(out, {recursive: false});
for (const d of ['cam', 'resized', 'invoke', 'sky']) await fs.mkdir(path.join(out, d));
const engine = path.join(root, 'assets/references/xr_reference/analysis-runtime/standalone');
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const meta = JSON.parse(await fs.readFile(path.join(framesDir, 'meta.json'), 'utf8'));
const skyConfig = JSON.parse(values['sky-config']);

const report = {
  schema: 'slam-refact-sky-reference-v1', label: values.label, startedAt: new Date().toISOString(),
  argv: process.argv, node: process.version, frames: framesDir,
  meta: {...meta, orientations: undefined, orientationCount: meta.orientations.length},
  frameMs: Number(values['frame-ms']), skyConfig,
  near: values.near === '' ? null : Number(values.near), far: values.far === '' ? null : Number(values.far),
  mirrored: values.mirrored === '1',
  engineAssets: {}, console: [], pageErrors: [], blockedExternal: [],
  limits: [
    'Desktop headless Chromium, SwiftShader WebGL: GPU render of SwiftShader, not a phone GPU',
    'Synthetic frames through MediaStreamTrackGenerator (or recorded gray phone frames); the recorded camera texture is the input',
    'A synthetic sky is not a real sky: no segmentation accuracy statement follows from this record',
  ],
};
for (const name of ['xr.js', 'xr-slam.js', 'resources/semantics-worker.js', 'resources/semantics-model.tflite']) {
  const bytes = await fs.readFile(path.join(engine, name));
  report.engineAssets[name] = {bytes: bytes.length, sha256: sha(bytes)};
}
report.frameSha256 = [];
for (let i = 0; i < meta.count; i++) {
  report.frameSha256.push(sha(await fs.readFile(path.join(framesDir, `frame-${String(i).padStart(4, '0')}.rgba`))));
}

const pageJs = `(async () => {
  const S = window.__sky = {ready: false, done: false, errors: [], events: [], updates: [], invokes: [],
    cams: [], staged: 0, resizedCount: 0, backend: null, orientationLog: []};
  window.addEventListener('error', e => S.errors.push({kind: 'window', message: e.message}));
  window.addEventListener('unhandledrejection', e => S.errors.push({kind: 'promise', message: String(e.reason)}));
  const cfg = await (await fetch('/config.json')).json();
  const W = cfg.meta.width, H = cfg.meta.height;
  const b64 = u8 => { let s = ''; const n = 0x8000; for (let i = 0; i < u8.length; i += n) s += String.fromCharCode.apply(null, u8.subarray(i, i + n)); return btoa(s); };
  // Worker hook: record every invoke message (input, rotation) and every answer (data, percentage).
  const OrigWorker = window.Worker;
  window.Worker = class extends OrigWorker {
    constructor(...a) { super(...a); S.workerCreated = true; }
    postMessage(msg, transfer) {
      if (msg && msg.type === 'invoke') {
        const k = S.invokes.length;
        S.invokes.push({k, sentAtStaged: S.staged, sentAt: performance.now(), rotation: msg.rotation, position: msg.position});
        window.__save('invoke-img', k, b64(new Uint8Array(msg.img.slice(0))));
      }
      return super.postMessage(msg, transfer);
    }
    set onmessage(h) {
      super.onmessage = e => {
        const d = e.data;
        if (d && d.type === 'invoke' && d.output) {
          const rec = S.invokes.find(r => r.answeredAt === undefined);
          if (rec) {
            rec.answeredAt = performance.now(); rec.answeredAtStaged = S.staged;
            rec.percentage = d.output.percentage; rec.debug = d.output.debug;
            rec.outRotation = d.output.rotation; rec.outPosition = d.output.position;
            window.__save('invoke-data', rec.k, b64(new Uint8Array(d.output.data.slice(0))));
          }
        } else if (d && d.type === 'error') { S.errors.push({kind: 'worker', message: JSON.stringify(d)}); }
        return h.call(this, e);
      };
    }
    get onmessage() { return super.onmessage; }
  };
  const canvas = document.createElement('canvas'); canvas.width = W; canvas.height = H;
  const ctx = canvas.getContext('2d');
  const track = new MediaStreamTrackGenerator({kind: 'video'});
  const writer = track.writable.getWriter();
  const stream = new MediaStream([track]);
  const settings = track.getSettings.bind(track);
  Object.defineProperty(track, 'getSettings', {value: () => ({...settings(), width: W, height: H, frameRate: 20, facingMode: 'environment'})});
  navigator.mediaDevices.getUserMedia = async () => stream;
  const frames = [];
  for (let i = 0; i < cfg.meta.count; i++) frames.push(new Uint8ClampedArray(await (await fetch('/frames/' + i)).arrayBuffer()));
  const write = async (i, ts) => {
    ctx.putImageData(new ImageData(frames[i], W, H), 0, 0);
    const vf = new VideoFrame(canvas, {timestamp: ts});
    try { await writer.write(vf); } finally { vf.close(); }
  };
  let fbo = null;
  const readTexture = (gl, tex, w, h) => {
    const prevFb = gl.getParameter(gl.FRAMEBUFFER_BINDING);
    const prevPackBuf = gl.PIXEL_PACK_BUFFER !== undefined ? gl.getParameter(gl.PIXEL_PACK_BUFFER_BINDING) : null;
    const prevAlign = gl.getParameter(gl.PACK_ALIGNMENT);
    if (!fbo) fbo = gl.createFramebuffer();
    gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, tex, 0);
    if (gl.PIXEL_PACK_BUFFER !== undefined) gl.bindBuffer(gl.PIXEL_PACK_BUFFER, null);
    gl.pixelStorei(gl.PACK_ALIGNMENT, 4);
    const px = new Uint8Array(w * h * 4);
    gl.readPixels(0, 0, w, h, gl.RGBA, gl.UNSIGNED_BYTE, px);
    const status = gl.checkFramebufferStatus(gl.FRAMEBUFFER);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, null, 0);
    gl.bindFramebuffer(gl.FRAMEBUFFER, prevFb);
    if (gl.PIXEL_PACK_BUFFER !== undefined) gl.bindBuffer(gl.PIXEL_PACK_BUFFER, prevPackBuf);
    gl.pixelStorei(gl.PACK_ALIGNMENT, prevAlign);
    if (status !== gl.FRAMEBUFFER_COMPLETE) S.errors.push({kind: 'fbo', status});
    return px;
  };
  let gl = null;
  const names = ['layerloading', 'layerscanning', 'layerfound'];
  const capture = {
    name: 'skycapture',
    listeners: names.map(n => ({event: 'layerscontroller.' + n, process: ({name, detail}) =>
      S.events.push({name, staged: S.staged, at: performance.now(), detail: JSON.stringify(detail)})})),
    onStart: args => {
      gl = args.GLctx;
      S.backend = gl ? {renderer: gl.getParameter(gl.RENDERER), version: gl.getParameter(gl.VERSION)} : null;
      S.onStart = {videoWidth: args.videoWidth, videoHeight: args.videoHeight, canvasWidth: args.canvasWidth,
        canvasHeight: args.canvasHeight, orientation: args.orientation};
    },
    // Runs before the layers module (module order): sets the device orientation of processed frame k.
    onProcessGpu: ({frameStartResult: f}) => {
      if (f.repeatFrame || !f.cameraTexture) return;
      const k = S.staged;
      const o = cfg.meta.orientations[k % cfg.meta.orientations.length];
      window.dispatchEvent(new DeviceOrientationEvent('deviceorientation',
        {alpha: o.alpha, beta: o.beta, gamma: o.gamma, absolute: false}));
      S.orientationLog.push({k, ...o, at: performance.now()});
      const px = readTexture(f.GLctx || gl, f.cameraTexture, f.textureWidth, f.textureHeight);
      S.cams.push({k, w: f.textureWidth, h: f.textureHeight, videoTime: f.videoTime, frameTime: f.frameTime});
      window.__save('cam', k, b64(px));
      S.staged++;
    },
    onProcessCpu: ({frameStartResult: f, processGpuResult: g}) => {
      const inv = g && g.layerscontroller && g.layerscontroller.invokeData;
      if (!f.repeatFrame && inv) {
        S.resizedCount++;
        window.__save('resized', S.staged - 1, b64(inv.pixels));
        S.resizedMeta = S.resizedMeta || {};
        S.resizedMeta[S.staged - 1] = {rows: inv.rows, cols: inv.cols, rotation: inv.rotation};
      }
    },
    onUpdate: ({frameStartResult: f, processCpuResult: p}) => {
      const lc = p.layerscontroller;
      const row = {at: performance.now(), repeat: f.repeatFrame, staged: S.staged, hasOutput: !!lc};
      if (lc) {
        row.intrinsics = Array.from(lc.intrinsics || []);
        row.rotation = lc.rotation; row.position = lc.position;
        const sky = lc.layers && lc.layers.sky;
        if (sky) {
          row.percentage = sky.percentage; row.textureWidth = sky.textureWidth; row.textureHeight = sky.textureHeight;
          if (!f.repeatFrame && sky.texture && sky.textureWidth > 0) {
            const px = readTexture(gl, sky.texture, sky.textureWidth, sky.textureHeight);
            const idx = S.updates.length;
            row.idx = idx;
            window.__save('sky', idx, b64(px));
          }
        }
      }
      S.updates.push(row);
    },
    onException: e => S.errors.push({kind: 'pipeline', message: String(e)}),
  };
  const loaded = async () => {
    S.sdkVersion = XR8.version();
    S.deviceEstimate = XR8.XrDevice.deviceEstimate();
    const conf = {layers: {sky: cfg.skyConfig}, coordinates: {mirroredDisplay: cfg.mirrored, axes: 'RIGHT_HANDED'}};
    if (cfg.near !== null) conf.nearClip = cfg.near;
    if (cfg.far !== null) conf.farClip = cfg.far;
    XR8.LayersController.configure(conf);
    S.configured = conf;
    XR8.addCameraPipelineModules([capture, XR8.GlTextureRenderer.pipelineModule(), XR8.LayersController.pipelineModule()]);
    S.ready = true;
    S.begin = async () => {
      XR8.run({canvas: document.getElementById('view'), allowedDevices: XR8.XrConfig.device().ANY,
        cameraConfig: {direction: XR8.XrConfig.camera().BACK}, verbose: true});
      let ts = 0;
      const t0 = performance.now();
      while (S.staged === 0) { await write(0, ts); ts += 16000; await new Promise(r => setTimeout(r, 16)); }
      S.firstStagedMs = performance.now() - t0;
      for (let i = 0; i < frames.length; i++) {
        const before = S.staged;
        const t1 = performance.now();
        // One write per source frame; retry only if the pipeline did not take it.
        let tries = 0;
        do {
          await write(i, ts); ts += 50000; tries++;
          const t2 = performance.now();
          while (S.staged === before && performance.now() - t2 < 1500) await new Promise(r => setTimeout(r, 20));
        } while (S.staged === before && tries < 5);
        while (performance.now() - t1 < cfg.frameMs) await new Promise(r => setTimeout(r, 10));
        S.written = i + 1;
      }
      await new Promise(r => setTimeout(r, cfg.tailMs));
      try { XR8.stop(); } catch (e) { S.errors.push({kind: 'stop', message: String(e)}); }
      S.done = true;
    };
  };
  if (window.XR8) loaded(); else window.addEventListener('xrloaded', loaded, {once: true});
})();`;
const html = '<!doctype html><html><head><meta charset="utf-8"><script src="/page.js" defer></script>' +
  '<script src="/engine/xr.js" async data-preload-chunks="slam"></script></head><body>' +
  `<canvas id="view" width="${meta.width}" height="${meta.height}"></canvas></body></html>`;
const config = {meta, frameMs: Number(values['frame-ms']), tailMs: Number(values['tail-ms']),
  skyConfig, near: report.near, far: report.far, mirrored: report.mirrored};

const serve = async (req, res) => {
  try {
    const p = new URL(req.url, 'http://localhost').pathname;
    let bytes; let type = 'application/octet-stream';
    if (p === '/' || p === '/index.html') { bytes = Buffer.from(html); type = 'text/html'; }
    else if (p === '/page.js') { bytes = Buffer.from(pageJs); type = 'text/javascript'; }
    else if (p === '/config.json') { bytes = Buffer.from(JSON.stringify(config)); type = 'application/json'; }
    else if (/^\/frames\/\d+$/.test(p)) {
      const i = Number(p.split('/').at(-1));
      if (i >= meta.count) throw new Error('frame index');
      bytes = await fs.readFile(path.join(framesDir, `frame-${String(i).padStart(4, '0')}.rgba`));
    } else if (p.startsWith('/engine/')) {
      const file = path.resolve(engine, p.slice(8));
      if (!file.startsWith(engine + path.sep)) throw new Error('path');
      bytes = await fs.readFile(file); type = file.endsWith('.js') ? 'text/javascript' : type;
    } else throw new Error('unserved ' + p);
    res.writeHead(200, {'Content-Type': type, 'Cache-Control': 'no-store'}); res.end(bytes);
  } catch (e) { res.writeHead(404); res.end(String(e)); }
};

let server; let browser;
try {
  server = http.createServer((q, r) => void serve(q, r));
  await new Promise(r => server.listen(0, '127.0.0.1', r));
  const origin = `http://127.0.0.1:${server.address().port}`;
  browser = await chromium.launch({executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome',
    headless: true, args: ['--enable-unsafe-swiftshader', '--use-fake-ui-for-media-stream',
      '--use-fake-device-for-media-stream', '--autoplay-policy=no-user-gesture-required']});
  report.browserVersion = browser.version();
  const mobile = values.mobile === '1';
  report.mobile = mobile;
  const context = await browser.newContext(mobile ? {
    viewport: {width: meta.width, height: meta.height}, screen: {width: meta.width, height: meta.height},
    deviceScaleFactor: 1, isMobile: true, hasTouch: true, permissions: ['camera'],
    userAgent: 'Mozilla/5.0 (Linux; Android 10; K) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/144.0.0.0 Mobile Safari/537.36',
  } : {viewport: {width: meta.width, height: meta.height}, permissions: ['camera']});
  const page = await context.newPage();
  await page.route('**/*', route => {
    const url = route.request().url();
    if (!url.startsWith(origin + '/') && !url.startsWith('data:') && !url.startsWith('blob:')) {
      report.blockedExternal.push(url); return route.abort('blockedbyclient');
    }
    return route.continue();
  });
  page.on('console', m => report.console.push({type: m.type(), text: m.text()}));
  page.on('pageerror', e => report.pageErrors.push({message: e.message}));
  const pending = [];
  const ext = {cam: 'rgba', resized: 'rgba', sky: 'rgba', 'invoke-img': 'img.rgba', 'invoke-data': 'data.f32'};
  await page.exposeFunction('__save', (kind, index, data) => {
    const dir = kind.startsWith('invoke') ? 'invoke' : kind;
    const prefix = kind.startsWith('invoke') ? 'invoke' : kind;
    const file = path.join(out, dir, `${prefix}-${String(index).padStart(4, '0')}.${ext[kind]}`);
    pending.push(fs.writeFile(file, Buffer.from(data, 'base64')));
  });
  await page.goto(origin + '/index.html');
  await page.waitForFunction(() => window.__sky && window.__sky.ready, null, {timeout: 60000});
  await page.evaluate(() => { window.__sky.begin(); });
  const budget = meta.count * (Number(values['frame-ms']) + 1700) + Number(values['tail-ms']) + 60000;
  await page.waitForFunction(() => window.__sky.done, null, {timeout: budget, polling: 500});
  const state = await page.evaluate(() => window.__sky);
  await Promise.all(pending);
  report.state = state;
  report.status = state.staged > 0 ? 'RECORDED' : 'NO_STAGED_FRAMES';
} catch (e) {
  report.status = 'FAILED'; report.error = {message: e.message, stack: e.stack};
} finally {
  if (browser) await browser.close();
  if (server) await new Promise(r => server.close(r));
  report.finishedAt = new Date().toISOString();
  await fs.writeFile(path.join(out, 'result.json'), JSON.stringify(report, null, 1) + '\n');
  console.log(JSON.stringify({status: report.status, staged: report.state?.staged,
    updates: report.state?.updates?.length, invokes: report.state?.invokes?.length,
    events: report.state?.events?.length, errors: report.state?.errors,
    pageErrors: report.pageErrors.length, out}, null, 1));
}
