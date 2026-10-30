#!/usr/bin/env node
// Original reference runtime face runtime (standalone xr.js + xr-face.js, original WASM and models) on
// synthetic frames in headless Chromium, recording what slam_refact's face pipeline needs for an
// A/B comparison (US-018/US-019; docs/spec/08-face-ear.md §8.4):
//   - the camera texture of every staged (non-repeat) frame, read back from the compute context
//     before FaceController draws it (texture-%04d.rgba) -> exact input for sr_face_replay;
//   - FaceController debug output per pipeline callback: rendered ROI image (rendered-%04d.rgba),
//     global detections and mesh detections, camera intrinsics;
//   - every facecontroller.* event with its full detail;
//   - with --ears 1 (RM-7, docs/spec/08-face-ear.md §5.5): the ear debug output of the original
//     runtime (extraOutput.earRenderedImg / earDetections): the ear ROI render (ear-rendered-%04d.rgba,
//     336x320) and the per-callback ear detections (earDetections field of the update rows).
// No network: only a 127.0.0.1 temporary server owned by this script; other origins are blocked.
// Nothing is written outside build/slam-refact-full-face-evidence/reference/<label>/.
//
// Usage: node run-face-reference.mjs --frames <dir> --label <new label> [--frame-ms 150]
//          [--ears 0|1] [--mirrored 0|1] [--tail-ms 3000]

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
  'frame-ms': {type: 'string', default: '150'},
  'tail-ms': {type: 'string', default: '3000'},
  ears: {type: 'string', default: '0'},
  mirrored: {type: 'string', default: '0'},
}});
if (!values.frames) throw new Error('--frames <dir> required');
if (!/^[a-z0-9-]+$/.test(values.label || '')) throw new Error('--label required (new, [a-z0-9-])');
const framesDir = path.resolve(values.frames);
const outRoot = path.join(root, 'build/slam-refact-full-face-evidence/reference');
const out = path.join(outRoot, values.label);
await fs.mkdir(outRoot, {recursive: true});
await fs.mkdir(out, {recursive: false});
await fs.mkdir(path.join(out, 'textures'));
await fs.mkdir(path.join(out, 'rendered'));
await fs.mkdir(path.join(out, 'ear-rendered'));
const engine = path.join(root, 'assets/references/xr_reference/analysis-runtime/standalone');
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const meta = JSON.parse(await fs.readFile(path.join(framesDir, 'meta.json'), 'utf8'));

const report = {
  schema: 'slam-refact-face-reference-v1', label: values.label, startedAt: new Date().toISOString(),
  argv: process.argv, node: process.version, frames: framesDir, meta,
  frameMs: Number(values['frame-ms']), ears: values.ears === '1', mirrored: values.mirrored === '1',
  engineAssets: {}, console: [], pageErrors: [], blockedExternal: [], requests: [],
  limits: [
    'Desktop headless Chromium, SwiftShader WebGL: GPU ROI render of SwiftShader, not a phone GPU',
    'Synthetic frames through MediaStreamTrackGenerator; the recorded camera texture is the input',
    'No real face: detections on synthetic texture are model false positives, not accuracy evidence',
  ],
};
for (const name of ['xr.js', 'xr-face.js', 'resources/face-model.tflite',
  'resources/face-mesh-model.tflite', 'resources/face-ear-model.tflite']) {
  const bytes = await fs.readFile(path.join(engine, name));
  report.engineAssets[name] = {bytes: bytes.length, sha256: sha(bytes)};
}

const pageJs = `(async () => {
  const S = window.__face = {ready: false, done: false, errors: [], events: [], updates: [],
    staged: 0, written: 0, backend: null};
  window.addEventListener('error', e => S.errors.push({kind: 'window', message: e.message}));
  window.addEventListener('unhandledrejection', e => S.errors.push({kind: 'promise', message: String(e.reason)}));
  const cfg = await (await fetch('/config.json')).json();
  const W = cfg.meta.width, H = cfg.meta.height;
  const frames = [];
  for (let i = 0; i < cfg.meta.count; i++) {
    frames.push(new Uint8ClampedArray(await (await fetch('/frames/' + i)).arrayBuffer()));
  }
  const canvas = document.createElement('canvas'); canvas.width = W; canvas.height = H;
  const ctx = canvas.getContext('2d');
  const track = new MediaStreamTrackGenerator({kind: 'video'});
  const writer = track.writable.getWriter();
  const stream = new MediaStream([track]);
  const settings = track.getSettings.bind(track);
  Object.defineProperty(track, 'getSettings', {value: () => ({...settings(), width: W, height: H, frameRate: 20, facingMode: 'user'})});
  navigator.mediaDevices.getUserMedia = async () => stream;
  const write = async (i, ts) => {
    ctx.putImageData(new ImageData(frames[i], W, H), 0, 0);
    const vf = new VideoFrame(canvas, {timestamp: ts});
    try { await writer.write(vf); } finally { vf.close(); }
  };
  const b64 = u8 => { let s = ''; const n = 0x8000; for (let i = 0; i < u8.length; i += n) s += String.fromCharCode.apply(null, u8.subarray(i, i + n)); return btoa(s); };
  let fbo = null;
  const readTexture = (gl, tex) => {
    const prevFb = gl.getParameter(gl.FRAMEBUFFER_BINDING);
    const prevPack = gl.PIXEL_PACK_BUFFER !== undefined ? gl.getParameter(gl.PIXEL_PACK_BUFFER_BINDING) : null;
    const prevAlign = gl.getParameter(gl.PACK_ALIGNMENT);
    if (!fbo) fbo = gl.createFramebuffer();
    gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, tex, 0);
    if (gl.PIXEL_PACK_BUFFER !== undefined) gl.bindBuffer(gl.PIXEL_PACK_BUFFER, null);
    gl.pixelStorei(gl.PACK_ALIGNMENT, 4);
    const px = new Uint8Array(W * H * 4);
    gl.readPixels(0, 0, W, H, gl.RGBA, gl.UNSIGNED_BYTE, px);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, null, 0);
    gl.bindFramebuffer(gl.FRAMEBUFFER, prevFb);
    if (gl.PIXEL_PACK_BUFFER !== undefined) gl.bindBuffer(gl.PIXEL_PACK_BUFFER, prevPack);
    gl.pixelStorei(gl.PACK_ALIGNMENT, prevAlign);
    return px;
  };
  const names = ['faceloading','facescanning','facefound','faceupdated','facelost','mouthopened',
    'mouthclosed','lefteyeopened','lefteyeclosed','righteyeopened','righteyeclosed',
    'lefteyebrowraised','lefteyebrowlowered','righteyebrowraised','righteyebrowlowered',
    'lefteyewinked','righteyewinked','blinked','interpupillarydistance','earfound','earlost',
    'earpointfound','earpointlost'];
  const capture = {
    name: 'facecapture',
    listeners: names.map(n => ({event: 'facecontroller.' + n, process: ({name, detail}) =>
      S.events.push({name, staged: S.staged, at: performance.now(), detail: JSON.stringify(detail)})})),
    onStart: args => {
      const gl = args.GLctx;
      S.backend = gl ? {renderer: gl.getParameter(gl.RENDERER), version: gl.getParameter(gl.VERSION)} : null;
      S.onStart = {videoWidth: args.videoWidth, videoHeight: args.videoHeight, canvasWidth: args.canvasWidth, canvasHeight: args.canvasHeight};
    },
    onProcessGpu: ({frameStartResult: f}) => {
      if (f.repeatFrame) return;
      const px = readTexture(f.computeCtx, f.computeTexture);
      window.__save('texture', S.staged, b64(px));
      S.staged++;
    },
    onUpdate: ({frameStartResult: f, processCpuResult: p}) => {
      const fc = p.facecontroller;
      const row = {at: performance.now(), repeat: f.repeatFrame, staged: S.staged, hasOutput: !!fc};
      if (fc) {
        row.intrinsics = Array.from(fc.intrinsics || []);
        row.rotation = fc.rotation; row.position = fc.position;
        const det = fc.debug && fc.debug.detections;
        if (det) row.detections = JSON.stringify({faces: det.faces || [], meshes: det.meshes || []});
        if (!f.repeatFrame && fc.debug && fc.debug.renderedImg) {
          const im = fc.debug.renderedImg;
          row.rendered = {rows: im.rows, cols: im.cols, rowBytes: im.rowBytes};
          window.__save('rendered', S.updates.length, b64(im.pixels));
        }
        if (fc.debug && fc.debug.detections && fc.debug.detections.ears) {
          const d = fc.debug.detections;
          row.earDetections = JSON.stringify({ears: d.ears, earsInFaceRoi: d.earsInFaceRoi || [],
            earsInCameraFeed: d.earsInCameraFeed || [], videoWidth: d.videoWidth, videoHeight: d.videoHeight});
        }
        if (!f.repeatFrame && fc.debug && fc.debug.earRenderedImg) {
          const im = fc.debug.earRenderedImg;
          row.earRendered = {rows: im.rows, cols: im.cols, rowBytes: im.rowBytes};
          window.__save('ear-rendered', S.updates.length, b64(im.pixels));
        }
      }
      S.updates.push(row);
    },
    onException: e => S.errors.push({kind: 'pipeline', message: String(e)}),
  };
  const loaded = async () => {
    await XR8.loadChunk('face');
    XR8.FaceController.configure({
      meshGeometry: [XR8.FaceController.MeshGeometry.FACE],
      coordinates: {mirroredDisplay: cfg.mirrored, axes: 'RIGHT_HANDED'},
      debug: {extraOutput: {renderedImg: true, detections: true,
        earRenderedImg: !!cfg.ears, earDetections: !!cfg.ears}},
      enableEars: cfg.ears,
    });
    XR8.addCameraPipelineModules([capture, XR8.FaceController.pipelineModule()]);
    S.ready = true;
    S.begin = async () => {
      XR8.run({canvas: document.getElementById('view'), allowedDevices: XR8.XrConfig.device().ANY,
        cameraConfig: {direction: XR8.XrConfig.camera().FRONT}, verbose: true});
      let ts = 0;
      // Hold frame 0 until the first staged frame, then step through the sequence.
      while (S.staged === 0) { await write(0, ts); ts += 16000; await new Promise(r => setTimeout(r, 16)); }
      for (let i = 0; i < frames.length; i++) {
        const t0 = performance.now();
        while (performance.now() - t0 < cfg.frameMs) {
          await write(i, ts); ts += 16000; await new Promise(r => setTimeout(r, 16));
        }
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
  '<script src="/engine/xr.js" async data-preload-chunks="face"></script></head><body>' +
  `<canvas id="view" width="${meta.width}" height="${meta.height}"></canvas></body></html>`;
const config = {meta, frameMs: Number(values['frame-ms']), tailMs: Number(values['tail-ms']),
  ears: values.ears === '1', mirrored: values.mirrored === '1'};

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
  const context = await browser.newContext({viewport: {width: meta.width, height: meta.height},
    permissions: ['camera']});
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
  await page.exposeFunction('__save', (kind, index, data) => {
    const dir = kind === 'texture' ? 'textures' : kind === 'ear-rendered' ? 'ear-rendered' : 'rendered';
    const file = path.join(out, dir, `${kind}-${String(index).padStart(4, '0')}.rgba`);
    pending.push(fs.writeFile(file, Buffer.from(data, 'base64')));
  });
  await page.goto(origin + '/index.html');
  await page.waitForFunction(() => window.__face && window.__face.ready, null, {timeout: 60000});
  await page.evaluate(() => { window.__face.begin(); });
  const budget = meta.count * (Number(values['frame-ms']) + 200) + Number(values['tail-ms']) + 60000;
  await page.waitForFunction(() => window.__face.done, null, {timeout: budget, polling: 500});
  const state = await page.evaluate(() => window.__face);
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
    updates: report.state?.updates?.length, events: report.state?.events?.length,
    errors: report.state?.errors, pageErrors: report.pageErrors.length, out}, null, 1));
}
