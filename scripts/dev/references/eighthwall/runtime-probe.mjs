import fs from 'node:fs/promises';
import path from 'node:path';
import http from 'node:http';
import {createHash} from 'node:crypto';
import {fileURLToPath} from 'node:url';
import {parseArgs} from 'node:util';
import {chromium} from '../../node_modules/playwright/index.mjs';
import {parsePGM} from '../../replay-android-input.mjs';
import {buildSensorPackets} from './sensor_adapter.mjs';

// Original SDK bytes, public XR8 API, synthetic browser inputs. No Wasm export wrapper.
const source = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(source, '../../../..');
const {values} = parseArgs({options: {
  label: {type: 'string'},
  output: {type: 'string', default: 'build/commercial-slam-20261013/8thwall-runtime'},
  frames: {type: 'string', default: '120'},
  mode: {type: 'string', default: 'mobile'},
  dataset: {type: 'string', default: 'build/mobile-web-20261009/synthetic/clean-seed101-v2'},
  scale: {type: 'string', default: 'responsive'},
}});
if (!/^[a-z0-9-]+$/.test(values.label || '')) throw new Error('--label required; preserve each attempt with a new label');
if (!['mobile', 'desktop'].includes(values.mode)) throw new Error('--mode mobile|desktop');
if (!['responsive', 'absolute'].includes(values.scale)) throw new Error('--scale responsive|absolute');
const count = Number(values.frames);
if (!Number.isInteger(count) || count < 1 || count > 400) throw new Error('--frames 1..400');
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const outputRoot = path.resolve(root, values.output);
const ownedOutputRoot = path.join(root, 'build/commercial-slam-20261013/8thwall-runtime');
if (outputRoot !== ownedOutputRoot && !outputRoot.startsWith(ownedOutputRoot + path.sep)) {
  throw new Error('Output must stay within build/commercial-slam-20261013/8thwall-runtime');
}
await fs.mkdir(outputRoot, {recursive: true});
const out = path.join(outputRoot, values.label);
await fs.mkdir(out, {recursive: false});
const engine = path.join(root, 'assets/references/8thwall/analysis-runtime/standalone');
const dataset = path.resolve(root, values.dataset);
const report = {
  schema: '8thwall-public-api-runtime-probe-v1', label: values.label,
  startedAt: new Date().toISOString(), argv: process.argv, node: process.version,
  mode: values.mode, scale: values.scale, requestedFrameCount: count,
  implementation: 'Original SDK bytes and existing synthetic input runtime; no WebAssembly API/export patching',
  requests: [], blockedExternal: [], console: [], pageErrors: [], requestFailures: [],
  limits: [
    'Desktop Chromium and software WebGL, not a physical Android device',
    'Synthetic getUserMedia plus untrusted DeviceMotion/DeviceOrientation events',
    'Input K/extrinsic/exposure and OS fused orientation are not identical to the own-engine rig',
    'Public pipeline callbacks can repeat a result; callback count is not processed frame count',
    'NORMAL status and a finite pose do not establish metric accuracy or translation observability',
    'No independent GT association, ATE/RPE, sustained phone timing or same-input engine A/B acceptance',
  ],
};
let server, browser;
try {
  const inputBytes = await fs.readFile(path.join(dataset, 'input.json'));
  const input = JSON.parse(inputBytes);
  if (input.schema !== 'mobile-slam-synthetic-vio-v1') throw new Error('Independent synthetic fixture required');
  if (input.frames.length < count) throw new Error('Fixture shorter than requested frame count');
  const frames = input.frames.slice(0, count);
  const imu = input.imu.filter(row => row[0] <= frames.at(-1).physicalTimestamp + 0.02);
  const gray = [];
  for (const frame of frames) {
    if (!/^[a-zA-Z0-9_.-]+$/.test(frame.file)) throw new Error('Unsafe fixture frame path');
    const bytes = await fs.readFile(path.join(dataset, 'mav0/cam0/data', frame.file));
    const pixels = parsePGM(bytes, input.config.width, input.config.height);
    if (sha(pixels) !== frame.sha256) throw new Error('Input pixel hash mismatch');
    gray.push(pixels);
  }
  const sensors = buildSensorPackets(imu);
  const fixture = {
    schema: input.schema, config: input.config, frames, imu,
    sourceInputSha256: sha(inputBytes), sensorPackets: sensors.packets,
    sensorProvenance: sensors.provenance, cameraSource: 'generated', scale: values.scale,
  };
  report.input = {dataset, sha256: sha(inputBytes), frames: frames.length, imuPackets: imu.length,
    sourcePixelHashes: frames.map(frame => frame.sha256), groundTruthProvidedToEngine: false,
    sensorProvenance: sensors.provenance};
  const stateScript = `window.__probe={ready:false,started:false,done:false,errors:[],events:[],rows:[],frames:[],imu:[],cameraRequests:[]};
window.addEventListener('error',e=>window.__probe.errors.push({kind:'window',message:e.message,stack:e.error?.stack}));
window.addEventListener('unhandledrejection',e=>window.__probe.errors.push({kind:'promise',message:String(e.reason),stack:e.reason?.stack}));`;
  const html = '<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>8th Wall public API probe</title><script src="/state.js"></script><script src="/runtime.js" defer></script><script src="/engine/xr.js" async data-preload-chunks="slam"></script></head><body><button id="start">Start isolated synthetic probe</button><canvas id="view" width="320" height="240"></canvas></body></html>';
  const runtimeBytes = await fs.readFile(path.join(source, 'runtime.js'));
  const servedSources = {'index.html': Buffer.from(html), 'state.js': Buffer.from(stateScript), 'runtime.js': runtimeBytes};
  report.sourceHashes = {};
  await fs.mkdir(path.join(out, 'source'));
  for (const name of ['runtime-probe.mjs', 'runtime.js', 'sensor_adapter.mjs']) {
    const bytes = await fs.readFile(path.join(source, name));
    report.sourceHashes[name] = sha(bytes);
    await fs.writeFile(path.join(out, 'source', name), bytes);
  }
  for (const name of ['index.html', 'state.js']) {
    report.sourceHashes[name] = sha(servedSources[name]);
    await fs.writeFile(path.join(out, 'source', name), servedSources[name]);
  }
  report.engineAssets = {};
  for (const name of ['xr.js', 'xr-slam.js', 'LICENSE']) {
    const bytes = await fs.readFile(path.join(engine, name));
    report.engineAssets[name] = {bytes: bytes.length, sha256: sha(bytes)};
  }
  report.priorEvidence = [];
  for (const label of ['clean101-mobile-400', 'desktop-current-80', 'clean101-absolute-so3-400']) {
    const previousPath = path.join(root, 'build/research-20261012/8thwall-binary/runtime', label, 'result.json');
    try {
      const bytes = await fs.readFile(previousPath), previous = JSON.parse(bytes);
      report.priorEvidence.push({path: path.relative(root, previousPath), sha256: sha(bytes), status: previous.status,
        summary: previous.summary, note: 'Historical observer-wrapped export evidence; not this new execution'});
    } catch (error) { report.priorEvidence.push({path: previousPath, error: String(error)}); }
  }
  const serve = async (req, res) => {
    try {
      const pathname = new URL(req.url, 'http://localhost').pathname;
      let bytes, type = 'application/octet-stream';
      if (pathname === '/fixture.json') { bytes = Buffer.from(JSON.stringify(fixture)); type = 'application/json'; }
      else if (/^\/gray\/\d+$/.test(pathname)) {
        const index = Number(pathname.split('/').at(-1));
        bytes = gray[index];
        if (!bytes) throw new Error('Unknown frame');
      } else if (pathname === '/' || Object.hasOwn(servedSources, pathname.slice(1))) {
        const name = pathname === '/' ? 'index.html' : pathname.slice(1);
        bytes = servedSources[name]; type = name.endsWith('.html') ? 'text/html' : 'text/javascript';
      } else if (pathname.startsWith('/engine/')) {
        const file = path.resolve(engine, pathname.slice(8));
        if (!file.startsWith(engine + path.sep)) throw new Error('Unserved path');
        bytes = await fs.readFile(file); type = file.endsWith('.js') ? 'text/javascript' : 'application/octet-stream';
      } else throw new Error('Unserved path');
      res.writeHead(200, {'Content-Type': type, 'Cache-Control': 'no-store'}); res.end(bytes);
    } catch (error) { res.writeHead(404); res.end(String(error)); }
  };
  server = http.createServer((req, res) => void serve(req, res));
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
  const origin = `http://127.0.0.1:${server.address().port}`;
  report.origin = origin;
  const launchOptions = {
    executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true,
    args: ['--enable-unsafe-swiftshader', '--autoplay-policy=no-user-gesture-required',
      '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream'],
  };
  report.browserLaunch = launchOptions;
  browser = await chromium.launch(launchOptions);
  report.browserVersion = browser.version();
  const options = {viewport: {width: 320, height: 240}, permissions: ['camera', 'microphone'],
    isMobile: values.mode === 'mobile', hasTouch: values.mode === 'mobile'};
  if (values.mode === 'mobile') options.userAgent = 'Mozilla/5.0 (Linux; Android 13; Pixel 7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/140.0.0.0 Mobile Safari/537.36';
  report.browserContext = options;
  const context = await browser.newContext(options), page = await context.newPage();
  await page.route('**/*', route => {
    const url = route.request().url();
    if (!url.startsWith(origin + '/') && !url.startsWith('data:') && !url.startsWith('blob:')) {
      report.blockedExternal.push(url); return route.abort('blockedbyclient');
    }
    return route.continue();
  });
  page.on('response', response => report.requests.push({url: response.url(), status: response.status()}));
  page.on('requestfailed', request => report.requestFailures.push({url: request.url(), failure: request.failure()}));
  page.on('console', message => report.console.push({type: message.type(), text: message.text()}));
  page.on('pageerror', error => report.pageErrors.push({message: error.message, stack: error.stack}));
  await page.goto(origin + '/index.html?mode=' + values.mode);
  await page.waitForFunction(() => window.__probe?.ready, null, {timeout: 45000});
  report.publicApi = await page.evaluate(async () => {
    const before = XR8.isInitialized();
    await XR8.initialize();
    await XR8.loadChunk('slam');
    return {initializeResolved: true, slamLoadChunkResolved: true, before, isInitialized: XR8.isInitialized(),
      version: XR8.version(), deviceEstimate: XR8.XrDevice.deviceEstimate(),
      controllerMethods: Object.keys(XR8.XrController).sort()};
  });
  await page.locator('#start').click();
  try { await page.waitForFunction(() => window.__probe?.done, null, {timeout: count * 50 + 15000}); }
  catch (error) { report.waitError = String(error); }
  await page.evaluate(() => window.__finishProbe?.());
  report.observed = await page.evaluate(() => window.__probe);
  await page.screenshot({path: path.join(out, 'screenshot.png')});
  const observed = report.observed;
  const summary = {sourceFrames: observed.frames.length, imuDispatched: observed.imu.length,
    callbacks: observed.rows.length, nonrepeatCallbacks: 0, finiteTransformCallbacks: 0,
    finiteIntrinsicsCallbacks: 0, nonzeroPositions: 0, trackingStates: {},
    positionMinimum: [Infinity, Infinity, Infinity], positionMaximum: [-Infinity, -Infinity, -Infinity]};
  for (const row of observed.rows) {
    const state = `${row.trackingStatus}/${row.trackingReason}`;
    summary.trackingStates[state] = (summary.trackingStates[state] || 0) + 1;
    if (row.repeatFrame === false) summary.nonrepeatCallbacks++;
    if (row.intrinsics?.length === 16 && row.intrinsics.every(Number.isFinite)) summary.finiteIntrinsicsCallbacks++;
    if (row.position && row.rotation && Object.values(row.position).concat(Object.values(row.rotation)).every(Number.isFinite)) {
      summary.finiteTransformCallbacks++;
      const coordinates = [row.position.x, row.position.y, row.position.z];
      coordinates.forEach((v, i) => { summary.positionMinimum[i] = Math.min(summary.positionMinimum[i], v); summary.positionMaximum[i] = Math.max(summary.positionMaximum[i], v); });
      if (Math.hypot(...coordinates) > 0) summary.nonzeroPositions++;
    }
  }
  summary.positionRange = summary.positionMinimum.map((v, i) => summary.positionMaximum[i] - v);
  report.summary = summary;
  report.status = summary.finiteTransformCallbacks > 0 ? 'ACTUAL_PUBLIC_API_TRANSFORM_OUTPUT' :
    summary.callbacks > 0 ? 'CAMERA_CALLBACKS_WITHOUT_TRANSFORM' : 'NO_CAMERA_CALLBACKS';
} catch (error) { report.status = 'FAILED'; report.error = {message: error.message, stack: error.stack}; }
finally {
  if (browser) await browser.close();
  if (server) await new Promise(resolve => server.close(resolve));
  report.finishedAt = new Date().toISOString();
  await fs.writeFile(path.join(out, 'result.json'), JSON.stringify(report, null, 2) + '\n');
  console.log(JSON.stringify({status: report.status, summary: report.summary, error: report.error, waitError: report.waitError, out}, null, 2));
  process.exitCode = report.status === 'FAILED' ? 1 : report.status === 'NO_CAMERA_CALLBACKS' ? 2 : 0;
}
