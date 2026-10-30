#!/usr/bin/env node
// Run the original reference runtime standalone SDK on a physical phone with its real camera and sensors.
// A throwaway 127.0.0.1 server is exposed to the phone through `adb reverse`, so the page origin is
// http://localhost (secure context) and nothing is served on the LAN. In-page recording is minimal
// (one row per onUpdate plus sensor event counts); GPU/thread cost comes from a Chrome trace taken over CDP.
import fs from 'node:fs/promises';
import http from 'node:http';
import path from 'node:path';
import os from 'node:os';
import { execFileSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { fileURLToPath } from 'node:url';
import { parseArgs } from 'node:util';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(here, '../../../..');
const { values } = parseArgs({ options: {
    serial: { type: 'string' }, adb: { type: 'string', default: process.env.ADB || path.join(process.env.ANDROID_SDK_ROOT || process.env.ANDROID_HOME || path.join(os.homedir(), 'android-sdk'), 'platform-tools/adb') },
    cdp: { type: 'string', default: 'http://127.0.0.1:9222' }, port: { type: 'string', default: '8890' },
    seconds: { type: 'string', default: '30' }, trace: { type: 'boolean', default: true }, output: { type: 'string' } } });
if (!values.serial || !values.output) throw new Error('--serial and --output required');
const seconds = Number(values.seconds), port = Number(values.port);
if (!Number.isInteger(seconds) || seconds < 5 || seconds > 60) throw new Error('seconds 5..60');
await fs.mkdir(values.output, { recursive: false });
const engine = path.join(root, 'assets/references/xr_reference/analysis-runtime/standalone');
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const adb = (...args) => execFileSync(values.adb, ['-s', values.serial, ...args], { encoding: 'utf8' });

const page = `<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>reference runtime phone probe</title><style>body{margin:0;background:#000;color:#0f8;font:15px sans-serif}#view{width:100vw;height:80vh}
#start{font-size:22px;padding:14px 24px;margin:12px}#badge{position:fixed;top:8px;left:8px;background:#111;padding:4px 8px}</style>
<script>
window.__phone={rows:[],events:[],errors:[],sensor:{devicemotion:0,deviceorientation:0,deviceorientationabsolute:0},sensorCtors:[]};
// Count sensor deliveries the SDK subscribes to; no per-event storage.
(()=>{const P=window.__phone,add=EventTarget.prototype.addEventListener;
EventTarget.prototype.addEventListener=function(t,l,o){if(this===window&&t in P.sensor&&l){const f=typeof l==='function'?l:e=>l.handleEvent(e);
return add.call(this,t,function(e){P.sensor[t]++;return f.call(this,e)},o)}return add.call(this,t,l,o)};
for(const n of ['Accelerometer','Gyroscope','LinearAccelerationSensor','GravitySensor','AbsoluteOrientationSensor','RelativeOrientationSensor']){
const O=window[n];if(typeof O!=='function')continue;window[n]=new Proxy(O,{construct(t,a,nt){P.sensorCtors.push({n,options:a[0]??null});return Reflect.construct(t,a,nt)}})}})();
</script>
<script src="/engine/xr.js" async data-preload-chunks="slam"></script></head>
<body><div id="badge">reference runtime · loading</div><button id="start">Start reference runtime</button><canvas id="view"></canvas>
<script>
const P=window.__phone,badge=document.getElementById('badge');
const loaded=()=>{P.version=XR8.version();
 XR8.XrController.configure({disableWorldTracking:false,enableWorldPoints:true,scale:'responsive'});
 XR8.addCameraPipelineModules([XR8.GlTextureRenderer.pipelineModule(),XR8.XrController.pipelineModule(),{name:'phone-observer',
  onStart:a=>{const gl=a.GLctx,ext=gl?.getExtension('WEBGL_debug_renderer_info');P.events.push({name:'onStart',at:performance.now(),videoWidth:a.videoWidth,videoHeight:a.videoHeight,renderer:ext?gl.getParameter(ext.UNMASKED_RENDERER_WEBGL):null});badge.textContent='reference runtime · running';},
  onCameraStatusChange:a=>P.events.push({name:'camera',at:performance.now(),status:a.status}),
  onException:e=>P.errors.push(String(e)),
  onUpdate:({frameStartResult:f,processCpuResult:p})=>{const r=p.reality;if(!r)return;P.rows.push([performance.now(),f.videoTime,f.repeatFrame?1:0,r.trackingStatus,r.trackingReason,r.position?.x,r.position?.y,r.position?.z,r.rotation?.w,r.rotation?.x,r.rotation?.y,r.rotation?.z,r.worldPoints?.length??null]);}
 }]);badge.textContent='reference runtime · ready (tap Start)';};
if(window.XR8)loaded();else window.addEventListener('xrloaded',loaded,{once:true});
document.getElementById('start').onclick=()=>{P.startedAt=performance.now();document.getElementById('start').remove();
 try{XR8.run({canvas:document.getElementById('view'),allowedDevices:XR8.XrConfig.device().MOBILE,cameraConfig:{direction:XR8.XrConfig.camera().BACK}})}catch(e){P.errors.push(String(e))}};
window.__stop=()=>{try{XR8.stop()}catch(e){P.errors.push(String(e))}P.stoppedAt=performance.now();badge.textContent='reference runtime · stopped';return true};
</script></body></html>`;

const files = new Map([['/phone.html', ['text/html', Buffer.from(page)]]]);
for (const name of ['xr.js', 'xr-slam.js']) files.set(`/engine/${name}`, ['text/javascript', await fs.readFile(path.join(engine, name))]);
const fingerprints = Object.fromEntries([...files].map(([url, [, bytes]]) => [url, { bytes: bytes.length, sha256: sha(bytes) }]));
const requests = [];
const server = http.createServer((request, response) => {
    const url = new URL(request.url, 'http://localhost').pathname, file = files.get(url);
    requests.push({ url, status: file ? 200 : 404, at: Date.now() });
    if (!file) { response.writeHead(404).end(); return; }
    response.writeHead(200, { 'content-type': file[0], 'cache-control': 'no-store' }).end(file[1]);
}).listen(port, '127.0.0.1');
await new Promise(resolve => server.once('listening', resolve));
const report = { schema: 'xr_reference-phone-probe-v1', startedAt: new Date().toISOString(), seconds, fingerprints,
    limits: ['Real phone camera and OS sensors; controlled human motion, no independent ground truth',
        'responsive scale is not metric; NORMAL status is not an accuracy result'] };
let target, socket, browserSocket, reverseCreated = false;
const cdpCall = (ws, method, params = {}) => new Promise((resolve, reject) => {
    const id = Math.floor(Math.random() * 1e9);
    const timer = setTimeout(() => reject(new Error(`CDP timeout ${method}`)), 30000);
    const receive = event => { const m = JSON.parse(event.data); if (m.id !== id) return; clearTimeout(timer); ws.removeEventListener('message', receive);
        m.error ? reject(new Error(m.error.message)) : resolve(m.result); };
    ws.addEventListener('message', receive); ws.send(JSON.stringify({ id, method, params }));
});
const evaluate = async expression => (await cdpCall(socket, 'Runtime.evaluate', { expression, awaitPromise: true, returnByValue: true })).result.value;
const open = url => new Promise((resolve, reject) => { const ws = new WebSocket(url); ws.onopen = () => resolve(ws); ws.onerror = reject; });
try {
    adb('reverse', `tcp:${port}`, `tcp:${port}`); reverseCreated = true;
    // Chrome on Android rejects /json/new; open the tab through the browser target instead.
    const version = await (await fetch(new URL('/json/version', values.cdp))).json();
    browserSocket = await open(version.webSocketDebuggerUrl);
    const { targetId } = await cdpCall(browserSocket, 'Target.createTarget', { url: `http://localhost:${port}/phone.html` });
    for (let i = 0; i < 20 && !target; i++) {
        target = (await (await fetch(new URL('/json/list', values.cdp))).json()).find(item => item.id === targetId);
        if (!target) await new Promise(r => setTimeout(r, 500));
    }
    if (!target) throw new Error('New tab did not appear in /json/list');
    socket = await open(target.webSocketDebuggerUrl);
    await cdpCall(socket, 'Performance.enable', { timeDomain: 'timeTicks' });
    console.log(JSON.stringify({ signal: 'OPENED', url: target.url, next: 'Allow camera on the phone, then tap Start reference runtime' }));
    for (let i = 0; i < 120 && !(await evaluate('!!window.__phone?.startedAt')); i++) await new Promise(r => setTimeout(r, 1000));
    if (!(await evaluate('!!window.__phone?.startedAt'))) throw new Error('Start was not tapped within 120 s');
    const metricsBefore = (await cdpCall(socket, 'Performance.getMetrics')).metrics;
    let tracing = null;
    if (values.trace) {
        await cdpCall(browserSocket, 'Tracing.start', { transferMode: 'ReturnAsStream', traceConfig: { includedCategories:
            ['toplevel', 'devtools.timeline', 'disabled-by-default-devtools.timeline', 'v8.execute', 'gpu', 'viz', '__metadata'], excludedCategories: ['*'] } });
        tracing = true;
    }
    console.log(JSON.stringify({ signal: 'RECORDING', seconds }));
    await new Promise(r => setTimeout(r, seconds * 1000));
    const metricsAfter = (await cdpCall(socket, 'Performance.getMetrics')).metrics;
    if (tracing) {
        const done = new Promise(resolve => { const receive = event => { const m = JSON.parse(event.data);
            if (m.method === 'Tracing.tracingComplete') { browserSocket.removeEventListener('message', receive); resolve(m.params.stream); } };
            browserSocket.addEventListener('message', receive); });
        await cdpCall(browserSocket, 'Tracing.end');
        const stream = await done, handle = await fs.open(path.join(values.output, 'trace.json'), 'wx');
        try { for (;;) { const r = await cdpCall(browserSocket, 'IO.read', { handle: stream, size: 1 << 20 });
            await handle.write(r.base64Encoded ? Buffer.from(r.data, 'base64') : r.data); if (r.eof) break; } }
        finally { await handle.close(); await cdpCall(browserSocket, 'IO.close', { handle: stream }); }
    }
    await evaluate('window.__stop()');
    report.page = await evaluate('window.__phone');
    const m = list => Object.fromEntries(list.map(x => [x.name, x.value]));
    const a = m(metricsBefore), b = m(metricsAfter), wall = b.Timestamp - a.Timestamp;
    report.mainThread = { wallS: wall, taskBusyFraction: (b.TaskDuration - a.TaskDuration) / wall, scriptBusyFraction: (b.ScriptDuration - a.ScriptDuration) / wall,
        jsHeapUsedMiB: b.JSHeapUsedSize / 1048576 };
} catch (error) { report.error = error.message; process.exitCode = 1; }
finally {
    if (socket) socket.close();
    if (target?.id && browserSocket) await cdpCall(browserSocket, 'Target.closeTarget', { targetId: target.id }).catch(() => {});
    if (browserSocket) browserSocket.close();
    if (reverseCreated) try { adb('reverse', '--remove', `tcp:${port}`); } catch {}
    server.close(); report.requests = requests;
    await fs.writeFile(path.join(values.output, 'result.json'), JSON.stringify(report) + '\n', { flag: 'wx' });
    console.log(JSON.stringify({ output: values.output, rows: report.page?.rows?.length ?? 0, error: report.error ?? null }));
}
