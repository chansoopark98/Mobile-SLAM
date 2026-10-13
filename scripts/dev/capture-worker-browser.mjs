/** Actual desktop VideoFrame/Worker/OffscreenCanvas contract probe; no phone timing claim. */
import fs from 'node:fs/promises';
import path from 'node:path';
import http from 'node:http';
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { createHash } from 'node:crypto';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const output = path.resolve(process.env.CAPTURE_WORKER_OUTPUT || path.join(root, 'build/recovery-20261012', `capture-worker-${Date.now()}`));
await fs.mkdir(output, { recursive: true });
const { chromium } = createRequire(path.join(root, 'scripts/dev/package.json'))('playwright');
const files = Object.fromEntries(await Promise.all(['camera.js', 'capture-worker.js', 'app.js'].map(async name =>
    [name, await fs.readFile(path.join(root, 'web/js', name), 'utf8')])));
const report = { scope: 'Desktop capture contracts only; no WASM estimator, phone capability/performance or exposure verification',
    physicalTargetVerified: false, defaultPromotion: false, hashes: Object.fromEntries(Object.entries(files).map(([name, source]) =>
        [name, createHash('sha256').update(source).digest('hex')])) };
const server = http.createServer((request, response) => {
    const url = new URL(request.url, 'http://localhost');
    const name = url.pathname.slice(1);
    response.setHeader('Content-Type', name.endsWith('.js') ? 'application/javascript' : 'text/html');
    if (files[name]) return response.end(files[name]);
    if (name === 'delayed-worker.js') return response.end(`
        import {createCaptureWorkerHandler} from '/capture-worker.js';
        const mode=${JSON.stringify(url.searchParams.get('fault') || '')};
        class Canvas extends OffscreenCanvas {
            getContext(...args) {
                const context=super.getContext(...args);
                if(mode==='init')return null;
                if(mode==='draw')context.drawImage=()=>{throw new Error('injected draw error')};
                return context;
            }
        }
        const handle=createCaptureWorkerHandler((message,transfer=[])=>postMessage(message,transfer),Canvas);
        self.onmessage=event=>{
            if(event.data.type!=='frame')return handle(event.data);
            if(mode==='crash')throw new Error('injected fatal Worker error');
            if(mode==='hang')return;
            setTimeout(()=>handle(event.data),${Number(url.searchParams.get('delay') || 0)});
        };`);
    response.end('<!doctype html><title>Capture Worker contract probe</title>');
});
await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
let browser;
try {
    browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true });
    report.browser = browser.version();
    const page = await browser.newPage();
    await page.goto(`http://127.0.0.1:${server.address().port}/?capture=worker&rotate=none`);
    await page.evaluate(async () => {
        Object.assign(window, await import('/camera.js'), await import('/capture-worker.js'));
        window.VIOWrapper = class {}; window.IMU = class {}; window.OrientationHandler = class {};
    });
    await page.addScriptTag({ content: files['app.js'].replace(/^import .*$/gm, '').replace(/const app = new App\(\);[\s\S]*$/, '') + '\nwindow.AppContract=App;' });
    report.result = await page.evaluate(async () => {
        const pause = ms => new Promise(resolve => setTimeout(resolve, ms));
        const until = async (predicate, description) => {
            const deadline = performance.now() + 4000;
            while (!predicate()) { if (performance.now() > deadline) throw new Error(`Timeout: ${description}`); await pause(5); }
        };
        const equal = (actual, expected, description) => {
            if (actual.length !== expected.length || actual.some((value, index) => value !== expected[index])) {
                throw new Error(`${description}: ${JSON.stringify({ actual: Array.from(actual), expected: Array.from(expected) })}`);
            }
        };
        const check = (condition, message) => { if (!condition) throw new Error(message); };
        const sourceColor = (x, y, version) => [(x * 17 + version * 31) % 256, (y * 23 + version * 47) % 256, ((x + y) * 13 + version * 19) % 256];
        const makeVideo = async (width, height) => {
            const canvas = document.createElement('canvas'); Object.assign(canvas, { width, height });
            const context = canvas.getContext('2d', { willReadFrequently: true });
            const paint = version => {
                const image = context.createImageData(width, height);
                for (let y = 0; y < height; y++) for (let x = 0; x < width; x++) {
                    image.data.set([...sourceColor(x,y,version),255], (y * width + x) * 4);
                }
                context.putImageData(image, 0, 0);
            };
            paint(0);
            const stream = canvas.captureStream(0), track = stream.getVideoTracks()[0];
            const video = document.createElement('video'); Object.assign(video, { muted: true, playsInline: true, srcObject: stream });
            document.body.append(video); await video.play();
            const nativeRequest = video.requestVideoFrameCallback.bind(video);
            const present = (version, callback = () => {}) => new Promise((resolve, reject) => {
                const timeout = setTimeout(() => reject(new Error('Video callback timeout')), 3000);
                nativeRequest((now, metadata) => { clearTimeout(timeout); try { callback(now,metadata); resolve({now,metadata}); } catch (error) { reject(error); } });
                paint(version); track.requestFrame();
            });
            return { video, present, close() { stream.getTracks().forEach(value => value.stop()); video.remove(); } };
        };
        const makeCamera = (video, geometry) => {
            const camera = new Camera();
            const swap = ['cw','ccw'].includes(geometry.rotation);
            Object.assign(camera, { video, running: true, width: geometry.cropWidth, height: geometry.cropHeight,
                _nativeWidth: geometry.sourceWidth, _nativeHeight: geometry.sourceHeight, _rotateMode: geometry.rotation,
                _cropOffsetX: geometry.cropX, _cropOffsetY: geometry.cropY,
                _portraitWidth: swap ? geometry.sourceHeight : geometry.sourceWidth,
                _portraitHeight: swap ? geometry.sourceWidth : geometry.sourceHeight });
            camera.canvas = document.createElement('canvas');
            camera.canvas.width = camera._portraitWidth; camera.canvas.height = camera._portraitHeight;
            camera.ctx = camera.canvas.getContext('2d', { willReadFrequently: true });
            return camera;
        };
        const packet = (video, geometry, seq, generation = 1) => ({ video, configuration: {}, captureGeneration: generation,
            geometry: { ...geometry, captureWidth: geometry.cropWidth, captureHeight: geometry.cropHeight,
                width: geometry.cropWidth, height: geometry.cropHeight, clientEpoch: 1 },
            timing: { timestampS: seq / 10, presentedFrames: seq, physicalExposureVerified: false } });
        const createWorker = (delay = 0, fault = '') => new Worker(`/delayed-worker.js?delay=${delay}&fault=${fault}`, { type: 'module' });
        const cases = [];
        for (const [sourceWidth, sourceHeight] of [[12,8],[8,12]]) {
            const source = await makeVideo(sourceWidth, sourceHeight);
            try {
                for (const rotation of ['none','cw','ccw','half']) for (const cropped of [false,true]) {
                    const swap = ['cw','ccw'].includes(rotation), width = swap ? sourceHeight : sourceWidth, height = swap ? sourceWidth : sourceHeight;
                    const g = { sourceWidth, sourceHeight, rotation, cropX: cropped ? 1 : 0, cropY: cropped ? 2 : 0,
                        cropWidth: cropped ? width-3 : width, cropHeight: cropped ? height-4 : height };
                    const camera = makeCamera(source.video, g);
                    let delivered, resolveFrame; const result = new Promise(resolve => { resolveFrame = resolve; });
                    const client = new CaptureWorker({ createWorker: () => createWorker(160), onFrame: value => { delivered=value; resolveFrame(value); } });
                    client.start(); await until(() => client.state === 'ready', 'Worker readiness');
                    let oracle;
                    await source.present(3, () => {
                        oracle = camera._captureGrayscaleCPU().slice();
                        check(client.capture(source.video,packet(source.video,g,3)), 'VideoFrame snapshot must be synchronous');
                    });
                    const advanced = await source.present(9);
                    check(!delivered, 'Video must advance before the delayed Worker reads frame A');
                    const actual = await result;
                    equal(actual.gray, oracle, 'Worker vs existing synchronous grayscale oracle');
                    const independent = [];
                    for (let y=0;y<g.cropHeight;y++) for (let x=0;x<g.cropWidth;x++) {
                        const rx=x+g.cropX, ry=y+g.cropY;
                        const [sx,sy] = rotation==='cw' ? [ry,sourceHeight-1-rx] : rotation==='ccw' ? [sourceWidth-1-ry,rx] :
                            rotation==='half' ? [sourceWidth-1-rx,sourceHeight-1-ry] : [rx,ry];
                        const [r,green,b] = sourceColor(sx,sy,3); independent.push((77*r+150*green+29*b)>>8);
                    }
                    equal(actual.gray, independent, 'Independent numbered/color/rotation/crop pixel oracle');
                    await client.stop();
                    const metrics = client.getMetrics();
                    check(metrics.scheduled===metrics.closedLocal+metrics.closedRemote+metrics.releasedByTermination, 'No outstanding frame resource');
                    check(actual.timing.timestampS===0.3 && actual.captureGeneration===1 && !actual.timing.physicalExposureVerified, 'Preserved timing and generation');
                    cases.push({ geometry:g, pixels:Array.from(actual.gray), advancedPresentedFrames:advanced.metadata.presentedFrames, metrics });
                }
            } finally { source.close(); }
        }
        const source = await makeVideo(12,8);
        const g = { sourceWidth:12,sourceHeight:8,rotation:'none',cropX:1,cropY:2,cropWidth:8,cropHeight:4 };
        const faults = [];
        try {
            for (const fault of ['draw','crash','hang']) {
                const received = [], errors = [];
                const client = new CaptureWorker({ createWorker: () => createWorker(50,fault), timeoutMs:300,
                    onFrame: value => received.push(value), onError: error => errors.push(error.message) });
                client.start(); await until(() => client.state==='ready','fault Worker ready');
                await source.present(4,()=>client.capture(source.video,packet(source.video,g,4)));
                await until(()=>client.state==='failed','fault result');
                check(received.length===0 && errors.length===1,'Failed old frame must not publish or recapture');
                const afterFailure = client.getMetrics();
                check(client.capture(source.video,packet(source.video,g,5))===false,'Next callback must select synchronous fallback');
                check(afterFailure.scheduled===afterFailure.closedLocal+afterFailure.closedRemote+afterFailure.releasedByTermination,'Resource accounting after failure');
                if(fault==='draw')check(afterFailure.closedRemote===1 && afterFailure.releasedByTermination===0,'Draw error closes explicitly');
                else check(afterFailure.closedRemote===0 && afterFailure.releasedByTermination===1,'Fatal/hung realm release cannot claim close');
                faults.push({ fault, errors, metrics:afterFailure }); await client.stop();
            }
            const lifecycleReceived=[];
            let createdWorkers=0;
            const lifecycleClient=new CaptureWorker({createWorker:()=>{createdWorkers++;return createWorker(100)},
                onFrame:value=>lifecycleReceived.push(value)});
            lifecycleClient.start();await until(()=>lifecycleClient.state==='ready','lifecycle Worker ready');
            await source.present(1,()=>lifecycleClient.capture(source.video,packet(source.video,g,1)));
            await source.present(2,()=>lifecycleClient.capture(source.video,packet(source.video,g,2)));
            check(lifecycleClient.getMetrics().inFlight===1&&lifecycleClient.getMetrics().pending===1,'Real stop fixture must own an in-flight and pending frame');
            const draining=lifecycleClient.stop();lifecycleClient.start();
            check(createdWorkers===1&&lifecycleClient.getMetrics().closedLocal===1,'Restart must wait; overwritten pending ownership closes immediately');
            await draining;await until(()=>lifecycleClient.state==='ready','restarted Worker ready');
            check(createdWorkers===2&&lifecycleReceived.length===0&&lifecycleClient.getMetrics().closedRemote===1,'Old remote frame must close before restart with no publication');
            await source.present(3,()=>lifecycleClient.capture(source.video,packet(source.video,g,3,2)));
            await until(()=>lifecycleReceived.length===1,'new realm capture');
            check(lifecycleReceived[0].captureGeneration===2,'Only the restarted generation publishes');
            await lifecycleClient.stop();
            const lifecycle=lifecycleClient.getMetrics();
            check(lifecycle.scheduled===3&&lifecycle.posted===2&&lifecycle.accepted===1&&lifecycle.dropped===2&&
                lifecycle.closedLocal===1&&lifecycle.closedRemote===2&&lifecycle.releasedByTermination===0,'Real stop/drain/restart resource accounting');
            let callback, epoch=1;
            source.video.requestVideoFrameCallback = value => { callback=value; return 1; };
            source.video.cancelVideoFrameCallback = () => { callback=null; };
            const app = new AppContract(), sent=[];
            const camera = makeCamera(source.video,g);
            app._captureWorker.createWorker = () => createWorker(100);
            Object.assign(app, { camera, running:true, _frameWarmupCount:5,_frameLoopStarted:true,_vioInitialized:true,_processScale:1,
                _captureWidth:g.cropWidth,_captureHeight:g.cropHeight,_processWidth:g.cropWidth,_processHeight:g.cropHeight,
                _lastConfigParams:{width:g.cropWidth,height:g.cropHeight},
                _flushAndSendIMU(){},_startIMUFlush(){},_stopIMUFlush(){},updateIMUOverlay(){},
                imu:{stop(){},getRate:()=>60},orientation:{stopListening(){}},
                vio:{workerBusy:true,configured:true,getMetrics:()=>({clientEpoch:epoch}),getLatestResult:()=>null,dispose(){},
                    sendFrame(gray,timestamp){sent.push({pixels:Array.from(gray),timestamp});return true;}} });
            app._startVideoFrameCallbacks(source.video);
            await until(()=>app._captureWorker.state==='ready','App Worker ready');
            let expected, expectedTime;
            for(const version of [1,2,3]) await source.present(version,(now,metadata)=>{
                expected=Array.from(camera._captureGrayscaleCPU()); expectedTime=(metadata.captureTime??metadata.presentationTime)/1000;
                callback(now,metadata);
            });
            await until(()=>app._captureWorker.getMetrics().inFlight===0,'App latest snapshot');
            check(app._captureWorker.getMetrics().coalesced>=1,'Busy conversion must coalesce owned frames');
            app.vio.workerBusy=false; app.vio.onFrameComplete();
            check(sent.length===1 && sent[0].timestamp===expectedTime,'Latest callback time submitted exactly once');
            equal(sent[0].pixels,expected,'App latest gray bytes');
            await source.present(5,(now,metadata)=>callback(now,metadata));
            app._lastConfigParams={...app._lastConfigParams};
            await until(()=>app._captureWorker.getMetrics().inFlight===0,'configuration fence');
            check(sent.length===1,'Old configuration response cannot publish');
            app._startVideoFrameCallbacks(source.video);
            await source.present(6,(now,metadata)=>callback(now,metadata));
            app._clearVideoFrameCapture(); epoch++;
            await until(()=>app._captureWorker.getMetrics().inFlight===0,'generation fence');
            check(sent.length===1,'Old generation response cannot publish');
            const fallbackCases=[];
            for(const fault of ['draw','transfer','constructor','init']) {
                await app._captureWorker.stop();
                app._captureWorker.createWorker=()=>{
                    const worker=createWorker(0,fault==='draw'||fault==='init'?fault:'');
                    if(fault==='transfer') {
                        const send=worker.postMessage.bind(worker);
                        worker.postMessage=(message,transfer)=>send(message,message.type==='frame'?[...transfer,{}]:transfer);
                    }
                    return worker;
                };
                const construct=app._captureWorker.createFrame;
                if(fault==='constructor')app._captureWorker.createFrame=()=>{throw new Error('injected VideoFrame constructor failure')};
                app._startVideoFrameCallbacks(source.video);
                await until(()=>app._captureWorker.state===(fault==='init'?'failed':'ready'),'App fault readiness');
                const reads=camera._captureCount, before=sent.length;
                let oldTime;
                await source.present(7,(now,metadata)=>{oldTime=(metadata.captureTime??metadata.presentationTime)/1000;callback(now,metadata)});
                await until(()=>app._captureWorker.state==='failed','App failed capture');
                if(fault==='draw'||fault==='transfer') {
                    check(camera._captureCount===reads && sent.length===before,'Failed consumed callback cannot be synchronously reread');
                    let newTime,wanted;
                    await source.present(8,(now,metadata)=>{
                        wanted=Array.from(camera._captureGrayscaleCPU());newTime=(metadata.captureTime??metadata.presentationTime)/1000;callback(now,metadata);
                    });
                    check(sent.length===before+1 && sent.at(-1).timestamp===newTime && newTime!==oldTime,'Only the next new callback may fall back');
                    equal(sent.at(-1).pixels,wanted,'New-callback synchronous fallback pixels');
                } else check(camera._captureCount===reads+1 && sent.length===before+1 && sent.at(-1).timestamp===oldTime,'Pre-transfer failure may capture synchronously in the current callback');
                fallbackCases.push({fault,metrics:app._captureWorker.getMetrics(),syncReads:camera._captureCount-reads});
                app._captureWorker.createFrame=construct;
                await pause(30);
            }
            const appMetrics=app._captureWorker.getMetrics();
            app.stop(); await app._captureWorker.stop();
            const finalMetrics=app._captureWorker.getMetrics();
            check(finalMetrics.scheduled===finalMetrics.closedLocal+finalMetrics.closedRemote+finalMetrics.releasedByTermination,'App resources closed');
            return { passed:true, pixelCases:cases, faults, lifecycle, app:{sent,appMetrics,finalMetrics,fallbackCases},
                secureContext:isSecureContext, support:{VideoFrame:typeof VideoFrame,OffscreenCanvas:typeof OffscreenCanvas,Worker:typeof Worker} };
        } finally { source.close(); }
    });
    assert.equal(report.result.passed,true); assert.equal(report.result.pixelCases.length,16);
    report.passed=true;
} catch (error) { report.passed=false; report.error=error.stack; process.exitCode=1; }
finally {
    if(browser)await browser.close(); await new Promise(resolve=>server.close(resolve));
    await fs.writeFile(path.join(output,'report.json'),JSON.stringify(report,null,2)+'\n');
}
console.log(JSON.stringify({passed:report.passed,browser:report.browser,output,pixelCases:report.result?.pixelCases.length,error:report.error},null,2));
