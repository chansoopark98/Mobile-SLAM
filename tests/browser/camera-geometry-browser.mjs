/** Actual synthetic HTMLVideoElement/canvas geometry; no physical camera or sensor. */
import fs from 'node:fs/promises';
import path from 'node:path';
import http from 'node:http';
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { createHash } from 'node:crypto';

const root = process.cwd();
const { chromium } = createRequire(path.join(root, 'scripts/dev/package.json'))('playwright');
const output = path.resolve(process.env.CAMERA_GEOMETRY_OUTPUT || 'build/mobile-web-20261009/path-audit/camera-geometry-browser');
const source = await fs.readFile(path.join(root, 'web/js/camera.js'), 'utf8');
const report = { scope: 'Desktop synthetic canvas captureStream, actual HTMLVideoElement and Camera CPU/WebGL pixels; physical device calibration unverified',
    cameraSha256: createHash('sha256').update(source).digest('hex'), scenarios: [], errors: [] };
const server = http.createServer((request, response) => {
    const module = request.url === '/camera.js';
    response.setHeader('Content-Type', module ? 'application/javascript' : 'text/html');
    response.end(module ? source : '<!doctype html><title>Camera pixel geometry regression</title>');
});
await fs.mkdir(output, { recursive: true });
await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
const browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true, args: ['--enable-unsafe-swiftshader'] });
report.browser = browser.version();
try {
    const page = await browser.newPage();
    page.on('pageerror', error => report.errors.push(error.message));
    const goldens = {
        none: [10,20,30,40,50,60], cw: [50,30,10,60,40,20],
        ccw: [20,40,60,10,30,50], half: [60,50,40,30,20,10],
    };
    const resizedGoldens = {
        none: [10,20,30,40,50,60], cw: [40,10,50,20,60,30],
        ccw: [30,60,20,50,10,40], half: [60,50,40,30,20,10],
    };
    const cases = [];
    for (const webgl of [false,true]) for (const rotation of Object.keys(goldens)) {
        cases.push({ rotation, webgl, width: 2, height: 3, values: [10,20,30,40,50,60], expected: goldens[rotation], resize: true });
    }
    for (const darkEdge of [false,true]) cases.push({ rotation: null, webgl: false, width: 40, height: 30,
        values: Array.from({ length: 1200 }, (_, index) => darkEdge && index % 40 >= 38 ? 0 : 120),
        expected: Array.from({ length: 1200 }, (_, index) => darkEdge && index % 40 >= 38 ? 0 : 120) });
    for (const webgl of [false,true]) cases.push({ rotation: 'none', webgl, width: 12, height: 6,
        values: Array.from({ length: 72 }, (_, index) => index + 10), crop: true,
        expected: Array.from({ length: 48 }, (_, index) => Math.floor(index / 8) * 12 + index % 8 + 12) });
    for (const scenario of cases) {
        const search = new URLSearchParams();
        if (scenario.rotation) search.set('rotate', scenario.rotation);
        if (scenario.webgl) search.set('grayscale', 'webgl');
        await page.goto(`http://127.0.0.1:${server.address().port}/?${search}`);
        const observed = await page.evaluate(async scenario => {
            const { Camera } = await import('/camera.js');
            const canvas = document.createElement('canvas');
            canvas.width = scenario.width; canvas.height = scenario.height;
            const context = canvas.getContext('2d');
            const pixels = (values, width, height) => {
                const image = context.createImageData(width,height);
                values.forEach((value,index) => { image.data.set([value,value,value,255],index*4); });
                return image;
            };
            let image = pixels(scenario.values,scenario.width,scenario.height);
            const render = () => context.putImageData(image,0,0);
            render(); const timer = setInterval(render,33), stream = canvas.captureStream(30);
            Object.defineProperty(navigator.mediaDevices,'getUserMedia',{configurable:true,value:async()=>stream});
            const camera = new Camera();
            const waitEvent = (target, name) => new Promise((resolve,reject) => {
                const timer = setTimeout(()=>reject(new Error(`${name} timeout`)),3000);
                target.addEventListener(name,()=>{clearTimeout(timer);resolve();},{once:true});
            });
            const frame = () => new Promise((resolve,reject) => {
                const timer = setTimeout(()=>reject(new Error('video frame timeout')),3000);
                camera.video.requestVideoFrameCallback(()=>{clearTimeout(timer);resolve();});
            });
            try {
                await camera.initialize(); if (scenario.crop) camera.enableLandscapeCrop(); await frame();
                const result = { natural: [camera.video.videoWidth,camera.video.videoHeight], output: [camera.width,camera.height],
                    rotation: camera.getRotateMode(), transform: camera.getPixelTransform(), webgl: camera._useWebGL,
                    pixels: Array.from(camera.captureGrayscale()) };
                if (scenario.resize) {
                    const resized = waitEvent(camera.video,'resize');
                    canvas.width = 3; canvas.height = 2; image = pixels([10,20,30,40,50,60],3,2); render();
                    await resized; await camera.redetectOrientation(); await frame();
                    result.resized = { natural: [camera.video.videoWidth,camera.video.videoHeight], output: [camera.width,camera.height],
                        transform: camera.getPixelTransform(), pixels: Array.from(camera.captureGrayscale()) };
                }
                return result;
            } finally { clearInterval(timer); camera.stop(); stream.getTracks().forEach(track=>track.stop()); }
        }, scenario);
        report.scenarios.push({ rotation: scenario.rotation, webgl: scenario.webgl, crop: !!scenario.crop, observed });
        assert.deepEqual(observed.natural,[scenario.width,scenario.height]);
        assert.deepEqual([observed.transform.sourceWidth,observed.transform.sourceHeight],observed.natural);
        assert.equal(observed.rotation,scenario.rotation || 'none');
        assert.equal(observed.webgl,scenario.webgl);
        assert.deepEqual(observed.pixels,scenario.expected);
        if (scenario.resize) {
            assert.deepEqual(observed.resized.natural,[3,2]);
            assert.deepEqual([observed.resized.transform.sourceWidth,observed.resized.transform.sourceHeight],[3,2]);
            assert.deepEqual(observed.resized.pixels,resizedGoldens[scenario.rotation]);
        }
        if (scenario.crop) assert.deepEqual([observed.transform.cropX,observed.transform.cropY,...observed.output],[2,0,8,6]);
    }
    assert.deepEqual(report.errors,[]); report.passed = true;
} catch (error) { report.passed = false; report.error = error.stack; process.exitCode = 1; }
finally {
    await browser.close(); await new Promise(resolve=>server.close(resolve));
    await fs.writeFile(path.join(output,'report.json'),JSON.stringify(report,null,2)+'\n');
    console.log(JSON.stringify({ passed: report.passed, scenarios: report.scenarios.length, output, error: report.error },null,2));
}
