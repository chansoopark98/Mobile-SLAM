/** Independent A/tA -> mutable B -> RAF counterexample with real video/canvas. */
import fs from 'node:fs/promises';
import path from 'node:path';
import http from 'node:http';
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { createHash } from 'node:crypto';

const root=process.cwd();
const output=path.resolve(process.env.VIDEO_SNAPSHOT_OUTPUT || 'build/mobile-web-20261009/phase-c/actual-video');
const {chromium}=createRequire(path.join(root,'scripts/dev/package.json'))('playwright');
const cameraSource=await fs.readFile(path.join(root,'web/js/camera.js'),'utf8');
const appSource=await fs.readFile(path.join(root,'web/js/app.js'),'utf8');
const report={scope:'Actual desktop HTMLVideoElement/canvas and App callback/processLoop; deterministic withheld B metadata, no engine or physical exposure/phone accuracy claim',
    hashes:Object.fromEntries([['camera',cameraSource],['app',appSource]].map(([name,source])=>[name,createHash('sha256').update(source).digest('hex')]))};
const server=http.createServer((request,response)=>{
    response.setHeader('Content-Type',request.url==='/camera.js'?'application/javascript':'text/html');
    response.end(request.url==='/camera.js'?cameraSource:'<!doctype html><title>Callback snapshot contract</title>');
});
await fs.mkdir(output,{recursive:true});await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
let browser;
try{
    browser=await chromium.launch({executablePath:process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome',headless:true});
    report.browser=browser.version();const page=await browser.newPage();await page.goto(`http://127.0.0.1:${server.address().port}/?rotate=none`);
    await page.evaluate(async()=>{
        Object.assign(window,await import('/camera.js'));
        window.VIOWrapper=class {};window.IMU=class {};window.OrientationHandler=class {};
    });
    await page.addScriptTag({content:appSource.replace(/^import .*$/gm,'').replace(/const app = new App\(\);[\s\S]*$/,'')+'\nwindow.AppContract=App;'});
    report.result=await page.evaluate(async()=>{
        const canvas=document.createElement('canvas');canvas.width=40;canvas.height=30;
        const context=canvas.getContext('2d');let color=40;
        const paint=()=>{context.fillStyle=`rgb(${color},${color},${color})`;context.fillRect(0,0,40,30);};
        paint();const timer=setInterval(paint,33),stream=canvas.captureStream(30);
        Object.defineProperty(navigator.mediaDevices,'getUserMedia',{configurable:true,value:async()=>stream});
        const camera=new Camera();await camera.initialize();const video=camera.getVideoElement();
        const requestNative=video.requestVideoFrameCallback.bind(video);
        const nativeFrame=()=>new Promise((resolve,reject)=>{const timeout=setTimeout(()=>reject(new Error('native frame timeout')),3000);
            requestNative((now,metadata)=>{clearTimeout(timeout);resolve({now,metadata});});});
        let pendingCallback,callbackId=0;
        video.requestVideoFrameCallback=callback=>{pendingCallback=callback;return ++callbackId;};
        video.cancelVideoFrameCallback=()=>{pendingCallback=null;};
        const app=new AppContract(),sent=[];
        Object.assign(app,{camera,running:true,_frameWarmupCount:5,_processScale:1,_captureWidth:40,_captureHeight:30,
            _processWidth:40,_processHeight:30,_lastConfigParams:{width:40,height:30},lastFPSTime:performance.now(),lastImuUpdateTime:performance.now(),
            _flushAndSendIMU(){},_startIMUFlush(){},_stopIMUFlush(){},updateIMUOverlay(){},
            imu:{stop(){},getRate:()=>60},orientation:{stopListening(){}},
            vio:{getMetrics:()=>({clientEpoch:1}),getLatestResult:()=>null,dispose(){},
                sendFrame(gray,timestamp){sent.push({timestamp,pixels:Array.from(gray)});return true;}}});
        try{
            app._startVideoFrameCallbacks(video);
            const frameA=await nativeFrame();pendingCallback(frameA.now,frameA.metadata);
            const timingA={...app._videoFrameTiming};
            color=180;paint();let observedB;
            for(let attempt=0;attempt<5;attempt++){await nativeFrame();observedB=Array.from(camera.captureGrayscale());if(observedB.every(value=>value===180))break;}
            const readsBeforeRAF=camera._captureCount;
            app.processLoop();const readsAfterRAF=camera._captureCount;
            return{frameA,timingA,observedB,sent,readsBeforeRAF,readsAfterRAF,metrics:{...app.frameMetrics},generation:app.captureGeneration};
        }finally{app.stop();clearInterval(timer);stream.getTracks().forEach(track=>track.stop());}
    });
    const r=report.result;
    assert.ok(r.observedB.every(value=>value===180),'Video must genuinely advance to B before RAF');
    assert.equal(r.sent.length,1);assert.ok(r.sent[0].pixels.every(value=>value===40),'Submitted pixels must remain A');
    const sourceTime=r.frameA.metadata.captureTime ?? r.frameA.metadata.presentationTime;
    assert.equal(r.sent[0].timestamp,sourceTime/1000);
    assert.equal(r.readsAfterRAF,r.readsBeforeRAF,'RAF cannot reread mutable video');
    assert.equal(r.timingA.captureGeneration,r.generation);assert.equal(r.timingA.physicalExposureVerified,false);
    assert.equal(r.timingA.association,'video_frame_callback_snapshot');report.passed=true;
}catch(error){report.passed=false;report.error=error.stack;process.exitCode=1;}
finally{if(browser)await browser.close();await new Promise(resolve=>server.close(resolve));await fs.writeFile(path.join(output,'report.json'),JSON.stringify(report,null,2)+'\n');}
console.log(JSON.stringify({passed:report.passed,browser:report.browser,output,error:report.error},null,2));
