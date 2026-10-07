/** Isolated browser module/API smoke for a specified candidate, without deployment. */
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import http from 'node:http';
import { createHash } from 'node:crypto';
import { chromium } from 'playwright';
const candidate = path.resolve(process.argv[2] || 'build/refactor-wasm/vio_engine.js');
const output = path.resolve(process.argv[3] || 'build/refactor-evaluation/wasm-candidate-smoke.json');
const bytes = await fs.readFile(candidate);
const server = http.createServer((request, response) => {
    response.setHeader('Cross-Origin-Opener-Policy', 'same-origin');
    response.setHeader('Cross-Origin-Embedder-Policy', 'require-corp');
    response.setHeader('Cache-Control', 'no-store');
    if (request.url === '/candidate.js') { response.setHeader('Content-Type', 'application/javascript'); response.end(bytes); }
    else if (request.url === '/') { response.setHeader('Content-Type', 'text/html'); response.end('<!doctype html><title>WASM candidate smoke</title>'); }
    else { response.writeHead(404); response.end(); }
});
await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
let browser;
try {
    browser = await chromium.launch({ executablePath: '/usr/bin/google-chrome', headless: true });
    const page = await browser.newPage();
    const failures=[];
    page.on('pageerror',error=>failures.push(error.message));
    await page.goto(`http://127.0.0.1:${server.address().port}/`);
    const result = await page.evaluate(async () => {
        const factory=(await import('/candidate.js')).default;
        const module=await factory();
        const engine=new module.VIOEngine();
        for(const method of ['configure','processFrame','getEpoch','getPoseTimestamp','getFrameTimestamp','getPoseFresh','getPoseValid','getLastReason','getLastSolverIterations','getLastSolverTermination','setExecutionParams','getExecutionSeed','getCVThreadCount','getIMUEndpointTimestamp','setDiagnosticCapture','getFeatureDiagnostics','setBenchmarkSolverProfile','getBenchmarkSolverProfile','reset'])
            if(typeof engine[method]!=='function')throw new Error('Missing mandatory binding '+method);
        const rotation=module._malloc(72),translation=module._malloc(24);
        module.HEAPF64.set([1,0,0,0,1,0,0,0,1],rotation/8);module.HEAPF64.set([0,0,0],translation/8);
        if(!engine.configure(96,96,100,100,48,48,2,0,0,0,0,rotation,translation,.1,.001,.01,.0001,9.81))throw new Error('Calibrated configure rejected');
        if(engine.getBenchmarkSolverProfile()!==false)throw new Error('Benchmark solver profile must default OFF');
        engine.setBenchmarkSolverProfile(true);
        if(engine.getBenchmarkSolverProfile()!==true)throw new Error('Benchmark profile enable did not apply');
        if(JSON.parse(engine.getFeatureDiagnostics()).enabled!==false)throw new Error('Diagnostic capture must default OFF');
        if(engine.getIMUEndpointTimestamp()!==-1)throw new Error('Unknown IMU endpoint must be invalid');
        engine.setExecutionParams(0,1);
        const configuredEpoch=Number(engine.getEpoch());
        if(engine.getExecutionSeed()!==0 || engine.getCVThreadCount()!==1)throw new Error('Execution profile did not apply');
        if(engine.getPoseFresh() || engine.getPoseValid() || engine.getPoseTimestamp()!==-1)throw new Error('Initial state is not invalid/unmeasured');
        engine.setDiagnosticCapture(true);
        const armedDiagnostic=JSON.parse(engine.getFeatureDiagnostics());
        if(armedDiagnostic.enabled!==true)throw new Error('Selected diagnostic arm must be valid JSON');
        const image=module._malloc(96*96),imu=module._malloc(112),pose=module._malloc(128);
        module.HEAPU8.fill(0,image,image+96*96);
        module.HEAPF64.set([0,0,0,9.81,0,0,0,.01,0,0,9.81,0,0,0],imu/8);
        engine.processFrame(image,96,96,imu,2,.005,pose);
        const warmupDiagnostic=JSON.parse(engine.getFeatureDiagnostics());
        if(warmupDiagnostic.enabled!==true)throw new Error('Warmup diagnostic must be valid JSON');
        engine.reset();
        if(engine.getBenchmarkSolverProfile()!==true)throw new Error('Reset must preserve explicitly enabled benchmark profile');
        const resetDiagnostic=JSON.parse(engine.getFeatureDiagnostics());
        if(resetDiagnostic.enabled!==true)throw new Error('Reset must preserve explicit capture flag and valid JSON');
        if(!engine.configure(96,96,100,100,48,48,2,0,0,0,0,rotation,translation,.1,.001,.01,.0001,9.81))throw new Error('Reconfigure rejected');
        if(engine.getBenchmarkSolverProfile()!==true)throw new Error('Reconfigure must preserve explicitly enabled benchmark profile');
        engine.setBenchmarkSolverProfile(false);
        engine.reset();
        if(engine.getBenchmarkSolverProfile()!==false)throw new Error('Explicit disable must survive reset');
        engine.setDiagnosticCapture(false);
        if(JSON.parse(engine.getFeatureDiagnostics()).enabled!==false)throw new Error('Disarmed diagnostic must be valid JSON');
        module._free(image);module._free(imu);module._free(pose);
        const resetEpoch=Number(engine.getEpoch());
        if(resetEpoch<=configuredEpoch || engine.getExecutionSeed()!==0 || engine.getCVThreadCount()!==1)throw new Error('Reset profile/epoch mismatch');
        engine.delete();module._free(rotation);module._free(translation);
        return {configuredEpoch,resetEpoch,executionSeed:0,cvThreads:1,mandatoryBindings:true,diagnosticCaptureDefault:false,diagnosticArmWarmupResetJson:true,benchmarkSolverProfileDefault:false,benchmarkEnableResetReconfigureDisable:true,initialPose:'invalid',acceptanceScope:'loader_and_public_contracts_only_not_tracking_or_accuracy'};
    });
    assert.deepEqual(failures,[]);
    await fs.mkdir(path.dirname(output),{recursive:true});
    await fs.writeFile(output,JSON.stringify({status:'pass',candidate,sha256:createHash('sha256').update(bytes).digest('hex'),...result},null,2)+'\n');
    console.log(JSON.stringify(result));
} finally {
    if(browser)await browser.close();
    await new Promise(resolve=>server.close(resolve));
}
