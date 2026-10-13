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
        for(const method of ['configure','processFrame','getEpoch','getPoseTimestamp','getFrameTimestamp','getPoseFresh','getPoseValid','getLastReason','getLastSolverIterations','getLastSolverTermination','setExecutionParams','getExecutionSeed','getCVThreadCount','getIMUEndpointTimestamp','setDiagnosticCapture','getFeatureDiagnostics','getPipelineDiagnostics','setBenchmarkSolverProfile','getBenchmarkSolverProfile','setExperimentalObservationPolicy','getExperimentalObservationPolicy','setRotationRescue','getRotationRescueEnabled','reset'])
            if(typeof engine[method]!=='function')throw new Error('Missing mandatory binding '+method);
        if(engine.getExperimentalObservationPolicy()!==0)throw new Error('Experimental observation policy must default to legacy0');
        const rotation=module._malloc(72),translation=module._malloc(24);
        module.HEAPF64.set([1,0,0,0,1,0,0,0,1],rotation/8);module.HEAPF64.set([0,0,0],translation/8);
        if(!engine.configure(96,96,100,100,48,48,2,0,0,0,0,rotation,translation,.1,.001,.01,.0001,9.81))throw new Error('Calibrated configure rejected');
        if(engine.getBenchmarkSolverProfile()!==false)throw new Error('Benchmark solver profile must default OFF');
        if(engine.getRotationRescueEnabled()!==false)throw new Error('Experimental rotation rescue must default OFF');
        engine.setRotationRescue(true);
        if(engine.getRotationRescueEnabled()!==true)throw new Error('Rotation rescue enable did not apply');
        engine.setBenchmarkSolverProfile(true);
        if(engine.getBenchmarkSolverProfile()!==true)throw new Error('Benchmark profile enable did not apply');
        if(JSON.parse(engine.getFeatureDiagnostics()).enabled!==false)throw new Error('Diagnostic capture must default OFF');
        if(engine.getIMUEndpointTimestamp()!==-1)throw new Error('Unknown IMU endpoint must be invalid');
        engine.setExecutionParams(0,1);
        const configuredEpoch=Number(engine.getEpoch());
        if(engine.getExecutionSeed()!==0 || engine.getCVThreadCount()!==1)throw new Error('Execution profile did not apply');
        if(engine.getPoseFresh() || engine.getPoseValid() || engine.getPoseTimestamp()!==-1)throw new Error('Initial state is not invalid/unmeasured');
        if(engine.getPipelineDiagnostics()!=='null')throw new Error('Unrun pipeline must be null');
        engine.setDiagnosticCapture(true);
        const armedDiagnostic=JSON.parse(engine.getFeatureDiagnostics());
        if(armedDiagnostic.enabled!==true)throw new Error('Selected diagnostic arm must be valid JSON');
        const checkPolicy=(expected)=>{
            const params=JSON.parse(engine.getFeatureDiagnostics()).params;
            if(engine.getExperimentalObservationPolicy()!==expected || params.experimental_observation_policy_selected!==expected || params.experimental_observation_policy_effective!==expected)
                throw new Error('Experimental observation policy selected/effective mismatch');
        };
        checkPolicy(0);
        let policyEpoch=Number(engine.getEpoch());engine.setExperimentalObservationPolicy(0);
        if(Number(engine.getEpoch())!==policyEpoch)throw new Error('Same experimental policy must not reset epoch');
        engine.setExperimentalObservationPolicy(1);checkPolicy(1);
        if(Number(engine.getEpoch())!==policyEpoch+1)throw new Error('Policy change must create a fresh epoch');
        policyEpoch=Number(engine.getEpoch());engine.setExperimentalObservationPolicy(1);
        if(Number(engine.getEpoch())!==policyEpoch)throw new Error('Repeated experimental policy must be a no-op');
        for(const invalid of [-1,3,2147483647]) {
            let rejected=false;try{engine.setExperimentalObservationPolicy(invalid);}catch{rejected=true;}
            if(!rejected || Number(engine.getEpoch())!==policyEpoch)throw new Error('Invalid experimental policy mutated epoch or was accepted');
            checkPolicy(1);
        }
        engine.setExperimentalObservationPolicy(2);checkPolicy(2);
        if(Number(engine.getEpoch())!==policyEpoch+1 || engine.getExecutionSeed()!==0 || engine.getCVThreadCount()!==1)
            throw new Error('Experimental policy change lost execution profile or epoch');
        const image=module._malloc(96*96),imu=module._malloc(112),pose=module._malloc(128);
        module.HEAPU8.fill(0,image,image+96*96);
        module.HEAPF64.set([0,0,0,9.81,0,0,0,.01,0,0,9.81,0,0,0],imu/8);
        engine.processFrame(image,96,96,imu,2,.005,pose);
        const pipeline=JSON.parse(engine.getPipelineDiagnostics());
        if(pipeline.frameTimestamp!==.005 || !Number.isFinite(pipeline.imuMs) || !Number.isFinite(pipeline.frontendMs))throw new Error('Pipeline frame identity/timing missing');
        const warmupDiagnostic=JSON.parse(engine.getFeatureDiagnostics());
        if(warmupDiagnostic.enabled!==true)throw new Error('Warmup diagnostic must be valid JSON');
        engine.reset();
        checkPolicy(2);
        if(engine.getPipelineDiagnostics()!=='null')throw new Error('External reset must clear pipeline');
        if(engine.getRotationRescueEnabled()!==true)throw new Error('Reset must preserve rotation rescue selection');
        if(engine.getBenchmarkSolverProfile()!==true)throw new Error('Reset must preserve explicitly enabled benchmark profile');
        const resetDiagnostic=JSON.parse(engine.getFeatureDiagnostics());
        if(resetDiagnostic.enabled!==true)throw new Error('Reset must preserve explicit capture flag and valid JSON');
        if(!engine.configure(96,96,100,100,48,48,2,0,0,0,0,rotation,translation,.1,.001,.01,.0001,9.81))throw new Error('Reconfigure rejected');
        checkPolicy(0);
        if(engine.getRotationRescueEnabled()!==false)throw new Error('Configure must reset rotation rescue to conservative default');
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
        return {configuredEpoch,resetEpoch,executionSeed:0,cvThreads:1,mandatoryBindings:true,diagnosticCaptureDefault:false,diagnosticArmWarmupResetJson:true,benchmarkSolverProfileDefault:false,benchmarkEnableResetReconfigureDisable:true,experimentalObservationPolicyDefault:0,experimentalPolicyReadbackResetConfigureRangeEpoch:true,rotationRescueDefault:false,rotationRescueEnableResetConfigure:true,initialPose:'invalid',acceptanceScope:'loader_and_public_contracts_only_not_tracking_or_accuracy'};
    });
    assert.deepEqual(failures,[]);
    await fs.mkdir(path.dirname(output),{recursive:true});
    await fs.writeFile(output,JSON.stringify({status:'pass',candidate,sha256:createHash('sha256').update(bytes).digest('hex'),...result},null,2)+'\n');
    console.log(JSON.stringify(result));
} finally {
    if(browser)await browser.close();
    await new Promise(resolve=>server.close(resolve));
}
