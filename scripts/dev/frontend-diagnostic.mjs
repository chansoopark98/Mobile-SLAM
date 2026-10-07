/** Short, explicit diagnostic: native-captured pixel/IMU bytes through real WASM engine. */
import fs from 'node:fs/promises';
import path from 'node:path';
import http from 'node:http';
import { createHash } from 'node:crypto';
import { chromium } from 'playwright';
const candidate=path.resolve(process.argv[2]||'build/refactor-wasm/vio_engine.js');
const input=path.resolve(process.argv[3]||'build/refactor-diagnostic/first5-inputs.bin');
const metadata=path.resolve(process.argv[4]||'build/refactor-diagnostic/first5-inputs.json');
const output=path.resolve(process.argv[5]||'build/refactor-diagnostic/frontend-wasm.json');
const candidateBytes=await fs.readFile(candidate),inputBytes=await fs.readFile(input),meta=JSON.parse(await fs.readFile(metadata,'utf8'));
const captureStart=process.argv[6]===undefined?0:Number(process.argv[6]);
const captureEnd=process.argv[7]===undefined?meta.timestamps.length:Number(process.argv[7]);
const benchmarkArguments=process.argv.slice(8);
if(benchmarkArguments.length!==0&&(benchmarkArguments.length!==2||benchmarkArguments[0]!=='--benchmark-zero-tolerances'||!['0','1'].includes(benchmarkArguments[1])))
 throw new Error('Expected optional --benchmark-zero-tolerances 0|1 after capture range');
const benchmarkRequested=benchmarkArguments[1]==='1';
if(!Number.isInteger(captureStart)||!Number.isInteger(captureEnd)||captureStart<0||captureEnd<=captureStart||captureEnd>meta.timestamps.length)
 throw new Error('Invalid diagnostic capture range [start,end)');
meta.diagnosticCaptureRange={startInclusive:captureStart,endExclusive:captureEnd};
meta.benchmarkRequested=benchmarkRequested;
const server=http.createServer((request,response)=>{
 response.setHeader('Cache-Control','no-store');response.setHeader('Cross-Origin-Opener-Policy','same-origin');response.setHeader('Cross-Origin-Embedder-Policy','require-corp');
 if(request.url==='/candidate.js'){response.setHeader('Content-Type','application/javascript');response.end(candidateBytes);}
 else if(request.url==='/input.bin'){response.setHeader('Content-Type','application/octet-stream');response.end(inputBytes);}
 else if(request.url==='/'){response.setHeader('Content-Type','text/html');response.end('<!doctype html><title>Explicit frontend diagnostic</title>');}
 else{response.writeHead(404);response.end();}
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));let browser;
try{
 browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true});const page=await browser.newPage();await page.goto(`http://127.0.0.1:${server.address().port}/`);
 const report=await page.evaluate(async meta=>{
  const module=await(await import('/candidate.js')).default();const engine=new module.VIOEngine();const c=meta.calibration;
  const rotation=module._malloc(72),translation=module._malloc(24);module.HEAPF64.set(c.r_ic,rotation/8);module.HEAPF64.set(c.t_ic,translation/8);
  if(!engine.configure(c.width,c.height,c.fx,c.fy,c.cx,c.cy,c.modelType,c.k2,c.k3,c.k4,c.k5,rotation,translation,c.acc_n,c.acc_w,c.gyr_n,c.gyr_w,c.g_norm))throw new Error('Diagnostic configure failed');
  engine.setExecutionParams(0,1);engine.setMobileParams(10,10,150);engine.setTrackingParams(21,3,20,0);engine.setFThreshold(1);engine.setPnPParams(false,3);
  engine.setBenchmarkSolverProfile(meta.benchmarkRequested);
  const benchmarkActual=engine.getBenchmarkSolverProfile();
  if(benchmarkActual!==meta.benchmarkRequested)throw new Error('Benchmark solver profile readback mismatch');
  let diagnosticActive=meta.diagnosticCaptureRange.startInclusive===0;engine.setDiagnosticCapture(diagnosticActive);
  const input=await(await fetch('/input.bin')).arrayBuffer(),view=new DataView(input);let offset=0;const rows=[];
  const initialWasmHeapBytes=module.HEAPU8.byteLength;let peakWasmHeapBytes=initialWasmHeapBytes;
  for(let frame=0;frame<meta.timestamps.length;frame++){
   const selected=frame>=meta.diagnosticCaptureRange.startInclusive&&frame<meta.diagnosticCaptureRange.endExclusive;
   if(selected!==diagnosticActive){engine.setDiagnosticCapture(selected);diagnosticActive=selected;}
   const length=view.getUint32(offset,true);offset+=4;const gray=new Uint8Array(input,offset,length);offset+=length;
   const count=view.getUint32(offset,true);offset+=4;const readings=new Uint8Array(input,offset,count*56);offset+=count*56;
   const image=module._malloc(length),imu=module._malloc(Math.max(count*56,8)),pose=module._malloc(128);
   module.HEAPU8.set(gray,image);module.HEAPU8.set(readings,imu);
   const processStart=performance.now();
   const valid=engine.processFrame(image,c.width,c.height,imu,count,meta.timestamps[frame],pose);
   const engineProcessMs=performance.now()-processStart,wasmHeapBytes=module.HEAPU8.byteLength;
   peakWasmHeapBytes=Math.max(peakWasmHeapBytes,wasmHeapBytes);
   const diagnostic=engine.getFeatureDiagnostics();
   rows.push({frame,inputTimestamp:meta.timestamps[frame],poseTimestamp:engine.getPoseTimestamp(),engineEpoch:Number(engine.getEpoch()),poseFresh:engine.getPoseFresh(),poseValid:engine.getPoseValid(),pose:valid?Array.from(module.HEAPF64.subarray(pose/8,pose/8+16)):null,valid,statusCode:engine.getStatusCode(),reason:engine.getLastReason(),solverIterations:engine.getLastSolverIterations(),solverTermination:engine.getLastSolverTermination(),featureCount:engine.getFeaturePointCount(),engineProcessMs,wasmHeapBytes,featureDiagnostics:typeof diagnostic==='string'?JSON.parse(diagnostic):diagnostic});
   module._free(image);module._free(imu);module._free(pose);
  }
  engine.delete();module._free(rotation);module._free(translation);
  return{schema:'mobile-slam-frontend-diagnostic-v1',scope:'bounded same-input development diagnostic; no performance or held-out acceptance',frameCount:meta.timestamps.length,diagnosticCaptureRange:meta.diagnosticCaptureRange,benchmarkSolverProfile:{requested:meta.benchmarkRequested,actual:benchmarkActual,name:benchmarkActual?'max10_zero_positive_tolerances':'default',maximumIterations:10,countSemantics:'actual Ceres Summary.iterations.size; includes iteration0; maximum10 is not exact10 guarantee'},wasmLinearMemory:{initialBytes:initialWasmHeapBytes,peakBytes:peakWasmHeapBytes,scope:'actual linear-memory capacity high-water; not allocator live bytes or total browser RSS'},rows};
 },meta);
 report.candidateSha256=createHash('sha256').update(candidateBytes).digest('hex');
 report.inputSha256=createHash('sha256').update(inputBytes).digest('hex');
 await fs.writeFile(output,JSON.stringify(report,null,2)+'\n');console.log(output);
}finally{if(browser)await browser.close();await new Promise(resolve=>server.close(resolve));}
