import fs from 'node:fs/promises';
import path from 'node:path';
import http from 'node:http';
import {createHash} from 'node:crypto';
import {fileURLToPath} from 'node:url';
import {parseArgs} from 'node:util';
import {chromium} from '../../node_modules/playwright/index.mjs';
import {parsePGM} from '../../replay-android-input.mjs';
import {bodyToDevice} from '../../synthetic-browser.mjs';
import {buildSensorPackets,bodyToDeviceRow,deviceToBodyRow} from './sensor_adapter.mjs';
const source=path.dirname(fileURLToPath(import.meta.url)),root=path.resolve(source,'../../../..');
const {values}=parseArgs({options:{label:{type:'string'},frames:{type:'string',default:'80'},mode:{type:'string',default:'mobile'},dataset:{type:'string',default:'build/mobile-web-20261009/synthetic/clean-seed101-v2'},scale:{type:'string',default:'responsive'},'camera-source':{type:'string',default:'generated'}}});
if(!/^[a-z0-9-]+$/.test(values.label||''))throw new Error('--label required; one new label for each preserved attempt');
if(!['mobile','desktop'].includes(values.mode))throw new Error('mode mobile or desktop');
if(!['responsive','absolute'].includes(values.scale))throw new Error('scale responsive or absolute');
if(!['generated','canvas'].includes(values['camera-source']))throw new Error('camera-source generated or canvas');
const count=Number(values.frames);if(!Number.isInteger(count)||count<1||count>400)throw new Error('frames1..400');
const out=path.join(root,'build/research-20261012/8thwall-binary/runtime',values.label);await fs.mkdir(out,{recursive:false});
const dataset=path.resolve(root,values.dataset),engine=path.join(root,'assets/references/8thwall/analysis-runtime/standalone');
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
const inputBytes=await fs.readFile(path.join(dataset,'input.json')),input=JSON.parse(inputBytes);if(input.schema!=='mobile-slam-synthetic-vio-v1')throw new Error('Independent synthetic fixture required');
const frames=input.frames.slice(0,count),imu=input.imu.filter(row=>row[0]<=frames.at(-1).physicalTimestamp+0.02),gray=[];
for(const frame of frames){const p=path.join(dataset,'mav0/cam0/data',frame.file);const raw=await fs.readFile(p);const pixels=parsePGM(raw,input.config.width,input.config.height);if(sha(pixels)!==frame.sha256)throw new Error('Input pixel hash mismatch');gray.push(pixels);}
const sensors=buildSensorPackets(imu);let roundtripError=0;
for(const row of imu){const actual=bodyToDeviceRow(row),validated=bodyToDevice(row),back=deviceToBodyRow(actual);for(let k=0;k<7;k++){if(actual[k]!==validated[k])throw new Error('Validated device mapping differs');roundtripError=Math.max(roundtripError,Math.abs(back[k]-row[k]));}}
const fixture={...input,frames,imu,sourceInputSha256:sha(inputBytes),sensorPackets:sensors.packets,sensorProvenance:{...sensors.provenance,roundtripMaximumError:roundtripError},cameraSource:values['camera-source'],scale:values.scale};
const report={schema:'8thwall-binary-runtime-probe-v2',label:values.label,mode:values.mode,scale:values.scale,cameraSource:values['camera-source'],node:process.version,startedAt:new Date().toISOString(),argv:process.argv,input:{dataset,sha256:sha(inputBytes),frames:count,imuPackets:imu.length,pixelHashes:frames.map(f=>f.sha256),sensorProvenance:fixture.sensorProvenance},requests:[],blockedExternal:[],console:[],pageErrors:[],scope:'Desktop emulation and synthetic boundary inputs; no phone accuracy or controlled latency comparison',limits:['Video/texture format conversion and internal K not matched to own-engine fixture','Gyro-only quaternion orientation is not the platform OS-fused orientation stream','Wasm export observer adds overhead; timings are invocation observations only']};
report.sourceHashes={};await fs.mkdir(path.join(out,'source'));for(const name of ['observer.js','runtime.js','index.html','run_browser.mjs','sensor_adapter.mjs']){const bytes=await fs.readFile(path.join(source,name));report.sourceHashes[name]=sha(bytes);await fs.writeFile(path.join(out,'source',name),bytes);}report.validatedMappingSourceSha256=sha(await fs.readFile(path.join(root,'scripts/dev/synthetic-browser.mjs')));
const serve=async(req,res)=>{
  try{const u=new URL(req.url,'http://localhost');let bytes,type='application/octet-stream';
    if(u.pathname==='/fixture.json'){bytes=Buffer.from(JSON.stringify(fixture));type='application/json';}
    else if(/^\/gray\/\d+$/.test(u.pathname)){bytes=gray[Number(u.pathname.split('/').at(-1))];if(!bytes)throw new Error('Unknown frame');}
    else{const relative=u.pathname==='/index.html'||u.pathname==='/'?'index.html':u.pathname.slice(1);const isEngine=relative.startsWith('engine/');const base=isEngine?engine:source;const file=path.resolve(base,isEngine?relative.slice(7):relative);if(!file.startsWith(base+path.sep)||(!isEngine&&!['index.html','observer.js','runtime.js'].includes(relative)))throw new Error('Unserved path');bytes=await fs.readFile(file);type=file.endsWith('.js')?'text/javascript':file.endsWith('.html')?'text/html':'application/octet-stream';}
    res.writeHead(200,{'Content-Type':type,'Cache-Control':'no-store'});res.end(bytes);
  }catch(error){res.writeHead(404);res.end(String(error));}
};
const server=http.createServer((req,res)=>void serve(req,res));let browser;
try{
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));const origin=`http://127.0.0.1:${server.address().port}`;report.origin=origin;
  browser=await chromium.launch({executablePath:process.env.BROWSER_EXECUTABLE||'/usr/bin/google-chrome',headless:true,args:['--enable-unsafe-swiftshader','--autoplay-policy=no-user-gesture-required','--use-fake-ui-for-media-stream','--use-fake-device-for-media-stream']});report.browserVersion=browser.version();
  const options={viewport:{width:320,height:240},permissions:['camera','microphone'],isMobile:values.mode==='mobile',hasTouch:values.mode==='mobile'};
  if(values.mode==='mobile')options.userAgent='Mozilla/5.0 (Linux; Android 13; Pixel 7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/140.0.0.0 Mobile Safari/537.36';
  const context=await browser.newContext(options);const page=await context.newPage();
  await page.route('**/*',route=>{const url=route.request().url();if(!url.startsWith(origin+'/')&&!url.startsWith('data:')&&!url.startsWith('blob:')){report.blockedExternal.push(url);return route.abort('blockedbyclient');}return route.continue();});
  page.on('response',response=>report.requests.push({url:response.url(),status:response.status()}));page.on('console',msg=>report.console.push({type:msg.type(),text:msg.text()}));page.on('pageerror',error=>report.pageErrors.push({message:error.message,stack:error.stack}));
  await page.goto(origin+'/index.html?mode='+values.mode);await page.waitForFunction(()=>window.__probe?.ready,null,{timeout:45000});await page.locator('#start').click();
  try{await page.waitForFunction(()=>window.__probe?.done,null,{timeout:count*50+15000});}catch(error){report.waitError=String(error);}
  await page.evaluate(()=>window.__finishProbe?.());report.observed=await page.evaluate(()=>window.__probe);await page.screenshot({path:path.join(out,'screenshot.png')});
  const calls=report.observed.calls,counts={};for(const c of calls)counts[c.name]=(counts[c.name]||0)+1;
  report.summary={exportCallCounts:counts,rows:report.observed.rows.length,sourceFrames:report.observed.frames.length,imuDispatched:report.observed.imu.length,trackingStates:{},nonzeroPositions:0,finiteTransformCallbacks:0,finiteIntrinsicsCallbacks:0,nonrepeatCallbacks:0};
  for(const row of report.observed.rows){const key=`${row.trackingStatus}/${row.trackingReason}`;report.summary.trackingStates[key]=(report.summary.trackingStates[key]||0)+1;if(row.position&&row.rotation&&Object.values(row.position).concat(Object.values(row.rotation)).every(Number.isFinite))report.summary.finiteTransformCallbacks++;if(row.intrinsics?.length===16&&row.intrinsics.every(Number.isFinite))report.summary.finiteIntrinsicsCallbacks++;if(row.repeatFrame===false)report.summary.nonrepeatCallbacks++;if(row.position&&Math.hypot(row.position.x,row.position.y,row.position.z)>0)report.summary.nonzeroPositions++;}
  report.status=counts.c8EmAsm_processStagedFrame>0?'ACTUAL_SLAM_EXPORT_INVOKED':'NO_SLAM_EXPORT_INVOCATION';
}catch(error){report.status='FAILED';report.error={message:error.message,stack:error.stack};}
finally{if(browser)await browser.close();await new Promise(resolve=>server.close(resolve));report.finishedAt=new Date().toISOString();await fs.writeFile(path.join(out,'result.json'),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify({status:report.status,error:report.error,summary:report.summary,waitError:report.waitError,out},null,2));process.exitCode=report.status==='ACTUAL_SLAM_EXPORT_INVOKED'?0:report.status==='FAILED'?1:2;}
