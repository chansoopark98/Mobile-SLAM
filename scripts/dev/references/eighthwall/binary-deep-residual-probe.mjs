// Pinned 8th Wall component experiment. Original SDK is read-only.
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
const args=process.argv.slice(2);
if(args.length!==4||args[0]!=='--input'||args[2]!=='--output') throw Error('--input pinned.wasm --output NEW_DIRECTORY');
const input=path.resolve(args[1]), output=path.resolve(args[3]);
if(fs.existsSync(output)) throw Error('Output exists; preserve evidence');
const parent=fs.realpathSync(path.dirname(output));
if(parent==='/mnt/backup/SLAM'||parent.startsWith('/mnt/backup/SLAM/')) throw Error('Dataset read-only');
const sha=b=>crypto.createHash('sha256').update(b).digest('hex');
const original=fs.readFileSync(input);
if(sha(original)!=='780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422') throw Error('Pinned original mismatch');
fs.mkdirSync(output);
fs.copyFileSync(new URL(import.meta.url),path.join(output,'probe.mjs'));
const leb=n=>{const out=[];do{let v=n&127;n>>>=7;if(n)v|=128;out.push(v);}while(n);return Buffer.from(out);};
const read=(b,p)=>{let n=0,s=0,v;do{v=b[p++];n|=(v&127)<<s;s+=7;}while(v&128);return[n,p];};
const exported=[['Snavely',1594],['TranslationStability',1601],['Scale3d',1610]];
const parts=[original.subarray(0,8)], sectionReceipts=[];
for(let pos=8;pos<original.length;){const start=pos,id=original[pos++],[length,p]=read(original,pos);pos=p+length;let payload=original.subarray(p,pos);const before=sha(payload);
 if(id===7){const[count,rest]=read(payload,0);payload=Buffer.concat([leb(count+exported.length),payload.subarray(rest),...exported.map(([name,i])=>Buffer.concat([leb(name.length),Buffer.from(name),Buffer.from([0]),leb(i)]))]);parts.push(Buffer.concat([Buffer.from([id]),leb(payload.length),payload]));}else parts.push(original.subarray(start,pos));
 sectionReceipts.push({id,before,after:sha(payload),unchanged:before===sha(payload)});
}
const analytical=Buffer.concat(parts);fs.writeFileSync(path.join(output,'analytical-export-copy.wasm'),analytical);
const module=new WebAssembly.Module(analytical), imports={}, invokedImports=[];
for(const imp of WebAssembly.Module.imports(module)){if(imp.kind!=='function')throw Error('Unexpected import');imports[imp.module]??={};imports[imp.module][imp.name]=(...args)=>{invokedImports.push({name:imp.name,args});throw Error('Unexpected imported function '+imp.name);};}
const instance=new WebAssembly.Instance(module,imports), view=new DataView(instance.exports.memory.buffer);
const O=2**21,F=O+256,P=O+512,A=O+768,B=O+1024,Y=O+1280,J=O+1536,JA=O+1792,JB=O+2304;
const put=(p,a)=>a.forEach((v,i)=>view.setFloat64(p+8*i,v,true));
const get=(p,n)=>Array.from({length:n},(_,i)=>view.getFloat64(p+8*i,true));
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
// Independent Rodrigues expression. Stable Taylor coefficients exercise the zero-angle branch.
function rotate(w,p){const theta2=dot(w,w);let a,b;if(theta2<1e-12){a=1-theta2/6+theta2*theta2/120;b=.5-theta2/24+theta2*theta2/720;}else{const t=Math.sqrt(theta2);a=Math.sin(t)/t;b=(1-Math.cos(t))/theta2;}const c=cross(w,p),cc=cross(w,c);return p.map((v,i)=>v+a*c[i]+b*cc[i]);}
const snavely=(camera,point,f)=>{const q=rotate(camera.slice(0,3),point).map((v,i)=>v+camera[i+3]);return[f[0]*q[0]/q[2]-f[1],f[0]*q[1]/q[2]-f[2]];};
const stability=(camera,_,f)=>rotate(camera.slice(0,3).map(v=>-v),camera.slice(3)).map((v,i)=>(-v-f[i+3]-f[i])*.1);
const scale3d=(scale,_,f)=>f.slice(3).map((v,i)=>scale[0]*v-f[i]);
const oracle={Snavely:snavely,TranslationStability:stability,Scale3d:scale3d};
const rows={Snavely:2,TranslationStability:3,Scale3d:3};
const cases=[];
const add=(name,label,a,b,f,kind='finite')=>cases.push({name,label,a,b,f,kind});
add('Snavely','zero_rotation',[0,0,0,.2,-.3,.4],[1,2,5],[500,25,-50]);
add('Snavely','finite_rotation',[.2,-.3,.4,.2,-.3,.4],[1,2,5],[500,25,-50]);
add('Snavely','tiny_rotation',[1e-12,-2e-12,3e-12,.2,-.3,.4],[1,2,5],[125,1.25,-3.5]);
add('Snavely','large_rotation',[1.7,-.8,2.1,-.2,.3,7],[1,-2,3],[80,-3,4]);
add('Snavely','behind_camera',[0,0,0,0,0,0],[1,2,-5],[500,25,-50]);
add('Snavely','near_zero_depth',[0,0,0,0,0,0],[1,2,1e-8],[500,25,-50],'domain');
add('Snavely','zero_depth',[0,0,0,0,0,0],[1,2,0],[500,25,-50],'domain');
add('Snavely','zero_over_zero',[0,0,0,0,0,0],[0,0,0],[500,25,-50],'domain');
add('TranslationStability','zero_rotation',[0,0,0,4,5,6],[],[.1,.2,.3,1,2,3]);
add('TranslationStability','finite_rotation',[.2,-.3,.4,4,5,6],[],[.1,.2,.3,1,2,3]);
add('TranslationStability','tiny_rotation',[1e-12,-2e-12,3e-12,4,5,6],[],[.1,.2,.3,1,2,3]);
// c1=(0,0,0), c2=(1,2,3), current center=(2,4,6): zero second difference.
add('TranslationStability','constant_world_step',[0,0,0,-2,-4,-6],[],[1,2,3,1,2,3]);
const wr=[.2,-.3,.4],center=[2,4,6],trans=rotate(wr,center).map(v=>-v);
add('TranslationStability','rotated_constant_world_step',[...wr,...trans],[],[1,2,3,1,2,3]);
// Same physical acceleration sampled at 30/60 Hz: temporal spacing is absent from this ABI.
for(const hz of [30,60]){const dt=1/hz,a=3,c2=.5*a*dt*dt,current=2*a*dt*dt;add('TranslationStability','acceleration_'+hz+'hz',[0,0,0,-current,0,0],[],[c2,0,0,c2,0,0]);}
// Constant velocity with unequal sample spacing (0, .01, .03 seconds) yields nonzero second difference.
add('TranslationStability','unequal_spacing_constant_velocity',[0,0,0,-.03,0,0],[],[.01,0,0,.01,0,0]);
add('Scale3d','positive_scale',[2],[],[1,2,3,.5,1,1.5]);
add('Scale3d','nonzero_residual',[1.25],[],[1,-2,3,.5,1,-1.5]);
add('Scale3d','zero_scale',[0],[],[1,2,3,.5,1,1.5]);
add('Scale3d','negative_scale',[-2],[],[1,2,3,.5,1,1.5]);
add('Scale3d','unobservable_scale',[2],[],[1,2,3,0,0,0]);
function expectedJac(c,block){const src=block===0?c.a:c.b, n=src.length, r=rows[c.name];const out=Array(r*n).fill(0);
 if(c.name==='Scale3d')return c.f.slice(3);
 if(c.a.slice(0,3).every(v=>v===0)){
  const point=c.b, t=c.a.slice(3);
  for(let col=0;col<n;col++){
   const e=[0,0,0];e[col%3]=1;
   if(c.name==='Snavely'){
    const q=point.map((v,i)=>v+t[i]);const d=block===1?e:(col<3?cross(e,point):e);
    for(let row=0;row<2;row++)out[row*n+col]=c.f[0]*(d[row]*q[2]-q[row]*d[2])/(q[2]*q[2]);
   }else{const d=col<3?cross(e,t):e.map(v=>-v);for(let row=0;row<3;row++)out[row*n+col]=.1*d[row];}
  }return out;
 }
 // Independent central finite differences of Rodrigues oracle, never finite differences of WASM.
 for(let col=0;col<n;col++){const h=1e-6*Math.max(1,Math.abs(src[col]));const lo=src.slice(),hi=src.slice();lo[col]-=h;hi[col]+=h;
  const l=oracle[c.name](block===0?lo:c.a,block===1?lo:c.b,c.f),u=oracle[c.name](block===0?hi:c.a,block===1?hi:c.b,c.f);
  for(let row=0;row<r;row++)out[row*n+col]=(u[row]-l[row])/(2*h);
 }return out;
}
const same=(actual,expected,absTol,relTol)=>actual.length===expected.length&&actual.every((v,i)=>Number.isNaN(expected[i])?Number.isNaN(v):!Number.isFinite(expected[i])?Object.is(v,expected[i]):Number.isFinite(v)&&Math.abs(v-expected[i])<=absTol+relTol*Math.abs(expected[i]));
const encode=x=>JSON.parse(JSON.stringify(x,(_,v)=>typeof v==='number'&&!Number.isFinite(v)?String(v):v));
const result=[];
for(const c0 of cases){const c={...c0,f:c0.name==='Snavely'?c0.f.map(Math.fround):c0.f};const modes=c.name==='Snavely'?['none','both','first','second','null-blocks']:['none','first','null-blocks'];
 for(const mode of modes){new Uint8Array(instance.exports.memory.buffer,O,4096).fill(0);view.setUint32(O+20,F,true);view.setUint32(P,A,true);view.setUint32(P+4,B,true);put(A,c.a);put(B,c.b);
  if(c.name==='Snavely')c.f.forEach((v,i)=>view.setFloat32(F+4*i,v,true));else put(F+(c.name==='Scale3d'?8:0),c.f);
  const active=[mode==='both'||mode==='first',mode==='both'||mode==='second'];view.setUint32(J,active[0]?JA:0,true);view.setUint32(J+4,active[1]?JB:0,true);
  // Sentinel permits checking null Jacobian blocks do not write either output buffer.
  put(JA,Array(36).fill(123456));put(JB,Array(18).fill(123456));
  const expected=oracle[c.name](c.a,c.b,c.f);let returned,error=null;try{returned=instance.exports[c.name](O,P,Y,mode==='none'?0:J);}catch(e){error=String(e);}
  const actual=get(Y,rows[c.name]),jacobians=[];
  for(let b=0;b<(c.name==='Snavely'?2:1);b++){const actualJ=get(b===0?JA:JB,rows[c.name]*(b===0?c.a.length:c.b.length));const expectedJ=active[b]?expectedJac(c,b):Array(actualJ.length).fill(123456);jacobians.push({block:b,requested:active[b],expected:expectedJ,actual:actualJ,method:active[b]?(c.name==='Scale3d'||c.a.slice(0,3).every(v=>v===0)?'analytic':'central-difference-independent-Rodrigues'):'write-sentinel',pass:same(actualJ,expectedJ,2e-5,2e-7)});}
  const pass=!error&&returned===1&&same(actual,expected,1e-9,1e-12)&&jacobians.every(j=>j.pass);
  result.push(encode({name:c.name,label:c.label,kind:c.kind,mode,input:{a:c.a,b:c.b,functor:c.f},returned,error,expected,actual,jacobians,pass}));
 }
}
const summary={time_utc:new Date().toISOString(),node:process.version,input,script_sha256:sha(fs.readFileSync(new URL(import.meta.url))),original_sha256:sha(original),analytical_sha256:sha(analytical),exported,sections:sectionReceipts,non_export_sections_unchanged:sectionReceipts.filter(s=>s.id!==7).every(s=>s.unchanged),code_section_sha256:sectionReceipts.find(s=>s.id===10).before,import_count:WebAssembly.Module.imports(module).length,invokedImports,residual_tolerance:{abs:1e-9,relative:1e-12},jacobian_tolerance:{abs:2e-5,relative:2e-7},count:result.length,passed:result.filter(r=>r.pass).length,results:result,pass:result.every(r=>r.pass)&&!invokedImports.length,limits:['Constructed component ABI calls, no SDK lifecycle/camera execution','Only export section changed; all original code/data bytes retained','Residual algebra/Jacobians do not establish runtime solver dispatch, weighting or field accuracy','Nonfinite depth cases characterize residual domain; they are not accepted trajectory tests']};
fs.writeFileSync(path.join(output,'result.json'),JSON.stringify(summary,null,2)+'\n');
console.log(JSON.stringify({count:summary.count,passed:summary.passed,pass:summary.pass,imports:invokedImports,failed:result.filter(r=>!r.pass)},null,2));
if(!summary.pass)process.exitCode=1;
