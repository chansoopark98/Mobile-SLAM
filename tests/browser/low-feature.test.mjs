import test from 'node:test';
import assert from 'node:assert/strict';
import {validateInertialPrediction, validateGroundPlane, fitGroundPlane, GroundPlaneTracker, isCleanVisualContext} from '../../web/js/low-feature.mjs';

const pose = [1,0,0,0,0,1,0,0,0,0,1,1.5,0,0,0,1];
const context = (extra={}) => ({inputTimestamp:2,engineFrameTimestamp:2,engineEpoch:3,clientEpoch:4,sequence:10,statusCode:2,initialized:true,reason:'tracking',solverQualityReason:'',pose:pose.slice(),...extra});
const lost = (extra={}) => context({statusCode:3,reason:'empty_features',pose:null,...extra});
const prediction = (extra={}) => ({schema:'mobile-slam-inertial-prediction-v1',mode:'inertial-only',poseFrame:'camera',engineEpoch:3,timestamp:2,anchorTimestamp:1.6,maxHorizonSeconds:.5,pose:pose.slice(),visualSupported:false,metricTranslationSupported:false,positionUncertain:true,...extra});
// Analytic grid coordinates; known plane z=.07x-.04y+.2 has unit normal [-.07,.04,1]/norm.
function grid({height=.2, slopeX=0, slopeY=0, noise=0,nx=8,ny=8,step=.12}={}) {
 const p=[]; for(let j=0;j<ny;j++)for(let i=0;i<nx;i++){const x=(i-(nx-1)/2)*step,y=(j-(ny-1)/2)*step;p.push(x,y,height+slopeX*x+slopeY*y+noise*((i*7+j*3)%5-2));}return p;
}
const close=(a,b,eps=1e-8)=>assert.ok(Math.abs(a-b)<=eps,`${a} != ${b}`);

test('transport recovery requires a fresh current clean visual pose with homogeneous SO3', () => {
 const good=context({poseFresh:true,poseValid:true,poseTimestamp:2});
 assert.equal(isCleanVisualContext(good),true);
 for(const patch of [{poseFresh:false},{poseValid:false},{poseTimestamp:1.99},{poseTimestamp:2.01},
  {pose:Array(16).fill(0)},{pose:[2,...pose.slice(1)]},{engineFrameTimestamp:1.99},{solverQualityReason:'warning'},
  {reason:'empty_features'},{error:true},{statusCode:3},{clientEpoch:-1}])assert.equal(isCleanVisualContext({...good,...patch}),false);
});

test('analytic tilted plane recovers normal/offset independent of parameterization',()=>{
 const p=fitGroundPlane(grid({slopeX:.07,slopeY:-.04}),context());assert.ok(p);const n=Math.hypot(.07,.04,1);p.normal.forEach((x,i)=>close(x,[-.07/n,.04/n,1/n][i]));close(p.offset,-.2/n);assert.equal(p.supportCount,64);assert.equal(p.semanticClassification,'unknown');assert.equal(p.state,'observed');assert.ok(validateGroundPlane(p,context()));
});
test('deterministic noisy plane rejects independent high outliers',()=>{
 const points=grid({noise:.002});for(let i=0;i<12;i++)points.push(i*.11,.3,.8+i*.06);const a=fitGroundPlane(points,context());assert.ok(a);close(a.offset,-.2,.006);assert.ok(a.rms<.01);assert.deepEqual(a,fitGroundPlane(points,context()));
});
test('table and raised floor have identical geometric result, semantic never invented',()=>{
 const table=fitGroundPlane(grid({height:.8}),context());assert.ok(table);assert.equal(table.semanticClassification,'unknown');close(table.centroid[2],.8);
});
for(const [name,points] of [
 ['vertical wall',grid().flatMap((_,i,a)=>i%3?[]:[a[i],.2,a[i+1]+.2])],
 ['ceiling',grid({height:2})],['sparse',grid({nx:4,ny:4})],
 ['line',Array.from({length:30},(_,i)=>[i*.05,i*.05,.2]).flat()],
 ['cluster',grid({step:.005})],['steep slope',grid({slopeX:1})],
 ['nonfinite',[...grid(),0,0,NaN]],['invalid shape',[...grid(),1]],
 ['equal two planes',[...grid({height:.1}),...grid({height:.8})]],
])test(`reject ${name}`,()=>assert.equal(fitGroundPlane(points,context()),null));
test('duplicates cannot manufacture support and cap512 counts',()=>{
 assert.equal(fitGroundPlane(Array(30).fill(grid({nx:2,ny:2})).flat(),context()),null);
 const p=fitGroundPlane(grid({nx:30,ny:30}),context());assert.ok(p);assert.equal(p.sourcePointCount,512);assert.ok(p.supportCount<=512);
});
test('no plane fitting without exact clean visual identity and valid camera pose',()=>{
 for(const patch of [{statusCode:3,reason:'empty_features'},{initialized:false},{solverQualityReason:'relative_translation_threshold'},{solverQualityReason:null},{reason:'tracking:warning'},{engineFrameTimestamp:1.99},{error:true},{motionDiscontinuity:{}},{transportLost:true},{pose:[...pose.slice(0,15),0]}])assert.equal(fitGroundPlane(grid(),context(patch)),null);
});
test('prediction has separate validity and exact native schema; copied values',()=>{
 const value=prediction(),out=validateInertialPrediction(value,lost());assert.ok(out);assert.notEqual(out.pose,value.pose);assert.equal(out.visualSupported,false);assert.equal(validateInertialPrediction(value,context()),null);
});
test('prediction source horizon boundaries and request identity',()=>{
 assert.ok(validateInertialPrediction(prediction({anchorTimestamp:1.5}),lost()));
 for(const v of [prediction({anchorTimestamp:1.49999}),prediction({anchorTimestamp:2.01}),prediction({timestamp:2.01}),prediction({engineEpoch:2}),prediction({maxHorizonSeconds:1}),prediction({confidence:.9}),prediction({visualSupported:true}),prediction({metricTranslationSupported:true}),prediction({positionUncertain:false})])assert.equal(validateInertialPrediction(v,lost()),null);
 for(const c of [lost({engineFrameTimestamp:1.9}),lost({reason:'insufficient_final_current_features;relative_translation_threshold'}),lost({initialized:false}),lost({solverQualityReason:'relative_translation_threshold'}),lost({solverQualityReason:null}),lost({engineEpoch:-1}),lost({sequence:NaN})])assert.equal(validateInertialPrediction(prediction(),c),null);
});
test('prediction rejects reflection, shear, scaling, NaN and nonhomogeneous poses',()=>{
 for(const [index,value] of [[0,-1],[1,.2],[0,2],[3,NaN],[12,.1],[15,0]]){const p=pose.slice();p[index]=value;assert.equal(validateInertialPrediction(prediction({pose:p}),lost()),null);}
});
test('ground payload rejects tampered schema, confidence, geometry and identity',()=>{
 const good=fitGroundPlane(grid(),context());assert.ok(good);
 for(const patch of [{confidence:.9},{semanticClassification:'floor'},{state:'confirmed'},{engineEpoch:2},{clientEpoch:5},{sourceSequence:11},{sourceTimestamp:2.01},{maxAgeSeconds:2},{supportCount:19},{sourcePointCount:1000},{rms:.04},{normal:[0,0,-1]},{normal:[0,0,2]},{offset:2},{centroid:[NaN,0,.2]},{corners:Array(12).fill(.2)}])assert.equal(validateGroundPlane({...good,...patch},context()),null);
 const corrupt=good.corners.slice();corrupt[2]+=.1;assert.equal(validateGroundPlane({...good,corners:corrupt},context()),null);
});
test('tracker owns5Hz fitting, frozen observation time and1second retention',()=>{
 const t=new GroundPlaneTracker();const first=t.update(grid(),context());assert.ok(first);assert.equal(t.lastFitAttempted,true);
 const retained=t.update(grid({height:.21}),context({inputTimestamp:2.1,engineFrameTimestamp:2.1,sequence:11}));assert.equal(t.lastFitAttempted,false);assert.equal(retained.state,'retained');assert.equal(retained.sourceTimestamp,2);assert.equal(retained.sourceSequence,10);
 const observed=t.update(grid({height:.21}),context({inputTimestamp:2.2,engineFrameTimestamp:2.2,sequence:12}));assert.equal(observed.state,'observed');assert.equal(t.lastFitAttempted,true);
 const hold=t.update(null,lost({inputTimestamp:3.2,engineFrameTimestamp:3.2,sequence:13}));assert.ok(hold);assert.equal(hold.sourceTimestamp,2.2);assert.equal(t.lastFitAttempted,false);
 assert.equal(t.update(grid(),lost({inputTimestamp:3.20001,engineFrameTimestamp:3.20001,sequence:14})),null);
});
test('tracker retains support shortage but clears malformed/ambiguous/error/epoch and regressions',()=>{
 for(const bad of ['nan','ambiguous','warning','epoch','error','backward']){
  const t=new GroundPlaneTracker();assert.ok(t.update(grid(),context()));let p=null,c=context({inputTimestamp:2.3,engineFrameTimestamp:2.3,sequence:11});
  if(bad==='nan')p=[0,NaN,0];if(bad==='ambiguous')p=[...grid({height:.1}),...grid({height:.8})];if(bad==='warning')c.reason='tracking:warning';if(bad==='epoch')c.engineEpoch=4;if(bad==='error')c.error=true;if(bad==='backward')c=context({inputTimestamp:1.9,engineFrameTimestamp:1.9,sequence:9});
  assert.equal(t.update(p,c),null,bad);
 }
 const t=new GroundPlaneTracker();t.update(grid(),context());const retained=t.update([],context({inputTimestamp:2.3,engineFrameTimestamp:2.3,sequence:11}));assert.equal(retained.state,'retained');assert.equal(retained.sourceTimestamp,2);t.clear();assert.equal(t.update(null,lost()),null);
});
test('retained plane validation uses frozen source sequence/time and no camera extrapolation',()=>{
 const p={...fitGroundPlane(grid(),context()),state:'retained'};
 assert.ok(validateGroundPlane(p,lost({inputTimestamp:3,engineFrameTimestamp:3,sequence:12})));
 assert.equal(validateGroundPlane(p,lost({inputTimestamp:3.001,engineFrameTimestamp:3.001,sequence:12})),null);
 assert.equal(validateGroundPlane({...p,state:'observed'},lost()),null);
});

test('plane is equivariant under world translation; no assumed zero or camera height',()=>{
 const input=grid({height:-2.3,slopeX:.05}),saved=input.slice();const first=fitGroundPlane(input,context());assert.ok(first);const translation=[4,-3,5];const moved=input.map((v,i)=>v+translation[i%3]);const camera=pose.slice();camera[3]+=4;camera[7]-=3;camera[11]+=5;const second=fitGroundPlane(moved,context({pose:camera}));assert.ok(second);first.normal.forEach((v,i)=>close(v,second.normal[i]));close(second.offset,first.offset-first.normal.reduce((s,v,i)=>s+v*translation[i],0));assert.deepEqual(input,saved);
});
test('plane at or above camera clearance is rejected; measured lower plane accepted',()=>{
 assert.equal(fitGroundPlane(grid({height:1.5}),context()),null);assert.equal(fitGroundPlane(grid({height:1.45}),context()),null);assert.ok(fitGroundPlane(grid({height:1.2}),context()));
});
test('valid rotated camera prediction survives SO3 validation, ordinary and typed arrays',()=>{
 const p=[0,-1,0,1,1,0,0,2,0,0,1,3,0,0,0,1];const out=validateInertialPrediction(prediction({pose:new Float64Array(p)}),lost());assert.ok(out);assert.deepEqual(out.pose,p);
});
test('cached plane cannot be mutated through returned arrays and diagnostics exclude retention',()=>{
 const t=new GroundPlaneTracker();const a=t.update(grid(),context());a.normal[2]=-1;a.corners.fill(999);a.centroid[2]=999;
 const b=t.update(null,lost({inputTimestamp:2.05,engineFrameTimestamp:2.05,sequence:11}));assert.ok(b);close(b.normal[2],1);close(b.centroid[2],.2);assert.equal(t.lastFitAttempted,false);
});
test('solver quality warning immediately clears retained geometry and cannot reactivate on Lost',()=>{
 const t=new GroundPlaneTracker();const p=t.update(grid(),context());assert.ok(p);
 assert.equal(validateGroundPlane({...p,state:'retained'},lost({solverQualityReason:'relative_translation_threshold'})),null);
 assert.equal(t.update(null,lost({inputTimestamp:2.1,engineFrameTimestamp:2.1,sequence:11,solverQualityReason:'relative_translation_threshold'})),null);
 assert.equal(t.update(null,lost({inputTimestamp:2.2,engineFrameTimestamp:2.2,sequence:12})),null);
});
test('duplicate sequence/time never records another observed fit',()=>{
 const t=new GroundPlaneTracker();assert.equal(t.update(grid(),context()).state,'observed');const repeat=t.update(grid(),context());assert.equal(repeat.state,'retained');assert.equal(t.lastFitAttempted,false);assert.equal(repeat.sourceSequence,10);assert.equal(repeat.sourceTimestamp,2);
});
