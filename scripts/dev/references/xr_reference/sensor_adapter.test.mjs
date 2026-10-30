import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {bodyToDevice} from '../../synthetic-browser.mjs';
import {bodyToDeviceRow,deviceToBodyRow,qMultiply,qExp,qRotate,qMatrix,gravityInitialOrientation,orientationZXy,buildSensorPackets} from './sensor_adapter.mjs';
let checks=0;const near=(actual,expected,tolerance=1e-11)=>{assert.equal(actual.length,expected.length);assert.ok(Math.max(...actual.map((v,i)=>Math.abs(v-expected[i])))<tolerance);checks++;};
const sentinel=[1.25,1,2,3,4,5,6];near(bodyToDeviceRow(sentinel),[1.25,1,3,-2,4,6,-5]);near(bodyToDeviceRow(sentinel),bodyToDevice(sentinel));near(deviceToBodyRow(bodyToDeviceRow(sentinel)),sentinel);
const q0=gravityInitialOrientation([0,9.81,0]);near(qRotate(q0,[0,9.81,0]),[0,0,9.81]);
const qx=qExp([Math.PI/2,0,0]),qy=qExp([0,Math.PI/2,0]);near(qRotate(qMultiply(qx,qy),[0,0,1]),[1,0,0]);near(qRotate(qMultiply(qy,qx),[0,0,1]),[0,-1,0]);
const multiply=(a,b)=>a.map(row=>b[0].map((_,j)=>row.reduce((sum,v,k)=>sum+v*b[k][j],0)));
for(const rotation of [[0,0,0],[0.3,-0.2,0.4],[1.4,2.2,-1.9],[Math.PI/2,0,0],[0,Math.PI,0]]){
 const q=qExp(rotation),e=orientationZXy(q),[a,b,c]=[e.alpha,e.beta,e.gamma].map(x=>x*Math.PI/180);
 const rz=[[Math.cos(a),-Math.sin(a),0],[Math.sin(a),Math.cos(a),0],[0,0,1]],rx=[[1,0,0],[0,Math.cos(b),-Math.sin(b)],[0,Math.sin(b),Math.cos(b)]],ry=[[Math.cos(c),0,Math.sin(c)],[0,1,0],[-Math.sin(c),0,Math.cos(c)]];
 const reconstructed=multiply(multiply(rz,rx),ry);near(reconstructed.flat(),qMatrix(q).flat());
}
const rows=Array.from({length:101},(_,i)=>[i/100,0,0,9.81,0.3,0.2,0.4]);const built=buildSensorPackets(rows);const expected=qMultiply(built.provenance.initialQuaternionEarthFromDevice,qExp([0.3,0.4,-0.2]));near(qMatrix(built.packets.at(-1).quaternionEarthFromDevice).flat(),qMatrix(expected).flat());
for(let i=0;i<rows.length;i++){const p=built.packets[i],device=[p.sourceTimestamp,...p.deviceAcceleration,p.rotationRate.alpha*Math.PI/180,p.rotationRate.beta*Math.PI/180,p.rotationRate.gamma*Math.PI/180];near(deviceToBodyRow(device),rows[i]);}
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../../../..');const result={schema:'xr_reference-raw-sensor-adapter-tests-v1',checks,passed:true,groundTruthUsed:false,osFusedOrientationReproduced:false,provenance:built.provenance};fs.writeFileSync(path.join(root,'build/research-20261012/xr_reference-binary/sensor-adapter-tests.json'),JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify({checks,passed:true,groundTruthUsed:false}));
