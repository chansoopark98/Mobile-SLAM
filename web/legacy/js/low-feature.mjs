/** Bounded display-only inertial/plane contracts. No estimator or clock mutation. */
const EPS = 1e-9;
const LOSS_REASONS = new Set(['empty_features', 'insufficient_current_features', 'insufficient_final_current_features']);
const COS_TILT = Math.cos(Math.PI / 12);
const PREDICTION_KEYS = ['schema','mode','poseFrame','engineEpoch','timestamp','anchorTimestamp','maxHorizonSeconds','pose','visualSupported','metricTranslationSupported','positionUncertain'];
const PLANE_KEYS = ['schema','kind','semanticClassification','state','engineEpoch','clientEpoch','sourceTimestamp','sourceSequence','normal','offset','centroid','corners','supportCount','sourcePointCount','rms','maxAgeSeconds'];
const finite = Number.isFinite;
const identity = x => Number.isSafeInteger(x) && x >= 0;
const dot = (a,b) => a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const vector = (v,n) => (Array.isArray(v) || (ArrayBuffer.isView(v) && !(v instanceof DataView))) && v.length === n && Array.from(v).every(finite);
const keysMatch = (v,keys) => v && typeof v === 'object' && Object.keys(v).length === keys.length && keys.every(k => Object.hasOwn(v,k));
const timestamp = c => c?.inputTimestamp ?? c?.timestamp;
function validContext(c) {
    return c && finite(timestamp(c)) && timestamp(c) >= 0 && c.engineFrameTimestamp === timestamp(c) &&
        identity(c.engineEpoch) && identity(c.clientEpoch) && identity(c.sequence) &&
        c.solverQualityReason === '' && !c.error && !c.motionDiscontinuity && !c.transportLost;
}
function validPose(p) {
    if (!vector(p,16) || Math.abs(p[12])>1e-6 || Math.abs(p[13])>1e-6 || Math.abs(p[14])>1e-6 || Math.abs(p[15]-1)>1e-6) return false;
    const rows = [[p[0],p[1],p[2]],[p[4],p[5],p[6]],[p[8],p[9],p[10]]];
    for(let i=0;i<3;i++)for(let j=i;j<3;j++)if(Math.abs(dot(rows[i],rows[j])-(i===j?1:0))>1e-5)return false;
    const det=p[0]*(p[5]*p[10]-p[6]*p[9])-p[1]*(p[4]*p[10]-p[6]*p[8])+p[2]*(p[4]*p[9]-p[5]*p[8]);
    return Math.abs(det-1)<=1e-5;
}
const clean = c => validContext(c) && c.statusCode===2 && c.initialized===true && c.reason==='tracking' && validPose(c.pose);
const allowedLost = c => validContext(c) && c.statusCode===3 && c.initialized===true && LOSS_REASONS.has(c.reason);

/** A new verified visual frame is required to release a transport-invalidated side channel. */
export function isCleanVisualContext(context) {
    return clean(context) && context.poseFresh === true && context.poseValid === true && context.poseTimestamp === timestamp(context);
}

export function validateInertialPrediction(value,context) {
    if (!allowedLost(context) || !keysMatch(value,PREDICTION_KEYS) ||
        value.schema!=='mobile-slam-inertial-prediction-v1' || value.mode!=='inertial-only' || value.poseFrame!=='camera' ||
        value.engineEpoch!==context.engineEpoch || value.timestamp!==timestamp(context) || !finite(value.anchorTimestamp) || value.anchorTimestamp<0 ||
        value.maxHorizonSeconds!==.5 || value.visualSupported!==false || value.metricTranslationSupported!==false || value.positionUncertain!==true ||
        !validPose(value.pose)) return null;
    const horizon=value.timestamp-value.anchorTimestamp;
    if(horizon < -EPS || horizon > .5+EPS)return null;
    return {...value,pose:Array.from(value.pose)};
}

function bounds(points) {
    const count=points.length, center=[0,0,0];
    let minX=Infinity,maxX=-Infinity,minY=Infinity,maxY=-Infinity;
    for(const p of points){for(let j=0;j<3;j++)center[j]+=p[j]/count;minX=Math.min(minX,p[0]);maxX=Math.max(maxX,p[0]);minY=Math.min(minY,p[1]);maxY=Math.max(maxY,p[1]);}
    let xx=0,xy=0,yy=0;
    for(const p of points){const x=p[0]-center[0],y=p[1]-center[1];xx+=x*x/count;xy+=x*y/count;yy+=y*y/count;}
    const small=(xx+yy-Math.hypot(xx-yy,2*xy))/2;
    return {center,minX,maxX,minY,maxY,xx,xy,yy,spread:maxX-minX>=.3-EPS && maxY-minY>=.3-EPS && small>=.0025-EPS};
}
function geometryValid(v) {
    if(!vector(v.normal,3)||!vector(v.centroid,3)||!vector(v.corners,12)||!finite(v.offset)||!finite(v.rms)||v.rms<0||v.rms>.03+EPS)return false;
    if(Math.abs(Math.hypot(...v.normal)-1)>1e-6 || v.normal[2]<COS_TILT-EPS)return false;
    if(Math.abs(dot(v.normal,v.centroid)+v.offset)>1e-5)return false;
    const corners=Array.from({length:4},(_,i)=>Array.from(v.corners).slice(i*3,i*3+3));
    if(corners.some(p=>Math.abs(dot(v.normal,p)+v.offset)>1e-5) || !bounds(corners).spread)return false;
    // Reject collapsed, crossed or concave quads, even when their bounding box is large.
    const crosses=corners.map((p,i)=>{const q=corners[(i+1)%4],r=corners[(i+2)%4];return (q[0]-p[0])*(r[1]-q[1])-(q[1]-p[1])*(r[0]-q[0]);});
    if(!crosses.every(x=>x>1e-8) && !crosses.every(x=>x < -1e-8))return false;
    return true;
}
export function validateGroundPlane(value,context) {
    if(!validContext(context) || (!clean(context)&&!allowedLost(context)) || !keysMatch(value,PLANE_KEYS) ||
        value.schema!=='mobile-slam-ground-plane-v1' || value.kind!=='horizontal-surface-candidate' || value.semanticClassification!=='unknown' ||
        !['observed','retained'].includes(value.state) || value.engineEpoch!==context.engineEpoch || value.clientEpoch!==context.clientEpoch ||
        !identity(value.sourceSequence) || value.sourceSequence>context.sequence || !finite(value.sourceTimestamp) || value.sourceTimestamp<0 ||
        value.maxAgeSeconds!==1 || !Number.isInteger(value.sourcePointCount) || value.sourcePointCount<20 || value.sourcePointCount>512 ||
        !Number.isInteger(value.supportCount) || value.supportCount<20 || value.supportCount>value.sourcePointCount || value.supportCount/value.sourcePointCount<.6 ||
        !geometryValid(value))return null;
    const age=timestamp(context)-value.sourceTimestamp;
    if(age < -EPS || age > 1+EPS)return null;
    if(value.state==='observed' && (!clean(context)||value.sourceTimestamp!==timestamp(context)||value.sourceSequence!==context.sequence))return null;
    if(clean(context) && dot(value.normal,[context.pose[3],context.pose[7],context.pose[11]])+value.offset<=.1)return null;
    return {...value,normal:Array.from(value.normal),centroid:Array.from(value.centroid),corners:Array.from(value.corners)};
}

function parsePoints(values) {
    if(values==null)return {points:[],invalid:false};
    if(!(Array.isArray(values)||(ArrayBuffer.isView(values)&&!(values instanceof DataView))) || values.length%3!==0)return {invalid:true};
    const points=[],seen=new Set();
    for(let i=0;i<values.length;i+=3){
        const p=[values[i],values[i+1],values[i+2]];
        if(!p.every(finite))return {invalid:true};
        // Quantize only for duplicate suppression; fit always uses original coordinates.
        const key=p.map(x=>Math.round(x/1e-5)).join(',');
        if(!seen.has(key) && points.length<512){seen.add(key);points.push(p);}
    }
    return {points,invalid:false};
}
function evaluate(points,normal,offset,camera) {
    if(!normal.every(finite)||!finite(offset)||normal[2]<COS_TILT-EPS || dot(normal,camera)+offset<=.1)return null;
    const inliers=points.filter(p=>Math.abs(dot(normal,p)+offset)<=.03+EPS);
    if(inliers.length<20)return null;
    const spatial=bounds(inliers);
    if(!spatial.spread)return null;
    const rms=Math.sqrt(inliers.reduce((sum,p)=>sum+(dot(normal,p)+offset)**2,0)/inliers.length);
    return {normal,offset,inliers,spatial,rms};
}
function fit(points,context) {
    if(points.length<20)return {candidate:null,reason:'insufficient'};
    const camera=[context.pose[3],context.pose[7],context.pose[11]],candidates=[];
    let seed=0x6d2b79f5;
    const next=()=>{seed^=seed<<13;seed^=seed>>>17;seed^=seed<<5;return (seed>>>0)%points.length;};
    for(let iteration=0;iteration<64;iteration++){
        const ia=next(),ib=next(),ic=next();if(ia===ib||ia===ic||ib===ic)continue;
        const a=points[ia],b=points[ib],c=points[ic],u=b.map((v,i)=>v-a[i]),v=c.map((n,i)=>n-a[i]);
        let n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
        const length=Math.hypot(...n);if(!finite(length)||length<1e-10)continue;
        n=n.map(x=>x/length*(n[2]<0?-1:1));
        const candidate=evaluate(points,n,-dot(n,a),camera);if(candidate)candidates.push(candidate);
    }
    if(!candidates.length)return {candidate:null,reason:'unsupported'};
    candidates.sort((a,b)=>b.inliers.length-a.inliers.length || a.rms-b.rms);
    const best=candidates[0];
    const height=p=>-(p.normal[0]*camera[0]+p.normal[1]*camera[1]+p.offset)/p.normal[2];
    if(candidates.some(p=>p.inliers.length>=.8*best.inliers.length && Math.abs(height(p)-height(best))>.1))return {candidate:null,reason:'ambiguous'};
    if(best.inliers.length/points.length<.6)return {candidate:null,reason:'unsupported'};
    // Centered least squares z=a*x+b*y+c. This is well-conditioned only after the XY-spread gate.
    const {center,xx,xy,yy}=best.spatial;
    let xz=0,yz=0;
    for(const p of best.inliers){xz+=(p[0]-center[0])*(p[2]-center[2])/best.inliers.length;yz+=(p[1]-center[1])*(p[2]-center[2])/best.inliers.length;}
    const determinant=xx*yy-xy*xy;if(!finite(determinant)||determinant<=1e-12)return {candidate:null,reason:'degenerate'};
    const a=(xz*yy-yz*xy)/determinant,b=(yz*xx-xz*xy)/determinant,length=Math.hypot(a,b,1),normal=[-a/length,-b/length,1/length];
    const refined=evaluate(points,normal,-dot(normal,center),camera);
    if(!refined||refined.inliers.length/points.length<.6)return {candidate:null,reason:'unsupported'};
    if(candidates.some(p=>p.inliers.length>=.8*refined.inliers.length && Math.abs(height(p)-height(refined))>.1))return {candidate:null,reason:'ambiguous'};
    const box=refined.spatial,offset=refined.offset;
    const centroid=box.center.map((v,i)=>v-normal[i]*(dot(normal,box.center)+offset));
    const corners=[[box.minX,box.minY],[box.maxX,box.minY],[box.maxX,box.maxY],[box.minX,box.maxY]].flatMap(([x,y])=>[x,y,-(normal[0]*x+normal[1]*y+offset)/normal[2]]);
    const candidate={schema:'mobile-slam-ground-plane-v1',kind:'horizontal-surface-candidate',semanticClassification:'unknown',state:'observed',
        engineEpoch:context.engineEpoch,clientEpoch:context.clientEpoch,sourceTimestamp:timestamp(context),sourceSequence:context.sequence,
        normal,offset,centroid,corners,supportCount:refined.inliers.length,sourcePointCount:points.length,rms:refined.rms,maxAgeSeconds:1};
    return {candidate:validateGroundPlane(candidate,context),reason:'fit'};
}
export function fitGroundPlane(values,context) {
    if(!clean(context))return null;
    const parsed=parsePoints(values);if(parsed.invalid)return null;
    return fit(parsed.points,context).candidate;
}

export class GroundPlaneTracker {
    constructor(){this.clear();}
    clear(){this._plane=null;this._context=null;this._lastFitTime=-Infinity;this._lastFitAttempted=false;}
    get lastFitAttempted(){return this._lastFitAttempted;}
    update(values,context){
        this._lastFitAttempted=false;
        if(!validContext(context)||(!clean(context)&&!allowedLost(context))){this.clear();return null;}
        const previous=this._context;
        if(previous && (context.engineEpoch!==previous.engineEpoch || context.clientEpoch!==previous.clientEpoch || timestamp(context)<previous.time || context.sequence<previous.sequence)){
            this.clear();return null;
        }
        this._context={engineEpoch:context.engineEpoch,clientEpoch:context.clientEpoch,time:timestamp(context),sequence:context.sequence};
        const parsed=parsePoints(values);
        if(parsed.invalid){this.clear();return null;}
        if(clean(context) && (!previous || (timestamp(context)>previous.time && context.sequence>previous.sequence)) && timestamp(context)-this._lastFitTime>=.2-EPS && parsed.points.length>=20){
            this._lastFitTime=timestamp(context);this._lastFitAttempted=true;
            const result=fit(parsed.points,context);
            if(result.reason==='ambiguous'){this._plane=null;return null;}
            if(result.candidate){this._plane=result.candidate;return validateGroundPlane(result.candidate,context);}
        }
        if(!this._plane)return null;
        const retained=validateGroundPlane({...this._plane,state:'retained'},context);
        if(!retained)this._plane=null;
        return retained;
    }
}
