// Raw-IMU-only adapter. No ground-truth files, poses or orientations enter this module.
export const R_DEVICE_FROM_BODY=[[1,0,0],[0,0,1],[0,-1,0]];
export const R_BODY_FROM_DEVICE=[[1,0,0],[0,0,-1],[0,1,0]];
export const bodyToDeviceRow=r=>[r[0],r[1],r[3],-r[2],r[4],r[6],-r[5]];
export const deviceToBodyRow=r=>[r[0],r[1],-r[3],r[2],r[4],-r[6],r[5]];
export const qNormalize=q=>{const n=Math.hypot(...q);if(!Number.isFinite(n)||n<1e-12)throw new Error('Invalid quaternion');return q.map(v=>v/n);};
export const qMultiply=(a,b)=>[a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]];
export const qExp=rotation=>{const angle=Math.hypot(...rotation);if(angle<1e-12)return qNormalize([1,...rotation.map(v=>v/2)]);const s=Math.sin(angle/2)/angle;return [Math.cos(angle/2),...rotation.map(v=>v*s)];};
export const qRotate=(q,v)=>qMultiply(qMultiply(q,[0,...v]),[q[0],-q[1],-q[2],-q[3]]).slice(1);
export const qMatrix=q=>{const [w,x,y,z]=qNormalize(q);return [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]];};
export function gravityInitialOrientation(acceleration){
  const norm=Math.hypot(...acceleration);if(norm<1)throw new Error('Initial gravity direction unavailable');const a=acceleration.map(v=>v/norm),b=[0,0,1];
  const dot=a[2];if(dot<-1+1e-10)return [0,1,0,0];
  // Minimum rotation maps device specific-force direction to reference +Z.
  // The remaining heading is an arbitrary gauge, not a compass measurement.
  return qNormalize([1+dot,a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]);
}
export function orientationZXy(q){
  const r=qMatrix(q),clamp=x=>Math.max(-1,Math.min(1,x));let beta=Math.asin(clamp(r[2][1])),alpha,gamma;
  if(Math.abs(Math.cos(beta))<1e-8){alpha=Math.atan2(r[1][0],r[0][0]);gamma=0;}
  else{alpha=Math.atan2(-r[0][1],r[1][1]);gamma=Math.atan2(-r[2][0],r[2][2]);}
  if(gamma>Math.PI/2||gamma<-Math.PI/2){gamma+=gamma>0?-Math.PI:Math.PI;beta=beta>=0?Math.PI-beta:-Math.PI-beta;alpha+=Math.PI;}
  const degrees=180/Math.PI;return {alpha:((alpha*degrees)%360+360)%360,beta:beta*degrees,gamma:gamma*degrees,absolute:false};
}
export function buildSensorPackets(rows){
  if(!rows.length)throw new Error('Raw IMU required');let previous=bodyToDeviceRow(rows[0]),q=gravityInitialOrientation(previous.slice(1,4));
  const initial=q.slice();const packets=rows.map((row,index)=>{
    const device=bodyToDeviceRow(row);const dt=index?device[0]-previous[0]:0;if(dt<0)throw new Error('Non-monotonic IMU');
    if(index){const rotation=device.slice(4).map((v,i)=>0.5*(v+previous[4+i])*dt);q=qNormalize(qMultiply(q,qExp(rotation)));}
    previous=device;return {sourceTimestamp:row[0],deviceAcceleration:device.slice(1,4),deviceGyroRadS:device.slice(4),rotationRate:{alpha:device[4]*180/Math.PI,beta:device[5]*180/Math.PI,gamma:device[6]*180/Math.PI},quaternionEarthFromDevice:q.slice(),orientation:orientationZXy(q)};
  });
  return {packets,provenance:{R_DEVICE_FROM_BODY,R_BODY_FROM_DEVICE,initialQuaternionEarthFromDevice:initial,initialGravitySource:'First raw accelerometer sample; assumes initial specific force is gravity-dominated',yaw:'Arbitrary heading gauge of minimum gravity alignment; no magnetometer',gyroIntegration:'q_E_D[k]=normalize(q_E_D[k-1] * Exp(0.5*(omega_D[k-1]+omega_D[k])*dt)); quaternion wxyz; body/device rates; Exp maps full rotation vector to unit quaternion',orientationConvention:'W3C intrinsic Z(alpha)-X(beta)-Y(gamma); DeviceMotion rate alpha=x,beta=y,gamma=z',gyroBias:'No bias correction or OS fusion reconstructed',groundTruthUsed:false,osFusedOrientationReproduced:false}};
}
