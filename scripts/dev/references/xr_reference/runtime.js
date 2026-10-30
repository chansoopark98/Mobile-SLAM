/* Synthetic camera/sensor boundary experiment; no real phone, camera calibration or GT fitting. */
(async () => {
  const state=window.__probe;
  const fixture=await (await fetch('/fixture.json')).json();
  const {width,height}=fixture.config;
  const input=await Promise.all(fixture.frames.map(async frame=>new Uint8Array(await (await fetch('/gray/'+frame.index)).arrayBuffer())));
  const source=document.createElement('canvas');source.width=width;source.height=height;
  const ctx=source.getContext('2d',{willReadFrequently:true});
  const draw=index=>{const image=ctx.createImageData(width,height),gray=input[index];for(let p=0;p<gray.length;p++){image.data[4*p]=gray[p];image.data[4*p+1]=gray[p];image.data[4*p+2]=gray[p];image.data[4*p+3]=255;}ctx.putImageData(image,0,0);};
  draw(0);
  let writer=null,stream,track;
  if(fixture.cameraSource==='generated'){track=new MediaStreamTrackGenerator({kind:'video'});writer=track.writable.getWriter();stream=new MediaStream([track]);}
  else{stream=source.captureStream(20);track=stream.getVideoTracks()[0];}
  const settings=track.getSettings.bind(track);
  Object.defineProperty(track,'getSettings',{value:()=>({...settings(),width,height,frameRate:20,facingMode:'environment'})});
  navigator.mediaDevices.getUserMedia=async constraints=>{state.cameraRequests.push({constraints,at:performance.now()});return stream;};
  const timers=new Set();let epoch=null;
  const schedule=(fn,ms)=>{const id=setTimeout(()=>{timers.delete(id);fn();},ms);timers.add(id);};
  const emit=(packet,initial=false)=>{
    const deviceA={x:packet.deviceAcceleration[0],y:packet.deviceAcceleration[1],z:packet.deviceAcceleration[2]},rotationRate=packet.rotationRate;
    const timeStamp=initial?performance.now():epoch+packet.sourceTimestamp*1000;
    const event=new DeviceMotionEvent('devicemotion',{acceleration:null,accelerationIncludingGravity:deviceA,rotationRate,interval:5});
    Object.defineProperty(event,'timeStamp',{value:timeStamp});
    window.dispatchEvent(event);
    const orientation=new DeviceOrientationEvent('deviceorientation',packet.orientation);
    Object.defineProperty(orientation,'timeStamp',{value:timeStamp});window.dispatchEvent(orientation);
    if(!initial)state.imu.push({sourceTimestamp:packet.sourceTimestamp,eventTimestampMs:timeStamp,dispatchedWallMs:performance.now(),accelerationIncludingGravity:deviceA,rotationRate,orientation:packet.orientation,quaternionEarthFromDevice:packet.quaternionEarthFromDevice,isTrusted:event.isTrusted});
  };
  const submit=async(index,timestampUs)=>{draw(index);if(writer){const frame=new VideoFrame(source,{timestamp:timestampUs});try{await writer.write(frame);}finally{frame.close();}}else track.requestFrame?.();};
  let finishPromise=null;
  const finish=()=>{if(finishPromise)return finishPromise;finishPromise=(async()=>{state.done=true;for(const id of timers)clearTimeout(id);timers.clear();clearInterval(bootstrap);try{XR8.stop();}catch(e){state.errors.push({kind:'stop',message:String(e)});}if(writer)await writer.close();stream.getTracks().forEach(t=>t.stop());state.ownedResourcesClosed=true;})();return finishPromise;};
  window.__finishProbe=finish;
  const begin=()=>{
    if(state.started)return;state.started=true;clearInterval(bootstrap);epoch=performance.now();state.epochWallMs=epoch;
    const publish=async index=>{if(index>=fixture.frames.length){schedule(finish,500);return;}const timestampUs=Math.round(epoch*1000)+Math.round(fixture.frames[index].physicalTimestamp*1000000);const publication={index,physicalTimestamp:fixture.frames[index].physicalTimestamp,videoFrameTimestampUs:writer?timestampUs:null,sourceSha256:fixture.frames[index].sha256,publishedWallMs:performance.now()};state.frames.push(publication);await submit(index,timestampUs);publication.writerAcceptedWallMs=performance.now();const next=index+1;if(next<fixture.frames.length)schedule(()=>publish(next),Math.max(0,epoch+fixture.frames[next].physicalTimestamp*1000-performance.now()));else schedule(finish,500);};
    const sensor=index=>{let next=index;const due=(performance.now()-epoch)/1000;while(next<fixture.sensorPackets.length&&fixture.sensorPackets[next].sourceTimestamp<=due+0.000001){emit(fixture.sensorPackets[next]);next++;}if(next<fixture.sensorPackets.length)schedule(()=>sensor(next),Math.max(0,epoch+fixture.sensorPackets[next].sourceTimestamp*1000-performance.now()));};
    publish(0);sensor(0);
  };
  const bootstrap=setInterval(()=>{void submit(0,Math.round(performance.now()*1000));emit(fixture.sensorPackets[0],true);},16);
  const loaded=()=>{
    state.sdkVersion=XR8.version();state.deviceEstimate=XR8.XrDevice.deviceEstimate();
    state.requestedConfiguration={disableWorldTracking:false,enableWorldPoints:true,scale:fixture.scale};
    XR8.XrController.configure(state.requestedConfiguration);
    state.renderCameraConfiguration={cam:{pixelRectWidth:width,pixelRectHeight:height,nearClipPlane:0.01,farClipPlane:1000},origin:{x:0,y:2,z:0},facing:{w:1,x:0,y:0,z:0}};
    XR8.XrController.updateCameraProjectionMatrix(state.renderCameraConfiguration);
    const reality=XR8.XrController.pipelineModule();
    XR8.addCameraPipelineModules([XR8.GlTextureRenderer.pipelineModule(),reality,{
      name:'isolated-reference-observer',
      onStart:args=>{const gl=args.GLctx;let backend=null;if(gl?.getParameter){const extension=gl.getExtension('WEBGL_debug_renderer_info');backend={version:gl.getParameter(gl.VERSION),vendor:gl.getParameter(gl.VENDOR),renderer:gl.getParameter(gl.RENDERER),unmaskedRenderer:extension?gl.getParameter(extension.UNMASKED_RENDERER_WEBGL):null};}state.events.push({name:'onStart',at:performance.now(),videoWidth:args.videoWidth,videoHeight:args.videoHeight,canvasWidth:args.canvasWidth,canvasHeight:args.canvasHeight,backend});begin();},
      onCameraStatusChange:args=>state.events.push({name:'cameraStatus',at:performance.now(),status:args.status,error:args.error?String(args.error):null}),
      onException:error=>{state.errors.push({kind:'pipeline',message:String(error),stack:error?.stack});},
      onUpdate:({frameStartResult:f,processCpuResult:p})=>{const r=p.reality;if(!r)return;state.rows.push({callbackWallMs:performance.now(),sourcePublication:state.frames.at(-1)?.index??null,videoTime:f.videoTime,frameTime:f.frameTime,repeatFrame:f.repeatFrame,textureWidth:f.textureWidth,textureHeight:f.textureHeight,orientation:f.orientation,position:r.position,rotation:r.rotation,intrinsics:Array.from(r.intrinsics||[]).map(x=>Number.isFinite(x)?x:String(x)),trackingStatus:r.trackingStatus,trackingReason:r.trackingReason,worldPointCount:r.worldPoints?.length??null});}
    }]);
    state.ready=true;
    document.getElementById('start').onclick=()=>{
      const query=new URLSearchParams(location.search);
      state.runOptions={allowedDevices:query.get('mode')==='desktop'?XR8.XrConfig.device().ANY:XR8.XrConfig.device().MOBILE,cameraConfig:{direction:XR8.XrConfig.camera().BACK}};
      try{XR8.run({canvas:document.getElementById('view'),...state.runOptions,verbose:true});}catch(error){state.errors.push({kind:'run',message:String(error),stack:error.stack});}
    };
  };
  if(window.XR8)loaded();else window.addEventListener('xrloaded',loaded,{once:true});
  state.inputProvenance={schema:fixture.schema,fixtureSha256:fixture.sourceInputSha256,frames:fixture.frames.length,imuPackets:fixture.imu.length,config:fixture.config,mode:fixture.cameraSource==='generated'?'MediaStreamTrackGenerator with declared VideoFrame timestamp; grayscale RGB expansion':'canvas captureStream grayscale RGB expansion',orientation:fixture.sensorProvenance,exactEngineInputBytesVerified:false,physicalExposureVerified:false,groundTruthUsed:false};
})();
