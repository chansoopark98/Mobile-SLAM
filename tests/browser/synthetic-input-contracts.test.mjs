import test from 'node:test';
import assert from 'node:assert/strict';
import { limitedPipeline } from './limited-fixture.mjs';
import { validateSyntheticInput, bodyToDevice, sourceCoverage, poseStateSummary, validateResultIdentity, distribution, assertSourceFrameHash, replayIMUBatches,
    profileFromEffectiveConfig,assertEffectiveProfile,assertSourceFramePixels,schedulerContract,resolvePublishedFrame,parseFThresholdOverride,parseRotationRescueOverride } from '../../scripts/dev/synthetic-browser.mjs';

function fixture() {
    return { schema: 'mobile-slam-synthetic-vio-v1', config: { width: 2, height: 2 },
        parameters: { mobile: [10,10,120], tracking: [21,3,20,0], pnp: [false,3], fThreshold: 1, execution: [0,1] },
        frames: [0,1].map(index => ({ index, timestamp: index/20, physicalTimestamp: index/20, file: `${index}.pgm`, sha256: 'a'.repeat(64) })),
        imu: [0,.025,.05,.075].map(t => [t,0,0,9.81,0,0,0]) };
}

test('State summary retains normal, Limited, Lost and unknown rows without removing Limited full-body accuracy coverage', () => {
    const frames=[0,1,2,3,4].map(index=>({index,timestamp:index,physicalTimestamp:index}));
    const rows=frames.map(frame=>({sourceIndex:frame.index,inputTimestamp:frame.timestamp,physicalTimestamp:frame.timestamp,
        poseTimestamp:frame.timestamp,poseFresh:true,poseValid:true,pose:new Array(16).fill(0)}));
    Object.assign(rows[0],{statusCode:2,pipeline:{schema:'mobile-slam-pipeline-v1'}});
    Object.assign(rows[1],{statusCode:5,pipeline:limitedPipeline(1),quality:limitedPipeline(1).quality});
    Object.assign(rows[2],{statusCode:3,poseFresh:false,poseValid:false,pose:null});
    Object.assign(rows[3],{statusCode:99,poseFresh:false,poseValid:false,pose:null});
    Object.assign(rows[4],{statusCode:5,quality:null});
    const state=poseStateSummary(rows),coverage=sourceCoverage(frames,frames.map(frame=>({sourceIndex:frame.index})),rows);
    assert.equal(state.terminalRows,5);assert.equal(state.states.tracking,1);assert.equal(state.states.tracking_limited,2);
    assert.equal(state.states.lost,1);assert.equal(state.states.unknown,1);
    assert.equal(state.normalFresh,1);assert.equal(state.limitedFresh,2);assert.equal(state.fullBodyFresh,3);
    assert.equal(state.limitedQualityUnavailable,1,'Missing quality is reported for assessment failure, never filtered to improve the score');
    assert.equal(coverage.planned,5);assert.equal(coverage.fresh,3);assert.equal(state.limitedExcludedFromAccuracy,false);
});

test('Synthetic schema rejects phone recordings and incomplete profile/physical identity', () => {
    assert.equal(validateSyntheticInput(fixture()).length, 2);
    for (const change of [
        input => { input.schema = 'mobile-slam-android-input-v1'; },
        input => { delete input.parameters; },
        input => { delete input.frames[0].physicalTimestamp; },
        input => { input.frames[1].physicalTimestamp = 0; },
        input => { input.frames[1].index = 0; },
        input => { input.frames[1].file = '../escape.pgm'; },
        input => { input.frames[1].sha256 = 'unknown'; },
        input => { input.config.rotationRescue = 1; },
        input => { input.imu[1][0] = 0; },
        input => { input.imu[0][0] = .01; },
        input => { input.imu = input.imu.slice(0,2); },
        input => { input.imu[1][1] = Infinity; },
    ]) { const input = fixture(); change(input); assert.throws(() => validateSyntheticInput(input)); }
});

test('Captured prefixes preserve original indices and physical time under a shifted source clock', () => {
    const input = fixture();
    input.frames = [{ ...input.frames[1], index: 51, timestamp: 100.05, physicalTimestamp: 2.55 }];
    input.imu.forEach(row => { row[0] += 100; });
    assert.equal(validateSyntheticInput(input)[0].index, 51);
    assert.equal(validateSyntheticInput(input)[0].physicalTimestamp, 2.55);
});

test('Independent inverse device transform preserves norm and known basis signs', () => {
    assert.deepEqual(bodyToDevice([3,1,2,3,4,5,6]), [3,1,3,-2,4,6,-5]);
    assert.deepEqual(bodyToDevice([0,0,0,9.81,0,1,0]), [0,0,9.81,-0,0,0,-1]);
    const row = [2,.3,-1.7,9.8,-.4,.7,1.1], device = bodyToDevice(row);
    assert.equal(Math.hypot(...device.slice(1,4)), Math.hypot(...row.slice(1,4)));
    assert.deepEqual([device[1],-device[3],device[2]], row.slice(1,4));
    assert.deepEqual([device[4],-device[6],device[5]], row.slice(4,7));
});

test('Coverage includes dropped, invalid and uninitialized frames in fixed source denominator', () => {
    const frames = Array.from({ length: 400 }, (_, index) => ({ index, physicalTimestamp: index/20 }));
    const accepted = frames.slice(100,300).map(frame => ({ sourceIndex: frame.index }));
    const rows = accepted.map(row => ({ ...row, poseFresh: true, poseValid: true, pose: new Array(16).fill(0) }));
    rows[0].poseFresh = false; rows[1].pose[3] = NaN;
    const coverage = sourceCoverage(frames,accepted,rows);
    assert.equal(coverage.planned,400); assert.equal(coverage.accepted,200); assert.equal(coverage.droppedOrSkipped,200);
    assert.equal(coverage.postWarmupPlanned,300); assert.equal(coverage.postWarmupFresh,198);
    assert.equal(coverage.postWarmupFreshCoverage,198/300); assert.equal(coverage.fixed400SourceScope,true);
    assert.equal(sourceCoverage(frames,[],[]).postWarmupFreshCoverage,0);
});

test('Prefix/memory limits fail closed and never silently rebase frame identity', () => {
    for (const count of [0,2001,1.5,NaN]) assert.throws(() => validateSyntheticInput(fixture(), count));
    const input=fixture(); input.config.width=16777216; input.config.height=2;
    assert.throws(() => validateSyntheticInput(input), /dimensions/);
});

test('A finite old pose cannot be attributed to a new physical source frame', () => {
    const accepted=[{sourceIndex:17,timestamp:100.85,physicalTimestamp:.85}];
    const row={sourceIndex:17,inputTimestamp:100.85,physicalTimestamp:.85,poseFresh:true,poseValid:true,poseTimestamp:100.85};
    assert.doesNotThrow(()=>validateResultIdentity([row],accepted));
    for(const changed of [{poseTimestamp:100.8},{sourceIndex:18},{inputTimestamp:100.8},{physicalTimestamp:.8}])assert.throws(()=>validateResultIdentity([{...row,...changed}],accepted));
    assert.throws(()=>validateResultIdentity([],accepted));
    assert.throws(()=>validateResultIdentity([row,row],accepted));
});

test('Stage percentiles keep unavailable measurements distinct from zero', () => {
    assert.deepEqual(distribution([]),{count:0,p50:null,p95:null,p99:null,max:null});
    assert.deepEqual(distribution([null,NaN,0,4,8,12]),{count:4,p50:4,p95:12,p99:12,max:12});
});

test('Decoded previous/wrong frame cannot inherit the new physical source identity', () => {
    const intended='a'.repeat(64), stale='b'.repeat(64);
    assert.doesNotThrow(()=>assertSourceFrameHash(intended,intended,14));
    assert.throws(()=>assertSourceFrameHash(intended,stale,14),/Source frame 14 pixel hash mismatch/);
    assert.throws(()=>assertSourceFrameHash(intended,undefined,14),/pixel hash mismatch/);
    assert.throws(()=>assertSourceFrameHash('','',14),/pixel hash mismatch/);
});

test('Captured replay preserves submitted IMU membership rather than adding a future sample', () => {
    const input=fixture();
    assert.deepEqual(replayIMUBatches(input,input.frames),[{start:0,count:2},{start:2,count:2}]);
    input.capture={imuSubmissionCounts:[1,3]};
    assert.doesNotThrow(()=>validateSyntheticInput(input));
    assert.deepEqual(replayIMUBatches(input,input.frames),[{start:0,count:1},{start:1,count:2}]);
    input.capture.imuSubmissionCounts=[2,1];assert.throws(()=>validateSyntheticInput(input),/submission counts/);
    input.capture.imuSubmissionCounts=[1,100];assert.throws(()=>validateSyntheticInput(input),/submission counts/);
});

function effectiveConfig() {
    return {schema:'mobile-slam-effective-config-v1',frameTimestamp:-1,executionSeed:0,cvThreads:1,params:{
        camera_model:2,width:320,height:240,fx:260,fy:255,cx:159.5,cy:119.5,acc_n:.5,acc_w:.003,gyr_n:.2,gyr_w:.0002,
        g_0:0,g_1:0,g_2:9.81,r_ic_00:1,r_ic_01:0,r_ic_02:0,r_ic_10:0,r_ic_11:0,r_ic_12:1,r_ic_20:0,r_ic_21:-1,r_ic_22:0,
        t_ic_0:.02,t_ic_1:-.01,t_ic_2:-.02,camera_parameter_0:0,camera_parameter_1:0,camera_parameter_2:0,camera_parameter_3:0,
        solver_time:.06,num_iterations:10,max_cnt:120,lk_window_size:21,lk_pyramid_levels:3,min_dist:30,f_threshold_edge_factor:0,
        pnp_enable:0,pnp_freq:3,f_threshold:1}};
}

test('Captured profile comes from actual C++ readback and retains wrapper-only guard provenance', () => {
    const actual=profileFromEffectiveConfig(effectiveConfig(),{mobileMotionGuard:true});
    assert.equal(actual.config.fx,260);assert.equal(actual.config.fy,255);assert.equal(actual.config.mobileMotionGuard,true);
    assert.deepEqual(actual.config.r_ic,[1,0,0,0,0,1,0,-1,0]);assert.deepEqual(actual.config.t_ic,[.02,-.01,-.02]);
    assert.deepEqual(actual.parameters.mobile,[.06,10,120]);assert.deepEqual(actual.parameters.tracking,[21,3,30,0]);
    assert.doesNotThrow(()=>assertEffectiveProfile(effectiveConfig(),actual.config,actual.parameters));
    const stale=effectiveConfig();stale.frameTimestamp=0;assert.throws(()=>profileFromEffectiveConfig(stale,{}),/before the first/);
    const partial=effectiveConfig();delete partial.params.pnp_enable;assert.throws(()=>profileFromEffectiveConfig(partial,{}),/PnP/);
});

test('Successful parameter acknowledgement cannot hide wrong effective solver or calibration', () => {
    const effective=effectiveConfig(),requested=profileFromEffectiveConfig(effective,{});
    effective.params.solver_time=10;
    assert.throws(()=>assertEffectiveProfile(effective,requested.config,requested.parameters),/parameters.mobile/);
    effective.params.solver_time=.06;effective.params.fx=466;
    assert.throws(()=>assertEffectiveProfile(effective,requested.config,requested.parameters),/config.fx/);
});

test('Experimental rotation selection requires actual engine readback', () => {
    assert.equal(parseRotationRescueOverride(),null);
    assert.equal(parseRotationRescueOverride('on'),true);
    assert.equal(parseRotationRescueOverride('off'),false);
    for(const value of ['true','1','',false])assert.throws(()=>parseRotationRescueOverride(value),/rotation-rescue/);
    const effective=effectiveConfig();
    assert.throws(()=>profileFromEffectiveConfig(effective,{rotationRescue:true}),/readback unavailable/);
    effective.params.rotation_rescue_enabled=0;
    const off=profileFromEffectiveConfig(effective,{rotationRescue:true});
    assert.equal(off.config.rotationRescue,false);
    assert.throws(()=>assertEffectiveProfile(effective,{...off.config,rotationRescue:true},off.parameters),/rotationRescue/);
    effective.params.rotation_rescue_enabled=1;
    assert.doesNotThrow(()=>assertEffectiveProfile(effective,{...off.config,rotationRescue:true},off.parameters));
});

test('Synchronous pixel guard rejects stale/wrong bytes before source identity can publish', () => {
    const source=new Uint8Array([0,10,127,255]);
    assert.doesNotThrow(()=>assertSourceFramePixels(source,source.slice(),19));
    for(const observed of [new Uint8Array([1,10,127,255]),new Uint8Array([0,10,127,254]),new Uint8Array([0,10,127])])assert.throws(()=>assertSourceFramePixels(source,observed,19),/Source frame 19 pixel/);
    assert.throws(()=>assertSourceFramePixels(source,undefined,19),/dimensions mismatch/);
});

test('Native rAF mode distinguishes browser animation scheduling from mocked camera exposure', () => {
    const native=schedulerContract('live-paced-native-raf'),controlled=schedulerContract('live-paced');
    assert.equal(native.nativeAnimationFrame,true);assert.equal(native.nativeVideoDelivery,true);assert.equal(native.physicalExposureVerified,false);
    assert.match(native.videoMetadata,/mocked/);assert.match(native.animationSchedule,/unmodified browser/);
    assert.equal(controlled.nativeAnimationFrame,false);assert.match(controlled.animationSchedule,/one controlled/);
    assert.throws(()=>schedulerContract('natural-phone-camera'),/Unknown/);
});

test('Older immutable pixel/time pair stays valid after a newer source publication', () => {
    const a={gray:new Uint8Array([7,11,23]),frame:{index:1},presentedFrames:2,captureGeneration:8};
    const b={gray:new Uint8Array([17,21,33]),frame:{index:2},presentedFrames:3,captureGeneration:8};
    const publications=new Map([[.05,a],[.1,b]]),timing={timestampS:.05,presentedFrames:2,captureGeneration:8};
    const selected=resolvePublishedFrame(publications,.05,timing,8);
    assert.equal(selected,a,'Timestamp selects A even though B is the latest publication');
    assert.doesNotThrow(()=>assertSourceFramePixels(selected.gray,a.gray,1));
    assert.throws(()=>assertSourceFramePixels(selected.gray,b.gray,1),/pixel mismatch/,'The old App old-time/new-video race still fails');
    assert.throws(()=>resolvePublishedFrame(publications,.15,{...timing,timestampS:.15}),/no verified source/);
    assert.throws(()=>resolvePublishedFrame(publications,.05,{...timing,presentedFrames:3}),/metadata differs/);
    assert.throws(()=>resolvePublishedFrame(publications,.05,{...timing,captureGeneration:9},8),/generation differs/);
    assert.throws(()=>resolvePublishedFrame(publications,.05,timing,9),/generation differs/,'An old valid pair cannot use a new capture/configuration generation');
    a.clientEpoch=3;
    timing.clientEpoch=3;
    assert.doesNotThrow(()=>resolvePublishedFrame(publications,.05,timing,8,3));
    assert.throws(()=>resolvePublishedFrame(publications,.05,timing,8,4),/client epoch differs/);
    assert.throws(()=>resolvePublishedFrame(publications,.05,{...timing,clientEpoch:2},8,3),/packet client epoch differs/);
    a.geometry={sourceWidth:320,sourceHeight:240,processWidth:320,processHeight:240};
    Object.assign(timing,a.geometry);
    assert.doesNotThrow(()=>resolvePublishedFrame(publications,.05,timing,8,3));
    assert.throws(()=>resolvePublishedFrame(publications,.05,{...timing,processWidth:240},8,3),/packet geometry differs/);
});

test('F-threshold override is explicit and no-override mode preserves the App default', () => {
    assert.equal(parseFThresholdOverride(),1);assert.equal(parseFThresholdOverride('default'),null);
    assert.equal(parseFThresholdOverride('5'),5);assert.equal(parseFThresholdOverride('0.5'),.5);
    for(const invalid of ['', ' ', '0','-1','100','NaN','Infinity','false'])assert.throws(()=>parseFThresholdOverride(invalid),/f-threshold/);
});
