/** Generated inputs through the real Worker or App/Camera/IMU entry points. */
import fs from 'node:fs/promises';
import path from 'node:path';
import http from 'node:http';
import { createHash } from 'node:crypto';
import { fileURLToPath } from 'node:url';
import { parseArgs } from 'node:util';
import { parsePGM, imuBatchRanges, summarizeResults } from './replay-android-input.mjs';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const inside = (directory, file) => file === directory || file.startsWith(directory + path.sep);
const modes = ['raw', 'live-deterministic', 'live-paced', 'live-paced-native-raf'];

export function schedulerContract(mode) {
    if(!modes.includes(mode))throw new Error('Unknown synthetic browser mode');
    return {nativeAnimationFrame:mode==='live-paced-native-raf',nativeVideoDelivery:mode!=='raw',
        videoMetadata:mode==='raw'?'none':'mocked source captureTime after verified native video delivery',
        animationSchedule:mode==='live-paced-native-raf'?'unmodified browser requestAnimationFrame/cancelAnimationFrame':mode==='raw'?'none':'one controlled callback per presented source frame',
        physicalExposureVerified:false};
}

export function parseFThresholdOverride(value='1') {
    if(value==='default')return null;
    const threshold=typeof value==='string'&&value.trim()?Number(value):NaN;
    if(!Number.isFinite(threshold)||threshold<=0||threshold>=100)throw new Error('f-threshold must be default or a finite value greater than0 and less than100');
    return threshold;
}

export function validateSyntheticInput(input, count = 400) {
    if (input?.schema !== 'mobile-slam-synthetic-vio-v1') throw new Error('Synthetic input schema required; Android records are not synthetic fixtures');
    if (!Number.isInteger(count) || count < 1 || count > 2000) throw new Error('frames must be 1..2000');
    const c = input.config;
    if (!c || ![c.width,c.height].every(n => Number.isInteger(n) && n > 0) || c.width*c.height > 16777216) throw new Error('Invalid image dimensions');
    if (!Array.isArray(input.frames) || !input.frames.length || !Array.isArray(input.imu) || !input.imu.length) throw new Error('Image and IMU inputs required');
    let previous = -Infinity, previousPhysical = -Infinity, previousIndex = -1;
    for (const frame of input.frames) {
        if (!Number.isInteger(frame.index) || frame.index <= previousIndex || !Number.isFinite(frame.timestamp) || frame.timestamp < 0 || frame.timestamp <= previous ||
            !Number.isFinite(frame.physicalTimestamp) || frame.physicalTimestamp < 0 || frame.physicalTimestamp <= previousPhysical ||
            typeof frame.file !== 'string' || path.basename(frame.file) !== frame.file || !/^[a-f0-9]{64}$/.test(frame.sha256)) throw new Error('Invalid physical frame identity, ordering, filename or hash');
        previous = frame.timestamp; previousPhysical = frame.physicalTimestamp; previousIndex = frame.index;
    }
    previous = -Infinity;
    for (const row of input.imu) {
        if (!Array.isArray(row) || row.length !== 7 || !row.every(Number.isFinite) || row[0] < 0 || row[0] <= previous) throw new Error('Invalid IMU ordering or values');
        previous = row[0];
    }
    const p = input.parameters;
    if (!p || !Array.isArray(p.mobile) || p.mobile.length !== 3 || !p.mobile.every(Number.isFinite) ||
        !Array.isArray(p.tracking) || p.tracking.length !== 4 || !p.tracking.every(Number.isFinite) ||
        !Array.isArray(p.pnp) || p.pnp.length !== 2 || typeof p.pnp[0] !== 'boolean' || !Number.isFinite(p.pnp[1]) ||
        !Number.isFinite(p.fThreshold) || !Array.isArray(p.execution) || p.execution.length !== 2 || !p.execution.every(Number.isInteger)) throw new Error('Explicit estimator profile required');
    const frames = input.frames.slice(0, count);
    if (c.width*c.height*frames.length > 512*1024*1024) throw new Error('Raw image budget exceeds 512 MiB');
    if (input.imu[0][0] > frames[0].timestamp || input.imu.at(-1)[0] < frames.at(-1).timestamp) throw new Error('Missing past/future IMU bracket');
    if(input.capture?.imuSubmissionCounts) {
        const counts=input.capture.imuSubmissionCounts;
        if(!Array.isArray(counts)||counts.length!==input.frames.length||counts.some((value,index)=>!Number.isInteger(value)||value<1||value>input.imu.length||(index>0&&value<counts[index-1])))throw new Error('Invalid captured IMU submission counts');
    }
    return frames;
}

export function replayIMUBatches(input, frames) {
    const counts=input.capture?.imuSubmissionCounts;
    return counts?frames.map((_,index)=>({start:index?counts[index-1]:0,count:counts[index]-(index?counts[index-1]:0)})):imuBatchRanges(frames,input.imu);
}

/** Independent inverse of the declared device-to-body +90 degree x rotation. */
export function bodyToDevice(row) {
    return [row[0], row[1], row[3], -row[2], row[4], row[6], -row[5]];
}

export function sourceCoverage(frames, accepted, rows) {
    const acceptedIds = new Set(accepted.map(row => row.sourceIndex));
    const freshIds = new Set(rows.filter(row => row.poseFresh && row.poseValid && row.pose?.length === 16 && row.pose.every(Number.isFinite)).map(row => row.sourceIndex));
    const postWarmup = frames.filter(frame => frame.physicalTimestamp >= 5 && frame.physicalTimestamp < 20);
    return { planned: frames.length, accepted: acceptedIds.size, droppedOrSkipped: frames.length - acceptedIds.size,
        completed: rows.length, fresh: freshIds.size, postWarmupPlanned: postWarmup.length,
        postWarmupFresh: postWarmup.filter(frame => freshIds.has(frame.index)).length,
        postWarmupFreshCoverage: postWarmup.length ? postWarmup.filter(frame => freshIds.has(frame.index)).length / postWarmup.length : null,
        fixed400SourceScope: frames.length === 400 && frames[0].physicalTimestamp === 0 && frames.at(-1).physicalTimestamp === 19.95 };
}

export function distribution(values) {
    const ordered=values.filter(Number.isFinite).sort((a,b)=>a-b);
    const at=p=>ordered[Math.max(0,Math.ceil(p*ordered.length)-1)]??null;
    return {count:ordered.length,p50:at(.5),p95:at(.95),p99:at(.99),max:ordered.at(-1)??null};
}

export function validateResultIdentity(rows, accepted) {
    const bySource=new Map(accepted.map(frame=>[frame.sourceIndex,frame]));
    const seen=new Set();
    for(const row of rows) {
        const frame=bySource.get(row.sourceIndex);
        if(!frame||seen.has(row.sourceIndex)||row.inputTimestamp!==frame.timestamp||row.physicalTimestamp!==frame.physicalTimestamp)throw new Error('Result lost accepted physical frame identity');
        if((row.poseFresh||row.poseValid)&&(!Number.isFinite(row.poseTimestamp)||row.poseTimestamp!==row.inputTimestamp))throw new Error('Fresh pose timestamp is stale or mismatched');
        seen.add(row.sourceIndex);
    }
    if(seen.size!==accepted.length)throw new Error('Missing accepted frame result');
}

export function assertSourceFrameHash(expected, observed, index) {
    if(!/^[a-f0-9]{64}$/.test(expected)||!/^[a-f0-9]{64}$/.test(observed)||expected!==observed) {
        throw new Error(`Source frame ${index} pixel hash mismatch: stale frame or undeclared color conversion`);
    }
}

/** Also injected verbatim into the page: synchronous, before source identity publication. */
export function assertSourceFramePixels(expected, observed, index) {
    if(!(expected instanceof Uint8Array)||!(observed instanceof Uint8Array)||expected.length!==observed.length)throw new Error(`Source frame ${index} pixel dimensions mismatch`);
    for(let pixel=0;pixel<expected.length;pixel++)if(expected[pixel]!==observed[pixel])throw new Error(`Source frame ${index} pixel mismatch at ${pixel}`);
}

/** Timestamp selects the oracle. Pixels can validate it, never select or relabel it. */
export function resolvePublishedFrame(publications, timestamp, timing, activeGeneration=null, activeClientEpoch=null) {
    const source=publications.get(timestamp);
    if(!source)throw new Error('Submitted timestamp has no verified source publication');
    if(timing?.timestampS!==timestamp||timing.presentedFrames!==source.presentedFrames)throw new Error('Submitted frame metadata differs from its verified publication');
    if(source.captureGeneration!==null&&(timing.captureGeneration!==source.captureGeneration||activeGeneration!==source.captureGeneration))throw new Error('Submitted frame capture generation differs from its verified publication/current configuration');
    if(source.clientEpoch!==undefined&&source.clientEpoch!==activeClientEpoch)throw new Error('Submitted frame client epoch differs from its verified publication');
    if(source.captureGeneration!==null&&source.clientEpoch!==undefined&&timing.clientEpoch!==source.clientEpoch)throw new Error('Submitted packet client epoch differs from its verified publication');
    if(source.captureGeneration!==null&&source.geometry)for(const [key,value]of Object.entries(source.geometry))if(timing[key]!==value)throw new Error(`Submitted packet geometry differs: ${key}`);
    return source;
}

/** Readback values, not successful RPC acknowledgements, define replay settings. */
export function profileFromEffectiveConfig(effective, requestedConfig) {
    const p=effective?.params;
    if(effective?.schema!=='mobile-slam-effective-config-v1'||!p||effective.frameTimestamp!==-1)throw new Error('Effective configuration must be read before the first input');
    if(![0,1].includes(p.pnp_enable))throw new Error('Incomplete effective PnP configuration');
    const config={...requestedConfig,modelType:p.camera_model};
    for(const key of ['width','height','fx','fy','cx','cy','acc_n','acc_w','gyr_n','gyr_w'])config[key]=p[key];
    config.g_norm=Math.hypot(p.g_0,p.g_1,p.g_2);
    config.r_ic=Array.from({length:9},(_,i)=>p[`r_ic_${Math.floor(i/3)}${i%3}`]);
    config.t_ic=Array.from({length:3},(_,i)=>p[`t_ic_${i}`]);
    for(let i=0;i<4;i++)config[`k${i+2}`]=p[`camera_parameter_${i}`];
    const parameters={mobile:[p.solver_time,p.num_iterations,p.max_cnt],tracking:[p.lk_window_size,p.lk_pyramid_levels,p.min_dist,p.f_threshold_edge_factor],
        pnp:[p.pnp_enable===1,p.pnp_freq],fThreshold:p.f_threshold,execution:[effective.executionSeed,effective.cvThreads]};
    if(!Object.values(config).flat().every(value=>typeof value==='boolean'||Number.isFinite(value))||
        !Object.values(parameters).flat().every(value=>typeof value==='boolean'||Number.isFinite(value)))throw new Error('Incomplete effective configuration readback');
    return {config,parameters};
}

export function assertEffectiveProfile(effective, requestedConfig, requestedParameters) {
    const actual=profileFromEffectiveConfig(effective,requestedConfig);
    const compare=(expected,observed,label)=>{
        if(Array.isArray(expected)){if(!Array.isArray(observed)||expected.length!==observed.length)throw new Error(`Effective profile mismatch: ${label}`);expected.forEach((value,index)=>compare(value,observed[index],`${label}[${index}]`));}
        else if(typeof expected==='number'?Math.abs(expected-observed)>1e-12*Math.max(1,Math.abs(expected)):expected!==observed)throw new Error(`Effective profile mismatch: ${label}`);
    };
    for(const [key,value]of Object.entries(requestedConfig))compare(value,actual.config[key],`config.${key}`);
    for(const [key,value]of Object.entries(requestedParameters))compare(value,actual.parameters[key],`parameters.${key}`);
    return actual;
}

async function rendererAssets() {
    const base = path.join(root, 'build/dev-tools/three-0.170.0');
    const assets = [];
    for (const relative of ['build/three.module.js', 'examples/jsm/controls/OrbitControls.js']) {
        const url = `https://cdn.jsdelivr.net/npm/three@0.170.0/${relative}`;
        const file = path.join(base, relative);
        let bytes;
        try { bytes = await fs.readFile(file); }
        catch (error) {
            if (error.code !== 'ENOENT') throw error;
            const response = await fetch(url, { signal: AbortSignal.timeout(30000) });
            if (!response.ok) throw new Error(`Pinned renderer fetch failed: ${url} ${response.status}`);
            bytes = Buffer.from(await response.arrayBuffer());
            await fs.mkdir(path.dirname(file), { recursive: true }); await fs.writeFile(file, bytes, { flag: 'wx' });
        }
        assets.push({ url, file, bytes });
    }
    return assets;
}

async function rawReplay(page, input, frames) {
    return page.evaluate(async ({ input, frames, batches }) => {
        const { VIOWrapper } = await import('/js/vio-wrapper.js');
        const vio = new VIOWrapper({ frameTimeoutMs: 60000 });
        const rows = [], accepted = [];
        try {
            await vio.load('/candidate/vio_engine.js');
            const p = input.parameters;
            const config = { ...input.config, execution_seed: p.execution[0], cv_threads: p.execution[1] };
            if (!await vio.configure(config)) throw new Error('Synthetic config rejected');
            for (const [method, values] of [['setMobileParams', p.mobile], ['setTrackingParams', p.tracking], ['setPnPParams', p.pnp], ['setFThreshold', [p.fThreshold]]]) {
                if (!await vio[method](...values)) throw new Error(`${method} rejected`);
            }
            const effectiveConfig=await vio.getEffectiveConfig();
            for (const [index, frame] of frames.entries()) {
                const { start, count } = batches[index];
                if (count && !vio.sendIMU(new Float64Array(input.imu.slice(start, start + count).flat()), count)) throw new Error(`IMU rejected at ${index}`);
                const response = await fetch(`/images/${index}`);
                const gray = new Uint8Array(await response.arrayBuffer());
                const started = performance.now();
                if (!vio.sendFrame(gray, frame.timestamp)) throw new Error(`Frame rejected at ${index}`);
                accepted.push({ sourceIndex: frame.index, physicalTimestamp: frame.physicalTimestamp, timestamp: frame.timestamp });
                await vio.waitForFree(60000);
                const result = vio.getLatestResult();
                if (!result || result.inputTimestamp !== frame.timestamp) throw new Error(`Mismatched result at ${index}`);
                const { mapPoints, ...fields } = result;
                rows.push({ ...fields, index, frame: index, sourceIndex: frame.index, sourceFrameIndex: frame.index, physicalTimestamp: frame.physicalTimestamp,
                    submittedIMUStart: start, submittedIMUCount: count, roundTripMs: performance.now() - started,
                    pose: result.pose ? Array.from(result.pose) : null });
            }
            return { rows, accepted, config, parameters: input.parameters, effectiveConfig, effectiveConfigStage:'after_all_profile_RPCS_before_first_input',transport: vio.getMetrics(),
                imuSamplesSubmitted: batches.at(-1).start + batches.at(-1).count,
                imuPolicy:input.capture?.imuSubmissionCounts?'captured_cumulative_before_frame':'one_future_bracket',secureContext: isSecureContext, crossOriginIsolated };
        } finally { vio.dispose(); }
    }, { input, frames, batches: replayIMUBatches(input, frames) });
}

/** Replace only browser device/clock APIs; App, Camera, IMU, Worker and renderer stay real. */
function installDeviceBoundary({ width, height, sensor, deterministic, orientationType, nativeRAF }) {
    const realNow = performance.now.bind(performance);
    const nativeRVFC = HTMLVideoElement.prototype.requestVideoFrameCallback;
    const nativeCancelRVFC = HTMLVideoElement.prototype.cancelVideoFrameCallback;
    const state = window.__synthetic = { realNow, nativeRVFC, nativeCancelRVFC, virtualMs: 100000,
        raf: new Map(), rvfc: new Map(), nextId: 1, sensors: [], current: null, deterministic,
        drawReadback: [], captures: [], accepted: [], frameAttempts: [], submittedIMU: [], rows: [], calls: [], timings: [], sourceDelivery: [],
        nativeRAF,observerTimings:[],rafTicks:0,rafIntervals:[],lastRAFTime:null,publications:new Map(),acceptedTimestamps:new Set() };
    if (deterministic) Object.defineProperty(performance, 'now', { configurable: true, value: () => state.virtualMs });
    if(!nativeRAF) {
        window.requestAnimationFrame = callback => { const id = state.nextId++; state.raf.set(id, callback); return id; };
        window.cancelAnimationFrame = id => state.raf.delete(id);
    }
    HTMLVideoElement.prototype.requestVideoFrameCallback = function(callback) { const id = state.nextId++; state.rvfc.set(id, { video: this, callback }); return id; };
    HTMLVideoElement.prototype.cancelVideoFrameCallback = id => state.rvfc.delete(id);
    Object.defineProperty(screen.orientation, 'type', { configurable: true, get: () => orientationType });
    Object.defineProperty(screen.orientation, 'lock', { configurable: true, value: async () => { throw new DOMException('Synthetic fixed orientation', 'NotSupportedError'); } });
    const canvas = document.createElement('canvas'); canvas.width = width; canvas.height = height;
    const ctx = canvas.getContext('2d', { alpha: false }); ctx.fillStyle = '#101010'; ctx.fillRect(0, 0, width, height);
    state.canvas = canvas; state.ctx = ctx; state.stream = canvas.captureStream(0); state.track = state.stream.getVideoTracks()[0];
    const originalSettings = state.track.getSettings.bind(state.track);
    state.track.getSettings = () => ({ ...originalSettings(), facingMode: 'environment', deviceId: 'synthetic-generated-camera' });
    navigator.mediaDevices.getUserMedia = async () => {
        // Repeated requests allow the real HTMLVideoElement to finish its startup.
        state.startupCapture = setInterval(() => state.track.requestFrame(), 25);
        return state.stream;
    };
    navigator.mediaDevices.enumerateDevices = async () => [{ kind: 'videoinput', deviceId: 'synthetic-generated-camera', label: 'camera 0, facing back' }];
    const query = navigator.permissions.query.bind(navigator.permissions);
    navigator.permissions.query = async descriptor => ['accelerometer', 'gyroscope'].includes(descriptor.name) ? { state: 'granted' } : query(descriptor);
    class MockSensor extends EventTarget {
        constructor(options, type) { super(); this.options = options; this.type = type; this.active = false; state.sensors.push(this); }
        start() { this.active = true; }
        stop() { this.active = false; }
    }
    window.Accelerometer = sensor === 'generic' ? class extends MockSensor { constructor(options) { super(options, 'accel'); } } : undefined;
    window.Gyroscope = sensor === 'generic' ? class extends MockSensor { constructor(options) { super(options, 'gyro'); } } : undefined;
    // W3C rotationRate alpha/beta/gamma identify device x/y/z; these are not
    // DeviceOrientation Euler angles. Values are independently inverse-rotated.
    state.emit = row => {
        const device = [row[0], row[1], row[3], -row[2], row[4], row[6], -row[5]];
        if (sensor === 'generic') {
            for (const type of ['accel', 'gyro']) for (const target of state.sensors.filter(s => s.active && s.type === type)) {
                const offset = type === 'accel' ? 1 : 4;
                [target.x, target.y, target.z] = device.slice(offset, offset + 3); target.timestamp = device[0] * 1000;
                target.dispatchEvent(new Event('reading'));
            }
        } else {
            const event = new Event('devicemotion');
            Object.defineProperties(event, { timeStamp: { value: device[0]*1000 },
                accelerationIncludingGravity: { value: { x: device[1], y: device[2], z: device[3] } },
                acceleration: { value: null }, rotationRate: { value: { alpha: device[4]*180/Math.PI, beta: device[5]*180/Math.PI, gamma: device[6]*180/Math.PI } }, interval: { value: 5 } });
            window.dispatchEvent(event);
        }
    };
}

async function liveReplay(page, input, frames, mode) {
    await page.waitForFunction(() => window.__mobileSLAM?.app?.startBtn?.disabled === false, null, { timeout: 60000, polling: 100 });
    await page.addScriptTag({content:`window.__assertSyntheticSourcePixels = (${assertSourceFramePixels.toString()}); window.__resolveSyntheticPublication = (${resolvePublishedFrame.toString()});`});
    return page.evaluate(async ({ input, frames, deterministic, nativeRAF }) => {
        const state = window.__synthetic, app = window.__mobileSLAM.app;
        const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
        const digest = async bytes => Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256', bytes))).map(v => v.toString(16).padStart(2, '0')).join('');
        const wrap = (object, name, observe) => { const fn = object[name].bind(object); object[name] = (...args) => observe(fn, args); };
        const associationFailure=(stage,expected,observed,fields={})=>{
            const mismatches=[];let count=0,maxDelta=0;
            for(let i=0;i<Math.min(expected?.length??0,observed?.length??0);i++)if(expected[i]!==observed[i]) {
                count++;maxDelta=Math.max(maxDelta,Math.abs(expected[i]-observed[i]));if(mismatches.length<8)mismatches.push({pixel:i,expected:expected[i],observed:observed[i]});
            }
            const equals=source=>source instanceof Uint8Array&&observed instanceof Uint8Array&&source.length===observed.length&&source.every((value,index)=>value===observed[index]);
            state.associationFailure={stage,arrivalMs:state.realNow(),publishedSourceFrame:state.current?.index??null,
                pendingSourceFrame:state.pendingSource?.index??null,publishedSourceSha256:state.current?.sha256??null,pendingSourceSha256:state.pendingSource?.sha256??null,
                matchesPublishedSource:equals(state.expectedGray),matchesPendingSource:equals(state.pendingSource?.gray),
                mismatchedPixels:count,maxGrayDelta:maxDelta,firstMismatches:mismatches,appTiming:app.lastFrameTiming,...fields};
        };
        if(nativeRAF)wrap(app,'processLoop',(fn,args)=>{
            const now=state.realNow();if(state.current&&app.running){state.rafTicks++;if(state.lastRAFTime!==null)state.rafIntervals.push(now-state.lastRAFTime);state.lastRAFTime=now;}
            return fn(...args);
        });
        for (const name of ['configure', 'setMobileParams', 'setTrackingParams', 'setPnPParams', 'setFThreshold']) wrap(app.vio, name, (fn, args) => {
            state.calls.push({ method: name, args: structuredClone(args) }); return fn(...args);
        });
        wrap(app.vio, 'sendIMU', (fn, args) => {
            const [values, count] = args, result = fn(...args);
            if (result) for (let i = 0; i < count; i++) state.submittedIMU.push(Array.from(values.slice(i*7, i*7+7)));
            return result;
        });
        wrap(app.camera, 'captureGrayscale', (fn, args) => {
            const started = state.realNow(), gray = fn(...args);
            if (gray && state.current) state.captures.push({ sourceIndex: state.current.index, initialized: app._vioInitialized, ms: state.realNow() - started });
            return gray;
        });
        wrap(app.vio, 'sendFrame', (fn, args) => {
            const [gray, timestamp] = args, copy = gray.slice(), started = state.realNow();
            let source;
            try {
                source=window.__resolveSyntheticPublication(state.publications,timestamp,app.lastFrameTiming,app.captureGeneration??null,app.vio.getMetrics().clientEpoch);
                if(state.acceptedTimestamps.has(timestamp))throw new Error('Source timestamp already accepted');
                window.__assertSyntheticSourcePixels(source.gray,gray,source.frame.index);
            }catch(error){associationFailure('pre_submit_association_guard',source?.gray,gray,{inputTimestamp:timestamp,submittedSourceFrame:source?.frame.index??null});state.associationError=error.message;throw error;}
            state.observerTimings.push({stage:'pre_submit_association_guard',sourceIndex:source.frame.index,ms:state.realNow()-started});
            const accepted = fn(...args), elapsed = state.realNow() - started;
            state.frameAttempts.push({sourceIndex:source.frame.index,accepted});
            if(accepted)state.acceptedTimestamps.add(timestamp);
            if (accepted) state.accepted.push({ sourceIndex: source.frame.index, physicalTimestamp: source.frame.physicalTimestamp,
                timestamp, gray: copy, expected: source.gray, submittedIMUCount:state.submittedIMU.length,
                submittedWallMs: started, submitCallMs: elapsed, sourceTiming: { ...app.lastFrameTiming } });
            return accepted;
        });
        wrap(app.vio, '_handleWorkerMessage', (fn, args) => {
            const result = fn(...args), latest = app.vio.getLatestResult();
            if (latest && !state.rows.some(row => row.sequence === latest.sequence && row.clientEpoch === latest.clientEpoch)) {
                const source = state.accepted.find(row => row.timestamp === latest.inputTimestamp);
                if (!source) throw new Error('Worker result lacks accepted physical frame');
                const { mapPoints, ...fields } = latest;
                state.rows.push({ ...fields, frame: state.rows.length, sourceIndex: source.sourceIndex, sourceFrameIndex: source.sourceIndex, physicalTimestamp: source.physicalTimestamp,
                    pose: latest.pose ? Array.from(latest.pose) : null, roundTripMs: state.realNow() - source.submittedWallMs,
                    poseAgeWallMs: !deterministic&&latest.poseFresh&&latest.poseValid&&Number.isFinite(latest.poseTimestamp) ? state.realNow() - latest.poseTimestamp*1000 : null });
            }
            return result;
        });
        if (app.renderer) wrap(app.renderer, 'render', (fn, args) => {
            const started = state.realNow(), result = fn(...args); state.timings.push({ stage: 'render', sourceIndex: state.current?.index ?? null, initialized: app._vioInitialized, ms: state.realNow() - started }); return result;
        });
        await app.start();
        if (!app.running) throw new Error(`App failed to start: ${app.statusEl.textContent}`);
        const effectiveConfig=await app.vio.getEffectiveConfig();
        clearInterval(state.startupCapture);
        const video = app.camera.getVideoElement();
        if (!(video instanceof HTMLVideoElement) || !(video.srcObject instanceof MediaStream)) throw new Error('Actual video stream route missing');
        const startup = app.imu.getDiagnostics();
        const rgbCanvas = document.createElement('canvas'); rgbCanvas.width = input.config.width; rgbCanvas.height = input.config.height;
        const rgbContext = rgbCanvas.getContext('2d', { willReadFrequently: true });
        const sourceGray = await Promise.all(frames.map(async (_, index) => new Uint8Array(await (await fetch(`/images/${index}`)).arrayBuffer())));
        const epochMs = deterministic ? state.virtualMs : state.realNow() + 100;
        let imuCursor = 0;
        const emitUntil = timestamp => {
            while (imuCursor < input.imu.length && input.imu[imuCursor][0] <= timestamp + 1e-10) {
                const row = [...input.imu[imuCursor++]]; row[0] += epochMs/1000;
                state.emit(row);
            }
        };
        let sensorTimer;
        if (!deterministic) sensorTimer = setInterval(() => emitUntil((state.realNow()-epochMs)/1000), 2);
        try {
            for (const [ordinal, frame] of frames.entries()) {
                if(state.associationError)throw new Error(state.associationError);
                const scheduledMs = epochMs + frame.timestamp*1000;
                if (deterministic) { state.virtualMs = scheduledMs; emitUntil(frame.timestamp); }
                else {
                    await delay(Math.max(0, scheduledMs-state.realNow()));
                    if (state.realNow()-scheduledMs >= 50) { state.sourceDelivery.push({ sourceIndex: frame.index, status: 'schedule_overrun', latenessMs: state.realNow()-scheduledMs }); continue; }
                }
                const gray = sourceGray[ordinal], rgba = new Uint8ClampedArray(gray.length*4);
                if(nativeRAF)state.pendingSource={index:frame.index,sha256:frame.sha256,gray};
                for (let i=0; i<gray.length; i++) { rgba[i*4]=rgba[i*4+1]=rgba[i*4+2]=gray[i]; rgba[i*4+3]=255; }
                // The real native compositor must present this exact submitted canvas
                // before an API-boundary metadata callback is made visible to App.
                const readPixels=()=>{
                    const started=state.realNow();
                    rgbContext.drawImage(video,0,0);
                    const pixels=rgbContext.getImageData(0,0,input.config.width,input.config.height).data;
                    const transformed=new Uint8Array(gray.length);let changed=0,maxDelta=0;
                    for(let i=0;i<gray.length;i++) {
                        transformed[i]=(pixels[i*4]*77+pixels[i*4+1]*150+pixels[i*4+2]*29)>>8;
                        const delta=Math.abs(transformed[i]-gray[i]);if(delta)changed++;maxDelta=Math.max(maxDelta,delta);
                    }
                    if(nativeRAF)state.observerTimings.push({stage:'source_video_readback',sourceIndex:frame.index,ms:state.realNow()-started});
                    return {transformed,changed,maxDelta};
                };
                let readback,delivery;
                const publish=()=>{
                    state.current=frame;state.currentSourceTimeS=scheduledMs/1000;state.expectedGray=readback.transformed;
                    if(state.publications.has(state.currentSourceTimeS))throw new Error('Duplicate verified source timestamp publication');
                    // Registration follows exact pixel verification and precedes
                    // callback delivery. Keep the independent source PGM buffer.
                    const publication={frame,gray,presentedFrames:ordinal+1,
                        captureGeneration:Number.isSafeInteger(app.captureGeneration)?app.captureGeneration:null,clientEpoch:app.vio.getMetrics().clientEpoch,
                        geometry:{sourceWidth:input.config.width,sourceHeight:input.config.height,captureWidth:input.config.width,captureHeight:input.config.height,
                            processWidth:effectiveConfig.params.width,processHeight:effectiveConfig.params.height}};
                    state.publications.set(state.currentSourceTimeS,publication);
                    delivery={sourceIndex:frame.index,status:'presented',latenessMs:deterministic?null:state.realNow()-scheduledMs};
                    state.sourceDelivery.push(delivery);
                    for(const [id,item]of [...state.rvfc]) {
                        state.rvfc.delete(id);item.callback(performance.now(),{presentedFrames:ordinal+1,mediaTime:frame.timestamp,
                            captureTime:scheduledMs,presentationTime:scheduledMs,width:input.config.width,height:input.config.height});
                    }
                    publication.packetTiming=app._videoFrameTiming?.timestampS===state.currentSourceTimeS?{...app._videoFrameTiming}:null;
                };
                const delivered = new Promise((resolve, reject) => {
                    const timer = setTimeout(() => { state.nativeCancelRVFC.call(video, id); reject(new Error(`Native canvas/video delivery timeout at ${frame.index}`)); }, 5000);
                    const id = state.nativeRVFC.call(video, (now, metadata) => {
                        clearTimeout(timer);
                        try {
                            if(nativeRAF) {
                                readback=readPixels();const started=state.realNow();
                                window.__assertSyntheticSourcePixels(gray,readback.transformed,frame.index);
                                state.observerTimings.push({stage:'source_exact_pixel_guard',sourceIndex:frame.index,ms:state.realNow()-started});
                                publish(); // Synchronous inside native rVFC, before any awaited hash/RPC.
                            }
                            resolve({now,metadata});
                        }catch(error){associationFailure('native_rvfc_before_metadata',gray,readback?.transformed,{nativeMetadata:metadata,intendedSourceFrame:frame.index});state.associationError=error.message;reject(error);}
                    });
                });
                state.ctx.putImageData(new ImageData(rgba, input.config.width, input.config.height), 0, 0);
                state.track.requestFrame();
                const native = await delivered;
                readback??=readPixels();
                const {transformed,changed,maxDelta}=readback,hashStarted=state.realNow();
                const readbackHash=await digest(transformed);
                // This gate executes the same independently tested Node contract.
                // Never attach current-frame GT identity to a stale decoded image.
                await window.__assertSyntheticSourceHash(frame.sha256,readbackHash,frame.index);
                if(nativeRAF)state.observerTimings.push({stage:'source_sha256_audit',sourceIndex:frame.index,ms:state.realNow()-hashStarted});
                if(!nativeRAF)publish();
                state.drawReadback.push({ sourceIndex: frame.index, sourceHash: frame.sha256,
                    readbackHash, changedPixels: changed, maxGrayDelta: maxDelta,
                    transform: 'Browser CanvasCaptureMediaStream to HTMLVideoElement RGB, then integer BT.601 luma (77R+150G+29B)>>8', nativeMediaTime: native.metadata.mediaTime });
                if(!nativeRAF) {
                    const before={...app.frameMetrics};
                    const callbacks = [...state.raf]; state.raf.clear(); for (const [, callback] of callbacks) callback(performance.now());
                    delivery.appDisposition=app.frameMetrics.accepted>before.accepted?'accepted':app.frameMetrics.attempted>before.attempted?'worker_rejected':
                        app.frameMetrics.captured>before.captured?'captured_not_submitted':'not_captured_rate_or_lifecycle';
                }
                if (deterministic) await app.vio.waitForFree(60000);
            }
            const finalIMUTime=input.imu.find(row=>row[0]>frames.at(-1).timestamp)?.[0]??frames.at(-1).timestamp;
            if (deterministic) { state.virtualMs = epochMs + finalIMUTime*1000; emitUntil(finalIMUTime); }
            else await delay(Math.max(0, epochMs + frames.at(-1).timestamp*1000 + (nativeRAF?100:20) - state.realNow()));
            clearInterval(sensorTimer);
            await app.vio.waitForFree(60000);
            if(nativeRAF){await delay(34);app.running=false;if(state.associationError)throw new Error(state.associationError);}
            // Poll completion through the actual App consume path once after the last frame.
            const callbacks = [...state.raf]; state.raf.clear(); for (const [, callback] of callbacks) callback(performance.now());
            app._flushAndSendIMU();
            await delay(20);
            const accepted = [];
            for (const row of state.accepted) {
                const { expected, ...saved } = row;
                const actual = row.gray, wanted = expected;
                saved.graySha256 = await digest(actual); saved.expectedGraySha256 = await digest(wanted);
                if (saved.graySha256 !== saved.expectedGraySha256) throw new Error(`App accepted pixels differ from independently read video frame ${row.sourceIndex}`);
                const response=await fetch(`/capture/${row.sourceIndex}`,{method:'POST',body:actual});
                if(!response.ok)throw new Error(`Failed to preserve accepted pixels ${row.sourceIndex}`);
                delete saved.gray;
                accepted.push(saved);
            }
            const expectedRows=input.imu.slice(0,imuCursor).map(row=>[row[0]+epochMs/1000,...row.slice(1)]);
            const maxIMUError=new Array(7).fill(0);
            for(let i=0;i<state.submittedIMU.length;i++) for(let axis=0;axis<7;axis++) maxIMUError[axis]=Math.max(maxIMUError[axis],Math.abs(state.submittedIMU[i][axis]-(expectedRows[i]?.[axis]??Infinity)));
            return { rows: state.rows, accepted, submittedIMU: state.submittedIMU, calls: state.calls, cameraTransform: app.camera.getPixelTransform(),
                config: app._lastConfigParams, effectiveConfig,effectiveConfigStage:'after_all_profile_RPCS_before_first_input',parameters: { mobile: state.calls.find(c=>c.method==='setMobileParams')?.args,
                    tracking: state.calls.find(c=>c.method==='setTrackingParams')?.args, pnp: state.calls.find(c=>c.method==='setPnPParams')?.args,
                    fThreshold: state.calls.find(c=>c.method==='setFThreshold')?.args[0] ?? effectiveConfig.params.f_threshold, execution: [0,1] },
                startup, imu: app.imu.getDiagnostics(), transport: app.vio.getMetrics(), appFrames: { ...app.frameMetrics },
                captures: state.captures, timings: state.timings, sourceDelivery: state.sourceDelivery, drawReadback: state.drawReadback,
                frameAttempts:state.frameAttempts,observerTimings:state.observerTimings,
                sourcePublications:Array.from(state.publications,([timestamp,source])=>({timestamp,sourceFrameIndex:source.frame.index,
                    physicalTimestamp:source.frame.physicalTimestamp,presentedFrames:source.presentedFrames,captureGeneration:source.captureGeneration,clientEpoch:source.clientEpoch,geometry:source.geometry,
                    packetTiming:source.packetTiming})),
                scheduler:{nativeAnimationFrame:nativeRAF,rafTicks:state.rafTicks,rafIntervalsMs:state.rafIntervals,videoMetadata:'mocked_capture_time',
                    sourceAssociation:nativeRAF?'synchronous_exact_pixels_inside_native_rVFC_before_metadata':'SHA256_before_controlled_metadata'},
                epochMs, imuSourceEmitted:imuCursor, imuInverseTransform:{expectedRows:expectedRows.length,submittedRows:state.submittedIMU.length,maxAbsoluteErrorByColumn:maxIMUError,
                    matched:expectedRows.length===state.submittedIMU.length&&maxIMUError.every(error=>error<=1e-12)},
                secureContext: isSecureContext, crossOriginIsolated, calibration: app.calibrationProvenance,
                actualRoute: { htmlVideoElement: true, canvasCaptureStream: true, drawImage: true, app: true, camera: true, imu: true, worker: true, renderer: Boolean(app.renderer) } };
        } finally { clearInterval(sensorTimer); app.stop(); }
    }, { input, frames, deterministic: mode === 'live-deterministic', nativeRAF:mode==='live-paced-native-raf' });
}

export async function main(argv = process.argv.slice(2)) {
    const { values } = parseArgs({ args: argv, options: { input: { type: 'string' }, loader: { type: 'string' }, output: { type: 'string' },
        mode: { type: 'string', default: 'raw' }, sensor: { type: 'string', default: 'generic' }, frames: { type: 'string', default: '400' },
        'f-threshold':{type:'string',default:'1'},
        'timeout-ms': { type: 'string', default: '600000' } } });
    if (!values.input || !values.loader || !values.output || !modes.includes(values.mode) || !['generic','device-motion'].includes(values.sensor)) {
        throw new Error('Required: --input synthetic/input.json --loader isolated/vio_engine.js --output NEW_DIRECTORY --mode raw|live-deterministic|live-paced|live-paced-native-raf [--sensor generic|device-motion --frames 400 --f-threshold 1|default]');
    }
    const fThresholdOverride=parseFThresholdOverride(values['f-threshold']);
    if(values.mode==='raw'&&fThresholdOverride!==1)throw new Error('f-threshold applies only to live App modes; raw replay uses its input profile');
    const timeoutMs = Number(values['timeout-ms']);
    if (!Number.isInteger(timeoutMs) || timeoutMs < 1000 || timeoutMs > 1800000) throw new Error('timeout-ms must be 1000..1800000');
    const inputFile = await fs.realpath(values.input), loaderFile = await fs.realpath(values.loader), inputRoot = path.dirname(inputFile);
    const inputBytes = await fs.readFile(inputFile), input = JSON.parse(inputBytes), frames = validateSyntheticInput(input, Number(values.frames));
    // Captured live replay files live beside their accepted-frames directory.
    const imageRoot = await fs.realpath(path.join(inputRoot, input.capture ? 'accepted-frames' : 'mav0/cam0/data'));
    const requested = path.resolve(values.output);
    let ancestor = path.dirname(requested), canonical;
    for (;;) { try { canonical = await fs.realpath(ancestor); break; } catch (e) { if (e.code !== 'ENOENT') throw e; ancestor = path.dirname(ancestor); } }
    const output = path.resolve(canonical, path.relative(ancestor, requested));
    if (inside(inputRoot, output)) throw new Error('Output must be outside the input directory');
    const report = { schema: 'mobile-slam-synthetic-browser-v1', createdAt: new Date().toISOString(), completed: false,
        completionMeaning: 'Input transport and browser execution checks only; accuracy acceptance is independently evaluated and never inferred from completion.',
        mode: values.mode, scheduler:schedulerContract(values.mode),sensor: values.mode === 'raw' ? null : values.sensor,
        requestedAppProfile:values.mode==='raw'?null:{config:'mobile_default',calibration:'supplied_synthetic_camera_profile',
            fThresholdOverride,fThresholdSource:fThresholdOverride===null?'unchanged_App_default':'URL_override'},
        source: { path: inputFile, sha256: sha(inputBytes), specSha256: input.spec_sha256,
            storedFrames: input.frames.length, imuRows: input.imu.length, imuFloat64Sha256: sha(Buffer.from(new Float64Array(input.imu.flat()).buffer)) },
        fingerprints: {}, images: [], requests: [], console: [], pageErrors: [],
        limits: ['Synthetic scene and ideal timestamps; no phone exposure, calibration, thermal or hardware timing proof.',
            'Camera/body GT is never provided to the estimator; known synthetic calibration is supplied explicitly.',
            values.mode === 'live-deterministic' ? 'Virtual source clock and serial drain prove association; virtual throughput/transport clock fields are not real-time latency.' :
                values.mode==='live-paced-native-raf'?'Unmodified desktop browser rAF with synthetic20Hzcamera/sensors; source captureTime metadata is mocked after synchronous native-video pixel verification. Not physical exposure or phone-performance proof.':
                'Controlled wall-clock API-boundary callbacks; natural live rAF/rVFC scheduling and mobile performance are not validated.',
            'Each native video readback must exactly match the intended source grayscale hash before its source identity/timestamp is published; any color conversion mismatch fails closed.',
            'Mock sensors emit the fixture rate, including ideal 200Hz tests above the App requested60Hz; this does not prove a browser hardware delivery rate.',
            'Live mode retains actual mobile_default budget/tracking/motion guard; fixed-work raw fixture is a different profile. Captured input/profile enable a matched replay.'] };
    const routes = new Map();
    const addFile = async (url, file, mime) => { const bytes=await fs.readFile(file); routes.set(url,{bytes,mime}); report.fingerprints[url]={path:file,bytes:bytes.length,sha256:sha(bytes)}; };
    await addFile('/candidate/vio_engine.js', loaderFile, 'application/javascript');
    report.fingerprints['scripts/dev/synthetic-browser.mjs']={sha256:sha(await fs.readFile(fileURLToPath(import.meta.url)))};
    await addFile('/', path.join(root, 'web/index.html'), 'text/html');
    for (const name of ['app.js','vio-wrapper.js','vio-worker.js','camera.js','imu.js','orientation.js','renderer.js']) await addFile(`/js/${name}`, path.join(root,'web/js',name), 'application/javascript');
    for (const [index, frame] of frames.entries()) {
        const file = await fs.realpath(path.join(imageRoot, frame.file)); if (!inside(imageRoot,file)) throw new Error('Frame link escapes input directory');
        const bytes = await fs.readFile(file), gray = parsePGM(bytes, input.config.width, input.config.height);
        if (sha(gray) !== frame.sha256) throw new Error(`Frame hash mismatch ${frame.file}`);
        report.images.push({ ...frame, pgmSha256:sha(bytes) }); routes.set(`/images/${index}`, {bytes:gray,mime:'application/octet-stream'});
    }
    let profile;
    if (values.mode !== 'raw') { profile=JSON.parse(await fs.readFile(path.join(inputRoot,'camera-profile.json'),'utf8')); await addFile('/camera-profile.json',path.join(inputRoot,'camera-profile.json'),'application/json'); }
    await fs.mkdir(path.dirname(output),{recursive:true}); await fs.mkdir(output,{recursive:false});
    if(values.mode!=='raw')await fs.mkdir(path.join(output,'accepted-frames'));
    const capturedWrites=new Map();
    const server = http.createServer(async (request,response) => {
        response.setHeader('Cross-Origin-Opener-Policy','same-origin'); response.setHeader('Cross-Origin-Embedder-Policy','credentialless'); response.setHeader('Cache-Control','no-store');
        const url=new URL(request.url,'http://localhost').pathname;
        if(/^\/capture\/\d+$/.test(url)&&values.mode!=='raw'&&request.method==='POST') {
            const index=Number(url.split('/').at(-1));
            if(!frames.some(frame=>frame.index===index)||capturedWrites.has(index)){request.resume();response.writeHead(400).end();return;}
            try {
                const chunks=[];let size=0;
                for await(const chunk of request){size+=chunk.length;if(size>input.config.width*input.config.height)throw new Error('Capture exceeds image byte budget');chunks.push(chunk);}
                if(size!==input.config.width*input.config.height)throw new Error('Capture pixel dimensions changed');
                const bytes=Buffer.concat(chunks),file=`${String(index).padStart(5,'0')}.pgm`;
                await fs.writeFile(path.join(output,'accepted-frames',file),Buffer.concat([Buffer.from(`P5\n${input.config.width} ${input.config.height}\n255\n`),bytes]),{flag:'wx'});
                capturedWrites.set(index,{file,sha256:sha(bytes)});response.writeHead(200).end();
            }catch(error){report.captureError=error.message;response.writeHead(500).end();}
            return;
        }
        if (url === '/log' || url === '/favicon.ico') { request.resume(); response.writeHead(204).end(); return; }
        const entry=routes.get(url); report.requests.push({path:request.url,status:entry?200:404});
        if (!entry) { response.writeHead(404).end(); return; }
        response.setHeader('Content-Type',entry.mime); response.end(entry.bytes);
    });
    let browser,page,timer;
    try {
        await new Promise((resolve,reject)=>{server.once('error',reject);server.listen(0,'127.0.0.1',resolve);});
        const {chromium}=await import('playwright');
        browser=await chromium.launch({executablePath:process.env.BROWSER_EXECUTABLE||'/usr/bin/google-chrome',headless:true,args:['--enable-unsafe-swiftshader','--autoplay-policy=no-user-gesture-required']});
        report.browser=browser.version();report.node=process.version;
        page=await browser.newPage({viewport:{width:800,height:600}});
        page.on('console',msg=>report.console.push({type:msg.type(),text:msg.text()}));page.on('pageerror',error=>report.pageErrors.push(error.message));
        if (values.mode !== 'raw') {
            await page.exposeFunction('__assertSyntheticSourceHash',assertSourceFrameHash);
            for (const asset of await rendererAssets()) { report.fingerprints[asset.url]={path:asset.file,bytes:asset.bytes.length,sha256:sha(asset.bytes)}; await page.route(asset.url,route=>route.fulfill({body:asset.bytes,contentType:'application/javascript'})); }
            await page.addInitScript(installDeviceBoundary,{width:input.config.width,height:input.config.height,sensor:values.sensor,deterministic:values.mode==='live-deterministic',orientationType:profile.orientationType,nativeRAF:values.mode==='live-paced-native-raf'});
        } else routes.set('/',{bytes:Buffer.from('<!doctype html><title>Synthetic Worker replay</title>'),mime:'text/html'});
        const query=new URLSearchParams({wasm:'candidate/vio_engine.js',cameraProfile:'/camera-profile.json'});
        if(values.mode!=='raw'&&fThresholdOverride!==null)query.set('fth',String(fThresholdOverride));
        await page.goto(`http://127.0.0.1:${server.address().port}/${values.mode==='raw'?'':`?${query}`}`);
        report.replay=await Promise.race([values.mode==='raw'?rawReplay(page,input,frames):liveReplay(page,input,frames,values.mode),
            new Promise((_,reject)=>{timer=setTimeout(()=>reject(new Error('Bounded synthetic browser timeout')),timeoutMs);})]);
        validateResultIdentity(report.replay.rows,report.replay.accepted);
        report.effectiveProfile=assertEffectiveProfile(report.replay.effectiveConfig,report.replay.config,report.replay.parameters);
        report.summary=summarizeResults(report.replay.rows);
        const planned=input.capture?.plannedFrames&&frames.length===input.frames.length?input.capture.plannedFrames:frames;
        report.coverage=sourceCoverage(planned,report.replay.accepted,report.replay.rows);
        report.frameLedger=planned.map(frame=>{
            const accepted=report.replay.accepted.find(row=>row.sourceIndex===frame.index);
            const row=report.replay.rows.find(result=>result.sourceIndex===frame.index);
            const source=report.replay.sourceDelivery?.find(delivery=>delivery.sourceIndex===frame.index);
            const nativeDisposition=values.mode==='live-paced-native-raf'?(accepted?'accepted':
                report.replay.frameAttempts.some(attempt=>attempt.sourceIndex===frame.index)?'worker_rejected':
                report.replay.captures.some(capture=>capture.sourceIndex===frame.index)?'captured_not_submitted':
                source?.status==='schedule_overrun'?'schedule_overrun':'not_captured_rate_or_lifecycle'):null;
            return {sourceFrameIndex:frame.index,physicalTimestamp:frame.physicalTimestamp,planned:true,accepted:Boolean(accepted),completed:Boolean(row),
                fresh:Boolean(row?.poseFresh&&row?.poseValid),inputTimestamp:accepted?.timestamp??null,poseTimestamp:row?.poseTimestamp??null,
                disposition:nativeDisposition??source?.appDisposition??source?.status??(accepted?'raw_submitted':'not_in_captured_replay'),reason:row?.reason??null};
        });
        report.poseEvidence={nonzeroFreshPoses:report.summary.freshPoseFrames>0,accuracyStatus:'not_evaluated_here',
            zeroPoseMeaning:'No accuracy success can be claimed from zero fresh poses; intentional degeneracy requires its own frozen expectation.'};
        const timedKeys=['engineProcessingMs','workerProcessingMs','workerQueueMs','imuWaitMs','frameCopyMs','roundTripMs','poseAgeWallMs'];
        report.performance={clock:values.mode==='live-deterministic'?'real engine/roundTrip; virtual frameCopy/queue not latency evidence':'wall-clock desktop',
            all:Object.fromEntries(timedKeys.map(key=>[key,distribution(report.replay.rows.map(row=>row[key]))])),
            beforeInitialization:Object.fromEntries(timedKeys.map(key=>[key,distribution(report.replay.rows.filter(row=>!row.initialized).map(row=>row[key]))])),
            initialized:Object.fromEntries(timedKeys.map(key=>[key,distribution(report.replay.rows.filter(row=>row.initialized).map(row=>row[key]))])),
            capture:distribution((report.replay.captures??[]).map(row=>row.ms)),render:distribution((report.replay.timings??[]).filter(row=>row.stage==='render').map(row=>row.ms))};
        for(const [label,initialized]of[['beforeInitialization',false],['initialized',true]]) {
            report.performance[label].capture=distribution((report.replay.captures??[]).filter(row=>row.initialized===initialized).map(row=>row.ms));
            report.performance[label].render=distribution((report.replay.timings??[]).filter(row=>row.initialized===initialized&&row.stage==='render').map(row=>row.ms));
        }
        report.performance.observerOverhead=Object.fromEntries(['source_video_readback','source_exact_pixel_guard','source_sha256_audit','pre_submit_association_guard'].map(stage=>
            [stage,distribution((report.replay.observerTimings??[]).filter(row=>row.stage===stage).map(row=>row.ms))]));
        report.performance.nativeRAFIntervalsMs=distribution(report.replay.scheduler?.rafIntervalsMs??[]);
        const snapshotTimings=(report.replay.sourcePublications??[]).map(row=>row.packetTiming).filter(Boolean);
        report.performance.snapshot={measuredPublications:snapshotTimings.length,
            pixelReadMs:distribution(snapshotTimings.map(timing=>timing.pixelReadEndMs-timing.pixelReadStartMs)),
            copyAndPostReadChecksMs:distribution(snapshotTimings.map(timing=>timing.snapshotCopyMs))};
        if (values.mode !== 'raw') {
            const capturedFrames=[];
            for (const row of report.replay.accepted) {
                const saved=capturedWrites.get(row.sourceIndex);
                if(!saved||saved.sha256!==row.graySha256)throw new Error('Preserved captured bytes do not match accepted grayscale');
                capturedFrames.push({index:row.sourceIndex,physicalTimestamp:row.physicalTimestamp,timestamp:row.timestamp,...saved});
            }
            const capture={schema:'mobile-slam-synthetic-vio-v1',capture:{mode:values.mode,sensor:values.sensor,sourceInputSha256:report.source.sha256,epochMs:report.replay.epochMs,plannedFrames:frames,
                imuSubmissionCounts:report.replay.accepted.map(row=>row.submittedIMUCount)},
                spec_sha256:input.spec_sha256,config:report.effectiveProfile.config,parameters:report.effectiveProfile.parameters,
                requestedProfile:{config:report.replay.config,parameters:report.replay.parameters},effectiveConfig:report.replay.effectiveConfig,
                frames:capturedFrames,imu:report.replay.submittedIMU};
            await fs.writeFile(path.join(output,'captured-input.json'),JSON.stringify(capture)+'\n',{flag:'wx'});
            report.capturedInput={path:path.join(output,'captured-input.json'),imuRows:capture.imu.length,imuFloat64Sha256:sha(Buffer.from(new Float64Array(capture.imu.flat()).buffer))};
            delete report.replay.submittedIMU;
        }
        const transport=report.replay.transport;
        report.inputDelivery={imuInverseTransformMatched:report.replay.imuInverseTransform?.matched??null,
            workerDrops:Object.fromEntries(['overflow','stale','capacity','invalid','order','reset','invalidBatches','gapCount','lossResets'].map(key=>[key,transport.imuDrops?.[key]??null])),
            browserSensorDrops:report.replay.imu?.drops??null};
        report.inputDelivery.noTransportLossObserved=['overflow','stale','capacity','invalid','order','invalidBatches'].every(key=>report.inputDelivery.workerDrops[key]===0)&&
            report.replay.imuInverseTransform?.matched!==false;
        if (report.pageErrors.length || report.requests.some(r=>r.status!==200) || transport.framesTimedOut || transport.workerErrors || transport.imuReadingsRejected ||
            report.replay.rows.length!==report.replay.accepted.length || (values.mode==='raw' && report.replay.rows.length!==frames.length) ||
            !report.inputDelivery.noTransportLossObserved) throw new Error('Browser route/transport completeness failed');
        report.completed=true;
    } catch(error) {
        report.error=error.stack;process.exitCode=1;
        if(page)try {report.failureDiagnostics=await page.evaluate(()=>{
            const s=window.__synthetic;if(!s)return null;
            return {associationFailure:s.associationFailure??null,acceptedFrames:s.accepted.length,completedFrames:s.rows.length,
                lastSourceDeliveries:s.sourceDelivery.slice(-5),lastReadbackHashes:s.drawReadback.slice(-5),lastResults:s.rows.slice(-5),
                scheduler:{nativeRAF:s.nativeRAF,rafTicks:s.rafTicks}};
        });}catch(diagnosticError){report.diagnosticError=diagnosticError.message;}
    }
    finally {clearTimeout(timer);if(browser)await browser.close();if(server.listening)await new Promise(resolve=>server.close(resolve));await fs.writeFile(path.join(output,'report.json'),JSON.stringify(report,null,2)+'\n',{flag:'wx'});}
    console.log(JSON.stringify({output,completed:report.completed,mode:report.mode,summary:report.summary,coverage:report.coverage,error:report.error},null,2));return report;
}
if(process.argv[1]&&path.resolve(process.argv[1])===fileURLToPath(import.meta.url))main().catch(error=>{console.error(error.stack);process.exitCode=1;});
