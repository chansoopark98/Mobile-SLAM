/**
 * TUM VI Dataset Test Harness for WASM VIO Module.
 *
 * Loads TUM VI benchmark dataset (images + IMU) and feeds them to the
 * WASM VIO engine via the existing VIOWrapper/Worker pipeline.
 * This isolates whether VIO issues come from the engine itself or
 * from the mobile sensor data pipeline.
 */

import { VIOWrapper } from './vio-wrapper.js';
import { Renderer } from './renderer.js';
import * as THREE from 'three';

// ─── Constants ───────────────────────────────────────────────────────────────

const REPLAY_PARAMS = new URLSearchParams(window.location.search);
const DATASET_NAMES = ['room1', 'room4'];
function datasetBase(name) {
    if (!DATASET_NAMES.includes(name)) throw new Error(`Unsupported dataset: ${name}`);
    return `/datasets/tum/dataset-${name}_512_16/mav0`;
}
async function fetchCSV(url) {
    const response = await fetch(url);
    if (!response.ok) throw new Error(`CSV load failed: ${url} (${response.status})`);
    return response.text();
}
function assertOrdered(entries, timestamp = row => row.timestamp_s) {
    let last = -Infinity;
    for (const row of entries) {
        const current = timestamp(row);
        if (!Number.isFinite(current) || current <= last) throw new Error('Dataset timestamps must be finite and strictly ordered');
        last = current;
    }
}
function replayDeadline(wallOriginMs, firstTimestampS, timestampS, speed) {
    return wallOriginMs + (timestampS - firstTimestampS) * 1000 / speed;
}
const IMAGE_WIDTH = 512;
const IMAGE_HEIGHT = 512;

// TUM VI cam0 calibration (from config/tum_vi_room1.yaml)
const TUM_VI_CONFIG = {
    width: 512,
    height: 512,
    fx: 190.97847715128717,   // mu
    fy: 190.97330705212260,   // mv
    cx: 254.93170605935475,   // u0
    cy: 256.89744289965040,   // v0
    modelType: 0,             // C++ enum: KANNALA_BRANDT=0 (equidistant)
    k2: 0.0034823894022493434,
    k3: 0.0007150348452162257,
    k4: -0.0020532361418706202,
    k5: 0.00020293673591811182,
    // Extrinsic rotation R_imu_cam (row-major 3x3, from tum_vi_room1.yaml)
    r_ic: [
        -0.9995250378696743,   0.0075019185074052044, -0.02989013031643309,
         0.029615343885863205, -0.03439736061393144,   -0.998969345370175,
        -0.008522328211654736, -0.9993800792498829,     0.03415885127385616
    ],
    // cam0 T_cam_imu inverse: -R_cam_imu^T*t_cam_imu; primary room1/room4 dso/camchain.yaml
    t_ic: [0.045574835649698026, -0.07116180183799704, -0.04468125411714437],
    // Existing estimator noise parameters, matching config/tum_vi_room1.yaml.
    // Dataset metadata uses continuous-time densities; numeric ratios alone do not establish estimator weighting.
    acc_n: 0.04,
    acc_w: 0.0004,
    gyr_n: 0.004,
    gyr_w: 2.0e-5,
    g_norm: 9.81007,
};

// Solver parameters (matching native config)
const SOLVER_CONFIG = {
    solver_time: 0.1,
    num_iterations: 10,
    max_features: 150,
};

const IMU_FIELDS = 7; // [ts, ax, ay, az, gx, gy, gz]

// ─── CSV Parsers ─────────────────────────────────────────────────────────────

/**
 * Parse cam0/data.csv -> Array of { timestamp_s, filename }
 * CSV: "#timestamp [ns],filename"
 */
async function parseImageList(url) {
    const text = await fetchCSV(url);
    const lines = text.trim().split('\n');
    const entries = [];
    for (const line of lines) {
        const trimmed = line.trim();
        if (trimmed.startsWith('#') || trimmed.length === 0) continue;
        const comma = trimmed.indexOf(',');
        if (comma < 1 || !/^\d+\.png$/.test(trimmed.substring(comma + 1).trim())) throw new Error('Malformed dataset image entry');
        entries.push({
            timestamp_ns: trimmed.substring(0, comma),
            timestamp_s: parseFloat(trimmed.substring(0, comma)) * 1e-9,
            filename: trimmed.substring(comma + 1).trim(),
        });
    }
    assertOrdered(entries);
    return entries;
}

/**
 * Parse imu0/data.csv -> Float64Array in WASM IMUReading layout.
 *
 * CRITICAL: TUM VI CSV column order is [timestamp, gx, gy, gz, ax, ay, az]
 *           but WASM IMUReading expects [timestamp, ax, ay, az, gx, gy, gz].
 *           This parser REORDERS the fields.
 */
async function parseIMUData(url) {
    const text = await fetchCSV(url);
    const lines = text.trim().split('\n');

    // Count data lines
    let dataLineCount = 0;
    for (const line of lines) {
        const t = line.trim();
        if (t.length > 0 && !t.startsWith('#')) dataLineCount++;
    }

    const data = new Float64Array(dataLineCount * IMU_FIELDS);
    const timestamps = new Float64Array(dataLineCount);
    let idx = 0;

    for (const line of lines) {
        const trimmed = line.trim();
        if (trimmed.startsWith('#') || trimmed.length === 0) continue;

        const parts = trimmed.split(',');
        if (parts.length !== 7 || parts.some(value => !Number.isFinite(Number(value)))) throw new Error('Malformed dataset IMU row');
        // CSV columns: [timestamp_ns, gx, gy, gz, ax, ay, az]
        const ts = parseFloat(parts[0]) * 1e-9;
        const gx = parseFloat(parts[1]);
        const gy = parseFloat(parts[2]);
        const gz = parseFloat(parts[3]);
        const ax = parseFloat(parts[4]);
        const ay = parseFloat(parts[5]);
        const az = parseFloat(parts[6]);

        const base = idx * IMU_FIELDS;
        // WASM layout: [ts, ax, ay, az, gx, gy, gz]  (accel FIRST, gyro SECOND)
        data[base + 0] = ts;
        data[base + 1] = ax;
        data[base + 2] = ay;
        data[base + 3] = az;
        data[base + 4] = gx;
        data[base + 5] = gy;
        data[base + 6] = gz;

        timestamps[idx] = ts;
        idx++;
    }

    assertOrdered(timestamps, value => value);
    return { data, count: idx, timestamps };
}

/**
 * Parse mocap0/data.csv -> Array of { timestamp_s, position, quaternion }
 * CSV: "#timestamp [ns], px, py, pz, qw, qx, qy, qz"
 */
async function parseGroundTruth(url) {
    const text = await fetchCSV(url);
    const lines = text.trim().split('\n');
    const entries = [];
    for (const line of lines) {
        const trimmed = line.trim();
        if (trimmed.startsWith('#') || trimmed.length === 0) continue;
        const p = trimmed.split(',');
        if (p.length !== 8 || p.some(value => !Number.isFinite(Number(value)))) throw new Error('Malformed ground truth row');
        entries.push({
            timestamp_s: parseFloat(p[0]) * 1e-9,
            position: [parseFloat(p[1]), parseFloat(p[2]), parseFloat(p[3])],
            quaternion: [parseFloat(p[4]), parseFloat(p[5]), parseFloat(p[6]), parseFloat(p[7])],  // qw, qx, qy, qz
        });
    }
    assertOrdered(entries);
    return entries;
}

/**
 * Normalize GT trajectory: apply T_first_inv so it starts at origin with identity orientation.
 * This aligns GT to a "first-pose-as-origin" frame, similar to how VIO starts at origin.
 */
function normalizeGroundTruth(gtPoses) {
    if (gtPoses.length === 0) return gtPoses;

    // First pose: position and quaternion (qw, qx, qy, qz)
    const p0 = gtPoses[0].position;
    const q0 = gtPoses[0].quaternion; // [qw, qx, qy, qz]

    // Compute R0 from quaternion (qw, qx, qy, qz)
    const [qw, qx, qy, qz] = q0;
    // R0 = rotation matrix from quaternion
    const R0 = [
        1 - 2*(qy*qy + qz*qz),  2*(qx*qy - qz*qw),      2*(qx*qz + qy*qw),
        2*(qx*qy + qz*qw),      1 - 2*(qx*qx + qz*qz),  2*(qy*qz - qx*qw),
        2*(qx*qz - qy*qw),      2*(qy*qz + qx*qw),      1 - 2*(qx*qx + qy*qy),
    ];

    // R0_inv = R0^T (rotation matrix transpose)
    const R0_inv = [
        R0[0], R0[3], R0[6],
        R0[1], R0[4], R0[7],
        R0[2], R0[5], R0[8],
    ];

    // For each pose: p_aligned = R0^T * (p - p0)
    return gtPoses.map(pose => {
        const dx = pose.position[0] - p0[0];
        const dy = pose.position[1] - p0[1];
        const dz = pose.position[2] - p0[2];
        return {
            timestamp_s: pose.timestamp_s,
            position: [
                R0_inv[0]*dx + R0_inv[1]*dy + R0_inv[2]*dz,
                R0_inv[3]*dx + R0_inv[4]*dy + R0_inv[5]*dz,
                R0_inv[6]*dx + R0_inv[7]*dy + R0_inv[8]*dz,
            ],
        };
    });
}

// ─── IMU Slicing (Binary Search) ─────────────────────────────────────────────

/**
 * Find all IMU readings with timestamps in (tPrev, tCurr] via binary search.
 * Returns a NEW Float64Array (safe for Transferable) and count.
 */
function sliceIMUCursor(allIMU, imuTimestamps, imuCount, cursor, tCurr, bracket = true) {
    // Engine owns future carry. Never resend a previously submitted bracket or
    // consume another future sample while the previous bracket is still future.
    if (bracket && cursor > 0 && imuTimestamps[cursor - 1] > tCurr) return { data: null, count: 0, nextCursor: cursor };
    let end = cursor;
    while (end < imuCount && imuTimestamps[end] <= tCurr) end++;
    if (bracket && end < imuCount) end++;
    const count = end - cursor;
    return { data: count ? new Float64Array(allIMU.subarray(cursor * IMU_FIELDS, end * IMU_FIELDS)) : null, count, nextCursor: end };
}

// ─── Image Loader ────────────────────────────────────────────────────────────

/**
 * Load a PNG image, decode via canvas, extract 8-bit grayscale.
 * TUM VI images are 16-bit grayscale PNGs; canvas auto-converts to 8-bit RGBA.
 */
async function loadGrayscaleImage(url) {
    const response = await fetch(url);
    if (!response.ok) throw new Error(`Image load failed: ${url} (${response.status})`);
    const blob = await response.blob();
    const bitmap = await createImageBitmap(blob);
    if (bitmap.width !== IMAGE_WIDTH || bitmap.height !== IMAGE_HEIGHT) { bitmap.close(); throw new Error('Dataset image dimensions do not match calibration'); }

    let canvas, ctx;
    if (typeof OffscreenCanvas !== 'undefined') {
        canvas = new OffscreenCanvas(bitmap.width, bitmap.height);
        ctx = canvas.getContext('2d');
    } else {
        canvas = document.createElement('canvas');
        canvas.width = bitmap.width;
        canvas.height = bitmap.height;
        ctx = canvas.getContext('2d', { willReadFrequently: true });
    }

    ctx.drawImage(bitmap, 0, 0);
    bitmap.close();

    const rgba = ctx.getImageData(0, 0, canvas.width, canvas.height).data;
    const gray = new Uint8Array(canvas.width * canvas.height);
    // Extract R channel (for grayscale PNG, R == G == B)
    for (let i = 0, j = 0; i < rgba.length; i += 4, j++) {
        gray[j] = rgba[i];
    }
    return gray;
}

/**
 * On-demand image loader with small prefetch window to avoid stalls.
 */
class ImagePrefetcher {
    constructor(imageList, baseUrl, windowSize = 10) {
        this.imageList = imageList;
        this.baseUrl = baseUrl;
        this.windowSize = windowSize;
        this.cache = new Map(); // index -> Promise<Uint8Array>
    }

    async get(index) {
        if (index < 0 || index >= this.imageList.length) {
            throw new Error(`Image index out of range: ${index}`);
        }

        // Prefetch ahead
        const end = Math.min(index + this.windowSize, this.imageList.length);
        for (let i = index; i < end; i++) {
            if (!this.cache.has(i)) {
                const url = `${this.baseUrl}/${this.imageList[i].filename}`;
                const idx = i;
                const p = loadGrayscaleImage(url).catch(err => {
                    this.cache.delete(idx); // evict on failure so next access retries
                    throw err;
                });
                // Mark speculative prefetch rejection handled; get(index) still
                // observes that rejection and fails the replay at the real input.
                p.catch(() => {});
                this.cache.set(i, p);
            }
        }

        // Evict old entries
        const evictBefore = index - this.windowSize * 2;
        if (evictBefore > 0) {
            for (const key of this.cache.keys()) {
                if (key < evictBefore) this.cache.delete(key);
            }
        }

        return this.cache.get(index);
    }

    clear() {
        this.cache.clear();
    }
}

// ─── Main Application ────────────────────────────────────────────────────────

class TUMVITestApp {
    constructor() {
        this.vio = new VIOWrapper();
        this.renderer = null;

        // Dataset
        this.imageList = [];
        this.imuData = null;
        this.imuTimestamps = null;
        this.imuCount = 0;
        this.groundTruth = null;
        this.prefetcher = null;

        // Playback state
        this.currentFrame = 0;
        this.playing = false;
        this.paused = false;
        this.speed = 1;
        this.playbackTimer = null;
        this.playbackTimerType = null; // 'raf' | 'timeout'
        this.processing = false;
        this.datasetName = REPLAY_PARAMS.get('dataset') || 'room1';
        this.datasetBase = datasetBase(this.datasetName);
        this.frameLimit = Infinity;
        this._imuCursor = 0;
        this.inputHashEnabled = REPLAY_PARAMS.get('inputHash') === '1';
        const benchmarkProfile = REPLAY_PARAMS.get('benchmarkZeroTolerances');
        if (benchmarkProfile !== null && !['0', '1'].includes(benchmarkProfile)) throw new Error('Invalid benchmarkZeroTolerances');
        this.benchmarkSolverProfileRequested = benchmarkProfile === '1';
        this._runGeneration = 0;
        this._nextScheduledFrame = 0;
        this.rows = [];
        this.replay = { completed: false, failed: false, startedAtMs: null, finishedAtMs: null, mode: null };
        this._renderResultKey = null;
        this._renderEpoch = null;
        this.solverConfig = { ...SOLVER_CONFIG };
        for (const [param, field] of [['solverTime', 'solver_time'], ['iterations', 'num_iterations'], ['features', 'max_features']]) {
            if (REPLAY_PARAMS.has(param)) {
                const value = Number(REPLAY_PARAMS.get(param));
                if (!Number.isFinite(value) || value <= 0 || (field !== 'solver_time' && !Number.isInteger(value))) throw new Error(`Invalid ${param}`);
                this.solverConfig[field] = value;
            }
        }

        // Diagnostics
        this.frameProcessingTime = 0;
        this.lastIMUSliceCount = 0;

        // UI
        this.ui = {};
        this.imageCtx = null;
        this._gtLine = null;

        // Reusable ImageData for preview (avoids 1MB allocation per frame)
        this._previewImageData = null;
    }

    async initialize() {
        this.ui = {
            startBtn:     document.getElementById('btn-start'),
            pauseBtn:     document.getElementById('btn-pause'),
            stepBtn:      document.getElementById('btn-step'),
            resetBtn:     document.getElementById('btn-reset'),
            speedSelect:  document.getElementById('speed-select'),
            imageCanvas:  document.getElementById('image-preview'),
            progressFill: document.getElementById('progress-fill'),
            progressText: document.getElementById('progress-text'),
            status:       document.getElementById('status'),
            logPanel:     document.getElementById('log-panel'),
            diagFrame:    document.getElementById('diag-frame'),
            diagStatus:   document.getElementById('diag-status'),
            diagFeatures: document.getElementById('diag-features'),
            diagProcTime: document.getElementById('diag-proc-time'),
            diagIMUCount: document.getElementById('diag-imu-count'),
            diagPoseX:    document.getElementById('diag-pose-x'),
            diagPoseY:    document.getElementById('diag-pose-y'),
            diagPoseZ:    document.getElementById('diag-pose-z'),
        };

        this.imageCtx = this.ui.imageCanvas.getContext('2d', { willReadFrequently: true });

        // 3D renderer (supports WebGL/WebGPU backend selection)
        const canvas3d = document.getElementById('canvas-3d');
        if (canvas3d) {
            const container = canvas3d.parentElement;
            canvas3d.width = container.clientWidth || 800;
            canvas3d.height = container.clientHeight || 600;

            const backendSelect = document.getElementById('renderer-backend');
            const backend = backendSelect ? backendSelect.value : 'webgl';
            this.renderer = await Renderer.create(canvas3d, backend);
            this.renderer.render();
            this.log(`3D Renderer: ${this.renderer.backendName.toUpperCase()} backend`);

            window.addEventListener('resize', () => {
                const c = canvas3d.parentElement;
                canvas3d.width = c.clientWidth;
                canvas3d.height = c.clientHeight;
                this.renderer.resize(c.clientWidth, c.clientHeight);
            });
        }

        const datasetSelect = document.getElementById('dataset-select');
        if (datasetSelect) {
            datasetSelect.value = this.datasetName;
            datasetSelect.addEventListener('change', event => {
                if (this.playing) return;
                this.datasetName = event.target.value;
                this.datasetBase = datasetBase(this.datasetName);
            });
        }
        document.getElementById('btn-export')?.addEventListener('click', () => this.downloadReport());

        // Button events
        this.ui.startBtn.addEventListener('click', () => this.start());
        this.ui.pauseBtn.addEventListener('click', () => this.togglePause());
        this.ui.stepBtn.addEventListener('click', () => this.stepOneFrame());
        this.ui.resetBtn.addEventListener('click', () => this.reset());
        this.ui.speedSelect.addEventListener('change', (e) => {
            this.speed = Number(e.target.value);
            if (this.playing && !this.paused) {
                this.stopPlaybackTimer();
                this.startPlaybackTimer();
            }
        });

        // Follow camera toggle
        const followCamCheckbox = document.getElementById('follow-cam');
        if (followCamCheckbox && this.renderer) {
            followCamCheckbox.addEventListener('change', (e) => {
                this.renderer.followCamera = e.target.checked;
                // Enable/disable orbit controls based on follow state
                this.renderer.controls.enabled = !e.target.checked;
            });
            // Initial state: follow on, orbit disabled
            this.renderer.controls.enabled = false;
        }

        this.setStatus('Loading WASM module...');
        this.log('Initializing WASM VIO engine...');

        try {
            const urlParams = new URLSearchParams(window.location.search);
            const wasmFile = urlParams.get('wasm') || 'vio_engine.js';
            this.log(`Loading WASM: ${wasmFile}`);
            await this.vio.load('/' + wasmFile);

            // Forward C++ stdout/stderr to log, filtering Ceres miniglog noise
            const _ceresNoise = /detect_structure|block_sparse_matrix|schur_eliminator|callbacks\.cc|trust_region_minimizer|Schur complement|Dynamic .* block size|Allocating values array|Terminating:/;
            this.vio.onWasmLog = (level, msg) => {
                if (msg && msg.trim().length > 0 && !_ceresNoise.test(msg)) {
                    this.log(`[WASM] ${msg}`, level);
                }
            };

            this.setStatus('WASM loaded. Click Start to begin.');
            this.log('WASM module loaded successfully.');
            this.ui.startBtn.disabled = false;
        } catch (err) {
            this.setStatus(`Failed to load WASM: ${err.message}`);
            this.log(`ERROR: ${err.message}`, 'error');
        }
    }

    async start() {
        this.ui.startBtn.disabled = true;
        this.setStatus(`Loading ${this.datasetName}...`);
        this.log('Fetching TUM VI dataset CSV files...');

        try {
            // Parse CSV files in parallel
            const [imageList, imuResult, groundTruth] = await Promise.all([
                parseImageList(`${this.datasetBase}/cam0/data.csv`),
                parseIMUData(`${this.datasetBase}/imu0/data.csv`),
                parseGroundTruth(`${this.datasetBase}/mocap0/data.csv`).catch(() => null),
            ]);

            this.imageList = imageList;
            this.imuData = imuResult.data;
            this.imuTimestamps = imuResult.timestamps;
            this.imuCount = imuResult.count;
            this.groundTruth = groundTruth;

            if (imageList.length === 0) throw new Error('No images found in cam0/data.csv');
            if (imuResult.count === 0) throw new Error('No IMU data found in imu0/data.csv');

            this.log(`Images: ${imageList.length} frames`);
            this.log(`IMU: ${imuResult.count} readings`);
            if (groundTruth) this.log(`Ground truth: ${groundTruth.length} poses`);

            // Log timestamp ranges for debugging
            this.log(`Image time range: ${imageList[0].timestamp_s.toFixed(3)}s - ${imageList[imageList.length - 1].timestamp_s.toFixed(3)}s`);
            this.log(`IMU time range: ${imuResult.timestamps[0].toFixed(3)}s - ${imuResult.timestamps[imuResult.count - 1].toFixed(3)}s`);

            // Verify first IMU reading (accel should be ~(-0.35, 0.00, 9.92), gyro ~(0.10, -0.08, 0.02))
            this.log(`First IMU [ax,ay,az,gx,gy,gz]: [${imuResult.data[1].toFixed(3)}, ${imuResult.data[2].toFixed(3)}, ${imuResult.data[3].toFixed(3)}, ${imuResult.data[4].toFixed(3)}, ${imuResult.data[5].toFixed(3)}, ${imuResult.data[6].toFixed(3)}]`);

            this.setStatus(`Dataset loaded. Configuring VIO...`);

            // Image prefetcher
            this.prefetcher = new ImagePrefetcher(imageList, `${this.datasetBase}/cam0/data`, 10);

            // Configure VIO
            this.log('Configuring VIO engine with TUM VI calibration...');
            const configured = await this.vio.configure({ ...TUM_VI_CONFIG,
                benchmark_zero_positive_tolerances: this.benchmarkSolverProfileRequested });
            if (!configured) {
                this.setStatus('VIO configuration FAILED');
                this.log('VIO configure() returned false!', 'error');
                this.ui.startBtn.disabled = false;
                return;
            }
            this.log('VIO configured successfully.');

            // Set solver params
            await this.vio.setMobileParams(
                this.solverConfig.solver_time,
                this.solverConfig.num_iterations,
                this.solverConfig.max_features
            );
            this.log(`Solver: time=${this.solverConfig.solver_time}s, iter=${this.solverConfig.num_iterations}, features=${this.solverConfig.max_features}`);

            // TUM VI tracking params: 512x512 calibrated fisheye needs different
            // settings from the mobile defaults hardcoded in configure().
            await this.vio.setTrackingParams(
                21,    // lk_window: default
                3,     // lk_pyramid: 3 levels for 512x512
                20,    // min_dist: 20px for good feature density at 512x512
                0.0    // f_edge_factor: disabled (calibrated fisheye, no unmodeled distortion)
            );
            await this.vio.setFThreshold(1.0);
            await this.vio.setPnPParams(false, 3);
            this.log('Tracking: LK21/3, min_dist20, F1; PnP disabled');

            // Ground truth visualization
            if (this.groundTruth && this.renderer) {
                this.addGroundTruthToRenderer(this.groundTruth);
            }

            // Exact bound belongs to the app: a runner does not race Pause.
            const requestedFrames = REPLAY_PARAMS.has('frames') ? Number(REPLAY_PARAMS.get('frames')) : imageList.length;
            if (!Number.isInteger(requestedFrames) || requestedFrames < 1 || requestedFrames > imageList.length) throw new Error('frames must be an exact count within the dataset');
            this.frameLimit = requestedFrames;
            this._runGeneration++;
            this._imuCursor = 0;
            this._nextScheduledFrame = 0;
            this.rows = [];
            this.replay = { completed: false, failed: false, startedAtMs: performance.now(), finishedAtMs: null, mode: null };
            this.currentFrame = 0;
            this.playing = true;
            this.paused = false;

            this.ui.pauseBtn.disabled = false;
            this.ui.stepBtn.disabled = false;
            this.ui.resetBtn.disabled = false;
            this.updateProgress();

            this.speed = REPLAY_PARAMS.has('speed') ? Number(REPLAY_PARAMS.get('speed')) : Number(this.ui.speedSelect.value);
            if (![-1,0,1,2,5].includes(this.speed)) throw new Error('Invalid replay speed');
            this.ui.speedSelect.value = this.speed;
            if (this.speed === 0) {
                this.setStatus('Step mode: click Step to advance.');
            } else {
                this.setStatus('Processing...');
                this.startPlaybackTimer();
            }

        } catch (err) {
            this.setStatus(`Error: ${err.message}`);
            this.log(`ERROR: ${err.message}`, 'error');
            console.error(err);
            this.ui.startBtn.disabled = false;
        }
    }

    /**
     * Process one frame: load image, slice IMU, send to worker, update UI.
     */
    _feedIMU(frameIdx) {
        const slice = sliceIMUCursor(this.imuData, this.imuTimestamps, this.imuCount, this._imuCursor, this.imageList[frameIdx].timestamp_s, true);
        const input = { count: slice.count, imuStartIndex: this._imuCursor, imuEndIndexExclusive: slice.nextCursor,
            imuFirstTimestamp: slice.count ? slice.data[0] : null, imuLastTimestamp: slice.count ? slice.data[(slice.count - 1) * IMU_FIELDS] : null };
        this._imuCursor = slice.nextCursor;
        if (slice.count > 0 && this.vio.sendIMU(slice.data, slice.count) === false) throw new Error('Worker rejected replay IMU');
        return input;
    }

    _finishReplay() {
        if (this.processing || this.currentFrame < this.frameLimit) return;
        this.stopPlaybackTimer();
        this.playing = false;
        this.replay.completed = !this.replay.failed;
        this.replay.finishedAtMs = performance.now();
        const report = this.exportReport();
        this.setStatus(`${this.replay.failed ? 'Failed' : 'Complete'}. ${report.counts.completed} completed / ${report.counts.dropped} dropped / ${this.frameLimit} input (${this.replay.mode}).`);
    }

    async processNextFrame(scheduledFrame = null, deadlineMs = null) {
        if (scheduledFrame === null && this.processing) return;
        const frameIdx = scheduledFrame ?? this.currentFrame;
        if (frameIdx >= this.frameLimit || frameIdx >= this.imageList.length) { this._finishReplay(); return; }
        const generation = this._runGeneration;
        const frameEntry = this.imageList[frameIdx];
        const t0 = performance.now();
        const row = { frame: frameIdx, timestamp: frameEntry.timestamp_s, inputTimestamp: frameEntry.timestamp_s,
            poseTimestamp: null, pose: null, accepted: false, completed: false, dropped: false,
            poseFresh: false, poseValid: false, deadlineMs, arrivedAtMs: t0,
            schedulingLatenessMs: deadlineMs === null ? null : t0 - deadlineMs };
        try {
            const { count: imuCount, ...input } = this._feedIMU(frameIdx);
            Object.assign(row, input, { submittedIMUCount: imuCount });
            if (this.processing || this.vio.workerBusy) {
                if (scheduledFrame === null) throw new Error('Unexpected busy worker in serial replay');
                row.dropped = true; row.reason = 'paced_worker_busy';
                this.rows.push(row);
                this.currentFrame = Math.max(this.currentFrame, frameIdx + 1);
                this.updateProgress();
                return;
            }
            this.processing = true;
            const decodeStart = performance.now();
            const gray = await this.prefetcher.get(frameIdx);
            if (generation !== this._runGeneration || !this.playing) return;
            row.decodeMs = performance.now() - decodeStart;
            row.grayBytes = gray.byteLength;
            if (this.inputHashEnabled) {
                const hashStart = performance.now();
                const digest = await crypto.subtle.digest('SHA-256', gray);
                row.decodedGraySha256 = Array.from(new Uint8Array(digest), byte => byte.toString(16).padStart(2, '0')).join('');
                row.inputHashMs = performance.now() - hashStart;
                if (generation !== this._runGeneration || !this.playing) return;
            }
            this.displayImagePreview(gray);
            this.lastIMUSliceCount = imuCount;
            row.submittedAtMs = performance.now();
            row.accepted = this.vio.sendFrame(gray, frameEntry.timestamp_s);
            if (!row.accepted) throw new Error('Worker rejected replay frame');
            await this.vio.waitForFree(10000);
            if (generation !== this._runGeneration || !this.playing) return;
            const result = this.vio.getLatestResult();
            if (!result || result.inputTimestamp !== frameEntry.timestamp_s) throw new Error('Replay result timestamp mismatch');
            const { pose, mapPoints: _mapPoints, ...diagnostics } = result;
            Object.assign(row, diagnostics, { pose: pose ? Array.from(pose) : null, completed: true,
                receivedAtMs: performance.now(), totalProcessingMs: performance.now() - t0 });
            row.deadlineToResultMs = deadlineMs === null ? null : row.receivedAtMs - deadlineMs;
            this.frameProcessingTime = row.totalProcessingMs;
            this.rows.push(row);
            this.currentFrame = Math.max(this.currentFrame, frameIdx + 1);
            this.updateDiagnostics();
            this.updateRenderer();
            this.updateProgress();
            if (frameIdx === 0 || frameIdx % 100 === 0 || this._lastStatus !== result.statusCode) this.log(`Frame ${frameIdx}: status=${result.statusCode}/${result.reason}, IMU=${imuCount}, fresh=${result.poseFresh}, time=${row.totalProcessingMs.toFixed(1)}ms`);
            this._lastStatus = result.statusCode;
        } catch (error) {
            if (generation !== this._runGeneration) return;
            row.reason = error.message;
            row.error = true;
            this.rows.push(row);
            this.replay.failed = true;
            this.replay.finishedAtMs = performance.now();
            this.playing = false;
            this.stopPlaybackTimer();
            this.setStatus(`Replay failed at frame ${frameIdx}: ${error.message}`);
            this.log(error.message, 'error');
        } finally {
            // A busy paced arrival must not release another frame's busy guard.
            if (generation === this._runGeneration && !row.dropped) this.processing = false;
            if (generation === this._runGeneration) this._finishReplay();
        }
    }

    exportReport() {
        const rows = [...this.rows].sort((a,b) => a.frame - b.frame);
        const appliedProfile = rows.findLast(row => typeof row.benchmarkSolverProfile === 'boolean')?.benchmarkSolverProfile ?? null;
        return { schema: 'mobile-slam-replay-v2', dataset: this.datasetName, datasetBase: this.datasetBase,
            requestedFrames: this.frameLimit, datasetFrames: this.imageList.length,
            inputIMUReadings: this.imuCount, deliveredIMUReadings: this._imuCursor, inputHashEnabled: this.inputHashEnabled,
            calibration: { ...TUM_VI_CONFIG }, solver: { ...this.solverConfig }, tracking: { lk_window: 21, lk_pyramid: 3, lk_criteria_count: 20, lk_criteria_eps: 0.03, min_dist: 20, f_edge_factor: 0, f_threshold: 1, pnp: false, freq: 3 },
            benchmarkSolverProfile: { requested: this.benchmarkSolverProfileRequested, actual: appliedProfile,
                profileName: appliedProfile === true ? 'max10_zero_positive_tolerances' : appliedProfile === false ? 'default' : 'unavailable',
                actualOptions: null, actualOptionsSource: 'Ordinary replay does not capture solver options; use bounded engine diagnostic capture.' },
            inputPolicy: 'ordered cursor; first frame includes pre-image readings; first future bracket delivered once; engine owns carry',
            poseFrame: 'camera T_W_C row-major', poseTimestampSource: 'actual engine poseTimestamp; invalid/fresh=false excluded',
            clock: { dataset: 'CSV timestamp seconds', browser: 'performance.timeOrigin relative ms', timeOriginMs: performance.timeOrigin },
            speed: this.speed, ...this.replay, counts: { input: rows.length, accepted: rows.filter(r => r.accepted).length,
                completed: rows.filter(r => r.completed).length, dropped: rows.filter(r => r.dropped).length,
                errors: rows.filter(r => r.error).length, freshPoses: rows.filter(r => r.poseFresh && r.poseValid && r.pose).length },
            transport: this.vio.getMetrics?.() ?? null, rows,
            limits: ['Dataset/desktop replay; physical phone sensors, exposure time, thermal and field accuracy unverified.', 'Paced deadline-to-result is a dataset scheduling proxy, not physical camera latency.'] };
    }

    downloadReport() {
        const url = URL.createObjectURL(new Blob([JSON.stringify(this.exportReport(), null, 2)], { type: 'application/json' }));
        const link = document.createElement('a'); link.href = url; link.download = `mobile-slam-${this.datasetName}-replay.json`; link.click();
        setTimeout(() => URL.revokeObjectURL(url), 1000);
    }

    /** Draw grayscale image on preview canvas (reuses ImageData to avoid 1MB alloc per frame) */
    displayImagePreview(gray) {
        const w = IMAGE_WIDTH, h = IMAGE_HEIGHT;
        if (!this._previewImageData) {
            this._previewImageData = this.imageCtx.createImageData(w, h);
        }
        const rgba = this._previewImageData.data;
        for (let i = 0, j = 0; i < gray.length; i++, j += 4) {
            rgba[j] = rgba[j + 1] = rgba[j + 2] = gray[i];
            rgba[j + 3] = 255;
        }
        this.imageCtx.putImageData(this._previewImageData, 0, 0);
    }

    /** Update diagnostics panel */
    updateDiagnostics() {
        const result = this.vio.getLatestResult();
        const ui = this.ui;

        if (ui.diagFrame) ui.diagFrame.textContent = `${this.currentFrame} / ${this.imageList.length}`;
        if (ui.diagProcTime) ui.diagProcTime.textContent = `${this.frameProcessingTime.toFixed(1)} ms`;
        if (ui.diagIMUCount) ui.diagIMUCount.textContent = `${this.lastIMUSliceCount}`;

        if (result) {
            const statusNames = ['NOT_CONFIGURED', 'INITIALIZING', 'TRACKING', 'LOST', 'COOLDOWN'];
            const statusName = statusNames[result.statusCode] || 'UNKNOWN';

            if (ui.diagStatus) {
                ui.diagStatus.textContent = statusName;
                ui.diagStatus.className = 'diag-value';
                if (result.statusCode === 2) ui.diagStatus.classList.add('tracking');
                else if (result.statusCode === 1) ui.diagStatus.classList.add('initializing');
                else if (result.statusCode === 3) ui.diagStatus.classList.add('lost');
            }
            if (ui.diagFeatures) ui.diagFeatures.textContent = `${result.featureCount}`;

            if (result.pose) {
                // Row-major 4x4: translation at indices [3], [7], [11]
                const x = result.pose[3], y = result.pose[7], z = result.pose[11];
                if (ui.diagPoseX) ui.diagPoseX.textContent = isNaN(x) ? 'NaN!' : x.toFixed(4);
                if (ui.diagPoseY) ui.diagPoseY.textContent = isNaN(y) ? 'NaN!' : y.toFixed(4);
                if (ui.diagPoseZ) ui.diagPoseZ.textContent = isNaN(z) ? 'NaN!' : z.toFixed(4);
            }
        }
    }

    /** Render each engine observation once; epoch/loss invalidates the display. */
    updateRenderer() {
        if (!this.renderer) return;
        const result = this.vio.getLatestResult();
        const epoch = `${result?.clientEpoch}:${result?.engineEpoch}`;
        const key = `${epoch}:${result?.sequence}`;
        if (result && key !== this._renderResultKey) {
            if (this._renderEpoch !== null && epoch !== this._renderEpoch) this.renderer.clear();
            this._renderEpoch = epoch;
            this._renderResultKey = key;
            if (result.poseFresh && result.poseValid && Number.isFinite(result.poseTimestamp) && result.pose) {
                this.renderer.updateCameraPose(result.pose);
                const map = this.vio.getMapPoints(); this.renderer.updateMapPoints(map.points, map.count);
            } else this.renderer.clear();
        }
        this.renderer.render();
    }

    /** Update progress bar */
    updateProgress() {
        const total = Number.isFinite(this.frameLimit) ? this.frameLimit : this.imageList.length || 1;
        const pct = (this.currentFrame / total) * 100;
        if (this.ui.progressFill) this.ui.progressFill.style.width = `${pct}%`;
        if (this.ui.progressText) this.ui.progressText.textContent = `${this.currentFrame} / ${total}`;
    }

    /** Add ground truth trajectory to 3D scene (blue line) */
    addGroundTruthToRenderer(gtPoses) {
        // Normalize GT: first pose becomes origin with identity orientation
        const normalized = normalizeGroundTruth(gtPoses);

        const positions = [];
        // Subsample for rendering performance
        const step = Math.max(1, Math.floor(normalized.length / 2000));
        for (let i = 0; i < normalized.length; i += step) {
            const p = normalized[i].position;
            positions.push(p[0], p[1], p[2]);
        }

        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        const material = new THREE.LineBasicMaterial({ color: 0x4488ff, linewidth: 2 });
        const line = new THREE.Line(geometry, material);
        this.renderer.worldRoot.add(line);
        this._gtLine = line;
        this.log(`Ground truth: ${positions.length / 3} points rendered (blue), normalized to origin.`);
    }

    // ─── Playback Controls ───────────────────────────────────────────────────

    startPlaybackTimer() {
        this.stopPlaybackTimer();
        if (this.speed === -1) {
            this.replay.mode = 'max_speed_serial';
            this.playbackTimerType = 'timeout';
            const generation = this._runGeneration;
            const loop = async () => {
                if (!this.playing || this.paused || generation !== this._runGeneration) return;
                if (this.processing) { this.playbackTimer = setTimeout(loop, 0); return; }
                await this.processNextFrame();
                if (this.playing && !this.paused && generation === this._runGeneration) this.playbackTimer = setTimeout(loop, 0);
            };
            this.playbackTimer = setTimeout(loop, 0);
        } else if (this.speed > 0) {
            this.replay.mode = 'paced_absolute_deadline_drop_busy';
            this.playbackTimerType = 'timeout';
            this._nextScheduledFrame = Math.max(this._nextScheduledFrame, this.currentFrame);
            const first = this.imageList[this._nextScheduledFrame]?.timestamp_s;
            const wallOriginMs = performance.now();
            const generation = this._runGeneration;
            const tick = () => {
                if (!this.playing || this.paused || generation !== this._runGeneration) return;
                const index = this._nextScheduledFrame++;
                if (index >= this.frameLimit) { this._finishReplay(); return; }
                const deadline = replayDeadline(wallOriginMs, first, this.imageList[index].timestamp_s, this.speed);
                void this.processNextFrame(index, deadline);
                if (this._nextScheduledFrame < this.frameLimit) {
                    const next = replayDeadline(wallOriginMs, first, this.imageList[this._nextScheduledFrame].timestamp_s, this.speed);
                    this.playbackTimer = setTimeout(tick, Math.max(0, next - performance.now()));
                }
            };
            this.playbackTimer = setTimeout(tick, 0);
        } else this.replay.mode = 'manual_step';
    }

    stopPlaybackTimer() {
        if (this.playbackTimer !== null) {
            if (this.playbackTimerType === 'raf') {
                cancelAnimationFrame(this.playbackTimer);
            } else {
                clearTimeout(this.playbackTimer);
            }
            this.playbackTimer = null;
            this.playbackTimerType = null;
        }
    }

    togglePause() {
        if (this.paused) {
            this.paused = false;
            this.ui.pauseBtn.textContent = 'Pause';
            this.setStatus('Processing...');
            if (this.speed !== 0) this.startPlaybackTimer();
        } else {
            this.paused = true;
            this.ui.pauseBtn.textContent = 'Resume';
            this.setStatus(`Paused at frame ${this.currentFrame}.`);
            this.stopPlaybackTimer();
        }
    }

    async stepOneFrame() {
        if (!this.playing) return;
        this.paused = true;
        this.ui.pauseBtn.textContent = 'Resume';
        this.stopPlaybackTimer();
        this.setStatus('Stepping...');
        await this.processNextFrame();
        this.setStatus(`Step complete. Frame ${this.currentFrame}.`);
    }

    async reset() {
        this.stopPlaybackTimer();
        this.playing = false;
        this.paused = false;
        this._runGeneration++;
        this.currentFrame = 0;
        this._nextScheduledFrame = 0;
        this._imuCursor = 0;
        this.rows = [];
        this.processing = false;
        this._renderResultKey = null;
        this._renderEpoch = null;

        this.vio.reset();
        if (this.renderer) {
            this.renderer.clear();
            if (this._gtLine) {
                this.renderer.worldRoot.remove(this._gtLine);
                this._gtLine = null;
            }
        }
        if (this.prefetcher) this.prefetcher.clear();

        this.ui.startBtn.disabled = false;
        this.ui.pauseBtn.disabled = true;
        this.ui.stepBtn.disabled = true;
        this.ui.pauseBtn.textContent = 'Pause';
        this.updateProgress();
        this.setStatus('Reset. Click Start to begin.');
        this.log('--- Reset ---');
    }

    // ─── Logging ─────────────────────────────────────────────────────────────

    setStatus(msg) {
        if (this.ui.status) this.ui.status.textContent = msg;
    }

    log(msg, level = 'info') {
        console.log(`[TUM-VI] ${msg}`);
        if (this.ui.logPanel) {
            const entry = document.createElement('div');
            entry.className = 'log-entry';
            if (level === 'warn') entry.classList.add('log-warn');
            if (level === 'error') entry.classList.add('log-error');
            entry.textContent = `[${new Date().toLocaleTimeString()}] ${msg}`;
            this.ui.logPanel.appendChild(entry);
            this.ui.logPanel.scrollTop = this.ui.logPanel.scrollHeight;
        }
        // Console mirror for developer tools
        if (level === 'error') console.error(`[TUM-VI] ${msg}`);
        else if (level === 'warn') console.warn(`[TUM-VI] ${msg}`);
    }
}

// ─── Entry Point ─────────────────────────────────────────────────────────────

const app = new TUMVITestApp();
window.__tumviReplay = { app, export: () => app.exportReport() };
document.addEventListener('DOMContentLoaded', () => app.initialize());
