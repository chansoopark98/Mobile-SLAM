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

const DATASET_BASE = '/datasets/tum/dataset-room1_512_16/mav0';
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
    modelType: 1,             // KANNALA_BRANDT (equidistant)
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
    // Extrinsic translation t_imu_cam (from tum_vi_room1.yaml)
    t_ic: [0.04517590, 0.07251590, -0.04395990],
    // IMU noise parameters
    acc_n: 0.0028,
    acc_w: 0.00086,
    gyr_n: 0.00016,
    gyr_w: 0.000022,
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
    const text = await (await fetch(url)).text();
    const lines = text.trim().split('\n');
    const entries = [];
    for (const line of lines) {
        const trimmed = line.trim();
        if (trimmed.startsWith('#') || trimmed.length === 0) continue;
        const comma = trimmed.indexOf(',');
        entries.push({
            timestamp_s: parseFloat(trimmed.substring(0, comma)) * 1e-9,
            filename: trimmed.substring(comma + 1).trim(),
        });
    }
    entries.sort((a, b) => a.timestamp_s - b.timestamp_s);
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
    const text = await (await fetch(url)).text();
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
        if (parts.length < 7) continue; // skip malformed lines
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

    return { data, count: idx, timestamps };
}

/**
 * Parse mocap0/data.csv -> Array of { timestamp_s, position, quaternion }
 * CSV: "#timestamp [ns], px, py, pz, qw, qx, qy, qz"
 */
async function parseGroundTruth(url) {
    const text = await (await fetch(url)).text();
    const lines = text.trim().split('\n');
    const entries = [];
    for (const line of lines) {
        const trimmed = line.trim();
        if (trimmed.startsWith('#') || trimmed.length === 0) continue;
        const p = trimmed.split(',');
        entries.push({
            timestamp_s: parseFloat(p[0]) * 1e-9,
            position: [parseFloat(p[1]), parseFloat(p[2]), parseFloat(p[3])],
            quaternion: [parseFloat(p[4]), parseFloat(p[5]), parseFloat(p[6]), parseFloat(p[7])],  // qw, qx, qy, qz
        });
    }
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
function sliceIMU(allIMU, imuTimestamps, imuCount, tPrev, tCurr) {
    // Binary search: first index where timestamp > tPrev
    let lo = 0, hi = imuCount;
    while (lo < hi) {
        const mid = (lo + hi) >>> 1;
        if (imuTimestamps[mid] <= tPrev) lo = mid + 1;
        else hi = mid;
    }
    const startIdx = lo;

    // Binary search: first index where timestamp > tCurr
    lo = startIdx;
    hi = imuCount;
    while (lo < hi) {
        const mid = (lo + hi) >>> 1;
        if (imuTimestamps[mid] <= tCurr) lo = mid + 1;
        else hi = mid;
    }
    const endIdx = lo;

    const count = endIdx - startIdx;
    if (count <= 0) return { data: null, count: 0 };

    // Create NEW Float64Array (will be transferred/detached by sendIMU)
    const slice = new Float64Array(count * IMU_FIELDS);
    slice.set(allIMU.subarray(startIdx * IMU_FIELDS, endIdx * IMU_FIELDS));

    return { data: slice, count };
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
        this.processing = false; // guard against concurrent processNextFrame

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
            this.log(`3D Renderer: ${this.renderer.backendName.toUpperCase()} backend`);

            window.addEventListener('resize', () => {
                const c = canvas3d.parentElement;
                canvas3d.width = c.clientWidth;
                canvas3d.height = c.clientHeight;
                this.renderer.resize(c.clientWidth, c.clientHeight);
            });
        }

        // Button events
        this.ui.startBtn.addEventListener('click', () => this.start());
        this.ui.pauseBtn.addEventListener('click', () => this.togglePause());
        this.ui.stepBtn.addEventListener('click', () => this.stepOneFrame());
        this.ui.resetBtn.addEventListener('click', () => this.reset());
        this.ui.speedSelect.addEventListener('change', (e) => {
            this.speed = parseInt(e.target.value);
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
            await this.vio.load('/vio_engine.js');

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
        this.setStatus('Loading dataset...');
        this.log('Fetching TUM VI dataset CSV files...');

        try {
            // Parse CSV files in parallel
            const [imageList, imuResult, groundTruth] = await Promise.all([
                parseImageList(`${DATASET_BASE}/cam0/data.csv`),
                parseIMUData(`${DATASET_BASE}/imu0/data.csv`),
                parseGroundTruth(`${DATASET_BASE}/mocap0/data.csv`).catch(() => null),
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
            this.prefetcher = new ImagePrefetcher(imageList, `${DATASET_BASE}/cam0/data`, 10);

            // Configure VIO
            this.log('Configuring VIO engine with TUM VI calibration...');
            const configured = await this.vio.configure(TUM_VI_CONFIG);
            if (!configured) {
                this.setStatus('VIO configuration FAILED');
                this.log('VIO configure() returned false!', 'error');
                this.ui.startBtn.disabled = false;
                return;
            }
            this.log('VIO configured successfully.');

            // Set solver params
            await this.vio.setMobileParams(
                SOLVER_CONFIG.solver_time,
                SOLVER_CONFIG.num_iterations,
                SOLVER_CONFIG.max_features
            );
            this.log(`Solver: time=${SOLVER_CONFIG.solver_time}s, iter=${SOLVER_CONFIG.num_iterations}, features=${SOLVER_CONFIG.max_features}`);

            // Ground truth visualization
            if (this.groundTruth && this.renderer) {
                this.addGroundTruthToRenderer(this.groundTruth);
            }

            // Start playback
            this.currentFrame = 0;
            this.playing = true;
            this.paused = false;

            this.ui.pauseBtn.disabled = false;
            this.ui.stepBtn.disabled = false;
            this.ui.resetBtn.disabled = false;
            this.updateProgress();

            this.speed = parseInt(this.ui.speedSelect.value);
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
    async processNextFrame() {
        if (this.processing) return; // guard re-entry
        if (this.currentFrame >= this.imageList.length) {
            this.stopPlaybackTimer();
            this.playing = false;
            this.setStatus(`Complete. ${this.currentFrame} frames processed.`);
            this.log('Dataset playback complete.');
            return;
        }

        this.processing = true;
        const frameIdx = this.currentFrame;
        const frameEntry = this.imageList[frameIdx];

        // Previous frame timestamp (for IMU slicing)
        // For the first frame, use just before the first IMU reading to capture all pre-image IMU data
        const prevTimestamp = frameIdx > 0
            ? this.imageList[frameIdx - 1].timestamp_s
            : this.imuTimestamps[0] - 1e-9;

        const t0 = performance.now();

        try {
            // 1. Load image from prefetch cache
            const gray = await this.prefetcher.get(frameIdx);

            // 2. Display on preview canvas
            this.displayImagePreview(gray);

            // 3. Slice IMU data for this inter-frame interval
            const imuSlice = sliceIMU(
                this.imuData, this.imuTimestamps, this.imuCount,
                prevTimestamp, frameEntry.timestamp_s
            );
            this.lastIMUSliceCount = imuSlice.count;

            // 4. Send IMU (always goes through, not blocked by busy)
            if (imuSlice.count > 0) {
                this.vio.sendIMU(imuSlice.data, imuSlice.count);
            }

            // 5. Wait for worker to be free, then send frame
            await this.vio.waitForFree(5000);
            const sent = this.vio.sendFrame(gray, frameEntry.timestamp_s);

            if (!sent) {
                this.log(`Frame ${frameIdx} dropped (worker busy)`, 'warn');
                this.processing = false;
                return;
            }

            // 6. Wait for processing to finish
            await this.vio.waitForFree(10000);

            const t1 = performance.now();
            this.frameProcessingTime = t1 - t0;

            // 7. Update UI
            this.updateDiagnostics();
            this.updateRenderer();

            // 8. Advance
            this.currentFrame++;
            this.updateProgress();

            // Log status transitions and diagnostics
            const result = this.vio.getLatestResult();
            if (result) {
                const statusNames = ['NOT_CONFIGURED', 'INITIALIZING', 'TRACKING', 'LOST', 'COOLDOWN'];
                const statusName = statusNames[result.statusCode] || 'UNKNOWN';

                // Log every 20 frames for first 200, then every 100
                const logInterval = frameIdx < 200 ? 20 : 100;
                if (frameIdx === 0 || frameIdx % logInterval === 0) {
                    let poseInfo = 'no pose';
                    if (result.pose) {
                        const x = result.pose[3], y = result.pose[7], z = result.pose[11];
                        const hasNaN = result.pose.some(v => isNaN(v));
                        const isZero = result.pose.every(v => v === 0);
                        poseInfo = hasNaN ? 'NaN!' : isZero ? 'all-zero' : `[${x.toFixed(4)}, ${y.toFixed(4)}, ${z.toFixed(4)}]`;
                    }
                    this.log(`Frame ${frameIdx}: status=${statusName}, features=${result.featureCount}, imu=${this.lastIMUSliceCount}, time=${this.frameProcessingTime.toFixed(0)}ms, pose=${poseInfo}`);
                }

                // Check for NaN in pose
                if (result.pose) {
                    const hasNaN = result.pose.some(v => isNaN(v));
                    if (hasNaN) {
                        this.log(`NaN detected in pose at frame ${frameIdx}!`, 'error');
                    }
                }

                // Log status transitions
                if (this._lastStatus !== undefined && this._lastStatus !== result.statusCode) {
                    this.log(`STATUS CHANGE at frame ${frameIdx}: ${statusNames[this._lastStatus]} -> ${statusName}`);
                }
                this._lastStatus = result.statusCode;
            }

        } catch (err) {
            this.log(`Frame ${frameIdx} error: ${err.message}`, 'error');
            console.error(`[TUM-VI] Frame ${frameIdx}:`, err);
            this.currentFrame++; // skip failed frame
        } finally {
            this.processing = false;
        }
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

    /** Update 3D renderer */
    updateRenderer() {
        if (!this.renderer) return;
        const result = this.vio.getLatestResult();
        if (result && result.pose) {
            this.renderer.updateCameraPose(result.pose);
        }
        const mapData = this.vio.getMapPoints();
        if (mapData && mapData.count > 0) {
            this.renderer.updateMapPoints(mapData.points, mapData.count);
        }
        this.renderer.render();
    }

    /** Update progress bar */
    updateProgress() {
        const total = this.imageList.length || 1;
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
            // Max speed: process as fast as possible with rAF yielding
            this.playbackTimerType = 'raf';
            const loop = async () => {
                if (!this.playing || this.paused) return;
                await this.processNextFrame();
                if (this.playing && !this.paused) {
                    this.playbackTimer = requestAnimationFrame(loop);
                }
            };
            this.playbackTimer = requestAnimationFrame(loop);
        } else if (this.speed > 0) {
            // Real-time playback: 20Hz * speed
            this.playbackTimerType = 'timeout';
            const intervalMs = 1000 / (20 * this.speed);
            const loop = async () => {
                if (!this.playing || this.paused) return;
                await this.processNextFrame();
                if (this.playing && !this.paused) {
                    this.playbackTimer = setTimeout(loop, intervalMs);
                }
            };
            this.playbackTimer = setTimeout(loop, 0);
        }
        // speed === 0: step mode, manual advance only
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
        this.currentFrame = 0;
        this.processing = false;

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
document.addEventListener('DOMContentLoaded', () => app.initialize());
