/**
 * Main application - combines Camera, IMU, VIO, and Renderer.
 * Includes IMU data overlay, coordinate system verification, and mobile config management.
 *
 * Architecture:
 * - IMU data is flushed and sent to the worker independently from camera frames.
 * - Camera frames are sent without IMU data; the worker drains its internal
 *   IMU buffer on each frame.
 * - This prevents IMU data loss when frames are dropped due to worker being busy.
 */
import { VIOWrapper } from './vio-wrapper.js?v=11';
import { Camera } from './camera.js?v=11';
import { IMU } from './imu.js?v=11';
import { Renderer } from './renderer.js?v=11';
import { OrientationHandler } from './orientation.js?v=11';

/**
 * URL parameter overrides for rapid mobile testing.
 * Usage: https://host:port/?fx=466&config=mobile_highend&fth=5.0&rotate=ccw
 *
 *   ?fx=NNN        — Override focal length (pixels). Bypasses FOV estimation.
 *   ?config=NAME   — Config profile: mobile_default, mobile_highend, euroc, tum_vi
 *   ?fth=N.N       — Override fundamental matrix RANSAC threshold (default: 5.0)
 *   ?rotate=none|cw|ccw — Force rotation mode (overrides auto-detection)
 */
const URL_PARAMS = new URLSearchParams(window.location.search);

/** Send a log message to the server for remote debugging on mobile */
function remoteLog(level, msg) {
    try {
        navigator.sendBeacon('/log', JSON.stringify({ level, msg }));
    } catch (_) { /* ignore */ }
}

/**
 * Mobile web VIO configuration profiles.
 * Mobile phones have different sensor characteristics than research-grade IMUs.
 */
const VIO_CONFIGS = {
    // Default mobile phone config
    // NOTE: Noise values enter pre-integration covariance as acc_n², so even
    // small changes have quadratic effect. Values must match VINS-Mono's
    // continuous-time noise density model (NOT raw sensor specs).
    // Reference: EuRoC acc_n=0.08, TUM-VI acc_n=0.028
    //
    // Previous values (acc_n=0.2, gyr_n=0.03) were ~50x too loose in covariance
    // (0.2²/0.028² ≈ 51), making the optimizer barely trust IMU data.
    // Result: visual-only tracking → rapid divergence on fast motion.
    // Tightened to ~2-3x EuRoC level, appropriate for modern mobile MEMS.
    mobile_default: {
        label: 'Mobile Default',
        acc_n: 0.3,       // accelerometer noise density (m/s²/√Hz)
                          //   ~4x EuRoC (0.08). Mobile MEMS is noisier but not 12x.
                          //   Previous 1.0 made IMU nearly weightless → no scale constraint,
                          //   causing depth over-estimation and tracking loss on fast motion.
                          //   Previous 0.08-0.15 caused Ba explosion, but that was due to
                          //   timestamp error (now fixed: requestVideoFrameCallback) and
                          //   low frame rate (now fixed: 30fps). 0.3 balances IMU/visual.
        acc_w: 0.003,     // accelerometer random walk (m/s²·s/√Hz)
                          //   ~75x EuRoC (0.00004). Loosened from 0.001 to allow faster
                          //   Ba convergence during initial motion. Ba is NOT estimated
                          //   during initialization (only Bg is), so it starts at zero.
                          //   At acc_w=0.001, Ba took ~200 frames to converge to -0.45,
                          //   accumulating significant scale error. 0.003 ≈ 3x faster.
        gyr_n: 0.02,      // gyroscope noise density (rad/s/√Hz)
                          //   ~5x EuRoC (0.004). Moderate: gives IMU rotational constraint
                          //   while tolerating mobile gyro noise. Helps maintain pose
                          //   during brief feature loss from fast motion.
        gyr_w: 0.0005,    // gyroscope random walk
                          //   Increased from 0.0001 to allow faster Bg adaptation on mobile.
        g_norm: 9.81,
        focalLengthFactor: null,  // null → estimate from FOV (see estimateFocalLength)
        modelType: 2,  // C++ enum: PINHOLE=2 (not 0)
        solver_time: 0.04,   // 40ms Ceres budget — MUST NOT reduce below this.
                             // At 25ms/6iter, Ba/Bg never update → velocity diverges monotonically.
                             // VINS-Mono needs sufficient iterations for bias convergence.
        num_iterations: 8,   // 8 iterations required for bias (Ba, Bg) convergence in DOGLEG.
                             // 6 iterations: optimizer terminates before reaching bias parameters
                             // → Ba=(0,0,0) throughout → ~1 m/s² drift → divergence in ~7s.
        max_features: 100,   // Proportional to 240x180 resolution (VINS-Mono uses 150 for 512x512).
                             // Fewer features = fewer Ceres residuals = faster optimization.
                             // 100 features at 240x180 ≈ same density as 150 at 512x512.
        // Image downscale factor: 0.5 = half resolution (240x320).
        // Reduces computation ~3x (CLAHE, LK pyramid, corner detection all O(pixels)).
        // Also halves barrel distortion magnitude in pixels, reducing F-matrix
        // edge rejection that causes feature center clustering.
        processScale: 0.5,
        // LK optical flow: 4 pyramid levels handle up to ~80px inter-frame
        // displacement (10px/level × 2^3). At 50ms init / 33ms tracking,
        // fast rotation (200°/s) causes ~40px shift at 240x180 → within range.
        // 3 pyramid levels only handled ~40px → tracking loss on fast motion.
        lk_window: 21,
        lk_pyramid: 4,
        min_dist: 15,
        // Edge distortion compensation: restores edge features rejected by F-matrix
        // due to unmodeled barrel distortion from PINHOLE model with zero distortion.
        // 4.0 = edge features get up to 5x the base RANSAC threshold.
        // Increased from 2.0: logs show F-matrix cascade (85→56→22→14) during
        // fast motion, with edge recovery only restoring 1-12 per frame.
        f_edge_factor: 4.0,
    },
    // Tuned for high-end phones (iPhone 14+, Pixel 7+, Galaxy S23+)
    mobile_highend: {
        label: 'Mobile High-end',
        acc_n: 0.05,
        acc_w: 0.0002,    // reduced from 0.001 (TUM VI BMI160 baseline)
        gyr_n: 0.005,
        gyr_w: 0.00005,   // reduced from 0.001
        g_norm: 9.81,
        focalLengthFactor: null,  // estimate from FOV
        modelType: 2,  // C++ enum: PINHOLE=2
        solver_time: 0.06,
        num_iterations: 10,
        max_features: 120,
        processScale: 0.67,       // ~320x427, good balance of speed vs quality
        lk_window: 17,
        lk_pyramid: 4,
        min_dist: 18,
        f_edge_factor: 3.0,
    },
    // EuRoC MAV dataset (research-grade VI sensor)
    euroc: {
        label: 'EuRoC Dataset',
        acc_n: 0.08,
        acc_w: 0.00004,
        gyr_n: 0.004,
        gyr_w: 2.0e-6,
        g_norm: 9.81007,
        focalLengthFactor: null,  // uses dataset intrinsics
        modelType: 2,  // C++ enum: PINHOLE=2
        solver_time: 0.1,
        num_iterations: 10,
        max_features: 150,
    },
    // TUM VI dataset
    tum_vi: {
        label: 'TUM VI Dataset',
        acc_n: 0.028,
        acc_w: 0.00086,
        gyr_n: 0.004,
        gyr_w: 2.2e-5,
        g_norm: 9.81007,
        focalLengthFactor: null,
        modelType: 0,
        solver_time: 0.1,
        num_iterations: 10,
        max_features: 150,
    },
};

/** IMU flush interval in ms (independent of rAF) */
const IMU_FLUSH_INTERVAL_MS = 10;

/**
 * Number of camera frames to skip at startup to allow auto-exposure
 * and white-balance to settle before VIO processing begins.
 * 8th Wall pattern: loading-module.js FIRST_FRAME_DELAY=5.
 */
const CAMERA_WARMUP_FRAMES = 5;

/**
 * Minimum interval between VIO frame processing (ms).
 * Target: 30fps (33ms interval).
 *
 * Original VINS-Mono was designed for 20fps camera + 200Hz IMU.
 * Mobile web has 60Hz IMU, so at 30fps: ~2 IMU readings per frame —
 * minimum but workable with interpolation.
 *
 * 33ms interval = smaller pixel displacement between frames = better
 * LK optical flow tracking. Previous 80ms (12.5fps) caused LARGER
 * displacements, breaking LK tracking on moderate motion.
 */
const MIN_FRAME_INTERVAL_MS = 33;

/**
 * Frame interval during VIO initialization phase (ms).
 * Target: 10fps (100ms interval).
 *
 * VINS-Mono initialization requires:
 *   - Sufficient IMU pre-integration quality (needs 4+ readings/frame)
 *   - Enough parallax across the 10-frame sliding window (need 30px)
 *   - IMU excitation (gravity vector variance > 0.25)
 *
 * At 60Hz IMU (mobile):
 *   - 33ms interval (30fps): ~2 IMU/frame — too sparse for init
 *   - 50ms interval (20fps): ~3 IMU/frame — marginal but adequate
 *   - 100ms interval (10fps): ~6 IMU/frame — good pre-integration but
 *     causes LK tracking loss on moderate motion (>90°/s → >37px displacement
 *     exceeds 3-level pyramid range of ~40px)
 *
 * 50ms balances IMU density (~3/frame) vs tracking robustness.
 * The 10-frame sliding window at 50ms spans 0.5s — still sufficient
 * for parallax if user moves steadily.
 * After initialization succeeds, we switch to MIN_FRAME_INTERVAL_MS (30fps).
 */
const INIT_FRAME_INTERVAL_MS = 50;

/**
 * Estimate focal length from camera FOV or track settings.
 * Priority:
 *   1. MediaStreamTrack.getCapabilities().fov (if available)
 *   2. Reasonable FOV assumption based on lens type detection
 *   3. Config factor (if explicitly provided)
 *   4. Default ~75° FOV (typical mobile rear camera, standard lens)
 *
 * Getting the focal length approximately right is CRITICAL for VIO:
 * - Too large fx (narrow FOV assumption): features appear to move less →
 *   under-estimated parallax → wrong triangulation → divergence
 * - Too small fx (wide FOV assumption): features appear to move more →
 *   over-estimated parallax → noisy geometry → drift
 *
 * @param {MediaStreamTrack} videoTrack - Camera video track
 * @param {number} width - Image width in pixels
 * @param {number} height - Image height in pixels
 * @param {number|null} configFactor - focalLengthFactor from config profile
 * @returns {{fx: number, method: string}} Estimated focal length and estimation method
 */
function estimateFocalLength(videoTrack, width, height, configFactor) {
    // Method 0: URL override (?fx=NNN) — highest priority for rapid testing
    const urlFx = URL_PARAMS.get('fx');
    if (urlFx) {
        const fx = parseFloat(urlFx);
        if (Number.isFinite(fx) && fx > 50 && fx < 2000) {
            console.log(`[VIO] ★ Focal length from URL override: fx=${fx}`);
            return { fx, method: `URL override ?fx=${urlFx}` };
        }
        console.warn(`[VIO] Invalid URL fx=${urlFx}, ignoring`);
    }

    // Method 1: Try to get FOV from MediaStreamTrack
    if (videoTrack) {
        try {
            const settings = videoTrack.getSettings();

            // Check for explicit FOV (future-proof, not yet widely supported)
            if (settings.fieldOfView) {
                const fovRad = settings.fieldOfView * Math.PI / 180;
                const refDim = Math.max(width, height);
                const fx = (refDim / 2) / Math.tan(fovRad / 2);
                console.log(`[VIO] Focal length from track FOV: ${settings.fieldOfView}° (ref=${refDim}) → fx=${fx.toFixed(1)}`);
                return { fx, method: `track FOV ${settings.fieldOfView}°` };
            }

            // Check zoom level - applies as multiplier to base focal length
            if (settings.zoom && settings.zoom > 1) {
                const baseFovDeg = 75;
                const refDim = Math.max(width, height);
                const baseFx = (refDim / 2) / Math.tan((baseFovDeg * Math.PI / 180) / 2);
                const fx = baseFx * settings.zoom;
                console.log(`[VIO] Focal length adjusted for zoom ${settings.zoom}x: fx=${fx.toFixed(1)}`);
                return { fx, method: `zoom ${settings.zoom}x (base ${baseFovDeg}°)` };
            }
        } catch (_) {
            // getSettings/getCapabilities not available
        }

        // Try to detect ultrawide lens (some devices expose this info)
        try {
            const capabilities = videoTrack.getCapabilities ? videoTrack.getCapabilities() : null;
            if (capabilities) {
                // Log capabilities for debugging
                const capKeys = Object.keys(capabilities);
                console.log(`[VIO] Track capabilities: ${capKeys.join(', ')}`);
                if (capabilities.width && capabilities.height) {
                    console.log(`[VIO] Track resolution range: ${capabilities.width.min}-${capabilities.width.max} x ${capabilities.height.min}-${capabilities.height.max}`);
                }
                // Some devices expose focusDistance; very short min suggests ultrawide
                if (capabilities.focusDistance && capabilities.focusDistance.min < 0.05) {
                    const ultrawideFovDeg = 100;
                    const refDim = Math.max(width, height);
                    const fx = (refDim / 2) / Math.tan((ultrawideFovDeg * Math.PI / 180) / 2);
                    console.log(`[VIO] Likely ultrawide lens (focusDist min=${capabilities.focusDistance.min}m) → FOV~${ultrawideFovDeg}°, fx=${fx.toFixed(1)}`);
                    return { fx, method: `ultrawide detect ${ultrawideFovDeg}°` };
                }
            }
        } catch (_) {
            // getCapabilities not available
        }
    }

    // Method 2: Use config factor if explicitly specified
    if (configFactor) {
        const refDim = Math.max(width, height);
        const fx = refDim * configFactor;
        console.log(`[VIO] Focal length from config factor ${configFactor} (ref=${refDim}): fx=${fx.toFixed(1)}`);
        return { fx, method: `config factor ${configFactor}` };
    }

    // Method 3: Default — assume typical mobile rear camera ~69° horizontal FOV
    // Measured main camera FOVs (2023-2026):
    //   iPhone 14 main: 69°, Pixel 7: 72°, Galaxy S23: 70°
    //   Ultrawide: 100-120° (detected by focusDistance check above)
    //   Telephoto: 35-50°
    // Using 69° based on iPhone 14 main camera (most common baseline).
    //
    // IMPORTANT: The 69° FOV is the camera's WIDER field of view, which
    // corresponds to the sensor's longer dimension (640px in landscape).
    // In portrait mode, the output width (480px) spans a narrower FOV (~54°).
    // The focal length in pixels is a lens property — independent of rotation.
    // Always compute using the longer dimension to get correct fx.
    const defaultFovDeg = 69;
    const longerDim = Math.max(width, height);
    const fx = (longerDim / 2) / Math.tan((defaultFovDeg * Math.PI / 180) / 2);
    console.log(`[VIO] Focal length from default FOV ${defaultFovDeg}° (ref dim=${longerDim}): fx=${fx.toFixed(1)}`);
    return { fx, method: `default ${defaultFovDeg}° (ref=${longerDim})` };
}

/**
 * Validate focal length is within reasonable bounds for a phone camera.
 * @param {number} fx - Estimated focal length in pixels
 * @param {number} width - Image width in pixels
 * @returns {{fx: number, valid: boolean}} Validated (or corrected) focal length
 */
function validateFocalLength(fx, width, height) {
    // Typical phone camera: FOV 50°-120° → fx ranges from 0.42 to 1.2 × longer dimension
    // Use longer dimension because fx is computed from the sensor's wider FOV.
    const longerDim = Math.max(width, height || width);
    const minFx = longerDim * 0.4;
    const maxFx = longerDim * 1.5;
    if (!Number.isFinite(fx) || fx < minFx || fx > maxFx) {
        const fallbackFov = 69;
        const corrected = (longerDim / 2) / Math.tan((fallbackFov * Math.PI / 180) / 2);
        console.warn(`[VIO] Invalid focal length ${fx} (bounds: ${minFx.toFixed(0)}-${maxFx.toFixed(0)}) → fallback ${fallbackFov}° FOV: fx=${corrected.toFixed(1)}`);
        return { fx: corrected, valid: false };
    }
    return { fx, valid: true };
}

/**
 * Downsample a grayscale image by 2x using area averaging.
 * Fast: only array access and bit shifts. Produces good quality for VIO.
 * @param {Uint8Array} gray - Source grayscale pixels
 * @param {number} w - Source width
 * @param {number} h - Source height
 * @returns {Uint8Array} Downsampled image (w/2 × h/2)
 */
function downsample2x(gray, w, h) {
    const nw = w >> 1;
    const nh = h >> 1;
    const out = new Uint8Array(nw * nh);
    for (let y = 0; y < nh; y++) {
        const sy = y << 1;
        const srcRow0 = sy * w;
        const srcRow1 = srcRow0 + w;
        const dstRow = y * nw;
        for (let x = 0; x < nw; x++) {
            const sx = x << 1;
            out[dstRow + x] = (gray[srcRow0 + sx] + gray[srcRow0 + sx + 1] +
                                gray[srcRow1 + sx] + gray[srcRow1 + sx + 1] + 2) >> 2;
        }
    }
    return out;
}

/**
 * Downsample a grayscale image by an arbitrary scale using bilinear area average.
 * Falls back to downsample2x when scale is exactly 0.5.
 * @param {Uint8Array} gray - Source grayscale pixels
 * @param {number} srcW - Source width
 * @param {number} srcH - Source height
 * @param {number} dstW - Destination width
 * @param {number} dstH - Destination height
 * @returns {Uint8Array} Downsampled image
 */
function downsampleGray(gray, srcW, srcH, dstW, dstH) {
    if (dstW === (srcW >> 1) && dstH === (srcH >> 1)) {
        return downsample2x(gray, srcW, srcH);
    }
    const out = new Uint8Array(dstW * dstH);
    const xRatio = srcW / dstW;
    const yRatio = srcH / dstH;
    for (let y = 0; y < dstH; y++) {
        const srcY = y * yRatio;
        const sy = Math.floor(srcY);
        const fy = srcY - sy;
        const sy1 = Math.min(sy + 1, srcH - 1);
        for (let x = 0; x < dstW; x++) {
            const srcX = x * xRatio;
            const sx = Math.floor(srcX);
            const fx = srcX - sx;
            const sx1 = Math.min(sx + 1, srcW - 1);
            // Bilinear interpolation
            const v00 = gray[sy * srcW + sx];
            const v10 = gray[sy * srcW + sx1];
            const v01 = gray[sy1 * srcW + sx];
            const v11 = gray[sy1 * srcW + sx1];
            out[y * dstW + x] = Math.round(
                v00 * (1 - fx) * (1 - fy) + v10 * fx * (1 - fy) +
                v01 * (1 - fx) * fy + v11 * fx * fy
            );
        }
    }
    return out;
}

/**
 * Convert deviceorientation Euler angles to quaternion (ZXY rotation order).
 * Formula extracted from 8th Wall XR engine (LA function).
 *
 * @param {number} alpha - Compass heading (Z rotation, degrees)
 * @param {number} beta  - Front-back tilt (X rotation, degrees)
 * @param {number} gamma - Left-right tilt (Y rotation, degrees)
 * @returns {{w: number, x: number, y: number, z: number}}
 */
function eulerToQuaternion(alpha, beta, gamma) {
    const D2R = Math.PI / 180;
    const hx = (beta  * D2R) / 2;
    const hy = (gamma * D2R) / 2;
    const hz = (alpha * D2R) / 2;
    const cx = Math.cos(hx), sx = Math.sin(hx);
    const cy = Math.cos(hy), sy = Math.sin(hy);
    const cz = Math.cos(hz), sz = Math.sin(hz);
    return {
        w: cx * cy * cz - sx * sy * sz,
        x: sx * cy * cz - cx * sy * sz,
        y: cx * sy * cz + sx * cy * sz,
        z: cx * cy * sz + sx * sy * cz,
    };
}

/**
 * Extract gravity direction from deviceorientation quaternion.
 * W3C Earth frame is Z-up (East-North-Up), so gravity = (0, 0, -g).
 * The deviceorientation quaternion q rotates device frame → earth frame.
 * To get gravity in device frame: v_device = q_conj * (0, 0, -g) * q.
 * Expanded: R^T * (0, 0, -g) where R is the rotation matrix of q.
 *
 * @param {{w,x,y,z}} q - Orientation quaternion from eulerToQuaternion
 * @returns {{x: number, y: number, z: number}} Gravity in W3C device frame (m/s²)
 */
function gravityFromOrientation(q) {
    const g = 9.81;
    return {
        x: -g * 2 * (q.x * q.z - q.w * q.y),
        y: -g * 2 * (q.y * q.z + q.w * q.x),
        z: -g * (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z),
    };
}

class App {
    constructor() {
        this.vio = new VIOWrapper();
        // Forward C++ stdout/stderr to browser console + remote log
        this.vio.onWasmLog = (level, msg) => {
            if (msg && msg.trim().length > 0) {
                const text = `[WASM] ${msg}`;
                if (level === 'warn') { console.warn(text); } else { console.log(text); }
                remoteLog(level, text);
            }
        };
        this.camera = new Camera();
        this.imu = new IMU();
        this.orientation = new OrientationHandler();
        this.renderer = null;

        this.running = false;
        this.frameCount = 0;
        this.totalFrameCount = 0;
        this.lastFPSTime = 0;
        this.fps = 0;
        this.imuLogCount = 0;

        // Active config profile (can be overridden with ?config=NAME)
        this.activeConfig = URL_PARAMS.get('config') || 'mobile_default';
        if (!VIO_CONFIGS[this.activeConfig]) {
            console.warn(`[VIO] Unknown config '${this.activeConfig}', falling back to mobile_default`);
            this.activeConfig = 'mobile_default';
        }

        // UI elements
        this.statusEl = null;
        this.fpsEl = null;
        this.featureEl = null;
        this.frameEl = null;
        this.startBtn = null;
        this.resetBtn = null;
        this.imuRateEl = null;

        // IMU overlay elements
        this.imuOverlayEls = {};
        this.lastImuUpdateTime = 0;

        // IMU flush timer (independent of rAF)
        this._imuFlushTimer = null;

        // deviceorientation gravity hint (8th Wall approach)
        // Provides immediate gravity direction from device orientation sensor,
        // used for initialization validation and future gravity seeding.
        this._deviceOrientationHandler = null;
        this._gravityHint = null;        // {x, y, z} gravity in VIO body frame
        this._gravityHintRaw = null;     // {x, y, z} gravity in device frame

        // Frame deduplication: skip if video.currentTime hasn't changed
        this._lastVideoTime = -1;
        // Frame rate limiting: ensure enough IMU between VIO frames
        this._lastVIOFrameTime = 0;
        this._lastFrameTimestamp = 0;

        // VIO initialization state (for adaptive frame rate)
        this._vioInitialized = false;
        this._initFrameCount = 0;

        // Grayscale preview canvas (debug visualization)
        this._grayPreviewCanvas = null;
        this._grayPreviewCtx = null;
        this._previewImageData = null;  // Reused ImageData to avoid GC pressure

        // Stored config for reconfiguration on orientation change
        this._lastConfigParams = null;

        // Image downscaling for mobile performance + distortion reduction
        this._processScale = 1.0;   // Set from config.processScale
        this._processWidth = 0;     // VIO processing width (after scale)
        this._processHeight = 0;    // VIO processing height (after scale)
        this._captureWidth = 0;     // Camera capture width (before scale)
        this._captureHeight = 0;    // Camera capture height (before scale)

        // Camera warmup: skip first N frames to allow AE/AWB to settle.
        // 8th Wall pattern: loading-module.js FIRST_FRAME_DELAY=5.
        this._frameWarmupCount = 0;

        // Window blur/focus lifecycle (8th Wall pauseonblur.js pattern).
        // Catches edge cases (dropdown menus, permission dialogs) that don't
        // trigger visibilitychange.
        this._blurPaused = false;
        this._onWindowBlur = null;
        this._onWindowFocus = null;
    }

    async initialize() {
        this.statusEl = document.getElementById('status');
        this.fpsEl = document.getElementById('fps');
        this.featureEl = document.getElementById('features');
        this.frameEl = document.getElementById('frame-count');
        this.startBtn = document.getElementById('btn-start');
        this.resetBtn = document.getElementById('btn-reset');
        this.imuRateEl = document.getElementById('imu-rate');

        // IMU overlay elements
        this.imuOverlayEls = {
            accX: document.getElementById('acc-x'),
            accY: document.getElementById('acc-y'),
            accZ: document.getElementById('acc-z'),
            gyroX: document.getElementById('gyro-x'),
            gyroY: document.getElementById('gyro-y'),
            gyroZ: document.getElementById('gyro-z'),
        };

        this.updateStatus('Loading WASM module...');

        try {
            const urlParams = new URLSearchParams(window.location.search);
            const wasmFile = urlParams.get('wasm') || 'vio_engine.js';
            console.log(`Loading WASM: ${wasmFile}`);
            await this.vio.load('/' + wasmFile);
            this.updateStatus('WASM loaded. Ready to start.');
            this.startBtn.disabled = false;
        } catch (e) {
            this.updateStatus(`Failed to load WASM: ${e.message}`);
            console.error(e);
            return;
        }

        this.startBtn.addEventListener('click', () => this.start());
        this.resetBtn.addEventListener('click', () => this.resetVIO());

        // Handle tab visibility changes to prevent stale IMU data
        document.addEventListener('visibilitychange', () => this._onVisibilityChange());

        // Setup renderer
        const canvas3d = document.getElementById('canvas-3d');
        if (canvas3d) {
            this.renderer = new Renderer(canvas3d);
            window.addEventListener('resize', () => {
                const container = canvas3d.parentElement;
                this.renderer.resize(container.clientWidth, container.clientHeight);
                canvas3d.width = container.clientWidth;
                canvas3d.height = container.clientHeight;
                // iOS: scroll to stable position after resize to suppress address bar bounce
                // 8th Wall pattern: full-window-canvas-module.ts
                if (/iPad|iPhone|iPod/.test(navigator.userAgent)) {
                    setTimeout(() => window.scrollTo(0, (window.scrollY + 1) % 2), 300);
                }
            });
        }
    }

    async start() {
        this.startBtn.disabled = true;
        this.updateStatus('Requesting camera...');

        try {
            // Try to lock portrait orientation (simplest for VIO)
            await this.orientation.tryLockPortrait();
            const orientationType = this.orientation.getType();

            // Initialize camera — requests 640x480 from sensor, outputs portrait
            // (480x640) with actual pixel rotation if needed (camera.js v9).
            // Then crop to landscape 4:3 for VIO processing.
            await this.camera.initialize(640, 480);

            // Crop portrait to landscape 4:3 (width > height).
            // VIO works better with landscape: wider horizontal FOV for parallax,
            // standard SLAM assumption, fewer wasted floor/ceiling pixels.
            // 480x640 portrait → 480x360 landscape (4:3)
            this.camera.enableLandscapeCrop();
            const width = this.camera.width;
            const height = this.camera.height;
            this.updateStatus(`Camera: ${width}x${height}`);

            // Display video feed
            const videoContainer = document.getElementById('video-container');
            if (videoContainer) {
                const video = this.camera.getVideoElement();
                video.style.width = '100%';
                videoContainer.appendChild(video);
            }

            // Setup grayscale preview canvas
            // Camera outputs landscape 4:3 crop (e.g., 480x360) from portrait feed.
            // No CSS rotation needed — pixel data matches display orientation.
            this._grayPreviewCanvas = document.getElementById('gray-preview');
            if (this._grayPreviewCanvas) {
                this._grayPreviewCanvas.width = width;
                this._grayPreviewCanvas.height = height;
                this._grayPreviewCanvas.style.display = 'block';
                this._grayPreviewCtx = this._grayPreviewCanvas.getContext('2d');
            }

            // iOS viewport stabilization (8th Wall full-window-canvas-module.ts pattern).
            // Prevents iOS address bar from resizing the viewport during VIO, which
            // causes layout reflows and disrupts canvas rendering.
            const isIOS = /iPad|iPhone|iPod/.test(navigator.userAgent);
            if (isIOS) {
                document.body.style.overflowY = 'scroll';
                document.documentElement.style.overflow = 'hidden';
                // Scroll to stable position to prevent address bar toggle
                setTimeout(() => window.scrollTo(0, 1), 300);
                console.log('[VIO] iOS viewport stabilization applied');
            }

            // Get active config profile
            const config = VIO_CONFIGS[this.activeConfig];

            // Get orientation-aware camera-IMU extrinsic rotation (R_ic).
            // Camera v9 outputs landscape 4:3 crop, but R_ic is a physical rotation
            // (camera→body), unchanged by crop since camera axes stay the same.
            const r_ic = this.orientation.getRIC();

            // Log coordinate system configuration
            const video = this.camera.getVideoElement();
            const portraitDims = this.camera.getPortraitDimensions();
            console.log('[VIO] ═══════ Configuration Summary ═══════');
            console.log(`[VIO] Screen orientation: ${orientationType}`);
            console.log(`[VIO] Camera native: ${video.videoWidth}x${video.videoHeight}`);
            console.log(`[VIO] Camera portrait: ${portraitDims.width}x${portraitDims.height} (rotate=${this.camera.getRotateMode()})`);
            console.log(`[VIO] Camera crop: ${width}x${height} (${width > height ? 'LANDSCAPE' : 'PORTRAIT'} ${(width/height).toFixed(2)})`);
            console.log(`[VIO] Config profile: ${this.activeConfig} (${config.label})`);
            console.log(`[VIO] R_ic: [${r_ic.map(v => v.toFixed(1)).join(', ')}]`);
            console.log('[VIO] Camera frame (OpenCV): x=right, y=down, z=forward');
            console.log('[VIO] IMU frame (DeviceMotion): x=right-edge, y=top-edge, z=out-of-screen');
            console.log('[VIO] VIO body frame: x=right, y=forward, z=up');
            console.log('[VIO] Expected portrait upright → acc_z_vio ≈ +9.81 (gravity on Z)');

            // Determine processing scale and dimensions
            // Downscaling reduces computation (O(pixels) for CLAHE, LK, corner detection)
            // and halves barrel distortion magnitude, preventing F-matrix edge rejection.
            const scale = config.processScale || 1.0;
            this._processScale = scale;
            this._captureWidth = width;
            this._captureHeight = height;
            // Ensure even dimensions for 2x downsample path
            this._processWidth = (scale === 1.0) ? width : (Math.round(width * scale) & ~1);
            this._processHeight = (scale === 1.0) ? height : (Math.round(height * scale) & ~1);
            const pW = this._processWidth;
            const pH = this._processHeight;

            // Estimate focal length using FOV or config factor.
            // IMPORTANT: fx is a lens property — must use native portrait dimensions
            // (before crop) for FOV→focal length conversion, since the 69° FOV
            // spans the camera's full native longer dimension (640px), not the crop.
            const videoTrack = this.camera.getVideoTrack ? this.camera.getVideoTrack() : null;
            // portraitDims already declared above for logging
            let { fx, method: fxMethod } = estimateFocalLength(videoTrack, portraitDims.width, portraitDims.height, config.focalLengthFactor);
            const fxValidation = validateFocalLength(fx, portraitDims.width, portraitDims.height);
            if (!fxValidation.valid) {
                fx = fxValidation.fx;
                fxMethod += ' (corrected)';
            }
            // Scale focal length to processing resolution
            const fxScaled = fx * scale;
            const fyScaled = fxScaled;

            // Camera-IMU translation offset in VIO body frame
            // VIO body frame: X=right, Y=forward, Z=up
            // Camera ~2cm below IMU center along Z_vio (gravity direction)
            const t_ic = [0, 0, -0.02];

            const configured = await this.vio.configure({
                width: pW,
                height: pH,
                fx: fxScaled,
                fy: fyScaled,
                cx: pW / 2,
                cy: pH / 2,
                modelType: config.modelType,
                r_ic: r_ic,
                t_ic: t_ic,
                acc_n: config.acc_n,
                acc_w: config.acc_w,
                gyr_n: config.gyr_n,
                gyr_w: config.gyr_w,
                g_norm: config.g_norm,
            });

            if (!configured) {
                this.updateStatus('VIO configuration failed');
                return;
            }

            // Store config for reconfiguration on orientation change
            this._lastConfigParams = {
                width: pW, height: pH, fx: fxScaled, fy: fyScaled,
                cx: pW / 2, cy: pH / 2,
                modelType: config.modelType,
                t_ic,
                acc_n: config.acc_n, acc_w: config.acc_w,
                gyr_n: config.gyr_n, gyr_w: config.gyr_w,
                g_norm: config.g_norm,
            };

            // Apply mobile solver parameters if present in config
            if (config.solver_time !== undefined) {
                await this.vio.setMobileParams(
                    config.solver_time,
                    config.num_iterations,
                    config.max_features
                );
            }

            // Apply mobile-optimized tracking parameters
            if (config.lk_window !== undefined) {
                await this.vio.setTrackingParams(
                    config.lk_window,
                    config.lk_pyramid || 4,
                    config.min_dist || 20,
                    config.f_edge_factor || 0.0
                );
            }

            // Apply f_threshold override from URL (?fth=N.N)
            const urlFth = URL_PARAMS.get('fth');
            if (urlFth) {
                const fth = parseFloat(urlFth);
                if (Number.isFinite(fth) && fth > 0 && fth < 100) {
                    await this.vio.setFThreshold(fth);
                    console.log(`[VIO] f_threshold overridden to ${fth} via URL`);
                }
            }

            console.log(`[VIO] ═══════ VIO Engine Configured ═══════`);
            console.log(`[VIO]   Portrait: ${portraitDims.width}x${portraitDims.height} → Crop: ${width}x${height} → Process: ${pW}x${pH} (scale=${scale})`);
            console.log(`[VIO]   Native: ${video.videoWidth}x${video.videoHeight}`);
            console.log(`[VIO]   ★ Focal: fx=${fxScaled.toFixed(1)}, fy=${fyScaled.toFixed(1)} (${fxMethod}, scaled from ${fx.toFixed(1)})`);
            console.log(`[VIO]   ★ fx/width=${(fxScaled/pW).toFixed(3)}, fx/max(w,h)=${(fxScaled/Math.max(pW,pH)).toFixed(3)}`);
            console.log(`[VIO]   Principal: cx=${(pW/2).toFixed(1)}, cy=${(pH/2).toFixed(1)}`);
            console.log(`[VIO]   hFOV=${(2*Math.atan(pW/(2*fxScaled))*180/Math.PI).toFixed(1)}°, vFOV=${(2*Math.atan(pH/(2*fxScaled))*180/Math.PI).toFixed(1)}°`);
            console.log(`[VIO]   t_ic: [${t_ic}]`);
            console.log(`[VIO]   IMU noise: acc_n=${config.acc_n}, acc_w=${config.acc_w}, gyr_n=${config.gyr_n}, gyr_w=${config.gyr_w}`);
            console.log(`[VIO]   Solver: time=${config.solver_time}s, iter=${config.num_iterations}, features=${config.max_features}`);
            console.log(`[VIO]   Tracking: lk_window=${config.lk_window||21}, lk_pyramid=${config.lk_pyramid||4}, min_dist=${config.min_dist||20}, f_edge=${config.f_edge_factor||0}`);
            console.log(`[VIO]   Rotation: ${this.camera.getRotateMode()}, DimsSwapped: ${this.camera.isDimsSwapped()}`);
            console.log(`[VIO]   URL overrides: fx=${URL_PARAMS.get('fx')||'(none)'}, config=${URL_PARAMS.get('config')||'(none)'}, fth=${URL_PARAMS.get('fth')||'(none)'}, rotate=${URL_PARAMS.get('rotate')||'(auto)'}`);

            // Remote log key params for mobile debugging
            remoteLog('info', `VIO configured: ${pW}x${pH} (portrait ${portraitDims.width}x${portraitDims.height} → crop ${width}x${height}, scale=${scale}) ` +
                `fx=${fxScaled.toFixed(1)} (${fxMethod}) ` +
                `hFOV=${(2*Math.atan(pW/(2*fxScaled))*180/Math.PI).toFixed(1)}° ` +
                `profile=${this.activeConfig} rotate=${this.camera.getRotateMode()}`);

            // DeviceMotion pre-check (8th Wall loading-module.js pattern).
            // Verify that devicemotion events are actually firing before starting the
            // VIO pipeline. A 3-second timeout catches broken/missing IMU hardware
            // early and logs a warning for remote debugging.
            await new Promise((resolve) => {
                let motionDetected = false;
                const onMotion = () => {
                    motionDetected = true;
                    window.removeEventListener('devicemotion', onMotion);
                    console.log('[VIO] DeviceMotion pre-check: events firing OK');
                    resolve();
                };
                window.addEventListener('devicemotion', onMotion);
                setTimeout(() => {
                    window.removeEventListener('devicemotion', onMotion);
                    if (!motionDetected) {
                        console.warn('[VIO] WARNING: No devicemotion events detected. IMU may not be available.');
                        this.updateStatus('WARNING: No IMU events — check permissions');
                    }
                    resolve();
                }, 3000);
            });

            // Request IMU permission and start
            if (IMU.isAvailable()) {
                const granted = await this.imu.requestPermission();
                if (granted) {
                    this.imu.start(60);  // Chrome caps Generic Sensor API at 60Hz
                    console.log(`[VIO] IMU started: sensor=${this.imu.getSensorType()}`);

                    // Calibrate gyroscope bias while device is stationary.
                    // Mobile MEMS gyros have large bias offsets (0.01-0.1 rad/s)
                    // that cause VIO divergence if not compensated.
                    this.updateStatus('Calibrating gyro bias (keep still)...');
                    const calResult = await this.imu.calibrate(1500);
                    if (calResult) {
                        const b = calResult.bias;
                        console.log(`[VIO] Gyro bias: (${b.x.toFixed(5)}, ${b.y.toFixed(5)}, ${b.z.toFixed(5)}) rad/s, |acc|=${calResult.gravMag.toFixed(3)}`);
                    }

                    this.updateStatus('Camera + IMU active');
                } else {
                    this.updateStatus('Camera active (no IMU permission)');
                }
            } else {
                this.updateStatus('Camera active (no IMU sensor)');
            }

            this.running = true;
            this.imuLogCount = 0;
            this.resetBtn.disabled = false;
            this.lastFPSTime = performance.now();

            // Start independent IMU flush timer
            this._startIMUFlush();

            // Window blur/focus lifecycle (8th Wall pauseonblur.js pattern).
            // Catches edge cases (dropdown menus, permission dialogs) that don't
            // trigger visibilitychange. Uses a _blurPaused flag so focus resumption
            // only undoes blur-pauses, not visibility-pauses.
            this._onWindowBlur = () => {
                if (!this.running || this._blurPaused) return;
                this._blurPaused = true;
                this._stopIMUFlush();
                console.log('[VIO] Window blur — pausing IMU flush');
            };
            this._onWindowFocus = () => {
                if (!this.running || !this._blurPaused) return;
                this._blurPaused = false;
                this.imu.flush();  // Discard stale data accumulated during blur
                this._startIMUFlush();
                console.log('[VIO] Window focus — resuming IMU flush');
            };
            window.addEventListener('blur', this._onWindowBlur);
            window.addEventListener('focus', this._onWindowFocus);

            // Start deviceorientation listener for gravity hint (8th Wall approach)
            this._startDeviceOrientationListener();

            // Start listening for orientation changes (reconfigures VIO if phone rotates)
            this.orientation.startListening((info) => this._onOrientationChange(info));

            // Register requestVideoFrameCallback for accurate capture timestamps.
            // performance.now() (loop iteration time) lags actual capture by 30-100ms.
            //
            // IMPORTANT: We must use presentationTime (DOMHighResTimeStamp, same
            // time base as performance.now()), NOT mediaTime. mediaTime is a media
            // playback position that starts from 0 — completely different time base
            // from the IMU timestamps which use performance.now()/1000.
            if ('requestVideoFrameCallback' in HTMLVideoElement.prototype) {
                const trackTimestamp = (now, metadata) => {
                    // presentationTime: DOMHighResTimeStamp (ms), same base as performance.now()
                    // Convert to seconds to match IMU timestamp convention
                    if (metadata.presentationTime) {
                        this._videoFrameTimestamp = metadata.presentationTime / 1000.0;
                    } else {
                        // Fallback: use the callback's 'now' parameter (also performance.now() based)
                        this._videoFrameTimestamp = now / 1000.0;
                    }
                    video.requestVideoFrameCallback(trackTimestamp);
                };
                video.requestVideoFrameCallback(trackTimestamp);
                console.log('[VIO] Using requestVideoFrameCallback for accurate frame timestamps');
            }

            // Wait for IMU buffer to accumulate before first frame
            // Without this, Frame #0 gets imuCount=0 (race condition after calibration flush)
            await new Promise(r => setTimeout(r, 200));

            // Start frame processing loop
            this.processLoop();

        } catch (e) {
            this.updateStatus(`Error: ${e.message}`);
            console.error(e);
            this.startBtn.disabled = false;
        }
    }

    /**
     * Flush IMU data from ring buffer, transform axes, and send to worker.
     * Called by both the periodic timer and explicitly before each VIO frame.
     * @returns {number} Number of IMU readings sent (0 if none available)
     */
    _flushAndSendIMU() {
        const { data, count } = this.imu.flush();
        if (count <= 0 || !data) return 0;

        // ── Transform IMU axes: W3C device frame → VIO body frame ──
        // Device: X_d=right-edge, Y_d=top-edge, Z_d=out-of-screen
        //   → phone upright portrait: acc_y ≈ +9.8 (gravity on Y)
        // VIO:    X_v=right, Y_v=forward, Z_v=up
        //   → phone upright portrait: acc_z ≈ +9.8 (gravity on Z)
        // Rotation: +90° around X axis
        //   x_v = x_d,  y_v = -z_d,  z_v = y_d
        for (let i = 0; i < count; i++) {
            const off = i * 7;
            const ay_dev = data[off + 2];
            const az_dev = data[off + 3];
            data[off + 2] = -az_dev;  // ay_vio = -az_device
            data[off + 3] = ay_dev;   // az_vio = ay_device

            const gy_dev = data[off + 5];
            const gz_dev = data[off + 6];
            data[off + 5] = -gz_dev;  // gy_vio = -gz_device
            data[off + 6] = gy_dev;   // gz_vio = gy_device
        }

        // Log first few batches for diagnostics (VIO body frame)
        if (this.imuLogCount < 10) {
            const r = {
                acc_x: data[1], acc_y: data[2], acc_z: data[3],
                gyro_x: data[4], gyro_y: data[5], gyro_z: data[6],
            };
            const accMag = Math.sqrt(r.acc_x**2 + r.acc_y**2 + r.acc_z**2);
            console.log(`[VIO] IMU batch #${this.imuLogCount} (VIO frame): ` +
                `count=${count} ` +
                `acc=(${r.acc_x.toFixed(3)}, ${r.acc_y.toFixed(3)}, ${r.acc_z.toFixed(3)}) ` +
                `gyro=(${r.gyro_x.toFixed(4)}, ${r.gyro_y.toFixed(4)}, ${r.gyro_z.toFixed(4)}) ` +
                `|acc|=${accMag.toFixed(3)}`);
            if (this.imuLogCount === 0) {
                console.log(`[VIO] IMU sensor type: ${this.imu.getSensorType()}`);
                console.log(`[VIO] IMU rate: ${Math.round(this.imu.getRate())}Hz`);
                console.log(`[VIO] Gyro bias calibrated: ${this.imu.isCalibrated()}`);
                // Gravity direction validation
                // In VIO body frame: phone upright portrait → acc_z ≈ +9.81
                if (accMag > 8.5 && accMag < 11.0) {
                    const gravAxis = Math.abs(r.acc_z) > Math.abs(r.acc_x) && Math.abs(r.acc_z) > Math.abs(r.acc_y) ? 'Z' :
                                     Math.abs(r.acc_y) > Math.abs(r.acc_x) ? 'Y' : 'X';
                    const gravSign = gravAxis === 'Z' ? Math.sign(r.acc_z) : gravAxis === 'Y' ? Math.sign(r.acc_y) : Math.sign(r.acc_x);
                    console.log(`[VIO] Gravity dominant axis: ${gravSign > 0 ? '+' : '-'}${gravAxis} (expected: +Z for portrait upright)`);
                    if (gravAxis !== 'Z' || gravSign < 0) {
                        console.warn(`[VIO] ⚠ Gravity NOT on +Z! Check IMU axis transform or phone orientation.`);
                        console.warn(`[VIO]   acc=(${r.acc_x.toFixed(2)}, ${r.acc_y.toFixed(2)}, ${r.acc_z.toFixed(2)})`);
                    }
                } else {
                    console.warn(`[VIO] ⚠ |acc|=${accMag.toFixed(2)} (expected ~9.81). Device may be moving or sensor error.`);
                }
                // Log deviceorientation gravity hint for cross-validation
                if (this._gravityHint) {
                    const gh = this._gravityHint;
                    const ghMag = Math.sqrt(gh.x * gh.x + gh.y * gh.y + gh.z * gh.z);
                    console.log(`[VIO] Gravity hint (deviceorientation): (${gh.x.toFixed(2)}, ${gh.y.toFixed(2)}, ${gh.z.toFixed(2)}), |g|=${ghMag.toFixed(2)}`);
                }
                // Log LINEAR_ACCELERATION hardware gravity estimate
                const imuGrav = this.imu.getGravityEstimate();
                if (imuGrav) {
                    const igMag = Math.sqrt(imuGrav.x * imuGrav.x + imuGrav.y * imuGrav.y + imuGrav.z * imuGrav.z);
                    console.log(`[VIO] Gravity estimate (LINEAR_ACCEL): (${imuGrav.x.toFixed(2)}, ${imuGrav.y.toFixed(2)}, ${imuGrav.z.toFixed(2)}), |g|=${igMag.toFixed(2)}`);
                }
            }
            this.imuLogCount++;
        }

        // Send IMU independently to worker (never blocked by workerBusy)
        this.vio.sendIMU(data, count);
        return count;
    }

    /**
     * Start independent IMU flush timer.
     * Ensures IMU data reaches the worker even when VIO frames are being
     * dropped (worker busy). processLoop also flushes right before each
     * frame to minimize data loss.
     */
    _startIMUFlush() {
        this._stopIMUFlush();
        this._imuFlushTimer = setInterval(() => {
            if (!this.running) return;
            this._flushAndSendIMU();
        }, IMU_FLUSH_INTERVAL_MS);
    }

    /** Stop the independent IMU flush timer. */
    _stopIMUFlush() {
        if (this._imuFlushTimer !== null) {
            clearInterval(this._imuFlushTimer);
            this._imuFlushTimer = null;
        }
    }

    /**
     * Start collecting deviceorientation for gravity hint.
     * 8th Wall pattern: deviceorientation quaternion provides immediate
     * gravity direction, avoiding the wait for accelerometer averaging.
     */
    _startDeviceOrientationListener() {
        this._stopDeviceOrientationListener();
        this._deviceOrientationHandler = (event) => {
            const alpha = event.alpha ?? 0;
            const beta = event.beta ?? 0;
            const gamma = event.gamma ?? 0;

            // Convert to quaternion (8th Wall ZXY formula)
            const quat = eulerToQuaternion(alpha, beta, gamma);

            // Extract gravity in W3C device frame
            const gravDev = gravityFromOrientation(quat);
            this._gravityHintRaw = gravDev;

            // Transform to VIO body frame (same as IMU: x_v=x_d, y_v=-z_d, z_v=y_d)
            this._gravityHint = {
                x: gravDev.x,
                y: -gravDev.z,
                z: gravDev.y,
            };
        };
        window.addEventListener('deviceorientation', this._deviceOrientationHandler, true);
    }

    /** Stop deviceorientation listener. */
    _stopDeviceOrientationListener() {
        if (this._deviceOrientationHandler) {
            window.removeEventListener('deviceorientation', this._deviceOrientationHandler, true);
            this._deviceOrientationHandler = null;
        }
        this._gravityHint = null;
        this._gravityHintRaw = null;
    }

    processLoop() {
        if (!this.running) return;

        const now = performance.now();

        // ── Frame deduplication + rate limiting ──
        // Problem: rAF fires at ~60Hz but camera runs at ~30fps.
        // Without dedup, the VIO processes the same image with different
        // timestamps → zero visual motion but IMU time advances →
        // conflicting visual/IMU constraints → optimizer divergence.
        //
        // Rate limiting ensures enough IMU readings accumulate between
        // VIO frames (at 60Hz IMU, 20fps VIO → ~3 readings per frame).
        // Adaptive frame interval: slower during init for better IMU density
        const frameInterval = this._vioInitialized ? MIN_FRAME_INTERVAL_MS : INIT_FRAME_INTERVAL_MS;
        const video = this.camera.getVideoElement();
        const videoTime = video ? video.currentTime : -1;
        const isNewFrame = videoTime > 0 && videoTime !== this._lastVideoTime;
        const enoughTime = (now - this._lastVIOFrameTime) >= frameInterval;

        if (isNewFrame && enoughTime) {
            this._lastVideoTime = videoTime;
            this._lastVIOFrameTime = now;

            // Use accurate video frame timestamp if available, otherwise fall back to performance.now().
            // performance.now() is the loop iteration time, NOT actual camera capture time.
            // requestVideoFrameCallback's presentationTime is in the same time base as
            // performance.now() (converted to seconds), matching IMU timestamps.
            let frameTimestamp;
            if (this._videoFrameTimestamp !== undefined && this._videoFrameTimestamp > 0) {
                frameTimestamp = this._videoFrameTimestamp;
            } else {
                frameTimestamp = now / 1000.0;
            }
            const gray = this.camera.captureGrayscale();
            if (gray) {
                // Draw grayscale preview at capture resolution (debug visualization).
                // Render every 5th frame to reduce main-thread load.
                // Previous every-3rd at 30fps camera = ~10fps visual → still felt slow.
                // Every 5th = ~6fps visual, but frees ~2ms/frame for VIO processing.
                // The preview is purely diagnostic — SLAM accuracy is unaffected.
                this._previewCounter = (this._previewCounter || 0) + 1;
                if (this._grayPreviewCtx && (this._previewCounter % 5 === 0)) {
                    const w = this.camera.width;
                    const h = this.camera.height;
                    // Reuse ImageData to avoid per-frame GC allocation (~700KB for 480x360)
                    if (!this._previewImageData || this._previewImageData.width !== w || this._previewImageData.height !== h) {
                        this._previewImageData = this._grayPreviewCtx.createImageData(w, h);
                        // Pre-fill alpha channel (never changes)
                        const d = this._previewImageData.data;
                        for (let i = 3; i < d.length; i += 4) d[i] = 255;
                    }
                    const d = this._previewImageData.data;
                    for (let i = 0, j = 0; i < gray.length; i++, j += 4) {
                        d[j] = d[j + 1] = d[j + 2] = gray[i];
                    }
                    this._grayPreviewCtx.putImageData(this._previewImageData, 0, 0);

                    // Draw version + rotation mode + processing info label
                    const mode = this.camera.getRotateMode();
                    const label = `v10 ${mode} ${this._captureWidth}x${this._captureHeight}→${this._processWidth}x${this._processHeight}`;
                    this._grayPreviewCtx.fillStyle = 'rgba(0,0,0,0.6)';
                    this._grayPreviewCtx.fillRect(0, 0, w, 18);
                    this._grayPreviewCtx.fillStyle = '#0f8';
                    this._grayPreviewCtx.font = '14px monospace';
                    this._grayPreviewCtx.fillText(label, 4, 14);
                }

                // Camera warmup: skip first CAMERA_WARMUP_FRAMES frames to allow
                // auto-exposure and white-balance to settle before VIO begins.
                // 8th Wall pattern: loading-module.js FIRST_FRAME_DELAY=5.
                if (this._frameWarmupCount < CAMERA_WARMUP_FRAMES) {
                    this._frameWarmupCount++;
                    console.log(`[VIO] Camera warmup frame ${this._frameWarmupCount}/${CAMERA_WARMUP_FRAMES}, skipping`);
                    requestAnimationFrame(() => this.processLoop());
                    return;
                }

                // Downscale for VIO processing if processScale < 1.0
                let vioGray = gray;
                if (this._processScale < 1.0) {
                    vioGray = downsampleGray(gray,
                        this._captureWidth, this._captureHeight,
                        this._processWidth, this._processHeight);
                }

                // Skip degenerate baseline: actual capture interval too short for meaningful parallax
                const captureIntervalMs = (frameTimestamp - this._lastFrameTimestamp) * 1000;
                const degenerateBaseline = this._lastFrameTimestamp > 0 && captureIntervalMs < 20;

                if (!degenerateBaseline) {
                    // Flush all pending IMU right before the frame so the worker
                    // has the most up-to-date IMU data for pre-integration.
                    this._flushAndSendIMU();

                    // Send frame only — worker drains its internal IMU buffer
                    this.vio.sendFrame(vioGray, frameTimestamp);
                    this._lastFrameTimestamp = frameTimestamp;
                    this.totalFrameCount++;
                    this.frameCount++;

                    // Frame timing diagnostics (every 30 frames)
                    if (this.totalFrameCount % 30 === 0) {
                        const elapsed = (now - (this._fpsStartTime || now)) / 1000;
                        if (elapsed > 0 && this._fpsStartTime) {
                            const fps = 30 / elapsed;
                            console.log(`[VIO] Frame rate: ${fps.toFixed(1)}fps (${this.totalFrameCount} total)`);
                        }
                        this._fpsStartTime = now;
                    }
                }
            }
        }

        // Read latest result from worker (non-blocking)
        const result = this.vio.getLatestResult();

        // Update UI
        if (result) {
            if (this.featureEl) {
                this.featureEl.textContent = result.featureCount;
            }
            if (this.frameEl) {
                this.frameEl.textContent = this.totalFrameCount;
            }

            // Track VIO initialization state for adaptive frame rate
            if (result.initialized && !this._vioInitialized) {
                this._vioInitialized = true;
                this._initFrameCount = 0;
                console.log(`[VIO] ✓ Initialized! Switching to ${MIN_FRAME_INTERVAL_MS}ms frame interval (tracking mode)`);
            } else if (!result.initialized && this._vioInitialized) {
                // Lost tracking, revert to init mode
                this._vioInitialized = false;
                this._initFrameCount = 0;
                console.log(`[VIO] Tracking lost — reverting to ${INIT_FRAME_INTERVAL_MS}ms frame interval (init mode)`);
            }

            // Update status based on status code
            let statusMsg;
            if (result.statusCode === 1 || (!result.initialized && result.statusCode !== 0)) {
                this._initFrameCount++;
                statusMsg = `Initializing VIO... (${this._initFrameCount} frames, move phone slowly)`;
            } else {
                const statusMessages = {
                    0: 'Not configured',
                    2: 'Tracking',
                    3: 'Lost - recovering...',
                    4: 'Cooldown - stabilizing...',
                };
                statusMsg = statusMessages[result.statusCode] ||
                    (result.initialized ? 'Tracking' : 'Initializing VIO...');
            }
            this.updateStatus(statusMsg);

            // Update 3D rendering
            if (this.renderer) {
                if (result.pose) {
                    this.renderer.updateCameraPose(result.pose);
                }
                const mapData = this.vio.getMapPoints();
                if (mapData.count > 0) {
                    this.renderer.updateMapPoints(mapData.points, mapData.count);
                }
                this.renderer.render();
            }
        }

        // Update FPS + IMU rate
        const nowFps = performance.now();
        if (nowFps - this.lastFPSTime >= 1000) {
            this.fps = Math.round(this.frameCount / ((nowFps - this.lastFPSTime) / 1000));
            if (this.fpsEl) this.fpsEl.textContent = this.fps;
            if (this.imuRateEl) this.imuRateEl.textContent = Math.round(this.imu.getRate());
            this.lastFPSTime = nowFps;
            this.frameCount = 0;
        }

        // Update IMU overlay at ~10Hz for readability
        if (nowFps - this.lastImuUpdateTime >= 100) {
            this.updateIMUOverlay();
            this.lastImuUpdateTime = nowFps;
        }

        requestAnimationFrame(() => this.processLoop());
    }

    /** Update IMU sensor data overlay display */
    updateIMUOverlay() {
        const l = this.imu.latest;
        if (!l) return;
        const els = this.imuOverlayEls;
        if (els.accX) els.accX.textContent = l.acc_x.toFixed(2);
        if (els.accY) els.accY.textContent = l.acc_y.toFixed(2);
        if (els.accZ) els.accZ.textContent = l.acc_z.toFixed(2);
        if (els.gyroX) els.gyroX.textContent = l.gyro_x.toFixed(3);
        if (els.gyroY) els.gyroY.textContent = l.gyro_y.toFixed(3);
        if (els.gyroZ) els.gyroZ.textContent = l.gyro_z.toFixed(3);
    }

    /**
     * Handle tab visibility changes.
     * When tab goes background, sensors stop. On return, flush stale IMU data
     * and reset VIO to prevent divergence from large timestamp gaps.
     */
    _onVisibilityChange() {
        if (!this.running) return;

        if (document.hidden) {
            // Tab going to background: stop IMU flush to prevent stale accumulation
            this._stopIMUFlush();
            this._stopDeviceOrientationListener();
            console.log('[VIO] Tab hidden — IMU flush paused');
        } else {
            // Tab returning: clear stale IMU data and restart
            this.imu.flush();  // Discard any stale buffered data
            this._startIMUFlush();
            this._startDeviceOrientationListener();
            this.imuLogCount = 0;
            this._lastVideoTime = -1;  // Force next frame to be treated as new
            this._lastVIOFrameTime = 0;
            this._lastFrameTimestamp = 0;
            console.log('[VIO] Tab visible — IMU flush resumed, stale data cleared');
        }
    }

    /**
     * Handle screen orientation change.
     * Reconfigures VIO with updated camera-IMU extrinsic (R_ic).
     * IMU data stays in device body frame — only the camera image frame rotates.
     *
     * Also re-detects camera rotation mode because the browser may change
     * how drawImage() renders the video after orientation change (some browsers
     * auto-rotate pixel content, others don't).
     */
    async _onOrientationChange(info) {
        if (!this.running || !this._lastConfigParams) return;

        console.log(`[VIO] Orientation changed to ${info.type} — resetting VIO`);

        // Flush stale IMU data
        this.imu.flush();

        // Reset VIO state (visual feature tracks become invalid after rotation)
        this.vio.reset();
        if (this.renderer) {
            this.renderer.clear();
        }

        // Re-detect camera rotation mode for the new orientation.
        // Camera stream stays the same, but drawImage behavior may change.
        await this.camera.redetectOrientation();
        const newWidth = this.camera.width;
        const newHeight = this.camera.height;
        console.log(`[VIO] Camera after orientation change: ${newWidth}x${newHeight}, rotate=${this.camera.getRotateMode()}`);

        // Update grayscale preview canvas if dimensions changed
        if (this._grayPreviewCanvas &&
            (this._grayPreviewCanvas.width !== newWidth || this._grayPreviewCanvas.height !== newHeight)) {
            this._grayPreviewCanvas.width = newWidth;
            this._grayPreviewCanvas.height = newHeight;
            this._grayPreviewCtx = this._grayPreviewCanvas.getContext('2d');
            console.log(`[VIO] Preview canvas resized to ${newWidth}x${newHeight}`);
        }

        // Recalculate processing dimensions
        const config = VIO_CONFIGS[this.activeConfig];
        const processScale = config.processScale || 1.0;
        this._captureWidth = newWidth;
        this._captureHeight = newHeight;
        this._processWidth = Math.round(newWidth * processScale);
        this._processHeight = Math.round(newHeight * processScale);
        this._processScale = processScale;

        // Reconfigure with new R_ic and potentially new dimensions.
        // Focal length (fx/fy) is a lens property — stays the same.
        // Principal point (cx/cy) must match new processing dimensions.
        const params = { ...this._lastConfigParams, r_ic: info.r_ic,
            width: this._processWidth, height: this._processHeight,
            cx: this._processWidth / 2, cy: this._processHeight / 2 };
        const configured = await this.vio.configure(params);
        if (!configured) {
            console.error('[VIO] Reconfiguration failed after orientation change');
            return;
        }

        // Re-apply mobile solver params
        if (config.solver_time !== undefined) {
            await this.vio.setMobileParams(
                config.solver_time,
                config.num_iterations,
                config.max_features
            );
        }

        // Re-apply tracking params
        if (config.lk_window !== undefined) {
            await this.vio.setTrackingParams(
                config.lk_window,
                config.lk_pyramid || 4,
                config.min_dist || 20,
                config.f_edge_factor || 0.0
            );
        }

        this.imuLogCount = 0;
        this._lastVideoTime = -1;
        this._lastVIOFrameTime = 0;
        this._lastFrameTimestamp = 0;
        this._vioInitialized = false;
        this._initFrameCount = 0;
        console.log(`[VIO] Reconfigured for ${info.type}, process=${this._processWidth}x${this._processHeight}, R_ic:`, info.r_ic);
    }

    resetVIO() {
        this.vio.reset();
        if (this.renderer) {
            this.renderer.clear();
        }
        this.frameCount = 0;
        this.totalFrameCount = 0;
        this.imuLogCount = 0;
        this._lastVideoTime = -1;
        this._lastVIOFrameTime = 0;
        this._lastFrameTimestamp = 0;
        this._vioInitialized = false;
        this._initFrameCount = 0;
        this.updateStatus('VIO reset');
    }

    updateStatus(msg) {
        if (this.statusEl) {
            this.statusEl.textContent = msg;
        }
        console.log('[VIO]', msg);
    }

    stop() {
        this.running = false;
        this._stopIMUFlush();
        this._stopDeviceOrientationListener();
        this.orientation.stopListening();
        this.camera.stop();
        this.imu.stop();
        this.vio.dispose();
        // Remove window blur/focus listeners (8th Wall pauseonblur.js pattern)
        if (this._onWindowBlur) {
            window.removeEventListener('blur', this._onWindowBlur);
            this._onWindowBlur = null;
        }
        if (this._onWindowFocus) {
            window.removeEventListener('focus', this._onWindowFocus);
            this._onWindowFocus = null;
        }
    }
}

// Initialize on page load
const app = new App();
document.addEventListener('DOMContentLoaded', () => app.initialize());
