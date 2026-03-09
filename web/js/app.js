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
import { VIOWrapper } from './vio-wrapper.js?v=5';
import { Camera } from './camera.js?v=5';
import { IMU } from './imu.js?v=5';
import { Renderer } from './renderer.js?v=5';
import { OrientationHandler } from './orientation.js?v=5';

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
        acc_n: 0.08,      // accelerometer noise density (m/s²/√Hz)
                          //   Measured: Pixel 7 ~0.088, iPhone XR ~0.094
                          //   Ref: Kalibr wiki recommends 10-20x inflate over datasheet
        acc_w: 0.0004,    // accelerometer random walk — reduced from 0.002
                          //   TUM VI BMI160: 0.0004. Previous 0.002 was 5x too high,
                          //   causing unstable bias estimation → pre-integration divergence
        gyr_n: 0.01,      // gyroscope noise density (rad/s/√Hz)
                          //   Measured: Pixel 7 ~0.008, iPhone XR ~0.027
        gyr_w: 0.0001,    // gyroscope random walk — reduced from 0.002
                          //   TUM VI BMI160: 2e-5. Previous 0.002 was 100x too high,
                          //   destabilizing gyro bias tracking → rotational drift
        g_norm: 9.81,
        focalLengthFactor: null,  // null → estimate from FOV (see estimateFocalLength)
        modelType: 0,  // PINHOLE
        solver_time: 0.04,
        num_iterations: 8,
        max_features: 120,
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
        modelType: 0,
        solver_time: 0.06,
        num_iterations: 10,
        max_features: 150,
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
        modelType: 0,
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
 * Minimum interval between VIO frame processing (ms).
 * At 60Hz DeviceMotion IMU, processing at 20fps yields ~3 IMU readings
 * per frame — the minimum for reasonable VINS-Mono pre-integration.
 * Higher fps → fewer IMU samples → weaker pre-integration → divergence.
 */
const MIN_FRAME_INTERVAL_MS = 50;

/**
 * Frame interval during VIO initialization phase (ms).
 *
 * VINS-Mono initialization requires:
 *   - Sufficient IMU pre-integration quality (needs 4+ readings/frame)
 *   - Enough parallax across the 10-frame sliding window (need 30px)
 *   - IMU excitation (gravity vector variance > 0.25)
 *
 * At 45Hz IMU (typical mobile):
 *   - 50ms interval (20fps): ~2.3 IMU/frame, 0.5s window → often fails
 *   - 100ms interval (10fps): ~4.5 IMU/frame, 1.0s window → much better
 *
 * Wider window = more time for parallax. More IMU/frame = better
 * pre-integration. Both dramatically improve initialization success rate.
 * After initialization, we switch to MIN_FRAME_INTERVAL_MS for tracking.
 */
const INIT_FRAME_INTERVAL_MS = 100;

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
        const fx = width * configFactor;
        console.log(`[VIO] Focal length from config factor ${configFactor}: fx=${fx.toFixed(1)}`);
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

class App {
    constructor() {
        this.vio = new VIOWrapper();
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

        // Active config profile
        this.activeConfig = 'mobile_default';

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

        // Frame deduplication: skip if video.currentTime hasn't changed
        this._lastVideoTime = -1;
        // Frame rate limiting: ensure enough IMU between VIO frames
        this._lastVIOFrameTime = 0;

        // VIO initialization state (for adaptive frame rate)
        this._vioInitialized = false;
        this._initFrameCount = 0;

        // Grayscale preview canvas (debug visualization)
        this._grayPreviewCanvas = null;
        this._grayPreviewCtx = null;

        // Stored config for reconfiguration on orientation change
        this._lastConfigParams = null;
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
            await this.vio.load('/vio_engine.js');
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
            const isPortrait = orientationType.startsWith('portrait');

            // Initialize camera with orientation hint — camera module will
            // auto-rotate the frame if the device gives landscape pixels in portrait mode.
            const { width, height, rotated } = await this.camera.initialize(640, 480, isPortrait);
            this.updateStatus(`Camera: ${width}x${height}${rotated ? ' (rotated)' : ''}`);

            // Display video feed
            const videoContainer = document.getElementById('video-container');
            if (videoContainer) {
                const video = this.camera.getVideoElement();
                video.style.width = '100%';
                videoContainer.appendChild(video);
            }

            // Setup grayscale preview canvas
            this._grayPreviewCanvas = document.getElementById('gray-preview');
            if (this._grayPreviewCanvas) {
                this._grayPreviewCanvas.width = width;
                this._grayPreviewCanvas.height = height;
                this._grayPreviewCanvas.style.display = 'block';
                this._grayPreviewCtx = this._grayPreviewCanvas.getContext('2d');
            }

            // Get active config profile
            const config = VIO_CONFIGS[this.activeConfig];

            // Get orientation-aware camera-IMU extrinsic rotation (R_ic).
            // IMU frame (W3C DeviceMotion): fixed to device body
            //   X_d = right edge, Y_d = top edge, Z_d = out of screen
            // Camera frame (OpenCV): rotates with screen orientation
            //   X_c = right in image, Y_c = down in image, Z_c = forward
            // R_ic transforms camera→IMU: v_imu = R_ic * v_cam
            const r_ic = this.orientation.getRIC();

            // Log coordinate system configuration
            const video = this.camera.getVideoElement();
            console.log('[VIO] ═══════ Configuration Summary ═══════');
            console.log(`[VIO] Screen orientation: ${orientationType}`);
            console.log(`[VIO] Camera native: ${video.videoWidth}x${video.videoHeight}`);
            console.log(`[VIO] Camera output: ${width}x${height} (rotated=${rotated})`);
            console.log(`[VIO] Image orientation: ${width > height ? 'LANDSCAPE' : 'PORTRAIT'}`);
            console.log(`[VIO] Config profile: ${this.activeConfig} (${config.label})`);
            console.log(`[VIO] R_ic: [${r_ic.map(v => v.toFixed(1)).join(', ')}]`);
            console.log('[VIO] Camera frame (OpenCV): x=right, y=down, z=forward');
            console.log('[VIO] IMU frame (DeviceMotion): x=right-edge, y=top-edge, z=out-of-screen');
            console.log('[VIO] VIO body frame: x=right, y=forward, z=up');
            console.log('[VIO] Expected portrait upright → acc_z_vio ≈ +9.81 (gravity on Z)');

            // Estimate focal length using FOV or config factor
            const videoTrack = this.camera.getVideoTrack ? this.camera.getVideoTrack() : null;
            let { fx, method: fxMethod } = estimateFocalLength(videoTrack, width, height, config.focalLengthFactor);
            const fxValidation = validateFocalLength(fx, width, height);
            if (!fxValidation.valid) {
                fx = fxValidation.fx;
                fxMethod += ' (corrected)';
            }
            const fy = fx;

            // Camera-IMU translation offset in VIO body frame
            // VIO body frame: X=right, Y=forward, Z=up
            // Camera ~2cm below IMU center along Z_vio (gravity direction)
            const t_ic = [0, 0, -0.02];

            const configured = await this.vio.configure({
                width: width,
                height: height,
                fx: fx,
                fy: fy,
                cx: width / 2,
                cy: height / 2,
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
                width, height, fx, fy,
                cx: width / 2, cy: height / 2,
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

            console.log(`[VIO] ═══════ VIO Engine Configured ═══════`);
            console.log(`[VIO]   Image: ${width}x${height}`);
            console.log(`[VIO]   Focal: fx=${fx.toFixed(1)}, fy=${fy.toFixed(1)} (${fxMethod})`);
            console.log(`[VIO]   Principal: cx=${(width/2).toFixed(1)}, cy=${(height/2).toFixed(1)}`);
            console.log(`[VIO]   t_ic: [${t_ic}]`);
            console.log(`[VIO]   IMU noise: acc_n=${config.acc_n}, acc_w=${config.acc_w}, gyr_n=${config.gyr_n}, gyr_w=${config.gyr_w}`);
            console.log(`[VIO]   Solver: time=${config.solver_time}s, iter=${config.num_iterations}, features=${config.max_features}`);

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

            // Start listening for orientation changes (reconfigures VIO if phone rotates)
            this.orientation.startListening((info) => this._onOrientationChange(info));

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

            const frameTimestamp = now / 1000.0;
            const gray = this.camera.captureGrayscale();
            if (gray) {
                // Draw grayscale preview (debug: verify capture quality + rotation)
                if (this._grayPreviewCtx) {
                    const w = this.camera.width;
                    const h = this.camera.height;
                    const imgData = this._grayPreviewCtx.createImageData(w, h);
                    const d = imgData.data;
                    for (let i = 0, j = 0; i < gray.length; i++, j += 4) {
                        d[j] = d[j + 1] = d[j + 2] = gray[i];
                        d[j + 3] = 255;
                    }
                    this._grayPreviewCtx.putImageData(imgData, 0, 0);

                    // Draw version + rotation mode label for cache verification
                    const mode = this.camera.getRotateMode();
                    const label = `v5 ${mode} ${w}x${h}`;
                    this._grayPreviewCtx.fillStyle = 'rgba(0,0,0,0.6)';
                    this._grayPreviewCtx.fillRect(0, 0, w, 18);
                    this._grayPreviewCtx.fillStyle = '#0f8';
                    this._grayPreviewCtx.font = '14px monospace';
                    this._grayPreviewCtx.fillText(label, 4, 14);
                }

                // Flush all pending IMU right before the frame so the worker
                // has the most up-to-date IMU data for pre-integration.
                this._flushAndSendIMU();

                // Send frame only — worker drains its internal IMU buffer
                this.vio.sendFrame(gray, frameTimestamp);
                this.totalFrameCount++;
                this.frameCount++;
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
            console.log('[VIO] Tab hidden — IMU flush paused');
        } else {
            // Tab returning: clear stale IMU data and restart
            this.imu.flush();  // Discard any stale buffered data
            this._startIMUFlush();
            this.imuLogCount = 0;
            this._lastVideoTime = -1;  // Force next frame to be treated as new
            this._lastVIOFrameTime = 0;
            console.log('[VIO] Tab visible — IMU flush resumed, stale data cleared');
        }
    }

    /**
     * Handle screen orientation change.
     * Reconfigures VIO with updated camera-IMU extrinsic (R_ic).
     * IMU data stays in device body frame — only the camera image frame rotates.
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

        // Reconfigure with new R_ic for the new orientation
        const params = { ...this._lastConfigParams, r_ic: info.r_ic };
        const configured = await this.vio.configure(params);
        if (!configured) {
            console.error('[VIO] Reconfiguration failed after orientation change');
            return;
        }

        // Re-apply mobile solver params
        const config = VIO_CONFIGS[this.activeConfig];
        if (config.solver_time !== undefined) {
            await this.vio.setMobileParams(
                config.solver_time,
                config.num_iterations,
                config.max_features
            );
        }

        this.imuLogCount = 0;
        this._lastVideoTime = -1;
        this._lastVIOFrameTime = 0;
        this._vioInitialized = false;
        this._initFrameCount = 0;
        console.log(`[VIO] Reconfigured for ${info.type}, R_ic:`, info.r_ic);
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
        this.orientation.stopListening();
        this.camera.stop();
        this.imu.stop();
        this.vio.dispose();
    }
}

// Initialize on page load
const app = new App();
document.addEventListener('DOMContentLoaded', () => app.initialize());
