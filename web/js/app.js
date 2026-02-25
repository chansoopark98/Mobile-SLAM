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
import { VIOWrapper } from './vio-wrapper.js';
import { Camera } from './camera.js';
import { IMU } from './imu.js';
import { Renderer } from './renderer.js';
import { OrientationHandler } from './orientation.js';

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
    mobile_default: {
        label: 'Mobile Default',
        acc_n: 0.2,       // accelerometer noise density - mobile MEMS ~5x EuRoC
        acc_w: 0.004,     // accelerometer random walk - relaxed for mobile drift
        gyr_n: 0.03,      // gyroscope noise density - mobile MEMS ~7x EuRoC
        gyr_w: 0.004,     // gyroscope random walk - CRITICAL: must be large enough
                          //   for optimizer to track mobile gyro bias drift.
                          //   Old value 0.0005 was 20x too tight → divergence.
        g_norm: 9.81,
        focalLengthFactor: 0.8,  // fx = fy = width * factor
        modelType: 0,  // PINHOLE
        solver_time: 0.04,
        num_iterations: 8,
        max_features: 120,
    },
    // Tuned for high-end phones (iPhone 14+, Pixel 7+, Galaxy S23+)
    mobile_highend: {
        label: 'Mobile High-end',
        acc_n: 0.1,
        acc_w: 0.002,
        gyr_n: 0.015,
        gyr_w: 0.002,
        g_norm: 9.81,
        focalLengthFactor: 0.85,
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
 * Estimate focal length from camera FOV or track settings.
 * Priority: MediaStreamTrack.getSettings() > config factor > default factor
 * @param {MediaStreamTrack} videoTrack - Camera video track
 * @param {number} width - Image width in pixels
 * @param {number} height - Image height in pixels
 * @param {number|null} configFactor - focalLengthFactor from config profile
 * @returns {number} Estimated focal length in pixels
 */
function estimateFocalLength(videoTrack, width, height, configFactor) {
    // Try to get FOV from MediaStreamTrack settings (Chrome Android 100+)
    if (videoTrack) {
        try {
            const settings = videoTrack.getSettings();
            // Some browsers expose fieldOfView or related capabilities
            // Check for explicit FOV (future-proof, not yet widely supported)
            if (settings.fieldOfView) {
                const fovRad = settings.fieldOfView * Math.PI / 180;
                const fx = (width / 2) / Math.tan(fovRad / 2);
                console.log(`[VIO] Focal length from track FOV: ${settings.fieldOfView}° → fx=${fx.toFixed(1)}`);
                return fx;
            }

            // Check zoom level - if available, adjust factor
            if (settings.zoom && settings.zoom > 1) {
                const baseFx = width * (configFactor || 0.8);
                const fx = baseFx * settings.zoom;
                console.log(`[VIO] Focal length adjusted for zoom ${settings.zoom}x: fx=${fx.toFixed(1)}`);
                return fx;
            }
        } catch (_) {
            // getSettings/getCapabilities not available
        }
    }

    // Use config factor if specified
    if (configFactor) {
        return width * configFactor;
    }

    // Default: typical mobile rear camera ~65° horizontal FOV
    // fx = (width/2) / tan(65°/2) ≈ width * 0.87
    const defaultFovDeg = 65;
    const fx = (width / 2) / Math.tan((defaultFovDeg * Math.PI / 180) / 2);
    console.log(`[VIO] Focal length from default FOV ${defaultFovDeg}°: fx=${fx.toFixed(1)}`);
    return fx;
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
            const { width, height } = await this.camera.initialize(640, 480);
            this.updateStatus(`Camera: ${width}x${height}`);

            // Display video feed
            const videoContainer = document.getElementById('video-container');
            if (videoContainer) {
                const video = this.camera.getVideoElement();
                video.style.width = '100%';
                videoContainer.appendChild(video);
            }

            // Get active config profile
            const config = VIO_CONFIGS[this.activeConfig];

            // Try to lock portrait orientation (simplest for VIO)
            await this.orientation.tryLockPortrait();

            // Get orientation-aware camera-IMU extrinsic rotation (R_ic).
            // IMU frame (W3C DeviceMotion): fixed to device body
            //   X_d = right edge, Y_d = top edge, Z_d = out of screen
            // Camera frame (OpenCV): rotates with screen orientation
            //   X_c = right in image, Y_c = down in image, Z_c = forward
            // R_ic transforms camera→IMU: v_imu = R_ic * v_cam
            const orientationType = this.orientation.getType();
            const r_ic = this.orientation.getRIC();

            // Log coordinate system configuration
            const video = this.camera.getVideoElement();
            console.log('[VIO] Coordinate system configuration:');
            console.log('[VIO]   Screen orientation:', orientationType);
            console.log('[VIO]   Camera frame (OpenCV): x=right, y=down, z=forward');
            console.log('[VIO]   IMU frame (DeviceMotion): x=right-edge, y=top-edge, z=out-of-screen');
            console.log('[VIO]   R_imu_camera:', r_ic);
            console.log('[VIO]   Expected: phone upright portrait → acc_y ≈ +9.8');
            console.log('[VIO]   Expected: phone flat face-up → acc_z ≈ +9.8');
            console.log('[VIO] Config profile:', this.activeConfig, config.label);
            console.log(`[VIO] Camera video: ${video.videoWidth}x${video.videoHeight}`);
            console.log(`[VIO] Camera capture: ${width}x${height}`);
            console.log(`[VIO] Image orientation: ${width > height ? 'LANDSCAPE' : 'PORTRAIT'}`);

            // Estimate focal length using FOV or config factor
            const videoTrack = this.camera.getVideoTrack ? this.camera.getVideoTrack() : null;
            const fx = estimateFocalLength(videoTrack, width, height, config.focalLengthFactor);
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

            console.log(`[VIO] Configured: ${width}x${height}, fx=${fx.toFixed(1)}, fy=${fy.toFixed(1)}, ` +
                         `t_ic=[${t_ic}], acc_n=${config.acc_n}, gyr_n=${config.gyr_n}` +
                         (config.solver_time ? `, solver_time=${config.solver_time}` : ''));

            // Request IMU permission and start
            if (IMU.isAvailable()) {
                const granted = await this.imu.requestPermission();
                if (granted) {
                    this.imu.start(100);  // Request 100Hz
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
            console.log(`[VIO] IMU batch #${this.imuLogCount} (VIO frame): ` +
                `count=${count} ` +
                `acc=(${r.acc_x.toFixed(3)}, ${r.acc_y.toFixed(3)}, ${r.acc_z.toFixed(3)}) ` +
                `gyro=(${r.gyro_x.toFixed(4)}, ${r.gyro_y.toFixed(4)}, ${r.gyro_z.toFixed(4)}) ` +
                `|acc|=${Math.sqrt(r.acc_x**2 + r.acc_y**2 + r.acc_z**2).toFixed(3)}`);
            if (this.imuLogCount === 0) {
                console.log(`[VIO] IMU sensor type: ${this.imu.getSensorType()}`);
                console.log(`[VIO] Expected: portrait upright → acc_z ~= +9.81 (gravity on Z)`);
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
        const video = this.camera.getVideoElement();
        const videoTime = video ? video.currentTime : -1;
        const isNewFrame = videoTime > 0 && videoTime !== this._lastVideoTime;
        const enoughTime = (now - this._lastVIOFrameTime) >= MIN_FRAME_INTERVAL_MS;

        if (isNewFrame && enoughTime) {
            this._lastVideoTime = videoTime;
            this._lastVIOFrameTime = now;

            const frameTimestamp = now / 1000.0;
            const gray = this.camera.captureGrayscale();
            if (gray) {
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

            // Update status based on status code
            const statusMessages = {
                0: 'Not configured',
                1: 'Initializing VIO...',
                2: 'Tracking',
                3: 'Lost - recovering...',
                4: 'Cooldown - stabilizing...',
            };
            const statusMsg = statusMessages[result.statusCode] ||
                (result.initialized ? 'Tracking' : 'Initializing VIO...');
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
