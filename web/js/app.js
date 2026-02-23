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
        acc_n: 0.1,       // accelerometer noise density - mobile MEMS ~2-3x EuRoC
        acc_w: 0.002,     // accelerometer random walk
        gyr_n: 0.01,      // gyroscope noise density - mobile MEMS ~2-3x EuRoC
        gyr_w: 0.0005,    // gyroscope random walk
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
        acc_n: 0.08,
        acc_w: 0.001,
        gyr_n: 0.005,
        gyr_w: 0.0001,
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
const IMU_FLUSH_INTERVAL_MS = 20;

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

            // Camera-IMU extrinsic rotation for smartphone (rear camera, portrait mode)
            // Camera frame (OpenCV): x=right, y=down, z=forward (into scene)
            // IMU frame (W3C DeviceMotion): x=right, y=up, z=out-of-screen
            // R_imu_camera = 180 deg rotation around x-axis
            const r_ic = [
                1,  0,  0,   // cam_x -> imu_x (same: right)
                0, -1,  0,   // cam_y -> -imu_y (down -> up)
                0,  0, -1    // cam_z -> -imu_z (forward -> out-of-screen)
            ];

            // Log coordinate system configuration for verification
            console.log('[VIO] Coordinate system configuration:');
            console.log('[VIO]   Camera frame (OpenCV): x=right, y=down, z=forward');
            console.log('[VIO]   IMU frame (DeviceMotion): x=right, y=up, z=out-of-screen');
            console.log('[VIO]   R_imu_camera:', r_ic);
            console.log('[VIO]   Expected: phone upright -> acc_y ~= +9.8 (gravity along +Y)');
            console.log('[VIO] Config profile:', this.activeConfig, config.label);

            // Log actual camera dimensions for orientation verification
            const video = this.camera.getVideoElement();
            console.log(`[VIO] Camera video dimensions: ${video.videoWidth}x${video.videoHeight}`);
            console.log(`[VIO] Camera capture dimensions: ${width}x${height}`);
            console.log(`[VIO] Orientation: ${width > height ? 'LANDSCAPE' : 'PORTRAIT'}`);

            // Handle landscape camera stream: try to lock portrait orientation
            if (width > height) {
                console.warn('[VIO] WARNING: Landscape video from mobile camera.');
                try {
                    await screen.orientation.lock('portrait');
                    console.log('[VIO] Screen orientation locked to portrait');
                } catch (_) {
                    console.warn('[VIO] Could not lock orientation. Using landscape dimensions.');
                }
            }

            // Estimate focal length using FOV or config factor
            const videoTrack = this.camera.getVideoTrack ? this.camera.getVideoTrack() : null;
            const fx = estimateFocalLength(videoTrack, width, height, config.focalLengthFactor);
            const fy = fx;

            // Camera-IMU translation offset
            // Typical smartphone: camera is ~2-3cm above IMU center
            const t_ic = [0, -0.02, 0];

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

            // Start frame processing loop
            this.processLoop();

        } catch (e) {
            this.updateStatus(`Error: ${e.message}`);
            console.error(e);
            this.startBtn.disabled = false;
        }
    }

    /**
     * Start independent IMU flush timer.
     * Flushes IMU data to the worker at a fixed interval, decoupled from rAF.
     */
    _startIMUFlush() {
        this._stopIMUFlush();
        this._imuFlushTimer = setInterval(() => {
            if (!this.running) return;
            const { data, count } = this.imu.flush();
            if (count > 0 && data) {
                // Log first few batches for diagnostics
                if (this.imuLogCount < 10) {
                    const r = {
                        acc_x: data[1], acc_y: data[2], acc_z: data[3],
                        gyro_x: data[4], gyro_y: data[5], gyro_z: data[6],
                    };
                    console.log(`[VIO] IMU batch #${this.imuLogCount}: ` +
                        `count=${count} ` +
                        `acc=(${r.acc_x.toFixed(3)}, ${r.acc_y.toFixed(3)}, ${r.acc_z.toFixed(3)}) ` +
                        `gyro=(${r.gyro_x.toFixed(4)}, ${r.gyro_y.toFixed(4)}, ${r.gyro_z.toFixed(4)}) ` +
                        `|acc|=${Math.sqrt(r.acc_x**2 + r.acc_y**2 + r.acc_z**2).toFixed(3)}`);
                    if (this.imuLogCount === 0) {
                        console.log(`[VIO] IMU sensor type: ${this.imu.getSensorType()}`);
                        console.log(`[VIO] Expected: |acc| ~= 9.81 (gravity)`);
                    }
                    this.imuLogCount++;
                }

                // Send IMU independently to worker (never blocked by workerBusy)
                this.vio.sendIMU(data, count);
            }
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

        const frameTimestamp = performance.now() / 1000.0;
        const gray = this.camera.captureGrayscale();
        if (gray) {
            // Send frame only (no IMU bundled — worker drains its internal buffer)
            this.vio.sendFrame(gray, frameTimestamp);
            this.totalFrameCount++;
            this.frameCount++;
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
        const now = performance.now();
        if (now - this.lastFPSTime >= 1000) {
            this.fps = Math.round(this.frameCount / ((now - this.lastFPSTime) / 1000));
            if (this.fpsEl) this.fpsEl.textContent = this.fps;
            if (this.imuRateEl) this.imuRateEl.textContent = Math.round(this.imu.getRate());
            this.lastFPSTime = now;
            this.frameCount = 0;
        }

        // Update IMU overlay at ~10Hz for readability
        if (now - this.lastImuUpdateTime >= 100) {
            this.updateIMUOverlay();
            this.lastImuUpdateTime = now;
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
            console.log('[VIO] Tab visible — IMU flush resumed, stale data cleared');
        }
    }

    resetVIO() {
        this.vio.reset();
        if (this.renderer) {
            this.renderer.clear();
        }
        this.frameCount = 0;
        this.totalFrameCount = 0;
        this.imuLogCount = 0;
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
        this.camera.stop();
        this.imu.stop();
        this.vio.dispose();
    }
}

// Initialize on page load
const app = new App();
document.addEventListener('DOMContentLoaded', () => app.initialize());
