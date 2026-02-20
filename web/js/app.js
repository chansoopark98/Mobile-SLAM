/**
 * Main application - combines Camera, IMU, VIO, and Renderer.
 */
import { VIOWrapper } from './vio-wrapper.js';
import { Camera } from './camera.js';
import { IMU } from './imu.js';
import { Renderer } from './renderer.js';

class App {
    constructor() {
        this.vio = new VIOWrapper();
        this.camera = new Camera();
        this.imu = new IMU();
        this.renderer = null;

        this.running = false;
        this.frameCount = 0;
        this.lastFPSTime = 0;
        this.fps = 0;

        // UI elements
        this.statusEl = null;
        this.fpsEl = null;
        this.featureEl = null;
        this.frameEl = null;
        this.startBtn = null;
        this.resetBtn = null;
    }

    async initialize() {
        this.statusEl = document.getElementById('status');
        this.fpsEl = document.getElementById('fps');
        this.featureEl = document.getElementById('features');
        this.frameEl = document.getElementById('frame-count');
        this.startBtn = document.getElementById('btn-start');
        this.resetBtn = document.getElementById('btn-reset');

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

            // Configure VIO with typical mobile camera parameters
            // Camera-IMU extrinsic rotation for smartphone (rear camera, portrait mode)
            // Camera frame (OpenCV): x=right, y=down, z=forward (into scene)
            // IMU frame (DeviceMotion): x=right, y=up, z=out-of-screen
            // R_imu_camera = 180° rotation around x-axis
            const r_ic = [
                1,  0,  0,   // cam_x → imu_x (same: right)
                0, -1,  0,   // cam_y → -imu_y (down → up)
                0,  0, -1    // cam_z → -imu_z (forward → out-of-screen)
            ];

            // Mobile phone IMU noise parameters (higher than research-grade sensors)
            const configured = this.vio.configure({
                width: width,
                height: height,
                fx: width * 0.8,  // Approximate focal length
                fy: width * 0.8,
                cx: width / 2,
                cy: height / 2,
                modelType: 0, // PINHOLE
                r_ic: r_ic,
                acc_n: 0.2,
                acc_w: 0.02,
                gyr_n: 0.02,
                gyr_w: 0.002,
                g_norm: 9.81,
            });

            if (!configured) {
                this.updateStatus('VIO configuration failed');
                return;
            }

            // Request IMU permission and start
            if (IMU.isAvailable()) {
                const granted = await this.imu.requestPermission();
                if (granted) {
                    this.imu.start();
                    this.updateStatus('Camera + IMU active');
                } else {
                    this.updateStatus('Camera active (no IMU permission)');
                }
            } else {
                this.updateStatus('Camera active (no IMU sensor)');
            }

            this.running = true;
            this.resetBtn.disabled = false;
            this.lastFPSTime = performance.now();
            this.processLoop();

        } catch (e) {
            this.updateStatus(`Error: ${e.message}`);
            console.error(e);
            this.startBtn.disabled = false;
        }
    }

    processLoop() {
        if (!this.running) return;

        const frameTimestamp = performance.now() / 1000.0;
        const gray = this.camera.captureGrayscale();
        if (gray) {
            const imuReadings = this.imu.flush();
            const result = this.vio.processFrame(gray, imuReadings, frameTimestamp);

            this.frameCount++;

            // Update UI
            if (this.featureEl) {
                this.featureEl.textContent = result.featureCount;
            }
            if (this.frameEl) {
                this.frameEl.textContent = this.frameCount;
            }

            // Update FPS
            const now = performance.now();
            if (now - this.lastFPSTime >= 1000) {
                this.fps = Math.round(this.frameCount / ((now - this.lastFPSTime) / 1000));
                if (this.fpsEl) this.fpsEl.textContent = this.fps;
                this.lastFPSTime = now;
                this.frameCount = 0;
            }

            // Update status
            if (result.initialized) {
                this.updateStatus('VIO initialized - tracking');
            } else {
                this.updateStatus('Initializing VIO...');
            }

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

        requestAnimationFrame(() => this.processLoop());
    }

    resetVIO() {
        this.vio.reset();
        if (this.renderer) {
            this.renderer.clear();
        }
        this.frameCount = 0;
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
        this.camera.stop();
        this.imu.stop();
        this.vio.dispose();
    }
}

// Initialize on page load
const app = new App();
document.addEventListener('DOMContentLoaded', () => app.initialize());
