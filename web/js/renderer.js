/**
 * Three.js trajectory and map point renderer.
 * Supports WebGL (default) and WebGPU (experimental) backends.
 *
 * Performance optimizations:
 * - Pre-allocated Float32Array buffers for trajectory and map points
 * - DynamicDrawUsage + needsUpdate + setDrawRange (zero per-frame allocation)
 * - Cached scratch Three.js objects for follow-cam (zero per-frame GC pressure)
 * - Fixed bounding spheres (skip O(n) computeBoundingSphere per frame)
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const MAX_TRAJECTORY_POINTS = 10000;
const MAX_MAP_POINTS = 2000;

export class Renderer {
    /**
     * Use Renderer.create() for async WebGPU support.
     * Direct construction defaults to WebGL.
     * @param {HTMLCanvasElement} canvas
     * @param {THREE.WebGLRenderer|object} [rendererInstance] - Pre-initialized renderer
     * @param {string} [backendName='webgl'] - 'webgl' or 'webgpu'
     */
    constructor(canvas, rendererInstance, backendName) {
        // Legacy direct construction: create WebGL renderer
        if (!rendererInstance) {
            rendererInstance = new THREE.WebGLRenderer({ canvas, antialias: true });
            rendererInstance.setSize(canvas.width, canvas.height);
            rendererInstance.setPixelRatio(Math.min(window.devicePixelRatio, 2));
            backendName = 'webgl';
        }

        this.backendName = backendName || 'webgl';
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1a1a2e);

        // Camera
        this.camera = new THREE.PerspectiveCamera(60, canvas.width / canvas.height, 0.01, 1000);
        this.camera.position.set(-3, 3, 3);
        this.camera.lookAt(0, 0, 0);

        // Renderer (pre-created or freshly constructed)
        this.renderer = rendererInstance;

        // OrbitControls for manual mouse interaction
        this.controls = new OrbitControls(this.camera, canvas);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.1;
        this.controls.target.set(0, 0, 0);

        // Lights
        this.scene.add(new THREE.AmbientLight(0xffffff, 0.6));
        const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
        dirLight.position.set(5, 10, 7);
        this.scene.add(dirLight);

        // VIO world root: VIO engine uses Z-up (gravity → -Z via g2R).
        // Three.js is Y-up, so rotate -90° around X to convert Z-up → Y-up.
        this.worldRoot = new THREE.Group();
        this.worldRoot.rotation.x = -Math.PI / 2;
        this.scene.add(this.worldRoot);

        // Grid on XY plane (Z-up world = XZ plane in Three.js after rotation)
        const grid = new THREE.GridHelper(20, 20, 0x444444, 0x333333);
        grid.rotation.x = Math.PI / 2;
        this.worldRoot.add(grid);

        // Origin axes
        this.worldRoot.add(new THREE.AxesHelper(1));

        // ── Pre-allocated trajectory buffer ──
        this._trajCount = 0;
        this._trajBuffer = new Float32Array(MAX_TRAJECTORY_POINTS * 3);
        const trajAttr = new THREE.BufferAttribute(this._trajBuffer, 3);
        trajAttr.setUsage(THREE.DynamicDrawUsage);
        this.trajectoryGeometry = new THREE.BufferGeometry();
        this.trajectoryGeometry.setAttribute('position', trajAttr);
        this.trajectoryGeometry.setDrawRange(0, 0);
        // Fixed bounding sphere avoids O(n) computeBoundingSphere() per frame
        this.trajectoryGeometry.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 500);
        this.trajectoryMaterial = new THREE.LineBasicMaterial({ color: 0x00ff88, linewidth: 2 });
        this.trajectoryLine = new THREE.Line(this.trajectoryGeometry, this.trajectoryMaterial);
        this.worldRoot.add(this.trajectoryLine);

        // Camera frustum
        this.frustumGroup = this._createFrustum();
        this.worldRoot.add(this.frustumGroup);

        // ── Pre-allocated map points buffer ──
        this._mapBuffer = new Float32Array(MAX_MAP_POINTS * 3);
        const mapAttr = new THREE.BufferAttribute(this._mapBuffer, 3);
        mapAttr.setUsage(THREE.DynamicDrawUsage);
        this.pointsGeometry = new THREE.BufferGeometry();
        this.pointsGeometry.setAttribute('position', mapAttr);
        this.pointsGeometry.setDrawRange(0, 0);
        this.pointsGeometry.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 500);
        this.pointsMaterial = new THREE.PointsMaterial({
            color: 0xff6644,
            size: 0.03,
            sizeAttenuation: true,
        });
        this.mapPoints = new THREE.Points(this.pointsGeometry, this.pointsMaterial);
        this.worldRoot.add(this.mapPoints);

        // Camera follow state
        this.followCamera = true;
        this._latestPoseMatrix = null;

        // ── Cached scratch objects for follow-cam (zero per-frame allocation) ──
        this._scratchMat4 = new THREE.Matrix4();
        this._scratchFrustumWorld = new THREE.Matrix4();
        this._scratchCamPos = new THREE.Vector3();
        this._scratchCamQuat = new THREE.Quaternion();
        this._scratchScale = new THREE.Vector3();
        this._scratchBehind = new THREE.Vector3();
        this._scratchUp = new THREE.Vector3();
        this._scratchTargetPos = new THREE.Vector3();
    }

    /**
     * Async factory: creates renderer with WebGL or WebGPU backend.
     * WebGPU requires browser support (navigator.gpu) and Three.js r165+.
     * Falls back to WebGL if WebGPU is unavailable or fails.
     *
     * @param {HTMLCanvasElement} canvas
     * @param {'webgl'|'webgpu'} [backend='webgl']
     * @returns {Promise<Renderer>}
     */
    static async create(canvas, backend = 'webgl') {
        let rendererInstance = null;
        let actualBackend = backend;

        if (backend === 'webgpu') {
            if (typeof navigator === 'undefined' || !navigator.gpu) {
                console.warn('[Renderer] WebGPU not available in this browser, falling back to WebGL.');
                actualBackend = 'webgl';
            } else {
                try {
                    const mod = await import('three/addons/renderers/webgpu/WebGPURenderer.js');
                    const WebGPURenderer = mod.default || mod.WebGPURenderer;
                    rendererInstance = new WebGPURenderer({ canvas, antialias: true });
                    await rendererInstance.init();
                    console.log('[Renderer] WebGPU backend initialized.');
                } catch (e) {
                    console.warn('[Renderer] WebGPU init failed, falling back to WebGL:', e.message);
                    rendererInstance = null;
                    actualBackend = 'webgl';
                }
            }
        }

        if (!rendererInstance) {
            rendererInstance = new THREE.WebGLRenderer({ canvas, antialias: true });
            actualBackend = 'webgl';
        }

        rendererInstance.setSize(canvas.width, canvas.height);
        rendererInstance.setPixelRatio(Math.min(window.devicePixelRatio, 2));

        return new Renderer(canvas, rendererInstance, actualBackend);
    }

    /** Create a wireframe camera frustum */
    _createFrustum() {
        const group = new THREE.Group();
        const s = 0.08; // frustum scale
        const vertices = [
            0, 0, 0,
            -s, -s * 0.75, s * 1.5,
            s, -s * 0.75, s * 1.5,
            s, s * 0.75, s * 1.5,
            -s, s * 0.75, s * 1.5,
        ];
        const indices = [
            0, 1, 0, 2, 0, 3, 0, 4,
            1, 2, 2, 3, 3, 4, 4, 1,
        ];
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
        geometry.setIndex(indices);
        const frustum = new THREE.LineSegments(
            geometry,
            new THREE.LineBasicMaterial({ color: 0xffff00 })
        );
        group.add(frustum);
        return group;
    }

    /**
     * Update camera pose from 4x4 transformation matrix.
     * Zero-allocation hot path: uses pre-allocated buffer and cached scratch objects.
     * @param {Float64Array} poseMatrix - 16 doubles, row-major 4x4 matrix
     */
    updateCameraPose(poseMatrix) {
        if (!poseMatrix || poseMatrix.length < 16) return;

        // Append to pre-allocated trajectory buffer (no new Float32Array)
        if (this._trajCount < MAX_TRAJECTORY_POINTS) {
            const base = this._trajCount * 3;
            this._trajBuffer[base] = poseMatrix[3];
            this._trajBuffer[base + 1] = poseMatrix[7];
            this._trajBuffer[base + 2] = poseMatrix[11];
            this._trajCount++;

            const attr = this.trajectoryGeometry.getAttribute('position');
            attr.needsUpdate = true;
            this.trajectoryGeometry.setDrawRange(0, this._trajCount);
        }

        // Update frustum position and orientation using cached Matrix4
        const m = this._scratchMat4;
        m.set(
            poseMatrix[0], poseMatrix[1], poseMatrix[2], poseMatrix[3],
            poseMatrix[4], poseMatrix[5], poseMatrix[6], poseMatrix[7],
            poseMatrix[8], poseMatrix[9], poseMatrix[10], poseMatrix[11],
            0, 0, 0, 1
        );
        this.frustumGroup.matrix.copy(m);
        this.frustumGroup.matrixAutoUpdate = false;
        this._latestPoseMatrix = m;

        // Follow camera using cached scratch objects (zero allocations)
        if (this.followCamera) {
            const frustumWorld = this._scratchFrustumWorld.multiplyMatrices(
                this.worldRoot.matrixWorld, m
            );

            const camPos = this._scratchCamPos;
            const camQuat = this._scratchCamQuat;
            frustumWorld.decompose(camPos, camQuat, this._scratchScale);

            // VIO camera looks along +Z; "behind" = -Z in camera local frame
            const behind = this._scratchBehind.set(0, 0, -1).applyQuaternion(camQuat);
            // Camera up = -Y local (image Y points down)
            const up = this._scratchUp.set(0, -1, 0).applyQuaternion(camQuat);

            // addScaledVector avoids mutating behind/up vectors
            const targetPos = this._scratchTargetPos.copy(camPos)
                .addScaledVector(behind, 1.5)    // followDist
                .addScaledVector(up, 0.5);        // followHeight

            this.camera.position.lerp(targetPos, 0.08);
            this.camera.lookAt(camPos);
            this.controls.target.copy(camPos);
        }
    }

    /**
     * Update map points using pre-allocated buffer.
     * @param {Float64Array} points - 3N doubles (x,y,z per point)
     * @param {number} count - Number of points
     */
    updateMapPoints(points, count) {
        if (!points || count === 0) return;

        const n = Math.min(count, MAX_MAP_POINTS);
        // Write directly into pre-allocated buffer (Float64 → Float32 downcast)
        for (let i = 0; i < n * 3; i++) {
            this._mapBuffer[i] = points[i];
        }

        const attr = this.pointsGeometry.getAttribute('position');
        attr.needsUpdate = true;
        this.pointsGeometry.setDrawRange(0, n);
    }

    /** Render one frame */
    render() {
        if (!this.followCamera) {
            this.controls.update();
        }
        this.renderer.render(this.scene, this.camera);
    }

    /** Handle window resize */
    resize(width, height) {
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }

    /** Clear trajectory and points */
    clear() {
        this._trajCount = 0;
        const trajAttr = this.trajectoryGeometry.getAttribute('position');
        trajAttr.needsUpdate = true;
        this.trajectoryGeometry.setDrawRange(0, 0);

        const mapAttr = this.pointsGeometry.getAttribute('position');
        mapAttr.needsUpdate = true;
        this.pointsGeometry.setDrawRange(0, 0);

        this._latestPoseMatrix = null;
    }
}
