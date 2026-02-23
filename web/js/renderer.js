/**
 * Three.js trajectory and map point renderer.
 * Displays camera trajectory, current camera frustum, and feature point cloud.
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export class Renderer {
    /**
     * @param {HTMLCanvasElement} canvas - Canvas for Three.js rendering
     */
    constructor(canvas) {
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1a1a2e);

        // Camera
        this.camera = new THREE.PerspectiveCamera(60, canvas.width / canvas.height, 0.01, 1000);
        this.camera.position.set(-3, 3, 3);
        this.camera.lookAt(0, 0, 0);

        // Renderer
        this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
        this.renderer.setSize(canvas.width, canvas.height);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

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

        // Trajectory line
        this.trajectoryPoints = [];
        this.trajectoryMaterial = new THREE.LineBasicMaterial({ color: 0x00ff88, linewidth: 2 });
        this.trajectoryGeometry = new THREE.BufferGeometry();
        this.trajectoryLine = new THREE.Line(this.trajectoryGeometry, this.trajectoryMaterial);
        this.worldRoot.add(this.trajectoryLine);

        // Camera frustum
        this.frustumGroup = this._createFrustum();
        this.worldRoot.add(this.frustumGroup);

        // Map points
        this.pointsMaterial = new THREE.PointsMaterial({
            color: 0xff6644,
            size: 0.03,
            sizeAttenuation: true,
        });
        this.pointsGeometry = new THREE.BufferGeometry();
        this.mapPoints = new THREE.Points(this.pointsGeometry, this.pointsMaterial);
        this.worldRoot.add(this.mapPoints);

        // Camera follow state
        this.followCamera = true;
        // Store latest VIO pose matrix for follow-cam computation
        this._latestPoseMatrix = null;
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
     * @param {Float64Array} poseMatrix - 16 doubles, row-major 4x4 matrix
     */
    updateCameraPose(poseMatrix) {
        if (!poseMatrix || poseMatrix.length < 16) return;

        const pos = new THREE.Vector3(poseMatrix[3], poseMatrix[7], poseMatrix[11]);

        // Update trajectory
        this.trajectoryPoints.push(pos.x, pos.y, pos.z);
        this.trajectoryGeometry.setAttribute(
            'position',
            new THREE.Float32BufferAttribute(this.trajectoryPoints, 3)
        );
        this.trajectoryGeometry.computeBoundingSphere();

        // Update frustum position and orientation (in worldRoot local space = VIO Z-up)
        const m = new THREE.Matrix4();
        m.set(
            poseMatrix[0], poseMatrix[1], poseMatrix[2], poseMatrix[3],
            poseMatrix[4], poseMatrix[5], poseMatrix[6], poseMatrix[7],
            poseMatrix[8], poseMatrix[9], poseMatrix[10], poseMatrix[11],
            0, 0, 0, 1
        );
        this.frustumGroup.matrix.copy(m);
        this.frustumGroup.matrixAutoUpdate = false;
        this._latestPoseMatrix = m;

        // Follow camera: view from behind the VIO camera, looking in its forward direction
        if (this.followCamera) {
            // Frustum's world matrix = worldRoot.matrixWorld * poseLocalMatrix
            const frustumWorld = new THREE.Matrix4().multiplyMatrices(this.worldRoot.matrixWorld, m);

            // Extract position and orientation in scene space
            const camPos = new THREE.Vector3();
            const camQuat = new THREE.Quaternion();
            frustumWorld.decompose(camPos, camQuat, new THREE.Vector3());

            // VIO camera looks along +Z in its local frame (camera convention).
            // "Behind" = negative Z direction in camera local frame.
            const behind = new THREE.Vector3(0, 0, -1).applyQuaternion(camQuat);
            // "Up" in camera local frame = -Y (image Y points down, so camera up = -Y local)
            const up = new THREE.Vector3(0, -1, 0).applyQuaternion(camQuat);

            // Position the viewer behind and slightly above the VIO camera
            const followDist = 1.5;
            const followHeight = 0.5;
            const targetPos = camPos.clone()
                .add(behind.multiplyScalar(followDist))
                .add(up.multiplyScalar(followHeight));

            this.camera.position.lerp(targetPos, 0.08);
            this.camera.lookAt(camPos);
            // Update orbit target to current camera position for smooth transition
            this.controls.target.copy(camPos);
        }
    }

    /**
     * Update map points.
     * @param {Float64Array} points - 3N doubles (x,y,z per point)
     * @param {number} count - Number of points
     */
    updateMapPoints(points, count) {
        if (!points || count === 0) return;

        const positions = new Float32Array(count * 3);
        for (let i = 0; i < count * 3; i++) {
            positions[i] = points[i];
        }
        this.pointsGeometry.setAttribute(
            'position',
            new THREE.Float32BufferAttribute(positions, 3)
        );
        this.pointsGeometry.computeBoundingSphere();
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
        this.trajectoryPoints = [];
        this.trajectoryGeometry.setAttribute(
            'position',
            new THREE.Float32BufferAttribute([], 3)
        );
        this.pointsGeometry.setAttribute(
            'position',
            new THREE.Float32BufferAttribute([], 3)
        );
        this._latestPoseMatrix = null;
    }
}
