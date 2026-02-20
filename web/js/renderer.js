/**
 * Three.js trajectory and map point renderer.
 * Displays camera trajectory, current camera frustum, and feature point cloud.
 */
import * as THREE from 'three';

export class Renderer {
    /**
     * @param {HTMLCanvasElement} canvas - Canvas for Three.js rendering
     */
    constructor(canvas) {
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1a1a2e);

        // Camera
        this.camera = new THREE.PerspectiveCamera(60, canvas.width / canvas.height, 0.01, 1000);
        this.camera.position.set(0, -2, 5);
        this.camera.lookAt(0, 0, 0);

        // Renderer
        this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
        this.renderer.setSize(canvas.width, canvas.height);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

        // Lights
        this.scene.add(new THREE.AmbientLight(0xffffff, 0.6));
        const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
        dirLight.position.set(5, 10, 7);
        this.scene.add(dirLight);

        // Grid
        const grid = new THREE.GridHelper(20, 20, 0x444444, 0x333333);
        grid.rotation.x = Math.PI / 2;
        this.scene.add(grid);

        // Origin axes
        this.scene.add(new THREE.AxesHelper(1));

        // Trajectory line
        this.trajectoryPoints = [];
        this.trajectoryMaterial = new THREE.LineBasicMaterial({ color: 0x00ff88, linewidth: 2 });
        this.trajectoryGeometry = new THREE.BufferGeometry();
        this.trajectoryLine = new THREE.Line(this.trajectoryGeometry, this.trajectoryMaterial);
        this.scene.add(this.trajectoryLine);

        // Camera frustum
        this.frustumGroup = this._createFrustum();
        this.scene.add(this.frustumGroup);

        // Map points
        this.pointsMaterial = new THREE.PointsMaterial({
            color: 0xff6644,
            size: 0.03,
            sizeAttenuation: true,
        });
        this.pointsGeometry = new THREE.BufferGeometry();
        this.mapPoints = new THREE.Points(this.pointsGeometry, this.pointsMaterial);
        this.scene.add(this.mapPoints);

        // Camera follow state
        this.followCamera = true;
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

        // Update frustum position and orientation
        const m = new THREE.Matrix4();
        m.set(
            poseMatrix[0], poseMatrix[1], poseMatrix[2], poseMatrix[3],
            poseMatrix[4], poseMatrix[5], poseMatrix[6], poseMatrix[7],
            poseMatrix[8], poseMatrix[9], poseMatrix[10], poseMatrix[11],
            0, 0, 0, 1
        );
        this.frustumGroup.matrix.copy(m);
        this.frustumGroup.matrixAutoUpdate = false;

        // Follow camera
        if (this.followCamera) {
            const offset = new THREE.Vector3(0, -1, 3);
            offset.applyMatrix4(m);
            this.camera.position.lerp(offset, 0.1);
            this.camera.lookAt(pos);
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
    }
}
