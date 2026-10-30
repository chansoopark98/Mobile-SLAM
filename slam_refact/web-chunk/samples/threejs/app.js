// three.js + the slam_refact xr-slam chunk: a camera feed background with a cube that is placed where you tap, using XR8.XrController.hitTest.
// Same structure as the reference runtime threejs-world-effects-example (assets/references/xr_reference/official/threejs-world-effects-example/src/app.js),
// without XRExtras / landing-page (they are not available locally and this sample must not need the network).
import * as THREE from '/three/three.module.min.js';

window.THREE = THREE;  // XR8.Threejs reads window.THREE
const probe = window.XrSampleProbe;
const hud = document.getElementById('hud');
const canvas = document.getElementById('camerafeed');

let cube = null;

// Tap -> the 3D point the engine estimates under the tap (FEATURE_POINT: depth from the tracked feature points near that ray).
const onTap = (event) => {
  const x = event.clientX / window.innerWidth;
  const y = event.clientY / window.innerHeight;
  const hits = XR8.XrController.hitTest(x, y, ['FEATURE_POINT']);
  // distance 0 is the engine's answer when no tracked feature lies near that ray (the "hit" is the camera position): not a surface.
  if (!hits.length || !(hits[0].distance > 0)) { hud.textContent = 'No surface estimate under the tap yet; move the phone and tap again.'; return; }
  // hits[0].rotation is (0, 0, 0, 0) (the engine fills only type, position and distance), not an orientation: do not use it as one.
  const { position, distance } = hits[0];
  cube.position.set(position.x, position.y, position.z);
  cube.visible = true;
  hud.textContent = `Cube placed ${distance.toFixed(2)} units away. Tap again to move it.`;
};

const sceneModule = () => ({
  name: 'threejs-tap-place',
  onStart: () => {
    const { scene, camera, renderer } = XR8.Threejs.xrScene();
    renderer.setClearColor(0x000000, 0);

    // A translucent floor at y = 0 (the engine's origin height is the camera height above it) and the cube to place. Unlit so its colour is exact.
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(40, 40).rotateX(-Math.PI / 2),
      new THREE.MeshBasicMaterial({ color: 0x4488aa, transparent: true, opacity: 0.2, side: THREE.DoubleSide }));
    scene.add(floor);
    cube = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.3, 0.3), new THREE.MeshBasicMaterial({ color: 0xff00ff }));
    cube.visible = false;
    scene.add(cube);

    // Start the 6DoF origin at the scene camera pose (height 2 above the floor, as in the official sample).
    camera.position.set(0, 2, 2);
    XR8.XrController.updateCameraProjectionMatrix({ origin: camera.position, facing: camera.quaternion });
    canvas.addEventListener('click', onTap);
    hud.textContent = 'Tap the screen to place a cube.';
  },
  onUpdate: ({ processCpuResult }) => {
    const r = processCpuResult.reality;
    if (r && !cube.visible && (r.worldPoints || []).length === 0) hud.textContent = `Tracking ${r.trackingStatus}; looking for features...`;
  },
});

const fitCanvas = () => {
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(window.innerWidth * dpr);
  canvas.height = Math.round(window.innerHeight * dpr);
};

const onxrloaded = () => {
  fitCanvas();
  window.addEventListener('resize', fitCanvas);
  XR8.XrController.configure({ enableWorldPoints: true });
  const modules = [
    XR8.GlTextureRenderer.pipelineModule(),  // draws the camera feed
    XR8.Threejs.pipelineModule(),            // three.js scene + camera + renderer, camera follows the reality pose
    XR8.XrController.pipelineModule(),       // the slam_refact chunk (reality, hitTest)
    sceneModule(),
  ];
  const probeModule = probe && probe.createModule({
    kind: 'threejs',
    getCamera: () => { const camera = XR8.Threejs.xrScene().camera; return { pose: camera, projection: camera, world: camera }; },
    getCube: () => cube,
    skipUpdates: 0,
  });
  if (probeModule) modules.push(probeModule);
  XR8.addCameraPipelineModules(modules);
  XR8.run({ canvas });
};

window.XR8 ? onxrloaded() : window.addEventListener('xrloaded', onxrloaded);
