// A-Frame + the slam_refact xr-slam chunk: the `xrweb` component (part of the deployed xr.js) runs the camera and the chunk; this component
// places a cube where you tap using XR8.XrController.hitTest. Same idea as assets/references/xr_reference/official/aframe-world-effects-example
// (tap-place.js), without XRExtras / landing-page (not available locally; the sample must not need the network).
const probe = window.XrSampleProbe;
const hud = () => document.getElementById('hud');

AFRAME.registerComponent('xr-hit-place', {
  init() {
    const scene = this.el;
    // World points are what hitTest FEATURE_POINT reads; ask the controller for them as the reference runtime examples do.
    const configure = () => XR8.XrController.configure({ enableWorldPoints: true });
    window.XR8 ? configure() : window.addEventListener('xrloaded', configure, { once: true });

    scene.addEventListener('renderstart', () => {
      scene.canvas.addEventListener('click', (event) => {
        const cube = document.getElementById('cube');
        const hits = XR8.XrController.hitTest(event.clientX / window.innerWidth, event.clientY / window.innerHeight, ['FEATURE_POINT']);
        // distance 0 is the engine's answer when no tracked feature lies near that ray (the "hit" is the camera position): not a surface.
        if (!hits.length || !(hits[0].distance > 0)) { hud().textContent = 'No surface estimate under the tap yet; move the phone and tap again.'; return; }
        // hits[0].rotation is (0, 0, 0, 0) (the engine fills only type, position and distance), not an orientation: do not use it as one.
        const { position, distance } = hits[0];
        cube.setAttribute('position', { x: position.x, y: position.y, z: position.z });
        cube.setAttribute('visible', true);
        hud().textContent = `Cube placed ${distance.toFixed(2)} units away. Tap again to move it.`;
      });
      hud().textContent = 'Tap the screen to place a cube.';
    });

    // Test instrumentation (inert without ?probe=1): appended after xrweb's own modules, so it runs after the camera was updated.
    if (probe && probe.enabled) {
      scene.addEventListener('renderstart', () => {
        const add = () => XR8.addCameraPipelineModule(probe.createModule({
          kind: 'aframe',
          getCamera: () => { const cam = scene.camera; return { pose: cam.el.object3D, projection: cam, world: cam.el.object3D }; },
          getCube: () => document.getElementById('cube').object3D,
          skipUpdates: 5,  // xraframe leaves the camera alone for the first 5 updates (xr.js: framesSinceLoad <= 5)
        }));
        window.XR8 ? add() : window.addEventListener('xrloaded', add, { once: true });
      });
    }
  },
});
