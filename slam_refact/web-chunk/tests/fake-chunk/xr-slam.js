// FAKE xr-slam.js chunk: the null-run N0 of the US-010 sample smoke (preregistration us010-samples.v1.md section 2). It has the XrController
// surface xr.js needs but no engine: a fixed pose that never moves, no world points, hitTest always empty. The smoke must FAIL on it (S3, S6, S10,
// S11, S13). Never served by the samples; only run-sample-smoke.mjs --control N0 serves it as /engine/xr-slam.js.
const ResourceUrls = { setBaseUrl: () => {}, getBaseUrl: () => '' };

const IDENTITY_POSE = { rotation: { x: 0, y: 0, z: 0, w: 1 }, position: { x: 0, y: 2, z: 0 } };
const PROJECTION = [1.6, 0, 0, 0, 0, 2.1, 0, 0, 0, 0, -1.002, -1, 0, 0, -0.2, 0];

const XrControllerFactory = async () => {
  const pipelineModule = () => ({
    name: 'reality',
    requiredPermissions: () => { const p = window.XR8.XrPermissions.permissions(); return [p.CAMERA]; },
    onProcessCpu: ({ frameStartResult }) => ({
      ...IDENTITY_POSE, intrinsics: PROJECTION.slice(), realityTexture: frameStartResult.cameraTexture,
      trackingStatus: 'NORMAL', trackingReason: 'UNSPECIFIED', worldPoints: [], lighting: undefined, detectedImages: [], points: [], meanRGB: 0,
    }),
  });
  const noop = () => {};
  return {
    pipelineModule, configure: noop, recenter: noop, hitTest: Object.assign(() => [], { type: {} }), updateCameraProjectionMatrix: noop,
    updateEnableEngineVpsEvents: noop, getEnableEngineVpsEvents: () => false, getIntrinsic: () => PROJECTION.slice(),
    getIsDataRecording: () => false, startDatarecorder: noop, stopDatarecorder: noop,
    xrController: () => ({ cameraPipelineModule: pipelineModule, configure: noop, recenter: noop, updateCameraProjectionMatrix: noop, updateCameraOrigin: noop }),
  };
};

export { ResourceUrls, XrControllerFactory };
