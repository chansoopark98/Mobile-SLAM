/* Camera capture size of the slam_refact XR8 samples. xr.js (the deployed SDK, used as it is) first asks Android Chrome for 960x720 exact
 * ("preferred android resolution for performance", docs/reports/camera-xr_reference.md section 1). The page draws that stream as the AR
 * background, cover-cropped to the screen: on a Galaxy Z Flip6 (1080x2232 canvas) only 465 of the 720 columns are visible, upscaled 2.3x.
 *
 * This script replaces only that first request with a larger 4:3 size. 4:3 keeps the sensor crop, so the engine intrinsics stay the same
 * (CropAndScale of the same full-sensor intrinsics), and the engine input stays 720x960 (the chunk resamples every capture size to the fixed
 * filtered input, as the original does for its own 1920x1440 fallback). The camera feed the page draws gets the full stream resolution.
 * When the camera refuses the size, xr.js's own request is made unchanged, so its cascade goes on as before.
 *
 * ?camera=WxH picks the size (landscape order). ?camera=960x720 keeps the SDK request (this script changes nothing). Default 1920x1440:
 * on the Flip6 (2026-10-30, build/xr-camera-res-20261030/) 1920x1440 kept the frame rate of 960x720 with 0 dropped frames; 2560x1920 and
 * 4000x3000 lowered the camera to 17-20 fps.
 * Classic script, loaded before xr.js. It never opens the camera itself; window.XrCameraResolution reports what was asked and granted. */
(function () {
  'use strict';
  const DEFAULT_SIZE = { width: 1920, height: 1440 };
  const SDK_FIRST = { width: 960, height: 720 };
  const parseSize = (text) => {
    const m = /^(\d{3,4})x(\d{3,4})$/.exec(text || '');
    // 4:3 only: another aspect changes the sensor crop, and with it the intrinsics the engine assumes for this device.
    return m && +m[1] * 3 === +m[2] * 4 ? { width: +m[1], height: +m[2] } : null;
  };
  const param = new URLSearchParams(window.location.search).get('camera');
  const size = parseSize(param) || DEFAULT_SIZE;
  const report = window.XrCameraResolution = {
    requested: size, sdkRequest: null, granted: null, fallbackReason: param && !parseSize(param) ? `ignored ?camera=${param}` : null,
  };
  const md = navigator.mediaDevices;
  if (!md || typeof md.getUserMedia !== 'function') return;
  if (size.width === SDK_FIRST.width && size.height === SDK_FIRST.height) return;
  const original = md.getUserMedia.bind(md);
  const isSdkFirst = (v) => !!(v && typeof v === 'object' && v.width && v.height
    && v.width.exact === SDK_FIRST.width && v.height.exact === SDK_FIRST.height);
  const settingsOf = (stream) => {
    const track = stream.getVideoTracks()[0];
    const s = track && track.getSettings ? track.getSettings() : {};
    return { width: s.width, height: s.height, frameRate: s.frameRate };
  };
  md.getUserMedia = function (constraints) {
    const video = constraints && constraints.video;
    if (!isSdkFirst(video)) return original(constraints);
    report.sdkRequest = { width: SDK_FIRST.width, height: SDK_FIRST.height };
    const larger = { ...constraints, video: { ...video, width: { exact: size.width }, height: { exact: size.height } } };
    return original(larger).then((stream) => {
      report.granted = settingsOf(stream);
      return stream;
    }, (error) => {
      report.fallbackReason = `${size.width}x${size.height} refused (${error && error.name}); SDK request used`;
      return original(constraints).then((stream) => { report.granted = settingsOf(stream); return stream; });
    });
  };
}());
