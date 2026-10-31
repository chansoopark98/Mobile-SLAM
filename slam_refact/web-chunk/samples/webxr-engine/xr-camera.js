// Camera geometry of the webxr-engine prototype: WebXR camera-access image -> engine capture image and its K. Pure module (no DOM, no GL).
//
// What the engine does with a capture of W x H (portrait, rotation 0), read from the source:
//  - Gr8ComputePyramid draws the input stage into a fixed 720 x 960 "filtered" target (gr8-input.frag, uv 0..1 of the whole camera
//    texture) and stage 1 samples that target into level 0. Nothing is cropped: the image is STRETCHED, per axis, onto level 0.
//  - level 0 is (int(aspect * 640), 640) with aspect = float(W) / H, clamped to 3:4 only when wider than 3:4
//    (engine/src/frontend/pyramid_layout.cc). 785 x 1920 -> 261 x 640: x scale 261/785 = 0.33248, y scale 1/3 (VERIFIED on the deployed
//    WASM: pyramidLayout().levels[0]).
//  - With config.intrinsics the engine uses the override AS the tracking (processing) K, with width/height = the capture size
//    (engine.cc: intrinsic_ = *intrinsics_override; bindings.cc sets in.width/height = captureWidth/Height). The device-table path instead
//    crops and scales K to level 0 (intrinsics.cc ComputeIntrinsicsSet .processing). So an override is only right when capture == level 0;
//    for 785 x 1920 a capture-pixel K (fx 1275) would be 3x the level-0 focal, and the frame bounds (visibility.cc frameBounds) would be
//    those of a 785 x 1920 image.
// Hence this page resamples the camera image itself to a capture whose level 0 is the capture: height 640, width W' <= 480 with
// level0(W', 640) = (W', 640). For 785 x 1920: crop x in [1, 784) (783 px, centred) and box-average 3 x 3 -> 261 x 640, an exact integer
// decimation, so K' = K / 3 with the half-pixel shift below, and the engine's processing K == the override at the right size.
// mode '34' instead centre-crops to 3:4 first (480 x 640 output, narrower vertical FOV, finer horizontal sampling); not the default.
//
// Pixel conventions: K here is pixel-INDEX based (pixel i covers [i - 0.5, i + 0.5], centre of a W-wide image = (W - 1) / 2, as the
// engine's MakeIntrinsics 239.5 / 319.5). A map that takes source edge coordinates u (pixel i covers [i, i + 1]) to output edge
// coordinates u' = (u - x0) / d gives index coordinates x' = (x + 0.5 - x0) / d - 0.5.
// Image rows are top-down (v down) in K and in the engine's capture: row 0 of the texture the pyramid reads is the image top
// (gr8-compute-pyramid.js readTextureRgba note, XR8 uploads video without a flip). Whether the WebXR camera texture has row 0 at the top or
// at the bottom (GL convention) is not documented for this device: imageOrientationCheck() measures it (phase A).
import { intrinsicsFromProjection } from '../webxr-probe/analysis.js';

export const ENGINE_HEIGHT = 640;  // level-0 height of the engine's 1024 pyramid (pyramid_layout.cc target_height)

// K (pixel-index convention, top-down rows) from a WebXR view projection for a W x H camera image. The probe's intrinsicsFromProjection
// gives edge coordinates (cx = W/2 at the centre); -0.5 converts.
export function intrinsicsFromXrProjection(P, width, height) {
  const k = intrinsicsFromProjection(P, width, height);
  return { fx: k.fx, fy: k.fy, cx: k.cx - 0.5, cy: k.cy - 0.5, width, height, squarePixelRatio: k.squarePixelRatio };
}

// Level-0 size of ComputePyramidLayout(W, H, 1024) for a portrait capture, with the engine's float32 arithmetic.
export function level0Size(w, h) {
  if (w > h) throw new Error('level0Size: portrait captures only');
  const f = Math.fround;
  let targetHeight = ENGINE_HEIGHT;
  let targetWidth = f(f(w / h) * targetHeight);
  const max = (targetHeight * 3 / 4) | 0;  // int arithmetic in C++
  if (targetWidth > max) {
    const scale = f(max / targetWidth);
    targetWidth = f(targetWidth * scale);
    targetHeight = Math.trunc(f(targetHeight * scale));
  }
  return { w: Math.trunc(targetWidth), h: targetHeight };
}

// Engine capture plan for a W x H portrait camera image with K: crop rectangle (source pixels, top-down), decimation footprint d (source
// px per output px), output size, output K. Throws for landscape images (this prototype assumes the phone is held portrait).
export function planEngineInput(width, height, K, { mode = 'full' } = {}) {
  if (!(width > 0 && height > 0)) throw new Error('planEngineInput: bad camera size');
  if (width > height) throw new Error(`planEngineInput: landscape camera image ${width}x${height} is not supported (hold the phone portrait)`);
  let cropW = width, cropH = height;
  if (mode === '34') {
    if (width / height > 3 / 4) cropW = height * 3 / 4; else cropH = width * 4 / 3;
  } else if (mode !== 'full') throw new Error(`planEngineInput: unknown mode ${mode}`);
  const d = cropH / ENGINE_HEIGHT;
  let outW = Math.min(480, Math.floor(cropW / d + 1e-9));
  while (outW > 1 && level0Size(outW, ENGINE_HEIGHT).w !== outW) outW--;  // capture must equal level 0 (see header)
  const usedW = outW * d;
  const x0 = (width - usedW) / 2, y0 = (height - cropH) / 2;
  const snap = (v) => (Math.abs(v - Math.round(v)) < 1e-9 ? Math.round(v) : v);
  const plan = { mode, source: { width, height }, crop: { x0: snap(x0), y0: snap(y0), width: snap(usedW), height: snap(cropH) },
    footprint: d, integerFootprint: Math.abs(d - Math.round(d)) < 1e-9, output: { width: outW, height: ENGINE_HEIGHT } };
  plan.K = K ? mapK(K, plan) : null;
  plan.level0 = level0Size(outW, ENGINE_HEIGHT);
  return plan;
}

// K of the output image: x' = (x + 0.5 - x0) / d - 0.5 (header).
export function mapK(K, plan) {
  const { x0, y0 } = plan.crop, d = plan.footprint;
  return { fx: K.fx / d, fy: K.fy / d, cx: (K.cx + 0.5 - x0) / d - 0.5, cy: (K.cy + 0.5 - y0) / d - 0.5,
    width: plan.output.width, height: plan.output.height };
}

// The source pixel (index coords, top-down) an output pixel centre maps to (inverse of the map above); used by tests and the shader comment.
export const outputToSource = (plan, xo, yo) => [(xo + 0.5) * plan.footprint + plan.crop.x0 - 0.5, (yo + 0.5) * plan.footprint + plan.crop.y0 - 0.5];

// SlamRefactGlue config for the plan (bindings.cc constructor keys). estimateScale needs VIO, so it is false with disableVio.
export function buildGlueConfig({ plan, model = '', disableVio = false, estimateScale = true, accuracy = null }) {
  const cfg = {
    captureWidth: plan.output.width, captureHeight: plan.output.height, rotationDeg: 0,
    manufacturer: '', model, estimateScale: estimateScale && !disableVio, disableVio, leftHandedAxes: false, mirroredDisplay: false,
    intrinsics: [plan.K.fx, plan.K.fy, plan.K.cx, plan.K.cy],
  };
  if (accuracy) cfg.accuracy = accuracy;
  return cfg;
}

// Body-frame rotation vector (rad, GL camera axes x right / y up / z back) from the camera-to-world orientations of two consecutive frames
// ([w, x, y, z]): axis * angle of q_prev^-1 q_curr.
export function relativeRotationVector(qPrev, qCurr) {
  const [a, b, c, d] = qPrev, [e, f, g, h] = qCurr;
  // conj(qPrev) * qCurr
  let w = a * e + b * f + c * g + d * h;
  let x = a * f - b * e - c * h + d * g;
  let y = a * g + b * h - c * e - d * f;
  let z = a * h - b * g + c * f - d * e;
  if (w < 0) { w = -w; x = -x; y = -y; z = -z; }
  const s = Math.hypot(x, y, z);
  if (s < 1e-12) return [0, 0, 0];
  const angle = 2 * Math.atan2(s, w);
  return [x / s * angle, y / s * angle, z / s * angle];
}

// Texture row order and mirror check (phase A). Pure rotation by theta (body frame, GL axes) moves the image centre content by
// du = +fx * theta_y, dv = +fy * theta_x in a top-down, unmirrored image (static point p_c' = -theta x p_c, OpenCV y = -y_GL). samples:
// [{dx, dy, ok, theta: [x, y, z]}] with dx/dy the measured shift in the readback image (rows in TEXTURE row order), fxSmall/fySmall the focal
// of that image. slopeY ~ +1: texture row 0 is the image top (no flip); ~ -1: row 0 is the bottom (flipY). slopeX ~ -1 would be a mirror.
export function imageOrientationCheck(samples, fxSmall, fySmall, { minPred = 0.3, minSamples = 15, minCorr = 0.5 } = {}) {
  const fit = (pairs) => {
    if (pairs.length < minSamples) return { n: pairs.length, slope: NaN, corr: NaN, conclusive: false };
    let sxy = 0, sxx = 0, syy = 0;
    for (const [p, m] of pairs) { sxy += p * m; sxx += p * p; syy += m * m; }
    const corr = sxy / Math.sqrt(sxx * syy), slope = sxy / sxx;
    return { n: pairs.length, slope, corr, conclusive: Math.abs(corr) >= minCorr && Math.abs(slope) > 0.3 && Math.abs(slope) < 3 };
  };
  const ok = samples.filter((s) => s.ok && Number.isFinite(s.dx) && Number.isFinite(s.dy) && s.theta);
  const x = fit(ok.map((s) => [fxSmall * s.theta[1], s.dx]).filter(([p]) => Math.abs(p) >= minPred));
  const y = fit(ok.map((s) => [fySmall * s.theta[0], s.dy]).filter(([p]) => Math.abs(p) >= minPred));
  return { x, y, flipY: y.conclusive ? y.slope < 0 : null, flipX: x.conclusive ? x.slope < 0 : null,
    conclusive: x.conclusive && y.conclusive };
}
