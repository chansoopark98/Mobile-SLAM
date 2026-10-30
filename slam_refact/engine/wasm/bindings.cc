// Embind surface of the engine for node and browser smoke tests (US-010 API on GlueRunner, D-3):
//   new Module.SlamRefactGlue(config)   configure (EngineConfig + GlueConfig fields)
//   pushDeviceMotion(...), pushDeviceOrientation(...)   pushImu (W3C values, receive time in ns)
//   pushDeviceMotionEvent(receiveTimeNs, {acceleration, accelerationIncludingGravity, rotationRate:
//                                       {alpha, beta, gamma}, interval})  FULL-7: the same entry with
//                                       every sensor optional, like the original's updateDeviceMotion
//                                       (a falsy value pushes no event of that kind)
//   processFrame(timeNs, videoTimeNs, gray, width, height)  -> published pose object
//   processFramePyramid(timeNs, videoTimeNs, atlas, size, rows) -> same, with a GPU readback
//                                       pyramid atlas (H7 injection; the CPU shader port is skipped)
//   pyramidLayout()                     atlas layout and shader parameters the engine expects
//   SlamRefactGlue.cpuPyramidAtlas(rgba, width, height, model)  CPU shader port atlas (diagnostic:
//                                       GPU-vs-CPU pyramid comparison)
//   getPose() / getWorldPoints()        getters of the last published response
//   getSurfaces()                       ACC-3 floor / horizontal surfaces (config.detectSurfaces)
//   hitTest(u, v, mask)                 ACC-3 hit test of a processed-image point (u, v in [0, 1])
//   reset()
// XrController API (contract FULL-1, docs/spec/06-xr-controller-api.md):
//   configure({enableWorldPoints, enableFeatureSet, scale: 'absolute'|'responsive', enableLighting,
//              leftHandedAxes, mirroredDisplay, disableWorldTracking})  throws the original errors
//   updateCameraProjectionMatrix({cam: {pixelRectWidth, pixelRectHeight, nearClipPlane,
//              farClipPlane}, origin: {x, y, z}, facing: {w, x, y, z}, updateRecenterPoint})
//   recenter()                          XrController.recenter (reset() is the older name)
//   xrHitTest(x, y, types)              XrController.hitTest: x, y in [0, 1] of the displayed feed,
//                                       types = ['FEATURE_POINT', 'ESTIMATED_SURFACE',
//                                       'DETECTED_SURFACE'] (empty = all) -> [{type, position{x,y,z},
//                                       rotation{x,y,z,w}, distance}]. ESTIMATED/DETECTED_SURFACE
//                                       are an EXTENSION (config.detectSurfaces), empty otherwise.
//   onCameraStatusChange(status, direction)  direction 'back' | 'front'; throws
//                                       slam-front-camera-unsupported like the original
//   getIntrinsic()                      reality.intrinsics of the last response (16 numbers)
//   getPose() also has intrinsics (16 numbers), lighting {exposure, temperature} (enableLighting)
//   and trackingStatusChanged (reality.trackingstatus would be dispatched).
//   config.facing = [w, x, y, z] (with config.origin) at construction; origin and facing must be
//   finite (throws; the original passes NaN / Infinity to capnp unchecked).
//   config.framesUntilRecenter (integer >= 0, default 5 = tracking-controller.cc:85) and
//   getFramesUntilRecenter(): the startup recenter counter. The original keeps it across an
//   engineInit re-run (video size / orientation change), so a runner that replaces another one for
//   such a re-run passes the old runner's getFramesUntilRecenter() (normally 0).
// Image targets (contract FULL-3, docs/design/image-target.md §4):
//   setImageTargets([{json, luminance, width, height}])  json = target JSON string (image-target-cli
//                                       format), luminance = decoded Uint8Array (width * height);
//                                       flat, cylinder and cone targets (FULL-9), at most 32; throws on
//                                       error (nothing changes)
//   unloadImageTargets([names]), clearImageTargets(), getImageTargetNames()
//   getPose().detectedImages = reality.detectedImages [{name, id, type, position{x,y,z},
//   rotation{x,y,z,w}, widthInMeters, heightInMeters, scale, trackingStatus, curvyGeometry?}]
//   (FULL-9: type CYLINDRICAL / CONICAL carry curvyGeometry = {curvyCircumferenceTop,
//   curvyCircumferenceBottom, curvySideLength, targetCircumferenceTop, height, topRadius,
//   bottomRadius, arcLengthRadians, arcStartRadians}, the 9 fields of reality.capnp CurvyGeometry) and
//   getPose().imageEvents = [{name: 'imagefound'|'imageupdated'|'imagelost', detail}].
// ROI request (contract FULL-6): getRequestedRois() = the ROIs a GPU pyramid producer must draw into
//   the ROI slots of the NEXT staged frame (what processFrame / processFramePyramid consumes next):
//   [{slot, source: 'IMAGE_TARGET'|'HIRES_SCAN'|'CURVY_IMAGE_TARGET', sourceId, faceId, name,
//   warp: Float32Array(16), curvy?}], FULL-9: a CURVY_IMAGE_TARGET ROI has warp = identity and
//   curvy = {rotation, curvyPostRotation, targetDims[2], roiDims[2], searchDims[2],
//   intrinsics: Float32Array(16), globalPoseInverse: Float32Array(16), scales[2], shift, radius,
//   radiusBottom} = the uniforms of gr8-stage1.frag's curvy branch (isCurvy = 1) for that slot draw,
//   slot = index into pyramidLayout().roiSlots, warp = the vertex-shader mvp column-major (use
//   gl.uniformMatrix4fv(loc, false, warp)). Empty without targets. pyramidLayout() also has
//   captureWidth / captureHeight (the shader scale and pixScale input size) and per ROI slot
//   shaderRotation (the `rotation` uniform of the slot draw) and roiAtlasRows (the
//   rows an atlas must have to contain every ROI slot).
//   FULL-7 (RM-4b): processFramePyramid takes an atlas whose ROI slots the producer drew (stage 1 of
//   getRequestedRois() into pyramidLayout().roiSlots, then stages 2 and 3 over the whole texture);
//   the engine reads them as given. While ROIs are requested the atlas needs rows >=
//   pyramidLayout().roiAtlasRows (processFramePyramid throws otherwise); without image targets the
//   rows range is unchanged ([rows, textureSize]).
// Accuracy mode (contract ACC-1..4): config.accuracy = {enabled, metricOutput, metricWindowS,
// metricPriorScale, localBa, multiviewTriangulation, baWindow}; config.intrinsics = [fx, fy, cx, cy]
// overrides the K rule.
// Times are JS numbers holding integer nanoseconds (exact below 2^53 ns, about 104 days).

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "frontend/frontend.h"
#include "frontend/roi_draw.h"
#include "slam_refact/glue_runner.h"

namespace slam_refact::wasm {
namespace {

using emscripten::val;

// Input errors of the JS surface throw a JS Error instead of reaching an engine SR_CHECK abort
// (which would kill the module): the worker reports it and can recreate the engine.
[[noreturn]] void ThrowJs(const std::string& what) {
  val::global("Error").new_(val("SlamRefactGlue: " + what)).throw_();
  __builtin_unreachable();
}

// Integer nanoseconds held in a JS number; non-finite or out-of-range values would be UB in the cast.
int64_t Ns(double t) {
  if (!std::isfinite(t) || std::fabs(t) > 9.0e18) ThrowJs("time is not a finite nanosecond value");
  return static_cast<int64_t>(t);
}

size_t ByteLength(const val& array) { return array["length"].as<size_t>(); }

const char* StatusName(TrackingStatus s) {
  switch (s) {
    case TrackingStatus::kNotAvailable: return "NOT_AVAILABLE";
    case TrackingStatus::kLimited: return "LIMITED";
    case TrackingStatus::kNormal: return "NORMAL";
    default: return "UNSPECIFIED";
  }
}

const char* ReasonName(TrackingReason r) {
  switch (r) {
    case TrackingReason::kInitializing: return "INITIALIZING";
    case TrackingReason::kRelocalizing: return "RELOCALIZING";
    case TrackingReason::kTooMuchMotion: return "TOO_MUCH_MOTION";
    case TrackingReason::kNotEnoughTexture: return "NOT_ENOUGH_TEXTURE";
    default: return "UNSPECIFIED";
  }
}

// A JS number that is finite after the conversion to float32 (RM-3c: 1e39 is a finite double but
// becomes +inf as a float, which the engine rejects with SR_CHECK = module abort; a double above
// FLT_MAX is out of range for the cast itself).
float FiniteFloat(const val& v, const char* what) {
  const double d = v.as<double>();
  if (!std::isfinite(d) || std::fabs(d) > static_cast<double>(std::numeric_limits<float>::max())) {
    ThrowJs(std::string(what) + " must be finite (as a float32)");
  }
  return static_cast<float>(d);
}

// The Int64 range of the event interval (ms * 1e6 ns) that capnp-ts accepts (IntervalMsToNanos:
// NaN becomes 0, |v| > 2^64 is a RangeError there and an SR_CHECK abort in the engine).
bool IntervalFitsInt64Ns(double interval_ms) {
  const double ns = interval_ms * 1000 * 1000;
  return std::isnan(ns) || std::fabs(ns) / 4294967296.0 <= 4294967296.0;
}

template <typename T>
T Get(const val& obj, const char* key, T def) {
  const val v = obj[key];
  return v.isUndefined() || v.isNull() ? def : v.as<T>();
}

class SlamRefactGlue {
 public:
  explicit SlamRefactGlue(val cfg) {
    EngineConfig e;
    e.estimate_scale = Get<bool>(cfg, "estimateScale", false);
    e.disable_vio = Get<bool>(cfg, "disableVio", false);
    e.capture_width = Get<int>(cfg, "captureWidth", 0);
    e.capture_height = Get<int>(cfg, "captureHeight", 0);
    e.rotation_deg = Get<int>(cfg, "rotationDeg", 0);
    e.manufacturer = Get<std::string>(cfg, "manufacturer", "");
    e.model = Get<std::string>(cfg, "model", "");
    e.pyramid_model = Get<std::string>(cfg, "pyramidModel", "ieee") == "swiftshader-approx"
                          ? ShaderNumericModel::kSwiftShaderApprox
                          : ShaderNumericModel::kIeee;
    const val k = cfg["intrinsics"];
    if (!k.isUndefined() && !k.isNull()) {
      PinholeIntrinsics in;
      in.width = e.capture_width;
      in.height = e.capture_height;
      in.fx = k[0].as<float>();
      in.fy = k[1].as<float>();
      in.cx = k[2].as<float>();
      in.cy = k[3].as<float>();
      if (!(in.fx > 0.f && in.fy > 0.f) || !std::isfinite(in.fx) || !std::isfinite(in.fy) ||
          !std::isfinite(in.cx) || !std::isfinite(in.cy)) {
        ThrowJs("intrinsics must be finite with fx, fy > 0");
      }
      e.intrinsics_override = in;
    }
    const val acc = cfg["accuracy"];
    if (!acc.isUndefined() && !acc.isNull()) {
      AccuracyConfig& a = e.accuracy;
      a.enabled = Get<bool>(acc, "enabled", false);
      a.metric_output = Get<bool>(acc, "metricOutput", a.metric_output);
      a.metric_window_s = Get<double>(acc, "metricWindowS", a.metric_window_s);
      a.metric_prior_scale = Get<double>(acc, "metricPriorScale", a.metric_prior_scale);
      a.metric_low_band = Get<bool>(acc, "metricLowBand", false);
      if (acc.hasOwnProperty("metricFixedLagS")) {
        a.metric_fixed_lag_s = Get<double>(acc, "metricFixedLagS", 0.0);
        if (!(std::fabs(a.metric_fixed_lag_s) <= 0.3)) ThrowJs("metricFixedLagS must be in [-0.3, 0.3]");
      }
      a.local_ba = Get<bool>(acc, "localBa", false);
      a.multiview_triangulation = Get<bool>(acc, "multiviewTriangulation", false);
      a.ba_window = Get<int>(acc, "baWindow", a.ba_window);
      a.vi_prediction = Get<bool>(acc, "viPrediction", false);
      a.vi_velocity_gain = Get<double>(acc, "viVelocityGain", a.vi_velocity_gain);
      if (!(a.vi_velocity_gain >= 0.0 && a.vi_velocity_gain <= 1.0)) ThrowJs("viVelocityGain must be in [0, 1]");
      if (!(a.metric_window_s >= 1.0 && a.metric_window_s <= 30.0)) ThrowJs("metricWindowS must be in [1, 30]");
      if (!(a.metric_prior_scale > 0.0 && std::isfinite(a.metric_prior_scale))) {
        ThrowJs("metricPriorScale must be finite and > 0");
      }
      if (a.ba_window < 2 || a.ba_window > 64) ThrowJs("baWindow must be in [2, 64]");
    }
    GlueConfig g;
    g.detect_surfaces = Get<bool>(cfg, "detectSurfaces", false);
    const val origin = cfg["origin"];
    if (!origin.isUndefined()) {
      g.origin = Vec3f{FiniteFloat(origin[0], "origin[0]"), FiniteFloat(origin[1], "origin[1]"),
                       FiniteFloat(origin[2], "origin[2]")};
    }
    const val facing = cfg["facing"];
    if (!facing.isUndefined() && !facing.isNull()) {
      g.facing = QuatWxyz{FiniteFloat(facing[0], "facing[0]"), FiniteFloat(facing[1], "facing[1]"),
                          FiniteFloat(facing[2], "facing[2]"), FiniteFloat(facing[3], "facing[3]")};
    }
    const val frames = cfg["framesUntilRecenter"];
    if (!frames.isUndefined() && !frames.isNull()) {
      const double d = frames.as<double>();
      if (!(d >= 0.0 && d <= 1000000.0) || d != std::floor(d)) {
        ThrowJs("framesUntilRecenter must be an integer >= 0");
      }
      g.frames_until_recenter = static_cast<int>(d);
    }
    g.left_handed_axes = Get<bool>(cfg, "leftHandedAxes", false);
    g.mirrored_display = Get<bool>(cfg, "mirroredDisplay", false);
    glue_ = std::make_unique<GlueRunner>(e, g);
    layout_ = frontend::ComputePyramidLayout(e.capture_width, e.capture_height);
    capture_width_ = e.capture_width;
    capture_height_ = e.capture_height;
    rotation_deg_ = e.rotation_deg;
  }

  // accelerationIncludingGravity (m/s^2), optional acceleration (hasAcceleration), rotationRate
  // (alpha, beta, gamma deg/s), interval in ms.
  void PushDeviceMotion(double receive_time_ns, float gx, float gy, float gz, bool has_acceleration,
                        float ax, float ay, float az, float ra, float rb, float rg,
                        double interval_ms) {
    DeviceMotionSample s;
    s.receive_time_ns = Ns(receive_time_ns);
    s.acceleration_including_gravity = Vec3f{gx, gy, gz};
    if (has_acceleration) s.acceleration = Vec3f{ax, ay, az};
    s.rotation_rate_abg = Vec3f{ra, rb, rg};
    if (!IntervalFitsInt64Ns(interval_ms)) ThrowJs("interval is outside the Int64 nanosecond range");
    s.interval_ms = interval_ms;
    glue_->PushDeviceMotion(s);
  }

  // FULL-7 (RM-4b): sensors.ts updateDeviceMotion with optional sensors. Each of acceleration,
  // accelerationIncludingGravity and rotationRate is used when it is truthy (the original's
  // `if (value)`); its numbers go through Number(...) like capnp's float setters (null -> 0,
  // undefined -> NaN). interval (ms) likewise (a NaN interval becomes 0 ns, as in the original).
  void PushDeviceMotionEvent(double receive_time_ns, val event) {
    if (event.isNull() || event.typeOf().as<std::string>() != "object") {
      ThrowJs("pushDeviceMotionEvent needs an event object");
    }
    DeviceMotionSample s;
    s.receive_time_ns = Ns(receive_time_ns);
    const val acceleration = event["acceleration"];
    if (acceleration.as<bool>()) s.acceleration = JsVec3(acceleration, "x", "y", "z");
    const val gravity = event["accelerationIncludingGravity"];
    if (gravity.as<bool>()) s.acceleration_including_gravity = JsVec3(gravity, "x", "y", "z");
    const val rate = event["rotationRate"];
    if (rate.as<bool>()) s.rotation_rate_abg = JsVec3(rate, "alpha", "beta", "gamma");
    s.interval_ms = JsNumber(event["interval"]);
    // sensors.ts computes interval * 1e6 for every event but stores it only with a sensor: an event
    // without sensors is a no-op whatever its interval; with a sensor an out-of-range interval is a
    // RangeError of the original (thrown later, in setEventQueue) and here a JS error at the push
    // (RM-3c; it used to abort the module).
    if (!s.acceleration.has_value() && !s.acceleration_including_gravity.has_value() &&
        !s.rotation_rate_abg.has_value()) {
      s.interval_ms = 0.0;
    } else if (!IntervalFitsInt64Ns(s.interval_ms)) {
      ThrowJs("interval is outside the Int64 nanosecond range");
    }
    glue_->PushDeviceMotion(s);
  }

  static double JsNumber(const val& v) { return val::global("Number")(v).as<double>(); }

  static Vec3f JsVec3(const val& o, const char* a, const char* b, const char* c) {
    return Vec3f{static_cast<float>(JsNumber(o[a])), static_cast<float>(JsNumber(o[b])),
                 static_cast<float>(JsNumber(o[c]))};
  }

  int GetFramesUntilRecenter() const { return glue_->frames_until_recenter(); }

  void PushDeviceOrientation(double receive_time_ns, double alpha, double beta, double gamma) {
    glue_->PushDeviceOrientation(DeviceOrientationSample{Ns(receive_time_ns), alpha, beta, gamma});
  }

  // gray: Uint8Array of width * height bytes (row stride = width).
  val ProcessFrame(double time_ns, double video_time_ns, val gray, int width, int height) {
    if (width <= 0 || height <= 0) ThrowJs("frame width and height must be positive");
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (ByteLength(gray) != n) ThrowJs("gray length differs from width * height");
    pixels_.resize(n);
    val(emscripten::typed_memory_view(n, pixels_.data())).call<void>("set", gray);
    return ToJs(glue_->ProcessFrame(Ns(time_ns), Ns(video_time_ns),
                                    GrayImageView{pixels_.data(), width, height, width}));
  }

  // atlas: Uint8Array of size * rows * 4 bytes (GPU readback, row 0 = GL row 0, row bytes 4 *
  // size). The engine validates it against its layout (SR_CHECK) and copies it into the pipeline.
  val ProcessFramePyramid(double time_ns, double video_time_ns, val atlas, int size, int rows) {
    if (size != layout_.texture_size || rows < layout_.rows || rows > size) {
      ThrowJs("atlas size/rows do not match pyramidLayout()");
    }
    // FULL-7: while image targets request ROIs the producer draws them into the atlas and the
    // engine reads the slots, so the atlas must contain them.
    if (rows < frontend::RoiAtlasRows(layout_) && !glue_->NextDrawRois().empty()) {
      ThrowJs("atlas rows do not contain the requested ROI slots (pyramidLayout().roiAtlasRows)");
    }
    const size_t n = static_cast<size_t>(size) * static_cast<size_t>(rows) * 4u;
    if (ByteLength(atlas) != n) ThrowJs("atlas length differs from size * rows * 4");
    atlas_.resize(n);
    val(emscripten::typed_memory_view(n, atlas_.data())).call<void>("set", atlas);
    return ToJs(glue_->ProcessFrame(Ns(time_ns), Ns(video_time_ns), GrayImageView{},
                                    PyramidAtlasView{atlas_.data(), size, rows}));
  }

  static val Level(const frontend::LevelLayout& l) {
    val o = val::object();
    o.set("r", l.r);
    o.set("c", l.c);
    o.set("w", l.w);
    o.set("h", l.h);
    o.set("rotated", l.rotated);
    return o;
  }

  // The layout and shader parameters of this engine's pyramid source (func 459, S1 §2.2), so a GPU
  // producer draws exactly the atlas the engine reads.
  val PyramidLayout() const {
    val o = val::object();
    o.set("textureSize", layout_.texture_size);
    o.set("rows", layout_.rows);
    o.set("filteredWidth", layout_.filtered_w);
    o.set("filteredHeight", layout_.filtered_h);
    o.set("rotateInput", layout_.rotate_input);
    o.set("roiAtlasRows", frontend::RoiAtlasRows(layout_));
    o.set("captureWidth", capture_width_);
    o.set("captureHeight", capture_height_);
    val levels = val::array();
    val rois = val::array();
    for (const frontend::LevelLayout& l : layout_.levels) levels.call<void>("push", Level(l));
    for (const frontend::LevelLayout& l : layout_.roi_slots) {
      val slot = Level(l);
      // gl-reality-frame.cc:379: shaderRotation = l.rotated ? rotation_ - 90 : rotation_ (rotation_ =
      // the engineInit rotation, FULL-8).
      slot.set("shaderRotation", l.rotated ? rotation_deg_ - 90 : rotation_deg_);
      rois.call<void>("push", slot);
    }
    o.set("levels", levels);
    o.set("roiSlots", rois);
    const frontend::ShaderParams p;
    val sp = val::object();
    sp.set("gain", p.gain);
    sp.set("scoreThresh", p.score_thresh);
    sp.set("nmsTolerance", p.nms_tolerance);
    sp.set("skipSubpixel", p.skip_subpixel);
    o.set("shaderParams", sp);
    return o;
  }

  // CPU shader port atlas of one RGBA camera frame (diagnostic only; not used for tracking).
  static val CpuPyramidAtlas(val rgba, int width, int height, std::string model) {
    if (width <= 0 || height <= 0) ThrowJs("frame width and height must be positive");
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height) * 4u;
    if (ByteLength(rgba) != n) ThrowJs("rgba length differs from width * height * 4");
    std::vector<uint8_t> pixels(n);
    val(emscripten::typed_memory_view(n, pixels.data())).call<void>("set", rgba);
    const frontend::PyramidLayout layout = frontend::ComputePyramidLayout(width, height);
    const frontend::CpuGr8Pyramid cpu(layout, frontend::ShaderParams{},
                                      model == "swiftshader-approx"
                                          ? ShaderNumericModel::kSwiftShaderApprox
                                          : ShaderNumericModel::kIeee);
    frontend::RgbaAtlas atlas;
    cpu.Build(RgbaImageView{pixels.data(), width, height, width * 4}, 0, &atlas);
    val out = val::object();
    out.set("size", atlas.size);
    out.set("rows", atlas.rows);
    out.set("rgba", val::global("Uint8Array").new_(
                        val(emscripten::typed_memory_view(atlas.rgba.size(), atlas.rgba.data()))));
    return out;
  }

  // Last published response (the original's `active`); `repeated` describes that response.
  val GetPose() const { return ToJs(glue_->last_published()); }

  static val ToJs(const PublishedFrame& f) {
    val out = val::object();
    val p = val::array();
    p.call<void>("push", f.position.x, f.position.y, f.position.z);
    val q = val::array();
    q.call<void>("push", f.rotation.w, f.rotation.x, f.rotation.y, f.rotation.z);
    out.set("position", p);
    out.set("rotationWxyz", q);
    out.set("status", std::string(StatusName(f.status)));
    out.set("reason", std::string(ReasonName(f.reason)));
    out.set("repeated", f.repeated);
    out.set("videoFrameIndex", static_cast<double>(f.video_frame_index));
    out.set("sourceFrameIndex", static_cast<double>(f.source_frame_index));
    out.set("worldPoints", static_cast<int>(f.world_points.size()));
    out.set("intrinsics", Intrinsics(f));
    if (f.has_lighting) {
      val l = val::object();
      l.set("exposure", f.lighting.exposure);
      l.set("temperature", f.lighting.temperature);
      out.set("lighting", l);
    }
    out.set("trackingStatusChanged", f.tracking_status_changed);
    val images = val::array();
    for (const DetectedImageOut& d : f.detected_images) images.call<void>("push", DetectedImage(d));
    out.set("detectedImages", images);
    val events = val::array();
    for (const ImageTargetEvent& e : f.image_events) {
      val ev = val::object();
      ev.set("name", std::string(e.kind == ImageTargetEventKind::kFound     ? "imagefound"
                                 : e.kind == ImageTargetEventKind::kUpdated ? "imageupdated"
                                                                            : "imagelost"));
      ev.set("detail", DetectedImage(e.image));
      events.call<void>("push", ev);
    }
    out.set("imageEvents", events);
    if (f.metric) {
      val m = val::object();
      m.set("scale", f.metric_scale);
      m.set("sigmaLog", f.metric_sigma_log);
      m.set("initialized", f.metric_initialized);
      m.set("lagS", f.metric_lag_s);
      out.set("metric", m);
    }
    if (f.step.has_value()) {
      val ms = val::object();
      ms.set("frontend", f.step->timings.frontend_ms);
      ms.set("mapTracker", f.step->timings.map_tracker_ms);
      ms.set("mapBuilder", f.step->timings.map_builder_ms);
      ms.set("metricScale", f.step->timings.metric_scale_ms);
      ms.set("glue", f.step->timings.glue_ms);
      out.set("stageMs", ms);
      out.set("keypoints", f.step->num_keypoints);
      // Internal tracker state (parity debug fields, engine.h TrackerStep): the published reason is
      // UNSPECIFIED in responsive mode, so these show initialization/tracking on the page.
      const TrackerStep& t = *f.step;
      val st = val::object();
      st.set("mapTrackerState", t.map_tracker_state);
      st.set("frameTrackerOk", t.frame_tracker_ok);
      st.set("ok", t.ok);
      st.set("lossState", t.loss_state);
      st.set("lostFrames", t.lost_frames);
      st.set("frames", t.frames);
      st.set("yawOffsetInitialized", t.yaw_offset_initialized);
      st.set("mapReset", t.map_reset);
      st.set("keyframeCreated", t.keyframe_created);
      st.set("scaleLatched", t.scale_latched);
      st.set("latchedScale", t.latched_scale);
      st.set("numKeyframes", t.num_keyframes);
      st.set("numOrb", t.num_orb);
      st.set("numGorb", t.num_gorb);
      st.set("groundMatches", t.ground_matches);
      val src = val::object();
      src.set("triangulated", t.num_points_by_source[0]);
      src.set("ground", t.num_points_by_source[1]);
      src.set("sky", t.num_points_by_source[2]);
      src.set("depth", t.num_points_by_source[3]);
      src.set("cloud", t.num_points_by_source[4]);
      st.set("pointsBySource", src);
      out.set("tracker", st);
    }
    return out;
  }

  // Flat Float64Array [id, confidence, x, y, z] * n (uint32 ids and float values are exact).
  val GetWorldPoints() const {
    const std::vector<WorldPointOut>& w = glue_->last_published().world_points;
    std::vector<double> flat;
    flat.reserve(w.size() * 5);
    for (const WorldPointOut& p : w) {
      flat.insert(flat.end(), {static_cast<double>(p.id), static_cast<double>(p.confidence),
                               static_cast<double>(p.position.x), static_cast<double>(p.position.y),
                               static_cast<double>(p.position.z)});
    }
    return val::global("Float64Array").new_(val(emscripten::typed_memory_view(flat.size(), flat.data())));
  }

  static val Surface(const HorizontalSurface& s) {
    val o = val::object();
    o.set("height", s.height);
    o.set("inliers", s.inliers);
    o.set("confidence", s.confidence);
    o.set("area", s.area);
    val hull = val::array();
    for (const Vec3f& v : s.hull) hull.call<void>("push", v.x, v.z);
    o.set("hullXz", hull);
    return o;
  }

  val GetSurfaces() const {
    const SurfaceState& st = glue_->surfaces();
    val o = val::object();
    o.set("floorDetected", st.floor_detected);
    o.set("cloudPoints", st.cloud_points);
    if (st.floor_detected) o.set("floor", Surface(st.floor));
    val others = val::array();
    for (const HorizontalSurface& s : st.others) others.call<void>("push", Surface(s));
    o.set("others", others);
    return o;
  }

  val HitTest(double u, double v, int mask) const {
    if (!std::isfinite(u) || !std::isfinite(v)) ThrowJs("hitTest u, v must be finite");
    val out = val::array();
    for (const HitResult& h : glue_->HitTest(static_cast<float>(u), static_cast<float>(v),
                                             static_cast<uint32_t>(mask))) {
      val o = val::object();
      o.set("type", h.type == HitType::kDetectedSurface   ? "DETECTED_SURFACE"
                    : h.type == HitType::kEstimatedSurface ? "ESTIMATED_SURFACE"
                                                           : "FEATURE_POINT");
      val p = val::array();
      p.call<void>("push", h.position.x, h.position.y, h.position.z);
      o.set("position", p);
      o.set("distance", h.distance);
      o.set("surface", h.surface);
      out.call<void>("push", o);
    }
    return out;
  }

  void Reset() { glue_->Reset(); }

  static val Intrinsics(const PublishedFrame& f) {
    val m = val::array();
    for (const float v : f.intrinsics) m.call<void>("push", v);
    return m;
  }

  val GetIntrinsic() const { return Intrinsics(glue_->last_published()); }

  // A JS Error with the original message and, when the original sets them, err / type / source.
  [[noreturn]] static void ThrowXr(const XrError& e) {
    val error = val::global("Error").new_(val(e.message));
    if (!e.code.empty()) {
      error.set("type", std::string("configuration"));
      error.set("source", std::string("reality"));
      error.set("err", e.code);
    }
    error.throw_();
    __builtin_unreachable();
  }

  static std::optional<bool> Flag(const val& obj, const char* key) {
    const val v = obj[key];
    if (v.isUndefined()) return std::nullopt;
    return v.as<bool>();  // JS truthiness (!!value), like the original
  }

  void Configure(val args) {
    if (args.isUndefined() || args.isNull()) return;
    XrConfigureOptions o;
    o.enable_world_points = Flag(args, "enableFeatureSet");
    if (const std::optional<bool> w = Flag(args, "enableWorldPoints")) o.enable_world_points = w;
    if (!args["scale"].isUndefined()) o.absolute_scale = args["scale"].strictlyEquals(val("absolute"));
    o.enable_lighting = Flag(args, "enableLighting");
    o.left_handed_axes = Flag(args, "leftHandedAxes");
    o.mirrored_display = Flag(args, "mirroredDisplay");
    o.disable_world_tracking = Flag(args, "disableWorldTracking");
    const XrError e = glue_->Configure(o);
    if (!e.ok()) ThrowXr(e);
  }

  void UpdateCameraProjectionMatrix(val args) {
    CameraProjectionUpdate u;
    const val cam = args["cam"];
    if (!cam.isUndefined() && !cam.isNull()) {
      for (const char* key : {"pixelRectWidth", "pixelRectHeight"}) {
        if (cam[key].isUndefined()) continue;
        const double d = cam[key].as<double>();
        if (!(d >= 0.0 && d <= 65536.0) || d != std::floor(d)) {
          ThrowJs(std::string("cam.") + key + " must be an integer in [0, 65536]");
        }
        (std::string(key) == "pixelRectWidth" ? u.pixel_rect_width : u.pixel_rect_height) =
            static_cast<int>(d);
      }
      if (!cam["nearClipPlane"].isUndefined()) {
        u.near_clip_plane = FiniteFloat(cam["nearClipPlane"], "cam.nearClipPlane");
      }
      if (!cam["farClipPlane"].isUndefined()) {
        u.far_clip_plane = FiniteFloat(cam["farClipPlane"], "cam.farClipPlane");
      }
    }
    const val origin = args["origin"];
    if (!origin.isUndefined() && !origin.isNull()) {
      u.origin = Vec3f{FiniteFloat(origin["x"], "origin.x"), FiniteFloat(origin["y"], "origin.y"),
                       FiniteFloat(origin["z"], "origin.z")};
    }
    const val facing = args["facing"];
    if (!facing.isUndefined() && !facing.isNull()) {
      u.facing = QuatWxyz{FiniteFloat(facing["w"], "facing.w"), FiniteFloat(facing["x"], "facing.x"),
                          FiniteFloat(facing["y"], "facing.y"), FiniteFloat(facing["z"], "facing.z")};
    }
    if (!args["updateRecenterPoint"].isUndefined()) {
      u.update_recenter_point = args["updateRecenterPoint"].as<bool>();
    }
    glue_->UpdateCameraProjectionMatrix(u);
  }

  void Recenter() { glue_->Recenter(); }

  // DetectedImageTarget fields of tracking-controller.ts:700-728 that come from the engine
  // (type apiName, :86-96 + reality/shared/engine/image-targets.ts:6-9: FLAT, CYLINDRICAL, CONICAL).
  static val DetectedImage(const DetectedImageOut& d) {
    val o = val::object();
    o.set("name", d.name);
    o.set("id", static_cast<double>(d.id));
    const bool conical = std::fabs(d.curvy_geometry.top_radius - d.curvy_geometry.bottom_radius) > 0.001f;
    o.set("type", std::string(d.type == DetectedImageType::kPlanar ? "FLAT"
                              : d.type == DetectedImageType::kCurvy ? (conical ? "CONICAL" : "CYLINDRICAL")
                                                                    : "UNSPECIFIED"));
    val p = val::object();
    p.set("x", d.position.x);
    p.set("y", d.position.y);
    p.set("z", d.position.z);
    o.set("position", p);
    val r = val::object();
    r.set("x", d.rotation.x);
    r.set("y", d.rotation.y);
    r.set("z", d.rotation.z);
    r.set("w", d.rotation.w);
    o.set("rotation", r);
    o.set("widthInMeters", d.width_in_meters);
    o.set("heightInMeters", d.height_in_meters);
    const float m = std::fmax(d.width_in_meters, d.height_in_meters);
    o.set("scale", std::isnan(d.width_in_meters) || std::isnan(d.height_in_meters) || m == 0.0f ? 1.0f : m);
    o.set("trackingStatus", static_cast<int>(d.tracking_status));
    if (d.type == DetectedImageType::kCurvy) {
      const CurvyGeometryOut& c = d.curvy_geometry;
      val g = val::object();
      g.set("curvyCircumferenceTop", c.curvy_circumference_top);
      g.set("curvyCircumferenceBottom", c.curvy_circumference_bottom);
      g.set("curvySideLength", c.curvy_side_length);
      g.set("targetCircumferenceTop", c.target_circumference_top);
      g.set("height", c.height);
      g.set("topRadius", c.top_radius);
      g.set("bottomRadius", c.bottom_radius);
      g.set("arcLengthRadians", c.arc_length_radians);
      g.set("arcStartRadians", c.arc_start_radians);
      o.set("curvyGeometry", g);
    }
    return o;
  }

  void SetImageTargets(val targets) {
    if (targets.isUndefined() || targets.isNull()) ThrowJs("setImageTargets needs an array");
    const size_t n = targets["length"].as<size_t>();
    std::vector<std::vector<uint8_t>> pixels(n);
    std::vector<ImageTargetInput> in(n);
    for (size_t i = 0; i < n; ++i) {
      const val t = targets[i];
      if (!t["json"].isString()) ThrowJs("image target json must be a string");
      const int w = Get<int>(t, "width", 0), h = Get<int>(t, "height", 0);
      if (w <= 0 || h <= 0) ThrowJs("image target width and height must be positive");
      const size_t bytes = static_cast<size_t>(w) * static_cast<size_t>(h);
      if (t["luminance"].isUndefined() || ByteLength(t["luminance"]) != bytes) {
        ThrowJs("image target luminance length differs from width * height");
      }
      pixels[i].resize(bytes);
      val(emscripten::typed_memory_view(bytes, pixels[i].data())).call<void>("set", t["luminance"]);
      in[i].json = t["json"].as<std::string>();
      in[i].luminance = GrayImageView{pixels[i].data(), w, h, w};
    }
    const std::string error = glue_->SetImageTargets(in);
    if (!error.empty()) ThrowJs(error);
  }

  void UnloadImageTargets(val names) {
    std::vector<std::string> v;
    const size_t n = names["length"].as<size_t>();
    for (size_t i = 0; i < n; ++i) v.push_back(names[i].as<std::string>());
    glue_->UnloadImageTargets(v);
  }

  void ClearImageTargets() { glue_->ClearImageTargets(); }

  val GetRequestedRois() const {
    val out = val::array();
    for (const PyramidRoiRequest& r : glue_->NextDrawRois()) {
      val o = val::object();
      o.set("slot", r.slot);
      o.set("source", std::string(r.source == PyramidRoiSource::kImageTarget       ? "IMAGE_TARGET"
                                  : r.source == PyramidRoiSource::kHiresScan       ? "HIRES_SCAN"
                                                                                   : "CURVY_IMAGE_TARGET"));
      o.set("sourceId", static_cast<int>(r.source));
      o.set("faceId", r.face_id);
      o.set("name", r.name);
      o.set("warp", val::global("Float32Array").new_(val(emscripten::typed_memory_view(16, r.warp))));
      if (r.curvy.is_curvy) {
        const PyramidRoiCurvy& c = r.curvy;
        val u = val::object();
        u.set("rotation", c.rotation);
        u.set("curvyPostRotation", c.curvy_post_rotation);
        const auto pair = [](const float* v) {
          val a = val::array();
          a.call<void>("push", v[0]);
          a.call<void>("push", v[1]);
          return a;
        };
        u.set("targetDims", pair(c.target_dims));
        u.set("roiDims", pair(c.roi_dims));
        u.set("searchDims", pair(c.search_dims));
        u.set("intrinsics", val::global("Float32Array").new_(val(emscripten::typed_memory_view(16, c.intrinsics))));
        u.set("globalPoseInverse",
              val::global("Float32Array").new_(val(emscripten::typed_memory_view(16, c.global_pose_inverse))));
        u.set("scales", pair(c.scales));
        u.set("shift", c.shift);
        u.set("radius", c.radius);
        u.set("radiusBottom", c.radius_bottom);
        o.set("curvy", u);
      }
      out.call<void>("push", o);
    }
    return out;
  }

  val GetImageTargetNames() const {
    val out = val::array();
    for (const std::string& n : glue_->image_target_names()) out.call<void>("push", n);
    return out;
  }

  // US-030 / D-1: the load-time CurvyGeometry of a curvy target (9 capnp fields as in DetectedImage.curvyGeometry); undefined for a
  // flat or unknown target.
  val GetImageTargetGeometry(const std::string& name) const {
    CurvyGeometryOut c;
    if (!glue_->image_target_geometry(name, &c)) return val::undefined();
    val g = val::object();
    g.set("curvyCircumferenceTop", c.curvy_circumference_top);
    g.set("curvyCircumferenceBottom", c.curvy_circumference_bottom);
    g.set("curvySideLength", c.curvy_side_length);
    g.set("targetCircumferenceTop", c.target_circumference_top);
    g.set("height", c.height);
    g.set("topRadius", c.top_radius);
    g.set("bottomRadius", c.bottom_radius);
    g.set("arcLengthRadians", c.arc_length_radians);
    g.set("arcStartRadians", c.arc_start_radians);
    return g;
  }

  static const char* XrHitTypeName(XrHitType t) {
    switch (t) {
      case XrHitType::kFeaturePoint: return "FEATURE_POINT";
      case XrHitType::kEstimatedSurface: return "ESTIMATED_SURFACE";
      case XrHitType::kDetectedSurface: return "DETECTED_SURFACE";
      default: return "UNSPECIFIED";
    }
  }

  val XrHitTest(double x, double y, val types) const {
    if (!std::isfinite(x) || !std::isfinite(y)) ThrowJs("xrHitTest x, y must be finite");
    std::vector<XrHitType> included;
    if (!types.isUndefined() && !types.isNull()) {
      const size_t n = types["length"].as<size_t>();
      for (size_t i = 0; i < n; ++i) {
        // Unknown names stay UNSPECIFIED (0), like the original's initIncludedTypes.
        const val t = types[i];
        included.push_back(t.strictlyEquals(val("FEATURE_POINT"))       ? XrHitType::kFeaturePoint
                           : t.strictlyEquals(val("ESTIMATED_SURFACE")) ? XrHitType::kEstimatedSurface
                           : t.strictlyEquals(val("DETECTED_SURFACE"))  ? XrHitType::kDetectedSurface
                                                                        : XrHitType::kUnspecified);
      }
    }
    val out = val::array();
    for (const XrHitResult& h :
         glue_->XrHitTest(static_cast<float>(x), static_cast<float>(y), included)) {
      val o = val::object();
      o.set("type", std::string(XrHitTypeName(h.type)));
      val p = val::object();
      p.set("x", h.position.x);
      p.set("y", h.position.y);
      p.set("z", h.position.z);
      o.set("position", p);
      val r = val::object();
      r.set("x", h.rotation.x);
      r.set("y", h.rotation.y);
      r.set("z", h.rotation.z);
      r.set("w", h.rotation.w);
      o.set("rotation", r);
      o.set("distance", h.distance);
      out.call<void>("push", o);
    }
    return out;
  }

  void OnCameraStatusChange(std::string status, std::string direction) {
    const XrError e = glue_->OnCameraStatusChange(
        status, direction == "back" ? CameraDirection::kBack : CameraDirection::kFront);
    if (!e.ok()) ThrowXr(e);
  }

 private:
  std::unique_ptr<GlueRunner> glue_;
  frontend::PyramidLayout layout_;
  int rotation_deg_ = 0;
  int capture_width_ = 0;
  int capture_height_ = 0;
  std::vector<uint8_t> pixels_;
  std::vector<uint8_t> atlas_;
};

EMSCRIPTEN_BINDINGS(slam_refact_engine) {
  emscripten::class_<SlamRefactGlue>("SlamRefactGlue")
      .constructor<val>()
      .function("pushDeviceMotion", &SlamRefactGlue::PushDeviceMotion)
      .function("pushDeviceMotionEvent", &SlamRefactGlue::PushDeviceMotionEvent)
      .function("getFramesUntilRecenter", &SlamRefactGlue::GetFramesUntilRecenter)
      .function("pushDeviceOrientation", &SlamRefactGlue::PushDeviceOrientation)
      .function("processFrame", &SlamRefactGlue::ProcessFrame)
      .function("processFramePyramid", &SlamRefactGlue::ProcessFramePyramid)
      .function("pyramidLayout", &SlamRefactGlue::PyramidLayout)
      .class_function("cpuPyramidAtlas", &SlamRefactGlue::CpuPyramidAtlas)
      .function("getPose", &SlamRefactGlue::GetPose)
      .function("getWorldPoints", &SlamRefactGlue::GetWorldPoints)
      .function("getSurfaces", &SlamRefactGlue::GetSurfaces)
      .function("hitTest", &SlamRefactGlue::HitTest)
      .function("reset", &SlamRefactGlue::Reset)
      .function("configure", &SlamRefactGlue::Configure)
      .function("updateCameraProjectionMatrix", &SlamRefactGlue::UpdateCameraProjectionMatrix)
      .function("recenter", &SlamRefactGlue::Recenter)
      .function("xrHitTest", &SlamRefactGlue::XrHitTest)
      .function("onCameraStatusChange", &SlamRefactGlue::OnCameraStatusChange)
      .function("getIntrinsic", &SlamRefactGlue::GetIntrinsic)
      .function("setImageTargets", &SlamRefactGlue::SetImageTargets)
      .function("unloadImageTargets", &SlamRefactGlue::UnloadImageTargets)
      .function("clearImageTargets", &SlamRefactGlue::ClearImageTargets)
      .function("getImageTargetNames", &SlamRefactGlue::GetImageTargetNames)
      .function("getImageTargetGeometry", &SlamRefactGlue::GetImageTargetGeometry)
      .function("getRequestedRois", &SlamRefactGlue::GetRequestedRois);
}

}  // namespace
}  // namespace slam_refact::wasm
