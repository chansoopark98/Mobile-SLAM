// Contract test for the c8-free public API (include/slam_refact/**). The static_asserts pin the
// layouts and enum values that the design fixes (architecture.md §4.1, §4.2; S2 §2.0, S4 §2.1,
// §2.6). A failing assert means a frozen contract was changed without contract-changes.md.

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "gtest/gtest.h"
#include "slam_refact/engine.h"
#include "slam_refact/engine_config.h"
#include "slam_refact/glue_runner.h"
#include "slam_refact/image_types.h"
#include "slam_refact/sensor_types.h"
#include "slam_refact/types.h"
#include "slam_refact/version.h"
#include "slam_refact/xr_controller.h"

namespace slam_refact {
namespace {

// ---- Value types ----
static_assert(sizeof(Vec3f) == 12 && std::is_standard_layout_v<Vec3f>);
static_assert(offsetof(Vec3f, x) == 0 && offsetof(Vec3f, y) == 4 && offsetof(Vec3f, z) == 8);
// Public quaternions are (w, x, y, z).
static_assert(sizeof(QuatWxyz) == 16 && offsetof(QuatWxyz, w) == 0 &&
              offsetof(QuatWxyz, x) == 4 && offsetof(QuatWxyz, y) == 8 &&
              offsetof(QuatWxyz, z) == 12);
static_assert(sizeof(HMatrix4f) == 64 && std::is_standard_layout_v<HMatrix4f>);

// PinholeIntrinsics mirrors c8_PixelPinholeCameraModel field order (24 B).
static_assert(sizeof(PinholeIntrinsics) == 24);
static_assert(offsetof(PinholeIntrinsics, width) == 0 && offsetof(PinholeIntrinsics, height) == 4 &&
              offsetof(PinholeIntrinsics, cx) == 8 && offsetof(PinholeIntrinsics, cy) == 12 &&
              offsetof(PinholeIntrinsics, fx) == 16 && offsetof(PinholeIntrinsics, fy) == 20);

// XrTrackingState enums (u16; S4 §2.6).
static_assert(std::is_same_v<std::underlying_type_t<TrackingStatus>, uint16_t>);
static_assert(static_cast<int>(TrackingStatus::kUnspecified) == 0 &&
              static_cast<int>(TrackingStatus::kNotAvailable) == 1 &&
              static_cast<int>(TrackingStatus::kLimited) == 2 &&
              static_cast<int>(TrackingStatus::kNormal) == 3);
static_assert(std::is_same_v<std::underlying_type_t<TrackingReason>, uint16_t>);
static_assert(static_cast<int>(TrackingReason::kUnspecified) == 0 &&
              static_cast<int>(TrackingReason::kInitializing) == 1 &&
              static_cast<int>(TrackingReason::kRelocalizing) == 2 &&
              static_cast<int>(TrackingReason::kTooMuchMotion) == 3 &&
              static_cast<int>(TrackingReason::kNotEnoughTexture) == 4);

static_assert(std::is_same_v<ParamValue, std::variant<bool, int, float, double>>);

// ---- Sensor types (D-14) ----
// capnp numbering of raw events.
static_assert(static_cast<int>(RawSensorKind::kAccelerometer) == 1 &&
              static_cast<int>(RawSensorKind::kGyroscope) == 2 &&
              static_cast<int>(RawSensorKind::kMagnetometer) == 3 &&
              static_cast<int>(RawSensorKind::kLinearAcceleration) == 4);
// Post-647 numbering (= c8::TrackingSensorEvent kinds).
static_assert(std::is_same_v<std::underlying_type_t<ImuEvent::Kind>, int32_t>);
static_assert(static_cast<int>(ImuEvent::Kind::kAccelerometer) == 1 &&
              static_cast<int>(ImuEvent::Kind::kMagnetometer) == 2 &&
              static_cast<int>(ImuEvent::Kind::kGyroscope) == 3 &&
              static_cast<int>(ImuEvent::Kind::kLinearAcceleration) == 4);
// 40-byte TrackingSensorEvent layout (S2 §2.0, S4 §2.1, S5 §2.4): same offsets on wasm32 and
// x86-64 because the largest member is int64.
static_assert(sizeof(ImuEvent) == 40);
static_assert(offsetof(ImuEvent, kind) == 0 && offsetof(ImuEvent, event_time_ns) == 8 &&
              offsetof(ImuEvent, interval_ns) == 16 && offsetof(ImuEvent, x) == 24 &&
              offsetof(ImuEvent, y) == 28 && offsetof(ImuEvent, z) == 32);
// A raw event and a post-647 event are unrelated types: no implicit conversion either way.
static_assert(!std::is_convertible_v<ImuEvent, RawSensorEvent>);
static_assert(!std::is_convertible_v<RawSensorEvent, ImuEvent>);
static_assert(!std::is_convertible_v<RawSensorKind, ImuEvent::Kind>);
// Engine input carries post-647 events only.
static_assert(std::is_same_v<decltype(FrameInput::imu_events), std::vector<ImuEvent>>);

// ---- Engine / glue surface (§4.3) ----
static_assert(std::has_virtual_destructor_v<TrackerCore>);
static_assert(std::is_abstract_v<TrackerCore>);
static_assert(std::is_base_of_v<TrackerCore, Engine> && std::is_final_v<Engine>);
static_assert(!std::is_copy_constructible_v<Engine> && !std::is_copy_assignable_v<Engine>);
static_assert(!std::is_copy_constructible_v<GlueRunner>);
static_assert(std::is_constructible_v<Engine, const EngineConfig&>);
static_assert(std::is_constructible_v<GlueRunner, EngineConfig, GlueConfig>);
static_assert(std::is_constructible_v<GlueRunner, EngineConfig, GlueConfig, TrackerCoreFactory>);
static_assert(std::is_same_v<decltype(&TrackerCore::Track), TrackerStep (TrackerCore::*)(const FrameInput&)>);
static_assert(std::is_same_v<decltype(&Engine::Configure), void (Engine::*)(const EngineConfig&)>);
static_assert(std::is_same_v<decltype(&GlueRunner::Publish), PublishedFrame (GlueRunner::*)(FrameInput)>);
static_assert(std::is_same_v<decltype(&GlueRunner::ProcessFrame),
                             PublishedFrame (GlueRunner::*)(int64_t, int64_t, const GrayImageView&,
                                                            PyramidAtlasView)>);
static_assert(std::is_same_v<decltype(&GlueRunner::PushDeviceMotion),
                             void (GlueRunner::*)(const DeviceMotionSample&)>);
static_assert(std::is_same_v<decltype(&GlueRunner::PushDeviceOrientation),
                             void (GlueRunner::*)(const DeviceOrientationSample&)>);
static_assert(std::is_same_v<decltype(&GlueRunner::Recenter), void (GlueRunner::*)()>);
static_assert(std::is_same_v<decltype(&GlueRunner::last_published),
                             const PublishedFrame& (GlueRunner::*)() const>);
static_assert(std::is_same_v<decltype(&DefaultEngineFactory),
                             std::unique_ptr<TrackerCore> (*)(const EngineConfig&)>);
static_assert(std::is_same_v<decltype(TrackerStep::num_points_by_source), int[5]>);

// FULL-1 (contract-changes.md): XrController API on GlueRunner (spec 06).
static_assert(std::is_same_v<decltype(&GlueRunner::Configure),
                             XrError (GlueRunner::*)(const XrConfigureOptions&)>);
static_assert(std::is_same_v<decltype(&GlueRunner::UpdateCameraProjectionMatrix),
                             void (GlueRunner::*)(const CameraProjectionUpdate&)>);
static_assert(std::is_same_v<decltype(&GlueRunner::XrHitTest),
                             std::vector<XrHitResult> (GlueRunner::*)(
                                 float, float, const std::vector<XrHitType>&) const>);
static_assert(std::is_same_v<decltype(&GlueRunner::OnCameraStatusChange),
                             XrError (GlueRunner::*)(const std::string&, CameraDirection)>);
static_assert(std::is_same_v<decltype(PublishedFrame::intrinsics), std::array<float, 16>>);
static_assert(static_cast<int>(XrHitType::kUnspecified) == 0 &&
              static_cast<int>(XrHitType::kFeaturePoint) == 1 &&
              static_cast<int>(XrHitType::kEstimatedSurface) == 2 &&
              static_cast<int>(XrHitType::kDetectedSurface) == 3);

// FULL-3 (contract-changes.md): image targets (image_targets.h, US-013).
static_assert(kMaxActiveImageTargets == 32 && kMaxTrackedImageTargets == 4);
static_assert(std::is_same_v<decltype(FrameInput::image_targets), const ImageTargetSet*>);
static_assert(std::is_same_v<decltype(TrackerStep::located_images), std::vector<LocatedImageOut>>);
static_assert(std::is_same_v<decltype(PublishedFrame::detected_images), std::vector<DetectedImageOut>>);
static_assert(std::is_same_v<decltype(PublishedFrame::image_events), std::vector<ImageTargetEvent>>);
static_assert(std::is_same_v<decltype(&GlueRunner::SetImageTargets),
                             std::string (GlueRunner::*)(const std::vector<ImageTargetInput>&)>);
static_assert(std::is_same_v<decltype(&GlueRunner::UnloadImageTargets),
                             void (GlueRunner::*)(const std::vector<std::string>&)>);
static_assert(std::is_same_v<decltype(&GlueRunner::ClearImageTargets), void (GlueRunner::*)()>);
static_assert(static_cast<int>(DetectedImageType::kPlanar) == 1 &&
              static_cast<int>(DetectedImageType::kCurvy) == 2);
static_assert(static_cast<int>(ImageTargetEventKind::kFound) == 0 &&
              static_cast<int>(ImageTargetEventKind::kUpdated) == 1 &&
              static_cast<int>(ImageTargetEventKind::kLost) == 2);

// FULL-1 field types (RM-3a: the fields and defaults below are pinned, see TEST FullOneDefaults).
static_assert(std::is_same_v<decltype(PublishedFrame::has_lighting), bool>);
static_assert(std::is_same_v<decltype(PublishedFrame::lighting), LightingEstimate>);
static_assert(std::is_same_v<decltype(PublishedFrame::tracking_status_changed), bool>);
static_assert(std::is_same_v<decltype(FrameInput::estimate_lighting), bool>);
static_assert(std::is_same_v<decltype(TrackerStep::has_lighting), bool>);
static_assert(std::is_same_v<decltype(TrackerStep::lighting_exposure), float>);
static_assert(std::is_same_v<decltype(GraphicsIntrinsics::texture_width), int> &&
              std::is_same_v<decltype(GraphicsIntrinsics::near_clip), float> &&
              std::is_same_v<decltype(GraphicsIntrinsics::far_clip), float>);
static_assert(std::is_same_v<std::underlying_type_t<CameraDirection>, uint8_t> &&
              static_cast<int>(CameraDirection::kBack) == 0 &&
              static_cast<int>(CameraDirection::kFront) == 1);
static_assert(std::is_same_v<std::underlying_type_t<XrHitType>, uint8_t>);
static_assert(std::is_same_v<decltype(GlueConfig::detect_surfaces), bool>);

// FULL-6 (RM-6): ROI request of the next draw (c8::ImageRoi::Source numbering, mvp = 16 floats).
static_assert(static_cast<int>(PyramidRoiSource::kImageTarget) == 2 &&
              static_cast<int>(PyramidRoiSource::kHiresScan) == 3 &&
              static_cast<int>(PyramidRoiSource::kCurvyImageTarget) == 6);
static_assert(sizeof(PyramidRoiRequest::warp) == 16 * sizeof(float));
static_assert(std::is_same_v<decltype(LocatedImageOut::roi_feats), bool> &&
              std::is_same_v<decltype(LocatedImageOut::local_inliers), int32_t> &&
              std::is_same_v<decltype(LocatedImageOut::global_inliers), int32_t>);

// FULL-7 (RM-4b): the startup recenter counter of the tracking-controller (original default 5).
static_assert(std::is_same_v<decltype(GlueConfig::frames_until_recenter), int>);
static_assert(std::is_same_v<decltype(&GlueRunner::frames_until_recenter), int (GlueRunner::*)() const>);

// FULL-8 (US-023 / US-024): behaviour of EngineConfig::capture_width / capture_height / rotation_deg (R16 lifted) and of the
// published pose / projection / hit input per device orientation; no new public type or function, so no signature checks.
// FULL-9 (US-014): the uniforms of a curvy ROI slot draw travel with the ROI request; curvy (CYLINDER / CONICAL)
// targets load and track; no other public type or function changes.
static_assert(std::is_same_v<decltype(PyramidRoiRequest::curvy), PyramidRoiCurvy>);
static_assert(sizeof(PyramidRoiCurvy::intrinsics) == 16 * sizeof(float));
static_assert(sizeof(PyramidRoiCurvy::global_pose_inverse) == 16 * sizeof(float));
static_assert(std::is_same_v<decltype(PyramidRoiCurvy::is_curvy), bool>);
static_assert(static_cast<int>(PyramidRoiSource::kCurvyImageTarget) == 6);
// FULL-14 (US-030 D-1): GlueRunner::image_target_geometry added; 9 -> 10.
static_assert(std::is_same_v<decltype(&GlueRunner::image_target_geometry),
                             bool (GlueRunner::*)(const std::string&, CurvyGeometryOut*) const>);
static_assert(kContractVersion == 10);

TEST(ContractPublic, DefaultValuesMatchDesign) {
  const HMatrix4f identity;
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      EXPECT_EQ(identity.m[4 * c + r], r == c ? 1.f : 0.f) << "r=" << r << " c=" << c;
    }
  }
  const QuatWxyz q;
  EXPECT_EQ(q.w, 1.f);
  EXPECT_EQ(q.x, 0.f);

  // GlueConfig defaults (§4.1): origin (0, 2, 0), identity facing, no axis flips.
  const GlueConfig glue;
  EXPECT_EQ(glue.origin.x, 0.f);
  EXPECT_EQ(glue.origin.y, 2.f);
  EXPECT_EQ(glue.origin.z, 0.f);
  EXPECT_EQ(glue.facing.w, 1.f);
  EXPECT_FALSE(glue.left_handed_axes);
  EXPECT_FALSE(glue.mirrored_display);

  // EngineConfig defaults: responsive scale, VIO on, IEEE shader model (D-9).
  const EngineConfig cfg;
  EXPECT_FALSE(cfg.estimate_scale);
  EXPECT_FALSE(cfg.disable_vio);
  EXPECT_EQ(cfg.rotation_deg, 0);
  EXPECT_EQ(cfg.pyramid_model, ShaderNumericModel::kIeee);
  EXPECT_FALSE(cfg.intrinsics_override.has_value());
  EXPECT_TRUE(cfg.param_overrides.empty());

  // Injected pyramid is absent by default (gray/rgba path).
  const FrameInput in;
  EXPECT_EQ(in.pyramid.rgba, nullptr);
  EXPECT_TRUE(in.imu_events.empty());

  const WorldPointOut wp;
  EXPECT_EQ(wp.id, 0u);
  EXPECT_EQ(wp.confidence, 0.f);
}

// FULL-1 defaults (spec 06): the values the glue and the original JS start from.
TEST(ContractPublic, FullOneDefaults) {
  // The original JS `cam`: pixelRect 0 = canvas size, near 0.01, far 1000 (xr-slam.js n).
  const GraphicsIntrinsics g;
  EXPECT_EQ(g.texture_width, 0);
  EXPECT_EQ(g.texture_height, 0);
  EXPECT_EQ(g.near_clip, 0.01f);
  EXPECT_EQ(g.far_clip, 1000.0f);

  // updateCameraProjectionMatrix: nothing set, updateRecenterPoint = true (xA default).
  const CameraProjectionUpdate u;
  EXPECT_FALSE(u.pixel_rect_width.has_value() || u.pixel_rect_height.has_value() ||
               u.near_clip_plane.has_value() || u.far_clip_plane.has_value() ||
               u.origin.has_value() || u.facing.has_value());
  EXPECT_TRUE(u.update_recenter_point);

  // configure(): every option unset.
  const XrConfigureOptions o;
  EXPECT_FALSE(o.enable_world_points.has_value() || o.absolute_scale.has_value() ||
               o.enable_lighting.has_value() || o.left_handed_axes.has_value() ||
               o.mirrored_display.has_value() || o.disable_world_tracking.has_value());

  // The hit result: rotation is the capnp default zero quaternion (w = 0, not identity).
  const XrHitResult h;
  EXPECT_EQ(h.type, XrHitType::kUnspecified);
  EXPECT_EQ(h.rotation.w, 0.f);
  EXPECT_EQ(h.rotation.x, 0.f);
  EXPECT_EQ(h.rotation.y, 0.f);
  EXPECT_EQ(h.rotation.z, 0.f);
  EXPECT_EQ(h.distance, 0.f);

  const LightingEstimate l;
  EXPECT_EQ(l.exposure, 0.f);
  EXPECT_EQ(l.temperature, 0.f);

  XrError e;
  EXPECT_TRUE(e.ok());
  e.message = "x";
  EXPECT_FALSE(e.ok());

  // New published / input fields: off or zero by default (default output unchanged).
  PublishedFrame f;
  EXPECT_FALSE(f.has_lighting);
  EXPECT_EQ(f.lighting.exposure, 0.f);
  EXPECT_EQ(f.lighting.temperature, 0.f);
  EXPECT_FALSE(f.tracking_status_changed);
  for (const float v : f.intrinsics) EXPECT_EQ(v, 0.f);
  FrameInput in_full;
  EXPECT_FALSE(in_full.estimate_lighting);
  TrackerStep step;
  EXPECT_FALSE(step.has_lighting);
  EXPECT_EQ(step.lighting_exposure, 0.f);

  // Surfaces extension is opt-in.
  EXPECT_FALSE(GlueConfig{}.detect_surfaces);
  EXPECT_EQ(GlueConfig{}.frames_until_recenter, 5);  // tracking-controller.cc:85
}

}  // namespace
}  // namespace slam_refact
