// Contract test for the frozen internal headers: src/common/tracking_types.h and the one public
// header per module (architecture.md §2.1 [M3], §3). Signatures are checked with decltype so a
// work package cannot change a frozen declaration silently; the bodies do not need to exist.

#include <array>
#include <cstdint>
#include <map>
#include <set>
#include <type_traits>
#include <vector>

#include "c8/geometry/pixel-pinhole-camera-model.h"
#include "common/c8_adapters.h"
#include "common/tracking_types.h"
#include "compat/random_numbers.h"
#include "frame_frame_tracker/frame_frame_tracker.h"
#include "frontend/frontend.h"
#include "gtest/gtest.h"
#include "map/world_map.h"
#include "map_builder/map_builder.h"
#include "map_tracker/map_tracker.h"
#include "metric_scale/metric_scale_estimator.h"
#include "slam_refact/types.h"
#include "vio_tracker/vio_tracker.h"

namespace slam_refact {
namespace {

// ---- tracking_types.h ----
// D-14: one event type for the public API and the engine.
static_assert(std::is_same_v<TrackingSensorEvent, ImuEvent>);
static_assert(std::is_same_v<decltype(TrackingSensorFrame::sensor_events),
                             std::vector<TrackingSensorEvent>>);
static_assert(std::is_same_v<decltype(TrackerInput::frame_points), c8::FrameWithPoints>);
static_assert(!std::is_default_constructible_v<TrackerInput>);
static_assert(std::is_same_v<map::PointId, c8::WorldMapPointId>);
static_assert(std::is_same_v<map::KeyframeId, c8::WorldMapKeyframeId>);

// Point source / state numbering (S2 §2.3, S3 §2.5).
static_assert(std::is_same_v<std::underlying_type_t<map::PointSource>, int32_t>);
static_assert(static_cast<int>(map::PointSource::kTriangulated) == 1 &&
              static_cast<int>(map::PointSource::kGround) == 2 &&
              static_cast<int>(map::PointSource::kSky) == 3 &&
              static_cast<int>(map::PointSource::kDepth) == 4 &&
              static_cast<int>(map::PointSource::kCloud) == 5);
static_assert(static_cast<int>(map::PointState::kNew) == 0 &&
              static_cast<int>(map::PointState::kTenured) == 1 &&
              static_cast<int>(map::PointState::kDead) == 2);

// MapTrackerOutput / MapBuilderOutput field types (S2 §2.2, S3 §2.6, D-5, D-7).
static_assert(std::is_same_v<decltype(MapTrackerOutput::state), int32_t>);
static_assert(std::is_same_v<decltype(MapTrackerOutput::frame_tracker_ok), bool>);
static_assert(std::is_same_v<decltype(MapTrackerOutput::consecutive_frame_tracker_ok), int64_t>);
static_assert(std::is_same_v<decltype(MapTrackerOutput::ground_idx), std::vector<uint32_t>>);
static_assert(std::is_same_v<decltype(MapTrackerOutput::visible_ids), std::vector<map::PointId>>);
static_assert(std::is_same_v<decltype(MapTrackerOutput::tracked), std::map<uint32_t, map::PointId>>);
static_assert(std::is_same_v<decltype(MapTrackerOutput::predicted), FrameState>);
static_assert(std::is_same_v<decltype(MapBuilderOutput::new_points), std::vector<map::PointId>>);
static_assert(std::is_same_v<decltype(MapBuilderOutput::world_correction), c8::HMatrix>);
static_assert(std::is_same_v<decltype(FrameState::time_ns), int64_t>);

// PinholeIntrinsics and c8_PixelPinholeCameraModel share layout (c8_adapters.h copies fields).
static_assert(sizeof(c8_PixelPinholeCameraModel) == sizeof(PinholeIntrinsics));

// RNG seed (§5.2).
static_assert(compat::kEngineRngSeed == 5489u);
static_assert(std::is_same_v<compat::RandomNumbers, c8::RandomNumbers>);

// ---- frontend.h (§3.2) ----
namespace fe = frontend;
static_assert(std::is_same_v<decltype(&fe::PrepareTrackingSensorEvents),
                             std::vector<ImuEvent> (*)(const std::vector<RawSensorEvent>&, bool)>);
// D-14 compile-time guard: func 647 cannot be applied to post-647 events.
static_assert(!std::is_invocable_v<decltype(&fe::PrepareTrackingSensorEvents),
                                   std::vector<ImuEvent>, bool>);
static_assert(std::is_same_v<decltype(&fe::SensorIntake::TakeForStage),
                             fe::StagedSensors (fe::SensorIntake::*)()>);
static_assert(std::is_constructible_v<fe::SensorIntake, bool>);
static_assert(std::is_same_v<decltype(&fe::GetQuaternion), c8::Quaternion (*)(double, double, double)>);
static_assert(std::is_same_v<decltype(&fe::ComputePyramidLayout),
                             fe::PyramidLayout (*)(int, int, int)>);
static_assert(std::is_same_v<decltype(&fe::FeatureFrontend::Detect),
                             void (fe::FeatureFrontend::*)(const c8::Gr8Pyramid&,
                                                           const c8::DetectionConfig&,
                                                           c8::FrameWithPoints*)>);
static_assert(std::is_same_v<decltype(&fe::CpuGr8Pyramid::Build),
                             void (fe::CpuGr8Pyramid::*)(const RgbaImageView&, int, fe::RgbaAtlas*)
                                 const>);

// ---- world_map.h (§3.3) ----
static_assert(std::is_same_v<decltype(&map::WorldMap::AddKeyframe),
                             map::KeyframeId (map::WorldMap::*)(const c8::FrameWithPoints&, int64_t)>);
static_assert(std::is_same_v<decltype(&map::WorldMap::AddPoint),
                             map::PointId (map::WorldMap::*)(uint32_t, const c8::HPoint3&,
                                                             map::PointSource)>);
static_assert(std::is_same_v<decltype(&map::WorldMap::AllPointIds),
                             std::vector<map::PointId> (map::WorldMap::*)() const>);
static_assert(std::is_same_v<decltype(&map::WorldMap::CovisibleKeyframes),
                             std::vector<map::KeyframeId> (map::WorldMap::*)(map::KeyframeId, int)
                                 const>);
static_assert(std::is_same_v<decltype(&map::ProjectPoints),
                             map::ProjectedPoints (*)(const map::WorldMap&, std::vector<map::PointId>,
                                                      const PinholeIntrinsics&, const c8::HMatrix&)>);
static_assert(std::is_same_v<decltype(&map::VerifyPointVisibility),
                             map::VisibilityResult (*)(const map::WorldMap&,
                                                       const PinholeIntrinsics&, const c8::HMatrix&,
                                                       const std::vector<map::PointId>&)>);
static_assert(!std::is_copy_constructible_v<map::WorldMap>);

// ---- map_tracker.h (§3.4) / frame_frame_tracker.h (§3.5) ----
namespace mt = map_tracker;
static_assert(std::is_same_v<decltype(&mt::MapTracker::Track),
                             void (mt::MapTracker::*)(map::WorldMap&, TrackerInput*,
                                                      const FrameState&, MapTrackerOutput*,
                                                      compat::RandomNumbers*)>);
static_assert(std::is_same_v<decltype(&mt::MapTracker::ApplyMapBuilderOutput),
                             c8::HMatrix (mt::MapTracker::*)(const MapBuilderOutput&)>);
static_assert(std::is_same_v<decltype(&mt::YawOf), float (*)(c8::Quaternion)>);
static_assert(std::is_same_v<decltype(&mt::ReplaceYaw),
                             c8::HMatrix (*)(const c8::HMatrix&, const c8::Quaternion&)>);
static_assert(std::is_same_v<decltype(&mt::Relocalize),
                             void (*)(RelocResult*, const map::WorldMap&, const TrackerInput&,
                                      const c8::HMatrix&, compat::RandomNumbers*)>);
namespace fft = frame_frame_tracker;
static_assert(std::is_same_v<decltype(&fft::FrameFrameTracker::Track),
                             void (fft::FrameFrameTracker::*)(
                                 c8::HMatrix*, const c8::FrameWithPoints&, const c8::Quaternion&,
                                 std::vector<uint32_t>*, std::vector<uint32_t>*,
                                 std::vector<uint32_t>*, bool*)>);

// WP2-PRE ②/①: horizon classes are int32; the FFT exposes its own c8::RandomNumbers.
static_assert(std::is_same_v<decltype(&fft::ClassifyHorizon),
                             std::vector<int32_t> (*)(const c8::Quaternion&,
                                                      const std::vector<c8::HPoint3>&)>);
static_assert(std::is_same_v<decltype(&fft::FrameFrameTracker::rng),
                             compat::RandomNumbers* (fft::FrameFrameTracker::*)()>);

// ---- map_builder.h (§3.6) ----
namespace mb = map_builder;
static_assert(std::is_same_v<decltype(&mb::MapBuilder::Update),
                             void (mb::MapBuilder::*)(map::WorldMaps*, const TrackerInput&,
                                                      const MapTrackerOutput&, int64_t,
                                                      MapBuilderOutput*)>);
static_assert(std::is_same_v<decltype(&mb::EpipolarMatcher),
                             void (*)(map::WorldMap&, std::vector<mb::KfPair>*, bool)>);
static_assert(std::is_same_v<decltype(&mb::PruneKeyframes),
                             void (*)(map::WorldMap&, const std::set<map::KeyframeId>&)>);

// ---- vio_tracker.h (§3.7) / metric_scale_estimator.h (§3.8) ----
namespace vt = vio_tracker;
static_assert(std::is_same_v<decltype(&vt::VioTracker::Track),
                             c8::HMatrix (vt::VioTracker::*)(const TrackingSensorFrame&,
                                                             const c8::Gr8Pyramid&,
                                                             compat::RandomNumbers*,
                                                             vt::StatusOut*, vt::TrackDebug*)>);
// FULL-3: 784 with image targets (938) and the located images.
static_assert(std::is_same_v<decltype(&vt::VioTracker::TrackWithTargets),
                             c8::HMatrix (vt::VioTracker::*)(const TrackingSensorFrame&,
                                                             const c8::Gr8Pyramid&,
                                                             const DetectionImagePtrMap&,
                                                             compat::RandomNumbers*,
                                                             vt::StatusOut*, vt::TrackDebug*)>);
static_assert(std::is_same_v<decltype(&vt::VioTracker::located_images),
                             const std::vector<c8::LocatedImage>& (vt::VioTracker::*)() const>);
static_assert(std::is_same_v<decltype(&fe::FeatureFrontend::DetectRois),
                             void (fe::FeatureFrontend::*)(const c8::Gr8Pyramid&,
                                                           const c8::DetectionConfig&,
                                                           const c8::FrameWithPoints&,
                                                           std::vector<c8::FrameWithPoints>*)>);
static_assert(std::is_same_v<decltype(&vt::VioTracker::GetWorldPoints),
                             void (vt::VioTracker::*)(std::vector<WorldPointOut>*) const>);
static_assert(std::is_same_v<decltype(&vt::PredictMotion),
                             FrameState (*)(const FrameState&, const vt::FrameRecord&)>);
static_assert(std::is_same_v<decltype(&vt::UpdateHistory),
                             void (*)(FrameState*, const c8::HMatrix&, const vt::FrameRecord&,
                                      bool)>);
static_assert(std::is_constructible_v<vt::VioTracker, bool, bool>);
// One getter per TrackDebug field (review NI-6; CONTRACTS.md "TrackDebug sources").
static_assert(std::is_same_v<decltype(&vt::VioTracker::last_prediction),
                             const FrameState& (vt::VioTracker::*)() const>);
static_assert(std::is_same_v<decltype(&vt::VioTracker::loss_state),
                             const vt::LossStateMachine& (vt::VioTracker::*)() const>);
static_assert(std::is_same_v<decltype(&vt::VioTracker::yaw_offset_initialized),
                             bool (vt::VioTracker::*)() const>);
static_assert(std::is_same_v<decltype(&vt::VioTracker::scale_latched),
                             bool (vt::VioTracker::*)() const>);
static_assert(std::is_same_v<decltype(&vt::VioTracker::latched_scale),
                             float (vt::VioTracker::*)() const>);
static_assert(std::is_same_v<decltype(&vt::LossStateMachine::state),
                             int (vt::LossStateMachine::*)() const>);
static_assert(std::is_same_v<decltype(&vt::LossStateMachine::lost_frames),
                             int (vt::LossStateMachine::*)() const>);
static_assert(std::is_same_v<decltype(&vt::LossStateMachine::frames),
                             int (vt::LossStateMachine::*)() const>);
static_assert(std::is_same_v<decltype(&vt::LossStateMachine::last_ok),
                             bool (vt::LossStateMachine::*)() const>);
static_assert(std::is_same_v<decltype(&fe::FeatureFrontend::last_stats),
                             const fe::DetectStats& (fe::FeatureFrontend::*)() const>);
static_assert(std::is_same_v<decltype(&map::WorldMap::NumKeyframes),
                             int (map::WorldMap::*)() const>);
static_assert(std::is_same_v<decltype(&map::WorldMap::NumPointsBySource),
                             std::array<int, 5> (map::WorldMap::*)() const>);
static_assert(std::is_same_v<decltype(&fft::FrameFrameTracker::last_ground_match_count),
                             int (fft::FrameFrameTracker::*)() const>);
namespace ms = metric_scale;
static_assert(std::is_same_v<decltype(&ms::MetricScaleEstimator::PushFrame),
                             bool (ms::MetricScaleEstimator::*)(const TrackingSensorFrame&,
                                                                const c8::HMatrix&, int64_t, int,
                                                                bool)>);
static_assert(std::is_constructible_v<ms::MetricScaleEstimator, float>);
static_assert(!std::is_convertible_v<float, ms::MetricScaleEstimator>);

TEST(ContractInternal, AdaptersAreExactFieldCopies) {
  const QuatWxyz q{0.5f, -0.25f, 0.125f, 0.75f};
  const c8::Quaternion c = ToC8Quaternion(q);
  EXPECT_EQ(c.w(), q.w);
  EXPECT_EQ(c.x(), q.x);
  EXPECT_EQ(c.y(), q.y);
  EXPECT_EQ(c.z(), q.z);
  const QuatWxyz back = ToQuatWxyz(c);
  EXPECT_EQ(back.w, q.w);
  EXPECT_EQ(back.z, q.z);

  const PinholeIntrinsics k{240, 320, 119.5f, 159.5f, 260.911f, 260.911f};
  const PinholeIntrinsics k2 = ToPinholeIntrinsics(ToC8Intrinsics(k));
  EXPECT_EQ(k2.width, k.width);
  EXPECT_EQ(k2.height, k.height);
  EXPECT_EQ(k2.cx, k.cx);
  EXPECT_EQ(k2.cy, k.cy);
  EXPECT_EQ(k2.fx, k.fx);
  EXPECT_EQ(k2.fy, k.fy);
}

TEST(ContractInternal, OutputDefaultsAreInert) {
  // Path A and loop-close fields stay off in world SLAM (architecture.md §1.2).
  const MapBuilderOutput mb;
  EXPECT_FALSE(mb.map_reset);
  EXPECT_FALSE(mb.keyframe_created);
  EXPECT_FALSE(mb.reloc_kf_request);
  EXPECT_FALSE(mb.world_transform_updated);
  EXPECT_TRUE(mb.new_points.empty());
  const MapTrackerOutput mt;
  EXPECT_EQ(mt.state, 0);
  EXPECT_FALSE(mt.frame_tracker_ok);
  EXPECT_FALSE(mt.depth_kf_reloc_success);
  EXPECT_TRUE(mt.depth_kf_reloc_id == map::KeyframeId::nullId());
  const RelocResult reloc;
  EXPECT_FALSE(reloc.success);
}

}  // namespace
}  // namespace slam_refact
