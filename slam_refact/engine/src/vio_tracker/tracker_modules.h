// Injection seam of VioTracker (WP4-private, not frozen). 784 calls the frontend (498), the
// MapTracker (926 / 664), the MapBuilder (936) and the active world map through this interface.
// The product implementation (MakeDefaultTrackerModules, default_tracker_modules.cc) forwards to
// the frozen module interfaces; unit tests inject doubles so WP4 does not depend on WP1-WP3.

#ifndef SLAM_REFACT_VIO_TRACKER_TRACKER_MODULES_H_
#define SLAM_REFACT_VIO_TRACKER_TRACKER_MODULES_H_

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/pixels/gr8-pyramid.h"
#include "common/check.h"
#include "common/tracking_types.h"
#include "compat/random_numbers.h"
#include "frontend/frontend.h"
#include "reality/engine/features/detection-config.h"
#include "reality/engine/features/frame-point.h"

namespace slam_refact::map {
class WorldMap;
}  // namespace slam_refact::map

namespace slam_refact::vio_tracker {

// One record of 632 (projectPoints with the MapTracker intrinsic and pose, 80 B in the original)
// joined with the world point it refers to (634), as 631 reads them.
struct VisibleMapPoint {
  uint32_t key_id = 0;          // record +0x10 = XrId id of the projected frame point
  uint32_t point_id = 0;        // WorldPoint +0x04
  map::PointState state = map::PointState::kNew;          // +0x18
  map::PointSource source = map::PointSource::kTriangulated;  // +0x40
  c8::HPoint3 position;         // +0x08 (x, y, z, w)
  uint32_t tracking_hits = 0;   // +0x34
  uint32_t tracking_count = 0;  // +0x38
};

class TrackerModules {
 public:
  virtual ~TrackerModules() = default;

  // 498 FeatureDetector::detectFeatures on the pyramid.
  virtual void Detect(const c8::Gr8Pyramid& pyr, const c8::DetectionConfig& cfg,
                      c8::FrameWithPoints* frame) = 0;
  virtual frontend::DetectStats detect_stats() const = 0;
  // FULL-3: ROI detection of 784 ("gr8gl-rois", D:82535-82807) for a pyramid with ROIs (image
  // targets). Modules without ROI support reject such a pyramid (the guard 784 had before FULL-3).
  virtual void DetectRois(const c8::Gr8Pyramid& pyr, const c8::DetectionConfig& cfg,
                          const c8::FrameWithPoints& frame,
                          std::vector<c8::FrameWithPoints>* rois) {
    (void)cfg;
    (void)frame;
    (void)rois;
    SR_CHECK(pyr.rois.empty(), "VioTracker: pyramid ROIs need a module with ROI detection");
  }

  // 926 on the active world map.
  virtual void TrackMap(TrackerInput* input, const FrameState& predicted, MapTrackerOutput* out,
                        compat::RandomNumbers* rng) = 0;
  // 936 with the WorldMaps context.
  virtual void UpdateMap(const TrackerInput& input, const MapTrackerOutput& trk, int64_t frame_ms,
                         MapBuilderOutput* out) = 0;
  // 664. Returns the MapTracker pose (tam_extrinsic).
  virtual c8::HMatrix ApplyMapBuilderOutput(const MapBuilderOutput& o) = 0;
  // 476 with disableVio on: new MapTracker (and its FFT RNG) and new MapBuilder; the world map
  // (+7224/+7232) is kept.
  virtual void RecreateMapTrackerAndBuilder() = 0;

  // 632 + 634 for 631, in projection order.
  virtual void VisibleMapPoints(std::vector<VisibleMapPoint>* out) const = 0;

  // US-015: the active WorldMap (b+7232) that 784 passes to 938 for the closed image world tracker
  // (994 reads its points through WorldMap::getPoint, 634). nullptr = no map (test doubles).
  virtual const map::WorldMap* active_world_map() const { return nullptr; }

  // TrackDebug sources (CONTRACTS.md §2.3).
  virtual int ground_match_count() const = 0;
  virtual int num_keyframes() const = 0;
  virtual std::array<int, 5> num_points_by_source() const = 0;
};

// Product modules: FeatureFrontend, WorldMaps, MapTracker, MapBuilder.
std::unique_ptr<TrackerModules> MakeDefaultTrackerModules();

}  // namespace slam_refact::vio_tracker

#endif  // SLAM_REFACT_VIO_TRACKER_TRACKER_MODULES_H_
