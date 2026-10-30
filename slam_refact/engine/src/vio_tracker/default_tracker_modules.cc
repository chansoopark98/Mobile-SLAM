// Product TrackerModules: forwards to the frozen module interfaces (frontend, map, map_tracker,
// map_builder). Also holds the public VioTracker constructor, so unit tests that inject doubles
// do not link the other work packages.

#include <memory>
#include <vector>

#include "common/c8_adapters.h"
#include "map/world_map.h"
#include "map_builder/map_builder.h"
#include "map_tracker/map_tracker.h"
#include "vio_tracker/tracker_modules.h"
#include "vio_tracker/vio_tracker.h"

namespace slam_refact::vio_tracker {
namespace {

class DefaultTrackerModules final : public TrackerModules {
 public:
  DefaultTrackerModules()
      : map_tracker_(std::make_unique<map_tracker::MapTracker>()),
        map_builder_(std::make_unique<map_builder::MapBuilder>()) {}

  void Detect(const c8::Gr8Pyramid& pyr, const c8::DetectionConfig& cfg,
              c8::FrameWithPoints* frame) override {
    frontend_.Detect(pyr, cfg, frame);
  }
  frontend::DetectStats detect_stats() const override { return frontend_.last_stats(); }
  void DetectRois(const c8::Gr8Pyramid& pyr, const c8::DetectionConfig& cfg,
                  const c8::FrameWithPoints& frame,
                  std::vector<c8::FrameWithPoints>* rois) override {
    frontend_.DetectRois(pyr, cfg, frame, rois);
  }

  void TrackMap(TrackerInput* input, const FrameState& predicted, MapTrackerOutput* out,
                compat::RandomNumbers* rng) override {
    // MapTracker copies the frame intrinsic into +0xff8 every Track (F926:311-313); 631 projects
    // with it.
    map_tracker_intrinsic_ = input->frame_points.intrinsic();
    map_tracker_->Track(world_maps_.active(), input, predicted, out, rng);
  }

  void UpdateMap(const TrackerInput& input, const MapTrackerOutput& trk, int64_t frame_ms,
                 MapBuilderOutput* out) override {
    map_builder_->Update(&world_maps_, input, trk, frame_ms, out);
  }

  c8::HMatrix ApplyMapBuilderOutput(const MapBuilderOutput& o) override {
    return map_tracker_->ApplyMapBuilderOutput(o);
  }

  void RecreateMapTrackerAndBuilder() override {
    map_tracker_ = std::make_unique<map_tracker::MapTracker>();
    map_builder_ = std::make_unique<map_builder::MapBuilder>();
    // The new MapTracker starts from zeroed memory (476 memsets 6296 B before 640; INFERRED that
    // 640 leaves +0xff8 untouched).
    map_tracker_intrinsic_ = c8_PixelPinholeCameraModel{0, 0, 0.f, 0.f, 0.f, 0.f};
  }

  void VisibleMapPoints(std::vector<VisibleMapPoint>* out) const override {
    out->clear();
    const map::WorldMap& wm = world_maps_.active();
    const map::ProjectedPoints proj = map::ProjectPoints(
        wm, {}, ToPinholeIntrinsics(map_tracker_intrinsic_), map_tracker_->pose());  // 632 -> 1548
    for (const c8::FramePoint& fp : proj.frame.points()) {
      const map::WorldPoint& wp = wm.Point(fp.id());  // 634
      VisibleMapPoint r;
      r.key_id = fp.id().id();
      r.point_id = wp.id;
      r.state = wp.state;
      r.source = wp.source;
      r.position = wp.position;
      r.tracking_hits = wp.tracking_hits;
      r.tracking_count = wp.tracking_count;
      out->push_back(r);
    }
  }

  const map::WorldMap* active_world_map() const override { return &world_maps_.active(); }

  int ground_match_count() const override {
    return map_tracker_->frame_tracker().last_ground_match_count();
  }
  int num_keyframes() const override { return world_maps_.active().NumKeyframes(); }
  std::array<int, 5> num_points_by_source() const override {
    return world_maps_.active().NumPointsBySource();
  }

 private:
  frontend::FeatureFrontend frontend_;
  map::WorldMaps world_maps_;  // +7224 / +7232
  std::unique_ptr<map_tracker::MapTracker> map_tracker_;
  std::unique_ptr<map_builder::MapBuilder> map_builder_;
  c8_PixelPinholeCameraModel map_tracker_intrinsic_{0, 0, 0.f, 0.f, 0.f, 0.f};
};

}  // namespace

std::unique_ptr<TrackerModules> MakeDefaultTrackerModules() {
  return std::make_unique<DefaultTrackerModules>();
}

VioTracker::VioTracker(bool estimate_scale, bool disable_vio)
    : VioTracker(estimate_scale, disable_vio, MakeDefaultTrackerModules()) {}

}  // namespace slam_refact::vio_tracker
