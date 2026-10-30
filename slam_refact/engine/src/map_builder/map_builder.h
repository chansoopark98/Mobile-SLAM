// MapBuilder module public header (architecture.md §3.6; S3).
//
// FROZEN CONTRACT (WP0b): every declaration in this file. Changes follow
// slam_refact/docs/design/contract-changes.md. `private:` sections are owned by WP3.
//
// Not frozen here (signature not fixed by the design; WP3 declares it in a private header):
// ShouldResetMap (S3 §3 [A]).

#ifndef SLAM_REFACT_MAP_BUILDER_MAP_BUILDER_H_
#define SLAM_REFACT_MAP_BUILDER_MAP_BUILDER_H_

#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <vector>

#include "common/tracking_types.h"
#include "map/world_map.h"

namespace slam_refact::map_builder {

struct LoopCloseState;  // loop_close.h (US-025); created on the first Update with loop close on

// Keyframe pair queued for epipolar matching (16 B; S3 §2.1 +0x9ec).
struct KfPair {
  map::KeyframeId a;
  map::KeyframeId b_newest;
};

// 936; constructor 641.
class MapBuilder {
 public:
  MapBuilder();
  ~MapBuilder();

  MapBuilder(const MapBuilder&) = delete;
  MapBuilder& operator=(const MapBuilder&) = delete;

  // S3 §3 [A]-[I]. world_matches = frm.frame_points.worldPointIndices().size() (D-7; 0 on the
  // original per-frame path per WP2-PRE §6, INFERRED).
  // MapBuilder.loopCloseEnabled = 1 runs the loop close of 936 [I] (US-025; spec/12-loop-close.md)
  // in builds with SLAM_REFACT_LOOP_CLOSE and aborts as unsupported in builds without it.
  void Update(map::WorldMaps* ctx, const TrackerInput& frm, const MapTrackerOutput& trk,
              int64_t frame_ms, MapBuilderOutput* out);

 private:
  // Owned by WP3. Members follow the original object (S3 §2.1, constructor 641) by meaning.
  void ResetMap(map::WorldMaps* ctx, MapBuilderOutput* out);  // 653 with clear flag 1

  std::set<map::KeyframeId> vps_corrected_kfs_;    // +0x000 (VPS; always empty in world SLAM)
  bool vps_baseline_mode_ = false;                 // +0x00c
  std::vector<KfPair> pair_queue_;                 // +0x9ec
  int64_t last_reset_frame_ = -1;                  // +0xa08 (641: -1)
  int64_t frame_count_ = -1;                       // +0xa10 (641: -1, 0 on the first Update)
  int64_t last_state_change_frame_ = 0;            // +0xa18
  int32_t state_ = 0;                              // +0xa20
  int32_t prev_state_ = 0;                         // +0xa24
  bool reset_allowed_ = true;                      // +0xc8c (641: 1)
  bool seeded_from_depth_kf_ = false;              // +0xc8d (depth reseed; never set here)
  bool ground_points_enabled_ = true;              // +0xc8e (641: 1, never written again)
  bool force_keyframe_ = false;                    // +0xc8f (set only by VPS 509)
  // +0xc90 pending VPS records keyed by frame ms; only their keyframe ids are read (prune
  // protection, reset gate). Always empty in world SLAM.
  std::map<int64_t, map::KeyframeId> pending_vps_;
  c8::HMatrix ground_prune_pose_ = c8::HMatrixGen::i();  // +0xcd8
  // +0xa28..+0xa3c (last newest keyframe, candidate ring, cursor) and the loop close working
  // state. Null until the first Update with MapBuilder.loopCloseEnabled on; never touched when
  // off and not reset by ResetMap (653 does not touch it). US025-CC-1.
  std::unique_ptr<LoopCloseState> loop_close_;
};

// 1112.
void EpipolarMatcher(map::WorldMap& wm, std::vector<KfPair>* queue, bool vps_mode);

// S3 §5.3.
bool NeedKeyframe(uint32_t tracked, int n_new);

// S3 §3.4.
void PruneKeyframes(map::WorldMap& wm, const std::set<map::KeyframeId>& protected_kfs);

// S3 §3.5. Off by default (MapBuilder.bundleAdjustmentIterativeEnabled = 0).
void IterativeBundleAdjustment(map::WorldMap& wm);

// Accuracy mode (ACC-1, config::Accuracy()): joint bundle adjustment of the newest
// AccuracyConfig::ba_window keyframes and the triangulated points they observe; other keyframes
// that observe those points are held fixed. `newest` is the keyframe created this frame.
struct LocalBaResult {
  bool applied = false;
  c8::HMatrix newest_correction = c8::HMatrixGen::i();  // new * old^-1 of `newest`
  int keyframes = 0;
  int fixed_keyframes = 0;
  int points = 0;
  int residuals = 0;
  int unlinked = 0;
  double initial_cost = 0.0;
  double final_cost = 0.0;
};
LocalBaResult LocalBundleAdjustment(map::WorldMap& wm, map::KeyframeId newest);

}  // namespace slam_refact::map_builder

#endif  // SLAM_REFACT_MAP_BUILDER_MAP_BUILDER_H_
