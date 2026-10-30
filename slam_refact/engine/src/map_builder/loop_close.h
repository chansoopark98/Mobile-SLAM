// Loop close of MapBuilder::Update (func 936 [I], G936:4159-5754) and the keyframe global
// descriptor it needs (US-025 LC-3; docs/spec/12-loop-close.md is the only basis, steps S0..S12 of
// spec §3.1). The implementation (loop_close.cc) is compiled only with SLAM_REFACT_LOOP_CLOSE=ON;
// this header is declaration-only and safe to include in every build. Design and the
// UNKNOWN/UNVERIFIED list: docs/design/us025-lc3-impl.md.

#ifndef SLAM_REFACT_MAP_BUILDER_LOOP_CLOSE_H_
#define SLAM_REFACT_MAP_BUILDER_LOOP_CLOSE_H_

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "common/tracking_types.h"
#include "map/world_map.h"
#include "map_builder/agate_descriptor.h"

namespace slam_refact::map_builder {

// One element of the original's distance list (12 B: float distance + a KeyframeId handle).
struct LoopCloseDistEntry {
  float distance = 0.0f;
  uint32_t index = 0;
  uint32_t id = 0;  // 0 = free slot / no keyframe
};

// MapBuilder +0xa28..+0xa3c and the working storage of the attempt (spec §2.1, §10 U2).
struct LoopCloseState {
  map::KeyframeId last_newest;       // +0xa28 (index), +0xa2c (id)
  std::array<uint32_t, 3> ring{};    // +0xa30..+0xa38: ids of the recently chosen candidates
  uint32_t cursor = 0;               // +0xa3c

  // U2 model (UNVERIFIED, docs/design/us025-lc3-impl.md §4). The original builds the distance
  // list in a malloc(n_slots * 12) block it never initializes and reads it at slot indices, i.e.
  // past the live entries when free slots exist. lc-g's G-C observation: the tail holds garbage
  // (87%), id 0 (11%) and only 1% handles of the previous attempt, and the id 0 model reproduces
  // 400/433 picks (92%). So the default is false: the tail is id 0 (skipped). The residual 8% is
  // unreproducible heap content (UNKNOWN). true keeps the previous attempt's storage when the slot
  // count is unchanged (refuted by that observation; kept for comparison only).
  bool model_stale_dist_tail = false;
  std::vector<LoopCloseDistEntry> dist_storage;

  // The agate extractor holds the 8192-draw random basis; created on first use (the original
  // builds its basis on the first call of func 1263).
  std::unique_ptr<AgateDescriptor> agate;
};

enum class LoopCloseOutcome {
  kTooFewSlots,            // S1: fewer than 5 keyframe slots (free slots included)
  kNoCandidate,            // S6: no entry below loopClosingDistanceThreshold
  kInvalidHandle,          // S7: newest or candidate handle no longer valid (silent not-found)
  kTooFewMatches,          // S9
  kTooFewCorrespondences,  // S11 (before PnP)
  kPoseEstimationFailed,   // 1146 returned false
  kApplied,                // S12
};

// Optional per-attempt facts for tests and diagnostics; the algorithm does not read it.
struct LoopCloseReport {
  LoopCloseOutcome outcome = LoopCloseOutcome::kTooFewSlots;
  map::KeyframeId newest;
  map::KeyframeId candidate;       // valid from kInvalidHandle on
  float candidate_distance = -1.0f;
  size_t num_slots = 0;
  size_t num_live = 0;
  size_t num_matches = 0;
  size_t num_correspondences = 0;
  size_t num_unlinked = 0;         // S12 final loop
};

// 654 -> 1263: the agate descriptor of the keyframe's ORB descriptors (descriptor vector order).
// Called by MapBuilder::Update right after WorldMap::AddKeyframe, only when loop close is on.
void ComputeKeyframeGlobalDescriptor(LoopCloseState* state, map::Keyframe* kf);

// S1..S12 (the caller has checked MapBuilder.loopCloseEnabled, S0, and runs the common
// AdjustDirtyPoints afterwards). Sets out->world_transform_updated and, on success,
// out->world_correction. Returns true when a loop close was applied.
bool LoopClose(map::WorldMap& wm, LoopCloseState* state, MapBuilderOutput* out,
               LoopCloseReport* report = nullptr);

}  // namespace slam_refact::map_builder

#endif  // SLAM_REFACT_MAP_BUILDER_LOOP_CLOSE_H_
