// Counters of the P2 drift candidates F1 and F2 (build/xr-drift-p2-20261031/f2-contract-v3.3.md,
// section "counters"). Process-global like config::Accuracy(); single-threaded. They only count:
// nothing in the engine reads them back. A replay driver reads Counters() after each frame and
// writes the values next to the trace. All counters stay 0 while both options are off.

#ifndef SLAM_REFACT_ACCURACY_COUNTERS_H_
#define SLAM_REFACT_ACCURACY_COUNTERS_H_

#include <array>
#include <cstdint>

namespace slam_refact {

// First-match reason of an F2 evaluation (contract 5.2 order). Index 0 = confirmed.
inline constexpr int kF2ReasonCount = 9;
inline constexpr const char* kF2ReasonNames[kF2ReasonCount] = {
    "confirmed",     "invalid_input",  "too_few_kfs",           "degenerate",
    "negative_depth", "low_parallax",  "large_residual",        "plane_invalid",
    "depth_ratio_out_of_band"};

// Invalidation events of F2 (contract 6.2), counted once per (ground point, event).
inline constexpr int kF2InvalidationCount = 9;
inline constexpr const char* kF2InvalidationNames[kF2InvalidationCount] = {
    "obs_added", "obs_removed", "obs_replaced", "merge",         "kf_pose_rev",
    "creator_deleted", "map_reset", "tracker_epoch", "rebuild"};
enum F2Invalidation : int {
  kInvObsAdded = 0,
  kInvObsRemoved,
  kInvObsReplaced,
  kInvMerge,
  kInvKfPoseRev,
  kInvCreatorDeleted,
  kInvMapReset,
  kInvTrackerEpoch,
  kInvRebuild,
};

// F1 reasons. 0 = the measured plane is used. Counted per seed point.
inline constexpr int kF1ReasonCount = 8;
inline constexpr const char* kF1ReasonNames[kF1ReasonCount] = {
    "ok", "initial_keyframe", "epoch_invalid", "insufficient_samples",
    "stale", "camera_too_close", "lambda_nonpositive", "invalid_input"};
enum F1Reason : uint8_t {
  kF1Ok = 0,
  kF1InitialKeyframe,
  kF1EpochInvalid,
  kF1InsufficientSamples,
  kF1Stale,
  kF1CameraTooClose,
  kF1LambdaNonpositive,
  kF1InvalidInput,
};

struct AccuracyCounters {
  // ---- F2 evaluation (one count per evaluation that ran; first matching reason only)
  std::array<uint64_t, kF2ReasonCount> f2_eval{};
  uint64_t f2_no_creator = 0;       // detail of too_few_kfs
  uint64_t f2_newly_confirmed = 0;  // transitions into confirmed
  uint64_t f2_hold_frames = 0;      // finalize calls that deferred the evaluation
  uint64_t f2_hold_points = 0;      // pending ground points at those calls
  uint64_t f2_id_reuse = 0;
  uint64_t f2_path_kf = 0;          // finalize calls from the keyframe path of Update
  uint64_t f2_path_nonkf = 0;       // ... from the non-keyframe path
  uint64_t f2_dirty_remainder = 0;  // f2_dirty entries left after a non-held finalize (must be 0)
  std::array<uint64_t, kF2InvalidationCount> f2_invalidation{};
  // ---- F2 selection (SelectSubset calls; only the calls with m >= maxSubsetPoints score)
  uint64_t sel_calls = 0;
  uint64_t sel_active = 0;
  uint64_t sel_cap_reached = 0;     // active calls where the 52 bonus slots were all used
  uint64_t sel_bonus_granted = 0;
  // ground points of all_matches / of the subset by state: [confirmed, unconfirmed, contradicted]
  std::array<uint64_t, 3> sel_all_ground{};
  std::array<uint64_t, 3> sel_subset_ground{};
  uint64_t sel_new_confirmed_all = 0;     // confirmed with applied_frame == this tracking frame
  uint64_t sel_new_confirmed_subset = 0;
  uint64_t used_state_before_epoch = 0;   // Track frames that changed the state after using F2 bonus
  // ---- F1 (per seed point)
  uint64_t f1_applied = 0;
  std::array<uint64_t, kF1ReasonCount> f1_fallback{};
  uint64_t f1_kf_applied = 0;       // keyframes whose plane was valid
  uint64_t f1_kf_fallback = 0;
};

const AccuracyCounters& Counters();
void ResetCounters();

}  // namespace slam_refact

#endif  // SLAM_REFACT_ACCURACY_COUNTERS_H_
