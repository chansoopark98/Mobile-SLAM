// P2 candidate F2: multi-view confirmation of ground seeds (build/xr-drift-p2-20261031/
// f2-contract-v3.3.md sections 5.2, 6, counters). Module-internal header of sr_map. Everything here
// is inert while AccuracyConfig::f2_ground_confirmation is off.
//
// The check is a geometric consistency test on the current keyframe poses: the seed's plane depth
// d_plane (creating keyframe pose, observation ray, plane) against a depth d_mv from a closed-form
// midpoint triangulation of the point's keyframe rays. Neither initialDepth, the stored position
// nor SetPointPosition's depths are used. It is not a metric check: a pose error shared by all
// keyframes is not visible to it (contract 5.2).

#ifndef SLAM_REFACT_MAP_F2_CONFIRM_H_
#define SLAM_REFACT_MAP_F2_CONFIRM_H_

#include <cstdint>
#include <limits>
#include <vector>

#include "Eigen/Core"
#include "map/world_map.h"
#include "map/world_map_internal.h"
#include "slam_refact/accuracy_counters.h"

namespace slam_refact::map {

// Contract constants (frozen; do not tune). Boundary comparisons are absolute, in double.
inline constexpr double kF2Pi = 3.14159265358979323846;
inline constexpr double kF2MinParallaxRad = 3.0 * kF2Pi / 180.0;
inline constexpr double kF2MaxResidualRad = 0.75 * kF2Pi / 180.0;
inline constexpr double kF2RatioTolerance = 0.05;
inline constexpr double kF2Epsilon = 1e-9;
inline constexpr double kF2MinPivot = 1e-12;
inline constexpr int kF2MinKeyframes = 3;

bool F2Enabled();  // Accuracy().enabled && Accuracy().f2_ground_confirmation

// One observation of the point: pose of the observing keyframe (world_from_camera) and the ray
// (x, y, 1) in its camera coordinates (z forward).
struct F2View {
  uint32_t kf_id = 0;  // KeyframeId.id(); equal ids are one keyframe (duplicates count once)
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Matrix3d r_wc = Eigen::Matrix3d::Identity();
  double x = 0.0;
  double y = 0.0;
  // Basis bookkeeping (not used by the geometry).
  uint32_t obs_idx = 0;
  uint64_t kf_serial = 0;
  uint32_t pose_rev = 0;
};

// Plane n . X = offset; the engine's ground plane is n = (0, 1, 0), offset = y.
struct F2Plane {
  Eigen::Vector3d n = Eigen::Vector3d(0.0, 1.0, 0.0);
  double offset = -1.0;
};

struct F2EvalInput {
  std::vector<F2View> views;
  uint32_t creator_kf_id = 0;  // the keyframe that created the seed
  bool creator_alive = true;   // false when the creating keyframe was removed or its slot reused
  F2Plane plane;
};

struct F2EvalResult {
  F2Status status = F2Status::kUnconfirmed;
  F2Reason reason = F2Reason::kTooFewKfs;
  bool no_creator = false;  // detail of kTooFewKfs
  int distinct_kfs = 0;
  // NaN when the step that computes them was not reached.
  double d_mv = std::numeric_limits<double>::quiet_NaN();
  double d_plane = std::numeric_limits<double>::quiet_NaN();
  double ratio = std::numeric_limits<double>::quiet_NaN();  // r = d_mv / d_plane
  double theta_max = std::numeric_limits<double>::quiet_NaN();
  double rho_max = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d x = Eigen::Vector3d::Zero();  // midpoint triangulation (valid from step 3)
};

// Closed-form linear least squares of sum |(I - d d^T)(X - c)|^2 (midpoint method, no initial
// value, no refinement). dirs must be unit vectors. False when A = sum(I - d d^T) is singular
// (all rays parallel or identical), its LDLT fails, a pivot is <= kF2MinPivot or X is not finite.
bool F2Triangulate(const std::vector<Eigen::Vector3d>& centers,
                   const std::vector<Eigen::Vector3d>& dirs, Eigen::Vector3d* x);

// Contract 5.2: the eight steps in this order, first failing step wins. Only step 8 yields
// kContradicted.
F2EvalResult F2Evaluate(const F2EvalInput& in);

// ---- WorldMap integration ----------------------------------------------------------------------

// Builds the evaluation input of a ground point from the current map (observations of valid
// keyframes only; the basis lists what the state was computed from).
struct F2Built {
  F2EvalInput input;
  std::vector<F2Basis> basis;
  uint64_t basis_hash = 0;
};
F2Built F2BuildInput(const WorldMapAccess::Impl& m, const WorldPoint& p);

// Evaluates a point now from the current map state; no cache, no counters. False if the point is
// unknown.
bool F2EvaluatePointNow(const WorldMap& wm, PointId id, F2EvalResult* out);

// Effective state used by the subset selection: the stored state unless the point's serial no
// longer matches the one it was evaluated for (then unconfirmed, counted as id_reuse).
// It also checks the keyframe basis against the map now: a change not yet finalized (a keyframe
// removed and its id drawn again, observations or poses changed) is never a confirmed bonus.
F2Status F2StateOf(const WorldMap& wm, const WorldPoint& p);

struct F2FinalizeArgs {
  int64_t frame = 0;     // MapBuilder frame_count_ of the Update that ends here
  bool hold = false;     // trk.state != 1 or RelocalizationHold: do not evaluate now
  bool kf_path = false;  // which Update exit calls (counters only)
};
// The single hook of MapBuilder::Update (both exits). Drains f2_dirty, re-evaluates the ground
// points whose basis or epoch changed and publishes the states for tracking frame frame + 1.
void F2Finalize(WorldMap& wm, const F2FinalizeArgs& args);

// Counted events (called from world_map.cc / the MapBuilder). No-ops while F2 is off.
void F2NoteChange(WorldMapAccess::Impl& m, PointId id, const WorldPoint& p, F2Invalidation why);
void F2NoteMapReset(const WorldMap& old_map);  // counts the states a map reset destroys

// Epoch events (contract 6.2): the tracker state changed, or a new MapBuilder starts on a map that
// already has ground points (engine rebuild). No-ops while F2 is off.
void F2BumpTrackerEpoch(WorldMap& wm);
void F2NoteBuilderStart(WorldMap& wm);

// Tracking frame that sees the states published by the last F2Finalize (applied_frame).
int64_t F2AppliedFrame();
// used_state_before_epoch bookkeeping of one MapTracker::Track call.
void F2TrackBegin();
void F2NoteBonusGranted();
void F2TrackEnd(bool state_changed);

AccuracyCounters& MutableCounters();

}  // namespace slam_refact::map

#endif  // SLAM_REFACT_MAP_F2_CONFIRM_H_
