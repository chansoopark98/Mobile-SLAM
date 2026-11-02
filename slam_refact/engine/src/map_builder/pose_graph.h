// P3 candidate R7 (AccuracyConfig::r7_pose_graph; not part of the original): a loop that the loop
// close (loop_close.cc) accepted is closed with an SE(3) keyframe pose-graph optimization instead of
// moving only the newest keyframe. Compiled only with SLAM_REFACT_LOOP_CLOSE=ON, like loop_close.cc.
//
//   nodes   the keyframes from the loop candidate to the newest keyframe, in creation order
//           (WorldMap::AllKeyframesByCreation); poses world_from_camera
//   fixed   the loop candidate (gauge: the older map region is the reference, as in VINS-Mono's
//           4-DoF pose graph, which holds the first looped keyframe constant); keyframes older than
//           the candidate are not in the graph and keep their poses.
//           The newest keyframe is held at the loop close's PnP pose (solved against the
//           candidate's map points), the pose the original loop close adopts for it: the loop
//           constraint is taken as exact, the limit of an infinitely weighted loop edge. A unit
//           weighted loop edge would leave the newest keyframe part of the way (1 / number of
//           edges of the misclosure), and the S12 merge then ties its off-pose observations to the
//           candidate's points (p3_r7 test NextTrackerFrame...: 0.072 rotation error of the next
//           tracked frame with a unit loop edge, 0.0033 with the original single-keyframe update).
//   edges   odometry: consecutive nodes, measured by their relative pose before the loop close
//   residual  [p_ab_est - p_ab; 2 vec(q_ab * q_ab_est^-1)], unit weight, no robust loss
// So R7 = the original loop close of the newest keyframe + the misclosure spread over the
// keyframes between the candidate and the newest one (and the triangulated points they anchor).
// With both ends anchored the graph has no gauge freedom (no local null space), but the objective
// is non-convex in rotation: for large misclosures (rotation errors approaching 180 degrees) a
// unique global minimum is not guaranteed and LM returns the local minimum reached from the
// pre-loop poses. With only the candidate and the newest keyframe (adjacent) there is no free node
// and nothing is solved; the newest keyframe gets the loop pose as in the original. Non-finite
// input poses or measurements make the graph unusable on every path (the caller then keeps the
// original update).
//
// SE(3), not Sim(3): the loop measurement is a 6-DoF PnP pose (scale 1) against the candidate's
// points, so a scale ratio between the two loop ends is not measured; and the map scale is tied at
// every keyframe to the ground points seeded on the plane y = -1 (936 [D]), so scale is not a free
// drift direction of this map.

#ifndef SLAM_REFACT_MAP_BUILDER_POSE_GRAPH_H_
#define SLAM_REFACT_MAP_BUILDER_POSE_GRAPH_H_

#include <cstddef>
#include <utility>
#include <vector>

#include "Eigen/Geometry"
#include "c8/hmatrix.h"
#include "map/world_map.h"

namespace slam_refact::map_builder {

// Relative pose measurement between nodes a and b: a_from_b = T_a^-1 * T_b (world_from_camera).
struct PoseGraphEdge {
  size_t a = 0;
  size_t b = 0;
  Eigen::Isometry3d a_from_b = Eigen::Isometry3d::Identity();
};

struct PoseGraphSummary {
  bool usable = false;  // ceres Summary::IsSolutionUsable
  int iterations = 0;
  double initial_cost = 0.0;
  double final_cost = 0.0;
};

// Minimizes the sum of squared edge residuals over the poses (world_from_camera, in/out); the
// nodes in `fixed` (at least one) are held constant. The poses are written back only when the
// solution is usable.
PoseGraphSummary OptimizePoseGraph(std::vector<Eigen::Isometry3d>* poses, const std::vector<size_t>& fixed,
                                   const std::vector<PoseGraphEdge>& edges);

struct KeyframeCorrectionResult {
  int checked_points = 0;         // live TRIANGULATED points observed by a corrected keyframe
  int moved_points = 0;           // of those, points whose reference keyframe was corrected
  int unlinked_observations = 0;  // removed by 1150 on the corrected poses
};

// Gives each keyframe in `corrected` its new pose (655) and re-places every live TRIANGULATED
// point that a corrected keyframe observes by its reference keyframe, the oldest keyframe (creation order)
// observing it: X' = P_new * P_old^-1 * X when the reference keyframe is corrected, X' = X when it
// is not (e.g. the fixed loop candidate). The point keeps its coordinates in the reference camera.
// X' is then set with 1150 (WorldMap::SetPointPosition) on the corrected poses: every observation
// depth is re-derived from X' and an observation whose ray residual exceeds
// WorldMap.maxInternalSquaredResidualForTriangulatedPts is unlinked, the engine's own rule for a
// placed point. X' is final for this update: the point leaves the dirty set, so the caller's 1127
// (AdjustDirtyPoints) does not re-average the observation rays into a position that neither the
// reference nor the corrected keyframes see (local BA treats its solved positions the same way). A
// point left without observations stays dirty so that 1127 retires it.
// Ground and sky points are not touched here: the original engine derives their position from the
// observing keyframes' poses (initialDepth on the plane y = -1 / the sky distance, 1127), so 655
// leaves them dirty and the caller's 1127 re-places them from the corrected poses.
KeyframeCorrectionResult ApplyKeyframeCorrections(
    map::WorldMap& wm, const std::vector<std::pair<map::KeyframeId, c8::HMatrix>>& corrected);

struct LoopPoseGraphReport {
  bool applied = false;
  int nodes = 0;
  int edges = 0;
  KeyframeCorrectionResult points;
  PoseGraphSummary solve;
};

// R7 for an accepted loop: builds the graph described above, optimizes it and applies the result
// with ApplyKeyframeCorrections. `newest_loop_pose` is the loop close's pose of `newest`. Returns
// false and leaves the map untouched when the graph has no cycle (candidate == newest, or the
// candidate is not older than the newest keyframe) or the solve is not usable.
bool CloseLoopWithPoseGraph(map::WorldMap& wm, map::KeyframeId cand, map::KeyframeId newest,
                            const c8::HMatrix& newest_loop_pose, LoopPoseGraphReport* report);

}  // namespace slam_refact::map_builder

#endif  // SLAM_REFACT_MAP_BUILDER_POSE_GRAPH_H_
