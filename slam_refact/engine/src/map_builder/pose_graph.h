// P3 candidate R7 (AccuracyConfig::r7_pose_graph; not part of the original): a loop that the loop
// close (loop_close.cc) accepted is closed with an SE(3) keyframe pose-graph optimization instead of
// moving only the newest keyframe. Compiled only with SLAM_REFACT_LOOP_CLOSE=ON, like loop_close.cc.
//
//   nodes   the keyframes from the loop candidate to the newest keyframe, in creation order
//           (WorldMap::AllKeyframesByCreation); poses world_from_camera
//   fixed   the loop candidate (gauge: the older map region is the reference, as in VINS-Mono's
//           4-DoF pose graph, which holds the first looped keyframe constant); keyframes older than
//           the candidate are not in the graph and keep their poses
//   edges   odometry: consecutive nodes, measured by their current relative pose;
//           loop: candidate -> newest, measured by candidate^-1 * the loop close's PnP pose of the
//           newest keyframe (that pose is solved against the candidate's map points)
//   residual  [p_ab_est - p_ab; 2 vec(q_ab * q_ab_est^-1)], unit weight, no robust loss
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

// Minimizes the sum of squared edge residuals over the poses (world_from_camera, in/out); node
// `fixed` is held constant. The poses are written back only when the solution is usable.
PoseGraphSummary OptimizePoseGraph(std::vector<Eigen::Isometry3d>* poses, size_t fixed,
                                   const std::vector<PoseGraphEdge>& edges);

// Gives each keyframe in `corrected` its new pose (655) and carries every live point along with its
// reference keyframe, the oldest keyframe (creation order) observing it: X' = P_new * P_old^-1 * X,
// so the point keeps its coordinates in the reference camera. Positions are written before the
// poses, so 655 derives the observation depths from the moved points; the common 1127
// (AdjustDirtyPoints) of the caller re-places the dirty points afterwards. Returns the number of
// points moved.
int ApplyKeyframeCorrections(map::WorldMap& wm,
                             const std::vector<std::pair<map::KeyframeId, c8::HMatrix>>& corrected);

struct LoopPoseGraphReport {
  bool applied = false;
  int nodes = 0;
  int edges = 0;
  int moved_points = 0;
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
