// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_FRONTEND_FEATURE_MANAGER_H_
#define SLAM_FRONTEND_FEATURE_MANAGER_H_

#include <Eigen/Dense>
#include <algorithm>
#include <list>
#include <numeric>
#include <utility>
#include <vector>

#include "slam/common/types.h"

namespace slam {
namespace frontend {

// ---------------------------------------------------------------------------
// Per-frame observation of a single feature
// ---------------------------------------------------------------------------

/// @brief Stores the observation of one feature in one frame of the sliding
///        window: normalised ray, pixel coordinates, optical-flow velocity,
///        and (optionally) the estimated depth.
struct FeaturePerFrame {
  /// @brief Construct from the standard 7-DOF observation vector.
  explicit FeaturePerFrame(const Eigen::Matrix<double, 7, 1>& point)
      : point_(point.head<3>()),
        uv_(point(3), point(4)),
        velocity_(point(5), point(6)),
        depth_(-1.0) {}

  /// Normalised bearing ray (x, y, z) on the unit-depth plane.
  Eigen::Vector3d point_;

  /// Raw pixel coordinates (u, v).
  Eigen::Vector2d uv_;

  /// Optical-flow velocity in normalised coordinates (vx, vy).
  Eigen::Vector2d velocity_;

  /// Estimated depth from triangulation (< 0 means unknown).
  double depth_;
};

// ---------------------------------------------------------------------------
// Lifetime record for one feature across multiple frames
// ---------------------------------------------------------------------------

/// @brief Tracks the complete observation history of a single feature inside
///        the sliding window, together with its estimated depth and solver
///        status.
struct FeaturePerId {
  FeaturePerId(int feature_id, int start_frame)
      : feature_id_(feature_id),
        start_frame_(start_frame),
        used_num_(0),
        estimated_depth_(-1.0),
        solve_flag_(0) {}

  /// @brief Index of the last frame (inclusive) in which this feature appears.
  int endFrame() const {
    return start_frame_ +
           static_cast<int>(feature_per_frame_.size()) - 1;
  }

  /// Globally unique feature identifier.
  const int feature_id_;

  /// Index of the first sliding-window frame that observes this feature.
  int start_frame_;

  /// Per-frame observations (oldest first).
  std::vector<FeaturePerFrame> feature_per_frame_;

  /// Number of frames in which this feature is actually used by the solver.
  int used_num_;

  /// Estimated inverse-depth or depth (< 0 means unsolved).
  double estimated_depth_;

  /// 0 = not yet solved, 1 = solved successfully, 2 = solve failed.
  int solve_flag_;
};

// ---------------------------------------------------------------------------
// Type aliases for feature correspondences
// ---------------------------------------------------------------------------

/// A pair of normalised bearing rays observed in two different frames.
using Correspondence = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
using Correspondences = std::vector<Correspondence>;

// ---------------------------------------------------------------------------
// FeatureManager
// ---------------------------------------------------------------------------

/// @brief Manages the lifecycle of visual features across the sliding window.
///
/// Responsibilities:
///   - Accept per-frame feature observations and decide whether the current
///     frame is a keyframe (based on tracked-feature parallax).
///   - Provide feature correspondences between arbitrary window frames.
///   - Triangulate feature depths using multi-view SVD.
///   - Maintain and update inverse-depth estimates during optimisation.
///   - Slide the window (removeBack / removeFront).
class FeatureManager {
 public:
  /// @param min_parallax  Minimum parallax (in pixels) to declare a new
  ///                      keyframe.  Internally converted to normalised
  ///                      coordinates using the focal length passed to
  ///                      addFeatureCheckParallax.
  explicit FeatureManager(double min_parallax = 10.0);

  // -----------------------------------------------------------------------
  // Feature insertion and keyframe decision
  // -----------------------------------------------------------------------

  /// @brief Register the features observed in the current frame and decide
  ///        whether it qualifies as a new keyframe.
  ///
  /// @param frame_count  Current frame index inside the sliding window.
  /// @param image_data   Feature observations (feature_id -> 7-DOF).
  /// @param focal_length Focal length in pixels (for parallax normalisation).
  /// @return true if the frame should be treated as a **keyframe** (enough
  ///         parallax or too few tracked features).
  bool addFeatureCheckParallax(
      int frame_count,
      const common::ImageData& image_data,
      double focal_length);

  // -----------------------------------------------------------------------
  // Correspondences
  // -----------------------------------------------------------------------

  /// @brief Return pairs of normalised bearing rays that are co-observed in
  ///        frames @p frame_0 and @p frame_1.
  Correspondences getCorresponding(int frame_0, int frame_1) const;

  // -----------------------------------------------------------------------
  // Depth management
  // -----------------------------------------------------------------------

  /// @brief Write inverse-depth estimates back into the feature bank after
  ///        optimisation.
  ///
  /// The order must match getDepthVector().
  void setDepth(const Eigen::VectorXd& depths);

  /// @brief Collect the current inverse-depth estimates into a dense vector
  ///        suitable for the optimiser.
  Eigen::VectorXd getDepthVector() const;

  /// @brief Remove features whose solve_flag is 2 (failed triangulation /
  ///        optimisation).
  void removeFailures();

  /// @brief Reset all depths to -1 (unsolved).
  void clearDepth();

  // -----------------------------------------------------------------------
  // Triangulation
  // -----------------------------------------------------------------------

  /// @brief Triangulate features with unknown depth using multi-view linear
  ///        SVD across the sliding window.
  ///
  /// @param Rs   Rotation matrices of each window frame (body-to-world).
  /// @param Ps   Position vectors of each window frame (world frame).
  /// @param ric  IMU-to-camera rotation.
  /// @param tic  IMU-to-camera translation.
  void triangulate(const std::vector<Eigen::Matrix3d>& Rs,
                   const std::vector<Eigen::Vector3d>& Ps,
                   const Eigen::Matrix3d& ric,
                   const Eigen::Vector3d& tic);

  // -----------------------------------------------------------------------
  // Sliding-window maintenance
  // -----------------------------------------------------------------------

  /// @brief Remove the oldest frame from the window (marginalize back).
  void removeBack();

  /// @brief Remove the oldest frame and shift depth estimates into the new
  ///        anchor frame.
  void removeBackShiftDepth(const Eigen::Matrix3d& marg_R,
                            const Eigen::Vector3d& marg_P,
                            const Eigen::Matrix3d& new_R,
                            const Eigen::Vector3d& new_P);

  /// @brief Remove the second-newest frame from the window (marginalize
  ///        front / non-keyframe).
  void removeFront(int frame_count);

  // -----------------------------------------------------------------------
  // State queries
  // -----------------------------------------------------------------------

  /// @brief Count features that are observed in >= 2 frames and started
  ///        early enough in the window to participate in optimisation.
  int getFeatureCount() const;

  /// @brief Direct (mutable) access to the feature bank.
  std::list<FeaturePerId>& features() { return features_; }

  /// @brief Direct (const) access to the feature bank.
  const std::list<FeaturePerId>& features() const { return features_; }

  /// @brief Reset the feature bank to empty.
  void clearState();

  /// @brief Number of features that were tracked (not newly detected) in
  ///        the last call to addFeatureCheckParallax.
  int lastTrackNum() const { return last_track_num_; }

 private:
  /// @brief Compute the compensated parallax of a feature between the
  ///        second-to-last and third-to-last frames.
  double compensatedParallax(const FeaturePerId& f, int frame_count) const;

  /// All features currently alive in the sliding window.
  std::list<FeaturePerId> features_;

  /// Minimum parallax threshold (in pixels, normalised by focal length
  /// when compared).
  double min_parallax_;

  /// Number of features that were successfully tracked (not new) in the
  /// most recent addFeatureCheckParallax call.
  int last_track_num_ = 0;

  /// Default initial depth for features with degenerate triangulation.
  static constexpr double kInitDepth = 5.0;
};

}  // namespace frontend
}  // namespace slam

#endif  // SLAM_FRONTEND_FEATURE_MANAGER_H_
