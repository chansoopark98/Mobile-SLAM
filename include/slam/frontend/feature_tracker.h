// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#ifndef SLAM_FRONTEND_FEATURE_TRACKER_H_
#define SLAM_FRONTEND_FEATURE_TRACKER_H_

#include <Eigen/Dense>
#include <map>
#include <opencv2/opencv.hpp>
#include <vector>

#include "slam/common/types.h"

namespace slam {
namespace frontend {

// ---------------------------------------------------------------------------
// Free helpers
// ---------------------------------------------------------------------------

/// @brief Check whether a pixel lies inside the image border.
/// @param pt  Pixel coordinates.
/// @param rows  Image height.
/// @param cols  Image width.
/// @param border_size  Minimum distance from the edge (default 1).
/// @return true if the point is at least @p border_size pixels from every edge.
bool inBorder(const cv::Point2f& pt, int rows, int cols, int border_size = 1);

/// @brief Remove elements from a vector<cv::Point2f> according to a status mask.
void filterByStatus(std::vector<cv::Point2f>& v, const std::vector<uchar>& status);

/// @brief Remove elements from a vector<int> according to a status mask.
void filterByStatus(std::vector<int>& v, const std::vector<uchar>& status);

// ---------------------------------------------------------------------------
// FeatureTracker
// ---------------------------------------------------------------------------

/// @brief Tracks features across consecutive monocular images using KLT
///        optical flow, with RANSAC outlier rejection and automatic feature
///        replenishment.
///
/// This class is a self-contained, ROS-free adaptation of the VINS-Mono
/// feature tracker.  Instead of relying on a generic camera model, it
/// performs simple pinhole un-distortion internally (no distortion
/// coefficients -- suitable for pre-rectified or low-distortion imagery).
///
/// Usage:
/// @code
///   FeatureTracker tracker(460.0, 320.0, 240.0);
///   auto observations = tracker.trackImage(t, gray_image);
/// @endcode
class FeatureTracker {
 public:
  /// @brief Construct a feature tracker with pinhole camera intrinsics.
  /// @param focal_length  Focal length in pixels (used for F-matrix RANSAC).
  /// @param cx  Principal point x (pixels).
  /// @param cy  Principal point y (pixels).
  /// @param max_cnt  Maximum number of features to maintain.
  /// @param min_dist  Minimum pixel distance between features.
  /// @param f_threshold  RANSAC fundamental-matrix inlier threshold (pixels).
  /// @param equalize  Whether to apply CLAHE histogram equalization.
  FeatureTracker(double focal_length, double cx, double cy,
                 int max_cnt = 150, int min_dist = 30,
                 double f_threshold = 1.0, bool equalize = true);

  /// @brief Process one grayscale frame and return feature observations.
  ///
  /// The returned map is keyed by a globally unique feature ID.  Each value
  /// is a 7-DOF vector:
  ///   [ray_x, ray_y, ray_z, pixel_u, pixel_v, vel_x, vel_y]
  ///
  /// @param timestamp  Capture time in seconds.
  /// @param image      Single-channel 8-bit grayscale image (CV_8UC1).
  /// @return Per-feature observation map.
  common::ImageData trackImage(double timestamp, const cv::Mat& image);

  // -----------------------------------------------------------------------
  // Accessors (for external inspection / debugging)
  // -----------------------------------------------------------------------

  /// @brief Number of currently tracked features.
  int getTrackedCount() const { return static_cast<int>(cur_pts_.size()); }

  /// @brief Read-only access to current tracked pixel positions.
  const std::vector<cv::Point2f>& getCurrentPoints() const { return cur_pts_; }

  /// @brief Read-only access to feature IDs (aligned with getCurrentPoints).
  const std::vector<int>& getIds() const { return ids_; }

 private:
  // -- Internal pipeline stages -------------------------------------------

  /// @brief Reject outliers via RANSAC on the fundamental matrix.
  void rejectWithF();

  /// @brief Build the feature mask so new detections keep min_dist_ spacing
  ///        and prioritise long-lived tracks.
  void setMask();

  /// @brief Append newly detected points to the tracking buffers.
  void addPoints();

  /// @brief Compute un-distorted (normalised) coordinates and velocities
  ///        for all currently tracked features.
  void undistortedPoints();

  /// @brief Convert a single pixel to a normalised camera coordinate.
  cv::Point2f pixelToNormalized(const cv::Point2f& pt) const;

  /// @brief Assign a fresh global ID to every feature whose id is still -1.
  void updateIds();

  // -- Camera parameters --------------------------------------------------
  double focal_length_;
  double cx_;
  double cy_;

  // -- Tracker parameters -------------------------------------------------
  int max_cnt_;
  int min_dist_;
  double f_threshold_;
  bool equalize_;

  // -- Frame buffers ------------------------------------------------------
  cv::Mat prev_image_;
  cv::Mat cur_image_;
  cv::Mat forw_image_;

  // -- Point buffers ------------------------------------------------------
  std::vector<cv::Point2f> prev_pts_;
  std::vector<cv::Point2f> cur_pts_;
  std::vector<cv::Point2f> forw_pts_;
  std::vector<cv::Point2f> n_pts_;  // newly detected points

  // -- Undistorted / normalised buffers -----------------------------------
  std::vector<cv::Point2f> cur_undistorted_pts_;
  std::vector<cv::Point2f> pts_velocity_;
  std::map<int, cv::Point2f> cur_undistorted_pts_map_;
  std::map<int, cv::Point2f> prev_undistorted_pts_map_;

  // -- Bookkeeping --------------------------------------------------------
  std::vector<int> ids_;
  std::vector<int> track_cnt_;
  int next_id_ = 0;

  // -- Timing -------------------------------------------------------------
  double cur_time_ = 0.0;
  double prev_time_ = 0.0;

  // -- Mask ---------------------------------------------------------------
  cv::Mat mask_;
};

}  // namespace frontend
}  // namespace slam

#endif  // SLAM_FRONTEND_FEATURE_TRACKER_H_
