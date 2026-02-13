// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "slam/frontend/feature_tracker.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>

namespace slam {
namespace frontend {

// ---------------------------------------------------------------------------
// Free helpers
// ---------------------------------------------------------------------------

bool inBorder(const cv::Point2f& pt, int rows, int cols, int border_size) {
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return border_size <= img_x && img_x < cols - border_size &&
         border_size <= img_y && img_y < rows - border_size;
}

void filterByStatus(std::vector<cv::Point2f>& v,
                    const std::vector<uchar>& status) {
  int j = 0;
  for (int i = 0; i < static_cast<int>(v.size()); ++i) {
    if (status[i]) {
      v[j++] = v[i];
    }
  }
  v.resize(j);
}

void filterByStatus(std::vector<int>& v,
                    const std::vector<uchar>& status) {
  int j = 0;
  for (int i = 0; i < static_cast<int>(v.size()); ++i) {
    if (status[i]) {
      v[j++] = v[i];
    }
  }
  v.resize(j);
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

FeatureTracker::FeatureTracker(double focal_length, double cx, double cy,
                               int max_cnt, int min_dist,
                               double f_threshold, bool equalize)
    : focal_length_(focal_length),
      cx_(cx),
      cy_(cy),
      max_cnt_(max_cnt),
      min_dist_(min_dist),
      f_threshold_(f_threshold),
      equalize_(equalize) {}

// ---------------------------------------------------------------------------
// Main entry point
// ---------------------------------------------------------------------------

common::ImageData FeatureTracker::trackImage(double timestamp,
                                             const cv::Mat& image) {
  cv::Mat img;
  cur_time_ = timestamp;

  // -- Optional CLAHE histogram equalization ------------------------------
  if (equalize_) {
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(image, img);
  } else {
    img = image;
  }

  // -- First frame initialisation -----------------------------------------
  if (forw_image_.empty()) {
    prev_image_ = cur_image_ = forw_image_ = img;
  } else {
    forw_image_ = img;
  }

  forw_pts_.clear();

  // -- KLT optical flow tracking ------------------------------------------
  if (!cur_pts_.empty()) {
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(cur_image_, forw_image_, cur_pts_, forw_pts_,
                             status, err, cv::Size(21, 21), 3);

    // Reject points that drifted outside the image border
    for (int i = 0; i < static_cast<int>(forw_pts_.size()); ++i) {
      if (status[i] &&
          !inBorder(forw_pts_[i], forw_image_.rows, forw_image_.cols)) {
        status[i] = 0;
      }
    }

    filterByStatus(prev_pts_, status);
    filterByStatus(cur_pts_, status);
    filterByStatus(forw_pts_, status);
    filterByStatus(ids_, status);
    filterByStatus(cur_undistorted_pts_, status);
    filterByStatus(track_cnt_, status);
  }

  // -- Increment track counts ---------------------------------------------
  for (auto& n : track_cnt_) {
    ++n;
  }

  // -- RANSAC outlier rejection -------------------------------------------
  rejectWithF();

  // -- Build mask and select long-lived features --------------------------
  setMask();

  // -- Detect new features to fill up to max_cnt_ -------------------------
  int supplementary = max_cnt_ - static_cast<int>(forw_pts_.size());
  if (supplementary > 0) {
    if (mask_.empty()) {
      std::cerr << "[FeatureTracker] mask is empty\n";
    }
    cv::goodFeaturesToTrack(forw_image_, n_pts_, supplementary, 0.01,
                            min_dist_, mask_);
  } else {
    n_pts_.clear();
  }

  addPoints();

  // -- Shift buffers ------------------------------------------------------
  prev_image_ = cur_image_;
  prev_pts_ = cur_pts_;
  cur_image_ = forw_image_;
  cur_pts_ = forw_pts_;

  // -- Assign IDs to newly detected features ------------------------------
  updateIds();

  // -- Compute undistorted coordinates and velocities ---------------------
  undistortedPoints();
  prev_time_ = cur_time_;

  // -- Pack output --------------------------------------------------------
  common::ImageData result;
  for (int i = 0; i < static_cast<int>(cur_pts_.size()); ++i) {
    Eigen::Matrix<double, 7, 1> obs;
    // Normalised ray (x, y, 1) -- already computed
    obs(0) = cur_undistorted_pts_[i].x;
    obs(1) = cur_undistorted_pts_[i].y;
    obs(2) = 1.0;
    // Raw pixel coordinates
    obs(3) = cur_pts_[i].x;
    obs(4) = cur_pts_[i].y;
    // Velocity in normalised coordinates
    obs(5) = pts_velocity_[i].x;
    obs(6) = pts_velocity_[i].y;
    result[ids_[i]] = obs;
  }

  return result;
}

// ---------------------------------------------------------------------------
// RANSAC outlier rejection via the fundamental matrix
// ---------------------------------------------------------------------------

void FeatureTracker::rejectWithF() {
  if (forw_pts_.size() < 8) {
    return;
  }

  // Project both point sets into a virtual undistorted image whose principal
  // point is at image centre and whose focal length equals focal_length_.
  // This gives well-scaled pixel coordinates for findFundamentalMat.
  std::vector<cv::Point2f> undist_cur(cur_pts_.size());
  std::vector<cv::Point2f> undist_forw(forw_pts_.size());

  for (size_t i = 0; i < cur_pts_.size(); ++i) {
    // Normalise then re-project with the virtual focal length
    cv::Point2f nc = pixelToNormalized(cur_pts_[i]);
    undist_cur[i].x =
        static_cast<float>(focal_length_ * nc.x + cx_);
    undist_cur[i].y =
        static_cast<float>(focal_length_ * nc.y + cy_);

    cv::Point2f nf = pixelToNormalized(forw_pts_[i]);
    undist_forw[i].x =
        static_cast<float>(focal_length_ * nf.x + cx_);
    undist_forw[i].y =
        static_cast<float>(focal_length_ * nf.y + cy_);
  }

  std::vector<uchar> status;
  cv::findFundamentalMat(undist_cur, undist_forw, cv::FM_RANSAC,
                         f_threshold_, 0.99, status);

  filterByStatus(prev_pts_, status);
  filterByStatus(cur_pts_, status);
  filterByStatus(forw_pts_, status);
  filterByStatus(cur_undistorted_pts_, status);
  filterByStatus(ids_, status);
  filterByStatus(track_cnt_, status);
}

// ---------------------------------------------------------------------------
// Mask construction -- prioritise long-lived features
// ---------------------------------------------------------------------------

void FeatureTracker::setMask() {
  mask_ = cv::Mat(forw_image_.rows, forw_image_.cols, CV_8UC1,
                  cv::Scalar(255));

  // Sort features by descending track count so long-lived tracks are
  // placed first (and thus get priority when the mask is burned).
  std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;
  cnt_pts_id.reserve(forw_pts_.size());

  for (size_t i = 0; i < forw_pts_.size(); ++i) {
    cnt_pts_id.emplace_back(
        track_cnt_[i],
        std::make_pair(forw_pts_[i], ids_[i]));
  }

  std::sort(cnt_pts_id.begin(), cnt_pts_id.end(),
            [](const auto& a, const auto& b) {
              return a.first > b.first;
            });

  forw_pts_.clear();
  ids_.clear();
  track_cnt_.clear();

  for (auto& it : cnt_pts_id) {
    if (mask_.at<uchar>(it.second.first) == 255) {
      forw_pts_.push_back(it.second.first);
      ids_.push_back(it.second.second);
      track_cnt_.push_back(it.first);
      cv::circle(mask_, it.second.first, min_dist_, 0, -1);
    }
  }
}

// ---------------------------------------------------------------------------
// Append newly detected points
// ---------------------------------------------------------------------------

void FeatureTracker::addPoints() {
  for (auto& p : n_pts_) {
    forw_pts_.push_back(p);
    ids_.push_back(-1);
    track_cnt_.push_back(1);
  }
}

// ---------------------------------------------------------------------------
// Assign globally unique IDs to features that are still marked as new (-1)
// ---------------------------------------------------------------------------

void FeatureTracker::updateIds() {
  for (auto& id : ids_) {
    if (id == -1) {
      id = next_id_++;
    }
  }
}

// ---------------------------------------------------------------------------
// Undistort to normalised camera coordinates and compute velocity
// ---------------------------------------------------------------------------

void FeatureTracker::undistortedPoints() {
  cur_undistorted_pts_.clear();
  cur_undistorted_pts_map_.clear();

  for (size_t i = 0; i < cur_pts_.size(); ++i) {
    cv::Point2f np = pixelToNormalized(cur_pts_[i]);
    cur_undistorted_pts_.push_back(np);
    cur_undistorted_pts_map_.insert(std::make_pair(ids_[i], np));
  }

  // -- Compute velocity in normalised coordinates -------------------------
  pts_velocity_.clear();
  if (!prev_undistorted_pts_map_.empty()) {
    double dt = cur_time_ - prev_time_;
    if (dt < 1e-9) {
      dt = 1e-9;  // guard against division by zero
    }
    for (size_t i = 0; i < cur_undistorted_pts_.size(); ++i) {
      if (ids_[i] != -1) {
        auto it = prev_undistorted_pts_map_.find(ids_[i]);
        if (it != prev_undistorted_pts_map_.end()) {
          double vx =
              (cur_undistorted_pts_[i].x - it->second.x) / dt;
          double vy =
              (cur_undistorted_pts_[i].y - it->second.y) / dt;
          pts_velocity_.emplace_back(
              static_cast<float>(vx), static_cast<float>(vy));
        } else {
          pts_velocity_.emplace_back(0.0f, 0.0f);
        }
      } else {
        pts_velocity_.emplace_back(0.0f, 0.0f);
      }
    }
  } else {
    for (size_t i = 0; i < cur_pts_.size(); ++i) {
      pts_velocity_.emplace_back(0.0f, 0.0f);
    }
  }

  prev_undistorted_pts_map_ = cur_undistorted_pts_map_;
}

// ---------------------------------------------------------------------------
// Pixel -> normalised camera coordinate  (simple pinhole, no distortion)
// ---------------------------------------------------------------------------

cv::Point2f FeatureTracker::pixelToNormalized(const cv::Point2f& pt) const {
  return cv::Point2f(
      static_cast<float>((pt.x - cx_) / focal_length_),
      static_cast<float>((pt.y - cy_) / focal_length_));
}

}  // namespace frontend
}  // namespace slam
