#ifndef FRONTEND__FEATURE_TRACKER_H
#define FRONTEND__FEATURE_TRACKER_H

#include <cstdio>
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>
#include <vector>
#include <map>
#include <string>

#include "common/camera_models/CameraFactory.h"
#include "common/camera_models/EquidistantCamera.h"
#include "common/camera_models/PinholeCamera.h"
#include "utility/config.h"

namespace frontend {

bool inBorder(const cv::Point2f& pt);

void filterByStatus(std::vector<cv::Point2f>& v, std::vector<uchar> status);
void filterByStatus(std::vector<int>& v, std::vector<uchar> status);

class FeatureTracker {
public:
    FeatureTracker();

    void detectAndTrack(const cv::Mat& _img, double _cur_time);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const std::string& calib_file);

    // Direct camera model initialization (no file I/O, for WASM)
    void setIntrinsicParameter(int model_type, int width, int height,
                               double fx, double fy, double cx, double cy,
                               double k2, double k3, double k4, double k5);

    void rejectWithFundamentalMatrix();

    void undistortedPoints();

    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, next_img;
    std::vector<cv::Point2f> n_pts;
    std::vector<cv::Point2f> prev_pts, cur_pts, next_pts;
    std::vector<cv::Point2f> prev_undistorted_pts, cur_undistorted_pts;
    std::vector<cv::Point2f> pts_velocity;
    std::vector<int> ids;
    std::vector<int> track_cnt;
    std::map<int, cv::Point2f> cur_undistorted_pts_map;
    std::map<int, cv::Point2f> prev_undistorted_pts_map;
    common::camera_models::CameraPtr m_camera;
    double cur_time;
    double prev_time;

    static int n_id;
};

}  // namespace frontend
#endif  // FRONTEND__FEATURE_TRACKER_H