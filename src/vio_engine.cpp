#include "vio_engine.h"
#include "utility/logging.h"
#include "backend/factor/projection_factor.h"
#include "common/common_types.h"

#include <opencv2/opencv.hpp>
#include <cstring>

VIOEngine::VIOEngine()
    : configured_(false),
      current_time_(-1.0),
      prev_image_timestamp_(-1.0),
      prev_acc_(Eigen::Vector3d::Zero()),
      prev_gyro_(Eigen::Vector3d::Zero()),
      first_imu_(false),
      latest_position_(Eigen::Vector3d::Zero()),
      latest_rotation_(Eigen::Matrix3d::Identity()),
      has_valid_pose_(false) {
}

VIOEngine::~VIOEngine() = default;

bool VIOEngine::configure(int width, int height,
                          double fx, double fy, double cx, double cy,
                          int model_type,
                          double k2, double k3, double k4, double k5,
                          const double* r_ic, const double* t_ic,
                          double acc_n, double acc_w,
                          double gyr_n, double gyr_w,
                          double g_norm) {
    // Set global config directly
    auto& cfg = utility::g_config;
    cfg.camera.col = width;
    cfg.camera.row = height;
    cfg.camera.fx = fx;
    cfg.camera.fy = fy;
    cfg.camera.cx = cx;
    cfg.camera.cy = cy;
    cfg.camera.focal_length = (fx + fy) / 2.0;

    // Extrinsic rotation (row-major 3x3)
    if (r_ic) {
        cfg.camera.r_ic << r_ic[0], r_ic[1], r_ic[2],
                           r_ic[3], r_ic[4], r_ic[5],
                           r_ic[6], r_ic[7], r_ic[8];
    }
    // Extrinsic translation
    if (t_ic) {
        cfg.camera.t_ic << t_ic[0], t_ic[1], t_ic[2];
    }

    // IMU noise parameters
    cfg.estimator.acc_n = acc_n;
    cfg.estimator.acc_w = acc_w;
    cfg.estimator.gyr_n = gyr_n;
    cfg.estimator.gyr_w = gyr_w;
    cfg.estimator.g = Eigen::Vector3d(0.0, 0.0, g_norm);

    // Feature tracker defaults for WASM
    cfg.feature_tracker.show_track = 0;  // No GUI in WASM
    cfg.feature_tracker.max_cnt = 150;
    cfg.feature_tracker.min_dist = 20;
    cfg.feature_tracker.equalize = 1;
    cfg.feature_tracker.fisheye = 0;
    cfg.feature_tracker.f_threshold = 1.0;
    cfg.feature_tracker.window_size = 20;

    // Estimator defaults
    cfg.estimator.solver_time = 0.1;
    cfg.estimator.num_iterations = 10;
    cfg.estimator.min_parallax = 10.0;

    // Create estimator and feature tracker
    estimator_ = std::make_unique<backend::Estimator>();
    estimator_->setParameter();

    feature_tracker_ = std::make_unique<frontend::FeatureTracker>();

    // For WASM, we need to create camera model from parameters directly
    // Build a temporary YAML-like config string for the camera factory
    // This uses the existing camera model infrastructure
    std::string temp_config;
    if (model_type == 1) { // KANNALA_BRANDT
        temp_config = "%YAML:1.0\n"
            "model_type: KANNALA_BRANDT\n"
            "camera_name: camera\n"
            "image_width: " + std::to_string(width) + "\n"
            "image_height: " + std::to_string(height) + "\n"
            "projection_parameters:\n"
            "   k2: " + std::to_string(k2) + "\n"
            "   k3: " + std::to_string(k3) + "\n"
            "   k4: " + std::to_string(k4) + "\n"
            "   k5: " + std::to_string(k5) + "\n"
            "   mu: " + std::to_string(fx) + "\n"
            "   mv: " + std::to_string(fy) + "\n"
            "   u0: " + std::to_string(cx) + "\n"
            "   v0: " + std::to_string(cy) + "\n";
    } else { // PINHOLE
        temp_config = "%YAML:1.0\n"
            "model_type: PINHOLE\n"
            "camera_name: camera\n"
            "image_width: " + std::to_string(width) + "\n"
            "image_height: " + std::to_string(height) + "\n"
            "distortion_parameters:\n"
            "   k1: " + std::to_string(k2) + "\n"
            "   k2: " + std::to_string(k3) + "\n"
            "   p1: " + std::to_string(k4) + "\n"
            "   p2: " + std::to_string(k5) + "\n"
            "projection_parameters:\n"
            "   fx: " + std::to_string(fx) + "\n"
            "   fy: " + std::to_string(fy) + "\n"
            "   cx: " + std::to_string(cx) + "\n"
            "   cy: " + std::to_string(cy) + "\n";
    }

    // Write temp config file for camera model initialization
    std::string temp_path = "/tmp/vio_engine_camera.yaml";
    std::ofstream ofs(temp_path);
    if (ofs.is_open()) {
        ofs << temp_config;
        ofs.close();
        feature_tracker_->readIntrinsicParameter(temp_path);
    } else {
        LOG_ERROR("Failed to create temp camera config");
        return false;
    }

    // Set projection factor info
    backend::factor::ProjectionFactor::sqrt_info =
        (cfg.camera.focal_length / 1.5) * Eigen::Matrix2d::Identity();

    configured_ = true;
    current_time_ = -1.0;
    prev_image_timestamp_ = -1.0;
    first_imu_ = false;
    has_valid_pose_ = false;

    LOG_INFO("VIOEngine configured: " << width << "x" << height
             << " fx=" << fx << " fy=" << fy);
    return true;
}

void VIOEngine::processIMUData(const IMUReading* readings, int count,
                                double image_timestamp) {
    for (int i = 0; i < count; i++) {
        const auto& imu = readings[i];
        double imu_time = imu.timestamp;
        Eigen::Vector3d acc(imu.acc_x, imu.acc_y, imu.acc_z);
        Eigen::Vector3d gyro(imu.gyro_x, imu.gyro_y, imu.gyro_z);

        if (imu_time <= image_timestamp) {
            if (current_time_ < 0.0) {
                current_time_ = imu_time;
            }
            double dt = imu_time - current_time_;
            // Skip invalid dt: negative (out-of-order) or too large (sensor gap)
            if (dt < 0.0 || dt > 0.5) {
                current_time_ = imu_time;
                prev_acc_ = acc;
                prev_gyro_ = gyro;
                continue;
            }
            current_time_ = imu_time;
            estimator_->processIMU(dt, acc, gyro);
        } else {
            // Interpolate at image timestamp
            double dt_to_image = image_timestamp - current_time_;
            // Skip invalid interpolation interval
            if (dt_to_image < 0.0 || dt_to_image > 0.5) {
                current_time_ = image_timestamp;
                prev_acc_ = acc;
                prev_gyro_ = gyro;
                continue;
            }
            current_time_ = image_timestamp;

            double total_dt = dt_to_image + (imu_time - image_timestamp);
            Eigen::Vector3d interp_acc, interp_gyro;
            if (total_dt < 1e-12) {
                interp_acc = acc;
                interp_gyro = gyro;
            } else {
                double w1 = (imu_time - image_timestamp) / total_dt;
                double w2 = dt_to_image / total_dt;
                interp_acc = w1 * prev_acc_ + w2 * acc;
                interp_gyro = w1 * prev_gyro_ + w2 * gyro;
            }
            estimator_->processIMU(dt_to_image, interp_acc, interp_gyro);
        }
        prev_acc_ = acc;
        prev_gyro_ = gyro;
    }
}

bool VIOEngine::processFrame(const uint8_t* gray_image, int width, int height,
                              const IMUReading* imu_readings, int imu_count,
                              double* pose_output) {
    if (!configured_ || !estimator_ || !feature_tracker_) {
        return false;
    }

    // Create cv::Mat from raw data (no copy)
    cv::Mat img(height, width, CV_8UC1, const_cast<uint8_t*>(gray_image));

    // Use the last IMU timestamp as image timestamp, or derive from IMU
    double image_timestamp;
    if (imu_count > 0) {
        image_timestamp = imu_readings[imu_count - 1].timestamp;
    } else {
        // Fallback: increment by ~33ms
        image_timestamp = (prev_image_timestamp_ > 0)
                              ? prev_image_timestamp_ + 0.033
                              : 0.0;
    }

    // Process IMU data
    if (imu_count > 0) {
        processIMUData(imu_readings, imu_count, image_timestamp);
    }

    // Feature tracking
    feature_tracker_->detectAndTrack(img, image_timestamp);

    // Update feature IDs
    for (unsigned int i = 0;; i++) {
        bool completed = feature_tracker_->updateID(i);
        if (!completed) break;
    }

    // Build image data for estimator
    common::ImageData image_data;
    auto& undistorted_pts = feature_tracker_->cur_undistorted_pts;
    auto& cur_pts = feature_tracker_->cur_pts;
    auto& ids = feature_tracker_->ids;
    auto& pts_velocity = feature_tracker_->pts_velocity;

    for (unsigned int j = 0; j < ids.size(); j++) {
        if (feature_tracker_->track_cnt[j] > 1) {
            int p_id = ids[j];
            double r_x = undistorted_pts[j].x;
            double r_y = undistorted_pts[j].y;
            double r_z = 1.0;
            double p_u = cur_pts[j].x;
            double p_v = cur_pts[j].y;
            double v_x = pts_velocity[j].x;
            double v_y = pts_velocity[j].y;
            Eigen::Matrix<double, 7, 1> ray_obs_vel;
            ray_obs_vel << r_x, r_y, r_z, p_u, p_v, v_x, v_y;
            image_data[p_id] = ray_obs_vel;
        }
    }

    // Process image in estimator
    if (!image_data.empty()) {
        estimator_->processImage(image_data, image_timestamp);
    }

    prev_image_timestamp_ = image_timestamp;

    // Extract pose if VIO is initialized
    if (estimator_->solver_flag_ == common::SolverFlag::NON_LINEAR) {
        int ws = utility::g_config.estimator.window_size;
        Eigen::Vector3d body_pos = estimator_->sliding_window_[ws].P;
        Eigen::Matrix3d body_rot = estimator_->sliding_window_[ws].R;

        // Detect numerical divergence: position > 1e6 or NaN/Inf
        if (!body_pos.allFinite() || !body_rot.allFinite() ||
            body_pos.norm() > 1e6) {
            LOG_WARN("VIO numerical divergence detected, resetting...");
            // Preserve config but reset estimator state
            estimator_ = std::make_unique<backend::Estimator>();
            estimator_->setParameter();
            current_time_ = -1.0;
            prev_image_timestamp_ = -1.0;
            first_imu_ = false;
            has_valid_pose_ = false;
            return false;
        }

        // Camera pose = body + extrinsic offset
        latest_position_ = body_pos + body_rot * estimator_->t_ic_;
        latest_rotation_ = body_rot * estimator_->r_ic_;
        has_valid_pose_ = true;

        // Write 4x4 transformation matrix (row-major)
        if (pose_output) {
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 3>(0, 0) = latest_rotation_;
            T.block<3, 1>(0, 3) = latest_position_;
            // Copy row-major
            for (int r = 0; r < 4; r++) {
                for (int c = 0; c < 4; c++) {
                    pose_output[r * 4 + c] = T(r, c);
                }
            }
        }
        return true;
    }

    return false;
}

bool VIOEngine::isInitialized() const {
    if (!estimator_) return false;
    return estimator_->solver_flag_ == common::SolverFlag::NON_LINEAR;
}

int VIOEngine::getFeaturePointCount() const {
    if (!feature_tracker_) return 0;
    int count = 0;
    for (unsigned int j = 0; j < feature_tracker_->ids.size(); j++) {
        if (feature_tracker_->track_cnt[j] > 1) {
            count++;
        }
    }
    return count;
}

int VIOEngine::getMapPoints(double* output, int max_count) const {
    if (!estimator_ || !has_valid_pose_) return 0;

    auto points = estimator_->getSlidingWindowMapPoints();
    int count = std::min(static_cast<int>(points.size()), max_count);
    for (int i = 0; i < count; i++) {
        output[i * 3 + 0] = points[i].x();
        output[i * 3 + 1] = points[i].y();
        output[i * 3 + 2] = points[i].z();
    }
    return count;
}

void VIOEngine::reset() {
    estimator_ = std::make_unique<backend::Estimator>();
    if (configured_) {
        estimator_->setParameter();
    }
    current_time_ = -1.0;
    prev_image_timestamp_ = -1.0;
    first_imu_ = false;
    has_valid_pose_ = false;
    latest_position_ = Eigen::Vector3d::Zero();
    latest_rotation_ = Eigen::Matrix3d::Identity();
    prev_acc_ = Eigen::Vector3d::Zero();
    prev_gyro_ = Eigen::Vector3d::Zero();
}
