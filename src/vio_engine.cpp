#include "vio_engine.h"
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
      latest_position_(Eigen::Vector3d::Zero()),
      latest_rotation_(Eigen::Matrix3d::Identity()),
      has_valid_pose_(false),
      consecutive_failures_(0),
      cooldown_counter_(0),
      frames_since_init_start_(0),
      init_start_time_(-1.0) {
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

    // Estimator defaults (optimized for mobile WASM: 30fps → 33ms budget)
    cfg.estimator.solver_time = 0.04;
    cfg.estimator.num_iterations = 6;
    cfg.estimator.min_parallax = 10.0;

    // Create estimator and feature tracker
    estimator_ = std::make_unique<backend::Estimator>();
    estimator_->setParameter();

    feature_tracker_ = std::make_unique<frontend::FeatureTracker>();

    // Direct camera model initialization (no file I/O, safe for WASM)
    feature_tracker_->setIntrinsicParameter(model_type, width, height,
                                             fx, fy, cx, cy,
                                             k2, k3, k4, k5);

    configured_ = true;
    current_time_ = -1.0;
    prev_image_timestamp_ = -1.0;
    has_valid_pose_ = false;
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
            // Skip invalid dt: near-zero, negative (out-of-order), or too large (sensor gap)
            // Threshold 1e-4 (0.1ms) allows up to 10kHz IMU rate while filtering
            // duplicate timestamps from browser jitter.
            if (dt < 1e-4 || dt > 0.5) {
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
            if (dt_to_image < 1e-4 || dt_to_image > 0.5) {
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
                              double image_timestamp,
                              double* pose_output) {
    if (!configured_ || !estimator_ || !feature_tracker_) {
        return false;
    }

    // Cooldown mode: skip processing to prevent infinite reset loop
    if (cooldown_counter_ > 0) {
        cooldown_counter_--;
        return false;
    }

    // Create cv::Mat from raw data (no copy)
    cv::Mat img(height, width, CV_8UC1, const_cast<uint8_t*>(gray_image));

    // Process IMU data
    if (imu_count > 0) {
        processIMUData(imu_readings, imu_count, image_timestamp);
    }

    // Initialization timeout check
    bool is_initializing = (estimator_->solver_flag_ != common::SolverFlag::NON_LINEAR);
    if (is_initializing) {
        frames_since_init_start_++;
        if (init_start_time_ < 0) {
            init_start_time_ = image_timestamp;
        } else if (image_timestamp - init_start_time_ > kInitTimeoutSeconds) {
            estimator_ = std::make_unique<backend::Estimator>();
            estimator_->setParameter();
            current_time_ = -1.0;
            prev_image_timestamp_ = -1.0;
            has_valid_pose_ = false;
            init_start_time_ = -1.0;
            frames_since_init_start_ = 0;
            return false;
        }
    } else {
        // Reset init timer once tracking
        init_start_time_ = -1.0;
        frames_since_init_start_ = 0;
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

    // Detect initialization loss (was tracking, now reverted to INITIAL)
    bool currently_initialized = (estimator_->solver_flag_ == common::SolverFlag::NON_LINEAR);
    if (has_valid_pose_ && !currently_initialized) {
        consecutive_failures_++;
        has_valid_pose_ = false;
        if (consecutive_failures_ >= kMaxConsecutiveFailures) {
            cooldown_counter_ = kCooldownFrames;
            consecutive_failures_ = 0;
        }
        return false;
    }

    // Extract pose if VIO is initialized
    if (currently_initialized) {
        Eigen::Vector3d body_pos = estimator_->sliding_window_[WINDOW_SIZE].P;
        Eigen::Matrix3d body_rot = estimator_->sliding_window_[WINDOW_SIZE].R;

        // Detect numerical divergence: position > 1e6 or NaN/Inf
        if (!body_pos.allFinite() || !body_rot.allFinite() ||
            body_pos.norm() > 1e6) {
            consecutive_failures_++;
            estimator_ = std::make_unique<backend::Estimator>();
            estimator_->setParameter();
            current_time_ = -1.0;
            prev_image_timestamp_ = -1.0;
            has_valid_pose_ = false;
            if (consecutive_failures_ >= kMaxConsecutiveFailures) {
                cooldown_counter_ = kCooldownFrames;
                consecutive_failures_ = 0;
            }
            return false;
        }

        // Camera pose = body + extrinsic offset
        latest_position_ = body_pos + body_rot * estimator_->t_ic_;
        latest_rotation_ = body_rot * estimator_->r_ic_;
        has_valid_pose_ = true;

        // Write 4x4 transformation matrix (row-major) directly
        if (pose_output) {
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    pose_output[r * 4 + c] = latest_rotation_(r, c);
                }
                pose_output[r * 4 + 3] = latest_position_(r);
            }
            pose_output[12] = 0.0;
            pose_output[13] = 0.0;
            pose_output[14] = 0.0;
            pose_output[15] = 1.0;
        }
        consecutive_failures_ = 0;
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

int VIOEngine::getStatusCode() const {
    if (!configured_) return static_cast<int>(VIOStatus::NOT_CONFIGURED);
    if (cooldown_counter_ > 0) return static_cast<int>(VIOStatus::COOLDOWN);
    if (!estimator_) return static_cast<int>(VIOStatus::NOT_CONFIGURED);
    if (estimator_->solver_flag_ == common::SolverFlag::NON_LINEAR) {
        return has_valid_pose_
            ? static_cast<int>(VIOStatus::TRACKING)
            : static_cast<int>(VIOStatus::LOST);
    }
    return static_cast<int>(VIOStatus::INITIALIZING);
}

void VIOEngine::setMobileParams(double solver_time, int num_iterations, int max_features) {
    auto& cfg = utility::g_config;
    cfg.estimator.solver_time = solver_time;
    cfg.estimator.num_iterations = num_iterations;
    cfg.feature_tracker.max_cnt = max_features;
}

void VIOEngine::reset() {
    estimator_ = std::make_unique<backend::Estimator>();
    if (configured_) {
        estimator_->setParameter();
    }
    current_time_ = -1.0;
    prev_image_timestamp_ = -1.0;
    has_valid_pose_ = false;
    latest_position_ = Eigen::Vector3d::Zero();
    latest_rotation_ = Eigen::Matrix3d::Identity();
    prev_acc_ = Eigen::Vector3d::Zero();
    prev_gyro_ = Eigen::Vector3d::Zero();
    consecutive_failures_ = 0;
    cooldown_counter_ = 0;
    frames_since_init_start_ = 0;
    init_start_time_ = -1.0;
}
