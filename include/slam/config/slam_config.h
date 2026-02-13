#ifndef SLAM__CONFIG__SLAM_CONFIG_H
#define SLAM__CONFIG__SLAM_CONFIG_H

#include <Eigen/Dense>
#include <string>

namespace slam {
namespace config {

/// Camera intrinsic and extrinsic parameters.
struct CameraConfig {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double k1 = 0.0;  // distortion
    double k2 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
    int width = 0;
    int height = 0;
    Eigen::Matrix3d r_ic = Eigen::Matrix3d::Identity();  // IMU-to-camera rotation
    Eigen::Vector3d t_ic = Eigen::Vector3d::Zero();       // IMU-to-camera translation
};

/// Feature tracker parameters.
struct FeatureTrackerConfig {
    int max_cnt = 150;
    int min_dist = 30;
    double f_threshold = 1.0;
    bool equalize = true;
};

/// Estimator (backend optimizer) parameters.
struct EstimatorConfig {
    int window_size = 10;
    int num_iterations = 10;
    double solver_time = 0.1;
    double min_parallax = 10.0;
    double init_depth = 5.0;
    double acc_n = 0.08;
    double gyr_n = 0.004;
    double acc_w = 0.00004;
    double gyr_w = 2.0e-6;
    Eigen::Vector3d gravity{0.0, 0.0, 9.81007};
};

/// Top-level SLAM configuration holding all sub-configs.
struct SlamConfig {
    CameraConfig camera;
    FeatureTrackerConfig feature_tracker;
    EstimatorConfig estimator;
    std::string dataset_path;

    /// Load configuration from a simple key=value text file.
    /// Lines starting with '#' are treated as comments and skipped.
    /// Returns true on success.
    bool loadFromFile(const std::string& filepath);

    /// Pre-configured defaults for TUM VI room1_512_16 dataset.
    static SlamConfig DefaultTumVi();

    /// Pre-configured defaults for a generic webcam given intrinsics.
    static SlamConfig DefaultWebcam(double fx, double fy, double cx, double cy);

    /// Print all configuration values to stdout.
    void print() const;
};

}  // namespace config
}  // namespace slam

#endif  // SLAM__CONFIG__SLAM_CONFIG_H
