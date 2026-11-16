#ifndef UTILITY__CONFIG_H
#define UTILITY__CONFIG_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace utility {

const int WINDOW_SIZE = 10;
const int NUM_OF_FEATURES = 1000;

enum SizeParameterization { SIZE_POSE = 7, SIZE_SPEEDANDBIAS = 9, SIZE_FEATURE = 1 };
enum StateOrder { O_P = 0, O_R = 3, O_V = 6, O_BA = 9, O_BG = 12 };

// Camera configuration
struct CameraConfig {
    double focal_length = 460.0;
    double row = 480.0;
    double col = 752.0;
    Eigen::Matrix3d r_ic = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_ic = Eigen::Vector3d::Zero();

    // Camera intrinsic parameters
    double fx = 460.0;
    double fy = 460.0;
    double cx = 376.0;  // col * 0.5
    double cy = 240.0;  // row * 0.5
};

// Feature tracker configuration — defaults based on VINS-Mobile (480x640 portrait)
struct FeatureTrackerConfig {
    int max_cnt = 120;       // VINS-Mobile: 70. 120 for better triangulation in WASM.
    int min_dist = 30;       // VINS-Mobile: 30. Uniform distribution at 480x640.
    int window_size = 20;
    double f_threshold = 1.0; // VINS-Mobile: 1.0. Same resolution → same threshold.
    int show_track = 1;
    int equalize = 1;
    int fisheye = 0;

    // LK optical flow parameters
    int lk_window_size = 21;    // Lucas-Kanade window size (must be odd)
    int lk_pyramid_levels = 3;  // 3 levels at 480x640 handles ~40px displacement.
    int lk_iterations = 30;     // VINS-Mono default. Precise tracking for all configs.
    double lk_eps = 0.01;       // VINS-Mono default. Sub-pixel precision for all configs.

    // Distance-aware F-matrix rejection for unmodeled lens distortion.
    // When enabled, edge features get a larger RANSAC threshold to
    // compensate for barrel distortion not captured by the camera model.
    // Factor of 2.0 means edge features get up to 3x the base threshold.
    double f_threshold_edge_factor = 0.0;  // 0 = disabled, >0 = quadratic edge boost

    // Image processing
    std::string fisheye_mask;
};

// Estimator configuration — defaults based on VINS-Mobile
struct EstimatorConfig {
    int window_size = 10;
    int num_iterations = 10;    // VINS-Mobile: 10
    double solver_time = 0.06;  // VINS-Mobile: 0.06
    double min_parallax = 10.0;
    double init_depth = 2.0;    // Mobile indoor ~10m environment

    // IMU noise parameters — VINS-Mobile baseline for mobile MEMS
    double acc_n = 0.5;         // VINS-Mobile: 0.5 (was EuRoC 0.08)
    double acc_w = 0.003;       // Fast Ba convergence (was EuRoC 0.00004)
    double gyr_n = 0.2;         // VINS-Mobile: 0.2. Full match — mobile MEMS gyro needs low trust
    double gyr_w = 0.0002;      // Stabilize Bg (was EuRoC 2e-6)

    // Gravity vector
    Eigen::Vector3d g = Eigen::Vector3d(0.0, 0.0, 9.81007);

    // Feature parameters
    int num_of_features = NUM_OF_FEATURES;
};

// PnP Frontend configuration — VINS-Mobile dual-rate pipeline
struct PnPConfig {
    int pnp_size = 6;              // Mini sliding window size (VINS-Mobile: PNP_SIZE=6)
    int freq = 3;                  // Backend decimation rate (every FREQ frames)
    int pnp_max_iterations = 5;    // Lightweight PnP solver iterations
    double pnp_solver_time = 0.01; // PnP solver time limit (10ms)
    bool enable_pnp = false;       // Disabled by default until PnP pipeline is verified. Enable via setPnPParams or ?pnp=1

    // Adaptive solver time for backend
    double base_solver_time = 0.06;  // Full solver time (no load)
    double min_solver_time = 0.04;   // WASM safety floor (Ba convergence)
    bool enable_adaptive = true;
};

// Main configuration structure
struct Config {
    CameraConfig camera;
    FeatureTrackerConfig feature_tracker;
    EstimatorConfig estimator;
    PnPConfig pnp;

    // Processing parameters
    int frame_skip = 2;  // Skip frames for processing
    int start_frame = 0;  // Start processing from this frame (0-based index)
    int end_frame = -1;   // End processing at this frame (-1 = process all frames)
    std::string dataset_path = "";
    std::string config_filepath = "";

    // Load configuration from YAML file
    bool loadFromYaml(const std::string& yaml_path);

    // Print configuration for debugging
    void print() const;
};

// Global configuration instance
extern Config g_config;

}  // namespace utility

#endif  // UTILITY__CONFIG_H