#include "slam/config/slam_config.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace slam {
namespace config {

// ---------------------------------------------------------------------------
// Helpers for parsing key=value config files
// ---------------------------------------------------------------------------
namespace {

/// Trim leading and trailing whitespace from a string.
std::string trim(const std::string& s) {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return "";
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

/// Parse a double from a trimmed string. Returns true on success.
bool parseDouble(const std::string& value, double& out) {
    try {
        out = std::stod(value);
        return true;
    } catch (...) {
        return false;
    }
}

/// Parse an int from a trimmed string. Returns true on success.
bool parseInt(const std::string& value, int& out) {
    try {
        out = std::stoi(value);
        return true;
    } catch (...) {
        return false;
    }
}

/// Parse a bool from a trimmed string ("true"/"1" -> true, else false).
bool parseBool(const std::string& value, bool& out) {
    std::string lower = value;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    if (lower == "true" || lower == "1") {
        out = true;
        return true;
    }
    if (lower == "false" || lower == "0") {
        out = false;
        return true;
    }
    return false;
}

}  // namespace

// ---------------------------------------------------------------------------
// SlamConfig::loadFromFile
// ---------------------------------------------------------------------------
bool SlamConfig::loadFromFile(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "[SlamConfig] Cannot open config file: " << filepath << std::endl;
        return false;
    }

    std::cout << "[SlamConfig] Loading configuration from: " << filepath << std::endl;

    std::string line;
    while (std::getline(file, line)) {
        line = trim(line);

        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') continue;

        // Find the '=' separator
        const auto eq_pos = line.find('=');
        if (eq_pos == std::string::npos) continue;

        std::string key = trim(line.substr(0, eq_pos));
        std::string value = trim(line.substr(eq_pos + 1));

        // -- Camera ---------------------------------------------------------
        if (key == "camera.fx")          { parseDouble(value, camera.fx); }
        else if (key == "camera.fy")     { parseDouble(value, camera.fy); }
        else if (key == "camera.cx")     { parseDouble(value, camera.cx); }
        else if (key == "camera.cy")     { parseDouble(value, camera.cy); }
        else if (key == "camera.k1")     { parseDouble(value, camera.k1); }
        else if (key == "camera.k2")     { parseDouble(value, camera.k2); }
        else if (key == "camera.p1")     { parseDouble(value, camera.p1); }
        else if (key == "camera.p2")     { parseDouble(value, camera.p2); }
        else if (key == "camera.width")  { parseInt(value, camera.width); }
        else if (key == "camera.height") { parseInt(value, camera.height); }

        // Extrinsic rotation (row-major, 9 comma-separated doubles)
        else if (key == "camera.r_ic") {
            std::istringstream ss(value);
            std::string token;
            std::vector<double> vals;
            while (std::getline(ss, token, ',')) {
                double v = 0.0;
                if (parseDouble(trim(token), v)) vals.push_back(v);
            }
            if (vals.size() == 9) {
                camera.r_ic << vals[0], vals[1], vals[2],
                               vals[3], vals[4], vals[5],
                               vals[6], vals[7], vals[8];
            }
        }

        // Extrinsic translation (3 comma-separated doubles)
        else if (key == "camera.t_ic") {
            std::istringstream ss(value);
            std::string token;
            std::vector<double> vals;
            while (std::getline(ss, token, ',')) {
                double v = 0.0;
                if (parseDouble(trim(token), v)) vals.push_back(v);
            }
            if (vals.size() == 3) {
                camera.t_ic << vals[0], vals[1], vals[2];
            }
        }

        // -- Feature tracker ------------------------------------------------
        else if (key == "feature_tracker.max_cnt")      { parseInt(value, feature_tracker.max_cnt); }
        else if (key == "feature_tracker.min_dist")      { parseInt(value, feature_tracker.min_dist); }
        else if (key == "feature_tracker.f_threshold")   { parseDouble(value, feature_tracker.f_threshold); }
        else if (key == "feature_tracker.equalize")      { parseBool(value, feature_tracker.equalize); }

        // -- Estimator ------------------------------------------------------
        else if (key == "estimator.window_size")     { parseInt(value, estimator.window_size); }
        else if (key == "estimator.num_iterations")  { parseInt(value, estimator.num_iterations); }
        else if (key == "estimator.solver_time")     { parseDouble(value, estimator.solver_time); }
        else if (key == "estimator.min_parallax")    { parseDouble(value, estimator.min_parallax); }
        else if (key == "estimator.init_depth")      { parseDouble(value, estimator.init_depth); }
        else if (key == "estimator.acc_n")           { parseDouble(value, estimator.acc_n); }
        else if (key == "estimator.gyr_n")           { parseDouble(value, estimator.gyr_n); }
        else if (key == "estimator.acc_w")           { parseDouble(value, estimator.acc_w); }
        else if (key == "estimator.gyr_w")           { parseDouble(value, estimator.gyr_w); }
        else if (key == "estimator.gravity_norm") {
            double g = 9.81007;
            if (parseDouble(value, g)) {
                estimator.gravity = Eigen::Vector3d(0.0, 0.0, g);
            }
        }

        // -- Dataset --------------------------------------------------------
        else if (key == "dataset_path") { dataset_path = value; }

        else {
            std::cerr << "[SlamConfig] Unknown key: " << key << std::endl;
        }
    }

    file.close();
    std::cout << "[SlamConfig] Configuration loaded successfully." << std::endl;
    return true;
}

// ---------------------------------------------------------------------------
// SlamConfig::DefaultTumVi  --  TUM VI dataset-room1_512_16
// ---------------------------------------------------------------------------
SlamConfig SlamConfig::DefaultTumVi() {
    SlamConfig cfg;

    // ------------------------------------------------------------------
    // Camera intrinsics from camchain.yaml  (cam0, pinhole + equidistant)
    // ------------------------------------------------------------------
    cfg.camera.fx = 190.97847715128717;
    cfg.camera.fy = 190.9733070521226;
    cfg.camera.cx = 254.93170605935475;
    cfg.camera.cy = 256.8974428996504;
    cfg.camera.k1 = 0.0034823894022493434;
    cfg.camera.k2 = 0.0007150348452162257;
    cfg.camera.p1 = -0.0020532361418706202;
    cfg.camera.p2 = 0.00020293673591811182;
    cfg.camera.width = 512;
    cfg.camera.height = 512;

    // ------------------------------------------------------------------
    // Extrinsic: T_cam_imu from camchain.yaml
    //   T_cam_imu = [R_ci | t_ci]   (transforms points from IMU to camera)
    //   We need R_ic = R_ci^T  and  t_ic = -R_ci^T * t_ci
    //   so that p_cam = R_ci * p_imu + t_ci
    //        => p_imu = R_ci^T * p_cam - R_ci^T * t_ci
    // ------------------------------------------------------------------
    Eigen::Matrix3d r_ci;
    r_ci << -0.9995250378696743,  0.029615343885863205, -0.008522328211654736,
             0.0075019185074052044, -0.03439736061393144, -0.9993800792498829,
            -0.02989013031643309,  -0.998969345370175,    0.03415885127385616;

    Eigen::Vector3d t_ci;
    t_ci << 0.04727988224914392, -0.047443232143367084, -0.0681999605066297;

    cfg.camera.r_ic = r_ci.transpose();
    cfg.camera.t_ic = -r_ci.transpose() * t_ci;

    // ------------------------------------------------------------------
    // Feature tracker defaults (good for 512x512)
    // ------------------------------------------------------------------
    cfg.feature_tracker.max_cnt = 150;
    cfg.feature_tracker.min_dist = 25;
    cfg.feature_tracker.f_threshold = 1.0;
    cfg.feature_tracker.equalize = true;

    // ------------------------------------------------------------------
    // Estimator / IMU noise from imu_config.yaml (inflated values)
    // ------------------------------------------------------------------
    cfg.estimator.window_size = 10;
    cfg.estimator.num_iterations = 10;
    cfg.estimator.solver_time = 0.1;
    cfg.estimator.min_parallax = 10.0;
    cfg.estimator.init_depth = 5.0;
    cfg.estimator.acc_n = 0.0028;     // accelerometer_noise_density
    cfg.estimator.gyr_n = 0.00016;    // gyroscope_noise_density
    cfg.estimator.acc_w = 0.00086;    // accelerometer_random_walk
    cfg.estimator.gyr_w = 0.000022;   // gyroscope_random_walk
    cfg.estimator.gravity = Eigen::Vector3d(0.0, 0.0, 9.81007);

    return cfg;
}

// ---------------------------------------------------------------------------
// SlamConfig::DefaultWebcam
// ---------------------------------------------------------------------------
SlamConfig SlamConfig::DefaultWebcam(double fx, double fy, double cx, double cy) {
    SlamConfig cfg;

    cfg.camera.fx = fx;
    cfg.camera.fy = fy;
    cfg.camera.cx = cx;
    cfg.camera.cy = cy;
    cfg.camera.k1 = 0.0;
    cfg.camera.k2 = 0.0;
    cfg.camera.p1 = 0.0;
    cfg.camera.p2 = 0.0;
    cfg.camera.width = static_cast<int>(cx * 2.0);
    cfg.camera.height = static_cast<int>(cy * 2.0);
    cfg.camera.r_ic = Eigen::Matrix3d::Identity();
    cfg.camera.t_ic = Eigen::Vector3d::Zero();

    cfg.feature_tracker.max_cnt = 150;
    cfg.feature_tracker.min_dist = 30;
    cfg.feature_tracker.f_threshold = 1.0;
    cfg.feature_tracker.equalize = true;

    cfg.estimator.window_size = 10;
    cfg.estimator.num_iterations = 10;
    cfg.estimator.solver_time = 0.1;
    cfg.estimator.min_parallax = 10.0;
    cfg.estimator.init_depth = 5.0;
    cfg.estimator.acc_n = 0.08;
    cfg.estimator.gyr_n = 0.004;
    cfg.estimator.acc_w = 0.00004;
    cfg.estimator.gyr_w = 2.0e-6;
    cfg.estimator.gravity = Eigen::Vector3d(0.0, 0.0, 9.81007);

    return cfg;
}

// ---------------------------------------------------------------------------
// SlamConfig::print
// ---------------------------------------------------------------------------
void SlamConfig::print() const {
    std::cout << "=== SlamConfig ===" << std::endl;

    std::cout << "Dataset:" << std::endl;
    std::cout << "  path: " << dataset_path << std::endl;

    std::cout << "Camera:" << std::endl;
    std::cout << "  resolution: " << camera.width << "x" << camera.height << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  fx: " << camera.fx << std::endl;
    std::cout << "  fy: " << camera.fy << std::endl;
    std::cout << "  cx: " << camera.cx << std::endl;
    std::cout << "  cy: " << camera.cy << std::endl;
    std::cout << "  distortion: [" << camera.k1 << ", " << camera.k2
              << ", " << camera.p1 << ", " << camera.p2 << "]" << std::endl;
    std::cout << "  t_ic: [" << camera.t_ic.transpose() << "]" << std::endl;

    std::cout << "Feature Tracker:" << std::endl;
    std::cout << "  max_cnt: " << feature_tracker.max_cnt << std::endl;
    std::cout << "  min_dist: " << feature_tracker.min_dist << std::endl;
    std::cout << "  f_threshold: " << feature_tracker.f_threshold << std::endl;
    std::cout << "  equalize: " << (feature_tracker.equalize ? "true" : "false") << std::endl;

    std::cout << "Estimator:" << std::endl;
    std::cout << "  window_size: " << estimator.window_size << std::endl;
    std::cout << "  num_iterations: " << estimator.num_iterations << std::endl;
    std::cout << "  solver_time: " << estimator.solver_time << std::endl;
    std::cout << "  min_parallax: " << estimator.min_parallax << std::endl;
    std::cout << "  init_depth: " << estimator.init_depth << std::endl;
    std::cout << std::scientific << std::setprecision(6);
    std::cout << "  acc_n: " << estimator.acc_n << std::endl;
    std::cout << "  gyr_n: " << estimator.gyr_n << std::endl;
    std::cout << "  acc_w: " << estimator.acc_w << std::endl;
    std::cout << "  gyr_w: " << estimator.gyr_w << std::endl;
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "  gravity: [" << estimator.gravity.transpose() << "]" << std::endl;

    std::cout << "==================" << std::endl;
}

}  // namespace config
}  // namespace slam
