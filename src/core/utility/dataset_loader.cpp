#include "slam/utility/dataset_loader.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <opencv2/imgcodecs.hpp>

namespace slam {
namespace utility {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
namespace {

/// Trim leading and trailing whitespace.
std::string trim(const std::string& s) {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return "";
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

/// Return true if the line is a comment (starts with '#') or empty.
bool isCommentOrEmpty(const std::string& line) {
    const std::string trimmed = trim(line);
    return trimmed.empty() || trimmed[0] == '#';
}

}  // namespace

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------
TumViLoader::TumViLoader(const std::string& dataset_path)
    : dataset_path_(dataset_path) {}

// ---------------------------------------------------------------------------
// load -- orchestrate all sub-loaders
// ---------------------------------------------------------------------------
bool TumViLoader::load() {
    std::cout << "[TumViLoader] Loading dataset from: " << dataset_path_ << std::endl;

    bool images_ok = loadImages();
    bool imu_ok = loadImu();
    bool gt_ok = loadGroundTruth();

    if (!images_ok) {
        std::cerr << "[TumViLoader] Failed to load image timestamps." << std::endl;
        return false;
    }
    if (!imu_ok) {
        std::cerr << "[TumViLoader] Failed to load IMU data." << std::endl;
        return false;
    }
    if (!gt_ok) {
        std::cerr << "[TumViLoader] Warning: ground truth not loaded "
                  << "(may not be available)." << std::endl;
        // Ground truth is optional -- do not return false.
    }

    std::cout << "[TumViLoader] Loaded " << image_timestamps_.size() << " images, "
              << imu_measurements_.size() << " IMU measurements, "
              << ground_truth_.size() << " ground-truth poses." << std::endl;

    if (!image_timestamps_.empty()) {
        std::cout << "[TumViLoader] Image time range: "
                  << std::fixed << std::setprecision(6)
                  << image_timestamps_.front().first << " -- "
                  << image_timestamps_.back().first << " s" << std::endl;
    }
    if (!imu_measurements_.empty()) {
        std::cout << "[TumViLoader] IMU time range:   "
                  << std::fixed << std::setprecision(6)
                  << imu_measurements_.front().timestamp << " -- "
                  << imu_measurements_.back().timestamp << " s" << std::endl;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------
const std::vector<std::pair<double, std::string>>& TumViLoader::image_timestamps() const {
    return image_timestamps_;
}

const std::vector<ImuMeasurement>& TumViLoader::imu_measurements() const {
    return imu_measurements_;
}

const std::vector<GroundTruthPose>& TumViLoader::ground_truth() const {
    return ground_truth_;
}

// ---------------------------------------------------------------------------
// loadImage -- read a single frame by index
// ---------------------------------------------------------------------------
cv::Mat TumViLoader::loadImage(size_t index) const {
    if (index >= image_timestamps_.size()) {
        std::cerr << "[TumViLoader] Image index " << index << " out of range ("
                  << image_timestamps_.size() << " images)." << std::endl;
        return {};
    }

    const std::string& filename = image_timestamps_[index].second;
    const std::string full_path = dataset_path_ + "/mav0/cam0/data/" + filename;
    cv::Mat image = cv::imread(full_path, cv::IMREAD_GRAYSCALE);

    if (image.empty()) {
        std::cerr << "[TumViLoader] Failed to load image: " << full_path << std::endl;
    }
    return image;
}

// ---------------------------------------------------------------------------
// getImuBetween -- return measurements in (t0, t1]
// ---------------------------------------------------------------------------
std::vector<ImuMeasurement> TumViLoader::getImuBetween(double t0, double t1) const {
    std::vector<ImuMeasurement> result;

    // Use lower_bound on a sorted sequence for efficient lookup.
    auto it = std::lower_bound(
        imu_measurements_.begin(), imu_measurements_.end(), t0,
        [](const ImuMeasurement& m, double t) { return m.timestamp <= t; });

    for (; it != imu_measurements_.end() && it->timestamp <= t1; ++it) {
        result.push_back(*it);
    }

    return result;
}

// ---------------------------------------------------------------------------
// loadImages -- parse mav0/cam0/data.csv
// Format:  #timestamp [ns],filename
// ---------------------------------------------------------------------------
bool TumViLoader::loadImages() {
    const std::string csv_path = dataset_path_ + "/mav0/cam0/data.csv";
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        std::cerr << "[TumViLoader] Cannot open image CSV: " << csv_path << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (isCommentOrEmpty(line)) continue;

        std::istringstream ss(line);
        std::string ts_str, filename;

        if (!std::getline(ss, ts_str, ',')) continue;
        if (!std::getline(ss, filename))    continue;

        ts_str = trim(ts_str);
        filename = trim(filename);
        if (ts_str.empty() || filename.empty()) continue;

        int64_t ts_ns = 0;
        try {
            ts_ns = std::stoll(ts_str);
        } catch (...) {
            continue;
        }

        image_timestamps_.emplace_back(nsToSec(ts_ns), filename);
    }

    file.close();
    return !image_timestamps_.empty();
}

// ---------------------------------------------------------------------------
// loadImu -- parse mav0/imu0/data.csv
// Format:  #timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z
// ---------------------------------------------------------------------------
bool TumViLoader::loadImu() {
    const std::string csv_path = dataset_path_ + "/mav0/imu0/data.csv";
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        std::cerr << "[TumViLoader] Cannot open IMU CSV: " << csv_path << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (isCommentOrEmpty(line)) continue;

        std::istringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(ss, token, ',')) {
            tokens.push_back(trim(token));
        }

        // Expect exactly 7 fields: timestamp, wx, wy, wz, ax, ay, az
        if (tokens.size() < 7) continue;

        ImuMeasurement meas;
        try {
            int64_t ts_ns = std::stoll(tokens[0]);
            meas.timestamp = nsToSec(ts_ns);
            meas.angular_velocity.x() = std::stod(tokens[1]);
            meas.angular_velocity.y() = std::stod(tokens[2]);
            meas.angular_velocity.z() = std::stod(tokens[3]);
            meas.linear_acceleration.x() = std::stod(tokens[4]);
            meas.linear_acceleration.y() = std::stod(tokens[5]);
            meas.linear_acceleration.z() = std::stod(tokens[6]);
        } catch (...) {
            continue;
        }

        imu_measurements_.push_back(meas);
    }

    file.close();
    return !imu_measurements_.empty();
}

// ---------------------------------------------------------------------------
// loadGroundTruth -- parse mav0/mocap0/data.csv
// Format:  #timestamp [ns],p_x,p_y,p_z,q_w,q_x,q_y,q_z
// ---------------------------------------------------------------------------
bool TumViLoader::loadGroundTruth() {
    const std::string csv_path = dataset_path_ + "/mav0/mocap0/data.csv";
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        std::cerr << "[TumViLoader] Cannot open ground-truth CSV: " << csv_path << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (isCommentOrEmpty(line)) continue;

        std::istringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(ss, token, ',')) {
            tokens.push_back(trim(token));
        }

        // Expect exactly 8 fields: timestamp, px, py, pz, qw, qx, qy, qz
        if (tokens.size() < 8) continue;

        GroundTruthPose pose;
        try {
            int64_t ts_ns = std::stoll(tokens[0]);
            pose.timestamp = nsToSec(ts_ns);
            pose.position.x() = std::stod(tokens[1]);
            pose.position.y() = std::stod(tokens[2]);
            pose.position.z() = std::stod(tokens[3]);
            // Quaternion: q_w, q_x, q_y, q_z
            double qw = std::stod(tokens[4]);
            double qx = std::stod(tokens[5]);
            double qy = std::stod(tokens[6]);
            double qz = std::stod(tokens[7]);
            pose.orientation = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
        } catch (...) {
            continue;
        }

        ground_truth_.push_back(pose);
    }

    file.close();
    return !ground_truth_.empty();
}

}  // namespace utility
}  // namespace slam
