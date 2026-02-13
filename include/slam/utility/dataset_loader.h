#ifndef SLAM__UTILITY__DATASET_LOADER_H
#define SLAM__UTILITY__DATASET_LOADER_H

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <string>
#include <utility>
#include <vector>

namespace slam {
namespace utility {

/// Single IMU measurement (gyroscope + accelerometer).
struct ImuMeasurement {
    double timestamp;  // seconds
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d linear_acceleration;
};

/// Ground-truth pose from motion capture.
struct GroundTruthPose {
    double timestamp;  // seconds
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

/// Loader for the TUM VI dataset in ASL format.
///
/// Expected directory layout under dataset_path:
///   mav0/cam0/data.csv          -- image timestamps + filenames
///   mav0/cam0/data/             -- image files (*.png)
///   mav0/imu0/data.csv          -- IMU measurements
///   mav0/mocap0/data.csv        -- ground-truth poses
class TumViLoader {
public:
    /// Construct a loader pointing at the dataset root.
    explicit TumViLoader(const std::string& dataset_path);

    /// Load all CSV files (images, IMU, ground truth).
    /// Returns true if at least images and IMU were loaded successfully.
    bool load();

    // -- Accessors --------------------------------------------------------

    /// Image timestamps (seconds) paired with filenames.
    const std::vector<std::pair<double, std::string>>& image_timestamps() const;

    /// All IMU measurements sorted by timestamp.
    const std::vector<ImuMeasurement>& imu_measurements() const;

    /// Ground-truth poses sorted by timestamp.
    const std::vector<GroundTruthPose>& ground_truth() const;

    // -- Utilities --------------------------------------------------------

    /// Load a single image by index (returns 512x512 grayscale cv::Mat).
    cv::Mat loadImage(size_t index) const;

    /// Return all IMU measurements with timestamps in the open interval (t0, t1].
    std::vector<ImuMeasurement> getImuBetween(double t0, double t1) const;

private:
    std::string dataset_path_;
    std::vector<std::pair<double, std::string>> image_timestamps_;  // (timestamp_sec, filename)
    std::vector<ImuMeasurement> imu_measurements_;
    std::vector<GroundTruthPose> ground_truth_;

    bool loadImages();
    bool loadImu();
    bool loadGroundTruth();

    /// Convert nanosecond integer timestamp to seconds.
    double nsToSec(int64_t ns) const { return static_cast<double>(ns) * 1e-9; }
};

}  // namespace utility
}  // namespace slam

#endif  // SLAM__UTILITY__DATASET_LOADER_H
