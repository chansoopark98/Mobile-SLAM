#ifndef VIO_ENGINE_H
#define VIO_ENGINE_H

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "backend/estimator.h"
#include "frontend/feature_tracker.h"
#include "utility/config.h"

// Headless VIO engine for WASM and programmatic use.
// No visualization, no file I/O, no threading.
// Accepts raw image data + IMU data arrays directly.

enum class VIOStatus : int {
    NOT_CONFIGURED = 0,
    INITIALIZING = 1,
    TRACKING = 2,
    LOST = 3,
    COOLDOWN = 4
};

struct IMUReading {
    double timestamp;
    double acc_x, acc_y, acc_z;
    double gyro_x, gyro_y, gyro_z;
};
static_assert(sizeof(IMUReading) == 7 * sizeof(double),
              "IMUReading must be 7 contiguous doubles (JS/WASM interop)");

class VIOEngine {
public:
    VIOEngine();
    ~VIOEngine();

    // Configure camera and IMU parameters directly (no YAML file needed).
    // Camera: width, height, fx, fy, cx, cy
    // Distortion: model_type (0=PINHOLE, 1=KANNALA_BRANDT), k2, k3, k4, k5
    // Extrinsics: r_ic (9 doubles, row-major), t_ic (3 doubles)
    // IMU noise: acc_n, acc_w, gyr_n, gyr_w, g_norm
    bool configure(int width, int height,
                   double fx, double fy, double cx, double cy,
                   int model_type,
                   double k2, double k3, double k4, double k5,
                   const double* r_ic, const double* t_ic,
                   double acc_n, double acc_w,
                   double gyr_n, double gyr_w,
                   double g_norm);

    // Process one camera frame with associated IMU readings.
    // gray_image: pointer to grayscale image data (width * height bytes)
    // imu_readings: array of IMU readings since last frame
    // imu_count: number of IMU readings
    // pose_output: pointer to 16 doubles (4x4 row-major transformation matrix)
    // Returns true if pose was computed (VIO initialized and running).
    bool processFrame(const uint8_t* gray_image, int width, int height,
                      const IMUReading* imu_readings, int imu_count,
                      double image_timestamp,
                      double* pose_output);

    // Check if VIO has initialized (solver in NON_LINEAR mode).
    bool isInitialized() const;

    // Get number of currently tracked feature points.
    int getFeaturePointCount() const;

    // Get 3D positions of tracked map points.
    // output: pointer to 3*count doubles (x,y,z for each point)
    // Returns actual number of points written.
    int getMapPoints(double* output, int max_count) const;

    // Set mobile-optimized solver parameters (call after configure).
    void setMobileParams(double solver_time, int num_iterations, int max_features);

    // Get current VIO status code.
    int getStatusCode() const;

    // Reset the VIO system to initial state.
    void reset();

private:
    void processIMUData(const IMUReading* readings, int count,
                        double image_timestamp);

    bool configured_;
    double current_time_;
    double prev_image_timestamp_;
    Eigen::Vector3d prev_acc_;
    Eigen::Vector3d prev_gyro_;
    bool first_imu_;

    std::unique_ptr<backend::Estimator> estimator_;
    std::unique_ptr<frontend::FeatureTracker> feature_tracker_;

    // Store latest pose for retrieval
    Eigen::Vector3d latest_position_;
    Eigen::Matrix3d latest_rotation_;
    bool has_valid_pose_;
    int consecutive_failures_;
    static constexpr int kMaxConsecutiveFailures = 5;
    static constexpr int kCooldownFrames = 30;
    int cooldown_counter_;
};

#endif // VIO_ENGINE_H
