#ifndef VIO_ENGINE_H
#define VIO_ENGINE_H

#include <memory>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>

#include "backend/estimator.h"
#include "frontend/feature_tracker.h"
#include "frontend/pnp_frontend.h"
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
    // Distortion: model_type (0=KANNALA_BRANDT, 2=PINHOLE), k2, k3, k4, k5
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

#ifndef __EMSCRIPTEN__
    // Native YAML adapter uses the same direct camera and processing contracts.
    bool configureFromConfig(const utility::Config& config, const std::string& camera_file);
#endif

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

    // Set fundamental matrix RANSAC threshold for feature rejection.
    // Higher values accommodate unmodeled lens distortion (mobile phones).
    void setFThreshold(double f_threshold);

    // Set feature tracking parameters for mobile optimization.
    // lk_window: LK optical flow window size (must be odd, default 21)
    // lk_pyramid: LK pyramid levels (default 3)
    // min_dist: minimum distance between features in pixels (default 20)
    // f_edge_factor: edge distortion compensation factor (0=off, 2.0=recommended for mobile)
    void setTrackingParams(int lk_window, int lk_pyramid, int min_dist, double f_edge_factor);

    // Get current VIO status code.
    int getStatusCode() const;
    uint64_t getEpoch() const { return epoch_; }
    double getPoseTimestamp() const { return pose_timestamp_; }
    double getFrameTimestamp() const { return frame_timestamp_; }
    double getIMUEndpointTimestamp() const { return current_time_; }
    const std::string& getLastReason() const { return last_reason_; }
    bool getPoseFresh() const { return has_valid_pose_; }
    bool getPoseValid() const { return has_valid_pose_; }
    int getLastSolverIterations() const;
    std::string getLastSolverTermination() const;
    // Explicit reproducibility profile shared by native and WASM callers.
    void setExecutionParams(int seed, int cv_threads);
    int getExecutionSeed() const { return execution_configured_ ? execution_seed_ : -1; }
    int getCVThreadCount() const;
    void setDiagnosticCapture(bool enabled);
    // Explicit numerical-validation control; persists reset/reconfigure until disabled.
    void setBenchmarkSolverProfile(bool enabled);
    bool getBenchmarkSolverProfile() const { return benchmark_solver_profile_; }
    std::string getFeatureDiagnostics() const;

    // Reset the VIO system to initial state.
    void reset();

    // Enable/disable PnP dual-rate pipeline and configure FREQ decimation.
    // enable_pnp=false reverts to full backend on every frame (current behavior).
    // freq: backend runs every freq frames (default 3).
    void setPnPParams(bool enable_pnp, int freq);

private:
    friend struct VIOEngineTestAccess;
    bool processIMUData(const IMUReading* readings, int count,
                        double image_timestamp);
    void resetState(const std::string& reason);
    void invalidatePose(const std::string& reason);
    bool feedIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro);
    bool publishPose(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation,
                     double timestamp, double* output);

    // Write 4x4 pose matrix to output buffer.
    void writePoseOutput(double* pose_output) const;

    // Match currently tracked features against cached solved features from backend.
    std::vector<common::SolvedFeature> matchFeaturesForPnP() const;

    bool configured_;
    double current_time_;
    double prev_image_timestamp_;
    double last_imu_timestamp_ = -1.0;
    std::deque<IMUReading> pending_imu_;
    size_t imu_samples_since_image_ = 0;
    int configured_width_ = 0;
    int configured_height_ = 0;
    uint64_t epoch_ = 0;
    uint64_t estimator_generation_ = 0;
    double pose_timestamp_ = -1.0;
    double frame_timestamp_ = -1.0;
    std::string last_reason_ = "not_configured";
    int last_solver_iterations_ = 0;
    std::string last_solver_termination_ = "not_run";
    std::string last_solver_quality_reason_;
    bool execution_configured_ = false;
    int execution_seed_ = 0;
    int execution_threads_ = 1;
    bool diagnostic_capture_ = false;
    bool benchmark_solver_profile_ = false;
    void recordSolver(const backend::SolverDiagnostics& diagnostics);
    static constexpr int kMaxIMUReadings = 4096;
    static constexpr double kMaxSensorGapSeconds = 0.5;
    Eigen::Vector3d prev_acc_;
    Eigen::Vector3d prev_gyro_;

    std::unique_ptr<backend::Estimator> estimator_;
    std::unique_ptr<frontend::FeatureTracker> feature_tracker_;

    // PnP dual-rate frontend (VINS-Mobile pattern)
    std::unique_ptr<frontend::PnPFrontend> pnp_frontend_;
    int img_cnt_;                  // Frame counter for FREQ decimation (0..FREQ-1)
    bool pnp_initialized_;         // True after first successful backend solve
    std::vector<common::SolvedFeature> cached_solved_features_;
    std::unordered_map<int, size_t> solved_feature_map_;  // feature_id -> index

    // Store latest pose for retrieval
    Eigen::Vector3d latest_position_;
    Eigen::Matrix3d latest_rotation_;
    bool has_valid_pose_;
    double init_start_time_;
    static constexpr double kInitTimeoutSeconds = 15.0;
};

#endif // VIO_ENGINE_H
