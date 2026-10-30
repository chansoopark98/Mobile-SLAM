#ifndef FRONTEND__PNP_FRONTEND_H
#define FRONTEND__PNP_FRONTEND_H

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "backend/factor/integration_base.h"
#include "backend/solver_diagnostics.h"
#include "common/common_types.h"
#include "utility/config.h"

namespace frontend {

// PnP Frontend: 30Hz motion-only tracking using known 3D points from backend.
// VINS-Mobile pattern: lightweight 6-frame sliding window with fixed bias.
//
// Usage:
//   1. Backend provides solved features + VINSResult after each optimization
//   2. Every frame: processIMU() with IMU readings, then processImage() with matched features
//   3. After PNP_SIZE frames accumulated, solveCeres() runs lightweight optimization
//   4. Pose available via getPosition()/getRotation()
//
// Reference: assets/references/VINS-Mobile/VINS_ios/vins_pnp.{hpp,cpp}
class PnPFrontend {
public:
    PnPFrontend();

    void setExtrinsicParameters(const Eigen::Matrix3d& ric, const Eigen::Vector3d& tic);
    void setIMUModel();

    // Initialize all PnP state from backend result (call once at creation).
    // Sets P, R, V, Ba, Bg for ALL window slots so IMU propagation starts correctly.
    void initializeState(const common::VINSResult& result);

    // Receive backend solution — updates bias, marks solved frames
    void setBackendSolution(const common::VINSResult& result);

    // IMU propagation (midpoint integration, same as backend)
    void processIMU(double dt, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro);

    // Process one frame with matched features. Returns true when pose is available.
    bool processImage(const std::vector<common::SolvedFeature>& features, double timestamp);

    // Pose output
    Eigen::Vector3d getPosition() const;
    Eigen::Matrix3d getRotation() const;
    Eigen::Vector3d getVelocity() const;
    bool hasPose() const;
    bool hasUsableLatestImageUpdate() const { return latest_image_update_usable_; }
    const backend::SolverDiagnostics& getLastSolverDiagnostics() const { return diagnostics_; }

    void clearState();

private:
    bool solveCeres();
    void slideWindow();
    void old2new();
    void new2old();

    static constexpr int kPnPSize = 6;

    // State arrays
    Eigen::Vector3d Ps_[kPnPSize + 1];
    Eigen::Matrix3d Rs_[kPnPSize + 1];
    Eigen::Vector3d Vs_[kPnPSize + 1];
    Eigen::Vector3d Bas_[kPnPSize + 1];
    Eigen::Vector3d Bgs_[kPnPSize + 1];

    // Ceres parameter arrays
    double para_Pose_[kPnPSize + 1][7];
    double para_Speed_[kPnPSize + 1][3];
    double para_Bias_[kPnPSize + 1][6];
    double para_Ex_Pose_[7];

    // Pre-integration per frame (value-initialized to nullptr for safe clearState)
    std::unique_ptr<backend::factor::IntegrationBase> pre_integrations_[kPnPSize + 1];

    // Per-frame feature observations
    std::vector<common::SolvedFeature> features_[kPnPSize + 1];

    // Backend solution matching
    bool find_solved_[kPnPSize + 1];
    double headers_[kPnPSize + 1];

    // IMU state
    bool first_imu_;
    Eigen::Vector3d acc_0_, gyr_0_;
    Eigen::Vector3d g_;
    int frame_count_;
    bool has_pose_;
    bool latest_image_update_usable_ = false;
    bool backend_initialized_ = false;
    double backend_anchor_timestamp_ = 0;
    backend::SolverDiagnostics diagnostics_;

    // Extrinsics
    Eigen::Matrix3d ric_;
    Eigen::Vector3d tic_;

    // IMU buffers
    std::vector<double> dt_buf_[kPnPSize + 1];
    std::vector<Eigen::Vector3d> acc_buf_[kPnPSize + 1];
    std::vector<Eigen::Vector3d> gyr_buf_[kPnPSize + 1];
};

}  // namespace frontend

#endif  // FRONTEND__PNP_FRONTEND_H
