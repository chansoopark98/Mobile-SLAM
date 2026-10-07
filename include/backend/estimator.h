#ifndef BACKEND__ESTIMATOR_H
#define BACKEND__ESTIMATOR_H

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <cstdint>
#ifndef __EMSCRIPTEN__
#include <mutex>
#endif

#include "backend/optimizer.h"
#include "backend/sliding_window.h"
#include "common/common_types.h"
#include "common/image_frame.h"
#include "frontend/failure_detector.h"
#include "frontend/feature_manager.h"
#include "frontend/initialization/initializer.h"

namespace backend {

class Estimator {
public:
    Estimator();

    void setParameter();

    void processIMU(double dt, const Eigen::Vector3d& linear_acceleration, const Eigen::Vector3d& angular_velocity);
    void processImage(const common::ImageData& image, double timestamp);
    void reset();
    bool hasUsableLatestImageUpdate() const { return latest_image_update_usable_; }
    uint64_t getResetGeneration() const { return reset_generation_; }
    const SolverDiagnostics& getLastSolverDiagnostics() const { return last_solver_diagnostics_; }
    void setDiagnosticCapture(bool enabled) { optimizer_.setDiagnosticCapture(enabled); }
    const std::string& getBackendDiagnostics() const { return optimizer_.getBackendDiagnostics(); }
    void setBenchmarkSolverProfile(bool enabled) { optimizer_.setBenchmarkSolverProfile(enabled); }
    bool getBenchmarkSolverProfile() const { return optimizer_.getBenchmarkSolverProfile(); }

    Eigen::Matrix3d r_ic_;
    Eigen::Vector3d t_ic_;

    SlidingWindow sliding_window_;

    common::SolverFlag solver_flag_;
    common::MarginalizationFlag marginalization_flag_;

    std::vector<Eigen::Vector3d> getSlidingWindowMapPoints() const;
    /** Log triangulation diagnostics: solve_flag counts, depth stats, velocity */
    void logTriangulationDiag(int frame_num) const;

    // PnP Frontend data extraction (VINS-Mobile pattern)
    /** Extract well-triangulated features as SolvedFeature list for PnP frontend */
    std::vector<common::SolvedFeature> getSolvedFeatures() const;
    /** Get latest backend solution as VINSResult for PnP frontend initialization */
    common::VINSResult getLatestVINSResult() const;

private:
    void clearState();

    void slideWindow();
    void slideWindowNewGeneralFrame();
    void slideWindowOldKeyframe();

    void solveOdometry();

    void cleanupOldImageFrames(double timestamp);
    void cleanupPreIntegration(common::ImageFrame& frame);

    void propagateIMUState(int frame_index, double dt, const Eigen::Vector3d& linear_acceleration,
                           const Eigen::Vector3d& angular_velocity);
    void storeLastPoseInSlidingWindow();

    bool first_imu_;
    bool failure_occur_;

    double initial_timestamp_;
    Eigen::Vector3d prev_acc_, prev_gyro_;
    int frame_count_;
    bool latest_image_update_usable_ = false;
    uint64_t reset_generation_ = 0;
    SolverDiagnostics last_solver_diagnostics_;

    Eigen::Vector3d g_;

    std::unique_ptr<backend::factor::IntegrationBase> tmp_pre_integration_;

    std::map<double, common::ImageFrame> all_image_frame_;

    Eigen::Matrix3d last_R_end_;
    Eigen::Vector3d last_P_end_;

    // Core components
    Optimizer optimizer_;
    frontend::FailureDetector failure_detector_;
    frontend::initialization::Initializer initializer_;
    frontend::FeatureManager feature_manager_;
    frontend::initialization::MotionEstimator motion_estimator_;

#ifndef __EMSCRIPTEN__
    mutable std::mutex estimator_mutex_;
#endif
};

}  // namespace backend

#endif  // BACKEND__ESTIMATOR_H
