#ifndef BACKEND__OPTIMIZER_H
#define BACKEND__OPTIMIZER_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <iostream>

#include "backend/factor/imu_factor.h"
#include "backend/factor/integration_base.h"
#include "backend/factor/marginalization_factor.h"
#include "backend/factor/pose_local_parameterization.h"
#include "backend/factor/projection_factor.h"
#include "backend/sliding_window.h"
#include "backend/solver_diagnostics.h"
#include "common/common_types.h"
#include "frontend/feature_manager.h"
#include "utility/config.h"

namespace backend {

class Optimizer {
public:
    Optimizer(SlidingWindow* sliding_window, frontend::FeatureManager* feature_manager);
    ~Optimizer();

    // Main optimization interface
    bool optimize(common::MarginalizationFlag marginalization_flag);
    void reset();
    const SolverDiagnostics& getLastSolverDiagnostics() const { return diagnostics_; }
    bool hasMarginalizationPrior() const { return last_marginalization_info_ != nullptr; }
    size_t getPriorParameterBlockCount() const { return last_marginalization_parameter_blocks_.size(); }
    void setDiagnosticCapture(bool enabled);
    const std::string& getBackendDiagnostics() const { return diagnostic_json_; }
    void setBenchmarkSolverProfile(bool enabled) { benchmark_solver_profile_ = enabled; }
    bool getBenchmarkSolverProfile() const { return benchmark_solver_profile_; }

    // Setter for extrinsic parameters
    void setExtrinsicParameters(const Vector3d& t_ic, const Matrix3d& r_ic);

    // Getter for extrinsic parameters
    Vector3d getTic() const {
        return t_ic_;
    }
    Matrix3d getRic() const {
        return r_ic_;
    }

private:
    // Optimization setup
    void setupOptimizationProblem(ceres::Problem& problem);
    void addMarginalizationFactor(ceres::Problem& problem);
    void addIMUFactors(ceres::Problem& problem);
    int addFeatureFactors(ceres::Problem& problem);
    // pending_frames: number of frames waiting to be processed (for adaptive solver time).
    // In WASM single-thread, always 0. In future multi-threaded mode, queue depth.
    void solveCeresProblem(ceres::Problem& problem, int pending_frames = 0);

    // Marginalization methods
    void marginalizeOldKeyframe();
    void marginalizeNewGeneralFrame();
    void addFeatureFactorsForMarginalization(factor::MarginalizationInfo* marginalization_info);
    void addIMUFactorForMarginalization(factor::MarginalizationInfo* marginalization_info);
    void performMarginalizationForOldKeyframe(factor::MarginalizationInfo* marginalization_info);
    void performMarginalizationForNewGeneralFrame(factor::MarginalizationInfo* marginalization_info);
    void handleMarginalization(common::MarginalizationFlag marginalization_flag);

    // Parameter management
    void prepareOptimizationParameters();
    void applyOptimizationResults();
    bool validateOptimizationParameters() const;
    void releaseMarginalizationPrior();
    void beginDiagnostics(common::MarginalizationFlag marginalization_flag);
    void finishDiagnostics();
    void recordMarginalizationDiagnostics(factor::MarginalizationInfo* info);
    std::string diagnosticBlockRole(const double* address) const;

    // Member variables
    SlidingWindow* sliding_window_;
    frontend::FeatureManager* feature_manager_;

    // Extrinsic parameters
    Vector3d t_ic_;
    Matrix3d r_ic_;

    // Optimization parameters
    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedAndBiases[WINDOW_SIZE + 1][SIZE_SPEEDANDBIAS];
    double para_Feature[NUM_OF_FEATURES][SIZE_FEATURE];
    double para_Ex_Pose[SIZE_POSE];

    // Marginalization info
    factor::MarginalizationInfo* last_marginalization_info_;
    std::vector<double*> last_marginalization_parameter_blocks_;
    std::unique_ptr<ceres::LossFunction> marginalization_loss_;
    SolverDiagnostics diagnostics_;
    bool benchmark_solver_profile_ = false;
    bool diagnostic_capture_ = false;
    double diagnostic_timestamp_ = -1;
    common::MarginalizationFlag diagnostic_margin_ = common::MarginalizationFlag::MARGIN_OLD_KEYFRAME;
    std::string diagnostic_json_ = "{\"enabled\":false}";
    std::string incoming_prior_json_, visual_membership_json_, gauge_json_, outgoing_prior_json_;
    std::string previous_marginalization_json_ = "{\"captured\":false}";
};

}  // namespace backend

#endif  // BACKEND__OPTIMIZER_H
