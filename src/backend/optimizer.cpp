#include "backend/optimizer.h"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace backend {

namespace {
void diagnosticNumber(std::ostream& out, double value) {
    if (std::isfinite(value)) out << std::setprecision(17) << value;
    else out << "null";
}
void diagnosticString(std::ostream& out, const std::string& value, size_t limit = 4096) {
    out << '"';
    for (unsigned char c : value.substr(0, limit)) {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 32) out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << int(c) << std::dec;
        else out << c;
    }
    out << '"';
}
}  // namespace

void Optimizer::setDiagnosticCapture(bool enabled) {
    diagnostic_capture_ = enabled;
    diagnostic_json_ = "{\"enabled\":false}";
    incoming_prior_json_.clear(); visual_membership_json_.clear(); gauge_json_.clear(); outgoing_prior_json_.clear();
    previous_marginalization_json_ = "{\"captured\":false}";
    diagnostics_.numericalDetails = "null";
    if (last_marginalization_info_) last_marginalization_info_->setDiagnosticCapture(enabled);
}

std::string Optimizer::diagnosticBlockRole(const double* address) const {
    if (address == para_Ex_Pose) return "extrinsic";
    for (int i = 0; i <= WINDOW_SIZE; ++i) {
        if (address == para_Pose[i]) return "pose_" + std::to_string(i);
        if (address == para_SpeedAndBiases[i]) return "speed_bias_" + std::to_string(i);
    }
    for (int i = 0; i < NUM_OF_FEATURES; ++i)
        if (address == para_Feature[i]) return "inverse_depth_" + std::to_string(i);
    return "unknown";
}

void Optimizer::beginDiagnostics(common::MarginalizationFlag margin) {
    if (!diagnostic_capture_) return;
    diagnostic_timestamp_ = sliding_window_->back().timestamp;
    diagnostic_margin_ = margin;
    visual_membership_json_ = "null"; gauge_json_ = "null"; outgoing_prior_json_ = "{\"updated\":false}";
    std::ostringstream out;
    out << "{\"stage\":\"before_solve\",\"available\":" << (last_marginalization_info_ ? "true" : "false")
        << ",\"cachedPreviousOperation\":" << previous_marginalization_json_;
    if (last_marginalization_info_) {
        out << ",\"semanticLayout\":[";
        const auto& info = *last_marginalization_info_;
        const size_t count = std::min(last_marginalization_parameter_blocks_.size(), size_t(1000));
        for (size_t i = 0; i < count; ++i) {
            if (i) out << ',';
            out << "{\"role\":"; diagnosticString(out, diagnosticBlockRole(last_marginalization_parameter_blocks_[i]));
            out << ",\"globalSize\":" << info.keep_block_size[i] << ",\"localColumn\":" << info.keep_block_idx[i] - info.m << '}';
        }
        out << "],\"layoutTruncated\":" << (last_marginalization_parameter_blocks_.size() > count ? "true" : "false")
            << ",\"priorAtCurrentParameters\":" << last_marginalization_info_->getPriorNormalDiagnostics(last_marginalization_parameter_blocks_);
    }
    out << '}';
    incoming_prior_json_ = out.str();
}

void Optimizer::finishDiagnostics() {
    if (!diagnostic_capture_) return;
    std::ostringstream out;
    out << "{\"enabled\":true,\"frameTimestamp\":"; diagnosticNumber(out, diagnostic_timestamp_);
    out << ",\"marginFlag\":" << int(diagnostic_margin_) << ",\"incomingPrior\":" << incoming_prior_json_
        << ",\"visualMembership\":" << visual_membership_json_ << ",\"solver\":" << diagnostics_.numericalDetails
        << ",\"solverUsable\":" << (diagnostics_.usable ? "true" : "false") << ",\"reason\":";
    diagnosticString(out, diagnostics_.reason);
    out << ",\"gaugeApplication\":" << gauge_json_ << ",\"outgoingPriorForNextFrame\":" << outgoing_prior_json_ << '}';
    diagnostic_json_ = out.str();
}

void Optimizer::recordMarginalizationDiagnostics(factor::MarginalizationInfo* info) {
    if (!diagnostic_capture_) return;
    std::ostringstream out;
    out << "{\"captured\":true,\"createdFromFrameTimestamp\":"; diagnosticNumber(out, diagnostic_timestamp_);
    out << ",\"marginFlag\":" << int(diagnostic_margin_) << ",\"operationSemanticLayout\":[";
    const auto& order = info->getParameterBlockOrder();
    const size_t count = std::min(order.size(), size_t(1000));
    for (size_t i = 0; i < count; ++i) {
        if (i) out << ',';
        out << "{\"role\":"; diagnosticString(out, diagnosticBlockRole(reinterpret_cast<double*>(order[i])));
        out << ",\"globalSize\":" << info->parameter_block_size.at(order[i])
            << ",\"localColumn\":" << info->parameter_block_idx.at(order[i])
            << ",\"dropped\":" << (info->parameter_block_idx.at(order[i]) < info->m ? "true" : "false") << '}';
    }
    out << "],\"layoutTruncated\":" << (order.size() > count ? "true" : "false")
        << ",\"actualMarginalization\":" << info->getMarginalizationDiagnostics() << '}';
    previous_marginalization_json_ = out.str();
    outgoing_prior_json_ = "{\"updated\":true,\"use\":\"next_frame_input\",\"snapshot\":" + previous_marginalization_json_ + "}";
}

Optimizer::Optimizer(SlidingWindow* sliding_window, frontend::FeatureManager* feature_manager)
    : sliding_window_(sliding_window), feature_manager_(feature_manager), last_marginalization_info_(nullptr) {
    t_ic_ = Vector3d::Zero();
    r_ic_ = Matrix3d::Identity();
}

Optimizer::~Optimizer() {
    releaseMarginalizationPrior();
}

void Optimizer::releaseMarginalizationPrior() {
    if (last_marginalization_info_ != nullptr) {
        delete last_marginalization_info_;
        last_marginalization_info_ = nullptr;
    }
    last_marginalization_parameter_blocks_.clear();
    previous_marginalization_json_ = "{\"captured\":false}";
    if (diagnostic_capture_) outgoing_prior_json_ = "{\"updated\":true,\"released\":true}";
}

void Optimizer::reset() {
    releaseMarginalizationPrior();
    marginalization_loss_.reset();
    diagnostics_ = SolverDiagnostics{};
    diagnostic_json_ = "{\"enabled\":false}";
    previous_marginalization_json_ = "{\"captured\":false}";
    incoming_prior_json_.clear(); visual_membership_json_.clear(); gauge_json_.clear(); outgoing_prior_json_.clear();
}

void Optimizer::setExtrinsicParameters(const Vector3d& t_ic, const Matrix3d& r_ic) {
    t_ic_ = t_ic;
    r_ic_ = r_ic;
}

bool Optimizer::optimize(common::MarginalizationFlag marginalization_flag) {
    diagnostics_ = SolverDiagnostics{};
    // STEP 1: Setup optimization problem and parameter blocks
    ceres::Problem problem;
    setupOptimizationProblem(problem);
    beginDiagnostics(marginalization_flag);

    // STEP 2: Add various constraint factors
    addMarginalizationFactor(problem);
    addIMUFactors(problem);
    diagnostics_.visualFactors = addFeatureFactors(problem);
    if (diagnostics_.rejectedImuFactors || diagnostics_.imuFactors != WINDOW_SIZE ||
        diagnostics_.currentVisualFactors < 2 || !validateOptimizationParameters()) {
        diagnostics_.reason = diagnostics_.rejectedImuFactors ? "invalid_or_uncovered_imu_interval" :
                              diagnostics_.currentVisualFactors < 2 ? "insufficient_current_features" :
                              "invalid_initial_parameters";
        // The estimator still advances its window on a rejected update. A prior
        // cannot retain the old slot mapping across that advance.
        releaseMarginalizationPrior();
        finishDiagnostics();
        return false;
    }

    // Backup parameters before Ceres solve
    double saved_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double saved_SpeedAndBiases[WINDOW_SIZE + 1][SIZE_SPEEDANDBIAS];
    double saved_Feature[NUM_OF_FEATURES][SIZE_FEATURE];
    std::memcpy(saved_Pose, para_Pose, sizeof(para_Pose));
    std::memcpy(saved_SpeedAndBiases, para_SpeedAndBiases, sizeof(para_SpeedAndBiases));
    std::memcpy(saved_Feature, para_Feature, sizeof(para_Feature));

    // STEP 3: Solve the optimization problem
    solveCeresProblem(problem);

    // NaN/Inf validation: rollback on failure
    if (!diagnostics_.usable || !validateOptimizationParameters()) {
#ifndef NDEBUG
        std::cout << "Optimization produced NaN/Inf, rolling back parameters" << std::endl;
#endif
        std::memcpy(para_Pose, saved_Pose, sizeof(para_Pose));
        std::memcpy(para_SpeedAndBiases, saved_SpeedAndBiases, sizeof(para_SpeedAndBiases));
        std::memcpy(para_Feature, saved_Feature, sizeof(para_Feature));
        diagnostics_.usable = false;
        if (diagnostics_.reason == "usable") diagnostics_.reason = "invalid_optimized_parameters";
        releaseMarginalizationPrior();
        finishDiagnostics();
        return false;  // Skip applying results and marginalization.
    }

    // STEP 4: Apply results and handle marginalization
    applyOptimizationResults();
    handleMarginalization(marginalization_flag);
    finishDiagnostics();
    return true;
}

void Optimizer::setupOptimizationProblem(ceres::Problem& problem) {
    // Add pose and speed-bias parameter blocks for sliding window
    for (int i = 0; i < WINDOW_SIZE + 1; i++) {
        ceres::Manifold* pose_manifold = new backend::factor::PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, pose_manifold);      // R, P
        problem.AddParameterBlock(para_SpeedAndBiases[i], SIZE_SPEEDANDBIAS);   // V, Ba, Bg
    }

    // Add extrinsic parameter block
    ceres::Manifold* pose_manifold = new backend::factor::PoseLocalParameterization();
    problem.AddParameterBlock(para_Ex_Pose, SIZE_POSE, pose_manifold);
    problem.SetParameterBlockConstant(para_Ex_Pose);

    // Prepare optimization parameters
    prepareOptimizationParameters();
    if (!last_marginalization_info_) problem.SetParameterBlockConstant(para_Pose[0]);
}

void Optimizer::addMarginalizationFactor(ceres::Problem& problem) {
    if (last_marginalization_info_) {
        // construct new marginlization_factor
        backend::factor::MarginalizationFactor* marginalization_factor =
            new backend::factor::MarginalizationFactor(last_marginalization_info_);
        problem.AddResidualBlock(marginalization_factor, NULL, last_marginalization_parameter_blocks_);
    }
}

void Optimizer::addIMUFactors(ceres::Problem& problem) {
    for (int i = 0; i < WINDOW_SIZE; i++) {
        int j = i + 1;
        if (!(*sliding_window_)[j].pre_integration) {
#ifndef NDEBUG
            std::cout << "Warning: pre_integration null at index " << j << ", skipping IMU factor" << std::endl;
#endif
            ++diagnostics_.rejectedImuFactors;
            continue;
        }
        const double expected_dt = (*sliding_window_)[j].timestamp - (*sliding_window_)[i].timestamp;
        auto imu_factor = std::make_unique<backend::factor::IMUFactor>((*sliding_window_)[j].pre_integration.get());
        if (!imu_factor->isValid() || !std::isfinite(expected_dt) || expected_dt <= 0 ||
            std::abs((*sliding_window_)[j].pre_integration->sum_dt - expected_dt) > 1e-6) {
            ++diagnostics_.rejectedImuFactors;
            continue;
        }
        ++diagnostics_.imuFactors;
        problem.AddResidualBlock(imu_factor.release(), NULL, para_Pose[i], para_SpeedAndBiases[i], para_Pose[j],
                                 para_SpeedAndBiases[j]);
    }
}

int Optimizer::addFeatureFactors(ceres::Problem& problem) {
    auto loss_function = std::make_unique<ceres::CauchyLoss>(1.0);
    std::unique_ptr<std::ostringstream> membership;
    size_t membership_count = 0;
    bool membership_truncated = false;
    if (diagnostic_capture_) membership = std::make_unique<std::ostringstream>();

    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto& it_per_id : feature_manager_->feature_bank_) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        ++feature_index;

        // Bounds check: prevent out-of-bounds access on para_Feature
        if (feature_index >= NUM_OF_FEATURES) {
#ifndef NDEBUG
            std::cout << "Warning: feature count (" << feature_index
                      << ") exceeds NUM_OF_FEATURES (" << NUM_OF_FEATURES
                      << "), skipping remaining features" << std::endl;
#endif
            membership_truncated = true;
            break;
        }
        int added_for_feature = 0, current_for_feature = 0, rejected_geometry = 0;
        const bool depth_valid = membership && it_per_id.solve_flag != 2 && std::isfinite(para_Feature[feature_index][0]) &&
                                 para_Feature[feature_index][0] > 1e-10;
        const auto record_membership = [&] {
            if (!membership) return;
            if (membership_count >= 1000) { membership_truncated = true; return; }
            if (membership_count++) *membership << ',';
            *membership << "{\"id\":" << it_per_id.feature_id << ",\"parameterIndex\":" << feature_index
                        << ",\"startFrame\":" << it_per_id.start_frame << ",\"observations\":" << it_per_id.used_num
                        << ",\"solveFlag\":" << it_per_id.solve_flag << ",\"depthValid\":" << (depth_valid ? "true" : "false")
                        << ",\"inverseDepth\":"; diagnosticNumber(*membership, para_Feature[feature_index][0]);
            *membership << ",\"estimatedDepth\":"; diagnosticNumber(*membership, it_per_id.estimated_depth);
            *membership << ",\"nonfinite\":" << (std::isfinite(para_Feature[feature_index][0]) && std::isfinite(it_per_id.estimated_depth) ? "false" : "true")
                        << ",\"addedResiduals\":" << added_for_feature << ",\"currentImageResiduals\":" << current_for_feature
                        << ",\"rejectedGeometry\":" << rejected_geometry << '}';
        };
        if (it_per_id.solve_flag == 2 || !std::isfinite(para_Feature[feature_index][0]) ||
            para_Feature[feature_index][0] <= 1e-10) { record_membership(); continue; }

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].ray_vector;

        for (auto& it_per_frame : it_per_id.feature_per_frame) {
            imu_j++;
            if (imu_i == imu_j) {
                continue;
            }
            Vector3d pts_j = it_per_frame.ray_vector;
            if (imu_j > WINDOW_SIZE) break;
            auto f = std::make_unique<backend::factor::ProjectionFactor>(pts_i, pts_j);
            const double* parameters[] = {para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose, para_Feature[feature_index]};
            double residual[2];
            if (!f->Evaluate(parameters, residual, nullptr)) { if (membership) ++rejected_geometry; continue; }
            problem.AddResidualBlock(f.release(), loss_function.get(), para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose,
                                     para_Feature[feature_index]);
            f_m_cnt++;
            if (imu_j == WINDOW_SIZE) ++diagnostics_.currentVisualFactors;
            if (membership) {
                ++added_for_feature;
                if (imu_j == WINDOW_SIZE) ++current_for_feature;
            }
        }
        record_membership();
    }
    if (membership) visual_membership_json_ = "{\"rowsLimit\":1000,\"truncated\":" +
        std::string(membership_truncated ? "true" : "false") + ",\"rows\":[" + membership->str() + "]}";
    if (f_m_cnt) static_cast<void>(loss_function.release());  // Ceres owns the shared loss once used.
    return f_m_cnt;
}

void Optimizer::solveCeresProblem(ceres::Problem& problem, int pending_frames) {
    double solver_time = g_config.estimator.solver_time;

    // Adaptive solver time: reduce under load to prevent frame queue buildup.
    // WASM single-thread: pending_frames=0 always → full solver_time.
    // Future multi-threaded/Web Worker mode: queue depth drives reduction.
    if (g_config.pnp.enable_adaptive && pending_frames > 0) {
        if (pending_frames < 2)
            solver_time = g_config.estimator.solver_time;
        else if (pending_frames < 4)
            solver_time = g_config.estimator.solver_time * 2.0 / 3.0;
        else
            solver_time = g_config.estimator.solver_time * 0.5;

        // Safety floor: solver_time < 0.04 causes Ba convergence failure
        solver_time = std::max(solver_time, g_config.pnp.min_solver_time);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_solver_time_in_seconds = solver_time;
    options.max_num_iterations = g_config.estimator.num_iterations;
    if (benchmark_solver_profile_) {
        options.function_tolerance = 0;
        options.gradient_tolerance = 0;
        options.parameter_tolerance = 0;
    }
    options.logging_type = ceres::SILENT;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    diagnostics_.iterations = static_cast<int>(summary.iterations.size());
    diagnostics_.terminationType = static_cast<int>(summary.termination_type);
    diagnostics_.initialCost = summary.initial_cost;
    diagnostics_.finalCost = summary.final_cost;
    diagnostics_.usable = summary.IsSolutionUsable() && std::isfinite(summary.initial_cost) &&
                          std::isfinite(summary.final_cost) && summary.num_residuals > 0;
    diagnostics_.reason = diagnostics_.usable ? "usable" : "unusable_solver_result";
    if (diagnostic_capture_) {
        std::ostringstream out;
        out << "{\"terminationType\":" << int(summary.termination_type) << ",\"message\":";
        diagnosticString(out, summary.message);
        out << ",\"messageTruncated\":" << (summary.message.size() > 4096 ? "true" : "false")
            << ",\"numThreadsGiven\":" << summary.num_threads_given << ",\"numThreadsUsed\":" << summary.num_threads_used
            << ",\"initialCost\":"; diagnosticNumber(out, summary.initial_cost);
        out << ",\"finalCost\":"; diagnosticNumber(out, summary.final_cost);
        out << ",\"nonfinite\":" << (std::isfinite(summary.initial_cost) && std::isfinite(summary.final_cost) ? "false" : "true")
            << ",\"benchmarkSolverProfile\":" << (benchmark_solver_profile_ ? "true" : "false")
            << ",\"solverProfile\":\"" << (benchmark_solver_profile_ ? "max10_zero_positive_tolerances" : "default") << "\""
            << ",\"iterationCountSemantics\":\"Ceres summary rows including iteration0; early-tolerance terminating trial may be absent\""
            << ",\"functionTolerance\":"; diagnosticNumber(out, options.function_tolerance);
        out << ",\"gradientTolerance\":"; diagnosticNumber(out, options.gradient_tolerance);
        out << ",\"parameterTolerance\":"; diagnosticNumber(out, options.parameter_tolerance);
        out << ",\"maxIterations\":" << options.max_num_iterations << ",\"maxTimeSeconds\":";
        diagnosticNumber(out, options.max_solver_time_in_seconds);
        const size_t count = std::min(summary.iterations.size(), size_t(100));
        out << ",\"iterationRowsLimit\":100,\"iterationRowsTruncated\":" << (summary.iterations.size() > count ? "true" : "false")
            << ",\"parameterRelativeStepAvailable\":false,\"iterations\":[";
        for (size_t i = 0; i < count; ++i) {
            if (i) out << ',';
            const auto& row = summary.iterations[i];
            out << "{\"iteration\":" << row.iteration << ",\"successfulStep\":" << (row.step_is_successful ? "true" : "false")
                << ",\"validStep\":" << (row.step_is_valid ? "true" : "false")
                << ",\"nonmonotonicStep\":" << (row.step_is_nonmonotonic ? "true" : "false")
                << ",\"linearSolverIterations\":" << row.linear_solver_iterations
                << ",\"cost\":"; diagnosticNumber(out, row.cost);
            out << ",\"costChange\":"; diagnosticNumber(out, row.cost_change);
            out << ",\"gradientMaxNorm\":"; diagnosticNumber(out, row.gradient_max_norm);
            out << ",\"stepNorm\":"; diagnosticNumber(out, row.step_norm);
            out << ",\"relativeDecrease\":"; diagnosticNumber(out, row.relative_decrease);
            out << ",\"trustRegionRadius\":"; diagnosticNumber(out, row.trust_region_radius);
            out << ",\"nonfinite\":" << (std::isfinite(row.cost) && std::isfinite(row.cost_change) &&
                std::isfinite(row.gradient_max_norm) && std::isfinite(row.step_norm) &&
                std::isfinite(row.relative_decrease) && std::isfinite(row.trust_region_radius) ? "false" : "true") << '}';
        }
        out << "]}";
        diagnostics_.numericalDetails = out.str();
    }
}

void Optimizer::applyOptimizationResults() {
    Vector3d origin_R0 = Utility::R2ypr((*sliding_window_).front().R);
    Vector3d origin_P0 = (*sliding_window_).front().P;

    Vector3d origin_R00 = Utility::R2ypr(
        Quaterniond(para_Pose[0][6], para_Pose[0][3], para_Pose[0][4], para_Pose[0][5]).toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();
    if (diagnostic_capture_) {
        const bool singularity_branch = abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0;
        std::ostringstream out;
        out << "{\"originPitchDegrees\":"; diagnosticNumber(out, origin_R0.y());
        out << ",\"optimizedPitchDegrees\":"; diagnosticNumber(out, origin_R00.y());
        out << ",\"yawDifferenceDegrees\":"; diagnosticNumber(out, y_diff);
        out << ",\"nonfinite\":" << (std::isfinite(origin_R0.y()) && std::isfinite(origin_R00.y()) && std::isfinite(y_diff) ? "false" : "true")
            << ",\"branch\":\"" << (singularity_branch ? "full_rotation_at_euler_singularity" : "yaw_gauge_only") << "\"}";
        gauge_json_ = out.str();
    }
    // TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0) {
#ifndef NDEBUG
        std::cout << "euler singular point!" << std::endl;
#endif
        rot_diff = (*sliding_window_).front().R *
                   Quaterniond(para_Pose[0][6], para_Pose[0][3], para_Pose[0][4], para_Pose[0][5])
                       .toRotationMatrix()
                       .transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; i++) {
        (*sliding_window_)[i].R =
            rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5])
                           .normalized()
                           .toRotationMatrix();

        (*sliding_window_)[i].P =
            rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0], para_Pose[i][1] - para_Pose[0][1],
                                para_Pose[i][2] - para_Pose[0][2]) +
            origin_P0;

        (*sliding_window_)[i].V =
            rot_diff * Vector3d(para_SpeedAndBiases[i][0], para_SpeedAndBiases[i][1], para_SpeedAndBiases[i][2]);

        (*sliding_window_)[i].Ba =
            Vector3d(para_SpeedAndBiases[i][3], para_SpeedAndBiases[i][4], para_SpeedAndBiases[i][5]);

        (*sliding_window_)[i].Bg =
            Vector3d(para_SpeedAndBiases[i][6], para_SpeedAndBiases[i][7], para_SpeedAndBiases[i][8]);
    }

    t_ic_ = Vector3d(para_Ex_Pose[0], para_Ex_Pose[1], para_Ex_Pose[2]);
    r_ic_ = Quaterniond(para_Ex_Pose[6], para_Ex_Pose[3], para_Ex_Pose[4], para_Ex_Pose[5]).toRotationMatrix();

    VectorXd dep = feature_manager_->getDepthVector();
    int feature_count = std::min(feature_manager_->getFeatureCount(), NUM_OF_FEATURES);
    for (int i = 0; i < feature_count; i++)
        dep(i) = para_Feature[i][0];
    feature_manager_->setDepth(dep);
}

void Optimizer::handleMarginalization(common::MarginalizationFlag marginalization_flag) {
    if (marginalization_flag == common::MarginalizationFlag::MARGIN_OLD_KEYFRAME) {
        marginalizeOldKeyframe();
    } else if (marginalization_flag == common::MarginalizationFlag::MARGIN_NEW_GENERAL_FRAME) {
        marginalizeNewGeneralFrame();
    }
}

void Optimizer::prepareOptimizationParameters() {
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        // P
        para_Pose[i][0] = (*sliding_window_)[i].P.x();
        para_Pose[i][1] = (*sliding_window_)[i].P.y();
        para_Pose[i][2] = (*sliding_window_)[i].P.z();

        // R
        Quaterniond q{(*sliding_window_)[i].R};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        // V
        para_SpeedAndBiases[i][0] = (*sliding_window_)[i].V.x();
        para_SpeedAndBiases[i][1] = (*sliding_window_)[i].V.y();
        para_SpeedAndBiases[i][2] = (*sliding_window_)[i].V.z();

        // Ba
        para_SpeedAndBiases[i][3] = (*sliding_window_)[i].Ba.x();
        para_SpeedAndBiases[i][4] = (*sliding_window_)[i].Ba.y();
        para_SpeedAndBiases[i][5] = (*sliding_window_)[i].Ba.z();

        // Bg
        para_SpeedAndBiases[i][6] = (*sliding_window_)[i].Bg.x();
        para_SpeedAndBiases[i][7] = (*sliding_window_)[i].Bg.y();
        para_SpeedAndBiases[i][8] = (*sliding_window_)[i].Bg.z();
    }

    // Camera-IMU Extrinsic
    para_Ex_Pose[0] = t_ic_.x();  // translation
    para_Ex_Pose[1] = t_ic_.y();
    para_Ex_Pose[2] = t_ic_.z();

    Quaterniond q{r_ic_};  // rotation
    para_Ex_Pose[3] = q.x();
    para_Ex_Pose[4] = q.y();
    para_Ex_Pose[5] = q.z();
    para_Ex_Pose[6] = q.w();

    // Feature point depth (with bounds check)
    VectorXd dep = feature_manager_->getDepthVector();
    int feature_count = std::min(feature_manager_->getFeatureCount(), NUM_OF_FEATURES);
    for (int i = 0; i < feature_count; i++)
        para_Feature[i][0] = dep(i);
}

void Optimizer::marginalizeOldKeyframe() {
    factor::MarginalizationInfo* marginalization_info = new factor::MarginalizationInfo();
    marginalization_info->setDiagnosticCapture(diagnostic_capture_);
    prepareOptimizationParameters();

    if (last_marginalization_info_) {
        vector<int> drop_set;
        for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks_.size()); i++) {
            bool does_last_margin_param_block_contain_the_old_keyframe_pose_and_speed_bias =
                last_marginalization_parameter_blocks_[i] == para_Pose[0] ||
                last_marginalization_parameter_blocks_[i] == para_SpeedAndBiases[0];

            if (does_last_margin_param_block_contain_the_old_keyframe_pose_and_speed_bias)
                drop_set.push_back(i);
        }
        factor::MarginalizationFactor* marginalization_factor =
            new factor::MarginalizationFactor(last_marginalization_info_);
        factor::ResidualBlockInfo* residual_block_info = new factor::ResidualBlockInfo(
            marginalization_factor, NULL, last_marginalization_parameter_blocks_, drop_set);
        marginalization_info->addResidualBlockInfo(residual_block_info);
    }

    addIMUFactorForMarginalization(marginalization_info);
    addFeatureFactorsForMarginalization(marginalization_info);

    performMarginalizationForOldKeyframe(marginalization_info);
}

void Optimizer::marginalizeNewGeneralFrame() {
    bool does_last_margin_param_block_contain_the_new_general_frame_pose =
        std::count(std::begin(last_marginalization_parameter_blocks_), std::end(last_marginalization_parameter_blocks_),
                   para_Pose[WINDOW_SIZE - 1]);

    if (does_last_margin_param_block_contain_the_new_general_frame_pose) {
        factor::MarginalizationInfo* marginalization_info = new factor::MarginalizationInfo();
        marginalization_info->setDiagnosticCapture(diagnostic_capture_);
        prepareOptimizationParameters();

        if (last_marginalization_info_) {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks_.size()); i++) {
                assert(last_marginalization_parameter_blocks_[i] != para_SpeedAndBiases[WINDOW_SIZE - 1]);
                if (last_marginalization_parameter_blocks_[i] == para_Pose[WINDOW_SIZE - 1])
                    drop_set.push_back(i);
            }
            factor::MarginalizationFactor* marginalization_factor =
                new factor::MarginalizationFactor(last_marginalization_info_);
            factor::ResidualBlockInfo* residual_block_info = new factor::ResidualBlockInfo(
                marginalization_factor, NULL, last_marginalization_parameter_blocks_, drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        performMarginalizationForNewGeneralFrame(marginalization_info);
    }
}

void Optimizer::addIMUFactorForMarginalization(factor::MarginalizationInfo* marginalization_info) {
    if (!(*sliding_window_)[1].pre_integration) {
#ifndef NDEBUG
        std::cout << "Warning: pre_integration null at index 1, skipping IMU marginalization factor" << std::endl;
#endif
        return;
    }
    auto imu_factor = std::make_unique<factor::IMUFactor>((*sliding_window_)[1].pre_integration.get());
    const double expected_dt = (*sliding_window_)[1].timestamp - (*sliding_window_)[0].timestamp;
    if (imu_factor->isValid() && expected_dt > 0 &&
        std::abs((*sliding_window_)[1].pre_integration->sum_dt - expected_dt) <= 1e-6) {
        factor::ResidualBlockInfo* residual_block_info = new factor::ResidualBlockInfo(
            imu_factor.release(), NULL,
            vector<double*>{para_Pose[0], para_SpeedAndBiases[0], para_Pose[1], para_SpeedAndBiases[1]},
            vector<int>{0, 1});
        marginalization_info->addResidualBlockInfo(residual_block_info);
    }
}

void Optimizer::addFeatureFactorsForMarginalization(factor::MarginalizationInfo* marginalization_info) {
    if (!marginalization_loss_) marginalization_loss_ = std::make_unique<ceres::CauchyLoss>(1.0);
    int feature_index = -1;
    for (auto& it_per_id : feature_manager_->feature_bank_) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        ++feature_index;

        // Bounds check
        if (feature_index >= NUM_OF_FEATURES)
            break;
        if (it_per_id.solve_flag == 2 || !std::isfinite(para_Feature[feature_index][0]) ||
            para_Feature[feature_index][0] <= 1e-10) continue;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        if (imu_i != 0)
            continue;

        Vector3d pts_i = it_per_id.feature_per_frame[0].ray_vector;

        for (auto& it_per_frame : it_per_id.feature_per_frame) {
            imu_j++;
            if (imu_i == imu_j)
                continue;

            Vector3d pts_j = it_per_frame.ray_vector;
            if (imu_j > WINDOW_SIZE) break;
            auto f = std::make_unique<factor::ProjectionFactor>(pts_i, pts_j);
            const double* parameters[] = {para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose, para_Feature[feature_index]};
            double residual[2];
            if (!f->Evaluate(parameters, residual, nullptr)) continue;
            factor::ResidualBlockInfo* residual_block_info = new factor::ResidualBlockInfo(
                f.release(), marginalization_loss_.get(),
                vector<double*>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose, para_Feature[feature_index]},
                vector<int>{0, 3});
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }
    }
}

void Optimizer::performMarginalizationForOldKeyframe(factor::MarginalizationInfo* marginalization_info) {
    std::unique_ptr<factor::MarginalizationInfo> candidate(marginalization_info);
    try {
        marginalization_info->preMarginalize();
        if (!marginalization_info->marginalize()) {
            diagnostics_.qualityReason = marginalization_info->getFailureReason();
            const std::string operation = marginalization_info->getMarginalizationDiagnostics();
            candidate.reset();
            releaseMarginalizationPrior();
            if (diagnostic_capture_) outgoing_prior_json_ =
                "{\"updated\":false,\"priorReleased\":true,\"failureReason\":\"" + diagnostics_.qualityReason +
                "\",\"actualMarginalization\":" + operation + "}";
            return;
        }

        std::unordered_map<long, double*> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++) {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedAndBiases[i])] = para_SpeedAndBiases[i - 1];
        }
        addr_shift[reinterpret_cast<long>(para_Ex_Pose)] = para_Ex_Pose;
        vector<double*> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info_)
            delete last_marginalization_info_;
        last_marginalization_info_ = candidate.release();
        last_marginalization_parameter_blocks_ = std::move(parameter_blocks);
        recordMarginalizationDiagnostics(marginalization_info);
    } catch (const std::bad_alloc&) {
        candidate.reset();
        releaseMarginalizationPrior();
        diagnostics_.qualityReason = "marginalization_allocation_failed";
    }
}

void Optimizer::performMarginalizationForNewGeneralFrame(factor::MarginalizationInfo* marginalization_info) {
    std::unique_ptr<factor::MarginalizationInfo> candidate(marginalization_info);
    try {
        marginalization_info->preMarginalize();
        if (!marginalization_info->marginalize()) {
            diagnostics_.qualityReason = marginalization_info->getFailureReason();
            const std::string operation = marginalization_info->getMarginalizationDiagnostics();
            candidate.reset();
            releaseMarginalizationPrior();
            if (diagnostic_capture_) outgoing_prior_json_ =
                "{\"updated\":false,\"priorReleased\":true,\"failureReason\":\"" + diagnostics_.qualityReason +
                "\",\"actualMarginalization\":" + operation + "}";
            return;
        }

        std::unordered_map<long, double*> addr_shift;
        for (int i = 0; i <= WINDOW_SIZE; i++) {
            if (i == WINDOW_SIZE - 1)
                continue;
            else if (i == WINDOW_SIZE) {
                addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                addr_shift[reinterpret_cast<long>(para_SpeedAndBiases[i])] = para_SpeedAndBiases[i - 1];
            } else {
                addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                addr_shift[reinterpret_cast<long>(para_SpeedAndBiases[i])] = para_SpeedAndBiases[i];
            }
        }
        addr_shift[reinterpret_cast<long>(para_Ex_Pose)] = para_Ex_Pose;

        vector<double*> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
        if (last_marginalization_info_)
            delete last_marginalization_info_;
        last_marginalization_info_ = candidate.release();
        last_marginalization_parameter_blocks_ = std::move(parameter_blocks);
        recordMarginalizationDiagnostics(marginalization_info);
    } catch (const std::bad_alloc&) {
        candidate.reset();
        releaseMarginalizationPrior();
        diagnostics_.qualityReason = "marginalization_allocation_failed";
    }
}

bool Optimizer::validateOptimizationParameters() const {
    for (int i = 0; i < WINDOW_SIZE + 1; i++) {
        for (int j = 0; j < SIZE_POSE; j++) {
            if (!std::isfinite(para_Pose[i][j])) return false;
        }
        const Eigen::Quaterniond q(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]);
        if (std::abs(q.squaredNorm() - 1) > 1e-6) return false;
        for (int j = 0; j < SIZE_SPEEDANDBIAS; j++) {
            if (!std::isfinite(para_SpeedAndBiases[i][j])) return false;
        }
    }
    int feature_count = std::min(feature_manager_->getFeatureCount(), NUM_OF_FEATURES);
    for (int i = 0; i < feature_count; i++) {
        if (!std::isfinite(para_Feature[i][0]) || para_Feature[i][0] <= 1e-10) return false;
    }
    return true;
}

}  // namespace backend
