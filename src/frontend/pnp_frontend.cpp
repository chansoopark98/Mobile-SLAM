#include "frontend/pnp_frontend.h"

#include <ceres/ceres.h>
#include <unordered_set>

#include "backend/factor/imu_factor_pnp.h"
#include "backend/factor/perspective_factor.h"
#include "backend/factor/pose_local_parameterization.h"
#include "utility/utility.h"

namespace frontend {

PnPFrontend::PnPFrontend() : first_imu_(false), frame_count_(0), has_pose_(false) {
    clearState();
}

void PnPFrontend::clearState() {
    for (int i = 0; i <= kPnPSize; i++) {
        Rs_[i].setIdentity();
        Ps_[i].setZero();
        Vs_[i].setZero();
        Bas_[i].setZero();
        Bgs_[i].setZero();
        pre_integrations_[i].reset();
        dt_buf_[i].clear();
        acc_buf_[i].clear();
        gyr_buf_[i].clear();
        features_[i].clear();
        find_solved_[i] = false;
        headers_[i] = 0;
    }
    tic_ = utility::g_config.camera.t_ic;
    ric_ = utility::g_config.camera.r_ic;
    g_ = utility::g_config.estimator.g;
    frame_count_ = 0;
    first_imu_ = false;
    has_pose_ = false;
    latest_image_update_usable_ = false;
    backend_initialized_ = false;
    backend_anchor_timestamp_ = 0;
    acc_0_.setZero();
    gyr_0_.setZero();
    diagnostics_ = backend::SolverDiagnostics{};
}

void PnPFrontend::setExtrinsicParameters(const Eigen::Matrix3d& ric, const Eigen::Vector3d& tic) {
    ric_ = ric;
    tic_ = tic;
}

void PnPFrontend::setIMUModel() {
    backend::factor::PerspectiveFactor::sqrt_info =
        utility::g_config.camera.focal_length / 1.5 * Eigen::Matrix2d::Identity();
}

void PnPFrontend::initializeState(const common::VINSResult& result) {
    if (!std::isfinite(result.timestamp) || !result.P.allFinite() || !result.R.allFinite() ||
        !result.V.allFinite() || !result.Ba.allFinite() || !result.Bg.allFinite() ||
        (result.R.transpose() * result.R - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
        std::abs(result.R.determinant() - 1) > 1e-6) return;
    for (int i = 0; i <= kPnPSize; i++) {
        Ps_[i] = result.P;
        Rs_[i] = result.R;
        Vs_[i] = result.V;
        Bas_[i] = result.Ba;
        Bgs_[i] = result.Bg;
    }
    backend_initialized_ = true;
    backend_anchor_timestamp_ = result.timestamp;
    headers_[0] = result.timestamp;
    find_solved_[0] = true;
}

void PnPFrontend::setBackendSolution(const common::VINSResult& result) {
    if (!backend_initialized_ || !std::isfinite(result.timestamp) || !result.P.allFinite() ||
        !result.R.allFinite() || !result.V.allFinite() || !result.Ba.allFinite() || !result.Bg.allFinite()) return;
    for (int i = 0; i <= kPnPSize; i++) {
        Bas_[i] = result.Ba;
        Bgs_[i] = result.Bg;
        if (headers_[i] == result.timestamp) {
            find_solved_[i] = true;
            Ps_[i] = result.P;
            Rs_[i] = result.R;
            Vs_[i] = result.V;
        }
    }
}

void PnPFrontend::processIMU(double dt, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) {
    if (!std::isfinite(dt) || dt < 0 || !acc.allFinite() || !gyro.allFinite()) {
        latest_image_update_usable_ = false;
        diagnostics_.reason = "invalid_imu_input";
        return;
    }
    if (!first_imu_) {
        first_imu_ = true;
        acc_0_ = acc;
        gyr_0_ = gyro;
    }
    if (dt == 0) {
        acc_0_ = acc;
        gyr_0_ = gyro;
        return;
    }

    if (pre_integrations_[frame_count_] == nullptr) {
        pre_integrations_[frame_count_] =
            std::make_unique<backend::factor::IntegrationBase>(acc_0_, gyr_0_, Bas_[frame_count_], Bgs_[frame_count_]);
    }

    if (frame_count_ != 0) {
        pre_integrations_[frame_count_]->push_back(dt, acc, gyro);
        dt_buf_[frame_count_].push_back(dt);
        acc_buf_[frame_count_].push_back(acc);
        gyr_buf_[frame_count_].push_back(gyro);

        // Midpoint integration for state propagation
        int j = frame_count_;
        Eigen::Vector3d un_acc_0 = Rs_[j] * (acc_0_ - Bas_[j]) - g_;
        Eigen::Vector3d un_gyr = 0.5 * (gyr_0_ + gyro) - Bgs_[j];
        Rs_[j] *= Utility::deltaQ(un_gyr * dt).normalized().toRotationMatrix();
        Eigen::Vector3d un_acc_1 = Rs_[j] * (acc - Bas_[j]) - g_;
        Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps_[j] += dt * Vs_[j] + 0.5 * dt * dt * un_acc;
        Vs_[j] += dt * un_acc;
    }
    acc_0_ = acc;
    gyr_0_ = gyro;
}

bool PnPFrontend::processImage(const std::vector<common::SolvedFeature>& features,
                                double timestamp) {
    latest_image_update_usable_ = false;
    has_pose_ = false;
    diagnostics_ = backend::SolverDiagnostics{};
    if (!std::isfinite(timestamp)) {
        diagnostics_.reason = "invalid_image_timestamp";
        return false;
    }
    features_[frame_count_] = features;
    headers_[frame_count_] = timestamp;
    if (frame_count_ == 0) find_solved_[0] = backend_initialized_ &&
        std::abs(timestamp - backend_anchor_timestamp_) <= 1e-6;

    // Update 3D positions and track counts in ALL previous window frames
    // using the latest feature data (VINS-Mobile: updateFeatures).
    // Without this, earlier frames use stale 3D positions from older backend solves,
    // creating inconsistent PerspectiveFactor residuals that degrade PnP accuracy.
    if (!features.empty()) {
        for (int i = 0; i < frame_count_; i++) {
            for (auto& stored : features_[i]) {
                for (const auto& latest : features) {
                    if (stored.id == latest.id) {
                        stored.position = latest.position;
                        stored.track_num = latest.track_num;
                        break;
                    }
                }
            }
        }
    }

    if (frame_count_ < kPnPSize) {
        const int next = frame_count_ + 1;
        Ps_[next] = Ps_[frame_count_];
        Rs_[next] = Rs_[frame_count_];
        Vs_[next] = Vs_[frame_count_];
        Bas_[next] = Bas_[frame_count_];
        Bgs_[next] = Bgs_[frame_count_];
        frame_count_++;
        diagnostics_.reason = "pnp_window_initializing";
        return false;
    }

    // PNP_SIZE frames accumulated — solve
    latest_image_update_usable_ = solveCeres();
    has_pose_ = latest_image_update_usable_;

    slideWindow();
    return latest_image_update_usable_;
}

void PnPFrontend::old2new() {
    for (int i = 0; i <= kPnPSize; i++) {
        para_Pose_[i][0] = Ps_[i].x();
        para_Pose_[i][1] = Ps_[i].y();
        para_Pose_[i][2] = Ps_[i].z();
        Eigen::Quaterniond q(Rs_[i]);
        para_Pose_[i][3] = q.x();
        para_Pose_[i][4] = q.y();
        para_Pose_[i][5] = q.z();
        para_Pose_[i][6] = q.w();

        para_Speed_[i][0] = Vs_[i].x();
        para_Speed_[i][1] = Vs_[i].y();
        para_Speed_[i][2] = Vs_[i].z();

        para_Bias_[i][0] = Bas_[i].x();
        para_Bias_[i][1] = Bas_[i].y();
        para_Bias_[i][2] = Bas_[i].z();
        para_Bias_[i][3] = Bgs_[i].x();
        para_Bias_[i][4] = Bgs_[i].y();
        para_Bias_[i][5] = Bgs_[i].z();
    }

    Eigen::Quaterniond q_ic(ric_);
    para_Ex_Pose_[0] = tic_.x();
    para_Ex_Pose_[1] = tic_.y();
    para_Ex_Pose_[2] = tic_.z();
    para_Ex_Pose_[3] = q_ic.x();
    para_Ex_Pose_[4] = q_ic.y();
    para_Ex_Pose_[5] = q_ic.z();
    para_Ex_Pose_[6] = q_ic.w();
}

void PnPFrontend::new2old() {
    for (int i = 0; i <= kPnPSize; i++) {
        Rs_[i] = Eigen::Quaterniond(para_Pose_[i][6], para_Pose_[i][3],
                                     para_Pose_[i][4], para_Pose_[i][5])
                     .normalized()
                     .toRotationMatrix();
        Ps_[i] = Eigen::Vector3d(para_Pose_[i][0], para_Pose_[i][1], para_Pose_[i][2]);
        Vs_[i] = Eigen::Vector3d(para_Speed_[i][0], para_Speed_[i][1], para_Speed_[i][2]);
        Bas_[i] = Eigen::Vector3d(para_Bias_[i][0], para_Bias_[i][1], para_Bias_[i][2]);
        Bgs_[i] = Eigen::Vector3d(para_Bias_[i][3], para_Bias_[i][4], para_Bias_[i][5]);
    }
}

bool PnPFrontend::solveCeres() {
    if (!backend_initialized_ || std::none_of(std::begin(find_solved_), std::end(find_solved_),
                                            [](bool solved) { return solved; })) {
        diagnostics_.reason = "missing_backend_anchor";
        return false;
    }
    if (features_[kPnPSize].size() < 6) {
        diagnostics_.reason = "insufficient_current_correspondences";
        return false;
    }
    std::unordered_set<int> current_ids;
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const auto& feature : features_[kPnPSize]) {
        if (!feature.position.allFinite() || !feature.observation.allFinite() || feature.track_num <= 0 ||
            !current_ids.insert(feature.id).second) {
            diagnostics_.reason = "invalid_current_correspondences";
            return false;
        }
        mean += feature.observation;
    }
    mean /= static_cast<double>(current_ids.size());
    Eigen::Matrix2d spread = Eigen::Matrix2d::Zero();
    for (const auto& feature : features_[kPnPSize]) {
        const Eigen::Vector2d offset = feature.observation - mean;
        spread += offset * offset.transpose();
    }
    if (spread.trace() <= 1e-12 || spread.determinant() <= 1e-12 * spread.trace() * spread.trace()) {
        diagnostics_.reason = "degenerate_current_correspondences";
        return false;
    }
    ceres::Problem problem;
    auto loss_function = std::make_unique<ceres::CauchyLoss>(1.0);

    // Add parameter blocks
    for (int i = 0; i <= kPnPSize; i++) {
        ceres::Manifold* pose_manifold = new backend::factor::PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose_[i], 7, pose_manifold);
        problem.AddParameterBlock(para_Speed_[i], 3);
        problem.AddParameterBlock(para_Bias_[i], 6);

        // Fix solved frames (from backend)
        if (find_solved_[i]) {
            problem.SetParameterBlockConstant(para_Pose_[i]);
            problem.SetParameterBlockConstant(para_Speed_[i]);
        }
        // ALL bias blocks are fixed (bias comes from backend)
        problem.SetParameterBlockConstant(para_Bias_[i]);
    }

    // Extrinsic: fixed
    ceres::Manifold* ex_manifold = new backend::factor::PoseLocalParameterization();
    problem.AddParameterBlock(para_Ex_Pose_, 7, ex_manifold);
    problem.SetParameterBlockConstant(para_Ex_Pose_);

    old2new();

    // IMU factors between consecutive frames
    for (int i = 0; i < kPnPSize; i++) {
        int j = i + 1;
        auto imu_factor = std::make_unique<backend::factor::IMUFactorPnP>(pre_integrations_[j].get());
        const double expected_dt = headers_[j] - headers_[i];
        if (!imu_factor->isValid() || expected_dt <= 0 ||
            std::abs(pre_integrations_[j]->sum_dt - expected_dt) > 1e-6) {
            ++diagnostics_.rejectedImuFactors;
            continue;
        }
        ++diagnostics_.imuFactors;
        problem.AddResidualBlock(imu_factor.release(), nullptr,
                                 para_Pose_[i], para_Speed_[i], para_Bias_[i],
                                 para_Pose_[j], para_Speed_[j], para_Bias_[j]);
    }

    // Perspective factors: known 3D → 2D projection
    for (int i = 0; i <= kPnPSize; i++) {
        for (const auto& feat : features_[i]) {
            auto f = std::make_unique<backend::factor::PerspectiveFactor>(feat.observation, feat.position, feat.track_num);
            const double* parameters[] = {para_Pose_[i], para_Ex_Pose_};
            double residual[2];
            if (!f->Evaluate(parameters, residual, nullptr)) continue;
            ++diagnostics_.visualFactors;
            if (i == kPnPSize) ++diagnostics_.currentVisualFactors;
            problem.AddResidualBlock(f.release(), loss_function.get(), para_Pose_[i], para_Ex_Pose_);
        }
    }
    if (diagnostics_.visualFactors) static_cast<void>(loss_function.release());
    if (diagnostics_.rejectedImuFactors || diagnostics_.currentVisualFactors < 6) {
        diagnostics_.reason = diagnostics_.rejectedImuFactors ? "invalid_or_uncovered_imu_interval" :
                              "insufficient_current_correspondences";
        return false;
    }

    // Solver options: lightweight (5 iter, 10ms)
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 1;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.use_explicit_schur_complement = true;
    options.logging_type = ceres::SILENT;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = utility::g_config.pnp.pnp_max_iterations;
    options.max_solver_time_in_seconds = utility::g_config.pnp.pnp_solver_time;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    diagnostics_.iterations = static_cast<int>(summary.iterations.size());
    diagnostics_.terminationType = static_cast<int>(summary.termination_type);
    diagnostics_.initialCost = summary.initial_cost;
    diagnostics_.finalCost = summary.final_cost;
    diagnostics_.usable = summary.IsSolutionUsable() && std::isfinite(summary.initial_cost) &&
                          std::isfinite(summary.final_cost);
    for (int i = 0; i <= kPnPSize; ++i) {
        for (double value : para_Pose_[i]) if (!std::isfinite(value)) diagnostics_.usable = false;
        for (double value : para_Speed_[i]) if (!std::isfinite(value)) diagnostics_.usable = false;
    }
    diagnostics_.reason = diagnostics_.usable ? "usable" : "unusable_solver_result";
    if (!diagnostics_.usable) return false;
    new2old();
    return true;
}

void PnPFrontend::slideWindow() {
    if (frame_count_ == kPnPSize) {
        for (int i = 0; i < kPnPSize; i++) {
            Rs_[i] = Rs_[i + 1];
            Ps_[i] = Ps_[i + 1];
            Vs_[i] = Vs_[i + 1];
            std::swap(pre_integrations_[i], pre_integrations_[i + 1]);
            dt_buf_[i].swap(dt_buf_[i + 1]);
            acc_buf_[i].swap(acc_buf_[i + 1]);
            gyr_buf_[i].swap(gyr_buf_[i + 1]);
            headers_[i] = headers_[i + 1];
            std::swap(features_[i], features_[i + 1]);
            find_solved_[i] = find_solved_[i + 1];
        }
        headers_[kPnPSize] = headers_[kPnPSize - 1];
        Ps_[kPnPSize] = Ps_[kPnPSize - 1];
        Vs_[kPnPSize] = Vs_[kPnPSize - 1];
        Rs_[kPnPSize] = Rs_[kPnPSize - 1];
        Bas_[kPnPSize] = Bas_[kPnPSize - 1];
        Bgs_[kPnPSize] = Bgs_[kPnPSize - 1];
        find_solved_[kPnPSize] = false;

        pre_integrations_[kPnPSize] =
            std::make_unique<backend::factor::IntegrationBase>(acc_0_, gyr_0_, Bas_[kPnPSize], Bgs_[kPnPSize]);

        features_[kPnPSize].clear();
        dt_buf_[kPnPSize].clear();
        acc_buf_[kPnPSize].clear();
        gyr_buf_[kPnPSize].clear();
    }
}

Eigen::Vector3d PnPFrontend::getPosition() const {
    return Ps_[kPnPSize - 1];
}

Eigen::Matrix3d PnPFrontend::getRotation() const {
    return Rs_[kPnPSize - 1];
}

Eigen::Vector3d PnPFrontend::getVelocity() const {
    return Vs_[kPnPSize - 1];
}

bool PnPFrontend::hasPose() const {
    return has_pose_;
}

}  // namespace frontend
