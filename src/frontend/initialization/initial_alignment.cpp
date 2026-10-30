#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "frontend/initialization/initial_alignment.h"
#include "utility/config.h"

namespace frontend {
namespace initialization {

namespace {
bool solveGyroscopeBias(std::map<double, common::ImageFrame> const& all_image_frame, Vector3d& bias) {
    Matrix3d A;
    Vector3d b;
    Vector3d delta_bg;
    A.setZero();
    b.setZero();
    std::map<double, common::ImageFrame>::const_iterator frame_i;
    std::map<double, common::ImageFrame>::const_iterator frame_j;

    // construct linear system to get the gyroscope bias delta
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++) {
        frame_j = next(frame_i);
        MatrixXd tmp_A(3, 3);
        tmp_A.setZero();
        VectorXd tmp_b(3);
        tmp_b.setZero();
        Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
        tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
        tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
    }
    // Check condition number via SVD before solving
    Eigen::JacobiSVD<Matrix3d> svd(A);
    double cond = svd.singularValues()(0) / svd.singularValues()(2);
    if (cond > 1e10 || !std::isfinite(cond)) {
#ifndef NDEBUG
        std::cout << "Gyroscope bias A matrix ill-conditioned (cond=" << cond << "), using zero bias" << std::endl;
#endif
        delta_bg.setZero();
    } else {
        delta_bg = A.ldlt().solve(b);
    }
#ifndef NDEBUG
    std::cout << "gyroscope bias initial calibration " << delta_bg.transpose() << std::endl;
#endif

    if (!delta_bg.allFinite()) return false;

    // Clamp gyroscope bias to physically reasonable range (±0.35 rad/s per axis)
    // Mobile MEMS gyroscopes can have bias 0.2-0.3 rad/s (11-17 deg/s).
    // Previous 0.05 was 7x too tight → truncated legitimate bias → pre-integration contamination.
    const double max_bias = 0.35;
    for (int axis = 0; axis < 3; axis++) {
        delta_bg(axis) = std::max(-max_bias, std::min(max_bias, delta_bg(axis)));
    }
#ifndef NDEBUG
    std::cout << "gyroscope bias clamped to " << delta_bg.transpose() << std::endl;
#endif

    bias += delta_bg;
    if (!bias.allFinite()) return false;

    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++) {
        frame_j = next(frame_i);
        frame_j->second.pre_integration->repropagate(Vector3d::Zero(), bias);
    }
    return true;
}

bool validAlignmentFrames(const std::map<double, common::ImageFrame>& frames) {
    if (frames.size() < 2 || !g_config.camera.t_ic.allFinite() ||
        !g_config.estimator.g.allFinite() || !std::isfinite(g_config.estimator.g.norm()) ||
        !(g_config.estimator.g.norm() > 0)) return false;
    for (auto frame = frames.begin(); frame != frames.end(); ++frame) {
        const auto& value = frame->second;
        if (!std::isfinite(frame->first) || !std::isfinite(value.t) ||
            !value.R.allFinite() || !value.T.allFinite() ||
            (value.R.transpose()*value.R-Matrix3d::Identity()).norm() > 1e-6 ||
            std::abs(value.R.determinant()-1) > 1e-6) return false;
        if (frame == frames.begin()) continue;
        const auto& pre = value.pre_integration;
        const double dt = frame->first - std::prev(frame)->first;
        if (!pre || !std::isfinite(pre->sum_dt) || !(pre->sum_dt > 0) ||
            std::abs(pre->sum_dt-dt) > 1e-6 || !pre->delta_p.allFinite() ||
            !pre->delta_v.allFinite() || !pre->delta_q.coeffs().allFinite() ||
            std::abs(pre->delta_q.squaredNorm()-1) > 1e-6 || !pre->jacobian.allFinite()) return false;
    }
    return true;
}

bool solveAlignmentSystem(const MatrixXd& matrix, const VectorXd& rhs, double rhs_reference, VectorXd& state, double& scale_resolution) {
    if (matrix.rows() < matrix.cols() || matrix.rows() != rhs.size() ||
        !matrix.allFinite() || !rhs.allFinite()) return false;
    VectorXd column_norms(matrix.cols());
    MatrixXd normalized = matrix;
    for (int column = 0; column < matrix.cols(); ++column) {
        column_norms[column] = matrix.col(column).stableNorm();
        if (!std::isfinite(column_norms[column]) || !(column_norms[column] > 0)) return false;
        normalized.col(column) /= column_norms[column];
    }
    const double relative_tolerance = std::numeric_limits<double>::epsilon() * std::max(matrix.rows(),matrix.cols());
    Eigen::JacobiSVD<MatrixXd> svd(normalized,Eigen::ComputeThinU|Eigen::ComputeThinV);
    svd.setThreshold(relative_tolerance);
    if (svd.info() != Eigen::Success || !svd.singularValues().allFinite() || svd.rank() != matrix.cols()) return false;
    const VectorXd scaled_state = svd.solve(rhs);
    if (!scaled_state.allFinite()) return false;
    const VectorXd residual = normalized*scaled_state-rhs;
    const VectorXd gradient = normalized.transpose()*residual;
    const double roundoff = std::numeric_limits<double>::epsilon() * (matrix.rows()+matrix.cols()) *
                            (normalized.norm()*scaled_state.norm()+rhs.norm()+rhs_reference);
    if (!residual.allFinite() || !gradient.allFinite() || !std::isfinite(roundoff) ||
        gradient.norm() > normalized.norm()*roundoff) return false;
    state = scaled_state.cwiseQuotient(column_norms);
    scale_resolution = roundoff / svd.singularValues().tail<1>()[0] / column_norms.tail<1>()[0];
    return state.allFinite() && std::isfinite(scale_resolution);
}
}

bool CheckIMUExcitation(const std::map<double, common::ImageFrame>& frames, double threshold) {
    if (frames.size() < 3 || !std::isfinite(threshold) || threshold < 0) return false;
    const auto validRotation = [](const Matrix3d& rotation) {
        return rotation.allFinite() &&
               (rotation.transpose()*rotation-Matrix3d::Identity()).norm() <= 1e-6 &&
               std::abs(rotation.determinant()-1) <= 1e-6;
    };
    std::vector<Vector3d> common_force;
    Vector3d sum = Vector3d::Zero();
    for (auto previous = frames.begin(), current = std::next(previous);
         current != frames.end(); ++previous,++current) {
        const auto& pre = current->second.pre_integration;
        const double interval = current->first-previous->first;
        if (!std::isfinite(previous->first) || !std::isfinite(current->first) ||
            !validRotation(previous->second.R) || !validRotation(current->second.R) ||
            !pre || !std::isfinite(pre->sum_dt) || !(pre->sum_dt > 0) ||
            !std::isfinite(interval) || std::abs(pre->sum_dt-interval) > 1e-6 || !pre->delta_v.allFinite()) return false;
        // delta_v is expressed in the interval's starting body frame, not the ending frame.
        const Vector3d force = previous->second.R*(pre->delta_v/pre->sum_dt);
        if (!force.allFinite()) return false;
        common_force.push_back(force);
        sum += force;
    }
    const Vector3d mean = sum/common_force.size();
    if (!mean.allFinite()) return false;
    double variance = 0;
    for (const auto& force : common_force) variance += (force-mean).squaredNorm();
    const double std_deviation = std::sqrt(variance/(common_force.size()-1));
    if (!std::isfinite(std_deviation)) return false;

#ifndef NDEBUG
    std::cout << "IMU excitation check: std_deviation = " << std_deviation << ", threshold = " << threshold
              << std::endl;
#endif

    if (std_deviation < threshold) {
#ifndef NDEBUG
        std::cout << "IMU excitation not enough! Please vary translational "
                     "acceleration."
                  << std::endl;
#endif
        return false;
    } else {
#ifndef NDEBUG
        std::cout << "IMU excitation sufficient for initialization." << std::endl;
#endif
        return true;
    }
}

MatrixXd TangentBasis(Vector3d& g0) {
    if (!g0.allFinite() || !std::isfinite(g0.norm()) || !(g0.norm() > 0)) return MatrixXd::Zero(3,2);
    const Vector3d a = g0.normalized();
    Eigen::Index axis;
    a.cwiseAbs().minCoeff(&axis);
    const Vector3d reference = Vector3d::Unit(axis);
    const Vector3d b = (reference-a*a.dot(reference)).normalized();
    MatrixXd basis(3,2);
    basis.col(0) = b;
    basis.col(1) = a.cross(b);
    return basis;
}

// reference: Formula Derivation and Analysis of the VINS-Mono, pp. 11, eq (33)
// https://arxiv.org/abs/1912.11986
bool RefineGravity(std::map<double, common::ImageFrame> const& all_image_frame, Vector3d& g, VectorXd& x) {
    if (!validAlignmentFrames(all_image_frame) || !g.allFinite() || !std::isfinite(g.norm()) || !(g.norm() > 0)) return false;
    Vector3d g0 = g.normalized()*g_config.estimator.g.norm();
    const int all_frame_count = all_image_frame.size();
    const int n_state = all_frame_count*3+3;
    VectorXd candidate;
    double scale_resolution = 0;
    for (int k = 0; k < 4; k++) {
        const MatrixXd lxly = TangentBasis(g0);
        MatrixXd matrix = MatrixXd::Zero(6*(all_frame_count-1),n_state);
        VectorXd rhs = VectorXd::Zero(matrix.rows());
        double rhs_reference = 0;
        int i = 0;
        for (auto frame_i = all_image_frame.begin(); std::next(frame_i) != all_image_frame.end(); ++frame_i,++i) {
            const auto frame_j = std::next(frame_i);
            const Matrix3d rotation = frame_i->second.R.transpose();
            const auto& pre = *frame_j->second.pre_integration;
            const double dt = pre.sum_dt;
            const int row = 6*i;
            matrix.block<3,3>(row,3*i) = -dt*Matrix3d::Identity();
            matrix.block<3,2>(row,n_state-3) = rotation*(dt*dt/2)*lxly;
            matrix.block<3,1>(row,n_state-1) = rotation*(frame_j->second.T-frame_i->second.T)/100;
            rhs.segment<3>(row) = pre.delta_p+rotation*frame_j->second.R*g_config.camera.t_ic-g_config.camera.t_ic-rotation*(dt*dt/2)*g0;
            matrix.block<3,3>(row+3,3*i) = -Matrix3d::Identity();
            matrix.block<3,3>(row+3,3*(i+1)) = rotation*frame_j->second.R;
            matrix.block<3,2>(row+3,n_state-3) = rotation*dt*lxly;
            rhs.segment<3>(row+3) = pre.delta_v-rotation*dt*g0;
            // Preserve the operand scale when gravity subtraction nearly cancels the RHS.
            rhs_reference += pre.delta_p.norm()+2*g_config.camera.t_ic.norm()+pre.delta_v.norm()+(dt*dt/2+dt)*g0.norm();
        }
        if (!solveAlignmentSystem(matrix,rhs,rhs_reference,candidate,scale_resolution)) return false;
        const Vector3d updated = g0+lxly*candidate.segment<2>(n_state-3);
        if (!updated.allFinite() || !std::isfinite(updated.norm()) || !(updated.norm() > 0)) return false;
        g0 = updated.normalized()*g_config.estimator.g.norm();
    }
    // Certify a positive scale above floating-point solve resolution, without a metric cap.
    if (!(candidate.tail<1>()[0] > scale_resolution) || !g0.allFinite()) return false;
    x = candidate;
    g = g0;
    return true;
}

// reference: Formula Derivation and Analysis of the VINS-Mono, pp. 11, eq (31)
// https://arxiv.org/abs/1912.11986
bool LinearAlignment(std::map<double, common::ImageFrame> const& all_image_frame, Vector3d& g, VectorXd& x) {
    if (!validAlignmentFrames(all_image_frame)) return false;
    const int all_frame_count = all_image_frame.size();
    const int n_state = all_frame_count*3+4;
    MatrixXd matrix = MatrixXd::Zero(6*(all_frame_count-1),n_state);
    VectorXd rhs = VectorXd::Zero(matrix.rows());
    double rhs_reference = 0;
    int i = 0;
    for (auto frame_i = all_image_frame.begin(); std::next(frame_i) != all_image_frame.end(); ++frame_i,++i) {
        const auto frame_j = std::next(frame_i);
        const Matrix3d rotation = frame_i->second.R.transpose();
        const auto& pre = *frame_j->second.pre_integration;
        const double dt = pre.sum_dt;
        const int row = 6*i;
        matrix.block<3,3>(row,3*i) = -dt*Matrix3d::Identity();
        matrix.block<3,3>(row,n_state-4) = rotation*(dt*dt/2);
        matrix.block<3,1>(row,n_state-1) = rotation*(frame_j->second.T-frame_i->second.T)/100;
        rhs.segment<3>(row) = pre.delta_p+rotation*frame_j->second.R*g_config.camera.t_ic-g_config.camera.t_ic;
        matrix.block<3,3>(row+3,3*i) = -Matrix3d::Identity();
        matrix.block<3,3>(row+3,3*(i+1)) = rotation*frame_j->second.R;
        matrix.block<3,3>(row+3,n_state-4) = rotation*dt;
        rhs.segment<3>(row+3) = pre.delta_v;
        rhs_reference += pre.delta_p.norm()+2*g_config.camera.t_ic.norm()+pre.delta_v.norm();
    }
    VectorXd candidate;
    double scale_resolution = 0;
    if (!solveAlignmentSystem(matrix,rhs,rhs_reference,candidate,scale_resolution) || !(candidate.tail<1>()[0] > scale_resolution)) return false;
    Vector3d candidate_gravity = candidate.segment<3>(n_state-4);
    if (!candidate_gravity.allFinite() || std::abs(candidate_gravity.norm()-g_config.estimator.g.norm()) > 2.5) return false;
    if (!RefineGravity(all_image_frame,candidate_gravity,candidate)) return false;
    candidate.tail<1>()[0] /= 100;
    if (!candidate.allFinite() || !(candidate.tail<1>()[0] > 0)) return false;
    x = candidate;
    g = candidate_gravity;
    return true;
}

bool VisualIMUAlignment(std::map<double, common::ImageFrame>& all_image_frame,
                        backend::SlidingWindow& sliding_window, Vector3d& g, VectorXd& x) {
    if (!validAlignmentFrames(all_image_frame)) return false;
    for (int i = 0; i <= WINDOW_SIZE; ++i)
        if (!sliding_window[i].Bg.allFinite()) return false;

    // Failed alignment must not change the next attempt's bias or measurements.
    // Copy only the alignment geometry and preintegrations, not the feature tracks.
    std::map<double, common::ImageFrame> candidates;
    Vector3d candidate_bias = sliding_window.front().Bg;
    for (auto frame = all_image_frame.begin(); frame != all_image_frame.end(); ++frame) {
        common::ImageFrame candidate({}, frame->second.t);
        candidate.R = frame->second.R;
        candidate.T = frame->second.T;
        if (frame != all_image_frame.begin()) {
            candidate.pre_integration = std::make_unique<backend::factor::IntegrationBase>(*frame->second.pre_integration);
            // tmp_pre_integration may have been created before a previous bias update.
            // A single bias increment is valid only from a common linearization point.
            if (!candidate.pre_integration->linearized_ba.isZero(0) ||
                !candidate.pre_integration->linearized_bg.isApprox(candidate_bias, 0))
                candidate.pre_integration->repropagate(Vector3d::Zero(), candidate_bias);
        }
        candidates.emplace(frame->first, std::move(candidate));
    }
    Vector3d candidate_gravity;
    VectorXd candidate_state;
    if (!solveGyroscopeBias(candidates, candidate_bias) || !CheckIMUExcitation(candidates, 0.25) ||
        !LinearAlignment(candidates, candidate_gravity, candidate_state)) return false;

    const Vector3d bias_delta = candidate_bias - sliding_window.front().Bg;
    for (int i = 0; i <= WINDOW_SIZE; ++i) sliding_window[i].Bg += bias_delta;
    for (auto frame = std::next(all_image_frame.begin()); frame != all_image_frame.end(); ++frame)
        frame->second.pre_integration.swap(candidates.at(frame->first).pre_integration);
    g = candidate_gravity;
    x = std::move(candidate_state);
    return true;
}
}  // namespace initialization
}  // namespace frontend
