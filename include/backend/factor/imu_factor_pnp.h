#ifndef BACKEND__FACTOR__IMU_FACTOR_PNP_H
#define BACKEND__FACTOR__IMU_FACTOR_PNP_H

#include <Eigen/Dense>
#include <ceres/ceres.h>

#include "backend/factor/integration_base.h"
#include "backend/factor/covariance_whitening.h"
#include "utility/config.h"
#include "utility/utility.h"

namespace backend {
namespace factor {

// IMUFactorPnP: IMU factor with Speed and Bias as SEPARATE parameter blocks.
// This allows Bias to be SetParameterBlockConstant (fixed from backend)
// while Speed remains free for optimization.
//
// Parameter blocks: Pose_i(7), Speed_i(3), Bias_i(6), Pose_j(7), Speed_j(3), Bias_j(6)
// Residual: 15-dimensional (same as IMUFactor)
//
// Reference: assets/references/VINS-Mobile/VINS_ios/imu_factor_pnp.h
class IMUFactorPnP : public ceres::SizedCostFunction<15, 7, 3, 6, 7, 3, 6> {
public:
    IMUFactorPnP() = delete;
    explicit IMUFactorPnP(IntegrationBase* pre_integration) : pre_integration_(pre_integration) {
        valid_ = pre_integration_ && pre_integration_->hasUsableInterval() &&
                 covarianceWhitening(pre_integration_->covariance, sqrt_info_);
    }
    bool isValid() const { return valid_; }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const {
        if (!valid_) return false;
        const int sizes[] = {7, 3, 6, 7, 3, 6};
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < sizes[i]; ++j)
                if (!std::isfinite(parameters[i][j])) return false;
        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);

        Eigen::Vector3d Bai(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Vector3d Bgi(parameters[2][3], parameters[2][4], parameters[2][5]);

        Eigen::Vector3d Pj(parameters[3][0], parameters[3][1], parameters[3][2]);
        Eigen::Quaterniond Qj(parameters[3][6], parameters[3][3], parameters[3][4], parameters[3][5]);

        Eigen::Vector3d Vj(parameters[4][0], parameters[4][1], parameters[4][2]);

        Eigen::Vector3d Baj(parameters[5][0], parameters[5][1], parameters[5][2]);
        Eigen::Vector3d Bgj(parameters[5][3], parameters[5][4], parameters[5][5]);

        Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
        residual = pre_integration_->evaluate(Pi, Qi, Vi, Bai, Bgi,
                                              Pj, Qj, Vj, Baj, Bgj);

        const auto& sqrt_info = sqrt_info_;
        residual = sqrt_info * residual;
        if (!residual.allFinite()) return false;

        if (jacobians) {
            double sum_dt = pre_integration_->sum_dt;
            Eigen::Matrix3d dp_dba = pre_integration_->jacobian.template block<3, 3>(O_P, O_BA);
            Eigen::Matrix3d dp_dbg = pre_integration_->jacobian.template block<3, 3>(O_P, O_BG);
            Eigen::Matrix3d dq_dbg = pre_integration_->jacobian.template block<3, 3>(O_R, O_BG);
            Eigen::Matrix3d dv_dba = pre_integration_->jacobian.template block<3, 3>(O_V, O_BA);
            Eigen::Matrix3d dv_dbg = pre_integration_->jacobian.template block<3, 3>(O_V, O_BG);
            const Eigen::Quaterniond corrected_delta_q =
                pre_integration_->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration_->linearized_bg));
            const double orientation_sign = IntegrationBase::canonicalOrientationErrorSign(
                corrected_delta_q.inverse() * (Qi.inverse() * Qj));

            Eigen::Vector3d G = utility::g_config.estimator.g;

            // jacobians[0]: d_residual / d_Pose_i (15x7)
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
                jacobian_pose_i.setZero();

                jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
                jacobian_pose_i.block<3, 3>(O_P, O_R) =
                    Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

                jacobian_pose_i.block<3, 3>(O_R, O_R) =
                    -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q))
                         .bottomRightCorner<3, 3>();
                jacobian_pose_i.block<3, 3>(O_R, O_R) *= orientation_sign;

                jacobian_pose_i.block<3, 3>(O_V, O_R) =
                    Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));

                jacobian_pose_i = sqrt_info * jacobian_pose_i;
            }

            // jacobians[1]: d_residual / d_Speed_i (15x3)
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_speed_i(jacobians[1]);
                jacobian_speed_i.setZero();
                jacobian_speed_i.block<3, 3>(O_P, 0) = -Qi.inverse().toRotationMatrix() * sum_dt;
                jacobian_speed_i.block<3, 3>(O_V, 0) = -Qi.inverse().toRotationMatrix();
                jacobian_speed_i = sqrt_info * jacobian_speed_i;
            }

            // jacobians[2]: d_residual / d_Bias_i (15x6)
            if (jacobians[2]) {
                Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>> jacobian_bias_i(jacobians[2]);
                jacobian_bias_i.setZero();
                jacobian_bias_i.block<3, 3>(O_P, 0) = -dp_dba;
                jacobian_bias_i.block<3, 3>(O_P, 3) = -dp_dbg;

                jacobian_bias_i.block<3, 3>(O_R, 3) =
                    -Utility::Qleft(Qj.inverse() * Qi * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
                jacobian_bias_i.block<3, 3>(O_R, 3) *= orientation_sign;

                jacobian_bias_i.block<3, 3>(O_V, 0) = -dv_dba;
                jacobian_bias_i.block<3, 3>(O_V, 3) = -dv_dbg;
                jacobian_bias_i.block<3, 3>(O_BA, 0) = -Eigen::Matrix3d::Identity();
                jacobian_bias_i.block<3, 3>(O_BG, 3) = -Eigen::Matrix3d::Identity();
                jacobian_bias_i = sqrt_info * jacobian_bias_i;
            }

            // jacobians[3]: d_residual / d_Pose_j (15x7)
            if (jacobians[3]) {
                Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[3]);
                jacobian_pose_j.setZero();

                jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

                jacobian_pose_j.block<3, 3>(O_R, O_R) =
                    Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
                jacobian_pose_j.block<3, 3>(O_R, O_R) *= orientation_sign;

                jacobian_pose_j = sqrt_info * jacobian_pose_j;
            }

            // jacobians[4]: d_residual / d_Speed_j (15x3)
            if (jacobians[4]) {
                Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_speed_j(jacobians[4]);
                jacobian_speed_j.setZero();
                jacobian_speed_j.block<3, 3>(O_V, 0) = Qi.inverse().toRotationMatrix();
                jacobian_speed_j = sqrt_info * jacobian_speed_j;
            }

            // jacobians[5]: d_residual / d_Bias_j (15x6)
            if (jacobians[5]) {
                Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>> jacobian_bias_j(jacobians[5]);
                jacobian_bias_j.setZero();
                jacobian_bias_j.block<3, 3>(O_BA, 0) = Eigen::Matrix3d::Identity();
                jacobian_bias_j.block<3, 3>(O_BG, 3) = Eigen::Matrix3d::Identity();
                jacobian_bias_j = sqrt_info * jacobian_bias_j;
            }
        }

        if (jacobians) {
            for (int block = 0; block < 6; ++block)
                if (jacobians[block] && !Eigen::Map<const Eigen::VectorXd>(jacobians[block], 15 * sizes[block]).allFinite())
                    return false;
        }
        return true;
    }

    IntegrationBase* pre_integration_;

private:
    Eigen::Matrix<double, 15, 15> sqrt_info_;
    bool valid_ = false;
};

}  // namespace factor
}  // namespace backend

#endif  // BACKEND__FACTOR__IMU_FACTOR_PNP_H
