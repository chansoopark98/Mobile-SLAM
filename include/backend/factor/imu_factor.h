#ifndef BACKEND__FACTOR__IMU_FACTOR_H
#define BACKEND__FACTOR__IMU_FACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <iostream>

#include "integration_base.h"
#include "covariance_whitening.h"
#include "utility/config.h"
#include "utility/utility.h"

namespace backend {
namespace factor {

class IMUFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9> {
public:
    IMUFactor() = delete;
    explicit IMUFactor(IntegrationBase* _pre_integration) : pre_integration(_pre_integration) {
        valid_ = pre_integration && pre_integration->hasUsableInterval() &&
                 covarianceWhitening(pre_integration->covariance, sqrt_info_);
    }
    bool isValid() const { return valid_; }
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
        if (!valid_) return false;
        const int sizes[] = {7, 9, 7, 9};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < sizes[i]; ++j)
                if (!std::isfinite(parameters[i][j])) return false;
        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
        Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

        Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

        Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
        Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
        Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

        Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
        residual = pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi, Pj, Qj, Vj, Baj, Bgj);

        const auto& sqrt_info = sqrt_info_;
        residual = sqrt_info * residual;
        if (!residual.allFinite()) return false;

        if (jacobians) {
            double sum_dt = pre_integration->sum_dt;
            Eigen::Matrix3d dp_dba = pre_integration->jacobian.template block<3, 3>(O_P, O_BA);
            Eigen::Matrix3d dp_dbg = pre_integration->jacobian.template block<3, 3>(O_P, O_BG);

            Eigen::Matrix3d dq_dbg = pre_integration->jacobian.template block<3, 3>(O_R, O_BG);

            Eigen::Matrix3d dv_dba = pre_integration->jacobian.template block<3, 3>(O_V, O_BA);
            Eigen::Matrix3d dv_dbg = pre_integration->jacobian.template block<3, 3>(O_V, O_BG);
            const Eigen::Quaterniond corrected_delta_q =
                pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
            const double orientation_sign = IntegrationBase::canonicalOrientationErrorSign(
                corrected_delta_q.inverse() * (Qi.inverse() * Qj));

            if (pre_integration->jacobian.maxCoeff() > 1e8 || pre_integration->jacobian.minCoeff() < -1e8) {
#ifndef NDEBUG
                std::cout << "numerical unstable in preintegration" << std::endl;
#endif
            }

            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
                jacobian_pose_i.setZero();

                jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
                jacobian_pose_i.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(
                    Qi.inverse() * (0.5 * g_config.estimator.g * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

                jacobian_pose_i.block<3, 3>(O_R, O_R) =
                    -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
                jacobian_pose_i.block<3, 3>(O_R, O_R) *= orientation_sign;

                jacobian_pose_i.block<3, 3>(O_V, O_R) =
                    Utility::skewSymmetric(Qi.inverse() * (g_config.estimator.g * sum_dt + Vj - Vi));

                jacobian_pose_i = sqrt_info * jacobian_pose_i;

                if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8) {
    #ifndef NDEBUG
                std::cout << "numerical unstable in preintegration" << std::endl;
#endif
                }
            }
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
                jacobian_speedbias_i.setZero();
                jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;

                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) =
                    -Utility::Qleft(Qj.inverse() * Qi * pre_integration->delta_q).bottomRightCorner<3, 3>() * dq_dbg;
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) *= orientation_sign;

                jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
                jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
                jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;

                jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;
            }
            if (jacobians[2]) {
                Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
                jacobian_pose_j.setZero();

                jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

                jacobian_pose_j.block<3, 3>(O_R, O_R) =
                    Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
                jacobian_pose_j.block<3, 3>(O_R, O_R) *= orientation_sign;

                jacobian_pose_j = sqrt_info * jacobian_pose_j;
            }
            if (jacobians[3]) {
                Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[3]);
                jacobian_speedbias_j.setZero();

                jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();

                jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;
            }
        }

        if (jacobians) {
            for (int block = 0; block < 4; ++block)
                if (jacobians[block] && !Eigen::Map<const Eigen::VectorXd>(jacobians[block], 15 * sizes[block]).allFinite())
                    return false;
        }
        return true;
    }

    IntegrationBase* pre_integration;

private:
    Eigen::Matrix<double, 15, 15> sqrt_info_;
    bool valid_ = false;
};

}  // namespace factor
}  // namespace backend

#endif  // BACKEND__FACTOR__IMU_FACTOR_H
