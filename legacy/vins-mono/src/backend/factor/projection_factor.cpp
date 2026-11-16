#include "backend/factor/projection_factor.h"
#include <cmath>
#include "utility/so3.h"
#include "utility/utility.h"

namespace backend {
namespace factor {

Eigen::Matrix2d ProjectionFactor::sqrt_info;

ProjectionFactor::ProjectionFactor(const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j)
    : pts_i(_pts_i), pts_j(_pts_j) {}

bool ProjectionFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    if (!pts_i.allFinite() || !pts_j.allFinite() || pts_i.z() <= 1e-10 || pts_j.z() <= 1e-10 ||
        !sqrt_info.allFinite()) return false;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 7; ++j)
            if (!std::isfinite(parameters[i][j])) return false;
    const Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    const Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    const Eigen::Vector3d t_ic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond q_ic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
    const double rho = parameters[3][0];
    if (!std::isfinite(rho) || Qi.squaredNorm() <= 1e-12 || Qj.squaredNorm() <= 1e-12 ||
        q_ic.squaredNorm() <= 1e-12) return false;

    double norm_i,norm_j,norm_ic;
    if (!utility::so3::normalize(Qi,Qi,norm_i) || !utility::so3::normalize(Qj,Qj,norm_j) ||
        !utility::so3::normalize(q_ic,q_ic,norm_ic)) return false;

    const Eigen::Matrix3d Ri = Qi.toRotationMatrix(), Rj = Qj.toRotationMatrix(), ric = q_ic.toRotationMatrix();
    // Homogeneous projection is continuous through rho=0 (a point at infinity).
    // Signed solver trials are allowed; final landmark cheirality is checked separately.
    const Eigen::Vector3d scaled_imu_i = ric * pts_i + rho * t_ic;
    const Eigen::Vector3d scaled_imu_j = Rj.transpose() * (Ri * scaled_imu_i + rho * (Pi - Pj));
    const Eigen::Vector3d h = ric.transpose() * (scaled_imu_j - rho * t_ic);
    if (!h.allFinite() || h.z() == 0) return false;
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual = sqrt_info * ((h / h.z()).head<2>() - pts_j.head<2>());
    if (!residual.allFinite()) return false;

    if (jacobians) {
        const double inverse_z = 1 / h.z();
        Eigen::Matrix<double,2,3> reduce;
        reduce << inverse_z, 0, -(h.x() * inverse_z) * inverse_z,
                  0, inverse_z, -(h.y() * inverse_z) * inverse_z;
        reduce = sqrt_info * reduce;
        const Eigen::Vector3d translation = ric.transpose() * (Rj.transpose() * (Ri * t_ic + Pi - Pj) - t_ic);
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double,2,7,Eigen::RowMajor>> j(jacobians[0]);
            Eigen::Matrix<double,3,6> local;
            local.leftCols<3>() = rho * ric.transpose() * Rj.transpose();
            local.rightCols<3>() = -ric.transpose() * Rj.transpose() * Ri * Utility::skewSymmetric(scaled_imu_i);
            j = reduce * local * utility::so3::poseLift(Qi,norm_i);
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double,2,7,Eigen::RowMajor>> j(jacobians[1]);
            Eigen::Matrix<double,3,6> local;
            local.leftCols<3>() = -rho * ric.transpose() * Rj.transpose();
            local.rightCols<3>() = ric.transpose() * Utility::skewSymmetric(scaled_imu_j);
            j = reduce * local * utility::so3::poseLift(Qj,norm_j);
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double,2,7,Eigen::RowMajor>> j(jacobians[2]);
            Eigen::Matrix<double,3,6> local;
            const Eigen::Matrix3d relative = ric.transpose() * Rj.transpose() * Ri * ric;
            local.leftCols<3>() = rho * ric.transpose() * (Rj.transpose() * Ri - Eigen::Matrix3d::Identity());
            local.rightCols<3>() = -relative * Utility::skewSymmetric(pts_i) +
                                  Utility::skewSymmetric(relative * pts_i) + rho * Utility::skewSymmetric(translation);
            j = reduce * local * utility::so3::poseLift(q_ic,norm_ic);
        }
        if (jacobians[3]) {
            Eigen::Map<Eigen::Vector2d> j(jacobians[3]);
            j = reduce * translation;
        }
        const int sizes[] = {7,7,7,1};
        for (int block = 0; block < 4; ++block)
            if (jacobians[block] && !Eigen::Map<const Eigen::VectorXd>(jacobians[block],2*sizes[block]).allFinite())
                return false;
    }
    return true;
}

}  // namespace factor
}  // namespace backend
