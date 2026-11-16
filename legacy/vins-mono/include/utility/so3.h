#ifndef UTILITY_SO3_H
#define UTILITY_SO3_H

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include "utility/utility.h"

namespace utility::so3 {

inline bool isRotation(const Eigen::Matrix3d& rotation) {
    return rotation.allFinite() &&
        (rotation.transpose()*rotation-Eigen::Matrix3d::Identity()).norm() <= 1e-6 &&
        std::abs(rotation.determinant()-1) <= 1e-6;
}

inline bool normalize(const Eigen::Quaterniond& raw, Eigen::Quaterniond& unit, double& norm) {
    if (!raw.coeffs().allFinite()) return false;
    norm = raw.coeffs().stableNorm();
    if (!std::isfinite(norm) || !(norm > 0)) return false;
    unit.coeffs() = raw.coeffs() / norm;
    return true;
}

inline double canonicalSign(const Eigen::Quaterniond& value) {
    if (value.w() != 0) return value.w() < 0 ? -1 : 1;
    // The pi cut is not differentiable. Choose the same physical axis for q/-q.
    for (int axis = 0; axis < 3; ++axis)
        if (value.vec()[axis] != 0) return value.vec()[axis] < 0 ? -1 : 1;
    return 1;
}

inline Eigen::Quaterniond exp(const Eigen::Vector3d& tangent) {
    const double angle = tangent.stableNorm();
    if (!tangent.allFinite() || !std::isfinite(angle)) {
        const double invalid = std::numeric_limits<double>::quiet_NaN();
        return {invalid,invalid,invalid,invalid};
    }
    const double squared = angle * angle;
    const double scale = angle < 1e-4 ? .5 - squared/48 + squared*squared/3840 : std::sin(angle/2)/angle;
    const double real = angle < 1e-4 ? 1 - squared/8 + squared*squared/384 : std::cos(angle/2);
    return Eigen::Quaterniond(real,scale*tangent.x(),scale*tangent.y(),scale*tangent.z()).normalized();
}

inline Eigen::Vector3d log(const Eigen::Quaterniond& unit) {
    const double sign = canonicalSign(unit);
    const Eigen::Vector3d imaginary = sign * unit.vec();
    const double magnitude = imaginary.stableNorm();
    if (magnitude == 0) return Eigen::Vector3d::Zero();
    return (2 * std::atan2(magnitude,sign*unit.w()) / magnitude) * imaginary;
}

inline Eigen::Matrix3d rightJacobian(const Eigen::Vector3d& tangent) {
    const double squared = tangent.squaredNorm();
    const double angle = std::sqrt(squared);
    const double a = angle < 1e-4 ? .5 - squared/24 + squared*squared/720 : (1-std::cos(angle))/squared;
    const double b = angle < 1e-4 ? 1./6 - squared/120 + squared*squared/5040 : (angle-std::sin(angle))/(squared*angle);
    const Eigen::Matrix3d cross = Utility::skewSymmetric(tangent);
    return Eigen::Matrix3d::Identity() - a*cross + b*cross*cross;
}

inline Eigen::Matrix<double,7,6> posePlusJacobian(const Eigen::Quaterniond& unit) {
    Eigen::Matrix<double,7,6> result = Eigen::Matrix<double,7,6>::Zero();
    result.topLeftCorner<3,3>().setIdentity();
    result.block<3,3>(3,3) = .5 * (unit.w()*Eigen::Matrix3d::Identity() + Utility::skewSymmetric(unit.vec()));
    result.block<1,3>(6,3) = -.5 * unit.vec().transpose();
    return result;
}

// Maps ambient pose perturbations to the world-translation/right-rotation tangent
// of the normalized quaternion residual extension. At unit states L*PlusJac=I.
inline Eigen::Matrix<double,6,7> poseLift(const Eigen::Quaterniond& unit, double raw_norm) {
    Eigen::Matrix<double,6,7> result = Eigen::Matrix<double,6,7>::Zero();
    result.topLeftCorner<3,3>().setIdentity();
    result.block<3,4>(3,3) = (4/raw_norm) * posePlusJacobian(unit).block<4,3>(3,3).transpose();
    return result;
}

}  // namespace utility::so3
#endif
