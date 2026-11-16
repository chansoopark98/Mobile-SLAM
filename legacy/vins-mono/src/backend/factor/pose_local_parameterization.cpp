#include "backend/factor/pose_local_parameterization.h"
#include "utility/so3.h"

namespace backend::factor {

bool PoseLocalParameterization::Plus(const double* x, const double* delta, double* output) const {
    const Eigen::Map<const Eigen::Matrix<double,6,1>> increment(delta);
    const Eigen::Map<const Eigen::Vector3d> position(x);
    Eigen::Quaterniond q;double norm;
    if (!position.allFinite() || !increment.allFinite() ||
        !utility::so3::normalize(Eigen::Map<const Eigen::Quaterniond>(x+3),q,norm)) return false;
    const Eigen::Vector3d translated = position + increment.head<3>();
    const Eigen::Quaterniond rotated = (q * utility::so3::exp(increment.tail<3>())).normalized();
    if (!translated.allFinite() || !rotated.coeffs().allFinite()) return false;
    Eigen::Map<Eigen::Vector3d> translated_output(output);translated_output = translated;
    Eigen::Map<Eigen::Quaterniond> rotation(output+3);rotation = rotated;
    return true;
}

bool PoseLocalParameterization::PlusJacobian(const double* x, double* jacobian) const {
    Eigen::Quaterniond q;double norm;
    if (!Eigen::Map<const Eigen::Vector3d>(x).allFinite() ||
        !utility::so3::normalize(Eigen::Map<const Eigen::Quaterniond>(x+3),q,norm)) return false;
    Eigen::Map<Eigen::Matrix<double,7,6,Eigen::RowMajor>> mapped(jacobian);mapped = utility::so3::posePlusJacobian(q);
    return mapped.allFinite();
}

bool PoseLocalParameterization::Minus(const double* y, const double* x, double* output) const {
    const Eigen::Vector3d translation = Eigen::Map<const Eigen::Vector3d>(y) - Eigen::Map<const Eigen::Vector3d>(x);
    Eigen::Quaterniond qx,qy;double nx,ny;
    if (!translation.allFinite() ||
        !utility::so3::normalize(Eigen::Map<const Eigen::Quaterniond>(x+3),qx,nx) ||
        !utility::so3::normalize(Eigen::Map<const Eigen::Quaterniond>(y+3),qy,ny)) return false;
    const Eigen::Vector3d rotation = utility::so3::log((qx.conjugate()*qy).normalized());
    if (!rotation.allFinite()) return false;
    Eigen::Map<Eigen::Vector3d> translated_output(output),rotated_output(output+3);
    translated_output = translation;rotated_output = rotation;
    return true;
}

bool PoseLocalParameterization::MinusJacobian(const double* x, double* jacobian) const {
    Eigen::Quaterniond q;double norm;
    if (!Eigen::Map<const Eigen::Vector3d>(x).allFinite() ||
        !utility::so3::normalize(Eigen::Map<const Eigen::Quaterniond>(x+3),q,norm)) return false;
    Eigen::Map<Eigen::Matrix<double,6,7,Eigen::RowMajor>> mapped(jacobian);mapped = utility::so3::poseLift(q,norm);
    return mapped.allFinite();
}

}  // namespace backend::factor
