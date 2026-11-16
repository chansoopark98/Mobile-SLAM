#ifndef BACKEND__FACTOR__POSE_LOCAL_PARAMETERIZATION_H
#define BACKEND__FACTOR__POSE_LOCAL_PARAMETERIZATION_H

#include <ceres/ceres.h>
#include <Eigen/Dense>

#include "utility/utility.h"

namespace backend {
namespace factor {

class PoseLocalParameterization : public ceres::Manifold {
public:
    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override;
    bool PlusJacobian(const double* x, double* jacobian) const override;
    bool Minus(const double* y, const double* x, double* y_minus_x) const override;
    bool MinusJacobian(const double* x, double* jacobian) const override;
    int AmbientSize() const override { return 7; }
    int TangentSize() const override { return 6; }
};

}  // namespace factor
}  // namespace backend

#endif  // BACKEND__FACTOR__POSE_LOCAL_PARAMETERIZATION_H
