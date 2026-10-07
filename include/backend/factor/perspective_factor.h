#ifndef BACKEND__FACTOR__PERSPECTIVE_FACTOR_H
#define BACKEND__FACTOR__PERSPECTIVE_FACTOR_H

#include <Eigen/Dense>
#include <ceres/ceres.h>

namespace backend {
namespace factor {

// PerspectiveFactor: Known 3D point → 2D observation residual
// Used by PnP Frontend for motion-only tracking (VINS-Mobile pattern).
// Unlike ProjectionFactor, this does NOT optimize feature depth —
// the 3D position is known from the backend and treated as observation.
//
// Parameter blocks: Pose_i(7), Ex_Pose(7)
// Residual: 2D reprojection error weighted by track_num
//
// Reference: assets/references/VINS-Mobile/VINS_ios/perspective_factor.{hpp,cpp}
class PerspectiveFactor : public ceres::SizedCostFunction<2, 7, 7> {
public:
    PerspectiveFactor(const Eigen::Vector2d& pts_2d,
                      const Eigen::Vector3d& pts_3d,
                      int track_num);

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const;

    Eigen::Vector3d pts_3d_;
    Eigen::Vector2d pts_2d_;
    int track_num_;
    static Eigen::Matrix2d sqrt_info;
};

}  // namespace factor
}  // namespace backend

#endif  // BACKEND__FACTOR__PERSPECTIVE_FACTOR_H
