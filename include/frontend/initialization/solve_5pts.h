#ifndef FRONTEND__INITIALIZATION__SOLVE_5PTS_H
#define FRONTEND__INITIALIZATION__SOLVE_5PTS_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

#include "frontend/feature_manager.h"

using namespace std;
using namespace Eigen;

namespace frontend {
namespace initialization {

class MotionEstimator {
public:
    bool solveRelativeRT(const frontend::Correspondences& corres, Matrix3d& R, Vector3d& T);
};

}  // namespace initialization
}  // namespace frontend

#endif  // FRONTEND__INITIALIZATION__SOLVE_5PTS_H
