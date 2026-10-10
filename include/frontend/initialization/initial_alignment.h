#ifndef FRONTEND__INITIALIZATION__INITIAL_ALIGNMENT_H
#define FRONTEND__INITIALIZATION__INITIAL_ALIGNMENT_H

#include <Eigen/Dense>
#include <iostream>
#include <map>

#include "backend/factor/imu_factor.h"
#include "backend/sliding_window.h"
#include "common/image_frame.h"
#include "frontend/feature_manager.h"
#include "utility/utility.h"

using namespace Eigen;
using namespace std;

namespace frontend {
namespace initialization {

bool CheckIMUExcitation(const std::map<double, common::ImageFrame>& frames, double threshold);
MatrixXd TangentBasis(Vector3d& gravity);
bool RefineGravity(const std::map<double, common::ImageFrame>& frames, Vector3d& gravity, VectorXd& state);
bool LinearAlignment(const std::map<double, common::ImageFrame>& frames, Vector3d& gravity, VectorXd& state);

bool VisualIMUAlignment(std::map<double, common::ImageFrame>& all_image_frame,
                        backend::SlidingWindow& sliding_window, Vector3d& g, VectorXd& x);

}  // namespace initialization
}  // namespace frontend

#endif  // FRONTEND__INITIALIZATION__INITIAL_ALIGNMENT_H
