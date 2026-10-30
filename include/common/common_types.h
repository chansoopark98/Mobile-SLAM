#ifndef COMMON__COMMON_TYPES_H
#define COMMON__COMMON_TYPES_H

#include <Eigen/Dense>
#include <vector>

namespace common {

enum SolverFlag { INITIAL = 0, NON_LINEAR = 1 };
enum MarginalizationFlag { MARGIN_OLD_KEYFRAME = 0, MARGIN_NEW_GENERAL_FRAME = 1 };

// Data passed from Backend to PnP Frontend (VINS-Mobile: IMG_MSG_LOCAL)
struct SolvedFeature {
    int id;
    Eigen::Vector2d observation;   // normalized 2D point in current frame
    Eigen::Vector3d position;      // world 3D position (from triangulation)
    int track_num;                 // tracking length (for PerspectiveFactor weighting)
};

// Backend solution snapshot for PnP frontend initialization (VINS-Mobile: VINS_RESULT)
struct VINSResult {
    double timestamp;
    Eigen::Vector3d Ba;
    Eigen::Vector3d Bg;
    Eigen::Vector3d P;
    Eigen::Matrix3d R;
    Eigen::Vector3d V;
};

} // namespace common

#endif  // COMMON__COMMON_TYPES_H