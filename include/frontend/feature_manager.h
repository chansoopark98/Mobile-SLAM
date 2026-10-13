#ifndef FRONTEND__FEATURE_MANAGER_H
#define FRONTEND__FEATURE_MANAGER_H

#include <algorithm>
#include <array>
#include <cmath>
#include <Eigen/Dense>
#include <limits>
#include <list>
#include <numeric>
#include <vector>

#include "backend/sliding_window.h"
#include "common/image_frame.h"
#include "utility/config.h"

using namespace std;
using namespace Eigen;

namespace frontend {

using Correspondence = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
using Correspondences = std::vector<Correspondence>;

class FeaturePerFrame {
public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1>& _point) {
        ray_vector = _point.head<3>();
        if (ray_vector.allFinite() && ray_vector.z() > 0)
            ray_vector /= ray_vector.z();
        else
            ray_vector.setConstant(std::numeric_limits<double>::quiet_NaN());
        observation.x() = _point(3);
        observation.y() = _point(4);
        velocity.x() = _point(5);
        velocity.y() = _point(6);
    }
    Vector3d ray_vector;
    Vector2d observation;
    Vector2d velocity;
    double z = 0;
    bool is_used = false;
    // A rejected stored observation stays excluded until that observation leaves the window.
    bool projection_outlier = false;
    double parallax = 0;
    MatrixXd A;
    VectorXd b;
    double dep_gradient = 0;
};

class FeaturePerId {
public:
    const int feature_id;
    int start_frame;
    vector<FeaturePerFrame> feature_per_frame;

    int used_num;
    bool is_outlier;
    bool is_margin;
    double estimated_depth;
    int solve_flag;  // 0 haven't solve yet; 1 solve succ; 2 solve fail;
    bool geometry_pending = false;
    bool geometry_admitted = false;
    double geometry_condition = std::numeric_limits<double>::quiet_NaN();
    const char* geometry_reason = "legacy";

    Vector3d gt_p;

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame), used_num(0), is_outlier(false), is_margin(false),
          estimated_depth(-1.0), solve_flag(0), gt_p(Vector3d::Zero()) {}

    int endFrame();
    bool isParameterCandidate() const {
        return start_frame >= 0 && start_frame <= WINDOW_SIZE && feature_per_frame.size() >= 2 &&
               feature_per_frame.size() <= static_cast<size_t>(WINDOW_SIZE - start_frame + 1);
    }
    bool hasLegacyAge() const { return start_frame < WINDOW_SIZE - 2; }
    bool hasValidAnchor() const {
        return !feature_per_frame.empty() && !feature_per_frame.front().projection_outlier &&
               feature_per_frame.front().ray_vector.allFinite() && feature_per_frame.front().ray_vector.z() > 0;
    }
    bool isDepthAdmitted() const {
        return hasValidAnchor() && solve_flag != 2 && std::isfinite(estimated_depth) && estimated_depth > 1e-10 &&
               (geometry_pending ? geometry_admitted : (hasLegacyAge() || solve_flag == 1));
    }
};

class FeatureManager {
public:
    FeatureManager();

    void clearState();

    int getFeatureCount();

    bool addFeatureAndCheckParallax(int frame_count, const common::ImageData& image);
    Correspondences getCorresponding(int frame_count_l, int frame_count_r);

    // void updateDepth(const VectorXd &x);
    void setDepth(const VectorXd& x, const std::array<bool, NUM_OF_FEATURES>* admitted = nullptr);
    void removeFailures();
    void clearDepth(const VectorXd& x);
    VectorXd getDepthVector();
    void triangulateAcrossAllViews(const backend::SlidingWindow& sliding_window, const Vector3d& t_ic,
                                   const Matrix3d& r_ic);
    static bool hasObservableGeometry(const FeaturePerId& feature,
        const std::array<Matrix3d, WINDOW_SIZE + 1>& camera_rotations, double* condition = nullptr);
    static bool hasConsistentReprojection(const FeaturePerId& feature, double depth,
        const std::array<Matrix3d, WINDOW_SIZE + 1>& camera_rotations,
        const std::array<Vector3d, WINDOW_SIZE + 1>& camera_centers);
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R,
                              Eigen::Vector3d new_P);
    void removeBack();
    void removeFront(int frame_count, const Matrix3d& old_R, const Vector3d& old_P,
                     const Matrix3d& new_R, const Vector3d& new_P);
    void removeOutlier();
    list<FeaturePerId> feature_bank_;
    int last_track_num_;

private:
    double compensatedParallax2(const FeaturePerId& it_per_id, int frame_count);
};

}  // namespace frontend

#endif  // FRONTEND__FEATURE_MANAGER_H
