#include "frontend/feature_manager.h"
#include <cmath>
#include "utility/so3.h"

namespace frontend {
namespace {
void invalidateAnchor(FeaturePerId& feature) {
    feature.estimated_depth = -1;
    feature.solve_flag = 2;
    feature.geometry_admitted = false;
    feature.geometry_reason = "invalid_anchor";
}
}

int FeaturePerId::endFrame() {
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager() : last_track_num_(0) {}

void FeatureManager::clearState() {
    feature_bank_.clear();
    last_track_num_ = 0;
}

int FeatureManager::getFeatureCount() {
    int cnt = 0;
    for (auto& it : feature_bank_) {
        it.used_num = it.feature_per_frame.size();

        if (it.isParameterCandidate()) {
            if (++cnt == NUM_OF_FEATURES) break;
        }
    }
    return cnt;
}

bool FeatureManager::addFeatureAndCheckParallax(int frame_count, const common::ImageData& image) {
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num_ = 0;

    // add features to feature bank
    for (auto& feature_id_and_info : image) {
        if (!feature_id_and_info.second.allFinite()) continue;
        int feature_id = feature_id_and_info.first;
        FeaturePerFrame feature_per_frame(feature_id_and_info.second);
        if (!feature_per_frame.ray_vector.allFinite()) continue;

        auto it = find_if(feature_bank_.begin(), feature_bank_.end(),
                          [feature_id](const FeaturePerId& it) { return it.feature_id == feature_id; });

        // Storage indexes assume consecutive frames. A reappearing ID begins a
        // new track rather than attaching its observation to a missing frame.
        if (it != feature_bank_.end() && it->endFrame() != frame_count - 1) {
            feature_bank_.erase(it);
            it = feature_bank_.end();
        }

        if (it == feature_bank_.end()) {
            // new feature
            feature_bank_.push_back(FeaturePerId(feature_id, frame_count));
            feature_bank_.back().feature_per_frame.push_back(feature_per_frame);
        } else if (it->feature_id == feature_id) {
            // tracked feature
            it->feature_per_frame.push_back(feature_per_frame);
            if (it->isParameterCandidate() && !it->hasLegacyAge() && it->solve_flag != 1)
                it->geometry_pending = true;
            it->geometry_admitted = false;
            last_track_num_++;
        }
    }

    // check parallax
    if (frame_count < 2 || last_track_num_ < 20) {
        // novel view
        return true;
    }

    for (auto& it_per_id : feature_bank_) {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1) {
            if (it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame].projection_outlier ||
                it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame].projection_outlier) continue;
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0) {
        // novel view
        return true;
    } else {
        // enough parallax, new keyframe
        // min_parallax (pixels) / focal_length = threshold in normalized coords
        return (parallax_sum / parallax_num) >= (g_config.estimator.min_parallax / g_config.camera.focal_length);
    }
}

Correspondences FeatureManager::getCorresponding(int frame_count_l, int frame_count_r) {
    frontend::Correspondences corres;
    for (auto& it : feature_bank_) {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r) {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;
            if (it.feature_per_frame[idx_l].projection_outlier || it.feature_per_frame[idx_r].projection_outlier) continue;

            // ray vector in frame_count_l of same feature_id
            a = it.feature_per_frame[idx_l].ray_vector;

            // ray vector in frame_count_r of same feature_id
            b = it.feature_per_frame[idx_r].ray_vector;

            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

void FeatureManager::setDepth(const VectorXd& x, const std::array<bool, NUM_OF_FEATURES>* admitted) {
    int feature_index = -1;
    for (auto& it_per_id : feature_bank_) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!it_per_id.isParameterCandidate())
            continue;

        if (++feature_index >= NUM_OF_FEATURES) break;
        if (admitted && !(*admitted)[feature_index]) continue;
        const double inverse_depth = feature_index < x.size() ? x(feature_index) : 0;
        if (!std::isfinite(inverse_depth) || inverse_depth <= 1e-10) {
            it_per_id.estimated_depth = -1;
            it_per_id.solve_flag = 2;
        } else {
            const double depth = 1.0 / inverse_depth;
            it_per_id.estimated_depth = depth;
            it_per_id.solve_flag = 1;
            if (admitted) {
                it_per_id.geometry_pending = false;
                it_per_id.geometry_admitted = false;
            }
        }
    }
}

void FeatureManager::removeFailures() {
    for (auto it = feature_bank_.begin(), it_next = feature_bank_.begin(); it != feature_bank_.end(); it = it_next) {
        it_next++;
        if (it->solve_flag == 2)
            feature_bank_.erase(it);
    }
}

void FeatureManager::clearDepth(const VectorXd& x) {
    int feature_index = -1;
    for (auto& it_per_id : feature_bank_) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!it_per_id.isParameterCandidate())
            continue;
        if (++feature_index >= NUM_OF_FEATURES) break;
        const double inverse_depth = feature_index < x.size() ? x(feature_index) : 0;
        it_per_id.estimated_depth = std::isfinite(inverse_depth) && inverse_depth > 1e-10
                                      ? 1.0 / inverse_depth : -1;
        it_per_id.solve_flag = 0;
        it_per_id.geometry_admitted = false;
    }
}

VectorXd FeatureManager::getDepthVector() {
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto& it_per_id : feature_bank_) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!it_per_id.isParameterCandidate())
            continue;
        if (++feature_index >= NUM_OF_FEATURES) break;
        dep_vec(feature_index) = std::isfinite(it_per_id.estimated_depth) && it_per_id.estimated_depth > 1e-10
                                      ? 1. / it_per_id.estimated_depth : 0;
    }
    return dep_vec;
}

bool FeatureManager::hasObservableGeometry(const FeaturePerId& feature,
        const std::array<Matrix3d, WINDOW_SIZE + 1>& camera_rotations, double* condition) {
    if (!feature.isParameterCandidate() || !feature.hasValidAnchor()) return false;
    Matrix3d information = Matrix3d::Zero();
    int frame = feature.start_frame, good_views = 0;
    for (const auto& observation : feature.feature_per_frame) {
        const int observation_frame = frame++;
        if (observation.projection_outlier) continue;
        if (!observation.ray_vector.allFinite() || observation.ray_vector.z() <= 0 ||
            !utility::so3::isRotation(camera_rotations[observation_frame])) return false;
        const Vector3d bearing = camera_rotations[observation_frame] * observation.ray_vector.normalized();
        information += Matrix3d::Identity() - bearing * bearing.transpose();
        ++good_views;
    }
    if (good_views < 2) return false;
    const Eigen::SelfAdjointEigenSolver<Matrix3d> eigen(information);
    if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite() || eigen.eigenvalues().minCoeff() <= 0)
        return false;
    const double kappa = eigen.eigenvalues().maxCoeff() / eigen.eigenvalues().minCoeff();
    if (condition) *condition = kappa;
    return std::isfinite(kappa) && kappa <= 10000;
}

bool FeatureManager::hasConsistentReprojection(const FeaturePerId& feature, double depth,
        const std::array<Matrix3d, WINDOW_SIZE + 1>& camera_rotations,
        const std::array<Vector3d, WINDOW_SIZE + 1>& camera_centers) {
    if (!feature.isParameterCandidate() || !feature.hasValidAnchor() || !std::isfinite(depth) || depth <= 1e-10 ||
        !std::isfinite(g_config.camera.fx) || !std::isfinite(g_config.camera.fy) ||
        g_config.camera.fx <= 0 || g_config.camera.fy <= 0) return false;
    const int anchor = feature.start_frame;
    const Vector3d world = camera_rotations[anchor] * (feature.feature_per_frame.front().ray_vector * depth) +
                           camera_centers[anchor];
    if (!world.allFinite()) return false;
    int frame = anchor, good_views = 0;
    for (const auto& observation : feature.feature_per_frame) {
        const int observation_frame = frame++;
        if (observation.projection_outlier) continue;
        const Vector3d camera = camera_rotations[observation_frame].transpose() * (world - camera_centers[observation_frame]);
        if (!camera.allFinite() || camera.z() <= 1e-10 || !observation.ray_vector.allFinite() ||
            observation.ray_vector.z() <= 0) return false;
        const double error = std::hypot(g_config.camera.fx * (camera.x()/camera.z()-observation.ray_vector.x()),
                                        g_config.camera.fy * (camera.y()/camera.z()-observation.ray_vector.y()));
        if (!std::isfinite(error) || error > 1.5) return false;
        ++good_views;
    }
    return good_views >= 2;
}

void FeatureManager::triangulateAcrossAllViews(const backend::SlidingWindow& sliding_window, const Vector3d& t_ic,
                                               const Matrix3d& r_ic) {
    std::array<Matrix3d, WINDOW_SIZE + 1> camera_rotations;
    std::array<Vector3d, WINDOW_SIZE + 1> camera_centers;
    for (int frame = 0; frame <= WINDOW_SIZE; ++frame) {
        camera_rotations[frame] = sliding_window[frame].R * r_ic;
        camera_centers[frame] = sliding_window[frame].P + sliding_window[frame].R * t_ic;
    }
    std::array<std::array<Eigen::Matrix<double, 3, 4>, WINDOW_SIZE + 1>, WINDOW_SIZE + 1> projections;
    std::array<std::array<bool, WINDOW_SIZE + 1>, WINDOW_SIZE + 1> projection_ready{};
    std::array<Eigen::JacobiSVD<Eigen::MatrixXd>, WINDOW_SIZE + 1> svd_workspaces;
    int feature_index = -1;
    for (auto& it_per_id : feature_bank_) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!it_per_id.isParameterCandidate()) continue;
        if (++feature_index >= NUM_OF_FEATURES) break;
        if (!it_per_id.hasValidAnchor()) { invalidateAnchor(it_per_id); continue; }
        if (!it_per_id.hasLegacyAge() && it_per_id.solve_flag != 1) it_per_id.geometry_pending = true;
        if (it_per_id.geometry_pending) {
            it_per_id.geometry_admitted = false;
            it_per_id.geometry_condition = std::numeric_limits<double>::quiet_NaN();
            it_per_id.geometry_reason = "rank_or_condition";
            if (!hasObservableGeometry(it_per_id, camera_rotations, &it_per_id.geometry_condition)) {
                it_per_id.estimated_depth = -1;
                it_per_id.solve_flag = 0;
                continue;
            }
        }

        if (!it_per_id.geometry_pending && std::isfinite(it_per_id.estimated_depth) && it_per_id.estimated_depth > 0)
            continue;

        int imu_i = it_per_id.start_frame;
        int imu_j = imu_i - 1;

        const int good_views = std::count_if(it_per_id.feature_per_frame.begin(),it_per_id.feature_per_frame.end(),
            [](const auto& observation) { return !observation.projection_outlier; });
        if (good_views < 2) {
            it_per_id.estimated_depth = -1;
            it_per_id.solve_flag = it_per_id.geometry_pending ? 0 : 2;
            it_per_id.geometry_reason = "insufficient_good_views";
            continue;
        }
        Eigen::MatrixXd svd_A(2 * good_views, 4);
        int svd_idx = 0;

        for (auto& it_per_frame : it_per_id.feature_per_frame) {
            imu_j++;
            if (it_per_frame.projection_outlier) continue;

            auto& P = projections[imu_i][imu_j];
            if (!projection_ready[imu_i][imu_j]) {
                const Eigen::Vector3d relative_t = camera_rotations[imu_i].transpose() *
                    (camera_centers[imu_j] - camera_centers[imu_i]);
                const Eigen::Matrix3d relative_R = camera_rotations[imu_i].transpose() * camera_rotations[imu_j];
                P.leftCols<3>() = relative_R.transpose();
                P.rightCols<1>() = -relative_R.transpose() * relative_t;
                projection_ready[imu_i][imu_j] = true;
            }
            Eigen::Vector3d f = it_per_frame.ray_vector.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        assert(svd_idx == svd_A.rows());
        // Eigen leaves V untouched on nonfinite input; never expose a previous landmark's V.
        if (!svd_A.allFinite()) {
            it_per_id.estimated_depth = -1;
            it_per_id.solve_flag = it_per_id.geometry_pending ? 0 : 2;
            it_per_id.geometry_reason = "nonfinite_dlt";
            continue;
        }
        auto& svd = svd_workspaces[good_views - 1];
        svd.compute(svd_A, Eigen::ComputeThinV);
        const Eigen::Vector4d svd_V = svd.matrixV().rightCols<1>();
        if (svd.info() != Eigen::Success || !svd_V.allFinite() || std::abs(svd_V[3]) <= 1e-10) {
            it_per_id.estimated_depth = -1;
            it_per_id.solve_flag = it_per_id.geometry_pending ? 0 : 2;
            it_per_id.geometry_reason = "nonfinite_dlt";
            continue;
        }
        double svd_method = svd_V[2] / svd_V[3];
        // it_per_id->estimated_depth = -b / A;
        // it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        // it_per_id->estimated_depth = INIT_DEPTH;

        if (!std::isfinite(it_per_id.estimated_depth) || it_per_id.estimated_depth <= 1e-10) {
            it_per_id.estimated_depth = -1;
            it_per_id.solve_flag = it_per_id.geometry_pending ? 0 : 2;
            it_per_id.geometry_reason = "invalid_anchor_depth";
        } else if (it_per_id.geometry_pending) {
            if (hasConsistentReprojection(it_per_id, it_per_id.estimated_depth, camera_rotations, camera_centers)) {
                it_per_id.geometry_admitted = true;
                it_per_id.geometry_reason = "accepted";
            } else {
                it_per_id.estimated_depth = -1;
                it_per_id.solve_flag = 0;
                it_per_id.geometry_reason = "cheirality_or_reprojection";
            }
        }
    }
}

void FeatureManager::removeOutlier() {
    assert(false);
    int i = -1;
    for (auto it = feature_bank_.begin(), it_next = feature_bank_.begin(); it != feature_bank_.end(); it = it_next) {
        it_next++;
        i += it->used_num != 0;
        if (it->used_num != 0 && it->is_outlier == true) {
            feature_bank_.erase(it);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R,
                                          Eigen::Vector3d new_P) {
    for (auto it = feature_bank_.begin(), it_next = feature_bank_.begin(); it != feature_bank_.end(); it = it_next) {
        it_next++;
        if (it->feature_per_frame.empty()) { feature_bank_.erase(it); continue; }
        it->geometry_admitted = false;

        if (it->start_frame != 0) {
            it->start_frame--;
            if (!it->hasValidAnchor()) invalidateAnchor(*it);
        } else {
            const bool valid_old_anchor = it->hasValidAnchor();
            Eigen::Vector3d uv_i = it->feature_per_frame[0].ray_vector;
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2) {
                feature_bank_.erase(it);
                continue;
            } else {
                if (!valid_old_anchor || !it->hasValidAnchor()) { invalidateAnchor(*it); continue; }
                if (!std::isfinite(it->estimated_depth) || it->estimated_depth <= 1e-10) {
                    it->estimated_depth = -1;
                    if (it->solve_flag != 2) it->solve_flag = 0;
                    continue;
                }
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (std::isfinite(dep_j) && dep_j > 1e-10)
                    it->estimated_depth = dep_j;
                else {
                    it->estimated_depth = -1;
                    it->solve_flag = 2;
                }
            }
        }
    }
}

void FeatureManager::removeBack() {
    for (auto it = feature_bank_.begin(), it_next = feature_bank_.begin(); it != feature_bank_.end(); it = it_next) {
        it_next++;
        if (it->feature_per_frame.empty()) { feature_bank_.erase(it); continue; }
        it->geometry_admitted = false;

        if (it->start_frame != 0) {
            it->start_frame--;
            if (!it->hasValidAnchor()) invalidateAnchor(*it);
        } else {
            const bool valid_old_anchor = it->hasValidAnchor();
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature_bank_.erase(it);
            else if (!valid_old_anchor || !it->hasValidAnchor()) invalidateAnchor(*it);
        }
    }
}

void FeatureManager::removeFront(int frame_count, const Matrix3d& old_R, const Vector3d& old_P,
                                 const Matrix3d& new_R, const Vector3d& new_P) {
    for (auto it = feature_bank_.begin(), it_next = feature_bank_.begin(); it != feature_bank_.end(); it = it_next) {
        it_next++;
        if (it->feature_per_frame.empty()) { feature_bank_.erase(it); continue; }
        it->geometry_admitted = false;

        if (it->start_frame == frame_count) {
            it->start_frame--;
            if (!it->hasValidAnchor()) invalidateAnchor(*it);
        } else {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            if (j < 0 || static_cast<size_t>(j) >= it->feature_per_frame.size()) continue;
            if (j == 0 && it->feature_per_frame.size() > 1) {
                if (!it->hasValidAnchor() || it->feature_per_frame[1].projection_outlier ||
                    !it->feature_per_frame[1].ray_vector.allFinite() || it->feature_per_frame[1].ray_vector.z() <= 0) {
                    invalidateAnchor(*it);
                } else {
                    const Eigen::Vector3d point = new_R.transpose() *
                        (old_R * (it->feature_per_frame.front().ray_vector * it->estimated_depth) + old_P - new_P);
                    const auto& ray = it->feature_per_frame[1].ray_vector;
                    const double error = point.allFinite() && point.z() > 1e-10
                        ? std::hypot(g_config.camera.fx * (point.x()/point.z()-ray.x()),
                                     g_config.camera.fy * (point.y()/point.z()-ray.y()))
                        : std::numeric_limits<double>::infinity();
                    if (std::isfinite(it->estimated_depth) && it->estimated_depth > 1e-10 && error <= 1.5)
                        it->estimated_depth = point.z();
                    else {
                        it->estimated_depth = -1;
                        if (it->solve_flag != 2) it->solve_flag = 0;
                    }
                }
            }
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                feature_bank_.erase(it);
            else if (!it->hasValidAnchor()) invalidateAnchor(*it);
        }
    }
}

double FeatureManager::compensatedParallax2(const FeaturePerId& it_per_id, int frame_count) {
    // check the second last frame is keyframe or not
    // parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame& frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame& frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.ray_vector;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.ray_vector;
    Vector3d p_i_comp;

    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}

}  // namespace frontend
