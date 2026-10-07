#include "frontend/feature_tracker.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <iomanip>

using namespace std;
using namespace common;
using namespace Eigen;
using namespace utility;

namespace {
constexpr size_t kDiagnosticPointLimit=1000;
void diagnosticNumber(std::ostream& output,double value) {
    if(std::isfinite(value)) output<<std::setprecision(17)<<value;
    else output<<"null";
}
std::string diagnosticPoints(const std::vector<cv::Point2f>& points,
                              const std::vector<int>* ids=nullptr,const std::vector<int>* tracks=nullptr,
                              const std::vector<uchar>* status=nullptr,
                              const std::vector<cv::Point2f>* normals=nullptr,
                              const std::vector<cv::Point2f>* velocity=nullptr) {
    std::ostringstream output;
    output<<"{\"count\":"<<points.size()<<",\"rows\":[";
    const size_t count=std::min(points.size(),kDiagnosticPointLimit);
    for(size_t i=0;i<count;++i) {
        if(i) output<<',';
        output<<"[";diagnosticNumber(output,points[i].x);output<<',';diagnosticNumber(output,points[i].y);
        if(ids) output<<','<<(i<ids->size()?(*ids)[i]:-1);
        if(tracks) output<<','<<(i<tracks->size()?(*tracks)[i]:0);
        if(status) output<<','<<(i<status->size()?int((*status)[i]):0);
        if(normals) { output<<',';diagnosticNumber(output,i<normals->size()?(*normals)[i].x:NAN);
                      output<<',';diagnosticNumber(output,i<normals->size()?(*normals)[i].y:NAN); }
        if(velocity) { output<<',';diagnosticNumber(output,i<velocity->size()?(*velocity)[i].x:NAN);
                       output<<',';diagnosticNumber(output,i<velocity->size()?(*velocity)[i].y:NAN); }
        output<<']';
    }
    output<<"]}";
    return output.str();
}
std::string diagnosticMask(const std::vector<uchar>& mask) {
    std::ostringstream output;output<<"{\"count\":"<<mask.size()<<",\"values\":[";
    for(size_t i=0;i<std::min(mask.size(),kDiagnosticPointLimit);++i) {if(i)output<<',';output<<int(mask[i]);}
    output<<"]}";return output.str();
}
std::string diagnosticImage(const cv::Mat& image) {
    uint64_t hash=14695981039346656037ULL;
    for(int row=0;row<image.rows;++row) for(int col=0;col<image.cols*image.elemSize();++col) {
        hash^=image.ptr<uchar>(row)[col];hash*=1099511628211ULL;
    }
    std::ostringstream output;
    output<<"{\"rows\":"<<image.rows<<",\"cols\":"<<image.cols
          <<",\"type\":"<<image.type()<<",\"fnv1a64\":\""<<std::hex<<std::setw(16)<<std::setfill('0')<<hash<<"\"}";
    return output.str();
}
}

namespace frontend {

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f& pt) {
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < g_config.camera.col - BORDER_SIZE && BORDER_SIZE <= img_y &&
           img_y < g_config.camera.row - BORDER_SIZE;
}

void filterByStatus(vector<cv::Point2f>& v, const vector<uchar>& status) {
    int j = 0;
    // Historical vectors can be shorter than current tracks; unmatched entries have no correspondence.
    for (int i = 0; i < int(v.size()) && i < int(status.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void filterByStatus(vector<int>& v, const vector<uchar>& status) {
    int j = 0;
    for (int i = 0; i < int(v.size()) && i < int(status.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker() : cur_time(-1.0), prev_time(-1.0) {}

void FeatureTracker::setDiagnosticCapture(bool enabled) {
    diagnostic_capture_=enabled;diagnostic_stages_.clear();diagnostic_json_.clear();
}
void FeatureTracker::recordDiagnostic(const char* name,const std::string& json) {
    if(!diagnostic_capture_) return;
    if(!diagnostic_stages_.empty()) diagnostic_stages_+=',';
    diagnostic_stages_+='"';diagnostic_stages_+=name;diagnostic_stages_+="\":";diagnostic_stages_+=json;
}

void FeatureTracker::pruneTrackedPoints(const vector<uchar>& status) {
    filterByStatus(prev_pts,status);
    filterByStatus(cur_pts,status);
    filterByStatus(next_pts,status);
    filterByStatus(prev_undistorted_pts,status);
    filterByStatus(cur_undistorted_pts,status);
    filterByStatus(pts_velocity,status);
    filterByStatus(ids,status);
    filterByStatus(track_cnt,status);
}

void FeatureTracker::reset() {
    mask.release();
    fisheye_mask.release();
    prev_img.release(); cur_img.release(); next_img.release();
    n_pts.clear(); prev_pts.clear(); cur_pts.clear(); next_pts.clear();
    prev_undistorted_pts.clear(); cur_undistorted_pts.clear(); pts_velocity.clear();
    ids.clear(); track_cnt.clear();
    cur_undistorted_pts_map.clear(); prev_undistorted_pts_map.clear();
    cur_pyramid_.clear(); next_pyramid_.clear();
    cur_time = prev_time = -1.0;
    diagnostic_stages_.clear();diagnostic_json_.clear();
}

void FeatureTracker::setMask() {
    if(diagnostic_capture_) recordDiagnostic("mask_input",diagnosticPoints(next_pts,&ids,&track_cnt));
    if (g_config.feature_tracker.fisheye) {
        // Lazily initialize fisheye mask on first use.
        if (fisheye_mask.empty()) {
            const std::string& mask_path = g_config.feature_tracker.fisheye_mask;
#ifndef __EMSCRIPTEN__
            if (!mask_path.empty()) {
                fisheye_mask = cv::imread(mask_path, cv::IMREAD_GRAYSCALE);
            }
#else
            (void)mask_path;  // No file I/O in WASM — always use generated mask
#endif
            if (fisheye_mask.empty()) {
                // Auto-generate circular mask: exclude black border ring of fisheye image.
                fisheye_mask = cv::Mat(g_config.camera.row, g_config.camera.col, CV_8UC1, cv::Scalar(0));
                const double cx = g_config.camera.col / 2.0;
                const double cy = g_config.camera.row / 2.0;
                const double radius = std::min({cx, cy, g_config.camera.col - cx, g_config.camera.row - cy}) * 0.95;
                cv::circle(fisheye_mask, cv::Point(static_cast<int>(cx), static_cast<int>(cy)),
                           static_cast<int>(radius), cv::Scalar(255), -1);
            }
        }
        mask = fisheye_mask.clone();
    } else
        mask = cv::Mat(g_config.camera.row, g_config.camera.col, CV_8UC1, cv::Scalar(255));

    if(diagnostic_capture_) recordDiagnostic("mask_image",diagnosticImage(mask));
    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < next_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(next_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(),
         [](const pair<int, pair<cv::Point2f, int>>& a, const pair<int, pair<cv::Point2f, int>>& b) {
             if(a.first!=b.first) return a.first>b.first;
             if(a.second.second!=b.second.second) return a.second.second<b.second.second;
             if(a.second.first.x!=b.second.first.x) return a.second.first.x<b.second.first.x;
             return a.second.first.y<b.second.first.y;
         });

    if(diagnostic_capture_) {
        std::vector<cv::Point2f> points;std::vector<int> sorted_ids,counts;
        const size_t count=std::min(cnt_pts_id.size(),kDiagnosticPointLimit);
        for(size_t i=0;i<count;++i) {points.push_back(cnt_pts_id[i].second.first);sorted_ids.push_back(cnt_pts_id[i].second.second);counts.push_back(cnt_pts_id[i].first);}
        recordDiagnostic("mask_sorted",diagnosticPoints(points,&sorted_ids,&counts));
    }

    next_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto& it : cnt_pts_id) {
        if (mask.at<uchar>(it.second.first) == 255) {
            next_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, g_config.feature_tracker.min_dist, 0, -1);
        }
    }
    if(diagnostic_capture_) recordDiagnostic("mask_output",diagnosticPoints(next_pts,&ids,&track_cnt));
}

void FeatureTracker::addPoints() {
    for (auto& p : n_pts) {
        next_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

void FeatureTracker::detectAndTrack(const cv::Mat& _img, double _cur_time, bool backend_frame) {
    cv::Mat img;
    cur_time = _cur_time;
    if(diagnostic_capture_) {diagnostic_stages_.clear();diagnostic_json_.clear();}
    double min_intensity, max_intensity;
    cv::minMaxLoc(_img, &min_intensity, &max_intensity);
    const bool has_image_contrast = min_intensity < max_intensity;

    // equalize histogram (CLAHE cached to avoid per-frame allocation)
    if (g_config.feature_tracker.equalize) {
        if (!clahe_) {
            clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
        }
        clahe_->apply(_img, img);
    } else
        img = _img.clone(); // Retained tracking images outlive the caller's reusable frame buffer.

    if(diagnostic_capture_) recordDiagnostic("clahe_image",diagnosticImage(img));
    const int lk_win = g_config.feature_tracker.lk_window_size;
    const int lk_pyr = g_config.feature_tracker.lk_pyramid_levels;
    const cv::Size win_size(lk_win, lk_win);

    if (next_img.empty()) {
        prev_img = cur_img = next_img = img;
    } else {
        next_img = img;
    }

    // Build pyramid for the new image. Pre-computes blurred + Scharr derivative
    // images for each pyramid level. When passed to calcOpticalFlowPyrLK, it
    // skips internal pyramid construction — saving ~30-40% of LK time.
    cv::buildOpticalFlowPyramid(next_img, next_pyramid_, win_size, lk_pyr, true);

    next_pts.clear();
    // LK uses reference gradients; an exact constant target cannot support a correspondence.
    if (!has_image_contrast) {
        pruneTrackedPoints({});
        n_pts.clear();
    }

    if (cur_pts.size() > 0) {
        vector<uchar> status;
        vector<float> err;
        cv::TermCriteria lk_criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                     g_config.feature_tracker.lk_iterations,
                                     g_config.feature_tracker.lk_eps);
        // Use cached pyramids: cur_pyramid_ from previous frame, next_pyramid_ just built.
        // On the very first tracking frame, cur_pyramid_ was built from cur_img below.
        cv::calcOpticalFlowPyrLK(cur_pyramid_, next_pyramid_, cur_pts, next_pts,
                                 status, err, win_size, lk_pyr, lk_criteria);

        if(diagnostic_capture_) recordDiagnostic("lk_raw",diagnosticPoints(next_pts,&ids,&track_cnt,&status));
        for (int i = 0; i < int(next_pts.size()); i++)
            if (status[i] && !inBorder(next_pts[i]))
                status[i] = 0;

        if(diagnostic_capture_) recordDiagnostic("lk_border_status",diagnosticMask(status));
        pruneTrackedPoints(status);
    }

    for (auto& n : track_cnt)
        n++;

    // Skip F-matrix RANSAC when features barely moved (avg displacement < 2px).
    // When the device is stationary, F-matrix provides no outlier rejection value
    // and wastes 5-12ms. RMS displacement is used to detect meaningful motion.
    {
        bool should_reject = true;
        if (next_pts.size() > 0 && cur_pts.size() == next_pts.size()) {
            double total_disp_sq = 0;
            for (size_t i = 0; i < next_pts.size(); i++) {
                double dx = next_pts[i].x - cur_pts[i].x;
                double dy = next_pts[i].y - cur_pts[i].y;
                total_disp_sq += dx * dx + dy * dy;
            }
            double rms_disp = std::sqrt(total_disp_sq / next_pts.size());
            if (rms_disp < 2.0) {
                should_reject = false;
            }
        }
        if (should_reject) {
            rejectWithFundamentalMatrix();
        }
    }

    // On backend frames: full pipeline (setMask + new feature detection + addPoints).
    // On PnP-only frames: skip — only existing feature tracks are needed.
    if (backend_frame && has_image_contrast) {
        setMask();

        int supplementary_points_count = g_config.feature_tracker.max_cnt - static_cast<int>(next_pts.size());
        if (supplementary_points_count > 0) {
            if (mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != next_img.size())
                cout << "wrong size " << endl;

            cv::goodFeaturesToTrack(next_img, n_pts, supplementary_points_count, 0.01, g_config.feature_tracker.min_dist,
                                    mask);
        } else
            n_pts.clear();

        if(diagnostic_capture_) recordDiagnostic("gftt_new",diagnosticPoints(n_pts));
        addPoints();
    }

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_undistorted_pts = cur_undistorted_pts;
    cur_img = next_img;
    cur_pts = next_pts;
    // Cache pyramid: next frame reuses this as the reference pyramid (zero-copy move).
    cur_pyramid_ = std::move(next_pyramid_);
    undistortedPoints();
    prev_time = cur_time;
    if(diagnostic_capture_) {
        recordDiagnostic("final",diagnosticPoints(cur_pts,&ids,&track_cnt,nullptr,&cur_undistorted_pts,&pts_velocity));
        std::ostringstream output;output<<"{\"time\":";diagnosticNumber(output,cur_time);
        output<<",\"stages\":{"<<diagnostic_stages_<<"}}";diagnostic_json_=output.str();
    }
}

void FeatureTracker::rejectWithFundamentalMatrix() {
    vector<cv::Point2f> undistorted_cur_pts(cur_pts.size()), undistorted_next_pts(next_pts.size());
    vector<uchar> valid(cur_pts.size(),0);
    const double cx = g_config.camera.col / 2.0;
    const double cy = g_config.camera.row / 2.0;
    const double float_limit = std::numeric_limits<float>::max();
    if (m_camera) for (size_t i=0;i<cur_pts.size() && i<next_pts.size();++i) {
        Eigen::Vector3d current_ray,next_ray;
        m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x,cur_pts[i].y),current_ray);
        m_camera->liftProjective(Eigen::Vector2d(next_pts[i].x,next_pts[i].y),next_ray);
        if (!current_ray.allFinite() || !next_ray.allFinite() ||
            current_ray.z()<=1e-12 || next_ray.z()<=1e-12) continue;
        const double coordinates[] = {
            g_config.camera.focal_length*current_ray.x()/current_ray.z()+cx,
            g_config.camera.focal_length*current_ray.y()/current_ray.z()+cy,
            g_config.camera.focal_length*next_ray.x()/next_ray.z()+cx,
            g_config.camera.focal_length*next_ray.y()/next_ray.z()+cy};
        if (!std::all_of(std::begin(coordinates),std::end(coordinates),[&](double value){
            return std::isfinite(value) && std::abs(value)<=float_limit;
        })) continue;
        undistorted_cur_pts[i]=cv::Point2f(coordinates[0],coordinates[1]);
        undistorted_next_pts[i]=cv::Point2f(coordinates[2],coordinates[3]);
        valid[i]=1;
    }
    if(diagnostic_capture_) recordDiagnostic("f_valid",diagnosticMask(valid));
    // Invalid rays must not enter either division/scoring or retained parallel track history.
    pruneTrackedPoints(valid);
    filterByStatus(undistorted_cur_pts,valid);
    filterByStatus(undistorted_next_pts,valid);
    // Preserve the existing minimum for stable F-matrix estimation after invalid pairs are removed.
    if (next_pts.size() >= 30) {
        const int before_count = static_cast<int>(next_pts.size());

        vector<uchar> status;
        cv::Mat F = cv::findFundamentalMat(undistorted_cur_pts, undistorted_next_pts, cv::FM_RANSAC,
                               g_config.feature_tracker.f_threshold, 0.99, status);

        // Distance-aware edge feature recovery for unmodeled lens distortion.
        // Mobile PINHOLE cameras with zero distortion coefficients have actual barrel
        // distortion that causes edge features to have larger epipolar residuals.
        // Without this, F-matrix RANSAC systematically rejects edge features,
        // causing all surviving features to cluster toward image center.
        const double edge_factor = g_config.feature_tracker.f_threshold_edge_factor;
        if (edge_factor > 0.0 && !F.empty() && F.rows == 3 && F.cols == 3) {
            const double r_max = std::sqrt(cx * cx + cy * cy);
            const double r_max_inv = (r_max > 1e-6) ? 1.0 / r_max : 0.0;
            int restored = 0;

            for (unsigned int i = 0; i < status.size(); i++) {
                if (status[i]) continue;  // Already inlier

                // Compute normalized distance from image center (0=center, 1=corner)
                const double dx = next_pts[i].x - cx;
                const double dy = next_pts[i].y - cy;
                const double r_ratio = std::sqrt(dx * dx + dy * dy) * r_max_inv;

                // Only apply edge recovery to features in the outer region
                if (r_ratio < 0.3) continue;

                // Compute epipolar distance: d = |p2^T F p1| / ||l||
                const double* f = F.ptr<double>();
                const double u1 = undistorted_cur_pts[i].x;
                const double v1 = undistorted_cur_pts[i].y;
                const double u2 = undistorted_next_pts[i].x;
                const double v2 = undistorted_next_pts[i].y;

                // Epipolar line l = F * [u1, v1, 1]^T
                const double la = f[0] * u1 + f[1] * v1 + f[2];
                const double lb = f[3] * u1 + f[4] * v1 + f[5];
                const double lc = f[6] * u1 + f[7] * v1 + f[8];
                const double norm_ab = std::sqrt(la * la + lb * lb);
                if (norm_ab < 1e-12) continue;

                const double dist = std::abs(la * u2 + lb * v2 + lc) / norm_ab;

                // Adaptive threshold: base * (1 + edge_factor * r_ratio^2)
                // Center features (r=0): 1x threshold
                // Edge features (r=r_max): (1 + edge_factor)x threshold
                const double adaptive_thresh =
                    g_config.feature_tracker.f_threshold * (1.0 + edge_factor * r_ratio * r_ratio);

                if (dist < adaptive_thresh) {
                    status[i] = 1;  // Restore this edge feature
                    restored++;
                }
            }

            if (restored > 0) {
                std::cout << "[FeatureTracker] Edge recovery: restored " << restored
                          << " features (edge_factor=" << edge_factor << ")" << std::endl;
            }
        }

        if(diagnostic_capture_) recordDiagnostic("f_inliers",diagnosticMask(status));
        int inlier_count = 0;
        for (const auto& s : status) { if (s) inlier_count++; }
        const int rejected = before_count - inlier_count;

        // Log when >30% features rejected (indicates tracking stress)
        if (rejected > before_count * 3 / 10) {
            std::cout << "[FeatureTracker] F-matrix: " << before_count
                      << " → " << inlier_count
                      << " (rejected " << rejected
                      << ", threshold=" << g_config.feature_tracker.f_threshold << ")"
                      << std::endl;
        }

        pruneTrackedPoints(status);
    }
}

bool FeatureTracker::updateID(unsigned int i) {
    if (i < ids.size()) {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    } else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string& calib_file) {
    m_camera = common::camera_models::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

bool FeatureTracker::setIntrinsicParameter(int model_type, int width, int height,
                                           double fx, double fy, double cx, double cy,
                                           double k2, double k3, double k4, double k5) {
    const double values[] = {fx,fy,cx,cy,k2,k3,k4,k5};
    if (width < 8 || height < 8 || fx <= 0 || fy <= 0 ||
        !std::all_of(std::begin(values),std::end(values),[](double value){return std::isfinite(value);})) return false;
    if (model_type == common::camera_models::Camera::PINHOLE) {
        // Pinhole: k2->k1, k3->k2, k4->p1, k5->p2
        common::camera_models::PinholeCamera::Parameters params(
            "camera", width, height, k2, k3, k4, k5, fx, fy, cx, cy);
        m_camera = std::make_shared<common::camera_models::PinholeCamera>(params);
    } else if (model_type == common::camera_models::Camera::KANNALA_BRANDT) {
        common::camera_models::EquidistantCamera::Parameters params(
            "camera", width, height, k2, k3, k4, k5, fx, fy, cx, cy);
        m_camera = std::make_shared<common::camera_models::EquidistantCamera>(params);
    } else {
        return false;
    }
    reset();
    return true;
}

void FeatureTracker::undistortedPoints() {
    cur_undistorted_pts.clear();
    cur_undistorted_pts.resize(cur_pts.size());
    cur_undistorted_pts_map.clear();
    pts_velocity.clear();
    std::vector<uchar> valid(cur_pts.size(),1);
    // cv::undistortPoints(cur_pts, undistorted_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++) {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        if (!b.allFinite() || b.z() <= 1e-12) { valid[i] = 0; continue; }
        const double x=b.x()/b.z(),y=b.y()/b.z();
        if (!std::isfinite(x) || !std::isfinite(y) || std::abs(x)>std::numeric_limits<float>::max() ||
            std::abs(y)>std::numeric_limits<float>::max()) { valid[i]=0; continue; }
        cur_undistorted_pts[i] = cv::Point2f(x,y);
        cur_undistorted_pts_map.insert(make_pair(ids[i],cur_undistorted_pts[i]));
        // printf("cur pts id %d %f %f", ids[i], cur_undistorted_pts[i].x, cur_undistorted_pts[i].y);
    }
    pruneTrackedPoints(valid);
    // caculate points velocity
    if (!prev_undistorted_pts_map.empty()) {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_undistorted_pts.size(); i++) {
            if (ids[i] != -1) {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_undistorted_pts_map.find(ids[i]);
                if (it != prev_undistorted_pts_map.end()) {
                    double v_x, v_y;
                    if (dt > 1e-4) {
                        v_x = (cur_undistorted_pts[i].x - it->second.x) / dt;
                        v_y = (cur_undistorted_pts[i].y - it->second.y) / dt;
                    } else {
                        v_x = 0;
                        v_y = 0;
                    }
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                } else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            } else {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    } else {
        for (unsigned int i = 0; i < cur_pts.size(); i++) {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_undistorted_pts_map = cur_undistorted_pts_map;
}

}  // namespace frontend
