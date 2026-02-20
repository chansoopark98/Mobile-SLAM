#include "frontend/initialization/solve_5pts.h"

namespace frontend {
namespace initialization {

bool MotionEstimator::solveRelativeRT(const Correspondences& corres, Matrix3d& Rotation, Vector3d& Translation) {
    if (corres.size() >= 15) {
        vector<cv::Point2f> ll, rr;
        for (int i = 0; i < int(corres.size()); i++) {
            ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));
            rr.push_back(cv::Point2f(corres[i].second(0), corres[i].second(1)));
        }
        // Use OpenCV's findEssentialMat for normalized coordinates (K=I, focal=1, pp=(0,0))
        cv::Mat mask;
        cv::Mat E = cv::findEssentialMat(ll, rr, 1.0, cv::Point2d(0, 0), cv::RANSAC, 0.99, 0.003, mask);
        if (E.empty()) {
            return false;
        }
        cv::Mat rot, trans;
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        int inlier_cnt = cv::recoverPose(E, ll, rr, cameraMatrix, rot, trans, mask);

        Eigen::Matrix3d R;
        Eigen::Vector3d T;
        for (int i = 0; i < 3; i++) {
            T(i) = trans.at<double>(i, 0);
            for (int j = 0; j < 3; j++)
                R(i, j) = rot.at<double>(i, j);
        }

        Rotation = R.transpose();
        Translation = -R.transpose() * T;
        if (inlier_cnt > 12)
            return true;
        else
            return false;
    }
    return false;
}

}  // namespace initialization
}  // namespace frontend
