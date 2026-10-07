#ifndef UTILITY__TRAJECTORY_EVALUATOR_H
#define UTILITY__TRAJECTORY_EVALUATOR_H

#include <Eigen/Dense>
#include <string>
#include <limits>
#include <vector>

namespace utility {

struct AteResult {
    double rmse = std::numeric_limits<double>::quiet_NaN();
    bool valid = false;
    double alignment_scale = 1.0;
    double mean = std::numeric_limits<double>::quiet_NaN();
    double median = std::numeric_limits<double>::quiet_NaN();
    double std_dev = std::numeric_limits<double>::quiet_NaN();
    double min = std::numeric_limits<double>::quiet_NaN();
    double max = std::numeric_limits<double>::quiet_NaN();
    int num_pairs = 0;
};

struct RpeResult {
    double rmse_trans = std::numeric_limits<double>::quiet_NaN();
    bool valid = false;
    double rmse_rot = std::numeric_limits<double>::quiet_NaN();
    int num_pairs = 0;
};

struct TimestampedPose {
    double timestamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

class TrajectoryEvaluator {
public:
    TrajectoryEvaluator() = default;

    // Loading
    bool loadVioTrajectory(const std::string& filepath);
    bool loadGroundTruth(const std::string& filepath);

    // Transform VIO poses from camera frame to body frame
    void transformVioToBodyFrame(const Eigen::Matrix3d& r_ic, const Eigen::Vector3d& t_ic);

    // Alignment
    int associateTrajectories(double max_dt = 0.01);
    // Rigid metric alignment by default; Sim(3) is diagnostic only.
    bool alignTrajectories(bool diagnostic_sim3 = false);

    // Evaluation
    AteResult computeATE() const;
    RpeResult computeRPE(double delta = 1.0) const;

    // Output
    void printResults(const AteResult& ate, const RpeResult& rpe) const;
    bool saveResults(const std::string& filepath, const AteResult& ate, const RpeResult& rpe) const;

    // Accessors for testing
    int getVioSize() const { return static_cast<int>(vio_trajectory_.size()); }
    int getGtSize() const { return static_cast<int>(gt_trajectory_.size()); }
    int getMatchedSize() const { return static_cast<int>(matched_vio_.size()); }

private:
    std::vector<TimestampedPose> vio_trajectory_;
    std::vector<TimestampedPose> gt_trajectory_;

    // Matched pairs after association
    std::vector<TimestampedPose> matched_vio_;
    std::vector<TimestampedPose> matched_gt_;

    double alignment_scale_ = 1.0;

    // Aligned positions; RPE uses metric matched poses, independent of alignment.
    std::vector<Eigen::Vector3d> aligned_vio_;
};

}  // namespace utility

#endif  // UTILITY__TRAJECTORY_EVALUATOR_H
