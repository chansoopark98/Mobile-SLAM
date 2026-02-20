#ifndef UTILITY__TRAJECTORY_EVALUATOR_H
#define UTILITY__TRAJECTORY_EVALUATOR_H

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace utility {

struct AteResult {
    double rmse = 0.0;
    double mean = 0.0;
    double median = 0.0;
    double std_dev = 0.0;
    double min = 0.0;
    double max = 0.0;
    int num_pairs = 0;
};

struct RpeResult {
    double rmse_trans = 0.0;
    double rmse_rot = 0.0;
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
    bool alignTrajectories();

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
    std::vector<Eigen::Vector3d> matched_vio_;
    std::vector<Eigen::Vector3d> matched_gt_;

    // Aligned VIO positions (after SE(3)+scale transform)
    std::vector<Eigen::Vector3d> aligned_vio_;
};

}  // namespace utility

#endif  // UTILITY__TRAJECTORY_EVALUATOR_H
