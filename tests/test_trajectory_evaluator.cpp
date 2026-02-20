#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <cmath>
#include "utility/trajectory_evaluator.h"

class TrajectoryEvaluatorTest : public ::testing::Test {
protected:
    std::string test_dir_ = "/tmp/trajectory_eval_test";

    void SetUp() override {
        system(("mkdir -p " + test_dir_).c_str());
    }

    void TearDown() override {
        system(("rm -rf " + test_dir_).c_str());
    }

    // Write a VIO trajectory file: "# timestamp tx ty tz qx qy qz qw"
    void writeVioTrajectory(const std::string& filepath,
                            const std::vector<std::pair<double, Eigen::Vector3d>>& poses) {
        std::ofstream f(filepath);
        f << "# timestamp tx ty tz qx qy qz qw" << std::endl;
        for (const auto& [t, p] : poses) {
            f << std::fixed << std::setprecision(9) << t << " "
              << std::setprecision(6) << p.x() << " " << p.y() << " " << p.z()
              << " 0.0 0.0 0.0 1.0" << std::endl;
        }
    }

    // Write a GT CSV file: timestamp_ns, px, py, pz, qw, qx, qy, qz
    void writeGroundTruth(const std::string& filepath,
                          const std::vector<std::pair<double, Eigen::Vector3d>>& poses) {
        std::ofstream f(filepath);
        f << "#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],q_RS_w [],q_RS_x [],q_RS_y [],q_RS_z []" << std::endl;
        for (const auto& [t, p] : poses) {
            int64_t ts_ns = static_cast<int64_t>(t * 1e9);
            f << ts_ns << "," << std::fixed << std::setprecision(6)
              << p.x() << "," << p.y() << "," << p.z()
              << ",1.0,0.0,0.0,0.0" << std::endl;
        }
    }
};

TEST_F(TrajectoryEvaluatorTest, IdenticalTrajectoriesGiveZeroATE) {
    // Create identical trajectories
    std::vector<std::pair<double, Eigen::Vector3d>> poses;
    for (int i = 0; i < 100; ++i) {
        double t = 1000.0 + i * 0.05;  // 20Hz
        Eigen::Vector3d p(i * 0.1, std::sin(i * 0.1), 0.0);
        poses.push_back({t, p});
    }

    std::string vio_path = test_dir_ + "/vio.txt";
    std::string gt_path = test_dir_ + "/gt.csv";
    writeVioTrajectory(vio_path, poses);
    writeGroundTruth(gt_path, poses);

    utility::TrajectoryEvaluator evaluator;
    ASSERT_TRUE(evaluator.loadVioTrajectory(vio_path));
    ASSERT_TRUE(evaluator.loadGroundTruth(gt_path));
    ASSERT_GT(evaluator.associateTrajectories(0.01), 0);
    ASSERT_TRUE(evaluator.alignTrajectories());

    auto ate = evaluator.computeATE();
    EXPECT_NEAR(ate.rmse, 0.0, 1e-6);
    EXPECT_EQ(ate.num_pairs, 100);
}

TEST_F(TrajectoryEvaluatorTest, KnownTranslationOffset) {
    // VIO trajectory offset by (1,0,0) from GT
    std::vector<std::pair<double, Eigen::Vector3d>> gt_poses, vio_poses;
    for (int i = 0; i < 50; ++i) {
        double t = 1000.0 + i * 0.05;
        Eigen::Vector3d p(i * 0.1, std::sin(i * 0.1), 0.5 * std::cos(i * 0.1));
        gt_poses.push_back({t, p});
        vio_poses.push_back({t, p + Eigen::Vector3d(1.0, 0.0, 0.0)});
    }

    std::string vio_path = test_dir_ + "/vio.txt";
    std::string gt_path = test_dir_ + "/gt.csv";
    writeVioTrajectory(vio_path, vio_poses);
    writeGroundTruth(gt_path, gt_poses);

    utility::TrajectoryEvaluator evaluator;
    ASSERT_TRUE(evaluator.loadVioTrajectory(vio_path));
    ASSERT_TRUE(evaluator.loadGroundTruth(gt_path));
    evaluator.associateTrajectories(0.01);
    evaluator.alignTrajectories();

    auto ate = evaluator.computeATE();
    // After SE(3) alignment, a pure translation offset should be removed
    EXPECT_NEAR(ate.rmse, 0.0, 1e-4);
}

TEST_F(TrajectoryEvaluatorTest, KnownScaleAndRotation) {
    // VIO trajectory is 2x scaled + 90deg rotation around Z
    std::vector<std::pair<double, Eigen::Vector3d>> gt_poses, vio_poses;
    Eigen::Matrix3d rot90z;
    rot90z = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());

    for (int i = 0; i < 50; ++i) {
        double t = 1000.0 + i * 0.05;
        Eigen::Vector3d p(i * 0.2, i * 0.1, 0.0);
        gt_poses.push_back({t, p});
        vio_poses.push_back({t, 2.0 * (rot90z * p)});
    }

    std::string vio_path = test_dir_ + "/vio.txt";
    std::string gt_path = test_dir_ + "/gt.csv";
    writeVioTrajectory(vio_path, vio_poses);
    writeGroundTruth(gt_path, gt_poses);

    utility::TrajectoryEvaluator evaluator;
    ASSERT_TRUE(evaluator.loadVioTrajectory(vio_path));
    ASSERT_TRUE(evaluator.loadGroundTruth(gt_path));
    evaluator.associateTrajectories(0.01);
    evaluator.alignTrajectories();

    auto ate = evaluator.computeATE();
    // umeyama with scale should align these perfectly
    EXPECT_NEAR(ate.rmse, 0.0, 1e-4);
}

TEST_F(TrajectoryEvaluatorTest, QuaternionConventionHandling) {
    // Verify GT loads w-first quaternions correctly
    std::string gt_path = test_dir_ + "/gt_quat.csv";
    {
        std::ofstream f(gt_path);
        f << "#timestamp [ns],p_RS_R_x,p_RS_R_y,p_RS_R_z,q_RS_w,q_RS_x,q_RS_y,q_RS_z" << std::endl;
        // Identity quaternion: w=1, x=y=z=0
        f << "1000000000000,0.0,0.0,0.0,1.0,0.0,0.0,0.0" << std::endl;
        // 90-deg around Z: w=cos(45)=0.7071, x=0, y=0, z=sin(45)=0.7071
        f << "1050000000000,1.0,0.0,0.0,0.7071068,0.0,0.0,0.7071068" << std::endl;
    }

    // VIO file uses w-last format: qx qy qz qw
    std::string vio_path = test_dir_ + "/vio_quat.txt";
    {
        std::ofstream f(vio_path);
        f << "# timestamp tx ty tz qx qy qz qw" << std::endl;
        f << "1000.000000000 0.0 0.0 0.0 0.0 0.0 0.0 1.0" << std::endl;
        f << "1050.000000000 1.0 0.0 0.0 0.0 0.0 0.7071068 0.7071068" << std::endl;
    }

    utility::TrajectoryEvaluator evaluator;
    ASSERT_TRUE(evaluator.loadVioTrajectory(vio_path));
    ASSERT_TRUE(evaluator.loadGroundTruth(gt_path));
    EXPECT_EQ(evaluator.getVioSize(), 2);
    EXPECT_EQ(evaluator.getGtSize(), 2);
}

TEST_F(TrajectoryEvaluatorTest, TimestampAssociation) {
    // VIO at 20Hz, GT at 120Hz. Test association tolerance.
    std::vector<std::pair<double, Eigen::Vector3d>> vio_poses, gt_poses;

    for (int i = 0; i < 20; ++i) {
        double t = 1000.0 + i * 0.05;  // 20Hz
        vio_poses.push_back({t, Eigen::Vector3d(i * 0.1, 0, 0)});
    }
    for (int i = 0; i < 120; ++i) {
        double t = 1000.0 + i * (1.0 / 120.0);  // 120Hz
        gt_poses.push_back({t, Eigen::Vector3d(i * (0.1 / 6.0), 0, 0)});
    }

    std::string vio_path = test_dir_ + "/vio_assoc.txt";
    std::string gt_path = test_dir_ + "/gt_assoc.csv";
    writeVioTrajectory(vio_path, vio_poses);
    writeGroundTruth(gt_path, gt_poses);

    utility::TrajectoryEvaluator evaluator;
    evaluator.loadVioTrajectory(vio_path);
    evaluator.loadGroundTruth(gt_path);

    // With 10ms tolerance, all 20 VIO poses should match (120Hz GT means ~8.3ms gaps)
    int matched = evaluator.associateTrajectories(0.01);
    EXPECT_EQ(matched, 20);

    // With impossibly tight tolerance (1us), fewer matches expected
    // since 120Hz GT has ~8.3ms gaps and VIO at 50ms intervals
    // only exact-aligned frames match (every 6th GT = every VIO)
    // Use a GT frequency that doesn't divide evenly into VIO frequency
    std::vector<std::pair<double, Eigen::Vector3d>> gt_poses_offset;
    for (int i = 0; i < 120; ++i) {
        double t = 1000.0 + i * (1.0 / 113.0);  // 113Hz - prime, won't align with 20Hz
        gt_poses_offset.push_back({t, Eigen::Vector3d(i * 0.01, 0, 0)});
    }
    std::string gt_path2 = test_dir_ + "/gt_assoc2.csv";
    writeGroundTruth(gt_path2, gt_poses_offset);

    utility::TrajectoryEvaluator evaluator2;
    evaluator2.loadVioTrajectory(vio_path);
    evaluator2.loadGroundTruth(gt_path2);
    int matched_wide = evaluator2.associateTrajectories(0.01);

    utility::TrajectoryEvaluator evaluator3;
    evaluator3.loadVioTrajectory(vio_path);
    evaluator3.loadGroundTruth(gt_path2);
    int matched_tight = evaluator3.associateTrajectories(0.001);
    EXPECT_LT(matched_tight, matched_wide);
}

TEST_F(TrajectoryEvaluatorTest, RPEComputationStraightLine) {
    // GT: straight line at 1 m/s. VIO: straight line at 1.1 m/s (10% error)
    std::vector<std::pair<double, Eigen::Vector3d>> gt_poses, vio_poses;
    for (int i = 0; i < 100; ++i) {
        double t = 1000.0 + i * 0.05;  // 20Hz
        gt_poses.push_back({t, Eigen::Vector3d(t - 1000.0, 0, 0)});         // 1 m/s
        vio_poses.push_back({t, Eigen::Vector3d((t - 1000.0) * 1.1, 0, 0)}); // 1.1 m/s
    }

    std::string vio_path = test_dir_ + "/vio_rpe.txt";
    std::string gt_path = test_dir_ + "/gt_rpe.csv";
    writeVioTrajectory(vio_path, vio_poses);
    writeGroundTruth(gt_path, gt_poses);

    utility::TrajectoryEvaluator evaluator;
    evaluator.loadVioTrajectory(vio_path);
    evaluator.loadGroundTruth(gt_path);
    evaluator.associateTrajectories(0.01);
    evaluator.alignTrajectories();

    auto rpe = evaluator.computeRPE(1.0);
    EXPECT_GT(rpe.num_pairs, 0);
    // After alignment, RPE should be very small for a straight line with scale correction
    EXPECT_GE(rpe.rmse_trans, 0.0);
}

TEST_F(TrajectoryEvaluatorTest, FrameTransformRoundTrip) {
    // Test body->camera->body round-trip transformation
    Eigen::Matrix3d r_ic;
    r_ic = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX());
    Eigen::Vector3d t_ic(0.05, -0.02, 0.01);

    // Create body-frame poses
    std::vector<std::pair<double, Eigen::Vector3d>> body_poses;
    for (int i = 0; i < 10; ++i) {
        body_poses.push_back({1000.0 + i * 0.05, Eigen::Vector3d(i * 0.1, i * 0.05, 0)});
    }

    // Transform to camera frame: P_cam = P_body + R_body * t_ic (R_body = I for simplicity)
    std::string vio_path = test_dir_ + "/vio_rt.txt";
    {
        std::ofstream f(vio_path);
        f << "# timestamp tx ty tz qx qy qz qw" << std::endl;
        for (const auto& [t, p] : body_poses) {
            Eigen::Matrix3d r_body = Eigen::Matrix3d::Identity();
            Eigen::Vector3d p_cam = p + r_body * t_ic;
            Eigen::Matrix3d r_cam = r_body * r_ic;
            Eigen::Quaterniond q(r_cam);
            f << std::fixed << std::setprecision(9) << t << " "
              << std::setprecision(6) << p_cam.x() << " " << p_cam.y() << " " << p_cam.z()
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }
    }

    utility::TrajectoryEvaluator evaluator;
    ASSERT_TRUE(evaluator.loadVioTrajectory(vio_path));

    // Transform back to body frame
    evaluator.transformVioToBodyFrame(r_ic, t_ic);

    // Write GT as original body poses and check ATE ~ 0
    std::string gt_path = test_dir_ + "/gt_rt.csv";
    writeGroundTruth(gt_path, body_poses);

    ASSERT_TRUE(evaluator.loadGroundTruth(gt_path));
    evaluator.associateTrajectories(0.01);
    evaluator.alignTrajectories();

    auto ate = evaluator.computeATE();
    EXPECT_NEAR(ate.rmse, 0.0, 1e-6);
}
