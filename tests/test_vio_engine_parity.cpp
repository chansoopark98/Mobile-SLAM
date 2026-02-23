/**
 * VIOEngine vs Native Pipeline Parity Test
 *
 * Feeds the same TUM VI dataset through:
 *   1. VIOEngine (headless WASM-equivalent path)
 *   2. Native pipeline (MeasurementProcessor → Estimator, same as VIOSystem)
 *
 * Both share g_config, so they run SEQUENTIALLY with full state reset between.
 * Verifies both produce functionally equivalent poses.
 */

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>
#include <map>
#include <cmath>
#include <iostream>
#include <iomanip>

#include "vio_engine.h"
#include "backend/estimator.h"
#include "utility/config.h"
#include "utility/measurement_processor.h"

using namespace Eigen;

// ── Dataset Config ──
static const std::string DATASET_PATH = "./assets/datasets/tum/dataset-room1_512_16";
static const std::string CONFIG_PATH  = "./config/tum_vi_room1.yaml";

// ── Structures ──
struct TimestampedPose {
    double timestamp;
    Vector3d position;
    Matrix3d rotation;
};

// ── Raw IMU loading (for VIOEngine path) ──
struct RawIMU {
    double timestamp;
    double gx, gy, gz;
    double ax, ay, az;
};

static std::vector<RawIMU> loadIMUCSV(const std::string& path) {
    std::vector<RawIMU> data;
    std::ifstream file(path);
    if (!file.is_open()) return data;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string token;
        std::vector<double> vals;
        while (std::getline(ss, token, ',')) {
            try { vals.push_back(std::stod(token)); }
            catch (...) { break; }
        }
        if (vals.size() >= 7) {
            RawIMU imu;
            imu.timestamp = vals[0] * 1e-9;  // ns → s
            imu.gx = vals[1]; imu.gy = vals[2]; imu.gz = vals[3];
            imu.ax = vals[4]; imu.ay = vals[5]; imu.az = vals[6];
            data.push_back(imu);
        }
    }
    return data;
}

struct ImageEntry {
    double timestamp;
    std::string full_path;
};

static std::vector<ImageEntry> loadImageCSV(const std::string& csv_path, const std::string& img_dir) {
    std::vector<ImageEntry> entries;
    std::ifstream file(csv_path);
    if (!file.is_open()) return entries;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        auto comma = line.find(',');
        if (comma == std::string::npos) continue;
        ImageEntry e;
        e.timestamp = std::stod(line.substr(0, comma)) * 1e-9;
        std::string fname = line.substr(comma + 1);
        while (!fname.empty() && (fname.back() == '\r' || fname.back() == ' '))
            fname.pop_back();
        e.full_path = img_dir + "/" + fname;
        entries.push_back(e);
    }
    std::sort(entries.begin(), entries.end(),
              [](const ImageEntry& a, const ImageEntry& b) { return a.timestamp < b.timestamp; });
    return entries;
}

// Slice IMU readings in (tPrev, tCurr]
static std::vector<IMUReading> sliceIMUForEngine(const std::vector<RawIMU>& all,
                                                  double tPrev, double tCurr) {
    std::vector<IMUReading> slice;
    for (const auto& imu : all) {
        if (imu.timestamp > tPrev && imu.timestamp <= tCurr) {
            IMUReading r;
            r.timestamp = imu.timestamp;
            r.acc_x = imu.ax; r.acc_y = imu.ay; r.acc_z = imu.az;
            r.gyro_x = imu.gx; r.gyro_y = imu.gy; r.gyro_z = imu.gz;
            slice.push_back(r);
        }
        if (imu.timestamp > tCurr) break;
    }
    return slice;
}

// ── Test Fixture ──
class VIOParityTest : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_TRUE(std::ifstream(DATASET_PATH + "/mav0/cam0/data.csv").good())
            << "TUM VI dataset not found at: " << DATASET_PATH;
        ASSERT_TRUE(std::ifstream(CONFIG_PATH).good())
            << "Config not found at: " << CONFIG_PATH;
    }

    // Run VIOEngine on TUM VI (same path as WASM)
    std::vector<TimestampedPose> runVIOEngine(int max_frames) {
        std::vector<TimestampedPose> trajectory;

        auto all_imu = loadIMUCSV(DATASET_PATH + "/mav0/imu0/data.csv");
        auto images = loadImageCSV(DATASET_PATH + "/mav0/cam0/data.csv",
                                   DATASET_PATH + "/mav0/cam0/data");
        EXPECT_GT(all_imu.size(), 0u);
        EXPECT_GT(images.size(), 0u);

        // Reset g_config and load YAML
        utility::g_config = utility::Config{};
        EXPECT_TRUE(utility::g_config.loadFromYaml(CONFIG_PATH));
        auto& cfg = utility::g_config;

        // Read distortion from YAML
        double k2 = 0, k3 = 0, k4 = 0, k5 = 0;
        {
            YAML::Node yaml = YAML::LoadFile(CONFIG_PATH);
            if (yaml["projection_parameters"]) {
                auto p = yaml["projection_parameters"];
                if (p["k2"]) k2 = p["k2"].as<double>();
                if (p["k3"]) k3 = p["k3"].as<double>();
                if (p["k4"]) k4 = p["k4"].as<double>();
                if (p["k5"]) k5 = p["k5"].as<double>();
            }
        }

        VIOEngine engine;
        bool ok = engine.configure(
            static_cast<int>(cfg.camera.col), static_cast<int>(cfg.camera.row),
            cfg.camera.fx, cfg.camera.fy, cfg.camera.cx, cfg.camera.cy,
            1,  // KANNALA_BRANDT
            k2, k3, k4, k5,
            cfg.camera.r_ic.data(), cfg.camera.t_ic.data(),
            cfg.estimator.acc_n, cfg.estimator.acc_w,
            cfg.estimator.gyr_n, cfg.estimator.gyr_w,
            cfg.estimator.g.norm()
        );
        EXPECT_TRUE(ok);

        // Match native solver params
        engine.setMobileParams(cfg.estimator.solver_time,
                               cfg.estimator.num_iterations,
                               cfg.feature_tracker.max_cnt);

        double pose_output[16];
        int n = std::min(max_frames, static_cast<int>(images.size()));
        double prev_ts = all_imu.empty() ? 0.0 : all_imu[0].timestamp - 1e-9;

        for (int i = 0; i < n; i++) {
            cv::Mat img = cv::imread(images[i].full_path, cv::IMREAD_GRAYSCALE);
            if (img.empty()) continue;

            double ts = images[i].timestamp;
            auto imu_slice = sliceIMUForEngine(all_imu, prev_ts, ts);
            prev_ts = ts;

            bool has_pose = engine.processFrame(
                img.data, img.cols, img.rows,
                imu_slice.data(), static_cast<int>(imu_slice.size()),
                ts, pose_output
            );

            if (has_pose) {
                TimestampedPose p;
                p.timestamp = ts;
                p.position = Vector3d(pose_output[3], pose_output[7], pose_output[11]);
                p.rotation << pose_output[0], pose_output[1], pose_output[2],
                              pose_output[4], pose_output[5], pose_output[6],
                              pose_output[8], pose_output[9], pose_output[10];
                trajectory.push_back(p);
            }
        }
        std::cout << "[VIOEngine] " << n << " frames → " << trajectory.size() << " poses" << std::endl;
        return trajectory;
    }

    // Run Native pipeline: MeasurementProcessor → Estimator (same as VIOSystem without visualization)
    std::vector<TimestampedPose> runNativePipeline(int max_frames) {
        std::vector<TimestampedPose> trajectory;

        // Reset g_config and load YAML
        utility::g_config = utility::Config{};
        EXPECT_TRUE(utility::g_config.loadFromYaml(CONFIG_PATH));

        // Initialize MeasurementProcessor (loads data, creates feature tracker from YAML)
        utility::MeasurementProcessor mp;
        bool init_ok = mp.initialize(
            DATASET_PATH + "/mav0/imu0/data.csv",
            DATASET_PATH + "/mav0/cam0/data.csv",
            DATASET_PATH + "/mav0/cam0/data",
            CONFIG_PATH
        );
        EXPECT_TRUE(init_ok);

        // Create estimator (reads g_config)
        backend::Estimator estimator;
        estimator.setParameter();

        const auto& image_file_data = mp.getImageFileData();
        int n = std::min(max_frames, static_cast<int>(image_file_data.size()));
        double current_time = -1;
        int32_t mid = 0;

        for (int i = 0; i < n; i++) {
            mid++;
            auto measurement = mp.createMeasurementMsg(mid, image_file_data[i]);
            auto& imu_msgs = measurement.imu_msg;
            auto& img_msg = measurement.image_feature_msg;

            // Process IMU (same as VIOSystem::processIMUData)
            Vector3d prev_acc = Vector3d::Zero();
            Vector3d prev_gyro = Vector3d::Zero();
            for (const auto& imu : imu_msgs) {
                double imu_time = imu.timestamp;
                double image_time = img_msg.timestamp;
                Vector3d acc(imu.linear_acc_x, imu.linear_acc_y, imu.linear_acc_z);
                Vector3d gyro(imu.angular_vel_x, imu.angular_vel_y, imu.angular_vel_z);

                if (imu_time <= image_time) {
                    if (current_time < 0.0) current_time = imu_time;
                    double dt = imu_time - current_time;
                    current_time = imu_time;
                    estimator.processIMU(dt, acc, gyro);
                } else {
                    double dt_to_image = image_time - current_time;
                    current_time = image_time;
                    double total_dt = dt_to_image + (imu_time - image_time);
                    Vector3d interp_acc, interp_gyro;
                    if (total_dt < 1e-12) {
                        interp_acc = acc; interp_gyro = gyro;
                    } else {
                        double w1 = (imu_time - image_time) / total_dt;
                        double w2 = dt_to_image / total_dt;
                        interp_acc = w1 * prev_acc + w2 * acc;
                        interp_gyro = w1 * prev_gyro + w2 * gyro;
                    }
                    estimator.processIMU(dt_to_image, interp_acc, interp_gyro);
                }
                prev_acc = acc;
                prev_gyro = gyro;
            }

            // Process image features
            common::ImageData image_data;
            for (unsigned int j = 0; j < img_msg.points_count; j++) {
                int fid = static_cast<int>(img_msg.channel_data[0][j]);
                Eigen::Matrix<double, 7, 1> v;
                v << img_msg.ray_vectors[j].x, img_msg.ray_vectors[j].y, img_msg.ray_vectors[j].z,
                     img_msg.channel_data[1][j], img_msg.channel_data[2][j],
                     img_msg.channel_data[3][j], img_msg.channel_data[4][j];
                image_data[fid] = v;
            }
            if (!image_data.empty()) {
                estimator.processImage(image_data, img_msg.timestamp);
            }

            // Extract pose
            if (estimator.solver_flag_ == common::SolverFlag::NON_LINEAR) {
                int ws = WINDOW_SIZE;
                Vector3d body_pos = estimator.sliding_window_[ws].P;
                Matrix3d body_rot = estimator.sliding_window_[ws].R;
                if (body_pos.allFinite() && body_rot.allFinite()) {
                    TimestampedPose p;
                    p.timestamp = img_msg.timestamp;
                    p.position = body_pos + body_rot * estimator.t_ic_;
                    p.rotation = body_rot * estimator.r_ic_;
                    trajectory.push_back(p);
                }
            }
        }
        std::cout << "[Native]    " << n << " frames → " << trajectory.size() << " poses" << std::endl;
        return trajectory;
    }
};

// ── Test: Both produce trajectories ──
TEST_F(VIOParityTest, BothProduceTrajectories) {
    const int N = 300;

    std::cout << "\n=== VIOEngine ===" << std::endl;
    auto eng = runVIOEngine(N);

    std::cout << "\n=== Native Pipeline ===" << std::endl;
    auto nat = runNativePipeline(N);

    EXPECT_GT(eng.size(), 0u) << "VIOEngine produced no poses";
    EXPECT_GT(nat.size(), 0u) << "Native pipeline produced no poses";

    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "VIOEngine: " << eng.size() << " poses" << std::endl;
    std::cout << "Native:    " << nat.size() << " poses" << std::endl;

    if (!eng.empty()) {
        auto& p = eng.back();
        std::cout << "VIOEngine last: [" << std::fixed << std::setprecision(4)
                  << p.position.x() << ", " << p.position.y() << ", " << p.position.z() << "]" << std::endl;
    }
    if (!nat.empty()) {
        auto& p = nat.back();
        std::cout << "Native    last: [" << std::fixed << std::setprecision(4)
                  << p.position.x() << ", " << p.position.y() << ", " << p.position.z() << "]" << std::endl;
    }
}

// ── Test: Pose-by-pose comparison ──
TEST_F(VIOParityTest, TrajectoryComparison) {
    const int N = 300;
    auto eng = runVIOEngine(N);
    auto nat = runNativePipeline(N);

    if (eng.empty() || nat.empty()) {
        GTEST_SKIP() << "One pipeline failed to initialize";
    }

    // Timestamp-match (within 1ms)
    std::map<double, TimestampedPose> nat_map;
    for (const auto& p : nat) nat_map[p.timestamp] = p;

    int matched = 0;
    double sum_pos = 0, max_pos = 0, sum_rot = 0, max_rot = 0;

    for (const auto& ep : eng) {
        auto it = nat_map.lower_bound(ep.timestamp - 0.001);
        if (it == nat_map.end() || std::abs(it->first - ep.timestamp) > 0.001) continue;
        const auto& np = it->second;

        double pe = (ep.position - np.position).norm();
        sum_pos += pe;
        max_pos = std::max(max_pos, pe);

        Matrix3d Rd = ep.rotation.transpose() * np.rotation;
        double tr = std::min(3.0, std::max(-1.0, Rd.trace()));
        double re = std::acos((tr - 1.0) / 2.0);
        sum_rot += re;
        max_rot = std::max(max_rot, re);
        matched++;
    }

    std::cout << "\n=== Parity ===" << std::endl;
    std::cout << "Matched: " << matched << " poses" << std::endl;

    if (matched > 0) {
        double avg_p = sum_pos / matched;
        double avg_r = sum_rot / matched * 180.0 / M_PI;
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "Position - avg: " << avg_p << " m, max: " << max_pos << " m" << std::endl;
        std::cout << "Rotation - avg: " << avg_r << " deg, max: " << max_rot * 180.0 / M_PI << " deg" << std::endl;

        // Both pipelines run independent feature trackers on the same images.
        // OpenCV KLT tracking has non-determinism, so exact parity is NOT expected.
        // But both should produce similar trajectories (same algorithm, same params).
        EXPECT_LT(avg_p, 1.0) << "Position divergence too large";
        EXPECT_LT(avg_r, 10.0) << "Rotation divergence too large";
    }
}

// ── Test: VIOEngine output sanity ──
TEST_F(VIOParityTest, VIOEngineSanity) {
    auto traj = runVIOEngine(200);
    for (const auto& p : traj) {
        EXPECT_TRUE(p.position.allFinite()) << "NaN at t=" << p.timestamp;
        EXPECT_TRUE(p.rotation.allFinite()) << "NaN at t=" << p.timestamp;
        EXPECT_LT(p.position.norm(), 100.0) << "Position too large at t=" << p.timestamp;
        Matrix3d I = p.rotation.transpose() * p.rotation;
        EXPECT_NEAR((I - Matrix3d::Identity()).norm(), 0.0, 1e-4) << "Non-orthogonal rotation";
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
