// Copyright 2025 Mobile-SLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

#include "slam/backend/estimator.h"
#include "slam/config/slam_config.h"
#include "slam/frontend/feature_tracker.h"
#include "slam/utility/dataset_loader.h"
#include "slam/utility/math_utils.h"

// ---------------------------------------------------------------------------
// run_tum_vi -- Run the Mobile-SLAM VIO pipeline on a TUM VI dataset.
//
// Usage:
//   ./run_tum_vi <dataset_path> [config_file]
//
// If no config file is given, uses DefaultTumVi() settings.
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <dataset_path> [config_file]\n";
    return EXIT_FAILURE;
  }

  const std::string dataset_path = argv[1];

  // -- Load configuration ---------------------------------------------------
  slam::config::SlamConfig config;
  if (argc >= 3) {
    if (!config.loadFromFile(argv[2])) {
      std::cerr << "[ERROR] Failed to load config: " << argv[2] << "\n";
      return EXIT_FAILURE;
    }
  } else {
    config = slam::config::SlamConfig::DefaultTumVi();
  }
  config.dataset_path = dataset_path;
  config.print();

  // -- Load dataset ---------------------------------------------------------
  slam::utility::TumViLoader loader(dataset_path);
  if (!loader.load()) {
    std::cerr << "[ERROR] Failed to load dataset: " << dataset_path << "\n";
    return EXIT_FAILURE;
  }

  const auto& images = loader.image_timestamps();
  const auto& imu_data = loader.imu_measurements();

  std::cout << "\n[INFO] Dataset loaded:\n"
            << "  Images: " << images.size() << "\n"
            << "  IMU:    " << imu_data.size() << "\n"
            << "  GT:     " << loader.ground_truth().size() << "\n\n";

  if (images.empty() || imu_data.empty()) {
    std::cerr << "[ERROR] No image or IMU data found.\n";
    return EXIT_FAILURE;
  }

  // -- Create pipeline components -------------------------------------------
  slam::frontend::FeatureTracker tracker(
      config.camera.fx, config.camera.cx, config.camera.cy,
      config.feature_tracker.max_cnt,
      config.feature_tracker.min_dist,
      config.feature_tracker.f_threshold,
      config.feature_tracker.equalize);

  slam::backend::Estimator estimator(config);

  // -- Main processing loop -------------------------------------------------
  slam::utility::TicToc total_timer;
  slam::utility::TicToc frame_timer;
  int imu_idx = 0;
  double last_image_time = -1.0;

  for (size_t img_i = 0; img_i < images.size(); ++img_i) {
    double img_time = images[img_i].first;

    // Feed IMU measurements between last image and current image
    if (last_image_time > 0.0) {
      auto imu_between = loader.getImuBetween(last_image_time, img_time);
      double prev_t = last_image_time;

      for (const auto& imu : imu_between) {
        double dt = imu.timestamp - prev_t;
        if (dt <= 0.0) continue;
        estimator.processIMU(dt, imu.linear_acceleration,
                             imu.angular_velocity);
        prev_t = imu.timestamp;
      }

      // Fill gap between last IMU and current image time
      if (prev_t < img_time) {
        double dt = img_time - prev_t;
        if (!imu_between.empty()) {
          estimator.processIMU(dt, imu_between.back().linear_acceleration,
                               imu_between.back().angular_velocity);
        }
      }
    }

    // Track features in the image
    frame_timer.Tic();
    cv::Mat image = loader.loadImage(img_i);
    if (image.empty()) {
      std::cerr << "[WARN] Failed to load image " << img_i << "\n";
      last_image_time = img_time;
      continue;
    }

    auto observations = tracker.trackImage(img_time, image);

    // Process image features
    estimator.processImage(observations, img_time);

    double frame_ms = frame_timer.Toc();

    // Print progress
    if (img_i % 50 == 0 || img_i == images.size() - 1) {
      const char* state =
          (estimator.getSolverFlag() == slam::common::SolverFlag::kNonLinear)
              ? "RUNNING"
              : "INIT";
      Eigen::Vector3d pos = estimator.getPosition();

      std::cout << "[" << state << "] Frame " << std::setw(5) << img_i
                << "/" << images.size()
                << "  t=" << std::fixed << std::setprecision(3) << img_time
                << "  features=" << std::setw(3) << observations.size()
                << "  pos=(" << std::setprecision(3) << pos.x()
                << ", " << pos.y() << ", " << pos.z() << ")"
                << "  " << std::setprecision(1) << frame_ms << "ms\n";
    }

    last_image_time = img_time;
  }

  double total_sec = total_timer.Toc() / 1000.0;
  std::cout << "\n[DONE] Processed " << images.size() << " frames in "
            << std::fixed << std::setprecision(1) << total_sec << "s"
            << " (" << std::setprecision(1)
            << (images.size() / total_sec) << " fps)\n";

  return EXIT_SUCCESS;
}
