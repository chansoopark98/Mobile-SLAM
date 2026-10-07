// Headless audit harness; no production-source changes.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "common/camera_models/Camera.h"
#include "vio_engine.h"

namespace {
using Clock = std::chrono::steady_clock;
struct ImageEntry {
    double timestamp;
    std::filesystem::path path;
};

std::vector<std::string> fields(const std::string& line) {
    std::vector<std::string> result;
    std::istringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) result.push_back(field);
    return result;
}

std::vector<IMUReading> loadIMU(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open IMU CSV: " + path.string());
    std::vector<IMUReading> readings;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '#') continue;
        const auto row = fields(line);
        if (row.size() < 7) throw std::runtime_error("Malformed IMU CSV row");
        IMUReading reading{};
        reading.timestamp = static_cast<double>(std::stoll(row[0])) * 1e-9;
        reading.gyro_x = std::stod(row[1]);
        reading.gyro_y = std::stod(row[2]);
        reading.gyro_z = std::stod(row[3]);
        reading.acc_x = std::stod(row[4]);
        reading.acc_y = std::stod(row[5]);
        reading.acc_z = std::stod(row[6]);
        if (!readings.empty() && reading.timestamp <= readings.back().timestamp)
            throw std::runtime_error("Non-increasing IMU timestamps");
        readings.push_back(reading);
    }
    if (readings.empty()) throw std::runtime_error("Empty IMU CSV");
    return readings;
}

std::vector<ImageEntry> loadImages(const std::filesystem::path& path) {
    std::ifstream input(path / "mav0/cam0/data.csv");
    if (!input) throw std::runtime_error("Cannot open camera CSV");
    std::vector<ImageEntry> images;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '#') continue;
        const auto row = fields(line);
        if (row.size() != 2) throw std::runtime_error("Malformed camera CSV row");
        const double timestamp = static_cast<double>(std::stoll(row[0])) * 1e-9;
        std::string filename = row[1];
        while (!filename.empty() && (filename.back() == '\r' || filename.back() == ' '))
            filename.pop_back();
        const std::filesystem::path relative(filename);
        if (relative.is_absolute() || relative.has_parent_path())
            throw std::runtime_error("Unsafe image filename in CSV");
        if (!images.empty() && timestamp <= images.back().timestamp)
            throw std::runtime_error("Non-increasing camera timestamps");
        images.push_back({timestamp, path / "mav0/cam0/data" / relative});
    }
    if (images.empty()) throw std::runtime_error("Empty camera CSV");
    return images;
}

double elapsedMs(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

double quantile(std::vector<double> values, double q) {
    if (values.empty()) return 0;
    std::sort(values.begin(), values.end());
    const double index = q * static_cast<double>(values.size() - 1);
    const auto lower = static_cast<size_t>(index);
    const auto upper = std::min(lower + 1, values.size() - 1);
    return values[lower] + (index - lower) * (values[upper] - values[lower]);
}

void timingJson(std::ostream& output, const std::vector<double>& values) {
    const double total = std::accumulate(values.begin(), values.end(), 0.0);
    output << "{\"count\":" << values.size()
           << ",\"mean\":" << (values.empty() ? 0 : total / values.size())
           << ",\"p50\":" << quantile(values, 0.50)
           << ",\"p95\":" << quantile(values, 0.95)
           << ",\"p99\":" << quantile(values, 0.99)
           << ",\"max\":" << (values.empty() ? 0 : *std::max_element(values.begin(), values.end()))
           << ",\"total\":" << total << "}";
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 4 || argc > 14) {
        std::cerr << "Usage: native-replay DATASET CONFIG OUTPUT [MAX_FRAMES=600] "
                     "[IMU_POLICY=bracket|until-image] [PNP=0|1] [CV_THREADS=1] [ITERATIONS=YAML] [SOLVER_TIME=YAML] [DUMP_INPUTS=0|1] [DIAGNOSTICS=0|1] [CAPTURE_START=0] [CAPTURE_END=PROCESSED_FRAMES]\n";
        return 2;
    }
    try {
        const auto dataset = std::filesystem::absolute(argv[1]);
        const auto config_path = std::filesystem::absolute(argv[2]);
        const auto output_dir = std::filesystem::absolute(argv[3]);
        const int max_frames = argc > 4 ? std::stoi(argv[4]) : 600;
        const std::string imu_policy = argc > 5 ? argv[5] : "bracket";
        const int pnp = argc > 6 ? std::stoi(argv[6]) : 0;
        const int cv_threads = argc > 7 ? std::stoi(argv[7]) : 1;
        const char* benchmark_environment = std::getenv("MOBILE_SLAM_BENCHMARK_ZERO_TOLERANCES");
        const std::string benchmark_option = benchmark_environment ? benchmark_environment : "0";
        if (benchmark_option != "0" && benchmark_option != "1")
            throw std::runtime_error("MOBILE_SLAM_BENCHMARK_ZERO_TOLERANCES must be 0 or 1");
        const bool benchmark_requested = benchmark_option == "1";
        if (max_frames < 1 || cv_threads > 4 || (imu_policy != "bracket" && imu_policy != "until-image") ||
            (pnp != 0 && pnp != 1) || cv_threads < 1)
            throw std::runtime_error("Invalid replay option");
        std::filesystem::create_directories(output_dir);
        if (std::filesystem::exists(output_dir / "summary.json") ||
            std::filesystem::exists(output_dir / "trajectory-camera.txt") ||
            std::filesystem::exists(output_dir / "frames.csv"))
            throw std::runtime_error("Output already exists; choose a new directory");

        utility::Config desired;
        if (!desired.loadFromYaml(config_path.string())) throw std::runtime_error("Config load failed");
        if (argc > 8) desired.estimator.num_iterations = std::stoi(argv[8]);
        if (argc > 9) desired.estimator.solver_time = std::stod(argv[9]);
        if (desired.estimator.num_iterations < 1 || desired.estimator.num_iterations > 100 || !std::isfinite(desired.estimator.solver_time) || desired.estimator.solver_time <= 0 || desired.estimator.solver_time > 10) throw std::runtime_error("Invalid solver budget");
        const YAML::Node yaml = YAML::LoadFile(config_path.string());
        const std::string model = yaml["model_type"].as<std::string>();
        const int camera_model = model == "KANNALA_BRANDT"
            ? common::camera_models::Camera::KANNALA_BRANDT
            : model == "PINHOLE" ? common::camera_models::Camera::PINHOLE : -1;
        if (camera_model < 0) throw std::runtime_error("Unsupported camera model");
        const YAML::Node distortion = model == "KANNALA_BRANDT"
            ? yaml["projection_parameters"] : yaml["distortion_parameters"];
        const std::array<std::string, 4> names = model == "KANNALA_BRANDT"
            ? std::array<std::string, 4>{"k2", "k3", "k4", "k5"}
            : std::array<std::string, 4>{"k1", "k2", "p1", "p2"};
        std::array<double, 4> k{};
        for (size_t i = 0; i < k.size(); ++i)
            if (distortion && distortion[names[i]]) k[i] = distortion[names[i]].as<double>();
        std::array<double, 9> rotation{};
        std::array<double, 3> translation{};
        for (int r = 0; r < 3; ++r) {
            translation[r] = desired.camera.t_ic(r);
            for (int c = 0; c < 3; ++c) rotation[r * 3 + c] = desired.camera.r_ic(r, c);
        }
        if (!desired.camera.r_ic.allFinite() || !desired.camera.t_ic.allFinite() ||
            (desired.camera.r_ic.transpose() * desired.camera.r_ic - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
            std::abs(desired.camera.r_ic.determinant() - 1.0) > 1e-6)
            throw std::runtime_error("Invalid extrinsic transform");

        const auto images = loadImages(dataset);
        const auto imu = loadIMU(dataset / "mav0/imu0/data.csv");
        const size_t count = std::min(static_cast<size_t>(max_frames), images.size());
        const int capture_start = argc > 12 ? std::stoi(argv[12]) : 0;
        const int capture_end = argc > 13 ? std::stoi(argv[13]) : static_cast<int>(count);
        if (capture_start < 0 || capture_end <= capture_start || capture_end > static_cast<int>(count))
            throw std::runtime_error("Invalid diagnostic capture range [start,end)");
        if (imu.back().timestamp < images.back().timestamp)
            throw std::runtime_error("IMU does not cover camera sequence");
        utility::g_config = desired;
        VIOEngine engine;
        if (!engine.configure(static_cast<int>(desired.camera.col), static_cast<int>(desired.camera.row),
                              desired.camera.fx, desired.camera.fy, desired.camera.cx, desired.camera.cy,
                              camera_model, k[0], k[1], k[2], k[3], rotation.data(), translation.data(),
                              desired.estimator.acc_n, desired.estimator.acc_w,
                              desired.estimator.gyr_n, desired.estimator.gyr_w, desired.estimator.g.norm()))
            throw std::runtime_error("Engine configure failed");
        engine.setExecutionParams(0, cv_threads);
        engine.setBenchmarkSolverProfile(benchmark_requested);
        if (engine.getBenchmarkSolverProfile() != benchmark_requested)
            throw std::runtime_error("Benchmark solver profile readback mismatch");
        const bool diagnostic_capture = argc > 11 && std::stoi(argv[11]) == 1;
        bool diagnostic_active = diagnostic_capture && capture_start == 0;
        engine.setDiagnosticCapture(diagnostic_active);
        // configure overrides defaults; restore independent YAML copy after construction.
        utility::g_config = desired;
        utility::g_config.feature_tracker.show_track = 0;
        engine.setMobileParams(desired.estimator.solver_time, desired.estimator.num_iterations, desired.feature_tracker.max_cnt);
        engine.setFThreshold(1.0);
        engine.setTrackingParams(21, 3, 20, 0.0);
        engine.setPnPParams(pnp != 0, 3);

        std::ofstream frames(output_dir / "frames.csv");
        std::ofstream trajectory(output_dir / "trajectory-camera.txt");
        const int dump_input_option = argc > 10 ? std::stoi(argv[10]) : 0;
        if (dump_input_option != 0 && dump_input_option != 1) throw std::runtime_error("Invalid input oracle option");
        const bool dump_inputs = dump_input_option == 1;
        std::ofstream raw_inputs;
        if (dump_inputs) raw_inputs.open(output_dir / "input-oracle.bin", std::ios::binary);
        std::ofstream results(output_dir / "results.json");
        results << "{\"frames\":[";
        results << std::setprecision(17);
        if (!frames || !trajectory) throw std::runtime_error("Cannot create output files");
        frames << "frame,timestamp_s,image_io_ms,process_ms,imu_count,features,status,has_pose,valid_pose,epoch,pose_timestamp_s,reason,solver_iterations,solver_termination\n";
        trajectory << "# timestamp tx ty tz qx qy qz qw; camera-to-world pose\n";
        frames << std::setprecision(15);
        trajectory << std::fixed << std::setprecision(12);
        std::vector<double> process_times, tracking_times, image_times;
        std::array<size_t, 5> status_counts{};
        size_t imu_cursor = 0, imu_delivered = 0, pose_count = 0, invalid_count = 0, init_loss = 0;
        int first_pose_frame = -1, previous_status = static_cast<int>(VIOStatus::INITIALIZING);
        double max_position = 0, max_rotation_error = 0;
        const auto replay_start = Clock::now();
        for (size_t i = 0; i < count; ++i) {
            const bool selected_diagnostic = diagnostic_capture && i >= static_cast<size_t>(capture_start) &&
                                             i < static_cast<size_t>(capture_end);
            if (selected_diagnostic != diagnostic_active) {
                engine.setDiagnosticCapture(selected_diagnostic);
                diagnostic_active = selected_diagnostic;
            }
            const auto image_start = Clock::now();
            const cv::Mat image = cv::imread(images[i].path.string(), cv::IMREAD_GRAYSCALE);
            const double image_ms = elapsedMs(image_start);
            if (image.empty() || !image.isContinuous() || image.cols != static_cast<int>(desired.camera.col) ||
                image.rows != static_cast<int>(desired.camera.row))
                throw std::runtime_error("Image missing or size mismatch: " + images[i].path.string());
            const size_t begin = imu_cursor;
            if (imu_cursor == 0 || imu[imu_cursor - 1].timestamp <= images[i].timestamp) {
                while (imu_cursor < imu.size() && imu[imu_cursor].timestamp <= images[i].timestamp) ++imu_cursor;
                if (imu_policy == "bracket" && imu_cursor < imu.size()) ++imu_cursor;
            }
            const size_t end = imu_cursor;
            imu_delivered += end - begin;
            if (dump_inputs) {
                const auto write_u32 = [&](uint32_t value) {for (int byte=0;byte<4;++byte) raw_inputs.put(static_cast<char>((value>>(8*byte))&255));};
                write_u32(static_cast<uint32_t>(image.total()));
                raw_inputs.write(reinterpret_cast<const char*>(image.data), static_cast<std::streamsize>(image.total()));
                write_u32(static_cast<uint32_t>(end-begin));
                for (size_t sample=begin;sample<end;++sample) {
                    const auto& reading=imu[sample];
                    const double values[7]={reading.timestamp,reading.acc_x,reading.acc_y,reading.acc_z,reading.gyro_x,reading.gyro_y,reading.gyro_z};
                    raw_inputs.write(reinterpret_cast<const char*>(values),sizeof(values));
                }
                if (!raw_inputs) throw std::runtime_error("Input oracle write failed");
            }
            std::array<double, 16> pose{};
            const auto process_start = Clock::now();
            const bool has_pose = engine.processFrame(image.data, image.cols, image.rows,
                imu.data() + begin, static_cast<int>(end - begin), images[i].timestamp, pose.data());
            const double process_ms = elapsedMs(process_start);
            const int status = engine.getStatusCode();
            if (status < 0 || status >= static_cast<int>(status_counts.size()))
                throw std::runtime_error("Unexpected status code");
            ++status_counts[status];
            if (previous_status == static_cast<int>(VIOStatus::TRACKING) && status != previous_status) ++init_loss;
            previous_status = status;
            bool valid = false;
            if (has_pose && engine.getPoseFresh() && engine.getPoseValid() && std::abs(engine.getPoseTimestamp()-images[i].timestamp) < 1e-6) {
                Eigen::Matrix3d r;
                Eigen::Vector3d t;
                for (int a = 0; a < 3; ++a) {
                    t(a) = pose[a * 4 + 3];
                    for (int b = 0; b < 3; ++b) r(a, b) = pose[a * 4 + b];
                }
                const double rotation_error = (r.transpose() * r - Eigen::Matrix3d::Identity()).norm();
                valid = r.allFinite() && t.allFinite() && rotation_error < 1e-4 && std::abs(r.determinant() - 1) < 1e-4;
                if (valid) {
                    const Eigen::Quaterniond q(r);
                    trajectory << engine.getPoseTimestamp() << ' ' << t.x() << ' ' << t.y() << ' ' << t.z()
                               << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
                    ++pose_count;
                    if (first_pose_frame < 0) first_pose_frame = static_cast<int>(i);
                    max_position = std::max(max_position, t.norm());
                    max_rotation_error = std::max(max_rotation_error, rotation_error);
                } else ++invalid_count;
            }
            process_times.push_back(process_ms);
            image_times.push_back(image_ms);
            if (status == static_cast<int>(VIOStatus::TRACKING)) tracking_times.push_back(process_ms);
            frames << i << ',' << images[i].timestamp << ',' << image_ms << ',' << process_ms << ','
                   << (end - begin) << ',' << engine.getFeaturePointCount() << ',' << status << ','
                   << has_pose << ',' << valid << ',' << engine.getEpoch() << ',' << engine.getPoseTimestamp() << ','
                   << engine.getLastReason() << ',' << engine.getLastSolverIterations() << ',' << engine.getLastSolverTermination() << '\n';
            results << (i ? "," : "") << "{\"frame\":" << i << ",\"inputTimestamp\":" << images[i].timestamp
                    << ",\"poseTimestamp\":" << engine.getPoseTimestamp() << ",\"engineEpoch\":" << engine.getEpoch()
                    << ",\"statusCode\":" << status << ",\"poseValid\":" << (valid ? "true" : "false")
                    << ",\"reason\":\"" << engine.getLastReason() << "\",\"solverIterations\":" << engine.getLastSolverIterations()
                    << ",\"solverTermination\":\"" << engine.getLastSolverTermination() << "\",\"imuCount\":" << end-begin << ",\"pose\":";
            if (valid) {results << '['; for(size_t k=0;k<pose.size();++k) results << (k ? "," : "") << pose[k];results << ']';}
            else results << "null";
            if (diagnostic_capture) results << ",\"featureDiagnostics\":" << engine.getFeatureDiagnostics();
            results << '}';
            if ((i + 1) % 100 == 0) std::cerr << "[Replay] " << (i + 1) << '/' << count
                << " frames, " << pose_count << " valid poses\n";
        }
        const double replay_ms = elapsedMs(replay_start);
        results << "]}\n";
        results.flush();
        frames.flush();
        trajectory.flush();
        if (!frames || !trajectory) throw std::runtime_error("Failed to write replay data");
        std::ofstream summary(output_dir / "summary.json");
        if (!summary) throw std::runtime_error("Cannot create summary");
        summary << std::setprecision(17)
                << "{\n\"frames_available\":" << images.size() << ",\n\"frames_processed\":" << count
                << ",\n\"imu_available\":" << imu.size() << ",\n\"diagnostic_capture_range\":{\"enabled\":" << (diagnostic_capture?"true":"false") << ",\"start_inclusive\":" << capture_start << ",\"end_exclusive\":" << capture_end << "}" << ",\n\"input_oracle_enabled\":" << (dump_inputs ? "true" : "false") << ",\n\"imu_delivered\":" << imu_delivered << ",\n\"imu_policy\":\"" << imu_policy << "\""
                << ",\n\"camera_model_enum\":" << camera_model << ",\n\"pnp_enabled\":" << pnp
                << ",\n\"cv_threads\":" << cv::getNumThreads() << ",\n\"rng_seed\":" << engine.getExecutionSeed()
                << ",\n\"executionSeed\":" << engine.getExecutionSeed()
                << ",\n\"cvThreads\":" << engine.getCVThreadCount()
                << ",\n\"pose_frame\":\"T_W_C\""
                << ",\n\"imu_noise\":{\"acc_n\":" << utility::g_config.estimator.acc_n << ",\"acc_w\":" << utility::g_config.estimator.acc_w << ",\"gyr_n\":" << utility::g_config.estimator.gyr_n << ",\"gyr_w\":" << utility::g_config.estimator.gyr_w << ",\"g_norm\":" << utility::g_config.estimator.g.norm() << "}"
                << ",\n\"solver_time_limit_s\":" << utility::g_config.estimator.solver_time
                << ",\n\"solver_iteration_limit\":" << utility::g_config.estimator.num_iterations
                << ",\n\"benchmark_solver_profile\":{\"requested\":" << (benchmark_requested ? "true" : "false")
                << ",\"actual\":" << (engine.getBenchmarkSolverProfile() ? "true" : "false")
                << ",\"name\":\"" << (engine.getBenchmarkSolverProfile() ? "max10_zero_positive_tolerances" : "default")
                << "\",\"count_semantics\":\"actual Ceres Summary.iterations.size; includes iteration0; maximum iterations is not an exact work-count guarantee\"}"
                << ",\n\"effective_derived\":{\"focal_length\":" << utility::g_config.camera.focal_length
                << ",\"projection_sqrt_info_diagonal\":[" << backend::factor::ProjectionFactor::sqrt_info(0,0) << "," << backend::factor::ProjectionFactor::sqrt_info(1,1)
                << "],\"window_size\":" << utility::g_config.estimator.window_size
                << ",\"minimum_parallax_pixels\":" << utility::g_config.estimator.min_parallax
                << ",\"minimum_parallax_normalized\":" << utility::g_config.estimator.min_parallax/utility::g_config.camera.focal_length
                << ",\"initial_depth\":" << utility::g_config.estimator.init_depth
                << ",\"number_of_features_capacity\":" << utility::g_config.estimator.num_of_features
                << ",\"equalize\":" << utility::g_config.feature_tracker.equalize
                << ",\"fisheye\":" << utility::g_config.feature_tracker.fisheye
                << ",\"tracker_window_size\":" << utility::g_config.feature_tracker.window_size
                << ",\"show_track\":" << utility::g_config.feature_tracker.show_track
                << ",\"marginalization_assembly_threads\":" << backend::factor::kMarginalizationAssemblyThreads
                << ",\"pnp_adaptive_enabled\":" << (utility::g_config.pnp.enable_adaptive?"true":"false")
                << "},\n\"feature_limit\":" << utility::g_config.feature_tracker.max_cnt
                << ",\n\"lk_window_size\":" << utility::g_config.feature_tracker.lk_window_size
                << ",\n\"lk_pyramid_levels\":" << utility::g_config.feature_tracker.lk_pyramid_levels
                << ",\n\"min_dist\":" << utility::g_config.feature_tracker.min_dist
                << ",\n\"f_threshold\":" << utility::g_config.feature_tracker.f_threshold
                << ",\n\"f_edge_factor\":" << utility::g_config.feature_tracker.f_threshold_edge_factor
                << ",\n\"lk_iterations\":" << utility::g_config.feature_tracker.lk_iterations
                << ",\n\"lk_epsilon\":" << utility::g_config.feature_tracker.lk_eps
                << ",\n\"pose_count\":" << pose_count << ",\n\"invalid_pose_count\":" << invalid_count
                << ",\n\"pose_coverage\":" << static_cast<double>(pose_count) / count
                << ",\n\"first_pose_frame\":" << first_pose_frame
                << ",\n\"first_pose_elapsed_dataset_s\":" << (first_pose_frame < 0 ? -1 : images[first_pose_frame].timestamp - images.front().timestamp)
                << ",\n\"initialization_loss_transitions\":" << init_loss
                << ",\n\"max_position_norm_m\":" << max_position
                << ",\n\"max_rotation_orthogonality_error\":" << max_rotation_error
                << ",\n\"dataset_span_s\":" << images[count - 1].timestamp - images.front().timestamp
                << ",\n\"wall_ms\":" << replay_ms << ",\n\"status_counts\":[";
        for (size_t i = 0; i < status_counts.size(); ++i) summary << (i ? "," : "") << status_counts[i];
        summary << "],\n\"camera_intrinsics_fx_fy_cx_cy\":["
                << desired.camera.fx << ',' << desired.camera.fy << ',' << desired.camera.cx << ',' << desired.camera.cy
                << "],\n\"image_width_height\":[" << desired.camera.col << ',' << desired.camera.row
                << "],\n\"camera_distortion_k2k3k4k5\":[" << k[0] << "," << k[1] << "," << k[2] << "," << k[3] << "],\n\"camera_to_imu_rotation_rowmajor\":[";
        for (size_t i = 0; i < rotation.size(); ++i) summary << (i ? "," : "") << rotation[i];
        summary << "],\n\"camera_to_imu_translation\":[";
        for (size_t i = 0; i < translation.size(); ++i) summary << (i ? "," : "") << translation[i];
        summary << "],\n\"engine_ms\":"; timingJson(summary, process_times);
        summary << ",\n\"tracking_engine_ms\":"; timingJson(summary, tracking_times);
        summary << ",\n\"image_io_ms\":"; timingJson(summary, image_times);
        summary << "\n}\n";
        summary.flush();
        if (!summary) throw std::runtime_error("Failed to write replay summary");
        std::cerr << "[Replay] complete: " << pose_count << '/' << count << " valid poses; "
                  << invalid_count << " invalid; " << replay_ms << " ms wall\n";
        return invalid_count ? 3 : pose_count == 0 ? 4 : 0;
    } catch (const std::exception& error) {
        std::cerr << "[Replay] " << error.what() << '\n';
        return 2;
    }
}
