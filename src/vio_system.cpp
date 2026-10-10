#include "vio_system.h"
#include "utility/logging.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <stdexcept>

VIOSystem::VIOSystem(std::shared_ptr<utility::Config> config, bool headless,
                     int execution_seed, int cv_threads, bool dump_inputs)
    : config_(std::move(config)), measurement_processor_(std::make_unique<utility::MeasurementProcessor>()),
      headless_(headless), execution_seed_(execution_seed), cv_threads_(cv_threads), dump_inputs_(dump_inputs),
      result_logger_(std::make_unique<utility::TestResultLogger>()) {}
VIOSystem::~VIOSystem() { shutdown(); }

bool VIOSystem::initialize() {
    if (!config_) { LOG_ERROR("Configuration not provided"); return false; }
#ifndef MOBILE_SLAM_WITH_VIEWER
    if (!headless_) { LOG_ERROR("Viewer unavailable in this build; use --headless"); return false; }
#endif
    const auto root = config_->dataset_path + "/mav0/";
    if (!measurement_processor_->initialize(root+"imu0/data.csv",root+"cam0/data.csv",root+"cam0/data",config_->config_filepath))
        return false;
#ifndef __EMSCRIPTEN__
    if (execution_seed_ < -1 || cv_threads_ < 1 || cv_threads_ > 4) {
        LOG_ERROR("Invalid execution seed/thread profile"); return false;
    }
    if (execution_seed_ >= 0) engine_.setExecutionParams(execution_seed_, cv_threads_);
    if (!engine_.configureFromConfig(*config_,config_->config_filepath)) {
        LOG_ERROR("Engine configuration failed: " << engine_.getLastReason()); return false;
    }
#ifdef MOBILE_SLAM_WITH_VIEWER
    if (!headless_) {
        visualizer_ = std::make_unique<utility::Visualizer>();
        imu_graph_visualizer_ = std::make_unique<utility::IMUGraphVisualizer>();
        if (!visualizer_->initialize() || !imu_graph_visualizer_->initialize(1024,768,300)) return false;
        imu_graph_visualizer_->start();
    }
#endif
#else
    return false; // Browser callers configure and call the engine directly, without dataset file I/O.
#endif
    if (!result_logger_->initialize(config_->config_filepath)) return false;
    const auto directory=result_logger_->getLogDirectory();
    std::ofstream effective(directory+"/effective-config.json");
    // Reuse the engine's complete readback, including actual camera model and execution profile.
    engine_.setDiagnosticCapture(true);
    effective<<engine_.getFeatureDiagnostics()<<'\n';
    engine_.setDiagnosticCapture(false);
    frame_log_.open(directory+"/frames.csv");
    result_log_.open(directory+"/results.json");
    if (dump_inputs_) input_log_.open(directory+"/input-oracle.bin",std::ios::binary);
    if (!effective || !frame_log_ || !result_log_ || (dump_inputs_ && !input_log_)) return false;
    frame_log_<<"frame,timestamp_s,decoded,process_ms,imu_count,imu_first_s,imu_last_s,imu_endpoint_s,features,status,initialized,has_pose,valid_pose,epoch,pose_timestamp_s,reason,solver_iterations,solver_termination\n";
    frame_log_<<std::setprecision(17);
    result_log_<<std::setprecision(17)<<"{\"frames\":[";
    return true;
}

void VIOSystem::processSequence() {
#ifdef MOBILE_SLAM_WITH_VIEWER
    if (!headless_) {
        vio_process_thread_ = std::make_unique<std::thread>([this]{ vioProcess(); });
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        visualizer_->pangolinViewerThread();
        if (vio_process_thread_->joinable()) vio_process_thread_->join();
        if (imu_graph_visualizer_) imu_graph_visualizer_->stop();
        return;
    }
#endif
    vioProcess();
}
void VIOSystem::shutdown() {
#ifdef MOBILE_SLAM_WITH_VIEWER
    if (visualizer_) visualizer_->stop();
    if (imu_graph_visualizer_) imu_graph_visualizer_->stop();
#endif
    if (vio_process_thread_ && vio_process_thread_->joinable()) vio_process_thread_->join();
}
void VIOSystem::vioProcess() {
    const auto& images=measurement_processor_->getImageFileData();
    const int total=static_cast<int>(images.size());
    const int start=std::max(0,config_->start_frame);
    const int end=config_->end_frame<0 ? total-1 : std::min(total-1,config_->end_frame);
    if(start>end) throw std::runtime_error("Empty or invalid frame range");
    const int skip=std::max(0,config_->frame_skip)+1;
    scheduled_frames_=static_cast<size_t>((end-start)/skip+1);
    if(start>0) measurement_processor_->beginAtImageTimestamp(images[start].timestamp);
    for(int i=start;i<=end;i+=skip)
        onFrameProcessed(measurement_processor_->createRawMeasurementMsg(i,images[i]));
    onSequenceComplete();
}
void VIOSystem::onFrameProcessed(const utility::RawMeasurementMsg& measurement) {
    if(measurement.gray_image.empty()) LOG_WARN("Cannot decode dataset frame " << measurement.measurement_id);
    std::vector<IMUReading> imu;
    imu.reserve(measurement.imu_msg.size());
    for(const auto& value:measurement.imu_msg) {
        imu.push_back({value.timestamp,value.linear_acc_x,value.linear_acc_y,value.linear_acc_z,
                       value.angular_vel_x,value.angular_vel_y,value.angular_vel_z});
#ifdef MOBILE_SLAM_WITH_VIEWER
        if(imu_graph_visualizer_ && imu_graph_visualizer_->isRunning())
            imu_graph_visualizer_->addIMUData(value.timestamp,
                Eigen::Vector3d(value.linear_acc_x,value.linear_acc_y,value.linear_acc_z),
                Eigen::Vector3d(value.angular_vel_x,value.angular_vel_y,value.angular_vel_z));
#endif
    }
    if (dump_inputs_) {
        const auto write_u32=[&](uint32_t value) {
            for (int byte=0;byte<4;++byte) input_log_.put(static_cast<char>((value>>(8*byte))&255));
        };
        write_u32(static_cast<uint32_t>(measurement.gray_image.total()));
        if (!measurement.gray_image.empty()) input_log_.write(reinterpret_cast<const char*>(measurement.gray_image.data),
            static_cast<std::streamsize>(measurement.gray_image.total()));
        write_u32(static_cast<uint32_t>(imu.size()));
        for (const auto& sample:imu) {
            const double row[7]={sample.timestamp,sample.acc_x,sample.acc_y,sample.acc_z,sample.gyro_x,sample.gyro_y,sample.gyro_z};
            input_log_.write(reinterpret_cast<const char*>(row),sizeof(row));
        }
        if (!input_log_) throw std::runtime_error("Failed to write PC input oracle");
    }
    double pose[16];
    const auto begin=std::chrono::steady_clock::now();
    const bool has_pose=engine_.processFrame(measurement.gray_image.data,measurement.gray_image.cols,measurement.gray_image.rows,
                            imu.data(),static_cast<int>(imu.size()),measurement.timestamp,pose);
    const double process_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    const bool valid=has_pose && engine_.getPoseFresh() && engine_.getPoseValid() &&
                     std::abs(engine_.getPoseTimestamp()-measurement.timestamp)<1e-6;
    const bool initialized=engine_.isInitialized();
    if (previously_initialized_ && !initialized) ++initialization_losses_;
    previously_initialized_=initialized;
    ++processed_frames_;
    decoded_frames_+=!measurement.gray_image.empty();
    imu_delivered_+=imu.size();
    if (valid) {
        ++fresh_poses_;
        if (first_pose_frame_<0) first_pose_frame_=measurement.measurement_id;
    }
    frame_log_<<measurement.measurement_id<<','<<measurement.timestamp<<','<<!measurement.gray_image.empty()<<','
        <<process_ms<<','<<imu.size()<<','<<(imu.empty()?-1:imu.front().timestamp)<<','<<(imu.empty()?-1:imu.back().timestamp)<<','
        <<engine_.getIMUEndpointTimestamp()<<','<<engine_.getFeaturePointCount()<<','<<engine_.getStatusCode()<<','
        <<initialized<<','<<has_pose<<','<<valid<<','<<engine_.getEpoch()<<','<<engine_.getPoseTimestamp()<<','
        <<engine_.getLastReason()<<','<<engine_.getLastSolverIterations()<<','<<engine_.getLastSolverTermination()<<'\n';
    result_log_<<(processed_frames_>1?",":"")<<"{\"frame\":"<<measurement.measurement_id
        <<",\"inputTimestamp\":"<<measurement.timestamp<<",\"poseTimestamp\":"<<engine_.getPoseTimestamp()
        <<",\"engineEpoch\":"<<engine_.getEpoch()<<",\"statusCode\":"<<engine_.getStatusCode()
        <<",\"poseValid\":"<<(valid?"true":"false")<<",\"poseFresh\":"<<(engine_.getPoseFresh()?"true":"false")
        <<",\"reason\":\""<<engine_.getLastReason()<<"\",\"solverIterations\":"<<engine_.getLastSolverIterations()
        <<",\"solverTermination\":\""<<engine_.getLastSolverTermination()<<"\",\"imuCount\":"<<imu.size()<<",\"pose\":";
    if (valid) { result_log_<<'['; for (int i=0;i<16;++i) result_log_<<(i?",":"")<<pose[i]; result_log_<<']'; }
    else result_log_<<"null";
    result_log_<<'}';
    if (!valid) return;
    const Eigen::Matrix4d camera=Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::RowMajor>>(pose);
    const Eigen::Vector3d position=camera.block<3,1>(0,3);
    const Eigen::Matrix3d rotation=camera.block<3,3>(0,0);
    const double timestamp=engine_.getPoseTimestamp();
    result_logger_->addPose(position,rotation,timestamp);
#ifdef MOBILE_SLAM_WITH_VIEWER
    if(visualizer_ && visualizer_->isRunning()) {
        visualizer_->updateCameraPose(position,rotation,timestamp);
        const Eigen::Matrix3d body_rotation=rotation*config_->camera.r_ic.transpose();
        const Eigen::Vector3d body_position=position-body_rotation*config_->camera.t_ic;
        visualizer_->updateIMUPose(body_position,body_rotation,timestamp);
        double map[3*utility::NUM_OF_FEATURES];
        const int count=engine_.getMapPoints(map,utility::NUM_OF_FEATURES);
        std::vector<Eigen::Vector3d> points;
        for(int i=0;i<count;++i) points.emplace_back(map[3*i],map[3*i+1],map[3*i+2]);
        visualizer_->updateFeaturePoints3D(points);
    }
#endif
    if(result_logger_->getPoseCount()%50==0) result_logger_->saveTrajectoryToFile();
}
void VIOSystem::onSequenceComplete() {
    result_log_<<"]}\n";
    result_log_.flush(); frame_log_.flush();
    if (!result_log_ || !frame_log_) throw std::runtime_error("Failed to write PC frame records");
    std::ofstream summary(result_logger_->getLogDirectory()+"/summary.json");
    summary<<std::setprecision(17)<<"{\"frames_available\":"<<measurement_processor_->getImageFileData().size()
        <<",\"frames_scheduled\":"<<scheduled_frames_<<",\"frames_processed\":"<<processed_frames_
        <<",\"frames_decoded\":"<<decoded_frames_<<",\"missing_images\":"<<processed_frames_-decoded_frames_
        <<",\"imu_available\":"<<measurement_processor_->getIMUData().size()<<",\"imu_delivered\":"<<imu_delivered_
        <<",\"imu_policy\":\"bracket\",\"pose_frame\":\"T_W_C\",\"pose_count\":"<<fresh_poses_
        <<",\"first_pose_frame\":"<<first_pose_frame_<<",\"initialization_loss_transitions\":"<<initialization_losses_
        <<",\"pose_coverage\":"<<(scheduled_frames_?static_cast<double>(fresh_poses_)/scheduled_frames_:0)
        <<",\"cv_threads\":"<<engine_.getCVThreadCount()<<",\"rng_seed\":"<<engine_.getExecutionSeed()
        <<",\"input_oracle_enabled\":"<<(dump_inputs_?"true":"false")<<"}\n";
    summary.flush();
    if (!summary) throw std::runtime_error("Failed to write PC summary");
    result_logger_->saveTrajectoryToFile();
    utility::TrajectoryEvaluator evaluator;
    if(evaluator.loadVioTrajectory(result_logger_->getLogDirectory()+"/trajectory_pose.txt") &&
       evaluator.loadGroundTruth(config_->dataset_path+"/mav0/mocap0/data.csv")) {
        evaluator.transformVioToBodyFrame(config_->camera.r_ic,config_->camera.t_ic);
        evaluator.associateTrajectories(.01);
        evaluator.alignTrajectories();
        const auto ate=evaluator.computeATE(); const auto rpe=evaluator.computeRPE(1.0);
        evaluator.printResults(ate,rpe);
        evaluator.saveResults(result_logger_->getLogDirectory()+"/evaluation.txt",ate,rpe);
    }
}
