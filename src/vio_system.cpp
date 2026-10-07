#include "vio_system.h"
#include "utility/logging.h"
#include <algorithm>
#include <chrono>

VIOSystem::VIOSystem(std::shared_ptr<utility::Config> config, bool headless)
    : config_(std::move(config)), measurement_processor_(std::make_unique<utility::MeasurementProcessor>()),
      headless_(headless), result_logger_(std::make_unique<utility::TestResultLogger>()) {}
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
    return result_logger_->initialize(config_->config_filepath);
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
    if(start>end) { LOG_ERROR("Empty or invalid frame range"); return; }
    const int skip=std::max(0,config_->frame_skip)+1;
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
    double pose[16];
    if(!engine_.processFrame(measurement.gray_image.data,measurement.gray_image.cols,measurement.gray_image.rows,
                            imu.data(),static_cast<int>(imu.size()),measurement.timestamp,pose)) return;
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
