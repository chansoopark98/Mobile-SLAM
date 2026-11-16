#ifndef VIO_SYSTEM_H
#define VIO_SYSTEM_H
#include <memory>
#include <thread>
#include <fstream>
#include "vio_engine.h"
#include "utility/measurement_processor.h"
#include "utility/test_result_logger.h"
#include "utility/trajectory_evaluator.h"
#ifdef MOBILE_SLAM_WITH_VIEWER
#include "utility/visualizer.h"
#include "utility/imu_graph_visualizer.h"
#endif

// Dataset and viewer adapter. Estimation, feature tracking and IMU boundaries live in VIOEngine.
class VIOSystem {
public:
    explicit VIOSystem(std::shared_ptr<utility::Config> config, bool headless = false,
                       int execution_seed = -1, int cv_threads = 1, bool dump_inputs = false);
    ~VIOSystem();
    bool initialize();
    void processSequence();
    void shutdown();
    const VIOEngine& getEngine() const { return engine_; }
private:
    std::shared_ptr<utility::Config> config_;
    std::unique_ptr<utility::MeasurementProcessor> measurement_processor_;
    VIOEngine engine_;
    bool headless_;
    int execution_seed_;
    int cv_threads_;
    bool dump_inputs_;
    std::ofstream frame_log_, result_log_, input_log_;
    size_t scheduled_frames_ = 0, processed_frames_ = 0, decoded_frames_ = 0, imu_delivered_ = 0;
    size_t fresh_poses_ = 0, initialization_losses_ = 0;
    int first_pose_frame_ = -1;
    bool previously_initialized_ = false;
#ifdef MOBILE_SLAM_WITH_VIEWER
    std::unique_ptr<utility::Visualizer> visualizer_;
    std::unique_ptr<utility::IMUGraphVisualizer> imu_graph_visualizer_;
#endif
    std::unique_ptr<utility::TestResultLogger> result_logger_;
    std::unique_ptr<std::thread> vio_process_thread_;
    void vioProcess();
    void onFrameProcessed(const utility::RawMeasurementMsg& measurement);
    void onSequenceComplete();
};
#endif
