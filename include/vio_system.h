#ifndef VIO_SYSTEM_H
#define VIO_SYSTEM_H
#include <memory>
#include <thread>
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
    explicit VIOSystem(std::shared_ptr<utility::Config> config, bool headless = false);
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
