#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <limits>
#include <deque>
#include <atomic>
#include <fstream>
#include <filesystem>
#include <opencv2/imgproc.hpp>
#include "vio_engine.h"

struct VIOEngineTestAccess {
    static bool integrate(VIOEngine& engine,const std::vector<IMUReading>& readings,double time) {
        return engine.processIMUData(readings.data(),static_cast<int>(readings.size()),time);
    }
    static backend::Estimator& estimator(VIOEngine& engine) { return *engine.estimator_; }
    static double cursor(const VIOEngine& engine) { return engine.getIMUEndpointTimestamp(); }
    static Eigen::Vector3d acceleration(const VIOEngine& engine) { return engine.prev_acc_; }
    static Eigen::Vector3d angularVelocity(const VIOEngine& engine) { return engine.prev_gyro_; }
    static size_t pending(const VIOEngine& engine) { return engine.pending_imu_.size(); }
    static bool pose(VIOEngine& engine,const Eigen::Vector3d& position,const Eigen::Matrix3d& rotation,double time,double* output) {
        return engine.publishPose(position,rotation,time,output);
    }
    static void solver(VIOEngine& engine,const backend::SolverDiagnostics& diagnostics) { engine.recordSolver(diagnostics); }
};

namespace {
class BoundaryRayCamera : public common::camera_models::PinholeCamera {
public:
    BoundaryRayCamera() : PinholeCamera(Parameters("boundary",96,96,0,0,0,0,100,100,48,48)) {}
    bool reject_right_half = false;
    void liftProjective(const Eigen::Vector2d& pixel,Eigen::Vector3d& ray) const override {
        if(pixel.x()==1) ray=Eigen::Vector3d(1,1,0);
        else if(pixel.x()==2) ray=Eigen::Vector3d(1,1,-1);
        else if(pixel.x()==3 || (reject_right_half && pixel.x()>=48))
            ray=Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),1,1);
        else ray=Eigen::Vector3d((pixel.x()-48)/100,(pixel.y()-48)/100,1);
    }
};
bool configure(VIOEngine& engine, const double* rotation = nullptr,
               const double* translation = nullptr, int model = 2, double fx = 100.0) {
    const double identity[9] = {1,0,0,0,1,0,0,0,1};
    const double zero[3] = {0,0,0};
    return engine.configure(96,96,fx,100,48,48,model,0,0,0,0,
                            rotation ? rotation : identity, translation ? translation : zero,
                            0.1,0.001,0.01,0.0001,9.81);
}
}

TEST(EngineContracts, RejectsInvalidCalibrationInsteadOfSilentFallback) {
    VIOEngine engine;
    EXPECT_FALSE(configure(engine,nullptr,nullptr,1));
    EXPECT_FALSE(configure(engine,nullptr,nullptr,2,0));
    EXPECT_FALSE(configure(engine,nullptr,nullptr,2,std::numeric_limits<double>::quiet_NaN()));
    const double reflection[9] = {-1,0,0,0,1,0,0,0,1};
    EXPECT_FALSE(configure(engine,reflection));
    const double skew[9] = {1,.1,0,0,1,0,0,0,1};
    EXPECT_FALSE(configure(engine,skew));
}

TEST(EngineContracts, ConfigureCopiesAliasedRowMajorInputBeforeWritingGlobalConfig) {
    utility::g_config.camera.r_ic = (Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(.9,Eigen::Vector3d::UnitZ())).toRotationMatrix();
    const Eigen::Matrix3d old = utility::g_config.camera.r_ic;
    // Deliberately interpret the existing column-major buffer under the public row-major contract.
    const Eigen::Matrix3d expected = old.transpose();
    VIOEngine engine;
    ASSERT_TRUE(configure(engine,utility::g_config.camera.r_ic.data(),utility::g_config.camera.t_ic.data()));
    EXPECT_TRUE(utility::g_config.camera.r_ic.isApprox(expected,1e-12));
}

TEST(EngineContracts, ResetDropsTrackedImageAndIds) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    cv::Mat checker(96,96,CV_8UC1,cv::Scalar(0));
    for(int y=8;y<90;y+=16) for(int x=8;x<90;x+=16)
        cv::rectangle(checker,cv::Rect(x,y,7,7),cv::Scalar(255),-1);
    double pose[16];
    engine.processFrame(checker.data,96,96,nullptr,0,1.0,pose);
    engine.processFrame(checker.data,96,96,nullptr,0,1.05,pose);
    ASSERT_GT(engine.getFeaturePointCount(),0);
    engine.reset();
    EXPECT_EQ(engine.getFeaturePointCount(),0);
}

TEST(EngineContracts, TrackerOwnsRetainedImageWhenEqualizationIsDisabled) {
    utility::g_config.camera.col = utility::g_config.camera.row = 96;
    utility::g_config.feature_tracker.equalize = 0;
    utility::g_config.feature_tracker.fisheye = 0;
    frontend::FeatureTracker tracker;
    ASSERT_TRUE(tracker.setIntrinsicParameter(2,96,96,100,100,48,48,0,0,0,0));
    cv::Mat reusable(96,96,CV_8UC1,cv::Scalar(17));
    tracker.detectAndTrack(reusable,1);
    reusable.setTo(255);
    EXPECT_EQ(tracker.cur_img.at<uint8_t>(0,0),17);
}

TEST(EngineContracts, FirstUniformTargetDropsLKTracksAndRecoversWithNewIds) {
    struct RestoreConfig {
        utility::Config saved = utility::g_config;
        ~RestoreConfig() { utility::g_config = saved; }
    } restore;
    utility::g_config.camera.col = utility::g_config.camera.row = 96;
    utility::g_config.feature_tracker.fisheye = 0;
    utility::g_config.feature_tracker.max_cnt = 150;
    utility::g_config.feature_tracker.min_dist = 10;
    for(int equalize : {0,1}) for(int uniform : {0,127,255}) for(bool backend_frame : {false,true}) {
        if(equalize && uniform==255) continue; // This seed loses LK tracks under CLAHE before the target transition.
        SCOPED_TRACE(::testing::Message()<<"equalize="<<equalize<<" uniform="<<uniform<<" backend="<<backend_frame);
        utility::g_config.feature_tracker.equalize = equalize;
        frontend::FeatureTracker tracker;
        ASSERT_TRUE(tracker.setIntrinsicParameter(2,96,96,100,100,48,48,0,0,0,0));
        const auto camera = tracker.m_camera;
        cv::Mat textured(96,96,CV_8UC1,cv::Scalar(uniform));
        const int contrast = uniform<239 ? uniform+16 : uniform-16;
        for(int y=8;y<90;y+=16) for(int x=8;x<90;x+=16)
            cv::rectangle(textured,cv::Rect(x,y,7,7),cv::Scalar(contrast),-1);
        tracker.detectAndTrack(textured,1);
        for(unsigned int i=0;tracker.updateID(i);++i) {}
        ASSERT_GT(tracker.cur_pts.size(),0u);
        tracker.detectAndTrack(textured,1.05,false); // Seed the real LK reference, including normals/history.
        ASSERT_GT(tracker.cur_pts.size(),0u);
        const int next_id = frontend::FeatureTracker::n_id;

        // An exact constant image has zero gradient everywhere, independently of LK's previous-image status.
        cv::Mat uniform_image(96,96,CV_8UC1,cv::Scalar(uniform));
        ASSERT_NO_THROW(tracker.detectAndTrack(uniform_image,1.1,backend_frame));
        EXPECT_TRUE(tracker.n_pts.empty());
        EXPECT_TRUE(tracker.prev_pts.empty()); EXPECT_TRUE(tracker.cur_pts.empty()); EXPECT_TRUE(tracker.next_pts.empty());
        EXPECT_TRUE(tracker.prev_undistorted_pts.empty()); EXPECT_TRUE(tracker.cur_undistorted_pts.empty());
        EXPECT_TRUE(tracker.pts_velocity.empty()); EXPECT_TRUE(tracker.ids.empty()); EXPECT_TRUE(tracker.track_cnt.empty());
        EXPECT_TRUE(tracker.cur_undistorted_pts_map.empty()); EXPECT_TRUE(tracker.prev_undistorted_pts_map.empty());
        EXPECT_EQ(tracker.m_camera,camera);
        EXPECT_DOUBLE_EQ(tracker.cur_time,1.1); EXPECT_DOUBLE_EQ(tracker.prev_time,1.1);
        EXPECT_EQ(frontend::FeatureTracker::n_id,next_id);

        ASSERT_NO_THROW(tracker.detectAndTrack(textured,1.15));
        ASSERT_GT(tracker.cur_pts.size(),0u);
        for(unsigned int i=0;tracker.updateID(i);++i) {}
        for(int id : tracker.ids) EXPECT_GE(id,next_id);
        for(int count : tracker.track_cnt) EXPECT_EQ(count,1);
        EXPECT_EQ(tracker.cur_pts.size(),tracker.ids.size());
        EXPECT_EQ(tracker.cur_pts.size(),tracker.track_cnt.size());
        EXPECT_EQ(tracker.cur_pts.size(),tracker.cur_undistorted_pts.size());
        EXPECT_EQ(tracker.cur_pts.size(),tracker.pts_velocity.size());
        EXPECT_TRUE(tracker.prev_pts.empty()); EXPECT_TRUE(tracker.prev_undistorted_pts.empty());
        ASSERT_NO_THROW(tracker.detectAndTrack(textured,1.2,false));
        EXPECT_GT(tracker.cur_pts.size(),0u);
        EXPECT_EQ(tracker.cur_pts.size(),tracker.ids.size());
        EXPECT_EQ(tracker.cur_pts.size(),tracker.track_cnt.size());
        EXPECT_EQ(tracker.cur_pts.size(),tracker.cur_undistorted_pts.size());
        EXPECT_EQ(tracker.cur_pts.size(),tracker.pts_velocity.size());
        EXPECT_LE(tracker.prev_pts.size(),tracker.cur_pts.size());
        EXPECT_LE(tracker.prev_undistorted_pts.size(),tracker.cur_pts.size());
    }
}

TEST(EngineContracts, UniformTargetReachesEmptyFeatureBoundaryAfterRealLKSeed) {
    struct RestoreConfig {
        utility::Config saved = utility::g_config;
        ~RestoreConfig() { utility::g_config = saved; }
    } restore;
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    utility::g_config.feature_tracker.equalize = 0;
    utility::g_config.feature_tracker.min_dist = 10;
    cv::Mat textured(96,96,CV_8UC1,cv::Scalar(127));
    for(int y=8;y<90;y+=16) for(int x=8;x<90;x+=16)
        cv::rectangle(textured,cv::Rect(x,y,7,7),cv::Scalar(143),-1);
    IMUReading imu[]={{1,0,0,9.81,0,0,0},{1.25,0,0,9.81,0,0,0}};
    double output[16];
    EXPECT_FALSE(engine.processFrame(textured.data,96,96,imu,2,1,output));
    EXPECT_FALSE(engine.processFrame(textured.data,96,96,nullptr,0,1.05,output));
    ASSERT_GT(engine.getFeaturePointCount(),0);
    const auto epoch = engine.getEpoch();
    cv::Mat uniform(96,96,CV_8UC1,cv::Scalar(127));
    EXPECT_FALSE(engine.processFrame(uniform.data,96,96,nullptr,0,1.1,output));
    EXPECT_EQ(engine.getFeaturePointCount(),0);
    EXPECT_EQ(engine.getLastReason(),"empty_features");
    EXPECT_FALSE(engine.getPoseValid()); EXPECT_FALSE(engine.getPoseFresh());
    EXPECT_DOUBLE_EQ(engine.getPoseTimestamp(),-1);
    EXPECT_DOUBLE_EQ(engine.getFrameTimestamp(),1.1);
    double map[3]; EXPECT_EQ(engine.getMapPoints(map,1),0);
    for(double value : output) EXPECT_TRUE(std::isnan(value));
    EXPECT_EQ(engine.getEpoch(),epoch);
    EXPECT_FALSE(engine.processFrame(textured.data,96,96,nullptr,0,1.15,output));
    EXPECT_EQ(engine.getFeaturePointCount(),0); // Newly detected points require a following observation.
    EXPECT_FALSE(engine.processFrame(textured.data,96,96,nullptr,0,1.2,output));
    EXPECT_GT(engine.getFeaturePointCount(),0);
    EXPECT_EQ(engine.getLastReason(),"initializing");
    EXPECT_EQ(engine.getEpoch(),epoch);
}

TEST(EngineContracts, AnalyticIMUIntegralAcrossRatesPhasesAndCarriedFutureSamples) {
    for (int imu_rate : {60,100,200}) for (int camera_rate : {10,20,30,60})
    for (double phase : {0.0,0.37}) for (bool rotation_only : {false,true}) for (bool ramp : {false,true}) {
        SCOPED_TRACE(::testing::Message()<<imu_rate<<"/"<<camera_rate<<" phase="<<phase<<" rotation="<<rotation_only<<" ramp="<<ramp);
        VIOEngine engine;
        ASSERT_TRUE(configure(engine));
        std::vector<IMUReading> source;
        source.push_back({0,rotation_only?0.0:(ramp?1.0:2.0),0,0,0,0,rotation_only?(ramp?.3:.4):0.0});
        for(int i=1;i<=imu_rate+20;++i) {
            const double time=(i+phase)/imu_rate;
            source.push_back({time,rotation_only?0.0:(ramp?1+2*time:2.0),0,0,0,0,rotation_only?(ramp?.3+.2*time:.4):0.0});
        }
        size_t sent=0;
        const auto send=[&](double time) {
            std::vector<IMUReading> batch;
            while(sent<source.size() && source[sent].timestamp<=time+.1) batch.push_back(source[sent++]);
            return VIOEngineTestAccess::integrate(engine,batch,time);
        };
        ASSERT_TRUE(send(0));
        ASSERT_GT(VIOEngineTestAccess::pending(engine),1u);
        common::ImageData observations;
        for(int i=0;i<12;++i) { Eigen::Matrix<double,7,1> value; value<<i*.01,0,1,48+i,48,0,0; observations[i]=value; }
        VIOEngineTestAccess::estimator(engine).processImage(observations,0);
        for(int i=1;i<camera_rate;++i) {
            const double time=(i+.19)/camera_rate;
            ASSERT_TRUE(send(time));
            EXPECT_NEAR(VIOEngineTestAccess::cursor(engine),time,1e-12);
            if(rotation_only) EXPECT_NEAR(VIOEngineTestAccess::angularVelocity(engine).z(),ramp?.3+.2*time:.4,1e-12);
            else EXPECT_NEAR(VIOEngineTestAccess::acceleration(engine).x(),ramp?1+2*time:2.0,1e-12);
        }
        ASSERT_TRUE(send(1));
        const auto& integration=VIOEngineTestAccess::estimator(engine).sliding_window_[1].pre_integration;
        ASSERT_TRUE(integration);
        EXPECT_NEAR(integration->sum_dt,1,1e-9);
        if(rotation_only) {
            // Fixed-axis closed form integral of .3+.2t rad/s is .4 rad.
            EXPECT_NEAR(Eigen::AngleAxisd(integration->delta_q).angle(),.4,1e-5);
        } else {
            EXPECT_NEAR(integration->delta_v.x(),2,1e-10);
            // Midpoint position truncation is bounded by max step cubed, not a copied implementation.
            EXPECT_NEAR(integration->delta_p.x(),ramp?5.0/6.0:1.0,1e-4);
        }
    }
}

TEST(EngineContracts, ConstantIMUSeedAndNoNewSamplesHaveExactCoverage) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    ASSERT_TRUE(VIOEngineTestAccess::integrate(engine,{{0,2,0,0,0,0,0},{.2,2,0,0,0,0,0}},0));
    common::ImageData observations;
    Eigen::Matrix<double,7,1> ray; ray<<0,0,1,48,48,0,0; observations[1]=ray;
    VIOEngineTestAccess::estimator(engine).processImage(observations,0);
    for(double time : {.01,.03,.05,.1,.2}) ASSERT_TRUE(VIOEngineTestAccess::integrate(engine,{},time));
    const auto& integration=VIOEngineTestAccess::estimator(engine).sliding_window_[1].pre_integration;
    ASSERT_TRUE(integration);
    EXPECT_NEAR(integration->sum_dt,.2,1e-12);
    EXPECT_NEAR(integration->delta_v.x(),.4,1e-12);
    EXPECT_NEAR(integration->delta_p.x(),.04,1e-12);
    EXPECT_EQ(VIOEngineTestAccess::pending(engine),0u);
}

TEST(EngineContracts, CameraContinuationCannotExtrapolatePastLastRealIMUDuringSensorPause) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    cv::Mat image(96,96,CV_8UC1,cv::Scalar(0));
    for(int y=8;y<90;y+=16) for(int x=8;x<90;x+=16)
        cv::rectangle(image,cv::Rect(x,y,7,7),cv::Scalar(255),-1);
    double output[16];
    IMUReading samples[]={{0,0,0,9.81,0,0,0},{.1,0,0,9.81,0,0,0}};
    engine.processFrame(image.data,96,96,samples,2,0,output);
    engine.processFrame(image.data,96,96,nullptr,0,.1,output);
    VIOEngineTestAccess::estimator(engine).solver_flag_=common::SolverFlag::NON_LINEAR;
    ASSERT_TRUE(VIOEngineTestAccess::pose(engine,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),.1,output));
    const auto epoch=engine.getEpoch();
    for(int i=3;i<=20;++i) {
        const double timestamp=i*.05;
        EXPECT_FALSE(engine.processFrame(image.data,96,96,nullptr,0,timestamp,output));
        EXPECT_EQ(engine.getLastReason(),"imu_missing_future");
        EXPECT_EQ(engine.getStatusCode(),static_cast<int>(VIOStatus::LOST));
        EXPECT_FALSE(engine.getPoseFresh());
        EXPECT_FALSE(engine.getPoseValid());
        EXPECT_EQ(engine.getPoseTimestamp(),-1);
        EXPECT_DOUBLE_EQ(engine.getFrameTimestamp(),timestamp);
        EXPECT_DOUBLE_EQ(VIOEngineTestAccess::cursor(engine),.1);
        EXPECT_EQ(engine.getEpoch(),epoch);
        for(double value:output) EXPECT_TRUE(std::isnan(value));
    }
    IMUReading resumed={.7,0,0,9.81,0,0,0};
    EXPECT_FALSE(engine.processFrame(image.data,96,96,&resumed,1,1.05,output));
    EXPECT_EQ(engine.getLastReason(),"imu_gap");
    EXPECT_EQ(engine.getEpoch(),epoch+1);
    EXPECT_EQ(VIOEngineTestAccess::cursor(engine),-1);
}

TEST(EngineContracts, DuplicateOutOfOrderAndNonfiniteIMUDoNotMoveCursorOrDropCarry) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    ASSERT_TRUE(VIOEngineTestAccess::integrate(engine,{{0,1,0,0,0,0,0},{.1,2,0,0,0,0,0}},.03));
    const auto pending=VIOEngineTestAccess::pending(engine);
    for(double time : {.1,.05}) {
        EXPECT_FALSE(VIOEngineTestAccess::integrate(engine,{{time,999,0,0,0,0,0}},.04));
        EXPECT_EQ(engine.getLastReason(),"imu_out_of_order");
        EXPECT_NEAR(VIOEngineTestAccess::cursor(engine),.03,1e-12);
        EXPECT_EQ(VIOEngineTestAccess::pending(engine),pending);
    }
    EXPECT_FALSE(VIOEngineTestAccess::integrate(engine,{{.2,std::numeric_limits<double>::quiet_NaN(),0,0,0,0,0}},.04));
    ASSERT_TRUE(VIOEngineTestAccess::integrate(engine,{},.08));
    EXPECT_NEAR(VIOEngineTestAccess::acceleration(engine).x(),1.8,1e-12);
}

TEST(EngineContracts, GapAndOverflowInvalidateAllStateAndIncrementEpoch) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    auto epoch=engine.getEpoch();
    EXPECT_FALSE(VIOEngineTestAccess::integrate(engine,{{0,1,0,0,0,0,0},{1,1,0,0,0,0,0}},.1));
    EXPECT_EQ(engine.getLastReason(),"imu_gap");
    EXPECT_EQ(engine.getEpoch(),epoch+1);
    EXPECT_EQ(VIOEngineTestAccess::cursor(engine),-1);
    EXPECT_EQ(VIOEngineTestAccess::pending(engine),0u);
    std::vector<IMUReading> readings;
    for(int i=0;i<4096;++i) readings.push_back({i*.00001,1,0,0,0,0,0});
    ASSERT_TRUE(VIOEngineTestAccess::integrate(engine,readings,0));
    epoch=engine.getEpoch();
    EXPECT_FALSE(VIOEngineTestAccess::integrate(engine,{{.05,1,0,0,0,0,0},{.06,1,0,0,0,0,0}},.001));
    EXPECT_EQ(engine.getLastReason(),"imu_overflow");
    EXPECT_EQ(engine.getEpoch(),epoch+1);
}

TEST(EngineContracts, MissingVisualUpdatesCannotAccumulateUnboundedIMUHistory) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    std::vector<IMUReading> readings;
    for(int i=0;i<4096;++i) readings.push_back({i*.000001,1,0,0,0,0,0});
    EXPECT_FALSE(VIOEngineTestAccess::integrate(engine,readings,.0040955));
    EXPECT_EQ(engine.getLastReason(),"imu_missing_future");
    const auto epoch=engine.getEpoch();
    EXPECT_FALSE(VIOEngineTestAccess::integrate(engine,{{.004096,1,0,0,0,0,0},{.004097,1,0,0,0,0,0}},.004097));
    EXPECT_EQ(engine.getLastReason(),"imu_history_overflow");
    EXPECT_EQ(engine.getEpoch(),epoch+1);
    EXPECT_EQ(VIOEngineTestAccess::pending(engine),0u);
}

TEST(EngineContracts, FrameBoundsMonotonicityFreshnessAndResetAreExplicit) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    std::vector<uint8_t> blank(96*96,0);
    double output[16];
    const auto epoch=engine.getEpoch();
    EXPECT_FALSE(engine.processFrame(nullptr,96,96,nullptr,0,1,output));
    EXPECT_EQ(engine.getLastReason(),"invalid_frame");
    EXPECT_FALSE(engine.processFrame(blank.data(),95,96,nullptr,0,1,output));
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,nullptr,-1,1,output));
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,nullptr,1,1,output));
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,nullptr,4097,1,output));
    EXPECT_EQ(engine.getEpoch(),epoch);
    IMUReading imu[]={{.99,0,0,9.81,0,0,0},{1.1,0,0,9.81,0,0,0}};
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,imu,2,1,output));
    EXPECT_EQ(engine.getLastReason(),"empty_features");
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,nullptr,0,1,output));
    EXPECT_EQ(engine.getLastReason(),"frame_out_of_order");
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,nullptr,0,2,output));
    EXPECT_EQ(engine.getLastReason(),"frame_gap");
    EXPECT_EQ(engine.getEpoch(),epoch+1);
    engine.reset();
    EXPECT_EQ(engine.getEpoch(),epoch+2);
    EXPECT_EQ(engine.getPoseTimestamp(),-1);
    EXPECT_EQ(engine.getFrameTimestamp(),-1);
    EXPECT_FALSE(engine.getPoseFresh());
    EXPECT_FALSE(engine.getPoseValid());
    for(double value:output) EXPECT_TRUE(std::isnan(value));
}

TEST(EngineContracts, EstimatorResetGenerationClearsCarryTrackerAndAdvancesEpochOnNextFrame) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    ASSERT_TRUE(VIOEngineTestAccess::integrate(engine,{{0,1,0,0,0,0,0},{.2,1,0,0,0,0,0}},.05));
    ASSERT_EQ(VIOEngineTestAccess::pending(engine),1u);
    const auto epoch=engine.getEpoch();
    VIOEngineTestAccess::estimator(engine).reset();
    std::vector<uint8_t> blank(96*96,0);
    double output[16];
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,nullptr,0,.1,output));
    EXPECT_EQ(engine.getLastReason(),"estimator_reset");
    EXPECT_EQ(engine.getEpoch(),epoch+1);
    EXPECT_EQ(engine.getFrameTimestamp(),.1);
    EXPECT_EQ(engine.getPoseTimestamp(),-1);
    EXPECT_EQ(VIOEngineTestAccess::cursor(engine),-1);
    EXPECT_EQ(VIOEngineTestAccess::pending(engine),0u);
    EXPECT_EQ(engine.getFeaturePointCount(),0);
    for(double value:output) EXPECT_TRUE(std::isnan(value));
}

TEST(EngineContracts, MeasuredQualityWarningDoesNotChangeNumericalPoseUsability) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    backend::SolverDiagnostics diagnostics;
    diagnostics.usable=true;
    diagnostics.iterations=7;
    diagnostics.terminationType=1;
    diagnostics.qualityReason="acc_bias_threshold";
    VIOEngineTestAccess::solver(engine,diagnostics);
    double output[16];
    ASSERT_TRUE(VIOEngineTestAccess::pose(engine,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),1,output));
    EXPECT_TRUE(engine.getPoseFresh());
    EXPECT_TRUE(engine.getPoseValid());
    EXPECT_EQ(engine.getLastReason(),"tracking:acc_bias_threshold");
    EXPECT_EQ(engine.getLastSolverIterations(),7);
    EXPECT_EQ(engine.getLastSolverTermination(),"NO_CONVERGENCE");
    EXPECT_FALSE(engine.processFrame(nullptr,96,96,nullptr,0,1.1,output));
    EXPECT_EQ(engine.getLastSolverIterations(),0);
    EXPECT_EQ(engine.getLastSolverTermination(),"not_run");
}

TEST(EngineContracts, ExplicitExecutionProfileAppliesAndResetsActualOpenCVState) {
    {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    EXPECT_EQ(engine.getExecutionSeed(),-1);
    cv::setRNGSeed(1234);
    engine.setExecutionParams(0,1);
    EXPECT_EQ(engine.getExecutionSeed(),0);
    EXPECT_EQ(engine.getCVThreadCount(),1);
    EXPECT_EQ(cv::theRNG().state,0xffffffffu);
    cv::theRNG().next();
    ASSERT_NE(cv::theRNG().state,0xffffffffu);
    engine.reset();
    EXPECT_EQ(cv::theRNG().state,0xffffffffu);
    EXPECT_EQ(engine.getCVThreadCount(),1);
    engine.setExecutionParams(-1,0);
    EXPECT_EQ(engine.getExecutionSeed(),0);
    EXPECT_EQ(engine.getCVThreadCount(),1);
    }
    // Leave scheduler release to the engine's scope exit; full LSan checks the lifetime.
    std::atomic<int> processed{0};
    cv::parallel_for_(cv::Range(0,4),[&](const cv::Range& range){ processed.fetch_add(range.end-range.start); });
    EXPECT_EQ(processed.load(),4);
}

TEST(EngineContracts, CameraPoseUsesBodyRotationAndLeverArmAndBlankFrameCannotReuseIt) {
    const Eigen::Matrix3d rotation=(Eigen::AngleAxisd(.5,Eigen::Vector3d::UnitX())*
                                    Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitY())).toRotationMatrix();
    Eigen::Matrix<double,3,3,Eigen::RowMajor> row_major=rotation;
    const double translation[3]={.4,-.2,.1};
    VIOEngine engine;
    ASSERT_TRUE(configure(engine,row_major.data(),translation));
    const Eigen::Matrix3d body_rotation=Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    double output[16];
    ASSERT_TRUE(VIOEngineTestAccess::pose(engine,Eigen::Vector3d::Zero(),body_rotation,1,output));
    const Eigen::Matrix4d actual=Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::RowMajor>>(output);
    EXPECT_TRUE((actual.block<3,3>(0,0).isApprox(body_rotation*rotation,1e-12)));
    EXPECT_TRUE((actual.block<3,1>(0,3).isApprox(Eigen::Vector3d(.2,.4,.1),1e-12)));
    VIOEngineTestAccess::estimator(engine).solver_flag_=common::SolverFlag::NON_LINEAR;
    std::vector<uint8_t> blank(96*96,0);
    IMUReading imu[]={{1,0,0,9.81,0,0,0},{1.2,0,0,9.81,0,0,0}};
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,imu,2,1.05,output));
    EXPECT_EQ(engine.getLastReason(),"empty_features");
    EXPECT_EQ(engine.getStatusCode(),static_cast<int>(VIOStatus::LOST));
    EXPECT_EQ(engine.getPoseTimestamp(),-1);
    double map[3]; EXPECT_EQ(engine.getMapPoints(map,1),0);
    for(double value:output) EXPECT_TRUE(std::isnan(value));
}

TEST(EngineContracts, RansacPrefiltersBothRayEndpointsBeforeDivisionAndPreservesAlignedMetadata) {
    utility::g_config.camera.col=utility::g_config.camera.row=96;
    utility::g_config.camera.focal_length=100;
    utility::g_config.feature_tracker.f_threshold_edge_factor=0;
    frontend::FeatureTracker tracker;
    tracker.m_camera=std::make_shared<BoundaryRayCamera>();
    for(int i=0;i<30;++i) {
        cv::Point2f current(20+(i%8)*5,20+(i/8)*5),next(current.x+.5f,current.y+.25f);
        if(i<3) current.x=i+1;
        if(i>=3 && i<6) next.x=i-2;
        tracker.cur_pts.push_back(current);tracker.next_pts.push_back(next);
        tracker.prev_pts.emplace_back(i,100+i);
        tracker.cur_undistorted_pts.emplace_back(i,200+i);
        tracker.prev_undistorted_pts.emplace_back(i,300+i);
        tracker.pts_velocity.emplace_back(i,400+i);
        tracker.ids.push_back(i);tracker.track_cnt.push_back(i+2);
    }
    // Six pairs have zero/negative/NaN rays. The 24 valid pairs do not reach the existing 30-point RANSAC gate.
    ASSERT_NO_THROW(tracker.rejectWithFundamentalMatrix());
    ASSERT_EQ(tracker.ids.size(),24u);
    EXPECT_EQ(tracker.cur_pts.size(),24u);EXPECT_EQ(tracker.next_pts.size(),24u);
    EXPECT_EQ(tracker.prev_pts.size(),24u);EXPECT_EQ(tracker.cur_undistorted_pts.size(),24u);
    EXPECT_EQ(tracker.prev_undistorted_pts.size(),24u);EXPECT_EQ(tracker.pts_velocity.size(),24u);
    EXPECT_EQ(tracker.track_cnt.size(),24u);
    for(size_t j=0;j<tracker.ids.size();++j) {
        const int original=static_cast<int>(j)+6;
        EXPECT_EQ(tracker.ids[j],original);
        EXPECT_EQ(tracker.track_cnt[j],original+2);
        EXPECT_EQ(tracker.prev_pts[j],cv::Point2f(original,100+original));
        EXPECT_EQ(tracker.cur_undistorted_pts[j],cv::Point2f(original,200+original));
        EXPECT_EQ(tracker.prev_undistorted_pts[j],cv::Point2f(original,300+original));
        EXPECT_EQ(tracker.pts_velocity[j],cv::Point2f(original,400+original));
    }
}

TEST(EngineContracts, InvalidCurrentRaysPruneHistoryBeforeTheFollowingTrackingFrame) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    utility::g_config.feature_tracker.equalize=0;
    frontend::FeatureTracker tracker;
    const auto camera=std::make_shared<BoundaryRayCamera>();
    tracker.m_camera=camera;
    cv::Mat image(96,96,CV_8UC1,cv::Scalar(0));
    for(int y=8;y<90;y+=16)for(int x=8;x<90;x+=16)
        cv::rectangle(image,cv::Rect(x,y,7,7),cv::Scalar(255),-1);
    tracker.detectAndTrack(image,1);
    for(unsigned int i=0;tracker.updateID(i);++i){}
    const size_t before=tracker.cur_pts.size();
    ASSERT_GT(before,0u);
    camera->reject_right_half=true;
    tracker.detectAndTrack(image,1.05);
    ASSERT_GT(tracker.cur_pts.size(),0u);
    ASSERT_LT(tracker.cur_pts.size(),before);
    EXPECT_LE(tracker.prev_pts.size(),tracker.cur_pts.size());
    EXPECT_LE(tracker.prev_undistorted_pts.size(),tracker.cur_pts.size());
    ASSERT_NO_THROW(tracker.detectAndTrack(image,1.1));
    EXPECT_EQ(tracker.cur_pts.size(),tracker.ids.size());
    EXPECT_EQ(tracker.cur_pts.size(),tracker.cur_undistorted_pts.size());
    EXPECT_EQ(tracker.cur_pts.size(),tracker.pts_velocity.size());
    EXPECT_EQ(tracker.cur_pts.size(),tracker.track_cnt.size());
}

TEST(EngineContracts, EqualPriorityMaskUsesStableIdsWithoutChangingLongerTrackPriority) {
    utility::g_config.camera.col=utility::g_config.camera.row=96;
    utility::g_config.feature_tracker.fisheye=0;
    utility::g_config.feature_tracker.min_dist=10;
    frontend::FeatureTracker tracker;
    tracker.next_pts={{30,30},{35,30},{60,60},{65,60}};
    tracker.ids={5,2,9,1};
    tracker.track_cnt={4,4,5,4};
    tracker.setMask();
    // Longest track9 wins its nearby group; equal-length group keeps lower id2 instead of input-order id5.
    EXPECT_EQ(tracker.ids,(std::vector<int>{9,2}));
    EXPECT_EQ(tracker.track_cnt,(std::vector<int>{5,4}));
    EXPECT_EQ(tracker.next_pts,(std::vector<cv::Point2f>{{60,60},{35,30}}));
}

TEST(EngineContracts, DiagnosticCaptureIsOptInAndEmitsJsonFixtureWithoutChangingFrameValidity) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    EXPECT_EQ(engine.getFeatureDiagnostics(),"{\"enabled\":false}");
    engine.setDiagnosticCapture(true);
    cv::Mat image(96,96,CV_8UC1,cv::Scalar(0));
    for(int y=8;y<90;y+=16)for(int x=8;x<90;x+=16)
        cv::rectangle(image,cv::Rect(x,y,7,7),cv::Scalar(255),-1);
    double output[16];
    IMUReading imu[]={{0,0,0,9.81,0,0,0},{.1,0,0,9.81,0,0,0}};
    EXPECT_FALSE(engine.processFrame(image.data,96,96,imu,2,0,output));
    const auto snapshot=engine.getFeatureDiagnostics();
    EXPECT_NE(snapshot.find("\"enabled\":true"),std::string::npos);
    EXPECT_NE(snapshot.find("\"clahe_image\""),std::string::npos);
    EXPECT_NE(snapshot.find("\"mask_sorted\""),std::string::npos);
    EXPECT_NE(snapshot.find("\"gftt_new\""),std::string::npos);
    EXPECT_NE(snapshot.find("projection_sqrt_info_00"),std::string::npos);
    EXPECT_FALSE(engine.getPoseValid());
    const auto directory=std::filesystem::path(__FILE__).parent_path().parent_path()/"build/refactor-evidence/engine-green";
    std::filesystem::create_directories(directory);
    std::ofstream(directory/"diagnostic-fixture.json")<<snapshot;
    engine.setDiagnosticCapture(false);
    EXPECT_EQ(engine.getFeatureDiagnostics(),"{\"enabled\":false}");
}
