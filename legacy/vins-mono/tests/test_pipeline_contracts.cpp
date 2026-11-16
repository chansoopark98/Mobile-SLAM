#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <vector>

#include "vio_engine.h"

namespace {
struct ConfigRestore {
    utility::Config saved = utility::g_config;
    Eigen::Matrix2d projection = backend::factor::ProjectionFactor::sqrt_info;
    ~ConfigRestore() {
        utility::g_config = saved;
        backend::factor::ProjectionFactor::sqrt_info = projection;
    }
};

bool configure(VIOEngine& engine) {
    const double rotation[] = {1,0,0,0,1,0,0,0,1}, translation[] = {0,0,0};
    return engine.configure(96,96,100,100,48,48,2,0,0,0,0,
                            rotation,translation,.1,.001,.01,.0001,9.81);
}

double jsonNumber(const std::string& json, const std::string& key) {
    const auto position = json.find('"' + key + "\":");
    EXPECT_NE(position, std::string::npos) << json;
    if (position == std::string::npos) return std::numeric_limits<double>::quiet_NaN();
    return std::stod(json.substr(position + key.size() + 3));
}

TEST(PipelineContracts, UnrunAndRejectedStagesNeverReusePreviousFrameTimings) {
    ConfigRestore restore;
    VIOEngine engine;
    EXPECT_EQ(engine.getPipelineDiagnostics(), "null");
    ASSERT_TRUE(configure(engine));
    std::vector<uint8_t> image(96*96,0);
    double pose[16];
    const IMUReading imu[] = {{.99,0,0,9.81,0,0,0},{1.01,0,0,9.81,0,0,0}};
    EXPECT_FALSE(engine.processFrame(image.data(),96,96,imu,2,1,pose));
    const auto first = engine.getPipelineDiagnostics();
    EXPECT_DOUBLE_EQ(jsonNumber(first,"frameTimestamp"),1);
    EXPECT_DOUBLE_EQ(jsonNumber(first,"imuCount"),2);
    EXPECT_GE(jsonNumber(first,"imuMs"),0);
    EXPECT_GE(jsonNumber(first,"frontendMs"),0);
    EXPECT_GE(jsonNumber(first,"totalMs"),jsonNumber(first,"frontendMs"));
    EXPECT_NE(first.find("\"backendMs\":null"),std::string::npos);
    EXPECT_NE(first.find("\"solver\":null"),std::string::npos);

    EXPECT_FALSE(engine.processFrame(nullptr,96,96,nullptr,0,2,pose));
    const auto rejected = engine.getPipelineDiagnostics();
    EXPECT_DOUBLE_EQ(jsonNumber(rejected,"frameTimestamp"),2);
    EXPECT_GT(jsonNumber(rejected,"sequence"),jsonNumber(first,"sequence"));
    EXPECT_NE(rejected.find("\"imuMs\":null"),std::string::npos);
    EXPECT_NE(rejected.find("\"frontendMs\":null"),std::string::npos);
    EXPECT_NE(rejected.find("\"backendMs\":null"),std::string::npos);
    EXPECT_NE(rejected.find("\"solver\":null"),std::string::npos);
    engine.reset();
    EXPECT_EQ(engine.getPipelineDiagnostics(),"null");
}

TEST(PipelineContracts, InternalResetKeepsAttemptIdentityAndExplicitUnrunStages) {
    ConfigRestore restore;
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    std::vector<uint8_t> image(96*96,0);
    double pose[16];
    const IMUReading imu[] = {{.99,0,0,9.81,0,0,0},{1.01,0,0,9.81,0,0,0}};
    engine.processFrame(image.data(),96,96,imu,2,1,pose);
    const auto epoch = engine.getEpoch();
    EXPECT_FALSE(engine.processFrame(image.data(),96,96,nullptr,0,2,pose));
    EXPECT_EQ(engine.getLastReason(),"frame_gap");
    const auto reset = engine.getPipelineDiagnostics();
    EXPECT_DOUBLE_EQ(jsonNumber(reset,"frameTimestamp"),2);
    EXPECT_DOUBLE_EQ(jsonNumber(reset,"engineEpochBefore"),epoch);
    EXPECT_DOUBLE_EQ(jsonNumber(reset,"engineEpoch"),epoch+1);
    EXPECT_GE(jsonNumber(reset,"totalMs"),0);
    EXPECT_NE(reset.find("\"imuMs\":null"),std::string::npos);
    EXPECT_NE(reset.find("\"solver\":null"),std::string::npos);
}

TEST(PipelineContracts, FactorCountsExplainRejectionWithoutEnablingDetailedCapture) {
    ConfigRestore restore;
    backend::factor::ProjectionFactor::sqrt_info.setIdentity();
    backend::SlidingWindow window;
    frontend::FeatureManager features;
    const Eigen::Matrix<double,7,1> ray = (Eigen::Matrix<double,7,1>() << 0,0,1,48,48,0,0).finished();
    for (int id=0;id<4;++id) {
        features.feature_bank_.emplace_back(id,0);
        auto& track = features.feature_bank_.back();
        track.estimated_depth = 5;
        track.solve_flag = id==1 ? 2 : 1;
        track.feature_per_frame.emplace_back(ray);
        if (id!=0) track.feature_per_frame.emplace_back(ray);
        if (id==2) track.feature_per_frame.back().ray_vector.z()=0;
    }
    backend::Optimizer optimizer(&window,&features);
    EXPECT_FALSE(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
    const auto first = optimizer.getLastSolverDiagnostics();
    EXPECT_EQ(first.immatureFeatures,1);
    EXPECT_EQ(first.eligibleFeatures,3);
    EXPECT_EQ(first.invalidDepthFeatures,1);
    EXPECT_EQ(first.projectionRejects,1);
    EXPECT_EQ(first.visualFactors,1);
    EXPECT_EQ(first.currentVisualFactors,0);
    EXPECT_EQ(first.imuFactors,0);
    EXPECT_EQ(first.rejectedImuFactors,utility::WINDOW_SIZE);
    EXPECT_TRUE(std::isnan(first.ceresMs));
    EXPECT_TRUE(std::isnan(first.marginalizationMs));
    EXPECT_EQ(optimizer.getBackendDiagnostics(),"{\"enabled\":false}");

    features.clearState();
    EXPECT_FALSE(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
    const auto second = optimizer.getLastSolverDiagnostics();
    EXPECT_EQ(second.immatureFeatures,0);
    EXPECT_EQ(second.eligibleFeatures,0);
    EXPECT_EQ(second.invalidDepthFeatures,0);
    EXPECT_EQ(second.projectionRejects,0);
    EXPECT_EQ(second.visualFactors,0);
}

TEST(PipelineContracts, SolverAndMarginalizationTimesAreClearedOnNextRejectedSolve) {
    ConfigRestore restore;
    utility::g_config.estimator.acc_n=.1;utility::g_config.estimator.acc_w=.001;
    utility::g_config.estimator.gyr_n=.01;utility::g_config.estimator.gyr_w=.0001;
    utility::g_config.estimator.g={0,0,9.81};
    backend::factor::ProjectionFactor::sqrt_info.setIdentity();
    backend::SlidingWindow window;
    frontend::FeatureManager features;
    const Eigen::Vector3d zero=Eigen::Vector3d::Zero(),gravity=utility::g_config.estimator.g;
    for (int frame=0;frame<=utility::WINDOW_SIZE;++frame) {
        window[frame].timestamp=.02*frame;
        if (frame==0) continue;
        window.createNewPreintegration(frame,gravity,zero);
        window.pushBackPreintegration(frame,.01,gravity,zero);
        window.pushBackPreintegration(frame,.01,gravity,zero);
    }
    for (int id=0;id<3;++id) {
        features.feature_bank_.emplace_back(id,0);
        auto& track=features.feature_bank_.back();
        track.estimated_depth=5;track.solve_flag=1;
        Eigen::Matrix<double,7,1> ray;
        ray << .1*id,.1*(id%2),1,48,48,0,0;
        for (int frame=0;frame<=utility::WINDOW_SIZE;++frame) track.feature_per_frame.emplace_back(ray);
    }
    backend::Optimizer optimizer(&window,&features);
    ASSERT_TRUE(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
    const auto solved=optimizer.getLastSolverDiagnostics();
    EXPECT_TRUE(std::isfinite(solved.ceresMs));EXPECT_GE(solved.ceresMs,0);
    EXPECT_TRUE(std::isfinite(solved.marginalizationMs));EXPECT_GE(solved.marginalizationMs,0);
    EXPECT_EQ(solved.imuFactors,utility::WINDOW_SIZE);
    EXPECT_EQ(solved.currentVisualFactors,3);
    optimizer.reset();
    features.clearState();
    EXPECT_FALSE(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
    EXPECT_TRUE(std::isnan(optimizer.getLastSolverDiagnostics().ceresMs));
    EXPECT_TRUE(std::isnan(optimizer.getLastSolverDiagnostics().marginalizationMs));
}
}  // namespace
