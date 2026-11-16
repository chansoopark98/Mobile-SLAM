#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <limits>
#include "config/config_manager.h"

class ConfigValidationTest : public ::testing::Test {
protected:
    std::filesystem::path path;
    void SetUp() override {
        path=std::filesystem::temp_directory_path()/"mobile-slam-config-validation.yaml";
        std::ofstream f(path);
        f << "image_width: 512\nimage_height: 512\nprojection_parameters: {fx: 190, fy: 190, cx: 256, cy: 256}\n";
        f.close();
        ASSERT_TRUE(manager().loadConfiguration(path.string()));
    }
    void TearDown() override { std::filesystem::remove(path); }
    config::ConfigManager& manager(){return config::ConfigManager::getInstance();}
};
TEST_F(ConfigValidationTest, ValidConfigPasses){EXPECT_TRUE(manager().validateConfiguration());}
TEST_F(ConfigValidationTest, ZeroFxFails){manager().setParameter<double>("camera.fx",0);EXPECT_FALSE(manager().validateConfiguration());}
TEST_F(ConfigValidationTest, NegativeWindowSizeFails){manager().setParameter<int>("estimator.window_size",-1);EXPECT_FALSE(manager().validateConfiguration());}
TEST_F(ConfigValidationTest, ZeroMaxCntFails){manager().setParameter<int>("feature_tracker.max_cnt",0);EXPECT_FALSE(manager().validateConfiguration());}
TEST_F(ConfigValidationTest, NonFiniteCalibrationFails){manager().setParameter<double>("camera.fx",std::numeric_limits<double>::infinity());EXPECT_FALSE(manager().validateConfiguration());}
TEST_F(ConfigValidationTest, InvalidExtrinsicFails){manager().getConfig()->camera.r_ic(0,0)=2;EXPECT_FALSE(manager().validateConfiguration());}
TEST_F(ConfigValidationTest, InvalidNoiseFails){manager().setParameter<double>("estimator.acc_n",-1);EXPECT_FALSE(manager().validateConfiguration());}
TEST_F(ConfigValidationTest, ZeroAccelerationRandomWalkFails){manager().setParameter<double>("estimator.acc_w",0);EXPECT_FALSE(manager().validateConfiguration());}
TEST_F(ConfigValidationTest, ZeroGyroscopeRandomWalkFails){manager().setParameter<double>("estimator.gyr_w",0);EXPECT_FALSE(manager().validateConfiguration());}
TEST_F(ConfigValidationTest, PositiveRandomWalkValuesRemainValid) {
    manager().setParameter<double>("estimator.acc_w",0.0001);
    manager().setParameter<double>("estimator.gyr_w",0.00002);
    EXPECT_TRUE(manager().validateConfiguration());
}
TEST_F(ConfigValidationTest, RejectedZeroRandomWalkLoadPreservesPriorConfiguration) {
    const auto accepted=manager().getConfig();
    const double accepted_acc_w=accepted->estimator.acc_w;
    const double accepted_gyr_w=accepted->estimator.gyr_w;
    const auto rejected_path=path.string()+".invalid.yaml";
    for (const char* field : {"acc_w","gyr_w"}) {
        SCOPED_TRACE(field);
        std::ofstream file(rejected_path);
        file<<"image_width: 512\nimage_height: 512\n"
            <<"projection_parameters: {fx: 250, fy: 250, cx: 256, cy: 256}\n"
            <<field<<": 0\n";
        file.close();
        EXPECT_FALSE(manager().loadConfiguration(rejected_path));
        EXPECT_EQ(manager().getConfig(),accepted);
        EXPECT_DOUBLE_EQ(manager().getConfig()->camera.fx,190);
        EXPECT_DOUBLE_EQ(manager().getConfig()->estimator.acc_w,accepted_acc_w);
        EXPECT_DOUBLE_EQ(manager().getConfig()->estimator.gyr_w,accepted_gyr_w);
        EXPECT_EQ(manager().getConfig()->config_filepath,path.string());
        EXPECT_TRUE(manager().validateConfiguration());
    }
    std::filesystem::remove(rejected_path);
}
