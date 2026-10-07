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
