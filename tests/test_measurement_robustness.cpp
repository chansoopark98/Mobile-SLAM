#include <gtest/gtest.h>
#include <fstream>
#include "utility/measurement_processor.h"

class MeasurementRobustnessTest : public ::testing::Test {
protected:
    std::string test_dir_ = "/tmp/measurement_robust_test";

    void SetUp() override {
        system(("mkdir -p " + test_dir_).c_str());
    }

    void TearDown() override {
        system(("rm -rf " + test_dir_).c_str());
    }
};

TEST_F(MeasurementRobustnessTest, MalformedImuLineSkipped) {
    std::string imu_path = test_dir_ + "/imu.csv";
    {
        std::ofstream f(imu_path);
        f << "#timestamp [ns],w_RS_S_x,w_RS_S_y,w_RS_S_z,a_RS_S_x,a_RS_S_y,a_RS_S_z" << std::endl;
        // Valid line
        f << "1403636579763555584,-0.099134701513277898,0.14730578886832138,0.02722713633111154,8.1476917083333333,-0.37592158333333331,-2.4026292499999999" << std::endl;
        // Malformed line (not enough fields)
        f << "INVALID_DATA" << std::endl;
        // Another valid line
        f << "1403636579783555584,-0.099134,0.14730,0.02722,8.1476,-0.3759,-2.4026" << std::endl;
    }

    utility::MeasurementProcessor processor;
    bool result = processor.loadImuData(imu_path);
    EXPECT_TRUE(result);
    // Should have loaded 2 valid entries, skipped the malformed one
    EXPECT_EQ(processor.getIMUData().size(), 2u);
}

TEST_F(MeasurementRobustnessTest, EmptyFieldsHandled) {
    std::string imu_path = test_dir_ + "/imu_empty.csv";
    {
        std::ofstream f(imu_path);
        f << "#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z" << std::endl;
        // Line with empty field
        f << "1403636579763555584,,0.14730,0.02722,8.1476,-0.3759,-2.4026" << std::endl;
        // Valid line
        f << "1403636579783555584,-0.099134,0.14730,0.02722,8.1476,-0.3759,-2.4026" << std::endl;
    }

    utility::MeasurementProcessor processor;
    bool result = processor.loadImuData(imu_path);
    EXPECT_TRUE(result);
    // The empty field line should be skipped (stod throws on empty string)
    EXPECT_GE(processor.getIMUData().size(), 1u);
}

TEST_F(MeasurementRobustnessTest, PathTraversalRejected) {
    std::string result = utility::MeasurementProcessor::cleanFilename("../../etc/passwd");
    EXPECT_TRUE(result.empty());
}

TEST_F(MeasurementRobustnessTest, AbsolutePathRejected) {
    std::string result = utility::MeasurementProcessor::cleanFilename("/etc/passwd");
    EXPECT_TRUE(result.empty());
}

TEST_F(MeasurementRobustnessTest, NormalFilenamePreserved) {
    std::string result = utility::MeasurementProcessor::cleanFilename("  1403636579763555584.png  ");
    EXPECT_EQ(result, "1403636579763555584.png");
}
