#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "backend/sliding_window.h"
#include "utility/config.h"

class SlidingWindowTest : public ::testing::Test {
protected:
    void SetUp() override {
        utility::g_config.estimator.acc_n = 0.08;
        utility::g_config.estimator.acc_w = 0.00004;
        utility::g_config.estimator.gyr_n = 0.004;
        utility::g_config.estimator.gyr_w = 2.0e-6;
        utility::g_config.estimator.g = Eigen::Vector3d(0.0, 0.0, 9.81007);
    }
};

TEST_F(SlidingWindowTest, ClearSlidingWindow) {
    backend::SlidingWindow sw;
    sw.clearSlidingWindow();

    for (int i = 0; i <= WINDOW_SIZE; i++) {
        EXPECT_TRUE(sw[i].P.isApprox(Eigen::Vector3d::Zero()));
        EXPECT_TRUE(sw[i].V.isApprox(Eigen::Vector3d::Zero()));
        EXPECT_TRUE(sw[i].Ba.isApprox(Eigen::Vector3d::Zero()));
        EXPECT_TRUE(sw[i].Bg.isApprox(Eigen::Vector3d::Zero()));
    }
}

TEST_F(SlidingWindowTest, CopyFrame) {
    backend::SlidingWindow sw;
    sw.clearSlidingWindow();

    sw[0].P = Eigen::Vector3d(1.0, 2.0, 3.0);
    sw[0].R = Eigen::Matrix3d::Identity();
    sw[0].V = Eigen::Vector3d(0.1, 0.2, 0.3);

    sw.copyFrame(1, 0);

    EXPECT_TRUE(sw[1].P.isApprox(sw[0].P));
    EXPECT_TRUE(sw[1].V.isApprox(sw[0].V));
    EXPECT_TRUE(sw[1].R.isApprox(sw[0].R));
}

TEST_F(SlidingWindowTest, SwapFrame) {
    backend::SlidingWindow sw;
    sw.clearSlidingWindow();

    sw[0].P = Eigen::Vector3d(1.0, 2.0, 3.0);
    sw[1].P = Eigen::Vector3d(4.0, 5.0, 6.0);

    Eigen::Vector3d p0 = sw[0].P;
    Eigen::Vector3d p1 = sw[1].P;

    sw.swapFrame(0, 1);

    EXPECT_TRUE(sw[0].P.isApprox(p1));
    EXPECT_TRUE(sw[1].P.isApprox(p0));
}

TEST_F(SlidingWindowTest, PushBackBuffer) {
    backend::SlidingWindow sw;
    sw.clearSlidingWindow();

    Eigen::Vector3d acc(0.0, 0.0, 9.81);
    Eigen::Vector3d gyr(0.0, 0.0, 0.0);

    sw.pushBackBuffer(0, 0.01, acc, gyr);

    EXPECT_EQ(sw[0].dt_buf.size(), 1u);
    EXPECT_EQ(sw[0].linear_acceleration_buf.size(), 1u);
    EXPECT_EQ(sw[0].angular_velocity_buf.size(), 1u);
    EXPECT_NEAR(sw[0].dt_buf[0], 0.01, 1e-10);
}

TEST_F(SlidingWindowTest, CreateNewPreintegration) {
    backend::SlidingWindow sw;
    sw.clearSlidingWindow();

    // Set bias first
    sw[0].Ba = Eigen::Vector3d::Zero();
    sw[0].Bg = Eigen::Vector3d::Zero();

    Eigen::Vector3d acc(0.0, 0.0, 9.81);
    Eigen::Vector3d gyr(0.0, 0.0, 0.0);

    sw.createNewPreintegration(0, acc, gyr);

    EXPECT_NE(sw[0].pre_integration, nullptr);
}
