#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "backend/factor/integration_base.h"
#include "utility/config.h"

class IntegrationBaseTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set default IMU noise parameters
        utility::g_config.estimator.acc_n = 0.08;
        utility::g_config.estimator.acc_w = 0.00004;
        utility::g_config.estimator.gyr_n = 0.004;
        utility::g_config.estimator.gyr_w = 2.0e-6;
        utility::g_config.estimator.g = Eigen::Vector3d(0.0, 0.0, 9.81007);
    }
};

TEST_F(IntegrationBaseTest, ConstructorInitializesCorrectly) {
    Eigen::Vector3d acc0(0.0, 0.0, 9.81);
    Eigen::Vector3d gyr0(0.0, 0.0, 0.0);
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();

    backend::factor::IntegrationBase integration(acc0, gyr0, ba, bg);

    EXPECT_DOUBLE_EQ(integration.sum_dt, 0.0);
    EXPECT_TRUE(integration.delta_p.isApprox(Eigen::Vector3d::Zero()));
    EXPECT_TRUE(integration.delta_v.isApprox(Eigen::Vector3d::Zero()));
    EXPECT_DOUBLE_EQ(integration.delta_q.w(), 1.0);
    EXPECT_DOUBLE_EQ(integration.delta_q.x(), 0.0);
    EXPECT_DOUBLE_EQ(integration.delta_q.y(), 0.0);
    EXPECT_DOUBLE_EQ(integration.delta_q.z(), 0.0);
}

TEST_F(IntegrationBaseTest, PropagateSingleStep) {
    Eigen::Vector3d acc0(0.0, 0.0, 9.81);
    Eigen::Vector3d gyr0(0.0, 0.0, 0.0);
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();

    backend::factor::IntegrationBase integration(acc0, gyr0, ba, bg);

    double dt = 0.01;  // 100Hz IMU
    Eigen::Vector3d acc1(0.0, 0.0, 9.81);
    Eigen::Vector3d gyr1(0.0, 0.0, 0.0);

    integration.push_back(dt, acc1, gyr1);

    EXPECT_NEAR(integration.sum_dt, 0.01, 1e-10);
    // With zero bias and gravity-only acceleration, delta_v should be ~[0,0,9.81]*dt
    EXPECT_NEAR(integration.delta_v.z(), 9.81 * dt, 1e-6);
}

TEST_F(IntegrationBaseTest, RepropagateMaintainsConsistency) {
    Eigen::Vector3d acc0(0.1, 0.2, 9.81);
    Eigen::Vector3d gyr0(0.01, 0.02, 0.03);
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();

    backend::factor::IntegrationBase integration(acc0, gyr0, ba, bg);

    // Add several measurements
    double dt = 0.01;
    for (int i = 0; i < 10; i++) {
        Eigen::Vector3d acc(0.1 + i * 0.01, 0.2, 9.81);
        Eigen::Vector3d gyr(0.01, 0.02, 0.03);
        integration.push_back(dt, acc, gyr);
    }

    Eigen::Vector3d dp_before = integration.delta_p;
    Eigen::Vector3d dv_before = integration.delta_v;

    // Repropagate with same biases should give same result
    integration.repropagate(ba, bg);

    EXPECT_TRUE(integration.delta_p.isApprox(dp_before, 1e-10));
    EXPECT_TRUE(integration.delta_v.isApprox(dv_before, 1e-10));
}
