#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "backend/factor/integration_base.h"
#include "backend/factor/covariance_whitening.h"
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

TEST_F(IntegrationBaseTest, AffineAccelerationUsesItsExactPositionTimeMoment) {
    const Eigen::Vector3d a0(.3, -.6, 9.81), jerk(4., -3., 2.);
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    for (const auto& intervals : {std::vector<double>{.037}, std::vector<double>{.009, .017, .011}}) {
        backend::factor::IntegrationBase value(a0, zero, zero, zero);
        double t = 0;
        for (double h : intervals) { t += h; value.push_back(h, a0 + jerk*t, zero); }
        const Eigen::Vector3d expected_p = .5*a0*t*t + jerk*t*t*t/6.;
        const Eigen::Vector3d expected_v = a0*t + .5*jerk*t*t;
        EXPECT_LT((value.delta_p - expected_p).norm(), 1e-14);
        EXPECT_LT((value.delta_v - expected_v).norm(), 1e-14);
    }
}

TEST_F(IntegrationBaseTest, SingleIntervalHasAnalyticFullRankEndpointNoiseCovariance) {
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    const double h=.031, variance=utility::g_config.estimator.acc_n*utility::g_config.estimator.acc_n;
    backend::factor::IntegrationBase value(zero, zero, zero, zero);
    value.push_back(h, zero, zero);
    ASSERT_TRUE(value.hasUsableInterval());
    for (int axis=0; axis<3; ++axis) {
        EXPECT_NEAR(value.covariance(axis,axis), 5.*variance*std::pow(h,4)/36., 1e-18);
        EXPECT_NEAR(value.covariance(axis,axis+6), variance*std::pow(h,3)/4., 1e-18);
        EXPECT_NEAR(value.covariance(axis+6,axis+6), variance*h*h/2., 1e-18);
        const double conditional=value.covariance(axis,axis)-std::pow(value.covariance(axis,axis+6),2)/value.covariance(axis+6,axis+6);
        EXPECT_NEAR(conditional, variance*std::pow(h,4)/72., 1e-18);
    }
    Eigen::Matrix<double,15,15> whitening;
    ASSERT_TRUE(backend::factor::covarianceWhitening(value.covariance, whitening));
    EXPECT_LT((whitening*value.covariance*whitening.transpose()-Eigen::Matrix<double,15,15>::Identity()).norm(), 1e-10);
}

TEST_F(IntegrationBaseTest, RotatingEndpointNoiseMatchesIndependentMeanPerturbations) {
    using Vector18=Eigen::Matrix<double,18,1>;
    using Vector15=Eigen::Matrix<double,15,1>;
    const Eigen::Vector3d zero=Eigen::Vector3d::Zero(), a0(.8,-.4,9.5), a1(-.3,.7,9.9), w0(1.2,-.8,.5), w1(.9,-.6,.4);
    const double h=.04;
    const Eigen::Quaterniond initial(Eigen::AngleAxisd(.7,Eigen::Vector3d(1,2,-1).normalized()));
    const auto endpoint=[&](const Vector18& n) {
        const Eigen::Vector3d phi=h*(.5*(w0+w1+n.segment<3>(3)+n.segment<3>(9)));
        const Eigen::Quaterniond q=(initial*Eigen::Quaterniond(1.,phi.x()/2.,phi.y()/2.,phi.z()/2.).normalized()).normalized();
        const Eigen::Vector3d u0=initial*(a0+n.segment<3>(0)),u1=q*(a1+n.segment<3>(6));
        Vector15 result=Vector15::Zero();
        result.segment<3>(0)=h*h*(u0/3.+u1/6.);
        result.segment<3>(6)=h*(u0+u1)/2.;
        result.segment<3>(9)=h*n.segment<3>(12);result.segment<3>(12)=h*n.segment<3>(15);
        return std::make_pair(result,q);
    };
    const auto nominal=endpoint(Vector18::Zero());
    Eigen::Matrix<double,15,18> numerical;
    for (int c=0;c<18;++c) {
        Vector18 plus=Vector18::Zero(),minus=Vector18::Zero();plus[c]=1e-5;minus[c]=-1e-5;
        auto a=endpoint(plus),b=endpoint(minus);
        a.first.segment<3>(3)=2.*(nominal.second.conjugate()*a.second).vec();
        b.first.segment<3>(3)=2.*(nominal.second.conjugate()*b.second).vec();
        numerical.col(c)=(a.first-b.first)/2e-5;
    }
    backend::factor::IntegrationBase value(a0,w0,zero,zero);value.delta_q=initial;value.push_back(h,a1,w1);
    const Eigen::Matrix<double,15,15> expected=numerical*value.noise*numerical.transpose();
    EXPECT_LT((value.covariance-expected).norm()/expected.norm(),1e-7);
    Eigen::Matrix<double,15,15> whitening;
    EXPECT_TRUE(backend::factor::covarianceWhitening(value.covariance,whitening));
}

TEST_F(IntegrationBaseTest, RotatingBiasJacobiansMatchRepropagation) {
    const Eigen::Vector3d zero=Eigen::Vector3d::Zero(),a0(.8,-.4,9.5),a1(-.3,.7,9.9),w0(1.2,-.8,.5),w1(.9,-.6,.4);
    for (int intervals : {1,3}) {
        backend::factor::IntegrationBase value(a0,w0,zero,zero);value.push_back(.04,a1,w1);
        if (intervals==3) {
            value.push_back(.017,Eigen::Vector3d(.9,-.2,9.7),Eigen::Vector3d(-.5,1.1,.3));
            value.push_back(.053,Eigen::Vector3d(-.8,.4,9.3),Eigen::Vector3d(.2,.6,-1.2));
        }
        for(int axis=0;axis<6;++axis) {
            Eigen::Vector3d ba=zero,bg=zero;
            (axis<3?ba[axis]:bg[axis-3])=1e-5;
            auto plus=value,minus=value;plus.repropagate(ba,bg);minus.repropagate(-ba,-bg);
            Eigen::Matrix<double,9,1> numerical;
            numerical.segment<3>(0)=(plus.delta_p-minus.delta_p)/2e-5;
            numerical.segment<3>(3)=((value.delta_q.conjugate()*plus.delta_q).vec()-(value.delta_q.conjugate()*minus.delta_q).vec())/1e-5;
            numerical.segment<3>(6)=(plus.delta_v-minus.delta_v)/2e-5;
            EXPECT_LT((value.jacobian.block<9,1>(0,9+axis)-numerical).norm(),1e-8) << intervals << " intervals, axis " << axis;
        }
    }
}
