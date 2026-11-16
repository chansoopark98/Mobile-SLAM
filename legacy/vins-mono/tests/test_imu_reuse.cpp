#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include "backend/factor/imu_factor.h"
#include "backend/factor/imu_factor_pnp.h"

namespace {
using Matrix15 = Eigen::Matrix<double,15,15>;
using Vector15 = Eigen::Matrix<double,15,1>;
using Jacobian = Eigen::Matrix<double,15,Eigen::Dynamic,Eigen::RowMajor>;

class IMUReuse : public ::testing::Test {
protected:
    utility::Config saved = utility::g_config;
    ~IMUReuse() override { utility::g_config = saved; }

    backend::factor::IntegrationBase integration() {
        const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
        backend::factor::IntegrationBase value({.2,-.1,9.81},{.3,-.2,.1},zero,zero);
        for (int sample=0;sample<12;++sample) value.push_back(.005,{.2+.001*sample,-.1,9.81},{.3,-.2,.1});
        Matrix15 lower = Matrix15::Identity();
        for (int row=1;row<15;++row) lower(row,row-1)=.1;
        value.covariance = lower*lower.transpose();
        return value;
    }
};

TEST_F(IMUReuse, CorrectionOutputsPreserveResidualAndPreintegrationState) {
    auto value = integration();
    const auto jacobian=value.jacobian, covariance=value.covariance;
    const auto delta_q=value.delta_q;
    const Eigen::Vector3d p(.1,-.2,.3),v(.2,-.1,.3),ba(.01,-.02,.03),bg(.4,-.2,.3);
    const Eigen::Quaterniond qi(Eigen::AngleAxisd(.3,Eigen::Vector3d(1,2,3).normalized()));
    const Eigen::Quaterniond qj(Eigen::AngleAxisd(-.2,Eigen::Vector3d(2,-1,3).normalized()));
    const auto expected=value.evaluate(p,qi,v,ba,bg,p,qj,v,ba,bg);
    Eigen::Matrix3d correction_jacobian,expected_jacobian;
    Eigen::Quaterniond corrected;
    const auto actual=value.evaluate(p,qi,v,ba,bg,p,qj,v,ba,bg,&corrected,&correction_jacobian);
    const auto expected_q=value.correctedDeltaQuaternion(bg,&expected_jacobian);
    EXPECT_TRUE((expected.array()==actual.array()).all());
    EXPECT_TRUE((corrected.coeffs().array()==expected_q.coeffs().array()).all());
    EXPECT_TRUE((correction_jacobian.array()==expected_jacobian.array()).all());
    EXPECT_TRUE((jacobian.array()==value.jacobian.array()).all());
    EXPECT_TRUE((covariance.array()==value.covariance.array()).all());
    EXPECT_TRUE((delta_q.coeffs().array()==value.delta_q.coeffs().array()).all());
}

TEST_F(IMUReuse, EveryJacobianSelectionMatchesUnchangedSplitBlockFactor) {
    auto value=integration();
    backend::factor::IMUFactor factor(&value);
    backend::factor::IMUFactorPnP reference(&value);
    ASSERT_TRUE(factor.isValid()); ASSERT_TRUE(reference.isValid());
    std::array<double,7> a{.1,-.2,.3}, b{.3,.1,-.2};
    Eigen::Map<Eigen::Quaterniond>(a.data()+3)=Eigen::Quaterniond(Eigen::AngleAxisd(.3,Eigen::Vector3d(1,2,3).normalized()));
    Eigen::Map<Eigen::Quaterniond>(b.data()+3)=Eigen::Quaterniond(Eigen::AngleAxisd(-.2,Eigen::Vector3d(2,-1,3).normalized()));
    for (int axis=3;axis<7;++axis) {a[axis]*=1.4; b[axis]*=-1.7;}
    double va[9]={.2,-.1,.3,.01,-.02,.03,.4,-.2,.3};
    double vb[9]={.1,.2,-.1,.02,-.01,.03,.3,-.1,.2};
    const double* parameters[]={a.data(),va,b.data(),vb};
    const double* split_parameters[]={a.data(),va,va+3,b.data(),vb,vb+3};
    std::array<Jacobian,6> split{Jacobian(15,7),Jacobian(15,3),Jacobian(15,6),
                                Jacobian(15,7),Jacobian(15,3),Jacobian(15,6)};
    std::array<double*,6> split_raw;
    for (int block=0;block<6;++block) split_raw[block]=split[block].data();
    Vector15 expected;
    ASSERT_TRUE(reference.Evaluate(split_parameters,expected.data(),split_raw.data()));
    std::array<Jacobian,4> full{split[0],Jacobian(15,9),split[3],Jacobian(15,9)};
    full[1]<<split[1],split[2]; full[3]<<split[4],split[5];
    for (int mask=-1;mask<16;++mask) {
        SCOPED_TRACE(mask);
        std::array<Jacobian,4> actual{Jacobian(15,7),Jacobian(15,9),Jacobian(15,7),Jacobian(15,9)};
        std::array<double*,4> raw;
        for (int block=0;block<4;++block) raw[block]=(mask&(1<<block))?actual[block].data():nullptr;
        Vector15 residual;
        ASSERT_TRUE(factor.Evaluate(parameters,residual.data(),mask<0?nullptr:raw.data()));
        EXPECT_TRUE((residual.array()==expected.array()).all());
        for (int block=0;mask>=0 && block<4;++block) if (raw[block]) {
            // Split speed/bias whitening products have their own rounding order.
            const double scale=std::max(1.0,full[block].cwiseAbs().maxCoeff());
            EXPECT_LE((actual[block]-full[block]).cwiseAbs().maxCoeff(),1e-12*scale) << "block="<<block;
        }
    }
}
}  // namespace
