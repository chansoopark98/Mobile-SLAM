#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <array>
#include <limits>
#include <new>
#include <vector>

#include "backend/factor/imu_factor.h"
#include "backend/factor/imu_factor_pnp.h"
#include "backend/factor/marginalization_factor.h"
#include "backend/factor/perspective_factor.h"
#include "backend/factor/pose_local_parameterization.h"
#include "backend/factor/projection_factor.h"
#include "backend/optimizer.h"
#include "frontend/pnp_frontend.h"

namespace {
using Pose=std::array<double,7>;
using namespace backend::factor;
const Eigen::Vector3d zero=Eigen::Vector3d::Zero();
Pose makePose(const Eigen::Vector3d& p,const Eigen::Quaterniond& q) {
    return {p.x(),p.y(),p.z(),q.x(),q.y(),q.z(),q.w()};
}
Eigen::Quaterniond rotation(const Pose& p) { return {p[6],p[3],p[4],p[5]}; }
Eigen::Quaterniond independentExp(const Eigen::Vector3d& v) {
    return v.norm()==0 ? Eigen::Quaterniond::Identity() : Eigen::Quaterniond(Eigen::AngleAxisd(v.norm(),v.normalized()));
}
void checkAmbientJacobian(ceres::CostFunction& factor,std::vector<double*> parameters,const std::vector<int>& blocks) {
    const int rows=factor.num_residuals();
    std::vector<double> residual(rows);
    std::vector<std::vector<double>> storage;
    std::vector<double*> jacobians;
    for(int size:factor.parameter_block_sizes()) storage.emplace_back(rows*size);
    for(auto& values:storage) jacobians.push_back(values.data());
    ASSERT_TRUE(factor.Evaluate(parameters.data(),residual.data(),jacobians.data()));
    for(int block:blocks) {
        const int cols=factor.parameter_block_sizes()[block];
        const Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> actual(jacobians[block],rows,cols);
        for(int axis=0;axis<cols;++axis) {
            SCOPED_TRACE(::testing::Message()<<"block="<<block<<" axis="<<axis);
            const double saved=parameters[block][axis];
            Eigen::VectorXd positive(rows),negative(rows);
            parameters[block][axis]=saved+1e-7;ASSERT_TRUE(factor.Evaluate(parameters.data(),positive.data(),nullptr));
            parameters[block][axis]=saved-1e-7;ASSERT_TRUE(factor.Evaluate(parameters.data(),negative.data(),nullptr));
            parameters[block][axis]=saved;
            const Eigen::VectorXd expected=(positive-negative)/2e-7;
            EXPECT_LE((actual.col(axis)-expected).cwiseAbs().maxCoeff(),1e-7+1e-5*expected.cwiseAbs().maxCoeff());
        }
    }
}
struct ConfigRestore {
    utility::Config config=utility::g_config;
    ~ConfigRestore(){utility::g_config=config;}
};
}

TEST(MathGeometryContracts, PosePlusUsesIndependentRightExponentialAndPhysicalRoundTrip) {
    PoseLocalParameterization manifold;
    const auto q=independentExp({.4,-.3,.2});
    Pose x=makePose({.2,-.1,.4},q),y;
    double delta[6]={.1,-.2,.3,.7,-.4,.2},recovered[6];
    ASSERT_TRUE(manifold.Plus(x.data(),delta,y.data()));
    EXPECT_LT((rotation(y).toRotationMatrix()-(q*independentExp({.7,-.4,.2})).toRotationMatrix()).norm(),1e-13);
    EXPECT_LT((Eigen::Map<Eigen::Vector3d>(y.data())-Eigen::Vector3d(.3,-.3,.7)).norm(),1e-14);
    ASSERT_TRUE(manifold.Minus(y.data(),x.data(),recovered));
    EXPECT_LT((Eigen::Map<Eigen::Matrix<double,6,1>>(recovered)-Eigen::Map<Eigen::Matrix<double,6,1>>(delta)).norm(),1e-12);
    Pose equivalent=y;for(int i=3;i<7;++i) equivalent[i]*=-1;
    double same[6];ASSERT_TRUE(manifold.Minus(equivalent.data(),x.data(),same));
    EXPECT_LT((Eigen::Map<Eigen::Matrix<double,6,1>>(same)-Eigen::Map<Eigen::Matrix<double,6,1>>(recovered)).norm(),1e-12);
}

TEST(MathGeometryContracts, PoseManifoldJacobiansAreActualAmbientDerivativesAtUnitStates) {
    PoseLocalParameterization manifold;
    for(double angle:{0.,.6,2.7}) {
        Pose x=makePose({.1,-.2,.3},independentExp(angle*Eigen::Vector3d(1,2,3).normalized()));
        double plus_data[42],minus_data[42];
        ASSERT_TRUE(manifold.PlusJacobian(x.data(),plus_data));ASSERT_TRUE(manifold.MinusJacobian(x.data(),minus_data));
        const Eigen::Map<const Eigen::Matrix<double,7,6,Eigen::RowMajor>> plus(plus_data);
        const Eigen::Map<const Eigen::Matrix<double,6,7,Eigen::RowMajor>> minus(minus_data);
        EXPECT_LT((minus*plus-Eigen::Matrix<double,6,6>::Identity()).norm(),1e-12);
        for(int column=0;column<6;++column) {
            double d[6]={};Pose a,b;d[column]=1e-7;ASSERT_TRUE(manifold.Plus(x.data(),d,a.data()));
            d[column]=-1e-7;ASSERT_TRUE(manifold.Plus(x.data(),d,b.data()));
            const Eigen::Matrix<double,7,1> numerical=(Eigen::Map<Eigen::Matrix<double,7,1>>(a.data())-Eigen::Map<Eigen::Matrix<double,7,1>>(b.data()))/2e-7;
            EXPECT_LT((plus.col(column)-numerical).cwiseAbs().maxCoeff(),1e-8);
        }
        for(int column=0;column<7;++column) {
            Pose a=x,b=x;a[column]+=1e-7;b[column]-=1e-7;double pa[6],pb[6];
            ASSERT_TRUE(manifold.Minus(a.data(),x.data(),pa));ASSERT_TRUE(manifold.Minus(b.data(),x.data(),pb));
            const Eigen::Matrix<double,6,1> numerical=(Eigen::Map<Eigen::Matrix<double,6,1>>(pa)-Eigen::Map<Eigen::Matrix<double,6,1>>(pb))/2e-7;
            EXPECT_LT((minus.col(column)-numerical).cwiseAbs().maxCoeff(),1e-8);
        }
    }
}

TEST(MathGeometryContracts, PosePiBranchIsDeterministicAndInvalidStatesAreRejected) {
    PoseLocalParameterization manifold;
    const Pose x=makePose(zero,Eigen::Quaterniond::Identity());
    for(double sign:{1.,-1.}) {
        Pose y=makePose(zero,Eigen::Quaterniond(0,sign,0,0)),back;
        double difference[6];ASSERT_TRUE(manifold.Minus(y.data(),x.data(),difference));
        EXPECT_NEAR(difference[3],std::acos(-1.),1e-14);
        EXPECT_EQ(difference[4],0);EXPECT_EQ(difference[5],0);
        ASSERT_TRUE(manifold.Plus(x.data(),difference,back.data()));
        EXPECT_LT((rotation(back).toRotationMatrix()-rotation(y).toRotationMatrix()).norm(),1e-13);
    }
    Pose bad=x,out;for(int i=3;i<7;++i) bad[i]=0;double delta[6]={},difference[6];
    EXPECT_FALSE(manifold.Plus(bad.data(),delta,out.data()));
    EXPECT_FALSE(manifold.Minus(bad.data(),x.data(),difference));
    delta[4]=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(manifold.Plus(x.data(),delta,out.data()));
}

TEST(MathGeometryContracts, ProjectionAndPerspectiveExposeTrueNormalizedAmbientDerivatives) {
    ProjectionFactor::sqrt_info.setIdentity();PerspectiveFactor::sqrt_info.setIdentity();
    for(double scale:{1.,1.7}) for(double sign:{1.,-1.}) {
        Pose a=makePose({.1,-.2,.3},independentExp({.2,-.1,.15}));
        Pose b=makePose({.4,.1,.2},independentExp({-.1,.2,.05}));
        Pose ex=makePose({.02,-.03,.01},independentExp({.1,.03,-.05}));
        for(int i=3;i<7;++i){a[i]*=scale*sign;b[i]*=scale;ex[i]*=scale;}
        double rho=.2;
        ProjectionFactor projection({.2,.1,1},{.08,.04,1});
        checkAmbientJacobian(projection,{a.data(),b.data(),ex.data(),&rho},{0,1,2,3});
        PerspectiveFactor perspective({.1,.03},{1,.3,5},10);
        checkAmbientJacobian(perspective,{a.data(),ex.data()},{0,1});
    }
}

TEST(MathGeometryContracts, ImuBiasRotationUsesUnitExponentialWithIndependentKnownResidual) {
    ConfigRestore restore;
    const auto g=utility::g_config.estimator.g;
    IntegrationBase pre(g,zero,zero,zero);for(int i=0;i<10;++i) pre.push_back(.1,g,zero);
    const Eigen::Vector3d bias(.4,0,0);
    const auto residual=pre.evaluate(zero,Eigen::Quaterniond::Identity(),zero,zero,bias,
                                    zero,Eigen::Quaterniond::Identity(),zero,zero,bias);
    EXPECT_NEAR(residual[utility::O_R],2*std::sin(.2),1e-12);
    EXPECT_NEAR(residual[utility::O_R+1],0,1e-14);EXPECT_NEAR(residual[utility::O_R+2],0,1e-14);
}

TEST(MathGeometryContracts, NativeAndPnpImuAmbientBiasDerivativesMatchAtNonzeroCorrection) {
    ConfigRestore restore;
    for(double duration:{.06,1.}) for(double scale:{1.,1.4}) {
        const auto g=utility::g_config.estimator.g;
        IntegrationBase pre(g,zero,zero,zero);for(int i=0;i<10;++i) pre.push_back(duration/10,g,zero);
        pre.covariance.setIdentity();
        Pose a=makePose({.1,-.2,.3},independentExp({.3,-.1,.2}));
        Pose b=makePose({.3,.1,-.2},independentExp({-.1,.2,.1}));
        for(int i=3;i<7;++i){a[i]*=scale;b[i]*=-scale;}
        double va[9]={.2,-.1,.3,.01,-.02,.03,.4,-.2,.3};
        double vb[9]={.1,.2,-.1,.02,-.01,.03,.3,-.1,.2};
        IMUFactor imu(&pre);IMUFactorPnP pnp(&pre);
        std::vector<double*> main_parameters{a.data(),va,b.data(),vb};
        std::vector<double*> pnp_parameters{a.data(),va,va+3,b.data(),vb,vb+3};
        checkAmbientJacobian(imu,main_parameters,{0,1,2,3});
        checkAmbientJacobian(pnp,pnp_parameters,{0,1,2,3,4,5});
        Eigen::Matrix<double,15,1> ra,rb;
        ASSERT_TRUE(imu.Evaluate(main_parameters.data(),ra.data(),nullptr));ASSERT_TRUE(pnp.Evaluate(pnp_parameters.data(),rb.data(),nullptr));
        EXPECT_LT((ra-rb).norm(),1e-12);
    }
}

TEST(MathGeometryContracts, PriorPiQuaternionRepresentativesHaveIdenticalPhysicalCost) {
    Pose reference=makePose(zero,Eigen::Quaterniond::Identity());
    MarginalizationInfo info;info.n=6;info.m=0;info.keep_block_size={7};info.keep_block_idx={0};info.keep_block_data={reference.data()};
    info.linearized_jacobians=Eigen::Matrix<double,6,6>::Identity();info.linearized_residuals=Eigen::VectorXd::Zero(6);info.linearized_residuals[3]=.5;
    MarginalizationFactor prior(&info);
    for(double sign:{1.,-1.}) {
        Pose current=makePose(zero,Eigen::Quaterniond(0,sign,0,0));const double* parameters[]={current.data()};Eigen::VectorXd residual(6);
        ASSERT_TRUE(prior.Evaluate(parameters,residual.data(),nullptr));
        EXPECT_NEAR(residual.squaredNorm(),6.25,1e-14);
    }
}

TEST(MathGeometryContracts, PriorAmbientDerivativeIncludesCurrentChainWithoutChangingFrozenInformation) {
    Pose reference=makePose({.1,-.2,.3},independentExp({.1,.2,-.15}));
    MarginalizationInfo info;info.n=6;info.m=0;info.keep_block_size={7};info.keep_block_idx={0};info.keep_block_data={reference.data()};
    info.linearized_jacobians=Eigen::Matrix<double,6,6>::Identity();info.linearized_jacobians(0,4)=.3;
    info.linearized_residuals=Eigen::VectorXd::Constant(6,.1);
    const Eigen::MatrixXd frozen=info.linearized_jacobians;
    MarginalizationFactor prior(&info);
    for(double angle:{.2,.8,2.4}) {
        Pose current=makePose({.4,.1,-.2},rotation(reference)*independentExp(angle*Eigen::Vector3d(1,2,3).normalized()));
        checkAmbientJacobian(prior,{current.data()},{0});
        EXPECT_TRUE(info.linearized_jacobians.isApprox(frozen,0));
    }
}

TEST(MathGeometryContracts, PriorCostIsInvariantUnderConsistentWorldYawAndTranslation) {
    Pose reference=makePose({.1,-.2,.3},independentExp({.1,.2,-.15}));
    Pose current=makePose({.4,.1,-.2},rotation(reference)*independentExp({.2,-.1,.3}));
    MarginalizationInfo info;info.n=6;info.keep_block_size={7};info.keep_block_idx={0};info.keep_block_data={reference.data()};
    info.linearized_jacobians=Eigen::VectorXd::LinSpaced(6,1,6).asDiagonal();info.linearized_residuals=Eigen::VectorXd::Constant(6,.1);
    MarginalizationFactor prior(&info);const double* parameters[]={current.data()};Eigen::VectorXd before(6),after(6);
    ASSERT_TRUE(prior.Evaluate(parameters,before.data(),nullptr));
    const auto gauge=independentExp({0,0,.7});const Eigen::Vector3d translation(1,2,-3);
    reference=makePose(gauge*Eigen::Map<Eigen::Vector3d>(reference.data())+translation,gauge*rotation(reference));
    current=makePose(gauge*Eigen::Map<Eigen::Vector3d>(current.data())+translation,gauge*rotation(current));
    info.linearized_jacobians.leftCols<3>()=(info.linearized_jacobians.leftCols<3>()*gauge.toRotationMatrix().transpose()).eval();
    ASSERT_TRUE(prior.Evaluate(parameters,after.data(),nullptr));
    EXPECT_LT((before-after).norm(),1e-12);
}

struct IndependentAmbientPoseCost {
    template<typename T> bool operator()(const T* p,const T* nuisance,T* residual) const {
        Eigen::Quaternion<T> q(p[6],p[3],p[4],p[5]);q.normalize();
        const Eigen::Matrix<T,3,1> direction=q*Eigen::Matrix<T,3,1>(T(1),T(0),T(0));
        residual[0]=p[0]+nuisance[0]+T(.3);residual[1]=p[1];residual[2]=p[2];
        residual[3]=direction.x();residual[4]=direction.y()+nuisance[0];residual[5]=direction.z();return true;
    }
};
TEST(MathGeometryContracts, MarginalizationConvertsTrueAmbientJacobiansBeforeSchurElimination) {
    Pose p=makePose({.1,-.2,.3},independentExp({.4,-.2,.3}));double nuisance=.4;
    MarginalizationInfo info;
    info.addResidualBlockInfo(new ResidualBlockInfo(new ceres::AutoDiffCostFunction<IndependentAmbientPoseCost,6,7,1>(
        new IndependentAmbientPoseCost),nullptr,{p.data(),&nuisance},{1}));
    info.preMarginalize();ASSERT_TRUE(info.marginalize());ASSERT_EQ(info.m,1);ASSERT_EQ(info.n,6);
    Eigen::Matrix<double,6,6> local=Eigen::Matrix<double,6,6>::Zero();local.topLeftCorner<3,3>().setIdentity();
    const Eigen::Matrix3d cross=(Eigen::Matrix3d()<<0,0,0,0,0,-1,0,1,0).finished();
    local.bottomRightCorner<3,3>()=-rotation(p).toRotationMatrix()*cross;
    Eigen::Matrix<double,6,1> dropped;dropped<<1,0,0,0,1,0;
    const Eigen::Matrix<double,6,6> projector=Eigen::Matrix<double,6,6>::Identity()-dropped*dropped.transpose()/2;
    Eigen::Matrix<double,6,1> residual;IndependentAmbientPoseCost{}(p.data(),&nuisance,residual.data());
    EXPECT_LT((info.linearized_jacobians.transpose()*info.linearized_jacobians-local.transpose()*projector*local).norm(),1e-10);
    EXPECT_LT((info.linearized_jacobians.transpose()*info.linearized_residuals-local.transpose()*projector*residual).norm(),1e-10);
}

TEST(MathGeometryContracts, LegacyCovarianceProfileIsNotSilentlyReinterpretedAsDensity) {
    ConfigRestore restore;
    utility::g_config.estimator.acc_n=utility::g_config.estimator.gyr_n=utility::g_config.estimator.gyr_w=0;
    utility::g_config.estimator.acc_w=1;
    for(double dt:{.005,.02,.1}) {
        IntegrationBase pre(zero,zero,zero,zero);for(int i=0;i<std::lround(1/dt);++i)pre.push_back(dt,zero,zero);
        EXPECT_NEAR(pre.covariance(9,9),dt,1e-14); // Legacy per-sample parameter contract, not continuous density.
    }
}

TEST(MathGeometryContracts, InvalidPoseIsRejectedBeforeCeresManifoldRegistration) {
    for(int invalid=0;invalid<3;++invalid) {
        backend::SlidingWindow window;frontend::FeatureManager features;
        if(invalid==0) window[0].R(0,0)=std::numeric_limits<double>::quiet_NaN();
        else if(invalid==1) window[0].R(0,1)=window[0].R(1,0)=.2; // Quaternion conversion otherwise hides symmetric shear.
        else window[0].R(0,0)=-1;
        backend::Optimizer optimizer(&window,&features);
        EXPECT_FALSE(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
        EXPECT_EQ(optimizer.getLastSolverDiagnostics().reason,"invalid_initial_parameters");
        EXPECT_FALSE(optimizer.hasMarginalizationPrior());
    }
}

TEST(MathGeometryContracts, FirstOptimizerCallPopulatesZeroedStorageBeforeManifoldRegistration) {
    backend::SlidingWindow window;frontend::FeatureManager features;
    alignas(backend::Optimizer) unsigned char storage[sizeof(backend::Optimizer)]{};
    auto* optimizer=new(storage) backend::Optimizer(&window,&features);
    EXPECT_FALSE(optimizer->optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
    EXPECT_EQ(optimizer->getLastSolverDiagnostics().reason,"invalid_or_uncovered_imu_interval");
    optimizer->~Optimizer();
}

TEST(MathGeometryContracts, PnpFirstSolveRegistersPreparedStatesAndRejectsRawShear) {
    ConfigRestore restore;
    utility::g_config.camera.r_ic.setIdentity();utility::g_config.camera.t_ic.setZero();
    std::vector<common::SolvedFeature> features;
    for(int id=0;id<8;++id) {
        common::SolvedFeature feature;feature.id=id;feature.position={.2*(id%4),.2*(id/4),5};
        feature.observation=feature.position.head<2>()/5;feature.track_num=10;features.push_back(feature);
    }
    for(bool shear:{false,true}) {
        frontend::PnPFrontend pnp;pnp.setIMUModel();
        common::VINSResult state;state.timestamp=0;state.P.setZero();state.R.setIdentity();state.V.setZero();state.Ba.setZero();state.Bg.setZero();
        pnp.initializeState(state);pnp.processIMU(0,utility::g_config.estimator.g,zero);
        EXPECT_FALSE(pnp.processImage(features,0));
        if(shear) {state.R(0,1)=state.R(1,0)=.2;pnp.setBackendSolution(state);}
        bool result=false;
        for(int frame=1;frame<=6;++frame) {
            pnp.processIMU(.03,utility::g_config.estimator.g,zero);pnp.processIMU(.03,utility::g_config.estimator.g,zero);
            result=pnp.processImage(features,.06*frame);
        }
        EXPECT_EQ(result,!shear);
        EXPECT_EQ(pnp.hasUsableLatestImageUpdate(),!shear);
        EXPECT_EQ(pnp.getLastSolverDiagnostics().reason,shear ? "invalid_initial_parameters" : "usable");
    }
}
