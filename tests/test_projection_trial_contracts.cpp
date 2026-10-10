#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <array>
#include <limits>
#include <vector>

#include "backend/estimator.h"
#include "backend/factor/pose_local_parameterization.h"
#include "backend/factor/projection_factor.h"

namespace backend {
struct OptimizerTestAccess {
    static void prepare(Optimizer& value) { value.prepareOptimizationParameters(); }
    static double* depth(Optimizer& value, int index) { return value.para_Feature[index]; }
    static double* pose(Optimizer& value, int index) { return value.para_Pose[index]; }
    static bool validState(const Optimizer& value) { return value.validateOptimizationParameters(); }
    static void problem(Optimizer& value, ceres::Problem& problem) {
        value.setupOptimizationProblem(problem);
        value.addFeatureFactors(problem);
    }
    static int filter(Optimizer& value) { return value.filterFinalLandmarks(); }
    static void apply(Optimizer& value) { value.applyOptimizationResults(); }
    static void marginalize(Optimizer& value) { value.marginalizeOldKeyframe(); }
    static std::vector<std::string> priorRoles(const Optimizer& value) {
        std::vector<std::string> roles;
        for (const auto* address : value.last_marginalization_parameter_blocks_)
            roles.push_back(value.diagnosticBlockRole(address));
        return roles;
    }
};
struct EstimatorTestAccess {
    static frontend::FeatureManager& features(Estimator& value) { return value.feature_manager_; }
    static Optimizer& optimizer(Estimator& value) { return value.optimizer_; }
};
}

namespace {
using Pose = std::array<double,7>;
using backend::factor::ProjectionFactor;
struct RestoreConfig {
    utility::Config value = utility::g_config;
    ~RestoreConfig() { utility::g_config = value; }
};
Pose makePose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q = Eigen::Quaterniond::Identity()) {
    return {p.x(),p.y(),p.z(),q.x(),q.y(),q.z(),q.w()};
}
void prepareScene(backend::SlidingWindow& window, frontend::FeatureManager& manager) {
    for (int frame = 0; frame <= utility::WINDOW_SIZE; ++frame) {
        auto& state = window[frame];
        state.timestamp = frame*.06;
        state.P = Eigen::Vector3d(state.timestamp,0,0);
        state.V = Eigen::Vector3d::UnitX();
        state.pre_integration = std::make_unique<backend::factor::IntegrationBase>(
            utility::g_config.estimator.g,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
        for (int i = 0; i < 6; ++i)
            state.pre_integration->push_back(.01,utility::g_config.estimator.g,Eigen::Vector3d::Zero());
        common::ImageData image;
        for (int id = 0; id < 8; ++id) {
            Eigen::Matrix<double,7,1> observation;
            observation << (.2*(id%4)-state.P.x())/5,.2*(id/4)/5,1,0,0,0,0;
            image.emplace(id,observation);
        }
        manager.addFeatureAndCheckParallax(frame,image);
    }
    manager.setDepth(Eigen::VectorXd::Constant(8,.2));
}
}

TEST(ProjectionTrialContracts, SignedAndZeroDepthHaveKnownContinuousResidualAndDerivative) {
    ProjectionFactor::sqrt_info.setIdentity();
    ProjectionFactor factor({.2,.1,1},{.1,.1,1});
    Pose a = makePose(Eigen::Vector3d::Zero()), b = makePose({1,0,0}), ex = a;
    for (double rho : {-.2,-1e-12,0.,1e-12,.2}) {
        SCOPED_TRACE(rho);
        const double* parameters[] = {a.data(),b.data(),ex.data(),&rho};
        double residual[2], depth_jacobian[2];
        double* jacobians[] = {nullptr,nullptr,nullptr,depth_jacobian};
        ASSERT_TRUE(factor.Evaluate(parameters,residual,jacobians));
        EXPECT_NEAR(residual[0],.1-rho,1e-14);
        EXPECT_NEAR(residual[1],0,1e-14);
        EXPECT_NEAR(depth_jacobian[0],-1,1e-14);
        EXPECT_NEAR(depth_jacobian[1],0,1e-14);
    }
}

TEST(ProjectionTrialContracts, EveryLocalJacobianMatchesActualPosePerturbationAcrossInfinity) {
    ProjectionFactor::sqrt_info << 2,.3,-.1,1.5;
    const Eigen::Vector3d ray(.2,-.1,1), observation(.05,.02,1);
    backend::factor::PoseLocalParameterization manifold;
    for (double angle : {-.5,.2,.7}) for (double rho : {-.3,0.,.2}) {
        SCOPED_TRACE(::testing::Message()<<"angle="<<angle<<" rho="<<rho);
        Pose a = makePose({.1,-.2,.3},Eigen::Quaterniond(Eigen::AngleAxisd(angle,Eigen::Vector3d(1,2,3).normalized())));
        Pose b = makePose({.4,.1,-.1},Eigen::Quaterniond(Eigen::AngleAxisd(-.3,Eigen::Vector3d(2,-1,3).normalized())));
        Pose ex = makePose({.03,-.02,.01},Eigen::Quaterniond(Eigen::AngleAxisd(.15,Eigen::Vector3d(-1,2,1).normalized())));
        ProjectionFactor factor(ray,observation);
        double* parameters[] = {a.data(),b.data(),ex.data(),&rho};
        double residual[2], ja[14], jb[14], je[14], jr[2];
        double* jacobians[] = {ja,jb,je,jr};
        ASSERT_TRUE(factor.Evaluate(parameters,residual,jacobians));
        for (int block = 0; block < 3; ++block) {
            Pose original;std::copy(parameters[block],parameters[block]+7,original.begin());
            double plus_jacobian[42];manifold.PlusJacobian(original.data(),plus_jacobian);
            const Eigen::Matrix<double,2,6> actual = Eigen::Map<Eigen::Matrix<double,2,7,Eigen::RowMajor>>(jacobians[block]) *
                Eigen::Map<Eigen::Matrix<double,7,6,Eigen::RowMajor>>(plus_jacobian);
            for (int axis = 0; axis < 6; ++axis) {
                double delta[6] = {}, positive[2], negative[2];delta[axis]=1e-7;
                manifold.Plus(original.data(),delta,parameters[block]);ASSERT_TRUE(factor.Evaluate(parameters,positive,nullptr));
                delta[axis]=-1e-7;manifold.Plus(original.data(),delta,parameters[block]);ASSERT_TRUE(factor.Evaluate(parameters,negative,nullptr));
                const Eigen::Vector2d numerical=(Eigen::Map<Eigen::Vector2d>(positive)-Eigen::Map<Eigen::Vector2d>(negative))/2e-7;
                EXPECT_LT((actual.col(axis)-numerical).cwiseAbs().maxCoeff(),1e-7);
            }
            std::copy(original.begin(),original.end(),parameters[block]);
            if (rho==0) EXPECT_EQ(actual.leftCols<3>().norm(),0);
        }
        const double original_rho=rho;double positive[2],negative[2];rho=original_rho+1e-7;
        ASSERT_TRUE(factor.Evaluate(parameters,positive,nullptr));rho=original_rho-1e-7;
        ASSERT_TRUE(factor.Evaluate(parameters,negative,nullptr));rho=original_rho;
        EXPECT_LT((Eigen::Map<Eigen::Vector2d>(jr)-(Eigen::Map<Eigen::Vector2d>(positive)-Eigen::Map<Eigen::Vector2d>(negative))/2e-7).norm(),1e-7);
    }
}

TEST(ProjectionTrialContracts, InvalidObservationsAndProjectionPoleRemainRejected) {
    ProjectionFactor::sqrt_info.setIdentity();
    Pose a=makePose(Eigen::Vector3d::Zero()),b=makePose({0,0,5}),ex=a;double rho=.2,residual[2];
    const double* parameters[]={a.data(),b.data(),ex.data(),&rho};
    ProjectionFactor pole({.2,.1,1},{.1,.1,1});
    EXPECT_FALSE(pole.Evaluate(parameters,residual,nullptr));
    b[2]=6;EXPECT_TRUE(pole.Evaluate(parameters,residual,nullptr)); // Finite trial beyond cheirality boundary.
    b[2]=0;
    for (const auto& ray : {Eigen::Vector3d(1,2,0),Eigen::Vector3d(1,2,-1),
            Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),0,1)}) {
        ProjectionFactor bad_source(ray,{.1,.1,1}),bad_target({.2,.1,1},ray);
        EXPECT_FALSE(bad_source.Evaluate(parameters,residual,nullptr));
        EXPECT_FALSE(bad_target.Evaluate(parameters,residual,nullptr));
    }
    rho=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(pole.Evaluate(parameters,residual,nullptr));
}

TEST(ProjectionTrialContracts, CeresCanCrossInfinityBeforeFinalLandmarkFiltering) {
    ProjectionFactor::sqrt_info.setIdentity();
    Pose a=makePose(Eigen::Vector3d::Zero()),b=makePose({1,0,0}),ex=a;double rho=.2;
    ceres::Problem problem;
    problem.AddResidualBlock(new ProjectionFactor({.2,.1,1},{.4,.1,1}),nullptr,a.data(),b.data(),ex.data(),&rho);
    problem.SetParameterBlockConstant(a.data());problem.SetParameterBlockConstant(b.data());problem.SetParameterBlockConstant(ex.data());
    ceres::Solver::Options options;options.linear_solver_type=ceres::DENSE_QR;options.trust_region_strategy_type=ceres::DOGLEG;
    options.max_num_iterations=10;options.logging_type=ceres::SILENT;
    ceres::Solver::Summary summary;ceres::Solve(options,&problem,&summary);
    ASSERT_TRUE(summary.IsSolutionUsable());
    EXPECT_NEAR(rho,-.2,1e-10);
    EXPECT_LT(summary.final_cost,1e-18);
}

TEST(ProjectionTrialContracts, OneBadLandmarkDoesNotInvalidateFiniteBodyState) {
    RestoreConfig restore;
    backend::SlidingWindow window;frontend::FeatureManager manager;prepareScene(window,manager);
    backend::Optimizer optimizer(&window,&manager);backend::OptimizerTestAccess::prepare(optimizer);
    for (double bad : {-.2,0.,std::numeric_limits<double>::quiet_NaN()}) {
        backend::OptimizerTestAccess::depth(optimizer,0)[0]=bad;
        EXPECT_TRUE(backend::OptimizerTestAccess::validState(optimizer));
    }
    backend::OptimizerTestAccess::pose(optimizer,3)[0]=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(backend::OptimizerTestAccess::validState(optimizer));
}

TEST(ProjectionTrialContracts, FinalCullPreservesFeatureSlotsPriorAndPhysicalMapPoints) {
    RestoreConfig restore;
    utility::g_config.camera.r_ic.setIdentity();utility::g_config.camera.t_ic.setZero();
    backend::Estimator estimator;estimator.setParameter();
    auto& manager=backend::EstimatorTestAccess::features(estimator);
    auto& optimizer=backend::EstimatorTestAccess::optimizer(estimator);
    prepareScene(estimator.sliding_window_,manager);
    estimator.solver_flag_=common::SolverFlag::NON_LINEAR;
    ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
    std::vector<const double*> addresses;
    for (int i=0;i<8;++i) addresses.push_back(backend::OptimizerTestAccess::depth(optimizer,i));
    backend::OptimizerTestAccess::depth(optimizer,0)[0]=-.2;
    backend::OptimizerTestAccess::depth(optimizer,2)[0]=0;
    backend::OptimizerTestAccess::depth(optimizer,4)[0]=std::numeric_limits<double>::quiet_NaN();
    backend::OptimizerTestAccess::depth(optimizer,6)[0]=1; // Positive anchor Z, behind the last camera.
    backend::OptimizerTestAccess::pose(optimizer,utility::WINDOW_SIZE)[2]=2;
    ASSERT_TRUE(backend::OptimizerTestAccess::validState(optimizer));
    EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),4);
    EXPECT_EQ(manager.feature_bank_.size(),8u); // No slot/index deletion before marginalization.
    int row=0;
    for (const auto& feature:manager.feature_bank_) {
        EXPECT_EQ(feature.feature_id,row);
        EXPECT_EQ(backend::OptimizerTestAccess::depth(optimizer,row),addresses[row]);
        EXPECT_DOUBLE_EQ(backend::OptimizerTestAccess::depth(optimizer,row)[0],row%2 ? .2 : -1);
        ++row;
    }
    backend::OptimizerTestAccess::apply(optimizer);
    backend::OptimizerTestAccess::marginalize(optimizer);
    ASSERT_TRUE(optimizer.hasMarginalizationPrior());
    const auto roles=backend::OptimizerTestAccess::priorRoles(optimizer);
    ASSERT_FALSE(roles.empty());
    for (const auto& role:roles) {
        EXPECT_NE(role,"unknown");
        EXPECT_NE(role.substr(0,14),"inverse_depth_"); // Removed depths never become kept prior addresses.
    }
    const auto solved=estimator.getSolvedFeatures();
    ASSERT_EQ(solved.size(),4u);
    EXPECT_EQ(solved[0].id,1);EXPECT_EQ(solved[1].id,3);EXPECT_EQ(solved[2].id,5);EXPECT_EQ(solved[3].id,7);
    const auto points=estimator.getSlidingWindowMapPoints();
    ASSERT_EQ(points.size(),4u);
    for (const auto& point:points) {
        EXPECT_TRUE(point.allFinite());
        EXPECT_NEAR(point.z(),5,1e-12);
        EXPECT_GT((point-estimator.sliding_window_.back().P).z(),0);
    }
    manager.removeFailures();
    EXPECT_EQ(manager.feature_bank_.size(),4u);
    EXPECT_EQ(backend::OptimizerTestAccess::priorRoles(optimizer),roles);
}

TEST(ProjectionTrialContracts, FinalCurrentSupportRequiresTwoActualRetainedImageFactors) {
    RestoreConfig restore;
    backend::SlidingWindow window;frontend::FeatureManager manager;prepareScene(window,manager);
    backend::Optimizer optimizer(&window,&manager);
    ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
    for (int i=0;i<7;++i) backend::OptimizerTestAccess::depth(optimizer,i)[0]=-.2;
    EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),1);
    backend::OptimizerTestAccess::depth(optimizer,7)[0]=-.2;
    EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),0);

    // An initial projection pole omits every current-image residual from the problem.
    backend::SlidingWindow pole_window;frontend::FeatureManager pole_manager;prepareScene(pole_window,pole_manager);
    pole_window[utility::WINDOW_SIZE].P.z()=5;
    backend::Optimizer pole_optimizer(&pole_window,&pole_manager);
    ceres::Problem pole_problem;backend::OptimizerTestAccess::problem(pole_optimizer,pole_problem);
    backend::OptimizerTestAccess::pose(pole_optimizer,utility::WINDOW_SIZE)[2]=0;
    EXPECT_EQ(backend::OptimizerTestAccess::filter(pole_optimizer),0);
    for (int i=0;i<8;++i) EXPECT_DOUBLE_EQ(backend::OptimizerTestAccess::depth(pole_optimizer,i)[0],.2);
}

TEST(ProjectionTrialContracts, InitialBadDepthDoesNotDiscardHealthyPoseOrPrior) {
    RestoreConfig restore;utility::g_config.estimator.num_iterations=0;
    backend::SlidingWindow window;frontend::FeatureManager manager;prepareScene(window,manager);
    Eigen::VectorXd depths=Eigen::VectorXd::Constant(8,.2);depths[0]=-.2;
    manager.setDepth(depths);
    backend::Optimizer optimizer(&window,&manager);
    const Eigen::Vector3d position=window.back().P;
    ASSERT_TRUE(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
    EXPECT_TRUE(optimizer.getLastSolverDiagnostics().usable);
    EXPECT_EQ(optimizer.getLastSolverDiagnostics().currentVisualFactors,7);
    EXPECT_TRUE(optimizer.hasMarginalizationPrior());
    EXPECT_TRUE(window.back().P.isApprox(position,1e-14));
    EXPECT_EQ(manager.feature_bank_.front().solve_flag,2);
    manager.removeFailures();
    EXPECT_EQ(manager.feature_bank_.size(),7u);
    EXPECT_EQ(manager.feature_bank_.front().feature_id,1);
}

TEST(ProjectionTrialContracts, NoPhysicalCurrentSupportRollsBackWithoutPublishingAnUpdate) {
    RestoreConfig restore;utility::g_config.estimator.num_iterations=0;
    backend::SlidingWindow window;frontend::FeatureManager manager;prepareScene(window,manager);
    window[utility::WINDOW_SIZE].P.z()=6; // Every 5m anchor point is behind this candidate camera.
    const Eigen::Vector3d position=window.back().P;
    backend::Optimizer optimizer(&window,&manager);
    EXPECT_FALSE(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
    const auto& diagnostics=optimizer.getLastSolverDiagnostics();
    EXPECT_EQ(diagnostics.reason,"insufficient_final_current_features");
    EXPECT_FALSE(diagnostics.usable);
    EXPECT_EQ(diagnostics.currentVisualFactors,0);
    EXPECT_TRUE(window.back().P.isApprox(position,0));
    EXPECT_FALSE(optimizer.hasMarginalizationPrior());
    ASSERT_EQ(manager.feature_bank_.size(),8u);
    int row=0;
    for (const auto& feature:manager.feature_bank_) {
        EXPECT_EQ(feature.feature_id,row);
        EXPECT_EQ(feature.solve_flag,1);
        EXPECT_DOUBLE_EQ(feature.estimated_depth,5);
        EXPECT_DOUBLE_EQ(backend::OptimizerTestAccess::depth(optimizer,row)[0],.2);
        ++row;
    }
}
