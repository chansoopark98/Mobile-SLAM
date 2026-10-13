#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <array>
#include <limits>
#include <stdexcept>
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
    static void marginalizeNew(Optimizer& value) { value.marginalizeNewGeneralFrame(); }
    static auto admission(const Optimizer& value) { return value.admitted_feature_slots_; }
    static bool priorHasDepthSlot(const Optimizer& value, int slot) {
        if (!value.last_marginalization_info_) return false;
        const auto& order = value.last_marginalization_info_->getParameterBlockOrder();
        return std::find(order.begin(), order.end(), reinterpret_cast<long>(value.para_Feature[slot])) != order.end();
    }
    static std::array<int,3> projectionPair(const Optimizer& value, const std::vector<double*>& parameters) {
        std::array<int,3> result{-1,-1,-1};
        for(int frame=0;frame<=utility::WINDOW_SIZE;++frame) {
            if(parameters[0]==value.para_Pose[frame])result[0]=frame;
            if(parameters[1]==value.para_Pose[frame])result[1]=frame;
        }
        for(int slot=0;slot<utility::NUM_OF_FEATURES;++slot)
            if(parameters[3]==value.para_Feature[slot])result[2]=slot;
        return result;
    }
    static std::vector<std::array<int,3>> projectionPairs(const Optimizer& value, const ceres::Problem& problem) {
        std::vector<std::array<int,3>> result;
        std::vector<ceres::ResidualBlockId> residuals;problem.GetResidualBlocks(&residuals);
        for(const auto residual:residuals) {
            if(!dynamic_cast<const factor::ProjectionFactor*>(problem.GetCostFunctionForResidualBlock(residual)))continue;
            std::vector<double*> parameters;problem.GetParameterBlocksForResidualBlock(residual,&parameters);
            result.push_back(projectionPair(value,parameters));
        }
        std::sort(result.begin(),result.end());return result;
    }
    static std::vector<std::array<int,3>> priorInputPairs(Optimizer& value, double* residual_squared_norm=nullptr) {
        factor::MarginalizationInfo info;value.addFeatureFactorsForMarginalization(&info);
        std::vector<std::array<int,3>> result;
        if(residual_squared_norm)*residual_squared_norm=0;
        for(const auto* factor:info.factors) {
            result.push_back(projectionPair(value,factor->parameter_blocks));
            if(residual_squared_norm) {
                double residual[2];
                if(!factor->cost_function->Evaluate(factor->parameter_blocks.data(),residual,nullptr))
                    *residual_squared_norm=std::numeric_limits<double>::infinity();
                else *residual_squared_norm+=residual[0]*residual[0]+residual[1]*residual[1];
            }
        }
        std::sort(result.begin(),result.end());return result;
    }
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
    static void slideNew(Estimator& value) { value.frame_count_=utility::WINDOW_SIZE; value.slideWindowNewGeneralFrame(); }
    static void slideOld(Estimator& value) { value.frame_count_=utility::WINDOW_SIZE; value.slideWindowOldKeyframe(); }
    static const auto& images(const Estimator& value) { return value.all_image_frame_; }
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
void prepareScene(backend::Estimator& estimator) {
    estimator.setExperimentalObservationPolicy(2);
    prepareScene(estimator.sliding_window_,backend::EstimatorTestAccess::features(estimator));
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
    backend::Optimizer optimizer(&window,&manager);optimizer.setExperimentalObservationPolicy(2);backend::OptimizerTestAccess::prepare(optimizer);
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
    prepareScene(estimator);
    estimator.sliding_window_[utility::WINDOW_SIZE].P.z()=2;
    // Keep the surviving 5m landmarks consistent with the moved current camera.
    // The 1m candidate below remains behind it, preserving this fixture's cull intent.
    for(auto& feature:manager.feature_bank_) {
        const Eigen::Vector3d point(.2*(feature.feature_id%4),.2*(feature.feature_id/4),5);
        const Eigen::Vector3d camera=point-estimator.sliding_window_.back().P;
        feature.feature_per_frame.back().ray_vector=camera/camera.z();
    }
    estimator.solver_flag_=common::SolverFlag::NON_LINEAR;
    ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
    std::vector<const double*> addresses;
    for (int i=0;i<8;++i) addresses.push_back(backend::OptimizerTestAccess::depth(optimizer,i));
    backend::OptimizerTestAccess::depth(optimizer,0)[0]=-.2;
    backend::OptimizerTestAccess::depth(optimizer,2)[0]=0;
    backend::OptimizerTestAccess::depth(optimizer,4)[0]=std::numeric_limits<double>::quiet_NaN();
    backend::OptimizerTestAccess::depth(optimizer,6)[0]=1; // Positive anchor Z, behind the last camera.
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

TEST(ProjectionTrialContracts, SolvedMapDepthsRemainVisibleAcrossFormerDefaultBoundary) {
    RestoreConfig restore;
    utility::g_config.camera.r_ic.setIdentity();utility::g_config.camera.t_ic.setZero();
    const std::array<double,9> depths={1.98,1.99,1.995,2.,2.005,2.01,2.02,5.,10.};
    for (double configured_default : {2.,5.}) {
        SCOPED_TRACE(configured_default);
        utility::g_config.estimator.init_depth=configured_default;
        backend::Estimator estimator;estimator.setParameter();
        estimator.solver_flag_=common::SolverFlag::NON_LINEAR;
        auto& anchor=estimator.sliding_window_[0];
        anchor.P={1,-2,3};
        anchor.R << 0,-1,0,1,0,0,0,0,1; // Exact 90-degree Z rotation.
        auto& manager=backend::EstimatorTestAccess::features(estimator);
        Eigen::Matrix<double,7,1> observation;observation<<.25,-.125,1,0,0,0,0;
        for (size_t i=0;i<depths.size();++i) {
            manager.feature_bank_.emplace_back(static_cast<int>(i),0);
            auto& feature=manager.feature_bank_.back();
            feature.feature_per_frame.assign(2,frontend::FeaturePerFrame(observation));
            feature.used_num=2;feature.estimated_depth=depths[i];feature.solve_flag=1;
        }
        const auto solved=estimator.getSolvedFeatures();
        const auto points=estimator.getSlidingWindowMapPoints();
        ASSERT_EQ(solved.size(),depths.size());
        ASSERT_EQ(points.size(),depths.size());
        for (size_t i=0;i<depths.size();++i) {
            SCOPED_TRACE(depths[i]);
            // Independent coordinates: Rz(90) maps (d/4,-d/8,d) to (d/8,d/4,d).
            const Eigen::Vector3d expected(1+depths[i]/8,-2+depths[i]/4,3+depths[i]);
            EXPECT_EQ(solved[i].id,static_cast<int>(i));
            EXPECT_LT((solved[i].position-expected).norm(),1e-12);
            EXPECT_LT((points[i]-expected).norm(),1e-12);
        }
    }
}

TEST(ProjectionTrialContracts, MapExportStillExcludesUnsolvedFailedAndInvalidDepths) {
    RestoreConfig restore;
    utility::g_config.camera.r_ic.setIdentity();utility::g_config.camera.t_ic.setZero();
    backend::Estimator estimator;estimator.setParameter();
    estimator.solver_flag_=common::SolverFlag::NON_LINEAR;
    auto& manager=backend::EstimatorTestAccess::features(estimator);
    struct Row { double depth; int solve_flag; };
    const std::array<Row,7> rows={{{3,1},{2,0},{2,2},{0,1},{-1,1},
        {std::numeric_limits<double>::quiet_NaN(),1},{std::numeric_limits<double>::infinity(),1}}};
    Eigen::Matrix<double,7,1> observation;observation<<.25,-.125,1,0,0,0,0;
    for (size_t i=0;i<rows.size();++i) {
        manager.feature_bank_.emplace_back(static_cast<int>(i),0);
        auto& feature=manager.feature_bank_.back();
        feature.feature_per_frame.assign(2,frontend::FeaturePerFrame(observation));
        feature.used_num=2;feature.estimated_depth=rows[i].depth;feature.solve_flag=rows[i].solve_flag;
    }
    const auto solved=estimator.getSolvedFeatures();
    const auto points=estimator.getSlidingWindowMapPoints();
    ASSERT_EQ(solved.size(),1u);ASSERT_EQ(points.size(),1u);
    EXPECT_EQ(solved.front().id,0);
    const Eigen::Vector3d expected(.75,-.375,3);
    EXPECT_LT((solved.front().position-expected).norm(),1e-12);
    EXPECT_LT((points.front()-expected).norm(),1e-12);
    estimator.solver_flag_=common::SolverFlag::INITIAL;
    EXPECT_TRUE(estimator.getSlidingWindowMapPoints().empty());
}

namespace {
Eigen::Matrix<double,7,1> projectKnownPoint(const Eigen::Vector3d& world,
        const Eigen::Matrix3d& rotation, const Eigen::Vector3d& center, double ray_scale=1) {
    const Eigen::Vector3d camera=rotation.transpose()*(world-center);
    Eigen::Matrix<double,7,1> observation;
    observation << ray_scale*camera.x()/camera.z(),ray_scale*camera.y()/camera.z(),ray_scale,37,53,.07,-.09;
    return observation;
}
}

TEST(RecordedRecoveryContracts, StructuralSlotsIncludeYoungUnknownAndSameCappedPrefix) {
    frontend::FeatureManager manager;
    Eigen::Matrix<double,7,1> observation;observation<<.1,-.05,1,37,53,.07,-.09;
    for (int anchor : {0,7,8,9}) {
        manager.feature_bank_.emplace_back(anchor,anchor);
        auto& feature=manager.feature_bank_.back();
        feature.feature_per_frame.assign(2,frontend::FeaturePerFrame(observation));
        feature.estimated_depth=anchor==8 ? -1 : 4;
    }
    EXPECT_EQ(manager.getFeatureCount(),4);
    const auto depths=manager.getDepthVector();
    EXPECT_EQ(depths.size(),4);
    if(depths.size()==4) {
        EXPECT_DOUBLE_EQ(depths(0),.25);EXPECT_DOUBLE_EQ(depths(1),.25);
        EXPECT_DOUBLE_EQ(depths(2),0);EXPECT_DOUBLE_EQ(depths(3),.25);
    }
    manager.clearState();
    for(int id=0;id<=utility::NUM_OF_FEATURES;++id) {
        manager.feature_bank_.emplace_back(id,0);
        auto& feature=manager.feature_bank_.back();
        feature.feature_per_frame.assign(2,frontend::FeaturePerFrame(observation));
        feature.estimated_depth=8;
    }
    EXPECT_EQ(manager.getFeatureCount(),utility::NUM_OF_FEATURES);
    manager.setDepth(Eigen::VectorXd::Constant(utility::NUM_OF_FEATURES,.25));
    EXPECT_DOUBLE_EQ(manager.feature_bank_.back().estimated_depth,8);
    EXPECT_EQ(manager.feature_bank_.back().solve_flag,0);
}

TEST(RecordedRecoveryContracts, PositiveRayScaleIsCanonicalAcrossMapAndResidual) {
    RestoreConfig restore;
    utility::g_config.camera.r_ic.setIdentity();utility::g_config.camera.t_ic.setZero();
    const Eigen::Vector3d point(.4,-.2,4);
    for(double scale : {.2,1.,7.}) {
        SCOPED_TRACE(scale);
        backend::Estimator estimator;estimator.setParameter();
        estimator.solver_flag_=common::SolverFlag::NON_LINEAR;
        auto& manager=backend::EstimatorTestAccess::features(estimator);
        for(int frame=0;frame<2;++frame)
            manager.addFeatureAndCheckParallax(frame,{{42,projectKnownPoint(point,Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero(),scale)}});
        const auto& stored=manager.feature_bank_.front().feature_per_frame.front();
        EXPECT_DOUBLE_EQ(stored.ray_vector.z(),1);
        EXPECT_LT((stored.ray_vector-Eigen::Vector3d(.1,-.05,1)).norm(),1e-14);
        EXPECT_EQ(stored.observation,Eigen::Vector2d(37,53));
        EXPECT_EQ(stored.velocity,Eigen::Vector2d(.07,-.09));
        manager.setDepth(Eigen::VectorXd::Constant(1,.25));
        const auto points=estimator.getSlidingWindowMapPoints();
        ASSERT_EQ(points.size(),1u);EXPECT_LT((points.front()-point).norm(),1e-12);
        Pose a=makePose(Eigen::Vector3d::Zero()),ex=a;double rho=.25,residual[2];
        const double* parameters[]={a.data(),a.data(),ex.data(),&rho};
        ProjectionFactor factor(stored.ray_vector,stored.ray_vector);
        ASSERT_TRUE(factor.Evaluate(parameters,residual,nullptr));
        EXPECT_NEAR(residual[0],0,1e-14);EXPECT_NEAR(residual[1],0,1e-14);
    }
}

TEST(RecordedRecoveryContracts, NewAnchorUsesOriginalOldPoseAndOpticalDepth) {
    RestoreConfig restore;
    const Eigen::Matrix3d ric=Eigen::AngleAxisd(.17,Eigen::Vector3d(1,2,-1).normalized()).toRotationMatrix();
    const Eigen::Vector3d tic(.03,-.02,.04),point(.4,-.2,4);
    utility::g_config.camera.r_ic=ric;utility::g_config.camera.t_ic=tic;
    backend::Estimator estimator;estimator.setParameter();
    auto& old=estimator.sliding_window_[9];auto& next=estimator.sliding_window_[10];
    old.R=Eigen::AngleAxisd(-.12,Eigen::Vector3d::UnitY()).toRotationMatrix();old.P={-.1,.02,0};
    next.R=Eigen::AngleAxisd(.08,Eigen::Vector3d::UnitX()).toRotationMatrix();next.P={.15,-.03,.2};
    const Eigen::Vector3d old_camera=old.R*tic+old.P,new_camera=next.R*tic+next.P;
    const Eigen::Matrix3d old_rotation=old.R*ric,new_rotation=next.R*ric;
    const double old_depth=(old_rotation.transpose()*(point-old_camera)).z();
    const double expected=(new_rotation.transpose()*(point-new_camera)).z();
    auto& manager=backend::EstimatorTestAccess::features(estimator);
    manager.addFeatureAndCheckParallax(9,{{42,projectKnownPoint(point,old_rotation,old_camera,7)}});
    manager.addFeatureAndCheckParallax(10,{{42,projectKnownPoint(point,new_rotation,new_camera,.2)}});
    manager.feature_bank_.front().estimated_depth=old_depth;manager.feature_bank_.front().solve_flag=1;
    backend::EstimatorTestAccess::slideNew(estimator);
    ASSERT_EQ(manager.feature_bank_.size(),1u);
    const auto& feature=manager.feature_bank_.front();
    ASSERT_EQ(feature.feature_per_frame.size(),1u);
    EXPECT_EQ(feature.start_frame,9);EXPECT_NEAR(feature.estimated_depth,expected,1e-12);
    EXPECT_LT((new_rotation*(feature.feature_per_frame.front().ray_vector*feature.estimated_depth)+new_camera-point).norm(),1e-12);
}

TEST(RecordedRecoveryContracts, InactiveUnknownSurvivesActualUsableSolveApply) {
    RestoreConfig restore;
    utility::g_config.estimator.num_iterations=0;
    backend::SlidingWindow window;frontend::FeatureManager manager;prepareScene(window,manager);
    Eigen::Matrix<double,7,1> observation;observation<<.1,-.05,1,0,0,0,0;
    manager.feature_bank_.emplace_front(99,0);
    manager.feature_bank_.front().feature_per_frame.assign(2,frontend::FeaturePerFrame(observation));
    backend::Optimizer optimizer(&window,&manager);optimizer.setExperimentalObservationPolicy(2);
    ASSERT_TRUE(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
    EXPECT_EQ(manager.feature_bank_.front().solve_flag,0);
    EXPECT_DOUBLE_EQ(manager.feature_bank_.front().estimated_depth,-1);
    manager.removeFailures();EXPECT_EQ(manager.feature_bank_.size(),9u);
}

TEST(RecordedRecoveryContracts, ObservableYoungDepthUsesIndependentPointAndConditionBoundary) {
    RestoreConfig restore;
    utility::g_config.camera.fx=400;utility::g_config.camera.fy=600;
    const Eigen::Vector3d point(0,0,4);
    for(double condition : {9990.,10010.}) {
        SCOPED_TRACE(condition);
        const double theta=std::acos(1-2/condition); // Exact two-bearing eigenvalue oracle.
        backend::SlidingWindow window;frontend::FeatureManager manager;
        window[10].P={4*std::tan(theta),0,0};
        manager.addFeatureAndCheckParallax(9,{{42,projectKnownPoint(point,window[9].R,window[9].P)}});
        manager.addFeatureAndCheckParallax(10,{{42,projectKnownPoint(point,window[10].R,window[10].P)}});
        manager.triangulateAcrossAllViews(window,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
        const auto& track=manager.feature_bank_.front();
        if(condition<10000) EXPECT_NEAR(track.estimated_depth,4,1e-9);
        else {EXPECT_EQ(track.solve_flag,0);EXPECT_DOUBLE_EQ(track.estimated_depth,-1);}
    }
}

TEST(RecordedRecoveryContracts, DeferredRotationCannotBypassGeometryWhenAnchorAges) {
    RestoreConfig restore;
    const Eigen::Vector3d point(.4,-.2,4);
    backend::SlidingWindow window;frontend::FeatureManager manager;
    manager.addFeatureAndCheckParallax(9,{{42,projectKnownPoint(point,window[9].R,window[9].P)}});
    manager.addFeatureAndCheckParallax(10,{{42,projectKnownPoint(point,window[10].R,window[10].P)}});
    for(int step=0;step<3;++step) {
        manager.triangulateAcrossAllViews(window,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
        manager.removeFailures();ASSERT_EQ(manager.feature_bank_.size(),1u);
        EXPECT_EQ(manager.feature_bank_.front().solve_flag,0);
        EXPECT_DOUBLE_EQ(manager.feature_bank_.front().estimated_depth,-1);
        if(step<2) manager.removeBack();
    }
    window[9].P={.2,0,0};
    manager.addFeatureAndCheckParallax(9,{{42,projectKnownPoint(point,window[9].R,window[9].P)}});
    manager.triangulateAcrossAllViews(window,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
    ASSERT_EQ(manager.feature_bank_.size(),1u);
    EXPECT_NEAR(manager.feature_bank_.front().estimated_depth,4,1e-9);
}

TEST(RecordedRecoveryContracts, RotatingCameraWithLeverArmDefersUntilIndependentTranslation) {
    RestoreConfig restore;
    const Eigen::Matrix3d ric=Eigen::AngleAxisd(.17,Eigen::Vector3d(1,2,-1).normalized()).toRotationMatrix();
    const Eigen::Vector3d tic(.03,-.02,.04),point(.4,-.2,4);
    backend::SlidingWindow window;frontend::FeatureManager manager;
    for(int frame : {9,10}) {
        window[frame].R=Eigen::AngleAxisd(.13*(frame-8),Eigen::Vector3d::UnitY()).toRotationMatrix();
        window[frame].P=-window[frame].R*tic; // Camera center exactly zero despite body lever arm.
        manager.addFeatureAndCheckParallax(frame,{{42,projectKnownPoint(point,window[frame].R*ric,Eigen::Vector3d::Zero())}});
    }
    for(int step=0;step<3;++step) {
        manager.triangulateAcrossAllViews(window,tic,ric);
        manager.removeFailures();ASSERT_EQ(manager.feature_bank_.size(),1u);
        EXPECT_TRUE(manager.feature_bank_.front().geometry_pending);
        EXPECT_FALSE(manager.feature_bank_.front().geometry_admitted);
        EXPECT_EQ(manager.feature_bank_.front().solve_flag,0);
        EXPECT_DOUBLE_EQ(manager.feature_bank_.front().estimated_depth,-1);
        if(step<2) {
            for(int frame=0;frame<10;++frame) {window[frame].R=window[frame+1].R;window[frame].P=window[frame+1].P;}
            manager.removeBack();
        }
    }
    window[9].R=Eigen::AngleAxisd(.31,Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Vector3d camera(.2,0,0);window[9].P=camera-window[9].R*tic;
    manager.addFeatureAndCheckParallax(9,{{42,projectKnownPoint(point,window[9].R*ric,camera)}});
    manager.triangulateAcrossAllViews(window,tic,ric);
    const auto& feature=manager.feature_bank_.front();
    ASSERT_TRUE(feature.geometry_admitted);EXPECT_TRUE(feature.geometry_pending);
    const Eigen::Vector3d recovered=window[7].R*ric*(feature.feature_per_frame.front().ray_vector*feature.estimated_depth)+
                                  window[7].P+window[7].R*tic;
    EXPECT_LT((recovered-point).norm(),1e-9);
}

TEST(RecordedRecoveryContracts, ReprojectionUsesAnisotropicL2PixelsAndEveryViewCheirality) {
    RestoreConfig restore;
    utility::g_config.camera.fx=400;utility::g_config.camera.fy=600;
    std::array<Eigen::Matrix3d,11> rotations;for(auto& value:rotations)value.setIdentity();
    std::array<Eigen::Vector3d,11> centers;for(auto& value:centers)value.setZero();
    const Eigen::Vector3d point(.4,-.2,4);centers[10]={.2,0,0};
    for(double error : {1.4,1.6}) {
        frontend::FeatureManager manager;
        manager.addFeatureAndCheckParallax(9,{{42,projectKnownPoint(point,rotations[9],centers[9],7)}});
        auto last=projectKnownPoint(point,rotations[10],centers[10],.2);
        last(0)+=.2*.6*error/400;last(1)+=.2*.8*error/600; // Independent L2 residual = error.
        manager.addFeatureAndCheckParallax(10,{{42,last}});
        EXPECT_EQ(frontend::FeatureManager::hasConsistentReprojection(manager.feature_bank_.front(),4,rotations,centers),error<1.5);
        centers[10].z()=5;
        EXPECT_FALSE(frontend::FeatureManager::hasConsistentReprojection(manager.feature_bank_.front(),4,rotations,centers));
        centers[10].z()=0;
    }
}

TEST(RecordedRecoveryContracts, FrozenAdmissionExcludesPositiveInactiveThroughFinalApplyAndOldPrior) {
    RestoreConfig restore;
    utility::g_config.camera.r_ic.setIdentity();utility::g_config.camera.t_ic.setZero();
    backend::Estimator estimator;estimator.setParameter();estimator.solver_flag_=common::SolverFlag::NON_LINEAR;
    auto& manager=backend::EstimatorTestAccess::features(estimator);
    auto& optimizer=backend::EstimatorTestAccess::optimizer(estimator);prepareScene(estimator);
    manager.feature_bank_.emplace_front(99,0);
    auto& pending=manager.feature_bank_.front();
    pending.feature_per_frame.assign(2,frontend::FeaturePerFrame(projectKnownPoint({.4,-.2,4},Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero())));
    pending.estimated_depth=4;pending.geometry_pending=true;pending.geometry_admitted=false;
    ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
    const auto original=backend::OptimizerTestAccess::admission(optimizer);
    EXPECT_FALSE(original[0]);for(int slot=1;slot<9;++slot)EXPECT_TRUE(original[slot]);
    pending.geometry_admitted=true; // Mutating track status cannot add a factor to this solve.
    backend::OptimizerTestAccess::depth(optimizer,2)[0]=-1;
    EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),7);
    backend::OptimizerTestAccess::apply(optimizer);
    EXPECT_EQ(pending.solve_flag,0);EXPECT_TRUE(pending.geometry_pending);EXPECT_DOUBLE_EQ(pending.estimated_depth,4);
    EXPECT_EQ(estimator.getSolvedFeatures().size(),7u);
    backend::OptimizerTestAccess::marginalize(optimizer);
    EXPECT_EQ(backend::OptimizerTestAccess::admission(optimizer),original);
    for(const auto& role:backend::OptimizerTestAccess::priorRoles(optimizer))EXPECT_NE(role.substr(0,14),"inverse_depth_");
    backend::OptimizerTestAccess::marginalizeNew(optimizer);
    for(const auto& role:backend::OptimizerTestAccess::priorRoles(optimizer))EXPECT_NE(role.substr(0,14),"inverse_depth_");
}

TEST(RecordedRecoveryContracts, CanonicalIngressFlowsThroughInitializerStorageSolveOldMapAndPnpInput) {
    RestoreConfig restore;
    utility::g_config.estimator.num_iterations=0;
    const Eigen::Matrix3d ric=Eigen::AngleAxisd(.17,Eigen::Vector3d(1,2,-1).normalized()).toRotationMatrix();
    const Eigen::Vector3d tic(.03,-.02,.04);
    utility::g_config.camera.r_ic=ric;utility::g_config.camera.t_ic=tic;
    std::array<double,14> reference_jacobian{};
    for(double scale : {.2,1.,7.}) {
        backend::Estimator storage;storage.setParameter();
        storage.processImage({{42,projectKnownPoint({.4,-.2,4},ric,tic,scale)}},0);
        const auto& saved=backend::EstimatorTestAccess::images(storage).begin()->second.points.at(42);
        EXPECT_DOUBLE_EQ(saved.z(),1);EXPECT_DOUBLE_EQ(saved(3),37);EXPECT_DOUBLE_EQ(saved(5),.07);
        backend::Estimator estimator;estimator.setParameter();estimator.solver_flag_=common::SolverFlag::NON_LINEAR;
        auto& window=estimator.sliding_window_;auto& manager=backend::EstimatorTestAccess::features(estimator);
        prepareScene(estimator);manager.clearState();
        for(int frame=0;frame<=10;++frame) {
            common::ImageData image;
            for(int id=0;id<8;++id)image.emplace(id,projectKnownPoint({.4+.05*id,-.2,4},ric,window[frame].P+tic,scale));
            manager.addFeatureAndCheckParallax(frame,image);
        }
        manager.triangulateAcrossAllViews(window,tic,ric);
        auto& optimizer=backend::EstimatorTestAccess::optimizer(estimator);
        ASSERT_TRUE(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
        const auto solved=estimator.getSolvedFeatures();ASSERT_EQ(solved.size(),8u);
        const auto points=estimator.getSlidingWindowMapPoints();ASSERT_EQ(points.size(),8u);
        for(int id=0;id<8;++id) {
            const Eigen::Vector3d expected(.4+.05*id,-.2,4);
            EXPECT_LT((solved[id].position-expected).norm(),1e-9);EXPECT_LT((points[id]-expected).norm(),1e-9);
        }
        const auto& feature=manager.feature_bank_.front();
        ProjectionFactor factor(feature.feature_per_frame.front().ray_vector,feature.feature_per_frame.back().ray_vector);
        const double* parameters[]={backend::OptimizerTestAccess::pose(optimizer,0),backend::OptimizerTestAccess::pose(optimizer,10),
                                    nullptr,backend::OptimizerTestAccess::depth(optimizer,0)};
        Pose ex=makePose(tic,Eigen::Quaterniond(ric));parameters[2]=ex.data();
        double residual[2],jacobian[14];double* jacobians[]={jacobian,nullptr,nullptr,nullptr};
        ASSERT_TRUE(factor.Evaluate(parameters,residual,jacobians));
        EXPECT_NEAR(residual[0],0,1e-9);EXPECT_NEAR(residual[1],0,1e-9);
        if(scale==.2)std::copy(jacobian,jacobian+14,reference_jacobian.begin());
        else for(int i=0;i<14;++i)EXPECT_NEAR(jacobian[i],reference_jacobian[i],1e-9);
        backend::EstimatorTestAccess::slideOld(estimator);
        const auto shifted=estimator.getSolvedFeatures();ASSERT_EQ(shifted.size(),8u);
        for(int id=0;id<8;++id)EXPECT_LT((shifted[id].position-Eigen::Vector3d(.4+.05*id,-.2,4)).norm(),1e-9);
    }
}

TEST(RecordedRecoveryContracts, ActualSelectorKeepsTwentyNewFramesWhileObservableYoungTracksSupportCurrent) {
    RestoreConfig restore;
    utility::g_config.estimator.num_iterations=0;
    utility::g_config.camera.fx=400;utility::g_config.camera.fy=400;utility::g_config.camera.focal_length=400;
    utility::g_config.camera.r_ic.setIdentity();utility::g_config.camera.t_ic.setZero();
    backend::Estimator estimator;estimator.setParameter();estimator.solver_flag_=common::SolverFlag::NON_LINEAR;
    auto& window=estimator.sliding_window_;auto& manager=backend::EstimatorTestAccess::features(estimator);
    const auto makeImage=[&](int frame,bool young) {
        common::ImageData image;
        image.emplace(0,projectKnownPoint({0,0,4},window[frame].R,window[frame].P));
        if(young)for(int id=1;id<=24;++id)
            image.emplace(id,projectKnownPoint({.02*(id%6)-.05,.02*(id/6)-.04,4},window[frame].R,window[frame].P));
        return image;
    };
    const auto integrate=[&](int frame) {
        auto& state=window[frame];
        state.pre_integration=std::make_unique<backend::factor::IntegrationBase>(utility::g_config.estimator.g,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
        for(int sample=0;sample<6;++sample) {
            state.pre_integration->push_back(.01,utility::g_config.estimator.g,Eigen::Vector3d::Zero());
            state.dt_buf.push_back(.01);state.linear_acceleration_buf.push_back(utility::g_config.estimator.g);
            state.angular_velocity_buf.push_back(Eigen::Vector3d::Zero());
        }
    };
    for(int frame=0;frame<=9;++frame) {
        window[frame].timestamp=.06*frame;window[frame].P.x()=frame==9 ? .09 : 0;
        integrate(frame);manager.addFeatureAndCheckParallax(frame,makeImage(frame,frame==9));
    }
    manager.feature_bank_.front().estimated_depth=4;manager.feature_bank_.front().solve_flag=1;
    for(int step=0;step<20;++step) {
        SCOPED_TRACE(step);
        window[10].timestamp=.06*(10+step);window[10].P.x()=step%2 ? .09 : 0;
        integrate(10);
        EXPECT_FALSE(manager.addFeatureAndCheckParallax(10,makeImage(10,true)));
        manager.triangulateAcrossAllViews(window,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
        auto& optimizer=backend::EstimatorTestAccess::optimizer(estimator);
        ASSERT_TRUE(optimizer.optimize(common::MarginalizationFlag::MARGIN_NEW_GENERAL_FRAME));
        EXPECT_EQ(optimizer.getLastSolverDiagnostics().currentVisualFactors,25);
        for(const auto& feature:manager.feature_bank_) {
            EXPECT_FALSE(feature.geometry_pending);EXPECT_EQ(feature.solve_flag,1);
            EXPECT_NEAR(feature.estimated_depth,4,1e-9);
        }
        backend::EstimatorTestAccess::slideNew(estimator);
    }
}

TEST(RecordedRecoveryContracts, MixedActualOldNewOldSlidesPreservePriorMapAndDeferredSlotUntilRejectedUpdate) {
    RestoreConfig restore;
    utility::g_config.estimator.num_iterations=0;
    utility::g_config.camera.r_ic.setIdentity();utility::g_config.camera.t_ic.setZero();
    backend::Estimator estimator;estimator.setParameter();estimator.solver_flag_=common::SolverFlag::NON_LINEAR;
    auto& window=estimator.sliding_window_;auto& manager=backend::EstimatorTestAccess::features(estimator);
    auto& optimizer=backend::EstimatorTestAccess::optimizer(estimator);prepareScene(estimator);
    estimator.processIMU(0,utility::g_config.estimator.g,Eigen::Vector3d::Zero());
    const Eigen::Vector3d remote(.6,.1,20);
    manager.feature_bank_.emplace_back(99,9);
    auto& pending=manager.feature_bank_.back();pending.geometry_pending=true;
    for(int frame : {9,10})pending.feature_per_frame.emplace_back(projectKnownPoint(remote,window[frame].R,window[frame].P));
    const auto verify=[&] {
        ASSERT_TRUE(optimizer.hasMarginalizationPrior());
        for(const auto& role:backend::OptimizerTestAccess::priorRoles(optimizer)) {
            EXPECT_NE(role,"unknown");EXPECT_NE(role.substr(0,14),"inverse_depth_");
        }
        ASSERT_EQ(manager.feature_bank_.back().feature_id,99);
        EXPECT_TRUE(manager.feature_bank_.back().geometry_pending);
        EXPECT_EQ(manager.feature_bank_.back().solve_flag,0);
        const auto solved=estimator.getSolvedFeatures();ASSERT_EQ(solved.size(),8u);
        for(int id=0;id<8;++id)
            EXPECT_LT((solved[id].position-Eigen::Vector3d(.2*(id%4),.2*(id/4),5)).norm(),1e-9);
    };
    const auto append=[&](double time,bool observations) {
        auto& state=window[10];state.timestamp=time;state.P={time,0,0};state.V=Eigen::Vector3d::UnitX();
        for(int sample=0;sample<6;++sample) {
            state.pre_integration->push_back(.01,utility::g_config.estimator.g,Eigen::Vector3d::Zero());
            state.dt_buf.push_back(.01);state.linear_acceleration_buf.push_back(utility::g_config.estimator.g);
            state.angular_velocity_buf.push_back(Eigen::Vector3d::Zero());
        }
        common::ImageData image;
        if(observations) {
            for(int id=0;id<8;++id)image.emplace(id,projectKnownPoint({.2*(id%4),.2*(id/4),5},state.R,state.P));
            image.emplace(99,projectKnownPoint(remote,state.R,state.P));
        }
        manager.addFeatureAndCheckParallax(10,image);
        manager.triangulateAcrossAllViews(window,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
    };
    manager.triangulateAcrossAllViews(window,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
    ASSERT_TRUE(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
    backend::EstimatorTestAccess::slideOld(estimator);verify();
    append(.66,true);
    ASSERT_TRUE(optimizer.optimize(common::MarginalizationFlag::MARGIN_NEW_GENERAL_FRAME));
    EXPECT_LT(optimizer.getLastSolverDiagnostics().initialCost,1e-12);
    backend::EstimatorTestAccess::slideNew(estimator);verify();
    append(.72,true);
    ASSERT_TRUE(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
    EXPECT_LT(optimizer.getLastSolverDiagnostics().initialCost,1e-12);
    backend::EstimatorTestAccess::slideOld(estimator);verify();
    EXPECT_EQ(manager.feature_bank_.back().start_frame,7);
    append(.78,false);
    EXPECT_FALSE(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
    EXPECT_EQ(optimizer.getLastSolverDiagnostics().reason,"insufficient_current_features");
    EXPECT_FALSE(optimizer.hasMarginalizationPrior());
    backend::EstimatorTestAccess::slideOld(estimator);
    EXPECT_TRUE(manager.feature_bank_.back().geometry_pending);
    EXPECT_EQ(manager.feature_bank_.back().solve_flag,0);
    const auto remaining=estimator.getSolvedFeatures();ASSERT_EQ(remaining.size(),8u);
    for(int id=0;id<8;++id)
        EXPECT_LT((remaining[id].position-Eigen::Vector3d(.2*(id%4),.2*(id/4),5)).norm(),1e-9);
}

TEST(ProjectionTrialContracts, FinalCurrentSupportRequiresTwoActualRetainedImageFactors) {
    RestoreConfig restore;
    backend::SlidingWindow window;frontend::FeatureManager manager;prepareScene(window,manager);
    backend::Optimizer optimizer(&window,&manager);optimizer.setExperimentalObservationPolicy(2);
    ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
    for (int i=0;i<7;++i) backend::OptimizerTestAccess::depth(optimizer,i)[0]=-.2;
    EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),1);
    backend::OptimizerTestAccess::depth(optimizer,7)[0]=-.2;
    EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),0);

    // An initial projection pole omits every current-image residual from the problem.
    backend::SlidingWindow pole_window;frontend::FeatureManager pole_manager;prepareScene(pole_window,pole_manager);
    pole_window[utility::WINDOW_SIZE].P.z()=5;
    backend::Optimizer pole_optimizer(&pole_window,&pole_manager);pole_optimizer.setExperimentalObservationPolicy(2);
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
    backend::Optimizer optimizer(&window,&manager);optimizer.setExperimentalObservationPolicy(2);
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
    backend::Optimizer optimizer(&window,&manager);optimizer.setExperimentalObservationPolicy(2);
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

TEST(ProjectionTrialContracts, LegacyCurrentOutlierPreservesHistoryWithoutEnteringPnPOrPriorCurrent) {
    RestoreConfig restore;
    utility::g_config.camera.fx=400;utility::g_config.camera.fy=600;
    utility::g_config.camera.r_ic.setIdentity();utility::g_config.camera.t_ic.setZero();
    backend::Estimator estimator;estimator.setParameter();estimator.solver_flag_=common::SolverFlag::NON_LINEAR;
    auto& manager=backend::EstimatorTestAccess::features(estimator);
    auto& optimizer=backend::EstimatorTestAccess::optimizer(estimator);
    prepareScene(estimator);
    manager.feature_bank_.back().feature_per_frame.back().ray_vector.x()+=100./400;
    ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
    backend::OptimizerTestAccess::depth(optimizer,7)[0]=.25; // The historical 5m depth must survive this trial's 4m value.
    const auto* rejected_slot=backend::OptimizerTestAccess::depth(optimizer,7);
    const auto admission=backend::OptimizerTestAccess::admission(optimizer);

    EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),7);
    EXPECT_EQ(manager.feature_bank_.size(),8u);
    EXPECT_EQ(backend::OptimizerTestAccess::depth(optimizer,7),rejected_slot);
    EXPECT_DOUBLE_EQ(rejected_slot[0],.2);
    EXPECT_TRUE(manager.feature_bank_.back().feature_per_frame.back().projection_outlier);
    EXPECT_DOUBLE_EQ(backend::OptimizerTestAccess::depth(optimizer,6)[0],.2);
    EXPECT_EQ(backend::OptimizerTestAccess::admission(optimizer),admission);
    backend::OptimizerTestAccess::apply(optimizer);
    EXPECT_EQ(manager.feature_bank_.back().solve_flag,1);
    EXPECT_DOUBLE_EQ(manager.feature_bank_.back().estimated_depth,5);
    const auto solved=estimator.getSolvedFeatures();ASSERT_EQ(solved.size(),7u);
    for(int id=0;id<7;++id)EXPECT_EQ(solved[id].id,id);
    EXPECT_EQ(estimator.getSlidingWindowMapPoints().size(),8u);
    auto expected_prior=std::vector<std::array<int,3>>{};
    for(int slot=0;slot<8;++slot)for(int frame=1;frame<=10;++frame)
        if(slot!=7 || frame!=10)expected_prior.push_back({0,frame,slot});
    std::sort(expected_prior.begin(),expected_prior.end());
    EXPECT_EQ(backend::OptimizerTestAccess::priorInputPairs(optimizer),expected_prior);
    backend::OptimizerTestAccess::marginalize(optimizer);
    ASSERT_TRUE(optimizer.hasMarginalizationPrior());
    EXPECT_TRUE(backend::OptimizerTestAccess::priorHasDepthSlot(optimizer,6));
    EXPECT_TRUE(backend::OptimizerTestAccess::priorHasDepthSlot(optimizer,7)); // Good historical rows still contribute.
    manager.removeFailures();
    ASSERT_EQ(manager.feature_bank_.size(),8u);
    EXPECT_EQ(manager.feature_bank_.back().feature_id,7);
}

TEST(ProjectionTrialContracts, LegacyCurrentResidualUsesAnisotropicRawPixelsAndCanonicalIngress) {
    RestoreConfig restore;
    utility::g_config.camera.fx=400;utility::g_config.camera.fy=600;
    for(double error : {1.4,1.6})for(double scale : {.2,1.,7.}) {
        SCOPED_TRACE(::testing::Message()<<"error="<<error<<" scale="<<scale);
        backend::SlidingWindow window;frontend::FeatureManager manager;prepareScene(window,manager);
        auto& current=manager.feature_bank_.back().feature_per_frame.back();
        Eigen::Matrix<double,7,1> observation;
        observation << scale*(current.ray_vector.x()+.6*error/400),
                       scale*(current.ray_vector.y()+.8*error/600),scale,0,0,0,0;
        current=frontend::FeaturePerFrame(observation); // Independent L2 pixel error equals error.
        backend::Optimizer optimizer(&window,&manager);optimizer.setExperimentalObservationPolicy(2);
        ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
        EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),error<1.5 ? 8 : 7);
        EXPECT_DOUBLE_EQ(backend::OptimizerTestAccess::depth(optimizer,7)[0],.2);
        EXPECT_EQ(manager.feature_bank_.back().feature_per_frame.back().projection_outlier,error>1.5);
    }
}

TEST(ProjectionTrialContracts, LegacyHistoricalResidualDoesNotExpandCurrentOnlyPolicy) {
    RestoreConfig restore;utility::g_config.camera.fx=400;
    backend::SlidingWindow window;frontend::FeatureManager manager;prepareScene(window,manager);
    manager.feature_bank_.back().feature_per_frame[5].ray_vector.x()+=100./400;
    backend::Optimizer optimizer(&window,&manager);optimizer.setExperimentalObservationPolicy(2);
    ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
    EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),8);
    EXPECT_DOUBLE_EQ(backend::OptimizerTestAccess::depth(optimizer,7)[0],.2);
}

TEST(ProjectionTrialContracts, CurrentImageFactorBoundaryRequiresTwoForPublication) {
    RestoreConfig restore;utility::g_config.estimator.num_iterations=0;
    for(int current_count : {0,1,2}) {
        SCOPED_TRACE(current_count);
        backend::SlidingWindow window;frontend::FeatureManager manager;prepareScene(window,manager);
        for(auto& feature:manager.feature_bank_)
            if(feature.feature_id>=current_count)feature.feature_per_frame.pop_back();
        backend::Optimizer optimizer(&window,&manager);optimizer.setExperimentalObservationPolicy(2);
        EXPECT_EQ(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME),current_count==2);
        EXPECT_EQ(optimizer.getLastSolverDiagnostics().currentVisualFactors,current_count);
        EXPECT_EQ(optimizer.getLastSolverDiagnostics().reason,current_count==2 ? "usable" : "insufficient_current_features");
    }
}

TEST(ProjectionTrialContracts, TwoCurrentOutliersRollbackAndOnlyNewFrameObservationsAreRevalidated) {
    RestoreConfig restore;utility::g_config.estimator.num_iterations=0;
    utility::g_config.camera.fx=400;utility::g_config.camera.fy=600;
    utility::g_config.camera.r_ic.setIdentity();utility::g_config.camera.t_ic.setZero();
    backend::Estimator estimator;estimator.setParameter();estimator.solver_flag_=common::SolverFlag::NON_LINEAR;
    auto& window=estimator.sliding_window_;
    auto& manager=backend::EstimatorTestAccess::features(estimator);prepareScene(window,manager);
    for(auto& feature:manager.feature_bank_) {
        if(feature.feature_id>=2)feature.feature_per_frame.pop_back();
        else feature.feature_per_frame.back().ray_vector.x()+=100./400;
    }
    const Eigen::Vector3d position=window.back().P;
    backend::Optimizer optimizer(&window,&manager);optimizer.setExperimentalObservationPolicy(2);
    EXPECT_FALSE(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
    EXPECT_EQ(optimizer.getLastSolverDiagnostics().reason,"insufficient_final_current_features");
    EXPECT_EQ(optimizer.getLastSolverDiagnostics().currentVisualFactors,0);
    EXPECT_FALSE(optimizer.hasMarginalizationPrior());
    EXPECT_TRUE(window.back().P.isApprox(position,0));
    int slot=0;
    for(auto& feature:manager.feature_bank_) {
        EXPECT_EQ(feature.solve_flag,1);EXPECT_DOUBLE_EQ(feature.estimated_depth,5);
        EXPECT_DOUBLE_EQ(backend::OptimizerTestAccess::depth(optimizer,slot++)[0],.2);
        if(feature.feature_id<2) {
            EXPECT_TRUE(feature.feature_per_frame.back().projection_outlier);
            feature.feature_per_frame.back().ray_vector.x()-=100./400;
        }
    }
    EXPECT_FALSE(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
    EXPECT_EQ(optimizer.getLastSolverDiagnostics().reason,"insufficient_current_features");
    backend::EstimatorTestAccess::slideOld(estimator);
    window[10].timestamp=.66;window[10].P={.66,0,0};
    window.createNewPreintegration(10,utility::g_config.estimator.g,Eigen::Vector3d::Zero());
    for(int sample=0;sample<6;++sample)
        window[10].pre_integration->push_back(.01,utility::g_config.estimator.g,Eigen::Vector3d::Zero());
    common::ImageData image;
    for(int id=0;id<2;++id)image.emplace(id,projectKnownPoint({.2*id,0,5},window[10].R,window[10].P));
    manager.addFeatureAndCheckParallax(10,image);
    for(const auto& feature:manager.feature_bank_)if(feature.feature_id<2) {
        EXPECT_TRUE(feature.feature_per_frame[9].projection_outlier);
        EXPECT_FALSE(feature.feature_per_frame.back().projection_outlier);
    }
    ASSERT_TRUE(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
    EXPECT_EQ(optimizer.getLastSolverDiagnostics().currentVisualFactors,2);
    EXPECT_TRUE(optimizer.hasMarginalizationPrior());
}

TEST(ProjectionTrialContracts, SolvedPureRotationRetainsCurrentMetricSupport) {
    RestoreConfig restore;utility::g_config.estimator.num_iterations=0;
    utility::g_config.camera.r_ic.setIdentity();utility::g_config.camera.t_ic.setZero();
    backend::SlidingWindow window;frontend::FeatureManager manager;prepareScene(window,manager);
    manager.clearState();
    for(int frame=0;frame<=utility::WINDOW_SIZE;++frame) {
        auto& state=window[frame];state.P.setZero();state.V.setZero();
        state.R=Eigen::AngleAxisd(.025*frame,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
        common::ImageData image;
        for(int id=0;id<8;++id)
            image.emplace(id,projectKnownPoint({.2*(id%4),.2*(id/4),5},state.R,state.P));
        manager.addFeatureAndCheckParallax(frame,image);
    }
    manager.setDepth(Eigen::VectorXd::Constant(8,.2));
    backend::Optimizer optimizer(&window,&manager);optimizer.setExperimentalObservationPolicy(2);
    ASSERT_TRUE(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
    EXPECT_EQ(optimizer.getLastSolverDiagnostics().currentVisualFactors,8);
    for(const auto& feature:manager.feature_bank_) {
        EXPECT_EQ(feature.solve_flag,1);EXPECT_NEAR(feature.estimated_depth,5,1e-12);
    }
}

TEST(ObservationRejection, FlaggedResidualsKeepActualPosePairsInSolveAndOldPrior) {
    RestoreConfig restore;utility::g_config.camera.fx=400;utility::g_config.camera.fy=600;
    backend::SlidingWindow window;frontend::FeatureManager manager;prepareScene(window,manager);
    manager.feature_bank_.erase(std::next(manager.feature_bank_.begin()),manager.feature_bank_.end());
    auto& feature=manager.feature_bank_.front();
    feature.feature_per_frame[3].ray_vector.x()+=100./400;
    feature.feature_per_frame[3].projection_outlier=true;
    feature.feature_per_frame[8].ray_vector.y()+=100./600;
    feature.feature_per_frame[8].projection_outlier=true;
    backend::Optimizer optimizer(&window,&manager);optimizer.setExperimentalObservationPolicy(2);
    ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
    const std::vector<std::array<int,3>> expected={{0,1,0},{0,2,0},{0,4,0},{0,5,0},
                                                 {0,6,0},{0,7,0},{0,9,0},{0,10,0}};
    EXPECT_EQ(backend::OptimizerTestAccess::projectionPairs(optimizer,problem),expected);
    double raw_prior_residual=0;
    EXPECT_EQ(backend::OptimizerTestAccess::priorInputPairs(optimizer,&raw_prior_residual),expected);
    EXPECT_LT(raw_prior_residual,1e-20); // The 100px raw observations contribute neither rows nor error.
    EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),1);
    EXPECT_DOUBLE_EQ(backend::OptimizerTestAccess::depth(optimizer,0)[0],.2);
}

TEST(ObservationRejection, UnknownOrPendingDepthCannotBeRescuedByObservationFlag) {
    RestoreConfig restore;utility::g_config.camera.fx=400;
    for(bool pending:{false,true}) {
        SCOPED_TRACE(pending);
        backend::SlidingWindow window;frontend::FeatureManager manager;prepareScene(window,manager);
        auto& feature=manager.feature_bank_.back();
        feature.solve_flag=0;feature.geometry_pending=pending;feature.geometry_admitted=pending;
        feature.feature_per_frame.back().ray_vector.x()+=100./400;
        backend::Optimizer optimizer(&window,&manager);optimizer.setExperimentalObservationPolicy(2);
        ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
        EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),7);
        EXPECT_DOUBLE_EQ(backend::OptimizerTestAccess::depth(optimizer,7)[0],-1);
        EXPECT_FALSE(feature.feature_per_frame.back().projection_outlier);
    }
}

TEST(ObservationRejection, RestoredDepthMustRemainForwardInEveryGoodHistoricalView) {
    RestoreConfig restore;utility::g_config.camera.fx=400;utility::g_config.camera.fy=600;
    utility::g_config.camera.r_ic.setIdentity();utility::g_config.camera.t_ic.setZero();
    for(double historical_camera_z:{1.,3.}) {
        SCOPED_TRACE(historical_camera_z);
        backend::Estimator estimator;estimator.setParameter();estimator.solver_flag_=common::SolverFlag::NON_LINEAR;
        auto& window=estimator.sliding_window_;auto& manager=backend::EstimatorTestAccess::features(estimator);
        auto& optimizer=backend::EstimatorTestAccess::optimizer(estimator);prepareScene(estimator);
        // Optical-axis history has identical target bearings and cannot certify candidate depth.
        for(int frame=0;frame<=10;++frame)window[frame].P.setZero();
        for(auto& track:manager.feature_bank_)for(int frame=0;frame<=10;++frame) {
            const Eigen::Vector3d point=track.feature_id==7 ? Eigen::Vector3d(0,0,2) :
                Eigen::Vector3d(.2*(track.feature_id%4),.2*(track.feature_id/4),5);
            track.feature_per_frame[frame]=frontend::FeaturePerFrame(projectKnownPoint(point,window[frame].R,window[frame].P));
        }
        auto& feature=manager.feature_bank_.back();feature.estimated_depth=2;
        feature.feature_per_frame.back()=frontend::FeaturePerFrame(projectKnownPoint({0,0,5},window[10].R,window[10].P));
        feature.feature_per_frame.back().ray_vector.x()+=100./400;
        ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
        backend::OptimizerTestAccess::depth(optimizer,7)[0]=.2; // Candidate optical depth5m is forward in both cases.
        backend::OptimizerTestAccess::pose(optimizer,5)[2]=historical_camera_z;
        EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),7);
        // Restored 2m point has historical camera Z=+1m or -1m, respectively.
        const bool restored_forward=historical_camera_z<2;
        EXPECT_DOUBLE_EQ(backend::OptimizerTestAccess::depth(optimizer,7)[0],restored_forward ? .5 : -1);
        backend::OptimizerTestAccess::apply(optimizer);
        EXPECT_EQ(feature.solve_flag,restored_forward ? 1 : 2);
        EXPECT_DOUBLE_EQ(feature.estimated_depth,restored_forward ? 2 : -1);
        EXPECT_EQ(estimator.getSlidingWindowMapPoints().size(),restored_forward ? 8u : 7u);
        EXPECT_EQ(estimator.getSolvedFeatures().size(),7u);
    }
}

TEST(DepthRetention, ObservableGoodHistoryKeepsItsIndependentCandidateDepth) {
    RestoreConfig restore;utility::g_config.camera.fx=400;utility::g_config.camera.fy=600;
    utility::g_config.camera.r_ic.setIdentity();utility::g_config.camera.t_ic.setZero();
    backend::Estimator estimator;estimator.setParameter();estimator.solver_flag_=common::SolverFlag::NON_LINEAR;
    auto& window=estimator.sliding_window_;auto& manager=backend::EstimatorTestAccess::features(estimator);
    auto& optimizer=backend::EstimatorTestAccess::optimizer(estimator);prepareScene(estimator);
    const Eigen::Vector3d known_world(.6,.2,5);
    auto& feature=manager.feature_bank_.back();feature.estimated_depth=4;
    feature.feature_per_frame.back().ray_vector.x()+=100./400;
    EXPECT_GT((known_world-window[0].P).normalized().cross((known_world-window[9].P).normalized()).norm(),.1);
    ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
    auto* slot=backend::OptimizerTestAccess::depth(optimizer,7);slot[0]=.2;
    EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),7);
    EXPECT_EQ(backend::OptimizerTestAccess::depth(optimizer,7),slot);
    EXPECT_EQ(feature.feature_id,7);EXPECT_DOUBLE_EQ(slot[0],.2);
    EXPECT_TRUE(feature.feature_per_frame.back().projection_outlier);
    backend::OptimizerTestAccess::apply(optimizer);
    EXPECT_EQ(feature.solve_flag,1);EXPECT_DOUBLE_EQ(feature.estimated_depth,5);
    const auto map=estimator.getSlidingWindowMapPoints();ASSERT_EQ(map.size(),8u);
    EXPECT_LT((map[7]-known_world).norm(),1e-12);
    EXPECT_EQ(estimator.getSolvedFeatures().size(),7u);
    double raw_prior_residual=0;
    const auto pairs=backend::OptimizerTestAccess::priorInputPairs(optimizer,&raw_prior_residual);
    EXPECT_EQ(pairs.size(),79u);EXPECT_EQ(std::count(pairs.begin(),pairs.end(),std::array<int,3>{0,10,7}),0);
    EXPECT_LT(raw_prior_residual,1e-20);
}

TEST(DepthRetention, PureRotationAndOpticalAxisHistoryRestoreTheKnownOldDepth) {
    RestoreConfig restore;utility::g_config.camera.fx=400;utility::g_config.camera.fy=600;
    for(bool pure_rotation:{false,true}) {
        SCOPED_TRACE(pure_rotation);
        backend::SlidingWindow window;frontend::FeatureManager manager;prepareScene(window,manager);manager.clearState();
        for(int frame=0;frame<=10;++frame) {
            window[frame].P={0,0,pure_rotation ? 0 : .1*frame};
            window[frame].R=Eigen::AngleAxisd(pure_rotation ? .02*frame : 0,Eigen::Vector3d::UnitY()).toRotationMatrix();
            common::ImageData image;
            for(int id=0;id<8;++id) {
                const Eigen::Vector3d point=id==7 ? Eigen::Vector3d(0,0,5) : Eigen::Vector3d(.2*(id%4),.2*(id/4),5);
                image.emplace(id,projectKnownPoint(point,window[frame].R,window[frame].P));
            }
            manager.addFeatureAndCheckParallax(frame,image);
        }
        manager.setDepth(Eigen::VectorXd::Constant(8,.2));
        auto& feature=manager.feature_bank_.back();feature.estimated_depth=2;
        const Eigen::Vector3d first_bearing=window[0].R*feature.feature_per_frame.front().ray_vector.normalized();
        const Eigen::Vector3d last_good_bearing=window[9].R*feature.feature_per_frame[9].ray_vector.normalized();
        EXPECT_LT(first_bearing.cross(last_good_bearing).norm(),1e-14);
        feature.feature_per_frame.back().ray_vector.x()+=100./400;
        backend::Optimizer optimizer(&window,&manager);optimizer.setExperimentalObservationPolicy(2);
        ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
        backend::OptimizerTestAccess::depth(optimizer,7)[0]=.2;
        EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),7);
        EXPECT_DOUBLE_EQ(backend::OptimizerTestAccess::depth(optimizer,7)[0],.5);
        EXPECT_TRUE(feature.feature_per_frame.back().projection_outlier);
        backend::OptimizerTestAccess::apply(optimizer);
        EXPECT_EQ(feature.solve_flag,1);EXPECT_DOUBLE_EQ(feature.estimated_depth,2);
        double raw_prior_residual=0;
        const auto pairs=backend::OptimizerTestAccess::priorInputPairs(optimizer,&raw_prior_residual);
        EXPECT_EQ(pairs.size(),79u);EXPECT_EQ(std::count(pairs.begin(),pairs.end(),std::array<int,3>{0,10,7}),0);
        EXPECT_LT(raw_prior_residual,1e-20);
    }
}

TEST(DepthRetention, CandidateMismatchInObservableGoodHistoryRestoresOldDepth) {
    RestoreConfig restore;utility::g_config.camera.fx=400;utility::g_config.camera.fy=600;
    backend::SlidingWindow window;frontend::FeatureManager manager;prepareScene(window,manager);
    auto& feature=manager.feature_bank_.back();feature.feature_per_frame.back().ray_vector.x()+=100./400;
    // Independent frame9 projection: candidate4m gives -0.015; known5m point gives +0.012.
    EXPECT_GT(400*std::abs((.12*4-.54)/4-(.6-.54)/5),1.5);
    backend::Optimizer optimizer(&window,&manager);optimizer.setExperimentalObservationPolicy(2);
    ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
    backend::OptimizerTestAccess::depth(optimizer,7)[0]=.25;
    EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),7);
    EXPECT_DOUBLE_EQ(backend::OptimizerTestAccess::depth(optimizer,7)[0],.2);
    EXPECT_TRUE(feature.feature_per_frame.back().projection_outlier);
    double raw_prior_residual=0;
    const auto pairs=backend::OptimizerTestAccess::priorInputPairs(optimizer,&raw_prior_residual);
    EXPECT_EQ(pairs.size(),79u);EXPECT_EQ(std::count(pairs.begin(),pairs.end(),std::array<int,3>{0,10,7}),0);
    EXPECT_LT(raw_prior_residual,1e-20);
}

TEST(ObservationRejection, FlaggedAnchorCannotAdmitDepthMapOrProjectionFactors) {
    RestoreConfig restore;
    utility::g_config.camera.r_ic.setIdentity();utility::g_config.camera.t_ic.setZero();
    backend::Estimator estimator;estimator.setParameter();estimator.solver_flag_=common::SolverFlag::NON_LINEAR;
    auto& manager=backend::EstimatorTestAccess::features(estimator);
    auto& optimizer=backend::EstimatorTestAccess::optimizer(estimator);
    prepareScene(estimator);
    manager.feature_bank_.erase(std::next(manager.feature_bank_.begin()),manager.feature_bank_.end());
    manager.feature_bank_.front().feature_per_frame.front().projection_outlier=true;
    EXPECT_FALSE(manager.feature_bank_.front().isDepthAdmitted());
    EXPECT_TRUE(estimator.getSlidingWindowMapPoints().empty());
    EXPECT_TRUE(estimator.getSolvedFeatures().empty());
    ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
    EXPECT_TRUE(backend::OptimizerTestAccess::projectionPairs(optimizer,problem).empty());
    EXPECT_TRUE(backend::OptimizerTestAccess::priorInputPairs(optimizer).empty());
    manager.triangulateAcrossAllViews(estimator.sliding_window_,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
    EXPECT_EQ(manager.feature_bank_.front().solve_flag,2);
    EXPECT_DOUBLE_EQ(manager.feature_bank_.front().estimated_depth,-1);
}

TEST(ObservationRejection, OldAndNewSlidesInvalidateFlaggedAnchorsWithoutVirtualRays) {
    RestoreConfig restore;
    for(int operation:{0,1,2,3}) {
        SCOPED_TRACE(operation);
        backend::SlidingWindow window;frontend::FeatureManager manager;prepareScene(window,manager);
        manager.feature_bank_.erase(std::next(manager.feature_bank_.begin()),manager.feature_bank_.end());
        auto& feature=manager.feature_bank_.front();
        if(operation==2) {
            feature.feature_per_frame.erase(feature.feature_per_frame.begin(),feature.feature_per_frame.begin()+9);
            feature.start_frame=9;
            feature.feature_per_frame[1].projection_outlier=true;
            manager.removeFront(10,Eigen::Matrix3d::Identity(),window[9].P,Eigen::Matrix3d::Identity(),window[10].P);
        } else {
            feature.feature_per_frame[operation==3 ? 0 : 1].projection_outlier=true;
            if(operation==0)manager.removeBack();
            else manager.removeBackShiftDepth(Eigen::Matrix3d::Identity(),window[0].P,Eigen::Matrix3d::Identity(),window[1].P);
        }
        ASSERT_EQ(manager.feature_bank_.size(),1u);
        EXPECT_FALSE(feature.isDepthAdmitted());
        EXPECT_EQ(feature.solve_flag,2);
        EXPECT_DOUBLE_EQ(feature.estimated_depth,-1);
    }
}

TEST(ObservationRejection, PnPExportSkipsLatestFlagAfterOldAndNewSlideThenReadmitsNewRow) {
    RestoreConfig restore;
    utility::g_config.camera.r_ic.setIdentity();utility::g_config.camera.t_ic.setZero();
    for(bool old:{false,true}) {
        SCOPED_TRACE(old);
        backend::Estimator estimator;estimator.setParameter();estimator.solver_flag_=common::SolverFlag::NON_LINEAR;
        auto& window=estimator.sliding_window_;auto& manager=backend::EstimatorTestAccess::features(estimator);
        prepareScene(estimator);
        manager.feature_bank_.back().feature_per_frame.back().projection_outlier=true;
        if(old)backend::EstimatorTestAccess::slideOld(estimator);
        else backend::EstimatorTestAccess::slideNew(estimator);
        ASSERT_TRUE(manager.feature_bank_.back().feature_per_frame.back().projection_outlier);
        EXPECT_EQ(estimator.getSolvedFeatures().size(),7u);
        EXPECT_EQ(estimator.getSlidingWindowMapPoints().size(),8u);
        auto& optimizer=backend::EstimatorTestAccess::optimizer(estimator);
        ceres::Problem shifted_problem;backend::OptimizerTestAccess::problem(optimizer,shifted_problem);
        std::vector<std::array<int,3>> shifted_pairs;
        for(int slot=0;slot<8;++slot)for(int frame=1;frame<=9;++frame)
            if(slot!=7 || frame!=9)shifted_pairs.push_back({0,frame,slot});
        std::sort(shifted_pairs.begin(),shifted_pairs.end());
        EXPECT_EQ(backend::OptimizerTestAccess::projectionPairs(optimizer,shifted_problem),shifted_pairs);
        EXPECT_EQ(backend::OptimizerTestAccess::priorInputPairs(optimizer),shifted_pairs);
        window[10].timestamp=.66;window[10].P={.66,0,0};
        common::ImageData image;
        for(int id=0;id<8;++id)
            image.emplace(id,projectKnownPoint({.2*(id%4),.2*(id/4),5},window[10].R,window[10].P));
        manager.addFeatureAndCheckParallax(10,image);
        EXPECT_FALSE(manager.feature_bank_.back().feature_per_frame.back().projection_outlier);
        EXPECT_TRUE(manager.feature_bank_.back().feature_per_frame[manager.feature_bank_.back().feature_per_frame.size()-2].projection_outlier);
        EXPECT_EQ(estimator.getSolvedFeatures().size(),8u);
        EXPECT_EQ(estimator.getSlidingWindowMapPoints().size(),8u);
        ceres::Problem next_problem;backend::OptimizerTestAccess::problem(optimizer,next_problem);
        for(int slot=0;slot<8;++slot)shifted_pairs.push_back({0,10,slot});
        std::sort(shifted_pairs.begin(),shifted_pairs.end());
        EXPECT_EQ(backend::OptimizerTestAccess::projectionPairs(optimizer,next_problem),shifted_pairs);
        EXPECT_EQ(backend::OptimizerTestAccess::priorInputPairs(optimizer),shifted_pairs);
    }
}

TEST(ObservationRejection, FlaggedAndInactiveTracksKeepTheStructuralThousandSlotPrefix) {
    RestoreConfig restore;
    for(int count:{999,1000,1001}) {
        SCOPED_TRACE(count);
        backend::SlidingWindow window;frontend::FeatureManager manager;
        window[10].P={.2,0,0};
        for(int id=0;id<count;++id) {
            manager.feature_bank_.emplace_back(id,9);auto& feature=manager.feature_bank_.back();
            for(int frame:{9,10})feature.feature_per_frame.emplace_back(projectKnownPoint({0,0,5},window[frame].R,window[frame].P));
            feature.estimated_depth=5;feature.solve_flag=1;
            if(id==0)feature.feature_per_frame.front().projection_outlier=true;
            if(id==1)feature.feature_per_frame.back().projection_outlier=true;
            if(id==2) {feature.solve_flag=0;feature.geometry_pending=true;feature.geometry_admitted=false;}
        }
        const int prefix=std::min(count,utility::NUM_OF_FEATURES);
        backend::Optimizer optimizer(&window,&manager);optimizer.setExperimentalObservationPolicy(2);
        ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
        std::vector<std::array<int,3>> expected;
        for(int slot=3;slot<prefix;++slot)expected.push_back({9,10,slot});
        EXPECT_EQ(backend::OptimizerTestAccess::projectionPairs(optimizer,problem),expected);
        EXPECT_EQ(manager.getFeatureCount(),prefix);
        EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),prefix-3);
        EXPECT_DOUBLE_EQ(backend::OptimizerTestAccess::depth(optimizer,1)[0],.2);
        EXPECT_EQ(manager.feature_bank_.back().feature_id,count-1);
    }
}

TEST(ExperimentalObservationPolicy, LegacyDefaultAndExplicitPoliciesHaveDistinctCurrentSupport) {
    RestoreConfig restore;utility::g_config.camera.fx=400;utility::g_config.camera.fy=600;
    for(int policy:{0,1,2}) {
        SCOPED_TRACE(policy);
        backend::SlidingWindow window;frontend::FeatureManager manager;prepareScene(window,manager);
        manager.feature_bank_.back().feature_per_frame.back().ray_vector.x()+=100./400;
        backend::Optimizer optimizer(&window,&manager);
        EXPECT_EQ(optimizer.getExperimentalObservationPolicy(),0);
        optimizer.setExperimentalObservationPolicy(policy);
        EXPECT_EQ(optimizer.getExperimentalObservationPolicy(),policy);
        EXPECT_THROW(optimizer.setExperimentalObservationPolicy(-1),std::invalid_argument);
        EXPECT_THROW(optimizer.setExperimentalObservationPolicy(3),std::invalid_argument);
        EXPECT_EQ(optimizer.getExperimentalObservationPolicy(),policy);
        ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
        EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),policy==0 ? 8 : 7);
        EXPECT_EQ(manager.feature_bank_.back().feature_per_frame.back().projection_outlier,policy!=0);
        EXPECT_DOUBLE_EQ(backend::OptimizerTestAccess::depth(optimizer,7)[0],.2);
        optimizer.reset();EXPECT_EQ(optimizer.getExperimentalObservationPolicy(),policy);
    }
}

TEST(ExperimentalObservationPolicy, OldRestoreAndGoodHistoryRetentionRemainExplicitlyDifferent) {
    RestoreConfig restore;utility::g_config.camera.fx=400;utility::g_config.camera.fy=600;
    for(int policy:{1,2}) {
        SCOPED_TRACE(policy);
        backend::SlidingWindow window;frontend::FeatureManager manager;prepareScene(window,manager);
        auto& feature=manager.feature_bank_.back();feature.estimated_depth=4;
        feature.feature_per_frame.back().ray_vector.x()+=100./400;
        backend::Optimizer optimizer(&window,&manager);optimizer.setExperimentalObservationPolicy(policy);
        ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
        backend::OptimizerTestAccess::depth(optimizer,7)[0]=.2;
        EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),7);
        EXPECT_DOUBLE_EQ(backend::OptimizerTestAccess::depth(optimizer,7)[0],policy==1 ? .25 : .2);
        EXPECT_TRUE(feature.feature_per_frame.back().projection_outlier);
    }
}

namespace {
void addDepthConeTrack(backend::SlidingWindow& window,frontend::FeatureManager& manager,
                      const std::vector<Eigen::Vector3d>& centers,const std::vector<Eigen::Vector3d>& rays,
                      int start=8,int id=42,const std::vector<int>& rejected={}) {
    manager.feature_bank_.emplace_back(id,start);
    auto& feature=manager.feature_bank_.back();feature.geometry_pending=true;
    for(size_t i=0;i<rays.size();++i) {
        const int frame=start+static_cast<int>(i);
        window[frame].P=centers[i];window[frame].R.setIdentity();
        Eigen::Matrix<double,7,1> value;value<<rays[i],37,53,.07,-.09;
        feature.feature_per_frame.emplace_back(value);
        feature.feature_per_frame.back().projection_outlier=std::find(rejected.begin(),rejected.end(),frame)!=rejected.end();
    }
}
std::vector<Eigen::Vector3d> depthConeCenters() {return {{0,0,0},{.4,0,0},{.8,0,0}};}
std::vector<Eigen::Vector3d> depthConeRays() {return {{0,0,1},{-47.4/460,0,1},{-91./460,0,1}};}
#if MOBILE_SLAM_EXPERIMENTAL_ANCHOR_DEPTH
double depthConeExpected(double scale=1) {
    // Independent pixel inequalities: |184/d-47.4|<=1.5 and |368/d-91|<=1.5.
    return scale*.5*(368./92.5+184./45.9);
}
bool depthConeConsistent(const frontend::FeaturePerId& feature,const backend::SlidingWindow& window,
                         const Eigen::Matrix3d& ric=Eigen::Matrix3d::Identity(),
                         const Eigen::Vector3d& tic=Eigen::Vector3d::Zero()) {
    std::array<Eigen::Matrix3d,11> rotations;std::array<Eigen::Vector3d,11> centers;
    for(int frame=0;frame<=10;++frame) {rotations[frame]=window[frame].R*ric;centers[frame]=window[frame].P+window[frame].R*tic;}
    return frontend::FeatureManager::hasConsistentReprojection(feature,feature.estimated_depth,rotations,centers);
}
#endif
}

#if MOBILE_SLAM_EXPERIMENTAL_ANCHOR_DEPTH
TEST(DepthConeFallback, PreservesAlreadyPassingDltAndSolvedLegacyDepth) {
    RestoreConfig restore;utility::g_config.camera.fx=utility::g_config.camera.fy=460;
    for(bool two_views:{false,true}) {
        backend::SlidingWindow window;frontend::FeatureManager manager;
        if(two_views)addDepthConeTrack(window,manager,{{0,0,0},{.4,0,0}},{{0,0,1},{-.1,1.4/460,1}},9);
        else addDepthConeTrack(window,manager,depthConeCenters(),{{0,0,1},{-.1+.5/460,0,1},{-.2-.5/460,0,1}});
        manager.triangulateAcrossAllViews(window,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
        const auto& feature=manager.feature_bank_.front();
        ASSERT_TRUE(feature.geometry_admitted);EXPECT_TRUE(feature.geometry_pending);EXPECT_EQ(feature.solve_flag,0);
        // Frozen independent all-view DLT values, not the feasible-interval midpoint.
        EXPECT_NEAR(feature.estimated_depth,two_views ? 3.9997820534062396 : 3.978786633534431,1e-11);
        EXPECT_TRUE(depthConeConsistent(feature,window));
    }
    backend::SlidingWindow window;frontend::FeatureManager manager;
    addDepthConeTrack(window,manager,depthConeCenters(),depthConeRays());
    auto& legacy=manager.feature_bank_.front();legacy.geometry_pending=false;legacy.solve_flag=1;legacy.estimated_depth=7;
    manager.triangulateAcrossAllViews(window,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
    EXPECT_DOUBLE_EQ(legacy.estimated_depth,7);EXPECT_EQ(legacy.solve_flag,1);EXPECT_FALSE(legacy.geometry_pending);
}

TEST(DepthConeFallback, RecoversOnlyFeasibleAnchorDepthWithoutPrematureMaturity) {
    RestoreConfig restore;utility::g_config.camera.fx=utility::g_config.camera.fy=460;
    for(double scale:{1.,2.3}) {
        backend::SlidingWindow window;frontend::FeatureManager manager;
        addDepthConeTrack(window,manager,depthConeCenters(),depthConeRays());
        const Eigen::Matrix3d ric=Eigen::AngleAxisd(.2,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
        const Eigen::Matrix3d gauge=Eigen::AngleAxisd(-.3,Eigen::Vector3d(2,-1,3).normalized()).toRotationMatrix();
        const Eigen::Vector3d tic(.03,-.02,.07),translation(1,-2,3);
        for(int frame=8;frame<=10;++frame) {
            const Eigen::Vector3d camera=gauge*(scale*window[frame].P)+translation;
            window[frame].R=gauge*ric.transpose();window[frame].P=camera-window[frame].R*tic;
        }
        manager.triangulateAcrossAllViews(window,tic,ric);
        const auto& feature=manager.feature_bank_.front();
        ASSERT_TRUE(feature.geometry_admitted);EXPECT_TRUE(feature.geometry_pending);EXPECT_EQ(feature.solve_flag,0);
        EXPECT_NEAR(feature.estimated_depth,depthConeExpected(scale),1e-10);
        EXPECT_STREQ(feature.geometry_reason,"accepted");EXPECT_EQ(feature.feature_id,42);EXPECT_EQ(feature.start_frame,8);
        EXPECT_TRUE(depthConeConsistent(feature,window,ric,tic));
    }
}

TEST(DepthConeFallback, IntersectsLinearAndNegativeLeadingCoefficientViews) {
    RestoreConfig restore;utility::g_config.camera.fx=utility::g_config.camera.fy=460;
    backend::SlidingWindow window;frontend::FeatureManager manager;
    // At frame7 q has negative d^2 coefficient; frame8 has exactly zero d^2
    // coefficient. Their lower bounds are below the same bounded 3-view interval.
    addDepthConeTrack(window,manager,{{0,0,0},{.004,0,0},{.01,0,0},{.4,0,0},{.8,0,0}},
        {{0,0,1},{0,0,1},{-1.5/460,0,1},{-47.4/460,0,1},{-91./460,0,1}},6);
    manager.triangulateAcrossAllViews(window,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
    const auto& feature=manager.feature_bank_.front();
    ASSERT_TRUE(feature.geometry_admitted);EXPECT_NEAR(feature.estimated_depth,depthConeExpected(),1e-10);
    EXPECT_TRUE(depthConeConsistent(feature,window));EXPECT_TRUE(feature.geometry_pending);EXPECT_EQ(feature.solve_flag,0);
}

TEST(DepthConeFallback, FlaggedRowsKeepTheirActualCameraFrameIndices) {
    RestoreConfig restore;utility::g_config.camera.fx=utility::g_config.camera.fy=460;
    backend::SlidingWindow window;frontend::FeatureManager manager;
    addDepthConeTrack(window,manager,{{0,0,0},{20,-9,8},{.4,0,0},{-8,40,-2},{.8,0,0}},
        {{0,0,1},{10,-4,1},{-47.4/460,0,1},{-5,8,1},{-91./460,0,1}},6,42,{7,9});
    manager.triangulateAcrossAllViews(window,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
    const auto& feature=manager.feature_bank_.front();
    ASSERT_TRUE(feature.geometry_admitted);EXPECT_NEAR(feature.estimated_depth,depthConeExpected(),1e-10);
    ASSERT_EQ(feature.feature_per_frame.size(),5u);EXPECT_EQ(feature.start_frame,6);
    EXPECT_TRUE(feature.feature_per_frame[1].projection_outlier);EXPECT_TRUE(feature.feature_per_frame[3].projection_outlier);
    EXPECT_DOUBLE_EQ(feature.feature_per_frame[1].ray_vector.x(),10);EXPECT_TRUE(depthConeConsistent(feature,window));
}

TEST(DepthConeFallback, ContradictoryViewsAndBehindCameraDoNotInventDepth) {
    RestoreConfig restore;utility::g_config.camera.fx=utility::g_config.camera.fy=460;
    for(bool behind:{false,true}) {
        backend::SlidingWindow window;frontend::FeatureManager manager;
        if(behind)addDepthConeTrack(window,manager,{{0,0,0},{.4,0,0},{0,0,5}},{{0,0,1},{-.1,0,1},{0,0,1}});
        else addDepthConeTrack(window,manager,{{0,0,0},{.4,0,0},{.4,0,0}},{{0,0,1},{-.1,0,1},{-.2,0,1}});
        manager.triangulateAcrossAllViews(window,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
        const auto& feature=manager.feature_bank_.front();
        EXPECT_FALSE(feature.geometry_admitted);EXPECT_TRUE(feature.geometry_pending);
        EXPECT_DOUBLE_EQ(feature.estimated_depth,-1);EXPECT_EQ(feature.solve_flag,0);
    }
}

TEST(DepthConeFallback, PureRotationStillRejectsPositivePendingDepth) {
    RestoreConfig restore;utility::g_config.camera.fx=utility::g_config.camera.fy=460;
    backend::SlidingWindow window;frontend::FeatureManager manager;
    addDepthConeTrack(window,manager,{{0,0,0},{0,0,0}},{{0,0,1},{-std::tan(.3),0,1}},9);
    window[10].R=Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitY()).toRotationMatrix();
    auto& feature=manager.feature_bank_.front();feature.estimated_depth=4;feature.geometry_admitted=true;
    manager.triangulateAcrossAllViews(window,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
    EXPECT_FALSE(feature.geometry_admitted);EXPECT_DOUBLE_EQ(feature.estimated_depth,-1);EXPECT_EQ(feature.solve_flag,0);
    EXPECT_TRUE(feature.geometry_pending);EXPECT_STREQ(feature.geometry_reason,"rank_or_condition");
}

TEST(DepthConeFallback, PreservesTheStructuralThousandSlotPrefix) {
    RestoreConfig restore;utility::g_config.camera.fx=utility::g_config.camera.fy=460;
    backend::SlidingWindow window;frontend::FeatureManager manager;
    for(int id=0;id<=utility::NUM_OF_FEATURES;++id)addDepthConeTrack(window,manager,depthConeCenters(),depthConeRays(),8,id);
    manager.triangulateAcrossAllViews(window,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
    ASSERT_EQ(manager.feature_bank_.size(),static_cast<size_t>(utility::NUM_OF_FEATURES+1));
    int expected_id=0;
    for(const auto& feature:manager.feature_bank_) {
        ASSERT_EQ(feature.feature_id,expected_id++);EXPECT_TRUE(feature.geometry_pending);EXPECT_EQ(feature.solve_flag,0);
        if(feature.feature_id<utility::NUM_OF_FEATURES) {
            ASSERT_TRUE(feature.geometry_admitted);EXPECT_NEAR(feature.estimated_depth,depthConeExpected(),1e-10);
        } else {EXPECT_FALSE(feature.geometry_admitted);EXPECT_DOUBLE_EQ(feature.estimated_depth,-1);}
    }
    EXPECT_EQ(manager.getFeatureCount(),utility::NUM_OF_FEATURES);
}

TEST(DepthConeFallback, OptimizerFinalGeometryStillRejectsAnInconsistentCandidate) {
    RestoreConfig restore;utility::g_config.camera.fx=utility::g_config.camera.fy=460;
    utility::g_config.camera.r_ic.setIdentity();utility::g_config.camera.t_ic.setZero();
    backend::SlidingWindow window;frontend::FeatureManager manager;
    addDepthConeTrack(window,manager,depthConeCenters(),depthConeRays());
    manager.triangulateAcrossAllViews(window,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
    auto& feature=manager.feature_bank_.front();ASSERT_TRUE(feature.geometry_admitted);
    backend::Optimizer optimizer(&window,&manager);
    ceres::Problem problem;backend::OptimizerTestAccess::problem(optimizer,problem);
    EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),1);
    backend::OptimizerTestAccess::pose(optimizer,10)[0]+=.02;
    EXPECT_EQ(backend::OptimizerTestAccess::filter(optimizer),0);
    EXPECT_DOUBLE_EQ(backend::OptimizerTestAccess::depth(optimizer,0)[0],-1);
    EXPECT_TRUE(feature.geometry_pending);EXPECT_EQ(feature.solve_flag,0);
    EXPECT_NEAR(feature.estimated_depth,depthConeExpected(),1e-10);
}
#else
TEST(DepthConeFallbackDefault, FeasibleDepthDoesNotEnableExperimentalRecovery) {
    RestoreConfig restore;utility::g_config.camera.fx=utility::g_config.camera.fy=460;
    // A 4m anchor depth is feasible, but the unchanged DLT depth has 1.886px
    // maximum reprojection error. Default builds must preserve its rejection.
    EXPECT_NEAR(std::abs(184./4-47.4),1.4,1e-12);
    EXPECT_DOUBLE_EQ(std::abs(368./4-91),1);
    EXPECT_NEAR(std::abs(184./4.04273377156683-47.4),1.8862436171037622,1e-12);
    backend::SlidingWindow window;frontend::FeatureManager manager;
    addDepthConeTrack(window,manager,depthConeCenters(),depthConeRays());
    manager.triangulateAcrossAllViews(window,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
    const auto& feature=manager.feature_bank_.front();
    EXPECT_FALSE(feature.geometry_admitted);EXPECT_TRUE(feature.geometry_pending);
    EXPECT_DOUBLE_EQ(feature.estimated_depth,-1);EXPECT_EQ(feature.solve_flag,0);
    EXPECT_STREQ(feature.geometry_reason,"cheirality_or_reprojection");
}
#endif
