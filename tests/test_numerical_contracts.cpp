#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <limits>
#include <new>

#include "backend/estimator.h"
#include "backend/factor/imu_factor_pnp.h"
#include "backend/factor/perspective_factor.h"
#include "frontend/pnp_frontend.h"

namespace frontend::initialization {
struct InitializerTestAccess {
    static bool align(Initializer& initializer) { return initializer.visualInitialAlign(); }
};
void RefineGravity(const std::map<double, common::ImageFrame>&, Eigen::Vector3d&, Eigen::VectorXd&);
}

namespace {
using Matrix15 = Eigen::Matrix<double, 15, 15>;
using Vector15 = Eigen::Matrix<double, 15, 1>;
const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
const Eigen::Vector3d gravity(0, 0, 9.81);

backend::factor::IntegrationBase integration(int count) {
    backend::factor::IntegrationBase value(gravity, zero, zero, zero);
    for (int i = 0; i < count; ++i) value.push_back(0.01, gravity, zero);
    return value;
}

bool evaluateImu(backend::factor::IntegrationBase& value, Vector15* residual = nullptr) {
    backend::factor::IMUFactor factor(&value);
    double pose_i[7] = {0, 0, 0, 0, 0, 0, 1};
    double pose_j[7] = {0.2, -0.1, 0.4, 0, 0, 0, 1};
    double speed_i[9] = {};
    double speed_j[9] = {};
    double const* parameters[] = {pose_i, speed_i, pose_j, speed_j};
    Vector15 result;
    const bool ok = factor.Evaluate(parameters, result.data(), nullptr);
    if (residual) *residual = result;
    return ok;
}

TEST(NumericalContracts, ImuRejectsZeroOneSampleAndSingularCovariance) {
    for (int count : {0, 1}) {
        auto value = integration(count);
        EXPECT_FALSE(evaluateImu(value)) << "count=" << count;
    }
    auto value = integration(6);
    value.covariance.setZero();
    EXPECT_FALSE(evaluateImu(value));
    value.covariance.setIdentity();
    value.covariance(0, 0) = -1;
    EXPECT_FALSE(evaluateImu(value));
    value.covariance.setIdentity();
    value.covariance(0, 1) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(evaluateImu(value));
    value.covariance.setIdentity();
    value.covariance(0, 1) = 0.1;  // Asymmetric, rather than a covariance.
    EXPECT_FALSE(evaluateImu(value));
    value.covariance.setIdentity();
    value.sum_dt += 0.01;  // Sample count is sufficient, interval is uncovered.
    EXPECT_FALSE(evaluateImu(value));
}

TEST(NumericalContracts, PnpImuRejectsUncoveredAndRankDeficientIntervals) {
    double pose[7] = {0, 0, 0, 0, 0, 0, 1};
    double speed[3] = {};
    double bias[6] = {};
    double const* parameters[] = {pose, speed, bias, pose, speed, bias};
    Vector15 residual;
    for (int count : {0, 1}) {
        auto value = integration(count);
        backend::factor::IMUFactorPnP factor(&value);
        EXPECT_FALSE(factor.isValid());
        EXPECT_FALSE(factor.Evaluate(parameters, residual.data(), nullptr));
    }
    auto value = integration(6);
    value.covariance.setZero();
    backend::factor::IMUFactorPnP factor(&value);
    EXPECT_FALSE(factor.isValid());
}

TEST(NumericalContracts, ImuWhiteningPreservesIndependentMahalanobisMetric) {
    auto value = integration(6);
    Matrix15 lower = Matrix15::Identity();
    for (int i = 0; i < 15; ++i) {
        lower(i, i) = 0.3 + i * 0.11;
        if (i > 0) lower(i, i - 1) = 0.17;
    }
    value.covariance = lower * lower.transpose();
    Vector15 actual;
    ASSERT_TRUE(evaluateImu(value, &actual));
    const Vector15 raw = value.evaluate(zero, Eigen::Quaterniond::Identity(), zero, zero, zero,
                                        Eigen::Vector3d(0.2, -0.1, 0.4), Eigen::Quaterniond::Identity(), zero, zero, zero);
    // Known lower matrix is independent of the production factorization.
    const Vector15 expected = lower.triangularView<Eigen::Lower>().solve(raw);
    EXPECT_NEAR(actual.squaredNorm(), expected.squaredNorm(), 1e-12);
    EXPECT_TRUE(actual.allFinite());
}

TEST(NumericalContracts, DepthRejectsZeroNegativeAndNonfiniteInverseDepth) {
    for (double depth : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
        frontend::FeatureManager manager;
        Eigen::Matrix<double, 7, 1> observation;
        observation << 0.2, 0.1, 1, 20, 10, 0, 0;
        manager.addFeatureAndCheckParallax(0, {{0, observation}});
        manager.addFeatureAndCheckParallax(1, {{0, observation}});
        manager.setDepth(Eigen::VectorXd::Constant(1, depth));
        ASSERT_EQ(manager.feature_bank_.size(), 1);
        EXPECT_EQ(manager.feature_bank_.front().solve_flag, 2);
        EXPECT_FALSE(std::isfinite(manager.feature_bank_.front().estimated_depth) &&
                     manager.feature_bank_.front().estimated_depth > 0);
    }
}

TEST(NumericalContracts, ProjectionRejectsInvalidGeometryAndPreservesValidGolden) {
    backend::factor::ProjectionFactor::sqrt_info.setIdentity();
    backend::factor::ProjectionFactor factor(Eigen::Vector3d(0.2, 0.1, 1), Eigen::Vector3d(0.1, 0.1, 1));
    double pose_i[7] = {0, 0, 0, 0, 0, 0, 1};
    double pose_j[7] = {0.5, 0, 0, 0, 0, 0, 1};
    double extrinsic[7] = {0, 0, 0, 0, 0, 0, 1};
    double inverse_depth = 0.2;
    double const* parameters[] = {pose_i, pose_j, extrinsic, &inverse_depth};
    double residual[2];
    ASSERT_TRUE(factor.Evaluate(parameters, residual, nullptr));
    EXPECT_NEAR(residual[0], 0, 1e-14);
    EXPECT_NEAR(residual[1], 0, 1e-14);
    for (double bad : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
        inverse_depth = bad;
        EXPECT_FALSE(factor.Evaluate(parameters, residual, nullptr));
    }
    inverse_depth = 0.2;
    pose_j[2] = 5;  // Zero target optical Z.
    EXPECT_FALSE(factor.Evaluate(parameters, residual, nullptr));
    pose_j[2] = 6;
    EXPECT_FALSE(factor.Evaluate(parameters, residual, nullptr));
}

TEST(NumericalContracts, PerspectiveRejectsZeroBehindAndNonfinitePoints) {
    backend::factor::PerspectiveFactor::sqrt_info.setIdentity();
    double identity[7] = {0, 0, 0, 0, 0, 0, 1};
    double const* parameters[] = {identity, identity};
    double residual[2];
    for (const auto& point : {Eigen::Vector3d(1, 2, 0), Eigen::Vector3d(1, 2, -1),
                              Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0, 1)}) {
        backend::factor::PerspectiveFactor factor(Eigen::Vector2d::Zero(), point, 10);
        EXPECT_FALSE(factor.Evaluate(parameters, residual, nullptr));
    }
    backend::factor::PerspectiveFactor valid(Eigen::Vector2d(0.2, 0.4), Eigen::Vector3d(1, 2, 5), 10);
    ASSERT_TRUE(valid.Evaluate(parameters, residual, nullptr));
    EXPECT_NEAR(residual[0], 0, 1e-14);
    EXPECT_NEAR(residual[1], 0, 1e-14);
}

void checkLocalPoseJacobian(ceres::CostFunction& factor, std::vector<double*>& parameters,
                            const std::vector<int>& pose_blocks, double tolerance) {
    const int rows = factor.num_residuals();
    std::vector<double> residual(rows);
    std::vector<std::vector<double>> storage;
    std::vector<double*> jacobians;
    for (int size : factor.parameter_block_sizes()) storage.emplace_back(rows * size);
    for (auto& values : storage) jacobians.push_back(values.data());
    ASSERT_TRUE(factor.Evaluate(parameters.data(), residual.data(), jacobians.data()));
    backend::factor::PoseLocalParameterization manifold;
    constexpr double epsilon = 1e-7;
    for (int block : pose_blocks) {
        double plus_jacobian[42];
        ASSERT_TRUE(manifold.PlusJacobian(parameters[block], plus_jacobian));
        const Eigen::MatrixXd effective =
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 7, Eigen::RowMajor>>(jacobians[block], rows, 7) *
            Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>>(plus_jacobian);
        Eigen::MatrixXd numeric(rows, 6);
        double original[7];
        std::copy(parameters[block], parameters[block] + 7, original);
        for (int axis = 0; axis < 6; ++axis) {
            double delta[6] = {};
            delta[axis] = epsilon;
            manifold.Plus(original, delta, parameters[block]);
            Eigen::VectorXd positive(rows), negative(rows);
            ASSERT_TRUE(factor.Evaluate(parameters.data(), positive.data(), nullptr));
            delta[axis] = -epsilon;
            manifold.Plus(original, delta, parameters[block]);
            ASSERT_TRUE(factor.Evaluate(parameters.data(), negative.data(), nullptr));
            numeric.col(axis) = (positive - negative) / (2 * epsilon);
        }
        std::copy(original, original + 7, parameters[block]);
        EXPECT_LT((effective - numeric).cwiseAbs().maxCoeff(), tolerance) << "pose block=" << block;
    }
}

void fillPose(double* pose, const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation) {
    Eigen::Map<Eigen::Vector3d> p(pose);
    Eigen::Map<Eigen::Quaterniond> q(pose + 3);
    p = translation;
    q = rotation;
}

TEST(NumericalContracts, LegacyProjectionEffectiveLocalJacobianMatchesFiniteDifference) {
    backend::factor::ProjectionFactor::sqrt_info.setIdentity();
    backend::factor::PerspectiveFactor::sqrt_info.setIdentity();
    double a[7], b[7], ex[7];
    fillPose(a, Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Quaterniond(Eigen::AngleAxisd(0.25, Eigen::Vector3d(1, 2, 3).normalized())));
    fillPose(b, Eigen::Vector3d(0.6, -0.1, 0.2), Eigen::Quaterniond(Eigen::AngleAxisd(-0.13, Eigen::Vector3d(2, -1, 3).normalized())));
    fillPose(ex, Eigen::Vector3d(0.03, -0.06, 0.02), Eigen::Quaterniond(Eigen::AngleAxisd(0.14, Eigen::Vector3d(-1, 2, 1).normalized())));
    double inverse_depth = 0.2;
    backend::factor::ProjectionFactor projection(Eigen::Vector3d(0.2, 0.1, 1), Eigen::Vector3d(0.1, 0.1, 1));
    std::vector<double*> projection_parameters = {a, b, ex, &inverse_depth};
    checkLocalPoseJacobian(projection, projection_parameters, {0, 1, 2}, 2e-8);
    backend::factor::PerspectiveFactor perspective(Eigen::Vector2d(0.2, 0.1), Eigen::Vector3d(1.1, 0.8, 5.4), 10);
    std::vector<double*> perspective_parameters = {a, ex};
    checkLocalPoseJacobian(perspective, perspective_parameters, {0, 1}, 2e-8);
}

TEST(NumericalContracts, LegacyImuEffectiveLocalJacobianMatchesFiniteDifference) {
    auto value = integration(6);
    value.covariance.setIdentity();
    double a[7], b[7];
    const Eigen::Quaterniond q(Eigen::AngleAxisd(0.25, Eigen::Vector3d(1, 2, 3).normalized()));
    fillPose(a, Eigen::Vector3d(0.1, -0.2, 0.3), q);
    fillPose(b, Eigen::Vector3d(0.3, -0.1, 0.4), q * Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY())));
    double va[9] = {0.2, 0.1, -0.3};
    double vb[9] = {0.3, -0.2, -0.1};
    backend::factor::IMUFactor factor(&value);
    std::vector<double*> parameters = {a, va, b, vb};
    checkLocalPoseJacobian(factor, parameters, {0, 2}, 2e-8);
    double speed_a[3] = {0.2, 0.1, -0.3}, speed_b[3] = {0.3, -0.2, -0.1};
    double bias_a[6] = {}, bias_b[6] = {};
    backend::factor::IMUFactorPnP pnp_factor(&value);
    std::vector<double*> pnp_parameters = {a, speed_a, bias_a, b, speed_b, bias_b};
    checkLocalPoseJacobian(pnp_factor, pnp_parameters, {0, 3}, 2e-8);
}

void checkEuclideanBlockJacobian(ceres::CostFunction& factor, std::vector<double*>& parameters,
                                 int block, double tolerance) {
    const int rows = factor.num_residuals();
    const int columns = factor.parameter_block_sizes()[block];
    std::vector<double> residual(rows), storage(rows * columns);
    std::vector<double*> jacobians(parameters.size(), nullptr);
    jacobians[block] = storage.data();
    ASSERT_TRUE(factor.Evaluate(parameters.data(), residual.data(), jacobians.data()));
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> analytic(
        storage.data(), rows, columns);
    const double step = 1e-7;
    for (int column = 0; column < columns; ++column) {
        std::vector<double> plus(parameters[block], parameters[block] + columns), minus = plus;
        plus[column] += step;
        minus[column] -= step;
        auto plus_parameters = parameters, minus_parameters = parameters;
        plus_parameters[block] = plus.data();
        minus_parameters[block] = minus.data();
        Eigen::VectorXd r_plus(rows), r_minus(rows);
        ASSERT_TRUE(factor.Evaluate(plus_parameters.data(), r_plus.data(), nullptr));
        ASSERT_TRUE(factor.Evaluate(minus_parameters.data(), r_minus.data(), nullptr));
        const Eigen::VectorXd numeric = (r_plus - r_minus) / (2 * step);
        EXPECT_LT((analytic.col(column) - numeric).cwiseAbs().maxCoeff(), tolerance)
            << "Euclidean block=" << block << " column=" << column;
    }
}

void checkImuQuaternionRepresentatives(bool verify_local_jacobians) {
    struct RestoreGravity {
        Eigen::Vector3d saved = utility::g_config.estimator.g;
        ~RestoreGravity() { utility::g_config.estimator.g = saved; }
    } restore;
    const Eigen::Vector3d known_gravity(0, 0, 9.81007);
    utility::g_config.estimator.g = known_gravity;
    const Eigen::Vector3d ba(0.01, -0.02, 0.03), bg(0.001, -0.002, 0.003);
    backend::factor::IntegrationBase preintegration(known_gravity + ba, bg, ba, bg);
    for (int sample = 0; sample < 6; ++sample) preintegration.push_back(0.01, known_gravity + ba, bg);
    const Eigen::Quaterniond non_pi_qi(std::sqrt(0.75), 0, 0, 0.5);
    const Eigen::Vector3d pi(0.3, -0.4, 0.6), vi(0.2, -0.1, 0.05);
    struct OrientationCase { Eigen::Vector3d imaginary; int correlation_axis; double expected_cost; };
    const OrientationCase cases[] = {{{0.1,0,0},0,1.18}, {{1,0,0},0,3.34},
                                    {{0,1,0},1,3.34}, {{0,0,1},2,3.34}};
    // Known P_x=1, V_y=-0.3, R_axis=0.2 (or2 at pi), with W r_Raxis=r_Raxis-0.5 r_Px.
    // The pi cases declare a positive-axis tie; the chart is nondifferentiable there, so FD excludes pi.
    for (int index = 0; index < (verify_local_jacobians ? 1 : 4); ++index) {
      const auto& test_case = cases[index];
      SCOPED_TRACE(::testing::Message() << "orientation case=" << index);
      // Identity source rotation makes the pi input exactly w=0 in binary,
      // rather than a near-pi rounded product of two nonidentity quaternions.
      const Eigen::Quaterniond qi = index ? Eigen::Quaterniond::Identity() : non_pi_qi;
      const Eigen::Vector3d pj = pi + 0.06 * vi + qi * Eigen::Vector3d(1, 0, 0);
      const Eigen::Vector3d vj = vi + qi * Eigen::Vector3d(0, -0.3, 0);
      Matrix15 lower = Matrix15::Identity();
      lower(O_R + test_case.correlation_axis, O_P) = 0.5;
      preintegration.covariance = lower * lower.transpose();
      backend::factor::IMUFactor imu(&preintegration);
      backend::factor::IMUFactorPnP pnp(&preintegration);
      ASSERT_TRUE(imu.isValid());
      ASSERT_TRUE(pnp.isValid());
      const Eigen::Quaterniond qj = qi * Eigen::Quaterniond(std::sqrt(1 - test_case.imaginary.squaredNorm()),
          test_case.imaginary.x(), test_case.imaginary.y(), test_case.imaginary.z());
      if (index) ASSERT_DOUBLE_EQ((qi.inverse() * qj).w(), 0);
      for (int sign_i : {1, -1}) for (int sign_j : {1, -1}) {
        SCOPED_TRACE(::testing::Message() << "sign_i=" << sign_i << " sign_j=" << sign_j);
        auto representative_i = qi;
        auto representative_j = qj;
        representative_i.coeffs() *= sign_i;
        representative_j.coeffs() *= sign_j;
        EXPECT_LT((representative_i.toRotationMatrix() - qi.toRotationMatrix()).norm(), 1e-14);
        EXPECT_LT((representative_j.toRotationMatrix() - qj.toRotationMatrix()).norm(), 1e-14);
        double a[7], b[7];
        fillPose(a, pi, representative_i);
        fillPose(b, pj, representative_j);
        double speedbias_i[9] = {vi.x(), vi.y(), vi.z(), ba.x(), ba.y(), ba.z(), bg.x(), bg.y(), bg.z()};
        double speedbias_j[9] = {vj.x(), vj.y(), vj.z(), ba.x(), ba.y(), ba.z(), bg.x(), bg.y(), bg.z()};
        std::vector<double*> imu_parameters = {a, speedbias_i, b, speedbias_j};
        std::vector<double*> pnp_parameters = {a, speedbias_i, speedbias_i + 3, b, speedbias_j, speedbias_j + 3};
        if (verify_local_jacobians) {
            checkLocalPoseJacobian(imu, imu_parameters, {0, 2}, 2e-8);
            checkLocalPoseJacobian(pnp, pnp_parameters, {0, 3}, 2e-8);
            checkEuclideanBlockJacobian(imu, imu_parameters, 1, 2e-8);
            checkEuclideanBlockJacobian(pnp, pnp_parameters, 2, 2e-8);
        } else {
            Vector15 residual;
            ASSERT_TRUE(imu.Evaluate(imu_parameters.data(), residual.data(), nullptr));
            EXPECT_NEAR(residual.squaredNorm(), test_case.expected_cost, 2e-12) << "main IMU";
            ASSERT_TRUE(pnp.Evaluate(pnp_parameters.data(), residual.data(), nullptr));
            EXPECT_NEAR(residual.squaredNorm(), test_case.expected_cost, 2e-12) << "PnP IMU";
        }
      }
    }
}

TEST(NumericalContracts, ImuQuaternionRepresentativesPreserveCorrelatedPhysicalCost) {
    checkImuQuaternionRepresentatives(false);
}

TEST(NumericalContracts, ImuQuaternionRepresentativesKeepEffectiveLocalJacobianConsistent) {
    checkImuQuaternionRepresentatives(true);
}

TEST(NumericalContracts, PropagationRotationUsesUnitLocalIncrement) {
    backend::Estimator estimator;
    estimator.setParameter();
    const Eigen::Vector3d gyro(0, 0, 2);
    estimator.processIMU(0, zero, gyro);
    estimator.processImage({}, 0);
    estimator.processIMU(0.5, zero, gyro);
    const auto& R = estimator.sliding_window_[1].R;
    EXPECT_NEAR(R.determinant(), 1, 1e-12);
    EXPECT_LT((R.transpose() * R - Eigen::Matrix3d::Identity()).norm(), 1e-12);
    const Eigen::Matrix3d expected = Eigen::AngleAxisd(2 * std::atan(0.5), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    EXPECT_LT((R - expected).norm(), 1e-12);
}

TEST(NumericalContracts, PropagationRotatedAccelerationUsesIndependentMidpointOracle) {
    backend::Estimator estimator;
    const Eigen::Vector3d acceleration(1, 0, 0), gyro(0, 0, 2);
    estimator.processIMU(0, acceleration, gyro);
    estimator.processImage({}, 0);
    estimator.processIMU(0.5, acceleration, gyro);
    // Normalized [1,0,0,0.5] has cos(theta)=0.6, sin(theta)=0.8.
    // Midpoint acceleration is ((1,0,0)+(0.6,0.8,0))/2=(0.8,0.4,0).
    EXPECT_LT((estimator.sliding_window_[1].V - Eigen::Vector3d(0.4, 0.2, 0)).norm(), 1e-12);
    EXPECT_LT((estimator.sliding_window_[1].P - Eigen::Vector3d(0.1, 0.05, 0)).norm(), 1e-12);
}

TEST(NumericalContracts, PreintegrationRotatedAccelerationUsesUnitIncrementBeforeRotation) {
    const Eigen::Vector3d acceleration(1, 0, 0), gyro(0, 0, 2);
    backend::factor::IntegrationBase value(acceleration, gyro, zero, zero);
    value.push_back(0.5, acceleration, gyro);
    // Normalized increment has cos=0.6, sin=0.8, independently known above.
    EXPECT_LT((value.delta_v - Eigen::Vector3d(0.4, 0.2, 0)).norm(), 1e-12);
    EXPECT_LT((value.delta_p - Eigen::Vector3d(0.1, 0.05, 0)).norm(), 1e-12);
    value.push_back(0.5, acceleration, gyro);
    // Second rotation gives (-0.28,0.96,0), midpoint (0.16,0.88,0).
    EXPECT_LT((value.delta_v - Eigen::Vector3d(0.48, 0.64, 0)).norm(), 1e-12);
    EXPECT_LT((value.delta_p - Eigen::Vector3d(0.32, 0.26, 0)).norm(), 1e-12);
    const Eigen::Matrix3d R = value.delta_q.toRotationMatrix();
    EXPECT_NEAR(R.determinant(), 1, 1e-12);
    EXPECT_LT((R.transpose() * R - Eigen::Matrix3d::Identity()).norm(), 1e-12);
    EXPECT_TRUE(value.covariance.allFinite());
    EXPECT_LT((value.covariance - value.covariance.transpose()).norm(), 1e-12);
    Eigen::Matrix<double, 15, 15> whitening;
    EXPECT_TRUE(backend::factor::covarianceWhitening(value.covariance, whitening));
}

TEST(NumericalContracts, PreintegrationSmallStepBiasLinearizationRemainsWithinTruncationBudget) {
    const Eigen::Vector3d acceleration(0.4, -0.2, 9.81), gyro(0.1, 0.2, -0.15);
    backend::factor::IntegrationBase nominal(acceleration, gyro, zero, zero);
    for (int i = 0; i < 50; ++i) nominal.push_back(0.005, acceleration, gyro);
    const Eigen::Vector3d ba(1e-4, -0.5e-4, 0.8e-4), bg(-0.6e-4, 0.9e-4, 0.3e-4);
    backend::factor::IntegrationBase perturbed(acceleration, gyro, ba, bg);
    for (int i = 0; i < 50; ++i) perturbed.push_back(0.005, acceleration, gyro);
    const Eigen::Vector3d predicted_p = nominal.delta_p + nominal.jacobian.block<3, 3>(O_P, O_BA) * ba +
                                      nominal.jacobian.block<3, 3>(O_P, O_BG) * bg;
    const Eigen::Vector3d predicted_v = nominal.delta_v + nominal.jacobian.block<3, 3>(O_V, O_BA) * ba +
                                      nominal.jacobian.block<3, 3>(O_V, O_BG) * bg;
    const Eigen::Quaterniond predicted_q = (nominal.delta_q *
        Utility::deltaQ(nominal.jacobian.block<3, 3>(O_R, O_BG) * bg)).normalized();
    EXPECT_LT((predicted_p - perturbed.delta_p).norm(), 2e-7);
    EXPECT_LT((predicted_v - perturbed.delta_v).norm(), 2e-6);
    EXPECT_LT(2 * (predicted_q.inverse() * perturbed.delta_q).vec().norm(), 2e-7);
}

TEST(NumericalContracts, PnpRequiresBackendAnchorAndCurrentMinimumCorrespondences) {
    frontend::PnPFrontend pnp;
    common::SolvedFeature feature;
    feature.id = 1;
    feature.position = Eigen::Vector3d(0.2, 0.1, 5);
    feature.observation = Eigen::Vector2d(0.04, 0.02);
    feature.track_num = 10;
    for (int frame = 0; frame < 10; ++frame) {
        pnp.processIMU(0.01, gravity, zero);
        EXPECT_FALSE(pnp.processImage({feature}, frame * 0.01));
        EXPECT_FALSE(pnp.hasPose());
    }
    EXPECT_FALSE(pnp.processImage({}, 0.2));
}

void prepareOptimizerFixture(backend::SlidingWindow& window, frontend::FeatureManager& manager) {
    for (int frame = 0; frame <= WINDOW_SIZE; ++frame) {
        auto& state = window[frame];
        state.timestamp = frame * 0.06;
        state.P = Eigen::Vector3d(state.timestamp, 0, 0);
        state.V = Eigen::Vector3d::UnitX();
        state.pre_integration = std::make_unique<backend::factor::IntegrationBase>(
            utility::g_config.estimator.g, zero, zero, zero);
        for (int sample = 0; sample < 6; ++sample)
            state.pre_integration->push_back(0.01, utility::g_config.estimator.g, zero);
        common::ImageData image;
        for (int id = 0; id < 8; ++id) {
            Eigen::Matrix<double, 7, 1> ray;
            ray << (0.2 * (id % 4) - state.P.x()) / 5, (0.2 * (id / 4)) / 5, 1, 0, 0, 0, 0;
            image.emplace(id, ray);
        }
        manager.addFeatureAndCheckParallax(frame, image);
    }
    manager.setDepth(Eigen::VectorXd::Constant(manager.getFeatureCount(), 0.2));
}

TEST(NumericalContracts, UsableBudgetTerminationAndResetReleasePriorAndBlocks) {
    backend::SlidingWindow window;
    frontend::FeatureManager manager;
    prepareOptimizerFixture(window, manager);
    backend::Optimizer optimizer(&window, &manager);
    const int saved_iterations = utility::g_config.estimator.num_iterations;
    utility::g_config.estimator.num_iterations = 0;
    ASSERT_TRUE(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
    utility::g_config.estimator.num_iterations = saved_iterations;
    EXPECT_EQ(optimizer.getLastSolverDiagnostics().terminationType, ceres::NO_CONVERGENCE);
    EXPECT_TRUE(optimizer.getLastSolverDiagnostics().usable);
    EXPECT_TRUE(optimizer.hasMarginalizationPrior());
    EXPECT_GT(optimizer.getPriorParameterBlockCount(), 0);
    optimizer.reset();
    EXPECT_FALSE(optimizer.hasMarginalizationPrior());
    EXPECT_EQ(optimizer.getPriorParameterBlockCount(), 0);
    EXPECT_FALSE(optimizer.getLastSolverDiagnostics().usable);
}

double capturedSolverOption(const std::string& details, const std::string& name) {
    const std::string token = "\"" + name + "\":";
    const auto position = details.find(token);
    EXPECT_NE(position, std::string::npos) << details;
    if (position == std::string::npos) return std::numeric_limits<double>::quiet_NaN();
    return std::stod(details.substr(position + token.size()));
}

TEST(NumericalContracts, BenchmarkProfileReportsActualOptionsAndResetPreservesMode) {
    backend::SlidingWindow window;
    frontend::FeatureManager manager;
    prepareOptimizerFixture(window, manager);
    backend::Optimizer optimizer(&window, &manager);
    const int saved_iterations = utility::g_config.estimator.num_iterations;
    utility::g_config.estimator.num_iterations = 0;
    EXPECT_FALSE(optimizer.getBenchmarkSolverProfile());
    for (bool enabled : {false, true, false}) {
        optimizer.setBenchmarkSolverProfile(enabled);
        optimizer.reset();
        EXPECT_EQ(optimizer.getBenchmarkSolverProfile(), enabled);
        optimizer.setDiagnosticCapture(true);
        const bool solved = optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME);
        const auto diagnostics = optimizer.getLastSolverDiagnostics();
        EXPECT_TRUE(solved);
        EXPECT_TRUE(diagnostics.usable);
        EXPECT_EQ(diagnostics.terminationType, ceres::NO_CONVERGENCE);
        EXPECT_EQ(diagnostics.iterations, 1);  // Actual Ceres iteration0, not a fabricated ten-step count.
        EXPECT_DOUBLE_EQ(capturedSolverOption(diagnostics.numericalDetails, "functionTolerance"), enabled ? 0 : 1e-6);
        EXPECT_DOUBLE_EQ(capturedSolverOption(diagnostics.numericalDetails, "gradientTolerance"), enabled ? 0 : 1e-10);
        EXPECT_DOUBLE_EQ(capturedSolverOption(diagnostics.numericalDetails, "parameterTolerance"), enabled ? 0 : 1e-8);
        EXPECT_DOUBLE_EQ(capturedSolverOption(diagnostics.numericalDetails, "maxIterations"), 0);
        EXPECT_NE(diagnostics.numericalDetails.find(enabled ? "\"benchmarkSolverProfile\":true" :
                                                             "\"benchmarkSolverProfile\":false"), std::string::npos);
        EXPECT_NE(diagnostics.numericalDetails.find("Ceres summary rows including iteration0"), std::string::npos);
    }
    utility::g_config.estimator.num_iterations = saved_iterations;
}

class StationaryScalarCost final : public ceres::SizedCostFunction<1, 1> {
public:
    bool Evaluate(double const* const* parameters, double* residual, double** jacobians) const override {
        residual[0] = parameters[0][0];
        if (jacobians && jacobians[0]) jacobians[0][0] = 1;
        return true;
    }
};

TEST(NumericalContracts, ZeroToleranceKeepsActualStationaryConvergence) {
    double value = 0;
    ceres::Problem problem;
    problem.AddResidualBlock(new StationaryScalarCost, nullptr, &value);
    ceres::Solver::Options options;
    options.max_num_iterations = 10;
    options.function_tolerance = 0;
    options.gradient_tolerance = 0;
    options.parameter_tolerance = 0;
    options.logging_type = ceres::SILENT;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    EXPECT_TRUE(summary.IsSolutionUsable());
    EXPECT_EQ(summary.termination_type, ceres::CONVERGENCE);
    EXPECT_EQ(summary.iterations.size(), 1u);
    EXPECT_NE(summary.message.find("Gradient tolerance reached"), std::string::npos);
    EXPECT_DOUBLE_EQ(value, 0);
}

TEST(NumericalContracts, UnusableSolveDoesNotApplyResultsOrProducePrior) {
    backend::SlidingWindow window;
    frontend::FeatureManager manager;
    prepareOptimizerFixture(window, manager);
    window[WINDOW_SIZE].P.x() = 1e200;
    backend::Optimizer optimizer(&window, &manager);
    const Eigen::Vector3d before = window[WINDOW_SIZE].P;
    EXPECT_FALSE(optimizer.optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME));
    EXPECT_FALSE(optimizer.getLastSolverDiagnostics().usable);
    EXPECT_EQ(optimizer.getLastSolverDiagnostics().reason, "unusable_solver_result");
    EXPECT_TRUE(window[WINDOW_SIZE].P == before);
    EXPECT_FALSE(optimizer.hasMarginalizationPrior());
}

TEST(NumericalContracts, EstimatorResetInvalidatesGenerationStateAndLargeAbsolutePositionIsAllowed) {
    backend::Estimator estimator;
    estimator.setParameter();
    for (int cycle = 0; cycle < 10; ++cycle) {
        const auto generation = estimator.getResetGeneration();
        estimator.processIMU(0, gravity, zero);
        estimator.processImage({}, cycle);
        estimator.processIMU(0.01, gravity, zero);
        estimator.reset();
        EXPECT_EQ(estimator.getResetGeneration(), generation + 1);
        EXPECT_FALSE(estimator.hasUsableLatestImageUpdate());
        EXPECT_EQ(estimator.solver_flag_, common::SolverFlag::INITIAL);
        for (int i = 0; i <= WINDOW_SIZE; ++i) {
            EXPECT_FALSE(estimator.sliding_window_[i].pre_integration);
            EXPECT_TRUE(estimator.sliding_window_[i].P.isZero());
        }
    }
    estimator.solver_flag_ = common::SolverFlag::NON_LINEAR;
    estimator.sliding_window_[WINDOW_SIZE].P = Eigen::Vector3d(101, 0, 0);
    const auto generation = estimator.getResetGeneration();
    estimator.processImage({}, 11);
    EXPECT_EQ(estimator.getResetGeneration(), generation);
    estimator.sliding_window_[0].P.x() = std::numeric_limits<double>::quiet_NaN();
    estimator.processImage({}, 12);
    EXPECT_EQ(estimator.getResetGeneration(), generation + 1);
    EXPECT_EQ(estimator.getLastSolverDiagnostics().reason, "nonfinite_state_reset");
}

TEST(NumericalContracts, PnpCurrentAnchoredSolveAndEmptyFrameFreshness) {
    frontend::PnPFrontend pnp;
    pnp.setExtrinsicParameters(Eigen::Matrix3d::Identity(), zero);
    pnp.setIMUModel();
    common::VINSResult state;
    state.timestamp = 0;
    state.R.setIdentity(); state.P.setZero(); state.V = Eigen::Vector3d(0.2, 0, 0);
    state.Ba.setZero(); state.Bg.setZero();
    pnp.initializeState(state);
    pnp.processIMU(0, utility::g_config.estimator.g, zero);
    const int saved_iterations = utility::g_config.pnp.pnp_max_iterations;
    const double saved_time = utility::g_config.pnp.pnp_solver_time;
    utility::g_config.pnp.pnp_max_iterations = 20;
    utility::g_config.pnp.pnp_solver_time = 1;
    for (int frame = 0; frame <= 6; ++frame) {
        const double time = frame * 0.06;
        if (frame) for (int sample = 0; sample < 6; ++sample) pnp.processIMU(0.01, utility::g_config.estimator.g, zero);
        std::vector<common::SolvedFeature> features;
        for (int id = 0; id < 8; ++id) {
            common::SolvedFeature feature;
            feature.id = id;
            feature.position = Eigen::Vector3d(0.3 * (id % 4), 0.3 * (id / 4), 5 + 0.1 * id);
            feature.observation = (feature.position - Eigen::Vector3d(0.2 * time, 0, 0)).head<2>() / feature.position.z();
            feature.track_num = 10;
            features.push_back(feature);
        }
        EXPECT_EQ(pnp.processImage(features, time), frame == 6);
    }
    utility::g_config.pnp.pnp_max_iterations = saved_iterations;
    utility::g_config.pnp.pnp_solver_time = saved_time;
    EXPECT_TRUE(pnp.hasUsableLatestImageUpdate());
    EXPECT_LT((pnp.getPosition() - Eigen::Vector3d(0.072, 0, 0)).norm(), 1e-5);
    EXPECT_FALSE(pnp.processImage({}, 0.42));
    EXPECT_FALSE(pnp.hasPose());
    EXPECT_FALSE(pnp.hasUsableLatestImageUpdate());
    pnp.clearState();
    EXPECT_FALSE(pnp.hasPose());
}

TEST(NumericalContracts, GravityRefinementUsesCurrentLinearizationOnly) {
    const int count = 12;
    const double dt = 0.13;
    const double scale = 2.3;
    const double gravity_magnitude = utility::g_config.estimator.g.norm();
    const Eigen::Vector3d truth = Eigen::Vector3d(0.5, -0.7, 9.75).normalized() * gravity_magnitude;
    const Eigen::Vector3d saved_tic = utility::g_config.camera.t_ic;
    utility::g_config.camera.t_ic.setZero();
    std::map<double, common::ImageFrame> frames;
    std::vector<Eigen::Vector3d> positions, velocities;
    for (int i = 0; i < count; ++i) {
        const double t = i * dt;
        positions.emplace_back(std::sin(3 * t), std::cos(2 * t), 0.3 * std::sin(5 * t));
        velocities.emplace_back(3 * std::cos(3 * t), -2 * std::sin(2 * t), 1.5 * std::cos(5 * t));
        common::ImageFrame frame({}, t);
        frame.R = (Eigen::AngleAxisd(0.2 * i, Eigen::Vector3d::UnitX()) *
                   Eigen::AngleAxisd(-0.1 * i, Eigen::Vector3d::UnitY())).toRotationMatrix();
        frame.T = positions.back() / scale;
        frame.pre_integration = std::make_unique<backend::factor::IntegrationBase>(zero, zero, zero, zero);
        frames.emplace(t, std::move(frame));
    }
    auto a = frames.begin();
    for (int i = 0; i + 1 < count; ++i, ++a) {
        auto b = std::next(a);
        auto& pre = *b->second.pre_integration;
        pre.sum_dt = dt;
        pre.delta_p = a->second.R.transpose() *
            (positions[i + 1] - positions[i] - velocities[i] * dt + 0.5 * truth * dt * dt);
        pre.delta_v = a->second.R.transpose() * (velocities[i + 1] - velocities[i] + truth * dt);
    }
    Eigen::Vector3d estimate = (truth + Eigen::Vector3d(1.2, -0.8, 0.3)).normalized() * gravity_magnitude;
    Eigen::VectorXd x;
    frontend::initialization::RefineGravity(frames, estimate, x);
    utility::g_config.camera.t_ic = saved_tic;
    ASSERT_TRUE(x.allFinite());
    EXPECT_LT((estimate - truth).norm(), 1e-5);
    EXPECT_NEAR(x.tail<1>()[0] / 100, scale, 1e-5);
}

TEST(NumericalContracts, InitializationMapsPhysicalVelocitiesToTheirSourceImageRows) {
    const auto saved_config = utility::g_config;
    const double frame_dt = 0.13, imu_dt = 0.0005, scale = 2.3;
    const Eigen::Vector3d true_gravity(0, 0, saved_config.estimator.g.norm());
    const Eigen::Vector3d lever_arm(0.04, 0.07, -0.03);
    const Eigen::Vector3d axis = Eigen::Vector3d(0.2, -0.3, 0.4).normalized();
    const Eigen::Vector3d angular_velocity = 0.4 * axis;
    const auto rotation = [&](double t) { return Eigen::AngleAxisd(0.4 * t, axis).toRotationMatrix(); };
    const auto position = [](double t) { return Eigen::Vector3d(std::sin(3 * t), std::cos(2 * t), 0.3 * std::sin(5 * t)); };
    const auto velocity = [](double t) { return Eigen::Vector3d(3 * std::cos(3 * t), -2 * std::sin(2 * t), 1.5 * std::cos(5 * t)); };
    const auto acceleration = [](double t) { return Eigen::Vector3d(-9 * std::sin(3 * t), -4 * std::cos(2 * t), -7.5 * std::sin(5 * t)); };
    const auto specific_force = [&](double t) -> Eigen::Vector3d {
        return rotation(t).transpose() * (acceleration(t) + true_gravity);
    };
    const auto preintegrate = [&](double begin, double end) {
        auto pre = std::make_unique<backend::factor::IntegrationBase>(specific_force(begin), angular_velocity, zero, zero);
        const int samples = static_cast<int>(std::lround((end - begin) / imu_dt));
        for (int sample = 1; sample <= samples; ++sample)
            pre->push_back(imu_dt, specific_force(begin + sample * imu_dt), angular_velocity);
        return pre;
    };
    utility::g_config.camera.t_ic = lever_arm;
    for (int count : {11, 13}) {
        SCOPED_TRACE(count);
        backend::SlidingWindow window;
        frontend::FeatureManager manager;
        frontend::initialization::MotionEstimator motion;
        std::map<double, common::ImageFrame> frames;
        std::vector<double> key_times;
        for (int row = 0; row < count; ++row) {
            const double time = row * frame_dt;
            common::ImageFrame frame({}, time);
            frame.R = rotation(time);
            // SfM camera position, with known arbitrary scale and physical nonzero lever arm.
            frame.T = (position(time) + frame.R * lever_arm) / scale;
            frame.pre_integration = preintegrate(time - frame_dt, time);
            frame.is_key_frame = count == 11 || (row != 1 && row != 3);
            if (frame.is_key_frame) key_times.push_back(time);
            frames.emplace(time, std::move(frame));
        }
        EXPECT_EQ(key_times.size(), std::size_t(WINDOW_SIZE + 1));
        for (int key = 0; key <= WINDOW_SIZE; ++key) {
            window[key].timestamp = key_times[key];
            window[key].pre_integration = preintegrate(key ? key_times[key - 1] : -frame_dt, key_times[key]);
        }
        Eigen::Vector3d estimated_gravity = zero;
        const Eigen::Matrix3d r_ic = Eigen::Matrix3d::Identity();
        int frame_count = WINDOW_SIZE;
        auto margin = common::MarginalizationFlag::MARGIN_OLD_KEYFRAME;
        frontend::initialization::Initializer initializer(&window, &manager, &motion, &frames, &frame_count,
                                                        &margin, &estimated_gravity, &r_ic, &lever_arm);
        const bool aligned = frontend::initialization::InitializerTestAccess::align(initializer);
        EXPECT_TRUE(aligned);
        if (aligned) {
            EXPECT_LT((estimated_gravity - true_gravity).norm(), 2e-5);
            for (int key = 0; key <= WINDOW_SIZE; ++key) {
                const double time = key_times[key];
                EXPECT_LT((window[key].V - velocity(time)).norm(), 2e-5) << "key=" << key << "source t=" << time;
                EXPECT_LT((window[key].P - (position(time) - position(0))).norm(), 2e-5) << "key=" << key;
                EXPECT_LT((window[key].R - rotation(time)).norm(), 2e-5) << "key=" << key;
            }
        }
    }
    utility::g_config = saved_config;
}

class LinearMarginalizationCost final : public ceres::SizedCostFunction<1, 1, 1, 1> {
public:
    LinearMarginalizationCost(const Eigen::Vector3d& coefficients, double residual)
        : coefficients_(coefficients), residual_(residual) {}
    bool Evaluate(const double* const* parameters, double* residuals, double** jacobians) const override {
        residuals[0] = residual_;
        for (int i = 0; i < 3; ++i) {
            residuals[0] += coefficients_[i] * parameters[i][0];
            if (jacobians && jacobians[i]) jacobians[i][0] = coefficients_[i];
        }
        return true;
    }
private:
    Eigen::Vector3d coefficients_;
    double residual_;
};

TEST(NumericalContracts, MarginalizationSemanticLayoutAndIndependentSchurOracle) {
    // These exact small integer rows give H=[[3,0,2],[0,4,-1],[2,-1,4]], b=[1,10,1].
    const double rows[5][3] = {{1,1,0},{1,0,1},{0,1,-1},{0,1,1},{1,-1,1}};
    const double residuals[5] = {1,2,3,4,-2};
    const int layouts[3][3] = {{0,4,8},{11,2,7},{23,17,5}};
    for (const auto& layout : layouts) {
        double storage[32] = {};
        std::vector<double*> blocks = {storage + layout[0], storage + layout[1], storage + layout[2]};
        backend::factor::MarginalizationInfo info;
        for (int i = 0; i < 5; ++i) {
            info.addResidualBlockInfo(new backend::factor::ResidualBlockInfo(
                new LinearMarginalizationCost(Eigen::Vector3d(rows[i][0], rows[i][1], rows[i][2]), residuals[i]),
                nullptr, blocks, {0}));
        }
        info.preMarginalize();
        ASSERT_TRUE(info.marginalize());
        ASSERT_EQ(info.m, 1);
        ASSERT_EQ(info.n, 2);
        // Semantic first-seen order A,B,C remains independent of physical allocation/address order.
        EXPECT_EQ(info.parameter_block_idx.at(reinterpret_cast<long>(blocks[0])), 0);
        EXPECT_EQ(info.parameter_block_idx.at(reinterpret_cast<long>(blocks[1])), 1);
        EXPECT_EQ(info.parameter_block_idx.at(reinterpret_cast<long>(blocks[2])), 2);
        Eigen::Matrix2d expected;
        expected << 4, -1, -1, 8.0 / 3;
        const Eigen::MatrixXd actual = info.linearized_jacobians.transpose() * info.linearized_jacobians;
        const Eigen::VectorXd gradient = info.linearized_jacobians.transpose() * info.linearized_residuals;
        EXPECT_LT((actual - expected).norm(), 2e-12);
        EXPECT_LT((gradient - Eigen::Vector2d(10, 1.0 / 3)).norm(), 2e-12);
        std::unordered_map<long, double*> shifts = {{reinterpret_cast<long>(blocks[1]), blocks[1]},
                                                   {reinterpret_cast<long>(blocks[2]), blocks[2]}};
        EXPECT_EQ(info.getParameterBlocks(shifts), (std::vector<double*>{blocks[1], blocks[2]}));
    }
}

TEST(NumericalContracts, MarginalizationUsesDeclaredFactorAccumulationOrder) {
    const double huge = std::ldexp(1.0, 54);
    const std::vector<double> rows{huge, 1, -huge, 1};
    // Historical normal-equation fold1 and thread-fold0 are recorded in the old red/green logs.
    // QR changes that floating implementation contract. Preserve the adversarial inputs,
    // semantic row order and address independence; exact real gradient is 2, not the old fold1.
    const std::uint64_t exact_huge = std::uint64_t{1} << 54;
    const std::uint64_t exact = ((exact_huge + 1) - exact_huge) + 1;
    ASSERT_EQ(exact, 2u);
    const double epsilon = std::numeric_limits<double>::epsilon();
    const double backward_budget = 128 * epsilon / (1 - 128 * epsilon) * (2 * huge + 2);
    Eigen::VectorXd reference;
    for (const auto& layout : {std::vector<int>{0,4,8}, std::vector<int>{11,2,7}}) {
        double storage[16] = {};
        std::vector<double*> blocks{storage + layout[0], storage + layout[1], storage + layout[2]};
        backend::factor::MarginalizationInfo info;
        for (double residual : rows)
            info.addResidualBlockInfo(new backend::factor::ResidualBlockInfo(
                new LinearMarginalizationCost(Eigen::Vector3d(0,1,0), residual), nullptr, blocks, {0}));
        info.addResidualBlockInfo(new backend::factor::ResidualBlockInfo(
            new LinearMarginalizationCost(Eigen::Vector3d(1,0,0), 0), nullptr, blocks, {0}));
        info.addResidualBlockInfo(new backend::factor::ResidualBlockInfo(
            new LinearMarginalizationCost(Eigen::Vector3d(0,0,1), 0), nullptr, blocks, {0}));
        info.preMarginalize();
        for (std::size_t i = 0; i < rows.size(); ++i) EXPECT_EQ(info.factors[i]->residuals[0], rows[i]);
        ASSERT_TRUE(info.marginalize());
        const Eigen::VectorXd gradient = info.linearized_jacobians.transpose() * info.linearized_residuals;
        EXPECT_LE(std::abs(gradient[0] - double(exact)), backward_budget);
        EXPECT_TRUE(gradient.allFinite());
        Eigen::Matrix2d expected = Eigen::Vector2d(4, 1).asDiagonal();
        EXPECT_LE((info.linearized_jacobians.transpose() * info.linearized_jacobians - expected).norm(), 1e-13);
        if (reference.size()) EXPECT_TRUE((gradient - reference).isZero(0));
        reference = gradient;
    }
}

class DeclaredLinearCost final : public ceres::CostFunction {
public:
    DeclaredLinearCost(Eigen::MatrixXd jacobian, Eigen::VectorXd residual)
        : jacobian_(std::move(jacobian)), residual_(std::move(residual)) {
        set_num_residuals(static_cast<int>(jacobian_.rows()));
        mutable_parameter_block_sizes()->assign(jacobian_.cols(), 1);
    }
    bool Evaluate(double const* const*, double* residual, double** jacobians) const override {
        Eigen::Map<Eigen::VectorXd>(residual, residual_.size()) = residual_;
        if (jacobians) for (Eigen::Index i = 0; i < jacobian_.cols(); ++i)
            if (jacobians[i]) Eigen::Map<Eigen::VectorXd>(jacobians[i], jacobian_.rows()) = jacobian_.col(i);
        return true;
    }
private:
    Eigen::MatrixXd jacobian_;
    Eigen::VectorXd residual_;
};

void knownWeakPrior(const Eigen::MatrixXd& jacobian, const Eigen::VectorXd& residual,
                    const std::vector<int>& dropped, int expected_dropped_rank, double scale,
                    double coefficient_error = 0) {
    std::vector<double> values(jacobian.cols(), 0);
    std::vector<double*> parameters;
    for (auto& value : values) parameters.push_back(&value);
    backend::factor::MarginalizationInfo info;
    info.addResidualBlockInfo(new backend::factor::ResidualBlockInfo(
        new DeclaredLinearCost(scale * jacobian, scale * residual), nullptr, parameters, dropped));
    info.preMarginalize();
    ASSERT_TRUE(info.marginalize());
    ASSERT_EQ(info.n, 2);
    EXPECT_EQ(info.getStatistics().droppedRank, expected_dropped_rank);
    ASSERT_EQ(info.linearized_jacobians.rows(), 2);
    ASSERT_EQ(info.linearized_residuals.size(), 2);
    ASSERT_TRUE(info.linearized_jacobians.allFinite());
    const Eigen::MatrixXd root = info.linearized_jacobians / scale;
    const Eigen::VectorXd rhs = info.linearized_residuals / scale;
    const double weak = std::ldexp(1.0, -8);
    Eigen::Matrix2d expected;
    expected << 1, -1, -1, 1;
    expected *= weak * weak;
    const Eigen::Vector2d expected_gradient(3 * weak, -3 * weak);
    const double roundoff = 64 * std::numeric_limits<double>::epsilon();
    EXPECT_LE((root.transpose() * root - expected).norm(),
              roundoff + coefficient_error * (2 * std::sqrt(2.0) * weak + coefficient_error));
    EXPECT_LE((root.transpose() * rhs - expected_gradient).norm(),
              roundoff + coefficient_error * (residual.norm() + 1));
    // Uniform global translation is a known nullspace; contrast is real observable information.
    EXPECT_LE((root * Eigen::Vector2d::Ones()).norm(), roundoff + std::sqrt(2.0) * coefficient_error);
    EXPECT_GT((root * Eigen::Vector2d(1, -1)).squaredNorm(), 2 * weak * weak);
}

TEST(NumericalContracts, MarginalizationPreservesWeakContrastGaugeAndUniformScale) {
    const double large = std::ldexp(1.0, 22), weak = std::ldexp(1.0, -8);
    Eigen::Matrix<double, 2, 3> jacobian;
    jacobian << large, -large / 2, -large / 2, 0, weak, -weak;
    // Exact elimination leaves only [weak,-weak], r=3. In normal equations,
    // weak^2 is below one ULP of large^2/4 and disappears before Schur subtraction.
    for (int exponent : {-16, 0, 16})
        knownWeakPrior(jacobian, Eigen::Vector2d(1, 3), {0}, 1, std::ldexp(1.0, exponent));
}

TEST(NumericalContracts, MarginalizationHandlesPartialDroppedRank) {
    const double large = std::ldexp(1.0, 22), weak = std::ldexp(1.0, -8);
    Eigen::Matrix<double, 2, 4> partial;
    partial << large / 2, large / 2, -large / 2, -large / 2, 0, 0, weak, -weak;
    knownWeakPrior(partial, Eigen::Vector2d(1, 3), {0, 1}, 1, 1);
}

TEST(NumericalContracts, MarginalizationHandlesZeroDroppedRank) {
    const double weak = std::ldexp(1.0, -8);
    Eigen::Matrix<double, 2, 3> zero_drop;
    zero_drop << 0, 0, 0, 0, weak, -weak;
    knownWeakPrior(zero_drop, Eigen::Vector2d(1, 3), {0}, 0, 1);
}

TEST(NumericalContracts, MarginalizationHandlesNoDroppedColumns) {
    const double weak = std::ldexp(1.0, -8);
    Eigen::Matrix<double, 2, 3> zero_drop;
    zero_drop << 0, 0, 0, 0, weak, -weak;
    knownWeakPrior(zero_drop.rightCols(2), Eigen::Vector2d(1, 3), {}, 0, 1);
}

TEST(NumericalContracts, MarginalizationMixedRowsPreserveKnownNullspaceAndWeakInformation) {
    const double large = std::ldexp(1.0, 22), weak = std::ldexp(1.0, -8);
    Eigen::Matrix<double, 4, 3> base = Eigen::Matrix<double, 4, 3>::Zero();
    base.row(0) << large, -large / 2, -large / 2;
    base.row(1) << 0, weak, -weak;
    Eigen::Matrix4d mix;
    mix << 1, 1, 1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1;
    mix *= 0.5;  // Independent exact orthogonal row mixing, not a production decomposition.
    const Eigen::MatrixXd jacobian = mix * base;
    const Eigen::Vector4d residual = mix * Eigen::Vector4d(1, 3, 5, 7);
    const double coefficient_error = 128 * std::numeric_limits<double>::epsilon() * base.norm();
    knownWeakPrior(jacobian, residual, {0}, 1, 1, coefficient_error);
}

TEST(NumericalContracts, MarginalizationDenseBudgetRejectsWholeSystemWithoutTruncation) {
    for (int columns : {1400, 1500}) {
        std::vector<double> values(columns, 0);
        std::vector<double*> parameters;
        for (auto& value : values) parameters.push_back(&value);
        backend::factor::MarginalizationInfo info;
        info.setDiagnosticCapture(true);
        info.addResidualBlockInfo(new backend::factor::ResidualBlockInfo(
            new DeclaredLinearCost(Eigen::MatrixXd::Ones(1, columns), Eigen::VectorXd::Ones(1)),
            nullptr, parameters, {}));
        info.preMarginalize();
        const bool success = info.marginalize();
        EXPECT_EQ(info.n, columns);
        EXPECT_EQ(info.getStatistics().inputRows, 1);
        if (columns == 1400) {
            ASSERT_TRUE(success);
            EXPECT_EQ(info.factors[0]->jacobians.size(), std::size_t(columns));
            EXPECT_EQ(info.linearized_jacobians.cols(), columns);
            EXPECT_LE(info.getStatistics().denseWorkspaceCells, backend::factor::kMarginalizationDenseWorkspaceCellBudget);
        } else {
            EXPECT_FALSE(success);
            EXPECT_TRUE(info.factors[0]->jacobians.empty());
            EXPECT_EQ(info.factors[0]->raw_jacobians, nullptr);
            EXPECT_EQ(info.getFailureReason(), "marginalization_dense_budget_exceeded");
            EXPECT_EQ(info.linearized_jacobians.size(), 0);
            EXPECT_EQ(info.linearized_residuals.size(), 0);
            EXPECT_GT(info.getStatistics().denseWorkspaceCells, backend::factor::kMarginalizationDenseWorkspaceCellBudget);
            EXPECT_NE(info.getMarginalizationDiagnostics().find("marginalization_dense_budget_exceeded"), std::string::npos);
        }
    }
}
class RejectedMarginalizationCost final : public ceres::SizedCostFunction<1, 1, 1> {
public:
    explicit RejectedMarginalizationCost(bool allocation_failure) : allocation_failure_(allocation_failure) {}
    bool Evaluate(double const* const*, double*, double**) const override {
        if (allocation_failure_) throw std::bad_alloc();
        return false;
    }
private:
    bool allocation_failure_;
};

TEST(NumericalContracts, MarginalizationEvaluationAndAllocationFailureAreExplicit) {
    for (bool allocation_failure : {false, true}) {
        double dropped = 0, kept = 0;
        backend::factor::MarginalizationInfo info;
        info.addResidualBlockInfo(new backend::factor::ResidualBlockInfo(
            new RejectedMarginalizationCost(allocation_failure), nullptr, {&dropped, &kept}, {0}));
        info.preMarginalize();
        EXPECT_FALSE(info.marginalize());
        EXPECT_EQ(info.getFailureReason(), allocation_failure ? "marginalization_allocation_failed" :
                                                              "marginalization_factor_evaluation_failed");
        EXPECT_EQ(info.linearized_jacobians.size(), 0);
        EXPECT_EQ(info.linearized_residuals.size(), 0);
        EXPECT_EQ(info.getMarginalizationDiagnostics(), "{\"enabled\":false}");
    }
}

class UnevaluatedOversizedCost final : public ceres::CostFunction {
public:
    explicit UnevaluatedOversizedCost(bool* evaluated) : evaluated_(evaluated) {
        set_num_residuals(std::numeric_limits<int>::max());
        mutable_parameter_block_sizes()->push_back(std::numeric_limits<int>::max());
    }
    bool Evaluate(double const* const*, double*, double**) const override {
        *evaluated_ = true;
        return false;
    }
private:
    bool* evaluated_;
};

TEST(NumericalContracts, MarginalizationWorkspaceOverflowIsRejectedBeforeAllocation) {
    double value = 0;
    bool evaluated = false;
    backend::factor::MarginalizationInfo info;
    info.setDiagnosticCapture(true);
    info.addResidualBlockInfo(new backend::factor::ResidualBlockInfo(
        new UnevaluatedOversizedCost(&evaluated), nullptr, {&value}, {}));
    info.preMarginalize();
    EXPECT_FALSE(info.marginalize());
    EXPECT_FALSE(evaluated);
    EXPECT_EQ(info.getFailureReason(), "marginalization_dimension_overflow");
    EXPECT_TRUE(info.getStatistics().workspaceEstimateOverflow);
    EXPECT_EQ(info.linearized_jacobians.size(), 0);
    EXPECT_EQ(info.factors[0]->raw_jacobians, nullptr);
    EXPECT_NE(info.getMarginalizationDiagnostics().find("\"denseWorkspaceCells\":null"), std::string::npos);
}

TEST(NumericalContracts, MarginalizationNoKeptColumnsIsAnExplicitEmptySystem) {
    double value = 0;
    backend::factor::MarginalizationInfo info;
    info.addResidualBlockInfo(new backend::factor::ResidualBlockInfo(
        new DeclaredLinearCost(Eigen::MatrixXd::Ones(1, 1), Eigen::VectorXd::Ones(1)), nullptr, {&value}, {0}));
    info.preMarginalize();
    EXPECT_FALSE(info.marginalize());
    EXPECT_EQ(info.n, 0);
    EXPECT_EQ(info.getFailureReason(), "marginalization_empty_system");
    EXPECT_EQ(info.linearized_jacobians.size(), 0);
}

}  // namespace
