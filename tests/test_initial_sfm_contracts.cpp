#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "frontend/initialization/initial_sfm.h"

namespace {
using frontend::initialization::InitialSFM;
using frontend::initialization::ReprojectionError3D;
using frontend::initialization::SFMFeature;

std::vector<SFMFeature> observedLandmarks(const std::map<int,Eigen::Vector3d>& landmarks) {
    std::vector<SFMFeature> features;
    for(const auto& entry : landmarks) {
        const auto& point = entry.second;
        SFMFeature feature{};
        feature.id = entry.first;
        feature.observation = {
            {0,{point.x()/point.z(),point.y()/point.z()}},
            {1,{(point.x()-1)/point.z(),point.y()/point.z()}}
        };
        features.push_back(feature);
    }
    return features;
}

std::map<int,Eigen::Vector3d> knownLandmarks(double depth_sign) {
    std::map<int,Eigen::Vector3d> points;
    for(int i=0;i<25;++i)
        points.emplace(i,Eigen::Vector3d((i%5-2)*.35,(i/5-2)*.25,depth_sign*(3+.13*(i%7))));
    return points;
}

bool reconstruct(std::vector<SFMFeature>& features,std::map<int,Eigen::Vector3d>& points,
                 Eigen::Quaterniond* rotations,Eigen::Vector3d* centers) {
    InitialSFM sfm;
    return sfm.construct(2,rotations,centers,0,Eigen::Matrix3d::Identity(),
                         Eigen::Vector3d(1,0,0),features,points);
}
}

TEST(InitialSFMContracts, ParallelRaysCannotBecomeAUsableStructureAfterFailedBA) {
    std::vector<SFMFeature> features;
    for(int i=0;i<20;++i) {
        SFMFeature feature{};
        feature.id = i;
        feature.observation = {{0,Eigen::Vector2d::Zero()},{1,Eigen::Vector2d::Zero()}};
        features.push_back(feature);
    }
    // P0=[I|0], P1=[I|(-1,0,0)]: identical origin rays have nullspace [0,0,1,0].
    // There is no finite landmark; this exercises public construct, triangulation and actual Ceres BA.
    Eigen::Quaterniond rotations[2]; Eigen::Vector3d centers[2];
    std::map<int,Eigen::Vector3d> points;
    EXPECT_FALSE(reconstruct(features,points,rotations,centers));
    EXPECT_TRUE(points.empty());
    for(const auto& feature : features) EXPECT_FALSE(feature.state);
}

TEST(InitialSFMContracts, EmptyResidualProblemCannotReportAReconstruction) {
    std::vector<SFMFeature> features;
    Eigen::Quaterniond rotations[2]; Eigen::Vector3d centers[2];
    std::map<int,Eigen::Vector3d> points;
    EXPECT_FALSE(reconstruct(features,points,rotations,centers));
    EXPECT_TRUE(points.empty());
}

TEST(InitialSFMContracts, BehindCameraLandmarksNeverCountAsTriangulated) {
    auto features = observedLandmarks(knownLandmarks(-1));
    Eigen::Quaterniond rotations[2]; Eigen::Vector3d centers[2];
    std::map<int,Eigen::Vector3d> points;
    EXPECT_FALSE(reconstruct(features,points,rotations,centers));
    EXPECT_TRUE(points.empty());
    for(const auto& feature : features) EXPECT_FALSE(feature.state);
}

TEST(InitialSFMContracts, KnownFrontLandmarksRecoverCameraCentersAndIndependentMetricCoordinates) {
    const auto expected = knownLandmarks(1);
    auto features = observedLandmarks(expected);
    Eigen::Quaterniond rotations[2]; Eigen::Vector3d centers[2];
    std::map<int,Eigen::Vector3d> points;
    ASSERT_TRUE(reconstruct(features,points,rotations,centers));
    ASSERT_EQ(points.size(),expected.size());
    EXPECT_TRUE(rotations[0].toRotationMatrix().isApprox(Eigen::Matrix3d::Identity(),1e-8));
    EXPECT_TRUE(rotations[1].toRotationMatrix().isApprox(Eigen::Matrix3d::Identity(),1e-8));
    EXPECT_LE(centers[0].norm(),1e-8);
    EXPECT_LE((centers[1]-Eigen::Vector3d(1,0,0)).norm(),1e-8);
    for(const auto& entry : expected) {
        ASSERT_EQ(points.count(entry.first),1u);
        const auto& actual = points.at(entry.first);
        EXPECT_TRUE(actual.allFinite());
        EXPECT_GT(actual.z(),0);
        EXPECT_LE((actual-entry.second).norm(),1e-7);
    }
}

TEST(InitialSFMContracts, ReprojectionRejectsZeroBehindAndNonfiniteCameraDepthOrCoordinates) {
    const double rotation[4] = {1,0,0,0}, translation[3] = {0,0,0};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const double invalid[][3] = {{1,2,0},{1,2,-1},{nan,0,1},{0,inf,1},{0,0,nan},{0,0,inf}};
    ReprojectionError3D factor(0,0);
    for(const auto& point : invalid) {
        SCOPED_TRACE(::testing::Message()<<"point="<<point[0]<<","<<point[1]<<","<<point[2]);
        double residual[2] = {};
        EXPECT_FALSE(factor(rotation,translation,point,residual));
    }
}

TEST(InitialSFMContracts, ReprojectionPositiveDepthPreservesWxyzRotationAndIndependentResidualGolden) {
    const double s = std::sqrt(.5);
    const double rotation[4] = {s,0,0,s}, translation[3] = {.5,-.5,1}, point[3] = {2,1,4};
    // Rz(90deg)*[2,1,4]+t=[-.5,1.5,5], projection=[-.1,.3].
    ReprojectionError3D factor(-.2,.1);
    double residual[2] = {};
    ASSERT_TRUE(factor(rotation,translation,point,residual));
    EXPECT_NEAR(residual[0],.1,1e-12);
    EXPECT_NEAR(residual[1],.2,1e-12);
}
