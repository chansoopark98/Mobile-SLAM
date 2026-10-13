#include <gtest/gtest.h>
#include <array>
#include <limits>
#include "frontend/feature_manager.h"

namespace {
struct RestoreGeometryConfig {
    utility::Config config=utility::g_config;
    ~RestoreGeometryConfig() {utility::g_config=config;}
};
TEST(GeometryReuse, SharedAnchorTransformsRecoverIndependentDepthsAfterWindowChanges) {
    backend::SlidingWindow window;
    frontend::FeatureManager manager;
    const Eigen::Vector3d tic(.07,-.03,.02);
    const Eigen::Matrix3d ric=Eigen::AngleAxisd(.13,Eigen::Vector3d(1,-2,3).normalized()).toRotationMatrix();
    constexpr std::array<int,4> anchors{0,3,8,9};
    for (int pass=0;pass<2;++pass) {
        manager.clearState();
        for (int frame=0;frame<=utility::WINDOW_SIZE;++frame) {
            window[frame].P={.25*frame,.004*frame*frame,.03*pass*frame};
            window[frame].R=Eigen::AngleAxisd(.03*frame+.02*pass,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
        }
        std::array<double,80> expected_depth;
        for (int id=0;id<80;++id) {
            const int anchor=anchors[id%anchors.size()];
            frontend::FeaturePerId feature(id,anchor);
            feature.geometry_pending=anchor>=8;
            const Eigen::Vector3d world(.15*(id%10)-.7,.13*(id/10)-.5,4.+.02*id);
            for (int frame=anchor;frame<=utility::WINDOW_SIZE;++frame) {
                const Eigen::Matrix3d rotation=window[frame].R*ric;
                const Eigen::Vector3d center=window[frame].P+window[frame].R*tic;
                const Eigen::Vector3d camera=rotation.transpose()*(world-center);
                if (frame==anchor) expected_depth[id]=camera.z();
                Eigen::Matrix<double,7,1> observation;
                observation<<camera.x()/camera.z(),camera.y()/camera.z(),1,0,0,0,0;
                feature.feature_per_frame.emplace_back(observation);
            }
            manager.feature_bank_.push_back(feature);
        }
        manager.triangulateAcrossAllViews(window,tic,ric);
        ASSERT_EQ(manager.feature_bank_.size(),expected_depth.size());
        for (const auto& feature:manager.feature_bank_) {
            SCOPED_TRACE(::testing::Message()<<"pass="<<pass<<" id="<<feature.feature_id);
            EXPECT_NEAR(feature.estimated_depth,expected_depth[feature.feature_id],1e-9);
            EXPECT_EQ(feature.solve_flag,0);
            EXPECT_TRUE(feature.isDepthAdmitted());
            if (feature.geometry_pending) {
                EXPECT_TRUE(feature.geometry_admitted);
                EXPECT_STREQ(feature.geometry_reason,"accepted");
            }
        }
    }
}

TEST(GeometryReuse, InvalidYoungRayDoesNotReuseAnotherFeatureAdmission) {
    backend::SlidingWindow window;
    frontend::FeatureManager manager;
    for (int frame=0;frame<=utility::WINDOW_SIZE;++frame) window[frame].P={.25*frame,0,0};
    for (int id=0;id<2;++id) {
        frontend::FeaturePerId feature(id,9);
        feature.geometry_pending=true;
        for (int frame=9;frame<=utility::WINDOW_SIZE;++frame) {
            Eigen::Matrix<double,7,1> observation;
            observation<<-window[frame].P.x()/4,0,1,0,0,0,0;
            if (id && frame==10) observation.x()=std::numeric_limits<double>::quiet_NaN();
            feature.feature_per_frame.emplace_back(observation);
        }
        manager.feature_bank_.push_back(feature);
    }
    manager.triangulateAcrossAllViews(window,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
    const auto& good=manager.feature_bank_.front();
    const auto& bad=manager.feature_bank_.back();
    EXPECT_TRUE(good.geometry_admitted);
    EXPECT_NEAR(good.estimated_depth,4,1e-10);
    EXPECT_FALSE(bad.geometry_admitted);
    EXPECT_EQ(bad.estimated_depth,-1);
    EXPECT_EQ(bad.solve_flag,0);
    EXPECT_STREQ(bad.geometry_reason,"rank_or_condition");
}

TEST(GeometryReuse, InvalidRotationOnlyRejectsTracksThatObserveIt) {
    for (bool nonfinite:{false,true}) {
        backend::SlidingWindow window;
        frontend::FeatureManager manager;
        for (int frame=0;frame<=utility::WINDOW_SIZE;++frame) window[frame].P={.25*frame,0,0};
        window[5].R(0,0)=nonfinite ? std::numeric_limits<double>::quiet_NaN() : 2;
        for (int id=0;id<2;++id) {
            frontend::FeaturePerId feature(id,id?9:0);
            feature.geometry_pending=true;
            for (int frame=feature.start_frame;frame<=utility::WINDOW_SIZE;++frame) {
                Eigen::Matrix<double,7,1> observation;
                observation<<-window[frame].P.x()/4,0,1,0,0,0,0;
                feature.feature_per_frame.emplace_back(observation);
            }
            manager.feature_bank_.push_back(feature);
        }
        manager.triangulateAcrossAllViews(window,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
        EXPECT_FALSE(manager.feature_bank_.front().geometry_admitted);
        EXPECT_EQ(manager.feature_bank_.front().estimated_depth,-1);
        EXPECT_STREQ(manager.feature_bank_.front().geometry_reason,"rank_or_condition");
        EXPECT_TRUE(manager.feature_bank_.back().geometry_admitted);
        EXPECT_NEAR(manager.feature_bank_.back().estimated_depth,4,1e-10);
    }
}

TEST(GeometryReuse, MalformedLegacyRayCannotInheritSameShapePredecessorDepth) {
    backend::SlidingWindow window;
    frontend::FeatureManager manager;
    for (int frame=0;frame<=utility::WINDOW_SIZE;++frame) window[frame].P={.25*frame,0,0};
    for (int id=0;id<3;++id) {
        frontend::FeaturePerId feature(id,0);
        for (int frame=0;frame<=utility::WINDOW_SIZE;++frame) {
            Eigen::Matrix<double,7,1> observation;
            observation<<-window[frame].P.x()/(4+id),0,1,0,0,0,0;
            if (id==1 && frame==5) observation.x()=std::numeric_limits<double>::quiet_NaN();
            feature.feature_per_frame.emplace_back(observation);
        }
        manager.feature_bank_.push_back(feature);
    }
    manager.triangulateAcrossAllViews(window,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity());
    auto feature=manager.feature_bank_.begin();
    EXPECT_NEAR(feature->estimated_depth,4,1e-10);
    ++feature;
    EXPECT_EQ(feature->estimated_depth,-1);
    EXPECT_EQ(feature->solve_flag,2);
    EXPECT_FALSE(feature->isDepthAdmitted());
    EXPECT_STREQ(feature->geometry_reason,"nonfinite_dlt");
    ++feature;
    EXPECT_NEAR(feature->estimated_depth,6,1e-10);
    EXPECT_TRUE(feature->isDepthAdmitted());
}

TEST(ObservationRejectionGeometry, ActualGoodViewsDetermineDltRowsAndCachedSvdShape) {
    RestoreGeometryConfig restore;utility::g_config.camera.fx=400;utility::g_config.camera.fy=600;
    backend::SlidingWindow window;frontend::FeatureManager manager;
    const Eigen::Vector3d tic(.03,-.02,.01),world(.4,-.2,4);
    const Eigen::Matrix3d ric=Eigen::AngleAxisd(.13,Eigen::Vector3d(1,-2,3).normalized()).toRotationMatrix();
    std::array<Eigen::Matrix3d,11> rotations;
    std::array<Eigen::Vector3d,11> centers;
    for(int frame=0;frame<=10;++frame) {
        window[frame].P={.2*frame,.01*frame,0};
        window[frame].R=Eigen::AngleAxisd(.02*frame,Eigen::Vector3d::UnitY()).toRotationMatrix();
        rotations[frame]=window[frame].R*ric;centers[frame]=window[frame].P+window[frame].R*tic;
    }
    const double depth=(rotations[8].transpose()*(world-centers[8])).z();
    for(int id=0;id<5;++id) {
        manager.feature_bank_.emplace_back(id,8);auto& feature=manager.feature_bank_.back();
        feature.geometry_pending=true;
        for(int frame=8;frame<=10;++frame) {
            const Eigen::Vector3d camera=rotations[frame].transpose()*(world-centers[frame]);
            Eigen::Matrix<double,7,1> observation;observation<<camera.x()/camera.z(),camera.y()/camera.z(),1,0,0,0,0;
            feature.feature_per_frame.emplace_back(observation);
            const bool bad=(id==0 && frame==9) || (id==1 && frame>=9) || (id==2 && frame==10) || (id==4 && frame==8);
            if(bad) {
                feature.feature_per_frame.back().projection_outlier=true;
                feature.feature_per_frame.back().ray_vector.x()=std::numeric_limits<double>::quiet_NaN();
            }
        }
    }
    manager.triangulateAcrossAllViews(window,tic,ric);
    for(const auto& feature:manager.feature_bank_) {
        SCOPED_TRACE(feature.feature_id);
        if(feature.feature_id==1 || feature.feature_id==4) {
            EXPECT_FALSE(feature.geometry_admitted);EXPECT_FALSE(feature.isDepthAdmitted());
            EXPECT_DOUBLE_EQ(feature.estimated_depth,-1);
            EXPECT_EQ(feature.solve_flag,feature.feature_id==4 ? 2 : 0);
        } else {
            EXPECT_TRUE(feature.geometry_admitted);EXPECT_TRUE(feature.isDepthAdmitted());
            EXPECT_NEAR(feature.estimated_depth,depth,1e-9);
            EXPECT_TRUE(frontend::FeatureManager::hasConsistentReprojection(feature,depth,rotations,centers));
        }
    }
}

TEST(ObservationRejectionGeometry, BadPairsDoNotEnterCorrespondencesOrParallaxDenominator) {
    RestoreGeometryConfig restore;utility::g_config.camera.focal_length=400;utility::g_config.estimator.min_parallax=10;
    frontend::FeatureManager manager;
    for(int id=0;id<21;++id) {
        manager.feature_bank_.emplace_back(id,0);auto& feature=manager.feature_bank_.back();
        for(int frame:{0,1}) {
            Eigen::Matrix<double,7,1> observation;observation<<((id<20 && frame==1) ? .5 : 0),0,1,0,0,0,0;
            feature.feature_per_frame.emplace_back(observation);
        }
        if(id<20)feature.feature_per_frame.front().projection_outlier=true;
    }
    const auto pairs=manager.getCorresponding(0,1);ASSERT_EQ(pairs.size(),1u);
    EXPECT_EQ(pairs.front().first,Eigen::Vector3d(0,0,1));
    EXPECT_EQ(pairs.front().second,Eigen::Vector3d(0,0,1));
    common::ImageData image;
    Eigen::Matrix<double,7,1> observation;observation<<0,0,1,0,0,0,0;
    for(int id=0;id<21;++id)image.emplace(id,observation);
    EXPECT_FALSE(manager.addFeatureAndCheckParallax(2,image));
    manager.addFeatureAndCheckParallax(4,{{0,observation}}); // A missing frame starts a new actual track.
    const auto fresh=std::find_if(manager.feature_bank_.begin(),manager.feature_bank_.end(),
        [](const auto& feature){return feature.feature_id==0;});
    ASSERT_NE(fresh,manager.feature_bank_.end());EXPECT_EQ(fresh->start_frame,4);
    EXPECT_FALSE(fresh->feature_per_frame.front().projection_outlier);
    manager.clearState();manager.addFeatureAndCheckParallax(0,{{0,observation}});
    EXPECT_FALSE(manager.feature_bank_.front().feature_per_frame.front().projection_outlier);
}
}  // namespace
