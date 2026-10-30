#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <limits>
#include <deque>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <stdexcept>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include "vio_engine.h"

struct VIOEngineTestAccess {
    static bool integrate(VIOEngine& engine,const std::vector<IMUReading>& readings,double time) {
        return engine.processIMUData(readings.data(),static_cast<int>(readings.size()),time);
    }
    static backend::Estimator& estimator(VIOEngine& engine) { return *engine.estimator_; }
    static double cursor(const VIOEngine& engine) { return engine.getIMUEndpointTimestamp(); }
    static Eigen::Vector3d acceleration(const VIOEngine& engine) { return engine.prev_acc_; }
    static Eigen::Vector3d angularVelocity(const VIOEngine& engine) { return engine.prev_gyro_; }
    static size_t pending(const VIOEngine& engine) { return engine.pending_imu_.size(); }
    static bool pose(VIOEngine& engine,const Eigen::Vector3d& position,const Eigen::Matrix3d& rotation,double time,double* output) {
        return engine.publishPose(position,rotation,time,output);
    }
    static void solver(VIOEngine& engine,const backend::SolverDiagnostics& diagnostics) { engine.recordSolver(diagnostics); }
    static frontend::FeatureTracker& tracker(VIOEngine& engine) { return *engine.feature_tracker_; }
    static utility::InertialPrediction& prediction(VIOEngine& engine) { return engine.inertial_prediction_; }
    static void finishPrediction(VIOEngine& engine) { engine.finishInertialFrame(); }
    static bool rawCoverage(const VIOEngine& engine) { return engine.inertial_raw_coverage_; }
    static void anchorInputs(VIOEngine& engine,double time,const Eigen::Vector3d& acc,const Eigen::Vector3d& gyro) {
        engine.current_time_=engine.prev_image_timestamp_=engine.last_imu_timestamp_=time;
        engine.raw_imu_left_timestamp_=engine.frame_timestamp_=time;engine.inertial_raw_coverage_=true;
        engine.prev_acc_=acc;engine.prev_gyro_=gyro;engine.pending_imu_.clear();
    }
    static void failure(VIOEngine& engine,const std::string& reason) { engine.invalidatePose(reason); }
    static void frameTime(VIOEngine& engine,double time) { engine.frame_timestamp_=time; }
};

namespace backend {
struct EstimatorTestAccess {
    static frontend::FeatureManager& features(backend::Estimator& estimator) { return estimator.feature_manager_; }
    static backend::Optimizer& optimizer(backend::Estimator& estimator) { return estimator.optimizer_; }
    static size_t imageCount(const backend::Estimator& estimator) { return estimator.all_image_frame_.size(); }
    static void initialized(backend::Estimator& estimator) {
        estimator.solver_flag_=common::SolverFlag::NON_LINEAR;
        estimator.frame_count_=utility::WINDOW_SIZE;
        estimator.g_=utility::g_config.estimator.g;
    }
    static void usable(backend::Estimator& estimator,bool value) { estimator.latest_image_update_usable_=value; }
    static void gravity(backend::Estimator& estimator,const Eigen::Vector3d& value) { estimator.g_=value; }
};
}

namespace {
struct RestorePolicyConfig {
    utility::Config config=utility::g_config;
    ~RestorePolicyConfig() {utility::g_config=config;}
};
class BoundaryRayCamera : public common::camera_models::PinholeCamera {
public:
    BoundaryRayCamera() : PinholeCamera(Parameters("boundary",96,96,0,0,0,0,100,100,48,48)) {}
    bool reject_right_half = false;
    void liftProjective(const Eigen::Vector2d& pixel,Eigen::Vector3d& ray) const override {
        if(pixel.x()==1) ray=Eigen::Vector3d(1,1,0);
        else if(pixel.x()==2) ray=Eigen::Vector3d(1,1,-1);
        else if(pixel.x()==3 || (reject_right_half && pixel.x()>=48))
            ray=Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),1,1);
        else ray=Eigen::Vector3d((pixel.x()-48)/100,(pixel.y()-48)/100,1);
    }
};
class InvalidPredictionCamera : public common::camera_models::PinholeCamera {
public:
    explicit InvalidPredictionCamera(bool nonfinite) :
        PinholeCamera(Parameters("invalid_prediction",640,480,0,0,0,0,461,449,319,239)),nonfinite_(nonfinite) {}
    void spaceToPlane(const Eigen::Vector3d&,Eigen::Vector2d& pixel) const override {
        pixel={nonfinite_?std::numeric_limits<double>::quiet_NaN():1e30,240};
    }
private:
    bool nonfinite_;
};
class OffsetPredictionCamera : public common::camera_models::PinholeCamera {
public:
    explicit OffsetPredictionCamera(double offset) :
        PinholeCamera(Parameters("offset_prediction",640,480,0,0,0,0,461,449,319,239)),offset_(offset) {}
    void spaceToPlane(const Eigen::Vector3d& ray,Eigen::Vector2d& pixel) const override {
        PinholeCamera::spaceToPlane(ray,pixel);pixel.x()+=offset_;
    }
private:
    double offset_;
};
bool configure(VIOEngine& engine, const double* rotation = nullptr,
               const double* translation = nullptr, int model = 2, double fx = 100.0) {
    const double identity[9] = {1,0,0,0,1,0,0,0,1};
    const double zero[3] = {0,0,0};
    return engine.configure(96,96,fx,100,48,48,model,0,0,0,0,
                            rotation ? rotation : identity, translation ? translation : zero,
                            0.1,0.001,0.01,0.0001,9.81);
}

bool populatePolicyBackend(VIOEngine& engine) {
    auto& estimator=VIOEngineTestAccess::estimator(engine);
    auto& manager=backend::EstimatorTestAccess::features(estimator);manager.clearState();
    backend::EstimatorTestAccess::initialized(estimator);
    for(int frame=0;frame<=utility::WINDOW_SIZE;++frame) {
        auto& state=estimator.sliding_window_[frame];state.timestamp=.06*frame;
        state.P={state.timestamp,0,0};state.V=Eigen::Vector3d::UnitX();
        state.pre_integration=std::make_unique<backend::factor::IntegrationBase>(
            utility::g_config.estimator.g,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
        for(int sample=0;sample<6;++sample)
            state.pre_integration->push_back(.01,utility::g_config.estimator.g,Eigen::Vector3d::Zero());
        common::ImageData image;
        for(int id=0;id<8;++id) {
            Eigen::Matrix<double,7,1> observation;
            observation<<(.2*(id%4)-state.P.x())/5,.2*(id/4)/5,1,0,0,0,0;
            image.emplace(id,observation);
        }
        manager.addFeatureAndCheckParallax(frame,image);
    }
    manager.setDepth(Eigen::VectorXd::Constant(8,.2));
    manager.feature_bank_.back().feature_per_frame.back().ray_vector.x()+=100/utility::g_config.camera.fx;
    const int iterations=utility::g_config.estimator.num_iterations;utility::g_config.estimator.num_iterations=0;
    const bool usable=backend::EstimatorTestAccess::optimizer(estimator).optimize(common::MarginalizationFlag::MARGIN_OLD_KEYFRAME);
    utility::g_config.estimator.num_iterations=iterations;
    return usable;
}

// Independent analytic projection and inverse mapping render a stationary textured
// scene at infinity. No tracker/camera-model implementation supplies expected pixels.
struct RotationFixture {
    utility::Config saved=utility::g_config;
    const int width=640,height=480;
    const double fx=461,fy=449,cx=319,cy=239;
    const int model;
    const double k1,k2,p1,p2;
    cv::Mat source;
    explicit RotationFixture(int camera_model) : model(camera_model),
        k1(model==2?-.08:.01),k2(model==2?.015:-.002),
        p1(model==2?.002:.0002),p2(model==2?-.001:-.00001) {
        utility::g_config.camera.col=width;utility::g_config.camera.row=height;
        utility::g_config.camera.focal_length=(fx+fy)/2;
        auto& cfg=utility::g_config.feature_tracker;
        cfg.max_cnt=120;cfg.min_dist=30;cfg.equalize=1;cfg.fisheye=model==0;
        cfg.lk_window_size=21;cfg.lk_pyramid_levels=3;cfg.lk_iterations=20;cfg.lk_eps=.03;
        cfg.f_threshold=1;cfg.f_threshold_edge_factor=0;
        source.create(height,width,CV_8UC1);cv::RNG random(20261010);random.fill(source,cv::RNG::UNIFORM,0,256);
        cv::GaussianBlur(source,source,cv::Size(5,5),.8);
    }
    ~RotationFixture() {utility::g_config=saved;}
    Eigen::Vector2d project(const Eigen::Vector3d& ray) const {
        double x=ray.x()/ray.z(),y=ray.y()/ray.z();
        if(model==2) {
            const double r2=x*x+y*y,radial=1+k1*r2+k2*r2*r2;
            return {fx*(x*radial+2*p1*x*y+p2*(r2+2*x*x))+cx,
                    fy*(y*radial+p1*(r2+2*y*y)+2*p2*x*y)+cy};
        }
        const double radius=std::hypot(x,y),theta=std::atan(radius),t2=theta*theta;
        const double distorted=theta*(1+t2*(k1+t2*(k2+t2*(p1+t2*p2))));
        const double scale=radius>0?distorted/radius:1;
        return {fx*x*scale+cx,fy*y*scale+cy};
    }
    Eigen::Vector3d lift(const Eigen::Vector2d& pixel) const {
        const double xd=(pixel.x()-cx)/fx,yd=(pixel.y()-cy)/fy;
        double x=xd,y=yd;
        if(model==2) for(int iteration=0;iteration<12;++iteration) {
            const double r2=x*x+y*y,radial=1+k1*r2+k2*r2*r2;
            x=(xd-2*p1*x*y-p2*(r2+2*x*x))/radial;
            y=(yd-p1*(r2+2*y*y)-2*p2*x*y)/radial;
        } else {
            const double radius=std::hypot(xd,yd);double theta=radius;
            for(int iteration=0;iteration<12;++iteration) {
                const double t2=theta*theta;
                theta-=(theta*(1+t2*(k1+t2*(k2+t2*(p1+t2*p2))))-radius)/
                    (1+t2*(3*k1+t2*(5*k2+t2*(7*p1+t2*9*p2))));
            }
            const double scale=radius>0?std::tan(theta)/radius:1;x=xd*scale;y=yd*scale;
        }
        return {x,y,1};
    }
    cv::Mat rotated(const Eigen::Matrix3d& old_to_new,const Eigen::Vector3d* translation=nullptr) const {
        cv::Mat map_x(height,width,CV_32F),map_y(height,width,CV_32F),target;
        for(int y=0;y<height;++y) for(int x=0;x<width;++x) {
            Eigen::Vector3d old_ray=old_to_new.transpose()*lift({double(x),double(y)});
            // Intersect the new viewing ray with the stationary old-camera Z=2m plane.
            if(translation) old_ray=*translation+old_ray*((2-translation->z())/old_ray.z());
            const Eigen::Vector2d old_pixel=project(old_ray);
            map_x.at<float>(y,x)=old_pixel.x();map_y.at<float>(y,x)=old_pixel.y();
        }
        cv::remap(source,target,map_x,map_y,cv::INTER_LINEAR,cv::BORDER_CONSTANT);
        return target;
    }
    bool configureTracker(frontend::FeatureTracker& tracker) const {
        return tracker.setIntrinsicParameter(model,width,height,fx,fy,cx,cy,k1,k2,p1,p2);
    }
};
struct RotationScore {int visible=0,correct=0,wrong=0;};
RotationScore scoreRotation(const RotationFixture& fixture,const frontend::FeatureTracker& tracker,
                           const std::map<int,cv::Point2f>& original,const Eigen::Matrix3d& rotation,
                           const Eigen::Vector3d* translation=nullptr) {
    RotationScore score;
    for(const auto& [id,pixel]:original) {
        Eigen::Vector3d ray=fixture.lift({pixel.x,pixel.y});
        if(translation) ray=2*ray-*translation;
        const Eigen::Vector2d expected=fixture.project(rotation*ray);
        if(expected.x()<12 || expected.x()>=fixture.width-12 || expected.y()<12 || expected.y()>=fixture.height-12) continue;
        ++score.visible;
        const auto found=std::find(tracker.ids.begin(),tracker.ids.end(),id);
        if(found==tracker.ids.end()) continue;
        const auto& actual=tracker.cur_pts[found-tracker.ids.begin()];
        const double error=(Eigen::Vector2d(actual.x,actual.y)-expected).norm();
        if(error<1) ++score.correct;
        if(error>3) ++score.wrong;
    }
    return score;
}
std::map<int,cv::Point2f> trackedPixels(frontend::FeatureTracker& tracker) {
    for(unsigned int i=0;tracker.updateID(i);++i) {}
    std::map<int,cv::Point2f> result;
    for(size_t i=0;i<tracker.ids.size();++i) result[tracker.ids[i]]=tracker.cur_pts[i];
    return result;
}
void expectSameTrackingState(const frontend::FeatureTracker& actual,int actual_first,
                             const frontend::FeatureTracker& expected,int expected_first) {
    EXPECT_EQ(actual.prev_pts,expected.prev_pts);EXPECT_EQ(actual.cur_pts,expected.cur_pts);
    EXPECT_EQ(actual.next_pts,expected.next_pts);EXPECT_EQ(actual.prev_undistorted_pts,expected.prev_undistorted_pts);
    EXPECT_EQ(actual.cur_undistorted_pts,expected.cur_undistorted_pts);EXPECT_EQ(actual.pts_velocity,expected.pts_velocity);
    EXPECT_EQ(actual.track_cnt,expected.track_cnt);EXPECT_DOUBLE_EQ(actual.cur_time,expected.cur_time);
    EXPECT_DOUBLE_EQ(actual.prev_time,expected.prev_time);
    std::vector<int> actual_ids,expected_ids;
    for(int id:actual.ids) actual_ids.push_back(id-actual_first);
    for(int id:expected.ids) expected_ids.push_back(id-expected_first);
    EXPECT_EQ(actual_ids,expected_ids);
    const auto normalized=[](const std::map<int,cv::Point2f>& input,int first) {
        std::map<int,cv::Point2f> output;
        for(const auto& [id,point]:input) output[id<0?id:id-first]=point;
        return output;
    };
    EXPECT_EQ(normalized(actual.cur_undistorted_pts_map,actual_first),normalized(expected.cur_undistorted_pts_map,expected_first));
    EXPECT_EQ(normalized(actual.prev_undistorted_pts_map,actual_first),normalized(expected.prev_undistorted_pts_map,expected_first));
    EXPECT_EQ(cv::norm(actual.cur_img,expected.cur_img,cv::NORM_INF),0);
    EXPECT_EQ(cv::norm(actual.prev_img,expected.prev_img,cv::NORM_INF),0);
}
}

TEST(EngineContracts, RotationPredictionRecoversCorrectIdsThroughDistortedCameraModels) {
    for(int model:{2,0}) {
        SCOPED_TRACE(model);RotationFixture fixture(model);
        const Eigen::Matrix3d rotation=Eigen::AngleAxisd(-6*M_PI/180,Eigen::Vector3d::UnitY()).toRotationMatrix();
        const cv::Mat target=fixture.rotated(rotation);
        const Eigen::Matrix3d inexact_rotation=Eigen::AngleAxisd(-5*M_PI/180,Eigen::Vector3d::UnitY()).toRotationMatrix();
        RotationScore baseline,seeded,inexact;
        for(int prediction:{0,1,2}) {
            frontend::FeatureTracker tracker;ASSERT_TRUE(fixture.configureTracker(tracker));
            tracker.detectAndTrack(fixture.source,0);const auto original=trackedPixels(tracker);
            if(prediction==1) tracker.setDiagnosticCapture(true);
            cv::setRNGSeed(20261010);
            tracker.detectAndTrack(target,.05,false,prediction==0?nullptr:prediction==1?&rotation:&inexact_rotation);
            (prediction==0?baseline:prediction==1?seeded:inexact)=scoreRotation(fixture,tracker,original,rotation);
            if(prediction==1) {
                const auto& diagnostics=tracker.getFeatureDiagnostics();
                const std::string ordinary_field="\"ordinaryInliers\":";
                const std::string adopted_field="\"adopted\":";
                const size_t ordinary_at=diagnostics.find(ordinary_field),adopted_at=diagnostics.find(adopted_field);
                ASSERT_NE(ordinary_at,std::string::npos);ASSERT_NE(adopted_at,std::string::npos);
                EXPECT_GT(std::stoi(diagnostics.substr(adopted_at+adopted_field.size())),
                          std::stoi(diagnostics.substr(ordinary_at+ordinary_field.size())));
            }
        }
        ASSERT_GT(seeded.visible,60);
        EXPECT_GE(seeded.correct,seeded.visible*9/10);
        EXPECT_LE(seeded.wrong,seeded.visible/20);
        EXPECT_GT(seeded.correct,baseline.correct+seeded.visible/4);
        // A valid but inaccurate 5-degree hint must be refined against the
        // rendered six-degree image, rather than counted as a correspondence.
        EXPECT_GE(inexact.correct,inexact.visible*9/10);
        EXPECT_LE(inexact.wrong,inexact.visible/20);
        std::cout<<"[Rotation fixture] model="<<model<<" visible="<<seeded.visible
            <<" baseline_correct="<<baseline.correct<<" baseline_wrong="<<baseline.wrong
            <<" seeded_correct="<<seeded.correct<<" seeded_wrong="<<seeded.wrong
            <<" inexact_correct="<<inexact.correct<<" inexact_wrong="<<inexact.wrong<<std::endl;
    }
}

TEST(EngineContracts, InvalidRotationPredictionFallsBackToUnseededTracking) {
    RotationFixture fixture(2);
    std::vector<cv::Point2f> baseline;
    std::vector<Eigen::Matrix3d> rotations{Eigen::Matrix3d::Identity(),Eigen::Matrix3d::Identity(),Eigen::Matrix3d::Identity()};
    rotations[0](0,0)=-1;rotations[1](0,1)=.1;rotations[2](0,0)=std::numeric_limits<double>::quiet_NaN();
    for(int trial=-1;trial<int(rotations.size());++trial) {
        frontend::FeatureTracker tracker;ASSERT_TRUE(fixture.configureTracker(tracker));
        tracker.detectAndTrack(fixture.source,0);trackedPixels(tracker);cv::setRNGSeed(20261010);
        ASSERT_NO_THROW(tracker.detectAndTrack(fixture.source,.05,false,trial<0?nullptr:&rotations[trial]));
        if(trial<0) baseline=tracker.cur_pts;
        else {ASSERT_EQ(tracker.cur_pts.size(),baseline.size());for(size_t i=0;i<baseline.size();++i)EXPECT_EQ(tracker.cur_pts[i],baseline[i]);}
    }
}

TEST(EngineContracts, SmallRotationPredictionPreservesExactUnseededPipeline) {
    for(int model:{2,0}) {
        SCOPED_TRACE(model);RotationFixture fixture(model);
        const Eigen::Matrix3d rotation=Eigen::AngleAxisd(-.5*M_PI/180,Eigen::Vector3d::UnitY()).toRotationMatrix();
        const cv::Mat target=fixture.rotated(rotation);
        std::vector<cv::Point2f> baseline_points;std::vector<int> baseline_ids;
        for(bool predict:{false,true}) {
            frontend::FeatureTracker tracker;ASSERT_TRUE(fixture.configureTracker(tracker));
            tracker.setDiagnosticCapture(true);tracker.detectAndTrack(fixture.source,0);
            const auto original=trackedPixels(tracker);ASSERT_FALSE(original.empty());
            const int first_id=original.begin()->first;
            double largest_shift=0;
            for(const auto& [id,pixel]:original) {
                const Eigen::Vector2d expected=fixture.project(rotation*fixture.lift({pixel.x,pixel.y}));
                largest_shift=std::max(largest_shift,(expected-Eigen::Vector2d(pixel.x,pixel.y)).norm());
            }
            // The independently rendered motion fits within the full-resolution
            // 21-pixel patch's ten-pixel radius for every original feature.
            ASSERT_LT(largest_shift,10);
            cv::setRNGSeed(20261010);tracker.detectAndTrack(target,.05,false,predict?&rotation:nullptr);
            std::vector<int> relative_ids;
            for(int id:tracker.ids) relative_ids.push_back(id-first_id);
            if(!predict) {baseline_points=tracker.cur_pts;baseline_ids=relative_ids;}
            else {
                EXPECT_EQ(relative_ids,baseline_ids);ASSERT_EQ(tracker.cur_pts.size(),baseline_points.size());
                for(size_t i=0;i<baseline_points.size();++i) EXPECT_EQ(tracker.cur_pts[i],baseline_points[i]);
                EXPECT_NE(tracker.getFeatureDiagnostics().find("\"available\":true"),std::string::npos);
                EXPECT_NE(tracker.getFeatureDiagnostics().find("\"attempted\":0"),std::string::npos);
            }
        }
    }
}

TEST(EngineContracts, RotationSeedWindowRadiusBoundaryUsesConfiguredPatchGeometry) {
    RotationFixture fixture(2);
    for(int window:{17,21,31}) for(bool outside:{false,true}) {
        SCOPED_TRACE(::testing::Message()<<"window="<<window<<" outside="<<outside);
        utility::g_config.feature_tracker.lk_window_size=window;
        const int radius=(window-1)/2;
        frontend::FeatureTracker tracker;ASSERT_TRUE(fixture.configureTracker(tracker));
        tracker.m_camera=std::make_shared<OffsetPredictionCamera>(radius+(outside?1:0));
        tracker.setDiagnosticCapture(true);tracker.detectAndTrack(fixture.source,0);trackedPixels(tracker);
        const Eigen::Matrix3d identity=Eigen::Matrix3d::Identity();
        tracker.detectAndTrack(fixture.source,.05,false,&identity);
        const auto diagnostics=tracker.getFeatureDiagnostics();
        EXPECT_NE(diagnostics.find("\"available\":true"),std::string::npos);
        if(outside) EXPECT_EQ(diagnostics.find("\"hintEligible\":0"),std::string::npos);
        else EXPECT_NE(diagnostics.find("\"hintEligible\":0"),std::string::npos);
    }
}

TEST(EngineContracts, RotationSeedPreservesCorrectTracksWhenTranslationOpposesRotation) {
    for(int model:{2,0}) {
        SCOPED_TRACE(model);RotationFixture fixture(model);
        const double angle=6*M_PI/180;
        const Eigen::Matrix3d rotation=Eigen::AngleAxisd(-angle,Eigen::Vector3d::UnitY()).toRotationMatrix();
        const Eigen::Vector3d translation(-2*std::tan(angle),0,0);
        const cv::Mat target=fixture.rotated(rotation,&translation);
        RotationScore baseline,seeded;
        frontend::FeatureTracker ordinary;
        int ordinary_first=0;uint64_t ordinary_rng=0;
        for(bool predict:{false,true}) {
            frontend::FeatureTracker tracker;ASSERT_TRUE(fixture.configureTracker(tracker));
            tracker.setDiagnosticCapture(true);
            tracker.detectAndTrack(fixture.source,0);const auto original=trackedPixels(tracker);
            cv::setRNGSeed(20261010);tracker.detectAndTrack(target,.05,false,predict?&rotation:nullptr);
            (predict?seeded:baseline)=scoreRotation(fixture,tracker,original,rotation,&translation);
            if(!predict) {ordinary=tracker;ordinary_first=original.begin()->first;ordinary_rng=cv::theRNG().state;}
            else {
                expectSameTrackingState(tracker,original.begin()->first,ordinary,ordinary_first);
                EXPECT_EQ(cv::theRNG().state,ordinary_rng);
                const auto diagnostic=tracker.getFeatureDiagnostics();
                EXPECT_NE(diagnostic.find("\"ordinaryTrusted\":true"),std::string::npos);
                EXPECT_EQ(diagnostic.find("\"hintEligible\":0"),std::string::npos);
                EXPECT_NE(diagnostic.find("\"attempted\":0"),std::string::npos);
            }
        }
        ASSERT_GT(baseline.visible,100);ASSERT_GE(baseline.correct,baseline.visible*9/10);
        std::cout<<"[Opposing translation] model="<<model<<" visible="<<baseline.visible
            <<" baseline_correct="<<baseline.correct<<" baseline_wrong="<<baseline.wrong
            <<" seeded_correct="<<seeded.correct<<" seeded_wrong="<<seeded.wrong<<std::endl;
        EXPECT_GE(seeded.correct,baseline.correct);
        EXPECT_LE(seeded.wrong,baseline.wrong);
    }
}

TEST(EngineContracts, WeakGeometryWithNoAdoptionPreservesEntireOrdinaryTrackingState) {
    for(int model:{2,0}) {
        SCOPED_TRACE(model);RotationFixture fixture(model);
        const Eigen::Matrix3d wrong_rotation=Eigen::AngleAxisd(-6*M_PI/180,Eigen::Vector3d::UnitY()).toRotationMatrix();
        frontend::FeatureTracker ordinary;
        int ordinary_first=0;uint64_t ordinary_rng=0;
        for(bool predict:{false,true}) {
            frontend::FeatureTracker tracker;ASSERT_TRUE(fixture.configureTracker(tracker));
            tracker.setDiagnosticCapture(true);tracker.detectAndTrack(fixture.source,0);
            const auto original=trackedPixels(tracker);ASSERT_FALSE(original.empty());
            cv::setRNGSeed(20261010);tracker.detectAndTrack(fixture.source,.05,false,predict?&wrong_rotation:nullptr);
            if(!predict) {ordinary=tracker;ordinary_first=original.begin()->first;ordinary_rng=cv::theRNG().state;}
            else {
                expectSameTrackingState(tracker,original.begin()->first,ordinary,ordinary_first);
                EXPECT_EQ(cv::theRNG().state,ordinary_rng);
                const auto diagnostic=tracker.getFeatureDiagnostics();
                EXPECT_NE(diagnostic.find("\"ordinaryEstimated\":false"),std::string::npos);
                EXPECT_EQ(diagnostic.find("\"attempted\":0"),std::string::npos);
                EXPECT_NE(diagnostic.find("\"adopted\":0"),std::string::npos);
            }
        }
    }
}

TEST(EngineContracts, RotationSeedDoesNotTurnUnsupportedPixelsIntoCorrespondences) {
    RotationFixture fixture(2);
    for(int invalid_projection:{0,1,2}) for(bool blank:{false,true}) {
        frontend::FeatureTracker tracker;ASSERT_TRUE(fixture.configureTracker(tracker));
        if(invalid_projection) tracker.m_camera=std::make_shared<InvalidPredictionCamera>(invalid_projection==1);
        tracker.setDiagnosticCapture(true);tracker.detectAndTrack(fixture.source,0);trackedPixels(tracker);
        // Behind-camera, nonfinite and out-of-image hints all leave ordinary LK
        // responsible for finding a correspondence; the hint is never published.
        const Eigen::Matrix3d wrong_rotation=Eigen::AngleAxisd(invalid_projection?0:M_PI,Eigen::Vector3d::UnitY()).toRotationMatrix();
        const cv::Mat target=blank?cv::Mat(fixture.height,fixture.width,CV_8UC1,cv::Scalar(127)):fixture.source;
        tracker.detectAndTrack(target,.05,false,&wrong_rotation);
        EXPECT_NE(tracker.getFeatureDiagnostics().find("\"attempted\":0"),std::string::npos);
        if(blank) EXPECT_TRUE(tracker.cur_pts.empty());
        else EXPECT_GT(tracker.cur_pts.size(),100u);
    }
}

TEST(EngineContracts, IdentityRotationSeedRetainsPureTranslationCorrespondences) {
    RotationFixture fixture(2);
    cv::Mat target;cv::warpAffine(fixture.source,target,cv::Matx23d(1,0,3,0,1,-2),fixture.source.size());
    frontend::FeatureTracker tracker;ASSERT_TRUE(fixture.configureTracker(tracker));
    tracker.detectAndTrack(fixture.source,0);const auto original=trackedPixels(tracker);
    const Eigen::Matrix3d identity=Eigen::Matrix3d::Identity();
    tracker.detectAndTrack(target,.05,false,&identity);
    int correct=0;
    for(size_t i=0;i<tracker.ids.size();++i) {
        const auto found=original.find(tracker.ids[i]);ASSERT_NE(found,original.end());
        if(cv::norm(tracker.cur_pts[i]-(found->second+cv::Point2f(3,-2)))<1) ++correct;
    }
    EXPECT_GE(correct,int(original.size())*9/10);
}

TEST(EngineContracts, IMURotationSeedUsesNonidentityCameraBodyMappingAndOriginalIds) {
    RotationFixture fixture(2);VIOEngine engine;
    const Eigen::Matrix3d camera_to_body=Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Matrix<double,3,3,Eigen::RowMajor> row_major=camera_to_body;
    const double zero[3]={0,0,0};
    ASSERT_TRUE(engine.configure(fixture.width,fixture.height,fixture.fx,fixture.fy,fixture.cx,fixture.cy,2,
        fixture.k1,fixture.k2,fixture.p1,fixture.p2,row_major.data(),zero,.1,.001,.01,.0001,9.81));
    engine.setTrackingParams(21,3,30,0);engine.setPnPParams(false,3);engine.setDiagnosticCapture(true);
    engine.setRotationRescue(true);
    const double rate=6*M_PI/180/.05;
    const Eigen::Vector3d omega=camera_to_body*Eigen::Vector3d(0,rate,0);
    IMUReading first{0,0,0,9.81,omega.x(),omega.y(),omega.z()};double pose[16];
    EXPECT_FALSE(engine.processFrame(fixture.source.data,fixture.width,fixture.height,&first,1,0,pose));
    const auto original=trackedPixels(VIOEngineTestAccess::tracker(engine));
    backend::EstimatorTestAccess::initialized(VIOEngineTestAccess::estimator(engine));
    std::vector<IMUReading> readings;
    for(int i=1;i<=6;++i)readings.push_back({i*.01,0,0,9.81,omega.x(),omega.y(),omega.z()});
    const Eigen::Matrix3d rotation=Eigen::AngleAxisd(-6*M_PI/180,Eigen::Vector3d::UnitY()).toRotationMatrix();
    const cv::Mat target=fixture.rotated(rotation);
    engine.processFrame(target.data,fixture.width,fixture.height,readings.data(),int(readings.size()),.05,pose);
    const auto score=scoreRotation(fixture,VIOEngineTestAccess::tracker(engine),original,rotation);
    ASSERT_GT(score.visible,60);EXPECT_GE(score.correct,score.visible*9/10);EXPECT_LE(score.wrong,score.visible/20);
    EXPECT_NE(engine.getFeatureDiagnostics().find("\"available\":true"),std::string::npos);
}

TEST(EngineContracts, IMURotationSeedRequiresInitializedCoveredConsecutiveImageEndpoints) {
    VIOEngine engine;ASSERT_TRUE(configure(engine));engine.setPnPParams(false,3);engine.setDiagnosticCapture(true);
    engine.setRotationRescue(true);
    cv::Mat blank(96,96,CV_8UC1,cv::Scalar(127));double pose[16];
    IMUReading first{0,0,0,9.81,0,1,0};
    engine.processFrame(blank.data,96,96,&first,1,0,pose);
    EXPECT_NE(engine.getFeatureDiagnostics().find("\"available\":false"),std::string::npos);
    backend::EstimatorTestAccess::initialized(VIOEngineTestAccess::estimator(engine));
    IMUReading covered[]={{.025,0,0,9.81,0,1,0},{.05,0,0,9.81,0,1,0},{.06,0,0,9.81,0,1,0}};
    engine.processFrame(blank.data,96,96,covered,3,.05,pose);
    EXPECT_NE(engine.getFeatureDiagnostics().find("\"available\":true"),std::string::npos);
    engine.processFrame(blank.data,96,96,nullptr,0,.1,pose);
    EXPECT_EQ(engine.getLastReason(),"imu_missing_future");
    EXPECT_NE(engine.getFeatureDiagnostics().find("\"available\":false"),std::string::npos);
    IMUReading resumed[]={{.07,0,0,9.81,0,1,0},{.08,0,0,9.81,0,1,0},{.09,0,0,9.81,0,1,0},
        {.1,0,0,9.81,0,1,0},{.11,0,0,9.81,0,1,0},{.12,0,0,9.81,0,1,0}};
    engine.processFrame(blank.data,96,96,resumed,6,.11,pose);
    EXPECT_NE(engine.getFeatureDiagnostics().find("\"available\":false"),std::string::npos);
    engine.processFrame(blank.data,96,96,nullptr,0,.12,pose);
    EXPECT_NE(engine.getFeatureDiagnostics().find("\"available\":true"),std::string::npos);
    const auto epoch=engine.getEpoch();engine.reset();
    IMUReading restarted[]={{.2,0,0,9.81,0,1,0},{.25,0,0,9.81,0,1,0}};
    engine.processFrame(blank.data,96,96,restarted,2,.2,pose);
    EXPECT_EQ(engine.getEpoch(),epoch+1);
    EXPECT_NE(engine.getFeatureDiagnostics().find("\"available\":false"),std::string::npos);
}

TEST(EngineContracts, RotationRescueDefaultsOffAndRequiresOptInAfterEachConfigure) {
    VIOEngine engine;
    EXPECT_FALSE(engine.getRotationRescueEnabled());
    auto epoch=engine.getEpoch();
    engine.setRotationRescue(true);
    EXPECT_TRUE(engine.getRotationRescueEnabled());EXPECT_EQ(engine.getEpoch(),epoch);
    ASSERT_TRUE(configure(engine));
    EXPECT_FALSE(engine.getRotationRescueEnabled());
    engine.setDiagnosticCapture(true);
    EXPECT_NE(engine.getFeatureDiagnostics().find("\"rotation_rescue_enabled\":0"),std::string::npos);
    epoch=engine.getEpoch();engine.setRotationRescue(true);engine.setRotationRescue(true);
    EXPECT_EQ(engine.getEpoch(),epoch);
    EXPECT_NE(engine.getFeatureDiagnostics().find("\"rotation_rescue_enabled\":1"),std::string::npos);
    engine.reset();
    EXPECT_TRUE(engine.getRotationRescueEnabled());EXPECT_EQ(engine.getEpoch(),epoch+1);
    ASSERT_TRUE(configure(engine));
    EXPECT_FALSE(engine.getRotationRescueEnabled());EXPECT_EQ(engine.getEpoch(),epoch+2);
}

TEST(EngineContracts, DisabledRotationRescueSkipsPredictionAndMidstreamToggleRetiresAllInputState) {
    VIOEngine engine;ASSERT_TRUE(configure(engine));engine.setPnPParams(false,3);engine.setDiagnosticCapture(true);
    cv::Mat image(96,96,CV_8UC1,cv::Scalar(0));
    for(int y=8;y<90;y+=16) for(int x=8;x<90;x+=16)
        cv::rectangle(image,cv::Rect(x,y,7,7),cv::Scalar(255),-1);
    IMUReading initial[]={{0,0,0,9.81,0,1,0},{.1,0,0,9.81,0,1,0}};double pose[16];
    engine.processFrame(image.data,96,96,initial,2,0,pose);
    backend::EstimatorTestAccess::initialized(VIOEngineTestAccess::estimator(engine));
    engine.processFrame(image.data,96,96,nullptr,0,.05,pose);
    ASSERT_TRUE(engine.isInitialized());ASSERT_GT(engine.getFeaturePointCount(),0);
    EXPECT_NE(engine.getFeatureDiagnostics().find("\"available\":false"),std::string::npos);
    EXPECT_NE(engine.getFeatureDiagnostics().find("\"rotation_rescue_enabled\":0"),std::string::npos);
    ASSERT_EQ(VIOEngineTestAccess::pending(engine),1u);
    const auto epoch=engine.getEpoch();
    engine.setRotationRescue(true);
    EXPECT_TRUE(engine.getRotationRescueEnabled());EXPECT_EQ(engine.getEpoch(),epoch+1);
    EXPECT_FALSE(engine.isInitialized());EXPECT_EQ(engine.getFeaturePointCount(),0);
    EXPECT_TRUE(VIOEngineTestAccess::tracker(engine).cur_pts.empty());
    EXPECT_EQ(VIOEngineTestAccess::pending(engine),0u);EXPECT_DOUBLE_EQ(engine.getIMUEndpointTimestamp(),-1);
    EXPECT_DOUBLE_EQ(engine.getFrameTimestamp(),-1);EXPECT_FALSE(engine.getPoseValid());EXPECT_FALSE(engine.getPoseFresh());
    EXPECT_EQ(engine.getLastReason(),"rotation_rescue_changed");
    engine.setRotationRescue(true);EXPECT_EQ(engine.getEpoch(),epoch+1);
    IMUReading restarted[]={{.2,0,0,9.81,0,1,0},{.3,0,0,9.81,0,1,0}};
    engine.processFrame(image.data,96,96,restarted,2,.2,pose);
    engine.setRotationRescue(false);
    EXPECT_FALSE(engine.getRotationRescueEnabled());EXPECT_EQ(engine.getEpoch(),epoch+2);
    EXPECT_TRUE(VIOEngineTestAccess::tracker(engine).cur_pts.empty());EXPECT_EQ(VIOEngineTestAccess::pending(engine),0u);
}

TEST(EngineContracts, RejectsInvalidCalibrationInsteadOfSilentFallback) {
    VIOEngine engine;
    EXPECT_FALSE(configure(engine,nullptr,nullptr,1));
    EXPECT_FALSE(configure(engine,nullptr,nullptr,2,0));
    EXPECT_FALSE(configure(engine,nullptr,nullptr,2,std::numeric_limits<double>::quiet_NaN()));
    const double reflection[9] = {-1,0,0,0,1,0,0,0,1};
    EXPECT_FALSE(configure(engine,reflection));
    const double skew[9] = {1,.1,0,0,1,0,0,0,1};
    EXPECT_FALSE(configure(engine,skew));
}

TEST(EngineContracts, ConfigureCopiesAliasedRowMajorInputBeforeWritingGlobalConfig) {
    utility::g_config.camera.r_ic = (Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(.9,Eigen::Vector3d::UnitZ())).toRotationMatrix();
    const Eigen::Matrix3d old = utility::g_config.camera.r_ic;
    // Deliberately interpret the existing column-major buffer under the public row-major contract.
    const Eigen::Matrix3d expected = old.transpose();
    VIOEngine engine;
    ASSERT_TRUE(configure(engine,utility::g_config.camera.r_ic.data(),utility::g_config.camera.t_ic.data()));
    EXPECT_TRUE(utility::g_config.camera.r_ic.isApprox(expected,1e-12));
}

TEST(EngineContracts, ResetDropsTrackedImageAndIds) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    cv::Mat checker(96,96,CV_8UC1,cv::Scalar(0));
    for(int y=8;y<90;y+=16) for(int x=8;x<90;x+=16)
        cv::rectangle(checker,cv::Rect(x,y,7,7),cv::Scalar(255),-1);
    double pose[16];
    engine.processFrame(checker.data,96,96,nullptr,0,1.0,pose);
    engine.processFrame(checker.data,96,96,nullptr,0,1.05,pose);
    ASSERT_GT(engine.getFeaturePointCount(),0);
    engine.reset();
    EXPECT_EQ(engine.getFeaturePointCount(),0);
}

TEST(EngineContracts, TrackerOwnsRetainedImageWhenEqualizationIsDisabled) {
    utility::g_config.camera.col = utility::g_config.camera.row = 96;
    utility::g_config.feature_tracker.equalize = 0;
    utility::g_config.feature_tracker.fisheye = 0;
    frontend::FeatureTracker tracker;
    ASSERT_TRUE(tracker.setIntrinsicParameter(2,96,96,100,100,48,48,0,0,0,0));
    cv::Mat reusable(96,96,CV_8UC1,cv::Scalar(17));
    tracker.detectAndTrack(reusable,1);
    reusable.setTo(255);
    EXPECT_EQ(tracker.cur_img.at<uint8_t>(0,0),17);
}

TEST(EngineContracts, FirstUniformTargetDropsLKTracksAndRecoversWithNewIds) {
    struct RestoreConfig {
        utility::Config saved = utility::g_config;
        ~RestoreConfig() { utility::g_config = saved; }
    } restore;
    utility::g_config.camera.col = utility::g_config.camera.row = 96;
    utility::g_config.feature_tracker.fisheye = 0;
    utility::g_config.feature_tracker.max_cnt = 150;
    utility::g_config.feature_tracker.min_dist = 10;
    for(int equalize : {0,1}) for(int uniform : {0,127,255}) for(bool backend_frame : {false,true}) {
        if(equalize && uniform==255) continue; // This seed loses LK tracks under CLAHE before the target transition.
        SCOPED_TRACE(::testing::Message()<<"equalize="<<equalize<<" uniform="<<uniform<<" backend="<<backend_frame);
        utility::g_config.feature_tracker.equalize = equalize;
        frontend::FeatureTracker tracker;
        ASSERT_TRUE(tracker.setIntrinsicParameter(2,96,96,100,100,48,48,0,0,0,0));
        const auto camera = tracker.m_camera;
        cv::Mat textured(96,96,CV_8UC1,cv::Scalar(uniform));
        const int contrast = uniform<239 ? uniform+16 : uniform-16;
        for(int y=8;y<90;y+=16) for(int x=8;x<90;x+=16)
            cv::rectangle(textured,cv::Rect(x,y,7,7),cv::Scalar(contrast),-1);
        tracker.detectAndTrack(textured,1);
        for(unsigned int i=0;tracker.updateID(i);++i) {}
        ASSERT_GT(tracker.cur_pts.size(),0u);
        tracker.detectAndTrack(textured,1.05,false); // Seed the real LK reference, including normals/history.
        ASSERT_GT(tracker.cur_pts.size(),0u);
        const int next_id = frontend::FeatureTracker::n_id;

        // An exact constant image has zero gradient everywhere, independently of LK's previous-image status.
        cv::Mat uniform_image(96,96,CV_8UC1,cv::Scalar(uniform));
        ASSERT_NO_THROW(tracker.detectAndTrack(uniform_image,1.1,backend_frame));
        EXPECT_TRUE(tracker.n_pts.empty());
        EXPECT_TRUE(tracker.prev_pts.empty()); EXPECT_TRUE(tracker.cur_pts.empty()); EXPECT_TRUE(tracker.next_pts.empty());
        EXPECT_TRUE(tracker.prev_undistorted_pts.empty()); EXPECT_TRUE(tracker.cur_undistorted_pts.empty());
        EXPECT_TRUE(tracker.pts_velocity.empty()); EXPECT_TRUE(tracker.ids.empty()); EXPECT_TRUE(tracker.track_cnt.empty());
        EXPECT_TRUE(tracker.cur_undistorted_pts_map.empty()); EXPECT_TRUE(tracker.prev_undistorted_pts_map.empty());
        EXPECT_EQ(tracker.m_camera,camera);
        EXPECT_DOUBLE_EQ(tracker.cur_time,1.1); EXPECT_DOUBLE_EQ(tracker.prev_time,1.1);
        EXPECT_EQ(frontend::FeatureTracker::n_id,next_id);

        ASSERT_NO_THROW(tracker.detectAndTrack(textured,1.15));
        ASSERT_GT(tracker.cur_pts.size(),0u);
        for(unsigned int i=0;tracker.updateID(i);++i) {}
        for(int id : tracker.ids) EXPECT_GE(id,next_id);
        for(int count : tracker.track_cnt) EXPECT_EQ(count,1);
        EXPECT_EQ(tracker.cur_pts.size(),tracker.ids.size());
        EXPECT_EQ(tracker.cur_pts.size(),tracker.track_cnt.size());
        EXPECT_EQ(tracker.cur_pts.size(),tracker.cur_undistorted_pts.size());
        EXPECT_EQ(tracker.cur_pts.size(),tracker.pts_velocity.size());
        EXPECT_TRUE(tracker.prev_pts.empty()); EXPECT_TRUE(tracker.prev_undistorted_pts.empty());
        ASSERT_NO_THROW(tracker.detectAndTrack(textured,1.2,false));
        EXPECT_GT(tracker.cur_pts.size(),0u);
        EXPECT_EQ(tracker.cur_pts.size(),tracker.ids.size());
        EXPECT_EQ(tracker.cur_pts.size(),tracker.track_cnt.size());
        EXPECT_EQ(tracker.cur_pts.size(),tracker.cur_undistorted_pts.size());
        EXPECT_EQ(tracker.cur_pts.size(),tracker.pts_velocity.size());
        EXPECT_LE(tracker.prev_pts.size(),tracker.cur_pts.size());
        EXPECT_LE(tracker.prev_undistorted_pts.size(),tracker.cur_pts.size());
    }
}

TEST(EngineContracts, UniformTargetReachesEmptyFeatureBoundaryAfterRealLKSeed) {
    struct RestoreConfig {
        utility::Config saved = utility::g_config;
        ~RestoreConfig() { utility::g_config = saved; }
    } restore;
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    utility::g_config.feature_tracker.equalize = 0;
    utility::g_config.feature_tracker.min_dist = 10;
    cv::Mat textured(96,96,CV_8UC1,cv::Scalar(127));
    for(int y=8;y<90;y+=16) for(int x=8;x<90;x+=16)
        cv::rectangle(textured,cv::Rect(x,y,7,7),cv::Scalar(143),-1);
    IMUReading imu[]={{1,0,0,9.81,0,0,0},{1.25,0,0,9.81,0,0,0}};
    double output[16];
    EXPECT_FALSE(engine.processFrame(textured.data,96,96,imu,2,1,output));
    EXPECT_FALSE(engine.processFrame(textured.data,96,96,nullptr,0,1.05,output));
    ASSERT_GT(engine.getFeaturePointCount(),0);
    const auto epoch = engine.getEpoch();
    cv::Mat uniform(96,96,CV_8UC1,cv::Scalar(127));
    EXPECT_FALSE(engine.processFrame(uniform.data,96,96,nullptr,0,1.1,output));
    EXPECT_EQ(engine.getFeaturePointCount(),0);
    EXPECT_EQ(engine.getLastReason(),"empty_features");
    EXPECT_FALSE(engine.getPoseValid()); EXPECT_FALSE(engine.getPoseFresh());
    EXPECT_DOUBLE_EQ(engine.getPoseTimestamp(),-1);
    EXPECT_DOUBLE_EQ(engine.getFrameTimestamp(),1.1);
    double map[3]; EXPECT_EQ(engine.getMapPoints(map,1),0);
    for(double value : output) EXPECT_TRUE(std::isnan(value));
    EXPECT_EQ(engine.getEpoch(),epoch);
    EXPECT_FALSE(engine.processFrame(textured.data,96,96,nullptr,0,1.15,output));
    EXPECT_EQ(engine.getFeaturePointCount(),0); // Newly detected points require a following observation.
    EXPECT_FALSE(engine.processFrame(textured.data,96,96,nullptr,0,1.2,output));
    EXPECT_GT(engine.getFeaturePointCount(),0);
    EXPECT_EQ(engine.getLastReason(),"initializing");
    EXPECT_EQ(engine.getEpoch(),epoch);
}

TEST(EngineContracts, AnalyticIMUIntegralAcrossRatesPhasesAndCarriedFutureSamples) {
    for (int imu_rate : {60,100,200}) for (int camera_rate : {10,20,30,60})
    for (double phase : {0.0,0.37}) for (bool rotation_only : {false,true}) for (bool ramp : {false,true}) {
        SCOPED_TRACE(::testing::Message()<<imu_rate<<"/"<<camera_rate<<" phase="<<phase<<" rotation="<<rotation_only<<" ramp="<<ramp);
        VIOEngine engine;
        ASSERT_TRUE(configure(engine));
        std::vector<IMUReading> source;
        source.push_back({0,rotation_only?0.0:(ramp?1.0:2.0),0,0,0,0,rotation_only?(ramp?.3:.4):0.0});
        for(int i=1;i<=imu_rate+20;++i) {
            const double time=(i+phase)/imu_rate;
            source.push_back({time,rotation_only?0.0:(ramp?1+2*time:2.0),0,0,0,0,rotation_only?(ramp?.3+.2*time:.4):0.0});
        }
        size_t sent=0;
        const auto send=[&](double time) {
            std::vector<IMUReading> batch;
            while(sent<source.size() && source[sent].timestamp<=time+.1) batch.push_back(source[sent++]);
            return VIOEngineTestAccess::integrate(engine,batch,time);
        };
        ASSERT_TRUE(send(0));
        ASSERT_GT(VIOEngineTestAccess::pending(engine),1u);
        common::ImageData observations;
        for(int i=0;i<12;++i) { Eigen::Matrix<double,7,1> value; value<<i*.01,0,1,48+i,48,0,0; observations[i]=value; }
        VIOEngineTestAccess::estimator(engine).processImage(observations,0);
        for(int i=1;i<camera_rate;++i) {
            const double time=(i+.19)/camera_rate;
            ASSERT_TRUE(send(time));
            EXPECT_NEAR(VIOEngineTestAccess::cursor(engine),time,1e-12);
            if(rotation_only) EXPECT_NEAR(VIOEngineTestAccess::angularVelocity(engine).z(),ramp?.3+.2*time:.4,1e-12);
            else EXPECT_NEAR(VIOEngineTestAccess::acceleration(engine).x(),ramp?1+2*time:2.0,1e-12);
        }
        ASSERT_TRUE(send(1));
        const auto& integration=VIOEngineTestAccess::estimator(engine).sliding_window_[1].pre_integration;
        ASSERT_TRUE(integration);
        EXPECT_NEAR(integration->sum_dt,1,1e-9);
        if(rotation_only) {
            // Fixed-axis closed form integral of .3+.2t rad/s is .4 rad.
            EXPECT_NEAR(Eigen::AngleAxisd(integration->delta_q).angle(),.4,1e-5);
        } else {
            EXPECT_NEAR(integration->delta_v.x(),2,1e-10);
            // Midpoint position truncation is bounded by max step cubed, not a copied implementation.
            EXPECT_NEAR(integration->delta_p.x(),ramp?5.0/6.0:1.0,1e-4);
        }
    }
}

TEST(EngineContracts, ConstantIMUSeedAndNoNewSamplesHaveExactCoverage) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    ASSERT_TRUE(VIOEngineTestAccess::integrate(engine,{{0,2,0,0,0,0,0},{.2,2,0,0,0,0,0}},0));
    common::ImageData observations;
    Eigen::Matrix<double,7,1> ray; ray<<0,0,1,48,48,0,0; observations[1]=ray;
    VIOEngineTestAccess::estimator(engine).processImage(observations,0);
    for(double time : {.01,.03,.05,.1,.2}) ASSERT_TRUE(VIOEngineTestAccess::integrate(engine,{},time));
    const auto& integration=VIOEngineTestAccess::estimator(engine).sliding_window_[1].pre_integration;
    ASSERT_TRUE(integration);
    EXPECT_NEAR(integration->sum_dt,.2,1e-12);
    EXPECT_NEAR(integration->delta_v.x(),.4,1e-12);
    EXPECT_NEAR(integration->delta_p.x(),.04,1e-12);
    EXPECT_EQ(VIOEngineTestAccess::pending(engine),0u);
}

TEST(EngineContracts, CameraContinuationCannotExtrapolatePastLastRealIMUDuringSensorPause) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    cv::Mat image(96,96,CV_8UC1,cv::Scalar(0));
    for(int y=8;y<90;y+=16) for(int x=8;x<90;x+=16)
        cv::rectangle(image,cv::Rect(x,y,7,7),cv::Scalar(255),-1);
    double output[16];
    IMUReading samples[]={{0,0,0,9.81,0,0,0},{.1,0,0,9.81,0,0,0}};
    engine.processFrame(image.data,96,96,samples,2,0,output);
    engine.processFrame(image.data,96,96,nullptr,0,.1,output);
    VIOEngineTestAccess::estimator(engine).solver_flag_=common::SolverFlag::NON_LINEAR;
    ASSERT_TRUE(VIOEngineTestAccess::pose(engine,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),.1,output));
    const auto epoch=engine.getEpoch();
    for(int i=3;i<=20;++i) {
        const double timestamp=i*.05;
        EXPECT_FALSE(engine.processFrame(image.data,96,96,nullptr,0,timestamp,output));
        EXPECT_EQ(engine.getLastReason(),"imu_missing_future");
        EXPECT_EQ(engine.getStatusCode(),static_cast<int>(VIOStatus::LOST));
        EXPECT_FALSE(engine.getPoseFresh());
        EXPECT_FALSE(engine.getPoseValid());
        EXPECT_EQ(engine.getPoseTimestamp(),-1);
        EXPECT_DOUBLE_EQ(engine.getFrameTimestamp(),timestamp);
        EXPECT_DOUBLE_EQ(VIOEngineTestAccess::cursor(engine),.1);
        EXPECT_EQ(engine.getEpoch(),epoch);
        for(double value:output) EXPECT_TRUE(std::isnan(value));
    }
    IMUReading resumed={.7,0,0,9.81,0,0,0};
    EXPECT_FALSE(engine.processFrame(image.data,96,96,&resumed,1,1.05,output));
    EXPECT_EQ(engine.getLastReason(),"imu_gap");
    EXPECT_EQ(engine.getEpoch(),epoch+1);
    EXPECT_EQ(VIOEngineTestAccess::cursor(engine),-1);
}

TEST(EngineContracts, DuplicateOutOfOrderAndNonfiniteIMUDoNotMoveCursorOrDropCarry) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    ASSERT_TRUE(VIOEngineTestAccess::integrate(engine,{{0,1,0,0,0,0,0},{.1,2,0,0,0,0,0}},.03));
    const auto pending=VIOEngineTestAccess::pending(engine);
    for(double time : {.1,.05}) {
        EXPECT_FALSE(VIOEngineTestAccess::integrate(engine,{{time,999,0,0,0,0,0}},.04));
        EXPECT_EQ(engine.getLastReason(),"imu_out_of_order");
        EXPECT_NEAR(VIOEngineTestAccess::cursor(engine),.03,1e-12);
        EXPECT_EQ(VIOEngineTestAccess::pending(engine),pending);
    }
    EXPECT_FALSE(VIOEngineTestAccess::integrate(engine,{{.2,std::numeric_limits<double>::quiet_NaN(),0,0,0,0,0}},.04));
    ASSERT_TRUE(VIOEngineTestAccess::integrate(engine,{},.08));
    EXPECT_NEAR(VIOEngineTestAccess::acceleration(engine).x(),1.8,1e-12);
}

TEST(EngineContracts, GapAndOverflowInvalidateAllStateAndIncrementEpoch) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    auto epoch=engine.getEpoch();
    EXPECT_FALSE(VIOEngineTestAccess::integrate(engine,{{0,1,0,0,0,0,0},{1,1,0,0,0,0,0}},.1));
    EXPECT_EQ(engine.getLastReason(),"imu_gap");
    EXPECT_EQ(engine.getEpoch(),epoch+1);
    EXPECT_EQ(VIOEngineTestAccess::cursor(engine),-1);
    EXPECT_EQ(VIOEngineTestAccess::pending(engine),0u);
    std::vector<IMUReading> readings;
    for(int i=0;i<4096;++i) readings.push_back({i*.00001,1,0,0,0,0,0});
    ASSERT_TRUE(VIOEngineTestAccess::integrate(engine,readings,0));
    epoch=engine.getEpoch();
    EXPECT_FALSE(VIOEngineTestAccess::integrate(engine,{{.05,1,0,0,0,0,0},{.06,1,0,0,0,0,0}},.001));
    EXPECT_EQ(engine.getLastReason(),"imu_overflow");
    EXPECT_EQ(engine.getEpoch(),epoch+1);
}

TEST(EngineContracts, MissingVisualUpdatesCannotAccumulateUnboundedIMUHistory) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    std::vector<IMUReading> readings;
    for(int i=0;i<4096;++i) readings.push_back({i*.000001,1,0,0,0,0,0});
    EXPECT_FALSE(VIOEngineTestAccess::integrate(engine,readings,.0040955));
    EXPECT_EQ(engine.getLastReason(),"imu_missing_future");
    const auto epoch=engine.getEpoch();
    EXPECT_FALSE(VIOEngineTestAccess::integrate(engine,{{.004096,1,0,0,0,0,0},{.004097,1,0,0,0,0,0}},.004097));
    EXPECT_EQ(engine.getLastReason(),"imu_history_overflow");
    EXPECT_EQ(engine.getEpoch(),epoch+1);
    EXPECT_EQ(VIOEngineTestAccess::pending(engine),0u);
}

TEST(EngineContracts, FrameBoundsMonotonicityFreshnessAndResetAreExplicit) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    std::vector<uint8_t> blank(96*96,0);
    double output[16];
    const auto epoch=engine.getEpoch();
    EXPECT_FALSE(engine.processFrame(nullptr,96,96,nullptr,0,1,output));
    EXPECT_EQ(engine.getLastReason(),"invalid_frame");
    EXPECT_FALSE(engine.processFrame(blank.data(),95,96,nullptr,0,1,output));
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,nullptr,-1,1,output));
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,nullptr,1,1,output));
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,nullptr,4097,1,output));
    EXPECT_EQ(engine.getEpoch(),epoch);
    IMUReading imu[]={{.99,0,0,9.81,0,0,0},{1.1,0,0,9.81,0,0,0}};
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,imu,2,1,output));
    EXPECT_EQ(engine.getLastReason(),"empty_features");
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,nullptr,0,1,output));
    EXPECT_EQ(engine.getLastReason(),"frame_out_of_order");
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,nullptr,0,2,output));
    EXPECT_EQ(engine.getLastReason(),"frame_gap");
    EXPECT_EQ(engine.getEpoch(),epoch+1);
    engine.reset();
    EXPECT_EQ(engine.getEpoch(),epoch+2);
    EXPECT_EQ(engine.getPoseTimestamp(),-1);
    EXPECT_EQ(engine.getFrameTimestamp(),-1);
    EXPECT_FALSE(engine.getPoseFresh());
    EXPECT_FALSE(engine.getPoseValid());
    for(double value:output) EXPECT_TRUE(std::isnan(value));
}

TEST(EngineContracts, EstimatorResetGenerationClearsCarryTrackerAndAdvancesEpochOnNextFrame) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    ASSERT_TRUE(VIOEngineTestAccess::integrate(engine,{{0,1,0,0,0,0,0},{.2,1,0,0,0,0,0}},.05));
    ASSERT_EQ(VIOEngineTestAccess::pending(engine),1u);
    const auto epoch=engine.getEpoch();
    VIOEngineTestAccess::estimator(engine).reset();
    std::vector<uint8_t> blank(96*96,0);
    double output[16];
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,nullptr,0,.1,output));
    EXPECT_EQ(engine.getLastReason(),"estimator_reset");
    EXPECT_EQ(engine.getEpoch(),epoch+1);
    EXPECT_EQ(engine.getFrameTimestamp(),.1);
    EXPECT_EQ(engine.getPoseTimestamp(),-1);
    EXPECT_EQ(VIOEngineTestAccess::cursor(engine),-1);
    EXPECT_EQ(VIOEngineTestAccess::pending(engine),0u);
    EXPECT_EQ(engine.getFeaturePointCount(),0);
    for(double value:output) EXPECT_TRUE(std::isnan(value));
}

TEST(EngineContracts, MeasuredQualityWarningDoesNotChangeNumericalPoseUsability) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    backend::SolverDiagnostics diagnostics;
    diagnostics.usable=true;
    diagnostics.iterations=7;
    diagnostics.terminationType=1;
    diagnostics.qualityReason="acc_bias_threshold";
    VIOEngineTestAccess::solver(engine,diagnostics);
    double output[16];
    ASSERT_TRUE(VIOEngineTestAccess::pose(engine,Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),1,output));
    EXPECT_TRUE(engine.getPoseFresh());
    EXPECT_TRUE(engine.getPoseValid());
    EXPECT_EQ(engine.getLastReason(),"tracking:acc_bias_threshold");
    EXPECT_EQ(engine.getLastSolverIterations(),7);
    EXPECT_EQ(engine.getLastSolverTermination(),"NO_CONVERGENCE");
    EXPECT_FALSE(engine.processFrame(nullptr,96,96,nullptr,0,1.1,output));
    EXPECT_EQ(engine.getLastSolverIterations(),0);
    EXPECT_EQ(engine.getLastSolverTermination(),"not_run");
}

TEST(EngineContracts, ExplicitExecutionProfileAppliesAndResetsActualOpenCVState) {
    {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    EXPECT_EQ(engine.getExecutionSeed(),-1);
    cv::setRNGSeed(1234);
    engine.setExecutionParams(0,1);
    EXPECT_EQ(engine.getExecutionSeed(),0);
    EXPECT_EQ(engine.getCVThreadCount(),1);
    EXPECT_EQ(cv::theRNG().state,0xffffffffu);
    cv::theRNG().next();
    ASSERT_NE(cv::theRNG().state,0xffffffffu);
    engine.reset();
    EXPECT_EQ(cv::theRNG().state,0xffffffffu);
    EXPECT_EQ(engine.getCVThreadCount(),1);
    engine.setExecutionParams(-1,0);
    EXPECT_EQ(engine.getExecutionSeed(),0);
    EXPECT_EQ(engine.getCVThreadCount(),1);
    }
    // Leave scheduler release to the engine's scope exit; full LSan checks the lifetime.
    std::atomic<int> processed{0};
    cv::parallel_for_(cv::Range(0,4),[&](const cv::Range& range){ processed.fetch_add(range.end-range.start); });
    EXPECT_EQ(processed.load(),4);
}

TEST(EngineContracts, CameraPoseUsesBodyRotationAndLeverArmAndBlankFrameCannotReuseIt) {
    const Eigen::Matrix3d rotation=(Eigen::AngleAxisd(.5,Eigen::Vector3d::UnitX())*
                                    Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitY())).toRotationMatrix();
    Eigen::Matrix<double,3,3,Eigen::RowMajor> row_major=rotation;
    const double translation[3]={.4,-.2,.1};
    VIOEngine engine;
    ASSERT_TRUE(configure(engine,row_major.data(),translation));
    const Eigen::Matrix3d body_rotation=Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    double output[16];
    ASSERT_TRUE(VIOEngineTestAccess::pose(engine,Eigen::Vector3d::Zero(),body_rotation,1,output));
    const Eigen::Matrix4d actual=Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::RowMajor>>(output);
    EXPECT_TRUE((actual.block<3,3>(0,0).isApprox(body_rotation*rotation,1e-12)));
    EXPECT_TRUE((actual.block<3,1>(0,3).isApprox(Eigen::Vector3d(.2,.4,.1),1e-12)));
    VIOEngineTestAccess::estimator(engine).solver_flag_=common::SolverFlag::NON_LINEAR;
    std::vector<uint8_t> blank(96*96,0);
    IMUReading imu[]={{1,0,0,9.81,0,0,0},{1.2,0,0,9.81,0,0,0}};
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,imu,2,1.05,output));
    EXPECT_EQ(engine.getLastReason(),"empty_features");
    EXPECT_EQ(engine.getStatusCode(),static_cast<int>(VIOStatus::LOST));
    EXPECT_EQ(engine.getPoseTimestamp(),-1);
    double map[3]; EXPECT_EQ(engine.getMapPoints(map,1),0);
    for(double value:output) EXPECT_TRUE(std::isnan(value));
}

TEST(EngineContracts, RansacPrefiltersBothRayEndpointsBeforeDivisionAndPreservesAlignedMetadata) {
    utility::g_config.camera.col=utility::g_config.camera.row=96;
    utility::g_config.camera.focal_length=100;
    utility::g_config.feature_tracker.f_threshold_edge_factor=0;
    frontend::FeatureTracker tracker;
    tracker.m_camera=std::make_shared<BoundaryRayCamera>();
    for(int i=0;i<30;++i) {
        cv::Point2f current(20+(i%8)*5,20+(i/8)*5),next(current.x+.5f,current.y+.25f);
        if(i<3) current.x=i+1;
        if(i>=3 && i<6) next.x=i-2;
        tracker.cur_pts.push_back(current);tracker.next_pts.push_back(next);
        tracker.prev_pts.emplace_back(i,100+i);
        tracker.cur_undistorted_pts.emplace_back(i,200+i);
        tracker.prev_undistorted_pts.emplace_back(i,300+i);
        tracker.pts_velocity.emplace_back(i,400+i);
        tracker.ids.push_back(i);tracker.track_cnt.push_back(i+2);
    }
    // Six pairs have zero/negative/NaN rays. The 24 valid pairs do not reach the existing 30-point RANSAC gate.
    ASSERT_NO_THROW(tracker.rejectWithFundamentalMatrix());
    ASSERT_EQ(tracker.ids.size(),24u);
    EXPECT_EQ(tracker.cur_pts.size(),24u);EXPECT_EQ(tracker.next_pts.size(),24u);
    EXPECT_EQ(tracker.prev_pts.size(),24u);EXPECT_EQ(tracker.cur_undistorted_pts.size(),24u);
    EXPECT_EQ(tracker.prev_undistorted_pts.size(),24u);EXPECT_EQ(tracker.pts_velocity.size(),24u);
    EXPECT_EQ(tracker.track_cnt.size(),24u);
    for(size_t j=0;j<tracker.ids.size();++j) {
        const int original=static_cast<int>(j)+6;
        EXPECT_EQ(tracker.ids[j],original);
        EXPECT_EQ(tracker.track_cnt[j],original+2);
        EXPECT_EQ(tracker.prev_pts[j],cv::Point2f(original,100+original));
        EXPECT_EQ(tracker.cur_undistorted_pts[j],cv::Point2f(original,200+original));
        EXPECT_EQ(tracker.prev_undistorted_pts[j],cv::Point2f(original,300+original));
        EXPECT_EQ(tracker.pts_velocity[j],cv::Point2f(original,400+original));
    }
}

TEST(EngineContracts, InvalidCurrentRaysPruneHistoryBeforeTheFollowingTrackingFrame) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    utility::g_config.feature_tracker.equalize=0;
    frontend::FeatureTracker tracker;
    const auto camera=std::make_shared<BoundaryRayCamera>();
    tracker.m_camera=camera;
    cv::Mat image(96,96,CV_8UC1,cv::Scalar(0));
    for(int y=8;y<90;y+=16)for(int x=8;x<90;x+=16)
        cv::rectangle(image,cv::Rect(x,y,7,7),cv::Scalar(255),-1);
    tracker.detectAndTrack(image,1);
    for(unsigned int i=0;tracker.updateID(i);++i){}
    const size_t before=tracker.cur_pts.size();
    ASSERT_GT(before,0u);
    camera->reject_right_half=true;
    tracker.detectAndTrack(image,1.05);
    ASSERT_GT(tracker.cur_pts.size(),0u);
    ASSERT_LT(tracker.cur_pts.size(),before);
    EXPECT_LE(tracker.prev_pts.size(),tracker.cur_pts.size());
    EXPECT_LE(tracker.prev_undistorted_pts.size(),tracker.cur_pts.size());
    ASSERT_NO_THROW(tracker.detectAndTrack(image,1.1));
    EXPECT_EQ(tracker.cur_pts.size(),tracker.ids.size());
    EXPECT_EQ(tracker.cur_pts.size(),tracker.cur_undistorted_pts.size());
    EXPECT_EQ(tracker.cur_pts.size(),tracker.pts_velocity.size());
    EXPECT_EQ(tracker.cur_pts.size(),tracker.track_cnt.size());
}

TEST(EngineContracts, EqualPriorityMaskUsesStableIdsWithoutChangingLongerTrackPriority) {
    utility::g_config.camera.col=utility::g_config.camera.row=96;
    utility::g_config.feature_tracker.fisheye=0;
    utility::g_config.feature_tracker.min_dist=10;
    frontend::FeatureTracker tracker;
    tracker.next_pts={{30,30},{35,30},{60,60},{65,60}};
    tracker.ids={5,2,9,1};
    tracker.track_cnt={4,4,5,4};
    tracker.setMask();
    // Longest track9 wins its nearby group; equal-length group keeps lower id2 instead of input-order id5.
    EXPECT_EQ(tracker.ids,(std::vector<int>{9,2}));
    EXPECT_EQ(tracker.track_cnt,(std::vector<int>{5,4}));
    EXPECT_EQ(tracker.next_pts,(std::vector<cv::Point2f>{{60,60},{35,30}}));
}

TEST(EngineContracts, DiagnosticCaptureIsOptInAndEmitsJsonFixtureWithoutChangingFrameValidity) {
    VIOEngine engine;
    ASSERT_TRUE(configure(engine));
    EXPECT_EQ(engine.getFeatureDiagnostics(),"{\"enabled\":false}");
    engine.setDiagnosticCapture(true);
    cv::Mat image(96,96,CV_8UC1,cv::Scalar(0));
    for(int y=8;y<90;y+=16)for(int x=8;x<90;x+=16)
        cv::rectangle(image,cv::Rect(x,y,7,7),cv::Scalar(255),-1);
    double output[16];
    IMUReading imu[]={{0,0,0,9.81,0,0,0},{.1,0,0,9.81,0,0,0}};
    EXPECT_FALSE(engine.processFrame(image.data,96,96,imu,2,0,output));
    const auto snapshot=engine.getFeatureDiagnostics();
    EXPECT_NE(snapshot.find("\"enabled\":true"),std::string::npos);
    EXPECT_NE(snapshot.find("\"clahe_image\""),std::string::npos);
    EXPECT_NE(snapshot.find("\"mask_sorted\""),std::string::npos);
    EXPECT_NE(snapshot.find("\"gftt_new\""),std::string::npos);
    EXPECT_NE(snapshot.find("projection_sqrt_info_00"),std::string::npos);
    EXPECT_FALSE(engine.getPoseValid());
    const auto directory=std::filesystem::path(__FILE__).parent_path().parent_path()/"build/refactor-evidence/engine-green";
    std::filesystem::create_directories(directory);
    std::ofstream(directory/"diagnostic-fixture.json")<<snapshot;
    engine.setDiagnosticCapture(false);
    EXPECT_EQ(engine.getFeatureDiagnostics(),"{\"enabled\":false}");
}

TEST(EngineContracts, ExperimentalObservationPolicyChangesRetireHistoryAndPreserveProfileCalibration) {
    RestorePolicyConfig restore;VIOEngine engine;
    EXPECT_EQ(engine.getExperimentalObservationPolicy(),0);
    EXPECT_EQ(engine.getEffectiveExperimentalObservationPolicy(),-1);
    EXPECT_EQ(engine.getFeatureDiagnostics(),"{\"enabled\":false}");
    const double translation[3]={.03,-.02,.01};ASSERT_TRUE(configure(engine,nullptr,translation,2,113));
    engine.setExecutionParams(17,1);engine.setMobileParams(.04,7,48);engine.setTrackingParams(15,2,7,.5);
    engine.setPnPParams(false,3);engine.setBenchmarkSolverProfile(true);engine.setRotationRescue(true);engine.setDiagnosticCapture(true);
    for(int next_policy:{1,2,0}) {
        const int current_policy=engine.getExperimentalObservationPolicy();
        cv::Mat image(96,96,CV_8UC1,cv::Scalar(0));
        for(int y=8;y<90;y+=16)for(int x=8;x<90;x+=16)cv::rectangle(image,cv::Rect(x,y,7,7),cv::Scalar(255),-1);
        IMUReading imu[]={{0,0,0,9.81,0,0,0},{.1,0,0,9.81,0,0,0}};double pose[16];
        engine.processFrame(image.data,96,96,imu,2,0,pose);
        // Public count requires retained tracks (age>1), so perform real LK on a second image.
        engine.processFrame(image.data,96,96,nullptr,0,.05,pose);
        ASSERT_GT(engine.getFeaturePointCount(),0);ASSERT_EQ(VIOEngineTestAccess::pending(engine),1u);
        ASSERT_GT(backend::EstimatorTestAccess::imageCount(VIOEngineTestAccess::estimator(engine)),0u);
        ASSERT_TRUE(populatePolicyBackend(engine));
        auto& old_estimator=VIOEngineTestAccess::estimator(engine);
        ASSERT_TRUE(backend::EstimatorTestAccess::optimizer(old_estimator).hasMarginalizationPrior());
        EXPECT_EQ(backend::EstimatorTestAccess::optimizer(old_estimator).getLastSolverDiagnostics().currentVisualFactors,current_policy==0 ? 8 : 7);
        const auto epoch=engine.getEpoch();
        const auto calibration=utility::g_config.camera;
        const auto* camera=VIOEngineTestAccess::tracker(engine).m_camera.get();
        for(int invalid:{-1,3,std::numeric_limits<int>::max()}) {
            EXPECT_THROW(engine.setExperimentalObservationPolicy(invalid),std::invalid_argument);
            EXPECT_THROW(old_estimator.setExperimentalObservationPolicy(invalid),std::invalid_argument);
            EXPECT_EQ(engine.getEpoch(),epoch);EXPECT_EQ(engine.getExperimentalObservationPolicy(),current_policy);
            EXPECT_EQ(engine.getEffectiveExperimentalObservationPolicy(),current_policy);
            EXPECT_EQ(backend::EstimatorTestAccess::features(old_estimator).feature_bank_.size(),8u);
            EXPECT_TRUE(backend::EstimatorTestAccess::optimizer(old_estimator).hasMarginalizationPrior());
            EXPECT_EQ(VIOEngineTestAccess::pending(engine),1u);EXPECT_EQ(VIOEngineTestAccess::tracker(engine).m_camera.get(),camera);
        }
        engine.setExperimentalObservationPolicy(current_policy);
        EXPECT_EQ(engine.getEpoch(),epoch);EXPECT_TRUE(backend::EstimatorTestAccess::optimizer(old_estimator).hasMarginalizationPrior());
        engine.setExperimentalObservationPolicy(next_policy);
        EXPECT_EQ(engine.getEpoch(),epoch+1);EXPECT_EQ(engine.getExperimentalObservationPolicy(),next_policy);
        EXPECT_EQ(engine.getEffectiveExperimentalObservationPolicy(),next_policy);
        auto& fresh=VIOEngineTestAccess::estimator(engine);
        EXPECT_TRUE(backend::EstimatorTestAccess::features(fresh).feature_bank_.empty());
        EXPECT_FALSE(backend::EstimatorTestAccess::optimizer(fresh).hasMarginalizationPrior());
        EXPECT_EQ(backend::EstimatorTestAccess::imageCount(fresh),0u);
        EXPECT_EQ(engine.getFeaturePointCount(),0);EXPECT_EQ(VIOEngineTestAccess::pending(engine),0u);
        EXPECT_DOUBLE_EQ(engine.getFrameTimestamp(),-1);EXPECT_DOUBLE_EQ(engine.getIMUEndpointTimestamp(),-1);
        EXPECT_FALSE(engine.getPoseFresh());EXPECT_FALSE(engine.getPoseValid());
        EXPECT_EQ(engine.getExecutionSeed(),17);EXPECT_EQ(engine.getCVThreadCount(),1);
        EXPECT_TRUE(engine.getBenchmarkSolverProfile());EXPECT_TRUE(engine.getRotationRescueEnabled());
        EXPECT_EQ(VIOEngineTestAccess::tracker(engine).m_camera.get(),camera);
        EXPECT_DOUBLE_EQ(utility::g_config.camera.fx,calibration.fx);
        EXPECT_EQ(utility::g_config.camera.r_ic,calibration.r_ic);EXPECT_EQ(utility::g_config.camera.t_ic,calibration.t_ic);
        EXPECT_DOUBLE_EQ(utility::g_config.estimator.solver_time,.04);EXPECT_EQ(utility::g_config.estimator.num_iterations,7);
        EXPECT_EQ(utility::g_config.feature_tracker.lk_window_size,15);EXPECT_EQ(utility::g_config.feature_tracker.min_dist,7);
        const auto diagnostics=engine.getFeatureDiagnostics();
        EXPECT_NE(diagnostics.find("\"experimental_observation_policy_selected\":"+std::to_string(next_policy)),std::string::npos);
        EXPECT_NE(diagnostics.find("\"experimental_observation_policy_effective\":"+std::to_string(next_policy)),std::string::npos);
        engine.reset();EXPECT_EQ(engine.getEpoch(),epoch+2);
        EXPECT_EQ(engine.getExperimentalObservationPolicy(),next_policy);EXPECT_EQ(engine.getEffectiveExperimentalObservationPolicy(),next_policy);
    }
}

TEST(EngineContracts, ExperimentalObservationPolicyInterleaveUsesIndependentActualBackends) {
    RestorePolicyConfig restore;VIOEngine a,b;ASSERT_TRUE(configure(a));ASSERT_TRUE(configure(b));
    a.setPnPParams(false,3);b.setExperimentalObservationPolicy(1);
    EXPECT_EQ(a.getExperimentalObservationPolicy(),0);EXPECT_EQ(a.getEffectiveExperimentalObservationPolicy(),0);
    EXPECT_EQ(b.getExperimentalObservationPolicy(),1);EXPECT_EQ(b.getEffectiveExperimentalObservationPolicy(),1);
    ASSERT_TRUE(populatePolicyBackend(a));ASSERT_TRUE(populatePolicyBackend(b));
    EXPECT_EQ(backend::EstimatorTestAccess::optimizer(VIOEngineTestAccess::estimator(a)).getLastSolverDiagnostics().currentVisualFactors,8);
    EXPECT_EQ(backend::EstimatorTestAccess::optimizer(VIOEngineTestAccess::estimator(b)).getLastSolverDiagnostics().currentVisualFactors,7);
    const auto a_epoch=a.getEpoch(),b_epoch=b.getEpoch();
    a.setExperimentalObservationPolicy(2);EXPECT_EQ(a.getEpoch(),a_epoch+1);
    EXPECT_EQ(b.getEpoch(),b_epoch);EXPECT_EQ(b.getEffectiveExperimentalObservationPolicy(),1);
    EXPECT_TRUE(backend::EstimatorTestAccess::optimizer(VIOEngineTestAccess::estimator(b)).hasMarginalizationPrior());
    ASSERT_TRUE(configure(a));EXPECT_EQ(a.getExperimentalObservationPolicy(),0);EXPECT_EQ(a.getEffectiveExperimentalObservationPolicy(),0);
    EXPECT_EQ(b.getExperimentalObservationPolicy(),1);EXPECT_EQ(b.getEffectiveExperimentalObservationPolicy(),1);EXPECT_EQ(b.getEpoch(),b_epoch);
    const auto a_reconfigured_epoch=a.getEpoch();b.reset();
    EXPECT_EQ(b.getEpoch(),b_epoch+1);EXPECT_EQ(b.getEffectiveExperimentalObservationPolicy(),1);EXPECT_EQ(a.getEpoch(),a_reconfigured_epoch);
    ASSERT_TRUE(populatePolicyBackend(a));ASSERT_TRUE(populatePolicyBackend(b));
    EXPECT_EQ(backend::EstimatorTestAccess::optimizer(VIOEngineTestAccess::estimator(a)).getLastSolverDiagnostics().currentVisualFactors,8);
    EXPECT_EQ(backend::EstimatorTestAccess::optimizer(VIOEngineTestAccess::estimator(b)).getLastSolverDiagnostics().currentVisualFactors,7);
    VIOEngineTestAccess::estimator(a).setExperimentalObservationPolicy(2);
    EXPECT_EQ(a.getExperimentalObservationPolicy(),0);EXPECT_EQ(a.getEffectiveExperimentalObservationPolicy(),2);
    a.reset();EXPECT_EQ(a.getExperimentalObservationPolicy(),0);EXPECT_EQ(a.getEffectiveExperimentalObservationPolicy(),0);
    EXPECT_EQ(b.getExperimentalObservationPolicy(),1);EXPECT_EQ(b.getEffectiveExperimentalObservationPolicy(),1);
}

TEST(EngineContracts, ConfigureFromOrdinaryConfigDisablesExperimentalObservationPolicy) {
    RestorePolicyConfig restore;VIOEngine engine;ASSERT_TRUE(configure(engine));engine.setExperimentalObservationPolicy(2);
    const auto config=utility::g_config;const auto epoch=engine.getEpoch();
    struct TemporaryCamera {
        std::filesystem::path path=std::filesystem::temp_directory_path()/"mobile-slam-experimental-policy-camera.yaml";
        ~TemporaryCamera() {std::error_code error;std::filesystem::remove(path,error);}
    } file;
    std::ofstream yaml(file.path);
    yaml<<"%YAML:1.0\nmodel_type: PINHOLE\ncamera_name: policy-control\nimage_width: 96\nimage_height: 96\n"
        <<"distortion_parameters: {k1: 0., k2: 0., p1: 0., p2: 0.}\n"
        <<"projection_parameters: {fx: 100., fy: 100., cx: 48., cy: 48.}\n";yaml.close();
    ASSERT_TRUE(engine.configureFromConfig(config,file.path.string()));
    EXPECT_EQ(engine.getEpoch(),epoch+1);EXPECT_EQ(engine.getExperimentalObservationPolicy(),0);
    EXPECT_EQ(engine.getEffectiveExperimentalObservationPolicy(),0);
    ASSERT_TRUE(populatePolicyBackend(engine));
    EXPECT_EQ(backend::EstimatorTestAccess::optimizer(VIOEngineTestAccess::estimator(engine)).getLastSolverDiagnostics().currentVisualFactors,8);
}

namespace {
common::VINSResult inertialSeed(double time = 1) {
    common::VINSResult seed;
    seed.timestamp=time;seed.P.setZero();seed.R.setIdentity();seed.V.setZero();
    seed.Ba.setZero();seed.Bg.setZero();return seed;
}
}

TEST(EngineContracts, InertialPredictionUsesKnownStateVelocityAndAffineAcceleration) {
    utility::InertialPrediction prediction;
    auto seed=inertialSeed();seed.V={1,-2,.5};
    const Eigen::Vector3d gravity(0,0,9.81),zero=Eigen::Vector3d::Zero();
    ASSERT_TRUE(prediction.seed(seed,gravity,Eigen::Matrix3d::Identity(),zero,gravity,zero,7));
    prediction.advance(1.1,gravity+Eigen::Vector3d(3,0,0),zero,7);
    ASSERT_TRUE(prediction.valid());
    // a(t)=30*t: displacement .1 + 30*.1^3/6 along x.
    EXPECT_TRUE((prediction.cameraPose().block<3,1>(0,3).isApprox(Eigen::Vector3d(.105,-.2,.05),1e-12)));
    EXPECT_DOUBLE_EQ(prediction.anchorTimestamp(),1);EXPECT_DOUBLE_EQ(prediction.timestamp(),1.1);
}

TEST(EngineContracts, InertialPredictionExactRotationIncludesCameraLeverArm) {
    utility::InertialPrediction prediction;auto seed=inertialSeed();
    const Eigen::Vector3d gravity(0,0,9.81),omega(0,0,2),lever(1,0,0);
    const Eigen::Matrix3d extrinsic=Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitX()).toRotationMatrix();
    ASSERT_TRUE(prediction.seed(seed,gravity,extrinsic,lever,gravity,omega,4));
    prediction.advance(1.1,gravity,omega,4);
    ASSERT_TRUE(prediction.valid());
    const Eigen::Matrix3d expected=Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    EXPECT_TRUE((prediction.cameraPose().block<3,3>(0,0).isApprox(expected*extrinsic,1e-12)));
    EXPECT_TRUE((prediction.cameraPose().block<3,1>(0,3).isApprox(expected*lever,1e-12)));
}

TEST(EngineContracts, InertialPredictionExpirationAndInvalidationRequireNewSeed) {
    utility::InertialPrediction prediction;auto seed=inertialSeed();
    const Eigen::Vector3d gravity(0,0,9.81),zero=Eigen::Vector3d::Zero();
    ASSERT_TRUE(prediction.seed(seed,gravity,Eigen::Matrix3d::Identity(),zero,gravity,zero,4));
    for(int i=1;i<=5;++i) prediction.advance(1+.1*i,gravity,zero,4);
    EXPECT_TRUE(prediction.valid());prediction.advance(1.50000001,gravity,zero,4);
    EXPECT_FALSE(prediction.valid());prediction.advance(1.51,gravity,zero,4);EXPECT_FALSE(prediction.valid());
    seed.timestamp=2;ASSERT_TRUE(prediction.seed(seed,gravity,Eigen::Matrix3d::Identity(),zero,gravity,zero,4));
    prediction.invalidate();prediction.advance(2.01,gravity,zero,4);EXPECT_FALSE(prediction.valid());
}

namespace {
void prepareInertialAnchor(VIOEngine& engine, const common::VINSResult& seed,
                           const Eigen::Vector3d& acc=Eigen::Vector3d(0,0,9.81),
                           const Eigen::Vector3d& gyro=Eigen::Vector3d::Zero()) {
    auto& estimator=VIOEngineTestAccess::estimator(engine);
    backend::EstimatorTestAccess::initialized(estimator);
    backend::EstimatorTestAccess::usable(estimator,true);
    for(int i=0;i<=utility::WINDOW_SIZE;++i) {
        auto& frame=estimator.sliding_window_[i];
        frame.timestamp=seed.timestamp;frame.P=seed.P;frame.R=seed.R;frame.V=seed.V;frame.Ba=seed.Ba;frame.Bg=seed.Bg;
    }
    VIOEngineTestAccess::anchorInputs(engine,seed.timestamp,acc,gyro);
    backend::SolverDiagnostics diagnostic;diagnostic.usable=true;
    VIOEngineTestAccess::solver(engine,diagnostic);
    double output[16];ASSERT_TRUE(VIOEngineTestAccess::pose(engine,seed.P,seed.R,seed.timestamp,output));
    VIOEngineTestAccess::finishPrediction(engine);
}
void addGroundFeatures(VIOEngine& engine,int count) {
    auto& features=backend::EstimatorTestAccess::features(VIOEngineTestAccess::estimator(engine));
    features.clearState();auto& tracker=VIOEngineTestAccess::tracker(engine);
    tracker.ids.clear();tracker.track_cnt.clear();tracker.cur_pts.clear();
    tracker.cur_undistorted_pts.clear();tracker.pts_velocity.clear();
    for(int id=0;id<count;++id) {
        Eigen::Matrix<double,7,1> observation;observation<<.01*id,.1,1,30,30,0,0;
        features.feature_bank_.emplace_back(id,0);auto& feature=features.feature_bank_.back();
        feature.feature_per_frame.emplace_back(observation);feature.feature_per_frame.emplace_back(observation);
        feature.estimated_depth=5;feature.solve_flag=1;
        tracker.ids.push_back(id);tracker.track_cnt.push_back(2);tracker.cur_pts.emplace_back(30,30);
        tracker.cur_undistorted_pts.emplace_back(.01*id,.1);tracker.pts_velocity.emplace_back(0,0);
    }
}
}

TEST(EngineContracts, InertialBlankOutputIsSeparateAndExplicitInvalidationIsSticky) {
    RestorePolicyConfig restore;VIOEngine engine;ASSERT_TRUE(configure(engine));engine.setPnPParams(false,3);
    auto seed=inertialSeed();seed.V={1,0,0};prepareInertialAnchor(engine,seed);
    ASSERT_TRUE(VIOEngineTestAccess::prediction(engine).valid());EXPECT_EQ(engine.getInertialPrediction(),"null");
    std::vector<uint8_t> blank(96*96,0);double output[16],map[3];
    const IMUReading first{1.05,0,0,9.81,0,0,0};
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,&first,1,1.05,output));
    EXPECT_FALSE(engine.getPoseFresh());EXPECT_FALSE(engine.getPoseValid());EXPECT_EQ(engine.getStatusCode(),3);
    EXPECT_EQ(engine.getMapPoints(map,1),0);EXPECT_EQ(engine.getGroundSupportPoints(map,1),0);
    for(double value:output) EXPECT_TRUE(std::isnan(value));
    const auto prediction=YAML::Load(engine.getInertialPrediction());ASSERT_TRUE(prediction.IsMap());
    EXPECT_EQ(prediction["schema"].as<std::string>(),"mobile-slam-inertial-prediction-v1");
    EXPECT_EQ(prediction["mode"].as<std::string>(),"inertial-only");EXPECT_EQ(prediction["poseFrame"].as<std::string>(),"camera");
    EXPECT_FALSE(prediction["visualSupported"].as<bool>());EXPECT_FALSE(prediction["metricTranslationSupported"].as<bool>());
    EXPECT_TRUE(prediction["positionUncertain"].as<bool>());EXPECT_DOUBLE_EQ(prediction["maxHorizonSeconds"].as<double>(),.5);
    EXPECT_DOUBLE_EQ(prediction["timestamp"].as<double>(),1.05);EXPECT_DOUBLE_EQ(prediction["anchorTimestamp"].as<double>(),1);
    EXPECT_NEAR(prediction["pose"][3].as<double>(),.05,1e-12);
    const auto epoch=engine.getEpoch();engine.invalidateInertialPrediction();EXPECT_EQ(engine.getEpoch(),epoch);
    const IMUReading next{1.1,0,0,9.81,0,0,0};
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,&next,1,1.1,output));EXPECT_EQ(engine.getInertialPrediction(),"null");
    seed.timestamp=2;prepareInertialAnchor(engine,seed);
    const IMUReading reseeded{2.05,0,0,9.81,0,0,0};
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,&reseeded,1,2.05,output));EXPECT_NE(engine.getInertialPrediction(),"null");
}

TEST(EngineContracts, InertialRawGapCannotBeLaunderedByCameraInterpolationOrReseed) {
    RestorePolicyConfig restore;VIOEngine engine;ASSERT_TRUE(configure(engine));engine.setPnPParams(false,3);
    prepareInertialAnchor(engine,inertialSeed());
    const IMUReading future{1.2,0,0,9.81,0,0,0};std::vector<uint8_t> blank(96*96,0);double output[16];
    for(int i=1;i<=4;++i) {
        const double time=1+.05*i;
        EXPECT_FALSE(engine.processFrame(blank.data(),96,96,i==1?&future:nullptr,i==1?1:0,time,output));
        EXPECT_EQ(engine.getInertialPrediction(),"null");EXPECT_FALSE(VIOEngineTestAccess::rawCoverage(engine));
        // A successful visual result at this interpolated endpoint still cannot seed this bad raw interval.
        auto& estimator=VIOEngineTestAccess::estimator(engine);
        estimator.sliding_window_[utility::WINDOW_SIZE-1].timestamp=time;
        backend::EstimatorTestAccess::usable(estimator,true);
        ASSERT_TRUE(VIOEngineTestAccess::pose(engine,estimator.sliding_window_[utility::WINDOW_SIZE].P,
                    estimator.sliding_window_[utility::WINDOW_SIZE].R,time,output));
        VIOEngineTestAccess::finishPrediction(engine);EXPECT_FALSE(VIOEngineTestAccess::prediction(engine).valid());
    }
    const IMUReading valid{1.25,0,0,9.81,0,0,0};
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,&valid,1,1.25,output));
    EXPECT_TRUE(VIOEngineTestAccess::rawCoverage(engine));EXPECT_EQ(engine.getInertialPrediction(),"null");
}

TEST(EngineContracts, InertialCoverageQualitySeedTimeAndPnPFailClosed) {
    RestorePolicyConfig restore;VIOEngine engine;ASSERT_TRUE(configure(engine));engine.setPnPParams(false,3);
    prepareInertialAnchor(engine,inertialSeed());auto& estimator=VIOEngineTestAccess::estimator(engine);
    estimator.sliding_window_[utility::WINDOW_SIZE-1].timestamp=.9;
    VIOEngineTestAccess::finishPrediction(engine);EXPECT_FALSE(VIOEngineTestAccess::prediction(engine).valid());
    prepareInertialAnchor(engine,inertialSeed());VIOEngineTestAccess::frameTime(engine,1.01);
    VIOEngineTestAccess::finishPrediction(engine);EXPECT_FALSE(VIOEngineTestAccess::prediction(engine).valid());
    prepareInertialAnchor(engine,inertialSeed());
    backend::SolverDiagnostics warning;warning.qualityReason="relative_translation_threshold";
    VIOEngineTestAccess::solver(engine,warning);EXPECT_EQ(engine.getLastSolverQualityReason(),warning.qualityReason);
    VIOEngineTestAccess::failure(engine,"insufficient_final_current_features");
    VIOEngineTestAccess::finishPrediction(engine);EXPECT_EQ(engine.getInertialPrediction(),"null");
    EXPECT_FALSE(VIOEngineTestAccess::prediction(engine).valid());
    prepareInertialAnchor(engine,inertialSeed());engine.setPnPParams(true,3);
    EXPECT_FALSE(VIOEngineTestAccess::prediction(engine).valid());VIOEngineTestAccess::finishPrediction(engine);
    EXPECT_EQ(engine.getInertialPrediction(),"null");
    engine.setPnPParams(false,3);prepareInertialAnchor(engine,inertialSeed());
    std::vector<uint8_t> blank(96*96,0);double output[16];
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,nullptr,0,1.05,output));
    EXPECT_EQ(engine.getLastReason(),"imu_missing_future");EXPECT_FALSE(VIOEngineTestAccess::prediction(engine).valid());
    prepareInertialAnchor(engine,inertialSeed());engine.reset();EXPECT_FALSE(VIOEngineTestAccess::prediction(engine).valid());
    EXPECT_EQ(engine.getLastSolverQualityReason(),"");
    prepareInertialAnchor(engine,inertialSeed());EXPECT_FALSE(engine.configure(96,96,100,100,48,48,2,0,0,0,0,nullptr,nullptr,.1,.001,.01,.0001,9.81));
    EXPECT_FALSE(VIOEngineTestAccess::prediction(engine).valid());
}

TEST(EngineContracts, InertialDomainsAndFiveMeterBoundHaveIndependentExpectedValues) {
    utility::InertialPrediction prediction;auto seed=inertialSeed();
    const Eigen::Vector3d gravity(0,0,9.81),zero=Eigen::Vector3d::Zero();
    seed.V={10,0,0};ASSERT_TRUE(prediction.seed(seed,gravity,Eigen::Matrix3d::Identity(),zero,gravity,zero,1));
    for(int i=1;i<=5;++i) prediction.advance(1+.1*i,gravity,zero,1);
    EXPECT_TRUE(prediction.valid());EXPECT_NEAR(prediction.cameraPose()(0,3),5,1e-12);
    seed.V={10.1,0,0};ASSERT_TRUE(prediction.seed(seed,gravity,Eigen::Matrix3d::Identity(),zero,gravity,zero,1));
    for(int i=1;i<=5;++i) prediction.advance(1+.1*i,gravity,zero,1);EXPECT_FALSE(prediction.valid());
    seed=inertialSeed();seed.Bg.x()=NAN;EXPECT_FALSE(prediction.seed(seed,gravity,Eigen::Matrix3d::Identity(),zero,gravity,zero,1));
    seed=inertialSeed();seed.R(0,0)=2;EXPECT_FALSE(prediction.seed(seed,gravity,Eigen::Matrix3d::Identity(),zero,gravity,zero,1));
    seed=inertialSeed();ASSERT_TRUE(prediction.seed(seed,gravity,Eigen::Matrix3d::Identity(),zero,gravity,zero,1));
    prediction.advance(1.01,gravity,zero,2);EXPECT_FALSE(prediction.valid());
    ASSERT_TRUE(prediction.seed(seed,gravity,Eigen::Matrix3d::Identity(),zero,gravity,zero,1));
    prediction.advance(1.10000001,gravity,zero,1);EXPECT_FALSE(prediction.valid());
    ASSERT_TRUE(prediction.seed(seed,gravity,Eigen::Matrix3d::Identity(),zero,gravity,zero,1));
    prediction.advance(1.01,Eigen::Vector3d(NAN,0,0),zero,1);EXPECT_FALSE(prediction.valid());
}

TEST(EngineContracts, GroundSupportRequiresCurrentFiniteFullObservationAndAdmittedDepth) {
    RestorePolicyConfig restore;VIOEngine engine;ASSERT_TRUE(configure(engine));engine.setPnPParams(false,3);
    prepareInertialAnchor(engine,inertialSeed());addGroundFeatures(engine,8);
    auto& tracker=VIOEngineTestAccess::tracker(engine);
    auto& features=backend::EstimatorTestAccess::features(VIOEngineTestAccess::estimator(engine));
    tracker.ids[1]=999;tracker.track_cnt[2]=1;tracker.cur_undistorted_pts[3].x=NAN;
    tracker.cur_pts[4].y=NAN;tracker.pts_velocity[5].x=NAN;
    auto feature=features.feature_bank_.begin();std::advance(feature,6);feature->geometry_pending=true;
    ++feature;feature->feature_per_frame.back().projection_outlier=true;
    double output[9]={0,0,0,123,123,123,123,123,123};
    ASSERT_EQ(engine.getGroundSupportPoints(output,3),1);EXPECT_DOUBLE_EQ(output[0],0);EXPECT_DOUBLE_EQ(output[1],.5);EXPECT_DOUBLE_EQ(output[2],5);
    EXPECT_DOUBLE_EQ(output[3],123);EXPECT_EQ(features.feature_bank_.size(),8u);
    EXPECT_EQ(engine.getGroundSupportPoints(nullptr,3),0);EXPECT_EQ(engine.getGroundSupportPoints(output,0),0);
    backend::EstimatorTestAccess::gravity(VIOEngineTestAccess::estimator(engine),Eigen::Vector3d(0,0,-9.81));
    EXPECT_EQ(engine.getGroundSupportPoints(output,3),0);
    backend::EstimatorTestAccess::gravity(VIOEngineTestAccess::estimator(engine),Eigen::AngleAxisd(.04,Eigen::Vector3d::UnitX())*Eigen::Vector3d(0,0,9.81));
    EXPECT_EQ(engine.getGroundSupportPoints(output,3),0);
    backend::EstimatorTestAccess::gravity(VIOEngineTestAccess::estimator(engine),Eigen::Vector3d(0,0,9.81));
    backend::SolverDiagnostics warning;warning.qualityReason="acc_bias_threshold";VIOEngineTestAccess::solver(engine,warning);
    EXPECT_EQ(engine.getGroundSupportPoints(output,3),0);
}

TEST(EngineContracts, GroundSupportCapacityAndReadsDoNotChangeBackendOrPrediction) {
    RestorePolicyConfig restore;VIOEngine engine;ASSERT_TRUE(configure(engine));engine.setPnPParams(false,3);
    prepareInertialAnchor(engine,inertialSeed());addGroundFeatures(engine,520);
    auto& estimator=VIOEngineTestAccess::estimator(engine);const auto before=estimator.getLatestVINSResult();
    const auto epoch=engine.getEpoch();const auto reason=engine.getLastReason();
    std::vector<double> output(3*513,123456);ASSERT_EQ(engine.getGroundSupportPoints(output.data(),999),512);
    EXPECT_DOUBLE_EQ(output[3*512],123456);EXPECT_EQ(engine.getEpoch(),epoch);EXPECT_EQ(engine.getLastReason(),reason);
    EXPECT_EQ(engine.getInertialPrediction(),"null");const auto after=estimator.getLatestVINSResult();
    EXPECT_TRUE(before.P.isApprox(after.P,0));EXPECT_TRUE(before.R.isApprox(after.R,0));EXPECT_TRUE(before.V.isApprox(after.V,0));
    EXPECT_TRUE(before.Ba.isApprox(after.Ba,0));EXPECT_TRUE(before.Bg.isApprox(after.Bg,0));
    const bool prior=backend::EstimatorTestAccess::optimizer(estimator).hasMarginalizationPrior();
    engine.invalidateInertialPrediction();EXPECT_EQ(backend::EstimatorTestAccess::optimizer(estimator).hasMarginalizationPrior(),prior);
    EXPECT_EQ(engine.getGroundSupportPoints(output.data(),1),1);engine.setPnPParams(true,3);EXPECT_EQ(engine.getGroundSupportPoints(output.data(),1),0);
}

TEST(EngineContracts, InertialAffineOracleIsIndependentOfSamplePartitionBiasGravityAndExtrinsic) {
    auto seed=inertialSeed();seed.P={.3,-.2,.1};seed.V={1,.2,-.1};seed.Ba={.02,-.03,.04};seed.Bg={.01,-.02,.03};
    seed.R=Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Vector3d gravity(0,9.81,0),omega(0,0,.7),a0(.4,-.3,.2),slope(.6,.2,-.1),lever(.2,-.1,.3);
    const Eigen::Matrix3d ric=Eigen::AngleAxisd(-.4,Eigen::Vector3d::UnitX()).toRotationMatrix();
    for(const std::vector<double>& times:{std::vector<double>{.05,.1,.15,.2,.25,.3,.35,.4,.45,.5},
                                        std::vector<double>{.02,.1,.15,.19,.25,.31,.36,.4,.45,.5}}) {
        utility::InertialPrediction prediction;
        ASSERT_TRUE(prediction.seed(seed,gravity,ric,lever,seed.R.transpose()*(gravity+a0)+seed.Ba,omega+seed.Bg,2));
        for(double time:times) {
            const Eigen::Matrix3d rotation=seed.R*Eigen::AngleAxisd(.7*time,Eigen::Vector3d::UnitZ()).toRotationMatrix();
            prediction.advance(1+time,rotation.transpose()*(gravity+a0+slope*time)+seed.Ba,omega+seed.Bg,2);
            ASSERT_TRUE(prediction.valid());
            const Eigen::Vector3d position=seed.P+seed.V*time+.5*a0*time*time+slope*time*time*time/6;
            EXPECT_TRUE((prediction.cameraPose().block<3,1>(0,3).isApprox(position+rotation*lever,1e-10)));
            EXPECT_TRUE((prediction.cameraPose().block<3,3>(0,0).isApprox(rotation*ric,1e-12)));
        }
    }
}

TEST(EngineContracts, InertialSeedRequiresMatchingPublishedStateAndActualGravity) {
    RestorePolicyConfig restore;VIOEngine engine;ASSERT_TRUE(configure(engine));engine.setPnPParams(false,3);
    prepareInertialAnchor(engine,inertialSeed());auto& estimator=VIOEngineTestAccess::estimator(engine);
    estimator.sliding_window_[utility::WINDOW_SIZE-1].P.x()=.1;
    VIOEngineTestAccess::finishPrediction(engine);EXPECT_FALSE(VIOEngineTestAccess::prediction(engine).valid());
    prepareInertialAnchor(engine,inertialSeed());backend::EstimatorTestAccess::usable(estimator,false);
    VIOEngineTestAccess::finishPrediction(engine);EXPECT_FALSE(VIOEngineTestAccess::prediction(engine).valid());
    prepareInertialAnchor(engine,inertialSeed());backend::EstimatorTestAccess::gravity(estimator,Eigen::Vector3d::Zero());
    VIOEngineTestAccess::finishPrediction(engine);EXPECT_FALSE(VIOEngineTestAccess::prediction(engine).valid());
    const Eigen::Vector3d gravity(0,9.81,0);
    prepareInertialAnchor(engine,inertialSeed(),gravity);backend::EstimatorTestAccess::gravity(estimator,gravity);
    VIOEngineTestAccess::finishPrediction(engine);ASSERT_TRUE(VIOEngineTestAccess::prediction(engine).valid());
    const IMUReading sample{1.05,0,9.81,0,0,0,0};std::vector<uint8_t> blank(96*96,0);double output[16];
    EXPECT_FALSE(engine.processFrame(blank.data(),96,96,&sample,1,1.05,output));
    const auto prediction=YAML::Load(engine.getInertialPrediction());ASSERT_TRUE(prediction.IsMap());
    EXPECT_NEAR(prediction["pose"][3].as<double>(),0,1e-12);EXPECT_NEAR(prediction["pose"][7].as<double>(),0,1e-12);
    EXPECT_NEAR(prediction["pose"][11].as<double>(),0,1e-12);
}

TEST(EngineContracts, InertialChannelAndGroundReadsNeverFeedBackIntoBackendState) {
    RestorePolicyConfig restore;VIOEngine predicted,control;ASSERT_TRUE(configure(predicted));ASSERT_TRUE(configure(control));
    predicted.setPnPParams(false,3);auto seed=inertialSeed();seed.V={.5,0,0};
    prepareInertialAnchor(predicted,seed);prepareInertialAnchor(control,seed);control.invalidateInertialPrediction();
    std::vector<uint8_t> blank(96*96,0);double out_a[16],out_b[16],ground[9];
    for(int frame=1;frame<=5;++frame) {
        const IMUReading sample{1+.05*frame,.1,0,9.81,0,0,.2};
        EXPECT_EQ(predicted.processFrame(blank.data(),96,96,&sample,1,sample.timestamp,out_a),
                  control.processFrame(blank.data(),96,96,&sample,1,sample.timestamp,out_b));
        EXPECT_NE(predicted.getInertialPrediction(),"null");EXPECT_EQ(control.getInertialPrediction(),"null");
        EXPECT_EQ(predicted.getGroundSupportPoints(ground,3),0);
        EXPECT_EQ(predicted.getEpoch(),control.getEpoch());EXPECT_EQ(predicted.getLastReason(),control.getLastReason());
        EXPECT_EQ(predicted.getStatusCode(),control.getStatusCode());EXPECT_EQ(predicted.getFeaturePointCount(),control.getFeaturePointCount());
        auto& a=VIOEngineTestAccess::estimator(predicted);auto& b=VIOEngineTestAccess::estimator(control);
        for(int slot=0;slot<=utility::WINDOW_SIZE;++slot) {
            const auto& x=a.sliding_window_[slot];const auto& y=b.sliding_window_[slot];
            EXPECT_TRUE(x.P.isApprox(y.P,0));EXPECT_TRUE(x.R.isApprox(y.R,0));EXPECT_TRUE(x.V.isApprox(y.V,0));
            EXPECT_TRUE(x.Ba.isApprox(y.Ba,0));EXPECT_TRUE(x.Bg.isApprox(y.Bg,0));EXPECT_EQ(x.dt_buf,y.dt_buf);
            ASSERT_EQ(bool(x.pre_integration),bool(y.pre_integration));
            if(x.pre_integration) {
                EXPECT_TRUE(x.pre_integration->jacobian.isApprox(y.pre_integration->jacobian,0));
                EXPECT_TRUE(x.pre_integration->covariance.isApprox(y.pre_integration->covariance,0));
            }
        }
        EXPECT_EQ(backend::EstimatorTestAccess::optimizer(a).hasMarginalizationPrior(),backend::EstimatorTestAccess::optimizer(b).hasMarginalizationPrior());
    }
}

TEST(EngineContracts, InertialFrameErrorsDoNotReuseAPreviousAvailablePrediction) {
    RestorePolicyConfig restore;VIOEngine engine;ASSERT_TRUE(configure(engine));engine.setPnPParams(false,3);
    std::vector<uint8_t> blank(96*96,0);double output[16];
    for(int fault=0;fault<4;++fault) {
        prepareInertialAnchor(engine,inertialSeed());const IMUReading valid{1.05,0,0,9.81,0,0,0};
        EXPECT_FALSE(engine.processFrame(blank.data(),96,96,&valid,1,1.05,output));ASSERT_NE(engine.getInertialPrediction(),"null");
        IMUReading next{1.1,0,0,9.81,0,0,0};
        if(fault==0)next.acc_x=NAN;
        if(fault==1)next.timestamp=1.05;
        const double time=fault==2?1.05:1.1;
        EXPECT_FALSE(engine.processFrame(blank.data(),fault==3?95:96,96,&next,1,time,output));
        EXPECT_EQ(engine.getInertialPrediction(),"null");EXPECT_FALSE(VIOEngineTestAccess::prediction(engine).valid());
    }
}
