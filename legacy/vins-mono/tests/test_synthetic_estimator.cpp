// Independent generated observations bypass optical flow only; physical GT is never supplied to Estimator.
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <opencv2/core.hpp>
#include "backend/estimator.h"
#include "backend/factor/integration_base.h"
#include "common/image_frame.h"
#include "utility/config.h"

namespace {
struct RestoreConfig { utility::Config saved=utility::g_config; ~RestoreConfig(){utility::g_config=saved;} };
struct ExecutionProfile { ExecutionProfile(){cv::setRNGSeed(0);cv::setNumThreads(1);} ~ExecutionProfile(){cv::setNumThreads(0);} };
struct IMU { double t; Eigen::Vector3d a,w; };
struct Truth { double t; Eigen::Vector3d p; Eigen::Matrix3d r; };
struct Pose { int frame; Truth value; };
std::vector<double> fields(std::string line) {
    std::replace(line.begin(),line.end(),',',' ');std::istringstream stream(line);
    std::vector<double> values;double value;while(stream>>value)values.push_back(value);return values;
}
std::filesystem::path fixture() {
    const char* selected=std::getenv("MOBILE_SLAM_SYNTHETIC_DATA");
    return selected?selected:"build/mobile-web-20261009/synthetic/clean-seed101-v2";
}
Eigen::Vector3d vector3(const YAML::Node& node) {return {node[0].as<double>(),node[1].as<double>(),node[2].as<double>()};}
Eigen::Matrix3d rotation(const YAML::Node& node) {
    Eigen::Matrix3d result;for(int r=0;r<3;r++)for(int c=0;c<3;c++)result(r,c)=node[r*3+c].as<double>();return result;
}
std::vector<Truth> loadTruth(const std::filesystem::path& root) {
    const auto document=YAML::LoadFile((root/"groundtruth.json").string());std::vector<Truth> result;
    for(const auto& frame:document["frames"])result.push_back({frame["timestamp"].as<double>(),vector3(frame["body"]["position"]),rotation(frame["body"]["rotation"])});
    return result;
}
double rotationError(const Eigen::Matrix3d& r) {return std::acos(std::clamp((r.trace()-1)*.5,-1.,1.));}
}

TEST(SyntheticEstimator, ConstantWorldAccelerationHasIndependentExactPreintegration) {
    RestoreConfig restore;
    const Eigen::Vector3d gravity(0,0,9.81),acceleration(.3,-.2,.15),force=gravity+acceleration;
    backend::factor::IntegrationBase pre(force,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
    for(int i=0;i<100;i++)pre.push_back(.01,force,Eigen::Vector3d::Zero());
    EXPECT_NEAR(pre.sum_dt,1.,1e-12);
    EXPECT_LT((pre.delta_v-force).norm(),1e-12);
    EXPECT_LT((pre.delta_p-.5*force).norm(),1e-12);
    EXPECT_LT((pre.delta_q.toRotationMatrix()-Eigen::Matrix3d::Identity()).norm(),1e-12);
}

TEST(SyntheticEstimator, GeneratedDirectFeaturesAndRawIMUProduceMetricTrajectory) {
    RestoreConfig restore;ExecutionProfile execution;const auto root=fixture();
    ASSERT_TRUE(std::filesystem::is_regular_file(root/"manifest.json"))<<"Generate the frozen synthetic fixture first: "<<root;
    const auto spec=YAML::LoadFile((root/"spec.json").string());
    ASSERT_EQ(spec["scenario"].as<std::string>(),"clean");
    ASSERT_EQ(spec["frame_count"].as<int>(),400);
    const auto truth=loadTruth(root);ASSERT_EQ(truth.size(),400u);
    utility::Config cfg;ASSERT_TRUE(cfg.loadFromYaml((root/"config.yaml").string()));
    utility::g_config=cfg;
    std::vector<common::ImageData> images(truth.size());
    std::ifstream feature_input(root/"features.csv");ASSERT_TRUE(feature_input);
    std::string line;std::getline(feature_input,line);
    while(std::getline(feature_input,line)) {
        const auto row=fields(line);ASSERT_EQ(row.size(),9u);
        const int frame=static_cast<int>(row[0]),id=static_cast<int>(row[1]);
        ASSERT_GE(frame,0);ASSERT_LT(frame,static_cast<int>(images.size()));
        Eigen::Matrix<double,7,1> value;for(int k=0;k<7;k++)value[k]=row[k+2];
        ASSERT_TRUE(value.allFinite());ASSERT_TRUE(images[frame].emplace(id,value).second);
    }
    std::ifstream camera_input(root/"mav0/cam0/data.csv");ASSERT_TRUE(camera_input);std::getline(camera_input,line);
    std::vector<double> camera_times;
    while(std::getline(camera_input,line))camera_times.push_back(static_cast<double>(std::stoll(line.substr(0,line.find(','))))*1e-9);
    ASSERT_EQ(camera_times.size(),truth.size());
    std::ifstream imu_input(root/"mav0/imu0/data.csv");ASSERT_TRUE(imu_input);std::getline(imu_input,line);
    std::vector<IMU> imu;
    while(std::getline(imu_input,line)) {
        const auto row=fields(line);ASSERT_EQ(row.size(),7u);
        imu.push_back({row[0]*1e-9,{row[4],row[5],row[6]},{row[1],row[2],row[3]}});
        if(imu.size()>1)ASSERT_GT(imu.back().t,imu[imu.size()-2].t);
    }
    ASSERT_LE(imu.front().t,truth.front().t);ASSERT_GE(imu.back().t,truth.back().t);
    const char* selected_output=std::getenv("MOBILE_SLAM_SYNTHETIC_OUTPUT");
    const auto stamp=std::chrono::steady_clock::now().time_since_epoch().count();
    const std::filesystem::path output=selected_output?std::filesystem::path(selected_output):
        std::filesystem::path("build/mobile-web-20261009/direct-feature")/("run-"+std::to_string(stamp));
    ASSERT_FALSE(std::filesystem::exists(output));ASSERT_TRUE(std::filesystem::create_directories(output));
    std::ofstream result(output/"results.json");ASSERT_TRUE(result);result<<std::setprecision(17)<<"{\"schema\":\"mobile-slam-synthetic-direct-feature-v1\",\"scope\":\"All visible analytic features; optical flow bypassed\",\"frames\":[";
    backend::Estimator estimator;estimator.setParameter();
    size_t cursor=1;double last_time=imu.front().t;estimator.processIMU(0,imu.front().a,imu.front().w);
    std::vector<Pose> poses;const uint64_t initial_epoch=estimator.getResetGeneration();
    for(size_t frame=0;frame<truth.size();frame++) {
        const double t=camera_times[frame];
        while(cursor<imu.size() && imu[cursor].t<t-1e-12) {
            estimator.processIMU(imu[cursor].t-last_time,imu[cursor].a,imu[cursor].w);last_time=imu[cursor].t;++cursor;
        }
        if(t>last_time+1e-12) {
            ASSERT_LT(cursor,imu.size());const auto& a=imu[cursor-1];const auto& b=imu[cursor];
            const double alpha=(t-a.t)/(b.t-a.t);ASSERT_GE(alpha,-1e-10);ASSERT_LE(alpha,1+1e-10);
            estimator.processIMU(t-last_time,(1-alpha)*a.a+alpha*b.a,(1-alpha)*a.w+alpha*b.w);last_time=t;
            if(std::abs(b.t-t)<1e-12)++cursor;
        }
        estimator.processImage(images[frame],t);
        bool fresh=estimator.solver_flag_==common::NON_LINEAR && estimator.hasUsableLatestImageUpdate();
        auto value=estimator.getLatestVINSResult();
        fresh=fresh && value.P.allFinite() && value.R.allFinite() && std::abs(value.timestamp-t)<1e-8;
        result<<(frame?",":"")<<"{\"frame\":"<<frame<<",\"inputTimestamp\":"<<t<<",\"poseTimestamp\":"<<(fresh?value.timestamp:-1)
            <<",\"poseValid\":"<<(fresh?"true":"false")<<",\"poseFresh\":"<<(fresh?"true":"false")
            <<",\"engineEpoch\":"<<estimator.getResetGeneration()<<",\"pose\":";
        if(fresh) {
            poses.push_back({static_cast<int>(frame),{t,value.P,value.R}});
            result<<'[';for(int r=0;r<4;r++)for(int c=0;c<4;c++)result<<((r||c)?",":"")<<(r==3?(c==3?1.:0.):c==3?value.P[r]:value.R(r,c));result<<']';
        } else result<<"null";
        result<<'}';
    }
    result<<"]}\n";result.close();
    std::cout<<"Synthetic direct-feature evidence: "<<output<<"; poses="<<poses.size()<<'\n';
    ASSERT_GE(poses.size(),3u)<<"No metric trajectory; inspect preserved raw result rows";
    Eigen::Matrix<double,3,Eigen::Dynamic> estimate(3,poses.size()),ground(3,poses.size());
    int warmup=0;
    for(size_t i=0;i<poses.size();i++) {estimate.col(i)=poses[i].value.p;ground.col(i)=truth[poses[i].frame].p;if(poses[i].value.t>=5)++warmup;}
    const Eigen::Matrix4d transform=Eigen::umeyama(estimate,ground,false);
    double translation_squared=0,rotation_squared=0;
    for(size_t i=0;i<poses.size();i++) {
        translation_squared+=(transform.block<3,3>(0,0)*estimate.col(i)+transform.block<3,1>(0,3)-ground.col(i)).squaredNorm();
        const double angle=rotationError(truth[poses[i].frame].r.transpose()*transform.block<3,3>(0,0)*poses[i].value.r);rotation_squared+=angle*angle;
    }
    const double ate=std::sqrt(translation_squared/poses.size()),ape_degrees=std::sqrt(rotation_squared/poses.size())*180/std::acos(-1.);
    std::cout<<"Direct-feature fixed coverage="<<warmup<<"/300; first="<<poses.front().value.t<<"; ATE="<<ate<<"; APE="<<ape_degrees<<" deg\n";
    EXPECT_LE(poses.front().value.t,5.);EXPECT_GE(warmup,270);EXPECT_EQ(estimator.getResetGeneration(),initial_epoch);
    EXPECT_LE(ate,.10);EXPECT_LE(ape_degrees,3.);
}
