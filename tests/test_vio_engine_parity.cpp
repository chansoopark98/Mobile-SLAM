#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <array>
#include <fstream>
#include "vio_engine.h"
#include "utility/measurement_processor.h"

namespace {
constexpr const char* dataset="./assets/datasets/tum/dataset-room1_512_16";
constexpr const char* config="./config/tum_vi_room1.yaml";
struct Pose {double time; Eigen::Vector3d p; Eigen::Matrix3d r;};
std::vector<Pose> run(bool raw_adapter,int frames) {
    utility::MeasurementProcessor data;
    const std::string root(dataset);
    if(!data.initialize(root+"/mav0/imu0/data.csv",root+"/mav0/cam0/data.csv",root+"/mav0/cam0/data",config))return {};
    utility::Config cfg=utility::g_config;
    cfg.estimator.num_iterations=10;cfg.estimator.solver_time=10;
    VIOEngine engine;
    if(!engine.configureFromConfig(cfg,config))return {};
    engine.setExecutionParams(0,1);
    engine.setMobileParams(10,10,cfg.feature_tracker.max_cnt);
    engine.setPnPParams(false,3);
    const auto& imu=data.getIMUData();size_t cursor=0;
    std::vector<Pose> result;
    for(int i=0;i<frames;++i){
        const auto& frame=data.getImageFileData().at(i);
        cv::Mat image;
        std::vector<IMUReading> packet;
        if(raw_adapter){
            const auto raw=data.createRawMeasurementMsg(i,frame);image=raw.gray_image;
            for(const auto& sample:raw.imu_msg)packet.push_back({sample.timestamp,sample.linear_acc_x,sample.linear_acc_y,sample.linear_acc_z,sample.angular_vel_x,sample.angular_vel_y,sample.angular_vel_z});
        }else{
            image=cv::imread(frame.full_path,cv::IMREAD_GRAYSCALE);
            if(cursor==0 || imu[cursor-1].timestamp<=frame.timestamp){
                while(cursor<imu.size()){
                    const auto& sample=imu[cursor++];
                    packet.push_back({sample.timestamp,sample.linear_acc_x,sample.linear_acc_y,sample.linear_acc_z,sample.angular_vel_x,sample.angular_vel_y,sample.angular_vel_z});
                    if(sample.timestamp>frame.timestamp)break;
                }
            }
        }
        if(image.empty())return {};
        std::array<double,16> output{};
        if(engine.processFrame(image.data,image.cols,image.rows,packet.data(),packet.size(),frame.timestamp,output.data())){
            EXPECT_TRUE(engine.getPoseFresh());EXPECT_TRUE(engine.getPoseValid());
            EXPECT_DOUBLE_EQ(engine.getPoseTimestamp(),frame.timestamp);
            Pose p{engine.getPoseTimestamp(),{output[3],output[7],output[11]},Eigen::Matrix3d::Identity()};
            for(int row=0;row<3;++row)for(int col=0;col<3;++col)p.r(row,col)=output[row*4+col];result.push_back(p);
        }
    }
    return result;
}
}
TEST(VIOParityTest,BothProduceTrajectories){ASSERT_FALSE(run(false,150).empty());ASSERT_FALSE(run(true,150).empty());}
TEST(VIOParityTest,TrajectoryComparison){
    const auto direct=run(false,150),native=run(true,150);
    ASSERT_GT(direct.size(),0u);ASSERT_EQ(direct.size(),native.size());
    size_t matches=0;
    for(size_t i=0;i<direct.size();++i){ASSERT_DOUBLE_EQ(direct[i].time,native[i].time);EXPECT_NEAR((direct[i].p-native[i].p).norm(),0,1e-10);EXPECT_NEAR((direct[i].r-native[i].r).norm(),0,1e-10);++matches;}
    EXPECT_GT(matches,0u);
}
TEST(VIOParityTest,VIOEngineSanity){const auto trajectory=run(false,150);ASSERT_GT(trajectory.size(),0u);for(const auto& p:trajectory){EXPECT_TRUE(p.p.allFinite());EXPECT_TRUE(p.r.allFinite());EXPECT_NEAR((p.r.transpose()*p.r-Eigen::Matrix3d::Identity()).norm(),0,1e-8);EXPECT_NEAR(p.r.determinant(),1,1e-8);}}
