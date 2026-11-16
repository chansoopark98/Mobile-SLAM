#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <yaml-cpp/yaml.h>
#include <opencv2/imgcodecs.hpp>
#include "utility/measurement_processor.h"
#include "vio_engine.h"
#include "vio_system.h"

class NativeAdapter : public ::testing::Test {
protected:
    std::filesystem::path root;
    std::filesystem::path previous_cwd;
    void SetUp() override {
        root=std::filesystem::temp_directory_path()/"mobile-slam-native-adapter";
        std::filesystem::create_directories(root);
        previous_cwd=std::filesystem::current_path();
        std::filesystem::current_path(root);
        cv::imwrite((root/"frame.png").string(),cv::Mat(96,96,CV_8UC1,cv::Scalar(0)));
    }
    void TearDown() override {
        std::filesystem::current_path(previous_cwd);
        std::filesystem::remove_all(root);
    }
};

TEST_F(NativeAdapter, OriginalIMUBracketIsForwardedExactlyOnceWithoutFeatureTracking) {
    std::ofstream file(root/"imu.csv");
    file<<"timestamp,gx,gy,gz,ax,ay,az\n";
    for(int i=0;i<=10;++i) file<<i*10000000<<",0,0,0,"<<i<<",0,9.81\n";
    file.close();
    utility::MeasurementProcessor processor;
    ASSERT_TRUE(processor.loadImuData((root/"imu.csv").string()));
    const auto frame=[&](double timestamp){ return processor.createRawMeasurementMsg(1,{timestamp,"frame.png",(root/"frame.png").string()}); };
    const auto first=frame(.015);
    ASSERT_EQ(first.imu_msg.size(),3u);
    EXPECT_DOUBLE_EQ(first.imu_msg.back().timestamp,.02);
    EXPECT_DOUBLE_EQ(first.imu_msg.back().linear_acc_x,2);
    EXPECT_EQ(first.gray_image.cols,96);
    EXPECT_TRUE(frame(.017).imu_msg.empty());
    const auto second=frame(.025);
    ASSERT_EQ(second.imu_msg.size(),1u);
    EXPECT_DOUBLE_EQ(second.imu_msg[0].timestamp,.03);
    EXPECT_DOUBLE_EQ(second.imu_msg[0].linear_acc_x,3);
    const auto third=frame(.04);
    ASSERT_EQ(third.imu_msg.size(),2u);
    EXPECT_DOUBLE_EQ(third.imu_msg[0].timestamp,.04);
    EXPECT_DOUBLE_EQ(third.imu_msg[1].timestamp,.05);
}

TEST_F(NativeAdapter, MissingFieldsNanAndDuplicateSamplesNeverProduceUninitializedReadings) {
    std::ofstream file(root/"imu.csv");
    file<<"timestamp,gx,gy,gz,ax,ay,az\n"
        <<"0,0,0,0,1,0,9.81\n"
        <<"10000000,0,0\n"
        <<"20000000,0,0,0,nan,0,9.81\n"
        <<"0,0,0,0,999,0,9.81\n"
        <<"30000000,0,0,0,3,0,9.81\n";
    file.close();
    utility::MeasurementProcessor processor;
    ASSERT_TRUE(processor.loadImuData((root/"imu.csv").string()));
    ASSERT_EQ(processor.getIMUData().size(),2u);
    EXPECT_DOUBLE_EQ(processor.getIMUData()[0].linear_acc_x,1);
    EXPECT_DOUBLE_EQ(processor.getIMUData()[1].linear_acc_x,3);
}

TEST_F(NativeAdapter, TimestampConversionMatchesBrowserNumberOracleAtRealEpochMagnitude) {
    // Golden value from Number('1520216380280000000') * 1e-9, exactly one ULP above division.
    constexpr double expected=1520216380.2800002;
    std::ofstream imu(root/"imu.csv");
    imu<<"timestamp,gx,gy,gz,ax,ay,az\n1520216380280000000,0,0,0,0,0,9.81\n";
    imu.close();
    std::ofstream image(root/"images.csv");
    image<<"timestamp,filename\n1520216380280000000,frame.png\n";
    image.close();
    utility::MeasurementProcessor processor;
    ASSERT_TRUE(processor.loadImuData((root/"imu.csv").string()));
    ASSERT_TRUE(processor.loadImageFileData((root/"images.csv").string(),root.string()));
    ASSERT_EQ(processor.getIMUData().size(),1u);
    ASSERT_EQ(processor.getImageFileData().size(),1u);
    EXPECT_EQ(processor.getIMUData()[0].timestamp,expected);
    EXPECT_EQ(processor.getImageFileData()[0].timestamp,expected);
}

TEST_F(NativeAdapter, LateSequenceStartUsesOriginalPastSeedAndFutureBracket) {
    std::ofstream file(root/"imu.csv");
    file<<"timestamp,gx,gy,gz,ax,ay,az\n";
    for(int i=0;i<=10;++i) file<<i*10000000<<",0,0,0,"<<i<<",0,9.81\n";
    file.close();
    utility::MeasurementProcessor processor;
    ASSERT_TRUE(processor.loadImuData((root/"imu.csv").string()));
    processor.beginAtImageTimestamp(.075);
    const auto first=processor.createRawMeasurementMsg(1,{.075,"frame.png",(root/"frame.png").string()});
    ASSERT_EQ(first.imu_msg.size(),2u);
    EXPECT_DOUBLE_EQ(first.imu_msg[0].timestamp,.07);
    EXPECT_DOUBLE_EQ(first.imu_msg[0].linear_acc_x,7);
    EXPECT_DOUBLE_EQ(first.imu_msg[1].timestamp,.08);
    EXPECT_TRUE(processor.createRawMeasurementMsg(2,{.077,"frame.png",(root/"frame.png").string()}).imu_msg.empty());
}

TEST_F(NativeAdapter, RawDatasetAdapterAndDirectEngineObserveIdenticalInputState) {
    std::ofstream file(root/"imu.csv");
    file<<"timestamp,gx,gy,gz,ax,ay,az\n";
    for(int i=0;i<=30;++i) file<<i*10000000<<",0,0,0,0,0,9.81\n";
    file.close();
    utility::MeasurementProcessor processor;
    ASSERT_TRUE(processor.loadImuData((root/"imu.csv").string()));
    VIOEngine from_dataset,direct;
    const double identity[9]={1,0,0,0,1,0,0,0,1},zero[3]={0,0,0};
    for(auto* engine : {&from_dataset,&direct})
        ASSERT_TRUE(engine->configure(96,96,100,100,48,48,2,0,0,0,0,identity,zero,.1,.001,.01,.0001,9.81));
    cv::Mat image=cv::imread((root/"frame.png").string(),cv::IMREAD_GRAYSCALE);
    // Exact packet timestamps from the browser Number(ns)*1e-9 oracle, including the right bracket.
    const std::vector<std::vector<double>> golden_packets={
        {0,.01,.02},{},{.030000000000000002},{.04,.05},
        {.060000000000000005,.07,.08},
        {.09000000000000001,.1,.11,.12000000000000001,.13,.14,.15000000000000002}};
    size_t frame_index=0;
    for(double timestamp : {.015,.017,.025,.04,.075,.15}) {
        auto raw=processor.createRawMeasurementMsg(1,{timestamp,"frame.png",(root/"frame.png").string()});
        std::vector<IMUReading> adapted,expected;
        for(const auto& sample:raw.imu_msg) adapted.push_back({sample.timestamp,sample.linear_acc_x,sample.linear_acc_y,
            sample.linear_acc_z,sample.angular_vel_x,sample.angular_vel_y,sample.angular_vel_z});
        for(double time:golden_packets[frame_index++]) expected.push_back({time,0,0,9.81,0,0,0});
        ASSERT_EQ(adapted.size(),expected.size());
        for(size_t i=0;i<adapted.size();++i) EXPECT_EQ(adapted[i].timestamp,expected[i].timestamp);
        double pose_a[16],pose_b[16];
        EXPECT_EQ(from_dataset.processFrame(raw.gray_image.data,96,96,adapted.data(),adapted.size(),timestamp,pose_a),
                  direct.processFrame(image.data,96,96,expected.data(),expected.size(),timestamp,pose_b));
        EXPECT_EQ(from_dataset.getLastReason(),direct.getLastReason());
        EXPECT_EQ(from_dataset.getStatusCode(),direct.getStatusCode());
        EXPECT_DOUBLE_EQ(from_dataset.getFrameTimestamp(),direct.getFrameTimestamp());
    }
}

TEST_F(NativeAdapter, ActualHeadlessVIOSystemDelegatesDecodedFramesToEngine) {
    const auto dataset=root/"dataset";
    std::filesystem::create_directories(dataset/"mav0/imu0");
    std::filesystem::create_directories(dataset/"mav0/cam0/data");
    cv::imwrite((dataset/"mav0/cam0/data/frame.png").string(),cv::Mat(96,96,CV_8UC1,cv::Scalar(0)));
    std::ofstream imu(dataset/"mav0/imu0/data.csv");
    imu<<"timestamp,gx,gy,gz,ax,ay,az\n";
    for(int i=0;i<=30;++i) imu<<i*10000000<<",0,0,0,0,0,9.81\n";
    imu.close();
    std::ofstream images(dataset/"mav0/cam0/data.csv");
    images<<"timestamp,filename\n0,frame.png\n50000000,frame.png\n100000000,frame.png\n150000000,missing.png\n";
    images.close();
    const auto yaml=root/"camera.yaml";
    std::ofstream config(yaml);
    config<<"%YAML:1.0\n---\nmodel_type: PINHOLE\ncamera_name: camera\nimage_width: 96\nimage_height: 96\n"
          <<"projection_parameters: {fx: 100.0, fy: 100.0, cx: 48.0, cy: 48.0}\n"
          <<"distortion_parameters: {k1: 0.0, k2: 0.0, p1: 0.0, p2: 0.0}\n"
          <<"dataset_path: "<<dataset.string()<<"\nframe_skip: 0\nshow_track: 0\n";
    config.close();
    auto parameters=std::make_shared<utility::Config>();
    ASSERT_TRUE(parameters->loadFromYaml(yaml.string()));
    VIOSystem adapter(parameters,true);
    ASSERT_TRUE(adapter.initialize());
    adapter.processSequence();
    EXPECT_DOUBLE_EQ(adapter.getEngine().getFrameTimestamp(),.15);
    EXPECT_EQ(adapter.getEngine().getLastReason(),"invalid_frame");
    EXPECT_FALSE(adapter.getEngine().getPoseFresh());
    EXPECT_EQ(adapter.getEngine().getEpoch(),1u);
    const auto log_dir=std::filesystem::directory_iterator(root/"logs")->path();
    ASSERT_TRUE(std::filesystem::exists(log_dir/"frames.csv"));
    const auto summary=YAML::LoadFile((log_dir/"summary.json").string());
    EXPECT_EQ(summary["frames_available"].as<int>(),4);
    EXPECT_EQ(summary["frames_scheduled"].as<int>(),4);
    EXPECT_EQ(summary["frames_processed"].as<int>(),4);
    EXPECT_EQ(summary["frames_decoded"].as<int>(),3);
    EXPECT_EQ(summary["missing_images"].as<int>(),1);
    EXPECT_EQ(summary["imu_available"].as<int>(),31);
    EXPECT_EQ(summary["imu_delivered"].as<int>(),12);
    EXPECT_EQ(summary["pose_count"].as<int>(),0);
    EXPECT_EQ(summary["first_pose_frame"].as<int>(),-1);
    EXPECT_EQ(summary["pose_frame"].as<std::string>(),"T_W_C");
    const auto effective=YAML::LoadFile((log_dir/"effective-config.json").string());
    EXPECT_EQ(effective["params"]["lk_iterations"].as<int>(),30);
    EXPECT_DOUBLE_EQ(effective["params"]["lk_eps"].as<double>(),.01);
    std::ifstream frames(log_dir/"frames.csv");
    std::string line;
    int rows=0;
    while(std::getline(frames,line)) ++rows;
    EXPECT_EQ(rows,5); // Header plus every scheduled image, including its decode failure.
}

TEST_F(NativeAdapter, ConsecutiveLoggersCannotOverwriteEachOther) {
    utility::TestResultLogger first,second;
    ASSERT_TRUE(first.initialize("absent.yaml"));
    ASSERT_TRUE(second.initialize("absent.yaml"));
    EXPECT_NE(first.getLogDirectory(),second.getLogDirectory());
}

TEST_F(NativeAdapter, TrajectorySerializationPreservesDoublePrecision) {
    utility::TestResultLogger logger;
    ASSERT_TRUE(logger.initialize("absent.yaml"));
    constexpr double timestamp=1520216380.2800002;
    const Eigen::Vector3d position(.123456789012345,-1.23456789012345,2.34567890123456);
    logger.addPose(position,Eigen::Matrix3d::Identity(),timestamp);
    logger.saveTrajectoryToFile();
    std::ifstream trajectory(logger.getLogDirectory()+"/trajectory_pose.txt");
    std::string header;
    std::getline(trajectory,header);
    double actual_time,x,y,z;
    trajectory>>actual_time>>x>>y>>z;
    EXPECT_DOUBLE_EQ(actual_time,timestamp);
    EXPECT_DOUBLE_EQ(x,position.x());
    EXPECT_DOUBLE_EQ(y,position.y());
    EXPECT_DOUBLE_EQ(z,position.z());
}
