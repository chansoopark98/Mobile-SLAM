#ifndef UTILITY__MEASUREMENT_PROCESSOR_H
#define UTILITY__MEASUREMENT_PROCESSOR_H

#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace utility {

struct IMUMsg {
    double timestamp;
    double linear_acc_x, linear_acc_y, linear_acc_z;
    double angular_vel_x, angular_vel_y, angular_vel_z;
};

struct RawMeasurementMsg {
    int measurement_id = 0;
    double timestamp = -1.0;
    cv::Mat gray_image;
    std::vector<IMUMsg> imu_msg;
};

struct ImageFileData {
    double timestamp;
    std::string filename;
    std::string full_path;
};

class MeasurementProcessor {
public:
    MeasurementProcessor();
    ~MeasurementProcessor();

    // Initialization
    bool initialize(const std::string& imu_filepath, const std::string& image_csv_filepath,
                    const std::string& image_dir, const std::string& config_filepath);

    // Data loading
    bool loadImuData(const std::string& filepath);
    bool loadImageFileData(const std::string& csv_filepath, const std::string& image_dir);

    // Forward each original IMU sample once, including one future bracket.
    // Only VIOEngine owns retained future samples and endpoint interpolation.
    RawMeasurementMsg createRawMeasurementMsg(int measurement_id, const ImageFileData& image_data);
    // Optional late-start adapter: seed with the original sample immediately before the first image.
    void beginAtImageTimestamp(double timestamp);

    // Data accessors
    const std::vector<IMUMsg>& getIMUData() const {
        return imu_data_;
    }
    const std::vector<ImageFileData>& getImageFileData() const {
        return image_file_data_;
    }

    // Utility functions
    void printDataRange() const;
    void printTimeInfo(double timestamp) const;

    // Filename sanitization (public static for testability)
    static std::string cleanFilename(const std::string& filename);

private:
    // Member variables
    std::vector<IMUMsg> imu_data_;
    std::vector<ImageFileData> image_file_data_;

    size_t imu_cursor_ = 0;
};

}  // namespace utility

#endif  // UTILITY__MEASUREMENT_PROCESSOR_H
