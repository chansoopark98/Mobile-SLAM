#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <cmath>

#include "utility/config.h"
#include "utility/logging.h"
#include "utility/measurement_processor.h"

namespace utility {

MeasurementProcessor::MeasurementProcessor() = default;

MeasurementProcessor::~MeasurementProcessor() {}

bool MeasurementProcessor::initialize(const std::string& imu_filepath, const std::string& image_csv_filepath,
                                      const std::string& image_dir, const std::string& config_filepath) {
    LOG_INFO("=== MeasurementProcessor Initialization ===");

    // Load configuration
    if (!g_config.loadFromYaml(config_filepath)) {
        LOG_ERROR("Failed to load config from " << config_filepath);
        return false;
    }

    // Print configuration for debugging
    g_config.print();

    imu_cursor_ = 0;

    // Load data
    if (!loadImuData(imu_filepath)) {
        LOG_ERROR("Failed to load IMU data");
        return false;
    }

    if (!loadImageFileData(image_csv_filepath, image_dir)) {
        LOG_ERROR("Failed to load image data");
        return false;
    }

    // Print data range
    printDataRange();

    return true;
}

bool MeasurementProcessor::loadImuData(const std::string& filepath) {
    LOG_INFO("1. Loading IMU data...");

    std::ifstream file(filepath);
    if (!file.is_open()) {
        LOG_ERROR("Cannot open IMU file: " << filepath);
        return false;
    }

    imu_data_.clear();
    imu_cursor_ = 0;
    std::string line;
    // Skip header
    std::getline(file, line);

    while (std::getline(file, line)) {
        if (line.empty())
            continue;

        try {
            std::istringstream iss(line);
            std::string token;
            double values[7];
            for (double& value : values) {
                if (!std::getline(iss, token, ',')) throw std::runtime_error("missing IMU field");
                size_t consumed = 0;
                value = std::stod(token, &consumed);
                if (!std::isfinite(value) || token.find_first_not_of(" \t\r", consumed) != std::string::npos)
                    throw std::runtime_error("invalid IMU field");
            }
            IMUMsg data{values[0]*1e-9, values[4], values[5], values[6], values[1], values[2], values[3]};
            if (data.timestamp < 0 || (!imu_data_.empty() && data.timestamp <= imu_data_.back().timestamp))
                throw std::runtime_error("nonmonotonic IMU timestamp");
            imu_data_.push_back(data);
        } catch (const std::exception& e) {
            LOG_WARN("Skipping malformed IMU line: " << e.what());
            continue;
        }
    }

    file.close();
    LOG_INFO("Loaded " << imu_data_.size() << " IMU data entries");
    return true;
}

bool MeasurementProcessor::loadImageFileData(const std::string& csv_filepath, const std::string& image_dir) {
    LOG_INFO("2. Loading image data...");

    std::ifstream file(csv_filepath);
    if (!file.is_open()) {
        LOG_ERROR("Cannot open image CSV file: " << csv_filepath);
        return false;
    }

    image_file_data_.clear();
    std::string line;
    // Skip header
    std::getline(file, line);

    while (std::getline(file, line)) {
        if (line.empty())
            continue;

        try {
            std::istringstream iss(line);
            std::string token;
            ImageFileData data{};

            // Convert timestamp [ns] to seconds
            if (std::getline(iss, token, ',')) {
                data.timestamp = std::stod(token) * 1e-9;  // Canonical browser Number(ns) * 1e-9 conversion.
            } else throw std::runtime_error("missing image timestamp");
            if (!std::isfinite(data.timestamp) || data.timestamp < 0 ||
                (!image_file_data_.empty() && data.timestamp <= image_file_data_.back().timestamp))
                throw std::runtime_error("invalid image timestamp");

            // filename
            if (std::getline(iss, token)) {
                data.filename = cleanFilename(token);
                if (data.filename.empty()) {
                    LOG_WARN("Skipping image entry with invalid filename");
                    continue;
                }
                data.full_path = image_dir + "/" + data.filename;
            } else throw std::runtime_error("missing image filename");

            image_file_data_.push_back(data);
        } catch (const std::exception& e) {
            LOG_WARN("Skipping malformed image line: " << e.what());
            continue;
        }
    }

    file.close();
    LOG_INFO("Loaded " << image_file_data_.size() << " image data entries");
    return true;
}

std::string MeasurementProcessor::cleanFilename(const std::string& filename) {
    std::string cleaned = filename;
    // Remove leading and trailing whitespace and newlines
    cleaned.erase(cleaned.find_last_not_of(" \n\r\t") + 1);
    cleaned.erase(0, cleaned.find_first_not_of(" \n\r\t"));

    // Reject path traversal sequences
    if (cleaned.find("..") != std::string::npos) {
        LOG_WARN("Path traversal detected in filename: " << filename);
        return "";
    }

    // Reject absolute paths
    if (!cleaned.empty() && cleaned[0] == '/') {
        LOG_WARN("Absolute path rejected in filename: " << filename);
        return "";
    }

    return cleaned;
}

RawMeasurementMsg MeasurementProcessor::createRawMeasurementMsg(int measurement_id, const ImageFileData& image_data) {
    RawMeasurementMsg result;
    result.measurement_id = measurement_id;
    result.timestamp = image_data.timestamp;
    result.gray_image = cv::imread(image_data.full_path, cv::IMREAD_GRAYSCALE);
    if (result.gray_image.empty()) return result;
    if (imu_cursor_ > 0 && imu_data_[imu_cursor_-1].timestamp > image_data.timestamp) return result;
    while (imu_cursor_ < imu_data_.size()) {
        const auto& sample = imu_data_[imu_cursor_++];
        result.imu_msg.push_back(sample);
        if (sample.timestamp > image_data.timestamp) break;
    }
    return result;
}

void MeasurementProcessor::beginAtImageTimestamp(double timestamp) {
    const auto next=std::upper_bound(imu_data_.begin(),imu_data_.end(),timestamp,
        [](double time,const IMUMsg& sample){ return time<sample.timestamp; });
    imu_cursor_=static_cast<size_t>(next-imu_data_.begin());
    if(imu_cursor_>0) --imu_cursor_;
}

void MeasurementProcessor::printDataRange() const {
    std::cout << "\n3. Data range information:" << std::endl;
    if (!imu_data_.empty()) {
        std::cout << "IMU data range: ";
        printTimeInfo(imu_data_.front().timestamp);
        std::cout << " ~ ";
        printTimeInfo(imu_data_.back().timestamp);
        std::cout << std::endl;
    }

    if (!image_file_data_.empty()) {
        std::cout << "Image data range: ";
        printTimeInfo(image_file_data_.front().timestamp);
        std::cout << " ~ ";
        printTimeInfo(image_file_data_.back().timestamp);
        std::cout << std::endl;
    }
}

void MeasurementProcessor::printTimeInfo(double timestamp) const {
    std::time_t time_t = static_cast<std::time_t>(timestamp);
    std::tm* tm = std::gmtime(&time_t);

    std::cout << std::fixed << std::setprecision(6) << timestamp << " (" << std::put_time(tm, "%Y-%m-%d %H:%M:%S")
              << " UTC)";
}

}  // namespace utility
