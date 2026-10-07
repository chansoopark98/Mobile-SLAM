#include "config/config_manager.h"
#include <iostream>
#include <cmath>

namespace config {

ConfigManager& ConfigManager::getInstance() {
    static ConfigManager instance;
    return instance;
}

bool ConfigManager::loadConfiguration(const std::string& config_path) {
    std::unique_lock<std::mutex> lock(config_mutex_);
    auto candidate = std::make_shared<utility::Config>();
    
    if (!candidate->loadFromYaml(config_path)) {
        std::cerr << "Failed to load configuration from: " << config_path << std::endl;
        return false;
    }
    
    const auto previous = config_;
    config_ = candidate;
    if (!validateCameraParams() || !validateEstimatorParams() || !validateFeatureTrackerParams()) {
        config_ = previous;
        return false;
    }
    config_file_path_ = config_path;
    
    // Basic validation: check if dataset path exists
    if (!config_->dataset_path.empty()) {
        // Simple warning if dataset path might not exist (without filesystem dependency)
        std::cout << "Configuration warnings:" << std::endl;
        std::cout << "  Warning: dataset_path may not exist: " << config_->dataset_path << std::endl;
    }
    
    std::cout << "Configuration loaded successfully from: " << config_path << std::endl;
    
    // Notify all registered callbacks
    lock.unlock();
    notifyChange("configuration_loaded");
    
    return true;
}

std::shared_ptr<utility::Config> ConfigManager::getConfig() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_;
}

bool ConfigManager::isLoaded() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_ != nullptr;
}

void ConfigManager::printConfiguration() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    if (config_) {
        config_->print();
    } else {
        std::cout << "No configuration loaded." << std::endl;
    }
}

void ConfigManager::registerChangeCallback(std::function<void(const std::string&)> callback) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    change_callbacks_.push_back(callback);
}

bool ConfigManager::validateConfiguration() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    if (!config_) {
        return false;
    }
    
    // Basic validation - check if all subsystems have valid parameters
    return validateCameraParams() && validateEstimatorParams() && validateFeatureTrackerParams();
}

bool ConfigManager::saveConfiguration(const std::string& config_path) const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    if (!config_) {
        std::cerr << "No configuration to save." << std::endl;
        return false;
    }
    
    // Note: This would require implementing a YAML writer
    // For now, we'll just copy the original file if it's the same path
    if (config_path == config_file_path_) {
        std::cout << "Configuration is already at the specified path." << std::endl;
        return true;
    }
    
    std::cerr << "Saving configuration to different paths not yet implemented." << std::endl;
    return false;
}

void ConfigManager::notifyChange(const std::string& key) {
    std::vector<std::function<void(const std::string&)>> callbacks;
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        callbacks = change_callbacks_;
    }
    for (const auto& callback : callbacks) {
        try {
            callback(key);
        } catch (const std::exception& e) {
            std::cerr << "Error in configuration change callback: " << e.what() << std::endl;
        }
    }
}

bool ConfigManager::validateCameraParams() const {
    if (!config_) return false;
    const auto& c=config_->camera;
    return std::isfinite(c.fx) && c.fx>0 && std::isfinite(c.fy) && c.fy>0 &&
        std::isfinite(c.cx) && std::isfinite(c.cy) && std::isfinite(c.row) && c.row>0 &&
        std::isfinite(c.col) && c.col>0 && c.r_ic.allFinite() && c.t_ic.allFinite() &&
        (c.r_ic.transpose()*c.r_ic-Eigen::Matrix3d::Identity()).norm()<1e-6 &&
        std::abs(c.r_ic.determinant()-1)<1e-6;
}
bool ConfigManager::validateEstimatorParams() const {
    if (!config_) return false;
    const auto& e=config_->estimator;
    return e.window_size>0 && e.window_size<=utility::WINDOW_SIZE && e.num_iterations>0 &&
        std::isfinite(e.solver_time) && e.solver_time>0 && std::isfinite(e.init_depth) && e.init_depth>0 &&
        e.g.allFinite() && e.g.norm()>0 && std::isfinite(e.acc_n) && e.acc_n>0 &&
        std::isfinite(e.acc_w) && e.acc_w>=0 && std::isfinite(e.gyr_n) && e.gyr_n>0 &&
        std::isfinite(e.gyr_w) && e.gyr_w>=0;
}
bool ConfigManager::validateFeatureTrackerParams() const {
    if (!config_) return false;
    const auto& f=config_->feature_tracker;
    return f.max_cnt>0 && f.min_dist>0 && f.window_size>0 &&
        std::isfinite(f.f_threshold) && f.f_threshold>0 && f.lk_iterations>0 &&
        std::isfinite(f.lk_eps) && f.lk_eps>0;
}
} // namespace config
