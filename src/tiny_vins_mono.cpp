#include <iostream>
#include <memory>

#include "vio_system.h"
#include "config/config_manager.h"
#include "utility/logging.h"


int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <config_file>" << std::endl;
        std::cout << "Example: " << argv[0] << " ./config/config.yaml" << std::endl;
        return 1;
    }

    // Load configuration using ConfigManager
    std::string config_file = argv[1];
    auto& config_manager = config::ConfigManager::getInstance();
    
    if (!config_manager.loadConfiguration(config_file)) {
        LOG_ERROR("Failed to load config from " << config_file);
        return 1;
    }

    if (!config_manager.validateConfiguration()) {
        LOG_ERROR("Configuration validation failed");
        return 1;
    }

    config_manager.printConfiguration();

    // Create VIO system with dependency injection
    VIOSystem vio_system(config_manager.getConfig());
    
    if (!vio_system.initialize()) {
        LOG_ERROR("Failed to initialize VIO system");
        return 1;
    }

    // Process the sequence
    vio_system.processSequence();

    return 0;
}