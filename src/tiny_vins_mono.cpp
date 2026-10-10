#include <iostream>
#include <memory>
#include <stdexcept>

#include "vio_system.h"
#include "config/config_manager.h"
#include "utility/logging.h"


int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <config_file> [--headless] [--seed N] [--cv-threads 1..4] [--dump-inputs]" << std::endl;
        std::cout << "Example: " << argv[0] << " ./config/config.yaml" << std::endl;
        return 1;
    }

    try {
        bool headless=false, dump_inputs=false, threads_specified=false;
        int seed=-1, cv_threads=1;
        for (int i=2;i<argc;++i) {
            const std::string option=argv[i];
            if (option=="--headless") headless=true;
            else if (option=="--dump-inputs") dump_inputs=true;
            else if ((option=="--seed" || option=="--cv-threads") && i+1<argc) {
                const std::string value=argv[++i];
                size_t consumed=0;
                const int parsed=std::stoi(value,&consumed);
                if (consumed!=value.size()) throw std::runtime_error("Invalid integer for "+option);
                if (option=="--seed") { seed=parsed; if (seed<0) throw std::runtime_error("Seed must be nonnegative"); }
                else { cv_threads=parsed; threads_specified=true; }
            } else throw std::runtime_error("Unknown or incomplete option: "+option);
        }
        if (threads_specified && seed<0) throw std::runtime_error("--cv-threads requires explicit --seed");
        if (cv_threads<1 || cv_threads>4) throw std::runtime_error("--cv-threads must be between 1 and 4");
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
        VIOSystem vio_system(config_manager.getConfig(), headless, seed, cv_threads, dump_inputs);

        if (!vio_system.initialize()) {
            LOG_ERROR("Failed to initialize VIO system");
            return 1;
        }

        // Process the sequence
        vio_system.processSequence();

        return 0;
    } catch (const std::exception& error) {
        LOG_ERROR(error.what());
        return 1;
    }
}
