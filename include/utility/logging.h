#ifndef UTILITY__LOGGING_H
#define UTILITY__LOGGING_H

#include <iostream>
#include <cstring>

// Log levels: 0=DEBUG, 1=INFO, 2=WARN, 3=ERROR
#ifndef SLAM_LOG_LEVEL
  #ifdef NDEBUG
    #define SLAM_LOG_LEVEL 1  // Release: INFO and above
  #else
    #define SLAM_LOG_LEVEL 0  // Debug: all levels
  #endif
#endif

// Extract filename from __FILE__
#define SLAM_LOG_FILENAME \
    (std::strrchr(__FILE__, '/') ? std::strrchr(__FILE__, '/') + 1 : __FILE__)

#define SLAM_LOG(level_str, msg) \
    std::cerr << "[" level_str "] " << SLAM_LOG_FILENAME << ":" << __LINE__ << " " << msg << std::endl

#if SLAM_LOG_LEVEL <= 0
  #define LOG_DEBUG(msg) SLAM_LOG("DEBUG", msg)
#else
  #define LOG_DEBUG(msg) ((void)0)
#endif

#if SLAM_LOG_LEVEL <= 1
  #define LOG_INFO(msg)  SLAM_LOG("INFO ", msg)
#else
  #define LOG_INFO(msg)  ((void)0)
#endif

#if SLAM_LOG_LEVEL <= 2
  #define LOG_WARN(msg)  SLAM_LOG("WARN ", msg)
#else
  #define LOG_WARN(msg)  ((void)0)
#endif

#if SLAM_LOG_LEVEL <= 3
  #define LOG_ERROR(msg) SLAM_LOG("ERROR", msg)
#else
  #define LOG_ERROR(msg) ((void)0)
#endif

#endif  // UTILITY__LOGGING_H
