#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <cstdint>
#include "vio_engine.h"

using namespace emscripten;

// Wrapper functions that take uintptr_t (JS numbers) and cast to proper pointer types.
// embind cannot directly bind const double* / const uint8_t* parameters.

bool configure_wrapper(VIOEngine& self,
                       int width, int height,
                       double fx, double fy, double cx, double cy,
                       int model_type,
                       double k2, double k3, double k4, double k5,
                       uintptr_t r_ic_ptr, uintptr_t t_ic_ptr,
                       double acc_n, double acc_w,
                       double gyr_n, double gyr_w,
                       double g_norm) {
    return self.configure(width, height, fx, fy, cx, cy,
                          model_type, k2, k3, k4, k5,
                          reinterpret_cast<const double*>(r_ic_ptr),
                          reinterpret_cast<const double*>(t_ic_ptr),
                          acc_n, acc_w, gyr_n, gyr_w, g_norm);
}

bool processFrame_wrapper(VIOEngine& self,
                          uintptr_t gray_image_ptr, int width, int height,
                          uintptr_t imu_readings_ptr, int imu_count,
                          double image_timestamp,
                          uintptr_t pose_output_ptr) {
    // Validate inputs at the WASM boundary
    static constexpr int kMaxIMUReadings = 512;
    if (imu_count < 0) imu_count = 0;
    if (imu_count > kMaxIMUReadings) imu_count = kMaxIMUReadings;
    if (gray_image_ptr == 0 || pose_output_ptr == 0) return false;

    return self.processFrame(reinterpret_cast<const uint8_t*>(gray_image_ptr),
                             width, height,
                             reinterpret_cast<const IMUReading*>(imu_readings_ptr),
                             imu_count,
                             image_timestamp,
                             reinterpret_cast<double*>(pose_output_ptr));
}

int getMapPoints_wrapper(const VIOEngine& self,
                         uintptr_t output_ptr, int max_count) {
    return self.getMapPoints(reinterpret_cast<double*>(output_ptr), max_count);
}

EMSCRIPTEN_BINDINGS(VIOModule) {
    class_<VIOEngine>("VIOEngine")
        .constructor()
        .function("configure", &configure_wrapper)
        .function("processFrame", &processFrame_wrapper)
        .function("isInitialized", &VIOEngine::isInitialized)
        .function("getFeaturePointCount", &VIOEngine::getFeaturePointCount)
        .function("getMapPoints", &getMapPoints_wrapper)
        .function("setMobileParams", &VIOEngine::setMobileParams)
        .function("getStatusCode", &VIOEngine::getStatusCode)
        .function("reset", &VIOEngine::reset);
}
