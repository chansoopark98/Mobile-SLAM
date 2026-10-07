#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <cstdint>
#include <emscripten/heap.h>
#include "vio_engine.h"

using namespace emscripten;

// Wrapper functions that take uintptr_t (JS numbers) and cast to proper pointer types.
// embind cannot directly bind const double* / const uint8_t* parameters.

namespace {
bool heapRange(uintptr_t pointer, size_t bytes, size_t alignment = 1) {
    const size_t size = emscripten_get_heap_size();
    return pointer != 0 && pointer % alignment == 0 && pointer <= size && bytes <= size-pointer;
}
double getEpoch_wrapper(const VIOEngine& self) { return static_cast<double>(self.getEpoch()); }
}

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
                          heapRange(r_ic_ptr,9*sizeof(double),alignof(double)) ? reinterpret_cast<const double*>(r_ic_ptr) : nullptr,
                          heapRange(t_ic_ptr,3*sizeof(double),alignof(double)) ? reinterpret_cast<const double*>(t_ic_ptr) : nullptr,
                          acc_n, acc_w, gyr_n, gyr_w, g_norm);
}

bool processFrame_wrapper(VIOEngine& self,
                          uintptr_t gray_image_ptr, int width, int height,
                          uintptr_t imu_readings_ptr, int imu_count,
                          double image_timestamp,
                          uintptr_t pose_output_ptr) {
    const bool image_range = width > 0 && height > 0 && width <= 8192 && height <= 8192 &&
        heapRange(gray_image_ptr,static_cast<size_t>(width)*height);
    const bool imu_range = imu_count > 0 && imu_count <= 4096 &&
        heapRange(imu_readings_ptr,static_cast<size_t>(imu_count)*sizeof(IMUReading),alignof(double));
    const bool pose_range = heapRange(pose_output_ptr,16*sizeof(double),alignof(double));

    return self.processFrame(image_range ? reinterpret_cast<const uint8_t*>(gray_image_ptr) : nullptr,
                             width, height,
                             imu_range ? reinterpret_cast<const IMUReading*>(imu_readings_ptr) : nullptr,
                             imu_count,
                             image_timestamp,
                             pose_range ? reinterpret_cast<double*>(pose_output_ptr) : nullptr);
}

int getMapPoints_wrapper(const VIOEngine& self,
                         uintptr_t output_ptr, int max_count) {
    if(max_count <= 0 || max_count > 100000 ||
       !heapRange(output_ptr,static_cast<size_t>(max_count)*3*sizeof(double),alignof(double))) return 0;
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
        .function("setFThreshold", &VIOEngine::setFThreshold)
        .function("setTrackingParams", &VIOEngine::setTrackingParams)
        .function("getStatusCode", &VIOEngine::getStatusCode)
        .function("getEpoch", &getEpoch_wrapper)
        .function("getPoseTimestamp", &VIOEngine::getPoseTimestamp)
        .function("getFrameTimestamp", &VIOEngine::getFrameTimestamp)
        .function("getIMUEndpointTimestamp", &VIOEngine::getIMUEndpointTimestamp)
        .function("getLastReason", &VIOEngine::getLastReason)
        .function("getPoseFresh", &VIOEngine::getPoseFresh)
        .function("getPoseValid", &VIOEngine::getPoseValid)
        .function("getLastSolverIterations", &VIOEngine::getLastSolverIterations)
        .function("getLastSolverTermination", &VIOEngine::getLastSolverTermination)
        .function("setExecutionParams", &VIOEngine::setExecutionParams)
        .function("getExecutionSeed", &VIOEngine::getExecutionSeed)
        .function("getCVThreadCount", &VIOEngine::getCVThreadCount)
        .function("setDiagnosticCapture", &VIOEngine::setDiagnosticCapture)
        .function("setBenchmarkSolverProfile", &VIOEngine::setBenchmarkSolverProfile)
        .function("getBenchmarkSolverProfile", &VIOEngine::getBenchmarkSolverProfile)
        .function("getFeatureDiagnostics", &VIOEngine::getFeatureDiagnostics)
        .function("reset", &VIOEngine::reset)
        .function("setPnPParams", &VIOEngine::setPnPParams);
}
