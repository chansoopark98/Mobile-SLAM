// reality.lighting exposure (spec 06 §4).

#include "engine/lighting.h"

#include <array>
#include <cstdint>

#include "c8/pixels/pixels.h"
#include "c8/task-queue.h"
#include "c8/thread-pool.h"
#include "common/check.h"
#include "reality/engine/lighting/lighting-estimator.h"

namespace slam_refact {

float ExposureScore(int rows, int cols, int row_bytes, const uint8_t* pixels, int num_channels) {
  // The vendored c8::LightingEstimator (reality/engine/lighting/lighting-estimator.cc) calls
  // c8::estimateExposureScore (pixel-transforms.cc:1927-1956) on every non-NEON target. The tasks
  // run one after the other (XREngine's thread pool is single-threaded under JAVASCRIPT,
  // thread-pool.cc:27) and the integer sum does not depend on their order.
  c8::ConstYPlanePixels src(rows, cols, row_bytes, pixels);
  c8::TaskQueue task_queue;
  c8::ThreadPool thread_pool(1);
  return c8::LightingEstimator::estimateLighting(src, &task_queue, &thread_pool, num_channels);
}

float EstimateExposure(const c8::Gr8Pyramid& pyramid) {
  SR_CHECK(!pyramid.levels.empty(), "EstimateExposure: pyramid without levels");
  // lighting-request-executor.cc:75-87: PYRAMID_NUM_CHANNELS = 4, last level, its top-left
  // pixel in the RGBA atlas (lighting-request-executor.h:35).
  constexpr int kPyramidNumChannels = 4;
  const c8::LevelLayout& last_level = pyramid.levels[pyramid.levels.size() - 1];
  const int rows = last_level.h;
  const int cols = last_level.w;
  const int bytes_per_row = pyramid.data.rowBytes();
  const uint8_t* row_start = pyramid.data.pixels() +
                             static_cast<int64_t>(last_level.r) * bytes_per_row +
                             static_cast<int64_t>(last_level.c) * kPyramidNumChannels;
  return ExposureScore(rows, cols, bytes_per_row, row_start, kPyramidNumChannels);
}

}  // namespace slam_refact
