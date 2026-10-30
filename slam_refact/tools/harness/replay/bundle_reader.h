// Replay bundle reader (architecture.md §6.1-6.2, WP5). Reads the text bundle written by
// slam_refact/tools/harness/prepare_inputs.py and produces the frozen public input types:
// DeviceMotionSample / DeviceOrientationSample for GlueRunner::Push*, and FrameInput (time, image,
// injected pyramid) for GlueRunner::ProcessFrame. Format conversion only; func 647 and sensors.ts
// stay inside the engine's SensorIntake (D-14).

#ifndef SLAM_REFACT_TOOLS_HARNESS_REPLAY_BUNDLE_READER_H_
#define SLAM_REFACT_TOOLS_HARNESS_REPLAY_BUNDLE_READER_H_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "slam_refact/engine.h"
#include "slam_refact/engine_config.h"
#include "slam_refact/sensor_types.h"

namespace slam_refact::harness {

// One W3C packet: dispatched as DeviceMotion then DeviceOrientation at the same receive time
// (page-runtime.js emit()).
struct SensorRow {
  bool bootstrap = false;
  DeviceMotionSample motion;
  DeviceOrientationSample orientation;
};

struct FrameRow {
  bool bootstrap = false;
  int index = -1;  // input frame index; -1 for the bootstrap stage
  int64_t time_ns = 0;
  int64_t video_time_ns = 0;
  // Optional column 9: TrackingSensorFrame.timeNanos as 647 picks it (frameTimestampNanos when non-zero,
  // else timestampNanos). 0 = use time_ns. Sensor binding always uses time_ns.
  int64_t frame_time_ns = 0;
  double t = 0.0;        // fixture seconds (trace field t)
  bool process = true;   // false = dropped in the reference run (drop-mask mode)
  std::string image;     // binary PGM (P5, maxval 255)
  std::string pyramid;   // gzip RGBA atlas (1024 x rows); empty = none
};

struct ReplayBundle {
  std::string dir;
  std::map<std::string, std::string> config;  // engine-config.tsv
  std::vector<SensorRow> sensors;              // bootstrap rows first, then run rows in time order
  std::vector<FrameRow> frames;                // bootstrap row first

  EngineConfig MakeEngineConfig() const;
  GlueConfig MakeGlueConfig() const;
};

// Owned pixel buffers of one stage; views point into them.
struct LoadedImages {
  std::vector<uint8_t> gray;
  int width = 0;
  int height = 0;
  std::vector<uint8_t> atlas;
  int atlas_rows = 0;
};

// All functions return false and set *error on malformed input.
bool LoadBundle(const std::string& dir, ReplayBundle* out, std::string* error);
bool ReadPgm(const std::string& path, std::vector<uint8_t>* pixels, int* width, int* height,
             std::string* error);
// Gzip RGBA atlas of width 1024 (S1 §2.2): rows = bytes / 4096.
bool ReadGzipAtlas(const std::string& path, std::vector<uint8_t>* rgba, int* rows,
                   std::string* error);
bool LoadImages(const FrameRow& frame, LoadedImages* out, std::string* error);

// FrameInput carrying the stage time and image sources (gray + optional injected pyramid).
// device_pose and imu_events are left to SensorIntake inside GlueRunner::ProcessFrame (D-14).
FrameInput MakeFrameInput(const FrameRow& frame, const LoadedImages& images);

}  // namespace slam_refact::harness

#endif  // SLAM_REFACT_TOOLS_HARNESS_REPLAY_BUNDLE_READER_H_
