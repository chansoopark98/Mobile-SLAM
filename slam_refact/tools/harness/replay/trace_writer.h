// Trace writer of the replay runner (architecture.md §6.3, WP5). Writes GlueRunner output in the
// schema of the original headless reference runtime trace (slam_refact/tools/re/runtime/run-reference.mjs):
// the same 18 keys in the same order and meaning, browser-only timing keys null, engine-only debug
// keys prefixed "sr". The world point bookkeeping (wpAdded / wpRemoved / wpMeanConfidence and the
// sampled-point file) follows page-runtime.js onUpdate line by line.

#ifndef SLAM_REFACT_TOOLS_HARNESS_REPLAY_TRACE_WRITER_H_
#define SLAM_REFACT_TOOLS_HARNESS_REPLAY_TRACE_WRITER_H_

#include <cstdint>
#include <cstdio>
#include <set>
#include <vector>
#include <string>

#include "bundle_reader.h"
#include "slam_refact/glue_runner.h"
#include "slam_refact/surfaces.h"

namespace slam_refact::harness {

// JSON number text of a double as JavaScript's Number::toString / JSON.stringify writes it
// (shortest round-trip digits, exponent form outside [1e-7, 1e21)); non-finite -> "null".
std::string JsNumber(double v);
// +v.toFixed(digits) then JsNumber (page-runtime.js world point samples).
std::string JsFixed(double v, int digits);

const char* StatusName(TrackingStatus s);
const char* ReasonName(TrackingReason r);

class TraceWriter {
 public:
  TraceWriter() = default;
  ~TraceWriter();
  TraceWriter(const TraceWriter&) = delete;
  TraceWriter& operator=(const TraceWriter&) = delete;

  // Opens <out_dir>/trace.jsonl and worldpoints-sampled.jsonl.
  bool Open(const std::string& out_dir, int world_point_sample_every, std::string* error);
  // Frame skipped in drop-mask mode: fresh false, engine fields null (as the original's
  // NO_FRESH_CALLBACK rows).
  void WriteDropped(const FrameRow& frame);
  // Processed frame: fresh true (also for the startup re-publication, srRepeated true).
  void WriteProcessed(const FrameRow& frame, const PublishedFrame& out);
  // ACC-3: surface state and the centre hit test of this frame (written into the processed row of
  // this frame; absent from faithful traces).
  void SetSurfaces(const SurfaceState& state, const std::vector<HitResult>& center_hits);
  bool Close(std::string* error);

  int rows() const { return rows_; }
  int processed() const { return processed_; }

 private:
  std::FILE* trace_ = nullptr;
  std::FILE* samples_ = nullptr;
  int sample_every_ = 50;
  int rows_ = 0;
  int processed_ = 0;
  std::set<uint64_t> last_ids_;
  bool have_surfaces_ = false;
  SurfaceState surfaces_;
  std::vector<HitResult> center_hits_;
};

}  // namespace slam_refact::harness

#endif  // SLAM_REFACT_TOOLS_HARNESS_REPLAY_TRACE_WRITER_H_
