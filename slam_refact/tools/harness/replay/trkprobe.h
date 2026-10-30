// US-026 root-cause A-3 diagnostic (REPORT tier, harness only): per-frame tracker-internal values of the engine
// in the names of the original memprobe / poke fields, plus a chronological stage log (MapTracker::Track,
// FrameFrameTracker::Track, RobustGroundHomography, TrackPoints, Relocalize, MapBuilder::Update) with RNG draw
// counts. Everything is read through linker --wrap captures of module entry points (see CMakeLists.txt) and the
// modules' own *Access / internal headers; no engine/src file is changed and, by default, nothing is computed
// differently (the trace is bit-identical to slam_refact_replay).
//
// Opt-in experiment (OFF unless TRKPROBE_STALE_DEPTHS=1, then the run is NOT an engine-equivalent replay).
// Historical since US026-CC-1: it was written before the engine modeled the original's depth-vector storage. The
// engine now does that itself (PrevDepthList, frame_frame_tracker_internal.h): an index in [size(), capacity()) reads
// the float a longer earlier list left in the storage and an index past capacity() is "no depth", so a default
// replay already contains the effect. With the variable set, the probe additionally rewrites prev_depths before
// FrameFrameTracker::Track: a list shorter than the previous frame is extended to the previous frame's size with
// the stale floats seen in [size(), capacity()), and "no depth" past capacity(). Because the extension is assigned
// through PrevDepthList::Assign, a list longer than capacity() reallocates the modeled storage (new capacity
// max(2 * capacity, n)), so later frames see a different capacity than a default replay. Treat results of this
// mode as an experiment on top of CC-1, not as the CC-1 behavior. The per-frame "stale" counters in trkprobe.jsonl
// (oob*) are read-only and stay valid in the default mode.
#ifndef SLAM_REFACT_TOOLS_HARNESS_REPLAY_TRKPROBE_H_
#define SLAM_REFACT_TOOLS_HARNESS_REPLAY_TRKPROBE_H_

#include <cstdio>

namespace slam_refact::harness::trkprobe {

// Drops the stage log collected so far (call after the discarded bootstrap stage).
void ResetFrame();

// Writes one JSON line for the frame just processed and clears the stage log.
void EmitFrame(std::FILE* f, int frame_index);

}  // namespace slam_refact::harness::trkprobe

#endif  // SLAM_REFACT_TOOLS_HARNESS_REPLAY_TRKPROBE_H_
