// slam_refact replay diagnostics: hooks for exact-input replays of a recorded original run.
// Not used by production code paths; every default is the production behavior.

#ifndef SLAM_REFACT_REPLAY_DIAGNOSTICS_H_
#define SLAM_REFACT_REPLAY_DIAGNOSTICS_H_

#include <cstdint>
#include <optional>

namespace slam_refact::diagnostics {

// Seed of the map / keyframe / point id generator (mt19937) of every WorldMaps constructed after
// this call (process-global). The original seeds that generator with one getentropy() word per
// run (G641), so its ids, and every iteration ordered by id (std::set / std::map keyed by
// WorldMapPointId or WorldMapKeyframeId, the world-point publication stride over ids), differ
// from run to run. An exact-input replay passes the recorded run's seed to reproduce them.
// std::nullopt (the default) keeps the engine's fixed seed kIdGeneratorSeed.
void SetReplayIdGeneratorSeed(std::optional<uint32_t> seed);

// OR-4 (gate or4-it-world-regression.v5 section 2): null-run / mutation hooks of the image target world
// regression gate. Header-only, process-global, single-threaded; the defaults are the production behavior.
namespace detail {
inline std::optional<uint32_t> engine_rng_seed;
inline bool force_all_orb_gorb = false;
}  // namespace detail

// Seed of the XREngine RandomNumbers (rng_ of Engine, the stream MapTracker 926 and the image target
// tracker 938 share) of every Engine constructed after this call. std::nullopt keeps compat::kEngineRngSeed
// (5489). A different seed is the benign "other RNG stream" null of the gate: the shared stream shifts the
// same way loading a target shifts it, without any target.
inline void SetReplayEngineRngSeed(std::optional<uint32_t> seed) { detail::engine_rng_seed = seed; }
inline std::optional<uint32_t> ReplayEngineRngSeed() { return detail::engine_rng_seed; }

// Forces the tracking DetectionConfig allOrb = allGorb = true as if an image target were loaded, without any
// target (decomposition experiment: descriptor configuration only, no 938, no RNG sharing).
inline void SetReplayForceAllOrbGorb(bool force) { detail::force_all_orb_gorb = force; }
inline bool ReplayForceAllOrbGorb() { return detail::force_all_orb_gorb; }

// US-015 (contract-changes.md FULL-12, default flipped by WT-DEFAULT-ON): the closed image world tracker of the
// original (xr-slam.wasm func938/func994, docs/spec/11-image-world-tracker.md section 9) on every VioTracker
// image target call while VIO is on. On by default, as in the original, which always runs it (commit 281db55).
// SetImageWorldTracker(false) restores the public ImageTrackingState behavior that the engine had before,
// bit for bit; it exists for A/B runs and gate arm 0. There is no switch for it in EngineConfig, the embind
// bindings or the web chunk (the original has none either), so the WASM build always runs it on.
// Set it before the session starts (before the first TrackWithTargets): toggling it mid-session mixes the
// public and world-tracker states of ImageTrackingState (worldInit_, scaleGuess_).
namespace detail {
inline bool image_world_tracker = true;
}  // namespace detail
inline void SetImageWorldTracker(bool on) { detail::image_world_tracker = on; }
inline bool ImageWorldTrackerEnabled() { return detail::image_world_tracker; }

}  // namespace slam_refact::diagnostics

#endif  // SLAM_REFACT_REPLAY_DIAGNOSTICS_H_
