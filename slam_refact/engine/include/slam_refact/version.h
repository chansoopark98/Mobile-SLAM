// slam_refact public API: version and provenance constants.
//
// FROZEN CONTRACT (WP0b). Changes follow slam_refact/docs/design/contract-changes.md.

#ifndef SLAM_REFACT_VERSION_H_
#define SLAM_REFACT_VERSION_H_

#define SLAM_REFACT_VERSION_MAJOR 0
#define SLAM_REFACT_VERSION_MINOR 1
#define SLAM_REFACT_VERSION_PATCH 0

namespace slam_refact {

// Bumped by every approved contract change (contract-changes.md). 1 = initial freeze (WP0b).
inline constexpr int kContractVersion = 10;

// Target of the reimplementation: xr-slam.wasm (architecture.md header).
inline constexpr char kTargetWasmSha256[] =
    "780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422";

// Public source reused by the engine (architecture.md §2.2).
inline constexpr char kUpstreamxr_referenceCommit[] = "4edfa484";

}  // namespace slam_refact

#endif  // SLAM_REFACT_VERSION_H_
