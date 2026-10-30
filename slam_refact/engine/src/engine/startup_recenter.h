// Engine private header: the startup recenter window of the glue (spec 06 §4.3, RM-3a).

#ifndef SLAM_REFACT_ENGINE_STARTUP_RECENTER_H_
#define SLAM_REFACT_ENGINE_STARTUP_RECENTER_H_

namespace slam_refact {

// Initial value of tracking-controller.cc's framesUntilRecenter (PUB:reality/app/xr/js/
// tracking-controller.cc:85). After this many processed frames GlueRunner rebuilds the core
// (rebuildEngine: pipeline.clear()), so the next callback finds an empty pipeline and publishes the
// previous response again (S4 §3.8). The evidence tests derive the one callback where the
// "previous fresh callback's frame" lighting association differs from the original from this
// constant: kInitialFramesUntilRecenter + 1 (the first re-published callback).
inline constexpr int kInitialFramesUntilRecenter = 5;

}  // namespace slam_refact

#endif  // SLAM_REFACT_ENGINE_STARTUP_RECENTER_H_
