// slam_refact public API: engine and glue configuration (architecture.md §4.1).
//
// FROZEN CONTRACT (WP0b). Changes follow slam_refact/docs/design/contract-changes.md.

#ifndef SLAM_REFACT_ENGINE_CONFIG_H_
#define SLAM_REFACT_ENGINE_CONFIG_H_

#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "slam_refact/types.h"

namespace slam_refact {

// Accuracy mode (kContractVersion 3, contract-changes.md ACC-1). Everything here is OFF by default:
// with enabled == false the engine is the bit-exact reimplementation of the original. The
// options are additions the original binary does not have; they are evaluated against ARCore- and
// TUM-referenced fixtures (slam_refact/docs/reports/accuracy.md) before a page may enable them.
struct AccuracyConfig {
  bool enabled = false;
  // Triangulated map points are placed by a Gauss-Newton multi-view triangulation on the current
  // keyframe poses (instead of the mean of per-view epipolar depths).
  bool multiview_triangulation = false;  // experimental (ACC-1), evaluated and not adopted
  // Joint bundle adjustment of the newest keyframes and their triangulated points after every
  // new keyframe; keyframes outside the window that see those points are held fixed.
  bool local_ba = false;  // experimental (ACC-1), evaluated and not adopted
  int ba_window = 8;
  int ba_max_iterations = 8;
  double ba_max_seconds = 0.05;
  // Robust loss of the reprojection residual: scale in pixels; ba_cauchy selects Cauchy instead of
  // Huber (reference runtime matches within 0.05 ray, so gross outliers are common).
  double ba_huber_px = 2.0;
  bool ba_cauchy = false;
  // After the solve, observations with a reprojection error above this many pixels are unlinked
  // from their point (0 = keep all).
  double ba_outlier_px = 0.0;
  // Standard deviation of the keyframe gravity (tilt) prior, radians. The prior keeps each
  // keyframe's gravity direction at the value the tracker gave it (device tilt).
  double ba_tilt_sigma_rad = 0.02;
  // Ground points in the bundle adjustment: 0 = not adjusted (original placement), 1 = (x, z) on
  // the ground plane y = -1, 2 = free 3D points like triangulated ones (needs the parallax gate).
  int ba_ground_points = 0;
  // Minimum ray angle (degrees) between two observations for a point to be adjusted.
  double ba_min_parallax_deg = 1.0;
  // Continuous metric scale of the published trajectory (ACC-2, engine/metric_scale_tracker.h):
  // positions and world points are published in metres once PublishedFrame::metric_initialized
  // (before that with metric_prior_scale); the tracking status is not changed. Independent of
  // EngineConfig::estimate_scale (the original latch).
  bool metric_output = false;
  // Tracking-stage visual-inertial coupling (ACC-5): the MapTracker's motion prediction (and so
  // the pose prior of its PnP) integrates the accelerometer in metres with a velocity fused from
  // IMU propagation and the tracked visual velocity, instead of the original constant-velocity
  // model. Metres per internal unit come from the metric scale tracker (prior until it converges).
  bool vi_prediction = false;
  // Weight of the visual velocity in the fused velocity per frame (0..1).
  double vi_velocity_gain = 0.2;
  double metric_window_s = 4.0;
  double metric_prior_scale = 0.7;  // metres per responsive unit (camera height 1.4 m at origin.y 2)
  // Experimental (ACC-2 option, default off): estimate the metric scale in the 0.1-0.8 Hz band
  // instead of 0.4-2.5 Hz, for EIS-stabilized camera streams (MetricScaleParams::low_band); use
  // with metric_window_s >= 6.
  bool metric_low_band = false;
  // Fixed IMU lag (s, |lag| <= 0.3) instead of the lag search; NaN = search (default).
  double metric_fixed_lag_s = std::numeric_limits<double>::quiet_NaN();
};

struct EngineConfig {
  bool estimate_scale = false;  // JS scale: 'absolute'
  bool disable_vio = false;
  // engineInit(captureWidth, captureHeight, rotation) (FULL-8, US-023; R16 is lifted): the camera
  // image size as the video delivers it (landscape when width > height) and the screen rotation
  // 0 / 90 / 180 / 270 (or -90 / -180 / -270) the pyramid renderer turns the image by; the published
  // camera pose and projection are corrected by the matching device orientation (90 = LANDSCAPE_LEFT,
  // 270 / -90 = LANDSCAPE_RIGHT). The tracker itself runs in the device's portrait frame.
  int capture_width = 0;
  int capture_height = 0;
  int rotation_deg = 0;
  // Inputs of the K rule (S1 §5.12) and of is_ios.
  std::string manufacturer;
  std::string model;
  // Diagnostic override of the K rule: the processing (level-0) intrinsics, used as given (equal to
  // the capture K when the capture is 480x640).
  std::optional<PinholeIntrinsics> intrinsics_override;
  ShaderNumericModel pyramid_model = ShaderNumericModel::kIeee;
  // Applied to the process-global globalParams() (§4.6).
  std::vector<std::pair<std::string, ParamValue>> param_overrides;
  // Process-global like param_overrides: set at construction, kept across engine rebuilds.
  AccuracyConfig accuracy;
};

// Post-processing of published poses (S4 §3.11).
// When EngineConfig::estimate_scale is true the glue forces origin.y = 1 and scale = 1 and ignores
// the user origin.y (JS@5517107, N4).
struct GlueConfig {
  Vec3f origin{0.f, 2.f, 0.f};
  QuatWxyz facing{1.f, 0.f, 0.f, 0.f};
  bool left_handed_axes = false;
  bool mirrored_display = false;
  // ACC-3: floor / horizontal surface detection and hit test on the published world points
  // (GlueRunner::surfaces, GlueRunner::HitTest). Does not change any published pose.
  bool detect_surfaces = false;
  // FULL-7 (RM-4b): framesUntilRecenter of the tracking-controller at construction. The default 5
  // is the original's initial value (tracking-controller.cc:85: after 5 processed frames the
  // engine is rebuilt and the first 5 responses are published at the configured origin). The
  // original keeps this counter in its EngineData singleton, so an engineInit re-run (video size or
  // orientation change) does NOT repeat the startup recenter; only engineCleanup (session detach)
  // makes a fresh counter. A caller that replaces the GlueRunner for an engineInit re-run passes
  // the old runner's GlueRunner::frames_until_recenter() (normally 0). Must be >= 0.
  int frames_until_recenter = 5;
};

}  // namespace slam_refact

#endif  // SLAM_REFACT_ENGINE_CONFIG_H_
