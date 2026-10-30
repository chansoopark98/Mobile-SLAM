// robustGroundHomography (func 1279) — WP2 private header (signature not frozen, CONTRACTS.md
// §2.2). Read from DCMP func1279 / G1279 with the RNG use VERIFIED by WP2-PRE (docs/re/
// 04-wp2-pre.md §1.3, §2): the input rays are HPoint2 (x, y, w) as in the binary (12 B elements).

#ifndef SLAM_REFACT_FRAME_FRAME_TRACKER_GROUND_HOMOGRAPHY_H_
#define SLAM_REFACT_FRAME_FRAME_TRACKER_GROUND_HOMOGRAPHY_H_

#include <cstdint>
#include <vector>

#include "c8/geometry/homography.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "compat/random_numbers.h"

namespace slam_refact::frame_frame_tracker {

// Closed constants of 1279 (VERIFIED G1279:283-287, 1035; f32/f64 constants of func 1279).
inline constexpr size_t kMinGroundHomographyPoints = 15;     // public MIN_POINTS_FOR_ROBUST_...
inline constexpr size_t kMaxGroundHomographyPairs = 73;
inline constexpr size_t kMaxGroundHomographySamples = 100;   // public MAX_POINT_SAMPLES_...
inline constexpr float kGroundHomographyKFraction = 0.3f;
inline constexpr float kGroundHomographyMaxScore = 2e-4f;    // public HOMOGRAPHY_FAILED_...
inline constexpr double kGroundHomographyInlierFactor = 1.1;

// 1279. Ground-plane homography between two frames from gravity rotations and matched rays.
// prev_rays[i] and cur_rays[i] are one match. Returns true when the k-th smallest sample
// residual (k = floor(0.3 N) if > 4, else min(N - 1, 4)) of the best pair hypothesis is
// <= 2e-4; then *h is the ray-space homography (noInvert set), *cam_motion =
// QA^-1 * m * QB (translation in camera-height units) and mask[i] = residual_i <= 1.1 * best.
// Rejection: h = cam_motion = identity; n < 15 resizes the mask to n (new entries 0, old kept),
// n >= 15 clears the mask (G1279 tail: mask.end = mask.begin).
// RNG (WP2-PRE §1.3): n(n-1)/2 <= 5*73 -> all (i<j) pairs (i outer) shuffled (libc++ 805) and the
// first min(total, 73) kept; otherwise 2 draws of nextUnsignedInt() % n per attempt into a
// std::set until it holds 73 distinct (min, max) pairs, used in set order. n > 100: point indices
// shuffled and the first 100 sampled.
bool RobustGroundHomography(const c8::Quaternion& q_prev, const std::vector<c8::HPoint2>& prev_rays,
                            const c8::Quaternion& q_cur, const std::vector<c8::HPoint2>& cur_rays,
                            c8::RobustHomographyScratchSpace* scratch,
                            compat::RandomNumbers* rng, c8::HMatrix* h, c8::HMatrix* cam_motion,
                            std::vector<uint8_t>* mask);

// Gravity-normal rotation of 1267/1279: xDegrees(-90) * R(q).
c8::HMatrix GravityNormalRotation(const c8::Quaternion& q);

}  // namespace slam_refact::frame_frame_tracker

#endif  // SLAM_REFACT_FRAME_FRAME_TRACKER_GROUND_HOMOGRAPHY_H_
