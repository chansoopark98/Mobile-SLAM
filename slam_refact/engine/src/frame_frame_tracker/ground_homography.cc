// robustGroundHomography (1279) — see ground_homography.h. The per-pair solver is the public
// homographyGroundPlane(QA, as, QB, bs) (PUB:c8/geometry/homography.cc:239-355), which the
// vendored file keeps in an anonymous namespace; it is copied here unchanged (1279 inlines it:
// DCMP func1279 lines with the xDegrees(-90) constants -0x1.777a5cp-25 / -1, atan2f, sinf/cosf).

#include "frame_frame_tracker/ground_homography.h"

#include <cmath>
#include <cstddef>
#include <utility>

#include "compat/libcxx_algorithms.h"

namespace slam_refact::frame_frame_tracker {

namespace {

constexpr float FLOAT_EPS = 1e-5;  // PUB:c8/geometry/homography.cc:38

// PUB:c8/geometry/homography.cc:239-355 (homographyGroundPlane, QA/QB overload), unchanged.
void homographyGroundPlane(
  const c8::HMatrix &QA,  // gravity normal rotation
  c8::HPoint2 const (&as)[2],
  const c8::HMatrix &QB,  // gravity normal rotation
  c8::HPoint2 const (&bs)[2],
  c8::HMatrix *homography,
  c8::HMatrix *camMotion) {
  using c8::HMatrix;
  using c8::HMatrixGen;
  using c8::HPoint2;
  using c8::Vector;
  *homography = HMatrixGen::i();
  *camMotion = HMatrixGen::i();
  // Warp points to be on the ground in both images.
  auto as3 = QA * c8::extrude<3>(Vector<HPoint2>{as[0], as[1]});
  auto bs3 = QB * c8::extrude<3>(Vector<HPoint2>{bs[0], bs[1]});
  auto as2 = c8::flatten<2>(as3);
  auto bs2 = c8::flatten<2>(bs3);

  // If points are too close to the horizon, we can't reliably estimate their location on the
  // ground.
  if (
    as3[0].z() <= c8::HORIZON_EXPANSION_THRESHOLD || bs3[0].z() <= c8::HORIZON_EXPANSION_THRESHOLD
    || as3[1].z() <= c8::HORIZON_EXPANSION_THRESHOLD
    || bs3[1].z() <= c8::HORIZON_EXPANSION_THRESHOLD) {
    return;
  }

  // Compute the homography for a plane facing the camera.
  if (
    (as2[0].x() == as2[1].x() && as2[0].y() == as2[1].y())
    || (bs2[0].x() == bs2[1].x() && bs2[0].y() == bs2[1].y())) {
    return;
  }

  // Compute centering transform for a0 and a1.
  HMatrix Ta = HMatrixGen::i();
  {
    float mx = (as2[0].x() + as2[1].x()) * .5;
    float my = (as2[0].y() + as2[1].y()) * .5;

    float dx = as2[0].x() - mx;
    float dy = as2[0].y() - my;
    float scale = 0.70710678118f * std::sqrt(dx * dx + dy * dy);

    if (scale <= FLOAT_EPS) {
      return;
    }

    float is = 1.0f / scale;
    Ta = HMatrix{
      {1.0f, 0.0f, -mx, 0.0f},
      {0.0f, 1.0f, -my, 0.0f},
      {0.0f, 0.0f, scale, 0.0f},
      {0.0f, 0.0f, 0.0f, 1.0f},
      {1.0f, 0.0f, mx * is, 0.0f},
      {0.0f, 1.0f, my * is, 0.0f},
      {0.0f, 0.0f, is, 0.0f},
      {0.0f, 0.0f, 0.0f, 1.0f}};
  }

  // Compute centering transform for b0 and b1.
  HMatrix Tb = HMatrixGen::i();
  {
    float mx = (bs2[0].x() + bs2[1].x()) * .5;
    float my = (bs2[0].y() + bs2[1].y()) * .5;

    float dx = bs2[0].x() - mx;
    float dy = bs2[0].y() - my;
    float scale = 0.70710678118f * std::sqrt(dx * dx + dy * dy);

    if (scale <= FLOAT_EPS) {
      return;
    }

    float is = 1.0f / scale;
    Tb = HMatrix{
      {1.0f, 0.0f, -mx, 0.0f},
      {0.0f, 1.0f, -my, 0.0f},
      {0.0f, 0.0f, scale, 0.0f},
      {0.0f, 0.0f, 0.0f, 1.0f},
      {1.0f, 0.0f, mx * is, 0.0f},
      {0.0f, 1.0f, my * is, 0.0f},
      {0.0f, 0.0f, is, 0.0f},
      {0.0f, 0.0f, 0.0f, 1.0f}};
  }

  // Centered points so that they lie on two lines with the same size, centered at the origin.
  auto aCentered = (Ta * as3[0]).flatten();
  auto bCentered = (Tb * bs3[0]).flatten();

  auto thetaA = std::atan2(aCentered.x(), aCentered.y());
  auto thetaB = std::atan2(bCentered.x(), bCentered.y());
  auto theta = thetaB - thetaA;

  HMatrix rotation = HMatrixGen::zRadians(theta);

  // Scale and translate the roll-only motions by undoing the centering transforms.
  HMatrix hs = Tb.inv() * rotation.inv() * Ta;

  // Compute the h & m, still assuming the camera is looking straight donw.
  float inorm = 1.0f / std::sqrt(hs(0, 0) * hs(0, 0) + hs(0, 1) * hs(0, 1));
  HMatrix h = HMatrix{
    {hs(0, 0) * inorm, hs(0, 1) * inorm, hs(0, 2) * inorm, 0.0f},
    {hs(1, 0) * inorm, hs(1, 1) * inorm, hs(1, 2) * inorm, 0.0f},
    {hs(2, 0) * inorm, hs(2, 1) * inorm, hs(2, 2) * inorm, 0.0f},
    {0.0000000000000f, 0.0000000000000f, 0.0000000000000f, 1.0f},
    true /* noInvert */};
  HMatrix m = rotation * HMatrixGen::translation(-h(0, 2), -h(1, 2), 1 - h(2, 2));

  // Unrotate the homography facing the camera.
  *homography = QB.inv() * h * QA;
  *camMotion = QA.inv() * m * QB;
}

// Squared ray distance between an estimated point and a measured ray (1279 fit / mask loops).
float SquaredResidual(const c8::HPoint2& est, const c8::HPoint2& b) {
  const float dx = est.x() - b.x();
  const float dy = est.y() - b.y();
  return dx * dx + dy * dy;
}

void SetIdentityAndClearMask(c8::HMatrix* h, c8::HMatrix* cam_motion,
                             std::vector<uint8_t>* mask) {
  *h = c8::HMatrixGen::i();
  *cam_motion = c8::HMatrixGen::i();
  if (mask != nullptr) {
    mask->clear();
  }
}

}  // namespace

c8::HMatrix GravityNormalRotation(const c8::Quaternion& q) {
  return c8::HMatrixGen::xDegrees(-90.0f) * q.toRotationMat();
}

bool RobustGroundHomography(const c8::Quaternion& q_prev, const std::vector<c8::HPoint2>& prev_rays,
                            const c8::Quaternion& q_cur, const std::vector<c8::HPoint2>& cur_rays,
                            c8::RobustHomographyScratchSpace* scratch,
                            compat::RandomNumbers* rng, c8::HMatrix* h, c8::HMatrix* cam_motion,
                            std::vector<uint8_t>* mask) {
  const size_t n = prev_rays.size();
  if (n < kMinGroundHomographyPoints) {
    *h = c8::HMatrixGen::i();
    *cam_motion = c8::HMatrixGen::i();
    if (mask != nullptr) {
      mask->resize(n, 0);
    }
    return false;
  }

  // ---- hypothesis pairs (WP2-PRE §1.3) ----
  const size_t total = (n - 1) * n >> 1;
  const size_t num_pairs = total < kMaxGroundHomographyPairs ? total : kMaxGroundHomographyPairs;
  const size_t num_samples = n < kMaxGroundHomographySamples ? n : kMaxGroundHomographySamples;
  scratch->reset(num_pairs, num_samples, n);
  auto& pairs = scratch->hSampleInds;
  if (total <= num_pairs * 5) {
    for (size_t i = 0; i + 1 < n; ++i) {
      for (size_t j = i + 1; j < n; ++j) {
        pairs.emplace_back(i, j);
      }
    }
    rng->shuffle(pairs.begin(), pairs.end());
    pairs.resize(num_pairs);
  } else {
    auto& set = scratch->ptSampleSet;
    const uint32_t n32 = static_cast<uint32_t>(n);
    while (set.size() < num_pairs) {
      const uint32_t r1 = rng->nextUnsignedInt() % n32;
      const uint32_t r2 = rng->nextUnsignedInt() % n32;
      if (r1 == r2) {
        continue;
      }
      set.insert(std::make_pair<size_t, size_t>(r1 < r2 ? r1 : r2, r1 < r2 ? r2 : r1));
    }
    for (const auto& p : set) {
      pairs.push_back(p);
    }
  }

  // ---- point samples ----
  if (n > kMaxGroundHomographySamples) {
    auto& inds = scratch->ptSampleInds;
    for (size_t k = 0; k < n; ++k) {
      inds.push_back(k);
    }
    rng->shuffle(inds.begin(), inds.end());
    for (size_t q = 0; q < num_samples; ++q) {
      scratch->aPtsSample.push_back(prev_rays[inds[q]]);
      scratch->bPtsSample.push_back(cur_rays[inds[q]]);
    }
  } else {
    for (size_t q = 0; q < n; ++q) {
      scratch->aPtsSample.push_back(prev_rays[q]);
      scratch->bPtsSample.push_back(cur_rays[q]);
    }
  }

  // ---- propose-homography ----
  const c8::HMatrix qa = GravityNormalRotation(q_prev);
  const c8::HMatrix qb = GravityNormalRotation(q_cur);
  for (const auto& [i0, i1] : pairs) {
    const c8::HPoint2 as[2] = {prev_rays[i0], prev_rays[i1]};
    const c8::HPoint2 bs[2] = {cur_rays[i0], cur_rays[i1]};
    c8::HMatrix hp = c8::HMatrixGen::i();
    c8::HMatrix mp = c8::HMatrixGen::i();
    homographyGroundPlane(qa, as, qb, bs, &hp, &mp);
    scratch->hSamples.push_back(hp);
    scratch->cameraMotionSamples.push_back(mp);
  }

  // ---- fit-pairs: LMedS-like score = k-th smallest sample residual ----
  const size_t floor_frac =
      static_cast<size_t>(static_cast<float>(num_samples) * kGroundHomographyKFraction);
  size_t k_idx = num_samples - 1;
  if (k_idx > 3) {
    k_idx = 4;
  }
  if (floor_frac > 4) {
    k_idx = floor_frac;
  }
  c8::extrude<3>(scratch->aPtsSample, &scratch->aSampleExtruded);
  float best = 3.4028235e+38f;
  c8::HMatrix best_h = c8::HMatrixGen::i();
  c8::HMatrix best_cam = c8::HMatrixGen::i();
  for (size_t x = 0; x < scratch->hSamples.size(); ++x) {
    const c8::HMatrix& hx = scratch->hSamples[x];
    hx.times(scratch->aSampleExtruded, &scratch->bSampleEstimated3);
    c8::flatten<2>(scratch->bSampleEstimated3, &scratch->bSampleEstimated);
    scratch->sampleResiduals.clear();
    for (size_t s = 0; s < num_samples; ++s) {
      scratch->sampleResiduals.push_back(
          SquaredResidual(scratch->bSampleEstimated[s], scratch->bPtsSample[s]));
    }
    slam_refact::libcxx::nth_element(scratch->sampleResiduals.begin(),
                                     scratch->sampleResiduals.begin() + k_idx,
                                     scratch->sampleResiduals.end());
    const float score = scratch->sampleResiduals[k_idx];
    if (score < best) {
      best = score;
      best_h = hx;
      best_cam = scratch->cameraMotionSamples[x];
    }
  }
  if (best > kGroundHomographyMaxScore) {
    SetIdentityAndClearMask(h, cam_motion, mask);
    return false;
  }
  if (mask != nullptr) {
    c8::extrude<3>(prev_rays, &scratch->aPtsExtruded);
    best_h.times(scratch->aPtsExtruded, &scratch->bPtsEstimated3);
    c8::flatten<2>(scratch->bPtsEstimated3, &scratch->bPtsEstimated);
    mask->resize(n, 0);
    const double threshold = static_cast<double>(best) * kGroundHomographyInlierFactor;
    for (size_t i = 0; i < n; ++i) {
      (*mask)[i] = static_cast<double>(SquaredResidual(scratch->bPtsEstimated[i], cur_rays[i])) <=
                           threshold
                       ? 1
                       : 0;
    }
  }
  *h = best_h;
  *cam_motion = best_cam;
  return true;
}

}  // namespace slam_refact::frame_frame_tracker
