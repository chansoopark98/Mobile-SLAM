// func994 (closed image-world-tracker), reconstructed from xr-slam.wasm. Line references: G = the
// Ghidra C of func994 (ram:800ce0de_unnamed_function_994.c), D = xr-slam.dcmp. Every step calls the
// public reference runtime function the binary inlines (spec/11 section 9.2).

#include "image_target/image_world_tracker.h"

#include <cstddef>

#include "c8/geometry/egomotion.h"
#include "c8/geometry/parameterized-geometry.h"
#include "common/stage_timer.h"
#include "reality/engine/geometry/bundle.h"

namespace slam_refact::image_target {
namespace {

constexpr int32_t kSourceTriangulated = 1;  // WorldPoint +0x40 (G:1381 `== 1`)
constexpr size_t kMinPairs = 6;             // G:1640 `size in bytes < 0x60` (16 B points)

}  // namespace

c8::HMatrix TrackImageWorld(c8::ImageWorldTrackerState* st, const TrackedPointLookup& lookup,
                            const MapToTrackerTransform& xf, const c8::DetectionImage& im,
                            const c8::HMatrix& target_view_pose, const c8::HMatrix& tam,
                            const c8::FrameWithPoints& feats, float scale_guess) {
  StageTimer timer("image-world-tracker");  // G:252-277

  // G:279-301 / G:1077-1092: the frame's feature rays (FramePoint +0x18) with w = 1.
  st->rays.clear();
  st->rays.reserve(feats.points().size());
  for (const c8::FramePoint& p : feats.points()) {
    st->rays.push_back(c8::HPoint2(p.position().x(), p.position().y()));
  }

  // G:302-1068 (type 2 = CURVY: cameraRaysToVisiblePointsInCameraCurvy inlined, 1044 =
  // cameraRaysToTargetWorld, 1045 = mapFromGeometry) and G:1069-1296 (planar: 1048 =
  // imageTargetCornerPixels, 739 = Poly2, 1049 = containsPointAssumingConvex, then the plane
  // intersection). Targets: st+0x94 points, st+0xa0 indices.
  if (im.getType() == c8::DetectionImageType::CURVY) {
    c8::cameraRaysToVisiblePointsInCameraCurvy(im.getGeometry(), st->rays, target_view_pose,
                                               &st->searchPtsInCamera, &st->visible);
  } else {
    // targetK = DetectionImage +0xf0 = framePoints_.intrinsic_; camK = FrameWithPoints +0x9c.
    c8::cameraRaysToVisiblePointsInCameraPlanar(im.framePoints().intrinsic(), feats.intrinsic(),
                                                st->rays, feats.pixels(), target_view_pose,
                                                &st->searchPtsInCamera, &st->visible);
  }

  // G:1297-1639 (scope "match"): visible features joined with the tracked map points. The binary
  // stages the pairs in a capnp MallocMessageBuilder list (G:1302-1413) and copies the non-null
  // entries in visible order (G:1479-1637); a vector in the same order is equivalent.
  st->worldPts.clear();
  st->ids.clear();
  st->targetPts.clear();
  {
    StageTimer match_timer("match");
    for (size_t idx : st->visible) {
      TrackedMapPoint mp;
      if (!lookup.Find(static_cast<uint32_t>(idx), &mp) || mp.source != kSourceTriangulated) {
        continue;
      }
      // G:1395-1403: (x/w - offset) * scale per axis.
      st->worldPts.push_back(c8::HPoint3((mp.position.x() - xf.offset[0]) * xf.scale,
                                         (mp.position.y() - xf.offset[1]) * xf.scale,
                                         (mp.position.z() - xf.offset[2]) * xf.scale));
      st->ids.push_back(c8::WorldMapPointId(mp.slot, mp.id));
      st->targetPts.push_back(st->searchPtsInCamera[idx]);
    }
  }

  if (st->worldPts.size() < kMinPairs) {  // G:1640-1835
    st->scale = scale_guess;
    st->worldPose = tam * c8::scaleTranslation(scale_guess, target_view_pose.inv());
    return st->worldPose;
  }

  // G:1888: the world points in the camera frame (564 = HMatrix * Vector<HPoint3>, with tam^-1).
  const c8::Vector<c8::HPoint3> model = tam.inv() * st->worldPts;
  StageTimer solver_timer("scale-solver");  // G:1889-1899, closed at G:2316
  // G:1900-2105: poseEstimationScale3d(observed = target points, model = world points in camera)
  // inlined (Scale3dPoseEstimation vtable 0x48200, AutoDiffCostFunction<., 3, 1> 0x484a8,
  // CauchyLoss 0x48920 with b = 1e-8, c = 1e8; Solver::Options as bundle.cc:938-946).
  if (c8::poseEstimationScale3d(st->targetPts, model, &st->scale)) {
    st->worldPose = tam * c8::scaleTranslation(st->scale, target_view_pose.inv());  // G:2106-2311
  }
  return st->worldPose;  // G:2321 memcpy(out, st + 4, 0x84)
}

c8::HMatrix ImageWorldTrackerHook::track(c8::ImageWorldTrackerState* st,
                                         const c8::DetectionImage& im,
                                         const c8::HMatrix& targetViewPose,
                                         const c8::HMatrix& tamExtrinsic,
                                         const c8::FrameWithPoints& feats, float scaleGuess) {
  struct NoPoints final : TrackedPointLookup {
    bool Find(uint32_t, TrackedMapPoint*) const override { return false; }
  };
  static const NoPoints kNoPoints;
  return TrackImageWorld(st, lookup_ != nullptr ? *lookup_ : kNoPoints, xf_, im, targetViewPose,
                         tamExtrinsic, feats, scaleGuess);
}

}  // namespace slam_refact::image_target
