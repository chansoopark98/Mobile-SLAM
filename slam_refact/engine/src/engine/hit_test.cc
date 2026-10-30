// XrController hitTest (spec 06 §2). Line references are to the public reference runtime source.

#include "engine/hit_test.h"

#include <cmath>
#include <cstdint>
#include <vector>

#include "c8/geometry/egomotion.h"
#include "c8/geometry/homography.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "c8/vector.h"

namespace slam_refact {
namespace {

// calculateDistance (hit-test-performer.cc:43-49), float.
float CalculateDistance(const Vec3f& camera_position, const c8::HPoint3& point) {
  const float xd = point.x() - camera_position.x;
  const float yd = point.y() - camera_position.y;
  const float zd = point.z() - camera_position.z;
  return std::sqrt((xd * xd) + (yd * yd) + (zd * zd));
}

// getCameraIntrinsics (hit-test-performer.cc:51-69): diag(-1, 1, 1, 1) x the column-major
// projection matrix of the response camera.
c8::HMatrix CameraIntrinsics(const std::array<float, 16>& i) {
  const c8::HMatrix intrinsics_mat{{i[0 + 0 * 4], i[0 + 1 * 4], i[0 + 2 * 4], i[0 + 3 * 4]},
                                   {i[1 + 0 * 4], i[1 + 1 * 4], i[1 + 2 * 4], i[1 + 3 * 4]},
                                   {i[2 + 0 * 4], i[2 + 1 * 4], i[2 + 2 * 4], i[2 + 3 * 4]},
                                   {i[3 + 0 * 4], i[3 + 1 * 4], i[3 + 2 * 4], i[3 + 3 * 4]}};
  const c8::HMatrix model{{-1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f},
                          {0.0f, 0.0f, 1.0f, 0.0f},  {0.0f, 0.0f, 0.0f, 1.0f},
                          {-1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f},
                          {0.0f, 0.0f, 1.0f, 0.0f},  {0.0f, 0.0f, 0.0f, 1.0f}};
  return model * intrinsics_mat;
}

// getCameraExtrinsics (hit-test-performer.cc:71-82).
c8::HMatrix CameraExtrinsics(const HitTestState& state) {
  const c8::Quaternion rot(state.camera_rotation.w, state.camera_rotation.x,
                           state.camera_rotation.y, state.camera_rotation.z);
  const c8::HPoint3 pt(state.camera_position.x, state.camera_position.y, state.camera_position.z);
  return c8::cameraMotion(pt, rot);
}

// estimateZDepth (hit-test-performer.cc:84-105): Gaussian weights (sigma 0.02) of the distance on
// the normalized image plane, weighted mean of the camera z; points behind the camera are kept.
float EstimateZDepth(const c8::Vector<c8::HPoint3>& points, c8::HPoint2 pt,
                     const c8::HMatrix& extrinsic) {
  auto projected_pts_3d = extrinsic.inv() * points;
  auto pts_2d = c8::flatten<2>(projected_pts_3d);

  double wt = 0.0f;
  double z_est = 0.0f;
  for (size_t i = 0; i < pts_2d.size(); ++i) {
    double xd = pt.x() - pts_2d[i].x();
    double yd = pt.y() - pts_2d[i].y();
    double distance = xd * xd + yd * yd;
    double invdist = std::exp(-0.5 * (1 / (.02 * .02)) * distance);

    z_est += projected_pts_3d[i].z() * invdist;
    wt += invdist;
  }

  if (wt > 0.0f) {
    z_est /= wt;
  }

  return static_cast<float>(z_est);
}

// recenterAndScale(Position32f) (orientation.cc:581-584) with the origin of the configured
// coordinate system (orientation.cc:367-369).
Vec3f RecenterAndScale(const CoordinateSystem& coords, float s, float px, float py, float pz) {
  const c8::Quaternion oq = c8::groundPlaneRotation(
      c8::Quaternion(coords.facing.w, coords.facing.x, coords.facing.y, coords.facing.z));
  const c8::HMatrix o =
      c8::cameraMotion(c8::HVector3(coords.origin.x, coords.origin.y, coords.origin.z), oq);
  const c8::HPoint3 np = o * c8::HPoint3(s * px, s * py, s * pz);
  return Vec3f{np.x(), np.y(), np.z()};
}

}  // namespace

uint32_t HitTypeMask(const std::vector<XrHitType>& included_types) {
  uint32_t mask = 0;
  for (const XrHitType t : included_types) {
    mask |= (0x1u << static_cast<uint32_t>(t));
  }
  return mask;
}

bool ContainsResultType(uint32_t mask, XrHitType type) {
  return (mask & (0x1u << static_cast<uint32_t>(type))) != 0 || mask == 0;
}

std::vector<InternalHit> FeaturePointHitTest(const HitTestState& state, float x, float y,
                                             uint32_t mask, bool mirrored_display) {
  // XREngine::query (xr-engine.cc:111-135): mirror x, then turn the input by the device orientation
  // of the response (FULL-8; PORTRAIT needs no rotation).
  if (mirrored_display) {
    x = 1 - x;
  }
  DeviceToPortraitHitInput(state.orientation, x, y, &x, &y);
  if (!ContainsResultType(mask, XrHitType::kFeaturePoint) || state.points.empty()) {
    return {};
  }
  const c8::HMatrix extrinsic = CameraExtrinsics(state);
  const c8::HMatrix intrinsic = CameraIntrinsics(state.projection);

  // getHitFeaturePoints (hit-test-performer.cc:107-135).
  c8::Vector<c8::HPoint3> feature_points_3d;
  feature_points_3d.reserve(state.points.size());
  for (const Vec3f& p : state.points) {
    feature_points_3d.push_back(c8::HPoint3(p.x, p.y, p.z));
  }
  c8::HPoint2 hit_point_in_graphics((x - 0.5f) * 2.0f, (y - 0.5f) * 2.0f);
  auto graphics_pt_3d = hit_point_in_graphics.extrude();
  auto hit_point = intrinsic.inv() * graphics_pt_3d;
  auto hit_point_2d = hit_point.flatten();

  auto z_est = EstimateZDepth(feature_points_3d, hit_point_2d, extrinsic);
  c8::HPoint3 est_pt = c8::HPoint3(z_est * hit_point_2d.x(), z_est * hit_point_2d.y(), z_est);
  c8::HPoint3 world_pt = extrinsic * est_pt;

  // setHitResult (hit-test-performer.cc:137-147): position and distance only.
  InternalHit hit;
  hit.type = XrHitType::kFeaturePoint;
  hit.position = Vec3f{world_pt.x(), world_pt.y(), world_pt.z()};
  hit.distance = CalculateDistance(state.camera_position, world_pt);
  return {hit};
}

XrHitResult RewriteHit(const CoordinateSystem& coords, const InternalHit& hit) {
  XrHitResult out;
  out.type = hit.type;
  float px = hit.position.x, py = hit.position.y, pz = hit.position.z;
  // place.rotation is never written by setHitResult: the capnp default (0, 0, 0, 0).
  float qw = 0.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
  if (!coords.left_handed_axes) {
    // X_LEFT_Y_UP_Z_FORWARD -> makeRightHanded (orientation.cc:174-179, 584-589).
    pz = -pz;
    qw = -qw;
    qz = -qz;
  }
  if (coords.mirrored_display) {
    // mirror(XrQueryResponse) moves the position only (orientation.cc:376-380).
    px = -px;
  }
  // recenterAndScale(XrQueryResponse) (orientation.cc:364-374): position and distance only.
  const float s = coords.scale > 0.0f ? coords.scale : 1.0f;
  out.position = RecenterAndScale(coords, s, px, py, pz);
  out.rotation = QuatWxyz{qw, qx, qy, qz};
  out.distance = hit.distance * s;
  return out;
}

Vec3f RewritePosition(const CoordinateSystem& coords, const Vec3f& internal) {
  float px = internal.x, py = internal.y, pz = internal.z;
  if (!coords.left_handed_axes) {
    pz = -pz;
  }
  if (coords.mirrored_display) {
    px = -px;
  }
  const float s = coords.scale > 0.0f ? coords.scale : 1.0f;
  return RecenterAndScale(coords, s, px, py, pz);
}

void DisplayRayInternal(const HitTestState& state, float x, float y, bool mirrored_display,
                        Vec3f* camera_centre, Vec3f* depth_one_point) {
  if (mirrored_display) {
    x = 1 - x;
  }
  DeviceToPortraitHitInput(state.orientation, x, y, &x, &y);
  const c8::HMatrix extrinsic = CameraExtrinsics(state);
  const c8::HMatrix intrinsic = CameraIntrinsics(state.projection);
  c8::HPoint2 hit_point_in_graphics((x - 0.5f) * 2.0f, (y - 0.5f) * 2.0f);
  auto hit_point_2d = (intrinsic.inv() * hit_point_in_graphics.extrude()).flatten();
  const c8::HPoint3 p = extrinsic * c8::HPoint3(hit_point_2d.x(), hit_point_2d.y(), 1.0f);
  *camera_centre = state.camera_position;
  *depth_one_point = Vec3f{p.x(), p.y(), p.z()};
}

}  // namespace slam_refact
