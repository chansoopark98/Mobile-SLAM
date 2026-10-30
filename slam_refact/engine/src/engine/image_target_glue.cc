// Image target glue (contract FULL-3, US-013). See image_target_glue.h for the sources.

#include "engine/image_target_glue.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <string>

#include "c8/geometry/egomotion.h"
#include "c8/geometry/homography.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"

namespace slam_refact {
namespace {

// Forward matrix only (the 733 / egomotion product reads the forward part of the located pose).
c8::HMatrix FromHMatrix4f(const HMatrix4f& m) { return c8::HMatrix(m.m, nullptr); }

// One JS number as JSON.stringify writes it: non-finite -> null, -0 -> 0.
bool SameJsonNumber(float a, float b) {
  const bool fa = std::isfinite(a), fb = std::isfinite(b);
  if (!fa || !fb) return fa == fb;
  return a == b;
}

// scale: Math.max(widthInMeters, heightInMeters) || 1.0 (tracking-controller.ts:720).
float JsScale(const DetectedImageOut& d) {
  const float w = d.width_in_meters, h = d.height_in_meters;
  if (std::isnan(w) || std::isnan(h)) return 1.0f;  // Math.max with NaN is NaN, NaN || 1 = 1
  const float m = std::max(w, h);
  return m == 0.0f ? 1.0f : m;
}

}  // namespace

std::vector<DetectedImageOut> DetectedImagesForResponse(const std::vector<LocatedImageOut>& located,
                                                        const c8::HMatrix& init_pose,
                                                        const CoordinateSystem& coords) {
  // responseTargets: TreeMap<String, DetectedImage> keyed by located.roi.name (= target name).
  std::map<std::string, DetectedImageOut> response;
  const float s = coords.scale > 0.0f ? coords.scale : 1.0f;
  const c8::Quaternion oq = c8::groundPlaneRotation(
      c8::Quaternion(coords.facing.w, coords.facing.x, coords.facing.y, coords.facing.z));
  const c8::HMatrix o =
      c8::cameraMotion(c8::HVector3(coords.origin.x, coords.origin.y, coords.origin.z), oq);
  for (const LocatedImageOut& l : located) {
    if (l.last_seen >= 3) continue;   // disappear quickly after tracking loss
    if (l.first_seen <= 3) continue;  // wait for three contiguous frames
    DetectedImageOut d;
    d.name = l.name;
    d.id = l.id;
    d.width_in_meters = l.width;
    d.height_in_meters = l.height;
    d.type = l.type;
    if (l.type == DetectedImageType::kCurvy) d.curvy_geometry = l.curvy_geometry;
    const c8::HMatrix pose = c8::egomotion(init_pose, FromHMatrix4f(l.pose));
    const c8::HVector3 t = c8::translation(pose);
    const c8::Quaternion q = c8::rotation(pose);
    // setPosition / setQuaternion into Position32f / Quaternion32f, then 704.
    float px = t.x(), py = t.y(), pz = t.z();
    float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
    if (!coords.left_handed_axes) {  // makeRightHanded (orientation.cc:146-150)
      pz = -pz;
      qw = -qw;
      qz = -qz;
    }
    if (coords.mirrored_display) {  // mirror (orientation.cc:330-334)
      px = -px;
      qy = -1.0f * qy;
      qz = -1.0f * qz;
    }
    // recenterAndScale (orientation.cc:309-315).
    const c8::HPoint3 np = o * c8::HPoint3(s * px, s * py, s * pz);
    const c8::Quaternion nq = oq.times(c8::Quaternion(qw, qx, qy, qz));
    d.position = Vec3f{np.x(), np.y(), np.z()};
    d.rotation = QuatWxyz{nq.w(), nq.x(), nq.y(), nq.z()};
    d.width_in_meters = d.width_in_meters * s;
    d.height_in_meters = d.height_in_meters * s;
    response[d.name] = d;
  }
  std::vector<DetectedImageOut> out;
  out.reserve(response.size());
  for (auto& e : response) out.push_back(std::move(e.second));
  return out;
}

bool SameForImageEvent(const DetectedImageOut& a, const DetectedImageOut& b) {
  const CurvyGeometryOut& ca = a.curvy_geometry;
  const CurvyGeometryOut& cb = b.curvy_geometry;
  const bool curvy_same =
      a.type != DetectedImageType::kCurvy ||
      (SameJsonNumber(ca.curvy_circumference_top, cb.curvy_circumference_top) &&
       SameJsonNumber(ca.curvy_circumference_bottom, cb.curvy_circumference_bottom) &&
       SameJsonNumber(ca.curvy_side_length, cb.curvy_side_length) &&
       SameJsonNumber(ca.target_circumference_top, cb.target_circumference_top) &&
       SameJsonNumber(ca.height, cb.height) && SameJsonNumber(ca.top_radius, cb.top_radius) &&
       SameJsonNumber(ca.bottom_radius, cb.bottom_radius) &&
       SameJsonNumber(ca.arc_length_radians, cb.arc_length_radians) &&
       SameJsonNumber(ca.arc_start_radians, cb.arc_start_radians));
  return a.name == b.name && a.type == b.type && curvy_same &&
         SameJsonNumber(a.position.x, b.position.x) && SameJsonNumber(a.position.y, b.position.y) &&
         SameJsonNumber(a.position.z, b.position.z) && SameJsonNumber(a.rotation.x, b.rotation.x) &&
         SameJsonNumber(a.rotation.y, b.rotation.y) && SameJsonNumber(a.rotation.z, b.rotation.z) &&
         SameJsonNumber(a.rotation.w, b.rotation.w) && SameJsonNumber(JsScale(a), JsScale(b));
}

std::vector<ImageTargetEvent> ImageEventTracker::Update(
    const std::vector<DetectedImageOut>& detected) {
  std::vector<ImageTargetEvent> events;
  std::vector<std::string> seen;
  for (const DetectedImageOut& img : detected) {
    seen.push_back(img.name);
    auto found = std::find_if(map_.begin(), map_.end(),
                              [&img](const DetectedImageOut& m) { return m.name == img.name; });
    if (found == map_.end()) {
      map_.push_back(img);
      events.push_back({ImageTargetEventKind::kFound, img});
      continue;
    }
    if (!SameForImageEvent(*found, img)) {
      *found = img;
      events.push_back({ImageTargetEventKind::kUpdated, img});
    }
  }
  for (auto it = map_.begin(); it != map_.end();) {
    if (std::find(seen.begin(), seen.end(), it->name) == seen.end()) {
      events.push_back({ImageTargetEventKind::kLost, *it});
      it = map_.erase(it);
    } else {
      ++it;
    }
  }
  return events;
}

}  // namespace slam_refact
