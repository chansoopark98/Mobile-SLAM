// Coordinate system rewrite of face responses (US-018/US-019). See face_coordinates.h.

#include "face/face_coordinates.h"

#include <utility>

#include "c8/geometry/egomotion.h"
#include "c8/geometry/homography.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/hvector.h"
#include "c8/quaternion.h"

namespace slam_refact::face {
namespace {

// orientation.cc:574-598.
void MakeRightHanded(Position32f* p) { p->z = -p->z; }

void MakeRightHanded(Quaternion32f* q) {
  q->w = -q->w;
  q->z = -q->z;
}

void RecenterAndScale(Quaternion32f* q, const c8::Quaternion& o) {
  const c8::Quaternion nq = o.times(c8::Quaternion(q->w, q->x, q->y, q->z));
  *q = {nq.w(), nq.x(), nq.y(), nq.z()};
}

void RecenterAndScale(Position32f* p, const c8::HMatrix& o, float s) {
  const c8::HPoint3 np = o * c8::HPoint3(s * p->x, s * p->y, s * p->z);
  *p = {np.x(), np.y(), np.z()};
}

void Mirror(Position32f* p) { p->x = -p->x; }

void Mirror(Quaternion32f* q) {
  q->y = -1.0f * q->y;
  q->z = -1.0f * q->z;
}

// orientation.cc:181-212.
void MakeRightHanded(FaceResponse* r) {
  MakeRightHanded(&r->camera.extrinsic_rotation);
  MakeRightHanded(&r->camera.extrinsic_position);
  for (auto& face : r->faces) {
    MakeRightHanded(&face.transform.position);
    MakeRightHanded(&face.transform.rotation);
    for (auto& local_point : face.vertices) {
      MakeRightHanded(&local_point);
    }
    for (auto& normal : face.normals) {
      MakeRightHanded(&normal);
    }
    for (auto& attachment_point : face.attachment_points) {
      MakeRightHanded(&attachment_point.position);
      MakeRightHanded(&attachment_point.rotation);
    }
    for (auto& local_point : face.ear_vertices) {
      MakeRightHanded(&local_point);
    }
    for (auto& attachment_point : face.ear_attachment_points) {
      MakeRightHanded(&attachment_point.position);
      MakeRightHanded(&attachment_point.rotation);
    }
  }
}

// orientation.cc:403-432.
void Mirror(FaceResponse* r) {
  for (auto& face : r->faces) {
    Mirror(&face.transform.position);
    Mirror(&face.transform.rotation);
    for (auto& local_point : face.vertices) {
      Mirror(&local_point);
    }
    for (auto& normal : face.normals) {
      Mirror(&normal);
    }
    for (auto& attachment_point : face.attachment_points) {
      Mirror(&attachment_point.position);
      Mirror(&attachment_point.rotation);
    }
    for (auto& local_point : face.ear_vertices) {
      Mirror(&local_point);
    }
    for (auto& attachment_point : face.ear_attachment_points) {
      Mirror(&attachment_point.position);
      Mirror(&attachment_point.rotation);
    }
  }
}

// orientation.cc:383-401. The face scale is multiplied by the raw system scale (not `s`), as in
// the original.
void RecenterAndScale(const CameraCoordinates& system, FaceResponse* r) {
  const float s = system.scale > 0.0f ? system.scale : 1.0f;
  const c8::HVector3 op(system.origin_position.x, system.origin_position.y,
                        system.origin_position.z);
  const c8::Quaternion oq = c8::groundPlaneRotation(
      c8::Quaternion(system.origin_rotation.w, system.origin_rotation.x, system.origin_rotation.y,
                     system.origin_rotation.z));
  const c8::HMatrix o = c8::cameraMotion(op, oq);

  RecenterAndScale(&r->camera.extrinsic_position, o, s);
  RecenterAndScale(&r->camera.extrinsic_rotation, oq);
  for (auto& face : r->faces) {
    RecenterAndScale(&face.transform.position, o, s);
    RecenterAndScale(&face.transform.rotation, oq);
    face.transform.scale = face.transform.scale * system.scale;
  }
}

// orientation.cc:223-244.
void SwapWinding(ModelGeometryMsg* m) {
  for (auto& idx : m->indices) {
    std::swap(idx.b, idx.c);
  }
}

}  // namespace

void RewriteCoordinateSystemPostprocess(const CameraCoordinates& system, FaceResponse* response) {
  switch (system.axes) {
    case Axes::kRightHanded:
      MakeRightHanded(response);
      break;
    case Axes::kLeftHanded:  // Already in the correct coordinate system.
    default:
      break;
  }
  if (system.mirrored_display) {
    Mirror(response);
  }
  RecenterAndScale(system, response);
}

void RewriteCoordinateSystemFrameworkPostprocess(const CameraCoordinates& system,
                                                 ModelGeometryMsg* model_geometry) {
  if (system.axes == Axes::kRightHanded) {
    SwapWinding(model_geometry);
  }
  if (system.mirrored_display) {
    SwapWinding(model_geometry);
    for (auto& uv : model_geometry->uvs) {
      uv.u = 1.0f - uv.u;
    }
  }
}

}  // namespace slam_refact::face
