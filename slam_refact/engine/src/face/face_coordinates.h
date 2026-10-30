// Coordinate system rewrite of face responses (US-018/US-019).
//
// Port of the FaceResponse / FrameworkResponse overloads of PUB:reality/engine/geometry/
// orientation.cc on the plain structs of face_messages.h, with the same operation order:
//   rewriteCoordinateSystemPostprocess(CameraCoordinates, FaceResponse)  :507-526
//     makeRightHanded (:181-212) if axes == RIGHT_HANDED, mirror (:403-432) if mirroredDisplay,
//     then recenterAndScale (:383-401);
//   rewriteCoordinateSystemFrameworkPostprocess(CameraCoordinates, FrameworkResponse) :552-571
//     makeRightHanded (:223-230), mirror (:233-244).
#ifndef SLAM_REFACT_FACE_FACE_COORDINATES_H_
#define SLAM_REFACT_FACE_FACE_COORDINATES_H_

#include "face/face_messages.h"

namespace slam_refact::face {

void RewriteCoordinateSystemPostprocess(const CameraCoordinates& system, FaceResponse* response);
void RewriteCoordinateSystemFrameworkPostprocess(const CameraCoordinates& system,
                                                 ModelGeometryMsg* model_geometry);

}  // namespace slam_refact::face

#endif  // SLAM_REFACT_FACE_FACE_COORDINATES_H_
