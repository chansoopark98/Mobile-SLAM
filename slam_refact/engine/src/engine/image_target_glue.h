// Engine private header: the image target part of the glue (contract FULL-3, US-013).
//   DetectedImagesForResponse = ImageDetectionRequestExecutor::fillDetectedImagesC8 (gating and
//     egomotion with the response's initial transform; PUB:reality/engine/imagedetection/
//     image-detection-request-executor.cc:181-272) + the image target branch of
//     rewriteCoordinateSystemPostprocess (PUB:reality/engine/geometry/orientation.cc:146-150,
//     330-334, 309-315, 483-503).
//   ImageEventTracker = the detectedImageMap_ diff of the JS tracking controller
//     (PUB:reality/app/xr/js/src/tracking-controller.ts:731-760).

#ifndef SLAM_REFACT_ENGINE_IMAGE_TARGET_GLUE_H_
#define SLAM_REFACT_ENGINE_IMAGE_TARGET_GLUE_H_

#include <vector>

#include "c8/hmatrix.h"
#include "engine/hit_test.h"
#include "slam_refact/image_targets.h"

namespace slam_refact {

// Located images of one response -> reality.detectedImages: lastSeen < 3 and firstSeen > 3,
// place = egomotion(init_pose, located.pose) then 704 (axes, mirror, recenterAndScale; width and
// height times the scale), sorted by ROI name (TreeMap), id = roi.faceId, trackingStatus unset.
std::vector<DetectedImageOut> DetectedImagesForResponse(const std::vector<LocatedImageOut>& located,
                                                        const c8::HMatrix& init_pose,
                                                        const CoordinateSystem& coords);

// JS imagefound / imageupdated / imagelost. Lives in the glue session (not reset by recenter).
class ImageEventTracker {
 public:
  // Events of one processed response, in dispatch order.
  std::vector<ImageTargetEvent> Update(const std::vector<DetectedImageOut>& detected);

 private:
  std::vector<DetectedImageOut> map_;  // detectedImageMap_ in key insertion order
};

// JSON.stringify(found) === JSON.stringify(img) for two published images of the same name: the
// position, rotation, scale = max(width, height) || 1, type and curvy geometry (the JS-side
// metadata and properties are constant per target). -0 == 0 and NaN == NaN like the strings.
bool SameForImageEvent(const DetectedImageOut& a, const DetectedImageOut& b);

}  // namespace slam_refact

#endif  // SLAM_REFACT_ENGINE_IMAGE_TARGET_GLUE_H_
