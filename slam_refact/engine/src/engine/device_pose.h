// TrackingSensorFrame.devicePose as func 647 prepareTrackingSensorFrame builds it
// (PUB:reality/engine/tracking/tracking-sensor-event.cc:76-86):
//   devicePose = toQuaternion(pose.devicePose).times(sensorRotationToPortrait)
// The web pose message carries no sensorRotationToPortrait, so the factor is Quaternion() (the
// identity). The product itself is exact, but Quaternion::times() normalizes its result: the
// device quaternion staged as four f32 values is divided by its own f32 norm. 647 stores that
// normalized quaternion at TrackingSensorFrame +104..+116 (f647 wat: quaternion product, then
// sqrt(z^2 + (y^2 + (w^2 + x^2))), 1 / n when n > 0, four multiplies), which is what 784 reads.

#ifndef SLAM_REFACT_ENGINE_DEVICE_POSE_H_
#define SLAM_REFACT_ENGINE_DEVICE_POSE_H_

#include "c8/quaternion.h"
#include "common/c8_adapters.h"
#include "slam_refact/types.h"

namespace slam_refact {

inline c8::Quaternion TrackingDevicePose(const QuatWxyz& staged) {
  return ToC8Quaternion(staged).times(c8::Quaternion());
}

}  // namespace slam_refact

#endif  // SLAM_REFACT_ENGINE_DEVICE_POSE_H_
