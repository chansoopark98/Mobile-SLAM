// Probe shim for public xr_reference c8/geometry/intrinsics.h.
//
// The original header includes reality/engine/api/reality.capnp.h (generated Cap'n Proto) for the
// device-specific Intrinsics factory. The world-SLAM files in the probe (bundle.cc) include it
// but only use c8_PixelPinholeCameraModel, so the shim exposes just the capnp-free includes. Any
// use of class Intrinsics fails to compile, which the probe reports instead of hiding.
#pragma once

#include "c8/geometry/pixel-pinhole-camera-model.h"
#include "c8/hmatrix.h"
