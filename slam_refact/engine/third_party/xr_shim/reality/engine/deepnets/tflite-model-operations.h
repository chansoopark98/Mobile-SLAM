// Shim of reality/engine/deepnets/tflite-model-operations.h (US-020).
//
// The original declares rotateSquareTensorFP16 / rotateModel (rotation of convolution weights for
// the landscape variants of the semantics model, built on third_party/half). The vendored
// tflite-multiclassifier.cc only includes this header and never calls either function, and the
// deployed LayersController always runs the model in portrait (layers-controller.ts
// SEMANTICS_MODEL_INPUT_WIDTH/HEIGHT), so nothing is declared here.
#pragma once
