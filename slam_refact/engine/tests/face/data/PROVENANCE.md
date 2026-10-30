# tests/face/data

Byte copies of the public reference runtime ear fidelity goldens (US-019, docs/spec/08-face-ear.md §5 T2/T3).
They are real files in the reference checkout (not git-lfs pointers) and are copied here so the
`unit` tests run on a fresh clone. `ear_fidelity_test.cc` checks the sha256 of each copy.

| file | upstream | upstream commit | sha256 (copy = upstream working tree = `git show HEAD:`) |
|---|---|---|---|
| `fidelity_testing_cropping.json` | `../../../../../README.md#external-code` `reality/engine/faces/testdata/fidelity_testing_cropping.json` | `4edfa484936a2cc691366eeec5c6e58bb6d4c8bd` | `5a3bc9dc09375eedef42f1b879c6c6ded91726fc83a46408c50b54da3ce603fe` |
| `fidelity_testing_lifting3d.json` | same repository, `reality/engine/faces/testdata/fidelity_testing_lifting3d.json` | same | `1a57fce2f9a7ea49efbe0ef1f16f2401f2456edf1d93ad0a8dd382d5d23a3eff` |

Not copied (git-lfs pointers in the reference checkout, contents unavailable): the image of the
original tests `reality/engine/testdata/real_test_face.jpg`, `reality/engine/faces/testdata/{roi.jpg,face_mesh.glb}`,
`reality/engine/deepnets/testdata/{faces,ear_left,ear_right}.jpg`, and the models
`third_party/mediapipe/models/face_*.tflite`, `reality/engine/ears/data/ear_model_v{1,2}.tflite`
(the deployed copies of face-model, face-mesh-model and ear_model_v1 are read from
`assets/references/xr_reference/engine/resources/` by the `mlmodel` tests (label renamed from `ml-evidence` in RM-1), never copied).

## RM-7 recorded original-runtime inputs (not reference runtime repository files)

Produced by `slam_refact/tools/re/runtime/run-face-reference.mjs` (label `sincos60-r1`, headless Chromium with SwiftShader, original `xr.js` + `xr-face.js` and models) on the synthetic frames of `slam_refact/tools/face/gen_synthetic_frames.py`; used only by `FaceRoiRenderer.SubpixelSnapMatchesTheRecordedOriginalGpuCrop` (`face_roi_renderer_test.cc`), which checks the sha256 of each file. They pin the 4 subpixel-bit snap of the CPU ROI renderer; they are a post-hoc regression pin (expected values chosen after observing the recorded crop), not original-equivalence evidence and not a face image. Evidence copies: `build/slam-refact-full-face-evidence/reference/sincos60-r1/textures/texture-0002.rgba`, `build/slam-refact-full-face-evidence/rm7/g2-gpu-crops/gpu-crop-0000.rgba`, `build/slam-refact-full-face-evidence/rm7/g2-rois.txt` (first line).

| file | content | sha256 |
|---|---|---|
| `rm7_snap_texture_0002.rgba` | camera texture the original read back before staging (480x640 RGBA8, row 0 = top) | `2d121a15d3ed6854ccec6abda7e5f3f23b789302ce424ddb5d340d1362769789` |
| `rm7_snap_roi_0.txt` | `<id> <texture> m0..m15`: column-major `renderTexToImageTexMatrix44f` of the first original mesh detection | `56c99f1fac828012b166e33d90fc298a2f21881d770cae292f4f016e9bffe712` |
| `rm7_snap_gpu_crop_0.rgba` | the 192x192 RGBA8 crop the original GPU drew for that detection (cut from the original `renderedImg`) | `8f62fbcba598a589214388ae10782fa74102b7aa68e9213175d1fb26aee90515` |
