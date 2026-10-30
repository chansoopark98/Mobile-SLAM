"""Classification vocabulary and direct-evidence rules for the xr-slam.wasm binary map.

Every rule here maps a piece of evidence decoded from the binary (import/export name,
referenced string, RTTI/vtable membership) to a class and optionally a module.
"""

import re

CLASSES = (
    "emscripten-runtime",
    "libc/libc++",
    "eigen",
    "ceres",
    "glog-mini/other-3p",
    "8w-public",
    "8w-closed-slam",
    "8w-other",
    "unknown",
)

# Config-key classes ("Class.key" strings read via globalParams().getOrSet).
# Public = key literal present in public xr_reference@4edfa484 sources (checked at run time too).
CLOSED_SLAM_CONFIG = {
    "VioTracker": "VioTracker",
    "MapTracker": "MapTracker",
    "MapBuilder": "MapBuilder",
    "BundleAdjustment": "MapBuilder",
    "WorldMaps": "WorldMaps",
    "WorldMap": "WorldMaps",
    "FrameFrameTracker": "FrameFrameTracker",
    "MetricScaleEstimator": "MetricScaleEstimator",
    "Alignment": "Alignment",
    "MotionPrediction": "VioTracker",  # MotionPrediction params are read inside VioTracker::track (func 784)
    "EpipolarMatcher": "EpipolarMatcher",
}
PUBLIC_SLAM_CONFIG = {
    "Bundle": "Bundle",
    "FramePoint": "FramePoint",
    "GlobalMatcher": "GlobalMatcher",
    "PosePnP": "PosePnP",
    "GlRealityFrame": "Gr8",
}
OTHER_CONFIG = {
    "DetectionImageLoader": "ImageTarget",
    "DetectionImageMatcher": "ImageTarget",
    "DepthRenderer": "Render",
    "VpsMeshManager": "VPS",
    "VpsRequestManager": "VPS",
}

# ScopeTimer names that are absent from public sources: name -> (class, module).
CLOSED_SCOPES = {
    "track-last-points": "MapTracker",
    "sample-track-points": "MapTracker",
    "track-sampled-points": "MapTracker",
    "match-frame-points-with-projected-points": "MapTracker",
    "secret-relocalization": "MapTracker",
    "state-tracking": "MapTracker",
    "state-lost": "MapTracker",
    "map-tracker-set-frame-tracker-scale": "MapTracker",
    "map-tracker-get-world-points": "MapTracker",
    "map-keyframe": "MapBuilder",
    "prune-keyframes": "MapBuilder",
    "global-iterative-bundle-adjustment-cameras": "MapBuilder",
    "global-iterative-bundle-adjustment-points": "MapBuilder",
    "loop-close": "MapBuilder",
    "find-loop-close-target": "MapBuilder",
    "find-loop-close-pose": "MapBuilder",
    "remove-ground-points-behind-triangulated": "MapBuilder",
    "epipolar-match": "EpipolarMatcher",
    "match-keyframes": "EpipolarMatcher",
    "match-new-points": "EpipolarMatcher",
    "new-pts": "EpipolarMatcher",
    "gen-map-pts": "EpipolarMatcher",
    "sort-and-filter-duplicates": "EpipolarMatcher",
    "frame-tracker": "FrameFrameTracker",
    "local-matcher": "FrameFrameTracker",
    "compute-2d-inliers": "FrameFrameTracker",
    "compute-3d-inliers": "FrameFrameTracker",
    "depth-egomotion": "FrameFrameTracker",
    "ground-egomotion": "FrameFrameTracker",
    "propose-homography": "FrameFrameTracker",
    "fit-pairs": "FrameFrameTracker",
    "compute-frame-pose-using-robust-p2p": "FrameFrameTracker",
    "pose-optimization": "WorldMaps",
    "local-matcher-gorb": "WorldMaps",
    "local-matcher-orb": "WorldMaps",
    "map-points-subset": "WorldMaps",
    "project-points": "WorldMaps",
    "project-points-map-point-ids": "WorldMaps",
    "project-points-visible-points": "WorldMaps",
    "verify-point-visibility": "WorldMaps",
    "collate-matches": "WorldMaps",
    "dedup-dictionary-matches": "WorldMaps",
    "match-frame-points-with-point-cloud": "GlobalMatcher",
    "global-match-map-brute-force-with-reader": "GlobalMatcher",
    "metric-scale-estimator-push-frame": "MetricScaleEstimator",
    "temporal-alignment-imu": "MetricScaleEstimator",
    "smooth-vio-accel": "MetricScaleEstimator",
    "scale-estimation-frequency": "MetricScaleEstimator",
    "scale-estimation-spatial-alignment": "MetricScaleEstimator",
    "scale-estimation-fixed-spatial-alignment": "MetricScaleEstimator",
    "scale-estimation-camera-space": "MetricScaleEstimator",
    "scale-estimation-l2-camera-space": "MetricScaleEstimator",
    "agate-pca": "Descriptor",
    "agate-aggregate-bits": "Descriptor",
    "agate-brute-search": "Descriptor",
    "agate-descriptor": "Descriptor",
    "agate-extract-features": "Descriptor",
}
OTHER_SCOPES = {
    "on-new-frame-for-cloud-map": "VPS",
    "process-pending-cloud-maps": "VPS",
    "process-pending-graph-result": "VPS",
    "update-map-from-vps-response": "VPS",
    "copy-arcore-geometry": "Surfaces",
    "copy-arkit-geometry": "Surfaces",
    "set-surfaces-arcore": "Surfaces",
    "set-surfaces-arkit": "Surfaces",
    "extract-points-for-meshes": "Surfaces",
    "scale-solver": "ImageTarget",
    "image-world-tracker": "ImageTarget",
}
# Public scope names -> module (public scope names are found in public sources at run time).
PUBLIC_SCOPE_MODULES = {
    "compute-orb-keypoints": "Gr8",
    "compute-gorb-keypoints": "Gr8",
    "compute-key-points": "Gr8",
    "gr8gl-rois": "Gr8",
    "do-descriptors": "Descriptor",
    "pyr-draw-tex": "Gr8",
    "pyr-read-pixels": "Gr8",
    "global-match-brute-force": "GlobalMatcher",
    "global-match": "GlobalMatcher",  # features/global-matcher.cc (also used by image targets)
    "local-match": "LocalMatcher",  # features/local-matcher.h (also used by image targets)
    "global-match-map-popcount": "GlobalMatcher",
    "compute-inliers": "PosePnP",
    "compute-pose-confidence": "PosePnP",
    "pose-robust-p2p": "PosePnP",
    "pose-robust-pnp": "PosePnP",
    "propose-pose": "PosePnP",
    "fit-trios": "PosePnP",
    "input-points": "PosePnP",
    "pose-image-homography": "PosePnP",
    "trial-count-fail": "PosePnP",
    "trial-count-success": "PosePnP",
    "reality-loop": "Executor",
    "xr-engine": "Executor",
    "xr-engine-query": "Executor",
}
PUBLIC_OTHER_SCOPES = {
    "arcore-image-detection": "ImageTarget",
    "arkit-image-detection": "ImageTarget",
    "detection-image-prepare-matcher": "ImageTarget",

    "image-locate": "ImageTarget",

    "track-global": "ImageTarget",
    "track-local": "ImageTarget",
    "feature-set": "ImageTarget",
    "flip-vertical": "Render",
    "pixel-quad": "Render",
    "renderer-result": "Render",
    "resize-texture": "Render",
}

# Scope names that identify the executor functions themselves (public xr-engine.cc / tracking-controller.cc).
EXECUTOR_SCOPES = {"reality-loop", "xr-engine", "xr-engine-query"}

# Log prefixes of small accessors that get inlined into many callers: weaker identity evidence.
ACCESSOR_LOG_PREFIXES = {"WorldMap::getKeyframe", "WorldMap::getMutablePoint", "WorldMap::getPoint",
                         "world-map", "keyframe-descriptor"}

# Log-message prefixes "[prefix@method] ..." / "[prefix] ..." -> (class, module).
LOG_PREFIX = {
    "vio-tracker": ("8w-closed-slam", "VioTracker"),
    "map-tracker": ("8w-closed-slam", "MapTracker"),
    "keyframe-descriptor": ("8w-closed-slam", "MapBuilder"),
    "world-map": ("8w-closed-slam", "WorldMaps"),
    "world-map-collection": ("8w-closed-slam", "WorldMaps"),
    "WorldMap::getKeyframe": ("8w-closed-slam", "WorldMaps"),
    "WorldMap::getMutablePoint": ("8w-closed-slam", "WorldMaps"),
    "WorldMap::getPoint": ("8w-closed-slam", "WorldMaps"),
    "map-serialization": ("8w-closed-slam", "WorldMaps"),
    "majority-image-descriptor": ("8w-closed-slam", "Descriptor"),
    "frequency-scale-estimator": ("8w-closed-slam", "MetricScaleEstimator"),
    "feature-manager": ("8w-public", "FeatureManager"),
    "vps-graph-helper": ("8w-other", "VPS"),
    "vps-replaced-node-manager": ("8w-other", "VPS"),
    "vps-request-manager": ("8w-other", "VPS"),
    "vps-types": ("8w-other", "VPS"),
    "depth-renderer": ("8w-other", "Render"),
    "renderer": ("8w-other", "Render"),
    "gl-program": ("8w-other", "Render"),
    "GlMaterialBinder": ("8w-other", "Render"),
    "GlTextureBinder": ("8w-other", "Render"),
    "detection-image-local-matcher": ("8w-other", "ImageTarget"),
    "detection-image-matcher": ("8w-other", "ImageTarget"),
    "device-infos": ("8w-public", "DeviceInfo"),
    "engine": ("8w-public", "Executor"),
    "pose-request-executor": ("8w-public", "Executor"),
    "file-io": ("8w-public", None),
    "empty_any": ("glog-mini/other-3p", None),
}

# VPS endpoint constants (header-defined std::string globals -> per-TU static initializers).
VPS_STATIC_INIT_RE = re.compile(r"^(GetDataLayer|GetMesh|https://vps-frontend)")

# Literal non-prefixed log strings that name a closed SLAM component.
CLOSED_MESSAGES = (
    ("Data length of vio is not the same as imu", "MetricScaleEstimator"),
    ("Can only compute diff for same frequencies", "MetricScaleEstimator"),
    ("Keyframe Id has", "WorldMaps"),
    ("Associating invalid world point id", "WorldMaps"),
    ("Invalid map point", "EpipolarMatcher"),
    ("Newly added keyframe already had", "WorldMaps"),
    ("Removing frameId which is not in the point", "WorldMaps"),
    ("Local descriptor size does not match", "Descriptor"),
    ("Dictionary data not a multiple of Descriptor size", "Descriptor"),
    ("kfId index out of range", "WorldMaps"),
    ("Invalid point requested.", "WorldMaps"),
)

# Library string signatures (specific enough to identify the library).
LIB_STRING_RULES = [
    (re.compile(r"^external/ceres/"), "ceres", "ceres-source-path"),
    (re.compile(r"^(Ceres Solver|Solver::Options::|Trust region strategy|Line search failed|"
                r"Eigen failure|Eigen LLT decomposition failed|Residual and Jacobian evaluation failed|"
                r"projected_gradient_step = Plus|The user-provided parameter_blocks_for_inner)"), "ceres",
     "ceres-message"),
    (re.compile(r"^external/capnproto/|^_kjCondition|WirePointer::|^Schema mismatch|"
                r"^Message (contain|did not|is too|ends)|Cap'n Proto"), "glog-mini/other-3p", "capnp/kj-message"),
    (re.compile(r"^(tj[A-Z][A-Za-z0-9]*\(\)|libjpeg-turbo|JPEGMEM|TJ_[A-Z]+$|Failed to initialize JPEG)"),
     "glog-mini/other-3p", "libjpeg-turbo-message"),
    (re.compile(r"^\[json\.exception\.|^invalid string: |^invalid number; |^syntax error |^parse_error$"),
     "glog-mini/other-3p", "nlohmann-json-message"),
    (re.compile(r"^(-\+   0X0x|\(null\)|-0X\+0X 0X-0x\+0x 0x|No error information|POSIX|LC_ALL|C\.UTF-8|rwa)$"),
     "libc/libc++", "musl-message"),
    (re.compile(r"^(0123456789abcdefABCDEFxX\+-pPiInN|%Lf|%\.0Lf|%I:%M:%S %p%H:%M|%a %b %d %H:%M:%S %Y|"
                r"%I:%M:%S %p|%m/%d/%y|%H:%M:%S|: out of range|: no conversion|random device not supported |"
                r"Thursday|February|00010203040506070809.*)$"), "libc/libc++", "libc++-message"),
    (re.compile(r"^(libc\+\+abi: |terminating|terminate_handler unexpectedly returned|"
                r"Pure virtual function called!|std::exception|std::__libcpp_tls_create\(\) failed.*)$"),
     "libc/libc++", "libc++abi-message"),
    (re.compile(r"^(Skipping triangulation for duplicate input\.|WARNING: Triangle %d, %d, %d is counter-clockwise)"),
     "glog-mini/other-3p", "delaunay/triangle-message"),
]

# RTTI namespace -> class for vtable-slot membership.
OTHER_8W_CLASS_RE = re.compile(
    r"^c8::(Vps|Wayspot|Gl[A-Z]|Scene|Camera$|Group$|Light$|Object8|Renderable|RendererState|SceneRenderer|"
    r"GeometryRenderer|MaterialBinder|TextureBinder|EmscriptenFetch|FetchInternal|VpsFetch|DepthRenderer|"
    r"\(anonymous namespace\)::writePixelsToJpg)")
CLOSED_8W_CLASSES = {"c8::VioTracker": "VioTracker"}
PUBLIC_8W_CLASS_MODULE = {
    "c8::FftCompute": "MetricScaleEstimator",
    "c8::Scale3dPoseEstimation": "Bundle",
    "c8::PoseEstimationAnalytic": "Bundle",
    "c8::TriangulatePointMultiviewAnalytic": "Bundle",
    "c8::InversePoseEstimationImageTargetAnalytic": "Bundle",
    "c8::analytic::PoseEstimation": "Bundle",
    "c8::(anonymous namespace)::Gr8CpuNop": "Gr8",
    "c8::Gr8CpuInterface": "Gr8",
}


def rtti_class(demangled):
  """Map a demangled RTTI type to (class, module)."""
  d = demangled
  if d.startswith(("ceres::AutoDiffCostFunction<c8::", "ceres::SizedCostFunction")):
    return ("8w-public", "Bundle") if "c8::" in d else ("ceres", None)
  if d.startswith("ceres::"):
    return ("ceres", None)
  if d.startswith(("capnp::", "kj::", "nlohmann::", "c8flann::")):
    return ("glog-mini/other-3p", None)
  if d.startswith(("std::", "__cxxabiv1::")):
    return ("libc/libc++", None)
  if d in CLOSED_8W_CLASSES:
    return ("8w-closed-slam", CLOSED_8W_CLASSES[d])
  if OTHER_8W_CLASS_RE.match(d):
    if d.startswith("c8::Vps") or d.startswith("c8::Wayspot") or "Fetch" in d:
      return ("8w-other", "VPS")
    return ("8w-other", "Render")
  if d in PUBLIC_8W_CLASS_MODULE:
    return ("8w-public", PUBLIC_8W_CLASS_MODULE[d])
  if d.startswith("c8::"):
    return ("8w-public", None)
  return (None, None)


EXPORT_RUNTIME = re.compile(r"^(dynCall_|stack(Save|Restore|Alloc)$|emscripten_stack_|__wasm_call_ctors$|setThrew$|"
                            r"saveSetjmp$|testSetjmp$|getTempRet0$|setTempRet0$|_emscripten_|__set_stack_limits)")
EXPORT_LIBC = re.compile(r"^(malloc|free|calloc|realloc|memalign|emscripten_builtin_[a-z]+|__errno_location|fflush|"
                         r"htonl|htons|ntohs|ntohl|__cxa_[a-z_]+|__getTypeName|strlen|memcpy|memset)$")
OTHER_EXPORTS = re.compile(r"(DetectionImage|AreaTarget|Vps|Mesh|CurvyGeometry|Recorder|DataRecord|"
                           r"XrSessionToken|CompressedFormat)")


def export_class(name):
  if EXPORT_RUNTIME.match(name):
    return ("emscripten-runtime", None)
  if EXPORT_LIBC.match(name):
    return ("libc/libc++", None)
  if name.startswith("c8EmAsm_"):
    if OTHER_EXPORTS.search(name):
      return ("8w-other", "VPS" if "Vps" in name or "AreaTarget" in name or "Mesh" in name else "ImageTarget")
    return ("8w-public", "Executor")
  return (None, None)

# Link-order regions (wasm-ld keeps input order: system libs, app objects, then archives).
# Boundaries are anchored on direct evidence (see 01-binary-map.md); purity is re-measured by
# classify.py on every run. class None = application region (no default class).
LINK_REGIONS = (
    (260, 368, "libc/libc++", "system-libs-head: musl/libm, dlmalloc (malloc=300, free=301), emscripten stack 317-329"),
    (369, 1622, None, "app-1: reference runtime objects (executor, VioTracker, SLAM modules, image targets, render)"),
    (1623, 3475, "ceres", "ceres: losses 1626+, parameter_block.h 1651 ... visibility_based_preconditioner.cc 3475"),
    (3476, 3806, None, "app-2a: VPS managers/fetch/file-io/thread-pool objects"),
    (3807, 3915, "glog-mini/other-3p", "libjpeg-turbo 1.5.91: jmemmgr method table 3807-3817 ... version 3903, turbojpeg 3914"),
    (3916, 4092, None, "app-2b: VPS fetch, device-info, world-map/descriptor 4050-4082, logging context"),
    (4093, 4239, "glog-mini/other-3p", "capnp/kj: arena.c++ 4093 ... kj/units.c++ 4239"),
    (4240, 4245, "emscripten-runtime", "emscripten system libs: fetch helpers 4240-4241, GL proc-address tables 4242-4245"),
    (4246, 5698, "libc/libc++", "libc/libc++/libc++abi: __errno_location 4246 ... ntohs/htons 5687-5690, musl math kernels 5691-5698"),
    (5699, 5713, "emscripten-runtime", "dynCall_* legalization thunks 5699-5713"),
)

OTHER_MODULES = {"VPS", "Render", "ImageTarget", "Surfaces"}

# Hub helpers identified by reading their Ghidra decompile (structure, not names): index ->
# (candidate identity, class). Used as weak (INFERRED) votes.
STRUCTURAL_NAMES = {
    370: ("operator new(size_t): malloc loop + new_handler (5582)", "libc/libc++"),
    381: ("operator delete(void*): tail-calls free (301)", "libc/libc++"),
    448: ("abort wrapper (std::__throw_* under -fno-exceptions)", "libc/libc++"),
    380: ("std::string::__init(const char*, size_t)", "libc/libc++"),
    418: ("std::__put_character_sequence (ostream <<, sentry 419)", "libc/libc++"),
    279: ("memcpy (462 callers, no callees except import)", "libc/libc++"),
    261: ("memset (373 callers, leaf)", "libc/libc++"),
    503: ("memcmp (word-wise compare loop, 95 callers)", "libc/libc++"),
    504: ("std::__tree_balance_after_insert (red-black rebalance, 96 callers)", "libc/libc++"),
    577: ("std::__tree_remove (red-black erase)", "libc/libc++"),
    405: ("std::ostream numeric operator<< (sentry 419, num_put)", "libc/libc++"),
    553: ("std::string::assign(const char*, size_t)", "libc/libc++"),
    567: ("std::string::~string (is_long -> operator delete)", "libc/libc++"),
    906: ("std::__next_prime (210-wheel, hash tables)", "libc/libc++"),
    959: ("std::vector<4-byte T>::__append (resize + memset)", "libc/libc++"),
    461: ("sinf (musl __sindf/__cosdf 5694/5695)", "libc/libc++"),
    462: ("cosf (musl __cosdf/__sindf 5695/5694)", "libc/libc++"),
    620: ("std::__tree::destroy (recursive node free)", "libc/libc++"),
    552: ("std::string::assign(const char*, size_t) (grow via 5599)", "libc/libc++"),
}

# reference runtime public helpers identified from decompile + prior getter-pattern evidence
# (build/rea-analysis-20261015/xr_reference/config-defaults.json 'getter' field) -> (identity, module).
PUBLIC_HELPERS = {
    458: ("globalParams().getOrSet<float> (getter of 85 config xrefs)", "Params"),
    851: ("globalParams().getOrSet<int> (getter of 75 config xrefs)", "Params"),
    881: ("globalParams().getOrSet<bool> (getter of 48 config xrefs)", "Params"),
    1131: ("globalParams().getOrSet<double> (getter of 10 config xrefs)", "Params"),
    466: ("ScopeTimer::ScopeTimer (c8/stats/scope-timer; lazy global profiler init)", "Profiling"),
    468: ("ScopeTimer::~ScopeTimer (c8/stats/scope-timer)", "Profiling"),
    507: ("C8 fatal: log via printf/fflush then abort (C8_THROW under -fno-exceptions)", None),
    520: ("3x3 rotation -> quaternion (Shepperd, c8 HMatrix/Quaternion)", None),
}
