# slam_refact engine — 동결 계약 (WP0b)

- 작성: 2026-10-15, WP0b. 근거: `slam_refact/docs/design/architecture.md` v2 §2.1, §3, §4, §8.1–8.2와 Fable 확인 리뷰의 구속 정정(NI-1, NI-6, NI-7, NI-9).
- 계약 버전: `slam_refact::kContractVersion = 10` (`include/slam_refact/version.h`). 1 → 2 변경(WP2-CC-1/3/4, WP3-CC-1, WP4-CC-1)은 `docs/design/contract-changes.md` "WP5b 반영 기록", 2 → 3은 같은 파일 "정확도 모드"(ACC-n), 3 → 4는 "reference runtime 전체 재구현"(FULL-1: XrController API, `include/slam_refact/xr_controller.h`), 4 → 5는 같은 절 FULL-3(이미지 타깃, `include/slam_refact/image_targets.h`), 5 → 6은 같은 절 FULL-6(이미지 타깃 ROI 요청 export와 추적 경로 debug, RM-6)에 있습니다, 6 → 7은 같은 절 FULL-7(RM-3a2/3b/4b: `GlueConfig::frames_until_recenter`, `GlueRunner::frames_until_recenter()`, 주입 pyramid의 ROI slot, metric 모드 hit, 비유한 origin·facing 거부, embind 확장)에 있습니다. 7 → 8은 같은 절 FULL-8(US-023/024: 가로·rotation 지원으로 R16 해소, 방향별 projection·pose·hit 입력 보정)에 있습니다. 8 → 9는 같은 절 FULL-9(US-014: 곡면 원통·원뿔 이미지 타깃, `PyramidRoiRequest::curvy`)에 있습니다. 9 → 10은 같은 절 FULL-14(US-030 D-1: `GlueRunner::image_target_geometry` 멤버 추가와 embind `getImageTargetGeometry`)와 US025-CC-1(동결 `Keyframe`의 꼬리 표시 위에 `global_descriptor` 추가, `MapBuilderOutput::world_correction`의 곱 순서·주석 정정)에 있습니다. 근거는 §1 3번(동결 선언을 바꾸면 버전을 올림)과 공개 헤더를 바꾼 FULL-1·3·6·7·8·9가 모두 올린 선례입니다. FULL-9의 APPLIED 범위는 **native·WASM의 CPU ROI 경로**(합성 gate, 원통·원뿔 VERIFIED, 단일 등록 시드)이고 웹 GPU 곡면 ROI는 미구현입니다(`docs/design/curvy-v3-firstrun-results.md`). 테스트 상태 주의: 곡면 gate v3의 첫 실제 실행이 끝났으므로(표식 5개, `build/slam-refact-full-20261019/preregistration/curvy-it-gate.v3.first-run.*.marker`) 표준 명령 `node --test slam_refact/engine/tests/wasm/*.test.mjs`는 **영구히 22개 중 20 통과**이고 곡면 2개(`image_target_curvy.test.mjs`의 cylinder·cone)는 `HELD: SLAM_REFACT_CURVY_DESIGNATION is not set`으로 실패하는 것이 정상입니다(회귀 아님; 지정 파일이 있어도 표식 때문에 재해제되지 않음). 같은 이유로 라벨 없는 ctest의 held 시험 2건도 영구 실패가 정상입니다. 곡면 WASM의 회귀 확인은 개발 fixture를 지정해(`SLAM_REFACT_IT_CYL_DIR`, `SLAM_REFACT_IT_CONE_DIR`) 실행합니다.
- 이 문서는 동결 대상, 변경 절차, 각 WP와의 접점을 적습니다. 구현(.cc)은 없습니다. 각 모듈의 .cc는 WP1–WP5가 씁니다.

## 1. 변경 절차

1. 동결 대상(§2)을 바꾸려는 WP는 먼저 `slam_refact/docs/design/contract-changes.md`(design lane 소유)에 요청을 적습니다. 요청에는 대상 선언, 바꿀 내용, 근거(spec 절, 증거 라벨), 영향받는 WP를 씁니다.
2. design lane이 승인을 같은 파일에 기록한 뒤에만 헤더를 고칩니다.
3. 고칠 때 함께 바꿉니다.
   - `include/slam_refact/version.h`의 `kContractVersion`을 1 올립니다.
   - `tests/common/contract_public_test.cc` 또는 `contract_internal_test.cc`의 해당 `static_assert`를 같은 변경으로 고칩니다. 이 테스트는 서명·레이아웃이 몰래 바뀌는 것을 막는 장치입니다.
   - 이 문서의 표를 고칩니다.
4. 다른 WP의 파일은 고치지 않습니다. 계약 변경이 다른 WP의 .cc를 깨면, 그 WP에 알리고 그 WP가 고칩니다.

변경 절차 없이 해도 되는 것 (§3 "WP 소유 영역")
- 동결 헤더의 `private:` 구역 내용
- `WorldPoint`, `Keyframe`의 "WP2-owned tail" 표시 아래에 멤버 추가
- 모듈 private 헤더(아래 표에 없는 `src/<module>/*.h`) 전부
- 모듈 `.cc`, `src/<module>/CMakeLists.txt`(동결 시점에 해당 WP로 소유권 이전), `tests/<module>/**`

## 2. 동결 대상

동결 범위 (NI-6): architecture.md §3 코드 블록의 모든 선언(어느 파일에 있든), §4 공개 타입, `src/common/tracking_types.h`, 그리고 §4.2 `TrackDebug` 필드마다 값을 꺼낼 getter 하나(§2.3 표).

### 2.1 파일 목록

| 파일 | 내용 | 구현 WP | 근거 |
|---|---|---|---|
| `include/slam_refact/version.h` | 버전, `kContractVersion`, 대상 WASM sha256, upstream commit | — | §2.1 |
| `include/slam_refact/types.h` | `Vec3f`, `QuatWxyz`, `HMatrix4f`, `TrackingStatus`, `TrackingReason`, `PinholeIntrinsics`, `ShaderNumericModel`, `ParamValue`, `WorldPointOut`, `StageTimings` | — | §4.1, §4.2 |
| `include/slam_refact/sensor_types.h` | `DeviceMotionSample`, `DeviceOrientationSample`, `RawSensorKind`, `RawSensorEvent`, `ImuEvent` | — | §4.1, D-14 |
| `include/slam_refact/image_types.h` | `GrayImageView`, `RgbaImageView`, `PyramidAtlasView`, `PyramidAtlasRows()`, `PyramidAtlasRoiRows()`(FULL-7, RM-3c: 주입 atlas의 최소 rows, 이미지 타깃 ROI slot 포함 여부별) | — | §4.1 |
| `include/slam_refact/engine_config.h` | `EngineConfig`, `GlueConfig` | — | §4.1 |
| `include/slam_refact/engine.h` | `FrameInput`, `TrackerStep`, `TrackerCore`, `Engine`(pimpl) | WP5 | §4.1–4.3, D-13 |
| `include/slam_refact/glue_runner.h` | `PublishedFrame`, `TrackerCoreFactory`, `DefaultEngineFactory`, `GlueRunner`(pimpl) | WP5 | §4.2–4.4 |
| `include/slam_refact/xr_controller.h` | XrController API 타입(`GraphicsIntrinsics`, `CameraProjectionUpdate`, `XrConfigureOptions`, `XrHitType`, `XrHitResult`, `LightingEstimate`, `CameraDirection`, `XrError`) | FULL-1 | spec 06 |
| `include/slam_refact/image_targets.h` | 이미지 타깃 공개 타입(`ImageTargetInput`, `LocatedImageOut`, `DetectedImageOut`, `ImageTargetEvent`, 상한 상수, 불투명 `ImageTargetSet`/`ImageRoiList`; FULL-6: `PyramidRoiSource`, `PyramidRoiRequest`, `LocatedImageOut`의 추적 경로 debug 필드 3개; FULL-9: `PyramidRoiCurvy`, `PyramidRoiRequest::curvy`) | FULL-3, FULL-6, FULL-9 | `docs/design/image-target.md` §4 |
| `src/common/tracking_types.h` | `map::PointSource`, `map::PointState`, `map::PointId`, `map::KeyframeId`, `TrackingSensorEvent`, `TrackingSensorFrame`, `TrackerInput`, `FrameState`, `MapTrackerOutput`, `MapBuilderOutput`, `RelocResult` | — | §3.1, D-5, D-6, D-7 |
| `src/frontend/frontend.h` | §3.2 블록 전체 + `DetectStats`, `FeatureFrontend::last_stats()` | WP1 | §3.2 |
| `src/map/world_map.h` | §3.3 블록 전체 + `PointView`, `VisibilityResult`, `ProjectedPoints`, `GetKeyframe/MutableKeyframe`, `NumKeyframes`, `NumPointsBySource`, `WorldMaps::active/ResetMap` | WP2 (M1) | §3.3 |
| `src/frame_frame_tracker/frame_frame_tracker.h` | §3.5 블록(아래 미동결 제외; `ClassifyHorizon`은 `std::vector<int32_t>`, CC-001) + `last_ground_match_count()`, `rng()`(CC-002) | WP2 | §3.5, WP2-PRE |
| `src/map_tracker/map_tracker.h` | §3.4 블록(아래 미동결 제외), `MapTrackResult` 전방 선언 | WP2 | §3.4 |
| `src/map_builder/map_builder.h` | §3.6 블록(아래 미동결 제외) + `KfPair` | WP3 | §3.6 |
| `src/vio_tracker/vio_tracker.h` | §3.7 블록 + `StatusOut`, `FrameRecord`, `TrackDebug`, TrackDebug getter | WP4 | §3.7 |
| `src/metric_scale/metric_scale_estimator.h` | §3.8의 `MetricScaleEstimator` | WP4 | §3.8 |

WP0b가 소유하고 구현까지 끝낸 것(동결은 아니지만 WP0b 외에는 고치지 않음): `src/common/{check.h, stage_timer.h, c8_adapters.h}`, `src/compat/{random_numbers.h, libcxx_algorithms.h}`, `src/config/**`, `cmake/**`, `CMakeLists.txt`, `CMakePresets.json`, `src/CMakeLists.txt`, `tests/CMakeLists.txt`, `tests/{common,config}/**`.

### 2.2 의도적으로 동결하지 않은 선언

architecture.md §3 블록에서 인자가 `/*…*/`로만 적힌 선언은 서명이 정해지지 않았으므로 동결하지 않았습니다. 해당 WP가 자기 private 헤더에 선언합니다. 다른 WP가 쓰게 되면 그때 계약 변경으로 올립니다.

| 선언 | 소유 WP | 이유 |
|---|---|---|
| `ComputeFramePoseUsingRobustP2P(/*…*/)` (1265) | WP2 | 인자 미정 |
| `SelectSubset(/*…*/)` (§5.4) | WP2 | 인자 미정 |
| `RobustGroundHomography(/*…*/)` (1279) | WP2 | 인자 미정, RNG 사용 방식은 WP2-PRE ①(S4 U6) 판독 후 확정 |
| `ShouldResetMap(/*…*/)` | WP3 | 인자 미정 |
| `FftCompute`, 순수 함수 1179–1200 | WP4 | 이름만 있고 서명 없음 |
| `MapTrackResult`의 정의 (S2 §2.5) | WP2 | `map_tracker.h`에는 전방 선언만 둠. `TrackPoints` 서명은 동결 |
| `FrameFrameTracker::SetPrevDepths`의 값 의미 | WP2 | 서명은 `const std::vector<float>&`(frame point index → depth)로 동결. 빈 칸 값(S2 FLT_MAX vs S4 −FLT_MAX)은 WP2가 판독해 정함 |

### 2.3 TrackDebug 필드별 값의 출처 (NI-6)

| `TrackerStep`/`TrackDebug` 필드 | 출처 (동결 getter 또는 출력 필드) |
|---|---|
| `predicted_pose` | `VioTracker::last_prediction().pose` (= `MapTrackerOutput::predicted.pose`) |
| `map_tracker_state` | `MapTrackerOutput::state` |
| `frame_tracker_ok` | `MapTrackerOutput::frame_tracker_ok` |
| `ok` | `VioTracker::loss_state().last_ok()` |
| `loss_state` | `VioTracker::loss_state().state()` |
| `lost_frames` | `VioTracker::loss_state().lost_frames()` |
| `frames` | `VioTracker::loss_state().frames()` |
| `yaw_offset_initialized` | `VioTracker::yaw_offset_initialized()` |
| `map_reset` | `MapBuilderOutput::map_reset` |
| `keyframe_created` | `MapBuilderOutput::keyframe_created` |
| `scale_latched` | `VioTracker::scale_latched()` |
| `latched_scale` | `VioTracker::latched_scale()` |
| `num_keypoints`, `num_orb`, `num_gorb` | `FeatureFrontend::last_stats()` |
| `num_keyframes` | `WorldMap::NumKeyframes()` |
| `num_points_by_source[5]` | `WorldMap::NumPointsBySource()` (index = `PointSource` − 1) |
| `ground_matches` | `FrameFrameTracker::last_ground_match_count()` |
| `timings` | `StageTimer` sink (`src/common/stage_timer.h`) |

## 3. 계약상 결정 (설계 문서와 다른 점 포함)

| # | 내용 | 근거 |
|---|---|---|
| C-1 | `map::PointId`/`KeyframeId`는 공개 `c8::WorldMapPointId`/`WorldMapKeyframeId`(= `XrId{size_t index, uint32 id}`)의 별칭입니다. vendored `FramePoint`가 이 타입을 저장하므로 경계에서 변환이 없습니다. **정렬은 공개 `XrId::operator<` = (id, index) 사전식**입니다. architecture.md §3.3 주석은 "(slot, id)"라고 적었으나 공개 소스(`PUB:reality/engine/xr-id.h:23`)와 다릅니다. S2 §3 G 주석도 "id 순 정렬"과 "(slot, id)"를 같이 적어 모순입니다. 원본 바이너리의 비교 순서 확인은 WP2가 판독해 `contract-changes.md`에 결과를 남기기를 요청합니다. | SOURCE `xr-id.h` |
| C-2 | `TrackingSensorEvent`는 공개 `ImuEvent`의 별칭입니다(D-14: 한 타입). 공개 c8 `tracking-sensor-event.h`는 capnp에 의존하므로 쓰지 않습니다. | §3.1, D-14 |
| C-3 | `TrackerInput`은 `tracking_types.h`에 공개 `tracker-input.h:8-20`을 옮겨 정의합니다(필드 `frame_points`, `roi_points`, `pose`, `time_ns`). 공개 헤더가 vendored closure에 없기 때문입니다. | WP0a 트리 확인 |
| C-4 | `TrackerCoreFactory`는 `std::function<std::unique_ptr<TrackerCore>(const EngineConfig&)>`입니다. §4.3은 인자 없는 factory에 `DefaultEngineFactory` 기본값을 적었는데, 인자가 없으면 기본 factory가 설정을 받을 수 없습니다. | §4.3 |
| C-5 | `Engine`, `GlueRunner`는 pimpl(`class Impl; std::unique_ptr<Impl>`)입니다. 공개 헤더가 c8 타입을 노출하지 않으면서 WP5가 멤버를 자유롭게 둘 수 있게 하기 위해서입니다. | §2.1 |
| C-6 | `HMatrix4f`는 기본값이 항등행렬입니다. | §4.1 보강 |
| C-7 | WP0b가 §3 블록에 더한 보조 선언(위 §2.1 표의 "+" 항목)은 §3 서명을 선언하거나 TrackDebug를 채우는 데 필요한 것만입니다. 모두 동결합니다. | NI-6 |
| C-9 | WP2-PRE(`docs/re/04-wp2-pre.md`) 결과를 동결 전에 반영했습니다: `ClassifyHorizon` int32 반환, FFT의 `compat::RandomNumbers` 객체와 `rng()` 접근자, `MapTracker` 초기값·1146 params 주석, D-7 주석. 목록은 `slam_refact/docs/design/contract-changes.md` CC-001…CC-009. | WP2-PRE |
| C-8 | `MapBuilderOutput`은 S3 §2.6과 S4 §3.5를 합쳐 이름 붙였습니다: `+0x10 reloc_kf_id`, `+0x18 reloc_kf_request`(world SLAM에서 항상 false, §1.2 M2), `+0x1c world_transform_updated`(S3의 loop-close 성공 = S4 +6628), `+0x20 world_correction`. | S3 §2.6, S4 §3.5 |

## 4. 빌드 계약

### 4.1 include 규칙

- 공개 API: `#include "slam_refact/<name>.h"` (`include/`).
- 엔진 내부: `src/`가 include root입니다. 예: `#include "common/tracking_types.h"`, `#include "map/world_map.h"`.
- vendored: `#include "c8/..."`, `#include "reality/..."`, `#include "libcxx_compat/..."`. 모두 SYSTEM include로 등록됩니다(§2.5). Ceres와 Eigen도 SYSTEM입니다.

### 4.2 WP0a와의 접점

- `third_party/CMakeLists.txt`가 `sr_vendor_c8`, `sr_vendor_features`, `sr_vendor_geometry`(STATIC), `sr_libcxx_compat`(INTERFACE)를 정의해야 합니다. 없으면 configure가 이유를 적고 실패합니다. `-DSLAM_REFACT_ALLOW_MISSING_VENDOR=ON`이면 c8에 의존하지 않는 계약 검사(공개 헤더, 공개 계약 테스트, config 기본값)만 빌드합니다.
- 최상위 CMake는 `third_party/` 앞에 `Eigen3::Eigen`, `Ceres::ceres`를 만들어 둡니다.
- 엔진 컴파일 플래그(`-fno-exceptions -fno-threadsafe-statics -DC8_NO_EXCEPT -DJAVASCRIPT -fno-fast-math -ffp-contract=off -DEIGEN_FAST_MATH=0`, native `-march=x86-64`, WASM `-msimd128`)는 `sr_*`/`slam_refact_*` 타깃마다 `target_compile_options`로 붙습니다(NI-9). 전역 `add_compile_options`는 쓰지 않으므로 Ceres와 GTest는 자기 기본 플래그로 빌드됩니다. vendored 타깃도 이름이 `sr_`로 시작하므로 같은 플래그를 받습니다. `c8/time/now.cc`의 native 예외 처리(`-UJAVASCRIPT`)는 WP0a가 하고, WP0b는 덮어쓰지 않습니다(NI-1).
- `config_defaults.inc`의 생성기는 `src/config/gen_config_defaults_inc.py`입니다(WP0b). 입력은 `build/rea-analysis-20261015/xr_reference/config-defaults.json`(NI-7, 244 entries / 211 unique keys)이고, `tests/data/config/config-defaults.json`(WP0a 사본)과 sha256이 같습니다. architecture.md §2.1의 `tools/gen_config_defaults.py`(WP0a)와 역할이 겹치므로, 하나로 합칠지는 orchestrator가 정합니다. 출력 형식(`SR_CONFIG_BEGIN/KEY/END/UNRECOVERED` 매크로)은 이 문서의 계약입니다.

### 4.3 모듈 타깃

`src/<module>/CMakeLists.txt`는 `sr_add_module(<target> DEPS ...)` 한 줄입니다. 디렉터리의 `*.cc`를 glob(CONFIGURE_DEPENDS)하므로 WP는 .cc만 추가하면 됩니다. .cc가 없으면 header-only(INTERFACE) 타깃입니다.

| 타깃 | 디렉터리 | 의존 |
|---|---|---|
| `sr_common` | `src/common` | `sr_includes`, `sr_vendor_features`, `sr_vendor_c8`, Eigen |
| `sr_compat` | `src/compat` | `sr_common`, `sr_libcxx_compat` |
| `sr_config` | `src/config` (STATIC, `params.cc`) | `sr_config_defaults`, `sr_common`, `sr_vendor_c8` |
| `sr_frontend` | `src/frontend` | common, compat, config, `sr_vendor_features` |
| `sr_map` | `src/map` | common, compat, config, `sr_vendor_features`, `sr_vendor_geometry` |
| `sr_frame_frame_tracker` | `src/frame_frame_tracker` | `sr_map`, `sr_vendor_geometry` |
| `sr_map_tracker` | `src/map_tracker` | `sr_frame_frame_tracker`, `sr_map` |
| `sr_map_builder` | `src/map_builder` | `sr_map`, `sr_vendor_geometry`, Ceres |
| `sr_metric_scale` | `src/metric_scale` | common, config, Eigen |
| `sr_vio_tracker` | `src/vio_tracker` | frontend, map_tracker, map_builder, metric_scale |
| `sr_engine` (= `slam_refact::engine`) | `src/engine` | `sr_vio_tracker` |

테스트: `tests/<module>/CMakeLists.txt`가 있으면 추가됩니다. `sr_add_unit_test(<name> SOURCES ... LIBS ... [LABELS unit|evidence|integration])`를 쓰면 GTest, `-Werror`, ctest label이 붙습니다. evidence 테스트는 `SLAM_REFACT_HAVE_EVIDENCE`가 ON일 때만 등록합니다(미등록이면 BLOCKED로 보고, skip 아님). 참조 자산(gitignored)이 필요한 `mlmodel` 테스트(ML 실모델 7개 + python3/numpy, face 모델 3개)는 다른 규칙을 씁니다(RM-1c, 리뷰 RM-1b/RM-5 MAJOR-3): **configure는 항상 성공**하고(fresh clone에서도 unit 테스트를 빌드할 수 있음), 자산이 없으면 실제 테스트 대신 무엇이 없는지 적은 **항상 실패하는 테스트 하나**(`sr_ml_assets_missing`, `sr_face_models_assets_missing`; `tests/ml/fail_missing_mlmodel.cmake`)를 label `mlmodel`(ML 쪽은 `mlgate`도)로 등록하므로 `ctest -L mlmodel`은 `--no-tests=error` 없이도 FAIL입니다. `-DSLAM_REFACT_ALLOW_BLOCKED_MLMODEL=ON`은 그 실패 테스트 등록만 끄고(그러면 빈 label은 통과하므로 `ctest --no-tests=error -L mlmodel`로 돌립니다), 자산이 있을 때는 영향이 없습니다. ML 회귀 gate v3를 판정하는 17개 테스트는 label `mlgate`를 함께 가집니다(face 모델 테스트는 `mlmodel`만). RM-1d: 이 17개는 이름 목록 `tests/ml/data/mlgate_names.txt`로 고정되고, 판정 명령은 등록 R1과 같은 `SR_ML_REF_DIR=<고정 ref 사본> SR_ML_REPORT_DIR=<새 디렉터리> ctest --no-tests=error -L '^mlgate$' -E sr_ml_ref_generate -FA sr_ml_ref`(고정 `ml_ref` 사본을 쓰고 재생성 fixture는 제외)입니다. 판정 기준은 "17개 통과, `sr_ml_gate_names_mismatch` 없음, exit 0"이며, **"정확히 17개가 선택됨"은 판정 기준이 아닙니다**(이름만 바뀐 경우에도 17개가 선택됩니다). 이름 집합 비교는 미빌드, 개수 증감, 이름 변경, 중복, 목록 밖 테스트를 항상 실패하는 `sr_ml_gate_names_mismatch`로 드러내지만, **같은 17개 이름을 가진 옛 소스의 바이너리는 통과시킵니다**(바이너리/소스 해시 대조가 아닙니다). 또 RM-1d가 돌린 mlgate 17/17은 빌드 디렉터리의 `ml_ref`로 돈 인프라 확인이며 gate 재판정이 아닙니다. `-DSLAM_REFACT_ALLOW_BLOCKED_MLMODEL=ON`은 자산이 하나도 없는 fresh clone에서만 실패 테스트를 끄고, 일부만 있으면 ON에서도 등록합니다. held 시험(첫 실제 실행 대기 gate)의 label은 `held`입니다(`evidence-held`는 `-L evidence` 정규식에 걸려 금지). `evidence` 테스트의 "등록하지 않고 BLOCKED 로그" 규칙은 그대로입니다. label 이름에 `evidence`를 넣지 않습니다(`ctest -L evidence`는 정규식이라 같이 선택됩니다. 정확히 고르려면 `-L '^evidence$'`). WASM용 `sr_wasm_link_options(<target>)`은 `cmake/SlamRefactWasm.cmake`에 있습니다(§2.6 링크 옵션).

### 4.4 계약 검사

| 검사 | 타깃 / 테스트 | 조건 |
|---|---|---|
| 공개 헤더 단독 컴파일 | `sr_header_selfcontained_public` (OBJECT, `all`) | 항상 |
| 엔진 헤더 단독 컴파일 (`src/**/*.h` 전부) | `sr_header_selfcontained_engine` (OBJECT, `all`) | vendor 있음 |
| 공개 레이아웃·enum·서명 | `sr_contract_public_test` | 항상 |
| 내부 레이아웃·enum·서명, D-14 컴파일 가드, TrackDebug getter | `sr_contract_internal_test` | vendor 있음 |
| config 표 불변식, spec 대조 | `sr_config_defaults_test` | 항상 |
| config 컴파일 기본값 vs `config-defaults.json` (211 key, bit 단위) | `sr_config_defaults_vs_binary` | json 있음 |
| `globalParams()` 버전 캐시, override, abort | `sr_config_params_test` | vendor 있음 |
