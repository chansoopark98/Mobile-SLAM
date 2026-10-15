# 저특징 환경·IMU 전파·수평면 후보: reference와 구현 경계

- 조사일: 2026-10-14. 현재 코드·pinned 8th Wall source·기존 바이너리 근거·공식 최신 웹 문서 확인. Context7 ARCore resolve/query 후 원문 확인.
- 결론: **기존에 관측한 수평면을 보관하고 IMU로 짧게 자세를 전파하는 것**과 **영상 특징 없이 새 바닥·위치·scale을 관측하는 것**을 구분해야 함.
- 권장 최소 범위: 기존 검증된 world map point로 수평면 후보를 추정하는 표시/관측 계층. 추정 카메라 높이·가짜 depth·새 plane factor·fresh 재분류 없이 구현. 실제 바닥 semantic 판별은 별도 미해결.
- 이 문서는 설계 제안. 제품 코드·SDK·서버·capture 변경 없음. 문서의 수치 gate는 vendor 알고리즘을 복원한 값이 아니라 검토할 **초기 engineering policy**.
- 후속 사용자 요청에 따라 **8th Wall source/WASM·실제 ARM64 ARCore·ARKit 공식 API**를 같은 설계 항목에 연결. 새 원문12개 HTTP200과 기존 ARCore disassembly3개를 [reference manifest](../../build/low-feature-20261014/references/manifest.json)에 URL/시각/byte/SHA256로 고정. Apple HTML이 JS 전용으로 반환되어 공식 `.md` 원문으로 확인.

## 개념 구분

| 결과 | 필요한 근거 | 말할 수 없는 것 |
|---|---|---|
| 기하학적 수평면 후보 | 공간적으로 퍼진 유효3D점, 중력 기준, 일관된 높이 | 바닥인지 탁자인지 semantic 구분 |
| semantic floor | 바닥을 구별하는 장면 정보·분류 또는 외부확인 | 수평이라는 이유만으로 바닥 확정 |
| assumed plane | 설정된 높이/원점/카메라 높이 등 가정 | 센서로 측정한 surface라고 표시 |
| 이전 관측면 보관 | 같은 map epoch의 과거 유효 geometry·마지막 관측시각 | 현재도 plane/camera가 정확히 추적된다는 보장 |
| IMU propagation | 마지막 pose/velocity/bias·동일 time base·유효 IMU interval | 새로운 시각 위치/scale/바닥 측정 |

- 화면의 `Z=0` 원점 grid는 현재 **좌표 기준면**. 실제 floor detection 결과가 아님.
- table/floor 모두 upward horizontal인 장면을 map geometry만으로 유일하게 구분할 수 없음. 가장 낮은 관측면도 실제 전체 공간의 바닥이라는 보장이 없음.

## ARCore에서 확인한 계약

- 공식 개념 설명: feature point 집합으로 plane을 추정하며 textureless surface는 잘 검출되지 않을 수 있음. low texture·반사·조명·움직임도 환경 제약. [fundamentals](https://developers.google.com/ar/develop/fundamentals), [environment](https://developers.google.com/ar/design/environment/definition).
- `HORIZONTAL_UPWARD_FACING`은 floor **또는 tabletop**에 해당하는 normal 방향 분류. semantic floor class가 아님. [Plane.Type](https://developers.google.com/ar/reference/java/com/google/ar/core/Plane.Type).
- Plane은 현재까지의 best knowledge이며 polygon/extents, normal, tracking state를 가진다. 여러 plane의 subsumption도 존재한다. [Plane](https://developers.google.com/ar/reference/java/com/google/ar/core/Plane).
- `PAUSED`는 나중에 재추적될 수 있지만 속성이 부정확할 수 있는 상태이고, `STOPPED`는 해당 trackable 추적 종료. 따라서 기존 plane 객체가 남아 있는 것과 계속 정확히 관측되는 것은 다름. [TrackingState](https://developers.google.com/ar/reference/java/com/google/ar/core/TrackingState).
- `INSUFFICIENT_FEATURES`는 실제 tracking failure reason이며 세부 없는 표면을 피하도록 안내한다. ARCore도 특징 부족을 완전히 해결한 엔진이라는 근거는 없음. [TrackingFailureReason](https://developers.google.com/ar/reference/java/com/google/ar/core/TrackingFailureReason).
- Instant Placement는 app이 공급한 초기 depth 추정을 사용할 수 있으며 실제 geometry 확인 후 pose/depth가 바뀔 수 있다. 이를 바닥 측정이나 metric scale 관측으로 쓰면 안 됨. [hit-test 종류](https://developers.google.com/ar/develop/hit-test).
- 이전 실제 ARM64 분석은 cached timestamp/state, pose별 dispatch, IMU 시간 guard·일부 extrapolation branch를 확인했음. **plane 검출 solver·featureless plane의 persistence 내부 정책·IMU-only 정확도를 복원한 것은 아님**. [기존 분석](2026-10-14-arcore-deep-runtime.md).

## 8th Wall source·공식 설명

- pinned monorepo: `4edfa484936a2cc691366eeec5c6e58bb6d4c8bd`.
- 공식 troubleshooting은 특징의 안정성으로 단일 ground plane을 갱신하며, 균일한 표면·반복무늬·탁자/바닥 동시 관측이 잘못된 floor 재설정을 일으킬 수 있다고 설명한다. 이는 ground 추정도 visual support에 의존한다는 근거다. [World Tracking Issues](https://8thwall.org/docs/troubleshooting/world-tracking-issues).
- 공개 source의 구체적 경계:
  - [tracker.h](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/tracking/tracker.h#L69): responsive 내부 공간에서 첫 camera를 origin, ground plane을 `y=-1`로 두는 설명. **실제 카메라 높이1m를 측정했다는 뜻이 아님**. 사용자-facing ground `Y=0` 설명과 내부 responsive 좌표를 혼합하지 않음.
  - [tracker.cc](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/tracking/tracker.cc#L113): 아래쪽45° ray 구간에 ORB feature를 확보하는 gravity-portion 설정. 이는 image feature를 쓰는 방법이지 특징 없는 면 관측이 아님.
  - [frame-point.cc](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/features/frame-point.cc#L99): initial depth를 `1/(scaledNormal·[x,y,1])` ray-plane 교차로 계산. 음수/매우 큰 값은 sky-distance fallback. 초기화 가정과 측정된 triangulation depth를 구분해야 함.
  - [epipolar.cc](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/geometry/epipolar.cc#L63): 내부 ground point `(0,-1,0)`와 normal을 camera frame으로 옮겨 scaled normal 구성.
  - [hit-test-performer.cc](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/hittest/hit-test-performer.cc#L211): 전달된 surface vertices/faces에 ray 교차를 수행해 detected/estimated 결과 구분. **query 함수가 plane을 새로 관측한다는 근거는 아님**.
- 원본 standalone WASM의 `configureXr/query/stageFrame/processStagedFrame` 경계는 이미 확인했지만 floor-fitting 전체 내부 구현과 위 source의 전체 binary build 동등성은 확인하지 않았음. [바이너리 보고서](2026-10-14-8thwall-binary.md).
- 공식 hitTest API URL의 현재 직접 fetch는 실패하여 해당 내용을 추정하지 않고 pinned source를 근거로 삼음. 오래된 forum/Stack Overflow 답변을 현재 vendor 기능 계약으로 사용하지 않음.

### 8th Wall detected/estimated surface의 정확한 source 조건

- 신규 획득 `hit-test-performer.cc` SHA256 `aae07cd46ad8f5027c3fb31d4ec23700a421949dd6b780241b24aa3901217a17`; 위 pinned revision. 실제 ray query와 새 plane 측정을 혼동하지 않도록 분기 자체를 확인.
- `getSurfacePoints`의 조기 반환: surface set 없음, surface 수0, **`!includes(DETECTED_SURFACE) && includes(ESTIMATED_SURFACE)`**. 따라서 이 pinned source는 estimated-only mask에서도 조기 반환한다. 이를 전체 vendor runtime의 의도된 최신 API 계약으로 일반화하지 않음.
- 각 surface의 기존 face triangle을 camera frame으로 변환하고 평면을 정면으로 회전시킨 뒤, `intrinsic.inv()`로 얻은 query ray와 교차. projected point가 convex triangle 내부인지와 camera Z≥0인지 확인.
- triangle 내부의 유효 교차는 requested mask에 detected가 있으면 detected 결과, 아니면 estimated 결과에 추가. triangle loop에서 검출되지 않았어도 마지막 `camIntersectPoint.z()>=0`이면 estimated 결과에 넣는 fallback이 있다. 이 마지막 fallback에는 별도 `includes(ESTIMATED_SURFACE)` 검사도 없다.
- 즉 **estimated hit에는 기존 polygon 밖의 평면 연장 결과가 포함될 수 있음**. 현재 신규 수평면 후보의 지지 영역을 넘어 무한 바닥을 렌더하는 근거로 쓰지 않음. fallback의 최종 triangle 선택·빈 face boundary까지 검증한 runtime 실험은 이번에 없음.
- `containsResultType`는 empty mask0을 모든 종류 허용으로 취급. `performHitTest`는 feature hits/detected surfaces/estimated surfaces를 별도 결과 유형으로 직렬화. 이 종류 구분은 재사용할 설계 원칙이며 public source의 의심스러운 mask 조건을 현재 제품에 복제하는 제안은 아님.

## ARKit: 수평 정렬·surface 분류·raycast·limited 상태

- 실제 자료 확인: 이 Linux 프로젝트의 `assets/references/ARKit` 디렉터리는 없음. 기존 ARKit 연구에는 Apple 공식 lifecycle/timestamp/intrinsics 문서만 있으며, Apple SDK engine binary·Apple runtime 실행 근거는 없음. [availability](../../build/low-feature-20261014/references/availability.json), [선행 범위](2026-10-13-commercial-engine-reinspection.md). **ARKit 바이너리를 역분석했다는 주장은 하지 않음.**
- 공개적으로 확인된 통합 원리: camera feature의 frame간 변화를 motion sensor 정보와 결합하는 visual-inertial odometry. 실제 internal state/noise/solver·plane classifier의 model/학습/threshold는 공개 API 설명에서 확인할 수 없음. [Understanding World Tracking](https://developer.apple.com/documentation/arkit/understanding-world-tracking).
- `ARPlaneAnchor.Alignment.horizontal`은 중력에 수직인 평면. 이것만으로 floor/table 의미가 결정되지 않는다. [Alignment](https://developer.apple.com/documentation/arkit/arplaneanchor/alignment-swift.enum).
- `ARPlaneAnchor.classification`은 지원 기기에서 floor/table/wall 등을 별도로 판단하는 결과. plane detection보다 늦게 나올 수 있고 classification이 없으면 `.none(status)`이다. 따라서 수평면 후보 생성 성공과 semantic floor 성공을 별도로 표시해야 한다. 실제 지원 여부는 `isClassificationSupported`로 검사하며 문서의 역사적 기기 목록을 최신 전체 지원표로 사용하지 않음. [classification](https://developer.apple.com/documentation/arkit/arplaneanchor/classification-2r4x8), [plane 시각화 예제](https://developer.apple.com/documentation/arkit/tracking-and-visualizing-planes).
- `ARRaycastQuery.Target.existingPlaneGeometry`는 shape/size가 있는 plane을 대상으로 한다. `estimatedPlane`은 ray 주변 feature에 기반한 surface 추정이며, `.any`+scene reconstruction mesh 사용 시 mesh와도 교차할 수 있다. mesh가 있는 무특징 표면의 ray hit는 **IMU만으로 새 표면을 측정했다는 뜻이 아님**. [existingPlaneGeometry](https://developer.apple.com/documentation/arkit/arraycastquery/target-swift.enum/existingplanegeometry), [estimatedPlane](https://developer.apple.com/documentation/arkit/arraycastquery/target-swift.enum/estimatedplane).
- `ARCamera.TrackingState.limited(reason)`는 pose가 있더라도 정확도가 불확실한 상태. `insufficientFeatures`는 구별 가능한 특징 부족, `excessiveMotion`은 영상 기반 위치 추적에 지나치게 빠른 움직임. 정상 추적으로 재명명하지 않고 원인을 유지해야 한다. [features](https://developer.apple.com/documentation/arkit/arcamera/trackingstate-swift.enum/reason/insufficientfeatures), [motion](https://developer.apple.com/documentation/arkit/arcamera/trackingstate-swift.enum/reason/excessivemotion).
- 공식 lifecycle의 구체적 제한: **limited 상태에서는 plane anchor가 추가·갱신되지 않고 hit testing도 결과를 내지 않는다**고 설명한다. 저장 world map의 anchor를 복원해도 relocalization이 normal로 돌아오기 전에는 내용의 올바른 정합을 가정하지 않는다. 이는 우리의 retained-plane 상태를 새 관측으로 표시하지 않는 설계 근거다. [Lifecycle](https://developer.apple.com/documentation/arkit/managing-session-life-cycle-and-tracking-quality).
- 위 제한은 인용한 world-tracking lifecycle/API 문서의 계약이며, 모든 최신 hardware·mesh·raycast 조합의 내부 동작을 실험했다는 뜻이 아님.

## 세 엔진 근거가 실제 설계에 미치는 영향

| 확인 근거 | 현재 구현 후보에 반영할 계약 | 아직 모르는 것 |
|---|---|---|
| 8th Wall responsive ground `y=-1` 및 ray-plane initial depth | 관측 후보와 assumed plane을 분리; 우리 map 높이를0/고정 카메라 높이에 끼워 맞추지 않음 | 실제 standalone의 모든 초기 scale/ground 선택 정책 |
| 8th Wall source의 polygon 내부 detected / 밖 estimated fallback | supported extent 안 candidate 표시; extrapolated hit면 별도 종류·낮은 신뢰 표시 | binary와 public source 전체 동등성·모든 mask runtime 동작 |
| ARCore upward plane은 floor 또는 tabletop; ARKit은 classification을 별도로 제공 | geometry-only 결과는 `horizontal_surface_candidate`, semanticFloor는 unknown | 자체 semantic floor 분류기·Apple classifier 내부 |
| ARKit limited에서 plane 업데이트 중단·ARCore PAUSED 부정확성 경고 | fresh visual support가 없으면 새 plane 갱신 금지; 이전geometry는 age/epoch를 가진 retained reference | vendor별 유지시간·정확도 horizon |
| 실제 ARCore `0x1942ff8`의 초기시간 guard | IMU bridge 시작 epoch·초기 timestamp·last IMU endpoint 정렬을 검사 | 초기화 state 전체와 최적화 알고리즘 |
| 실제 ARCore `0x197fd1c`의 마지막 IMU 이후 query/extrapolation branch | missing bracket과 propagation을 측정·표시; future sample 가공이나 무제한 외삽 금지 | 사용중인 backend·실제 branch 빈도·허용시간 |
| 실제 ARCore `ArFrame_getTimestamp`는 저장값 accessor | API 호출시각과 observation timestamp 구분; plane observation age는 callback 횟수로 갱신하지 않음 | 물리 exposure/전체 clock calibration |

- ARCore 재사용 증거: ELF SHA `7c32d45c2ef1ebaf92deae86a7ba31b0dd00d39bde09ab0890535ff396d62050`. initializer guard는 timestamp≤0 또는 마지막 IMU보다 앞선 값을 오류 분기로 보내며, extrapolation 문자열은 bounded 실제 ARM64 함수 안 exact code xref와 연결됨. raw disassembly3개를 위 manifest와 함께 복사·hash 확인. 새 device 실행이나 전체 IMU-only 정확도 실험을 한 것은 아님.

## IMU 단독 구간에서 가능한 것

- 회전 전파: bias를 뺀 angular velocity를 적분. gyro bias와 noise가 있으면 누적 오차가 생김.
- 위치 전파: 마지막 velocity와 gravity-compensated acceleration을 적분할 수 있으나, visual correction 부재 상태에서 bias·gravity 방향 오차·초기 velocity 오차가 누적됨. 구현 가능한 propagation과 계속 관측 가능한 위치는 다름. [OpenVINS propagation](https://docs.openvins.com/propagation.html), [Propagator](https://docs.openvins.com/classov__msckf_1_1Propagator.html).
- 가속도 측정은 specific force이므로 정지/충분한 excitation 같은 조건 없이 gravity와 병진가속도를 완전히 분리할 수 없음. 정지에 가까울 때 gravity 방향이 roll/pitch의 기준이 될 수 있지만 height·지면까지 거리·floor semantic은 얻지 못함.
- 간단한 오차 예시: 상수 accel bias 오차 `δb`가 남으면 위치오차 성분은 `0.5·δb·t²`; 작은 tilt 오차 `δθ`는 약 `g·δθ`의 가짜 수평가속도를 만들 수 있음. 특정 기기에서 측정한 오차값이나 허용 시간을 뜻하지 않음.
- 일반 VIO도 global translation과 gravity축 yaw의 gauge 자유도를 가짐. 여기에 vision이 사라지면 이미 확보한 기준으로 dead reckoning하는 상태이며 새로운 world constraint를 얻는 것이 아님. [OpenVINS observability/FEJ](https://docs.openvins.com/fej.html).
- 권장 상태 계약: `visual-tracking`, `imu-propagated/limited`, `unavailable`을 구분. prediction의 최신 timestamp와 시각 보정 성공을 별도 의미로 유지. `poseFresh`가 현시각 결과를 뜻하는 기존 계약이라면 limited provenance/동일 frame identity를 함께 검사하고, 그 Boolean만으로 시각 보정 성공이나 신규 plane observation으로 집계하지 않음. pose age·last visual time·prediction horizon·epoch를 함께 노출.
- fallback 시간 제한은 안전 정책이며 vendor 수치가 아님. 현재 gyro-only rotation rescue·bias·axis·clock 동작을 독립 검증하기 전 full translation rescue를 기본 채택하지 않음.

## 현재 map export의 좌표·필터

- `Estimator::getSolvedFeatures()`는 PnP용 ID/track count가 있는 경로. parameter candidate, slot cap, depth admission, pending 해제, 마지막 observation의 projection outlier 배제, solve flag1, finite positive depth를 요구. [estimator.cpp](../../src/backend/estimator.cpp#L477).
- **현재 JS의 `getMapPoints`는 위 함수를 호출하지 않는다.** `VIOEngine::getMapPoints()`→`getSlidingWindowMapPoints()` 경로다. [vio_engine.cpp](../../src/vio_engine.cpp#L411).
- 실제 exported map 필터: NON_LINEAR, parameter candidate/slot cap, `isDepthAdmitted`, `geometry_pending=false`, `solve_flag==1`, positive depth, `start_frame<WINDOW_SIZE`, finite world coordinate. `isDepthAdmitted` 내부에 valid anchor·finite depth 검사가 있음. [estimator.cpp](../../src/backend/estimator.cpp#L410), [feature_manager.h](../../include/frontend/feature_manager.h#L74).
- 차이: exported map은 마지막 observation의 `projection_outlier`를 별도 거부하지 않으며 ID/track age/covariance/current-observed 여부를 내보내지 않음. 따라서 동일 packet이 반복되거나 과거 window feature가 남아 있어도 새로운 독립 관측으로 세면 안 됨. 현 point set의 모든 점이 이번 frame에서 실제 보였다고 주장하지 않음.
- world position: `R_wb·(R_bc·ray·depth+t_bc)+P_wb`. gravity alignment는 `g2R`에서 입력 g를 `+Z`로 맞추고 초기 yaw를 제거. engine world는 Z-up이며, renderer `worldRoot.rotation.x=-π/2`로 Three.js Y-up으로 표시. gravity state의 부호와 물리 gravity force 방향은 따로 확인. [initializer.cpp](../../src/frontend/initialization/initializer.cpp#L324), [utility.cpp](../../src/utility/utility.cpp#L11), [renderer.js](../../web/js/renderer.js#L60).
- Worker는 유효 pose에서만 map을 읽고 wrapper는 request/epoch/pose freshness·finite point 배열을 검사한다. Lost에서 신규 map observation을 만들면 이 경계를 우회하게 됨. [vio-worker.js](../../web/js/vio-worker.js#L438), [vio-wrapper.js](../../web/js/vio-wrapper.js#L361).

## 최소 구현 후보: gravity-aligned map plane

1. 입력: 같은 `(clientEpoch,engineEpoch)`의 fresh visual result에 동반된 기존 map packet. 유효 point 부족·미초기화·invalid pose에서는 새 후보를 만들지 않음.
2. XYZ는 engine world 좌표로 유지. finite 검증 후 spatial duplicate를 억제. 기존 point의 수를 인위적으로 늘리거나 zero point를 채우지 않음.
3. gravity-aligned horizontal hypothesis `z=h`를 조사. 관측된 point의 z 값으로 height consensus를 찾고 inlier median으로h 추정. h를0이나 카메라 높이 상수로 설정하지 않음.
4. 높이 residual과 XY 분산/extent를 동시에 검사. 벽 위의 얇은 수평선·한 ray의 depth cluster는 2차원 support가 부족하므로 거부. confidence를 단순 point count로만 계산하지 않음.
5. 지도점의 한 부분에서 통과한 경우 그 support polygon/extent 안에서만 표시. 전체 공간을 무한 floor로 채우거나 unseen 영역으로 확장하지 않음.
6. 비슷한 지지를 가진 다른 높이 plane이 있으면 `ambiguous`로 유지. 가장 낮은 면이나 가장 많은 점의 면을 자동 semantic floor로 확정하지 않음.
7. 반환: `horizontal_surface_candidate`, center/normal/extent, inlier count·fraction·residual, source timestamp·epoch·last accepted age. 실제 label은 **수평면 후보/이전 관측 수평면**.
8. **estimator 입력과 분리**: scale/pose/origin/feature depth를 plane에 맞춰 수정하거나 IMU drift를 plane height로 감추지 않음. 이번 candidate를 새로운 metric constraint로 넣지 않음.

- 최신 projection-outlier 필터가 필요한 경우: `getSlidingWindowMapPoints`를 PnP solved filter와 정렬하는 작은 별도 변경 또는 더 엄격한 optional export를 review해야 함. 시각화용 baseline export를 무심코 바꾸지 않음. 새 dependency·semantic network는 이 최소안에 필요 없음.

| 초기 engineering policy 제안 | 이유·검증 요구 |
|---|---|
| spatially unique inlier≥20 | 3점 우연평면 억제.20은 vendor 상수나 현장 승인값이 아님 |
| height residual≤0.03 nominal m, median residual≤0.015 | 단위는 현재 map scale에 의존. 잘못된 metric 초기화 상태에서 실거리 정확도를 보장하지 않음 |
| 후보 support≥60% | 잡음 지배·혼합평면 억제. 전체 scene 아닌 평가 대상 packet의 비율임을 기록 |
| XY 두 principal extent 각각≥0.3m, covariance condition≤25 | 벽의 수평 strip·선/점 퇴화 거부. scale·point 분포 sweep으로 gate 검토 |
| 갱신 간 height jump≤0.05m | 갑작스러운 table/floor 교체를 거부하고 ambiguous로 전환. 실제지형변화를 강제로 평탄화하지 않음 |
| 같은 map epoch에서만 유지 | reset/reconfigure 이후 다른 좌표계 geometry 재사용 방지 |

- 여러 frame에서 같은 plane을 반복 fit했다고 독립 evidence가 늘어난 것은 아님. 최소안에서는 temporal consistency만 확인하고 confidence boost를 하지 않음; ID/관측 lineage가 없으면 unique multi-view support 수치를 만들지 않음.
- 위 threshold는 **제안**이며 구현 전 parent의 acceptance 기준으로 확정할 항목. 실제 broad floor를 약간 못 찾는 것을 false-floor 방지 gate 완화의 자동 근거로 쓰지 않음.

## featureless 구간의 이전 plane 보관

- 같은 epoch의 마지막 accepted geometry는 별도 cached reference로 유지 가능. `lastObservedTimestamp`는 새 관측 없이 갱신하지 않음.
- map inspector에서는 회색/점선의 이전 관측면으로 표시 가능. camera tracking이 유효하지 않으면 live-camera 정합 overlay는 숨기거나 정합 미검증임을 명시; 가짜 fresh pose로 정렬하지 않음.
- short IMU-only 구간에서 plane geometry를 보관해도, 그 plane에 대한 현재 카메라 위치 정확도를 새롭게 증명하지 못함.
- 시간 만료·epoch 변경·reset·stop·새 map으로 전환 시 정책대로 invalidation. parent가 TTL을 두면 그 숫자는 rendering 보관 정책이며 물리적 정확도 보장 시간이 아님.
- ARCore의 PAUSED trackable과 유사한 상태 분리는 참고 가능하지만 vendor가 임의 시간 동안 정확한 plane을 유지한다는 주장이나 구현 복제가 아님.

## 독립 회귀·허위 floor 방지

| fixture | 요구 결과 |
|---|---|
| 폭넓은 horizontal points + bounded noise | 알려진h·XYextent 허용오차 내 candidate |
| 동일점 반복·collinear·매우좁은 strip | unique-count/spread gate 거부 |
| vertical wall, tilted wall | horizontal candidate 거부; 좁은 z slice를 바닥으로 오인하지 않음 |
| table only | 수평면 후보는 가능, semantic floor 판정은 불가 |
| floor+table 혼합·비슷한 support | ambiguous 또는 다중 후보; 임의 plane 확정 금지 |
| sparse/outliers/NaN/Inf/negative·bad depth | 무효점이 support를 올리지 않음; 부족하면 unknown |
| 초기 origin grid·plane 없음 | measured floor가 새로 생성되지 않음 |
| known plane 이후 feature0 | 기존geometry aged/retained, observation count/time 불변, fresh 검출0 |
| 같은 timestamp 반복 packet | 새독립 관측으로 누적하지 않음 |
| epoch 변경·reset·stale worker reply | 과거plane 즉시 무효 또는 거부 |
| Z-up→renderer변환 | horizontal engine XY plane이 render XZ plane으로 일치; 높이부호/중복회전 없음 |
| IMU-only rotation/translation·bias/clockgap | limited state·bounded horizon·no new metric floor; extrapolated pose와 visual success 분리 |

- 현실 수락은 별도: textured floor→blank floor 전환, wall/table 혼동, 실제중력·camera/body extrinsic·실제floor height 독립측정, raw device motion/thermal 포함. 합성 통과를 현장 floor 정확도나 완전 IMU-only SLAM으로 보고하지 않음.
