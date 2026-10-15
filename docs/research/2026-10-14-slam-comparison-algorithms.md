# Mobile-SLAM · 8th Wall · ARCore 알고리즘·병목 비교

- 분석일: 2026-10-14, Asia/Seoul.
- 현재 Mobile-SLAM: `5d48e4bd7466842dad98928fb1f12988488402df`. 분석 대상은 browser가 쓰는 **VIOEngine → FeatureTracker → Estimator → Optimizer**. 별도 native `VIOSystem` 결과를 같은 경로로 취급하지 않음.
- 변경: 이 문서만 추가. 제품 코드·설정·active WASM·기존 미추적 연구 파일 변경 없음.
- 재검토한 최신 근거: [Oct13 성능 결과][D01], [코드 리뷰][D02], [Flip6 종합][D03], [후속 원인 분리·C3 결과][D04]. 과거 메모의 “A3/H4 계획 중” 상태는 폐기: H4·K1/K2·P1/P2·G1·C3를 실제 시험했고 다수 기각, C3는 기본 OFF 실험으로 남음.
- 이 보고서는 **현재 구현/기존 재현 증거**, **설계상 한계**, **아직 검증하지 않은 성능·정확도 가설**을 구분. 새 phone 측정·원본 engine 전체 재빌드·ARCore 동등 입력 실험은 수행하지 않음.

## 1. 결론과 우선순위

- 현재의 핵심 문제는 단순히 IMU 연산이 느린 현상으로 설명되지 않음. Oct12 numeric 기록의 Lost33개는 current visual factors가 0인29개·1인4개였고, IMU interval rejection0, Ceres·marginalization 미실행. **현재 영상과 연결된 허용 깊이 제약을 확보하지 못해 계산을 시작하기 전에 거절된 프레임**이다. [D03]
- 시간 병목은 frontend/backend tail을 우선 조사할 근거가 있음: 같은 기록의 frontend p95 41.377ms, backend p95 42.453ms, Worker queue p95 0.863ms. stage p95를 합산하지 않으며 numeric 기록의 IMU 정상 판정은 물리 exposure/IMU 동기화 정확도를 증명하지 않음. [D03]
- 원본 동일300frame에서 active239fresh→current53fresh의 직접 원인은 source ablation으로 A2의 persistent geometry admission까지 좁혀짐. frame37은 tracker41 동일인데 current39→1. 이는 “Tracker가 이미 41개를 찾았는데 왜 SLAM이 안 되는가”에 대한 현재 가장 구체적인 답이다. [D04]
- 다만 기존239fresh에는 acc-bias 경고128개와 최대 공개 step3.786m가 있음. admission을 풀어 fresh만 되돌리는 것은 정확도 복구가 아님. C3 ON은 raw53→93fresh이나 reset2→3으로 기각, 보정 입력179→250fresh만으로 기본 승격 불가. [D04]
- 검토 범위에서 새 CRITICAL/HIGH 메모리·수명 결함을 재현한 것은 없음. 기존 review APPROVE는 policy containment와 회귀 범위의 결론이며 추적/GT 수락은 여전히 NOT_ACCEPTED. [D01] [D02] [D04]

## 2. 동일 pipeline 기능별 비교

| 단계 | 현재 Mobile-SLAM | 확인된 8th Wall | ARCore 공개 계약 / 미확인 |
|---|---|---|---|
| 영상 입력 | gray image, dimensions, frame timestamp, IMU batch를 VIOEngine 단일 호출로 처리; browser/Worker 계약 별도 | getUserMedia video→compute/draw texture, currentTime repeat 판정, frame별 texture·time 보관 | frame/camera API 제공. raw image·texture K와 display pose 구분; 내부 capture scheduling은 여기서 미분해 |
| IMU 소비 | 영상 이전/이후 bracket, affine endpoint interpolation, gyro/acc 적분, interval equality guard | orientation quaternion+motion queue를 stage frame에 전달. 공개 world VIO 전체는 미공개 | visual features와 IMU 융합은 공식 확인; bias/preintegration/state 세부 미공개 |
| frontend | CLAHE, cached image/Scharr pyramid, KLT, border/F mask, 선택 gyro-seeded rescue, GFTT 보충 | GPU pyramid/score/readback + CPU ORB/GORB 및 ROI. GORB gravity orientation은 source 확인 | feature tracking 존재만 공식 확인; FAST/ORB/KLT 선택이나 feature budget 추측 금지 |
| feature identity | 직전 frame KLT lineage, 잃은 ID의 일반 descriptor 재획득 경로 없음; reappearing ID gap이면 새 track | 공개 image target local/global descriptor matching+기하 검증; world relocalization과 동일하다고 주장 불가 | feature map 존재; association/reidentification 상세 미공개 |
| 초기화 | 11-frame window, relative RT/SfM/PnP, gyro bias·gravity·positive scale 정렬, 실패 재시도 | closed world init 상세 미복원; 공개 wrapper의 origin/recenter와 solver init은 다름 | tracking state/실패 이유 API; 초기화 수식·기간·threshold 미공개 |
| 깊이 admission | 2-view candidate, pending rank/condition≤10000, positive depth·cheirality·all-view≤1.5px, 최종 검사 | 공개 target geometry는 known planar/curvy 모델; world depth admission 미복원 | 내부 깊이/map admission 미공개; Depth API 존재를 VIO landmark 정책으로 대체 금지 |
| backend | 11 pose/speed/bias 상태, 10 IMU factor, anchored inverse depth, Cauchy visual loss, Ceres DENSE_SCHUR/DOGLEG | binary frame export→Ceres/BA getter 경로 확인, 일부 residual 원본 code 수치 실행. 전체 solver/state/active branch 미복원 | 공개 근거로 Ceres/EKF/BA 종류·window size·iteration 단정 불가 |
| window 관리 | OLD keyframe / NEW general frame; square-root QR prior; 실패 시 prior release와 window advance | BA keyframe 설정 상수 일부 확인, runtime 적용 여부 미측정 | keyframe/marginalization 내부 미공개 |
| 가벼운 추적 | PnPFrontend 구현 있음, 기본 OFF, freq3 사용 시 backend 사이 별도 Ceres+IMU+fixed landmarks | stage 간 GPU/CPU 중첩 공개; 같은 PnP decimation 구조인지 미확인 | session update mode와 internal solve decimation은 다른 개념 |
| lost / recovery | current support<2이면 Lost; gyro rescue는 기존 feature 재추적 범위. 일반 map relocalizer 없음 | wrapper status/events, binary solver 복구 전체 미복원. image target global reacquire는 공개 | TRACKING/PAUSED/STOPPED 계약; 내부 recovery 전략 미공개 |
| 출력 / 표시 | camera `T_W_C`, image timestamp corrected pose; 미래 pose는 계약상 거부 | response와 texture 결합, display orientation/recenter/scale; generic camera pose smoothing 미확인 | physical camera pose는 image 중간 행 exposure 중심, display pose는 회전된 렌더 좌표; 유효 tracking state 확인 |

- Mobile-SLAM 근거: [C01]–[C14]. 8th Wall 근거는 [소스 architecture][D05] 및 [binary 분석][D06]의 pinned source/function/table 검증을 사용.
- ARCore는 visual+IMU 융합까지만 [공식 fundamentals](https://developers.google.com/ar/develop/fundamentals)에서 확인. CPU image intrinsics·physical camera·display-oriented pose·sensor pose 구분은 [Camera reference](https://developers.google.com/ar/reference/java/com/google/ar/core/Camera). update mode의 nonblocking은 최신 frame 재반환을 허용하므로 renderer FPS를 unique estimator FPS로 셀 수 없음. [UpdateMode](https://developers.google.com/ar/reference/java/com/google/ar/core/Config.UpdateMode)

### 모바일 웹 비교에서 8th Wall을 우선 볼 지점

| 구체적인 차이 | 8th Wall 실제 확인 | Mobile-SLAM 현재 | 의미 / 한계 |
|---|---|---|---|
| 픽셀 경로 | video→WebGL texture→shader pyramid→PBO/readback→WASM Gr8Gl | browser가 gray pixels 제공→WASM OpenCV CLAHE/pyramid/KLT | GPU 전처리로 CPU 비용 이동 가능성. readback·GL state·두 context upload·추가 frame age까지 계측해야 함 |
| 회전 강인성 | ORB intensity orientation와 GORB gravity-projected orientation, descriptor region 선택 | optical-flow local patch, IMU-rotation seed와 reverse/F 선택 rescue | 단순 “8th가 ORB라 우수”가 아니라 patch search 범위/descriptor 재검색/중력오류 민감도 차이로 시험 |
| 재대응 | 공개 image-target local/global matcher가 descriptor와 geometric model로 reacquire | current KLT ID가 사라지면 일반 mature map 재대응 없음 | descriptor matching이 private world recovery와 같다는 증거 없음; false-ID 인증을 포함한 연구 후보 |
| 작업 분산 | GPU current stage와 CPU 이전 frame 처리·render 분리, response texture 결합 | 한 Worker frame 호출에서 IMU→frontend→backend 수행 | main-thread/UI 분리와 algorithm intra-frame 분산을 구분. staging은 throughput 이득과 pose age 비용 동시 존재 |
| 품질/scale | closed world binary의 tracking state와 responsive/absolute mode·origin 변환은 별도 축 | independent metric scale/SE(3) 기준의 initialized/current-support/fresh 계약 | NORMAL 응답이나 매 callback pose만으로 metric fresh/GT_PASS와 동급 판정 금지 |

- GORB 호출 chain: `Tracker::track → FeatureDetector::detectFeatures(pyramid) → Gr8Gl::detectAndCompute → computeRayVectors → computeGravityPortions → computeOrbKeypoints/computeGorbKeypoints → computeGr8Descriptors`. 여기서 GPU에서 읽은 pyramid를 C++가 사용한다. 이름이 `Gr8Gl`이어도 descriptor 전체가 GPU라는 뜻은 아님. [D05]
- GPU chain: `onProcessGpu → stageFrame → GlRealityFrame.draw(DEFER_READ) → 이전 readPixels/getBufferSubData → 현재 drawTexture/filter/blur/decimate/score`; CPU `processStagedFrame → execute → response+texture`. PBO fence wait는 긴 polling이 아니라 단발 `clientWaitSync` 뒤 `getBufferSubData`에 의존하는 경로. GPU가 늦으면 readback stall 가능성을 배제하지 못한다. [D05]
- 공개 core `tracking-controller.ts`는 `configMask.setEstimateScale(false)`를 설정하고 origin.y 기반 scale 변환 code가 남아 있다. **이 공개 tracking source 설정을 별도 배포 `xr-slam.js`의 absolute scale 구현이라고 주장하지 않음**. binary 실험은 scale mode·origin·camera projection·orientation 공급을 반드시 기록해야 한다. [W01]
- 같은400 RGB frame fixture를 두 웹 엔진에 넣는 시험은 frame ingestion·event/status·latency 비교에는 유용하지만, 8th Wall deviceorientation/OS fusion·auto camera intrinsics·responsive scale과 현재 raw-IMU+명시 calibration이 다르면 동일 입력 SLAM 정확도 A/B가 아니다. source timestamp, frame texture 대응, result duplicate, per-engine unit/axes까지 맞추기 전 metric 우열 판단 보류.

## 3. 현재 실제 호출 순서와 비용

```text
VIOEngine.processFrame
  입력/epoch/timestamp 검증
  processIMUData
    pending queue + 이전/이후 bracket → image endpoint 보간
    Estimator.processIMU
      window IntegrationBase.push_back
      tmp_pre_integration.push_back
      P/V/R propagation
    [PnP 활성일 때 별도 PnP IMU 적분]
  FeatureTracker.detectAndTrack
    CLAHE → next pyramid → ordinary LK → F
    [약한 ordinary consensus일 때 gyro seed LK + reverse LK + trial F]
    [backend frame일 때 mask → GFTT → 새 ID]
  [PnP-only frame]
    solved ID 교집합 → lightweight Ceres → publish/invalid
  [backend frame]
    track_cnt>1 관측 수집 → Estimator.processImage
    parallax/keyframe → init 또는 solveOdometry
    triangulate/admit → removeFailures → Optimizer.optimize
      parameter/factor assembly → current/IMU gate
      Ceres → final landmark gate → apply → QR marginalization
    window slide → map/solved features → camera pose publish
```

- `processFrame`는 core 기준 동기 실행. WASM이 Worker 안에 있다는 사실은 C++ frontend/backend가 서로 병렬이라는 뜻이 아님. [C01]
- full backend는 기본 PnP OFF에서 매 accepted image마다 경유. PnP ON일 때 이미 backend decimation과 detector 보충 생략 경로가 있으므로 “가벼운 PnP 기능을 처음 추가”가 다음 작업이 아님. 해당 path의 전체 GT/phone 수락이 먼저 필요. [C01] [C07]
- `solver_time=0.06s`는 Ceres 옵션에만 전달. triangulation, factor assembly/whitening, postfilter, QR prior, rendering까지 포함한 frame deadline이 아님. `max_num_iterations`도 전체 pipeline cap이 아니다. [C06]
- adaptive solver 분기는 `pending_frames>0`일 때만 작동하지만 실제 `optimize()`는 `solveCeresProblem(problem)`의 default0 호출. queue 부하를 전달하는 active feedback은 이 경로에 없음. “adaptive가 켜져 있으므로 backlog가 자동 감소”는 성립하지 않음. 새로운 정책은 정확도 sentinel 없이 활성화하지 않음. [C06] [C19]
- 8th Wall은 공개 소스에서 CPU previous result→update→current capture→render→GPU current stage 순서. GPU pyramid/readback을 frame 사이에 분산하지만 지연이 없어지는 것은 아님. Mobile-SLAM의 CPU grayscale/KLT 경로와 native ARCore를 같은 이름의 “SLAM ms”로 비교하려면 구간 경계를 먼저 맞춰야 함. [D05]

## 4. tracker features와 current visual factors의 차이

| 지표 | 실제 포함 범위 | 잘못된 해석 |
|---|---|---|
| tracker count | LK 유지와 이번 GFTT 보충을 거친 활성2D 점 | 전부 map에 연결된 3D 제약 |
| track count>1 | 두 frame 이상 이어진 ID; VIOEngine 관측 전달 후보 | metric depth가 검증된 점 |
| isParameterCandidate | 유효 frame span 및 관측수≥2 | 단순 age로 전부 수락 |
| eligibleFeatures | candidate가 solver parameter slot 안에 있음; invalid depth도 포함 가능 | Ceres에 실제 추가된 visual factor |
| admitted depth | anchor/positive depth/geometry policy 통과 | 독립 GT 인증 |
| visualFactors | window 전체의 valid anchor→다른 frame projection residual 수 | 현재 image 제약 개수 |
| currentVisualFactors | 현재 window slot `WINDOW_SIZE`로 연결된 추가 residual 수; solve 후 final cull에서 다시 갱신 | 독립3D point 수나6DoF rank/innovation 인증 |
| historical map points | solved·nonpending·admitted landmark의 world 좌표 | 현재 frame에서 보이는 point |

- 실제 연결: [C01]의 `track_cnt>1` → [C03] [C18] candidate/admission → [C04] projection-factor 생성 → `imu_j==WINDOW_SIZE`인 경우만 current 증가. historical feature80개여도 현재관측 연결0이면 current0이다.
- 각 projection factor는 2D residual. current≥2는 **최소 fail-closed count**이며 visual-only 6DoF 관측가능성 보증이 아니다. 전체 graph는 IMU·prior·depth nuisance 상태를 함께 가지므로 단순히 “4식<6미지수”만으로 전체 underdetermined라고 결론내리는 것도 부정확. 필요한 것은 tangent-state Jacobian/Schur 정보의 rank·conditioning·innovation과 실제 GT. [C04] [C05]
- `hasObservableGeometry()`는 world-oriented bearing의 `Σ(I−bbᵀ)` eigenvalue 비와 good-view 수를 검사. camera rotation 자체가 추정값이므로 noisy orientation으로 생긴 겉보기 parallax와 진짜 translation geometry를 독립 검증하지 못한다. 이어 DLT·cheirality·reprojection guard가 존재하지만, 같은 window pose에 대한 self-consistency는 GT 인증과 다름. [C03]
- 현재 young track admission은 이미 구현됨. persistent pending이 legacy age만으로 자동 해제되지 않는 것이 A2의 의도이며, 그로 인한 support 감소를 관측한 상태. 단순 age gate 제거를 새로운 해결책으로 제안하지 않음. [C03] [D04]

## 5. frontend: 이미 있는 기능과 남은 범위

- CLAHE 객체 재사용, reference pyramid 재사용, Scharr derivative 포함 pyramid를 한 번 빌드해 ordinary/rescue가 공유. “LK마다 pyramid를 재생성하므로 cache부터 추가”라는 설명은 현재 code와 다름. [C02]
- ordinary branch는 KLT→border→undistorted F-RANSAC. 평균 이동<2px이면 stationary bypass, 유효 correspondence<30이면 F 추정 생략. 이때 finite/ray/border는 확인하지만 geometric consensus로 신뢰를 승격하지 않는 설계. **F 필터가 모든 frame에 적용된다고 말할 수 없음**. [C02]
- rescue는 **IMU 적분 전후의 동일 tracked-image interval**로 camera relative rotation 구성 → 기존 pixel ray 회전 seed → LK initial flow → ordinary 성공이면 patch error2배 개선 요구 → level0 reverse cycle≤0.5px → trial F inliers>ordinary2배일 때 채택. ordinary strict-majority가 강하면 추가 rescue 실행을 건너뜀. [C01] [C02]
- hard frame에서는 ordinary LK + seeded LK + reverse LK, F2회가 가능하므로 “IMU를 사용하니 항상 빨라진다”는 보장 없음. gyro는 seed 개선 정보이며 계산량 감소를 자동 보장하지 않는다. stage별 attempted/adopted/false-ID와 elapsed를 같이 측정해야 함.
- 지금 rescue는 frame-to-frame **이미 살아 있는 ID**에 한정. 가림 뒤 삭제된 mature map ID를 descriptor/patch로 재찾고 인증하는 일반 map-projection recovery는 확인되지 않음. `FeatureManager`는 frame gap ID를 기존 history에 접착하지 않고 새 track으로 다시 시작한다. 이것은 현재 지도의 오연결을 막는 안전 계약이다. [C03]
- 8th Wall 공개 image-target는 known target의 projected local descriptor search, global matcher, planar homography/curvy PnP 및 gravity P2P가 구현되어 있음. 이 구조는 비교 후보지만 world SLAM이 같은 reidentification을 쓰거나 Mobile-SLAM을 즉시 개선한다는 근거가 아니다. [D05]
- 기존 patch-warp 실험은 312개 case, true correspondence 회복과 함께 가려진 ID2774를 reverse/F까지 통과시킨 오채택이 있었음. cycle+F 통과만으로 map lineage를 인증하지 말아야 하는 실제 반례. [D01]

## 6. IMU: 느린 처리와 잘못된 측정의 구분

- 현재 endpoint interpolation·future bracket 유지·monotonic timestamp·gap·sum_dt 검사는 존재한다. 과거 missing_future/missing_past를 현재 미구현 기능처럼 다시 수정할 필요는 없다. 물리 capture offset, gyro unit/axis, K/extrinsic의 실측은 별도 미완료. [C01] [C04] [D03]
- `IntegrationBase`는 각 sample에서 delta p/q/v, 15×15 Jacobian/covariance, 18×18 noise를 갱신. pose propagation 자체와 covariance/Jacobian 누적 비용을 분리해야 함. residual evaluate와 sample integration도 다른 구간. [C08]
- `IMUFactor`는 생성 시 covariance whitening을 계산해 `sqrt_info_`에 보관. Evaluate마다 covariance 역분해하는 구조가 아님. D1은 bias-correction quaternion·Jacobian과 inverse rotation 재사용도 이미 적용됨. [C09] [D01]
- sample당 window preintegration과 `tmp_pre_integration`에 각각 `push_back`하는 것은 현재 source에서 확인됨. tmp는 image-frame map/initializer용이고 nonlinear 단계에서도 생성·갱신한다. **최적화 후보**는 초기화 후 tmp 데이터의 실제 소비를 확정하고 비용을 계측하는 것. 무조건 중복 버그 또는 큰 phone 병목이라고 판정하지 않음. reset/reinit·NEW interval merge·PnP가 서로 다른 이력을 필요로 하므로 단순 삭제 금지. [C10]
- NEW general-frame marginalization에서는 현재 raw IMU buffer를 이전 keyframe preintegration에 다시 push하여 합친다. frame별 sample 수·누적 interval·OLD/NEW 빈도에 따라 IMU 비용이 달라질 수 있다. 정상 적분과 장기간 retained interval의 재작업을 구분할 계측이 필요. [C10]
- 기존 D1 microbench: IMU1092 evaluation exact, 100000회264.673→257.752ms(2.62%, 목표10% 실패). fixed-work ABBA의 전체 engine p95 개선은 clean1.59%, fast는0.056% 악화로3% 목표 실패. 이는 “IMU kernel 손보면 전체 지연 해결”의 반증이며, **새 phone IMU 비용 측정값은 아님**. [D01]
- ARCore의 IMU·camera 접근과 8th Wall의 deviceorientation queue는 입력 품질/시간 계약이 서로 다름. 이 둘의 proprietary preintegration/EKF를 복제했다는 설명이나 IMU rate만 높이면 해결된다는 결론은 근거 없음.

## 7. 초기화·최적화·marginalization

- 초기화는 `solveInitialSfM→relativePose→global SfM→모든 frame PnP→VisualIMUAlignment`. reference correspondence>20, virtual-pixel parallax>30을 요구; gyro bias·gravity·positive scale 및 finite/rank 검증 존재. 초기화 지연에는 CPU time뿐 아니라 필요한 관측 운동이 오지 않아 계속 실패하는 시간이 포함됨. [C11]
- 초기화 gyro bias의 per-axis clamp±0.35rad/s는 실제 source 상수. 실제 Flip6 bias가 그 범위라는 측정은 아니며, 범위를 넓히는 행위로 시간 정렬/axis 문제를 보상하면 안 됨. [C12]
- 본 backend는 `WINDOW_SIZE=10`, 총11 pose/speed-bias 상태, extrinsic, candidate inverse depth. fresh solve 전10개 IMU factor의 실제 interval 일치와 current≥2 요구, 실패 시 prior release. Ceres usable+finite와 final depth/geometry 검사 통과 후에만 적용·marginalization. [C04] [C05] [C13]
- default observation policy0의 final filter와 실험1/2를 분리. policy1/2는 solved historical landmark의 나쁜 current row 제외·old/candidate depth 인증을 추가하지만 GT sentinel 실패로 opt-in에 격리. 기본0이 raw current residual1.5px reject 정책을 이미 모두 적용한다고 요약하면 틀림. [C05] [D01]
- Ceres의 `IsSolutionUsable`·finite cost·count는 global consistency 또는 충분한 실제 correction을 보증하지 않음. candidate evaluation이 전부 실패한 경우는 qualityReason으로 분리되며 finite propagation이 여전히 usable일 수 있다. 이 결정을 바꾸려면 기존 calibrated sequence와 GT을 포함한 새 regression 필요. [C06]
- marginalization은 **normal-equation eigen Schur 구현이 아니라 square-root QR**: dropped tangent columns CPQR→Qᵀ kept+residual→droppedRank 제거→kept unpivoted QR→prior. dropped rank만 thresholding하고 retained weak modes를 임의 제거하지 않음. diagnostic eigenvalue 분석이 있다는 이유로 runtime 계산을 eigen-Schur라고 부르면 틀림. [C14]
- 비용 후보: residual/Jacobian 재평가, dense dropped/kept matrix, 두 QR, frame마다 분배/복사. 기존 Qᵀ 최적화 후보는46fixture 중4개 strict byte parity 실패로 철회. 이미 실패한 후보를 성공한 최적화로 다시 계산하지 않음. [D01]
- keyframe 함수 `compensatedParallax2`는 현재 `p_i_comp=p_i`: 회전 보상하지 않는다는 source 사실. 하지만 K1/K2 실제 수정 실험이 GT/phone에 악화되어 폐기되었으므로 단순 gyro compensation 재도입이 안전한 fix라는 결론은 아님. [C15] [D04]
- 8th Wall에서는 binary Ceres 직접 도달·BA configuration·일부 residual Evaluate/Jacobian6/6까지 확인됨. window 전체·prior·IMU model·robust loss·actual solver-time/iteration은 여전히 미복원. “8th Wall이 BA를 안 하므로 빠르다”는 설명은 반박되며, “같은 Ceres이므로 성능이 같다”도 성립하지 않음. [D06]

## 8. PnP·recovery·출력 품질

- PnP는 단순 IMU-only prediction이 아님: 현재 correspondence≥6와 image spread, fixed backend anchor, IMU interval, Cauchy perspective residual, DENSE_SCHUR/DOGLEG를 쓰는 별도 small-window solve. bias와 extrinsic 고정, backend 사이 frame에서는 새 feature 보충을 건너뜀. [C07]
- speed 기능은 이미 존재하지만 기본 OFF, ON full-sequence GT/physical acceptance 없음. mature map/current ID 교집합이 줄어든 상황에서는 PnP도 current support를 새로 만들지 못한다. 이 옵션을 켜기만 하면 Lost가 복구된다는 보장 없음.
- current support 실패 시 prior release는 stale slot prior 적용을 막는다. window는 계속 이동하므로 정보량이 더 줄어드는 tradeoff가 있지만, 이전 P1/P2 보존 후보는 corrected phone fresh179→77/reset1 등의 실패가 있어 raw prior를 그대로 유지하는 변경은 금지해야 함. [C04] [C10] [D04]
- 코드의 failure detector threshold는 measured quality reason으로 기록되는 경로와 직접 reset 경로를 구분. C++ quality reason이 붙은 pose가 존재할 수 있고 제품 Worker motion guard는 translation warning 또는 공개 translation step 초과를 보고 reset·출력 억제할 수 있음. headless/native 결과와 제품 App의 published continuity를 같게 비교하면 안 됨. [C16] [C17]
- 현재 publication은 backend body pose를 extrinsic으로 camera pose로 바꾼 뒤 image timestamp로 반환. 현재 corrected output 계약에는 display-time future prediction이 없음; 최근 Worker/Wrapper가 future pose를 차단하도록 수정됨. ARCore의 display-oriented pose도 이름만으로 미래 extrapolation이라고 해석하지 않음. [C01] [D03]

## 9. 순위별 문제·후속 실험·수락 조건

| 우선순위 | 문제 / 증거 수준 | 다음에 분리해서 검증할 것 | 통과 / 중단 기준 |
|---|---|---|---|
| P0 | current metric support starvation **인과 재현됨** | frame28/37·가림 frame204 등 fixed pixels/IMU에서 ID별 tracker→candidate→admitted→current→final lineage; 실제 visible mature point의 재대응 가능한 최대치 | virtual support/GT 입력 주입 금지; false-ID0 고정 sentinel, current support·fresh 개선과 reset/jump·GT gate 동시 만족 |
| P0 | 현장 calibration/time/gyro 품질 **미확인** | 원본과 corrected gyro sensitivity를 독립 arm 유지, physical camera K/crop/IMU extrinsic/exposure offset 실측 | raw 입력 gate 실패를 corrected arm으로 대체 금지; independent body GT association 포함 |
| P0 | 비교 baseline source/active WASM mismatch **기존 확인됨** | source commit·build flags·loader hash·effective profile을 각 run에 저장; init/valid/Lost 집단 분리 | old active239 vs current53를 D1 단독 효과로 단정 금지; matched deterministic control 필요 |
| P1 | frontend/backend tail **기록 관찰됨** | same input/work, seed/thread, diagnostics OFF, ABBA; CLAHE/pyramid/LK/rescue/F/triangulation/Ceres/QR 각각 duration | primary pair 및 median engine p95≤0.97, p99≤1.10 안전; coverage·ATE/RPE 비악화, tail stage percentile 합산 금지 |
| P1 | tmp/window IMU 이중 계산 **source 확인, 비용 가설** | NON_LINEAR tmp consumer 검증; sample별 두 적분·covariance·propagation·NEW merge 비용 비중 | 현행 IMU1092 및 GT/initialization/reset/NEW merge parity; 실제 전체 speed 개선 없으면 채택하지 않음 |
| P1 | raw2D count·usable solver를 품질로 과대 해석 **관측/계약 한계** | full graph current support rank/conditioning/innovation 진단, gyro consistency와 output age | count threshold 완화 금지; independent GT gate로 새 quality 정책 검증; D0/D2/D3 실패 sentinel 필수 |
| P2 | adaptive backlog feedback inactive **source 확인** | pending frame 수 전달 대신 먼저 existing queue/age trace로 실제 backlog 원인 확인 | feedback 정책은 fixed budget/GT 대조; time cap만 낮춰 coverage/정확도 악화하면 폐기 |
| P2 | PnP decimation 기본 OFF **의도된 검증 경계** | 실제 ON/OFF full trajectory와 support-depletion/occlusion 비교 | solve 시간·pose age 개선 + map ID/anchor consistency·GT/current coverage 유지; ON 단독FPS 통과 금지 |
| P2 | GPU frontend/reidentification **연구 후보** | 8th Wall식 staged pyramid 또는 local descriptor search를 원본과 독립 prototype으로 비교 | real GPU/provider·frame association·readback age, false-ID 및 geometry gate 모두 통과 후 이식 판단 |

- 기존 수락 기준 유지: synthetic warmup300에서 fresh≥280, current-support failure<35, Lost episode<7, reset0; body-frame SE(3) scale1 ATE≤0.25m, rotation APE≤5°, 1s RPE≤0.10m/3°. full TUM은 matched baseline 대비 위치/회전RMSE≤1.05와 coverage 손실≤1%p. 모든 지표는 원래 해당 test scope에 적용하고 다른 입력에 임의 일반화하지 않음. [D01]
- **이미 기각한 방향을 재명명해 반복하지 않음**: H4 full free-state, D0 whole-landmark cull, D2/D3 default 승격, legacy-age bypass, P1/P2 prior 보존, K1/K2 keyframe 보상, G1 relaxed bearing geometry, patch-warp 복구. 새 가설은 기존 반례를 먼저 통과해야 함. [D01] [D04]
- 속도 측정의 선택 편향 주의: Lost에서는 Ceres/QR를 건너뛰어 더 빨라질 수 있고, recovery가 성공하면 처리비용이 증가할 수 있음. 성공 frame만 또는 전체 평균만 보고 성능을 판정하지 말고 fixed work + status별 time + end-to-end fresh age를 함께 기록.

## 10. 근거와 검사 범위

- 확인 완료: current HEAD, 현재 함수 호출·정책 default·source line, Oct13 결과/후속 리뷰·C3 실패와 격리, Oct14 8th Wall source/binary 보고서, ARCore 공식 API live 확인.
- 미수행: 새 성능 benchmark, 수정 코드 검증, phone/ARCore device 실행, vendor 내부 전체 역공학. 이 문서의 새 가설을 이미 실험 통과한 수정으로 보고하지 않음.
- 이하 C source link는 현재 checkout의 지정 시작 줄; 괄호 줄 범위는 검토 범위. D link는 결과·리뷰 원문. 기존 raw evidence는 해당 문서가 연결한 build artifact에서 확인.

[C01]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/vio_engine.cpp:175
- **C01** — [src/vio_engine.cpp (175–398)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/vio_engine.cpp:175)

[C02]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/frontend/feature_tracker.cpp:198
- **C02** — [src/frontend/feature_tracker.cpp (198–468)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/frontend/feature_tracker.cpp:198)

[C03]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/frontend/feature_manager.cpp:139
- **C03** — [src/frontend/feature_manager.cpp (139–438)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/frontend/feature_manager.cpp:139)

[C04]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/backend/optimizer.cpp:143
- **C04** — [src/backend/optimizer.cpp (143–374)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/backend/optimizer.cpp:143)

[C05]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/backend/optimizer.cpp:801
- **C05** — [src/backend/optimizer.cpp (801–891)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/backend/optimizer.cpp:801)

[C06]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/backend/optimizer.cpp:376
- **C06** — [src/backend/optimizer.cpp (376–430)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/backend/optimizer.cpp:376)

[C07]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/frontend/pnp_frontend.cpp:132
- **C07** — [src/frontend/pnp_frontend.cpp (132–345)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/frontend/pnp_frontend.cpp:132)

[C08]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/include/backend/factor/integration_base.h:20
- **C08** — [include/backend/factor/integration_base.h (20–240)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/include/backend/factor/integration_base.h:20)

[C09]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/include/backend/factor/imu_factor.h:16
- **C09** — [include/backend/factor/imu_factor.h (16–157)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/include/backend/factor/imu_factor.h:16)

[C10]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/backend/estimator.cpp:117
- **C10** — [src/backend/estimator.cpp (117–353)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/backend/estimator.cpp:117)

[C11]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/frontend/initialization/initializer.cpp:20
- **C11** — [src/frontend/initialization/initializer.cpp (20–154)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/frontend/initialization/initializer.cpp:20)

[C12]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/frontend/initialization/initial_alignment.cpp:13
- **C12** — [src/frontend/initialization/initial_alignment.cpp (13–118)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/frontend/initialization/initial_alignment.cpp:13)

[C13]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/include/utility/config.h:11
- **C13** — [include/utility/config.h (11–90)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/include/utility/config.h:11)

[C14]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/backend/factor/marginalization_factor.cpp:370
- **C14** — [src/backend/factor/marginalization_factor.cpp (370–421)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/backend/factor/marginalization_factor.cpp:370)

[C15]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/frontend/feature_manager.cpp:556
- **C15** — [src/frontend/feature_manager.cpp (556–584)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/frontend/feature_manager.cpp:556)

[C16]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/frontend/failure_detector.cpp:7
- **C16** — [src/frontend/failure_detector.cpp (7–55)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/frontend/failure_detector.cpp:7)

[C17]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/web/js/vio-worker.js:391
- **C17** — [web/js/vio-worker.js (391–435)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/web/js/vio-worker.js:391)

[C18]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/include/frontend/feature_manager.h:74
- **C18** — [include/frontend/feature_manager.h (74–86)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/include/frontend/feature_manager.h:74)

[C19]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/include/backend/optimizer.h:55
- **C19** — [include/backend/optimizer.h (55–63)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/include/backend/optimizer.h:55)

[D01]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/docs/fixes/2026-10-13-tracking-performance-results.md:1
- **D01** — [docs/fixes/2026-10-13-tracking-performance-results.md](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/docs/fixes/2026-10-13-tracking-performance-results.md:1)

[D02]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/docs/fixes/2026-10-13-code-review.md:1
- **D02** — [docs/fixes/2026-10-13-code-review.md](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/docs/fixes/2026-10-13-code-review.md:1)

[D03]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/docs/research/2026-10-13-flip6-analysis-and-application.md:1
- **D03** — [docs/research/2026-10-13-flip6-analysis-and-application.md](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/docs/research/2026-10-13-flip6-analysis-and-application.md:1)

[D04]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/docs/fixes/2026-10-13-flip6-tracking-recovery-results.md:1
- **D04** — [docs/fixes/2026-10-13-flip6-tracking-recovery-results.md](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/docs/fixes/2026-10-13-flip6-tracking-recovery-results.md:1)

[D05]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/docs/research/2026-10-14-8thwall-architecture.md:1
- **D05** — [docs/research/2026-10-14-8thwall-architecture.md](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/docs/research/2026-10-14-8thwall-architecture.md:1)

[D06]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/docs/research/2026-10-14-8thwall-binary.md:1
- **D06** — [docs/research/2026-10-14-8thwall-binary.md](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/docs/research/2026-10-14-8thwall-binary.md:1)

[W01]: /home/park-ubuntu/park/SOLUTION/Mobile-SLAM/assets/references/8thwall/official/8thwall/reality/app/xr/js/src/tracking-controller.ts:1214
- **W01** — [assets/references/8thwall/official/8thwall/reality/app/xr/js/src/tracking-controller.ts (1214–1408)](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/assets/references/8thwall/official/8thwall/reality/app/xr/js/src/tracking-controller.ts:1214)
