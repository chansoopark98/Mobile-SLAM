# 상용 추적 엔진 비교와 적용 조건

- 조회일: **2026-10-13**, Asia/Seoul. 범위: 공식 API·공개 MIT 소스·기존 프로젝트 근거; 제품 runtime 수정 없음
- 결론: **영상 대응 유지, 깊이 관측가능성, 출력 상태, 입력 시각을 각각 다뤄야 함. 상용 엔진의 비공개 solver·임계값을 복구했다는 근거 없음**
- 기준: [10월12일 상용 조사](2026-10-12-commercial-vio.md), [웹 엔진 조사](2026-10-12-web-engines.md), [회전·깊이 후속 분석](2026-10-12-rotation-recovery-followup.md)
- 새 원문: [primary-manifest.json](../../build/tracking-performance-20261013/research/primary-manifest.json). 24파일 모두 획득, 총 **1,708,171bytes**, 파일당2MiB 제한, binary 다운로드0
- 조회 순서: Context7 ARCore·ARKit·8th Wall resolve/query → 공식 웹 조회 → Apple Markdown·고정 MIT source 저장·SHA256 검증

## 공개 사실과 프로젝트 차이

| 항목 | 현재 확인한 공개 사실 | 프로젝트 적용 판단·미확인 범위 |
|---|---|---|
| ARCore 영상 pose | `Camera.getPose()`는 영상 중앙 행의 노출 중앙 시점 physical camera pose. `getDisplayOrientedPose()`는 local Z축90° 배수 회전 차이. [Camera](https://developers.google.com/ar/reference/java/com/google/ar/core/Camera) | image-corrected pose의 시각과 렌더 pose를 구분. display accessor를 미래 prediction의 증거로 사용하지 않음. 내부 prediction horizon·fusion 주기 미공개 |
| ARCore capture time | `Frame.getTimestamp()`는 ns capture timestamp이며 time base가 명시적으로 정의되지 않음. [Frame](https://developers.google.com/ar/reference/java/com/google/ar/core/Frame) | browser `performance.now`, IMU event timestamp와 자동 동일시 금지. clock provenance·offset·drift·drop을 기록하고 exposure 검증과 분리 |
| ARCore 반복 frame | `Session.update()`는 blocking timeout 뒤 최근 frame을 다시 반환할 수 있음. 새 영상 여부는 timestamp 차이로 판정. [Session](https://developers.google.com/ar/reference/java/com/google/ar/core/Session), [UpdateMode](https://developers.google.com/ar/reference/java/com/google/ar/core/Config.UpdateMode) | output callback 수를 영상 처리율로 세지 않음. 현재 frame ID/timestamp 중복 검사는 유지; playback·UI FPS와 fresh image/pose coverage를 별도 집계 |
| ARCore calibration | CPU image와 GPU texture용 intrinsics API가 구분되며 focal/principal point/image size가 update마다 변경될 수 있음. [CameraIntrinsics](https://developers.google.com/ar/reference/java/com/google/ar/core/CameraIntrinsics) | 최종 crop/resize/rotation 픽셀의 K와 camera/body 변환·image geometry를 함께 고정. native ARCore K를 임의 browser stream에 재사용할 근거 없음 |
| ARCore 빠른 motion | 짧은 motion 동안 IMU로 pose를 추정하고 visual tracking을 재개; 긴 빠른 motion은 blur·tracking loss 가능. [Performance](https://developers.google.com/ar/develop/runtime) | bounded inertial propagation은 설계 단서. visual support 없는 pose를 fresh metric tracking으로 승격하는 근거 아님. 내부 시간 예산·covariance gate 미확인 |
| ARCore map lifecycle | PointCloud ID는 세션 내 지속되고 confidence·자체 timestamp를 제공. anchor/world 추정은 update 중 보정됨. [PointCloud](https://developers.google.com/ar/reference/java/com/google/ar/core/PointCloud), [Anchors](https://developers.google.com/ar/develop/anchors) | ID lineage·현재 관측과 역사 map·reset epoch를 구분. 출력 point 수가 내부 current factor 수를 의미하지 않음. anchor 안정성이 독립 절대 정확도는 아님 |
| ARKit 입력·추적 | camera feature의 frame간 변화와 motion sensing을 결합. 과도한 motion·blur·무특징/저조도는 품질 저하. [World Tracking](https://developer.apple.com/documentation/arkit/understanding-world-tracking) | total tracker feature 수만으로 current metric support를 판정하지 않음. 이미지·IMU·K/extrinsic·timestamp 오류와 depth 비관측을 분리 |
| ARKit capture/K | `ARFrame.timestamp`는 capture 시각, `ARCamera.intrinsics`는 이미지 픽셀 단위 pinhole K. [timestamp](https://developer.apple.com/documentation/arkit/arframe/timestamp), [intrinsics](https://developer.apple.com/documentation/arkit/arcamera/intrinsics) | physical sensor 동기·readout·보정 내부 구현이 공개된 것은 아님. SDK의 calibrated 입력 경로와 browser proxy 입력의 증거 수준이 다름 |
| ARKit 회복 | `notAvailable`, `limited(reason)`, `normal`; interruption 뒤 world-map relocalization 시도. 기존 위치·방향/환경과 불일치하면 계속 relocalizing일 수 있어 reset 경로 필요. [Lifecycle](https://developer.apple.com/documentation/arkit/managing-session-life-cycle-and-tracking-quality) | 관성 bridge·2D 재연결·metric 회복·기존 map relocalization·새 epoch 초기화를 구분. retry 또는 reset 횟수를 감춘 near-zero Lost 판정 제외 |
| 8th Wall 현재 배포 | 2026년1월 배포 binary에 SLAM 포함. MIT framework는 architecture/Face/Image/Sky를 포함하나 SLAM 제외. [Open Source](https://8thwall.org/docs/open-source) | 공개 framework 분석과 proprietary world-tracking 내부 분석을 구분. binary SIMD/thread/solver/threshold 미검사 |
| 8th Wall world API | output `intrinsics`는16개 column-major **projection matrix**, `trackingStatus/reason/worldPoints` 제공. [XrController](https://8thwall.org/docs/api/engine/xrcontroller/pipelinemodule) | projection matrix를 fx/fy/cx/cy의3×3 K로 직접 전달하지 않음. 문서 status 범위와 source comment 범위가 다르므로 실제 runtime enum은 미확인 |
| 8th Wall 환경 가정 | troubleshooting은 grayscale feature와 동적으로 갱신되는 단일 ground plane을 설명. 반복무늬·반사·조명 변화·빠른 motion에 취약. [World Tracking Issues](https://8thwall.org/docs/troubleshooting/world-tracking-issues) | 공개 UX 설명의 범위. arbitrary room VIO에 고정 Y=0·floor prior를 이식하면 새로운 물리 가정이 생김. 해당 설명으로 SLAM의 모든 landmark가 planar라고 추정하지 않음 |

## 8th Wall MIT 소스의 제한된 구조 분석

- 현재 framework `main`: **`4edfa484936a2cc691366eeec5c6e58bb6d4c8bd`**, commit date `2026-10-12T20:48:15Z`
- engine 배포 repo `main`: **`e1421ae742a9c5d654871367c64c1de3ad044caf`**, commit date `2026-04-03T16:59:30Z`
- 고정 소스5개는 이전 `eaee2b9a3677a83da27733af23ec7bcc112b4c34` snapshot과 **모두 byte 동일**. [비교·SHA](../../build/tracking-performance-20261013/research/source-comparison.json). 최신 commit 존재만으로 추적 알고리즘이 변경됐다고 보고하지 않음

| 고정 source | 직접 확인한 동작 | 시험 가능한 적용 후보 |
|---|---|---|
| [image-tracking-state.cc:60](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/imagedetection/image-tracking-state.cc#L60) | 알려진 target의 이전 pose로 local descriptor 검색→homography/PnP 검증→성공 시 좁은 재검색; 실패 후 global 검색 | 직전 supported mature ID의 **현재 프레임 재연결 subset**에 제한된 projection prior/descriptor 실험. arbitrary map의 metric depth 회복 증거 아님. radius/Hamming/inlier 수치는 복사하지 않음 |
| [camera-loop.ts:296](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/camera-loop.ts#L296) | 반복 frame 구분, compute/draw texture 재사용, GPU→CPU→update 분리. `frameTime=performance.now()/1000` | 기존 최신 pending slot·rVFC·cached pyramid를 재도입하지 않음. 신규 GPU 후보는 texture upload/readback stall 포함 end-to-end 계측; 이 frameTime은 노출시각 증거 아님 |
| [tracking-controller.ts:439](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/tracking-controller.ts#L439) | repeatFrame이면 stage 생략; `videoTime`, `frameTime`, 별도 callback 시각 전달. pose event timestamp 전달 TODO 존재 | API parameter 존재와 physical synchronization 구분. event clock·arrival clock·영상 clock을 보존하고 synthetic offset sweep으로 오차 민감도 검사 |
| [sensors.ts:49](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/sensors.ts#L49) | DeviceMotion arrival/event timestamp/interval 보존; event queue가5초 초과 지연되면 비움 | queue drop을 진단에 남기고 IMU interval/bracket 실패와 구분. 상용 wrapper의5초 수치를 자체 무영상 propagation budget으로 채택하지 않음 |
| [gl-reality-frame.cc:658](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/features/gl-reality-frame.cc#L658) | deferred 옵션은 readPixels→현재 texture draw, ordinary는 draw→readPixels. WebGL pixel-pack-buffer/getBufferSubData 사용 | 병렬화가 자동으로 같은 frame·낮은 age를 보장하지 않음. deferred 후보는 픽셀 ID와 대응 timestamp를 같이 이월; 현재 시각으로 이전 픽셀 라벨링 금지 |

## 적용 순서와 실패 조건

1. **현 시점의 병목을 재확인**
   - Oct12 실제 기록은 Lost33행 currentVisualFactors0/1, 해당 행 solver 미실행. 수치는 이전 분석 근거이며 이번 연구 lane에서 재실행하지 않음
   - 2D track 정확도·ID 생존, depth rank/condition/reprojection, current residual membership, pose publication을 동일 timestamp로 연결
   - frame300의 정확한2D 대응과 metric current0은 depth 비관측 경로의 단서. descriptor로 parallax를 만들 수 없음
2. **건강한 추적의 비용을 먼저 보존**
   - ordinary cached-pyramid LK가 충분하면 recovery 경로 생략. 실패 subset만 seed/warp/descriptor 재시도; 별도 cap·추가 시간·채택 수 기록
   - 잘못된 gyro 축/부호/시간, 반복무늬·가림·FOV exit에서 false ID 증가하면 제외. F 완화·feature 총량 감소로 통과시키지 않음
3. **깊이와 bearing 정보의 수명 분리**
   - provisional/projective factor는 metric solved landmark와 구분. rho0에서도 회전 정보가 있다는 projective geometry를 사용하려면 nuisance-depth rank·gauge·OLD/NEW anchor·prior 계약부터 검증
   - metric map/PnP publication은 finite-positive depth·cheirality·all-view reprojection·관측가능성 gate 유지. solved/positive rho만으로 metric certainty 판정 금지
   - 상용 내부가 같은 factor를 사용한다는 증거 없음. 설계 근거는 [기존 공개 문헌·수식 분석](2026-10-12-rotation-recovery-followup.md)
4. **출력 예측은 별도 기능으로 검증**
   - 향후 render prediction을 넣으면 corrected image timestamp·prediction target timestamp·horizon·상태를 함께 출력
   - 예측 pose를 independent GT와 그 target 시각에서 비교하고 corrected ATE/RPE·fresh coverage를 보존. pose_age 감소만으로 estimator 정확도/속도 개선 판정 금지
5. **회복과 재초기화를 별도 평가**
   - same-map recovery success·time-to-recover·false recovery·reset epoch·warmup gap을 기록
   - map relocalization은 기존 landmark descriptor/geometry 검증과 좌표 epoch 계약이 필요한 별도 기능; 유효 pose extrapolation만으로 구현 완료 판정 금지
6. **수락은 자체 데이터로 결정**
   - 같은 image bytes·IMU/config/seed/thread/source/artifact에서 기존 room1/room4·fast rotation·churn·stationary/weak-translation controls 비교
   - one-to-one timestamp·body-frame·SE(3) scale1 translation/rotation ATE·RPE·coverage, Lost/reset/max-gap을 함께 비교
   - 속도는 frontend/backend/capture/copy/queue/render의 p50/p95/p99, 동일 입력·직렬 ABBA·초기화 전후 분리. 상용 문서는 성능 수락 근거를 대신하지 않음

## 라이선스·검증 경계

- 8th Wall framework: MIT, [고정 LICENSE](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/LICENSE). 개별 third-party 파일·모델·데이터 권리 별도. 이번 source 분석에서 제품 코드 복사·새 의존성0
- 8th Wall binary: [고정 LICENSE §1.2](https://github.com/8thwall/engine/blob/e1421ae742a9c5d654871367c64c1de3ad044caf/LICENSE)에 역공학·파생·경쟁 제품 관련 제한. §1.2(iv)는 Software 또는 해당 documentation을 경쟁 제품 개발·개선에 사용하는 행위도 제한하므로 engine 문서를 framework MIT 범위로 간주하지 않음. 이번 10월13일 연구 lane에서는 binary 추가 취득·내용 분석·실행 없음; 기존 10월12일 분석 산출물은 `build/research-20261012/8thwall-binary/`에 보존
- Google docs: 원문 표기 CC BY4.0, sample Apache2.0. SDK/service binary는 별도 약관. ARCore proprietary 내부 분석 없음
- Apple docs: Copyright Apple, 공개 문서·API contract 조사. ARKit estimator source·binary 분석/실행 없음
- 이번 완료 범위: 공식 문서 재조회·공개 MIT source5개 구조 분석·원문 해시·조건부 개발 방향. 새 phone ARCore/ARKit/8th Wall 실행·상용 속도 비교·실기기 GT·physical sensor calibration·독립 정확도는 미검증
- source manifest의 SHA는 이번 획득 byte 확인이며 upstream 서명·동일 runtime 빌드 증거 아님

## 재현

```bash
python3 build/tracking-performance-20261013/research/fetch-primary.py
git diff --check -- docs/research/2026-10-13-commercial-tracking.md
```

- 재획득 시 live 문서·`main` metadata가 바뀔 수 있음. source URL은 위 SHA 고정; 기존 manifest 비교 후 새 snapshot으로 구분
