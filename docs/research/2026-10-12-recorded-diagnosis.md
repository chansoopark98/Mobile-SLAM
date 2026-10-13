# 기존 Flip6 기록 재생·현재 영상 제약 진단

- 날짜: 2026-10-12, Asia/Seoul
- 판정: **기존 영상 prefix에서 current factor 소실·최종 cheirality 거부 재현. 실기기 원인 확정·물리 정확도 검증은 미완료**
- 범위: 기존 기록 읽기, 동결 binary/loader 실행, 개발용 합성 fixture 생성. 제품 source·서비스·원본 기록 변경 없음
- 근거: `build/research-20261012/replay-diagnosis/{data-audit,replay-summary,input-derivation}.json`

## 동결 기준

- HEAD `98b396c29ce9f6994b738190094e6397a2c6ace8`; 실제 기준은 최적화 후 dirty source. HEAD 자체의 빌드로 해석 금지
- 동결 source: `build/research-20261012/baseline/{source.tar.gz,manifest.json}`
- native: 같은 폴더 `native_replay`, SHA256 `43418010831c8b8747bf1e5c25ca88bc9815eb58132fb396c8449d27159bad7c`
- embedded WASM loader: 같은 폴더 `vio_engine.js`, SHA256 `140e29a8d0717daca8b330d50849671ddb13403e0e82ccb9f8f221a764bf1595`
- browser 재생 도구·App/Wrapper/Worker도 source archive에서 `replay-diagnosis/frozen-browser/`로 추출. 제품 변경과 독립된 기준 실행
- 실행: 원래 double timestamp, seed0, OpenCV1thread, feature120, LK21/3·20iteration/0.03, min_dist30, solver0.06초/10iteration, PnP OFF
- 동일 source라도 native CSV의 정수 nanosecond 변환·플랫폼 수치·진단 capture 차이가 존재. browser exact-double을 원본 시각 재생의 우선 근거로 사용

## 재생 가능한 범위

| 기록 | 실제 보존 | 재생·판정 범위 |
|---|---|---|
| 2026-10-12 05:40:50 numeric | 결과480, IMU1249, 영상0 | 기존 Lost33: 영상27·IMU6. 영상 correspondence 재생 불가 |
| 2026-10-12 11:59:15 numeric | 결과609, IMU1277, 영상0 | Lost33 전부 current factor0/1. 영상·초기 window·GT 없음 |
| 2026-10-07 minute1 | 첫300/558accepted, 32.06778초, IMU2239 | 저장 prefix와 과거/미래 bracket 검증. pre-axis-fix gyro 원본·명시한 수정 derivative 구분 |
| 2026-10-07 minute2 | 첫300/847accepted, 18.733525초, IMU2761 | 저장 prefix만 재생. 최초 fresh41.116735초가 영상 보존 범위 밖 |

- 원본 archive: `/home/park-ubuntu/Documents/Mobile-SLAM-recordings/2026-10-07-ZFlip6/records/`
- 직접 확인: PGM600개 payload hash 일치, frame index 연속·timestamp 단조 증가·IMU 유한/단조 증가·prefix 양 끝 bracket·recorder overflow0
- 최대 source IMU 간격: minute1 163.12ms, minute2 66.815ms. source 간격 측정이며 완전한 하드웨어 sampling 기록 보증 없음
- `completed=true`는 원래 recorder 종료 상태. minute1 uncopied258, minute2 uncopied547개의 후반 영상은 복구 불가
- numeric에는 입력 영상이 없으므로 전체 sensor replay 또는 최신 회전 Lost 재생으로 보고 금지
- 원래 estimator의 기록 전 상태 없음. replay는 새 lifecycle·직렬 입력·미리 제출한 미래 IMU bracket 사용. 원래 arrival·drop·정지/재시작·실기기 속도 재현 제외
- heuristic K/extrinsic, EIS/OIS·노출/IMU offset·independent GT 미검증. phone derivative의 위치 norm 감소는 정확도 근거 제외

## 최신 numeric 독립 재계산

- 공정 시간창 `startedAtMs <= inputTimestamp×1000 < stoppedAtMs`: 607결과/574fresh. 전체 recorder609/576과 분모 구분
- Lost33개: current factor0 29개·1 4개, tracker 중앙값53, eligible bank80, immature146
- Lost 모든 행: IMU factor10, rejected IMU0, invalid depth0, projection reject0. Ceres/marginalization 미실행
- 가장 긴 Lost 구간: 결과277–291, recorder13.07401–13.60605초, 15개 연속 current factor0. 주변 제출 gyro norm 중앙값85.20°/s
- 최적화 후 quality warning159개: `tracking:acc_bias_threshold`. finite/fresh를 물리 정확도 통과로 해석 금지
- 결과406→407: 위치1.072161m/37.545ms, 회전25.90324°, 같은 기록 gyro norm 적분 상계2.97094°. 결과 간 자기 일관성 이상이며 GT 오차는 아님
- 결과296→297: 회전44.69679°/22.785ms, 기록 gyro 적분 상계2.21552°, current factor9. 단순히 current factor2개 이상인 것만으로 pose 일관성 보장 불가
- gyro 비교는 source bracket 안에서 선형 보간·norm의 사다리꼴 적분 상계를 계산. 기록 바깥은 null로 거부; 물리 exposure 동기 증명 제외
- 시간창·Lost별 factor·gyro·pose 쌍: `data-audit.json`, `replay-summary.json`. 기존 after-optimization 문서와 독립 계산 일치

## 동결 browser 재생

| 입력 | fresh/300 | 초기화 첫 frame | current/final 부족 | guard reset | 관측 최대 인접 위치 변화 |
|---|---:|---:|---:|---:|---:|
| minute1 원본, rotation OFF | 239 | 28 | 20 / 1 | 1 | 3.786019m |
| minute2 원본, rotation OFF | 0 | 없음 | 0 / 0 | 0 | 없음 |
| minute1 축 수정 derivative, 현재 ON | 270 | 28 | 1 / 1 | 0 | 2.451924m |
| minute2 현재 ON derivative | 0 | 없음 | 0 / 0 | 0 | 없음 |

- 네 실행 모두300개 결과 완료·worker transport overflow/stale/capacity/invalid/order0·missing bracket0
- minute2 두 실행 모두 initialization_timeout1. 초기화 실패로 tracking 문제가 검증된 것은 아님
- minute1 derivative: body gyro `(-old_y,-old_z,old_x)`, `rotationRescue=true`만 변경; pixels·시각·가속도·K/extrinsic·noise·solver/features 불변
- 축 보정은 고정 Chrome154/표준의 코드 결함에 대한 명시적 변환. 새 물리 측정 또는 실측 calibration으로 해석 금지
- 원본/derivative hash 및 변경 목록: `input-derivation.json`. 원본 PGMs·record.json 그대로 보존
- 두 요소를 함께 바꾼 minute1 전후값은 rotation ON의 독립 개선량으로 사용할 수 없음
- 최대 위치 변화·bias warning이 남음. reset0·90% 전체 coverage는 phone 정확도 수락 조건이 아님
- desktop 실행 시간은 보존했으나 phone latency·전후 성능 개선 근거로 사용하지 않음

## source와 실제 membership 대조

- `src/vio_engine.cpp`: tracker age1은 `image_data`에 넣지 않음. 검출 수와 backend 전달 수부터 다름
- `src/frontend/feature_manager.cpp`: 기본 eligibility `used_num>=2 && start_frame<WINDOW_SIZE-2`; window10에서는 anchor index0–7만 수락
- `src/backend/optimizer.cpp`: eligible는 역사 bank 전체. 현재 영상 factor는 해당 track의 `imu_j==10`인 경우만 증가
- `src/backend/estimator.cpp::solveOdometry`: triangulate → removeFailures → optimize. 삼각측량에서 제거된 track은 optimizer invalidDepth/projectionReject 계측에 포함되지 않음
- `FeatureManager::triangulateAcrossAllViews`: 유한·양수 depth를 얻지 못하면 solve_flag2로 삭제. 양수 depth는 별도 초기 triangulation parallax/conditioning 품질 검사 없이 유지
- reject update도 `slideWindow()`를 수행. optimizer는 이전 slot prior를 release. 연결 잃음→성숙 대기→prior 소실의 연쇄 가능

### mobile 조건을 맞춘 native auxiliary

- `native-minute1-mobile-match-on/`: 270/300fresh, 최초28, current 부족1·final 부족1. exact-double browser와 frame outcome·reason count 동일
- frame82: tracker120개 중 age>1 25개, age1 95개. eligible96개 전부 과거 track, current0. margin OLD, invalidDepth/projectionReject0
- 해당25개 현재 ID는 eligible membership에 없음. 기존 capture로는 young maturity 제외와 삼각측량 삭제를 분리할 수 없음
- frame98: NEW margin, solve 전 current residual67개이나 solve 후 `insufficient_final_current_features`. 초기 factor 부족과 최종 cheirality 부족은 서로 다른 실패
- 최초 auxiliary `native-minute1-axis-fixed-on/`은 YAML 기본 LK30/0.01로261/300. 비일치 조건 실패 기록으로 보존하고 parity 비교에서 제외
- 전체 native detailed membership: 각 `results.json`; 요약 `replay-summary.json`. pre-solve membership과 최종 current-factor count 구분

## 개발 가설·검증 조건

- 우선 가설: 현재 살아 있는 young track이 geometry를 제공해도 anchor index gate로 제외되어 회전 중 current factor 부족을 연장
- 개선 범위: 삼각측량 geometry 검증과 eligibility를 모든 depth vector·optimizer·final filter·marginalization·export·PnP consumer에서 일치시켜 검토
- 별도 회귀: mature current1개 + young20개 이상 + low parallax + 반복 NEW에서 young anchor9가 남는 성숙 정체. all-old-ID 소실만 있는 fixture는 parallax_num0→OLD fallback이 있어 영구 정체 증명에 부적절
- OLD/NEW source 정책·관측 ray와 depth reanchor·parameter index/prior 주소 일관성은 같은 회귀로 확인
- 기존 prefix의 pre-solve current 부족은 OLD에서도 재현됨. NEW 정체가 최신 phone Lost의 확정 원인이라는 주장 제외
- PnP는 실제 기록 OFF. 활성화하면 현재6개 3D 대응을 요구하므로2개 gate의 간단한 대체 경로가 아님
- 금지: current threshold 완화, stale pose 공개, smoothing, depth clamp/fake prior, solver cap 증가로 성공 처리
- 필요 gate: 독립 analytic young-track geometry·pure rotation/low baseline 음성 회귀, full TUM room1/room4, 기존 fast101/707/2203/8809, 개발 churn·정책 동결 후 새 확인 seed

## 개발용 churn fixture

- 위치: `replay-diagnosis/fast12617-{development,churn-development}/`
- generator는 동결 source 그대로 import. seed12617, fast-rotation portrait,400영상/1201IMU,20초. 개발 입력이며 새 held-out 확인 입력 아님
- 파생48frame: `[6,6.8)`, `[10,10.8)`, `[14,14.8)`초에 영상 폭60%를 값16으로 가림. 나머지 pixels·영상 timestamp·IMU·analytic GT 불변
- occlusion은 feature ID 교체·회복 검증을 위한 제어 입력. 실제 Flip6 blur/rolling shutter/EIS 모델이나 속도 수락 기준으로 해석 금지
- GT는 estimator에 전달하지 않음. body frame·일대일 timestamp·단일 SE3 scale1·ATE/APE/RPE·전체400/warmup300분모 평가
- 동일 caps: solver0.06초/10iteration, feature120, LK21/3·20/0.03, min_dist30, rotation ON
- 기본 generator YAML의 legacy min_dist20/solver10초는 보존하고 명시적 `mobile-match.yaml` 사용. runtime generator 변경 없음
- 생성/원본/파생 manifest·SHA256·수정 frame index를 보존. 새로운 확인 seed는 정책 동결 후 선택
- frozen native 기준: 원본359/400, warmup300/300, reset0, ATE0.030876m/APE0.639329°, GT gate 통과
- 같은 caps의 occlusion 파생: 324/400, warmup265/300(88.333%), reset0, Lost35(current33/final2), bias warning146
- 파생 SE3 scale1: **ATE49.835073m, APE123.353636°, 1초 RPE29.432040m/50.240926°; GT gate 실패 보존**
- 파생35개 factor 실패 모두 OLD margin. frame127은 tracker79/age>1 18/eligible143이지만 current0. NEW 정체만으로 본 회귀를 설명할 수 없음
- 결과: `native-fast12617-{development,churn-development}-on/`, `fast12617-{development,churn-development}-evaluation.json`
- evaluator는 동결 archive의 `evaluate-synthetic-vio.py` 사용. archive에 없던 `scripts/evaluation/audit_metrics.py`는 현재 workspace에서 읽기 전용 복사·SHA256 `80414ca3fb75d1f9556aa0d689be2e8a52f0f7a969c69f395720e03132ee79f1` 기록
- 위 실패는 개발 회귀의 기준. 같은 fixture로 반복 최적화한 후보를 held-out 확인으로 보고 금지

## 재현 명령

```bash
# 원본 double timestamp를 사용하는 browser 기준. minute2는 record/input-dir를 함께 교체
node build/research-20261012/replay-diagnosis/frozen-browser/scripts/dev/replay-android-input.mjs \
  --record /home/park-ubuntu/Documents/Mobile-SLAM-recordings/2026-10-07-ZFlip6/records/controlled-minute-1/record.json \
  --input-dir /home/park-ubuntu/Documents/Mobile-SLAM-recordings/2026-10-07-ZFlip6/records/controlled-minute-1/mav0/cam0/data \
  --loader build/research-20261012/baseline/vio_engine.js \
  --output build/research-20261012/replay-diagnosis/browser-minute1-recorded-off-repeat \
  --frames 300 --timeout-ms 180000

# 명시한 축 수정 + 현재 ON derivative. 입력 생성 근거: input-derivation.json
node build/research-20261012/replay-diagnosis/frozen-browser/scripts/dev/replay-android-input.mjs \
  --record build/research-20261012/replay-diagnosis/input-records/minute1-current-on.json \
  --input-dir /home/park-ubuntu/Documents/Mobile-SLAM-recordings/2026-10-07-ZFlip6/records/controlled-minute-1/mav0/cam0/data \
  --loader build/research-20261012/baseline/vio_engine.js \
  --output build/research-20261012/replay-diagnosis/browser-minute1-axis-fixed-on-repeat \
  --frames 300 --timeout-ms 180000

# CSV ns 시각을 사용하는 native 보조 진단; 제품 binary 재빌드 없음
build/research-20261012/baseline/native_replay \
  build/research-20261012/replay-diagnosis/minute1-axis-fixed \
  build/research-20261012/replay-diagnosis/minute1-axis-fixed/mobile-match.yaml \
  build/research-20261012/replay-diagnosis/native-minute1-mobile-match-on-repeat \
  300 bracket 0 1 10 0.06 0 1 0 300 yaml 1

python3 build/research-20261012/replay-diagnosis/audit-records.py
python3 build/research-20261012/replay-diagnosis/summarize-replays.py

# 개발 churn 비교: 동일 caps·동일 입력, binary 경로만 교체
build/research-20261012/baseline/native_replay \
  build/research-20261012/replay-diagnosis/fast12617-churn-development \
  build/research-20261012/replay-diagnosis/fast12617-churn-development/mobile-match.yaml \
  build/research-20261012/replay-diagnosis/native-fast12617-churn-development-on-repeat \
  400 bracket 0 1 10 0.06 0 1 90 330 yaml 1
python3 build/research-20261012/replay-diagnosis/frozen-browser/scripts/dev/evaluate-synthetic-vio.py \
  --dataset build/research-20261012/replay-diagnosis/fast12617-churn-development \
  --results build/research-20261012/replay-diagnosis/native-fast12617-churn-development-on-repeat/results.json \
  --estimate-frame camera \
  --output build/research-20261012/replay-diagnosis/fast12617-churn-development-repeat-evaluation.json
```

- output은 새 이름만 허용. 원본 dataset·기존 결과 덮어쓰기 금지
- 재계산 script는 개발 증거 디렉터리만 갱신. 제품 테스트·physical acceptance를 대체하지 않음
