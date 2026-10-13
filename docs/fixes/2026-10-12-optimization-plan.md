# Flip 6 회전·IMU·처리 지연 수정 개발 플랜

- 날짜: 2026-10-12, Asia/Seoul
- 기준: `98b396c`, 제품 source86개가 활성 manifest와 일치. 기존 untracked 측정 보고서 보존
- 증상: 실제30초 480결과/fresh447, Lost33(visual27, IMU6), 실효16fps, engine p95 62.985ms, 수신 pose age p95 117.896ms
- 보존: NAS 읽기 전용, 원본 전화 숫자·IMU/과거 영상, TLS, 현재 direct 서버, baseline JS/WASM/native 및 실패 후보
- 실행 모드: root가 IMU·engine·통합·빌드·검증 소유, 독립 tracker 및 web 구현 lane. 제품 dependency 추가 없음
- baseline: `build/optimize-20261012/baseline/`. 새 candidate·실패 결과는 별도 경로, 기본 build jobs총2

## 결함 범위·가설

- 확정: 빠른 회전에서 유효 current visual factors가 부족. detector count와 factor count는 별개
- 확정: bracket deadline3건, 한 영상 구간 내 실제 IMU sample0인 조건에서 invalid interval3건. 단일 적분 covariance rank 부족 guard는 유지해야 함
- 미확정: 휴대폰 영상 correspondence/blur/rolling shutter/실측 보정 원인. 새 영상이 없어 기존 phone numeric으로 광학 정확도 재생 불가
- 확정: rVFC packet은 이미 소유 snapshot이며 전송 wrapper가 추가 copy. 소유권을 명시할 때만 copy 생략 가능
- 가설: current factor 손실을 줄이는 선택적 회전 correspondence 검증과 frontend/backend tail 비용 개선. 기존 ON 후보의 TUM 회귀는 실패 기록으로 유지

## 구현 순서·파일

1. 현재 source/dependency/artifact fingerprint 및 C++/JS baseline 회귀 확보
2. tracker lane: `src/frontend/feature_tracker.cpp`, `include/frontend/feature_tracker.h`, `tests/test_engine_contracts.cpp`의 tracker 부분. 잘못된 ID 대응을 배제하는 독립 회귀→선택적 예측 후보. 원래 LK/F 경로와 품질 기준 보존
3. web lane: `web/js/app.js`, `web/js/vio-wrapper.js`, 관련 browser tests. owned image만 transfer하고 reusable view는 복사, 정확한 pixels/timestamp·epoch·geometry 회귀. render 불필요한 호출은 동작 계약 확보 후 축소
4. root IMU lane: `include/backend/factor/{integration_base,imu_factor,covariance_whitening}.h`, optimizer/engine/Worker 필요 범위. 단일 구간 수치 문제를 독립 입력으로 재현하고 물리·수치 근거 있는 수정만 적용. 가짜 sample·epsilon covariance·guard 우회 금지
5. root 계측/도구: frontend/backend/factor count와 IMU interval 실패를 같은 frame에 관측. `scripts/dev/`의 재시험 도구·화면 타이머 및 기록 provenance 보강
6. 통합 native/WASM·실제 App synthetic·GT 비교·독립 리뷰. 통과한 후보만 Flip6 재시험용으로 준비, source/artifact/served hash 연결

## 수락 조건

- 기본 control full TUM room1/room4: body frame, 일대일 timestamp, 단일 SE(3) scale1, 위치/회전 RMSE 각각 baseline 대비 악화≤5%, pose coverage 감소≤1percentage point
- 독립 fast-motion: seed101/707/2203의 고정 영상·analytic IMU·withheld GT, 기존 cap 유지. 새 확인 seed는 알고리즘 동결 후 생성. 전체분모·warmup분모·reset·ATE/APE/RPE 모두 기록
- IMU: 0/비유한/역순/미coverage/비SPD 입력은 기존 거부. 실제 하나의 양수 구간 수정에는 analytic covariance/residual/Jacobian 및 source endpoint 검증 필요
- 웹: zero-copy에서 packet 소유권·view 경계·실패·busy·기본 reusable buffer 보호·callback pixel/time·epoch/geometry를 검증. stale pose를 성공으로 표시하지 않음
- 성능: 같은 입력·seed/threads·설정, frontend/backend 및 Worker/copy/render p50/p95/p99. serial 비교·반복, synthetic 가상 clock은 성능 숫자에서 제외
- native 전체13suite(실제 discover 수 확인), browser contracts TODO/skip0, JS/Python/shell syntax, 해당 C++ clangd/ASan·UBSan, fresh WASM binding/실제 Worker 실행
- Flip6 목표: 같은 동작에서 manual reinit0, reset/Lost 감소 및 fresh coverage≥99.9% 지향, p95 age/처리 간격 개선. 실기기 재시험 전 달성 주장 금지

## 실패 조건·중단 기준

- GT 회귀·오대응 증가·coverage 감소·수치 유효성 훼손 시 후보 제외. 기준 완화·clamp/smoothing·pose 숨김으로 통과시키지 않음
- 데이터·서비스·무관 변경 보존. 구현 완료 전 phone 반복시험을 자동 시작하지 않음
- 전화 재시험은 사용자 동작과 화면 타이머를 맞춰 새 image/IMU/상태·stage timing을 기록. 독립 GT 없는 phone 숫자를 ATE 또는 production 정확도로 확대하지 않음

## IMU 수치 후보의 모델·검증

- 끝점 world acceleration `u0,u1`, 구간 `h`: `Δv=h(u0+u1)/2`, `Δp=h²(u0/3+u1/6)`. affine world acceleration의 시간모멘트 적분
- legacy 독립 끝점 가속도 noise 분산 `σ²` 유지: `Qpp=5σ²h⁴/36`, `Qpv=σ²h³/4`, `Qvv=σ²h²/2`, 조건부 위치분산 `σ²h⁴/72>0`
- 단일 양수 구간에 새 sample·epsilon 없이 full-rank 공분산 구성. whitening의 finite/symmetric/positive pivot 검사는 유지
- 실제 normalized quaternion `(1,φ/2)`의 right derivative `C=(I−[φ]/2)/(1+||φ||²/4)`, `φ=ωh`. F/V의 position·velocity·rotation 행을 실제 mean과 함께 맞춤
- root 독립 affine 해석값·endpoint noise perturbation covariance·bias repropagation Jacobian 회귀4개는 기존에서 실패→후보에서 통과. 기존 constant acceleration/repropagation 포함7개 통과
- 실제 보간된 두 끝점은 같은 raw bracket을 공유할 수 있어 noise 상관이 존재. 후보는 기존 독립 끝점 noise 모델을 유지한 수치 수정이며 raw sensor posterior covariance의 완전 모델링 주장은 제외
- 회전 중 world acceleration은 끝점 chord 근사. 전체 TUM/fast-motion GT 수락 전 후보 적용 판정 보류
- bracket 대기는 최근 유효 source 간격16개의 p95로 `clamp(2×p95,40,80)ms` 후보. 실제 기록 p95 51.86/max72.77ms와 고정40ms의 충돌을 대상으로 하며 실제 미래 sample 수락 조건 유지. 드문 frame의 대기 지연 증가를 함께 기록
- 더 낮은 IMU 주파수 수치 stress는 원본 clean101의 영상/GT를 유지하고 analytic noiseless IMU만10Hz로 subsample한 동결 fixture. 실제 phone sampling-rate 모사로 해석하지 않음

## 프레임 제출 스케줄 최적화

- IMU microbenchmark는 직렬14반복 중앙값83.179→86.234ms(+3.7%)로 CPU 단독 속도 향상 아님. 정확한 적분과 Jacobian의 추가 비용을 숨기지 않음
- root 통합 source의 자연 rAF3회는 GT gate 통과·reset0이나 고정 warmup coverage98.0/97.67/99.67%. `pre-scheduler/`에 당시 App/Wrapper/Worker hash와 실행 근거 동결
- 수정 파일: `web/js/app.js`, `web/js/vio-wrapper.js`, video-frame/transport 회귀. 캡처 준비 및 Worker 완료 이벤트에서 기존 소유 최신 packet을 직접 제출하여 다음 rAF 대기 제거
- 기존33/50ms pacing·warmup·single in-flight·IMU flush·pixels/time·geometry/epoch·stop/blur/config fences 유지. 렌더/UI는 기존 rAF 경로 유지, 추가 rAF loop 생성 금지
- 독립 fake clock으로 제출시각 개선과 lifecycle/reentrancy 음성 회귀, 실제 App 자연 rAF source/hash·GT 및 같은 C++ loader의 직렬3회 before/after 비교
- 참고: [OpenVINS inertial integration](https://docs.openvins.com/propagation.html), [Forster et al. on-manifold preintegration](https://arxiv.org/abs/1512.02363). 후보의 시간모멘트 가중치·공분산 해석식은 위 명시 모델에서 직접 유도·독립 검증
