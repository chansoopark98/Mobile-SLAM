# 회전 추적·웹 입력 처리 수정 개발 플랜

- 날짜: 2026-10-10, Asia/Seoul
- 증상: Z Flip 6, 책상에 앉아 근거리·원거리 모두 조금 빠른 회전에서 Lost → 발산. 원거리 특징점 수 감소
- 요청 범위: 처리 속도, 웹 데이터 처리, IMU 수집·처리의 결함·개선 확인 및 수정
- 기준: `3177951`, clean working tree; source/artifact snapshot `build/motion-20261010/baseline/`
- 보존: 원본 전화 기록·dataset·기존 빌드·TLS·서비스 상태. 새 dependency·commit·서버 시작 없음

## 확인한 결함과 가설

- 확인: 정상 solve_flag=1 점도 기본 depth 2m ±1cm이면 map export에서 삭제. 현재 실패 depth=-1 계약과 불일치
- 확인: 현재 LK는 IMU 회전 예측 없이 이전 픽셀에서 탐색. 빠른 회전의 실제 Lost 지배 원인은 미확정
- 확인: Lost 한 frame도 renderer.clear()로 궤적을 지움. engineEpoch 변경과 화면 소실은 별개
- 확인: mobile guard는 추정 위치차 >5m 또는 동일 detector warning에 reset. 실제 회전 속도 제한 아님
- 조사: timestamp·sensor pairing·오래된 IMU 폐기·queue·capture/copy·map/renderer 비용. 재현 없는 threshold/noise 튜닝 제외

## 변경 파일·순서

1. baseline C++/JS 회귀, source/dependency/artifact fingerprint 및 독립 synthetic motion 조건 고정
2. `src/backend/estimator.cpp`, `tests/test_projection_trial_contracts.cpp`: 정상 2m/5m/10m export red 확보 → 낡은 sentinel 필터 삭제
3. `src/vio_engine.cpp`, `include/frontend/feature_tracker.h`, `src/frontend/feature_tracker.cpp`, `tests/test_engine_contracts.cpp`: 회전 영상의 독립 correspondence 기준으로 현재 LK 재현 → 같은 IMU endpoint에서 계산한 rotation-only seed 평가
4. 회전 seed는 초기화 이후, 이전 tracker 시각=IMU endpoint, 현재 IMU coverage 및 유효 SO(3)일 때만 사용. `R_Cnew_Cold=R_BCᵀ R_afterᵀ R_before R_BC`; 기존 camera lift/project로 왜곡 반영. 누락·reset·invalid에서는 기존 LK 유지
5. 웹·IMU 감사에서 재현되는 결함과 중복 작업만 파일 소유권 분리 후 수정. pixel/timestamp 소유·future bracket·epoch·새 pose 유효성 유지
6. native/WASM·실제 App synthetic 재생 및 독립 리뷰. 결과·실패·한계와 변경 파일 문서화

## 수락 조건·실패 조건

- 회전 영상: GT 기반 기존 ID correspondence 오차·정확한 잔존율 비교, 잘못된 대응을 잔존 성공으로 세지 않음. pinhole/fisheye·비항등 extrinsic·invalid/missing IMU fallback 검사
- 합성 입력: analytic IMU·rendered pixels·withheld GT, 회전 속도와 실제 frame 간격 기록. stationary/pure-rotation-only 초기화 음성 사례 유지
- 정상 control: 같은 입력·설정·seed/threads, 단일 SE(3) scale=1·회전 포함. baseline 대비 위치/회전 RMSE 5% 초과 악화 또는 coverage 1percentage point 초과 감소 시 candidate 재검토
- 속도: 같은 입력에서 engine/Worker/capture 비용 p50/p95/p99 분리. desktop 결과를 phone 성능으로 확대하지 않음
- CTest 전체 suite, browser contracts, syntax, 해당 C++ clangd 검사·WASM API/실제 Worker 실행; TODO/skip을 통과로 세지 않음
- 금지: guard 완화·pose clamp/smoothing·정확도 없는 finite pose를 해결 근거로 사용
- 실기기 검증 한계: 새 Z Flip 6 image/IMU 기록·실측 calibration·현장 GT가 없으면 실제 Lost/발산 해결 판정 보류

## 첫 후보의 기각·후속 검증

- 모든 유효 회전에 예측을 적용한 후보: fast-rotation101/707 수락 통과, TUM-VI room1/room4 기존 정확도 gate 실패
- room1 자세 APE 1.1633→1.5666°, room4 위치 ATE 0.04692→0.06637m·자세 APE 0.9183→1.4455°
- 실패 후보 binary·source·manifest는 `build/motion-20261010/all-motion-candidate/`, 원본 결과는 기존 경로에 보존
- 후속: LK window에서 유도한 큰 픽셀 변위에만 회전 seed 적용하는 후보 검증. 작은 회전은 기존 탐색 경로와 수치 동작 유지 여부 검사
- 동일 수락 기준 유지; dataset별 threshold·metric cap 조정 제외
- radius-only 후보는 회전·이동 상쇄 영상에서 오대응 발생, 기각. photometric/local reverse 후보는 영상 회귀 통과 후에도 full TUM gate 실패, 기각
- ordinary reverse 보호는 잘못된 성공 대응까지 보호하여 빠른 회전 회귀 실패, 기각. 각 source/binary/log 보존
- 후속 고정 정책: ordinary LK의 실제 F consensus가 원본 track의 과반이면 기존 결과 그대로 유지. weak/absent F에서만 큰 회전 후보 재검토; 기존 성공점 교체 시 patch L1 절반 미만, 모든 채택 후보에 local reverse 0.5px 조건 적용
- F 계산을 원본 index mask 반환과 최종 pruning으로 분리. 채택 없으면 첫 mask 재사용, 실제 채택이 있을 때만 병합 후 refit. 기존 RMS/최소30/threshold/RANSAC 확률·edge recovery 계약 유지

## 최종 적용 범위

- 기하학적 과반 gate 후보도 ON의 full TUM-VI 정확도 유지 gate 실패. 기존 cap을 완화하지 않음
- 기본 OFF로 격리하고 `rotation-rescue=1`에서만 실험적으로 활성화. native·Worker·App 모두 실제 설정값을 조회·기록하며, 요청 ON의 API/readback 누락은 거부
- 기본 OFF의 full room1/room4 회귀는 통과. ON의 고정 fast-rotation101/707 및 실제 App 합성 검증은 별도로 기록
- 질문에 응답이 없으면 기존 정확도 유지·실험 옵션 제공을 기본 판단으로 사용. 실기기 Lost/발산 해결·phone 속도 향상은 미확인
- 기본 적용: Worker busy snapshot/pacing 결함, Generic Sensor callback-order pairing 결함, 정상 2m 점 export 누락 수정. 실험 ON의 추가 처리 비용·정확도 회귀는 결과 보고서에 명시
