# Flip6 tracking 회귀 원인 분리·수정 계약

- 시작 revision: `d8090cc`, clean `codex/flip6-analysis-20261013`.
- 목표: 같은 기록의 active239/300→candidate53/300 감소 원인 분리, 실제 추적 로직 수정, 독립 GT 정확도와 추적 안정성의 동시 검증.
- 실행 결과: [원인 분리·C3 opt-in·GT/안정성 판정](2026-10-13-flip6-tracking-recovery-results.md). 아래 수락 기준은 변경하지 않음; C3는 원본 안정성 gate 실패로 기본 OFF.
- 기존 데이터·active WASM·실패 receipt·TLS·stopped 서비스 보존. 새 실험은 `build/flip6-recovery-20261013/`, compile jobs 총2.
- 현재 baseline: 기존 Worker replay 두 loader/300frame, frozen A0/A1/A2 source와 binary, current source/dependency manifest. 원본 gyro/time/K를 바꾸지 않는 재생이 1차 control.
- raw 기록의 과거 gyro 변환 오류를 보정한 별도 input은 sensitivity control로만 사용; 원본을 수정하거나 같은 입력으로 집계하지 않음.

## 순서

1. 입력·RPC profile·loader·Wrapper/Worker·IMU batch hash 확인 → 최초 수치/상태/current factor 차이 및 reset 전후 분리.
2. 보존 A0/A1/A2 native prefix로 변화 단계 구분. Native ns-quantized 입력을 browser exact-double 재생과 동일하다고 가정하지 않음. 선택된 원인은 같은 browser 경로로 재확인.
3. 깊이 admission/초기화/anchor lifecycle 변화의 최소 ablation. 임계값 완화·pending 무조건 해제·허위 depth·fresh 재분류는 수리 후보에서 제외.
4. 상용 API의 관측/지도 수명 분리와 공개 VIO의 회전/병진 관측가능성을 근거로 원인에 맞는 작은 로직 변경. 수식·부호·slot 수명은 독립 fixture에서 먼저 검증.
5. recorded raw/axis-corrected control·synthetic held-out·full TUM room1/room4 ON/OFF → native/WASM/JS 회귀·독립 review → docs·commit.

## 수락

- 같은 Flip6 원본300frame 분모, fresh/initialized coverage·current/final factor 부족·Lost 구간·reset·pose jump·경고를 모두 기록. fresh 증가만으로 수락하지 않음; 기본후보53/300보다 회복하고 reset/불연속도 악화하지 않아야 함. active239의 bias 경고128개·3.786m jump를 성공 기준으로 복사하지 않음.
- minute2 저장prefix는 초기화 전 구간. pose0은 UNKNOWN으로 유지하며 정확도 통과로 세지 않음.
- 합성 GT: body frame·일대일 timestamp·SE(3) scale1, ATE≤0.25m·rotation APE≤5°·1s RPE≤0.10m/3°, 기존 coverage/warmup 기준 유지. 기존 churn 실패를 성공으로 소급 변경하지 않음.
- TUM: 같은 profile의 baseline 대비 translation/rotation RMSE≤1.05, coverage 손실≤1%p. 전체 sequence·ON/OFF를 각각 보고.
- finite/positive/cheirality·current support≥2·IMU bracket·epoch·lost/fresh 계약 유지. 공개 engine의 threshold를 새로 복사하거나 시험 결과에 맞춰 기준 변경하지 않음.
- Native·ASan/UBSan·browser 계약·WASM public API와 실제 Worker 재생 통과. 새 source/artifact hash 기록; 검증 실패 candidate는 active로 승격하지 않음.
- 실제 신규 Flip6·physical GT·thermal은 현재 기기/입력 부재로 별도 미검증. 원본과 desktop 실행의 한계를 최종 보고.
