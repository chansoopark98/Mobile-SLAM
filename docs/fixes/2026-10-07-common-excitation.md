# 공통 좌표계의 초기화 excitation

- 날짜: 2026-10-07
- 소유 파일: `src/frontend/initialization/initializer.cpp`, `tests/test_numerical_contracts.cpp`, 이 문서; 필요 없는 header 변경 제외
- baseline: 현재 native CTest10개·NumericalContracts41개 통과
- 변경 전 계획: 독립 물리fixture red → 기존check를SfM 후·metric alignment 전에 이동 → interval 시작body→공통world 변환 → 같은0.25threshold·sample std 검사 → affected target green
- 보존: 기존 gyro bias 정렬·SVD·noise·velocity·scale·모바일 공개guard·WASM·데이터·다른 lane 변경

## 좌표·단계

- `pre_integration_j.delta_v / dt_ij`: interval 시작frame_i의 body 좌표
- SfM 이후 `frame_i.R`: body_i→공통SfMworld 회전
- 비교 벡터: `frame_i.R * (delta_v_ij / dt_ij)`; currentframe_j.R 사용 제외
- 표준편차: `sqrt(sum(||a_i - mean(a)||²)/(N-1))`; 기존0.25m/s² 유지
- 순서: SfM → 공통frame excitation → visualInitialAlign/gyro bias → metric scale
- 검사: finite timestamp·delta_v·SO(3), positive dt·영상interval과IMU dt 차이≤기존1e-6s, finite mean/std
- debug안내: 회전량 증가 대신 translation acceleration 변화

## 독립 기대값

- pure rotation·constant world velocity: specific force의 world 평균은 gravity로 일정; common std≈0, 거부
- rotating gravity의 body std는0.25초과 가능; 기존 서로 다른body frame 비교의 false pass
- alternating0.05/0.30s interval·0.3rad/s 회전: 잘못된 currentframe 회전 사용시 std≈0.388>0.25; 시작frame anchor 검사
- `a_x(t)=0.5cos(3t)`·11frame·0.13s: interval 평균의 common std0.382763m/s², 수락; 전체orientation에 무관

## 관측·한계

- `controlled-minute-1/record.json` correctedgyro적분의 마지막11consecutive frame proxy:547windows,body통과211중115common미달; 최초300image구간122통과중59미달
- 실제SfM retainedhistory·회전은 기록되지 않음; proxy로 실제새gate 동작·정확도 확정 제외
- preBias 검사: gyro bias 보정으로 delta_v가 재계산되기 전; 큰미확인gyro bias·시간동기·SfM rotation 오차에 의해 false rejection/acceptance 가능
- preBias analytic counterexample: true정지·SfM R=I·gravity와직교하는gyro bias0.35rad/s·interval0.05/0.50s교대 →common std0.406848m/s²로falsepass가능. bias0.14→0.162845,0.01→0.011633; 실제phone bias 가정 제외, postBias/rollback은 이번범위 제외
- metric alignment 이전의 준비조건; numerical rank·scale observability·field GT 정확도의 충분조건 아님
- 새gate 때문에 pose0·반복초기화만 남는 결과를 해결로 보고 제외; same-input replay·실기기coverage/reset/drift owner 판정 필요

## 검증

- `cmake --build build/mobile-divergence-native --target test_numerical_contracts --parallel 2`
- old core red·affected green: `build/dev-android/common-excitation-*.log`
- VERIFIED red: 기존gate로3개회귀 모두실패. pure rotation·constantvelocity를수락,NaN·dt0·reflection을정상excitation으로수락. `common-excitation-red.log`, `common-excitation-red-build.log`
- VERIFIED green: 기존41개 + 새물리fixture2개, **NumericalContracts43/43 PASS**. 시작frame anchor·varying translation·전체orientation 불변성·invalid interval 검사 포함. `common-excitation-green.log`, `common-excitation-green-build.log`
- VERIFIED CTest: NumericalContractsTest1/1suite PASS. `common-excitation-ctest.log`
- VERIFIED genuine clangd23.1.0: 현재native compile DB로initializer source probe, source diagnostic error0/warning0. `common-excitation-clangd.json`
- 환경: GCC11.4.0, C++17, 기존Eigen/Ceres, 실제compile DB `-O2 -DNDEBUG`; 새의존성 없음
- initializer SHA-256: `c71c1a7cf5feddfa5b334b9653b327b4a28794eaa36ea65b2374515175e84f45`
- test SHA-256: `c7f43ef8e0743a4c79a2f722f0c486e17404777a5a05517d7cd8f0a382bfc439`
- freshNumerical binary SHA-256: `c8bceae489919174621af2ec034939896c53f42d26d619b30b83f6d4fd4f2cf2`
- 정리: body frame 벡터의중복sum/variance 순회·부분invalid interval 묵인 제거. 공통frame 벡터를1회구성,동일sample std 적용
- UNVERIFIED: fullnative/WASM·actualsame-input SfM gate값·실기기fresh coverage/reset/drift는 leader 후속검증. 포즈0·반복초기화만으로발산해결 보고 제외
