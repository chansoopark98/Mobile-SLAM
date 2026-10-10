# 초기 metric scale 정렬의 유효성

- 날짜: 2026-10-07
- 소유 파일: `src/frontend/initialization/initial_alignment.cpp`, `include/frontend/initialization/initial_alignment.h`, `tests/test_numerical_contracts.cpp`, 이 문서
- 근거·전체 수락 조건: `docs/fixes/2026-10-07-mobile-divergence-plan.md`
- 기존 baseline: 모든10개 native CTest 통과, `build/dev-android/native-baseline.log`; source fingerprint `build/dev-android/pre-alignment-source.json`
- 변경 전 계획: 기존 core로 독립 회귀 red → rectangular·column-scaled rank-revealing solve → strict-positive finite scale·refinement 실패 전파 → 새 isolated native target green
- 보존: gyro bias 수식·정렬의 물리 Jacobian·4회 gravity refinement·기존 regression 오차 bound; 임의 scale/velocity 상한·threshold 완화 추가 제외
- 출력 위치: `build/mobile-divergence-native/`, `build/dev-android/alignment*.log`; 기존 native/WASM 바이너리 덮어쓰기 제외

## 결함·독립 기대값

- 기존 정렬: `A=JᵀJ` normal equation의 LDLT solve; rank 검사 없음, column 단위 차이·condition 제곱
- constant world velocity + 회전: scale/velocity nullspace 존재; body gravity variance가 scale 관측 가능성을 보장하지 못함
- 독립11frame/0.13s·4.584°/s 회전 예: excitation 표준편차0.308838m/s²>기존0.25; 정렬J60×37 rank36·refinementJ60×36 rank35
- stationary·constant velocity의 비관측 scale: 정렬 성공 제외
- excited analytic motion: metric scale2.3·configured gravity·속도 회복; 기존 known-truth 회귀 유지
- gravity±Z: tangent basis `BᵀB=I₂`, `Bᵀg=0`; 기존 -Z에서 basis 두 열 모두0
- scale0·NaN·invalid interval/geometry·실패 refinement: 성공 반환 제외

## 구현 범위

- normal equation 누적 대신 기존6개 물리 equation을 rectangular matrix로 결합
- column norm으로 단위 정규화; 기존 Eigen SVD로 rank·solve, 새 의존성 없음
- 수치 rank 기준: `epsilon(double) × max(rows, cols) × largest singular value`; metric scale/속도 cap과 구분
- finite 입력·timestamp/preintegration interval·SO(3)·solution·least-squares stationarity 검사
- positive scale의 수치 해상도: column-scaled solve의 `epsilon × (rows+cols) × (||J||·||y|| + ||b|| + RHS 원래 operand norm 합)`을 최소singular value·scale column norm으로 환산. gravity subtraction으로 RHS가 거의0인 경우 cancellation 전 크기 보존; 고정 metric scale cutoff 제외
- gravity refinement 실패를 bool로 반환; 성공한 finite 결과만 caller 출력에 적용
- 양극 gravity의 안전한 tangent basis: gravity와 가장 덜 평행한 coordinate axis 사용
- strict-positive finite scale, 기존 gravity norm consistency 조건 유지

## 검증·한계

- `cmake --build build/mobile-divergence-native --target test_numerical_contracts --parallel 2`
- `build/mobile-divergence-native/test_numerical_contracts --gtest_filter='NumericalContracts.Alignment*:NumericalContracts.Gravity*'`
- 전체 NumericalContractsTest green 후 native/WASM/replay owner에게 소스 완료 전달
- machine rank 통과가 noise에 대한 충분한 metric scale 관측 가능성·실기기 정확도 보장 없음
- 실제 동일입력 replay·field GT·기기 calibration·coverage·reset·drift 판정은 leader 결과 필요

## 실행 결과

- 환경: GCC11.4.0, C++17, 실제compile DB의 `-O2 -DNDEBUG`, Eigen 기존설치, pinned Ceres2.2.0/GoogleTest1.14.0
- VERIFIED red: old core + 최초5개 독립/기존 회귀 중2개 실패. -Z tangent basis의 `||BᵀB-I||=sqrt(2)`, stationary+rotation에서 실제 `LinearAlignment=true` 반환. `build/dev-android/alignment-red.log`, 빌드 `alignment-red-build.log`
- VERIFIED green: 기존34개 + 신규7개, **NumericalContracts41/41 PASS**. scale2.3·±Zgravity·physical velocity, visual units1e±12, stationary/constant velocity rank 실패,11개 invalid/overflow 조건·실패시 caller g/x 보존,0/NaNgravity/refinement 거부 포함
- green 실행·빌드: `build/dev-android/alignment-green.log`, `alignment-green-build.log`
- VERIFIED CTest: `ctest --test-dir build/mobile-divergence-native -R '^NumericalContractsTest$' --output-on-failure`,1/1 suite PASS. `build/dev-android/alignment-ctest.log`
- VERIFIED genuine clangd23.1.0: 현재compile DB로 이정렬source probe, error0·warning1. warning은 Release에서 제외된 gyro debug 출력의 `<iostream>` include; debug 경로 보존. `build/dev-android/alignment-clangd.json`
- VERIFIED `git diff --check`: 소유 source/header/test whitespace 오류0
- source SHA-256: `59e9e374d3d1549392d7a173c21d64c785e77e10be6e2232eab884cbef9249ac`
- header SHA-256: `cc30e65758b0a5b6460db6837dda9990bfa8432041b3626846089646549abce6`
- test SHA-256: `68233ed68cb7aa69847c11a221d37cb5c393c4745151d19090a6c4802adb985e`
- fresh Numerical binary SHA-256: `8b5cbd8f17e0cf2a12014a068a9d24618ab486af162fa751fdbd02512124a267`
- 삭제·정리: 정렬2곳의 normal equation 누적·identity covariance·uniform1000배 scaling·중복local matrix 제거.6개 원래 물리 equation·내부scale100× parameterization·gyro bias 수식·4회 refinement 유지
- UNVERIFIED: full native suite·fresh WASM·동일실기기입력 replay·phone 재기록은 leader 소유 후속검증. 이41개 regression으로 Z Flip 6 field 정확도·발산 해결 주장 제외
