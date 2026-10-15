# 8th Wall 원본 WASM 잔차·Jacobian 추가 실행 — 2026-10-14

- 결과: 이전 단순 residual6건에 이어 **서로 다른 내부 함수3개의79조건 실행 PASS**. Snavely projection40, translation stability24, scalar3D alignment15.
- 새 도구: [binary-deep-residual-probe.mjs](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/scripts/dev/references/eighthwall/binary-deep-residual-probe.mjs).
- 원시 결과: [최종79조건 JSON](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/build/slam-revision-20261014/eighthwall/residuals-v2/result.json), [최초70조건 보존](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/build/slam-revision-20261014/eighthwall/residuals-v1/result.json), [WABT/radare 명령·hash receipt](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/build/slam-revision-20261014/eighthwall/disassembly-v2/receipt.json).
- 범위: binary 분석으로 좁힌 Evaluate 후보를 원래 WASM code로 실행. **SDK camera lifecycle·추적 optimizer 전체·새 phone trajectory 실험은 아님**. 공개 source는 수식/ABI 비교 자료이며 source↔binary 전체 build parity를 가정하지 않음.
- [최종 독립 값·hash 검사](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/build/slam-revision-20261014/eighthwall/verification.json) PASS. 제품 코드·active loader·원본 SDK 수정0. 분석 사본과 생성 evidence는 `build/slam-revision-20261014/eighthwall/`에만 보관.

## 1. 원본·분석 사본·도구 검증

| 대상 | SHA256 / 결과 |
|---|---|
| 원본 `xr-slam` WASM | `780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422` |
| 분석 사본 | `02a2d94b64be8d1049b985579e9653f8857688412a320e90a35c6f9894d1544c` |
| 원본=사본 Code section | `6ff897c9bfb8709f4b792d1595eae175e4bc5947f09d16c011273e94be00a809` |
| 최종 probe script | `6b417286e87183c898b0d309beab34531f9974135fd2655dba56f3caf54823a9` |
| 변경 section | Export section7에 `Snavely=1594`, `TranslationStability=1601`, `Scale3d=1610` 추가만 수행 |
| 나머지 section | code/data/table/memory/global 등 **모두 byte hash 동일** |
| WASM 검증 | 원본·사본 `wasm-validate` 각각 exit0 |
| import | 함수259개 trap stub;79조건 실제 호출0 |
| JS syntax | `node --check` exit0 |

- input SHA가 다르면 실험 중단. output directory가 있으면 덮어쓰지 않고 중단. 원본 dataset 경로 출력 금지.
- 본 실험은 code 교체·solver mock이 아니다. 원본 함수 body와 정적 data를 보존한 사본에 직접 호출 entry만 추가하고, 독립적으로 만든 object/parameter memory를 전달했다.
- Emscripten constructor·camera·GL을 시작하지 않은 성분 호출이다. import가 필요한 경로는 즉시 throw하도록 설정해 숨은 외부 의존성을 무시하지 않았다.
- RTTI→typeinfo→vtable→element table의 함수 후보 연결과 processStagedFrame에서 constructor 경로까지의 정적 도달성은 [이전 binary 보고서](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/docs/research/2026-10-14-8thwall-binary.md)에서 확인. 이번은 그 후보의 산술 및 Jacobian을 직접 검증한 추가 증거다.

## 2. 독립 예상값과 ABI

- 공통 ABI: `Evaluate(object, parameterBlockPointers, residualOutput, jacobianBlockPointers)`; object+20에서 functor pointer를 읽는 원래 instruction 확인.
- Snavely functor: float32 `scale,u,v` offset0/4/8; camera double6 `[angle-axis3,translation3]`, world point double3.
- TranslationStability functor: double6 `[이전world위치차분3,직전world위치3]`; camera double6.
- Scale3d functor: virtual-table 영역 이후 offset8부터 double6 `[model3,observed3]`; scalar scale double1. offset은 source class member만 보고 추측하지 않고 해당 WASM load instruction과 대조.
- 예상 residual은 독립 JavaScript Rodrigues 회전식으로 계산. zero-angle에서는 회전 미분 `e_i×point`, perspective quotient rule, translation derivative를 사용한 **분석적 Jacobian**으로 대조.
- nonzero/tiny/large-angle Jacobian은 **원본 WASM을 차분하지 않고**, 별도 Rodrigues 예상 함수의 중앙차분 `h=1e−6·max(1,|parameter|)`과 대조. Scale3d Jacobian은 observed vector 자체가 독립 예상값.
- residual 허용오차 `1e−9 + 1e−12·|expected|`; Jacobian `2e−5 + 2e−7·|expected|`. float32 functor 값은 실제 저장 형식대로 반올림하고 비교.
- Jacobian 미요청은 출력 buffer에 sentinel123456을 넣고 그대로 남는지 검사. 전체 pointer null, block pointer null, Snavely의 camera-only/point-only/both 조합을 분리.
- domain15조건은 Infinity/NaN까지 별도 문자열로 저장. 이는 nonfinite를 정상값으로 승인한 시험이 아니라 **원래 residual이 무엇을 반환하는지 예상과 대조하는 경계 특성 시험**이다.

## 3. SnavelyReprojectionResidual — 함수1594,40/40

- 확인 수식: `q=R(angleAxis)·X+t`, `r=[scale·q.x/q.z−u, scale·q.y/q.z−v]`.
- 공개 수식: [bundle-residual.h:338–348](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/geometry/bundle-residual.h#L338-L348).
- tested: zero/tiny/finite/large rotation, positive/negative/near-zero/zero depth, zero/zero point; 각 경우 Jacobian5가지 선택.

| 입력 / 조건 | 반환 | 실제 residual | 해석 |
|---|---:|---|---|
| camera `[0,0,0,.2,−.3,.4]`, point `[1,2,5]`, functor `[500,25,−50]` |1|`[86.1111111111111,207.4074074074074]`|분석적 projection·camera/point Jacobian 일치|
| camera rotation `[.2,−.3,.4]`, 같은 t/point |1|`[−111.02833994738165,103.89085901249189]`|독립 Rodrigues 및 차분 Jacobian 일치|
| 뒤쪽 point `[1,2,−5]`, zero camera |1|`[−125,−150]`|이 residual 자체에는 positive-depth guard 없음|
| point `[1,2,1e−8]` |1|`[49999999975,100000000050]`|작은 depth의 큰 영향 확인|
| point `[1,2,0]` |1|`[Infinity,Infinity]`|boolean success와 finite residual은 다른 계약|
| point `[0,0,0]` |1|`[NaN,NaN]`|0/0 경계도 외부 검증 필요|

- **관찰을 확대하지 않음**: Ceres 호출자·map admission·loss가 해당 sample을 실제로 받아들인다는 실험이 아니다. binary 전체의 cheirality bug를 발견했다고 주장하지 않는다.
- 공개 bundle source는 해당 residual을 `HuberLoss(0.5)`로 감싸는 경로가 존재. raw residual 시험에는 loss가 포함되지 않는다. source의 이 loss가 같은 binary tracking branch에서 활성이라는 검증도 이번 범위 밖. [bundle.cc:425–449](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/geometry/bundle.cc#L425-L449)
- Mobile-SLAM 함의: 원래 vendor residual조차 함수 성공값만으로 geometry validity를 보장하지 않는다. 현재 projection/cheirality/domain guard를 제거해 solver failure를 줄이는 변경은 근거가 없다. residual·Jacobian·robust loss·admission을 따로 비교해야 한다.

## 4. PoseEstimationTranslationStability — 함수1601,24/24

- 확인 수식: camera parameter가 world→camera angle-axis/translation이면 `C=−Rᵀt`, `r=0.1·(C−C₂−v₁)`, `v₁=C₂−C₁`.
- 따라서 `r=0.1·(C−2C₂+C₁)`: **world camera position의 frame-step 두 번째 차분**이다. 이 함수의 입력에는 dt나 IMU sample이 없다.
- 공개 class/method: [bundle-residual.h:133–151](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/geometry/bundle-residual.h#L133-L151), [519–546](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/geometry/bundle-residual.h#L519-L546).

| 실험 | 실제 residual | 의미 |
|---|---|---|
| zero rotation, t=`[4,5,6]`, v₁=`[.1,.2,.3]`, C₂=`[1,2,3]` |`[−.51,−.72,−.93]`|view translation 부호와 world center 변환 확인|
| 같은 history·nonzero rotation |`[−.8318791232527762,−.5862510504068151,−.6687487261787233]`|rotation이 world center와 Jacobian에 영향|
| C₁=0, C₂=`[1,2,3]`, C=`[2,4,6]` |`[0,0,0]`|일정한 frame별 위치차분은 penalty0|
| 같은 world centers, camera rotation 변경 |최대`1.33e−16`|view→world 변환으로 회전표현과 무관한 center 일관성 확인|
| 물리 가속도3m/s², 균일30Hz |`[.0003333333333333333,0,0]`|dt²에 비례하는 raw penalty|
| 같은 가속도·60Hz |`[.00008333333333333333,0,0]`|30Hz 대비1/4|
| 일정 속도1m/s, 시각0/.01/.03s 위치 |`[.001,0,0]`|불균일 frame 간격은 일정 실제 속도에도 nonzero penalty|

- 균일 sampling에서 `r≈0.1·a·dt²`; IMU의 물리 가속도 residual과 같지 않음. source comment의 acceleration이라는 단어만 보고 dt-normalized motion factor로 이식하면 안 된다.
- 공개 pose-estimation code에는 이전 extrinsics가2개일 때 stability residual을 `CauchyLoss(7e−3)`로 추가하는 branch가 있다. 인접 CameraMovement는 작은 움직임에 유리하다는 주석과 다른 robust loss를 사용. [bundle.cc:671–691](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/geometry/bundle.cc#L671-L691)
- **아직 미확인**: private tracker가 이 branch를 언제 켜는지, 입력이 일정한 간격인지, 외부 weight가 sampling을 보정하는지, real-frame 사용 빈도와 성능 효과. 이번24건은 해당 성분이 원래 binary에 있고 수식대로 실행된다는 증거.
- Mobile-SLAM 함의: “8th Wall smoothness”를 단순 화면 EMA로만 설명할 수 없음. 적어도 optimizer 안에 world-position smoothness residual 후보가 실제 계산 가능하다. 그러나 metric scale/GT/current support가 실패한 상태에서 이를 기본 추가해 jump만 숨기는 변경은 부적절. variable-dt/drop/빠른 방향전환·bias/scale drift와 지연을 함께 검증해야 함.

## 5. Scale3dPoseEstimation — 함수1610,15/15

- 확인 수식: `r=s·observed−model`, Jacobian `dr/ds=observed`.
- 이 클래스는3D 점 집합 alignment의 scalar 성분이다. 이름의 Scale만으로 world VIO의 absolute metric scale estimator라고 동일시하지 않음. [수식](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/geometry/bundle-residual.h#L679-L685)

| 조건 | 실제 residual | 실제 scalar Jacobian |
|---|---|---|
| model=`[1,2,3]`, observed=`[.5,1,1.5]`, s=2 |`[0,0,0]`|`[.5,1,1.5]`|
| 같은 points, s=0 |`[−1,−2,−3]`|`[.5,1,1.5]`|
| 같은 points, s=−2 |`[−2,−4,−6]`|`[.5,1,1.5]`|
| model=`[1,2,3]`, observed0, s=2 |`[−1,−2,−3]`|`[0,0,0]`|

- 모든 경우 boolean1 반환. **positive scale constraint와 observability는 scalar residual 자체에서 보장되지 않음**. observed0이면 scale 정보를 제공하지 않는다.
- 공개 wrapper `poseEstimationScale3d`는 point별 CauchyLoss와 Ceres solver를 구성하지만 이것이 binary의 같은 실행 branch/옵션임은 별도 문제. [bundle.cc:910–948](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/geometry/bundle.cc#L910-L948)
- Mobile-SLAM 함의: responsive output scale·NORMAL status·finite optimizer를 metric scale/GT 통과로 평가할 수 없다. current scale gate를 vendor function 존재만으로 완화할 근거 없음.

## 6. 도구 신뢰도와 복구·처리 파이프라인에 대한 한계

| 함수 | fresh WABT 명령 수 | fresh radare 명령 수 | 판정 |
|---|---:|---:|---|
|1594 Snavely|2390|2390|개수 일치와 수치40/40은 서로 별도 증거|
|1601 TranslationStability|1595|566|불일치 유지; radare CFG만으로 함수 전체 복원 주장 금지. WASM 실제24조건 실행 통과|
|1610 Scale3d|89|89|개수 일치와 수치15/15 확인|

- 새 external tool6명령 모두 exit0. stderr와 hash를 receipt에 보존했으며 경고·불일치를 삭제하지 않음.
- synthetic memory test는 helper의 SIMD/rotation/derivative arithmetic을 실제 Node WASM runtime으로 실행. 79회 중 import0이므로 browser/device/GL 파이프라인 실행 증거가 아니다.
- texture queue getReady/mark/release/reset의 독립 binary state-machine 실험은 이번에 수행하지 않았음. 기존 source staging 분석을 이번 동적 검증으로 승격하지 않음.
- IMU integration, feature reacquisition, relocalization, marginalization 전체, field accuracy·phone processing time은 이번 함수3종에서 복원되지 않음.

## 7. 구현 전 검토에 반영할 항목

1. **projection domain 분리 유지**: finite residual/Jacobian·positive Z·current support·rank·lineage는 서로 다른 승인 조건. Snavely boolean1은 대체 기준이 아님.
2. **smoothness 후보는 opt-in 연구로만**: adaptive dt factor·고정 frame-step regularizer를 구분하고, stationary/constant-velocity/acceleration/variable-dt/drop/급회전 입력에서 GT와 delay를 동시 평가. 현재 default에 추가할 근거 없음.
3. **recovery에서 숫자 안정화와 관측 회복을 구분**: translation residual은 missing correspondence를 생성하지 않음. 현재 support0/1 문제는 별도 visual correspondence/admission 연구로 해결해야 함.
4. **scale 수락 분리**: scalar alignment, responsive unit, metric IMU initialization은 다른 계산. 원본/보정 gyro arm·SE(3) scale1 GT gate 유지.
5. **binary의 좁은 검증을 재사용**: 다음 shader/queue/solver 실험도 원본 hash·분석 사본 mutation·미호출 import·원시 실패를 명시. 디컴파일 결과만으로 private API 구현을 만들어 적용하지 않음.

## 재현

```bash
node --check scripts/dev/references/eighthwall/binary-deep-residual-probe.mjs
node scripts/dev/references/eighthwall/binary-deep-residual-probe.mjs \
  --input build/8thwall-deep-20261014/binary/survey/xr-slam/target.wasm \
  --output build/slam-revision-20261014/eighthwall/new-residual-run
```

- output은 반드시 새 디렉터리. node version·script/original/사본 SHA·section별 hash·입력·독립예상·실제값·Jacobian mode·tolerance는 생성 `result.json`에 기록.
- 이번 새 결과를 기존6건과 합산해 camera tracking85회 성공이라고 표현하지 않음. **이번79건은 세 residual 성분의 수치/경계 검사**이며 full optimizer·mobile tracking 성능은 다른 실험이다.
