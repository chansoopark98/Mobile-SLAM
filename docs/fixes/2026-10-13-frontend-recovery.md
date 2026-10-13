# Gyro patch warp 격리 실험

- 날짜: 2026-10-13, Asia/Seoul
- 판정: **제품 반영 기각**. frame127의 정확한 2D 대응은 회복했지만 frame204에서 실제 scene occlusion ID1개를 잘못 채택. F·reverse 통과만으로 ID 정확도를 보증하지 못함
- 제품 코드 변경0. 결과: `build/tracking-performance-20261013/frontend-recovery/verification.json`, `evaluation.json`, `matched-summary.json`, `provenance.json`
- 범위: PRD/test-spec G2의 선택지A, gyro patch warp 한 방식. descriptor·F 완화·depth 생성·solver 변경0

## 입력·방법

- frozen churn12617의 사전 선택13frame `[112,120,127,135,136,200,204,216,280,287,295,296,300]`와 직전 PGM, 같은 previous feature 순서·ID·IMU·K·`T_B_C` 사용
- OpenCV **4.5.4** native C++17·`-O2`·1thread. `pkg-config`와 실행 `CV_VERSION` 모두 확인. Python은 JSON·NumPy·offline oracle 처리만 수행; Python OpenCV4.12 실행 없음
- 기존 center-seed LK는 점 중심만 gyro로 옮김. 후보는 `H=K R K^-1`로 source image를 회전한 뒤, 회전된 source patch에서 current image로 local LK level0 수행
- source21×21 patch 유효 mask·original ID 순서·ordinary 과반 trust skip·2배 photometric 개선·reverse 왕복≤0.5px·verified>ordinary·trialF>2ordinary·F1px 보존. 현재 신규 검출과 기존 ID를 섞지 않음
- current frame에서는 직전 age1도 age2가 되므로 기존 제품과 동일하게 모든 previous ID를 seed 대상으로 유지. priorSupported ID의 회복은 별도 집계
- control: 같은 raw IMU gyro hint의 기존 center-seed LK level3. 추가 negative: gyro 부호 반전·x/z 축 교환·100ms 지연·전체 constant 가림·정지12px 반복 checker
- `13frame × 4gyro × 3pixel × 2method = 312`회 각각 exit0·JSON readback. ordinary F inlier 수는 frozen 결과의13frame와 **13/13 동일**

## 결과

- 표의 정확 대응은 독립 ray→scene 교차→GT camera 투영·가시성 검사에서 current 오차≤1.5px. GT는 probe에 전달하지 않음. 점 중심 가시성은 patch 전체의 가시성을 보장하지 않음
- `priorSupported`는 직전 solveFlag1·positive depth·current residual 보유 ID이며, 독립 metric 성숙도 인증이 아님

|frame|center-seed retained 정확/전체|warp retained 정확/전체|warp 변경 후 F잔존 정확/전체|그중 priorSupported 정확|
|---:|---:|---:|---:|---:|
|127|0/21|42/48|42/43|1|
|200|44/51|47/52|44/44|18|
|204|40/48|53/59|52/53|1|
|216|48/51|60/61|45/45|1|
|296|14/20|47/50|31/31|0|

- frame127 control: attempted79→photometric40→reverse verified36→trialF40. `40>2×21` 실패로 채택0. warp:71→44→43→48, 채택43. F threshold 완화 없이 실제42개 exact 대응 회복
- frame127의 추가 ID784: previous pixel `[359,140]`, candidate `[325.905975,143.484131]`. oracle `no_previous_surface`로 **정확 여부 인증 불가**. 잘못된 유한3D 대응이라고 단정하지 않음
- frame204의 ID2774: previous `[181.679443,154.812256]`, candidate `[149.974808,177.047867]`. 독립 scene 교차에서 `scene_occlusion`인데 reverse/F를 통과하여 **새 false recovery1개**. raw IMU와100ms delay 모두 발생
- wrong sign/axis와 전체 가림·반복 checker에서는 center-seed/warp 모두 채택0. 이 제한된 negative 통과가 scene occlusion 실패를 상쇄하지 않음
- ordinary trust가 있는112/135/136/300에서는 후보 시도0. frame300 retained115/118 정확 대응을 유지했지만 metric depth 비관측을 해결한 결과가 아님

## 보존한 첫 실험

- `revision-v1/`: source warp+LK level3·직전 age>1 제한의312raw·source·binary·결과 보존. frame127 정확 회복 채택0으로 기각
- 첫 probe의 age 제한은 제품보다 좁았음. frame127 visible53 중 age>1은29, age1은24; source의 seed population과 맞추기 위해 v2에서 해당 제한 제거
- v2 후보의 local level0는 같은 warp 방식의 잔여 motion 검색을 patch 범위에 제한. center-seed control은 원래 level3 유지. 이후 파라미터 탐색·threshold 수정0
- 초기 harness 오류2개: final diagnostic의 신규 ID가 updateID 전 `-1`이라는 점을 current `lk_raw`의 같은 index assigned ID로 수정; report의 중복 `control/method` keyword 제거. 실패 출력 뒤 모든312case 재실행 성공

## 적용 판단·한계

- **채택하지 않음**: 원래 F/reverse/dominance를 보존해도 가려진 ID를 다른 texture로 연결할 수 있음. 대응 수 증가만으로 false ID gate를 통과시키지 않음
- 확인한 방향: source patch의 회전과 local search는 큰 회전에서 center seed만 사용하는 LK의 실패 일부를 줄임. 제품 후보로 다시 검토하려면 scene occlusion·고유성에 대한 별도 증거가 먼저 필요; 임계값 완화로 재분류하지 않음
- 실제 엔진 camera_rotation/bias readback이 기존 diagnostics에 없어 **raw gyro midpoint integration ablation**으로 제한. actual bias-corrected hint와의 parity 미검증
- 13개 기존 engine snapshot의 same-frame2D 실험이며 수정 엔진 전체400frame replay·fresh heldout·TUM·WASM·phone 실행 없음
- seed원점의ray를 scene에 교차하는 oracle는 lifetime semantic ID·map metric accuracy의 보증이 아님. 신규 임의 depth·GT estimator 입력0
- 병렬 native accuracy/build 부하가 있었으므로 raw `ordinaryMs/recoveryMs`는 benchmark 근거 제외. 속도 개선·60ms live budget 통과 주장0
- 반복 checker는 정지 negative 한 종류. moving repeated texture·완전한 scene geometry occlusion suite·phone textures 검증으로 확대하지 않음

## 재현

```bash
python3 build/tracking-performance-20261013/frontend-recovery/prepare.py
c++ -O2 -std=c++17 build/tracking-performance-20261013/frontend-recovery/patch-warp-probe.cpp -o build/tracking-performance-20261013/frontend-recovery/patch-warp-probe $(pkg-config --cflags --libs opencv4)
python3 build/tracking-performance-20261013/frontend-recovery/evaluate.py
```

- source·binary·input·label SHA256: `provenance.json`, `cases.json`. 기존 raw 결과는 보존한 뒤 재실행. 제품 product diff0·dependency0·서비스 변경0
