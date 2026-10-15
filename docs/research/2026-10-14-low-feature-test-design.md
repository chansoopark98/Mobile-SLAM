# Low-feature·바닥 후보·IMU 예측 검증 설계

- 날짜:2026-10-14. 상태: **제품 구현 전 acceptance 설계 + 실제 baseline 확보**. 바닥 검출기/IMU fallback 구현·제품 테스트 파일 수정 없음.
- 새 fixture/evidence 전용 경로: `build/low-feature-20261014/fixtures/`.
- 핵심 관측: initialized 상태에서 짧은0.25초/긴2초 blank를 넣으면 각각5/40개 전부Lost. 영상이 돌아와도 즉시 metric tracking 회복·GT 통과를 보장하지 않음.
- 현재 binary: `38837d4b79abb088610484f4e2c016aac4336fe9761dcfc56874d48e64785589`를 `fixtures/engine/`에 복사·고정. 원본 active loader/manifest·TLS·production server·실기기 미변경.

## 1. 생성한 fixture와 입력 계약

| fixture | 구성 | 원본과 동일한 부분 | 바뀌는 부분 |
|---|---|---|---|
| `textured-control/` | fast-rotation seed707,400frames/20s | pixels·timestamp·rawIMU·GT·config·parameters | provenance/spec intervention metadata만 추가 |
| `blank-short/` | texture→index140–144 blank→texture | source identity·시간·rawIMU·GT·profile |5개480×640 영상만 gray128,7.00≤t<7.25s |
| `blank-long/` | texture→index140–179 blank→texture | source identity·시간·rawIMU·GT·profile |40개 영상만 gray128,7.00≤t<9.00s |
| `floor-plane-cases.json` | noisy floor/wall/ceiling/table +동일관측 의미모호성5건 | 명시된world Z-up·gravity·camera pose | 평면 방향/높이·semantic oracle |
| `imu-analytic-cases.json` | known initialized state의 일정속도/일정가속도/일정yaw3건 | body specific-force·gyro SI·200Hz·2s | 각 운동의closed-form oracle |

- parent: `build/motion-20261010/fast-rotation-707`, 기존 독립 generator `scripts/dev/generate-synthetic-vio.py`의 analytic motion·GT. estimator에서 생성한 pose를 GT로 사용하지 않음.
- GT의 물리 motion은 그대로이고 gray 이미지 교란은 가림/무정보 관측을 뜻함. GT를 엔진/회복 정책에 주입하지 않음; GT 존재가 알고리즘이 그 상태를 추정할 수 있음을 뜻하지 않음.
- blank가 아닌 모든 PGM은 원본 bytes 복사, blank payload SHA 새로 기록. 재생기의 input-root confinement를 유지하기 위해 외부 symlink를 사용하지 않음.
- `fixture-validation.json`:3arms×400pixel SHA,변경index5/40,rawIMU/config/parameters/GT 동일성PASS. `preparation-receipt.json`·`copy-materialization-receipt.json`에 source/script/engine provenance.
- script는 `fixtures/prepare-fixtures.py`; 기존 경로가 있으면 재생성하지 않음. 최초 symlink 방식 source와실패log도 별도 보존.

## 2. 실제 현재 WASM baseline

- 실행: 기존 `scripts/dev/synthetic-browser.mjs --mode raw --imu-policy` 기본future-bracket, `--rotation-rescue on`,400frames. ephemeral localhost server·headless browser; 실제camera/서비스 접근 없음.
- 실제 profile: solver budget10s/max10iterations,features120,tracking21/3/20,F1px,PnP OFF,execution seed0/OpenCV1thread. 기존 parent의fixed-work raw profile이며Mobile App의0.06s profile과 다름. 이 결과를 phone 처리속도와 직접 비교하지 않음.
- 3runs 모두400/400 input completion. 최초fresh index41(2.05s),index100–139는40/40 Tracking. **preblank0–139행의 pose/state/timestamp/epoch가 세arm에서 exact일치**.
- engine binaryhash38837d4,endpoint IMU정책/profile 동일. 차이는 선언한gray5/40개뿐; 원래highspeedrotation 자체의실패도control에 보존.

| arm | 전체fresh/400 | blank 구간 | texture복원→첫fresh | 최대인접fresh 위치변화 | 기존GT gate |
|---|---:|---|---|---:|---|
| textured-control |287|해당없음;140–179는40fresh|해당없음|0.266829m|FAIL:coverage/RPE|
| blank-short |198|5/5Lost,empty_features|index161,+0.80s|0.359714m|FAIL:coverage/rotation/RPE|
| blank-long |296|40/40Lost,empty_features|index186,+0.30s|0.957045m|FAIL:coverage|

- 세arm 모두epoch변화0. 장기blank의총fresh가control보다높아도 성공으로 승격하지 않음: 관측 history/state 경로가 달라졌고 큰reacquire jump가 남음.
- SE(3) scale1 기존evaluator: ATE/rotation RMSE는control0.194799m/3.303119°,short0.203234m/11.511998°,long0.116128m/4.946354°. blankshort 이후전체rotation오류 증가·coverage실패를 보존.
- `runs/*-v3/report.json`, `evaluation.json`, `baseline-summary.json`, `execution-receipt-v3.json`, `evaluation-receipt.json`에원문/명령/hash. evaluator exit1은위GT gate실패이며 정상으로숨기지 않음.
- 최초실행은잘못된CLI값`rotation-rescue1`,다음실행은외부frame symlink거부로실패. `on`과byte복사로수정한v3에서실제실행; v1/v2실패receipt보존.

## 3. IMU 예측의 수락 경계 — 구현 전 제안

- metric visual tracking과 **inertial prediction**을다른출력종류로정의. 예측을fresh visual pose·새map point·optimizer prior·floor확정근거로세지않음.
- 최초성공추정/속도/bias/gravity가없는cold-start blank에서는prediction생성금지. 일정속도·순수회전만으로metric scale초기화됐다고판정금지.
- 제안temporal budget은마지막independent visual correction으로부터 **최대0.5s**. currentUI500ms와같은숫자라도별도prediction정책이며정확도보장상수가아님; parent계획승인전미적용.
- 0.25s shortfixture는budget안의동작,2s longfixture는만료를검사. 긴구간에서매tick 새timestamp를붙여horizon을무한갱신하면실패.
- 필수음성조건: missing real IMU bracket,비단조/중복/비유한sample,큰gap,epoch/reset/calibration 변경,시각ahead,관성불확실성상한초과. 사용자에게표시하는prediction은이유·last visual age·expiry를동반.
- 갱신된prediction의timestamp와시작anchor timestamp를분리. source sample없으면velocity를외삽하며fresh로보고하지않음. IMU가계속온다는사실만으로translation정확도를인증하지않음.
- recovery는새로운visual support/geometry gate통과를요구. timestamp만new이거나기존pose를재사용한것을회복으로판정하지않음.

### 독립 native expected values

- world Z-up,g=[0,0,−9.81],초기R=I,bias0;센서는`f_B=R_WBᵀ(a_W−g_W)`·gyro rad/s. known-state fixture는native unit에서초기조건을직접지정하기위함이며browser engine에GT주입하는시험과분리.
- 일정속도v0=(0.6,−0.2,0.1):0.5s에p=(0.3,−0.1,0.05),v불변. IMU는gravity-only여도초기속도를0으로재설정하면실패.
- 일정가속도a=(0.4,−0.2,0.1),v0=0:0.5s에p=(0.05,−0.025,0.0125),v=(0.2,−0.1,0.05).
- 일정yawω=π/2rad/s:0.5s에q_wxyz=(cosπ/8,0,0,sinπ/8). zerotranslation이지만rotation은45°.
- 중간t=0/0.1/0.25/0.5/1/2s의closed-form oracle과401rawsample을JSON에분리저장. native gate는배열길이/단위/knowninitialstate/identity부터검사.
- exactconstant translational moments는double허용오차1e−10수준을제안. 회전은기존normalized first-order increment가Exp와같지않음을반영:200Hz·0.5s의독립유도각오차`π/4−100·2atan((π/2)·.005/2)=4.0372383e−6rad`. Exp구현이면analytic1e−12수준,기존increment재사용이면유도된truncation budget을명시. 구현출력을그대로expected로복사하지않음.
- 기존관련회귀: `tests/test_engine_contracts.cpp`의AnalyticIMUIntegralAcrossRatesPhasesAndCarriedFutureSamples,ConstantIMUSeedAndNoNewSamplesHaveExactCoverage,CameraContinuationCannotExtrapolatePastLastRealIMUDuringSensorPause,BlankFrameCannotReuseIt; `test_numerical_contracts.cpp`의독립rotation/time-moment검사; `test_integration_base.cpp`의AffineAccelerationUsesItsExactPositionTimeMoment.
- 이기존source계약을새prediction정책에맞게약화하지않음. 새component핵심native/동일WASM계약을추가해야하며이번작업에서native predictor테스트가실행·통과한것은아님.

## 4. Floor/ground-like plane의 수락 경계

- 직접pointcloud성분fixture:160planeinlier+40outlier,normal방향Gaussianσ0.01m,고정seed14102026. `input`에는gravity/camera/points만,semantic truth는`oracleOnly`에별도.
- floor z=0,wall x=2,ceiling z=3,table z=0.8,camera z=1.6. 정상·높이·inlier noise·outlier의독립expected를제공.
- floor와table은둘다gravity에수직이며camera아래일수있음. **동일한관측pointcloud를table또는raised floor로해석하는2개oracle**을준비;입력이완전히같은데semantic floor를구분한다고주장하면시험설계오류.
- 따라서UI는검증한근거범위에서`horizontal plane/ground-like candidate`로표시하고,semantic floor확정에는추가cameraheight/장면근거필요. wall/ceiling을floor로표시하면실패;table-only모호성에서는abstain/uncertain.
- 제안성분gate:높은inlier비율에서plane normalerror≤3°,offseterror≤0.05m. 이는고정noisefixtures의연구기준이지실기기보장치가아님; metric scale/gravity가신뢰되지않으면미터heightgate미사용.
- sign동등성n,d↔−n,−d를정규화하고distance·camera측의signedheight를명시. 단순normal dot절댓값만으로ceiling/floor를구별하지않음.
- degenerate예정추가:3미만point,동일점/일직선,비유한점,gravity불가,pointscale불가,고비율outlier,두horizontalplane경쟁,시간상plane ID switching. 이번준비5건이이전체음성시험을대신하지않음.
- map이불충분한blank에서는새plane이생겨나지않음. 마지막plane을화면에유지할경우그anchor timestamp/epoch/불확실성을노출하고추적지속·새floor관측으로세지않음.
- 이JSON은geometric detector단위입력이며실제mono triangulation의point오차·scale·semantic분류를검증한것이아님. 실제영상부터의floor통합시험은별도승인gate.

## 5. Baseline 이후 필요한 비교

1. 동일fixture·loader/profile로기준을고정하고product수정전이계획/정책review.이번control의기존GTFAIL을없던것으로하지않음.
2. known-state analyticnative성분부터prediction정확도·expiry·no-correction증명을red/green. floor계측은별도독립성분fixture.
3. actualraw/WASM의textured→blank→restore3arm실행;processed/visualfresh/predicted/expired/Unknown/Lost를모두별도집계. 영상GT평가에는동일source400분모유지.
4. 예측구간ATE/RPE·rotation·bias/velocityuncertainty·horizon별오차를별도평가. 전체SE3정렬만으로blank구간drift를숨기지말고preblank고정alignment의구간오차도사용.
5. source→pose/render latency·predictCPU p50/p95/p99·prediction expiry·회복jump를검사. 장면/phase가다른arm의빠른firstfresh만으로개선을채택하지않음.
6. 원본sensor/camera·물리GT가없으므로최종phoneacceptance미검증. 실제phonecamera/server는사용자가시작하는범위로유지.

## 재현

```bash
node scripts/dev/synthetic-browser.mjs \
  --input build/low-feature-20261014/fixtures/blank-short/input.json \
  --loader build/low-feature-20261014/fixtures/engine/vio_engine.js \
  --output build/low-feature-20261014/fixtures/runs/new-blank-short \
  --mode raw --frames 400 --rotation-rescue on --timeout-ms 180000
python3 scripts/dev/evaluate-synthetic-vio.py \
  --dataset build/low-feature-20261014/fixtures/blank-short \
  --results build/low-feature-20261014/fixtures/runs/new-blank-short/report.json \
  --estimate-frame camera \
  --output build/low-feature-20261014/fixtures/runs/new-blank-short/evaluation.json
```

- 기존output사용금지. benchmark20s는source시간이며wallclockruntime가아님. runtime완료와GTgate통과를분리.
- 신규파일:이문서및`build/low-feature-20261014/fixtures/`만. 제품/기존tests/source/기존dataset수정0.

## 추가 준비: 실제 영상→현재WASM→map의 floor feasibility

- mockpose/합성pointcloud만으로floor E2E를주장하지않도록추가실행. `fixtures/floor-visible-clean101/`은기존independent generator의clean101에**known camera extrinsic pitch40° down**·넓은textured floor를적용한400frame입력.
- generator원래floor는world z=0,center(0,4,0),u=(1,0,0),v=(0,1,0),halfextent(4,4);새fixture는floorhalfextent(6,8). body trajectory/rawIMU는기존clean101과exact동일. cameraGT/K/extrinsic는새synthetic rig에맞게독립계산.
- `fixtures/generate-floor-visible.py --output NEW_DIRECTORY`로생성. 관측pixel은실제raycast·가림·texture sampling 결과이며floor point/GT pose를엔진입력으로주지않음.
- build-only `capture-map-baseline.mjs`는기존raw harness를고정복사하고버려지던**실제Wrapper mapPoints**를JSON에보존. product/harness원본·WASM변경0. actualexport pointcount와XYZ array길이전부대조.
- binary388의`getMapPoints`는`getSlidingWindowMapPoints`의solved/depth-admittedXYZ를내보내지만**feature ID와현재영상관측membership은없음**. currentVisualFactors의개수만으로export된각point가현재보이는것을증명할수없음.
- 분석기`fixtures/analyze-map-floor.py`:원래engine frame/up에서160개deterministicRANSAC triple,20inlier/3cm/60%/tilt≤15°/두principal축P05–P95extent≥0.3m/camera아래조건. GT는그후SE3scale1평가에만사용. 이것은검증용fitter이지승인된제품검출기구현이아님.

| actual input/WASM 결과 | 기존clean101 | floor-visible-clean101 |
|---|---:|---:|
| fresh / planned |339 /400|357 /400|
| raycast floor pixel비율(frame100/200/300) |20.08 /21.89 /28.66%|97.13 /94.77 /100%|
| legacy map에서20개이상horizontal inlier를fit한frame |202|357|
| 제안map support/extent/tilt gate 통과frame |0|280|
| 추가GT floor normal≤3°·offset≤5cm까지통과 |0|264|

- 기존clean의floor후보support비율은11.8–25.7%로60%기준미달. floor검출기가잘못됐다고단정할상황이아니라영상/지도에서다른surface가우세한명시적negativecontrol.
- 새fixture의첫수락frame43:actualexport118points,currentVisualFactors79,planeinlier79/118(66.95%),두축extent3.458×2.892m,engineup대비tilt0.713°. GT평가normalerror0.653°·flooroffseterror0.04060m.
- 수락280frame의GTflooroffset p50/p95/max=0.01340/0.03907/0.05095m,angle p50/p95/max=2.01074/3.02712/3.67235°. 따라서16frame은별도GTfloorgate를통과하지못함. 성공frame만남기지않고전체400/357/280/264분모보존.
- 새fixturetrajectory 기존evaluator **PASS**:warmup300/300,ATE0.01806234m,rotationAPE1.84936590°,epoch변화0. 해당syntheticrig gate이며물리floor정확도나새current-point API통과가아님.
- 증거:`floor-map-baseline.json`,`floor-map-analysis.log`,`floor-visible-provenance.json`,`runs/{legacy-clean101,floor-visible-clean101}-map/report.json`,floor-visible의`evaluation.json`.
- 다음E2E수락:계획된`getCurrentMapPoints`가현재관측/depthsolved/accepted ID만내보낸다는native예상값검증후같은actualimage/WASMfixture에연결. **현legacy map264frame을새current-visible20point gate의통과로승격하지않음.** floor없는/희소화면에서도예전map point가수락되어유령floor를만들지않는negativecontrol추가.
- parent가검토중인prediction horizon0.5s/sourcegap0.1s와floor20point/3cm/60%/15°/0.3m는제안정책. 위분석은수락가능fixture를확보한것이며제품기본값채택은별도계획review이후.
