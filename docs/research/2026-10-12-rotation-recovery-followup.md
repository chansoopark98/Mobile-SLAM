# 회전 복구 후속 조사: 대응 소실과 깊이 비관측 분리

- 날짜: 2026-10-12, Asia/Seoul. 담당: GPT-6.1 Sol Ultra, 제품 write 없음
- 판정: **후반 Lost에는 정확한 2D 대응이 있어도 metric depth가 부족한 구간 존재. ORB·F 완화로 해결된다고 판단할 근거 없음**
- 다음 설계 검토 대상: A1 control 위의 provisional anchored projective inverse depth. `h=Rji*b_i+ρ*tji`를 유지하며 깊이 불확실성과 metric publication을 분리. 구현은 새 Sol 계획·테스트 명세와 Astra 계획 검토 이후
- 별도 현상: 빠른 회전의 앞부분에는 실제 오대응이 많음. 현재 F 거부·wrong gyro·반복무늬 검사를 유지; 이 현상을 depth 비관측과 합쳐 설명하지 않음
- 자료: [기록 진단](2026-10-12-recorded-diagnosis.md), [open VIO](2026-10-12-open-vio.md), [web engine](2026-10-12-web-engines.md), `build/recovery-20261012/frontend-followup/`
- A0/A1·A2 모두 아직 수락 후보 아님. churn12617은 고정 개발 fixture이며 held-out·물리 Flip6 증거 아님

## 조사 범위·provenance

- 현재 `src/frontend/feature_tracker.cpp` SHA256 `dfcde83a2deaa45fe8c0f0c5d08381a28083cd254cfff8a983cc8f4959dffbeb`; frozen dirty A0·A2 manifest·현재 source 모두 byte 동일
- `include/frontend/feature_tracker.h` SHA256 `288869b66319969735cf056faad0d74c0c7115cca3f83b784f46e10fbee6584d`; `src/backend/factor/projection_factor.cpp` SHA256 `4e581dbb2b07cd8dbf7427d5dfbf5cebb27b0c6fe15be88c3590da52e33f54ae`; 역시 A0/A2/current 동일
- 입력: frozen churn12617의480×640·20Hz 영상400개,60Hz IMU1201개, 같은 K/extrinsic·기록 시각. 6–6.8초 left60%,10–10.8초 right60%,14–14.8초 center60%를 값16으로 가린 원본 파생 보존
- A0/A2 기존 detailed result를 읽기만 함. 관측 window240frame의 LK/F/mask/age·backend membership 집계. 추정기·full GT matrix·새 binary build·물리 측정 실행 없음
- 사전에 고정한13frame `[112,120,127,135,136,200,204,216,280,287,295,296,300]`과 직전 영상의 선택 PGM24개 payload SHA가 input manifest와 일치
- `provenance.json`: 선택 PGM/file·IMU CSV·input JSON·source SHA. `stage-visibility.json`: 입력 result/spec/GT/input SHA, 모든 stage행, 선택13frame의 GT 가시성·오차
- `consistency.json`: A0 240+A2 240+phone299=779개 stage행의 소실 집합·subset·mask+new=final·가시성 분모 일관성 확인. 제품 regression 또는 물리 검증이 아님
- 독립 GT oracle: 이전 detector pixel의 ray와 알려진 유한 scene plane을 교차→현재 GT camera로 투영→FOV·명시 strip·scene occlusion 검사. GT는 추정기에 전달하지 않음. 가시성은 **점 중심** 기준의 상계이며21×21 patch 전체 가시성·descriptor 복구 성공을 증명하지 않음
- native 실제 readback OpenCV4.5.4, WASM header4.5.5. 현재 Python cv2는4.12.0이므로 그것을4.5.4 실행·candidate 성능 근거로 사용하지 않음

## 같은 입력의 실제 소실 단계

- 단계 순서: previous tracks → LK status → border → F/ray 검사 → spatial mask → GFTT 신규 ID. `trackerFinal`에 age1 신규 검출이 포함되며 engine은 age>1만 backend에 전달
- `previousSupported`: 직전 pre-solve membership에서 solveFlag1·positive depth·실제 current residual>0인 ID. GT metric maturity 또는 직전 final filter 통과를 뜻하지 않음
- rescue 채택 frame의 `lk_raw`는 ordinary 전용 status가 아니라 trial 채택 뒤 status. 따라서 아래 LK 합계를 순수 ordinary LK 성공률로 부르지 않음

| 사례 | previous→LK→border→F→mask | 새 검출 / 최종 검출 | 직전 supported ID→LK→F→mask | 현재 pre-solve factor |
|---|---|---|---|---:|
| A0 churn120, mask 진입 |120→116→116→56→53|27 /80|83→83→46→45|46|
| A0 churn127, current 부족 |79→73→73→21→18|61 /79|4→4→0→0|0|
| A0 churn136, mask 해제 |76→76→72→45→44|76 /120|4→4→2→2|7|
| A2 churn136, 같은 frontend |76→76→72→45→44|76 /120|0→0→0→0|0|
| A0 churn300 |120→119→119→118→116|4 /120|2→2→2→2|2|
| A2 churn300 |120→119→119→119→114|6 /120|0→0→0→0|0|
| phone prefix82, GT 없음 |120→95→91→26→25|95 /120|2→2→0→0|0|
| phone prefix98, final 부족 |120→119→115→114→112|8 /120|68→68→67→67|67|

- A0 관측 Lost35frame의3328개 previous feature-frame: LK3039, border2972, F1723, mask1559. F에서1249개 제거되며, 이를 임계값 결함으로 단정하지 않음
- A2 관측 Lost79frame의8350개 previous feature-frame: LK7897, border7753, F6049, mask5723. **2D 생존량은 많지만** pre-solve metric admission은 별개
- phone82는25개 retained old ID+95개 age1 신규 검출. 역사 eligible bank96은 현 영상 지원0과 양립. phone98은 pre-solve67개가 있어도 final failure이므로 초기 지원 부족과 구분

### 독립 GT로 확인한 대응 품질

| 사례 | GT-visible retained 중심 |1.5px 이내|retained 오차 중앙값|GT-visible F 거부 / 거부 오차 중앙값 |
|---|---:|---:|---:|---:|
| A0/A2 healthy112 |92|90|0.07986px|8 /67.68184px|
| A0 churn127 |8|0|52.07022px|40 /51.23109px|
| A0 churn136 |43|39|0.17559px|27 /94.93508px|
| A0 churn300 |115|114|0.11447px|0 /해당 없음|
| A2 churn300 |112|111|0.11119px|0 /해당 없음|

- frame127에서40개 점 중심이 실제 visible이어도 기존 LK 결과가 약51px 잘못된 위치. F를 완화하면 잘못된 대응을 추가할 위험이 있음. 실제 가림·FOV 밖·patch 일부 가림을 ORB가 복구한다고 가정하지 않음
- frame127의 retained 중에도 큰 GT 오대응 존재. RANSAC의 가상 consensus·reverse cycle·작은 photometric error 자체를 ID 정확도나 current support의 독립 증명으로 사용하지 않음
- frame300은111개의 정확한 현재 2D 대응을 보유한 A2에서 metric current0. 새로운 descriptor로 parallax를 만들 수 없음. 유지된 bearing의 정보 활용과 depth lifecycle을 먼저 검토할 근거
- root의 별도 GT-rotation counterfactual에서 rank rejection3505/3543(98.93%)가 κ>10000 유지됨을 보고받음. 이 문서는 그 counterfactual을 재실행하지 않았으며, 본 독립 증거는 위2D/visibility 결과
- 고정 churn SE3 scale1 재확인: A0 fresh324/400·warmup265/300·ATE49.835073m/APE123.353636°, A2 fresh274/400·warmup215/300·ATE0.317694m/APE13.265090°. 오차 감소가 coverage·절대 GT gate 실패를 상쇄하지 않음

## 현재 rescue가 이미 수행하는 일

- ordinary LK는 cached image/Scharr pyramid 재사용. ordinary F inlier가 original count의 과반이면 추가 rescue를 생략하여 기존 건강한 흐름 보존
- gyro hint로 **점 중심**을 투영하여 displacement>patch radius10px인 후보만 initial-flow LK. source patch의 회전·affine shape는 변환하지 않음
- ordinary 성공점을 교체하려면 seed photometric error가2배 이상 개선; 후보 reverse LK level0의 왕복 거리≤0.5px; verified candidate와 최종 F support의 엄격한 dominance 검사
- frame127: ordinary inlier21, seed attempted79/reverse attempted40/adopted0. 현재 진단에는 verified 수·seed/reverse mask·trial F inlier가 없어 최종 탈락이 reverse 검증 또는 dominance 중 어느 단계인지 확정 불가. trial은 기존 대응도 포함하므로 reverse 후보40만으로 trial F 상한을 계산하지 않음; 임계값 완화 근거 없음
- cached pyramid·SIMD는 이미 있음. 동일 코드의 재도입·전체 feature 수 감소·solver cap 변경을 새 해결책으로 제안하지 않음

## primary source와 실제 적용 경계

| 근거 | 확인한 내용 | 적용 경계 |
|---|---|---|
| [Civera/Davison/Montiel, TRO2008](https://webdiis.unizar.es/~jcivera/papers/civera_etal_tro08.pdf), Eq3–5·IV/V | anchored inverse depth는ρ0에서도 유한한 projection과 깊이 불확실성을 표현, uncertainty가 줄기 전에도 pose에 관측 사용 |6D EKF state와 covariance/correlation 포함. 논문의 초기 평균·분산 또는 XYZ 전환값을 현재1ρ Ceres에 복사하지 않음 |
| [Montiel/Civera/Davison, RSS2006](https://www.roboticsproceedings.org/rss02/p11.pdf), II–IV | bearing·inverse depth의 통합 표현, motion 후 유한 depth로 이동 가능 | 영구 infinity label·확정 깊이 대입 없이 불확실성 유지 |
| [Basalt `ba_utils.h`](https://github.com/VladyslavUsenko/basalt/blob/0f3b2b52c807f70ff4e2973ce253c73329eea7bc/include/basalt/utils/ba_utils.h#L83), BSD3 | homogeneous `[unproject(direction);inv_dist]`에 SE3 적용; translation Jacobian은 inv_dist에 비례, inverse-distance Jacobian은 상대 translation |2방향+inverse distance의3변수. fixed anchor bearing1ρ와 동일 모델 아님. rho0 projection 식 존재는 해당 landmark의 실제 admission 정책 통과 증거가 아님 |
| [Demmel et al., ICCV2021](https://www.usenko.net/pdf/demmel2021rootvo.pdf),3.2.2·4.1·4.3.2 | rank-deficient QR의 prior는 Moore–Penrose pseudo-Schur와 동등; landmark nullspace projection·frozen linearization으로 gauge 보존 | 원래 nullspace를 artificial depth prior로 없애지 않음. 전체 minimum-norm 해 동등성에는 논문의 rank 분리 조건26 또는 추가 residual 조건이 있음 |
| [OpenVINS single-depth 공식 유도](https://docs.openvins.com/update-feat.html#feat-rep-anchor-inv3) | full3D 초기화 뒤 bearing marginalization, 고정 bearing과 매 update nullspace 제거 | 이 구현을 단순히 초기ρ0의 fixed-ray1ρ factor와 동일하다고 부르지 않음 |
| [RD-VIO v3](https://arxiv.org/html/2310.15072v3), III-C/D·Eq13/14 | depth-free bearing orientation residual·R-subframe IMU chain, 선택적 위치 regularizer | depth가 관측되지 않는 구간의 별도 구조 후보. finite translation에서 순수 rotation을 강제하면 bias 위험; 위치 regularizer의 새 물리 가정·수락 계약 필요 |
| [Strasdat et al., DWO ICCV2011](https://www.doc.ic.ac.uk/~ajd/Publications/strasdat_etal_iccv2011.pdf),4.3/4.4 | 읽은 논문은 global XYZ 사용과 지역 co-visibility/occlusion 후보 선택을 명시 | 이 논문을 anchored inverse-depth 구현 근거로 잘못 인용하지 않음. 후기2012 thesis6.2.4는30MB fetch 한계로 원문 미검증, 채택 근거에서 제외 |

- 고정 코드 revision: Basalt `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`; OpenVINS `69488123ed9362dd44b6f28e7f4680abbff1442b`·GPLv3; XRSLAM `4beb1a942f33da9afbfae2d70e2c641cfc2bb675`·Apache2. 의존성·개별 header의 license는 별도
- OpenCV4.5.4 commit `4223495e6cd67011f86b8ecd9be1fa105018f3b1`,4.5.5 commit `dad26339a975b49cfb6c7dbe4bd5276c9dcb36e2`. 확인한 LK/ORB/matcher 세 파일은 두 tag에서 byte 동일; native/WASM 실행·JIT 성능 동일 주장 제외
- OpenCV top-level Apache2; 실제 LK 파일의 기존 OpenCV notice, ORB 파일의 Willow Garage BSD notice 보존. source 복사·새 runtime dependency 없음
- 원문·URL·revision·SHA·실패는 `sources/manifest.json`. 논문 PDF는 연구 근거이며 open-source code license를 부여하지 않음
- OpenCV 공식4.5.4 문서3개는 web 도구에서 실제 열어 확인; direct HTML 저장은403 실패로 보존. 해당 API 근거의 재현 가능한 로컬 원문은 위 pinned LK/ORB/matcher source. Demmel CVF URL fetch 실패는 author PDF로 대체했고 실패도 보존
- PDF SHA256: Civera2008 `23061290fbce2ee95edcca201198e393377f3acba59379fc7196dbf89d6f7467`; RSS2006 `8f28e46f49a2c3fceb2dc99850e1f553e6df0124a236b58ff2261d32dc5a84ab`; Demmel2021 `a5b74285e06e10149e1d8845f35edeebecf0d01a805103a149a05e8e64886a93`

## generalized provisional inverse-depth에 필요한 계약

- 아래 식은 현재 `ProjectionFactor::Evaluate`35–77행과 projective geometry에서 직접 유도. 문헌의 parameterization·threshold·상태 구조를 그대로 이식한다는 의미 아님
- anchor optical ray `b_i=[u_i,v_i,1]`, inverse optical depthρ, 상대 camera transform `(Rji,tji)`에서 `h=Rji*b_i+ρ*tji`, residual `r=W(π(h)-[u_j,v_j])`
- `ρ=0`: translation Jacobian0, orientation Jacobian은 일반적으로 유한. inverse-depth Jacobian `Jρ=W*Dπ(h)*tji`이므로 finite translation에서는ρ가 변할 수 있음. camera pure rotation `tji=0`이면 Jρ도0. body rotation의 camera lever arm translation은 tji에 포함
- **ρ0은 초기 projective 좌표일 수 있으나 finiteT에서 고정하면 안 됨.** 깊이가 약하게 관측될 때 joint pose/ρ solve와 depth uncertainty·rank 진단을 유지. 작은 cost·positiveρ·solver usable만으로 metric landmark 승격 금지
- provisional state와 metric solved state를 구분. metric map/PnP publication은 기존 finite-positive·cheirality·all-view reprojection·geometry gate 통과 뒤만 허용. unknown/signed trial이 apply에서 자동 삭제·metric 성공승격되지 않도록 별도 수명 정의
- `currentBearingConstraints`, `currentProjectiveConstraints`, `currentMetricLandmarks`, translation information/rank를 분리. provisional factor 수를 기존 metric current≥2 조건에 합산하면 수락 계약 변경; 새 계획에서 명시·검증
- pure/weak translation에서 bearing-only update가 orientation/bias를 개선할 수 있다는 것과 fresh metric translation을 독립 관측했다는 것은 별개. stale pose 공개·가짜 metric support·INIT_DEPTH 대입 금지
- scalar nuisance depth의 elimination: `P=I-jρ*jρ⁺`. jρ0이면 P=I로 orientation 정보 보존; rank1이면 그 방향만 제거. 영·작은 depth column을 임의 ridge로 metric certainty로 바꾸고 이를 marginalization prior에 보존하지 않음. 새 전체 QR framework 도입보다 기존 Eigen/pseudoinverse 계약의 충분성부터 검증
- solve 중 damping과 영구 prior의 정보를 구분. 같은 tangent/linearization에서 Schur·Moore–Penrose 또는 rank-revealing nullspace 결과·비용·rank를 독립 비교; global translation/yaw gauge 유지
- OLD/NEW anchor 변환은 homogeneous `[b;ρ]→[R*b+ρ*t;ρ]`. 새 h.z로 나누어 `b_new=h/h.z`, `ρ_new=ρ/h.z`;ρ0은0 유지. prior가 active depth block을 포함하면 block 좌표·Jacobian·주소를 함께 변환하거나 해당 block을 정확히 eliminate한 뒤 reanchor
- 기존 NEW의 “depth block이 outgoing prior에 없다” 전제는 provisional factor 추가 뒤 다시 증명. 현재 단순 metric depth reanchor만 호출하는 변경은 불충분
- 동일 관측을 provisional residual과 이미 marginalize된 prior 또는 metric residual 양쪽에서 중복 사용하지 않음. ID·slot cap·admission mask·prior remap과 lifecycle를 같은 계획에 포함

## frontend 후보는 대응 회복이 증명될 때만

| 후보 | 최소 구현 경계 | 비용·오매칭 통제 |
|---|---|---|
| gyro 기반 local patch warp | ordinary healthy 경로 그대로; ordinary가 실패한 finite in-FOV subset에만 calibrated ray rotation의 local warp Jacobian을 적용한 patch 비교·refine. 기존 cached pyramid 재사용 | 중심 seed와 다른 실험. rotation-only shape는 translation/depth·rolling shutter·wrong gyro에서 틀릴 수 있음. plane 경계·부분 가림·warp footprint 밖·wrong sign/axis/delay를 실패로 처리. full image warp+pyramid는 추가 O(W×H) 비용이므로 먼저 채택하지 않음 |
| same-frame mature ID descriptor 재연결 | 직전 frame에서 실제 supported였고 이번 flow/F에서 소실한 ID만, 현재 frame 신규 후보에 one-to-one로 매칭; 같은 ID의 기존 retained 대응을 덮어쓰지 않음 | 양방향·ratio/absolute Hamming·spatial gate·subpixel cycle·기존 F/multiview 검증. previous/current invisibility면 recovery0이 정답. 오래된 다중 frame database·가짜 깊이·map wholesale match 제외 |

- [OpenCV4.5.4 LK 공식 계약](https://docs.opencv.org/4.5.4/dc/d6b/group__video__track.html): initial-flow는 center 초기값이며 affine patch estimator가 아님. 현재 err는 일반 L1 patch error; `OPTFLOW_LK_GET_MIN_EIGENVALS`를 켜면 의미가 달라져 기존 “2배 photo 개선” 비교를 그대로 사용할 수 없음
- [ORB 공식 계약](https://docs.opencv.org/4.5.4/db/d95/classcv_1_1ORB.html)과 pinned `orb.cpp`: 새 detect는 orientation을 계산하지만, **제공된 GFTT keypoint에 단순 `compute()`만 호출하면 그 keypoint의 angle을 자동 재추정하는 경로가 아님**. orientation/scale·class_id·border drop·octave reorder의 ID 대응을 명시해야 함
- ORB는 이미 native/WASM features2d에 링크되어 있지만, 별도 image pyramid·blur·descriptor 비용이 생김.8level default·500feature를 그대로 추가하지 않음. 이전 descriptor도 필요하므로 정상 frame에서의 유지 비용과 lazy 실패 시 생성 비용을 각각 측정
- Hamming cross-check·ratio는 [공식 matcher 예](https://docs.opencv.org/4.5.4/dc/dc3/tutorial_py_matcher.html)의 알고리즘 의미만 참고. tutorial 수치를 전화 수락 threshold로 채택하지 않음
- Basalt pinned `frame_to_frame_optical_flow.h`243–299/`patch.h`는 patch SE2 shape update·정규화 photometric residual·양방향 검증을 실제 수행. dependency나 전체 optical-flow 구현 복사 없이 local warp의 설계 근거로만 사용
- 이번 조사에서 새 patch/ORB의 실제 복구·비용·false-match 개선은 **미검증**. frame300의 깊이 비관측을 해결하는 후보로 선택하지 않음

## 새 계획·테스트 명세에 인계할 최소 순서

1. 기존 A0/A1/A2 실패·same pixels/IMU·source/artifact를 보존하고 provisional depth 설계의 상태·publication·current count 의미 동결
2. analytic3D·pure camera rotation·finite weak translation·forward translation·camera lever arm·ρ0/±tiny/finite Jacobian·rank·gauge·OLD/NEW prior oracle 선작성. 무관측 depth는 metric map0, finiteT에서 depth를 고정한 rotation factor의 bias control 포함
3. 작은 membership/apply/optimizer 변경을 한 lane에서 연결. 예상 파일: `include/frontend/feature_manager.h`, `src/frontend/feature_manager.cpp`, `include/backend/optimizer.h`, `src/backend/optimizer.cpp`, `src/backend/estimator.cpp`, 기존 diagnostics/engine/binding; factor 수식 변경이 실제 필요한지 먼저 확인
4. tracker 변경 없이 churn frame300/136과 원본 fast12617에서 projective constraint 효과·coverage·translation/rotation GT를 확인. accepted-only 좋은 frame 선택 금지
5. 대응 문제는 별도 ablation. ordinary healthy·같은 실제 IMU·gyro inverse/축오류/시간오프셋·translation-only·반복무늬·완전/부분 occlusion·FOV exit/reentry controls에서 true ID와 false ID를 독립 계산. 실제로 복구 가능한 subset·고정 cap·phase 비용이 증명될 때만 `feature_tracker.{h,cpp}` 변경 계획을 검토
6. 평가 기준을 바꾸지 않고 전체400/warmup300·Lost/fresh-gap/reset·SE3 ATE/rotation/RPE·기존 TUM/fast held-out 회귀. speed는 직렬 비교; main capture·frontend·backend·IMU queue를 분리

- 연구 재현: `python3 build/recovery-20261012/frontend-followup/analyze-stages.py`; 기존 result·선택 PGM·spec·GT 읽기, owned output만 작성
- 후속 계획은 `development_plan_sol` 소유. 이 문서는 연구·인계이며 실행 승인이나 A3 수락 문서가 아님
- 보존: 원본 PGM/IMU/GT·실패 결과·baseline/A2 binaries·현재 제품·서비스·다른 lane 작업. 새 dependencies·제품 코드·설정·threshold 변경0
