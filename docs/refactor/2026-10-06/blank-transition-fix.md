# 첫 균일 영상 추적점 제거 계획·검증

- 소유: `src/frontend/feature_tracker.cpp`, 필요 시 대응 header, `tests/test_engine_contracts.cpp`, 본 문서
- 보존: 기존 dirty source·dataset·TLS·서비스·generated WASM; Engine·Estimator·Worker·수학 코드 변경 제외
- 실제 실패: `build/refactor-validation/run-5/textured-blank-recover/{failure,summary,results}.json`
- frame99: textured·feature150·map161·fresh tracking; frame100: exact zero262144B·feature8·map157·새 timestamp의 fresh tracking; frame101~111: feature0·pose null·map0; frame121: tracking 복구
- 원인: LK의 이전 영상 gradient 기반 status가 첫 균일 target의 잘못된 correspondence를 허용; stale timestamp 판정과 별도
- 수정 전 회귀: 실제 FeatureTracker의 textured seed→동일 textured LK→exact constant target; uniform 값0/중간값/255, CLAHE on/off, backend/track-only 경로 확인
- 최소 수정: 입력 grayscale의 `min==max`만 reject; 기존 `pruneTrackedPoints`로 point/history/normal/velocity/id/count 동시 제거, pending `n_pts` 제거; 표준 image/time/pyramid/normal finalization 유지
- ID·calibration: `reset()` 호출·`n_id` 재설정 제외; 이전 Estimator track과 복구 후 ID 충돌 방지
- 단순화: 별도 reset·pose 주입·blur/low-contrast 임계값·추가 backward LK 경로 없음; 기존 prune 호출 재사용
- 검증 명령: 지정 regression의 독립 scoped tracker fixture compile/run RED→GREEN; fresh 통합 native·ASan/UBSan·WASM·실 Worker350 frame replay는 parent/build owner 후속
- 수락: 첫 균일 frame의 모든 tracked 배열/normal maps 비움, camera 보존·timestamp 진행; 다음 textured backend frame 새 점·단조 ID·정렬된 배열; 다음 실제 LK frame 정상 유지
- 한계: 정확히 균일한 영상의 validity 계약; nonconstant blur/저조도·실제 phone·trajectory/GT 정확도·room4 별도 문제 검증 제외

## 수정·독립 회귀 결과

- production diff: raw input `cv::minMaxLoc` 1회·`min<max` predicate, constant target의 기존 prune·`n_pts.clear()`, constant target의 new-feature detector 생략; header/API/layout 변경0
- caller 영향: 기존 Engine의 `image_data.empty()` 분기 도달; timestamp는 진행, pose/map invalidation·reason/status 판정은 기존 caller 경로 유지
- tracker golden: 96×96 constant background +16/-16 사각형36점 seed→동일 영상의 실제 LK→첫 constant target. 수정 전 surviving4≠0; `build/refactor-evidence/blank-transition/probe-before.log`
- tracker 범위: CLAHE off의 constant0/127/255, on의0/127 × backend/track-only2 = 10 transition; constant255+CLAHE on seed는 target 이전 LK0으로 독립 transition baseline 제외
- tracker RED/GREEN: `FirstUniformTargetDropsLKTracksAndRecoversWithNewIds` old1/1 FAIL(exit1)→new1/1 PASS(exit0), 10 transition의 모든 병렬 배열·normal maps empty, camera 유지·시각1.1 진행·ID allocator 불변
- recovery golden: 바로 다음 textured backend frame 신규 count1·ID≥이전 nextID·history empty; 그 다음 real LK frame의 point/ID/count/normal/velocity 길이 일치·유효 점 존재
- Engine RED/GREEN: `UniformTargetReachesEmptyFeatureBoundaryAfterRealLKSeed` old feature4/reason`initializing`→new feature0/reason`empty_features`, 다음 textured 관측의 feature 복구·epoch 유지; 주입 pose/Estimator state 없음
- evidence: `build/refactor-evidence/blank-transition/{red,green,engine-red,engine-green}.log`; 각 compile command JSON·source snapshot·source/dependency/binary hash manifest 보존
- scoped compile: baseline/current FeatureTracker TU 직접 compile, 고정 native archive/Ceres/OpenCV/gtest 의존성 재사용. 전체 repository fresh ABI·SAN·WASM·served Worker 증거와 구분
- 후속 gate: parent/build owner의 fresh 통합 native/SAN/WASM, 새 artifact 승격 뒤 실제350frame Worker 첫 blank row100의 feature0·pose null·map0·회복 검증
