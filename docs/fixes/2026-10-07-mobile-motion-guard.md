# 모바일 포즈 이동 불연속 차단

- 날짜: 2026-10-07
- 소유 범위: `web/js/vio-worker.js`, `tests/browser/mobile-motion-guard.test.mjs`, 이 문서
- 변경 전 계획: 실제 Worker 회귀 실패 확보 → live app의 `mobileMotionGuard: true` 설정만 적용 → 같은 Worker 처리에서 reset·공개 차단 → 회귀·JS 문법 검증
- 기존 변경: app·IMU·renderer·서버·WASM·데이터셋 보존
- 기존 정책: C++ quality warning과 numerical usability 별도 유지; native/direct WASM/TUM replay의 기본 동작 유지

## 근거

- 실기기: Galaxy Z Flip 6, Android 16, Chrome 154
- 기록: `build/dev-android/divergence-observation.json`; 약30초, 포즈398개, 동일 client/engine epoch2
- 관측: 포즈398개 모두 finite·fresh·valid·`tracking:relative_translation_threshold`
- 독립 연속397쌍: 위치차 최소44.204m / 중앙90.066m / p95 120.980m / 최대158.525m
- 출력간격 중앙80.06ms; 위치norm 51,796.956→87,665.929m; solver 모두 `NO_CONVERGENCE`, 11회
- 독립 위치차: row-major camera pose의 translation 열3·7·11에서 `||P_current - P_last_published||`
- 기존 detector 기준5m 재사용: 모바일 실행의 큰 이동 불연속에 대한 운영 차단 기준; 정확도·속도·기기 calibration 기준 아님

## 공개·회복 계약

- Worker configure: `mobileMotionGuard: true`일 때만 활성; 생략·false는 기존 동작
- 차단: 정확한 `relative_translation_threshold` warning token 또는 같은 engine lifecycle의 마지막 공개 포즈 대비 위치차 `>5m`
- 첫포즈: engine의 정확한 relative translation warning만 검사; 절대 world 위치norm으로 차단 금지
- `marginalization_*`, bias warning, `NO_CONVERGENCE`만으로 차단 금지
- 차단 시 같은 Worker 처리에서 `engine.reset()`; IMU ring·guard history 초기화; 손상된 window·prior 다음 frame 진행 차단
- 결과: 기존 request 상관정보·입력timestamp 유지, 새engineEpoch, `pose=null`, `poseFresh=false`, `poseValid=false`, `poseTimestamp=null`, `mapPoints=null`, `mapPointCount=0`
- reason: `pose_translation_discontinuity`; 원래reason·측정 위치차·이전epoch는 `motionDiscontinuity`에 기록
- reset·reconfigure·dispose·engine epoch 변경·transport loss: 마지막 공개 포즈 기준 삭제
- 회전·velocity·절대위치 gate·smoothing 추가 제외

## 검증 계획

- 실제 Worker script + controlled engine protocol fixture; 독립 expected 좌표·거리로 경계 검사
- `(0,0,0)→(3,4,0)`: 정확히5m, 허용
- `(0,0,0)→(3,4,0.1)`: `sqrt(25.01)>5m`, 차단
- 첫 warning pose·warning 없는 큰 이동·invalid 중간 frame·정상 marginalization warning·기본 비활성·reset/epoch recovery 검사
- 명령: `node --test tests/browser/mobile-motion-guard.test.mjs tests/browser/transport.test.mjs`
- 문법: `bash scripts/dev/check.sh syntax`
- 한계: fixture는 Worker 공개·회복 계약 검증; estimator 정확도·실기기 개선 판정은 별도 실제 재기록 필요

## 실행 결과

- 환경: Node.js22.21.1, base revision `f2105709e4e9a4d8006f5a78e1f1fe9e989a6072` + 기존 local 변경
- VERIFIED red: 구현 전 최초8개 중3개 실패; 첫 warning pose·warning 없는 `sqrt(25.01)` 이동·invalid frame을 사이에 둔8m 이동이 정상 공개되던 근거. `build/dev-android/mobile-motion-guard-red.log`
- VERIFIED green: guard10개 + 기존transport40개, **50/50 PASS**, TODO/skip0. `build/dev-android/mobile-motion-guard-transport.log`
- VERIFIED 문법: `bash scripts/dev/check.sh syntax` exit0, 신규test `node --check` exit0; ESLint/TypeScript typecheck 구성 없음. `build/dev-android/mobile-motion-guard-syntax.log`
- VERIFIED `git diff --check`: 변경Worker whitespace 오류0
- Worker SHA-256: `8adfeea20d71b072e5c3134a2272077f2c113ead12d305b8d9e4933899cd0e7b`
- test SHA-256: `093811ea7c9a2da3947393d957adcaa9f2a82491319f0a83dd2d5b2dee67f255`
- C++·WASM·의존성·dataset·서비스 변경 없음; 새WASM 빌드 주장 제외
- UNVERIFIED: 입력수정·guard 활성 후 실기기 초기화 coverage·포즈 jump·reset 횟수·독립GT 정확도; 실제phone 재기록 owner 결과 필요
- 남은한계:5m 미만 점진drift·첫포즈의 warning 없는 잘못된scale는 이 운영차단만으로 검출 불가. reset 반복·pose coverage 감소는 해결 증거 제외
