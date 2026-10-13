# Mobile-SLAM 개발·검증 도구

- 범위: 개발 전용 npm 패키지, 로컬 MCP, native C++ LSP, 재현 가능한 검증 명령
- 제품 의존성: 기존 `web/package.json` 유지
- 산출물: `build/dev-tools/`, `build/dev-native/`, `build/dev-wasm/`; WASM 빌드 후 `web/` 자동 덮어쓰기 없음
- 기준: [2026-10-06 도구 진단](../../docs/audit/2026-10-06/tooling.md), [전체 진단](../../docs/audit/2026-10-06/)

## 데이터 경로

- canonical 데이터: `/mnt/backup/SLAM`; 읽기 전용 사용, 외부 원본 변경/삭제/재다운로드 없음
- 다른 경로: `MOBILE_SLAM_DATA_ROOT=/path/to/datasets`; doctor가 해당 경로의 root 최대16개 + 각 디렉터리 child 최대8개 목록만 확인
- doctor 데이터 검사: 최대2깊이, 데이터 내용 읽기/hash/recursive scan/download 없음; tool/source fingerprint와 구분
- 데이터 경로 접근 실패는 JSON `BLOCKED` 기록; doctor `--strict`는 필수 개발 도구/package gate이며 dataset 품질·가용성 gate와 별도
- repo 공식 복원 fixture: `assets/datasets/tum/dataset-room1_512_16`; canonical 공유 원본과 별도 provenance
- 기존 parity test/browser TUM 페이지의 repo 경로 고정은 현재 유지; 공유 데이터를 자동 symlink/copy하지 않음
- 공식 room1 복구 fallback: `python3 scripts/dev/fetch-tum-room1.py --evidence build/dev-tools/dataset-provenance.json`; 기존 target/staging 보존, 공식 archive MD5 확인 + SHA256 기록 + 안전한 staging extraction. canonical root 확인 후 해당 fixture가 필요한 경우에만 실행

## 설치

```bash
npm ci --prefix scripts/dev --ignore-scripts --no-fund
python3 scripts/dev/install-clangd.py
python3 scripts/dev/doctor.py --strict --output build/dev-tools/doctor.json
```

- 평가용 Python 환경 복제: `python3 -m venv scripts/dev/.venv`, `scripts/dev/.venv/bin/python -m pip install -r scripts/dev/requirements-evaluation.txt`; 검사 시 `MOBILE_SLAM_PYTHON=scripts/dev/.venv/bin/python`
- Node.js 22 이상, Python 3.11 이상, CMake 3.21 이상, Ninja, GCC C++17
- native 라이브러리: Eigen3, OpenCV, yaml-cpp, Pangolin; CMake가 고정된 Ceres 2.2.0 / GoogleTest 1.14.0 확보
- Emscripten: 기존 SDK 사용; 기본 `$HOME/emsdk`, 다른 경로는 `EMSDK=/path/to/emsdk`
- Chrome: 기존 시스템 Chrome 사용; 다른 실행 파일은 `MOBILE_SLAM_BROWSER=/path/to/chrome`
- clangd: 공식 Linux x86_64 23.1.0 ZIP + SHA256 확인, `scripts/dev/tools/clangd_23.1.0/` 격리 설치; 다른 OS는 공식 대응 바이너리 준비 후 `MOBILE_SLAM_CLANGD` 지정
- 고정 npm: Playwright 1.63.0, Playwright MCP 0.0.83, Context7 MCP 4.1.1, MCP SDK 1.32.1
- Playwright MCP 0.0.83의 내부 Playwright: upstream 고정 `1.64.0-alpha-1790635538000`; 브라우저 baseline harness는 별도 stable 1.63.0 사용

## 검증 명령

| 명령 | 검증 범위 | 한계 |
|---|---|---|
| `bash scripts/dev/check.sh doctor` | 실제 실행 파일 버전, package pin, hot-file SHA256 | 빌드 신선도/정확도 보증 없음 |
| `bash scripts/dev/check.sh syntax` | 기존 web JS + 개발 MJS parse, Python compile | ESLint/TypeScript typecheck 아님 |
| `bash scripts/dev/check.sh eval` | ATE/RPE/frame/scale/rank 9개 + 합성 생성·평가 계약 18개 회귀 | 실데이터 정확도와 구분 |
| `bash scripts/dev/check.sh quick` | syntax + evaluator regression | native/WASM/browser는 별도 |
| `bash scripts/dev/check.sh browser-contracts` | strict browser contracts + transport + exporter analytic | mandatory TODO/skip는 gate 실패; 실기기와 구분 |
| `bash scripts/dev/check.sh mobile-contracts` | dev mobile recorder bounded/copy/pose-delta 계약 | 실제 전화 camera/IMU/추정 정확도와 구분 |
| `bash scripts/dev/check.sh native` | headless configure → 현재16개 suite target build → discovered CTest | empty/missing suite·mandatory skip·실패는 gate 실패 |
| `bash scripts/dev/check.sh wasm-build` | Emsdk subprocess 활성화 → 별도 WASM configure/build | `web/`로 미배포, 실기기 성능 미확인 |
| `bash scripts/dev/check.sh clangd` | C++ LSP initialize/symbol/diagnostics | 컴파일 DB 필수, 단일 source 검사 |
| `bash scripts/dev/check.sh mcp` | MCP initialize/tools/list + 브라우저 fixture + 문서 조회 | Codex 현재 세션 도구 reload와 구분 |
| `npm --prefix scripts/dev run browser:baseline` | browser 소유 harness의 실행/산출물 | 실제 휴대전화 camera/IMU와 구분 |
| `bash scripts/dev/check.sh all` | doctor + quick + native | WASM/MCP/browser는 별도 실행 |

```bash
# CMake presets 직접 사용
cmake --preset native-dev
cmake --build --preset native-tests
ctest --preset native-tests

# 기존 fresh baseline DB로 genuine C++ LSP 확인
MOBILE_SLAM_COMPILE_DB=build/audit-native bash scripts/dev/check.sh clangd

# 검사 파일 선택
python3 scripts/dev/clangd-probe.py --compile-db build/audit-native --file src/backend/estimator.cpp
```

- 기본 병렬 jobs=2; `MOBILE_SLAM_JOBS=1..4`
- configure/build/test subprocess 기본 timeout=900초; `MOBILE_SLAM_TIMEOUT`으로 조정
- 각 CTest 기본 timeout=180초; 모든 실패 결과 확인, empty CTest는 오류
- dependency source cache 재사용 예시: `cmake --preset native-dev -DFETCHCONTENT_SOURCE_DIR_CERES-SOLVER="$PWD/build/_deps/ceres-solver-src" -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST="$PWD/build/_deps/googletest-src"`
- 최초 설정과 기존 test 실패 원인은 날짜별 audit 문서/로그 확인

## PC·실제 App 합성 검증

- [2026-10-09 통합 결과](../../docs/fixes/2026-10-09-mobile-web-system.md), [레퍼런스 비교](../../docs/research/2026-10-09/reference-comparison.md)
- `generate-synthetic-vio.py`: 독립 analytic GT·ray-cast gray·IMU·YAML·camera profile·hash manifest 생성. 출력은 새 폴더만 허용, canonical dataset 쓰기 금지
- `pc-entry-check.py`: 실제 `tiny_vins_mono`와 direct engine의 input bytes·IMU bracket·실제 설정·source timestamp·pose 비교
- `synthetic-browser.mjs`: 실제 HTMLVideo·Camera·IMU·App·Worker·renderer 경로. fixture의 GT는 estimator에 전달하지 않음
- `evaluate-synthetic-vio.py`: camera→body, 일대일 source 시각, 단일 SE3/scale1, ATE·APE·RPE·고정 coverage. 결과의 `passed`는 합성 rig 수락 기준

```bash
# 예시 output 이름이 이미 있으면 새 이름 사용
python3 scripts/dev/generate-synthetic-vio.py --output build/synthetic-example \
  --seed 101 --scenario clean
python3 scripts/dev/pc-entry-check.py --dataset build/synthetic-example \
  --pc build/dev-native/tiny_vins_mono --native build/dev-native/native_replay \
  --output build/synthetic-example-pc
node scripts/dev/synthetic-browser.mjs --input build/synthetic-example/input.json \
  --loader build/dev-wasm/vio_engine.js --output build/synthetic-example-browser \
  --mode live-paced-native-raf --sensor generic --frames 400
python3 scripts/dev/evaluate-synthetic-vio.py --dataset build/synthetic-example \
  --results build/synthetic-example-browser/report.json --estimate-frame camera \
  --output build/synthetic-example-evaluation.json
```

- browser modes: `raw`=직접 Worker, `live-deterministic`=가상 시계·직렬 처리, `live-paced`=제어된 callback cadence, `live-paced-native-raf`=실제 desktop rAF
- 회전 회귀: `--scenario fast-rotation --portrait`는 첫5초 기존 mobile60 motion을 유지한 뒤 복합축 빠른 회전을 추가. analytic gyro·body GT·영상은 독립 생성하며, 실제 phone 회전 허용 속도 근거가 아님
- 재현: `python3 scripts/dev/generate-synthetic-vio.py --output build/fast-rotation-example --seed 707 --scenario fast-rotation --portrait`. 출력 폴더는 새 경로만 허용
- 2026-10-12 회전 복구: live App 기본 ON, `?rotation-rescue=0`은 control OFF, `=1`은 ON. C++/Worker의 명시하지 않은 configure 기본값은 OFF 유지. synthetic browser `--rotation-rescue on|off|default`, native replay PROFILE 다음 ROTATION_RESCUE 인자 `0|1`로 선택
- 활성값은 configure ACK 대신 C++ `rotation_rescue_enabled` 또는 Worker `rotationRescueEnabled`로 확인. 기본값의 TUM 회귀와 활성화한 합성 회전 수락은 별도 결과로 기록
- 2026-10-10 ON 후보의 full TUM-VI gate 실패는 역사 기록으로 유지. 2026-10-12 경쟁 F 지지 후보와 IMU 적분 수정의 full TUM/독립 회전 결과는 [최적화 보고서](../../docs/fixes/2026-10-12-optimization-results.md) 참조. 실기기 Lost/발산 해결·속도 향상은 후속 재시험으로 확인
- 기본 harness는 명시 `fth=1`; `--f-threshold default`는 URL override 생략 후 실제 C++ readback 확인. 현재 engine 기본값도1
- `live-deterministic`의 가상 clock 비용을 실제 latency로 보고하지 않음. 실제 rAF도 camera captureTime·sensor API는 mock이며 물리 phone 검증과 구분
- 원본400frame·warmup 이후300frame 분모 유지. 누락·busy·실패·pose 없는 frame을 평가에서 숨기지 않음
- 캡처한 App 입력 재생: 생성된 `captured-input.json`을 `--mode raw`에 전달하고 `--frames`를 실제 accepted 수로 지정. 실제 C++ readback 설정·accepted pixels·IMU batch 경계 보존
- native replay trailing 인자: `[PROFILE=legacy|yaml] [ROTATION_RESCUE=0|1] [EXPERIMENTAL_OBSERVATION_POLICY=0|1|2]`. 기본 `legacy`는 기존 고정 tracker 유지, `yaml`은 supplied tracker 설정 사용
- 실패한 current-observation 실험은 instance API `setExperimentalObservationPolicy(0|1|2)`로만 선택. 기본0=시작 A2 legacy,1=D2 old-depth 복구,2=D3 good-history candidate 선택. Config/g_config·App/Worker의 자동 활성화·UI 추가 없음
- `getExperimentalObservationPolicy()`는 해당 Engine selected 값, native `getEffectiveExperimentalObservationPolicy()`와 `getFeatureDiagnostics().params.experimental_observation_policy_effective`는 실제 backend 값. backend 미생성 시 effective−1; configure/configureFromConfig는0, reset은 선택 보존, 값 변경은 새 epoch와 빈 history, 같은 값은 무동작
- native 마지막 실험 인자는 정확한 문자열 `0|1|2`만 허용하며 입력 처리·출력 생성 전에 검증. raw/summary에 `experimental_observation_policy_selected`·`experimental_observation_policy_effective` 기록. 이 격리는 추적·정확도 수락을 의미하지 않으며 원래 목표는 NOT_ACCEPTED
- 초기 covariance 파라미터는 기존 `V noise Vᵀ` 구현의 수치 profile. 주입한 sensor noise density와 같은 단위라고 가정하거나 dt 보정 없이 서로 복사하지 않음

- anchor-depth C3는 별도 CMake 옵션 `-DMOBILE_SLAM_EXPERIMENTAL_ANCHOR_DEPTH=ON`으로만 빌드(native/WASM 공통, 기본 OFF). runtime observation policy0/1/2와 별개. 원본 Flip6 안정성 gate 실패로 기본·active 승격 금지; [원인·A/B·재현](../../docs/fixes/2026-10-13-flip6-tracking-recovery-results.md) 참조. 비교 시 source SHA뿐 아니라 feature_manager.cpp의 실제 compile definition0/1와 artifact SHA를 함께 기록.

## 실기기 pose 흔들림 기록

- 보존 기록 재분석: `python3 scripts/dev/analyze-flip6-recordings.py /absolute/record.json --output build/new-flip6-analysis.json`. numeric/input schema, 결과 시간창·pose/초기화 coverage·지연 quantile·epoch·부분 영상·PGM hash를 검증하며 원본과 기존 출력을 덮어쓰지 않음.
- [2026-10-13 Flip6 종합 분석·적용](../../docs/research/2026-10-13-flip6-analysis-and-application.md): 재계산된 원본4개·상용 엔진 도구 검사·현재 로직·검증 한계.

- 장면 구분: 정지 구간, 제어된 천천한 이동/회전, 시작 위치 복귀; 장면/장치/browser/카메라 해상도/calibration/화면 방향과 기록시간 함께 보존
- 관측 필드: IMU event/sensor/callback 시각과 clock origin, video presentation/callback/capture-call 메타데이터, frame accept/drop, submitted-result roundtrip, pose 수신시각, translation/rotation jump, initialization/lost/reset, 오류
- timing provenance: 물리 exposure/capture 시각·독립 engine pose 시각·queue duration은 없거나 proxy일 수 있음. 누락/출처를 JSON에 명시; Worker stage instrumentation 없이 roundtrip을 queue/processing으로 분리 금지
- 기본 portrait480×640에서 landscape crop 없음; crop-K 위험은 crop 활성화/다른 profile 사용 시 조건부 확인
- smoothing 적용 전 기록 우선; 표시 jitter 감소와 실제 estimator 정확도/pose-age 개선 구분
- 정지 구간 jitter는 self-consistency 근거; 외부 GT 없으면 ATE/RPE 정확도 판단 제외

```bash
# 실기기에서 접근 가능한 HTTPS host + 해당 host의 trusted TLS cert 사용
python3 scripts/dev/serve-mobile-audit.py --host 0.0.0.0 --port 8766 \
  --cert /absolute/trusted-cert.pem --key /absolute/key.pem

# localhost mock 검증용 임시 인증서; 실제 phone trusted TLS와 별도
python3 scripts/dev/serve-mobile-audit.py --generate-dev-cert --host 127.0.0.1 --port 8766
```

- 전화 페이지: `https://<phone-access-host>:8766/audit-mobile.html`; 실제 카메라/IMU 권한 허용 후 recorder 사용
- localhost mock: `https://127.0.0.1:8766/audit-mobile.html`; browser mock 결과는 실제 sensor/phone 성능과 구분
- 관측 시간: 기본60초/최대120초; channel 최대10,000 samples, 전체40,000 samples. 제한 초과·stop 이후 drop 수를 export에서 확인
- Stop: sensor 중지, worker dispose, embedded app unload; JSON은 기기 browser 다운로드, schema `mobile-slam-observation-v1`
- raw image/device ID/track label 제외; 완전한 sensor replay 데이터와 구분
- 기존 `index.html?v=11` 관측 prototype 사용; 기본 제품 페이지/SLAM 추정 알고리즘 변경 없이 별도 audit 페이지 사용
- 기존 `sendBeacon` 차단 및 server POST405; 서버로 관측 로그 자동 업로드 없음
- 임시 인증서: `build/dev-mobile-audit/certs`, 7일/localhost SAN; phone 신뢰 인증서로 해석 금지

## Android 무선 ADB·Chrome 실제 탭 관측

- 도구: 기존 Android SDK `adb`; 추가 제품·개발 package 설치 없음
- 휴대전화: 같은 LAN의 개발자 옵션 → 무선 디버깅. 새 PC는 페어링 코드 창의 IP/port로 `adb pair`; 기본 화면의 연결 port와 구분
- 실기기 Chrome에서 `https://dev.serdic.com:7002/` 유지·화면 잠금 해제; 화면이 `Dozing`이면 CDP 요청 timeout 가능
- 대상 선택: USB·무선이 동시에 보여도 무선 IP:port를 `-s`로 명시

```bash
adb version
adb mdns services
adb devices -l

# 실제 휴대전화에 표시되는 값 사용. 포트는 무선 디버깅 재시작 시 변경 가능.
phone_address=192.168.0.126:46635
adb connect "$phone_address"
adb -s "$phone_address" shell getprop ro.product.model
adb -s "$phone_address" forward --no-rebind tcp:9222 localabstract:chrome_devtools_remote

node scripts/dev/android-snapshot.mjs --samples 8
node scripts/dev/android-snapshot.mjs --samples 5 --record-seconds 30
node scripts/dev/android-snapshot.mjs --samples 5 --record-seconds 30 --timer

# 이 작업에서 만든 9222 포워딩만 종료
adb -s "$phone_address" forward --remove tcp:9222
```

- `9222` 충돌: 기존 forwarding 보존, 다른 port 선택 후 `--cdp http://127.0.0.1:PORT`
- helper: 해당 origin의 기존 `/` 또는 `/index.html` 탭 1개만 page WebSocket으로 관측; 다른 탭·Chrome 재시작·navigation·권한 emulation 없음
- 기본 출력: 고유 `build/dev-android/snapshot-<UTC>.json`; 명시 `--output`도 기존 파일 덮어쓰기 금지
- `--record-seconds 1..60`: 실제 Worker 결과 최대1,500개, 전송 시도 IMU 최대12,000개; hook은 시간 상한·종료에서 복원
- `--timer`: 30초 numeric 기록 전용. Tracking 상태에서 시작, 화면 타이머 기준 0–10초 정지/10–20초 평소 Lost가 발생하는 정도의 이동·회전/20–30초 정지. source gyro를 함께 확인하여 실제 동작 구간 판단
- 결과별 `pipeline`: IMU/frontend/backend·Ceres/marginalization 시간, 유효 current visual/IMU factor·immature/depth/projection 거부 수. `candidatePipeline`은 motion guard reset 이전 시도의 별도 identity; 미실행·불일치 stage는 null
- `imuWaitBudgetMs`: 최근 수락 source cadence로 고정한40–80ms bracket deadline. 실제 future sample은 필수이며, 추가 대기는 최대40ms 지연 tradeoff가 있음
- numeric `submissions`와 결과의 sourceTiming·frameCopyMs·roundtripMs·uiStateUpdatedAtMs는 request/epoch/sequence로 결합. 마지막 snapshot cameraTiming을 개별 결과와 임의 결합하지 않음. UI 시각은 상태 갱신 완료이며 GPU 표시 완료·물리 exposure 시각이 아님
- 수집 부하: IMU row 복사, sourceTiming clone, hook·타이머 DOM 갱신 포함. 속도 비교는 같은 관측 설정으로 수행, raw image recorder와 numeric recorder 동시 사용 금지
- 기록: raw pose·timestamp·상태·epoch·solver·입력 IMU·camera 수치·gyro candidate/적용 여부; 영상·camera label/device ID·개인 탭 URL 제외
- 판정: IMU 호출 인자는 Worker 수락·엔진 입력과 구분; callback·presentation은 physical exposure 시각과 구분; 정지 자기일관성은 독립 GT 정확도와 구분
- 2026-10-07 연결 확인: Z Flip 6 `SM-F741N`, Android16/API36, Chrome154.0.8037.126, ADB36.0.2; 무선 shell·Chrome CDP 실제 호출 성공
- 실제 발산·입력 수정: [gyro 원본 보존](../../docs/fixes/2026-10-07-imu-bias.md), [FOV 오판 제거](../../docs/fixes/2026-10-07-focal-length.md), [모바일 이동 불연속 회복](../../docs/fixes/2026-10-07-mobile-motion-guard.md)
- 공식 절차: [Android ADB](https://developer.android.com/tools/adb), [Chrome Android CDP](https://developer.chrome.com/docs/devtools/remote-debugging/)

### Grayscale·IMU 입력 보존과 오프라인 재생

```bash
# 준비된 실제 기기에서 app 실행 후 시작. 원래 첫300장 제한 대신 PC로 계속 저장.
node scripts/dev/android-input-record.mjs --seconds 60 --frames 24 --timer \
  --output build/dev-android/new-controlled-minute

# 별도 검증된 loader를 지정; VIO 시작부터 저장된 전체 prefix 처리.
node scripts/dev/replay-android-input.mjs \
  --record build/dev-android/new-controlled-minute/record.json \
  --input-dir build/dev-android/new-controlled-minute/mav0/cam0/data \
  --loader build/zflip6-followup-wasm/vio_engine.js \
  --frames 2000 --output build/dev-android/new-controlled-minute-replay
```

- 기록 `--frames`: 총 저장 수가 아닌 동시 대기 이미지 수, 기본24·최대300; raw queue 최대16MiB
- PC 배출: 100ms 간격·최대4MiB batch; PGM과 timestamp/hash receipt 동기화 후 queue 해제
- 시작 시 estimator reset1회·전후 epoch 기록; 기존 다른 capture hook은 거부
- `acceptedFrameMetadata`: 모든 accepted 요청과 payload 상태; `frames`: 저장 확인된 이미지
- `completion`: missing payload·pending/foreign result·partial 구분; 포즈0·측정 종료를 정확도 통과로 해석 금지
- 설정/해상도 변경·overflow·연결/디스크 오류: partial 기록, 저장된 입력 보존; 복구 token과 `capture-start.json`, `received-frames.jsonl` 확인
- phone 복구 객체: 최종 `record.json` 동기화 완료 후 삭제; `cleanup.json`에 실제 삭제 결과
- 재생: 기본300·최대2,000장, 선택 raw 영상 합계512MiB 상한; PGM/hash·연속 frame·IMU endpoint 검증
- 재생 profile: `mobile_default`만 지원; solver/tracking RPC는 과거 app 기본값으로 복원한 설정임을 결과에 명시
- 결과: `report.json`의 loader/Worker/input hash, 실제 처리 수, 초기화·fresh pose 비율·reset·IMU drop 별도 기록
- desktop 순차 재생·미리 공급된 IMU bracket; 원래 전화 scheduling·capture latency·누락 영상·독립 GT 재현과 구분
- [Z Flip6 후속 진단·검증](../../docs/fixes/2026-10-07-zflip6-followup.md)

## 실제 HTTPS 7002 수동 운영

```bash
# 검증된 paired candidate 승격/부모 start flag 이후
bash scripts/ops/serve.sh start
bash scripts/ops/serve.sh status

# 소유 PID/명령/start ticks 검증 후 종료; 타 서비스에는 신호 없음
bash scripts/ops/serve.sh stop

# strict hostname/SNI + 기본 시스템 trust, localhost TCP 확인
curl -q --noproxy '*' --cacert /etc/ssl/certs/ca-certificates.crt \
  --resolve dev.serdic.com:7002:127.0.0.1 --fail --head https://dev.serdic.com:7002/
```

- 접속: `https://dev.serdic.com:7002/`, `/test-tumvi.html?dataset=room1|room4`, `/audit-mobile.html`; 실제 listener `0.0.0.0:7002`
- 기존 leaf + ordered intermediate2 + private key 사용; root 전송/자동 self-signed fallback 없음. SAN/keymatch/validity/system trust startup 검증
- TLS 파일 내용·ownership 보존, key600/cert644/keys directory750. 인증서 교체 후 보호 mode 확인·수동 restart
- PID record: `build/server/service.json`; exact repo/uid/exe/cwd/argv/start ticks 확인. 충돌/foreign/stale 기록은 자동 종료·덮어쓰기 없이 실패
- stale/dead 소유 record는 `stop`으로 archive 후 `start`; 포트/host 변경도 명시 `stop` 후 실행
- 로그: `logs/https/<UTC>-7002-<instance>.log`, 세션별 owner-only 새 파일, 1MiB 뒤 discard/drain. 기존 로그 truncate 없음; remote `/log`405
- health: `/__health__`의 active loader startup hash·`wasmMode`·server/source manifest hash·TLS metadata·PID 확인; 전체 NAS·private path/credentials 비공개
- room1 local fixture, room4 local stage만 공개. 선택 `MOBILE_SLAM_ROOM4_ROOT=/abs/checkout/local-room4/mav0`; symlink로 checkout 밖을 가리키면403
- 현재 candidate는 JS에 WASM을 embed한 SINGLE_FILE; flag `single_file_embedded_wasm:true`, `wasm_sha256:null`, 검증된 `js_sha256` 사용. 외부 `.wasm` 불필요, 기존 파일은 보존하되 direct/alias403·health active:false
- 향후 external paired mode는 `single_file_embedded_wasm:false` + 실제 JS/WASM 각각 hash 확인
- 시작 flag: `build/refactor-orchestration/server-start-authorized.json`의 authorized·active artifact hash·mode 필요. 실제 public7002는 fresh candidate 준비 후 실행
- 검증: `node --test tests/server/server.test.cjs`, `python3 -m unittest discover -s tests/server -p 'test_*.py' -v`
- localhost/LAN-host strict TLS 성공과 외부 internet/실제 phone 접근·센서 품질을 구분
- 현재 실행 기록(2026-10-07 01:48:26 KST): PID `3002812`, instance `c98f10df76144a3c80311b09d1e0d5cf`, deployment `refactor-final-20261007`, embedded loader `6,009,632 bytes` / SHA256 `9973d58eca6f95fcf5fcaf2d553d02bc2215f419e4ba1fc1cc94ae417eb3014d`; source manifest `b936982e5fea0c8ad0fb77e3e8b311770998763ff77ef2ef4bb082945866674b`
- 세션 로그: `logs/https/20261006T164826765565Z-7002-c98f10df76144a3c80311b09d1e0d5cf.log`; exact-owned PID2449945 종료 → 기존 helper atomic backup/promotion → 새 후보 시작1회. 이전 후보·session 로그·TLS 보존, 현재 상태는 `status`로 재확인. 상세 근거: [HTTPS/Ops 결과](../../docs/refactor/2026-10-06/server-lane.md)
- 같은 host의 기본 hostname은 기존 `/etc/hosts` development override로 `127.0.0.1` 연결. public 경로 근거는 `dig @1.1.1.1`의 A `183.98.179.103` 조회 후 해당 주소로 `--resolve` strict SNI 연결; 둘을 구분, ops에서 DNS/hosts 변경 없음

## MCP / skills / plugin

- `.codex/config.toml`: `mobile_slam_playwright`, `mobile_slam_context7`; 이 checkout의 절대 경로 사용, 이동 시 `cwd`/launcher 경로 변경
- [공식 Codex 설정](https://developers.openai.com/codex/config-basic): 신뢰된 프로젝트에서 project config 로드; 사용자 global config/credentials 수정 없음
- [공식 Codex MCP](https://developers.openai.com/codex/mcp): stdio launcher + startup/tool timeout 설정
- [공식 Playwright MCP](https://github.com/microsoft/playwright-mcp): headless + isolated profile, 시스템 Chrome, devtools 기능
- [공식 Context7](https://github.com/upstash/context7): keyless 실제 조회 확인; API key 선택 사항, 이미 보유한 키는 `CONTEXT7_API_KEY` 환경변수로만 전달
- `codex mcp get mobile_slam_playwright`, `codex mcp get mobile_slam_context7`: 설정 확인; 실제 서버 연결은 probe로 별도 확인
- 추가된 서버가 현재 세션 목록에 없으면 새 Codex 세션에서 project config 로드 확인
- `.agents/skills/mobile-slam-validation/SKILL.md`: [공식 repository skill 경로](https://developers.openai.com/codex/skills); 회귀/성능/정확도 증거 계약
- 외부 marketplace plugin 설치 대신 공식 stdio MCP + 프로젝트 스킬 직접 구성; 같은 기능의 중복 plugin 없음
- 사용 가능한 기존 Codex/OMX 도구는 유지. `omx_code_intel`의 TypeScript fallback 또는 `/usr/bin/sg`를 genuine C++ LSP/static analysis로 해석 금지
- `clang-tidy`, `clang-format`, `tsc`, `ast-grep`는 현재 미설치; 해당 lint/typecheck 통과 주장 금지


## Refactor build·replay·staging

- Shared source 목록: `cmake/SLAMCoreSources.cmake`; native viewer 기본 OFF, Pangolin 없는 headless `tiny_vins_mono --headless`/`native_replay`.
- Native: `cmake --preset native-dev && cmake --build --preset native-tests && ctest --preset native-tests`.
- 이번 candidate: `build/refactor-native`, `build/refactor-wasm`; 새 artifact 생성만으로 배포 완료 처리 금지.
- Native replay 정규 CMake target: `bash scripts/dev/native-replay.sh DATASET CONFIG OUTPUT MAX_FRAMES bracket PNP CV_THREADS ITERATIONS SOLVER_TIME DUMP_INPUTS`.
- Browser profile parity: tracking 21/3/min20/edge0 → LK criteria20/0.03, F threshold1, PnP 기본 OFF/freq3; summary에 실제 적용값 기록.
- Fixed pilot: `ITERATIONS=10 SOLVER_TIME=10`, OpenCV threads1/seed0; replay timing은 capture latency와 구분.
- Profiling 재빌드 생략: 검증된 build만 `MOBILE_SLAM_SKIP_BUILD=1`; 실행 중 source/build 변경 없이 순차 측정.
- NAS room4 staging: `python3 scripts/dev/stage-tum-room4.py`; `/mnt/backup/SLAM` read-only, `build/refactor-data/tum`만 local 공개 가능. Convenience DSO image symlinks는 추출 제외·manifest 기록.
- Native/Worker parity: `scripts/dev/compare-replay.py --freeze-pilot NATIVE_JSON WORKER_JSON`을 3쌍 + `--output CONTRACT`로 사전 independent caps 내부 tolerance 동결; room4 이후 재동결 금지. 비교는 `--native --browser --expected-frames --contract --output`.
- Source/flags/dependency/artifact manifest: `python3 scripts/dev/build-provenance.py --build BUILD --artifact ARTIFACT --output MANIFEST`.
- 승격: `python3 scripts/dev/deploy-candidate.py --candidate JS --manifest MANIFEST`; parent의 `build/refactor-orchestration/deploy-authorized.json`에서 `authorized: true`, `candidate_sha256` 일치 필수. Single-file embedded WASM만 원자적 교체, 이전 JS/manifest는 `build/refactor-deploy/` 보존.
- ASan/UBSan은 별도 fresh build 전체 core/dependency instrumentation; 오래된 unsanitized core objects를 link하여 통과 판정 금지.
- 단위 테스트·calibrated dataset·HTTPS browser 검증으로 physical phone stability/full SLAM/production claim 금지.

- Candidate isolated browser API smoke: `node scripts/dev/wasm-smoke.mjs build/refactor-wasm/vio_engine.js`; loader/mandatory bindings/configure/epoch/reset/seed/thread만 검증, tracking·accuracy와 분리.
- Native raw input oracle: replay 마지막 `DUMP_INPUTS=1` → `input-oracle.bin`; `python3 scripts/dev/hash-inputs.py --raw FILE --dataset DATASET --config CONFIG --output MANIFEST`로 native OpenCV decoded gray와 exact Float64 LE IMU packets SHA 기록. Hash I/O는 pilot 검증용, paced latency 실행에서는 비활성.
