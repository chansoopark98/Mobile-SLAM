# Galaxy Z Flip 6 상용 SDK 실기기 검증

- 작업일: **2026-10-14, Asia/Seoul**. 사용자 무선 디버깅 활성화 및 분석 앱 설치·실행 요청에 따른 후속 작업.
- 완료: 무선 ADB 연결, 실제 ARM64 service 함수 분석, 별도 앱 빌드·설치·무수집 준비 점검, 사용자 시작 신호 이후 **30초 실측과 저장 MP4 재생2회**, 원본 PC 보존·시각/추적/재생 분석.
- 사용자 승인: **“영상·IMU·ARCore MP4 저장 허용, 측정 시작”** 수신 후 CAMERA 권한을 부여하고 시작. [승인·조건 기록](../../build/physical-arcore-20261014/live-01/authorization.json). 데이터는 로컬 보관.
- 실측: **607frame 중601 TRACKING(99.0115%)**, 첫 tracking+0.248706s, 초기화 이후 손실0. 독립 GT·측정 calibration·동일 입력의 자체 엔진 비교가 없으므로 physical accuracy·상용 동급 수락을 의미하지 않음.
- 이전 결과: [상용 SLAM 통합 보고서](2026-10-14-commercial-slam-lab.md), [ARCore emulator 실패 근거](2026-10-13-arcore-runtime-verification.md).

## 기기·설치 소프트웨어

- 실제 연결: Galaxy Z Flip 6 **SM-F741N**, Android16/API36, `arm64-v8a`. 기존 ARCore **1.56.262080393**, versionCode262080393.
- `adb mdns services`로 현재 endpoint 탐색 후 `adb connect` 실제 성공. endpoint는 세션마다 바뀔 수 있으며 재현 시 다시 조회.
- [intake manifest](../../build/physical-arcore-20261014/intake/manifest.json): 모델·API·ABI·ARCore package path·분석 package 부재·초기 git status. 카메라/IMU 수집 없이 targeted metadata만 확인.
- 분석 package **`com.mobileslam.reference.arcore`** 새 설치. 기존 기기 앱·ARCore service 보존, 기존 emulator x86 APK 설치 없음.

## 실제 설치·시작 차단·빌드 검증

- 실제 APK: [reference-physical.apk](../../build/physical-arcore-20261014/app-v2/reference-physical.apk), **4,378,399B**, SHA256 `a3e728150d160b5b914841932f0e98b0d0b1a75f46fd64daf74aa273a72bbc9b`.
- signer SHA256 `b07bc3eda189173d10a8d9b7f1afd008188e2bb7ef7fd071a3ce43fd344790c0`. ARM64 only, launcher `PreflightActivity`, capture `HelloArActivity` non-exported, self-target instrumentation.
- official sample source commit `3abfeb18669c2cbb2d07057f135d117ee9d91826`; 별도 source snapshot74파일, APK/Java/build source hash 일치. Gradle assembleDebug **34tasks/9s/exit0**, jobs2·SDK/cache 원본 read-only. [build verification](../../build/physical-arcore-20261014/app-v2/build-verification.json).
- 실제 `adb install -t --no-streaming` 성공. 설치 직후 CAMERA `granted=false`·appops `ignore`·app files 디렉터리 부재 확인. [preflight 명령](../../build/physical-arcore-20261014/phone-preflight-v1/commands.json).
- 설치 APK의 token 누락/오류·mode 누락·save flag 누락·기존 runId·경로 탈출 runId·31초 초과·CAMERA 미허용 **8건 거부**, 수집 파일0. [guard verification](../../build/physical-arcore-20261014/phone-guards-v1/verification.json). `adb am instrument` 외부 exit0도 내부 rejected일 수 있어 문자열·결과 코드를 검사.
- Android lint: **0errors/16warnings/exit0**, APK·소스·키 hash 불변. [lint 결과](../../build/physical-arcore-20261014/app-lint-v2-online/lint-results-debug.txt). 공식 sample12·기존 API35/observer2·ARM64/high-sampling 설정2개. DefaultLocale PGM 이름 경고는 남기며 현재 실제 PGM 이름·hash는 검증.
- 최초 offline lint는 캐시의 `com.android.tools.lint:lint-gradle:31.6.1` 부재로 실패. [실패 보존](../../build/physical-arcore-20261014/app-lint-v2/result.json) 후 기존 Google/Maven 저장소에서 개발 도구만 확보해 완료; APK·ARCore dependency version 변경 없음.

## 실제 ARM64 바이너리 역분석

- 입력: 기기에 설치된 `com.google.ar.core`의 base APK만 ADB pull. 개인 영상·센서 기록·다른 앱 데이터 수집 없음.
- APK: **50,954,278B**, SHA256 `905d63c2e97cef8df19c1621b517dea9b8775183916f16cb65ab64ba9fe5e3ff`.
- `lib/arm64-v8a/libarcore_c.so`: **35,801,624B**, SHA256 `7c32d45c2ef1ebaf92deae86a7ba31b0dd00d39bde09ab0890535ff396d62050`.
- 도구: 기존 radare2 6.2.4, GNU readelf 및 `aarch64-linux-gnu-objdump`. ELF wrapper에 `--objdump` 선택을 추가해 실제 ARM64 명령을 교차 검사.
- 실행 **18명령 모두 exit0**, 입력·도구·분석 script snapshot·stdout/stderr SHA 검증. [획득 manifest](../../build/physical-arcore-20261014/device-service/manifest.json), [함수 summary](../../build/physical-arcore-20261014/device-service/functions/summary.json), [독립 명령 계수 검증](../../build/physical-arcore-20261014/device-service/verification.json).

| 함수 | symbol bytes | radare blocks | radare/GNU 명령 수 | radare callsites |
|---|---:|---:|---:|---:|
| `ArSession_update` | 1,016 | 18 | 254/254 | 34 |
| `ArFrame_getTimestamp` | 572 | 18 | 143/143 | 24 |
| `ArCamera_getTrackingState` | 1,084 | 22 | 271/271 | 37 |
| `ArCamera_getTrackingFailureReason` | 1,084 | 21 | 271/271 | 37 |

- 각 함수는 ARM64 고정4B 명령 수×4와 symbol 크기 일치. 이는 선택 entrypoint의 명령 inventory 검증이며, private dispatch·solver 수식·상태 threshold의 완전 복원 증거는 아님.
- 기존 emulator x86_64 service47,269,520B와 다른 실제 phone ARM64 배포임을 확인. 바이너리 크기·명령 수로 tracking 성능을 비교하지 않음.

```bash
python3 scripts/dev/references/re-lab/analyze-elf.py \
  --input build/physical-arcore-20261014/device-service/base.apk \
  --apk-member lib/arm64-v8a/libarcore_c.so \
  --output build/physical-arcore-20261014/device-service/functions-next \
  --objdump aarch64-linux-gnu-objdump \
  --export ArSession_update --export ArFrame_getTimestamp \
  --export ArCamera_getTrackingState --export ArCamera_getTrackingFailureReason
```

## 실기기 실행 경계

- 기본 launcher는 준비 화면만 표시: CAMERA 권한 요청·Session 생성·SensorManager 등록·녹화 파일 생성 없음.
- 실제 측정 진입: owned self-target Android Instrumentation → non-exported capture Activity. token·mode·30초 이하 duration·고유 runId·각 저장 flag 명시 필수.
- PGM 영상·외부 IMU·SDK MP4는 저장 범위를 각각 적용. MP4 자체가 영상 기록이므로 PGM 저장 여부와 독립적으로 제어.
- `측정 시작` 이후 CAMERA 권한 부여·측정 실행. 첫 프레임과 무관한 monotonic watchdog, pause/error/timeout 종료·부분 기록 보존.
- [공식 recording/playback 계약](https://developers.google.com/ar/develop/java/recording-and-playback/developer-guide): MP4에는 영상·AR 데이터 포함; 재생마다 pose/trackable 결과가 달라질 수 있음. 반복 재생은 입력 hash·timestamp 대응과 pose 차이를 평가하고 bitwise pose 동일성을 요구하지 않음.
- 무수집 준비 확인 → 현재 저장 범위 및 시작 신호 → bounded capture → 필요 시 승인된 MP4 재생 → timing/tracking 비교 순서. 독립 GT가 없으므로 physical ATE/RPE 정확도 수락은 별도 미검증.

## 실측·재생 결과

| 실행 | 고유 frame / TRACKING | 초기 tracking 지연¹ | 이후 손실 | PGM / 외부 IMU | 종료·Session close |
|---|---:|---:|---:|---:|---|
| live30초 | 607/601,99.0115% | 0.248706s | 0 | 606/27,062 | duration_limit30.235s /30.480s |
| playback01 | 616/606,98.3766% | 0.448571s | 0 | 615/0 | playback_finished29.829s /29.947s |
| playback02 | 609/599,98.3580% | 0.448571s | 0 | 608/0 | playback_finished29.771s /29.853s |

- ¹ 각 실행에서 첫 양수 frame timestamp부터 첫 TRACKING까지. 앱 실행·sensor exposure부터의 전체 initialization latency가 아님.
- live source timestamp span **28.227617s**, 관측 약21.4683Hz. camera interval p50/p95 **33.315/66.633ms**, 최대100.000ms. 영상 저장·sample 렌더링 부하가 포함된 관측률이며 내부 solver 처리량/최대 FPS가 아님.
- live `Session.update` 호출 p50/p95/p99 **3.519739/4.480568/4.952851ms**. blocking API·IPC 경계 시간이며 VIO solver 또는 capture-to-pose latency가 아님.
- 외부 SensorManager: accelerometer **13,532**, gyro **13,530**, 각각 약461Hz, timestamp 유일·단조 증가. ARCore 내부에 투입된 동일 IMU packet이라는 증거는 없음.
- public pose: translation↔matrix 차0, quaternion/rotation/homogeneous 계약 검증. pointcloud count 중앙85/최대203. 이는 유효한 수치 출력·tracking 관측이며 실제 위치 오차 GT 검증이 아님.
- **CPU image timestamp−AR frame timestamp** 중앙 **+1.994077ms**, 범위 **−2.943422…+2.799020ms**. 저장 영상 시각과 pose frame 시각을 독립 보존; 임의 equality·timebase 동일성 가정 금지.
- SDK setup/cleanup error0,3회 모두 `sessionClosed=true`, host 강제 timeout0. 앱 timer는30초 종료 요청이고 실제 close는 위 값; hard30초 release 보장으로 표기하지 않음. 소유 package만 host fallback 대상으로 지정.
- [실측 summary](../../build/physical-arcore-20261014/live-01/summary.json), [재생1](../../build/physical-arcore-20261014/playback-01/summary.json), [재생2](../../build/physical-arcore-20261014/playback-02/summary.json), [전체 상세 분석](2026-10-14-physical-arcore-analysis.md).

## 데이터 보존·반복 재생

- 원본 PC 위치: `build/physical-arcore-20261014/captures/{physical-live-01,physical-playback-01,physical-playback-02}/`. app private 원본도 보존. 각 archive/member SHA와 file count 검증.
- 장기 보관: [`Documents/Mobile-SLAM-recordings/2026-10-14-ZFlip6-ARCore/`](/home/park-ubuntu/Documents/Mobile-SLAM-recordings/2026-10-14-ZFlip6-ARCore/). 원본 tar3개·manifest·분석·승인·APK/source snapshot 등14개 artifact, **609,538,539B**, 원본과 SHA 일치. [archive receipt](../../build/physical-arcore-20261014/archive-receipt.json).
- live **611files/224,286,345B**, playback01 **618files/189,590,415B**, playback02 **611files/187,432,473B**. tar는 별도 보존; dataset/기존 recording 덮어쓰기 없음.
- MP4 **31,422,921B**, SHA256 `8a822bdd97a09ebdad0986469493d02407241304d4ce2be90c2d4a4b33adaea0`. 폰의 재생 전·후와 PC 복사본 hash 동일. [MP4 identity](../../build/physical-arcore-20261014/live-01/mp4-identity.json).
- MP4 decoder metadata: H.264·640×480·860video frames·표시 frame rate29.33575. 이는 container/decoder 관측이며 SDK public update607개와 다른 분모. ffprobe 미설치 실패를 보존하고 기존 OpenCV4.12/FFMPEG backend로 조회. [metadata](../../build/physical-arcore-20261014/live-01/mp4-metadata-opencv.json).
- live↔playback01: CPU image timestamp 교집합453개, PGM byte 동일 **0/453**. MP4 저장·재생 경로가 원본 gray byte를 보존한다고 가정할 수 없음. 압축/변환이 후보이나 원인 분리 실험은 수행하지 않음.
- playback01↔02: CPU image timestamp 교집합452개, PGM byte 동일 **452/452**. 양쪽 TRACKING443개에서 정렬 없는 pose 차 중앙 **0.009095m/0.104142°**. world gauge·image/frame 시각차·재생 비결정성을 포함하므로 ATE/절대 정확도가 아님.
- 두 playback은 `saveImu=false/saveArcoreRecording=false`, 새 IMU/MP4 생성0. CAMERA appops의 마지막 사용 시각·duration은 live 값을 유지. 역사적 센서/영상이 담긴 같은 MP4의 반복 실행임을 input hash와 함께 확인.

## 현재 프로젝트와의 차이·적용 판단

| 증거 | 현재 프로젝트에 대한 판단 |
|---|---|
| Emulator tracking0·camera interval0.100003ms 대 physical601/607·interval33.315ms | 기존 emulator 실패를 ARCore 실기기 실패로 일반화할 수 없음. 가상 camera/clock 경로의 결함 가능성을 분리해야 함 |
| Native 외부 IMU 약461Hz | 기존 Flip6 browser DeviceMotion 약42.5Hz 기록과 수집 경로가 다름. 동일 시간·동일 내부 packet의 A/B가 아니며 단순 rate 우열만으로 tracking 차이 원인 확정 불가 |
| 실제 CPU image와 AR frame 시각 약2ms 차 | 자체 requestId·input timestamp·pose timestamp·sourceTiming 계약을 유지하고, camera exposure/이미지 시각을 분리한 replay 필요 |
| 재생별 update616/609와 동일 영상452개 | raw callback 수·정상 tracking flag·실제 고유 영상 coverage 구분. 상용 엔진에서도 출력 pose를 결정론적 GT로 사용 불가 |
| 실측 초기화 이후 loss0 | 이번30초 경로의 관측 결과. 이전 Mobile-SLAM의 다른 영상/움직임/보정 조건과 직접 비교해 개선율·정확도 동급을 주장할 수 없음 |

- 이번 작업은 **실기기 설치·SDK 실행·기록/재생·입력/상태 분석 완료**. 원시 기록을 확보했지만 camera/body extrinsic·동일 preprocessing/IMU·독립 GT가 없어 Mobile-SLAM 동일 조건 정확도 비교는 후속 실험.
- private solver 수식·threshold 전체 복원, ARKit/Vuforia 실기기 실행, 장시간 thermal/battery·held-out 경로 정확도는 미완료. 도구 실행과 실제 물리 검증의 완료 범위를 분리.

## 도구 수정·최종 검증

- 코드: [physical 앱 빌드·실행](../../scripts/dev/references/arcore/physical/README.md), [실측 분석기](../../scripts/dev/references/arcore/analyze_physical.py), ELF cross-objdump 선택. 제품 estimator·served WASM 변경0.
- 단순화: 공식 sample·기존 public observer·기존 stats/compare 재사용. 불필요한 abstraction/cosmetic 변경 없음.
- cleanup에서 발견한2건 수정: host의 nonzero adb 종료를 성공으로 처리하지 않음; object가 아닌 JSON/JSONL도 빈 output 대신 명시 `validation:FAIL`을 보존. red/green 및 원래 실패 evidence 보존.
- 검사: actual guard8개, host runner 회귀3개, 분석기 회귀8개(신규20개 하위 경우 포함), 실제3run manifest/PGM/pose/lifecycle 검증, static18명령, Android build/lint. [최종3run 분석](../../build/physical-arcore-20261014/analysis/all-three-verified.json).
- 독립 architect가3개 capture를 다시 읽어 검증한 값이 최종 JSON과 전부 일치. cleanup pass3도 통과. [cleanup 검증](../../build/physical-arcore-20261014/cleanup-verification-pass3.json).
- 마지막 host exit gate 변경은 unit·실제 기존-run 거절로 검증; phone APK/Java는 v2 frozen artifact이며 실제3회 결과는 각 실행의 source snapshot을 기준으로 보존.
- 종료 상태: 분석 앱 force-stop 후 CAMERA 권한 회수, 무수집 준비 화면 표시. 분석 앱·원본 recordings는 유지. 기존 ARCore version 불변. [device-final verification](../../build/physical-arcore-20261014/device-final/verification.json).
- commit/push/deploy 없음. 추정 알고리즘 개선·실제 phone 정확도 달성을 이번 연구 완료로 대체하지 않음.
