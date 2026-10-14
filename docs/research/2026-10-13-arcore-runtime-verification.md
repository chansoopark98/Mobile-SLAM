# ARCore SDK 독립 빌드·가상 실행 검증

- 완료 확인: **2026-10-14, Asia/Seoul**. 경로의 `20261013`은 작업 시작일.
- 결과: 공식 HelloAR sample을 새 출력에서 빌드·설치하고 **실제 ARCore 1.56.0 `Session.update`·camera/image·recording·playback API 호출** 완료. 계약 fixture 실행과 구분.
- 최종 `isolated-v3-motion`: live30, record30, playback26/30 frame. **모든 모드 TRACKING=0, 추적 수락 실패, 정확도=null**.
- 실패 보존: v1 AVD 경로 오류, v2 timeout·sample renderer GL 오류·camera timestamp 이상. v2의 일회성 TRACKING1 frame을 연속 추적 성공으로 계산하지 않음.
- 범위: 개발용 runner·analyzer와 이 문서. Mobile-SLAM 제품 알고리즘·기본 설정·served WASM 변경 없음. 상용 solver 내부 수식 복원·동일 입력의 Mobile-SLAM A/B 정확도 비교는 미수행.

## 독립 환경과 source/artifact

- 기존 `assets/references/ARCore/`와 SDK를 Docker **read-only mount**. 기존 `mobile-slam-arcore-20261012` 컨테이너·AVD·Gradle cache를 변경하지 않음.
- 새 출력: `build/commercial-slam-20261013/arcore/isolated-v{1,2}/`, `isolated-v3-motion/`. 각 실행에 source snapshot·새 Gradle cache·새 AVD·새 container 생성. host ADB·실제 휴대전화 사용 없음.
- container image: `sha256:f916020d2978b27752fdf583cefb724a488bd98fb799f7615b0cb7728862388f`. 신규 container는 `--cpus=4 --memory=8g --shm-size=1g`; emulator4cores/2GiB, build2workers.
- 환경: Temurin21.0.12.1, Gradle8.7/AGP8.6.1, compile/target API35, API30 Google APIs x86_64 image revision16, Android Emulator37.2.12.0/build16428233, adb37.0.1.
- 새 AVD: back camera `virtualscene`, front `none`,480×800 display, SwiftShader, `-feature -Vulkan`, KVM, `-no-window -no-snapshot`. CPU camera image는 **640×480**, 실제 SDK focal480px/principal point(320,240).
- upstream source commit: `3abfeb18669c2cbb2d07057f135d117ee9d91826`. 기존 Apache-2.0 sample의 API35/Gradle 호환 profile·공개 SDK observer를 복사한 뒤, pose/IMU `PrintWriter`에 **autoflush만 추가**해 timeout 이전 관측 보존. proprietary binary 변경 없음.
- v3 offline `assembleDebug`: **34 tasks executed, BUILD SUCCESSFUL**. 기존 APK 재설치만으로 새 빌드를 주장하지 않음.
- [source manifest](../../build/commercial-slam-20261013/arcore/isolated-v3-motion/source-manifest.json): sample source별 SHA256·upstream commit. [artifact manifest](../../build/commercial-slam-20261013/arcore/artifact-manifest.json): runner snapshot·APK·MP4·analysis·emulator log SHA256와 종료 상태.

| 자산 | bytes / SHA256 |
|---|---|
| ARCore emulator service APK1.56.0 | `7cbeb1bba836ab69551ac7c7ae64ed6e94f0b2818fc921b83d5edd57f51f4066` |
| v3 새 HelloAR APK | 4,886,595B / `6aaa4c6ce1d247745bfab63380be8de8ea777e30613aa9dae4d0c6b6263b45e4` |
| v3 ARCore recording MP4 | 1,238,271B / `a5a46c0037cee78a70899815051500b38e692426f2ccad681e3639de31f315d7` |
| v3 실행 runner snapshot | `80c153330e09c2719567bec81cb7f02fb1c008a0f4fc1a30edfb4dcce47b7467` |

## 실제 SDK 결과

- 실행 argv·exit·duration·log SHA256: [v2 commands37개](../../build/commercial-slam-20261013/arcore/isolated-v2/commands.json), [v3 commands58개](../../build/commercial-slam-20261013/arcore/isolated-v3-motion/commands.json).
- 아래 row는 sample의 `session.update()` 반환 뒤 `Frame.getTimestamp`, `Camera.getPose/getTrackingState/getTrackingFailureReason`, intrinsics·pointcloud·CPU image를 **실제 SDK에서 읽은 값**.
- `Frame`/image 시각·pose translation/quaternion/column-major `T_W_C_OpenGL`·blocking update API 시간·SensorManager 관측을 JSONL로 보존. Camera state `PAUSED/NONE`도 그대로 기록.

| v3 mode | 실제 frame row | TRACKING | 검증한 PGM hash | 완료 조건 |
|---|---:|---:|---:|---|
| live + 가상 이동 | 30 | 0 | 30/30 | frame_limit |
| record + 가상 이동 | 30 | 0 | 30/30 | frame_limit, `startRecording` 후 `stopRecording`, MP4 생성 |
| playback1 | 26 | 0 | 26/26 | 실제 `PlaybackStatus.FINISHED` |
| playback2 | 30 | 0 | 30/30 | frame_limit, 마지막 status=OK |

- [v3 analysis](../../build/commercial-slam-20261013/arcore/isolated-v3-motion/analysis.json): 모든 실제 summary의 frame수와 JSONL row수 일치, image hash 실패0, malformed JSONL0.
- playback 두 번은 같은 private recording URI 사용. 정확히 공통인 timestamp **22개**에서 CPU image payload hash **22/22 동일**, state mismatch0, tracking intersection0. playback별 in-device MP4 hash를 전후로 재측정하지 않았으므로 완전한 입력 동일성 증명으로 확대하지 않음.
- live IMU: accel403/gyro406 row. record IMU: accel531/gyro536 row. **외부 SensorManager 관측이 ARCore 내부에 쓰인 동일 packet이라는 증거 없음**. playback IMU는 ARCore MP4 내부이며 외부 row0; live IMU를 recorded IMU로 대체하지 않음.
- 가상 이동: 기존 gRPC1.78.0 도구로 emulator public physical model만 제어. mode별24steps/12초+1초, position x±0.15m·z0…−0.20m, yaw±15°. [live readback](../../build/commercial-slam-20261013/arcore/isolated-v3-motion/live-motion.log), [record readback](../../build/commercial-slam-20261013/arcore/isolated-v3-motion/record-motion.log)에 actual value·status0 보존.
- 30frame limit가 이동 명령 전체보다 먼저 끝남. 따라서 12초 전체 경로를 SLAM이 처리했다고 주장하지 않음. 가상 physical-model request/readback은 독립 trajectory GT·camera/body extrinsic 검증 자료가 아님.

## 실패 진단과 제한

- **v1**: sample build 성공 후 AVD 설정 경로가 틀려 exit1. `avdmanager`는 `ANDROID_USER_HOME/avd`에 생성했으나 runner가 다른 경로 조회. 새 v2에서 AVD home을 명시 수정; [실패 로그](../../build/commercial-slam-20261013/arcore/isolated-v1/avd-config.log) 보존.
- **v2 live**: 60초 limit에서 완료 summary 없음.48frame/48images, TRACKING0, pointcloud0. source timestamp span0.117880272s에 관측 arrival span5.403470391s; camera interval p50 **0.066668ms**. camera/IMU 시간 동작의 이상 징후이며 특정 SDK 내부 버그의 인과 증명은 아님.
- **v2 record**: `RecordingStatus.OK` 실제 반환,8frame/8images 중 TRACKING1 이후 sample renderer **`glDrawArrays: invalid operation (1282)`** fatal. 완료 summary·정상 종료 MP4 없음. [record logcat](../../build/commercial-slam-20261013/arcore/isolated-v2/record-logcat.log) 보존.
- camera HAL: 음수/non-increasing timestamp와 `Capture failed`. ARCore native log: **`Last visual features at: 0 ns`**, IMU buffer overflow. 공개 로그의 관측이며 solver 구현을 복구했다는 근거가 아님.
- **v3도 시간 이상 지속**: record camera interval p50 **0.100003ms**; frame/image timestamp 차 최대159.766666ms. 실제 row에 값을 각각 보존하고 같은 시각이라고 임의 강제하지 않음. 모든 pointcloud=0, TRACKING=0.
- `Session.update` p50은 v3 live1.07832ms/record3.425655ms/playback11.811514·23.153853ms. **BLOCKING API wait·IPC 등을 포함한 호출 시간**이며 private VIO solver·capture-to-pose latency가 아님. image 저장/autoflush/SwiftShader 부하도 있어 Mobile-SLAM 속도 비교에 사용하지 않음.
- 저비용 호환성 진단: `emulator -version`은 GUI library `libxkbfile.so.1` 부재 오류였으나 `emulator -no-window -version`은 **exit0/37.2.12**. [독립 probe](../../build/commercial-slam-20261013/arcore/emulator-version-headless-command.json). 실제 headless emulator boot 성공과 별개로 기존 combined version command가 오류를 숨긴 문제를 수정.
- 공식 [ARCore emulator 절차](https://developers.google.com/ar/develop/java/emulator)·[record/playback guide](https://developers.google.com/ar/develop/java/recording-and-playback/developer-guide)·[Emulator archive](https://developer.android.com/studio/emulator_archive) 확인. Context7 snippet의1.54.0 예시를 설치 version으로 오인하지 않고 실제1.56.0 source/APK/build를 기준으로 기록.
- 구버전 emulator binary download/전체 SDK 교체·실제 휴대전화 재시험은 하지 않음. 현재 emulator/image/renderer 조합의 실패를 ARCore 실기기 tracking 성능으로 일반화하지 않음.

## 현재 프로젝트와 비교

| 직접 관측한 경계 | 현재 Mobile-SLAM과 비교·적용 판단 |
|---|---|
| SDK package/client와 별도 AR service가 실제 Session 처리 | browser WASM 자체 solver와 배치·프로세스 구조가 다름. SDK stub disassembly를 전체 VIO core 분석으로 보고하지 않음 |
| public Camera state·failure reason·Frame 시각·CPU image 시각을 각각 반환 | 자체 request/epoch/pose·input timing identity를 별도 보존하는 검증 필요. SDK에서도 이 가상 실행의 frame/image 시각 차가 관측되어 무조건 equality를 가정하지 않음 |
| SDK recording/playback API와 실제 MP4 output | Mobile-SLAM numeric observation과 완전 camera/IMU replay를 구분. 이 MP4 내부 proprietary sensor packets를 자체 engine 입력으로 해석·변환하지 못했음 |
| 반복 playback의 공통22개 image bytes 동일, tracking0 | 입력 영상 반복성 검증과 estimator 추적/정확도 수락은 분리. 현재 자료로 ARCore 대 Mobile-SLAM ATE·RPE 또는 회복 우열 판단 불가 |

## 재현·wrapper gate

```bash
# 기존 output/container 이름은 재사용 금지; 아래는 새 실행 예시
python3 scripts/dev/references/arcore/run_isolated.py \
  --output build/commercial-slam-20261013/arcore/isolated-next \
  --container mobile-slam-arcore-20261013-isolated-next \
  --frames 30 --timeout 45 --motion
python3 scripts/dev/references/arcore/analyze_isolated.py \
  build/commercial-slam-20261013/arcore/isolated-next
python3 -m py_compile scripts/dev/references/arcore/run_isolated.py \
  scripts/dev/references/arcore/analyze_isolated.py
git diff --check -- scripts/dev/references/arcore/ \
  docs/research/2026-10-13-arcore-runtime-verification.md
```

- 새 runner는 SDK/reference를 읽기 전용으로 사용하고 새 container만 종료. 기존 이름이면 source copy·start/stop 전에 거부.
- v2/v3 실행 당시 snapshot의 CLI는 timeout에도 exit0일 수 있었음. **exit0 자체를 통과 근거로 사용하지 않고 실제 summary/row/로그로 판정**.
- 최종 source는4모드 모두 setupError 없음·nonzero frame·완료 summary가 있어야 exit0; timeout/누락/zero frame/setupError는 exit1. 이 gate는 SDK 실행 완료이며 TRACKING·정확도 수락을 포함하지 않음.
- 통합 검토에서 analyzer의 기존 `analysis.json` 덮어쓰기를 제거. 재분석은 새 `--output` 필요; v3 원본 summary SHA 보존·새 `analysis-verified.json` 내용 동일 확인. 이 postprocessing은 새 SDK 실행으로 계산하지 않음.
- [최종 runner 확인](../../build/commercial-slam-20261013/arcore/current-runner-checks-v2.json): 실제 v2/v3 summary를 포함한 completion 판정5사례 통과. 기존 container 이름 실제 실행은 exit1, 전후 inspect SHA256 동일. source에는 최종 gate/version-probe 수정이 추가되었으며 **그 최종 source의 전체 Android sequence를 다시 실행하지는 않음**.
- 종료: 새 v1/v2/v3 container 모두 stopped, AVD·sample·로그 보존. `docker stop -t10`으로 sleep PID1이 종료되어 새 container exit137인 것은 wrapper cleanup 결과; 이전 container exit137 원인은 이번 작업에서 확정하지 않음.
- 미검증: 추적 수렴·연속 coverage·실기기 thermal/latency, calibration/body frame/physical exposure, independent GT·SE(3) scale1 ATE/RPE, proprietary solver semantics, 동일 image/IMU의 Mobile-SLAM A/B.
