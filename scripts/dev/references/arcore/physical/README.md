# 물리 ARCore 앱 빌드

- 범위: 기존 public HelloAR sample을 별도 출력으로 복사·패치·오프라인 빌드. ARCore service·제품 solver·원본 sample 변경 없음.
- 빌드: `python3 scripts/dev/references/arcore/physical/build.py --output build/physical-arcore-20261014/app-<unique>`
- 갱신 빌드: 위 명령에 `--keystore build/physical-arcore-20261014/app-v1/debug.keystore` 추가. 출력 폴더가 이미 있으면 거절; 이전 실패·서명 키 보존.
- 필요 자산: pinned `mobile-slam-arcore-env:20261012` image ID, `assets/references/ARCore/toolchain/{sdk,gradle}`, 기존 patched HelloAR source. 컨테이너 네트워크 비활성·CPU2·Gradle workers2. 원본 SDK/cache read-only, cache 복사본만 쓰기.
- 산출물: `reference-physical.apk`, `apk-sha256.txt`, `apk-signature.txt`, `apk-{badging,manifest}.txt`, `source-snapshot.tar.gz`, `source-manifest.json`, `build-command.json`, `build.log`, `start-token.txt`.
- 앱 ID: `com.mobileslam.reference.arcore`; ARM64 only. Launcher `com.google.ar.core.examples.java.helloar.PreflightActivity`는 텍스트만 표시. `HelloArActivity`는 non-exported.
- 측정 진입: self-target Android Instrumentation `com.google.ar.core.examples.java.helloar.MeasurementInstrumentation`. 빌드 스크립트 자체는 ADB·설치·실행하지 않음.
- 필수 인자: `token`, `mode=live|playback`, `durationSeconds=1..30`, `runId=[A-Za-z0-9][A-Za-z0-9_-]{0,63}`, `saveImages=true|false`, `saveImu=true|false`, `saveArcoreRecording=true|false`. 자동 기본 모드/저장 플래그 없음.
- 토큰은 실수로 시작하는 것을 막는 빌드별 값. 보안 인증 기능 아님. 유효 인자라도 CAMERA가 미리 허용되지 않으면 거절. 앱 내부 CAMERA 요청·권한 허용·AR 서비스 설치/업데이트 요청 없음.
- 실제 측정은 현재 데이터 저장 범위 및 사용자의 측정 시작 cue 이후에만 root가 실행. 설치 시 `-g` 사용 금지. MP4는 이미지/IMU와 독립된 명시 승인·플래그 필요.
- run 경로: 앱 private `files/reference-runs/<runId>/`. 기존 runId 거절. `poses.jsonl`은 자동 flush. 선택 저장: `images/*.pgm`, `imu.jsonl`, `recording.mp4`. 종료 시 `summary.json`, `lifecycle.json`.
- playback은 `sourceRunId` 필수, 같은 앱이 소유한 이전 run의 `recording.mp4`만 사용. playback에서 saveImu 또는 saveArcoreRecording=true 거절. playback PGM은 기록된 영상; 새 카메라 입력이라는 주장 금지.
- watchdog은 첫 frame 이전부터 monotonic timer로 시작; 기한 도달 시 observation/sensor 저장 중단 요청. UI thread가 GL을 정지한 뒤 recording·Session을 닫음. Native/UI hang 시 자원 해제가 지연될 수 있으므로 lifecycle elapsed·phone appops로 실제 종료 확인 필요.
- 검증 구분: Gradle 컴파일·APK manifest/signature 점검은 build evidence. 설치 후 잘못된 token/mode/flags/runId 거절·CAMERA 미허용 preflight는 root의 물리기기 evidence. 실제 tracking·정확도는 별도이며 independent GT 없음.

## 감독 실행·결과 분석

- [실제 설치·측정·재생 결과](../../../../../docs/research/2026-10-14-physical-arcore.md).
- `run.py`: 명시적인 현재 승인 이후에만 호출. 권한을 자동 부여하지 않음. host는 duration+6초에 소유 package만 force-stop하고 모든 원본을 보존; 실제 elapsed·lifecycle·appops 확인.
- CLI exit0은 현재 instrumentation 성공·nonzero frame·요청 flags·Session close 조건. tracking/정확도 수락과 별도. 기존 run 거절 후 남아 있는 과거 summary로 성공 판정하지 않음.
- 아래 예시는 **이미 승인·수집된 recording 재생**. 새 output/runId 필요. live 실행은 새 시작 신호와 현재 저장 범위를 먼저 확인.

```bash
python3 scripts/dev/references/arcore/physical/run.py \
  --serial DEVICE_SERIAL --build build/physical-arcore-20261014/app-v2 \
  --output build/physical-arcore-20261014/playback-next \
  --run-id playback-next --mode playback --source-run-id physical-live-01 \
  --duration 30 --save-images true --save-imu false --save-recording false \
  --authorization-note '현재 승인된 로컬 recording 재생'
python3 -m unittest discover -s scripts/dev/references/arcore/physical -p test_run.py -v
python3 -m unittest discover -s scripts/dev/references/arcore -p test_analyze_physical.py -v
```

- 물리 캡처 분석: `analyze_physical.py`는 pull manifest·PGM·pose/IMU·lifecycle 검증, 새 repository `build/` JSON만 생성. 자세한 명령은 위 결과 문서에서 연결.
