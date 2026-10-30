# Vuforia reference 환경

- 고정 API: Vuforia Engine **11.4.4**, Driver API **7**.
- 제품 estimator 변경 없음. 실행 자산·빌드·SDK는 gitignored `assets/references/Vuforia/`; 명령·결과는 `build/research-20261012/vuforia/environment/`.
- 공식 공개 저장소: `PTCInc/vuforia-engine`, commit `1d42568da43bf850270fe9e17fbb511e349c95b7`; **issue-only**, SDK·샘플 구현 아님.
- 공식 SDK/샘플 배포: <https://developer.vuforia.com/downloads/sdk>, <https://developer.vuforia.com/downloads/samples>. GitHub release의 archive는 issue-only snapshot; public Maven/registry 배포 확인 안 됨.
- 라이선스: <https://developer.vuforia.com/legal/vuforia-developer-agreement>. SDK와 sample 사용권은 PTC 약관; 공개 repository를 MIT SDK로 취급 금지. 아래 offline fixture에는 PTC SDK/sample 코드 복사 없음.

## 실제 offline self-test

```bash
python3 scripts/dev/references/vuforia/reference.py self-test
python3 scripts/dev/references/vuforia/reference.py status
```

- `self-test`: CMake/C++17 host build 1job → CTest → 실제 독립 executable → Python12개·Node6개 계약 검사·공급 거절 fixture3개.
- `status`: SDK 미공급 시 **exit2/BLOCKED**. Offline PASS와 SDK·engine activation을 별도로 표시.
- 검사: CV/GL·world/camera inverse·column-major·camera lever arm·정확한 ns pose-first pair·노출 종료 시각·NV21 stride·JS Number 정밀도.
- **Vuforia 함수 호출0**; proprietary runtime·ABI·tracking·ATE·latency 검증 아님.

## 공식 SDK 공급 후 빌드

```bash
python3 scripts/dev/references/vuforia/reference.py provision \
  --sdk-archive /absolute/path/vuforia-sdk-android-11-4-4.zip \
  --sdk-sha256 OFFICIAL_64_HEX_SHA256
python3 scripts/dev/references/vuforia/reference.py build-native \
  --ndk /absolute/path/android-sdk/ndk/26.1.10909125
python3 scripts/dev/references/vuforia/reference.py build-app \
  --android-sdk /absolute/path/android-sdk \
  --gradle /absolute/path/gradle-8.7/bin/gradle \
  --java-home /absolute/path/jdk-17
```

- `provision`: ZIP checksum 일치 전 추출 금지; 경로 traversal/symlink·크기 한도 검사; 헤더11.4.4·ARM64 library 유일성 검사; SDK/headers/library SHA 기록. 공식 checksum은 실행자가 공급; 사용자 제공 hash를 upstream 인증으로 확대하지 않음.
- `build-native`: NDK toolchain으로 실제 public header compile·`vuEngineGetLibraryVersionInfo` link. Android ARM64 `.so` 생성; host 실행 없음.
- `build-app`: 별도 reference source copy에서 Java17·Gradle8.7·AGP8.6.1·SDK35·NDK26.1.10909125·CMake3.22.1 사용, `--max-workers=1`.
- APK는 library version을 보여주는 일반 앱; camera 권한·license key 입력·engine activation 경로 없음. 새 SDK/key 수락이나 계정 생성 자동화 없음.
- SDK 없는 현재 환경에서는 native/APK build **BLOCKED**. 위 API compile·link 및 APK 생성은 SDK 공급 후 검증 대상.
- Vuforia11.4.4 Android runtime은 ARM64만 지원. x86_64 ARCore emulator·Linux host·Model Target CLI를 Vuforia tracking 실행 환경으로 간주 금지.
- tracking 실험에는 유효 Basic/Premium/Enterprise key와 지원되는 실제 target이 추가 필요. FileDriver는 기록된 pose를 주입하므로 raw image+IMU VIO replay와 구분.
