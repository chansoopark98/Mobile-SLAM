# Mobile-SLAM

C++17 Visual-Inertial Odometry와 WebAssembly 브라우저 SLAM 연구 프로젝트입니다.

| 엔진 | 용도 | 주요 의존성 |
|---|---|---|
| VIO / VINS-Mono 기반 | native 영상·IMU 처리와 모바일 웹 VIO | Eigen, OpenCV, Ceres, yaml-cpp |
| [slam_refact](slam_refact/README.md) | 별도 SLAM 코어, 이미지 타깃, 브라우저 XR 연결 | Eigen, Ceres; native replay는 zlib |

- 테스트·재생 도구는 [개발 도구 안내](scripts/dev/README.md)를 참고합니다.
- 데이터셋·모델·키·소스 의존성 캐시·빌드 출력은 저장소에 포함하지 않습니다.

## Native VIO

- 필요: CMake 3.21+, Ninja, C++17 compiler, Eigen 3.4, OpenCV 4.x, yaml-cpp, pkg-config.
- Ceres 2.2.0과 GoogleTest 1.14.0은 CMake가 고정 버전으로 확보합니다.
- Pangolin은 `MOBILE_SLAM_BUILD_VIEWER=ON`일 때만 필요합니다. 기본 빌드는 headless입니다.

```bash
cmake --preset native-dev
cmake --build --preset native-tests --parallel 2
ctest --preset native-tests
```

TUM-VI를 실행하려면 먼저 입력 데이터와 calibration을 준비하고 YAML의 `dataset_path`를 맞춥니다.

```bash
build/dev-native/tiny_vins_mono config/tum_vi_room1.yaml --headless --seed 19 --cv-threads 1
```

- 출력: ignored `logs/`의 궤적·평가 파일.
- IMU 단위: timestamp 초, 가속도 m/s², 각속도 rad/s. 영상과 IMU는 같은 시간축이어야 합니다.
- camera/body extrinsic 방향과 crop·회전 후 intrinsics를 확인해야 합니다.

## WASM VIO

Emscripten 5.0.0을 활성화한 셸에서 실행합니다. OpenCV 4.5.5·Ceres 2.2.0의 소스와 설치 결과는 로컬 `wasm/libs/`에 생성됩니다.

```bash
source "$EMSDK/emsdk_env.sh"
bash wasm/build_opencv.sh
bash wasm/build_ceres.sh
emcmake cmake -S wasm -B build/vio-wasm -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build/vio-wasm --parallel 2
node scripts/dev/wasm-smoke.mjs build/vio-wasm/vio_engine.js build/vio-wasm/smoke.json
```

- 출력: embedded WASM을 포함한 ES module `build/vio-wasm/vio_engine.js`와 classic Worker용 별도 target.
- 브라우저 VIO는 `web/js/vio-wrapper.js`의 `VIOWrapper`, native API는 `include/vio_engine.h`, 바인딩은 `wasm/vio_bindings.cpp`를 확인합니다.
- pose의 timestamp·epoch·fresh/valid 상태를 함께 읽어야 합니다. 유한한 pose만으로 추적 성공을 판단하지 않습니다.
- 새 빌드는 배포 파일을 자동 교체하지 않습니다. artifact SHA와 manifest를 확인한 후 배포해야 합니다.

## Browser pages

`web/`를 HTTPS로 제공하고 camera·motion sensor 권한을 허용합니다. 개발용 localhost mock 서버는 다음과 같이 실행할 수 있습니다.

```bash
python3 scripts/dev/serve-mobile-audit.py --generate-dev-cert --host 127.0.0.1 --port 8766
```

| 경로 | 기능 |
|---|---|
| `/` | VIO camera·IMU·trajectory viewer |
| `/test-tumvi.html` | 준비된 TUM-VI 입력 replay |
| `/audit-mobile.html` | 제한된 sensor·timing 관측 |
| `/slam-refact.html` | 별도 SLAM 엔진 측정·녹화 |
| `/slam-refact-xr.html` | XR phone·Three.js·A-Frame 샘플 |

- self-signed localhost 인증서는 실제 휴대전화의 trusted HTTPS를 대체하지 않습니다.
- VIO와 별도 SLAM 엔진은 서로 다른 artifact·manifest를 사용합니다.
- XR 파일 무결성 정보는 `config/runtime-integrity/xr-tree.json`에서 관리합니다.

## Validation and limits

```bash
npm ci --prefix scripts/dev
bash scripts/dev/check.sh syntax
bash scripts/dev/check.sh browser-contracts
bash scripts/dev/check.sh eval
```

- 두 엔진의 native·WASM·브라우저 경로를 별도로 검사합니다. desktop replay·합성 입력의 통과는 실제 기기의 정확도·발열·장기 성능을 보증하지 않습니다.
- 궤적 정확도에는 독립 GT, 일대일 timestamp association, body frame과 SE(3) scale=1 기준이 필요합니다. Sim(3)는 scale 진단으로 구분합니다.
- 별도 SLAM unit에는 알려진 `FrameFrameTrackerVfft.AcceptedMotionAccuracy` 실패 1개와 optional 입력에 따른 skip 2개가 남아 있습니다. 시험 임계값을 완화해 통과시키지 않습니다.
- 공개 artifact metadata는 포함된 실행 파일의 실제 hash·크기만 기록합니다. 재빌드 결과의 provenance와 배포 동작은 별도로 검증해야 합니다.
- low-feature 예측·바닥 후보는 실험 옵션이며 기본 OFF입니다. 실제 Android/iOS 센서·기기 GPU·현장 GT·장시간 연속 실행은 별도 검증 대상입니다.

## External code

- `slam_refact/`는 **8th Wall 공개 소스의 복사·수정본과 프로젝트에서 재구현한 엔진을 함께 사용**합니다. 전체가 독립 작성된 코드는 아닙니다.
- 기준 출처: [8th Wall 공개 저장소](https://github.com/8thwall/8thwall), commit `4edfa484936a2cc691366eeec5c6e58bb6d4c8bd`.
- 원 저작권 주체: **8th Wall, LLC / Niantic, Inc.** 공개 경로 이름의 변경은 출처나 권리를 바꾸지 않습니다.

| 경로 | 출처·구현 범위 |
|---|---|
| `slam_refact/engine/third_party/xr_source/` | 공개 C8·특징점·기하·이미지 타깃·얼굴/귀·하늘 소스와 호환 patch |
| `slam_refact/engine/third_party/xr_shim/` | 공개 인터페이스·schema를 바탕으로 한 로컬 호환 구현 |
| `slam_refact/engine/src/`, `include/`, `wasm/` | 공개 타입·연산 재사용 및 reference binary 관찰·실험 기반의 SLAM 재구현·연결 코드 |
| `slam_refact/engine/tests/`, `tools/` | 공개 테스트·fixture의 복사/변환본과 프로젝트의 계약·회귀 도구 |
| `slam_refact/web-chunk/`, `web/slam-refact-xr/` | 공개 JS·센서·GPU frontend 동작을 참고/재사용한 코드와 프로젝트의 연결·UI |
| `web/slam-refact-xr/engine/xr.js` | 배포 SDK bundle 복사본. protocol·CDN·인증 식별자를 포함한 원문 문자열 9개를 보존하며 로컬 소스 재빌드 결과로 표시하지 않음 |

파일별 원본 경로·commit·hash·patch는 `slam_refact/engine/third_party/xr_source/MANIFEST.tsv`, `PROVENANCE.md`, `patches/`와 테스트 데이터의 `MANIFEST.tsv`를 확인합니다. 원 작성자 이름은 코드에 유지하고 원 연락처는 아래에 모았습니다.

| 작성자 | 원 연락처 |
|---|---|
| Dat Chu | `dat@8thwall.com` |
| Erik Murphy-Chutorian | `mc@8thwall.com` |
| Nathan Waters | `nathan@8thwall.com` |
| Nicholas Butko | `nb@8thwall.com` |
| Paris Morgan | `paris@8thwall.com` |
| Pooja Bansal | `pooja@8thwall.com` |
| Scott Pollack | `scott@8thwall.com` |

- libc++, musl, FLANN, MurmurHash, Three.js, A-Frame 등의 개별 고지는 [제3자 코드 고지](slam_refact/engine/third_party/THIRD_PARTY_NOTICES.md)와 해당 소스에서 유지합니다.
- 외부 SDK bundle의 라이선스 원문 검증은 미완료 상태입니다. 프로젝트 MIT 고지로 외부 코드를 재라이선스하지 않습니다.
- 아래 블록은 source/provenance 도구가 읽는 upstream metadata입니다.

```public-reference-metadata
{
  "repository_url": "https://github.com/8thwall/8thwall",
  "organization_url": "https://github.com/8thwall",
  "organization_api_url": "https://api.github.com/orgs/8thwall/repos",
  "archive_url_template": "https://media.githubusercontent.com/media/8thwall/engine/{commit}/xr-standalone.zip",
  "core_repositories": [
    "8thwall",
    "engine",
    "web",
    "archive",
    "8thwall.github.io",
    "desktop"
  ],
  "main_repository_name": "8thwall",
  "copyright_owners": [
    "8th Wall",
    "Niantic"
  ],
  "legacy_local_root": "assets/references/8thwall"
}
```

## License

프로젝트 고지는 [LICENSE](LICENSE)를 확인합니다. 외부 코드에는 각 소스의 고지와 라이선스가 적용됩니다.
