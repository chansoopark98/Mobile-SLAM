# Mobile-SLAM

C++17 SLAM 연구 프로젝트입니다. 기준(baseline) 엔진은 `slam_refact`이고, VINS-Mono 기반 VIO는 legacy입니다.

| 엔진 | 위치 | 상태 | 주요 의존성 |
|---|---|---|---|
| [slam_refact](slam_refact/README.md) | `slam_refact/` | 기준 엔진. 이미지 타깃, 브라우저 XR 연결 포함 | Eigen, Ceres; native replay는 zlib |
| VINS-Mono 기반 VIO | `legacy/vins-mono/`, `web/legacy/` | legacy. 이전 `main`은 branch `legacy/vins-mono`, tag `vins-mono-final`(`6567d8a`)에 보존 | Eigen, OpenCV, Ceres, yaml-cpp |

- 테스트·재생 도구는 [개발 도구 안내](scripts/dev/README.md)를 참고합니다.
- 데이터셋·모델·키·소스 의존성 캐시·빌드 출력은 저장소에 포함하지 않습니다.

## slam_refact (baseline)

- 필요: CMake 3.21+, Ninja, C++17 compiler, Eigen 3.4, Ceres 2.2.0. GTest 1.14.0은 시험 전용, zlib은 replay 전용입니다.
- 엔진 CMake는 기본 offline 모드입니다. 처음에는 Ceres·GTest 소스 cache를 `build/_deps/`에 준비합니다.

```bash
mkdir -p build/_deps
git clone --depth 1 --branch 2.2.0 https://github.com/ceres-solver/ceres-solver.git build/_deps/ceres-solver-src
git clone --depth 1 --branch v1.14.0 https://github.com/google/googletest.git build/_deps/googletest-src
cmake -S slam_refact/engine -B build/slam-refact-native -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/slam-refact-native --parallel 2
ctest --test-dir build/slam-refact-native --no-tests=error -L '^unit$' --parallel 2 --output-on-failure
```

개발 점검 명령은 저장소 루트에서 실행합니다.

```bash
bash scripts/dev/check.sh refact-native   # 엔진 구성·빌드·unit ctest, 알려진 실패 대조
bash scripts/dev/check.sh refact-web      # slam_refact/web-chunk/tests·web-smoke
bash scripts/dev/check.sh refact-tools    # vendor·std algorithm 점검
```

- WASM build, replay bundle, XR 배포(`slam_refact/tools/deploy_xr.py`)는 [slam_refact 안내](slam_refact/README.md)를 확인합니다.
- 브라우저 페이지에서는 `/`(XR 페이지)와 `/slam-refact`(측정·녹화)가 이 엔진을 씁니다.

## Browser pages

`web/`를 HTTPS로 제공하고 camera·motion sensor 권한을 허용합니다. 서버(`web/server.js`, 시작·중지는 `scripts/ops/serve.sh`)의 경로는 다음과 같습니다.

| 경로 | 기능 | 엔진 |
|---|---|---|
| `/` | `web/slam-refact-xr.html` — XR phone·Three.js·A-Frame 샘플("공간 살펴보기") | slam_refact |
| `/slam-refact-xr` | `/slam-refact-xr.html`로 302. `/`와 같은 페이지 | slam_refact |
| `/slam-refact` | `/slam-refact.html`로 302. 엔진 측정·녹화 | slam_refact |
| `/legacy/` | VIO camera·IMU·trajectory viewer (`web/legacy/index.html`) | VINS legacy |
| `/legacy/test-tumvi.html` | 준비된 TUM-VI 입력 replay | VINS legacy |
| `/legacy/audit-mobile.html` | 제한된 sensor·timing 관측 | VINS legacy |

- 이전 주소 `/index.html`, `/test-tumvi.html`, `/audit-mobile.html`, `/vio_engine.js`는 `/legacy/...`로 301 redirect합니다.
- `/js/`에서는 `slam-refact-*.js`만 제공합니다. VINS 브라우저 코드는 `/legacy/js/`입니다.
- 개발용 localhost 정적 서버: `python3 scripts/dev/serve-mobile-audit.py --generate-dev-cert --host 127.0.0.1 --port 8766`. 이 서버는 `/`를 `/legacy/audit-mobile.html`로 연결하고 위의 alias·redirect가 없으므로 `/slam-refact-xr.html`, `/slam-refact.html`을 파일 이름으로 지정합니다.
- self-signed localhost 인증서는 실제 휴대전화의 trusted HTTPS를 대체하지 않습니다.
- VINS와 slam_refact는 서로 다른 artifact·manifest를 사용합니다(`web/legacy/vio_engine.*`, `web/slam-refact/`, `web/slam-refact-xr/`).
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

## Legacy: VINS-Mono

`legacy/vins-mono/`(`src/ include/ cmake/ wasm/ config/ tests/ third_party/licenses`)와 `web/legacy/`는 기준 엔진이 아니며 보조·동결 상태입니다(`docs/STATUS.md`). 이전 `main`의 전체 상태는 branch `legacy/vins-mono` 또는 tag `vins-mono-final`로 꺼냅니다. 이전 경로(`src/`, `wasm/`, `web/vio_engine.js` 등)를 쓰는 과거 문서는 이 tag 기준입니다.

### Native VIO

- 필요: CMake 3.21+, Ninja, C++17 compiler, Eigen 3.4, OpenCV 4.x, yaml-cpp, pkg-config.
- Ceres 2.2.0과 GoogleTest 1.14.0은 CMake가 고정 버전으로 확보합니다.
- Pangolin은 `MOBILE_SLAM_BUILD_VIEWER=ON`일 때만 필요합니다. 기본 빌드는 headless입니다.
- preset은 `legacy/vins-mono`에서 실행하며 binaryDir은 저장소 루트의 `build/dev-native`입니다. 저장소 루트에서는 `bash scripts/dev/check.sh native`가 같은 절차를 실행합니다.

```bash
cd legacy/vins-mono
cmake --preset native-dev
cmake --build --preset native-tests --parallel 2
ctest --preset native-tests
```

- `build/_deps`에 `ceres-solver-src`, `googletest-src`가 있으면 `check.sh native`가 `FETCHCONTENT_SOURCE_DIR_*`로 연결합니다. 다른 위치는 `MOBILE_SLAM_DEPS_DIR`로 지정합니다.
- 예전 소스 루트(저장소 루트의 `CMakeLists.txt`)로 구성한 `build/dev-native` 등은 다시 만들어야 합니다.

TUM-VI를 실행하려면 먼저 입력 데이터와 calibration을 준비하고 YAML의 `dataset_path`를 맞춥니다. 저장소 루트에서 실행합니다.

```bash
build/dev-native/tiny_vins_mono legacy/vins-mono/config/tum_vi_room1.yaml --headless --seed 19 --cv-threads 1
```

- 출력: ignored `logs/`의 궤적·평가 파일.
- IMU 단위: timestamp 초, 가속도 m/s², 각속도 rad/s. 영상과 IMU는 같은 시간축이어야 합니다.
- camera/body extrinsic 방향과 crop·회전 후 intrinsics를 확인해야 합니다.

### WASM VIO

Emscripten 5.0.0을 활성화한 셸에서 실행합니다. OpenCV 4.5.5·Ceres 2.2.0의 소스와 설치 결과는 로컬 `legacy/vins-mono/wasm/libs/`에 생성됩니다.

```bash
source "$EMSDK/emsdk_env.sh"
bash legacy/vins-mono/wasm/build_opencv.sh
bash legacy/vins-mono/wasm/build_ceres.sh
emcmake cmake -S legacy/vins-mono/wasm -B build/vio-wasm -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build/vio-wasm --parallel 2
node scripts/dev/wasm-smoke.mjs build/vio-wasm/vio_engine.js build/vio-wasm/smoke.json
```

- 출력: embedded WASM을 포함한 ES module `build/vio-wasm/vio_engine.js`와 classic Worker용 별도 target.
- 브라우저 VIO는 `web/legacy/js/vio-wrapper.js`의 `VIOWrapper`, native API는 `legacy/vins-mono/include/vio_engine.h`, 바인딩은 `legacy/vins-mono/wasm/vio_bindings.cpp`를 확인합니다.
- pose의 timestamp·epoch·fresh/valid 상태를 함께 읽어야 합니다. 유한한 pose만으로 추적 성공을 판단하지 않습니다.
- 새 빌드는 배포 파일(`web/legacy/vio_engine.js`, `web/legacy/vio_engine.manifest.json`)을 자동 교체하지 않습니다. artifact SHA와 manifest를 확인한 후 `scripts/dev/deploy-candidate.py`로만 배포합니다.

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
