# E5: 원본 reference runtime SDK headless 기록 하네스 (패키지 H, US-015 기준 2')

desktop headless Chromium(SwiftShader) + 합성 시퀀스 한정 개발 도구입니다. 제품 runtime이 아닙니다. 서버는 하네스가 띄우는 임시 `127.0.0.1`뿐이고 외부 요청은 모두 abort합니다(`blockedExternal`). 사용자 서버·`web/`·원본 SDK 파일은 건드리지 않습니다. Chromium은 `flock build/c1-chromium.lock` 안에서 1개만 씁니다.

| 파일 | 역할 |
|---|---|
| `scene.py` | 합성 방(앞 벽 + 왼쪽 벽 + 탁자, 타깃 0.6×0.8 m)을 부드러운 궤적으로 렌더하고 같은 궤적에서 해석적 IMU(중력 포함 specific force, 자이로)를 만든다. 출력: `input.json`(tum 형식 fixture), `mav0/cam0/data/*.pgm`, `target/e5-target*`, `gt.json/tsv`. 시드가 방·타깃·위상·잡음을 정한다 |
| `run_original.mjs` | `tools/re/runtime/run-reference-it.mjs` 복사본(차이 7개가 파일 머리에 있음). 원본 SDK를 실행해 `image-targets.json`(found/lost/updated 이벤트, frame별 `detectedImages`), `trace.jsonl`, `recorded-inputs.jsonl.gz`, `memprobe.jsonl.gz`를 쓴다. 출력은 `build/slam-refact-pkgh-20261022` 아래만 허용. 시나리오 파일은 `$E5_SCENARIOS` |
| `e5_image_hook.js` | `image-target-hook.js` 복사본(행 상한만 400 → 100000) |
| `prepare_bundle.py` | `harness/prepare_inputs.py`를 그대로 쓰는 얇은 래퍼(E5 시나리오·run 경로 지정). 뒤에 `divergence/make_variant.py --capture-binding --capture-wallclock --queue-times --capture-id-seed`로 exact-input bundle을 만든다 |
| `e5_replay_main.cc`, `snapshot-cmake.patch` | `engine/tests/image_target/or4_replay_main.cc` 복사본 + `--world-tracker 0|1` + scale 필드. 엔진 HEAD의 **스냅샷 사본**에 넣어 빌드한다(저장소 엔진 트리는 바뀌지 않는다) |
| `e5_extract.py`, `e5_compare.py` | 원본·엔진 run을 frame별 series로 바꾸고 gauge 불변량(ρ, 방향, 상대 자세)·이벤트·상태를 비교 |
| `e5_pipeline.py` | 한 시퀀스: 원본 K회 → bundle → 엔진 arm들 → `metrics.json` |
| `e5_kmutation.py` | dev 변이 검출력: 엔진 bundle 복사본의 초점거리 override |

gate: `slam_refact/tools/gates/it-scale-v2/`(규칙·실행기), 서류: `build/slam-refact-full-20261019/preregistration/it-scale-gate.v2.*`, 보고서: `docs/design/pkg-h-e5-report.md`.

엔진 빌드(스냅샷): `git archive HEAD slam_refact/engine slam_refact/tools | tar -x -C build/slam-refact-pkgh-20261022/snap`, `e5_replay_main.cc`를 `snap/slam_refact/engine/tests/image_target/`에 복사하고 `snapshot-cmake.patch`의 3줄을 그 디렉터리 `CMakeLists.txt`의 `sr_or4_replay` 블록 뒤(`if(SLAM_REFACT_BUILD_HARNESS)` 안)에 추가, `cmake -S snap/slam_refact/engine -B engine -G Ninja -DCMAKE_BUILD_TYPE=Release -DSLAM_REFACT_WERROR=ON …(us015re2의 CMakeCache와 같은 변수)`, `ninja -C engine -j2 sr_e5_replay`.
