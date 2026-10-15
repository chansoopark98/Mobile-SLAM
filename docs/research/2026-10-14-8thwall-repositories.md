# 8th Wall 공식 저장소 수집·고정 목록 — 2026-10-14

- 목적: 공식 조직의 공개 저장소 전체를 로컬에 확보하고, 소스·fork·예제·배포 바이너리의 출처와 한계를 분리.
- 범위: [공식 8thwall 조직](https://github.com/8thwall)의 공개 저장소. 비공개 저장소·삭제된 저장소·모든 과거 branch/tag는 범위에 포함되지 않음.
- GitHub REST API 원본: `build/8thwall-deep-20261014/catalog/github-page-{1,2}.json`; page 1 = 52, page 2 = 0.
- 실행 시간: 2026-10-14 12:29:59–12:31:00 KST.
- 결과: **52/52 clone 성공, 실패 0, fork 31**. 분석용 분류: core 6, dependency-fork 31, example 9, supporting 6.
- 분류는 조사 편의를 위한 이름 기반 분류. fork 존재 자체는 현재 엔진의 의존성·실제 호출 증거가 아님.
- 저장 위치: `assets/references/8thwall/official/<repository>`; 전체 점유량 약 5.1 GiB (`du -sh`, 실행 직후).
- 기존 `assets/references/8thwall/engine`, `web`, `analysis-runtime` 보존. 이들은 origin 검증 가능한 Git clone이 아니므로 공식 clone으로 대체·덮어쓰기하지 않음.
- 모든 clone: `--depth 1 --single-branch --branch <API default_branch>`, `GIT_LFS_SKIP_SMUDGE=1`. submodule 미초기화.
- 초기 상태: 52개 모두 shallow, tracked/untracked 변경 없음, 파일 스캔 오류 0.
- 재실행: 52개 모두 재사용, HEAD·작업 트리 상태 동일. fetch/reset/stash/clean 없이 확인.

## 재현·증거

```bash
python3 scripts/dev/references/eighthwall/catalog.py --jobs 3
python3 scripts/dev/references/eighthwall/catalog.py --jobs 3 \
  --evidence build/8thwall-deep-20261014/catalog/reuse-check
python3 -m py_compile scripts/dev/references/eighthwall/catalog.py
```

- 재현 도구: `scripts/dev/references/eighthwall/catalog.py` — Python 표준 라이브러리 + Git; 새 Python 패키지 설치 없음.
- API pagination 종료까지 원본 저장 → 기존 origin 확인 → 없는 저장소만 최대 3개 병렬 clone → HEAD/status/license/LFS pointer 기록.
- 이미 존재하는 대상 경로가 검증된 clone이 아니면 보존 후 failure 기록. clone 실패·timeout도 `repo-<name>.error.json`과 clone log에 보존.
- 원본 API URL·응답 hash·headers: `github-pages.json`.
- 전체 manifest: `manifest.json`; 저장소별 상세: `repo-<name>.json`; 최초 clone 출력: `clone-<name>.log`; 최종 상태: `run.log`.
- 재사용 검증: `reuse-check/manifest.json`, `reuse-check.log`, `verification.txt`.
- 각 상세 JSON에 remote, exact HEAD, branch, shallow, porcelain status, tracked file count, 모든 탐지 LICENSE/LICENCE/COPYING/NOTICE 경로·SHA256, LFS pointer 원문 포함.

## 소스·바이너리·라이선스 경계

- 사용자 제공 전제: 2026-10-14 법률 검토를 거쳐 바이너리 및 전체 프로젝트 분석을 승인했다고 명시. 해당 승인을 작업 범위로 수용하며, 아래 라이선스 파일 사실은 법적 판단이나 작업 중단 사유로 사용하지 않음.
- [8thwall/8thwall](https://github.com/8thwall/8thwall), [8thwall/archive](https://github.com/8thwall/archive): 루트 MIT LICENSE. 포함된 제3자 코드 라이선스는 별도 파일 확인 필요.
- [8thwall/engine](https://github.com/8thwall/engine): 공개 배포 바이너리용 저장소. README는 MIT 공개 엔진 소스 저장소와 구분하며 소스 접근·재컴파일 미포함을 명시.
- `engine/LICENSE` 29–33행에 reverse engineer/decompile/disassemble, competitive product 관련 제한 문구 존재. 파일 공개와 MIT 허용은 동일하지 않음. 여기서는 계약 효력·적법성을 판정하지 않음.
- `web`, `desktop`, `8thwall.github.io`는 루트 license 탐지 없음. `web/xrextras/LICENSE`와 QR 코드 COPYING 등 하위 라이선스는 manifest에 별도 기록.
- LICENSE 탐지는 basename이 LICENSE/LICENCE/COPYING/NOTICE인 tracked regular file 기준. 소스 주석·다른 이름의 약관까지 자동 판정하지 않음.

| 핵심 저장소 | 루트 LICENSE SHA256 |
| --- | --- |
| 8thwall | `0a320756838ccfc28f5652e81e9d20dec1a23428c1841d9d378af5f44c7b5cde` |
| 8thwall.github.io | 루트 LICENSE 탐지 없음 |
| archive | `0a320756838ccfc28f5652e81e9d20dec1a23428c1841d9d378af5f44c7b5cde` |
| desktop | 루트 LICENSE 탐지 없음 |
| engine | `a2b3be43396a2554a4ab5fefd85086e593e5103f6b1160df4bb3ebb49b033750` |
| web | 루트 LICENSE 탐지 없음 |

## LFS 실제 객체 확보 범위

- 전체 **6,408개 pointer**. pointer의 `size` 필드 합계 **7,453,258,877 bytes**; 저장소 간 중복 객체를 제거하지 않은 논리 합계.
- clone 단계에서 LFS 대용량 객체는 자동 다운로드하지 않음. 특히 `engine`의 pointer 하나를 실행 가능한 엔진 바이너리로 계산하지 않음.
- 기존 엔진 JS와 별도 실험 lane의 실제 바이너리 다운로드·hash·실행 결과는 종합 보고서의 바이너리/브라우저 실험 증거를 참조. 이 목록은 clone만으로 바이너리 실행 완료를 주장하지 않음.
- 탐지 범위: tracked regular working-tree file 중 2,048 bytes 미만이면서 LFS v1 magic으로 시작하는 파일; symlink 제외.

| 저장소 | LFS pointer 수 | pointer가 가리키는 bytes 합계 |
| --- | ---: | ---: |
| 8thwall | 322 | 249,031,813 |
| 8thwall.github.io | 3 | 29,520,645 |
| aframe-image-targets-example | 63 | 38,164,354 |
| aframe-sky-effects-example | 4 | 4,452,176 |
| aframe-world-effects-example | 4 | 9,522,386 |
| agent8 | 36 | 38,515,756 |
| agent8-testing-can-delete | 1 | 7,456,839 |
| agent8-testing-can-delete-2 | 1 | 7,456,839 |
| archive | 5828 | 6,970,291,422 |
| engine | 1 | 13,998,848 |
| studio-face-effects-example | 3 | 990,011 |
| studio-image-targets-example | 66 | 52,773,086 |
| studio-physics-playground-example | 74 | 28,212,122 |
| studio-world-effects-example | 2 | 2,872,580 |

## 전체 저장소 고정 목록

- remote: 각 이름 링크의 URL + `.git`. local: `assets/references/8thwall/official/<이름>`.
- 전체 52개 shallow=true, 최초 status=clean. 아래 exact SHA는 수집 시점의 default branch HEAD이며 향후 upstream 최신값을 의미하지 않음.

| 저장소 | 분류 | branch | exact HEAD | LFS | LICENSE 계열 파일 수 |
| --- | --- | --- | --- | ---: | ---: |
| [8frame](https://github.com/8thwall/8frame) | dependency-fork | master | `409dd7623e0f69b86d5a983735be7104900dfdb0` | 0 | 1 |
| [8thwall](https://github.com/8thwall/8thwall) | core | main | `4edfa484936a2cc691366eeec5c6e58bb6d4c8bd` | 322 | 22 |
| [8thwall.github.io](https://github.com/8thwall/8thwall.github.io) | core | main | `1cdf08391d7c732d0bee1c4ba34d252ea098951d` | 3 | 0 |
| [XNNPACK](https://github.com/8thwall/XNNPACK) | dependency-fork | master | `6e03011a0fd74ff0b3bba3a874e452543ccbaa86` | 0 | 1 |
| [aframe-image-targets-example](https://github.com/8thwall/aframe-image-targets-example) | example | main | `0eefbe133eb5adbb69ced43c05bef9ddca20ea45` | 63 | 6 |
| [aframe-particle-system-component](https://github.com/8thwall/aframe-particle-system-component) | dependency-fork | master | `da457f55d875a5e8cc4d0be54ea9f9fdb6beea6e` | 0 | 1 |
| [aframe-sky-effects-example](https://github.com/8thwall/aframe-sky-effects-example) | example | main | `4068f3405fd4fcfb1b36eece14cfc1cc7e62eb0e` | 4 | 2 |
| [aframe-world-effects-example](https://github.com/8thwall/aframe-world-effects-example) | example | main | `582b28a94182f0243346f1c22df4b49091dbf2ad` | 4 | 2 |
| [agent8](https://github.com/8thwall/agent8) | dependency-fork | main | `bb71b22e2356268165634b226afe751a9b104869` | 36 | 2 |
| [agent8-testing-can-delete](https://github.com/8thwall/agent8-testing-can-delete) | dependency-fork | main | `86299adde7b49fc6f210d464180aa47c852a3fb8` | 1 | 2 |
| [agent8-testing-can-delete-2](https://github.com/8thwall/agent8-testing-can-delete-2) | dependency-fork | main | `137d3f4fd8a1195bd2a2f228b00c58683e0e77b8` | 1 | 1 |
| [archive](https://github.com/8thwall/archive) | core | main | `fccdda4995e1897fec39811cc2b26622fcdc96ea` | 5828 | 223 |
| [bazel-fork](https://github.com/8thwall/bazel-fork) | dependency-fork | master | `c893286f22ead570948ee130006476d9430cd441` | 0 | 67 |
| [bazel-issue-3080](https://github.com/8thwall/bazel-issue-3080) | supporting | master | `430b01a436d518a6db2cc2131f0e3e52e85edb2a` | 0 | 0 |
| [capnp-ts](https://github.com/8thwall/capnp-ts) | dependency-fork | master | `ca0aded8ff6bd46b13a7bcfd0a84bbbba6e488f3` | 0 | 1 |
| [capnproto-fork](https://github.com/8thwall/capnproto-fork) | dependency-fork | master | `c3c2655f6d6efeb93ae6b3ea718eecd00b72f5d0` | 0 | 1 |
| [capnproto-net](https://github.com/8thwall/capnproto-net) | supporting | master | `6c80919ec6657f6e8c81b085ca69eb6e85b11b74` | 0 | 2 |
| [cc-rs-fork](https://github.com/8thwall/cc-rs-fork) | dependency-fork | main | `6f3dff90a19757d4e1193310efeafd603993afb8` | 0 | 0 |
| [cpuinfo](https://github.com/8thwall/cpuinfo) | dependency-fork | master | `cbf1796004caf2aba93e6026650508791d8ac4dd` | 0 | 2 |
| [desktop](https://github.com/8thwall/desktop) | core | main | `5c8e1225e820594d071742e3fcd77ecc3d4fa23e` | 0 | 0 |
| [discord-activity-example](https://github.com/8thwall/discord-activity-example) | example | main | `60f46e841191378022cef4ef6db4069a80d92f94` | 0 | 0 |
| [eigen](https://github.com/8thwall/eigen) | supporting | 2.0 | `2827b35b29f361431f8f5f81bfd2f3456e06f695` | 0 | 3 |
| [emscripten](https://github.com/8thwall/emscripten) | dependency-fork | master | `368bf80cd877dfa9e614e19d1aaff7b8dbe20ac9` | 0 | 25 |
| [engine](https://github.com/8thwall/engine) | core | main | `e1421ae742a9c5d654871367c64c1de3ad044caf` | 1 | 1 |
| [flatbuffers-fork](https://github.com/8thwall/flatbuffers-fork) | dependency-fork | master | `dddd071976c716635aec26bfcde81432ec9ad171` | 0 | 2 |
| [go-links](https://github.com/8thwall/go-links) | supporting | main | `803a8b70c7a55492ad9449e512c9cdb969980983` | 0 | 0 |
| [google-auth-yourls](https://github.com/8thwall/google-auth-yourls) | supporting | master | `058e6687c82bce970ce3d8e2e2475205a408fe0b` | 0 | 1 |
| [headless-gl-fork](https://github.com/8thwall/headless-gl-fork) | dependency-fork | master | `dc9064e144c59b2de2f45187b4f4394879c45230` | 0 | 0 |
| [keyword-not-found-set-field-yourls](https://github.com/8thwall/keyword-not-found-set-field-yourls) | supporting | master | `a0c5b6bd92f4f1f67d8d749d16df75e09b435675` | 0 | 1 |
| [libgit2-fork](https://github.com/8thwall/libgit2-fork) | dependency-fork | master | `e6a784efd91ec0ea89d73f7d5d3ae28f91469ed4` | 0 | 7 |
| [libmp4v2-fork](https://github.com/8thwall/libmp4v2-fork) | dependency-fork | master | `b0e3019fb301362dedd376ba3f4fb7263b277c11` | 0 | 1 |
| [mediapipe-fork](https://github.com/8thwall/mediapipe-fork) | dependency-fork | master | `b6e680647cebf925ebc6a3f82b469d57c1c68da9` | 0 | 2 |
| [node](https://github.com/8thwall/node) | dependency-fork | main | `08a6bc1d35d2538def3e97151aad68a3005c83d6` | 0 | 476 |
| [objc2-fork](https://github.com/8thwall/objc2-fork) | dependency-fork | master | `724f8c74e472089890619ed03dd2e099d53c212d` | 0 | 3 |
| [openh264-fork](https://github.com/8thwall/openh264-fork) | dependency-fork | master | `22a46624f3b5c88e017ae5fb78f7a66e06bfccb9` | 0 | 1 |
| [parquetjs](https://github.com/8thwall/parquetjs) | dependency-fork | main | `88f124c61e97c254d83af349f346a5f9cd7c58b4` | 0 | 1 |
| [rust-openssl-fork](https://github.com/8thwall/rust-openssl-fork) | dependency-fork | master | `463f1bb84f171accfa51c870b34622af6769a884` | 0 | 1 |
| [studio-face-effects-example](https://github.com/8thwall/studio-face-effects-example) | example | main | `b8854ac332fddbfbbee736050df76a944197b009` | 3 | 2 |
| [studio-image-targets-example](https://github.com/8thwall/studio-image-targets-example) | example | main | `78acb27325fddb59a529451ea8b4a7d19f9ab45e` | 66 | 2 |
| [studio-physics-playground-example](https://github.com/8thwall/studio-physics-playground-example) | example | main | `80b24599973581196a9c3a05d2abc9860b1e368f` | 74 | 2 |
| [studio-world-effects-example](https://github.com/8thwall/studio-world-effects-example) | example | main | `c489770113bfb8384a2913f0584f65e0cf013c8d` | 2 | 2 |
| [swift-rs-fork](https://github.com/8thwall/swift-rs-fork) | dependency-fork | main | `e86b0968d5e7b6b9bd8a6451f85c3b5409dffb46` | 0 | 0 |
| [tauri-fork](https://github.com/8thwall/tauri-fork) | dependency-fork | main | `5cbdece0e115532ea2601e5afedd3d53f208e4e2` | 0 | 3 |
| [tauri-plugin-cors-fetch-fork](https://github.com/8thwall/tauri-plugin-cors-fetch-fork) | dependency-fork | main | `92ba272750c78e5b410d34db9d622e46703cd2f2` | 0 | 1 |
| [tensorflow-fork](https://github.com/8thwall/tensorflow-fork) | dependency-fork | master | `a586b6e380344a423caa5944965112dd05e126e8` | 0 | 15 |
| [tfjs](https://github.com/8thwall/tfjs) | dependency-fork | master | `ec66b7ba84c542eb64b72a7cb2310ecc1c8324c5` | 0 | 3 |
| [three.js](https://github.com/8thwall/three.js) | dependency-fork | dev | `3b1ff7661884f26e6d9af1d94c293129aaba885c` | 0 | 13 |
| [threejs-world-effects-example](https://github.com/8thwall/threejs-world-effects-example) | example | main | `2e18f3abefc4db2d190b66c2ffcdc9c852ebf2e5` | 0 | 0 |
| [ua-parser-js](https://github.com/8thwall/ua-parser-js) | dependency-fork | master | `46379495762d2db94463ad0781c8a2bce5e3cef8` | 0 | 1 |
| [web](https://github.com/8thwall/web) | core | master | `72556f9122f5b0e861f55748cc9a938a59691273` | 0 | 2 |
| [webxr-samples](https://github.com/8thwall/webxr-samples) | dependency-fork | main | `6a85276690f8c5ce9f2ade3314a97899b6ef92cf` | 0 | 3 |
| [zerocopy-fork](https://github.com/8thwall/zerocopy-fork) | dependency-fork | main | `dafed1baca408e7a15f7deae23af1266ca9e7d4a` | 0 | 0 |

## 검증 한계

- 확인: 공개 조직 inventory pagination, 52 clone, origin/HEAD/branch/shallow/status, license hash, LFS pointer, 비파괴 재실행.
- 미확인: 모든 저장소 build/test, 모든 LFS 객체·submodule, 비공개 SLAM 소스, 모든 역사 branch/tag, 실제 모바일 추적 정확도.
- clone 성공은 dependency 활성 사용·기술 적용·실시간 성능 검증을 대체하지 않음. 구체적인 기술 주장에는 별도 소스 경로 또는 실행 증거 필요.
