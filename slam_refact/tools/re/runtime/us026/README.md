# US-026 S6 config poke 하네스 (원본 reference runtime xr-slam)

원본 xr-slam WASM(sha256 `780731fe…c422`)의 설정 두 개를 선형 메모리 쓰기로 켜고, 그 결과를 원본 메모리 읽기로 관측합니다. 설계: `slam_refact/docs/design/us026-design.md` §2. dev 결과: `slam_refact/docs/design/us026-dev-harness-report.md`.

- WASM bytes, import, SDK 파일은 바꾸지 않습니다. `config-poke.js`는 compile하지 않고, 앞서 설치된 `WebAssembly.instantiate`(memprobe → input-recorder → native)가 돌려준 exports에서 `c8EmAsm_processStagedFrame` 하나만 다시 감쌉니다(가장 바깥).
- 쓰는 주소는 아래 표의 두 개뿐입니다. 다른 key나 값은 runner와 init script 양쪽에서 거부합니다.
- 결과는 desktop headless(SwiftShader) 한정입니다. 실기기 동작으로 확대하지 않습니다.

## 주소표 (런타임 확인은 dev 보고서 V1–V3)

| key | 이름 | 주소 | 타입 | 기본 | 값 | getter 캐시 version |
|---|---|---|---|---|---|---|
| `ba` | `MapBuilder.bundleAdjustmentIterativeEnabled` | 606188 | u8 | 0 | 0 \| 1 | @527080 (func651) |
| `mse` | `MetricScaleEstimator.scaleEstimationMethod` | 606476 | i32 | 1 | 1..5 | @527084 (func637) |
| (읽기만) | Params version `PV` | 613472 | i32 | | | |
| (읽기만) | func1145 1회 초기화 flag `baLatch` | 608328 | u8 | 0 | | |

## 호출마다 하는 일

1. pre-call: `[PV, ba, baCache, mse, mseCache]`를 읽고, 요청한 key를 씁니다. 캐시 version은 쓰지 않습니다.
2. 원래 함수 호출(안쪽에서 memprobe가 표본을 남김).
3. post-call: 같은 다섯 값을 다시 읽습니다. 쓴 값과 post 값이 다르면 그 호출은 `gap`(호출 안에서 getter가 Params를 다시 읽어 덮음)입니다. 관측 field(아래)를 읽습니다.

유효성 규칙(설계 §2.3-4, 사전 선언): BA run은 memprobe `outKeyframeCreated = 1`인 모든 호출이 gap 아님, MSE run은 seq ≥ 2인 모든 호출이 gap 아님. 어긋나면 그 run은 INVALID(인프라)입니다.

## 출력 `poke.jsonl.gz`

1행 header(`schema = slam-refact-us026-config-poke-v1`, `keys`, `writes`, `wasmSha256`, `wrapped`, `error`, `modules`, `fields`, `prePostLayout`, `mseCheck`), 2행부터 호출당 객체 하나 `{i, t, <fields>}`. `i`는 memprobe.jsonl.gz와 같은 규칙(같은 seq)입니다.

| field | 의미 |
|---|---|
| `seq`, `callMs`, `root` | 호출 순번, 호출 시간(ms, memprobe 표본 포함), 같은 seq의 memprobe root(0 = 유효 root 없음) |
| `pre`, `post`, `gap` | `[PV, ba, baCache, mse, mseCache]` 호출 전/후, gap인 key 목록 |
| `baLatch` | u8 @608328 |
| `kfLive`, `kfNewest`, `kfPoseHash` | live keyframe 수, 최신 index(wm+0x1a8), forward extrinsic(kf+332, 16 f32) FNV-1a 64 |
| `kfPoseChanged`, `kfPoseMaxDt` | 직전 호출(같은 WorldMap)에 같은 (slot, kf seq)로 있었고 extrinsic bit가 바뀐 최신 아닌 keyframe 수, 그 translation 변화 최댓값 |
| `pts1Live`, `ptsHash1`, `pts1Changed` | type 1 pool live 수, [slot, 위치 4 f32] FNV, 같은 (slot, 생성 seq)인데 위치 bit가 바뀐 점 수 |
| `mseOk` | MSE = b+8 서명: f32 @MSE+2752 = 1.0, @MSE+2792 = 1.0(RollingWindowSum 창 = movementCheckPeriodInSec) |
| `mseScale`, `mseResidual`, `mseSum`, `mseHave`, `mseActive` | MSE +2824, +2828, +2816 f32, +2821, +2812 u8 |
| `mseKfHash`, `mseImuHash` | 9-state x·P, 6-state x·P FNV(`mse_precision_golden.mjs` 규칙) |
| `vtLatch`, `vtLatchedScale` | VioTracker latch u8 @b+2840, f32 @b+2844 |

`mseCheck`(header): root가 바뀔 때마다 첫 표본에서 설계 §2.2 검사(`scaleIs1`, `minLinIsFltMax`)와 창 검사(`windows`)를 기록합니다. §2.2 검사는 첫 linacc event 뒤에는 성립하지 않을 수 있어(+2808이 갱신됨) `mseOk`에는 창 검사를 씁니다.

## 실행 (저장소 루트, dev 시나리오만)

```bash
# precheck + flock build/c1-chromium.lock + dev-run-log 기록. dev 시나리오(flip6-n1, flip6-arcore-s3-absolute)가 아니면 거부
bash slam_refact/tools/re/runtime/us026/dev-run.sh flip6-n1 <새 label> "<목적>" \
  --memprobe --record-inputs --memprobe-detail 12 --poke '{"ba":1}'
bash slam_refact/tools/re/runtime/us026/dev-run.sh flip6-n1 <새 label> "null-poke" \
  --memprobe --record-inputs --memprobe-detail 12 --poke '{"ba":0,"mse":1}'
# exact-input bundle + params 변형, replay, 진단
bash slam_refact/tools/harness/us026/make_bundles.sh flip6-n1 <label> on="MapBuilder.bundleAdjustmentIterativeEnabled=b:1"
python3 -I slam_refact/tools/harness/divergence/exact_input_report.py --bundles <list> --replays build/slam-refact-full-20261019/us026/replays \
  --out <새 dir> --binary build/slam-refact-full-c2-20261020/harness/slam_refact_replay --run --jobs 2
python3 -I slam_refact/tools/harness/us026/poke_summary.py --runs <run dir>... --out <새 json>       # V1–V4
python3 -I slam_refact/tools/harness/us026/compare_us026.py --original <run dir> --engine <replay dir> \
  [--engine-off <replay dir>] --out <새 json>                                                       # G-BA-D/G-BA-S 초안 수치
```

- 출력: `build/slam-refact-full-20261019/us026/{runs,bundles,replays,reports,compare}/`, dev 실행 기록 `build/slam-refact-full-20261019/us026-draft/dev-run-log.tsv`.
- `run-reference.mjs`에서 `--poke`와 `--memprobe-detail`은 `--memprobe`가 있어야 하고, 둘 다 기본 off입니다. 끈 상태의 형식 호환 증거: `compat_check.py`, `build/slam-refact-full-20261019/us026/compat/`.
- 원본은 부하에 민감합니다. 부하 약 30에서 만든 null-poke run은 exact-input replay와 frame 29부터 달랐고, 부하 약 3에서는 bit 동일했습니다(dev 보고서 NR-BA-2). 원본 run은 부하가 낮을 때만 만듭니다.
