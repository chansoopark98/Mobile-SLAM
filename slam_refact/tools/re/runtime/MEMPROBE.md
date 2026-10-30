# 원본 reference runtime memory probe (spec 03 §8.4 구현)

원본 xr-slam WASM(sha256 `780731fe…c422`)이 `c8EmAsm_processStagedFrame`에서 돌아올 때마다 linear memory를 **읽기만** 해서, MapBuilder·WorldMap·MapTracker 내부 상태를 frame마다 기록합니다.

- WASM bytes, import, SDK 파일은 바꾸지 않습니다. 원본 bytes를 그대로 compile하고 원래 import 객체로 instantiate합니다.
- glue에 돌려주는 exports 객체만 복사본입니다. 그 안에서 `c8EmAsm_processStagedFrame` 하나만 감쌉니다(`memprobe.js`).
- `--wrap`(import·export 전부 계측)과는 같이 쓸 수 없습니다.

## 실행

repo root에서 실행합니다. 브라우저는 한 번에 하나만 띄웁니다. 기존 label은 덮어쓰지 않습니다.

```bash
node slam_refact/tools/re/runtime/run-reference.mjs --scenario flip6-s2 --label <새 label> --memprobe [--max-frames N]
python3 -I slam_refact/tools/harness/eval/memprobe.py --run flip6-s2=build/slam-refact-20261015/reference/runs/flip6-s2/<label> \
  --reference-runs build/slam-refact-20261015/reference/runs --out-json <새 파일>   # 지표·검증·r1/r2 대비 대표성
python3 -I slam_refact/tools/harness/eval/parity_report.py --run ... \
  --memprobe flip6-s2=<probe run 1>,<probe run 2> ...                               # G-M3(b)/G-M4/G-M5/G-M7
```

- 출력 위치: `build/slam-refact-20261015/reference/runs/<scenario>/<label>/`
  - 원래 파일: `trace.jsonl`, `callbacks.jsonl`, `meta.json` 등 기존과 같습니다.
  - `memprobe.jsonl.gz`: 새로 추가된 probe 출력입니다.
  - `meta.json.memprobe`: record 수, root 이력, probe 시간과 호출 시간 통계입니다.
- `callbacks.jsonl`에는 `probeSeq` 열이 추가됩니다. 이 callback 직전에 실행된 `processStagedFrame`의 순번입니다.
  - trace의 `i`와 probe record를 잇는 데 씁니다.
  - fresh callback의 첫 `probeSeq`가 그 record의 `i`입니다.
  - 연결되지 않은 호출(bootstrap, repeat)은 `i = null`입니다.

## 파일 형식 (`memprobe.jsonl.gz`)

- 1행은 header입니다(`schema = slam-refact-xr_reference-memprobe-v1`). `fields`, `scans`(root 탐색 기록), `rootHistory`, `error`가 들어 있습니다.
- 2행부터는 `processStagedFrame` 호출 하나당 객체 하나입니다. `{i, t, <fields>}`를 담습니다.

| field | 주소 (b = VioTracker) | 의미 |
|---|---|---|
| `seq`, `tMs`, `callMs`, `probeMs` | — | 호출 순번, 반환 시각(performance.now), 호출 시간, probe 읽기 시간 (ms) |
| `root`, `rootScans` | — | 사용한 VioTracker 주소, 누적 heap scan 횟수 |
| `mbFrameCount` | `*(b+6596)+0xa10` i64 | MapBuilder `frame_count_` |
| `mbLastResetFrame` | mb+0xa08 i64 | `last_reset_frame_` (653이 `frame_count_`로 설정) |
| `mbLastStateChangeFrame`, `mbState` | mb+0xa18 i64, mb+0xa20 i32 | |
| `outMapReset`, `outKeyframeCreated` | b+6600 u8, b+6601 u8 | MapBuilderOutput [0], [1] |
| `outNewPoints` | b+6604 vector<PointHandle 8 B> 길이 | 이번 frame에 만든 ground+sky 점 |
| `trkState`, `trkFrameTrackerOk`, `trkConsecutiveOk` | trk=b+6768: +0x84 i32, +0x88 u8, +0x90 i64 | |
| `trkCandidates`, `trkSky` | trk+0x98, trk+0xa4 vector<u32> 길이 | ground candidate / sky 관측 index 수 |
| `frmWorldMatches` | `*(b+6456)+0x98` u32 | `worldPointIndices().size()` (skypoint gate, D-7) |
| `wm` | `*(b+7232)` | 현재 WorldMap |
| `kfSlots`, `kfFree`, `kfLive` | wm+0xc vector<Keyframe 984 B>, wm+0x18 free list, id(kf+0x10)≠0 수 | |
| `wmKfSeq`, `wmLastKeyframeFrame` | wm+0x8 u32, wm+0x1a0 i64 | |
| `newestKfIndex`, `newestKfCreatedFrame` | wm+0x1a8 u32 → kf+0x8 i64 | |
| `pts1`..`pts5` | pool k: wm+0x24+(k−1)·0x24 (vector<Point 72 B>), free list +0x18 | `[slot 수, free 수, live(state≠2), tenured(state=1)]` |
| `newGround`, `newSky` | keyframe frame만: type 2/3 slot 중 생성 seq(p+0x0) = 최신 keyframe seq(kf+0x0) | `[생성 수, frame 끝에 살아 있는 수]` |
| `fftAll`, `fftGround`, `fftSky` | f = `*(b+6460)`+0x88: +0xf4c/+0xf58/+0xf64 vector<PointMatch 20 B> 길이 | FrameFrameTracker 매치 수 |
| `fftLastOk`, `fftHeight`, `fftCalls` | f+0xf3c u8, f+0xb50 f32, f+0xf48 i32 | |

## root 탐색과 offset 근거

- 루트 탐색 절차:
  - 값이 `c8::VioTracker` vtable address point `0x250b0`인 u32 word를 heap에서 찾습니다.
  - 찾은 후보 b마다 사슬을 검사합니다. `*(b+6596)`, `*(b+6460)`, `*(b+7232)`가 memory 안을 가리키고, `frame_count_`가 0 이상 1e7 미만이어야 합니다.
  - 이 검사를 통과한 후보만 root로 씁니다.
  - 매 sample 전에 load 1회로 root를 다시 확인합니다. 실패하면 다시 scan합니다.
- 실측 결과 (VERIFIED):
  - 후보는 항상 1개입니다.
  - 첫 호출(seq 1)에는 사슬이 유효하지 않습니다.
  - seq 2부터 유효합니다.
  - seq 6–7(시작 recenter의 엔진 재생성 구간)에는 유효하지 않습니다.
  - seq 8부터는 같은 주소가 다시 유효합니다.
  - memory 16 MiB에서 scan 1회는 약 4.5 ms입니다.
- spec offset 검증 (VERIFIED):
  - 사슬 offset(vio+6596 mb, +6600 out, +6768 trk, +6456 frm, +7232 wm)은 dcmp 784 본문과 일치합니다(dcmp:84105–84124의 `f_aja(b[1649], b+7224, la=b[1614], na=b+6768, …, b+6600)`).
  - MapBuilder offset은 Ghidra 936.c:448–456(`param1+0x284` = frame_count_ ++, `+0x288` state)과 653.c:84–85(`*param3 = 1`, `param1+0x282 = param1+0x284`)에서 확인했습니다.
  - point pool layout은 657.c(type별 `param2+9/0x12/0x1b/0x24/0x2d` vector, free list `+0xf/0x18/0x21/0x2a`, Point stride 0x48, type +0x40, state +0x18, seq +0x0)와 1130.c(retire 때 free list `wm+0x3c/0x60/0x84/0xa8`에 push)에서 확인했습니다.
  - 런타임 불변식(`slot − free = live`, `frame_count_` +1/호출, `out` 벡터 길이 = 생성 ground + sky)이 모든 run에서 성립했습니다.
  - **spec 03 §2·§8.4의 offset 중 틀린 것은 발견하지 못했습니다.**
- FFT offset(spec 04 §3.7.1)은 값의 범위로만 확인했습니다(INFERRED). 예: 초기 height 1.0, 매 호출마다 `fftCalls` 증가.

## probe 참조 run과 r1/r2 대체 (G-M3(b), G-M4, G-M5, G-M7)

- 원본은 같은 입력에서도 실행마다 결과가 다릅니다(`docs/reports/parity.md` §3.1). r1/r2에는 memory 관측이 없습니다.
- 그래서 위 gate들의 "reference runs"는 **같은 원본으로 새로 실행한 probe run 2개**(label `pmr1`, `pmr2`)로 대체합니다.
- 이 대체가 등록된 `blocked_if` 조건("memory probe not built")을 해소하는 유일한 방법입니다.
- 등록된 tolerance 규칙과 class는 바꾸지 않았습니다(`thresholds-v1.json` sha256 `9a616652…` 그대로).
- 규칙을 적용하는 방식(중심값 = 두 run 평균, 중앙값 = sorted[n//2], 30 s 기준 = 첫 trace 행의 t)은 `eval/memprobe.py` docstring에 결과를 보기 전에 고정했습니다.
- probe run이 r1/r2를 대표하는지는 trace 수준 지표(첫 translation, 첫 world points, LIMITED 비율, world point p50, burst 5/6 비율)로 비교합니다(`memprobe.py --reference-runs`).
- 증거: `build/slam-refact-20261016/memprobe/` (summary, validation, commands, SHA256SUMS).

## 2026-10-16 probe run 메모 (`pmr1`, `pmr2`)

- 10개 run 모두 검증 hard check를 통과했습니다(`validation.json`).
  - 첫 MAP_RESET frame은 공개 첫 world points frame과 같고, reset frame의 새 점 수는 공개 world point 수와 같습니다.
  - keyframe 간격은 5 이상이고, live keyframe은 64 이하입니다.
  - free list 일관성, `frame_count_` +1, `out` 벡터와 생성 수의 일치도 모두 성립했습니다.
- `frm+0x98`(world matches)은 probe한 모든 frame에서 0입니다. D-7/D-22 가설을 확인했습니다(VERIFIED, 원본 실행 관측).
- spec 03 U1(TUM에서 첫 translation 뒤 33 frame 후 reset)
  - 해소: 연속 FFT 성공이 frame 164에서 끊깁니다.
  - frame 171은 연속 성공 7회지만 candidate가 14개(<20)입니다.
  - frame 172에서 연속 성공 8회, candidate 23개로 reset됩니다(VERIFIED).
- 한계: `pmr2`는 다른 작업 때문에 load 27–33인 상태에서 실행했습니다.
  - `tum-room1-full/pmr2`는 fresh가 2483/2821이고(r1 2812, r2 2805), 공개 world point p50이 0입니다(r1 29, r2 40). 대표성이 떨어집니다.
  - "±2·run 간 차" 규칙은 이 차이만큼 tolerance를 넓힙니다. 해석할 때 이 점을 고려해야 합니다.

## 비도달 경로 field (`--memprobe-paths`, 2026-10-18)

- `--memprobe`와 함께 줄 때만 켜집니다. 끄면 header `fields`와 record key가 이전과 같습니다(`build/slam-refact-20261018/re-paths/compat-default-output.json`).
- 켜면 기존 40개 field 뒤에 12개를 덧붙입니다: `mtRelocPending`(MapTracker+0x12ac), `mtRelocKf`(+0x12a4), `mtKfCreated`(+0x12b0, 대조군), `outRelocKf`/`outRelocKfId`(b+6624 / b+6616 = MapBuilderOutput+0x18/+0x10), `vtIncreasePts`(b+6208), `vpsMode`(b+2980 u16), `engineOk`, `holderVps`(holder+2200..2202), `mbCfg`(func651 설정 byte 9개), `frmPoints`(`*(b+6456)` FramePoint 0x50 vector 길이), `vtCfgPts`(531848/531852 i32).
- offset 근거와 해석: `memprobe.js` 주석, `slam_refact/docs/reports/re-unreachable-paths.md`. 요약: `summarize_paths.py`.
