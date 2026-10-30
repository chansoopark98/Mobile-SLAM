# LC-6a 진단 바이너리 (S2 덤프), NOW-2

이 문서는 리뷰가 지정한 NOW-2(엔진 S2·공유 점 수 덤프)의 결과 문서입니다. 리뷰가 지정한 문서 이름(`us025-lc-v3-room2-diagnosis.md`)과 이름이 다릅니다(도구와 같은 디렉터리에 둠). 위치 분석(NOW-1, NOW-3)은 `slam_refact/docs/design/us025-lc-room2-positional-diag-2026-10-29.md`에 있습니다.

dev 진단입니다. gate 판정, 비교 기준, pin 파일은 바꾸지 않았고 이 결과는 PASS 근거가 아닙니다(D-V3-LOCK). 모든 실행은 desktop native replay입니다. 라벨: VERIFIED(실행 관측), INFERRED(추정), UNVERIFIED.

## 만든 것

| 파일 | 내용 |
|---|---|
| `lc6a_diag_main.cc` | `lc6a_replay_main.cc`(pin sha256 `fa1d7796…d17d`, 수정 없음)의 사본. `DIAG` 표시 hunk만 다릅니다(`diff`로 확인). 시도마다 `<out>/lc6a-s2-diag.jsonl` 한 줄을 추가로 씁니다 |
| `build_lc6a_diag.sh` | pin 커밋 `d11d3e0abca62604de28b1db5863e39d198877eb`를 `git archive`(`slam_refact/engine`, `slam_refact/tools/harness/replay`)로 `build/slam-refact-review-20261029/lc6a-diag/src`에 풀고, pin의 native-build와 같은 cache 값(Release, `SLAM_REFACT_LOOP_CLOSE=ON`, `WERROR=ON`, `BUILD_HARNESS=ON`, `BUILD_TESTS=OFF`, 오프라인 Ceres 2.2)으로 빌드합니다. 컴파일 플래그와 링크 줄, `--wrap` 심볼은 `build_lc6a.sh`와 같은 방식(maphash target의 `compile_commands.json`, `ninja -t commands`)입니다. jobs는 1 또는 2 |
| `DIAG.md` | 이 문서 |

비침습 방식: `map_builder::LoopClose` linker `--wrap`에서 실제 호출 전에 const 접근자(`WorldMap::CovisibleKeyframes`, `KeyframeAtSlot`, `Keyframe::frame.pointsViewedByKeyframe()`)만 써서 `FindTarget` S4(`loop_close.cc` 104-117)와 같은 순서로 h1, h2, S2를 다시 계산해 덤프합니다. setter 호출 없음, 실제 호출 인자 그대로, `lc6a-attempts.jsonl` 내용 변경 없음.

`lc6a-s2-diag.jsonl` 한 줄의 필드: `frame`, `nSlots`, `newest`[idx,id], `minShared`(=10), `nS2`, `nS1`, `h1`(1143(newest)), `s1`(h1과 h2의 합집합에서 newest 제외, 핸들), `s2`(슬롯 index 오름차순), `newestShared`(newest의 `pointsViewedByKeyframe` 전체 [idx,id,공유 점 수]), `h1Detail`(h1마다 `h2` 목록과 그 `shared` 전체), `outcome`, `cand`(실제 호출이 고른 후보). 슬롯 수 5 미만이거나 newest가 무효인 시도는 `"skipped":true`.

## 산출물과 명령 (VERIFIED)

- 빌드: `bash slam_refact/tools/harness/us025/lc6a/build_lc6a_diag.sh 2` 종료 코드 0(로그 `build/slam-refact-review-20261029/lc6a-diag/build.log`, 324 step). 바이너리 `…/lc6a-diag/bin/slam_refact_replay_lc6a_diag`, sha256 `9d8d8cd2…ed459`(`bin/binaries.sha256`).
- 입력 라이브러리 sha256은 pin의 `input-libs.sha256`과 다릅니다(다른 build 경로의 재빌드, 비트 동일성은 요구하지 않음). 비침습성은 아래 출력 비교로 확인했습니다.
- replay(2개 동시, 입력은 v3 gate bundle): `…/lc6a-diag/bin/slam_refact_replay_lc6a_diag --bundle build/slam-refact-pkgh-20261022/us025/gate-runs/us025-lc-e2e-gate-v3/us025-lc-e2e-gate/tum-room2-full/bundles/tum-room2-full/gate-lc1-0{3,4}-on --out build/slam-refact-review-20261029/lc6a-diag/replays/gate-lc1-0{3,4}-on`. 둘 다 종료 코드 0(gate 때 replay 76개도 전부 0), wall 479 s, 485 s. 출력은 `…/lc6a-diag/replays/gate-lc1-0{3,4}-on/`.

## 비침습 검증 (VERIFIED)

gate v3 출력(`<G>/tum-room2-full/replays/gate-lc1-0{3,4}-on/`)과 비교했습니다.

| 파일 | lc1-03 | lc1-04 |
|---|---|---|
| `accuracy-config.json` | `cmp` 동일 | `cmp` 동일 |
| `lc6a-attempts.jsonl`(154줄, 166줄) | `cmp` 동일 | `cmp` 동일 |
| `worldpoints-sampled.jsonl` | `cmp` 동일 | `cmp` 동일 |
| `trace.jsonl` | `srStageMs`(단계 시간, wall-clock) 값을 지우면 sha256 동일(`c2e37832…`, `58a313bf…`) | 같음 |
| `meta.json` | `bundle` 경로 표기(절대/상대)와 `wallSeconds`만 다름 | 같음 |

`trace.jsonl`의 원시 `cmp`는 `srStageMs` 때문에 다릅니다(첫 차이 byte 682). 이 필드는 실행 시간이라 어떤 두 실행도 다릅니다. 비교 규칙은 사후 리뷰 §3의 지정 제외 필드와 같습니다.

## S2 덤프 (VERIFIED: 엔진 쪽 값)

`<D>` = `build/slam-refact-review-20261029/lc6a-diag/replays`. 두 시도 모두 `outcome` 3(kTooFewMatches, 후보 선택 뒤 매칭 부족), 후보 선택까지만 의미가 있습니다.

### gate-lc1-03, frame 1988 (`<D>/gate-lc1-03-on/lc6a-s2-diag.jsonl`)

- newest `[4, 1342934579]`, nSlots 26, `minShared` 10.
- `h1` = 슬롯 `{12, 20, 24, 3}`(공유 21, 30, 105, 149). 슬롯 6은 공유 **9**라서 제외입니다(경계 바로 아래, H1b의 9/10 경계에 해당).
- **S2 = {3, 5, 6, 9, 11, 12, 14, 20, 21, 24}** (10개, newest 제외). 슬롯 0, 1, 2는 S2에 **없음**.
- 엔진의 `cand` = 슬롯 0(id 3111974223). 원본 pick은 슬롯 3(id 3038758246).
- newest의 공유 점 수 전체: 슬롯 12:21, 24:105, 6:9, 3:149, 20:30. 슬롯 0, 1, 2와 newest의 공유는 없음(항목 없음).

### gate-lc1-04, frame 1503 (`<D>/gate-lc1-04-on/lc6a-s2-diag.jsonl`)

- newest `[24, 298376053]`, nSlots 28, `minShared` 10.
- `h1` = 슬롯 `{12, 5, 17, 25, 4, 14}`(공유 16, 12, 11, 13, 25, 46). 슬롯 3(공유 **9**)과 8(공유 7)은 h1이 아닙니다.
- **S2 = {3, 4, 5, 6, 7, 8, 9, 12, 14, 17, 21, 23, 25}** (13개). 슬롯 0, 1, 2는 S2에 **없음**. 슬롯 3, 8은 h1이 아니라 h2 경유입니다.
- 엔진의 `cand` = 슬롯 0(id 2790389203). 원본 pick은 슬롯 4(id 510507183).
- newest의 공유 점 수 전체: 슬롯 4:25, 9:4, 11:1, 3:9, 5:12, 8:7, 14:46, 17:11, 12:16, 25:13, 21:3.

### 이 시도에서 후보가 갈리는 방식 (VERIFIED + 계산)

두 시도 모두 모든 keyframe의 `orbN`이 0이고 global descriptor가 전부 같습니다(엔진 `desc` 값 1종류, sha256이 원본 `newestAgate` `f257ca9c…`와 같음). 그래서 거리가 모두 0이고 pick은 "S2, ring, newest에 걸리지 않는 첫 슬롯 위치"입니다. 엔진은 S2에 없는 슬롯 0을 고르고, 원본은 슬롯 0, 1, 2(f1503은 3도)를 건너뛰었습니다. ring은 두 시도 모두 newest가 바뀌어 초기화되므로(원본 `ringPre` 앞 두 값 ≠ newest) 건너뜀을 설명하지 못합니다.

## 원본 쪽에서 S2를 추론할 근거 (INFERRED, hook 관측 아님)

- 근거 위치: 원본 gate run의 `lcgold/events.jsonl`(`<G>/tum-room2-full/runs/tum-room2-full/gate-lc1-0{3,4}/lcgold/`)의 `covis` 필드. 시도마다 모든 슬롯의 `pointsViewedByKeyframe` 전체 [슬롯, id, 공유 점 수]가 `--memprobe-lc` 메모리 읽기로 기록되어 있습니다. **1143 hook 출력이 아니라 시도 호출 뒤의 메모리 상태**입니다. 따라서 LoopClose 호출 시점의 상태와 같다는 보장은 없습니다(실패한 시도는 LoopClose가 맵을 바꾸지 않으므로 같다고 보는 것이 자연스러우나 INFERRED).
- 이 `covis`에 엔진과 같은 규칙(공유 ≥ 10으로 h1, h1의 h2, 합집합에서 newest 제외)을 적용한 S2가 **엔진 덤프 S2와 일치**합니다: lc1-03 145/145 시도, lc1-04 157/157 시도(슬롯 5 이상, 덤프와 `covis`가 모두 있는 시도). f1988, f1503 포함. 즉 view 장부(공유 점 수)가 두 시도에서 원본 메모리와 엔진이 같다는 쪽의 증거입니다(INFERRED).
- 그러나 원본의 pick은 이 S2로 설명되지 않습니다. lc1-01..04의 시도 전부에서 "원본이 건너뛴 live 위치가 covis로 계산한 S2에 없는 경우"는 **f1988(위치 0, 1, 2)과 f1503(위치 0, 1, 2) 뿐**입니다(lc1-01, lc1-02는 0건. ring은 빼지 않았으므로 ring이 설명할 수 있는 건너뜀도 "설명 안 됨" 쪽으로 세어지는 보수적 계수입니다. 위 두 시도는 ring이 초기화된 시도라 영향이 없습니다). 또 f1988에서 원본이 고른 슬롯 3, f1503에서 고른 슬롯 4는 covis로 계산한 S2에 **들어 있습니다**.
- 해석 후보(모두 UNVERIFIED):
  - (a) 원본의 S2 집합이 호출 시점에는 이 `covis` 후처리 상태와 달랐다(예: 호출 뒤 장부가 바뀜). 엔진은 S2가 같다고 보이므로 엔진 장부 쪽 결함 가능성은 이 증거로는 낮아집니다.
  - (b) 원본의 후보 선택 또는 S2 구성이 spec §3의 서술과 다른 경로(G936:4606-4745의 트리 순회, U2/U3 계열)를 탄다.
  - (c) 원본의 위치/슬롯 대응이 이 두 시도에서 다르다(압축 목록 위치 vs 슬롯 index).
- 확정하려면 원본 1143 hook 출력(USER-2, Chromium 필요)이 필요합니다. **원본이 f1988, f1503에서 S2에 슬롯 0, 1, 2를 넣었는지는 UNVERIFIED**입니다. 이 문서는 엔진 결함 여부나 원인을 판정하지 않으며, 그 frame에 맞추는 수정은 금지입니다(CLAUDE.md §2).

## 노출 기록 (사용한 시나리오와 run)

읽은 시나리오와 run은 아래뿐입니다. 이 목록 밖의 gate run, replay OFF 팔, null 팔, 다른 S 팔 run, 다른 시나리오는 열지 않았습니다.

| 구분 | 시나리오 | run | 읽은/실행한 것 |
|---|---|---|---|
| 원본 `covis` 대조 (읽기) | `tum-room2-full` | `gate-lc1-01`, `-02`, `-03`, `-04` (4개 전부, 원본 ON 팔) | 원본 `runs/tum-room2-full/<run>/lcgold/events.jsonl`의 `covis` 필드("원본 쪽에서 S2를 추론할 근거"의 lc1-01..04 시도 전부 계수) |
| 엔진 diag replay (실행) | `tum-room2-full` | `gate-lc1-03-on`, `gate-lc1-04-on` (gate v3 번들, ON 팔만) | 번들 입력으로 `slam_refact_replay_lc6a_diag` 실행, 출력과 gate v3 replay 출력(`replays/gate-lc1-0{3,4}-on/`)의 비침습 비교 |

- 엔진 diag replay는 lc1-03, lc1-04 ON만 실행했습니다. lc1-01, lc1-02의 엔진 replay와 OFF, null 팔, 다른 시나리오(room1, room4, s3-lowtex, flip6-s2 등)는 실행하지도 열지도 않았습니다.
- lc1-01, lc1-02는 원본 `covis`만 읽었고(엔진 덤프 없음), "원본이 건너뛴 위치가 S2에 없는 경우는 f1988과 f1503뿐"이라는 계수에 쓰였습니다.
- 위치 분석 문서 §1의 노출 기록과 겹칩니다(같은 gate v3 room2 run). 이 pass가 새로 더한 노출은 엔진 `lc6a-s2-diag.jsonl`(lc1-03, lc1-04)과 원본 `covis` 필드입니다.

## 재현

```
bash slam_refact/tools/harness/us025/lc6a/build_lc6a_diag.sh 2
build/slam-refact-review-20261029/lc6a-diag/bin/slam_refact_replay_lc6a_diag --bundle <G>/tum-room2-full/bundles/tum-room2-full/gate-lc1-03-on --out <새 디렉터리>
```

`--out`은 새 디렉터리여야 합니다. 시나리오당 프로세스 1개 규칙(엔진 전역 설정)을 지키세요.
