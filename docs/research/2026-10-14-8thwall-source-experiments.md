# 8th Wall 공개 소스 직접 실행 — 2026-10-14

- 목적: 문서·이름 추정 외에 데이터 변환과 module lifecycle의 실제 동작 확인.
- 대상: 공식 `8thwall/8thwall@4edfa484936a2cc691366eeec5c6e58bb6d4c8bd`의 원본 TypeScript 5개.
- 실행: Node **v22.21.1**의 `stripTypeScriptTypes(mode: 'strip')`로 타입만 제거한 뒤 모듈 import. 원본 파일 수정 없음, 패키지 추가 없음.
- 증거: [result.json](../../build/8thwall-deep-20261014/source-contracts/result.json); 같은 폴더에 원본 `.ts`, 실행 `.mjs`, 실험 script snapshot 및 SHA256 보존.
- 결과: **9/9 독립 기대값 검사 통과**. upstream 전체 test suite·typecheck·Bazel build·배포 바이너리 parity·SLAM 정확도 검증과 구분.
- Node의 experimental API warning은 보존. 재현 환경을 Node 버전과 함께 고정.

## 확인한 동작

| 실험 | 독립 입력·기대값 | 실제 결과·의미 |
| --- | --- | --- |
| Crop scale | 640×480 → 320×320 | `max(320/640, 320/480)=2/3`, 변환 결과 426.667×320. cover 후 crop할 스케일 |
| 영상 90° 회전 | 2×3 ID 영상 `[1,2;3,4;5,6]` | CW `[5,3,1;6,4,2]`, CCW `[2,4,6;1,3,5]`; 색 채널·alpha 보존 |
| Letterbox padding | 77로 채운 4×4 버퍼 중앙에 2×2 입력 | 중앙만 교체하고 테두리77 유지. helper 자체가 black padding을 채우지는 않음; 호출자가 초기화해야 함 |
| 색 정규화 | Float32 `[0,127.5,255]` | `[0,0.5,1]` |
| Chunk URL parsing | `slam, face: https://local.test/xr-face.js, , sky:` | 빈 segment 제거, URL의 colon 보존, URL 없는 chunk는 `null` |
| Callback 제거 | a,b 등록 → callback 배열 보관 → a 제거 → 보관 배열 실행 | b만 호출. 제거된 module의 오래된 callback도 deactivate됨 |
| Callback wrapper | onStart와 onUpdate에 wrapper 등록 | `start,wrapped,update`. frame callback만 wrap, lifecycle callback 제외 |
| Pause 중 module 추가 | a attach → pause → b 추가 → resume → a 제거 → detach | `start:a,start:b,stop:a,stop:b`; pause 중 b 즉시 시작 안 함 |
| 동적 resource set | base + 함수 provider, provider 값 변경 | 다음 get에 변경 반영, getCopy 수정은 원본 격리, provider 제거 시 base만 유지 |

- Letterbox 결과는 helper 책임 범위의 관측이며, 실제 SDK 화면의 검은 테두리 결함을 재현한 것은 아님.
- Crop·회전 검사는 제한된 짝수 padding/유효 치수 사례. 홀수 padding·invalid dimension 전 범위 수락으로 확대하지 않음.
- callback manager 자체의 중복 이름 검증은 이번 실험 대상 아님. 상위 module 등록 gate와 혼동하지 않음.

## 원본 근거

- [pixel-transforms.ts](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/pixel-transforms.ts): crop, letterbox, 회전, normalize.
- [preload-chunks-format.ts](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/preload-chunks-format.ts): preload attribute → chunk specifier.
- [callback-manager.ts](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/callback-manager.ts#L315): callback wrapper·deactivation.
- [lifecycle.ts](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/lifecycle.ts): module attach/pause/resume/detach.
- [set-manager.ts](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/set-manager.ts): named resources와 dynamic provider.

## 재현

```bash
node --check scripts/dev/references/eighthwall/source-contracts.mjs
node scripts/dev/references/eighthwall/source-contracts.mjs \
  build/8thwall-deep-20261014/source-contracts-rerun
```

- 출력 경로는 기존에 없는 로컬 `build/` 하위 폴더만 사용. 기존 결과 덮어쓰기 거부.
- 실험기: [source-contracts.mjs](../../scripts/dev/references/eighthwall/source-contracts.mjs).
- 브라우저 원본 바이너리 실행은 [별도 실험 보고서](2026-10-14-8thwall-experiments.md) 참조.
