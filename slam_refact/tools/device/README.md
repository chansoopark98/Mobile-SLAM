# 기기 비교 측정 (지도 유지 옵션 K1·K1F)

Galaxy Z Flip6(Chrome)에서 배포본을 주소만 바꿔 세 번 돌리고, 받은 지표 JSON을 한 명령으로 비교합니다. 5분 안에 끝납니다.
K1·K1F는 2026-11-16 gate에서 **채택되지 않은 실험 옵션**입니다(`build/xr-commercial-20261116/DECISION-product-default.md`). 기본 주소는 이전과 같은 엔진 설정으로 동작합니다.

## 1. 주소 (HTTPS 서버가 떠 있어야 합니다. 시작·중지는 사용자가 합니다)

| 모드 | 주소 |
|---|---|
| 기본 (이전과 같은 엔진) | `https://dev.serdic.com:7002/slam-refact-xr.html?record=1&diag=1&run=d1` |
| K1 (지도 유지) | `https://dev.serdic.com:7002/slam-refact-xr.html?record=1&diag=1&keepmap=1&run=k1` |
| K1F (지도 유지 + 빠른 확장) | `https://dev.serdic.com:7002/slam-refact-xr.html?record=1&diag=1&keepmap=fast&run=f1` |

`record=1&diag=1`은 pose와 엔진 tracker 상태를 기록합니다(영상과 IMU 원본은 기록하지 않음. 프레임별 gyro·가속도 요약값 3개는 포함).

## 2. 동작 (모드마다 같은 동작을 약 40초)

1. 시작을 누르고 바닥이나 책상을 비춥니다. "준비됐어요"가 뜨면 물체를 하나 놓습니다.
2. **놓은 방향을 3초 동안 그대로 봅니다.** (분석 도구가 이 구간을 "처음 본 방향"으로 씁니다.)
3. 몸을 돌리듯 폰을 빠르게 옆으로 돌립니다(1초 안에 90° 정도). 2–3초 다른 곳을 봅니다.
4. 처음 방향으로 돌아와 2–3초 봅니다. 물체가 놓은 자리에 있는지 눈으로 확인합니다.
5. 3–4를 다섯 번 반복합니다.
6. 도구를 열어 "지표 JSON 받기"를 누릅니다. 받은 파일을 PC로 옮깁니다.

K1·K1F 모드에서는 지도 추적이 끊긴 동안 안내가 "주변을 다시 찾는 중"으로 바뀝니다.

## 3. 비교

```bash
python3 slam_refact/tools/device/compare_keepmap_runs.py d1.json k1.json f1.json
```

볼 숫자:
- `map discards`: 지도를 버린 횟수. 기본 모드에서 빠르게 돌 때 생기던 것입니다(2026-11-16 f1 기록: 43초에 8회). K1·K1F에서는 0이어야 합니다.
- `extension keyframes`: K1·K1F가 버리는 대신 keyframe을 추가한 횟수. `longest consecutive run`이 1보다 크면 연속 확장이 일어난 것입니다.
- `map-lost frames`, `lost episodes`: 지도 추적이 끊긴 프레임 수와 길이.
- `same-view ... median`: 처음 본 방향을 다시 봤을 때의 위치 차이(페이지 상대 단위, 시작 높이가 1.5–2). 물체가 어긋나 보이는 크기입니다.
- `latency ms map-lost`: 끊긴 프레임의 처리 시간. 지도를 유지하면 끊긴 프레임마다 더 큰 지도와 맞춰 보므로 늘어납니다(desktop 재생에서 약 12 → 28 ms).
- `engine mode`, `flags active`: 엔진이 옵션을 실제로 받았는지. WARNING이 나오면 그 run은 옵션 없이 돈 것입니다.

기준 값(2026-11-16 기본 모드 기록으로 도구를 돌린 결과):

```bash
python3 slam_refact/tools/device/compare_keepmap_runs.py build/xr-tracker-diag-20261116/device/f1-metrics.json
```

`map discards 8`, `preceded by gyro > 80 deg/s 8/8`, `map-lost frames 58`, 끊긴 프레임 지연 p50 49.9 ms(추적 중 39.2 ms).

## 4. 한계

- GT가 없습니다. 위치는 상대 단위입니다.
- 모드마다 run 하나로는 통계가 되지 않습니다. 같은 동작으로 두 번씩 하면 더 낫습니다.
- 기록은 엔진이 main thread에서 돌 때만 됩니다(`engineWorker=1`에서는 tracker 상태가 비어 있음).
