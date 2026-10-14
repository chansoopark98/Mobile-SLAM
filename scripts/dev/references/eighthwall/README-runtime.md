# 8th Wall 원본 binary 공개 API 실행

- 목적: 기존 공식 standalone SDK를 실제 Chromium에서 초기화·실행하고 공개 camera pipeline 결과 수집.
- 환경: Node22 이상, 기존 `scripts/dev/node_modules/playwright`1.63.0, `/usr/bin/google-chrome`. `BROWSER_EXECUTABLE`로 별도 Chrome 지정 가능.
- 자산: `assets/references/8thwall/analysis-runtime/standalone/{xr.js,xr-slam.js,LICENSE}`. 없으면 기존 `bootstrap.py`의 고정 공개 archive 획득 절차 사용.
- 입력: 기존 `build/mobile-web-20261009/synthetic/clean-seed101-v2/input.json` 및 gray PGM. `--dataset`으로 같은 schema의 별도 fixture 지정 가능. GT trajectory는 읽거나 SDK에 전달하지 않음.
- 재사용: `runtime.js`의 camera/sensor 가상 입력·public pipeline observer, `sensor_adapter.mjs`의 raw IMU 변환. 기존 파일 수정 없음.
- 이번 wrapper: `observer.js` 미사용, `WebAssembly.instantiate`·WASM exports·SDK source 수정 없음. 기존 `run_browser.mjs`의 export 호출 observer와 구분.
- 각 실행은 전용 ephemeral localhost server·새 headless browser context 사용. 외부 HTTP 차단, 종료 시 자체 stream·browser·server 정리.

```bash
node --check scripts/dev/references/eighthwall/runtime-probe.mjs

# label은 매 실행 새 값 사용; 기존 evidence 덮어쓰기 거부
node scripts/dev/references/eighthwall/runtime-probe.mjs \
  --label example-mobile-responsive-400 --mode mobile --scale responsive --frames 400

node scripts/dev/references/eighthwall/runtime-probe.mjs \
  --label example-mobile-absolute-400 --mode mobile --scale absolute --frames 400

node scripts/dev/references/eighthwall/runtime-probe.mjs \
  --label example-desktop-responsive-120 --mode desktop --scale responsive --frames 120
```

- 출력: `build/commercial-slam-20261013/8thwall-runtime/<label>/result.json`, `source/`, `screenshot.png`.
- `ACTUAL_PUBLIC_API_TRANSFORM_OUTPUT`: 원본 SDK의 공개 callback에서 finite position·quaternion 관측. tracking accuracy·metric initialization 통과 판정 아님.
- `CAMERA_CALLBACKS_WITHOUT_TRANSFORM`: camera callback은 있으나 finite transform 없음. desktop control의 실제 관측 상태.
- `NO_CAMERA_CALLBACKS`/`FAILED`: timeout·console·page/network error를 `result.json`에서 확인. 성공으로 숨기지 않음.
- exit0은 위 두 정상 관측 상태 모두에 해당. shell exit만으로 SLAM pose 발행·tracking 수락을 판단하지 않음.
- `mobile`은 Android UA·touch·viewport의 가상 device emulation. physical phone·OS fusion·IMU latency를 재현하지 않음.
- callback은 같은 결과를 반복할 수 있음. `callbacks`, `nonrepeatCallbacks`, source publication 수를 별도 기록; 정확한 SDK 내부 처리 frame 수와 동일시하지 않음.
- 상세 결과·공식 계약·차이: [2026-10-13-8thwall-runtime.md](../../../../docs/research/2026-10-13-8thwall-runtime.md).
