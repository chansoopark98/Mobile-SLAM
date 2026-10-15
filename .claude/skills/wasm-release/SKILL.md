---
name: wasm-release
description: 현재 C++ 소스로 Mobile-SLAM WASM(vio_engine.js)을 다시 빌드하고 provenance를 기록한 뒤, 사용자가 명시적으로 요청하면 web/에 승격하는 절차. "WASM 빌드", "서버 파일 갱신", "vio_engine.js 업데이트", "배포" 요청에 사용한다.
---

# WASM 빌드·승격

`web/vio_engine.js`는 실기기가 서빙받는 파일이다. 직접 수정하거나 복사하지 않는다.
- `.claude/settings.json`은 Edit/Write 도구만 차단한다.
- `cp`, `sed -i`, `tee`, `python3 -c` 같은 Bash 명령은 차단되지 않는다. 이 규칙은 스스로 지켜야 한다.

## 1. 빌드 (승인 불필요)

```bash
git status --short     # dirty 여부 기록 — provenance에 dirty diff hash가 남음
bash scripts/dev/check.sh wasm-build                   # → build/dev-wasm/vio_engine.js (SINGLE_FILE, embedded wasm)
node scripts/dev/wasm-smoke.mjs build/dev-wasm/vio_engine.js build/dev-wasm/wasm-smoke.json   # 출력 경로 생략 시 기존 증거(build/refactor-evaluation/)를 덮어씀
python3 scripts/dev/build-provenance.py --build build/dev-wasm \
  --artifact build/dev-wasm/vio_engine.js --output build/dev-wasm/vio_engine.manifest.json
sha256sum build/dev-wasm/vio_engine.js
```

- lane을 분리해야 하면 `emcmake cmake -S wasm -B build/<lane>-wasm -G Ninja`를 쓴다. emsdk는 `source ~/emsdk/emsdk_env.sh`로 활성화한다.
- 새 ABI(예: `getInertialPrediction`)가 들어갔는지 확인한다: `grep -c <symbol> build/dev-wasm/vio_engine.js`.

## 2. 승격 전 검증

- 기존 WASM(`web/vio_engine.js`)과 같은 입력으로 replay해 pose, status, solver, map이 같은지 확인한다.
  - 의도한 변경이 아니라면 timer 외에는 같아야 한다.
  - 예: `scripts/dev/synthetic-browser.mjs`, `scripts/dev/replay-android-input.mjs --loader <candidate>`
- `bash scripts/dev/check.sh browser-contracts`가 통과해야 한다.
- 기본으로 켜지는 새 기능이 있다면 gate를 통과했는지 확인한다.
  - 예: low-feature는 바닥 GT gate에 미달했다. 기본 OFF 플래그 없이 승격하면 안 된다.
- `web/js/*.js`를 바꿨다면 import의 cache-busting query(`?v=N`)를 일관되게 올린다.

## 3. 승격 (사용자의 명시적 요청이 있을 때만)

1. 사용자에게 candidate sha256, 크기, 바뀐 내용, 검증 결과를 보여주고 승인을 받는다.
2. 승인을 받은 뒤에만 **새 lane 경로**에 authorization 파일을 쓴다. 예: `build/<lane>-YYYYMMDD/deploy-authorized.json`.
   - 기본 경로 `build/refactor-orchestration/deploy-authorized.json`은 2026-10-06 승인 기록이다. 덮어쓰지 않는다.
   - 필수 필드는 `authorized`, `candidate_sha256`이다. 나머지 필드는 기록용이다.
   ```json
   {"authorized": true, "candidate_sha256": "<sha256>", "approved_by": "user", "reason": "<요청 요지>"}
   ```
3. 승격한다:
   ```bash
   python3 scripts/dev/deploy-candidate.py --candidate build/dev-wasm/vio_engine.js \
     --manifest build/dev-wasm/vio_engine.manifest.json \
     --authorization build/<lane>-YYYYMMDD/deploy-authorized.json
   ```
   - 원자적으로 교체된다.
   - 이전 파일은 `build/refactor-deploy/<ns>/`에 백업된다.
   - `web/vio_engine.manifest.json`에 deployment 정보가 기록된다.
4. `sha256sum web/vio_engine.js`가 candidate의 hash와 같은지 확인한다.
5. 서버 재시작은 사용자가 한다(`scripts/ops/serve.sh`). 내가 시작하지 않은 `node server.js`는 종료하지 않는다.
6. 결과를 `docs/fixes/YYYY-MM-DD-<topic>.md`에 기록하고 `docs/STATUS.md` §1의 활성 WASM 항목을 갱신한다.

## 롤백

`build/refactor-deploy/<ns>/vio_engine.js`를 candidate로 지정해 같은 절차를 다시 밟는다. 롤백도 승인이 필요하다.
