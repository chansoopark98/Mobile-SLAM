# 최신 서버 파일 준비 — 2026-10-14

- 사용자 요청: 서버 파일을 최신 버전으로 맞추고, 서버 실행은 사용자가 직접 수행.
- 반영: 현재 소스와 일치하는 기본 WASM(C3 OFF)을 `web/vio_engine.js`에 원자적으로 승격. 최신 App/Renderer U 수정은 기존 작업 내용을 유지.
- SHA256: `38837d4b79abb088610484f4e2c016aac4336fe9761dcfc56874d48e64785589`, 6,044,038 bytes.
- `web/vio_engine.manifest.json`과 `build/refactor-orchestration/server-start-authorized.json` 동기화. 수동 deployment ID `mobile-slam-latest-20261014`, embedded WASM, 외부 WASM 불필요.
- 현재 소스86개·의존성792개 hash 일치, 최신 기본 OFF artifact와 동일. 기존 browser297/297·exporter3/3·현재 App 합성 실행 근거 재확인.
- 승격 권한은 이번 명시적 사용자 요청. 버전 갱신이며 실제 모바일 정확도 수락을 뜻하지 않음. 알려진 원본 phone prefix53/300fresh·추적 한계는 유지.

## 서버 실행 상태

- 이 작업에서 `start`, `stop`, `restart`, `listen` 호출 없음. 별도 테스트 listener도 만들지 않음.
- 준비 시작 때7002 listener 없음. 확인 도중 별도로 실행된 `node server.js` PID577654를 관측. 해당 프로세스에 신호·재시작 없이 보존.
- 관리 helper의 소유 PID record는 없음. 따라서 helper status는 stopped여도 별도 직접실행 listener는 존재할 수 있음.
- 관측한 실제 HTTPS GET: `/vio_engine.js` SHA는 새 `38837d4…`, `/js/app.js?v=11` SHA는 최신 `c8454d99…`와 일치.
- 해당 프로세스의 health는 시작 시점 snapshot인 `140e29a…`, `unlabelled`, `external-paired`를 유지. 실제 GET bytes와 구분. 아래 수동 helper로 재시작하면 새 flag·metadata 적용.

```bash
# 직접 실행 중인 기존 서버는 해당 터미널에서 종료한 뒤 실행
bash scripts/ops/serve.sh start
bash scripts/ops/serve.sh status
```

- 주소: `https://dev.serdic.com:7002/`.
- TLS는 파일 읽기 기반 SAN/keymatch/chain/system trust 검증 통과. 기존 인증서·개인키·권한 변경 없음. leaf 유효기간 `2027-03-27 08:39:11 UTC`.

## 검증·보존

- [offline-preflight.json](../../build/server-ready-20261014/offline-preflight.json): 실제 request handler를 in-process 호출한7개검사. index/App/Renderer/loader byte 일치, health의 새hash/mode, private/legacy route403. 서버 실행 없이 수행.
- [preparation verification](../../build/server-ready-20261014/verification.json), [관측한 별도 프로세스 health](../../build/server-ready-20261014/observed-external-health.json).
- 이전 JS·manifest: `build/refactor-deploy/1791960718048999299/` 및 `build/server-ready-20261014/before/`.
- 이전 수동 시작flag: `build/server-ready-20261014/before/server-start-authorized.json`. 원본dataset·TLS·연구자료·미커밋변경 보존.
- commit/push/merge 없음. 신규서버 시작·실기기재시험·성능/정확도 개선 주장은 없음.
