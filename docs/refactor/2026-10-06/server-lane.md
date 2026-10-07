# HTTPS/Ops 구현 계획·진행

- 담당: 현재 native GPT-6.1 Sol / ultra executor, 재귀 위임 없음
- 소유: `web/server.js`, `scripts/ops/serve.sh`와 lifecycle helper, `tests/server/`, 서비스 runbook
- 보존: 기존 dirty source·다른 lane, NAS, TLS 내용/ownership, 기존 로그·타 서비스
- 실제 시작 조건: 부모가 후보 검증 후 `build/refactor-orchestration/server-start-authorized.json` 작성; 사용자 재확인 없음

## 수정 전 cleanup·회귀 계획

1. 기존 서버 source/log/TLS hash·metadata·7002 점유 snapshot. 기존 서버 코드는 임시 fixture VM에서 실행해 private path·POST·import side effect red 확보; 실제 로그/키에는 기존 코드를 실행하지 않음
2. startup side effect·자동 self-signed fallback·원격 무제한 로그 경로 삭제. `createServer`/handler export + strict leaf/intermediate/key validation, 기본7002
3. 공개 경계를 실제 realpath와 명시한 web/staged dataset route로 제한. encoded traversal/NUL/dot/private suffix/outside·hidden symlink/HEAD·method/MIME/COOP/COEP/cache 회귀
4. 수동 lifecycle: PID/starttime/repo/정확한 명령 확인, stale/collision fail closed, session 로그 보존·로컬 저장량 상한, 소유 process만 stop
5. 임시 localhost TLS에서 실제 leaf+ordered intermediate/SNI/system trust 확인; golden sentinel static 차단, 실제 key response 수집 없음
6. 후보 승인 후 실제 `0.0.0.0:7002` 시작, strict hostname TLS/HTTP/health/servedhash/PID·log 보존 검증; 요청 서비스 실행 유지

- runtime 의존성: Node/Python stdlib·기존 OpenSSL/curl만 사용
- 리스크·한계: off-LAN 관측점/실전화 부재는 별도 미검증; localhost SNI 성공으로 외부 internet·phone trust를 대체하지 않음
- 상태: 부모 fresh candidate 승격/시작 flag 후 실제7002 시작·검증 완료, 요청 서비스 실행 유지

## 실제 red/green·운영 회귀

| 검증 | 결과 | 근거 |
|---|---|---|
| 변경 전 임시 fixture | 6/6 RED | `build/refactor-orchestration/server-red.log`; 실제 키/기존 로그에서 legacy startup 미실행 |
| 정적 공개·method·헤더·TLS | 8/8 PASS | `server-green.log`; encoded traversal/NUL/dot/private suffix/hidden·outside·server symlink, GET/HEAD, room1/4, import no side effects, real leaf+2 intermediate/key/SAN/system trust/root 제외, embedded legacy file/alias 차단, optional favicon204 |
| 수동 lifecycle | 9/9 PASS | `server-lifecycle-tests.log`; trusted ephemeral HTTPS/curl, start/status/stop, stale/collision/foreign PID, paired/embedded hash gate, 1MiB/control strip, transient post-exec metadata 회귀 |
| detached 반복 startup/stop | 40/40 PASS | `server-postfix-start-stop.json`; localhost 임시 port, strict TLS health, 요청별 소유 종료 |
| syntax / diff | PASS | Node `--check`, Python compile, Bash `-n`, `git diff --check`; ESLint/TypeScript 검사와 구분 |
| TLS·기존 로그 보존 | PASS | before snapshot hash 일치; ownership 유지, key600/cert644/keys directory750 |

- 구현 단순화: automatic self-signed fallback·import startup·원격 무제한/truncate 로그 제거; Node stdlib streaming handler·명시 maps·factory만 유지
- 공개 web: index/test-tumvi/audit-mobile, active vio_engine loader, js/css/images/public 허용 파일; `public/*.json` camera profile 지원
- dataset maps: room1 repo fixture + room4 `build/refactor-data/tum/dataset-room4_512_16/mav0`; 실제 realpath가 checkout 밖이면 거부. archive·다른 room/NAS root 미공개
- `/log`: 모든 remote write405, body 수집/서버 로그 저장 없음
- health: `/__health__`, `/__audit-health__`; PID/instance/build label, startup artifact SHA256/bytes, server/source-manifest hash, TLS DER leaf fingerprint/intermediate count/validity. root path/key/credential 미공개
- lifecycle: `scripts/ops/serve.sh {start|status|stop}`; `build/server/service.json`, `/proc` start ticks/uid/exe/cwd/전체 argv 검증; 자동 재시작 없음
- 세션 로그: `logs/https/` 새 timestamp file, owner-only, 1MiB 상한 뒤 출력 drain/discard; 기존 `logs/test-tumvi.log` 보존
- 실제 시작 flag: `{authorized:true, js_sha256, wasm_sha256:null, single_file_embedded_wasm:true, deployment_id, source_manifest_sha256?}`; 현재 JS-embedded WASM candidate의 실제 deployed loader hash 일치 필요
- embedded mode: 외부 `.wasm`이 없어도 검증/시작 가능. 기존 legacy file은 보존하되 request/alias403, health active:false/available:false. future external paired mode에서는 두 파일 hash 모두 확인
- ephemeral 테스트: 별도 private state + localhost 임시 port; 정상 종료/정리 완료. actual7002는 아래 별도 실행 결과

## Startup 실패·수정 근거

- 관측: lifecycle 반복 중 1회 start exit1, 소유 Node의 listening 로그는 존재. 최초 assertion에서 stderr 사유 미보존; 실제 단일 원인은 확정하지 않음
- 보존: `server-lifecycle-failure-summary.json`; 당시 소유 PID1440898/port39919는 exact argv/cwd/uid + trusted health PID/instance 확인 후 해당 테스트 process만 종료
- 발견한 경계: startup의 `owned()` metadata 검사 예외가 readiness retry 바깥으로 빠지는 경로. 독립 transient fixture로 이전 배치를 RED 재현(`build/refactor-op-transition/red.log`)
- 수정: expected argv/exe/cwd/uid의 post-exec identity가 연속2회 안정될 때만 PID record 작성; readiness에서 일시적 identity/health 실패를 bounded retry. 종료 신호 전 실제 전체 identity 재검증 유지
- launcher: Conda Python에서 `posix_spawn(setsid=True)` 미지원 관측 → stdlib double-fork/setsid + intermediate reap 사용; 호출자/terminal 종료와 독립, 자동 restart 없음
- 검증: ResourceWarning을 error로 설정한 lifecycle9 PASS, 수정 후 실제 detached40회 startup/stop 모두 PASS; 한 번의 실패를 숨기거나 실제 원인을 단정하지 않음

## 초기 HTTPS 7002 실행 — superseded 후보 기록

- URL: `https://dev.serdic.com:7002/`; listener `0.0.0.0:7002`, Node PID `1520254`
- 시작: `2026-10-06T09:54:17.925Z` / `2026-10-06 18:54:17 KST`
- 로그: `logs/https/20261006T095417852240Z-7002-8d9cdb3139bd4405afffdc4832b029ab.log`
- 종료: `bash scripts/ops/serve.sh stop`; 현재 실행 유지, 자동 restart 없음
- SINGLE_FILE loader: `5,957,393 bytes`, SHA256 `b813c97cf636fad4eee68bd5e6240caae2b063b662544d294441f2800514e9fb`
- source manifest: `05cae05d058c1abdceb0f651597a9a999cc92f132b4c5c14f8ada20e07e4990f`
- 실제 TLS: hostname/SNI + system CA 검증, peer chain3(leaf+intermediate2), root 미전송; DER fingerprint만 기록, PEM/key 내용 출력 없음
- strict localhost HTTP12: health/index/replay/audit/room1·room4 CSV HEAD/loader hash/비공개 sentinel 경로/legacy WASM 차단 통과
- 같은 host의 LAN `192.168.0.35`: HTTP200, ssl_verify_result0, 동일PID/candidate hash
- 같은 host의 public DNS `183.98.179.103`: HTTP200, ssl_verify_result0, 동일PID/candidate hash; off-LAN client 증거와 구분
- 기존 TLS/로그 content hash·ownership 재확인 동일, private key600/cert644/keys directory750
- 실제 근거: `build/refactor-orchestration/server-{live-evidence,status}.json`, `server-{start,final-status,listener}.log`, `server-curl-{local,lan,public-dns}.*`, `server-result.md`
- off-LAN client·물리 phone/thermal/GT 미검증. 외부 web fetch 도구의 URL/ref 거부는 실제 네트워크 실패 판정으로 사용하지 않음

## 이전 parity HTTPS 7002 실행 — superseded 후보 기록

- URL: `https://dev.serdic.com:7002/`; `0.0.0.0:7002`, Node PID `1810169`
- 시작: `2026-10-06T11:08:45.026Z` / `2026-10-06 20:08:45 KST`
- 순서: old PID1520254/instance/전체 identity + trusted health 확인 → 소유서비스만 stop → 부모 승인 candidate atomic promotion/backup → new1810169 start; 혼합 health/loader로 운영하지 않음
- 배포 ID: `refactor-final-parity-20261006`
- SINGLE_FILE loader: `5,980,340 bytes`, SHA256 `882791a5f876cfda36d413058ea83725e6f73663e974e641ae85da8ad4f75d12`
- core manifest: `5624dc8c9ff4fa6a02b4abf77a8bf38467aee7bf4a89f059f516ddf2dd92110c`
- 서버 source: `ea2891b7f4e4a5ffad7aacc8abfa17c7724501023007d176e1a2c234a50ec27e`
- 로그: `logs/https/20261006T110844961886Z-7002-1c28d6180ded4d3587f4aaad834bca7a.log`
- 실제 strict hostname/SNI/systemCA + peerchain3(leaf+intermediate2), HTTP12/servedloader/sourcehash 통과; `-k`/ignoreHTTPS 없음
- actual favicon GET/HEAD204, body0/Content-Length 없음; 기존 private boundary 유지
- 같은 host LAN/publicDNS HTTP200/verify0, 동일 newPID/hash 확인; off-LAN client·실전화 검증과 구분
- SSL/legacy 로그 내용·ownership 재확인 보존; backups 및 old session 로그/기록 보존
- 서비스 실행 유지, 자동 restart 없음. 수동 상태/종료: `bash scripts/ops/serve.sh status`, `bash scripts/ops/serve.sh stop`
- 최종 근거: `build/refactor-orchestration/server-status.json`, `server-final-live-evidence.json`, `server-final-favicon.json`, `server-final-update-*.log`, `server-final-{lan,public-dns}*`, `server-final-status.log`, `server-listener.log`, `server-result.md`
- CPU/performance replay 없음; parent profiling freeze·actual browser/Worker/full dataset 결과는 해당 owner 근거와 통합

## 이전 portability HTTPS 7002 실행 — 재시작 전 기록

- URL: `https://dev.serdic.com:7002/`; `0.0.0.0:7002`, Node PID `1947692`
- 시작: `2026-10-06T11:52:37.218Z` / `2026-10-06 20:52:37 KST`
- 순서: old1810169/instance/전체 identity + trusted health 확인 → 해당 소유서비스만 stop → 부모 승인 atomic promotion/backup → new1947692 start
- deployment: `refactor-portable-prior-20261006`
- SINGLE_FILE loader: `5,981,589 bytes`, SHA256 `b75645443e52c8b51a0b78116ecad2f9cd02e4d0d94b7e64d6af2897f6008bff`
- common CPP source manifest: `f0f96be4215384ae4504e682765df05608ee529889e972c80e54e848dec24be9`
- 서버 source: `ea2891b7f4e4a5ffad7aacc8abfa17c7724501023007d176e1a2c234a50ec27e`; 제품 source·방화벽·router·caps·noise·manifold 변경 없음
- 로그: `logs/https/20261006T115237143005Z-7002-eb0273dd7e3d445bb406e0eeb8090cc1.log`
- actual strict TLS/SNI/systemCA/peerchain3, module/source/health hash, HTTP12, private GET+HEAD, favicon GET+HEAD204/body0, samehost LAN/publicDNS HTTP200 verify0 동일PID/hash 확인
- SSL·legacy log 내용/ownership 재확인 보존. 기존 후보 backup/session 로그 보존, 다른 서비스 신호 없음
- 현재 서비스 실행 유지; `bash scripts/ops/serve.sh status`, `bash scripts/ops/serve.sh stop`. 자동 restart 없음
- 근거: `build/refactor-orchestration/server-status.json`, `server-portable-live-evidence.json`, `server-portable-boundary.json`, `server-portable-update-*.log`, `server-portable-{lan,public-dns}*`, `server-final-status.log`, `server-listener.log`, `server-result.md`
- CPU build/replay 없음; full run4/전체 가상 검증은 부모 승인 freeze 이후 해당 owner 수행. off-LAN/실전화/thermal/fieldGT 미검증 유지

## 이전 동일 portability 후보 복구 — 신규 QR 승격 전 기록

- 관측: 기존 Node PID `1947692`·monitor PID `1947688` 부재, 7002 listener 부재. 중단 원인 미확정; 자동 재시작 없음
- 순서: helper `status` stale → `stop`으로 exact-owned metadata archive → 기존 승인·배포 SHA `b7564544…` 일치 확인 → `start` 새 세션. 부재 process/타서비스 신호 없음, 새 후보 승격 없음
- archive: `build/server/stopped-20261006T131743680634Z-1947692.json`; 이전 TLS·legacy/session 로그 내용·mode·uid/gid 그대로 보존
- URL: `https://dev.serdic.com:7002/`; listener `0.0.0.0:7002`, Node PID `2190754`, instance `1570f01b52fc4b64adb1ebcd7ec587bb`
- 시작: `2026-10-06T13:17:43.782Z` / `2026-10-06 22:17:43 KST`
- deployment `refactor-portable-prior-20261006`; 실제 GET module `5,981,589 bytes` / SHA256 `b75645443e52c8b51a0b78116ecad2f9cd02e4d0d94b7e64d6af2897f6008bff`
- startup health·web sidecar·start flag source SHA `f0f96be4215384ae4504e682765df05608ee529889e972c80e54e848dec24be9` 일치; server SHA `ea2891b7f4e4a5ffad7aacc8abfa17c7724501023007d176e1a2c234a50ec27e` 일치
- strict hostname/SNI/system CA + supplied leaf·ordered intermediate2 peer chain 일치, root 미전송. HTTP34: active GET hash·공개 HEAD·비공개 GET/HEAD403·favicon GET/HEAD204/body0·POST `/log`405 통과
- 같은 host LAN `192.168.0.35`·public DNS `183.98.179.103`: HTTP200/verify0·동일 PID/instance/loader 확인. off-LAN·실전화·thermal·field GT 미검증
- 로그: `logs/https/20261006T131743715711Z-7002-1570f01b52fc4b64adb1ebcd7ec587bb.log`; 실행 유지. CPU build/replay/regression 재실행 없음, `performance_claim:false`
- 근거: `build/refactor-orchestration/server-resume-{before,lifecycle,live-evidence}.json`, `server-resume-{lan,public-dns}*`, `server-resume-final-status.log`, `server-resume-listener.log`, 갱신된 `server-status.json`
- 다음: 부모의 정확한 신규 QR candidate SHA 승인 후에만 owned stop·atomic backup/promotion·start

## 이전 QR 후보 HTTPS 7002 실행 — 최종 후보 승격 전 기록

- 부모 exact SHA 승인: loader `fce25012d61a0d5965e4d5afadda25aecf96c219b30451e0394575c3caa10da1`, `6,007,610 bytes`; source manifest `11ca788ff68b39653984b2d384c672820fcf698cfc0a621d273a49311ae9075c`
- 순서: PID2190754/instance1570… 전체 `/proc` identity·trusted health 확인 → helper owned stop·빈7002 확인 → 기존 `deploy-candidate.py` atomic backup/promotion → 새 start1회. foreign process·방화벽·router·TLS 내용 변경 없음
- backup: `build/refactor-deploy/1791296770309121296`; 이전 b756 loader+sidecar SHA 검증, 기존 후보·실패 근거·metadata/session 로그 보존
- URL: `https://dev.serdic.com:7002/`; `0.0.0.0:7002`, Node PID `2449945`, instance `41fe340bca3a44ad8405987db68b53a3`
- 시작: `2026-10-06T14:26:10.461Z` / `2026-10-06 23:26:10 KST`; deployment `refactor-qr-benchmark-20261006`
- actual GET loader SHA·bytes = startup health = deployed sidecar = parent flag; source manifest `11ca788f…` 일치, server SHA `ea2891b7f4e4a5ffad7aacc8abfa17c7724501023007d176e1a2c234a50ec27e` 유지
- strict hostname/SNI/system CA + supplied leaf·ordered intermediate2 peer chain3 일치·root 미전송. HTTP34: 공개 HEAD·비공개 GET/HEAD403·favicon GET/HEAD204/body0·POST `/log`405 통과; `-k` 없음
- 같은 host LAN `192.168.0.35`·public DNS `183.98.179.103`: HTTP200/verify0·동일 PID/instance/loader/source 확인. off-LAN·실전화·thermal·field GT 미검증
- TLS·legacy log 내용/mode/uid/gid 동일. 종료된 직전 session log 원문 prefix/hash 유지, 서버 graceful shutdown의 `stopped` event20B 추가 확인; truncate/overwrite 없음
- 새 로그: `logs/https/20261006T142610395901Z-7002-41fe340bca3a44ad8405987db68b53a3.log`; 서비스 실행 유지, 자동 restart 없음. CPU build/replay/regression 재실행 없음, `performance_claim:false`
- 근거: `build/refactor-orchestration/server-qr-{before,lifecycle,live-evidence}.json`, `server-qr-update-*.log`, `server-qr-{lan,public-dns}*`, `server-qr-final-status.log`, `server-qr-listener.log`, `server-status.json`
- 다음: 부모 actual7002 fullrun5·strict browser/Worker·전체 dataset 검증과 통합

## 현재 최종 9973 후보 HTTPS 7002 실행

- 부모 exact SHA release 후 실행: loader `9973d58eca6f95fcf5fcaf2d553d02bc2215f419e4ba1fc1cc94ae417eb3014d`, `6,009,632 bytes`; common source manifest `b936982e5fea0c8ad0fb77e3e8b311770998763ff77ef2ef4bb082945866674b`
- 순서: old PID2449945/instance41fe… 전체 `/proc` identity·trusted health 확인 → helper exact-owned stop·빈7002 확인 → 기존 `deploy-candidate.py` atomic backup/promotion → start1회. 타process·방화벽·router·DNS/hosts·TLS 내용 변경 없음
- backup: `build/refactor-deploy/1791305306687347354`; 이전 fce250 loader+sidecar SHA 검증. 이전 후보·실패 근거·metadata/session 로그 보존
- URL: `https://dev.serdic.com:7002/`; listener `0.0.0.0:7002`, Node PID `3002812`, instance `c98f10df76144a3c80311b09d1e0d5cf`
- 시작: `2026-10-06T16:48:26.826Z` / `2026-10-07 01:48:26 KST`; deployment `refactor-final-20261007`
- actual GET loader SHA·bytes = startup health = deployed sidecar = parent start flag; source manifest `b936982e…` 일치, server SHA `ea2891b7f4e4a5ffad7aacc8abfa17c7724501023007d176e1a2c234a50ec27e` 유지
- strict hostname/SNI/system CA + supplied leaf·ordered intermediate2 peer chain3 일치·root 미전송. HTTP34: 공개 HEAD·비공개 GET/HEAD403·favicon GET/HEAD204/body0·POST `/log`405 통과; `-k` 없음
- same-host LAN `192.168.0.35`: HTTP200/verify0·동일 PID/instance/loader/source. public DNS `dig @1.1.1.1` A `183.98.179.103` 조회 후 해당 IPv4로 forced TCP `--resolve` strict SNI 연결: HTTP200/verify0·동일 identity/hash
- 기본 hostname 요청은 기존 `/etc/hosts` development override 때문에 `127.0.0.1`로 연결; 이 결과는 localhost 경로로 별도 보존. ops DNS/hosts 변경 없음, off-LAN·실전화·thermal·field GT 미검증
- TLS·legacy log 내용/mode/uid/gid 동일. 종료된 직전 session log 원문 prefix/hash 유지 + 서버 graceful shutdown `stopped` event20B append; truncate/overwrite 없음
- 로그: `logs/https/20261006T164826765565Z-7002-c98f10df76144a3c80311b09d1e0d5cf.log`; 새서비스 실행 유지, 자동 restart 없음. helper/server code 변경·CPU build/replay/추가 lifecycle 회귀·room2 추출 없음
- 실제 ops 근거: `build/refactor-orchestration/server-final-9973-{before,lifecycle,live-evidence}.json`, `server-final-9973-update-*.log`, `server-final-9973-{lan,public-dns,system-hostname-local}*`, `server-final-9973-final-status.log`, `server-final-9973-listener.log`, 현재 `server-status.json`
- 부모 독립 actual-service 검증·fullrun6 전 `performance_claim:false`; 알고리즘/모바일 production 성능 통과로 확대 없음

## 부모 통합 gate

- `-k`/browser ignoreHTTPS 없이 actual7002 strict browser/Worker/mock/room1·room4 검증은 부모/transport/mobile lane 결과와 통합
- served active loader hash·health startup snapshot·PID/명령·TLS chain/SNI·공개/차단/HEAD는 위 실제 검증 참조
- 부모/transport/mobile lane에 실제PID/readiness 전달 완료
- 완료 시 실제 요청 서비스 유지; off-LAN reachability·physical phone/thermal/GT는 해당 관측 없으면 미검증
