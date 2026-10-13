# Capture Worker 독립 코드·테스트 검토

- 날짜: 2026-10-12, Asia/Seoul
- 담당: GPT-6.1 Sol Ultra, 구현 lane와 독립 검토. 제품 write 없음
- 범위: `web/js/capture-worker.js`, `web/js/app.js`, `web/js/camera.js`, `tests/browser/capture-worker.test.mjs`, `scripts/dev/capture-worker-browser.mjs`
- 기준: 승인된 [개발 계획](2026-10-12-recorded-recovery-plan.md)의 C1, [테스트 명세](2026-10-12-recorded-recovery-test-spec.md)의 T6, [Web engine 연구](../research/2026-10-12-web-engines.md)
- 코드 판정: **RESOLVED — 독립 재현한 P2 2개 수정 확인, 현재 미해결 코드 결함0**
- 제품 선택: synchronous 기본 유지, `?capture=worker` 실험만 허용. 물리 Flip6 지원·성능·정확도 및 C1 기본 승격은 미검증

## 발견·수정 검증

| 항목 | 수정 전 재현 | 소유자의 수정 | 독립 재검증 |
|---|---|---|---|
| P2 자원 상한 | inFlight1+pending1 상태에서 새 VideoFrame을 먼저 생성. pending 해제 전 live peak3; 기존 테스트는 capture 반환 뒤만 표본화 | `capture-worker.js:41`에서 기존 pending을 새 constructor보다 먼저 close/coalesced 처리 | constructor 내부까지 표본화한 `peakLive` 3→2, remote peak1, 종료 후0 |
| P2 completion callback 재진입 | seq1 완료 callback이 seq3을 capture하는 동안 기존 seq2 pending. seq3 dispatch 뒤 seq2가 inFlight를 덮어씀. posted `[1,3,2]`, remote peak2, seq3 응답 stale, accepted `[1,2]` | 교체 pending을 constructor 전에 close; `capture-worker.js:126`에서 `!inFlight`일 때만 pending dispatch | posted `[1,3]`, remote peak1, accepted `[1,3]`, staleReplies0 |

- 수정 전 source SHA256: `923f3fc57b7a6970a3dbf93a10854a3986cf98db3e0acc81aad85dfa026698c3`
- 수정 후 source SHA256: `6447aff5c06b4f189d1e4dc2d057fab617e1b2ee18278e11f8a048519cddf5ad`
- 독립 재현: `build/recovery-20261012/capture-review/edge-probes.mjs`, `edge-probes-before-v2.json`, `edge-probes-after-v2.json`. 같은 harness를 보존된 수정 전 source와 현재 source 양쪽에 실행; 처음 before/after 결과도 보존. 가벼운 소유권 모델과 실제 JS 호출 순서 검토를 결합; browser/GC/GPU 물리 자원 계측으로 확대하지 않음
- 소유자 regression: `build/recovery-20261012/capture-review-regression-red.log`에 수정 전 2개 red 보존
- 검토자가 현재 수정본에서 실행한 `node --test tests/browser/capture-worker.test.mjs`: 11pass/0fail/0skip/0TODO. `build/recovery-20261012/capture-review/current-contracts.log`
- 이후 stop/start·old realm event 회귀를 추가한 owner 결과: `build/recovery-20261012/capture-worker-contracts-final.log`, 12pass/0fail/0skip/0TODO. 새 worker는 old frame drain 뒤 생성되고 종료된 worker의 ready/error가 새 상태를 바꾸지 않는 테스트를 직접 검토

## 계약별 검토

| 계약 | 확인·근거 |
|---|---|
| 픽셀 확보·시각 | App rVFC callback의 await 없는 구간에서 VideoFrame 생성. main proxy timestamp의 microsecond 값만 frame metadata에 부여; worker가 video를 다시 읽는 경로 없음. `physicalExposureVerified=false` 유지 |
| 자원 소유 | 성공 transfer 뒤 local 참조 null, worker handler finally의 close 후 ACK. pending 교체·cancel은 local close. crash/hang은 `releasedByTermination`으로 별도 기록; 이를 explicit close ACK로 합산하지 않음 |
| 상한·최신성 | 수정 후 converting≤1+pending≤1, overwrite 시 이전 pending close. seq 단조 증가·중복 응답은 현 inFlight를 종료하지 않음. 재진입에서도 이전 pending을 부활시키지 않음 |
| 수락 fence | worker identity에 seq/generation/configurationId/clientEpoch/timestampUs/presentedFrames/전체 geometry. App가 current video·generation·configuration 참조·current epoch/geometry·presentedFrames 단조성·running/hidden/blur/reconfigure를 다시 검사 |
| 실패 fallback | constructor 이전 startup 미준비 및 constructor 실패는 같은 callback의 기존 sync 경로 가능. transfer 실패는 요청을 소비·drop; 전송 후 draw/crash/hang은 해당 요청 폐기. 다음 신규 rVFC만 sync fallback하며 이전 timestamp로 나중 영상 재캡처 없음 |
| stop/restart | cancel은 pending close·inFlight cancelled 표시. stop은 기존 frame deadline 아래 drain 또는 realm terminate. old worker event는 worker 객체 동일성 fence로 새 session에 전달되지 않음 |
| clock | main snapshot/receive/submit는 main performance clock, worker는 draw/read·gray duration만 반환. worker timeOrigin을 main과 같다고 가정하는 뺄셈 없음 |
| 기존 App 보존 | frozen dirty A0 tar와 비교. root의 rVFC snapshot·event submit·owned gray transfer·rotation 설정을 보존. `camera.js`는 A0와 byte 동일 |

- A0 비교: `build/recovery-20261012/capture-review/A0-C1-app.diff`; A0 app SHA256 `6311a5a2e990ccd962d1f348bbf8e3c479bf1173f8a11e6a31b85b9efe0682c1`
- current App SHA256 `173d987f4d8537a30c2adeb2950b49a673de8cd10fb84da82e966c2a6f70f595`; camera SHA256 `fa31559030d598bd0fb14d152d8f9a9b5c4a383096bdc7741f955d12530b01cd`
- 검토한 최종 source 보존: `build/recovery-20261012/capture-review/final-source/`, `final-source-manifest.json`. `scripts/dev/synthetic-browser.mjs`의 후속 C1 선택 adapter는 root 통합 검토 범위이며 본 제품 검토의 변경 파일에 포함하지 않음
- 별도 feature 저장소·새 dependency·새 grayscale 식 없음. 기존 `(77R+150G+29B)>>8`, rotation/crop 경로 유지

## 실제 브라우저 증거·한계

- 기존 actual desktop report: `build/recovery-20261012/capture-worker-1791810175743/report.json`, SHA256 `96a35ac63ca44daf223eb2fed92b11a3b9ec0a285d321b48146781521b336816`, Chrome `144.0.7559.96`
- 위 report의 제품 source hash와 당시 작업 트리 일치 확인. 16개 portrait/landscape×rotation none/cw/ccw/half×full/noncenter crop의 전체 byte를 기존 Camera CPU oracle와 독립 좌표·색상 oracle 양쪽에 비교
- 실제 HTMLVideoElement→VideoFrame→transfer→OffscreenCanvas2D draw/read 사용. worker를160ms 지연시키고 source video를 A→B로 진행시킨 뒤에도 retained A 픽셀 유지 확인
- draw 오류는 explicit remote close1, fatal crash/hang은 remote close0·realm release1. App busy coalescing/configuration/generation/stop fence 확인
- 위 16case report는 **수정 전 capture-worker SHA**의 보존 증거. 새 SHA의 통과 근거는 아래 재실행 report로 구분
- 수정 후 최종 actual desktop report: `build/recovery-20261012/capture-worker-1791810816204/report.json`, SHA256 `ed1b7c26098f547f8200945f9a34994ffc86a50bcfefd5693674083dbbf9bb6c`, Chrome `144.0.7559.96`. capture-worker/App/camera source SHA 모두 현 파일과 일치, 16pixel+3fault+4fallback+실제 stop/drain/restart PASS 확인
- 수정 후 report의 실제 App fallback4: draw·native invalid transfer list는 이전 callback의 sync read0·submit0, 다음 callback의 새 timestamp/픽셀로만 submit1. constructor·init 실패는 같은 callback의 sync read1·submit1. capture-worker 최종 scheduled7=local close2+remote close5, inFlight0/pending0/staleReplies0; sync callback 분모와 별도
- 최종 report의 실제 inFlight stop→start:100ms 지연 real Worker 처리1+pending1에서 stop, pending local close1 즉시 확인·새 worker 생성 대기, old remote close ACK1 뒤 새 worker ready, old publish0·generation2의 frame3만 publish. 최종 scheduled3/posted2/completed2/accepted1/drop2, local close1/remote close2/termination0/inFlight0/pending0
- 수정 후 초기 report `capture-worker-1791810657945/report.json`에는 actual inFlight drain이 없음을 찾아 위 마지막 fixture를 추가 요청. 해당 원본도 보존하며 최종 lifecycle 주장에는 마지막 report만 사용
- synchronous 기본의 실제 browser snapshot report `build/recovery-20261012/capture-sync-default-browser/report.json`: App/camera 현 SHA와 일치, Chrome144 PASS. root의 callback 보존·rAF에서 재read 없음·즉시 제출 확인
- owner 전체 최종 regression `build/recovery-20261012/capture-browser-contracts-final.log`:262 Node pass/0fail/0skip/0TODO + analytic exporter3pass. 수정 전259개 결과는 `capture-browser-contracts.log`에 별도 보존; 새 test 개수를 이전 SHA의 결과로 복사하지 않음
- root ADB 조사와 승인 review 기록상 target device/mDNS 없음·연결 timeout. 실제 target Chrome construct/transfer/draw/read·직렬 ABBA·동일 accepted-input GT gate는 미검증. desktop 지원·pixel 통과를 전화 지연·추적 정확도 개선으로 확대하지 않음
- 검토 중 heavy browser performance·추정기 replay·추가 build는 실행하지 않음. 제품·dataset·서비스·다른 lane 변경 보존
