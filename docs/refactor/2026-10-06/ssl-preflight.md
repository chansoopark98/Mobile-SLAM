# HTTPS 7002 사전 점검

- 점검: `2026-10-06T08:30:16Z`, revision `b2826c037a50036d286329b1a56daa1a75ba57a3`
- 외부 hostname 확정 후 추가 점검: `2026-10-06T08:37:02Z`, 사용자 지정 `dev.serdic.com:7002`
- 범위: 인증서·개인키·DNS·리스너 읽기; 서버 시작, 인증서/키 수정, 방화벽/DNS/router 변경 없음
- 근거: [JSON](ssl-preflight.json), [명령 결과](ssl-preflight.log)
- 도구: OpenSSL `3.0.16`, Node.js `22.21.1`; shell/OpenSSL CLI 결과와 실제 browser trust 구분

## 인증서·체인

| 항목 | 관측 | 판정 |
|---|---|---|
| leaf CN / SAN | `*.serdic.com`; SAN `*.serdic.com`, `serdic.com` | hostname 용도 적합 |
| 유효기간 UTC | `2026-09-09 08:39:11` → `2027-03-27 08:39:11` | 점검 시각 기준 유효 |
| leaf issuer | `GlobalSign GCC R46 DV TLS CA 2025` | chain 첫 인증서 subject와 일치 |
| chain 첫 장 | `GlobalSign GCC R46 DV TLS CA 2025` → `GlobalSign Root R46` | issuing intermediate |
| chain 둘째 장 | `GlobalSign Root R46` → `GlobalSign Root CA - R3` | cross-signed CA, self-signed root와 구분 |
| root 파일 | `GlobalSign Root CA - R3`, self-signed | 서버 전송 체인에서 제외 권고 |
| private key / leaf public key | SPKI SHA-256 모두 `dff6302f81895217b4f1d87670c1d26f7a0957f0a67278ddf09483291dd628d2` | 일치, 개인키 원문 출력 없음 |
| system trust + chain | 실제 지정 `dev.serdic.com`, 초기 후보 `serdic.com`, `www.serdic.com`, `slam.serdic.com` 모두 OpenSSL verify exit `0` | offline 인증 경로 검증 완료 |
| leaf 단독 | error `20`, `unable to get local issuer certificate` | 중간 인증서 전송 필요 |
| `localhost` / `192.168.0.35` | hostname error `62` / IP error `64` | URL hostname 불일치 |

- 기존 `web/server.js`: `cert`에 `serdic_com_cert.crt` 한 장만 로드; 아직 HTTPS listener handshake 검증 없음
- 권고: Node TLS `cert`에 `leaf PEM + chain PEM` 순서로 연결, supplied root PEM 제외; `ca`는 peer trust 설정이므로 서버 체인 제공 목적에 대신 사용하지 않기. [Node.js 22.21.1 TLS 계약](https://nodejs.org/download/release/v22.21.1/docs/api/tls.html#tlscreatesecurecontextoptions)
- 대안: Python explicit `--cert` 사용 시 별도 `build/`에 위 순서의 fullchain 생성; 원본 `assets/keys/` 보존
- 별도 한계: revocation/OCSP 검증, 물리 전화 trust store, 실제 SNI handshake 미검증

## 파일 metadata

| 파일 | 크기 bytes | owner:group | mode | mtime UTC |
|---|---:|---|---|---|
| `serdic_com.key` | 1674 | `park-ubuntu:park-ubuntu` | `0666` | `2026-09-22T02:12:42Z` |
| `serdic_com_cert.crt` | 2190 | `park-ubuntu:park-ubuntu` | `0666` | `2026-09-22T02:12:34Z` |
| `serdic_com_chain_cert.crt` | 3914 | `park-ubuntu:park-ubuntu` | `0666` | `2026-09-22T02:14:02Z` |
| `serdic_com_root_cert.crt` | 1248 | `park-ubuntu:park-ubuntu` | `0666` | `2026-09-22T02:14:08Z` |

- 개인키의 other read/write 권한 관측; 실행 전 owner-only `0600` 권고. 사전 점검에서는 mode 변경 없음
- 인증서 SHA-256, 인증서별 fingerprint, 공개키 fingerprint: JSON 보존
- `.gitignore`의 `assets/keys/` 제외 규칙 실제 확인

## 리스너·DNS·접속

| 검증 층 | 관측 | 범위/판정 |
|---|---|---|
| TCP bind `7002` | `ss -ltnp 'sport = :7002'`: 리스너 없음 | 아직 서버 시작 전 |
| LAN 주소 | `eno2=192.168.0.35/24`, `wlp0s20f3=192.168.0.170/24` | default route `eno2`, gateway `192.168.0.1` |
| 지정 DNS `dev.serdic.com` | A `183.98.179.103`; AAAA는 `NOERROR`, answer 없음 | local resolver + authoritative A 일치 |
| authoritative A | `ns.gabia.co.kr`, `ns1.gabia.co.kr`, `ns.gabia.net` 직접 `+norecurse` 모두 `qr aa`, TTL `600`, A `183.98.179.103` | 세 authoritative 서버 응답 확인 |
| DNS `serdic.com` | `198.49.23.144`, `.145`, `198.185.159.144`, `.145` | LAN 서버 주소와 다름; WAN/router 소유 주소 미검증 |
| DNS `www.serdic.com` | `ext-cust.squarespace.com` + 위 네 주소 | 현재 웹사이트 DNS 경로 관측 |
| DNS `slam.serdic.com` | `getent ahosts` exit `2`, curl exit `6` | local resolver 결과 없음; authoritative DNS 변경 없음 |
| local/LAN curl | `127.0.0.1`, `192.168.0.35`, `.170` 모두 exit `7`, connection refused | TLS 시작 전 TCP 실패; 같은 host에서 요청 |
| DNS 그대로 curl | apex/www `7002` exit `28`, TCP timeout | 실제 LAN 서버 대상인지 확인되지 않은 주소 |
| 지정 hostname curl | `https://dev.serdic.com:7002/` exit `7`, `Connection refused`, HTTP `000` | 지정 A를 향한 TCP 단계 실패; 서버 시작 전, TLS 미진입 |
| UFW read-only | `sudo -n ufw status verbose` exit `1`, `a password is required` | 사용자 allow7002 진술 기록; live UFW 상태 미검증, prompt 없음 |
| router forwarding | 사용자 설정 완료 진술 | router read/외부 client 미검증 |
| browser / 물리 전화 | 사전 점검에서 실행 없음 | 서버 시작 후 별도 단계 |
| 외부 인터넷 inbound | off-LAN client 없음 | 미검증; 서버 host 요청/NAT hairpin 결과로 대체 불가 |

- curl에서 `ssl_verify_result=0` 출력만으로 TLS 통과 판단 금지: 위 요청은 HTTP `000`, TCP/DNS 단계에서 실패
- `183.98.179.103` route: `via 192.168.0.1 dev eno2 src 192.168.0.35`; public A와 router WAN 주소 일치 여부·hairpin 동작·off-LAN 도달성 별도 확인 필요
- 기존 Node 서버 시작은 `logs/test-tumvi.log`를 초기화하는 side effect 포함; startup 개선 시 기존 로그 보존 필요

## 서버 시작 후 strict 검증 권고

- 아래 명령은 **사전 점검에서 미실행**; leaf + chain 로딩 및 7002 bind 이후 실행
- URL의 인증서 hostname 유지, DNS override로 연결 대상만 변경; curl `-k`, browser `ignoreHTTPSErrors`, 인증서 경고 우회 제외

```bash
# localhost TCP + 실제 cert hostname/SNI + system trust + HTTP
curl --noproxy '*' --cacert /etc/ssl/certs/ca-certificates.crt \
  --resolve dev.serdic.com:7002:127.0.0.1 \
  --fail --head https://dev.serdic.com:7002/

# 같은 host의 LAN address 바인딩 확인; 다른 LAN client 도달성과 구분
curl --noproxy '*' --cacert /etc/ssl/certs/ca-certificates.crt \
  --resolve dev.serdic.com:7002:192.168.0.35 \
  --fail --head https://dev.serdic.com:7002/audit-mobile.html

# SNI / chain / hostname 검증; verification error 발생 시 즉시 실패
openssl s_client -connect 127.0.0.1:7002 -servername dev.serdic.com \
  -verify_hostname dev.serdic.com -verify_return_error \
  -CAfile /etc/ssl/certs/ca-certificates.crt -showcerts </dev/null
```

- isolated Chromium launch: `--host-resolver-rules=MAP dev.serdic.com 127.0.0.1`, `https://dev.serdic.com:7002/`, context `ignoreHTTPSErrors=false`; 개인 browser profile/global hosts 변경 없음
- browser 근거: document HTTP200, `isSecureContext=true`, JS/WASM 실제 응답/hash, console/network 오류, virtual camera/IMU fixture 결과; physical phone 증거와 구분
- 물리 전화 LAN 접속: 동일 hostname을 해당 LAN IP로 해석 가능한 local DNS 필요; IP URL은 supplied cert SAN과 불일치
- 외부 접속: 실제 forwarded WAN 주소·해당 hostname 경로 + off-LAN client 요청 필요; router/UFW 상태 변경은 사전 점검 범위 밖
