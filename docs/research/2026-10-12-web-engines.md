# 모바일 웹 엔진·영상 수집 조사

- 조회일: 2026-10-12, Asia/Seoul; 연구 단계, 제품 코드·의존성 변경 없음
- 결론: **우선 현 프레임의 성숙 track 소실 경로 확인. 영상 수집 Worker는 별도 지연 실험 후보. 8th Wall 공개 framework를 SLAM 공개 구현으로 분류하지 않음**
- 기준: [10월 9일 비교](2026-10-09/reference-comparison.md), [최신 실기기 기록](../recordings/2026-10-12-zflip6-after-optimization.md); 과거 공개 여부·wrapper 설명을 최신 사실로 전용하지 않고 공식 소스 재조회
- 보관: [원문·버전·해시](../../build/research-20261012/web-engine/); `source-manifest.json`, `extra-source-manifest.json`, `metadata-manifest.json`, `official-docs-manifest.json`, `local-source-fingerprint.json`

## 현장 관측과 조사 우선순위

- 최신 30초 기록: 처리율20.21fps, Lost33개 중 `currentVisualFactors=0` 29개/`=1` 4개, 해당 행 Ceres·marginalization 미실행
- Lost 때 tracker feature 중앙값53, eligible bank track80, immature track146: 전체 feature 수와 현 프레임의 유효 성숙 landmark 연결 수 구분
- IMU interval/bracket Lost0/0, Worker queue p95 0.863ms: 대기열 증가를 현재 Lost의 주원인으로 단정할 근거 없음
- main pixel-read p50/p95 16.93/23.50ms, source→submit proxy31.98/44.48ms: main 점유를 줄이는 별도 실험의 근거. API 이동만으로 총 지연·정확도 개선 확정 불가
- 기존 구현: rVFC 안에서 영상 gray snapshot 확보, 최신 pending slot, 소유한 full-view gray ArrayBuffer 전송, 전송 추가 복사0. rVFC·소유 이전·단일 pending slot 재도입 제안 제외
- 출처: 위 실기기 보고서; 숫자 기록만 존재하며 동일 영상 재생·독립 GT·실측 K/extrinsic 부재

## 8th Wall의 현재 배포와 분석 범위

| 대상 | 2026-10-12 확인 | 분석·사용 판정 |
|---|---|---|
| [8thwall/engine](https://github.com/8thwall/engine/tree/e1421ae742a9c5d654871367c64c1de3ad044caf) | commit `e1421ae742a9c5d654871367c64c1de3ad044caf`, npm metadata `@8thwall/engine-binary` 1.0.0, `data-preload-chunks="slam"` 공개 | SLAM 포함 binary 배포 존재. 다운로드 불가능한 비공개 SDK라는 과거 전제 사용 금지 |
| [binary LICENSE](https://github.com/8thwall/engine/blob/e1421ae742a9c5d654871367c64c1de3ad044caf/LICENSE) | §1.2(i) 역공학·디컴파일·디스어셈블 제한, §1.2(iv) 유사·경쟁 제품 생성/개선 목적 사용 제한 명시 | 권리 예외·별도 허가 확인 없음. 이번 조사는 license·README·배포 포인터까지만 확인, binary 내용 추출·디스어셈블 제외 |
| [8thwall/8thwall](https://github.com/8thwall/8thwall/tree/eaee2b9a3677a83da27733af23ec7bcc112b4c34) | commit `eaee2b9a3677a83da27733af23ec7bcc112b4c34`, MIT framework, GPU feature/image tracking 소스 공개 | 아래 구조 분석 대상. 별도 SLAM optimizer·relocalization·metric accuracy 확인 근거로 확대 금지 |

- 공식 구분 재확인: [Engine Distribution](https://8thwall.org/docs/open-source)에서 SLAM은 binary에 포함, MIT framework의 공개 기능과 분리
- `engine/xr-standalone.zip`의 Git LFS 포인터: 선언 크기13,998,848bytes, 선언 SHA-256 `a2fc73ba450c1a96ee2a20db442f7fb328a3c8b76565e7e9d15b68cd75c86a93`
- 위 SHA는 **원격 LFS 객체 식별자**, 로컬 ZIP 다운로드·내용 검증 SHA가 아님. 로컬 보존 파일은133bytes LFS 포인터
- binary imports/exports·symbol·memory·pthread·SIMD: **미검사**. MIT 소스의 빌드 옵션으로 binary 내부를 추정하지 않음
- `engine-binary`라는 저장소 URL은404; 실제 공식 저장소는 `8thwall/engine`. 오류·수정 경로 보존

## MIT framework에서 실제 확인한 구조

| 관측 소스 | 확인 내용 | 현재 프로젝트 적용 판단 |
|---|---|---|
| [camera-loop.ts](https://github.com/8thwall/8thwall/blob/eaee2b9a3677a83da27733af23ec7bcc112b4c34/reality/app/xr/js/src/camera-loop.ts#L296) | camera texture를 compute/draw context로 연결, GPU→CPU→update pipeline; `frameTime=performance.now()/1000` | texture 중심 처리 참고. 해당 frameTime을 실제 exposure로 간주 금지 |
| [tracking-controller.ts](https://github.com/8thwall/8thwall/blob/eaee2b9a3677a83da27733af23ec7bcc112b4c34/reality/app/xr/js/src/tracking-controller.ts#L439) | texture·orientation·videoTime·frameTime·별도 callback 시각 stage, CPU에서 staged frame 처리 | 공개 호출 계약 확인. stage 함수 이름만으로 비공개 SLAM 상태·factor·solver 확인 불가 |
| [sensors.ts](https://github.com/8thwall/8thwall/blob/eaee2b9a3677a83da27733af23ec7bcc112b4c34/reality/app/xr/js/src/sensors.ts#L49) | DeviceMotion callback 시각과 event timestamp, interval을 각각 보존·일괄 전송 | 이미 프로젝트의 source/arrival 계약과 방향 일치. callback 시각으로 hardware sampling 동기 주장 금지 |
| [offscreen-canvas.ts](https://github.com/8thwall/8thwall/blob/eaee2b9a3677a83da27733af23ec7bcc112b4c34/reality/app/xr/js/src/offscreen-canvas.ts#L22) | OffscreenCanvas에서 실제 WebGL1/2 context 생성 성공 확인 | class 존재보다 실제 context probe 필요. 이 파일은 Worker 처리의 증거가 아님 |
| [gl-reality-frame.cc](https://github.com/8thwall/8thwall/blob/eaee2b9a3677a83da27733af23ec7bcc112b4c34/reality/engine/features/gl-reality-frame.cc#L484) | GPU pyramid·score shader 단계, WebGL pixel-pack-buffer/getBufferSubData 경로, 선택적 deferred read-before-draw | GPU 사용 자체가 readback stall 제거 보증 아님. deferred 모드는 이전 영상 데이터에 대응하는 시각을 함께 유지해야 함 |
| [camera-providers.ts](https://github.com/8thwall/8thwall/blob/eaee2b9a3677a83da27733af23ec7bcc112b4c34/reality/app/xr/js/src/camera-providers.ts#L160), [yuv-pixels-array.ts](https://github.com/8thwall/8thwall/blob/eaee2b9a3677a83da27733af23ec7bcc112b4c34/reality/app/xr/js/src/yuv-pixels-array.ts#L158) | 이전 renderTex readPixels 후 현재 texture 렌더. YUV provider는 packed YUVA4bytes/pixel | 영상 1개 지연을 도입하는 순서 그대로 이식 금지. 현재 프레임의 rVFC 시각으로 이전 픽셀 라벨링 금지 |
| [image-tracking-state.cc](https://github.com/8thwall/8thwall/blob/eaee2b9a3677a83da27733af23ec7bcc112b4c34/reality/engine/imagedetection/image-tracking-state.cc#L60) | 알려진 image target의 투영 local descriptor 검색→기하 검증→실패 시 global 검색 | 단절된 track 회복의 설계 단서. planar/curved 참조 이미지용이며 임의 3D map SLAM 회복 구현으로 간주 불가 |
| [.bazelrc](https://github.com/8thwall/8thwall/blob/eaee2b9a3677a83da27733af23ec7bcc112b4c34/.bazelrc#L113) | `wasmreleasesimd`는 `wasmrelease`+`--features=simd` | 소스 설정 확인만 완료. 실제 배포 binary SIMD·thread 수는 미확인 |

## 다른 공개 웹 추적 구현

| 대상·고정 revision | 소스에서 확인한 구조 | 판정 |
|---|---|---|
| [AlvaAR](https://github.com/alanross/AlvaAR/tree/7796af500ee92001ac2a9888363ff64d7a3bee75), GPLv3 | [visual_frontend.cpp](https://github.com/alanross/AlvaAR/blob/7796af500ee92001ac2a9888363ff64d7a3bee75/src/slam/src/visual_frontend.cpp#L104): 3D map point 투영 prior→낮은 pyramid LK→실패 track만 full pyramid 재시도, [feature_tracker.cpp](https://github.com/alanross/AlvaAR/blob/7796af500ee92001ac2a9888363ff64d7a3bee75/src/slam/src/feature_tracker.cpp#L34): forward/backward 검사 | 성숙 track의 지원을 유지하는 bounded 후보. 현재 cached pyramid·rotation rescue 중복 구현 제외; 동일 영상·잘못된 prior control에서 평가 필요 |
| [AlvaAR IMU 함수](https://github.com/alanross/AlvaAR/blob/7796af500ee92001ac2a9888363ff64d7a3bee75/src/slam/src/system.cpp#L57) | `findCameraPoseWithIMU()`의 gyro/acc 샘플은 지역 변수로 읽고 estimator에 전달하지 않음; 외부 orientation과 visual translation 조합 | 함수명·IMU 샘플 수용만으로 tightly coupled VIO·metric scale 검증 주장 불가 |
| [MindAR](https://github.com/hiukim/mind-ar-js/tree/fb5db87b1e5076ba86fde82101955da70b629a1c), MIT | [controller.js](https://github.com/hiukim/mind-ar-js/blob/fb5db87b1e5076ba86fde82101955da70b629a1c/src/image-target/controller.js#L152)의 GPU tracker/detector와 [controller.worker.js](https://github.com/hiukim/mind-ar-js/blob/fb5db87b1e5076ba86fde82101955da70b629a1c/src/image-target/controller.worker.js)의 match/pose refinement 분리 | 알려진 image target 추적 사례. arbitrary metric VIO의 정확도·실시간 성능 비교 대상 아님 |
| [ReactVision/viro-web-renderer](https://github.com/ReactVision/viro-web-renderer/tree/e58c377257c61b8d120251556ecf3c9f7158cf33), wrapper MIT | 별도 tinyvio WASM과 [copy-slam.sh](https://github.com/ReactVision/viro-web-renderer/blob/e58c377257c61b8d120251556ecf3c9f7158cf33/scripts/copy-slam.sh)의 sibling source 경로 확인. README·arSession.ts에 distortion/noise 옵션 미사용 명시 | 공식 `ReactVision/tinyvio` 요청404. tracker 내부·실제 바이너리 미확인, wrapper의 MIT/성능 설명을 자체 검증 결과로 사용하지 않음 |
| [OpenCV4.5.5 LK 소스](https://github.com/opencv/opencv/blob/4.5.5/modules/video/src/lkpyramid.cpp), top-level Apache2.0·개별 파일의 기존 notice | pyramid 입력 재사용, initial-flow prior, LK 최소 고유값 의미. [WASM SIMD intrinsics](https://github.com/opencv/opencv/blob/4.5.5/modules/core/include/opencv2/core/hal/intrin_wasm.hpp) 공개 | local WASM dependency header4.5.5와 버전 일치. latest4.x docs의 수치를 현재 binary 검증으로 전용하지 않음 |

- AlvaAR의 `OPTFLOW_LK_GET_MIN_EIGENVALS` 경로는 `err`를 최소 고유값으로 변경하지만 그 값에 `> errorValue` 거부를 적용. 그대로 이식하기 전에 품질 의미 검토 필요; 공개 코드가 자동으로 정확한 구현이라는 가정 제외
- 라이선스 표: 조회한 코드 범위. 제3자 library·모델·데이터 전체 권리 및 실제 device accuracy 포함 아님

## VideoFrame→수집 Worker 실험 판정

- 판정: **조건부 채택 후보. 영상/IMU 기록과 track-lineage 진단 이후 작은 선택 경로로 검증**
- 목적: main의 draw/read/gray 약17–24ms 점유를 다른 Worker로 옮기는 실험; 현 VIO Worker의 Ceres·LK 처리와 병렬 가능성 확인. CPU/GPU 경쟁으로 총 지연 증가 가능
- 근거: WebCodecs [§9.4.2 constructor](https://w3c.github.io/webcodecs/#dom-videoframe-videoframe)의 HTMLVideoElement 분기는 호출 순간 current playback VideoFrame 사용. `new VideoFrame(video, {timestamp: ...})`는 동기 생성; 이후 비동기 처리에서 video를 다시 읽지 않는 구조 가능
- [§9.4.7 transfer](https://w3c.github.io/webcodecs/#videoframe-transfer-serialization): `postMessage(packet, [frame])`는 송신 객체 detach, 수신 객체에 resource 소유 이전. [`close()`](https://w3c.github.io/webcodecs/#dom-videoframe-close)로 참조 해제
- [HTML canvas image source](https://html.spec.whatwg.org/multipage/canvas.html#image-sources-for-2d-rendering-contexts): `drawImage(VideoFrame)`은 해당 객체 픽셀 사용. OffscreenCanvas2D에서 기존 회전/crop/gray 계산 유지 가능
- SecureContext의 Window/DedicatedWorker VideoFrame, Worker OffscreenCanvas2D **실제 생성·전송·readback** probe 필요. ordinary transferable 경로에 SAB/pthread/COOP 요구 없음; class 존재나 desktop probe만으로 전화 지원 주장 금지
- 명시 timestamp는 기존 `captureTime/presentationTime/arrival` 선택값의 microsecond 표현. optical exposure 검증을 추가하는 API가 아니므로 `physicalExposureVerified=false` 유지

| 계약 | 실험 구현·수락 요구 |
|---|---|
| 픽셀 확보 | rVFC 동기 구간에서 VideoFrame 생성, 생성 전 await 제외. packet의 presentedFrames/metadata·capture generation·geometry 함께 고정 |
| 픽셀 변환 | 첫 실험은 기존 integer gray `(77R+150G+29B)>>8`, rotation/crop/resize 그대로. Y-plane 직접 사용·새 color matrix·GPU packed-gray는 별도 실험 |
| 소유권·backpressure | 최대1 converting+1 pending VideoFrame, 교체·거절·예외·stop의 모든 경로 close. 무제한 Worker message queue 제외 |
| 응답 수락 | frameSeq 단조성·capture generation·configuration·geometry·client epoch 일치. 재설정/stop 이후 늦은 결과 폐기; 기존 최신 gray slot 재사용 |
| 준비·fallback | Worker ready+context probe 성공 후 활성화. 아직 전달되지 않은 요청의 초기화/생성 실패는 기존 동기 rVFC 경로 사용 가능. 전송 완료 후 Worker 실패 요청은 폐기; 나중 video를 읽어 과거 timestamp에 붙이는 fallback 금지 |
| 시각·지연 | main proxy timestamp 유지; Worker timeOrigin을 main과 동일하다고 가정하지 않음. Worker duration, main capture/receive/submit를 각각 기록 |
| 독립 픽셀 증거 | 번호·색상·비대칭 무늬가 있는 고정 frame에 모든 회전·비중심 crop oracle. 원본 read와 전달 frame의 gray SHA 비교; 실제 video 시간 진행 중 지연된 Worker도 동일 ID 확인 |
| 생명주기 증거 | busy/reorder/세대변경/stop→restart, 오류 주입에서 stale 제출0·보유 resource 상한·close 완료 확인 |
| 성능 수락 | 전화에서 main 점유·source→submit·최종 pose age p50/p95/p99, IMU arrival skew/drop, coverage·Lost·reset·온도 비교. 총 지연 또는 tracking 악화 시 채택 제외 |
| 정확도 수락 | 동일 image/IMU replay와 held-out GT의 SE(3) ATE/RPE·rotation·coverage 회귀 없음. desktop 픽셀 통과만으로 실기기 수락 주장 금지 |

- [rVFC](https://wicg.github.io/video-rvfc/)는 compositor 제출 영상에 대한 callback 계약. 생성한 VideoFrame이 metadata frame과 맞는지 실제 번호 영상으로 확인 필요; API 설명만으로 물리 sensor time 동등성 주장 금지
- [VideoFrame.copyTo()](https://w3c.github.io/webcodecs/#dom-videoframe-copyto)는 비동기이며 복수 promise 완료 순서를 보장하지 않음. source format/plane stride/visibleRect/rotation/colorSpace도 별도 계약이므로 첫 실험에서 제외
- [MediaStreamTrackProcessor](https://www.w3.org/TR/mediacapture-transform/#track-processor)는 VideoFrame stream·유한 최신 queue 제공. worker exposure/track-transfer API의 표준·브라우저 차이, timestamp clock 확인이 추가되어 첫 경로로 권고하지 않음

## 구현 순서 권고

| 순서 | 좁은 변경 후보 | 진행 조건·실패 조건 |
|---|---|---|
| 1 | 현 ID/age→LK 성공→geometry inlier→삼각측량/삭제→current factor 연결 계측 | 0/1 factor가 되는 최초 stage 식별, image/IMU 전체 보존. 수량 합계만 증가하고 ID 연결이 없으면 원인 미확인 |
| 2 | 성숙 landmark projection prior·bounded 재매칭 | 소실 stage에 맞는 subset만, 잘못된 pose prior·반복무늬·out-of-FOV control 포함. 잘못된 ID로 pose 개선처럼 보이는 결과 제외 |
| 3 | 위 VideoFrame capture Worker | 실제 main 점유 개선과 end-to-end 수락 gate 만족 시 활성화. VIO 엔진 threading 변경 없이 선택 경로로 평가 |
| 4 | 필요 시 fused video texture/rotation/crop 또는 packed-gray readback | 현재 optional WebGL은 2D portrait→texture upload→RGBA read→R 추출. 이 중복을 줄일 근거는 있으나 color/geometry oracle·동기 readback 비용 검증 먼저 |

- 제외: 전체 sensor Hz·feature 수 일괄 감소, Lost를 stale pose/외삽으로 표시 정상화, threshold 완화로 factor 부재 숨기기
- 제외: 현재 `-msimd128` 설정을 추가하는 중복 변경. [Emscripten SIMD](https://emscripten.org/docs/porting/simd.html)의 코드 생성과 실제 hot kernel 실행·시간 검증 필요
- [Emscripten pthread](https://emscripten.org/docs/porting/pthreads.html)는 SAB/COOP/COEP·별도 threading 설계가 필요. 현재 작은 Worker queue를 근거로 전면 pthread 전환 우선순위 부여하지 않음
- [WebXR](https://immersive-web.github.io/webxr/)은 runtime pose 비교 경로. `isSessionSupported('immersive-ar')`, 실제 session/pose·emulatedPosition·reference-space reset 확인 필요. 자체 입력/추정기 수정의 정확도 증거와 분리

## 재현·검증 한계

```bash
python3 build/research-20261012/web-engine/verify_sources.py
python3 build/research-20261012/web-engine/verify_sources.py --fetch-missing
```

- 첫 명령: 보존 원문 SHA 검사. 둘째: 누락된 공개 원문만 고정 URL에서 재획득 후 같은 SHA 검사; binary LFS 객체 다운로드 제외
- 공식 docs는 조회 당시 snapshot SHA 보존; 향후 live docs 변경은 다른 원문으로 취급. API tree/commit JSON·404도 보존
- source inspection 완료, 제품 실행·binary 역공학·소스 재빌드·신규 전화 비교·성능 개선·독립 정확도 검증 미수행
- 변경 파일: 이 문서와 `build/research-20261012/web-engine/` 근거만. 기존 제품 변경·dataset·서비스 보존
