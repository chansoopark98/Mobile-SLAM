# RM-2 sharp 대조 (B6 레인, LFS 이미지 활용)

2026-10-20. 공개 `apps/image-target-cli`(sharp 0.34.5 / libvips 8.17.3, 소스 무수정)를 실제로 실행해 우리 Python CLI(`../image_target_cli.py`)와 비교했습니다. 이 디렉터리는 **신규 도구와 이 문서만** 담고, 기존 파일(`image_target_cli.py` 포함)은 건드리지 않았습니다. 네트워크 요청과 빌드는 없습니다(sharp와 LFS 이미지는 사용자가 승인해 이미 받아 둔 것). 실행 결과와 로그는 `build/slam-refact-full-c1-net/rm2-results/`에 있고 `SHA256SUMS`가 모든 결과 파일의 sha256을 담습니다(이 파일 자체의 sha256은 보고에 적음).

라벨: **VERIFIED** = 이번에 실행해 관측, **UNVERIFIED** = 관측 못함, **EXPLORATORY** = 실행했지만 gate·수락 증거가 아닌 탐색.

## 1. 결론

| 질문 | 답 | 라벨 |
|---|---|---|
| 공개 CLI 출력과 우리 JSON이 같은가 | 16개 입력(공개 test 스크립트 13 + docs 이미지 3) 모두 **JSON이 텍스트까지 동일**(`created`/`updated` 제외), 출력 파일 목록 동일, `_original`·`_cropped`·`_geometry` 이미지는 15/16·15/16·4/4 파일이 픽셀 동일 | VERIFIED |
| 예외 1개의 원인 | `docs/cylinder-diagram.jpg`에 비-sRGB ICC 프로파일("Display")이 있고 sharp는 이를 sRGB로 변환, PIL은 안 함(평균 1.8, 최대 79 레벨). 같은 이유로 `office.png`·`office_w360_h640.png`("DELL U3417W"), `flower-big.jpg`("Camera RGB Profile")도 달라짐 | VERIFIED |
| sharp `.grayscale()`의 실제 식 | 채널별 sRGB→선형, `Y = 0.2126R + 0.7152G + 0.0722B`(선형), **256칸 sRGB 인코딩 표에 선형 보간**(`T[k] = rint(255·enc(k/255))`, `x = 255·Y`), 반올림은 half-up. 순수 채널 ramp 3×256개 100 %, 17³ 격자 4,913개 100 %, 실제 이미지의 서로 다른 RGB 삼중항 363,424개(적합에 쓴 9장)에서 99.9983 %, **적합에 쓰지 않은 24장**의 737,849개에서 99.9982 %. 틀린 경우는 전부 ±1이고, 적합용 6건은 보간값이 `x.49998…`인 반올림 경계임을 확인했습니다(나머지 13건은 경계 여부를 확인하지 않음) | VERIFIED(식), 6/363,424·13/737,849 불일치(±1)의 정확한 원인(libvips 내부 float 순서)은 미규명 |
| 변환 순서 | sharp 파이프라인은 **gray → resize**(`resize().grayscale()`로 써도 gray가 먼저). resize→gray 대 gray→resize 비교에서 33장 중 31장이 비트 동일, 2장이 99.7 % 이상(`livingroom`, `faces`, 스케일≈1) | VERIFIED |
| 현재 PIL 경로(`--gray-mode pil`, 기본값)의 차이 | 16개 케이스의 luminance(JPEG 왕복 포함) 평균의 평균 6.47, 최악 케이스 평균 12.77/p95 32. 순색이 많은 이미지에서 크다 | VERIFIED |
| 현재 `--gray-mode libvips`(연속식)의 차이 | 평균의 평균 0.75, 최악 평균 1.12. 식이 연속값이라 ±1 오차가 남음 | VERIFIED |
| 제안(표 보간 + 그대로 PIL resize)의 차이 | 평균의 평균 0.35, 최악 평균 1.05. **리샘플이 필요 없는 4개 케이스(crop이 이미 480×640)는 JPEG 파일까지 비트 동일** | VERIFIED(몽키패치로 실행) |
| PIL `resize(LANCZOS)`를 sharp와 비트 동일하게 만들 수 있는가 | **못 했다.** sharp와의 차이(리샘플만): 평균 0.1–0.4, p95 ≤ 2, 최대 2–29(날카로운 가장자리). 수직 먼저 + 패스 사이 uint8 반올림 + Lanczos3를 numpy로 흉내 낸 변형은 일부(`flower-big` 94.96 %, `love` 94.79 % 동일)만 가까워지고 `tower`/`flat-diagram`은 그대로(최대 12/19) | VERIFIED(차이), 비트 동일 UNVERIFIED |
| JPEG 인코더 | sharp q80 JPEG의 Y 성분을 PIL q80(L 또는 RGB)로 다시 쓰면 디코드 결과가 6/6 파일에서 비트 동일. **인코더는 차이 원인이 아님** | VERIFIED |
| EXIF 방향 처리 | 방향 2(flip-horizontal)·6(rotate-90-cw) 이미지에서 방향 적용 후 크기·`_original`·`_cropped` 픽셀·JSON이 sharp와 동일(기본/수동 crop 4케이스). 방향 3·4·5·7·8은 샘플이 없어 미검증 | VERIFIED(2, 6), UNVERIFIED(그 밖) |

### 이전 문서의 주장 중 바뀐 것

- `docs/design/image-target.md` §2.4가 "libvips 순서는 crop → gray → resize(리뷰어 서술), UNVERIFIED"라고 했던 순서는 **맞았다**(VERIFIED). "LUT 근사라 마지막 1단계가 다를 수 있다"는 추측도 **맞았고**, 구체적으로 256칸 표 + 선형 보간임을 확인했다.
- 같은 문서가 "sharp `.grayscale()`은 알파를 유지하지만 luminance는 알파를 쓰지 않는다"고 했는데, 관측은 다르다. sharp의 `_luminance.png`는 **RGBA(알파 255 고정, R=G=B)**, `_luminance.jpg`는 **3성분 JPEG(R=G=B)**이다. 우리 CLI는 1채널(L)이다. 픽셀 값(R)은 같고, 우리 엔진은 PGM을 읽으므로 영향이 없지만, **luminance 파일을 직접 읽는 소비자(reference runtime JS 로더 등)가 1채널을 받아 주는지는 확인하지 못했다(UNVERIFIED)**.
- 리뷰 MAJOR-8이 언급한 ICC 변환(Display P3→sRGB)은 실제로 존재한다(위 표). 이전 문서는 이를 다루지 않았다.

## 2. 만든 파일과 재현

`bash slam_refact/tools/image_target/rm2/run_all.sh` (약 1분, 네트워크 없음, `rm2-results/`를 지우고 다시 만든다). 개별 도구:

| 파일 | 역할 |
|---|---|
| `run_cli_matrix.sh` | 공개 CLI 소스를 `rm2-results/orig-cli/`에 복사(원본과 byte 동일을 sha256으로 확인), `node_modules`는 설치된 sharp를 symlink. 16개 입력을 공개 CLI, 우리 CLI `pil`, 우리 CLI `libvips`로 실행 |
| `compare_runs.py` | 세 실행의 JSON(구조·텍스트)과 모든 래스터(크기·평균/p95/최대/동일 비율)를 비교 |
| `sharp_gray_stages.mjs` | 공개 `apply.js`의 luminance 단계를 sharp로 재현(같은 `getDefaultCrop`, `autoOrient`, `extract`, `resize({height:640})`, `grayscale()`)하고 중간 버퍼를 덤프 |
| `sharp_gray_probe.mjs`, `fit_gray_lut.py` | 합성 RGB로 sharp 그레이 식을 규명하고 후보 두 개(연속식 / 표+선형 보간)를 비교 |
| `analyze_gray.py` | 단계별 분석(식, 순서, 우리 경로와의 차이, JPEG 손실) |
| `resample_emulation.py` | sharp resize 대 PIL / numpy 흉내(비트 동일 시도) |
| `run_lut_patch.py` | 제안 변경을 **몽키패치로** 실행(`image_target_cli.py` 무수정). 결과는 `runs/libvips_lut/` |
| `icc_probe.mjs`, `icc_probe.py` | 33장의 임베디드 ICC 프로파일 유무와 sharp 대 PIL 디코드 차이, lcms 변환으로 재현 가능한지 |
| `lfs_image_inventory.mjs` | LFS 이미지의 실제 포맷·크기·EXIF·채널·알파 목록(`lfs_inventory.tsv`) |
| `palm_probe.py`, `blazeface_probe.py` | 손/얼굴 이미지를 numpy 참조 인터프리터로 모델에 넣어 본 **탐색 실행**(§6) |

환경: node v22.21.1, sharp 0.34.5(libvips 8.17.3), Python 3.13.5, PIL 12.3.0, numpy 2.2.6(`rm2-results/env.txt`).

## 3. 결과 상세

### 3.1 공개 CLI 대 우리 CLI (16개 입력, `compare_runs.txt`)

| 항목 | pil (기본) | libvips (연속식) | libvips + 표 보간 (제안) |
|---|---|---|---|
| 파일 목록 동일 | 16/16 | 16/16 | 16/16 |
| JSON 파싱 동일 / 텍스트 동일 | 16/16 / 16/16 | 16/16 / 16/16 | 16/16 / 16/16 |
| `_original` 동일 파일 수 | 15/16 | 15/16 | 15/16 |
| `_cropped` 동일 파일 수 | 15/16 | 15/16 | 15/16 |
| `_geometry`(cone 4케이스) 동일 | 4/4 | 4/4 | 4/4 |
| `_thumbnail` | 평균 0.05–2.2(ICC 이미지 제외 최악 1.96), 최대 18–82 | 같음 | 같음 |
| `_luminance` 평균의 평균 / 최악 평균 | 6.47 / 12.77 | 0.75 / 1.12 | **0.35 / 1.05** |
| `_luminance` 비트 동일 파일 | 0/16 | 0/16 | **4/16** |

- `_original`/`_cropped`가 15/16인 것은 위 ICC 예외 1개(`docs-cylinder-diagram`)입니다.
- `_geometry` 4/4 동일은 `unconify`(원뿔 펼치기) 포트의 픽셀 결과가 sharp 경로와 같다는 뜻입니다. 이전에는 합성 링 텍스처로만 확인했습니다.
- 비트 동일 4개는 `exif-rotated-manual`, `exif-rotated-manual-portrait`, `manual-crop`, `manual-crop-rotated`로, crop이 이미 480×640이라 resize가 없는 입력입니다.
- 썸네일 차이는 PIL Lanczos 대 libvips 차이이고 엔진은 썸네일을 쓰지 않습니다.
- 공개 CLI의 `_luminance.jpg`는 3성분, `_luminance.png`는 RGBA이고 우리 것은 1채널입니다(§1). `cone-diagram.jpg`는 이름과 달리 **PNG 내용**이고 두 CLI 모두 `.png`로 냈습니다(확장자 규칙 일치, VERIFIED).

### 3.2 sharp 그레이 단계 분석 (33장 분석, 아래 표는 CLI 계약을 지키는 16장 중 14장; `gray_analysis*.tsv`)

CLI 계약: 공개 `validateCrop`이 480×640 미만 crop을 거부하므로 그보다 작은 이미지(손·얼굴·ear·하늘 소형 입력)는 CLI 입력이 될 수 없습니다. 이 이미지들의 수치는 `gray_analysis_extra.*`에 있지만 아래 표에서 뺐습니다. 계약을 지키는 16장 중 표에 없는 2종: `amity_up`/`amity_rotated`(원래 gray 480×640, resize 없음이라 모든 경로에서 차이 0)와 `livingroom`(482×642, 스케일 1.003이라 §3.3-4에서 따로 다룸).

| 이미지(기본 crop) | 형식 | pil 식 대 sharp | 연속식 대 sharp | 표 보간 + PIL resize 대 sharp | 같은 해상도에서 표 보간 대 sharp 동일 비율 |
|---|---|---|---|---|---|
| docs/cone-diagram | PNG(RGBA) | 4.87/13/29 | 0.68/1/29 | **0.23/2/29** | 1.0000 |
| docs/cylinder-diagram (ICC) | JPEG | 2.33/14/35 | 0.54/1/9 | 0.23/1/9 | 1.0000 |
| docs/flat-diagram | JPEG | 0.77/2/27 | 0.45/2/27 | 0.41/2/27 | 1.0000 |
| test/cone-cup | PNG | 7.88/13/25 | 0.68/1/22 | **0.10/0/22** | 1.0000 |
| test/cone-fez | PNG | 7.88/13/25 | 0.68/1/22 | **0.10/0/22** | 1.0000 |
| test/exif/flip-horizontal | JPEG | 1.62/5/20 | 0.28/1/20 | 0.22/1/20 | 1.0000 |
| test/exif/rotate-90-cw | JPEG | 1.62/5/19 | 0.28/1/20 | 0.22/1/20 | 1.0000 |
| test/love | JPEG | 5.89/29/41 | 0.17/1/2 | 0.12/1/2 | 1.0000 |
| test/tower | JPEG | 5.06/16/23 | 0.33/1/11 | 0.25/1/11 | 1.0000 |
| imagedetection/flower-big (ICC) | JPEG | 5.35/13/21 | 0.22/1/2 | 0.17/1/2 | 1.0000 |
| semantics/test-input-landscape·portrait (480×640, resize 없음) | PNG | 2.52/10/28 | 0.17/1/1 | **0.00/0/1** | 1.0000 |
| testdata/real_test_face | JPEG | 0.65/2/7 | 0.39/1/5 | 0.24/1/5 | 1.0000 |

(열의 `a/b/c` = 평균 / p95 / 최대 레벨 차이, luminance를 JPEG 인코딩하기 전의 8비트 값에서 계산. 입력 RGB는 sharp가 디코드·ICC 변환한 값이라 ICC 효과는 이 표에 들어 있지 않습니다.)

- 같은 해상도 열: `.grayscale()`만 한 풀해상도 결과(D)와 표 보간 식의 비교로, 33장 모두 동일 비율 ≥ 0.9998, 최대 오차 1입니다.
- 남은 차이(`표 보간 + PIL resize`)는 거의 전부 **리샘플 차이**입니다. 같은 이미지에서 RGB 리샘플만 비교한 차이(`Q3_resample_only_rgb_C_vs_pil`)와 거의 같습니다.
- JPEG 손실(sharp가 쓴 파일을 sharp로 다시 읽은 값 대 인코딩 전 값): JPEG 이미지 평균 1.1–3.4(gray 원본 amity 포함), PNG 0. 우리 CLI도 같은 q80 JPEG이고 인코더 출력이 동일하므로(§1, `analyze_gray.py`의 `JPEG_PIL_*_q80_vs_sharp_file`) 이 손실은 양쪽에 똑같이 들어갑니다.

### 3.3 PIL 경로를 sharp 호환으로 바꿀 수 있는가

1. **그레이 식: 가능, 이미 정확.** 제안 코드(아래)는 33장 전체에서 풀해상도 기준 99.98 % 이상, 최대 ±1입니다.
2. **변환 순서: 이미 맞음.** `--gray-mode libvips`가 이미 gray → resize 순서입니다. `--gray-mode pil`은 resize → gray라서 다릅니다.
3. **ICC: 가능.** 임베디드 RGB 프로파일(Display, DELL U3417W, Camera RGB)을 PIL `ImageCms.profileToProfile(..., createProfile('sRGB'), renderingIntent=perceptual)`로 변환하면 sharp 결과와 99.80–99.94 % 동일, 최대 1 레벨입니다(`icc_probe.json`; relative/saturation도 같고 absolute만 크게 다름). sRGB를 설명하는 프로파일 2종은 변환해도 변화가 없고(차이 0), 그레이 프로파일 2종(amity)은 sharp도 값을 바꾸지 않으므로(차이 0) lcms를 건드릴 필요가 없습니다. CMYK·16비트·팔레트 입력은 샘플이 없어 **UNVERIFIED**.
4. **리샘플: 불가(이번에는).** PIL Lanczos와 sharp의 차이는 평균 0.1–0.4이고 날카로운 가장자리에서 최대 12–29입니다. numpy로 libvips 방식을 흉내 낸 변형도 일부 이미지에서만 줄고(위 §1) 비트 동일은 못 얻었습니다. libvips 소스를 읽지 않았고 정확한 커널·고정소수점 규칙은 UNVERIFIED입니다. crop 크기가 480×640에 가까울 때(`livingroom` 482×642, 스케일 1.003)는 차이가 크게 남습니다(평균 1.5, 최대 55).
5. **luminance 파일 형식(1채널 대 RGB/RGBA)**: 값은 같고 형식만 다름. 소비자가 무엇을 읽는지는 UNVERIFIED.

#### 제안: `image_target_cli.py` 변환 수정안 (이 레인은 적용하지 않음, 소유 레인이 적용)

(a) `_to_gray_libvips`의 식을 표 보간으로 교체(`run_lut_patch.py`가 이 본문을 몽키패치로 실행해 위 결과를 얻음):

```python
_SRGB8_ENCODE_LUT = None  # T[k] = rint(255 * srgb_encode(k / 255)), k = 0..255

def _to_gray_libvips(img):
  if img.mode in _SIXTEEN_BIT_MODES or img.mode == 'L':
    return _to_gray8(img)
  c = np.asarray(img.convert('RGB'), dtype=np.float64) / 255.0
  lin = np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)
  x = np.clip(lin @ np.array([0.2126, 0.7152, 0.0722]), 0.0, 1.0) * 255.0
  i = np.minimum(np.floor(x).astype(np.int64), 254)
  f = x - i
  v = T[i] * (1.0 - f) + T[i + 1] * f          # linear interpolation between table entries
  return Image.fromarray(np.floor(v + 0.5).astype(np.uint8), 'L')
```

(b) 기본값을 `libvips`로 바꾸고 이름을 `sharp`로 바꾸는 것을 권장합니다(실제로 sharp와 대조한 식이므로 "libvips 기억에 따른 UNVERIFIED" 문구 삭제). **기본값 변경은 이미 만든 타깃의 luminance를 바꿉니다**(평균 1–13 레벨). 사용자·리뷰어 결정이 필요한 일이므로 이 레인은 제안만 합니다.
(c) 입력 디코드 직후 임베디드 RGB ICC 프로파일이 sRGB가 아니면 lcms로 sRGB 변환(`original`/`cropped`/`luminance` 모두에 영향).
(d) luminance 파일 형식을 공개 CLI와 맞출지(RGB JPEG/RGBA PNG) 결정. 엔진은 PGM이라 영향 없음.
(e) resize는 PIL Lanczos 유지. 비트 동일 parity가 필요하면 "같은 PGM을 양쪽 엔진에 넣는다"는 기존 규칙을 유지합니다.
(f) 이 문서 `image-target.md` §2.4의 UNVERIFIED 문구, §2.3의 "sharp 설치(네트워크)가 필요해 BLOCKED"와 §6 항목 6을 이 결과로 갱신(문서 소유 레인).

### 3.4 공개 테스트와의 대조 범위

- 공개 `apps/image-target-cli/test/`에는 **기대 출력이 없습니다.** `test/*.txt`는 CLI에 줄 답변 스크립트 13개이고 `test/all.sh`는 출력을 `/dev/null`로 버립니다. 따라서 "공개 기대 출력과의 대조"는 불가능하고, 대신 **공개 CLI를 직접 실행한 출력을 oracle로 삼아** 위 16개 입력을 비교했습니다(VERIFIED).
- 공개 단위 테스트 `src/crop.test.js`, `src/interactive.test.js`를 복사본에서 실행: **23/23 통과**(`logs/public_node_tests.log`). 우리 이식 `tests/test_image_target_cli.py`: **48/48 통과**(`logs/our_unittest.log`). 이 두 테스트는 이미지 픽셀이 아니라 숫자(치수)만 씁니다.
- `love.jpg + default crop`, `tower.jpg` 테스트가 쓰는 입력 치수(1500×1128, 1000×1500)는 실제 이미지와 일치합니다(VERIFIED, `lfs_inventory.tsv`). 이전 문서가 걱정한 "대체 이미지가 3:4라 판별력 약함"은 이 두 테스트에 한해 해소됩니다(수치 기대값은 원래 하드코딩).
- 공개 C++ 테스트가 읽는 이미지의 **실제 크기와 형식**(이전에는 포인터라 알 수 없었음): `flower-big.jpg` 1440×1920 RGB, `amity_up.jpg`/`amity_rotated.jpg` **480×640 8비트 gray JPEG**(둘 다 1채널, 임베디드 gray ICC), `real_test_face.jpg` 2208×2944(3:4). `flower-big.jpg`는 비-sRGB ICC("Camera RGB Profile")가 있어 sharp 경로의 픽셀이 PIL 디코드와 평균 2.7 다르고(§3.3-3), 공개 C++ 테스트의 `readJpgToRGBA`가 ICC를 적용하는지는 확인하지 못했습니다(UNVERIFIED, 적용하지 않을 것으로 추정). 현재 이식(`engine/tests/image_target/*`, `face/ear_fidelity_test.cc`)은 `SyntheticTarget()`과 `kSizes` 목록으로 대체하고 있습니다.

## 4. 새로 가능해진 검증 (LFS 이미지 34개)

| 항목 | 닫을 수 있는 UNVERIFIED | 이번 레인에서 한 것 | 후속 작업 | 이번에 안 한 것 |
|---|---|---|---|---|
| **US-012 CLI 출력 대조** | `image-target.md` §2.3 "공개 CLI 실행 비교 BLOCKED", PRD US-012 보류 사유 "P5 B2(sharp) 대조 승인 대기", §6 항목 6 | 공개 CLI 직접 실행, 16개 입력의 JSON·래스터 대조(§3.1). JSON 텍스트 16/16 동일 | RM-2 재리뷰(독립), `image-target.md`·PRD 문구 갱신, §3.3 (a)–(d) 적용 여부 결정 | 우리 CLI 수정, 문서 갱신, passes 판단 |
| US-012 EXIF 방향 | RM-2의 EXIF 입력 처리 | 방향 2·6 일치 | 방향 3·4·5·7·8 샘플 제작(합성 EXIF) | 합성 샘플 |
| US-012 ICC·색 처리(리뷰 MAJOR-8) | "sharp ICC 변환 미다룸" | 존재 확인, 영향 수치, lcms로 재현 가능함(§3.3-3) | CLI에 ICC 변환 추가·재검증 | 구현 |
| **US-014 곡면 geometry 이미지** | `unconify` 포트의 실 이미지 픽셀 parity | cone-cup/fez 4케이스 `_geometry` 4/4 픽셀 동일 | 다른 반지름·fez 외 케이스 입력 추가 | 추가 케이스 |
| **US-013/014 공개 C++ 테스트 이식 후보** | 이식이 합성 텍스처를 씀(loader·tracker 테스트) | 실제 크기·형식 확인(§3.4) | `flower-big`/`amity_*`를 우리 도구로 luminance(PGM)로 만들어 fixture로 쓰거나 합성 대체 유지를 결정. **공개 테스트는 libjpeg RGBA → GL 셰이더 luminance 경로이고 sharp 경로와 다를 수 있음**(grayscale 식 대 셰이더 식 대조 필요). amity는 이미 gray라 변환 차이 없음 | 이식 테스트 수정, 엔진 코드 |
| OR-4 / US-013 합성 render의 입력 | 합성 텍스처만 사용 | 후보 이미지 크기·형식 목록(`lfs_inventory.tsv`) | `render_synthetic.py`에 실텍스처(`flower-big`, `love`, `tower`) 입력 옵션. 단 실텍스처 render도 실기기 정확도 아님 | render 변경 |
| **US-018 Face 끝단 입력** | "실제 얼굴 입력 없음 → CONFIGURED/UNVERIFIED 상한" | 얼굴 이미지가 존재하고 사람 얼굴임을 확인(육안 1회). `faces.jpg`로 BlazeFace 대체 모델 numpy 실행: 공개 테스트 기대값과 일치(§6) | 우리 엔진 face lane(검출→mesh→ROI)에 실제 이미지를 넣는 harness와 사전등록 gate. 현재 어느 도구도 이 이미지를 읽지 않음(§6) | 엔진 실행, gate |
| US-019 Ear | 실귀 입력 없음 | `ear_left/right.jpg`(112×160)는 귀 사진. palm 모델은 귀 이미지에서 검출 0(관련 없음, 대조군) | ear 모델·face 파이프라인에 넣는 harness. `ear_fidelity_test.cc`의 `kSizes`에 `real_test_face.jpg`의 실제 크기 2208×2944를 추가하는 것을 검토 | 실행 |
| US-018 `face_mesh.glb`, `roi.jpg` | 얼굴 ROI·mesh 렌더 입력 | glb 헤더 파싱: glTF 2.0, 메시 1개, 정점 2718, 인덱스 2718. `roi.jpg` 384×384 | `face_roi_renderer`/mesh 데이터와 대조하는 gate 설계 | 대조 |
| **US-020 Sky 끝단** | "하늘 이미지 입력이 없으면 mask IoU 끝단 UNVERIFIED"(prd-amendment-us020 기준 5) | 입력 후보 목록: `input-landscape-sky.jpg`(256×144, sRGB ICC), `input-sky-landscape.png`, `input-sky-portrait.png`(144×256), `landscape-sky-mask.png`(256×144 RGBA, 5.17 %가 흰색, 좌상단 흰 영역이 같은 이름 입력의 하늘 위치와 겹쳐 보임) | **페어링은 이름과 눈으로 추정(INFERRED)이며 공개 코드는 이 파일들을 어디서도 읽지 않음**(grep). mask가 무엇의 기대값인지(클래스, 좌표 방향) 확정 후 IoU gate 설계. `semantics-model.tflite`(로컬, 입력 [1,256,144,3], 출력 [1,256,144,20])로 실행 가능 | IoU 계산, 실행 |
| US-020 `test-input-portrait.png` | 공개 `semantics-classifier-test`(출력 비율 > 0.04)가 이 파일을 씀 | 파일 확인(480×640 RGBA, ICC 없음) | 같은 입력으로 우리 sky 경로 실행·비율 비교(전처리가 GPU 리사이저라 대조 설계 필요) | 실행 |
| US-021 Hand 끝단 | "깨끗한 손 영상이 없다"(hand-us021 §1) | 손 사진 3장(128/192/224 정사각) 확인, palm 모델 numpy 실행(§6, EXPLORATORY) | 우리 엔진 hand lane(palm→landmark)에 입력하는 harness와 사전등록 gate. 라벨 없음 → 정확도는 비교 기준 설계부터 | 엔진 실행, 정확도 |
| `tflite-interpreter-test` 이식 | 공개 `faces.jpg` 테스트(face_detection_front, 10/0/0) | numpy로 재현(§6, 대체 모델) | 원본 모델(LFS, B1c 제외)을 받으면 우리 C++ 인터프리터로 같은 테스트 이식 | C++ 실행 |
| `tflite-multiclass-test`(office_w360_h640) | 공개 selfie 모델 테스트 | 이미지만 확인(ICC "DELL U3417W", sharp는 색 변환함) | 모델(`selfie_multiclass_256x256`)이 B1c 제외라 보류 | 없음 |

## 5. 하지 않은 것 (경계)

- `image_target_cli.py`, `image-target.md`, PRD, gate 파일, 엔진 코드 수정. 빌드, 서버, headless Chromium, 추가 네트워크.
- sharp와 우리 CLI의 resize를 비트 동일하게 만드는 일(§3.3-4).
- 얼굴/손 이미지를 **우리 엔진**에 넣는 일(§6). 엔진·harness를 건드리거나 빌드해야 합니다.
- 결과를 본 뒤 기준을 바꾸지 않았습니다. 모든 비교는 "차이를 그대로 보고"이고 PASS/FAIL 기준을 새로 만들지 않았습니다. §6의 기대값(10/0/0)은 공개 테스트 원문 그대로입니다.
- 제안 변경(§3.3)의 적용, 기본값 변경, passes/등급 판단.

## 6. 얼굴/손 이미지는 실제 입력으로 쓰이는가 (실행한 것만 VERIFIED)

**현재 repo의 어느 코드도 이 이미지 파일을 열지 않습니다.** `slam_refact/**`와 공개 소스에서 파일명을 grep한 결과 참조는 주석·BUILD·문서뿐입니다(VERIFIED, grep). 우리 엔진(face/hand/sky lane)에 넣는 실행은 이번 레인에서 하지 못했습니다(UNVERIFIED). 대신 **이미 있는 numpy 참조 인터프리터(`tools/ml_ref`)**로 모델에 직접 넣어 봤습니다. 이는 gate도 수락 증거도 아닌 EXPLORATORY이고, 전처리는 PIL(OpenCV 아님), 인터프리터는 float64 numpy입니다.

### 6.1 `faces.jpg` + BlazeFace(`face_detection_front`): 공개 테스트 `TestInvoke` 재현 — EXPLORATORY

공개 테스트 본문(244×128 이미지의 행 0..128, 열 110..238 crop, `x/127.5 - 1`, 로짓 > −log((1−0.75)/0.75) = 1.0986인 위치 수)을 그대로 numpy로 실행했습니다.

| 입력 | 공개 기대 | 실행 결과 |
|---|---:|---:|
| 얼굴 crop | 10 | **10** (상위 10개 로짓 7.76, 7.54, 7.26, 6.94, 4.13, 3.58, 3.36, 2.75, 2.48, 1.42, 11번째 0.73) |
| 검은 이미지 | 0 | **0** (최대 −6.28) |
| 흰 이미지 | 0 | **0** (최대 −1.05) |

**한계(중요)**: 공개 테스트가 쓰는 모델(`third_party/mediapipe/models/face_detection_front.tflite`, LFS oid `0d800dd3…`, 423,340 B)은 B1c 제외로 받지 않았습니다. 대신 `mediapipe-fork/.../face_detection_front.tflite`(423,256 B, sha256 `a31cce73…`)를 썼고 **다른 파일**입니다. 인터프리터도 C++ TFLite가 아닙니다. 그래서 "공개 테스트가 통과했다"가 아니라 "공개 기대값과 일치했다(대체 모델)"입니다. 10번째 위치의 마진은 0.32 로짓으로 크지 않습니다(`blazeface_probe.json`).

### 6.2 손 사진 + palm 모델(`palm_detection`, 256 입력) — EXPLORATORY

`tools/hand/hand_ref.py`(같은 executor의 이전 작성물, T3)의 anchors/디코드/NMS로 점수 ≥ 0.5 검출을 냈습니다(`palm_probe.json`, 겹쳐 그린 그림 `palm_probe_overlay.png`: 빨강 = 최고 점수, 초록 = 2번째).

| 이미지 | 최고 점수 | 검출(NMS 후) | 육안(1회) |
|---|---:|---|---|
| hand_left_up_224 | 0.8695 | 2 | 1위는 엄지 끝, 2위(0.6519)가 손바닥 |
| hand_right_up_128 | 0.9959 | 2 | 1위가 손바닥, 2위(0.9776)는 엄지 근처 |
| hand_left_192 | 0.5179 | 1 | 엄지 끝(손바닥 아님) |
| 대조군: desk, livingroom, office.png | 0.5515, 0.9579, 0.987 | 1, 1, 3 | 손이 없는 장면에서 작은 상자(8–29 px)가 높은 점수로 나옴 |
| 대조군: real_test_face, faces, ear_left | 0.7468, 0.0527, 0.2339 | 2, 0, 0 | 손이 없는 사진인데 검출됨(오검출) |

읽는 법: 손 사진 3장 중 2장에서 손바닥 영역이 검출됐지만 3장 모두 손바닥만 잡는 것은 아니고, 손이 없는 장면에서도 높은 점수의 오검출이 있습니다. **검출 정확도는 UNVERIFIED**(GT 라벨 없음, 육안 1회). 이 모델은 2020 MediaPipe 대체 모델이고 원본 reference runtime 모델이 아닙니다(`spec/10-hand.md` §0).

## 7. 증거 위치

`build/slam-refact-full-c1-net/rm2-results/`: `compare_runs.{txt,json}`, `gray_analysis{,_extra}.{txt,tsv,json}`, `fit_gray_lut.json`, `resample_emulation{,_extra}.{txt,json}`, `icc_probe.json`, `lfs_inventory.tsv`, `blazeface_probe.json`, `palm_probe.json`, `palm_probe_overlay.png`, `env.txt`, `orig-src.{ref,copy}.sha256`(공개 소스 복사본 일치), `logs/`, `runs/{orig,pil,libvips,libvips_lut}/`(전 출력), `SHA256SUMS`.
