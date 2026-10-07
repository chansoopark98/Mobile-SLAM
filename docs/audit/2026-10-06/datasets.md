# 데이터셋 inventory — 2026-10-06

- Canonical dataset home: **`/mnt/backup/SLAM`** — 사용자 지정 원본 저장소
- 실행용 local copy: `assets/datasets/tum/dataset-room1_512_16` 유지; 원본 NAS와 local staging 역할 분리
- 작업 범위: NAS 읽기 전용; 다운로드·압축 해제·삭제·symlink 변경 없음
- 상세 수치·CSV/YAML SHA-256·sample image 검사: `docs/audit/2026-10-06/datasets-inventory.json`
- 판정: 파일·형식 가용성 조사; 추정기 수렴·정확도·production 품질 증거와 구분

## 바로 사용할 자료

| 자료 | 현재 형태 | 현재 loader 기준 |
|---|---|---|
| TUM VI room1 local copy | 추출된 `mav0/cam0`, `imu0`, `mocap0`, `dso` | 현재 headless native replay에 바로 전달 가능 |
| TUM VI room1~4·slides1·magistrale1 NAS | CSV·image·GT·calibration 확인한 tar | local staging 추출 후 사용; sensor 형식 변환 불필요, profile 확인 필요 |
| TUM VI outdoors1 NAS | 17.645GB tar 존재 | 내용 미검증; 현재 실행 준비 완료로 표시 불가 |
| EuROC machine_hall | 12.684GB outer ZIP 안의 MH_01~05 `.zip`/`.bag` | nested ZIP 추출·sensor CSV/calibration/GT 검사 필요; bag 선택 시 별도 변환 |
| Redwood | RGB-D + pose JSON/log + intrinsic NPY | 현재 camera+IMU VIO loader와 형식 불일치; RGB-D/VO 실험 adapter 후보 |
| TartanAir | RGB/depth/7-column pose TXT, 일부 metadata fragment | timestamp·좌표·calibration·IMU 입력 계약 변환 필요; synthetic VO/geometry 후보 |

## TUM VI

- 원본 경로: `/mnt/backup/SLAM/TUM VI/`
- 7개 tar 존재; 6개 archive 전체 member metadata·CSV/YAML 및 첫/마지막 cam0 image sample 확인
- outdoors1: 파일 존재·크기만 확인; 큰 archive 전체 순회는 조사 시간 범위에서 제외

| Sequence | Archive bytes | cam0/cam1 각각 | IMU rows | mocap GT rows | Camera 구간 s |
|---|---:|---:|---:|---:|---:|
| room1 | 1,707,110,400 | 2,821 | 28,122 | 16,541 | 141.00 |
| room2 | 1,759,395,840 | 2,882 | 28,732 | 15,477 | 144.05 |
| room3 | 1,742,612,480 | 2,821 | 28,124 | 15,200 | 141.00 |
| room4 | 1,356,206,080 | 2,228 | 22,212 | 13,075 | 111.35 |
| slides1 | 3,509,268,480 | 5,582 | 55,646 | 12,972 | 279.06 |
| magistrale1 | 10,284,318,720 | 15,447 | 153,989 | 14,218 | 772.32 |
| outdoors1 | 17,645,230,080 | 미검증 | 미검증 | 미검증 | 미검증 |

- 확인한 6개 archive: CSV timestamp 비증가 0건, cam0/cam1 CSV가 참조한 이미지 누락 0건, zero-byte regular member 0건
- 각 archive 첫/마지막 cam0 PNG: OpenCV decode 성공, 512×512 `uint16`; 전체 PNG decode·archive payload 무결성 검사는 미수행
- Camera CSV: `timestamp_ns,filename`; IMU CSV: `timestamp_ns,gyro_xyz(rad/s),accel_xyz(m/s²)`; GT CSV: `timestamp_ns,xyz(m),qw,qx,qy,qz`
- Calibration: `dso/camchain.yaml`의 512×512 pinhole + equidistant distortion, camera↔IMU transform; `dso/imu_config.yaml`의 200Hz·noise 정보
- 확인한 6개 sequence의 camchain/IMU YAML SHA-256은 room1과 동일
- 현재 `config/tum_vi_room1.yaml`: 같은 intrinsic/distortion과 inverse extrinsic 사용, estimator IMU noise는 별도 tuned 값 — `config/tum_vi_room1.yaml:18`, `config/tum_vi_room1.yaml:28`, `config/tum_vi_room1.yaml:59`
- **room1 원본↔복원본 일치**: cam0/cam1/imu0/mocap0 CSV 4종, camchain/IMU YAML 2종의 작은 파일 SHA-256 일치; NAS tar 전체 hash·모든 이미지 동일성은 미검증
- **GT 평가 제한**: GT 행 존재·첫/마지막 시간만으로 연속 GT coverage 보장 불가. 특히 slides1·magistrale1은 camera duration 대비 GT 행 수가 적으므로 association 가용 구간·gap을 평가 시 확인 필요

## EuROC

- 원본: `/mnt/backup/SLAM/EuROC/machine_hall.zip`
- Outer ZIP central directory 20 entries; zero-byte regular entry 0개; central directory parsing 성공
- 내부 sequence: `MH_01_easy`, `MH_02_easy`, `MH_03_medium`, `MH_04_difficult`, `MH_05_difficult`
- 각 sequence에 `.bag`와 `.zip` 존재; 예: `machine_hall/MH_02_easy/MH_02_easy.zip` 1,292,413,952 bytes
- Outer archive에는 직접 `.csv`·`.yaml` entry 없음. Nested ZIP은 compression method 8(deflate); 내부 확인에 필요한 큰 payload 전개는 미수행
- `.zip` 기반 EuRoC `mav0/cam0`·`imu0` 형태는 현재 native 입력 계약과 대응 가능; 실제 nested 내용·PNG·calibration·GT는 추출 후 검증 대상
- GT 경로 차이: native `VIOSystem` 자동 평가는 `mav0/mocap0/data.csv` 고정 — `src/vio_system.cpp:116`. EuRoC `state_groundtruth_estimate0/data.csv` 선택은 별도 evaluator에 명시 필요
- 무결성 한계: outer 전체 CRC·nested ZIP CRC·ROS bag contents 미검증

## 다른 자료

| Redwood raw scene | RGB count | Depth count |
|---|---:|---:|
| apartment | 31,919 | 31,919 |
| bedroom | 21,931 | 21,931 |
| boardroom | 24,321 | 24,321 |
| lobby | 20,000 | 20,000 |
| loft | 25,300 | 25,300 |

- Redwood sample: scene별 RGB 1개 `640×480 uint8`, depth 1개 `640×480 uint16` decode 성공
- `redwood/intrinsic.npy`: `[[525,0,319.5],[0,525,239.5],[0,0,1]]`
- Pose: raw scene의 `.json`은 `PoseGraph`/4×4 pose, `.log`와 `camera_poses/pose_*.zip` 존재. Pose의 원작성·독립 GT·timestamp·depth 단위·camera extrinsic provenance 미검증
- `train/validation/test`는 동일 scene 이름을 공유; 기존 분할을 independent scene/session split으로 채택하지 않음
- TartanAir: 18개 환경의 Easy/Hard Pxxx sequence directory inventory 기록; 전체 image/depth payload 검사 미수행
- Sample `abandonedfactory/abandonedfactory/Easy/P000`: image/depth/pose 각 2,176개; `office/office/Easy/P000`: 각 1,419개
- Sample RGB: `640×480 uint8`, depth NPY: `640×480 float32`, pose TXT: timestamp 없는 7개 숫자. 각 sequence에서 RGB/depth sample 2개 decode 성공·서로 다른 image hash 확인
- `abandonedfactory_Easy_image_left/abandonedfactory/Easy/P000`은 이름과 달리 pose_left/right TXT만 존재하는 metadata fragment. 완전한 image sequence로 계산하지 않음
- Sample 영역에서 실제 IMU CSV·timestamp·calibration 파일 미발견; GT 미분으로 생성한 IMU를 measured VIO 독립 평가에 사용하지 않음

## 분할 후보·실행 순서

1. Development: 이미 실행·분석에 사용한 local room1 유지
2. Engine-selection validation: **room4** 우선 — 가장 작은 TUM archive, 111.35초 전체 sequence, 같은 calibration YAML. 별도 local staging 후 manifest·기준을 동결하고 최초 estimator 결과 관찰
3. Locked final 후보: room2/room3 결과를 tuning에 사용하지 않고 보존; 별도 EuRoC·field scene 추가 검토. Sequence 이름만으로 scene 독립성을 확정하지 않음
4. slides1·magistrale1: 긴 동작·부분 GT association·복구 검사 후보; full-duration 정확도 coverage 보장 없음
5. Redwood/TartanAir: VIO 필수 gate를 대신하지 않는 별도 RGB-D/VO/geometry 실험; conversion 계약·provenance 확인 뒤 사용

- 위 분할: 후보이며 승인된 final split 아님. Scene·session·device 중복·기존 결과 열람 이력·checksum·사용 목적을 split manifest에 기록
- 공식 출처·원본 다운로드 receipt·dataset 사용 조건은 storage 존재만으로 검증되지 않음

바로 실행 가능한 명령 — 현재 local room1, 새로운 결과 directory 사용:

```bash
bash scripts/dev/native-replay.sh \
  "$PWD/assets/datasets/tum/dataset-room1_512_16" \
  "$PWD/config/tum_vi_room1.yaml" \
  "$PWD/build/audit-baseline/nas-room1-inventory-replay" \
  600 bracket 0 1
```

- Harness positional 계약: `DATASET CONFIG OUTPUT [MAX_FRAMES] [IMU_POLICY] [PNP] [CV_THREADS]` — `tests/audit_dataset_replay.cpp:111`
- Room4 후속 실행: local staging 추출·profile 확인 후 dataset 인자만 해당 folder로 변경, 전체 2,228 frames·`bracket 0 1`로 고정; NAS에 결과 저장 금지
- Browser replay는 현재 room1 경로 고정 — `web/js/test-tumvi-app.js:16`; arbitrary NAS/sequence 지원으로 해석 금지

## 조사 한계

- 15~60초 command timeout, top-level/maxdepth listing, scoped archive metadata·CSV·sample decode 사용
- 초기 전체 TUM 순회는 60초 timeout으로 중단; 작은 room1~4/slides1 별도 검사 완료. Timeout을 corruption 판정으로 사용하지 않음
- 전체 `/mnt` scan, NAS 쓰기, huge archive hash, 모든 image decode, nested EuRoC 전개 없음
- 변경 파일: 본 문서와 `datasets-inventory.json`만 작성
