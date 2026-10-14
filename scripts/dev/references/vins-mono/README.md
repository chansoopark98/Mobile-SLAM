# VINS-Mono 비교 환경

- 소스: `assets/references/VINS-Mono`, GPLv3, commit `90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d`
- 기존 checkout을 변경하지 않고 `build/reference-engines/vins-mono/docker-context/`에 SHA256 snapshot 생성
- 공식 ROS Kinetic 환경 사용. ROS image digest·Ceres1.12.0 commit 고정, Ceres/catkin build jobs2
- desktop VIO 기준: camera_model·feature_tracker·vins_estimator. loop closure/AR GUI는 이 비교에서 제외
- 빌드·실행 성공과 추적/GT 통과는 별도 판정. 원본 데이터는 read-only, 결과는 `build/`에 저장

```bash
python3 scripts/dev/references/vins-mono/reference.py stage
python3 scripts/dev/references/vins-mono/reference.py build
python3 scripts/dev/references/vins-mono/reference.py inspect
```

- `build`는 전체 참조 환경의 공통 lock을 사용. 다른 참조 엔진은 다운로드를 병렬 수행하되 CPU build는 같은 lock으로 직렬 실행
- 현재 동결 이미지·source manifest·명령·빌드 로그·실행 결과를 함께 확인. Dockerfile 존재만으로 빌드 성공을 주장하지 않음

## 실제 TUM-VI replay

- `replay.py`: 동결 image ID·source label 확인 → repo room1 calibration과 대조 → gray PNG/IMU ROS bag 생성 → 격리 roscore·tracker·estimator 실행. 원본 mount는 `:ro`, 네트워크는 `none`, CPU 상한2
- `freq:0`으로 첫 입력과 tracker의 첫 publish 생략 외 camera frame을 전달. upstream TUM 기본10Hz와 구분. 현재 room1 YAML의 K·extrinsic·noise·tracker distance·solver 한계를 사용하지만 내부 focal·mask·Ceres·scheduler는 다른 상태
- `verify_inputs.py`: 실제 bag pixel bytes·순서대로 나열한 unique IMU 7-double bytes와 native `input-oracle.bin` 비교. packet 경계·callback timing 일치는 보장하지 않음
- `evaluate.py`: 기존 strict evaluator 재사용. body frame·일대일 GT association·SE3 scale1 ATE/rotation/RPE와 공통 pose timestamp 결과 분리. 속도 우열·phone 정확도 수락 없음
- VINS-Mono의 bounded replay 완료 후 SIGINT 종료에서 estimator가 `-6`을 반환할 수 있음. `shutdown-summary.json`에 보존; 정상 종료 성공으로 보고하지 않음

```bash
python3 scripts/dev/references/vins-mono/replay.py \
  --dataset assets/datasets/tum/dataset-room1_512_16 \
  --output build/new-vins-room1-600 --frames 600 --rate 0.5

# fresh native_replay, yaml profile, 동일600frame·고유IMU·실제 입력 dump
build/new-current-native/native_replay \
  assets/datasets/tum/dataset-room1_512_16 config/tum_vi_room1.yaml \
  build/new-current-room1-600 600 bracket 0 1 10 0.1 1 0 0 600 yaml 0 0

python3 scripts/dev/references/vins-mono/verify_inputs.py \
  --vins build/new-vins-room1-600 --native build/new-current-room1-600 \
  --output build/new-input-equivalence.json
python3 scripts/dev/references/vins-mono/evaluate.py \
  --vins build/new-vins-room1-600 --native build/new-current-room1-600 \
  --ground-truth assets/datasets/tum/dataset-room1_512_16/dso/gt_imu.csv \
  --config config/tum_vi_room1.yaml --output build/new-reference-metrics.json
```

- 실제 실행·hash·결과: [2026-10-13 reference execution](../../../../docs/research/2026-10-13-reference-execution.md)
