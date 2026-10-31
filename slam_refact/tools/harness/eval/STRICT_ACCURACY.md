# XR P0 strict 정확도 평가기

- 새 경로: `strict_accuracy.py`, 독립 합성 시험: `test_strict_accuracy.py`.
- 기존 `accuracy.py`, `accuracy_matrix.py`, 등록 gate·과거 결과는 변경하지 않음.
- 범위: 단일 ONLINE pose 기록의 시간 1:1 대응, camera→body 변환, 고정 SE(3), 위치·회전 ATE와 1 s RPE, 전체 세션 coverage.
- 미구현: 두 arm의 공통 mask, macro 비교, correction event 평가, locked-final·제품 gate. `assessment=metrics_only_not_acceptance`이며 종료 0도 제품 수락이 아님.

## 입력 계약

- 녹화 JSON: `slam-refact-phone-metrics-v1.poseLog` 또는 `slam-refact-xr-poselog-v1`/`v2`/`v3` 객체. `rows`, `droppedRows`, `nullUpdates` 필수. 각 row의 quaternion은 키 객체 `{x,y,z,w}`이며 v1 metric 입력에서 명시적으로 wxyz로 변환.
- GT JSON: 각 행 `[t_seconds, x_m, y_m, z_m, qx, qy, qz, qw]`. **world-from-body**, 오른손 좌표계. 시간은 엄격 증가, 중복 금지.
- sidecar protocol 필수. 이미 계측한 사실을 기록하는 파일이며, 누락된 frame ID·시각·metric scale을 만들어 넣는 용도가 아님.
- 현재 phone recorder v1은 responsive 비미터 pose이며 source identity·epoch·clock mapping이 부족함. 이 상태 그대로는 `UNVERIFIED`/종료 2가 정상. `tPerfMs`는 callback 시각이므로 source/GT 시각으로 사용하지 않음.
- **v2는 형식·모순 검사만 지원하며 metric 계산은 거부**: 현재 v2 schema 계약은 responsive 비미터 좌표이고 `videoTimeS`/`frameTimeS`는 업로드된 현재 영상의 시각이다. `sourceFrameIndex`가 가리키는 pose는 한 프레임 늦으므로, 현재 영상 시각을 pose-source 시각으로 옮기지 않는다. `meta` 서술을 바꾸거나 protocol에 `metres`라고 써도 `UNVERIFIED`/종료 2. metric 변환 기능은 추가하지 않음.
- v2에서 non-null `sourceFrameIndex`는 명시 sidecar의 `source_id`와 같아야 함. null은 unknown이며 추정하지 않음. `repeatFrame`은 boolean/null 형식만 검사하고 `fresh=!repeatFrame`로 추론하지 않음. `fresh`·`epoch`·full source schedule은 여전히 별도 계측 근거로 제공해야 하며 누락·충돌·fresh 중복·역전은 거부.
- v2 후속 metric export에 필요한 새 계약: pose의 실제 source timestamp/clock mapping, 기계 판독 가능한 단위와 검증된 metric scale, source ID·fresh·epoch, 고정 전체 frame schedule. 이 API/계측 변경은 recorder 소유자에게 인계하며 현재 evaluator에서 만들어 넣지 않음.
- **v3도 진단 전용**: M0 `row.source`의 `{sessionEpoch,sourceFrameIndex}`와 `sourceVideoTimeS`를 읽지만 단위는 여전히 responsive 비미터, 물리 exposure 미검증. CLI는 항상 `UNVERIFIED`/종료 2이며 `diagnostics`에 source freshness·unknown/no-pose/old-epoch 수만 추가. v2의 검증 전용 동작도 유지.
- v3 `fresh=true`는 hasPose·matched metadata·현재 epoch·engine non-repeated·publication non-repeated가 모두 맞아야 함. 중복 fresh tuple, epoch 역행, 동일 epoch의 source index/time 역행, raw legacy index 모순을 거부. `newEngineResponse=false`인 callback-repeat라도 이전 실패 후 처음 발행한 source라면 fresh가 될 수 있어 이 플래그로 fresh를 대신 판단하지 않음.
- `row.videoTimeS/frameTimeS`는 각각 source의 **callback** 시각과만 대조. 정상적인 업로드 영상–pose source의 한 프레임 차이는 오류가 아님. source nanoseconds는 staged source frame/stage 시간과 대조. no-result·source=null·metadata 미상·이전 epoch held를 fresh pose로 승격하지 않음.
- `no_pose_rows`는 명시적인 `hasPose=false`만 집계. 누락/null은 포즈 부재로 단정하지 않음. `kind=no-result`에 pose·source metadata·fresh 또는 source tuple/time이 있으면 모순으로 거부하되 현재 session epoch와 callback/engine 실행 정보는 보존.
- v3 진단만 읽을 때 protocol `{}`를 줄 수 있음. GT 변환은 수행하지 않으며 `clock_mapping=unverified_no_GT_conversion`. sidecar row_metadata를 주면 기존 full schedule·epoch·source ID 계약과 **`timestamp_field: "source.sourceVideoTimeS"`**의 명시적 clock mapping을 추가 검사하지만 metric 계산은 여전히 거부. v1용 `videoTimeS`를 v3 pose 시각으로 선택하면 거부. 현재 sidecar의 정수 source ID는 epoch 재시작을 평탄화하는 새 매핑 계약이 아니므로 다중 epoch의 재사용 ID를 임의로 재번호 매기지 않음.

| protocol 필드 | 의미 |
|---|---|
| `schema` | `slam-refact-xr-strict-protocol-v1` |
| `pose_convention`, `pose_axes`, `pose_units`, `trajectory_kind` | 각각 `world_from_camera`, `right_handed`, `metres`, `published_online` |
| `metric_scale_provenance` | 독립적으로 확인한 metre scale의 근거. GT에 맞춘 scale 또는 임의 `0.7`은 근거가 아님 |
| `ground_truth_kind`, `ground_truth_provenance` | `independent` / `synthetic` / `reference`와 출처. ARCore는 `reference` |
| `physical_exposure_verified` | 명시 boolean. false여도 계산은 가능하지만 exposure 검증으로 표현하지 않음 |
| `timestamp_field` | `videoTimeS` 또는 `frameTimeS`; 자동 선택 없음 |
| `timestamp_scale_to_gt`, `timestamp_offset_to_gt_s`, `clock_mapping_provenance` | `t_GT = scale × row[field] + offset`, 검증한 clock mapping의 근거 |
| `source_schedule` | 결과를 보기 전에 고정한 전체 source frame 배열 `[{"id": 0, "t": 0.0}, ...]`; `t`는 GT clock의 초 |
| `expected_frames` | `source_schedule.length`와 정확히 같아야 함. 성공 pose 수로 분모를 줄이지 않음 |
| `row_metadata` | 저장된 row마다 `{source_id, fresh, epoch}`. **원 source 계측**에서 생성, callback 순서로 추정 금지. 길이는 poseLog.rows와 같아야 함 |
| `accepted_statuses` | 사전 고정한 유효 pose 상태. 예: `["NORMAL"]`; 다른 상태의 row는 분모에 남김 |
| `T_body_camera` | row-major SE(3) 4×4. 입력 camera 축의 점을 body 축으로 옮기는 변환. metre translation |
| `association_max_dt_s`, `source_timestamp_tolerance_s`, `rpe_tolerance_s` | GT 대응, source identity 시각 일치, `1 s ± tolerance`의 사전 고정 범위 |
| `alignment` | `{type:"fixed", T_gt_est:[...]}` 또는 `{type:"fit_se3", source_ids:[...]}`. 초기 정렬 frame 집합은 사전 고정 |

- camera pose가 `R_WC, p_WC`, `T_body_camera=(R_BC,t_BC)`라면 `R_WB=R_WC R_BCᵀ`, `p_WB=p_WC−R_WB t_BC`. GL/OpenCV 자동 축 추정 없음. 해당 카메라 축에 맞는 extrinsic을 제공해야 함.
- source ID 재사용 fresh row, fresh source 시각 역전, GT 중복/역전, epoch 역행, 비유한 수치·비정상 quaternion·잘못된 rigid transform은 거부.
- 입력이 유한해도 body 변환·정렬·잔차·통계 계산 중 overflow/비유한 값이 생기면 거부. legacy 통계 helper가 잔차를 걸러내기 전에 유한성·association/pair 수와 표본 수의 일치를 검사하며, 잘못된 `alignment` 타입도 `UNVERIFIED`/종료 2로 반환.
- GT 대응은 모든 tolerance 내 후보를 시간차 우선으로 정렬하고 양쪽 한 번만 사용. 동률은 입력/GT 인덱스 순. crossing 대응은 모호한 조건으로 거부. 보간·GT 재사용 없음.
- source timestamp는 매핑 후 schedule 시각과 맞아야 함. 대조된 pose의 실제 source 시각으로 RPE pair를 고르며 correction/reset 경계를 제외하지 않음.
- SE(3) 정렬은 단 한 번, scale=1. 같은 회전·병진을 위치와 자세 모두에 적용. 위치 기반 정렬의 rank<2 또는 지정 초기 frame 누락 시 조용히 다른 정렬로 바꾸지 않음.
- coverage 분모는 모든 source schedule frame. lost/미초기화/미수신/source 누락/repeated/GT 미대응을 별도 목록과 수로 보존.
- `droppedRows>0`인 제한 녹화는 계산값이 있어도 `recording_complete=false`, CLI 종료 2. 최소 3개 GT 대응 미만·RPE pair 0이면 완료 아님.

## 실행

저장소 루트에서:

```bash
python3 -m unittest discover -s slam_refact/tools/harness/eval \
  -p 'test_strict_accuracy.py' -v

python3 slam_refact/tools/harness/eval/strict_accuracy.py \
  --recording <pose-log.json> --protocol <frozen-protocol.json> \
  --ground-truth <body-gt.json> --output <new-output.json>
```

- output이 이미 존재하면 덮어쓰지 않음.
- 종료 0: 요구된 단일 sequence 지표가 계산됨. 종료 2: 메타/대응 부족·불법 입력·누락 녹화·RPE 미산출. `UNVERIFIED`와 오류 사유 또는 계산된 부분 결과를 보존.
- 합성 시험: 정확한 square 좌표, 2배 scale, 알려진 전역 SE(3), 일정/변화 yaw, 회전 lever arm, 반복/누락/GT 경쟁, reset 경계, 불명확한 calibration/provenance.
- legacy 보호: `accuracy.py`의 GT 재사용 가능성과 별도 rotation-only 정렬은 이 파일에서 변경하지 않음. strict 경로와 legacy 수치를 같은 정의라고 비교하지 않음.
