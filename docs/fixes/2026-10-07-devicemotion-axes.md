# DeviceMotion angular velocity 축 수정

- 원인: `rotationRate`를 DeviceOrientation Euler 이름과 혼동하여 `(beta,gamma,alpha)`를 device `(x,y,z)`로 입력
- 근거: [W3C DeviceMotionEventRotationRate](https://www.w3.org/TR/orientation-event/#devicemotion-rotationrate)의 `alpha=x`, `beta=y`, `gamma=z`; 각속도 단위 `deg/s`
- Chrome `154.0.8037.126` 원본 확인: `build/dev-android/chrome154-device-motion-pump.cc:165`의 gyro `(x,y,z)` → `Create(...)`, `build/dev-android/chrome154-device_motion_event_rotation_rate.cc`의 `Create(alpha,beta,gamma)`·동일 순서 저장
- 수정 범위: `web/js/imu.js`, 새 `tests/browser/imu-axes-contracts.test.mjs`, 기존 clock/bias/mobile-source 계약 중 gyro 입력·기대값
- 회귀 우선: device 각 축 `90deg/s` 독립 기대값 `π/2rad/s`, 동일 물리 `(x,y,z)`에 대한 Generic Sensor·DeviceMotion 동등성 검증
- 변경: DeviceMotion 각속도 입력만 `(alpha,beta,gamma)`로 수정
- 유지: Generic Sensor `(x,y,z)`, body 변환 `[x,-z,y]`, 가속도 부호, 시간축·raw gyro 보존, DeviceOrientation Euler `(alpha=Z,beta=X,gamma=Y)`
- 제외: `R_ic` 보정·calibration fitting·C++/WASM·의존성 변경
- 검증: `node --test tests/browser/imu-axes-contracts.test.mjs tests/browser/imu-clock-contracts.test.mjs tests/browser/imu-bias-contracts.test.mjs tests/browser/mobile-source-contracts.test.mjs`, `bash scripts/dev/check.sh syntax`
- 한계: 실제 확인 기기는 Galaxy Z Flip 6·Chrome; iOS는 동일 표준 입력 mock 검증이며 실제 iOS sensor 실행 미검증

## 회귀 결과

- 변경 전 새 회귀 4/4 실패: 각 축 분리·Generic/DeviceMotion 동등성 오류; `build/dev-android/axes-red.log`
- 변경 후 새 4개 + 기존 clock/bias/mobile-source 계약 30/30 통과·skip/TODO 0; `build/dev-android/axes-green.log`
- 전체 browser JS 계약 117/117 통과·skip/TODO 0; `build/dev-android/axes-all-browser-contracts.log`
- `bash scripts/dev/check.sh syntax`, `git diff --check` 통과; `build/dev-android/axes-syntax.log`
- 기존 잘못된 테스트 기대값도 W3C·고정 Chrome 원본에 맞춰 수정; 새 회귀는 축별 물리 입력과 `π/2rad/s` 독립 기대값 사용
- 이 변경 작업 중 실제 전화 reload·C++/WASM 수정 없음; phone 전후 추정 결과·native A/B는 별도 root 검증
