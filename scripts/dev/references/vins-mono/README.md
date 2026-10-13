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
