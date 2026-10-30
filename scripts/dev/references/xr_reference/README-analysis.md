# reference runtime 상세 분석·실험 도구

- 진입 문서: [2026-10-14 종합 조사](../../../../docs/research/2026-10-14-xr_reference-study.md).
- 전체 공식 저장소: `assets/references/xr_reference/official/`; 원본 SDK: `assets/references/xr_reference/analysis-runtime/standalone/`.
- clone·원본 bytes·SDK 실행·tracking·GT 정확도를 각각 구분. 제품 source와 기존 SDK는 수정하지 않음.
- 기존 public probe 설명은 [README-runtime.md](README-runtime.md) 유지.

```bash
# 새 증거 폴더 이름 사용; 기존 결과 덮어쓰기 금지
python3 scripts/dev/references/xr_reference/catalog.py --jobs 3 \
  --evidence build/xr_reference-catalog-rerun
python3 scripts/dev/references/xr_reference/binary-survey.py \
  --input assets/references/xr_reference/analysis-runtime/standalone \
  --output build/xr_reference-binary-survey-rerun
node scripts/dev/references/xr_reference/source-contracts.mjs \
  build/xr_reference-source-contracts-rerun
node scripts/dev/references/xr_reference/experiment-matrix.mjs \
  --label new-moving-responsive --arm moving --scale responsive --frames 240
```

- `catalog.py`: API 전체 페이지 원문·origin/HEAD/LFS 기록, 없는 공개 repo clone, 기존 checkout 보존.
- `binary-survey.py`: 5개 embedded WASM validation/disassembly/import/export/opcode/direct graph·radare 경계 함수 관측. 도구 설치는 기존 [re-lab](../re-lab/README.md) 참조.
- `source-contracts.mjs`: upstream TypeScript 원본을 Node22 type stripping으로 실행; 9개 독립 기대값 검증.
- `experiment-matrix.mjs` + `experiment-runtime.js`: 실제 원본 SDK와 합성 camera/sensor, scale/visual/sensor/lifecycle 제어. 새 browser와 localhost server만 사용.
- 추가 내부 semantic 분석·feature 실행·실험 요약 명령은 상세 보고서의 재현 절차 참조.
- dependency 추가 없음. 전체 upstream build/test·물리 phone 실험은 별도 검증 영역.
