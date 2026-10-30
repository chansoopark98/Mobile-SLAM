# 상용 SLAM 바이너리 분석 도구

- 완료 기록: [2026-10-14 통합 보고서](../../../../docs/research/2026-10-14-commercial-slam-lab.md).
- 설치: 공식 radare2 **6.2.4**·WABT **1.0.42**, 배포 SHA256 고정. `scripts/dev/tools/re-lab/`에만 추출; sudo·제품 dependency 추가 없음.
- 분석: 원본 JS/WASM/ELF 읽기 → 새 output snapshot → disassembly·직접 call graph·선택 함수 pseudo-disassembly·WASM C lowering. `analysis_sources`에 실행 script snapshot/SHA 보존.
- `pdc`는 원본 source 복구가 아님. 실제 reference runtime stageFrame은 WABT7077명령과 radare7051명령이 달라 분기 누락 경고가 있음. WABT·C lowering·GNU objdump와 교차 확인.
- 직접 call reachability는 간접 호출을 해결하지 않은 하한. 명령 exit0·pseudo-C 생성·정확한 알고리즘 복구·실제 runtime tracking을 별도로 판정.
- output은 새 디렉터리만 허용. 기존 evidence와 canonical `/mnt/backup/SLAM`에 쓰지 않음. 실패 stdout/stderr·hash·exit도 보존.

```bash
python3 scripts/dev/references/re-lab/bootstrap.py

# 최초 reference runtime 자산 획득이 필요한 경우 기존 pinned downloader 사용
python3 scripts/dev/references/xr_reference/bootstrap.py

python3 scripts/dev/references/re-lab/analyze.py \
  --input assets/references/xr_reference/analysis-runtime/standalone/xr-slam.js \
  --output build/re-lab-example/world \
  --export c8EmAsm_stageFrame --export c8EmAsm_setEventQueue \
  --export c8EmAsm_processStagedFrame

# 이미 공급된 공식 ARCore emulator APK의 실제 service ELF
python3 scripts/dev/references/re-lab/analyze-elf.py \
  --input assets/references/ARCore/toolchain/downloads/Google_Play_Services_for_AR_1.56.0_x86_for_emulator.apk \
  --apk-member lib/x86_64/libarcore_c.so \
  --output build/re-lab-example/arcore \
  --export ArSession_update --export ArFrame_getTimestamp
```

- `callgraph-control.wat`: 함수2·import1·직접 edge2(중복 callsite3)·간접 site1의 독립 control. `entry`의 직접 reachability는 import를 포함3·defined2. 실제 toolchain의 알려진 입력으로 parser를 확인.
- 실행 tests: [reference runtime public SDK probe](../xr_reference/README-runtime.md), [ARCore 격리 실행](../../../../docs/research/2026-10-13-arcore-runtime-verification.md), [VINS-Mono replay](../vins-mono/README.md).
- REA6.4.0은 기존 `build/research-20261012/commercial/rea-cli/`에 설치되어 있음. 이 도구의 artifact inventory와 함수 semantic recovery를 동일시하지 않음. 계정·글로벌 MCP/provider 등록 변경 없음.
- raw disassembly·lowered C는 ignored `build/`에만 저장. 제품 수정의 근거는 공개 API·공개 reference source·독립 regression gate로 남김.
- 공식 근거: [radare2 release](https://github.com/radareorg/radare2/releases/tag/6.2.4), [radare2 pseudo-decompiler 한계](https://book.rada.re/arch/decompile.html), [WABT](https://github.com/WebAssembly/wabt/tree/1.0.42).
