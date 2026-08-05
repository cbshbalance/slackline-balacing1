# exp_data — 실물 실험 데이터 보관소

어름이(줄타기 로봇) 실물 실험의 **원시 데이터 정본** 폴더. 앞으로 실험 데이터는 전부 여기에 쌓는다.

## 폴더 규약

```
exp_data/
  <YYMMDD><요일>/          예: 260805WED  ← 실험일 단위 폴더
    ereumi/ 또는 직접      ← 절차서 절 번호별 하위 폴더 (10.1, 10.2, 11, 12 …)
      <절번호>/*.csv       ← 로그 (exp_logger.py 산출)
      <절번호>/<스케치>/   ← 그날 실제 사용한 펌웨어 사본 (당시 상태 보존)
      *.mp4 등             ← 검증용 영상 (가급적 50MB 이하로 압축)
```

- **파일명**: `MMDD_실험_회차.csv` (예: `0805_exp2_tilt_r3.csv`). 재실험은 회차 숫자만 올린다.
- **원시 CSV는 불변** — 수정·삭제 금지. 잘못된 회차도 지우지 말고 그대로 둔다(0바이트 파일 = 포트 충돌 등으로 기록 실패한 회차의 흔적).
- 코드의 **정본은 `v19_bringup/`** (firmware/, calib/)에 두고 거기서 유지보수한다. 데이터 폴더 안의 사본은 "그날 쓴 버전"의 스냅샷으로 보존.
- 분석 결과 수치는 `docs/어름이_조립후_실험절차서_완전판_v19.docx`의 기록란과 `v19_bringup/params_v19.py`(MEASURED=True)에 기입.

## 보관 현황

| 폴더 | 실험 (절차서 절) | 내용 |
|---|---|---|
| `260805WED/ereumi/10.1` | 크랭크 단독 자유흔들기 | `0729_crank_r1~r5.csv` + `crank_swing_log.ino` |
| `260805WED/ereumi/10.2` | 로봇 장착 자유흔들기 | `mounted_swing_log.ino` (CSV 없음) |
| `260805WED/ereumi/11` | 실험② 기울여 놓기 (λ_u·w) | `0805_exp2_tilt_r1~r3.csv` (r2는 0바이트 실패), `0802_exp2_r1.csv`(0바이트), `0802_exp3_r1~r4.csv`, `tilt_release_test.ino`, 촬영 영상 |
| `260805WED/ereumi/12` | 실험③ 허리 흔들기 (I_δ) | `hip_swing_test.ino`, `exp_analysis_v19.py` 사본 |
| `260805WED/ereumi/` | 공통 | `exp_logger.py` 사본, 완전판 절차서 v19 사본, motor_profile_test류 |

## 실험 절차 빠른 참조 (실험② 기준)

1. `tilt_release_test.ino` 업로드 → 시리얼 모니터에서 각도 3개(φ·발목·δ) 정상 확인(φ 배선 헐거움 이력!) → 시리얼 모니터 닫기
2. `python exp_logger.py COM<n> MMDD_exp2_tilt_r<k>.csv`
3. `k`(허리 잠금) → `z`(영점) → `s`(스트리밍) → [`m` → 손 놓기 → 잡고 `m`] 좌우 10회 → `s` → `quit`
4. 분석: `python v19_bringup/calib/exp_analysis_v19.py exp2 <csv> --params v19_bringup --alpha <확정값>`
