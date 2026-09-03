# v22_logger — Claude Code 작업 규칙

로거·측정 앱 (v21_pres 사본 + 측정 앱, 포트 8220). **먼저 `README_v22.md` 와 `../docs/v22_로거_측정앱_설계노트_20260902.md` 를 읽을 것.**
v21 발표 시뮬레이터는 `/pres` 로 그대로 살아 있다 (`static/index.html`, `sim_server_v21.py`).

## 절대 규칙 (프로젝트 문서 55 §6 계승)

1. **코드를 고치기 전에 변경안을 사용자에게 보이고 확인받는다.**
2. `v19_*`, `v21_pres/` 등 **원본 폴더는 절대 수정하지 않는다.** 작업은 이 폴더 안에서만.
3. **주장하기 전에 잰다** — `python tests_v22.py` + Playwright `e2e_v22.py` + `python tests_v21.py`(물리 회귀) 통과 확인.
4. 틀린 결론은 문서에 철회 기록을 남긴다.

## 설계 원칙 (사용자 의도, 9/2)

- **펌웨어 계산값을 믿고 그리지 않는다.** 원시 엔코더 열에서 앱이 다시 계산(`dataset_v22.py`), 펌웨어 열은 `_fw` 로 나란히.
- **적합은 전부 구간·점·식·잔차를 되돌려준다** (`analysis_v22.py` 의 `steps/used/overlay/curves`). 숨은 처리 금지.
- 정본 대조값을 시험에 박아 둔다: P2R 0.4285 ± 0.0011 / λ 5.44 ± 0.59, φ_eq +1.40 (문서 64·70).
- 시뮬 물리(`sim_engine.py`)는 여기서도 불변. 평면 상수(P, r, slopeA0…)만 읽어 쓴다. MuJoCo 없으면 `PL_FALLBACK`.

## 파일 지도

- `server.py` 허브(WS `/ws2`, 바이너리 프레임, 기록, 파일, 분석 디스패치) · `serial_bridge.py` 소스/분류/기록기 ·
  `dataset_v22.py` 데이터 정본 · `analysis_v22.py` 측정 도구 · `commands.json` 명령 팔레트(펌웨어별 프로파일)
- `static/logger.html` 레이아웃 · `lg_core.js`(상태·WS·저장소) → `lg_3d.js`(트윈) → `lg_plane.js` → `lg_chart.js` → `lg_panels.js`(패널·트랜스포트·분석 UI·키·프레임 루프) — **이 순서로 로드된다**
- `logs/` 기록 (4파일 + .folds). `e2e_shots/` 스크린샷.
- `mujoco_source.py` MuJoCo 가상 로봇(v22_raw v2 펌웨어 흉내, `sim release β φ`) · `video_v22.py` 리허설 영상 감독(Playwright 녹화, `video_out/`).
- 발표 창: `static/deck.html`+`lg_deck.js`(조종석, 큐·장면 신호) · `static/show.html`+`lg_show.js`(무대, 장면 전환·실사 오버랩·HUD) · `static/lab.html`+`lg_lab.js`(측정실 = logger.html 의 모든 id 를 가진 상위집합 + 엑셀식 표·차트·추세선; `lg_panels.js` 가 그대로 돌고 `lg_lab.js` 는 `LG.chart.win/view`·`LG.sel`·`LG.setCursor` 를 공유). 리허설 `video_show_v22.py`(무대) · `video_lab_v22.py`(측정실, video_v22.py 의 Director 상속). 서버 `set_scene`/`scene` 방송, 라우트 `/deck /show /lab /pmedia/ /repo/`. 절차 `docs/v22_발표_무대_운용_20260903.md`.

## 함정

- 스크립트 로드 순서가 곧 의존 순서다. `LG` 전역 하나. `const` TDZ 주의 (문서 58 교훈).
- 오른쪽 탭 안의 요소는 탭이 켜져야 보인다 — E2E 에서 `fill` 전에 탭 클릭.
- 열 이름은 펌웨어가 준 그대로다. canonical 접근은 `LG.col('phi')` / `ds.col('phi', i)` (별칭표 `ALIASES`).
- 라이브 append 와 rebuild 는 같은 결과여야 한다 (tests_v22 결정론 시험).
- 서버 프로세스 정리는 포트 기준 (`fuser -k 8220/tcp`).
