# -*- coding: utf-8 -*-
"""v22 브라우저 E2E (Playwright). 서버를 --fake 로 먼저 띄운다:
    python server.py --fake --port 8231 &   ;   python e2e_v22.py 8231
검사: 로드·JS 오류 0 · 라이브 스트림 · 명령/마크 · 파일 로드 · 분석 도구(P2R·시행·λ) · 스크럽/재생 · 파이프라인 · 숨김 · 재연결
스크린샷: e2e_shots/*.png
"""
import os
import sys
import time

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8231
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "e2e_shots")
os.makedirs(OUT, exist_ok=True)
FAILS, ERRS = [], []


def check(name, cond, detail=""):
    print(f"  [{'OK ' if cond else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))
    if not cond:
        FAILS.append(name)


from playwright.sync_api import sync_playwright

with sync_playwright() as p:
    br = p.chromium.launch(executable_path="/opt/pw-browsers/chromium", args=["--use-gl=swiftshader", "--enable-unsafe-swiftshader"])
    pg = br.new_context(viewport={"width": 1600, "height": 950}).new_page()
    pg.on("pageerror", lambda e: ERRS.append("pageerror: " + str(e)))
    pg.on("console", lambda m: ERRS.append("console: " + m.text) if m.type == "error" and "404" not in m.text else None)
    pg.goto(f"http://localhost:{PORT}/")
    pg.wait_for_function("() => window.LG && LG.PL && LG.ds.n > 50", timeout=20000)
    time.sleep(1.0)
    check("페이지 로드 + 라이브 스트림 수신", pg.evaluate("LG.ds.n") > 50, str(pg.evaluate("LG.ds.n")))
    pg.screenshot(path=os.path.join(OUT, "s1_live.png"))
    chip = pg.text_content("#hConn")
    check("연결 칩 (가짜 · Hz)", "가짜" in chip and "Hz" in chip, chip)
    ov = pg.text_content("#ov3d")
    check("3D 판독 표시", "φ" in ov and "Â" in ov, ov.split("\n")[0])
    check("원본값 표 행", pg.locator("#rawTable tr").count() >= 5, str(pg.locator("#rawTable tr").count()))
    check("파생값 Δ(펌웨어 대조) 표시", "펌" in (pg.text_content("#derTable") or ""))
    check("LIVE 모드", pg.text_content("#tMode").strip() == "LIVE")
    n1 = pg.evaluate("LG.ds.n"); time.sleep(1.0); n2 = pg.evaluate("LG.ds.n")
    check("라이브 append 진행 (~100 Hz)", n2 - n1 > 50, f"+{n2-n1}/s")
    # 명령·마크
    pg.fill("#iLine", "t"); pg.press("#iLine", "Enter"); time.sleep(0.5)
    check("콘솔 명령 전송·에코", "명령 수신: t" in (pg.text_content("#console") or ""))
    pg.click(".tabs button[data-tab=rec]")
    pg.fill("#iMark", "RELEASE 왼손"); pg.click("#bMark"); time.sleep(0.5)
    check("마크 → 이벤트", pg.evaluate("LG.aux.events.some(e=>e[1]==='RELEASE')"))
    check("기록 칩 (자동 기록)", "REC" in pg.text_content("#hRec"))
    # 명령 팔레트
    pg.click(".tabs button[data-tab=cmd]")
    check("명령 팔레트 버튼 생성", pg.locator("#cmdGroups .btn").count() >= 10, str(pg.locator("#cmdGroups .btn").count()))
    pg.locator("#cmdGroups .btn", has_text="p 1회").click(); time.sleep(0.4)
    check("팔레트 버튼 → 전송", "명령 수신: p" in (pg.text_content("#console") or ""))
    # 파일 로드 (끊은 뒤)
    pg.click("#bDisc"); time.sleep(0.6)
    check("끊기", not pg.evaluate("LG.link.connected"))
    pg.click(".tabs button[data-tab=rec]"); pg.click("#bFiles"); time.sleep(0.5)
    pg.select_option("#fileList", "0822_p2r_test.csv"); pg.click("#bFileLoad")
    pg.wait_for_function("() => LG.ds.n === 3013", timeout=15000); time.sleep(0.5)
    check("파일 로드 (P2R 3013행)", True)
    check("END 모드 (연결 없음·따라가기)", pg.text_content("#tMode").strip() == "END")
    pg.screenshot(path=os.path.join(OUT, "s2_file_loaded.png"))
    # 분석: P2R
    pg.select_option("#anTool", "p2r"); pg.click("#bAnAll"); pg.click("#bAnRun")
    pg.wait_for_function("() => LG.results.length >= 1 && LG.results[0].res.tool === 'p2r'", timeout=20000)
    r = pg.evaluate("LG.results[0].res.result")
    check("P2R 도구 결과 0.4285", abs(r["P2R"] - 0.4285) < 0.0011, str(r["P2R"]))
    check("결과 절차(steps)·표·오버레이 렌더", pg.locator("#anOut ol li").count() >= 4 and pg.locator("#anOut table.tb tr").count() >= 9 and pg.evaluate("LG.chartOverlay.length") >= 9)
    pg.screenshot(path=os.path.join(OUT, "s3_p2r.png"))
    # 분석: 시행 + λ (lambda 파일)
    pg.select_option("#fileList", "0822_lambda_test.csv"); pg.click("#bFileLoad")
    pg.wait_for_function("() => LG.ds.n === 9087", timeout=15000); time.sleep(0.3)
    pg.select_option("#anTool", "trials")
    pg.fill("#anParams input[data-k=phi_eq]", "1.4"); pg.click("#bAnRun")
    pg.wait_for_function("() => LG.results.length >= 2 && LG.results[0].res.tool === 'trials'", timeout=30000)
    nt = pg.locator("#anTrial option").count()
    check("시행 나누기 → 시행 선택 목록", nt >= 10, str(nt))
    check("놓기점 평면 오버레이", pg.evaluate("LG.planeOverlay.length") >= 1)
    pg.select_option("#anTrial", "2"); time.sleep(0.3)
    check("시행 선택 → 구간·커서·REPLAY", pg.text_content("#tMode").strip() == "REPLAY" and pg.input_value("#anT0") != "")
    pg.select_option("#anTool", "lambda"); pg.fill("#anParams input[data-k=phi_eq]", "1.4"); pg.click("#bAnRun")
    pg.wait_for_function("() => LG.results.length >= 3 && LG.results[0].res.tool === 'lambda'", timeout=20000)
    lam = pg.evaluate("LG.results[0].res.result")
    check("λ 적합 결과 (시행 2)", lam and 3 < lam["lam"] < 8, str(lam and lam["lam"]))
    check("λ 오버레이 (적합선·밴드·통과시각)", pg.evaluate("LG.chartOverlay.length") >= 5)
    time.sleep(0.4)
    pg.screenshot(path=os.path.join(OUT, "s4_lambda.png"))
    # φ_eq 훑기
    pg.select_option("#anTool", "phi_eq"); pg.click("#bAnRun")
    pg.wait_for_function("() => LG.results.length >= 4 && LG.results[0].res.tool === 'phi_eq'", timeout=60000)
    pe = pg.evaluate("LG.results[0].res.result")
    check("φ_eq 훑기 결과 + 곡선", pe and pe["phi_eq_best"] is not None and pg.evaluate("LG.results[0].res.curves.length") == 1, str(pe and pe["phi_eq_best"]))
    pg.screenshot(path=os.path.join(OUT, "s5_phieq.png"))
    # 다음 놓기 추천 → 목표점 자동 입력
    pg.select_option("#anTool", "recommend"); pg.click("#bAnRun")
    pg.wait_for_function("() => LG.results.length >= 5 && LG.results[0].res.tool === 'recommend'", timeout=60000)
    rc = pg.evaluate("LG.results[0].res")
    check("다음 놓기 추천 결과 (next·추정선·목표점)", rc["ok"] and rc["next"] and pg.evaluate("LG.el('cTgt').checked") and abs(float(pg.input_value("#iTgtB")) - rc["next"]["beta"]) < 0.01, str(rc.get("result")))
    check("추천 배너 렌더", "다음 놓기" in (pg.text_content("#anOut") or ""))
    # 스크럽·재생
    pg.evaluate("LG.setCursor(3000)"); time.sleep(0.2)
    check("스크럽 → 커서·3D 자세", pg.evaluate("LG.cur()") == 3000 and "t = " in pg.text_content("#ov3d"))
    pg.keyboard.press("Space"); time.sleep(0.8)
    pg.keyboard.press("Space"); time.sleep(0.1); c1 = pg.evaluate("LG.cur()")
    check("Space 재생 → 커서 진행", c1 > 3000, str(c1))
    pg.keyboard.press("ArrowRight"); time.sleep(0.1)
    check("→ 한 칸", pg.evaluate("LG.cur()") == c1 + 1, f"{pg.evaluate('LG.cur()')} vs {c1+1}")
    # 선택 구간 (Shift+드래그)
    box = pg.locator("#chart").bounding_box()
    pg.mouse.move(box["x"] + box["width"] * 0.3, box["y"] + box["height"] * 0.5); pg.keyboard.down("Shift"); pg.mouse.down()
    pg.mouse.move(box["x"] + box["width"] * 0.6, box["y"] + box["height"] * 0.5, steps=5); pg.mouse.up(); pg.keyboard.up("Shift"); time.sleep(0.2)
    sel = pg.evaluate("LG.sel")
    check("Shift+드래그 구간 선택 → 분석 입력칸", sel and sel["t1"] > sel["t0"] and pg.input_value("#anT0") != "", str(sel))
    pg.select_option("#anTool", "stats"); pg.click("#bAnRun")
    pg.wait_for_function("() => LG.results[0].res.tool === 'stats'", timeout=20000)
    check("구간 통계 (선택 구간)", pg.evaluate("LG.results[0].res.ok"))
    # 파이프라인
    pg.click(".tabs button[data-tab=pipe]")
    b_before = pg.evaluate("LG.val('a_beta', 500)")
    pg.fill("#pipeForm input[data-k=p2r]", "0.6"); pg.click("#bPipeApply")
    pg.wait_for_function("() => LG.PIPE && Math.abs(LG.PIPE.p2r - 0.6) < 1e-9", timeout=10000); time.sleep(0.6)
    check("파이프라인 적용 → 재전송·재계산", pg.evaluate("LG.PIPE.p2r") == 0.6 and pg.evaluate("LG.ds.n") == 9087)
    pg.fill("#pipeForm input[data-k=p2r]", "0.4285"); pg.click("#bPipeApply"); time.sleep(0.6)
    # 숨김·재연결
    pg.keyboard.press("h"); time.sleep(0.2)
    check("H 숨김", pg.evaluate("document.body.classList.contains('hide')"))
    pg.screenshot(path=os.path.join(OUT, "s6_hidden.png"))
    pg.keyboard.press("h")
    pg.click("#bFakeSynth"); pg.wait_for_function("() => LG.link.connected && LG.ds.n > 20 && LG.ds.n < 2000", timeout=10000)
    check("재연결 → 파일 버퍼 비우고 라이브", pg.evaluate("LG.ds.source") == "" or pg.evaluate("LG.ds.n") < 2000)
    pg.click("#bFollow"); time.sleep(0.3)
    check("따라가기 복귀 LIVE", pg.text_content("#tMode").strip() == "LIVE")
    # MuJoCo 가상 로봇 (리허설 소스)
    pg.click("#bFakeMj")
    try:
        pg.wait_for_function("() => LG.link.connected && LG.link.src.kind === 'mujoco' && LG.ds.n > 50", timeout=25000)
        check("가짜: MuJoCo 연결 칩", "MuJoCo" in pg.text_content("#hConn"), pg.text_content("#hConn"))
        pg.evaluate("LG.send({cmd:'robot', text:'release 1.0 0.0 0.8'})"); time.sleep(3.5)
        pg.select_option("#anTool", "trials"); pg.click("#bAnAll"); pg.click("#bAnRun")
        pg.wait_for_function("() => LG.results.length >= 1 && LG.results[0].res.tool === 'trials'", timeout=30000)
        check("가상 로봇 놓기 → 시행 감지", pg.evaluate("LG.results[0].res.result.n_dir_valid") >= 1, str(pg.evaluate("LG.results[0].res.result")))
        pg.screenshot(path=os.path.join(OUT, "s7_mujoco.png"))
        # ---- 측정실 /lab · 무대 /show · 조종석 /deck (같은 서버, 같은 데이터) ----
        ctx = pg.context
        lab = ctx.new_page(); lab.on("pageerror", lambda e: ERRS.append("lab pageerror: " + str(e)))
        lab.goto(f"http://localhost:{PORT}/lab"); lab.wait_for_function("() => window.LG && LG.PL && LG.ds.n > 50", timeout=20000); time.sleep(0.8)
        check("/lab 로드 · 원본 표 행", lab.locator("#tbody .row").count() >= 10, str(lab.locator("#tbody .row").count()))
        bb = lab.locator("#tc").bounding_box(); y = bb["y"] + bb["height"] * 0.5
        lab.mouse.move(bb["x"] + bb["width"] * 0.4, y); lab.mouse.down(); lab.mouse.move(bb["x"] + bb["width"] * 0.7, y, steps=6); lab.mouse.up(); time.sleep(0.3)
        check("/lab 차트 드래그 = 구간 선택", lab.evaluate("LG.sel && LG.sel.t1 > LG.sel.t0"))
        lab.select_option("#tcFit", "lin"); lab.click("#bTcFit"); time.sleep(0.3)
        check("/lab 선형 추세선 (식·R²)", "R²" in (lab.text_content("#fits") or ""))
        lab.select_option("#xyFit", "rec"); lab.click("#bXyFit"); time.sleep(4.0)
        check("/lab 놓기점 → 다음 놓기 추천", "다음 놓기" in (lab.text_content("#xyFits") or "") or "r̂" in (lab.text_content("#xyFits") or ""), (lab.text_content("#xyFits") or "")[:80])
        lab.screenshot(path=os.path.join(OUT, "s8_lab.png"))
        show = ctx.new_page(); show.set_viewport_size({"width": 608, "height": 1080}); show.on("pageerror", lambda e: ERRS.append("show pageerror: " + str(e)))
        show.goto(f"http://localhost:{PORT}/show"); show.wait_for_function("() => window.LG && LG.PL", timeout=20000)
        deck = ctx.new_page(); deck.on("pageerror", lambda e: ERRS.append("deck pageerror: " + str(e)))
        deck.goto(f"http://localhost:{PORT}/deck"); deck.wait_for_function("() => window.LG && LG.PL && LG.ds.n > 20", timeout=20000); time.sleep(0.5)
        check("/deck 큐 8개", deck.locator(".cue").count() == 8, str(deck.locator(".cue").count()))
        deck.keyboard.press("4"); time.sleep(1.5)
        check("조종석 4 → 무대 twin+pl1", show.evaluate("() => document.querySelector('.view.on').id") == "vTwin" and show.evaluate("() => document.getElementById('pl1').classList.contains('on')"))
        deck.keyboard.press("6"); time.sleep(1.0)
        check("조종석 6 → 무대 ε* 기하", show.evaluate("() => LG.PLOPT.epsGeom === true"))
        deck.keyboard.press("2"); time.sleep(2.5)
        vs = show.evaluate("() => { const v = document.getElementById('vid'); return {view: document.querySelector('.view.on').id, ready: v.readyState, paused: v.paused, t: v.currentTime}; }")
        check("조종석 2 → 무대 실사 반복 재생", vs["view"] == "vVideo" and vs["ready"] >= 2 and not vs["paused"], str(vs))
        show.screenshot(path=os.path.join(OUT, "s9_show.png")); deck.screenshot(path=os.path.join(OUT, "s10_deck.png"))
        check("무대 칩 (조종석)", "무대" in (deck.text_content("#hStage") or ""))
        lab.close(); show.close(); deck.close()
    except Exception as ex:
        check("가짜: MuJoCo / lab / show / deck (mujoco 없으면 실패해도 됨)", False, str(ex)[:160])
    pg.click("#bDisc")
    br.close()

print("\nJS 오류:", len(ERRS)); [print("   ", e[:200]) for e in ERRS[:10]]
if ERRS:
    FAILS.append("JS 오류 %d" % len(ERRS))
print("ALL PASS" if not FAILS else "FAILED: " + ", ".join(FAILS))
sys.exit(1 if FAILS else 0)
