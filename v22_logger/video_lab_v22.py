# -*- coding: utf-8 -*-
"""
video_lab_v22.py — 측정실(/lab) 리허설 영상: 로거 앱 기능 전부 + 엑셀식 차트·추세선으로 λ · r · γ 를 재고 증분접기까지.
video_v22.py 의 감독(Director: 실제 마우스·자막·사람 카드)을 그대로 쓰고, 화면만 /lab 이다.
엑셀 방식 추가분: 시간 차트 구간 드래그 → 지수 추세선(λ) · XY 차트 「다음 놓기 추천」(놓기점·경계선·다음 점) · 「복사 (엑셀)」.

    python video_lab_v22.py                 # 서버(--mujoco, 8237) 띄우고 녹화 → video_out/v22_lab_rehearsal.mp4 (+ _small)
    python video_lab_v22.py --no-server --port 8237
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

from video_v22 import Director, W, H, OUT, HERE, fmt


class LabDirector(Director):
    def tab(self, name):            # 측정실은 탭이 없다 — 전부 펼쳐져 있다
        pass

    # ---------- 엑셀식 차트 ----------
    def tc_x(self, t):
        bb = self.box("#tc")
        t0, t1 = self.ev("() => LG.chartRange()")
        return bb["x"] + 52 + (t - t0) / (t1 - t0) * (bb["width"] - 64), bb["y"] + bb["height"] * 0.5

    def drag_tc(self, ta, tb):
        """시간 차트에서 [ta, tb] 를 왼쪽 버튼으로 드래그 (= 엑셀에서 셀 범위 고르기). 선택 = LG.sel = 분석 구간.
        LIVE 로 흐르는 중이면 먼저 창을 멈춘다(드래그가 하는 일과 같다) — 좌표를 잰 뒤 차트가 흘러가 버리지 않게."""
        self.ev("() => { if (LG.follow) { const [a, b] = LG.chartRange(); LG.follow = false; LG.playing = false; LG.chart.view = { t0: a, t1: b }; LG.emit('follow'); } }")
        time.sleep(0.2)
        x0, y = self.tc_x(ta); x1, _ = self.tc_x(tb)
        self.move_to(x0, y); time.sleep(0.15)
        self.pg.mouse.down()
        n = 22
        for i in range(1, n + 1):
            self.pg.mouse.move(x0 + (x1 - x0) * i / n, y); time.sleep(0.03)
        self.pg.mouse.up(); self.pos = (x1, y)
        time.sleep(0.3)
        self.log(f"DRAG tc {ta:.2f}–{tb:.2f} → sel {self.ev('() => LG.sel && [LG.sel.t0.toFixed(2), LG.sel.t1.toFixed(2)]')}")

    def trend(self, kind, col, y0=None):
        self.select("#tcFit", kind)
        self.select("#tcCol", col)
        if y0 is not None:
            self.type_into("#tcY0", y0)
        n0 = self.ev("() => LG.lab.S.tcFits.length")
        self.click("#bTcFit")
        if kind == "osc":
            self.pg.wait_for_function("() => !LG.lab.S.tcFits.some(f => /중…/.test(f.eq))", timeout=30000)
        f = self.ev(f"() => LG.lab.S.tcFits.length > {n0} ? LG.lab.S.tcFits[LG.lab.S.tcFits.length - 1] : null")
        self.log("TREND " + (f["eq"] if f else "실패"))
        return f

    def xy(self, kind):
        self.select("#xyFit", kind)
        self.click("#bXyFit")
        self.pg.wait_for_function("() => !LG.lab.S.xyFits.some(f => /중…/.test(f.eq))", timeout=60000)
        f = self.ev("() => LG.lab.S.xyFits[LG.lab.S.xyFits.length - 1]")
        self.log("XY " + (f["eq"] if f else "실패"))
        return f

    def exp_window(self, t0, t1, lo=2.0, hi=9.0):
        """놓기 t0 뒤 |φ−φ_eq| 가 lo 를 넘는 순간부터 hi 에 닿기까지 — 지수 추세선을 그을 구간"""
        return self.ev(f"""() => {{ const peq = LG.PIPE ? LG.PIPE.phi_eq : 0, t = LG.ds.data.t, a = LG.col('u_phi'), n = LG.ds.n;
            let ta = null, tb = null; for (let i = LG.idxOfT({t0}); i <= LG.idxOfT({t1}) && i < n; i++) {{ const v = Math.abs(a[i] - peq);
            if (ta == null && v >= {lo}) ta = t[i]; if (ta != null && v >= {hi}) {{ tb = t[i]; break; }} }} return [ta ?? {t0} + 0.2, tb ?? {t1}]; }}""")


def scenario(D):
    pg = D.pg
    D.title("v22 측정실 — 실기 측정 리허설 (엑셀 방식)",
            ["로거 앱의 기능은 전부 그대로 + 원본 표 · 시간 차트 · XY 차트 · 추세선",
             "· 로봇 = MuJoCo 가상 로봇 (v21 실측 프리셋)  ·  앱 조작 = 실제 마우스",
             "· 🤚 주황 카드 = 사람이 로봇에 하는 동작  ·  파란 자막 = 앱에서 보는 것"], hold=5.0)

    # ================= 1. 연결 =================
    D.step("1 / 4  연결 · 영점")
    D.cap("측정실: 왼쪽 트윈·평면, 가운데 엑셀식 시간 차트·XY 차트, 오른쪽 명령 팔레트·측정 도구·기록·파이프라인·콘솔, 아래 원본 표·스트립 차트·결과", hold=2.5)
    D.cap("실기에서는 포트를 고르고 「연결」. 오늘은 로봇 대신 「가짜: MuJoCo」", hold=1.0)
    D.click("#bFakeMj")
    pg.wait_for_function("() => LG.ds.n > 50", timeout=20000)
    D.wait(1.5)
    D.cap("연결 직후 D행이 100 Hz 로 들어온다: 원본 표가 자라고, 시간 차트가 흐르고, 트윈·평면이 산다. 기록은 자동 시작(logs/ 4파일)", hold=3.5)
    D.cap("영점: 매달아 멎게 한 뒤 「z 영점」, 반대쪽에서 한 번 더 (2단 매달림 영점)")
    D.human("로봇을 매단 채 한쪽으로 정착시켜 멎게 한다"); D.wait(1.2); D.palette("z 영점"); D.wait(0.8)
    D.human("반대쪽으로 살짝 밀어 정착시킨다"); D.wait(1.2); D.palette("z 영점"); D.wait(0.6); D.human_off()
    D.palette("t 상태")
    D.cap("「t 상태」 — 모드 · w 벡터 · γ · 영점 단계가 콘솔에 나온다", hold=2.0)

    # ================= 2. λ =================
    D.step("2 / 4  λ — 놓기 3회 · 지수 추세선")
    D.cap("λ: 「mode 0 측정」. β = +1°, φ = 0° 에서 1 초 멈췄다 놓는다 → 발산 → 9° 쯤에서 잡는다", hold=1.0)
    D.palette("mode 0 측정")
    D.release(1.0, 0.0)
    D.cap("엑셀에서 하던 그대로: 시간 차트에서 발산 구간을 드래그하고 「지수 추세선」 — ln|φ| 직선의 기울기가 λ", hold=1.0)
    res = D.run_tool("trials")
    tb = [r for r in res.get("table", []) if r.get("dir_valid")]
    r1 = tb[-1] if tb else {}
    if r1:
        D.select("#anTrial", str(r1["k"])); D.wait(0.6)
        ta, tb2 = D.exp_window(r1["t0"], r1["t1"])
        D.drag_tc(ta, tb2)
        f = D.trend("exp", "u_phi", y0=f"{(D.ev('() => LG.PIPE ? LG.PIPE.phi_eq : 0') or 0):.2f}")
        lam_x = f["res"]["lam"] if f else None
        D.cap(f"추세선: λ = {fmt(lam_x, 3)} /s (R² {fmt(f['res']['r2'] if f else None, 4)}, |φ| 2~9° 구간) — 도구 「시행 나누기」의 λ = {fmt(r1.get('lam'))}",
              "점선이 추세선, 식은 차트 위와 아래 목록에. 같은 구간이 곧 측정 도구의 분석 구간이다", hold=4.0)
        D.run_tool("lambda", whole=False)
        D.cap("같은 구간에 「λ — ln|ψ| 직선적합」 도구: 쓴 밴드(2~9°) · 적합선 · 통과 시각이 엑셀 차트와 스트립 차트에 같이 겹친다", hold=3.5)
        D.click("#bFollow")
    D.cap("두 번 더 — 반대쪽(β = −1°)과 다시 +1°", hold=0.8)
    D.release(-1.0, 0.0)
    D.release(1.0, 0.0)
    res = D.run_tool("trials")
    rr = res.get("result", {})
    D.cap(f"시행 {rr.get('n_dir_valid')}개: λ₊ = {fmt(rr.get('lam_plus'))}, λ₋ = {fmt(rr.get('lam_minus'))} (손으로 옮긴 동작은 자동 제외)",
          "두 방향이 20 % 넘게 갈리면 φ_eq 를 의심", hold=3.0)
    lam_vals = [x for x in (rr.get("lam_plus"), rr.get("lam_minus")) if x]
    lam_hat = sum(lam_vals) / len(lam_vals) if lam_vals else 5.4
    D.cap(f"λ̂ = {lam_hat:.2f} → 파이프라인 lam 과 펌웨어 lam", hold=0.8)
    D.type_into("#pipeForm [data-k=lam]", f"{lam_hat:.2f}"); D.click("#bPipeApply"); D.wait(0.5)
    D.param_set("lam", f"{lam_hat:.2f}"); D.wait(0.8)

    # ================= 3. r =================
    D.step("3 / 4  r — 다음 놓기 추천 루프 · XY 차트")
    D.cap("r: 「★ 다음 놓기 추천」. 놓을 때마다 선까지 거리 s 를 재고 다음 놓기점을 앱이 정한다. 「추천→목표」로 평면에 십자선", hold=1.0)
    res = D.run_tool("recommend")
    nx = res.get("next") or {}
    for i in range(6):
        rr = res.get("result", {})
        D.cap(f"추천 #{i + 1}: β = {nx.get('beta', 0):+.2f}°, φ = {nx.get('phi', 0):+.2f}°  ({nx.get('reason', '')})",
              f"r̂ = {fmt(rr.get('r'), 3)} ± {fmt(rr.get('se_r'), 3)} · ĉ₀ = {fmt(rr.get('c0'), 2)} · ω̂ = {fmt(rr.get('om_hat'))} · n = {rr.get('n')}", hold=1.0)
        D.release(nx.get("beta", 0.0), nx.get("phi", 0.7), sub="십자선 자리에서 놓는다. 넘어지는 쪽만 보면 된다")
        if i == 2:
            D.cap("XY 차트에 「다음 놓기 추천」: 놓기점(분홍 +낙하 · 파랑 −낙하), 추정 경계선, 다음 놓기 표적이 엑셀 산점도처럼 그려진다", hold=0.8)
            f = D.xy("rec")
            D.cap(f["eq"] if f else "—", hold=3.0)
        res = D.run_tool("recommend")
        nx = res.get("next") or {}
    rr = res.get("result", {})
    r_hat = rr.get("r") or -1.5
    c0_hat = rr.get("c0") or 0.0
    f = D.xy("rec")
    D.cap(f"놓기 {rr.get('n')}회 → r̂ = {fmt(rr.get('r'), 3)} ± {fmt(rr.get('se_r'), 3)}, ĉ₀ = {fmt(rr.get('c0'), 2)}"
          + ("  — 충분 (SE_r < 0.05)" if rr.get("enough") else "  — 실기에서는 SE_r < 0.05 까지 계속"),
          "XY 차트: 점 = 놓기점(색 = 넘어진 쪽), 점선 = 추정한 안정모드선 φ = r̂·β + ĉ₀, 십자 = 다음 놓기", hold=4.0)
    D.cap(f"r̂ = {r_hat:.3f}, ĉ₀ = {c0_hat:.2f} 을 파이프라인(r, c0)과 펌웨어(r, c0)에 넣는다", hold=0.8)
    D.type_into("#pipeForm [data-k=r]", f"{r_hat:.3f}"); D.type_into("#pipeForm [data-k=c0]", f"{c0_hat:.2f}"); D.click("#bPipeApply"); D.wait(0.5)
    D.param_set("r", f"{r_hat:.3f}"); D.param_set("c0", f"{c0_hat:.2f}"); D.wait(0.8)

    # ================= 4. γ =================
    D.step("4 / 4  γ — 단일접기 성적표 → 증분접기")
    D.cap("γ: 「mode 1 단일접기」. 「g 시작」으로 무장 → 첫 |Â| > trig 에서 한 번 접고 δ 고정 → 자유 발산 → 잡기", hold=1.0)
    D.palette("mode 1 단일접기")
    for i, (b, f_) in enumerate([(0.3, 0.0), (-0.3, 0.0), (0.35, 0.0)]):
        D.live()
        D.human(f"로봇을 β = {b:+.2f}°, φ = {f_:+.2f}° 로 잡는다 (|Â| < trig 0.6°)", "아직 놓지 않는다")
        D.robot(f"hold {b} {f_}"); D.wait(1.6)
        D.cap(f"접기 시행 {i + 1}: 「g 시작」 직후 놓는다 — 시간 차트에 접기(분홍 띠)와 발산(주황 띠)이 표시된다", hold=0.8)
        D.human("「g」 직후 놓는다", "접힌 뒤에도 δ 고정 — 발산하게 두고 9° 에서 잡는다")
        D.palette("g 시작", settle=0.05); D.robot("free")
        D.wait_stage("free", 5.0); D.wait_stage("held", 8.0); D.wait(0.5)
        D.human_off()
        D.palette("h 정지"); D.type_into("#iDelta", "0"); D.click("#bDeltaGo"); D.wait(0.6)
    res = D.run_tool("fold")
    rr = res.get("result", {})
    D.cap(f"「접기 성적표」: A⁺ = G·A⁻ − g·Δδ,  G = {fmt(rr.get('G'))}, g = {fmt(rr.get('g_mean'))} → γ* = G/g = {fmt(rr.get('gamma_star'))}  (시행 {rr.get('n_valid')}회)",
          "시행별 접기 직전 Â · Δδ · 관측창 뒤 Â 가 표와 차트 오버레이로. 「펌웨어로 보내기」 = gam 명령", hold=4.5)
    if res.get("next_cmd"):
        D.click("#bSendNext"); D.wait(0.8)
    D.cap("「mode 2 증분접기」 — 잰 λ · r · γ 만 넣은 본 제어", hold=0.8)
    D.palette("mode 2 증분접기")
    D.live()
    D.human("로봇을 β = +0.3°, φ = 0° 로 잡는다", "「g」 직후 바로 놓는다")
    D.robot("hold 0.3 0.0"); D.wait(1.8)
    D.human("「g」 — 놓는다")
    D.palette("g 시작", settle=0.05); D.robot("free")
    D.wait(1.0); D.human_off()
    D.cap("증분접기가 잡는다: 트윈이 접었다 펴고, hold 가 계단처럼, φ 는 유계 — 시간 차트와 원본 표가 같은 순간을 보여준다", hold=6.0)
    D.cap("예측점 평면: 점이 A = 0 선 근처를 오간다. 원위치 평면: 안정모드선 둘레를 돈다", hold=5.0)
    st = D.stage()
    D.palette("h 정지")
    # 엑셀로 내보내기
    t1 = D.ev("() => LG.tOf(LG.ds.n - 1)")
    D.drag_tc(t1 - 8.0, t1 - 1.0)
    D.click("#bCopy")
    D.cap("「복사 (엑셀)」: 드래그한 구간의 원본 표가 탭 구분으로 클립보드에 — 엑셀에 붙이면 그대로 표. 기록 파일(logs/ 5종)도 그대로 남는다",
          f"(가상 로봇 상태: {'자유 — 균형 유지 중' if st == 'free' else st})", kind="note", hold=4.0)
    D.click("#bFollow")
    D.title("끝 — 실기에서 같은 순서로",
            ["1  연결 · 2단 영점 · P2R 0.4285 유지", "2  λ: mode 0 · ±1° 놓기 3회 · 차트 드래그 → 지수 추세선 / 시행 나누기 · lam 넣기",
             "3  r: 다음 놓기 추천 루프 · XY 차트로 확인 · SE_r < 0.05 · r 넣기", "4  γ: mode 1 · g → 놓기 3회 · 접기 성적표 · gam → mode 2",
             "· docs/v22_발표_무대_운용_20260903.md"], hold=6.0)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8237)
    ap.add_argument("--no-server", action="store_true")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--name", default="v22_lab_rehearsal")
    a = ap.parse_args(argv)
    os.makedirs(OUT, exist_ok=True)
    srv = None
    if not a.no_server:
        srv = subprocess.Popen([sys.executable, os.path.join(HERE, "server.py"), "--port", str(a.port)], cwd=HERE,
                               env=dict(os.environ, PYTHONUNBUFFERED="1"), stdout=open(os.path.join(OUT, "server_lab.log"), "w"), stderr=subprocess.STDOUT)
        time.sleep(4.0)
    from playwright.sync_api import sync_playwright
    webm = None
    try:
        with sync_playwright() as p:
            br = p.chromium.launch(executable_path="/opt/pw-browsers/chromium", args=["--use-gl=swiftshader", "--enable-unsafe-swiftshader", "--lang=ko-KR"])
            ctx = br.new_context(viewport={"width": W, "height": H}, record_video_dir=OUT, record_video_size={"width": W, "height": H}, locale="ko-KR")
            pg = ctx.new_page(); errs = []
            pg.on("pageerror", lambda e: errs.append(str(e)))
            pg.goto(f"http://localhost:{a.port}/lab"); pg.wait_for_function("() => window.LG && LG.PL && LG.lab", timeout=20000)
            time.sleep(1.0)
            D = LabDirector(pg, quick=a.quick)
            try:
                scenario(D)
            except Exception as ex:
                D.log(f"!! 시나리오 오류: {type(ex).__name__}: {ex}"); pg.screenshot(path=os.path.join(OUT, "error_lab.png")); raise
            finally:
                if errs: print("페이지 오류:", errs[:5])
                pg.evaluate("() => LG.send({cmd:'disconnect'})"); time.sleep(0.5)
                webm = pg.video.path(); ctx.close(); br.close()
    finally:
        if srv: srv.terminate()
    if not webm or not os.path.exists(webm):
        print("녹화 파일이 없다"); return 1
    dst = os.path.join(OUT, a.name + ".webm"); shutil.move(webm, dst)
    print("webm:", dst, f"{os.path.getsize(dst) / 1e6:.1f} MB")
    try:
        import imageio_ffmpeg; ff = imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        ff = shutil.which("ffmpeg")
    if ff:
        for suffix, crf, preset, vf in (("", 22, "medium", None), ("_small", 30, "slow", "scale=1280:-2")):
            mp4 = os.path.join(OUT, a.name + suffix + ".mp4")
            rc = subprocess.call([ff, "-y", "-loglevel", "error", "-i", dst, "-c:v", "libx264", "-preset", preset, "-crf", str(crf)] + (["-vf", vf] if vf else []) + ["-pix_fmt", "yuv420p", "-r", "25", "-movflags", "+faststart", mp4])
            if rc == 0 and os.path.exists(mp4): print("mp4:", mp4, f"{os.path.getsize(mp4) / 1e6:.1f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
