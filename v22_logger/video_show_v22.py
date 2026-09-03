# -*- coding: utf-8 -*-
"""
video_show_v22.py — 무대(/show, 세로) 리허설 녹화: 조종석 큐 1~8 을 순서대로 넘기며 무대 창을 녹화한다.
로봇은 MuJoCo 가상(사람 동작은 'sim …' 지시), 조종석은 Playwright 가 실제 키를 누른다.

    python video_show_v22.py                # 서버(--mujoco, 8235)를 띄우고 녹화 → video_out/v22_stage.mp4 (+ _small)
    python video_show_v22.py --no-server --port 8235
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "video_out")
W, H = 1080, 1920           # 세로 모니터 그대로


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8235)
    ap.add_argument("--no-server", action="store_true")
    ap.add_argument("--name", default="v22_stage")
    ap.add_argument("--scale", type=float, default=0.5, help="녹화 해상도 배율 (0.5 → 540×960)")
    a = ap.parse_args(argv)
    os.makedirs(OUT, exist_ok=True)
    srv = None
    if not a.no_server:
        srv = subprocess.Popen([sys.executable, os.path.join(HERE, "server.py"), "--mujoco", "--port", str(a.port), "--no-autorec"],
                               cwd=HERE, stdout=open(os.path.join(OUT, "server_stage.log"), "w"), stderr=subprocess.STDOUT)
        time.sleep(5.0)
    from playwright.sync_api import sync_playwright
    vw, vh = int(W * a.scale), int(H * a.scale)
    webm = None
    log = lambda m: print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)
    try:
        with sync_playwright() as p:
            br = p.chromium.launch(executable_path="/opt/pw-browsers/chromium",
                                   args=["--use-gl=swiftshader", "--enable-unsafe-swiftshader", "--autoplay-policy=no-user-gesture-required", "--lang=ko-KR"])
            ctx = br.new_context(viewport={"width": vw, "height": vh}, record_video_dir=OUT, record_video_size={"width": vw, "height": vh}, locale="ko-KR")
            show = ctx.new_page()
            show.goto(f"http://localhost:{a.port}/show"); show.wait_for_function("() => window.LG && LG.PL", timeout=20000)
            deck = ctx.new_page(); deck.set_viewport_size({"width": 1200, "height": 800})
            deck.goto(f"http://localhost:{a.port}/deck"); deck.wait_for_function("() => window.LG && LG.PL && LG.ds.n > 20", timeout=20000)
            show.bring_to_front()
            robot = lambda t: deck.evaluate(f"() => LG.send({{cmd:'robot', text:{t!r}}})")
            cmd = lambda t: deck.evaluate(f"() => LG.send({{cmd:'send', text:{t!r}}})")
            cue = lambda k: (deck.keyboard.press(str(k)), log(f"cue {k}"))
            stage = lambda: deck.evaluate("() => LG.link.src && LG.link.src.stage")
            def wait_stage(want, tmo=8.0):
                t0 = time.time()
                while time.time() - t0 < tmo:
                    if stage() == want: return True
                    time.sleep(0.1)
                return False
            time.sleep(1.0)
            # 1 문제: 실사 재생 (6.2 s) → 정지
            cue(1); time.sleep(7.5)
            # 2 발상: 휘청 반복 + 오버랩
            cue(2); time.sleep(9.0)
            # 3 수단: 트윈, 접기 시연 (손에 잡힌 채 δ 20 → 0)
            cmd("k"); robot("hold 0.0 0.0"); time.sleep(1.0)
            cue(3); time.sleep(1.5); cmd("20"); time.sleep(1.6); cmd("0"); time.sleep(1.4); cmd("-20"); time.sleep(1.6); cmd("0"); time.sleep(1.5)
            # 4 균형의 재정의: 트윈 + 원위치 평면 — 선 위에 놓기 (r·c0 = 모델값 근처)
            cue(4); time.sleep(1.0)
            cmd("mode 0"); robot("release 1.0 -1.65 1.2"); wait_stage("free"); time.sleep(3.0); robot("catch"); time.sleep(0.8)
            robot("release -1.0 1.65 1.2"); wait_stage("free"); time.sleep(3.0); robot("catch"); time.sleep(0.8)
            # 5 모드: 평면 크게 — 선 밖에서 놓아 발산
            cue(5); time.sleep(1.0); robot("release 1.0 0.0 1.2"); wait_stage("free"); wait_stage("held", 6.0); time.sleep(1.0)
            robot("release -1.0 0.0 1.2"); wait_stage("free"); wait_stage("held", 6.0); time.sleep(1.0)
            # 6 클라이막스: ε* 기하 — 손으로 기울인다
            cue(6); robot("hold 0.0 0.0"); time.sleep(1.5)
            for b, f in [(1.5, 0.5), (2.5, 1.0), (1.0, -0.5), (-1.5, 0.5), (-2.5, -1.0), (0.5, 1.5)]:
                robot(f"hold {b} {f}"); time.sleep(1.6)
            time.sleep(1.0)
            # 7 한계와 답: mode 2 증분접기 — 트윈 + 예측점 평면
            cmd("mode 2"); cmd("gam 9.1"); cmd("r -1.65"); cmd("lam 5.2"); cmd("c0 0.1"); robot("hold 0.3 0.0"); time.sleep(2.0)
            cue(7); time.sleep(1.0); cmd("g"); robot("free"); time.sleep(12.0)
            # 8 방점: 트윈 크게 + HUD
            cue(8); time.sleep(10.0)
            # 질의응답 맛보기: 실사 → 모델 변신, ε* 카드
            deck.locator("#qaBtns .btn", has_text="실사 → 모델 변신").click(); time.sleep(7.5)
            deck.locator("#qaBtns .btn", has_text="ε* 카드").click(); time.sleep(4.0)
            deck.locator("#qaBtns .btn", has_text="트윈 크게").click(); time.sleep(4.0)
            cmd("h")
            show.bring_to_front(); time.sleep(0.5)
            webm = show.video.path()
            ctx.close(); br.close()
    finally:
        if srv: srv.terminate()
    if not webm or not os.path.exists(webm):
        print("녹화 파일 없음"); return 1
    dst = os.path.join(OUT, a.name + ".webm"); shutil.move(webm, dst)
    # 조종석 페이지의 녹화(같은 컨텍스트)도 생기면 지운다
    for f in os.listdir(OUT):
        if f.endswith(".webm") and f.startswith("page@"): os.remove(os.path.join(OUT, f))
    try:
        import imageio_ffmpeg; ff = imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        ff = shutil.which("ffmpeg")
    if ff:
        for suffix, crf in (("", 22), ("_small", 29)):
            mp4 = os.path.join(OUT, a.name + suffix + ".mp4")
            subprocess.call([ff, "-y", "-loglevel", "error", "-i", dst, "-c:v", "libx264", "-preset", "medium", "-crf", str(crf), "-pix_fmt", "yuv420p", "-r", "25", "-movflags", "+faststart", mp4])
            if os.path.exists(mp4): print("mp4:", mp4, f"{os.path.getsize(mp4) / 1e6:.1f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
