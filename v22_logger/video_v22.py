# -*- coding: utf-8 -*-
"""
video_v22.py — v22 측정 리허설 영상 (앱 화면 캡처 + 실제 마우스 조작 + 자막 + MuJoCo 가상 로봇)
================================================================================================
실기로 곧 할 실험(λ 3회 → r 추천 루프 → γ 단일접기 3회 + 성적표 → 증분접기)을, 로봇만 MuJoCo 로 바꿔
앱을 진짜로 조작하며 녹화한다. 사람이 하는 동작(잡기·옮기기·놓기)은 화면 위 카드로 설명하고,
가상 로봇에는 'sim release β φ' 지시로 같은 동작을 시킨다. 마우스는 실제로 움직여 클릭한다.

    python video_v22.py                 # 서버(--mujoco, 포트 8233)를 스스로 띄우고 녹화 → video_out/v22_rehearsal.mp4
    python video_v22.py --port 8233 --no-server   # 이미 떠 있는 서버 사용
    python video_v22.py --quick         # 대기 시간 짧게 (점검용)

산출: video_out/v22_rehearsal.webm (Playwright 원본) + .mp4 (imageio-ffmpeg 의 ffmpeg 로 H.264 변환)
"""
import argparse
import math
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "video_out")
W, H = 1600, 900

OVERLAY_JS = r"""
(() => {
  if (window.__ovl) return;
  const css = document.createElement('style');
  css.textContent = `
    #__cur{position:fixed;left:0;top:0;width:26px;height:26px;pointer-events:none;z-index:2147483647;transform:translate(-3px,-2px);filter:drop-shadow(0 1px 2px rgba(0,0,0,.7))}
    #__rip{position:fixed;width:34px;height:34px;border-radius:50%;border:3px solid #ffd166;pointer-events:none;z-index:2147483646;transform:translate(-50%,-50%) scale(.3);opacity:0}
    #__rip.go{animation:__rp .45s ease-out}
    @keyframes __rp{0%{transform:translate(-50%,-50%) scale(.3);opacity:1}100%{transform:translate(-50%,-50%) scale(1.4);opacity:0}}
    #__cap{position:fixed;left:50%;bottom:26px;transform:translateX(-50%);max-width:1180px;min-width:520px;padding:12px 20px;border-radius:12px;
      background:rgba(10,14,24,.92);color:#fff;font:600 19px/1.45 "Noto Sans KR","Malgun Gothic","WenQuanYi Zen Hei",sans-serif;border:2px solid #4cc9f0;
      box-shadow:0 6px 24px rgba(0,0,0,.6);pointer-events:none;z-index:2147483645;opacity:0;transition:opacity .25s}
    #__cap.on{opacity:1}  #__cap.note{border-color:#06d6a0}  #__cap .sub{display:block;font-weight:400;font-size:15px;color:#cfd8e3;margin-top:4px}
    #__hum{position:fixed;left:50%;top:62px;transform:translateX(-50%);max-width:980px;padding:12px 22px 12px 18px;border-radius:12px;
      background:rgba(60,28,0,.94);color:#fff;font:700 21px/1.4 "Noto Sans KR","Malgun Gothic","WenQuanYi Zen Hei",sans-serif;border:2px solid #ff9f1c;
      box-shadow:0 6px 24px rgba(0,0,0,.6);pointer-events:none;z-index:2147483645;opacity:0;transition:opacity .25s;display:flex;gap:14px;align-items:center}
    #__hum.on{opacity:1}  #__hum .ic{font-size:34px}  #__hum .sub{display:block;font-weight:400;font-size:15px;color:#ffe2b8;margin-top:3px}
    #__step{position:fixed;left:14px;top:50px;padding:6px 12px;border-radius:8px;background:rgba(10,14,24,.9);color:#ffd166;border:1px solid #ffd166;
      font:700 15px "Noto Sans KR","Malgun Gothic","WenQuanYi Zen Hei",sans-serif;pointer-events:none;z-index:2147483645;opacity:0;transition:opacity .25s}
    #__step.on{opacity:1}
    #__title{position:fixed;inset:0;background:rgba(6,9,16,.96);color:#fff;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:14px;
      font:400 22px/1.5 "Noto Sans KR","Malgun Gothic","WenQuanYi Zen Hei",sans-serif;pointer-events:none;z-index:2147483644;opacity:0;transition:opacity .4s}
    #__title.on{opacity:1}  #__title h1{font-size:40px;margin:0;color:#4cc9f0}  #__title .d{color:#9fb3c8;font-size:18px}
  `;
  document.head.appendChild(css);
  const cur = document.createElement('div'); cur.id = '__cur';
  cur.innerHTML = '<svg viewBox="0 0 24 24" width="26" height="26"><path d="M4 2 L4 19 L8.5 14.8 L11.6 21.5 L14.6 20.2 L11.6 13.6 L18 13.4 Z" fill="#fff" stroke="#111" stroke-width="1.4" stroke-linejoin="round"/></svg>';
  const rip = document.createElement('div'); rip.id = '__rip';
  const cap = document.createElement('div'); cap.id = '__cap';
  const hum = document.createElement('div'); hum.id = '__hum';
  const step = document.createElement('div'); step.id = '__step';
  const title = document.createElement('div'); title.id = '__title';
  for (const e of [title, cap, hum, step, rip, cur]) document.body.appendChild(e);
  document.addEventListener('mousemove', e => { cur.style.left = e.clientX + 'px'; cur.style.top = e.clientY + 'px'; }, true);
  document.addEventListener('mousedown', e => { rip.style.left = e.clientX + 'px'; rip.style.top = e.clientY + 'px'; rip.classList.remove('go'); void rip.offsetWidth; rip.classList.add('go'); }, true);
  window.__ovl = {
    cap(html, sub, kind) { cap.innerHTML = html + (sub ? `<span class="sub">${sub}</span>` : ''); cap.className = 'on' + (kind ? ' ' + kind : ''); },
    capOff() { cap.className = ''; },
    hum(html, sub) { hum.innerHTML = `<span class="ic">🤚</span><div>${html}${sub ? `<span class="sub">${sub}</span>` : ''}</div>`; hum.className = 'on'; },
    humOff() { hum.className = ''; },
    step(html) { step.textContent = html; step.className = 'on'; },
    title(h, lines) { title.innerHTML = `<h1>${h}</h1>` + lines.map(l => `<div class="${l.startsWith('·') ? 'd' : ''}">${l}</div>`).join(''); title.className = 'on'; },
    titleOff() { title.className = ''; },
  };
})();
"""


class Director:
    def __init__(self, page, quick=False):
        self.pg = page
        self.quick = quick
        self.pos = (W * 0.5, H * 0.55)
        self.t0 = time.time()
        page.evaluate(OVERLAY_JS)
        page.mouse.move(*self.pos)

    # ---------- 시간 ----------
    def wait(self, s):
        time.sleep(s * (0.35 if self.quick else 1.0))

    def log(self, msg):
        print(f"[{time.time() - self.t0:6.1f}s] {msg}", flush=True)

    # ---------- 자막 ----------
    def cap(self, text, sub="", kind="", hold=None):
        self.pg.evaluate("([a,b,c]) => window.__ovl.cap(a,b,c)", [text, sub, kind])
        self.log("CAP  " + text)
        if hold:
            self.wait(hold)

    def cap_off(self):
        self.pg.evaluate("() => window.__ovl.capOff()")

    def human(self, text, sub=""):
        self.pg.evaluate("([a,b]) => window.__ovl.hum(a,b)", [text, sub])
        self.log("HUM  " + text)

    def human_off(self):
        self.pg.evaluate("() => window.__ovl.humOff()")

    def step(self, text):
        self.pg.evaluate("(a) => window.__ovl.step(a)", text)

    def title(self, h, lines, hold=4.0):
        self.pg.evaluate("([h,l]) => window.__ovl.title(h,l)", [h, lines])
        self.wait(hold)
        self.pg.evaluate("() => window.__ovl.titleOff()")

    # ---------- 마우스 ----------
    def move_to(self, x, y, dur=0.55):
        x0, y0 = self.pos
        d = math.hypot(x - x0, y - y0)
        dur = max(0.25, min(dur, 0.25 + d / 1400.0))
        n = max(10, int(dur * 50))
        for i in range(1, n + 1):
            u = i / n
            s = 0.5 - 0.5 * math.cos(math.pi * u)
            self.pg.mouse.move(x0 + (x - x0) * s, y0 + (y - y0) * s)
            time.sleep(dur / n)
        self.pos = (x, y)

    def box(self, sel_or_loc):
        loc = self.pg.locator(sel_or_loc).first if isinstance(sel_or_loc, str) else sel_or_loc
        loc.scroll_into_view_if_needed()
        bb = loc.bounding_box()
        if bb is None:
            raise RuntimeError(f"보이지 않음: {sel_or_loc}")
        return bb

    def click(self, sel_or_loc, settle=0.35):
        bb = self.box(sel_or_loc)
        self.move_to(bb["x"] + bb["width"] / 2, bb["y"] + bb["height"] / 2)
        time.sleep(0.12)
        self.pg.mouse.down(); time.sleep(0.07); self.pg.mouse.up()
        time.sleep(settle)

    def palette(self, label, settle=0.35):
        """명령 팔레트 버튼 (명령 탭이 켜져 있어야 한다)"""
        self.click(self.pg.locator("#cmdGroups .btn", has_text=label).first, settle=settle)

    def tab(self, name):
        self.click(f".tabs button[data-tab={name}]", settle=0.3)

    def select(self, sel, value):
        bb = self.box(sel)
        self.move_to(bb["x"] + bb["width"] / 2, bb["y"] + bb["height"] / 2)
        time.sleep(0.15)
        self.pg.select_option(sel, value)
        time.sleep(0.3)

    def type_into(self, sel, text):
        self.click(sel, settle=0.1)
        self.pg.fill(sel, "")
        self.pg.keyboard.type(str(text), delay=55)
        time.sleep(0.25)

    def param_set(self, name, value):
        """명령 탭 파라미터 줄: 이름 고르고 값 치고 「설정」"""
        self.select("#paramSel", name)
        self.type_into("#paramVal", value)
        self.click("#bParamSet")

    # ---------- 앱 상태 ----------
    def ev(self, js):
        return self.pg.evaluate(js)

    def stage(self):
        return self.ev("() => (LG.link && LG.link.src && LG.link.src.stage) || ''")

    def wait_stage(self, want, timeout=12.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.stage() == want:
                return True
            time.sleep(0.1)
        return False

    def robot(self, text):
        self.ev(f"() => LG.send({{cmd:'robot', text:{text!r}}})")
        self.log("ROBOT sim " + text)

    def live(self):
        """REPLAY(시행 선택 뒤) 에 머물러 있으면 「따라가기」로 LIVE 복귀 — 트윈·평면이 실시간을 보여야 한다"""
        if (self.pg.text_content("#tMode") or "").strip() != "LIVE":
            self.click("#bFollow", settle=0.2)

    def release(self, beta, phi, hold_s=1.2, sub=""):
        """사람 카드 + 가상 로봇 놓기 + 잡힐 때까지 대기"""
        self.live()
        self.human(f"로봇을 β = {beta:+.2f}°, φ = {phi:+.2f}° 로 잡는다 → 1 초 멈춘다 → 놓는다",
                   sub or "|φ| 가 9° 쯤 되면 잡는다 (넘어뜨리지 않는다)")
        self.robot(f"release {beta} {phi} {hold_s}")
        self.wait_stage("free", timeout=8.0)
        self.wait_stage("held", timeout=8.0)
        self.wait(0.6)
        self.human_off()

    def run_tool(self, tool, params=None, whole=True):
        n0 = self.ev("() => LG.results.length")
        self.select("#anTool", tool)
        for k, v in (params or {}).items():
            self.type_into(f"#anParams [data-k={k}]", v)
        if whole:
            self.click("#bAnAll", settle=0.15)
        self.click("#bAnRun")
        self.pg.wait_for_function(f"() => LG.results.length > {n0} && LG.results[0].res.tool === '{tool}'", timeout=40000)
        res = self.ev("() => LG.results[0].res")
        self.log(f"TOOL {tool} ok={res.get('ok')} {str(res.get('result'))[:160]}")
        return res


def fmt(v, d=2):
    return "—" if v is None else f"{v:.{d}f}"


def scenario(D):
    pg = D.pg
    # ================= 타이틀 =================
    D.title("v22 로거·측정 앱 — 실기 측정 리허설",
            ["λ · r · γ 를 순서대로 재고, 그대로 증분접기까지",
             "· 로봇 = MuJoCo 가상 로봇 (v21 실측 프리셋)  ·  앱 조작 = 실제 마우스",
             "· 🤚 주황 카드 = 사람이 로봇에 하는 동작  ·  파란 자막 = 앱에서 보는 것"], hold=5.0)

    # ================= 1. 연결·준비 =================
    D.step("1 / 4  연결 · 영점 · 준비")
    D.cap("실기에서는 포트를 고르고 「연결」. 오늘은 로봇 대신 「가짜: MuJoCo」 — 나머지는 모두 같다", hold=1.2)
    D.click("#bFakeMj")
    pg.wait_for_function("() => LG.ds.n > 50", timeout=20000)
    D.wait(1.5)
    D.cap("연결 직후: D행이 100 Hz 로 들어오고 3D 트윈 · 원위치 평면 · 예측점 평면 · 차트가 살아난다. 기록은 자동으로 시작(logs/ 4파일)",
          "펌웨어에는 아무것도 먼저 보내지 않는다 (문서 77 §6)", hold=4.0)
    D.tab("cmd")
    D.cap("영점: 로봇을 매달아 완전히 멎게 한 뒤 「z 영점」, 반대쪽에서 정착시킨 뒤 한 번 더 (2단 매달림 영점)")
    D.human("로봇을 매단 채 한쪽으로 정착시켜 멎게 한다")
    D.wait(1.2); D.palette("z 영점"); D.wait(0.8)
    D.human("이번엔 반대쪽으로 살짝 밀어 정착시킨다")
    D.wait(1.2); D.palette("z 영점"); D.wait(0.6); D.human_off()
    D.palette("t 상태")
    D.cap("「t 상태」 — 모드 · w 벡터 · γ · 영점 단계가 콘솔에 나온다. 파라미터는 「w 파라미터」", hold=2.5)
    D.tab("pipe")
    D.cap("파이프라인: 앱은 원시 엔코더 열(φ, ank, δ)에서 α · β · 속도 · Â 를 다시 계산한다. P2R 는 지난 측정값 0.4285 그대로",
          "λ · r 은 이제부터 재서 채운다", hold=3.5)
    D.tab("cmd")

    # ================= 2. λ =================
    D.step("2 / 4  λ — 놓기 3회")
    D.cap("λ 측정: 「mode 0 측정」(접지 않음). 로봇을 β = +1°, φ = 0° 에서 1 초 멈췄다가 놓는다 → 발산 → 9° 쯤에서 잡는다", hold=1.0)
    D.palette("mode 0 측정")
    D.release(1.0, 0.0)
    D.cap("한 번이 제대로 됐는지 먼저 본다: 분석 도구 「시행 나누기」 → 전체 → 실행", hold=0.8)
    res = D.run_tool("trials")
    tb = [r for r in res.get("table", []) if r.get("dir_valid")]
    r1 = tb[-1] if tb else {}
    D.cap(f"시행 {r1.get('k', '?')}: 방향 {'+' if r1.get('dir', 1) > 0 else '−'}, 놓기점 (β {fmt(r1.get('beta0'))}, φ {fmt(r1.get('phi0'))}), "
          f"λ = {fmt(r1.get('lam'))} (R² {fmt(r1.get('lam_r2'), 3)})",
          "정지(손) → 이탈 로 놓기 순간을 찾고, 놓기 뒤 ln|ψ| 직선 적합. 차트에 시행 띠와 놓기 선이 겹친다", hold=4.0)
    D.cap("좋다 — 두 번 더. 반대쪽(β = −1°)과 다시 +1°", hold=0.8)
    D.release(-1.0, 0.0)
    D.release(1.0, 0.0)
    res = D.run_tool("trials")
    rr = res.get("result", {})
    D.cap(f"시행 {rr.get('n_dir_valid')}개 (손으로 옮긴 동작은 자동 제외): λ₊ = {fmt(rr.get('lam_plus'))}, λ₋ = {fmt(rr.get('lam_minus'))}",
          "두 방향이 20 % 넘게 갈리면 φ_eq 를 의심 (문서 79 §3)", hold=3.5)
    lam_vals = [x for x in (rr.get("lam_plus"), rr.get("lam_minus")) if x]
    lam_hat = sum(lam_vals) / len(lam_vals) if lam_vals else 5.4
    # 시행 하나의 λ 적합을 차트에 겹쳐 본다
    ks = [r["k"] for r in res.get("table", []) if r.get("valid")]
    if ks:
        D.select("#anTrial", str(ks[-1]))
        D.wait(0.5)
        D.run_tool("lambda", whole=False)
        D.cap("「λ — ln|ψ| 직선적합」: 어느 구간을 썼는지(밴드 2~9°), 통과 시각, 적합선, 잔차가 차트 위에 그대로 그려진다", hold=4.0)
        D.cap("시행을 고르면 REPLAY 로 그 순간을 되짚어 본다 — 「따라가기」로 다시 LIVE", hold=0.8)
        D.click("#bFollow")
    D.cap(f"λ̂ = {lam_hat:.2f} → 파이프라인 lam 과 펌웨어 lam 에 넣는다 (Â 의 속도항 w = r·λ 닫힌형이 이 값을 쓴다)", hold=1.0)
    D.tab("pipe"); D.type_into("#pipeForm [data-k=lam]", f"{lam_hat:.2f}"); D.click("#bPipeApply"); D.wait(0.6)
    D.tab("cmd"); D.param_set("lam", f"{lam_hat:.2f}"); D.wait(1.0)

    # ================= 3. r =================
    D.step("3 / 4  r — 다음 놓기 추천 루프")
    D.cap("r 측정: 「★ 다음 놓기 추천」. 놓을 때마다 선까지의 거리 s 를 재고, 다음에 어디서 놓을지 앱이 정한다",
          "「추천→목표」가 켜져 있으면 원위치 평면에 십자선으로 다음 점이 찍힌다. 안정모드 진동수 ω 도 시행들에서 함께 맞춘다", hold=1.0)
    res = D.run_tool("recommend")
    nx = res.get("next") or {}
    for i in range(6):
        rr = res.get("result", {})
        D.cap(f"추천 #{i + 1}: β = {nx.get('beta', 0):+.2f}°, φ = {nx.get('phi', 0):+.2f}°  ({nx.get('reason', '')})",
              f"현재 r̂ = {fmt(rr.get('r'), 3)} ± {fmt(rr.get('se_r'), 3)} · ĉ₀ = {fmt(rr.get('c0'), 2)} · ω̂ = {fmt(rr.get('om_hat'))} · n = {rr.get('n')}", hold=1.2)
        D.release(nx.get("beta", 0.0), nx.get("phi", 0.7), sub="십자선 자리에서 놓는다. 넘어지는 쪽만 보면 된다")
        res = D.run_tool("recommend")
        nx = res.get("next") or {}
    rr = res.get("result", {})
    r_hat = rr.get("r") or -1.5
    D.cap(f"놓기 {rr.get('n')}회 → r̂ = {fmt(rr.get('r'), 3)} ± {fmt(rr.get('se_r'), 3)}, ĉ₀ = {fmt(rr.get('c0'), 2)}"
          + ("  — 충분 (SE_r < 0.05)" if rr.get("enough") else "  — 실기에서는 SE_r < 0.05 까지 계속"),
          "원위치 평면: 점 색 = 넘어진 방향, 주황 실선 = 추정한 안정모드선", hold=4.0)
    c0_hat = rr.get("c0") or 0.0
    D.cap(f"r̂ = {r_hat:.3f}, ĉ₀ = {c0_hat:.2f} 을 파이프라인(r, c0)과 펌웨어(r, c0)에 넣는다 — c₀ 를 빼먹으면 한쪽으로만 접어 hold 가 흘러간다", hold=0.8)
    D.tab("pipe"); D.type_into("#pipeForm [data-k=r]", f"{r_hat:.3f}"); D.type_into("#pipeForm [data-k=c0]", f"{c0_hat:.2f}"); D.click("#bPipeApply"); D.wait(0.6)
    D.tab("cmd"); D.param_set("r", f"{r_hat:.3f}"); D.param_set("c0", f"{c0_hat:.2f}"); D.wait(1.0)

    # ================= 4. γ =================
    D.step("4 / 4  γ — 단일접기 성적표 → 증분접기")
    D.cap("γ 측정: 「mode 1 단일접기」. 놓기 전에 「g 시작」으로 무장 → 첫 |Â| > trig 에서 딱 한 번 접고 δ 고정 → 자유 발산 → 잡기", hold=1.0)
    D.palette("mode 1 단일접기")
    for i, (b, f) in enumerate([(0.3, 0.0), (-0.3, 0.0), (0.35, 0.0)]):
        D.live()
        D.human(f"로봇을 β = {b:+.2f}°, φ = {f:+.2f}° 로 잡는다 (|Â| ≈ {abs(b):.1f}° < trig 0.6° — 손에서 접히지 않게 여유를 둔다)", "아직 놓지 않는다")
        D.robot(f"hold {b} {f}")
        D.wait(1.6)
        D.cap(f"접기 시행 {i + 1}: 「g 시작」으로 무장하자마자 놓는다 — 손에 든 채 오래 두면 손떨림으로 Â 가 문턱을 넘어 손 안에서 접힌다", hold=1.0)
        D.human("「g」 직후 놓는다", "접힌 뒤에도 δ 는 고정 — 그대로 발산하게 두고 9° 에서 잡는다")
        D.palette("g 시작", settle=0.05)
        D.robot("free")
        D.cap(f"Â 가 0.6° 를 넘는 순간 펌웨어가 γ·Â 만큼 접는다 (콘솔 E,FOLD → 250 ms 뒤 F행)")
        D.wait_stage("free", 5.0); D.wait_stage("held", 8.0); D.wait(0.5)
        D.human_off()
        D.palette("h 정지")
        D.type_into("#iDelta", "0"); D.click("#bDeltaGo")
        D.cap("「h 정지」 → δ 를 0 으로 펴고 다음 시행", hold=1.0)
    res = D.run_tool("fold")
    rr = res.get("result", {})
    D.cap(f"「접기 성적표」: 시행별 A⁻(접기 직전 Â), Δδ, A⁺(관측창 뒤) → A⁺ = G·A⁻ − g·Δδ,  G = e^(λ·lock) = {fmt(rr.get('G'))}, "
          f"g = {fmt(rr.get('g_mean'))} → γ* = G/g = {fmt(rr.get('gamma_star'))}  (시행 {rr.get('n_valid')}회, γ 중앙값 {fmt(rr.get('gamma_median'))} ± {fmt(rr.get('gamma_se'))})",
          "γ* 는 한 번 접기로 예측점을 선 위에 올리는(deadbeat) 이득. 「펌웨어로 보내기」가 gam 명령을 보낸다", hold=5.0)
    if res.get("next_cmd"):
        D.click("#bSendNext")
        D.wait(1.0)
    D.cap("이제 「mode 2 증분접기」 — 잰 λ · r · γ 만 넣은 본 제어. |Â| > trig 마다 hold += γ·Â, |Â| 가 작아지면 천천히 편다", hold=1.0)
    D.palette("mode 2 증분접기")
    D.live()
    D.human("로봇을 β = +0.3°, φ = 0° 로 잡는다", "「g」 직후 바로 놓는다")
    D.robot("hold 0.3 0.0"); D.wait(1.8)
    D.human("「g」 — 놓는다, 손을 뗀다")
    D.palette("g 시작", settle=0.05)
    D.robot("free")
    D.wait(1.0); D.human_off()
    D.cap("증분접기가 잡는다: 3D 트윈이 접었다 펴고, hold 가 계단처럼 움직이며 φ 가 유계로 남는다", hold=6.0)
    D.cap("예측점 평면에서는 점이 A = 0 선(초록) 근처를 오가고, 원위치 평면에서는 안정모드선 둘레를 돈다", hold=6.0)
    st = D.stage()
    D.palette("h 정지")
    D.cap("「h 정지」. 기록은 logs/ 에 .csv · .trials.csv · .folds.csv · .events.csv · .raw.txt 로 남아 있어 언제든 다시 불러 검토할 수 있다",
          f"(가상 로봇 상태: {'자유 — 균형 유지 중' if st == 'free' else st})", kind="note", hold=4.0)
    D.title("끝 — 실기에서 같은 순서로",
            ["1  연결 · 2단 영점 · P2R 0.4285 유지", "2  λ: mode 0 · β = ±1° 놓기 3회 · 시행 나누기 · lam 넣기",
             "3  r: 다음 놓기 추천 루프 · SE_r < 0.05 까지 · r 넣기", "4  γ: mode 1 · g → 놓기 3회 · 접기 성적표 · gam 보내기 → mode 2",
             "· docs/v22_측정_조작_가이드_20260902.md"], hold=6.0)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8233)
    ap.add_argument("--no-server", action="store_true")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--name", default="v22_rehearsal")
    a = ap.parse_args(argv)
    os.makedirs(OUT, exist_ok=True)
    srv = None
    if not a.no_server:
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        srv = subprocess.Popen([sys.executable, os.path.join(HERE, "server.py"), "--port", str(a.port)],
                               cwd=HERE, env=env, stdout=open(os.path.join(OUT, "server.log"), "w"), stderr=subprocess.STDOUT)
        time.sleep(4.0)
    from playwright.sync_api import sync_playwright
    webm = None
    try:
        with sync_playwright() as p:
            br = p.chromium.launch(executable_path="/opt/pw-browsers/chromium",
                                   args=["--use-gl=swiftshader", "--enable-unsafe-swiftshader", "--lang=ko-KR"])
            ctx = br.new_context(viewport={"width": W, "height": H}, record_video_dir=OUT,
                                 record_video_size={"width": W, "height": H}, locale="ko-KR")
            pg = ctx.new_page()
            errs = []
            pg.on("pageerror", lambda e: errs.append(str(e)))
            pg.goto(f"http://localhost:{a.port}/")
            pg.wait_for_function("() => window.LG && LG.PL", timeout=20000)
            time.sleep(1.0)
            D = Director(pg, quick=a.quick)
            try:
                scenario(D)
            except Exception as ex:
                D.log(f"!! 시나리오 오류: {type(ex).__name__}: {ex}")
                pg.screenshot(path=os.path.join(OUT, "error.png"))
                raise
            finally:
                if errs:
                    print("페이지 오류:", errs[:5])
                pg.evaluate("() => LG.send({cmd:'disconnect'})")
                time.sleep(0.5)
                webm = pg.video.path()
                ctx.close()
                br.close()
    finally:
        if srv:
            srv.terminate()
    if not webm or not os.path.exists(webm):
        print("녹화 파일이 없다"); return 1
    dst = os.path.join(OUT, a.name + ".webm")
    shutil.move(webm, dst)
    print("webm:", dst, f"{os.path.getsize(dst) / 1e6:.1f} MB")
    mp4 = os.path.join(OUT, a.name + ".mp4")
    ff = None
    try:
        import imageio_ffmpeg
        ff = imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        ff = shutil.which("ffmpeg")
    if ff:
        cmd = [ff, "-y", "-loglevel", "error", "-i", dst, "-c:v", "libx264", "-preset", "medium", "-crf", "22",
               "-pix_fmt", "yuv420p", "-r", "25", "-movflags", "+faststart", mp4]
        rc = subprocess.call(cmd)
        if rc == 0 and os.path.exists(mp4):
            print("mp4:", mp4, f"{os.path.getsize(mp4) / 1e6:.1f} MB")
        else:
            print("mp4 변환 실패 (rc", rc, ") — webm 은 있다")
    else:
        print("ffmpeg 없음 — webm 만 남긴다 (pip install imageio-ffmpeg)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
