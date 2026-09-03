# -*- coding: utf-8 -*-
"""sim_recommend.py — 다음 놓기 추천 루프의 합성 검증.
선형 2모드 모델(참값 r, c₀, λ, ω)에서 '사용자'가 추천대로 놓고(손 오차 ±0.2°), 앱이 시행 나누기 → 추천을 반복.
몇 회 만에 r̂ 이 참값 ±0.1 에 드는지 본다.   실행: python sim_recommend.py [N회] [seed]
"""
import sys
import time
import numpy as np
from dataset_v22 import Dataset
import analysis_v22 as an


class World:
    def __init__(self, r=-1.5, c0=0.4, lam=5.5, om=5.0, r_u=-0.5, noise=0.03, hz=100.0, seed=0):
        # r_u = 불안정모드 φ/β 비. ★음수다 (MuJoCo·0822 실측: 선 위에서 놓으면 β + 로 자라며 φ 는 − 로 넘어진다).
        #   2026-09-02 판의 +0.7 은 부호가 거꾸로였고, 추천 도구도 같은 가정을 해 서로 맞아 보였다 — 9/3 정정.
        self.r, self.c0, self.lam, self.om, self.r_u, self.noise, self.hz = r, c0, lam, om, r_u, noise, hz
        self.rng = np.random.default_rng(seed)
        self.ds = Dataset(); self.ds.set_header("t_ms,phi,ank,del_now".split(","))
        self.t = 0.0
        self.pos = (0.0, 0.0)          # (β, φ) 현재 손 위치

    def _row(self, beta, phi):
        ph = phi + self.rng.normal(0, self.noise); ak = beta + phi + self.rng.normal(0, self.noise)
        self.ds.add_data_row(f"{int(round(self.t*1000))},{ph:.4f},{ak:.4f},0")
        self.t += 1.0 / self.hz

    def hold(self, beta, phi, sec):
        for _ in range(int(sec * self.hz)):
            self._row(beta, phi)

    def move(self, beta, phi, sec=0.6):
        b0, p0 = self.pos
        n = int(sec * self.hz)
        for k in range(n):
            f = (k + 1) / n
            self._row(b0 + (beta - b0) * f, p0 + (phi - p0) * f)
        self.pos = (beta, phi)

    def release(self, beta0, phi0, hand_err=0.2):
        """목표에 손 오차를 더한 실제 놓기점에서 정지 → 놓기 → 발산/진동 → 8.5° 이탈 시 잡음."""
        beta0 = beta0 + self.rng.uniform(-hand_err, hand_err); phi0 = phi0 + self.rng.uniform(-hand_err, hand_err)
        self.move(beta0, phi0); self.hold(beta0, phi0, 1.0)
        A0 = (phi0 - self.c0 - self.r * beta0) / (self.r_u - self.r); B0 = beta0 - A0
        tt = 0.0
        while tt < 3.0:
            A = A0 * np.cosh(self.lam * tt); B = B0 * np.cos(self.om * tt)
            phi = self.c0 + A * self.r_u + B * self.r; beta = A + B
            self._row(beta, phi)
            tt += 1.0 / self.hz
            if abs(phi - phi0) >= 8.5:
                break
        self.pos = (beta, phi)
        self.hold(beta, phi, 0.3)          # 잡은 채 잠깐
        return beta0, phi0


def run(N=8, seed=0, verbose=True, **kw):
    W = World(seed=seed, **kw)
    hist = []
    for i in range(N):
        rec = an.run(W.ds, "recommend", dict(lam_fixed=W.lam, r_guess=-1.64, off=0.5))
        nx = rec["next"]; R = rec["result"]
        hist.append(dict(i=i, r=R["r"], c0=R["c0"], se_r=R["se_r"], n=R["n"], method=R["method"], next=(nx["beta"], nx["phi"])))
        if verbose:
            print(f"  {i:2d}회 전: n={R['n']} r̂={R['r']} ĉ₀={R['c0']} SE_r={R['se_r']} [{R['method']}]  → 다음 (β {nx['beta']:+.2f}, φ {nx['phi']:+.2f})  {nx['reason'][:40]}")
        W.release(nx["beta"], nx["phi"])
    rec = an.run(W.ds, "recommend", dict(lam_fixed=W.lam, r_guess=-1.64, off=0.5)); R = rec["result"]
    hist.append(dict(i=N, r=R["r"], c0=R["c0"], se_r=R["se_r"], n=R["n"], method=R["method"]))
    if verbose:
        print(f"  {N:2d}회 후: n={R['n']} r̂={R['r']} ĉ₀={R['c0']} SE_r={R['se_r']}  (참값 r={W.r} c₀={W.c0})")
        for row in rec["table"]: print("     ", row)
    return hist, W


if __name__ == "__main__" and not (len(sys.argv) > 1 and sys.argv[1] in ("fold", "mujoco")):
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    run(N, seed)


# ============================================================================
# World2 — 수치 적분 2모드 모델 (접기 실험용). 불안정/안정 모드 좌표를 오일러 적분하고,
# 접기(Δδ)는 무게중심 등고선을 따라 (Δβ, Δφ) = (−kFold·Δδ, +sC·kFold·Δδ) 위치 점프로 넣는다 (v21 규약).
# 앱과 같은 닫힌형 w 로 Â 를 계산해 |Â|>trig 에서 γ·Â 만큼 접는 '단일접기' 시행을 만든다.
# ============================================================================
class World2:
    def __init__(self, r=-1.5, c0=0.0, lam=5.5, om=5.0, r_u=-0.5, kfold=0.35, sC=0.89, p2r=0.4285,
                 noise=0.03, hz=100.0, seed=0):
        self.r, self.c0, self.lam, self.om, self.r_u, self.kfold, self.sC, self.p2r = r, c0, lam, om, r_u, kfold, sC, p2r
        self.noise, self.hz = noise, hz
        self.rng = np.random.default_rng(seed)
        self.ds = Dataset(pipe=dict(r=r, c0=c0, lam=lam, om=om, p2r=p2r, wmode="closed", kv=1.0))
        self.ds.set_header("t_ms,phi,ank,alpha,beta,dphi,dbeta,Ahat,hold,del_now,phase,cue,err".split(","))
        self.t = 0.0; self.delta = 0.0; self.hold = 0.0; self.phase = 4
        self.q = np.zeros(4)                # A, Adot, B, Bdot (모드 좌표)
        self.ev = []

    # --- 모드 ↔ (β, φ)
    def to_bp(self, A, B):
        return A + B, self.c0 + A * self.r_u + B * self.r
    def from_bp(self, beta, phi):
        A = (phi - self.c0 - self.r * beta) / (self.r_u - self.r); B = beta - A
        return A, B
    def Ahat(self, beta, phi, bd, pd):
        r, lam = self.r, self.lam
        return (-1.0 / r) * phi + beta + (-1.0 / (r * lam)) * pd + (1.0 / lam) * bd + self.c0 / r

    def _row(self, beta, phi, phase=None):
        ph = phi + self.rng.normal(0, self.noise)
        ak = beta + phi - self.p2r * self.delta + self.rng.normal(0, self.noise)   # 앱의 β = (ank−φ) + P2R·δ 가 world 의 β 가 되게
        self.ds.add_data_row(f"{int(round(self.t*1000))},{ph:.4f},{ak:.4f},0,0,0,0,0,{self.hold:.2f},{self.delta:.2f},{self.phase if phase is None else phase},0,0")
        self.t += 1.0 / self.hz

    def hold_still(self, beta, phi, sec):
        A, B = self.from_bp(beta, phi); self.q[:] = [A, 0, B, 0]
        for _ in range(int(sec * self.hz)):
            self._row(beta, phi)

    def move(self, beta, phi, sec=0.6):
        b0, p0 = self.to_bp(self.q[0], self.q[2])
        n = int(sec * self.hz)
        for k in range(n):
            f = (k + 1) / n
            self._row(b0 + (beta - b0) * f, p0 + (phi - p0) * f)
        A, B = self.from_bp(beta, phi); self.q[:] = [A, 0, B, 0]

    def step(self, dt):
        A, Ad, B, Bd = self.q
        Ad += self.lam ** 2 * A * dt; A += Ad * dt
        Bd += -self.om ** 2 * B * dt; B += Bd * dt
        self.q[:] = [A, Ad, B, Bd]

    def fold(self, dd):
        """접기: 위치 점프 (Δβ, Δφ) = (−kFold·Δδ, sC·kFold·Δδ), 속도 불변. δ 는 프로파일(80 ms)로 따라간다."""
        beta, phi = self.to_bp(self.q[0], self.q[2])
        bd, pd = self.q[1] + self.q[3], self.q[1] * self.r_u + self.q[3] * self.r
        u = self.kfold * dd
        A2, B2 = self.from_bp(beta - u, phi + self.sC * u)
        # 속도는 모드 좌표로 다시 분해 (β̇, φ̇ 불변)
        Ad2 = (pd - self.r * bd) / (self.r_u - self.r); Bd2 = bd - Ad2
        self.q[:] = [A2, Ad2, B2, Bd2]
        self.hold += dd
        self.ds.add_event(int(round(self.t * 1000)), "FOLD", f"{dd:.2f}")

    def release_fold(self, beta0, phi0, gamma, trig=0.6, hand_err=0.2, fold_ms=80.0, max_s=3.0, single=True):
        beta0 += self.rng.uniform(-hand_err, hand_err); phi0 += self.rng.uniform(-hand_err, hand_err)
        self.move(beta0, phi0); self.hold_still(beta0, phi0, 1.0)
        self.phase = 0
        dt = 1.0 / self.hz; sub = 10; tt = 0.0; folded = False; pending = None
        while tt < max_s:
            for _ in range(sub):
                self.step(dt / sub)
                if pending is not None:                       # δ 프로파일 (선형)
                    frac = min(1.0, (self.t + tt * 0 - pending[0]) / (fold_ms / 1000.0))
                    self.delta = pending[1] + (pending[2] - pending[1]) * frac
            beta, phi = self.to_bp(self.q[0], self.q[2])
            bd, pd = self.q[1] + self.q[3], self.q[1] * self.r_u + self.q[3] * self.r
            Ah = self.Ahat(beta, phi, bd, pd)
            if not folded and abs(Ah) > trig:
                dd = gamma * Ah
                pending = (self.t, self.delta, self.delta + dd)
                self.fold(dd); folded = True; self.phase = 1
            if pending is not None and self.t - pending[0] >= fold_ms / 1000.0:
                self.delta = pending[2]; pending = None; self.phase = 0
            beta, phi = self.to_bp(self.q[0], self.q[2])
            self._row(beta, phi)
            tt += dt
            if abs(phi - phi0) >= 8.5:
                break
        self.phase = 4
        beta, phi = self.to_bp(self.q[0], self.q[2])
        self.hold_still(beta, phi, 0.3)
        return folded

    def g_true(self):
        """접기 맵의 g (Â 단위/°): 위치 점프가 Â 에 주는 변화 = w_pos·(Δφ, Δβ)/Δδ"""
        return -((-1.0 / self.r) * self.sC * self.kfold - self.kfold)


def run_fold(N=5, seed=0, gamma=6.0, lock_ms=250.0, verbose=True):
    W = World2(seed=seed)
    for i in range(N):
        W.release_fold(0.0, 0.7 if i % 2 == 0 else -0.7, gamma)
    rep = an.run(W.ds, "fold", dict(lock_ms=lock_ms))
    G = math.exp(W.lam * lock_ms / 1000.0); g = W.g_true()
    if verbose:
        print("fold report ok", rep["ok"], rep.get("msg"), {k: rep["result"].get(k) for k in ("n_folds", "n_valid", "G", "gamma_star", "gamma_median", "gamma_se", "g_mean", "G_fit", "g_fit", "gamma_fit")})
        print(f"   (시행 γ={gamma}, G=e^(λ·lock)={G:.3f}. γ* 의 참값은 접기 프로파일·관측창에 따라 정해지므로 γ=6 과 γ=10 시행의 γ* 일치와 γ* 로 접었을 때 A⁺≈0 으로 검증한다)")
        for r in rep["table"]:
            print("   ", {k: r.get(k) for k in ("k", "A_pre", "dd_act", "A_post", "ratio", "g", "gamma", "verdict")})
    return rep, W


import math  # noqa: E402  (World2 에서 씀)
if __name__ == "__main__" and len(sys.argv) > 1 and sys.argv[1] == "fold":
    run_fold(int(sys.argv[2]) if len(sys.argv) > 2 else 5, gamma=float(sys.argv[3]) if len(sys.argv) > 3 else 6.0)



# ============================================================================
# MuJoCo 가상 로봇으로 추천 루프 폐루프 검증 (물리 참값: 엔진 평면 r).  python sim_recommend.py mujoco [N] [seed]
# ============================================================================
def run_mujoco(N=10, seed=0, verbose=True, first=((1.0, 0.0), (-1.0, 0.0))):
    import mujoco_source as mjs, serial_bridge as sb
    src = mjs.MujocoSource(seed=seed, realtime=False); src.start()
    t0 = time.time()
    while src.eng is None and not src.eng_err and time.time() - t0 < 60:
        time.sleep(0.1)
    if src.eng is None:
        raise RuntimeError(src.eng_err or "엔진 없음")
    ds = Dataset(); sink = sb.LineSink()
    def pump():
        for _, ln in src.drain():
            kind, prefix, payload = sink.classify(ln)
            if kind == "header" and prefix == "D": ds.set_header(payload)
            elif kind == "data": ds.add_data_row(payload)
    def until(cond, timeout=60.0):
        t_ = time.time()
        while time.time() - t_ < timeout:
            if cond(): return True
            time.sleep(0.03)
        return False
    def release(b, f):
        nc = src.n_catch
        src.write(f"sim release {b:.3f} {f:.3f} 1.0"); until(lambda: src.n_catch > nc and src.stage == "held"); time.sleep(0.12)
    src.write("z"); src.write("z")
    for b, f in first:                      # λ 시행과 같은 자리(±1°, φ=0) 둘
        release(b, f)
    r_true = float(src.eng.g["mdl"]["modes"]["r"]); om_true = float(src.eng.g["mdl"]["modes"]["w_eff"])
    hist = []
    for i in range(N):
        pump()
        rec = an.run(ds, "recommend", dict(phi_eq=0.0))
        nx, R = rec["next"], rec["result"]
        hist.append(dict(i=i, r=R["r"], c0=R["c0"], se_r=R["se_r"], n=R["n"], om=R.get("om_hat"), method=R["method"], next=(nx["beta"], nx["phi"])))
        if verbose:
            print(f"  {i:2d}회 전: n={R['n']} r̂={R['r']} ĉ₀={R['c0']} SE_r={R['se_r']} ω̂={R.get('om_hat')} [{R['method']}] → 다음 (β {nx['beta']:+.2f}, φ {nx['phi']:+.2f})")
        release(nx["beta"], nx["phi"])
    pump()
    rec = an.run(ds, "recommend", dict(phi_eq=0.0)); R = rec["result"]
    hist.append(dict(i=N, r=R["r"], c0=R["c0"], se_r=R["se_r"], n=R["n"], om=R.get("om_hat"), method=R["method"]))
    if verbose:
        print(f"  {N:2d}회 후: n={R['n']} r̂={R['r']} ĉ₀={R['c0']} SE_r={R['se_r']} ω̂={R.get('om_hat')}   (MuJoCo 선형모델 r={r_true:.3f}, ω_eff={om_true:.2f})")
        for row in rec["table"]:
            if row.get("dir_valid"): print("     ", {k: row.get(k) for k in ("k", "beta0", "phi0", "dir", "s", "B_osc", "fit_r2")})
    src.close()
    return hist, dict(r_true=r_true, om_true=om_true, rec=rec)


if __name__ == "__main__" and len(sys.argv) > 1 and sys.argv[1] == "mujoco":
    import time
    run_mujoco(int(sys.argv[2]) if len(sys.argv) > 2 else 10, int(sys.argv[3]) if len(sys.argv) > 3 else 0)
