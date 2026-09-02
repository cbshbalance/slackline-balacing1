# -*- coding: utf-8 -*-
"""sim_recommend.py — 다음 놓기 추천 루프의 합성 검증.
선형 2모드 모델(참값 r, c₀, λ, ω)에서 '사용자'가 추천대로 놓고(손 오차 ±0.2°), 앱이 시행 나누기 → 추천을 반복.
몇 회 만에 r̂ 이 참값 ±0.1 에 드는지 본다.   실행: python sim_recommend.py [N회] [seed]
"""
import sys
import numpy as np
from dataset_v22 import Dataset
import analysis_v22 as an


class World:
    def __init__(self, r=-1.5, c0=0.4, lam=5.5, om=5.0, r_u=0.7, noise=0.03, hz=100.0, seed=0):
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


if __name__ == "__main__":
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    run(N, seed)
