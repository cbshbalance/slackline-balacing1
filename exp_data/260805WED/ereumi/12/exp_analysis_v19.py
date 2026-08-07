# -*- coding: utf-8 -*-
"""
exp_analysis_v19.py — 자가 측정 실험 분석 (v19, 2026-07-27)
=============================================================
exp_selftest_v19.ino 가 출력하고 calib/serial_logger.py 로 저장한 CSV를 분석한다.
(실험 ① 자유 흔들기는 기존 calib/calib_analysis.py freeswing 사용 — 여기 없음)

사용:
  python exp_analysis_v19.py backlash log0.csv
  python exp_analysis_v19.py exp2 log2.csv [--alpha ank-phi]
  python exp_analysis_v19.py exp3 log3.csv [--kt 1.304]
  python exp_analysis_v19.py exp4 log4.csv --lam 6.4 [--w wphi,wbeta,wphid,wbetad]

공통 옵션:
  --params <v19_bringup 경로>   params_v19.flat() 에서 p1,p2,h 자동 로드 (권장)
  --p1 --p2 --h                 수동 입력 (params 임포트 실패 시)
  --alpha {ank-phi, ank+phi, ank}  하체 절대각 α 계산식 (기본 ank-phi)
      ※ '정합 점검'으로 확정: 로봇을 스윙 +방향으로 살짝 기울였을 때 β>0 이어야 한다.

CSV 형식: D,t_ms,phi_deg,ank_deg,del_deg,dvel_dps,cur_mA  /  E,t_ms,이벤트[,인자]
"""
import sys, os, csv, argparse
import numpy as np


# ---------------- 공통 ----------------
def load(path):
    T, PHI, ANK, DEL, VEL, CUR = [], [], [], [], [], []
    ev = []
    with open(path, encoding="utf-8", errors="ignore") as f:
        for ln in f:
            p = ln.strip().split(",")
            try:
                if p[0] == "D" and len(p) >= 7:
                    T.append(float(p[1])); PHI.append(float(p[2])); ANK.append(float(p[3]))
                    DEL.append(float(p[4])); VEL.append(float(p[5])); CUR.append(float(p[6]))
                elif p[0] == "E":
                    ev.append((float(p[1]), p[2], float(p[3]) if len(p) > 3 else np.nan))
            except ValueError:
                continue
    if not T:
        sys.exit(f"데이터 행(D,...)이 없음: {path}")
    d = dict(t=np.array(T) / 1000.0, phi=np.array(PHI), ank=np.array(ANK),
             dl=np.array(DEL), vel=np.array(VEL), cur=np.array(CUR), ev=ev)
    print(f"[load] {path}: {len(T)}행 {d['t'][-1]-d['t'][0]:.1f}s, 이벤트 {len(ev)}개")
    return d


def body_params(args):
    """p1, p2, h — params_v19 우선, 실패 시 CLI 값"""
    if args.params:
        sys.path.insert(0, args.params)
        try:
            import params_v19
            p = params_v19.flat()
            m1, m2, L1 = p["m1"], p["m2"], p["L1"]
            lc1 = p.get("ell1", p.get("LC1", L1 * 0.5))
            lc2 = p.get("ell2", p.get("LC2", p["L2"] * 0.5))
            p1 = m1 * lc1 + m2 * L1
            p2 = m2 * lc2
            h = (p1 + p2) / (m1 + m2)
            print(f"[params] p1={p1:.4f} p2={p2:.4f} h={h:.4f} (params_v19)")
            return p1, p2, h
        except Exception as e:
            print(f"[params] 임포트 실패({e}) → --p1 --p2 --h 사용")
    if args.p1 is None:
        sys.exit("--params 또는 --p1/--p2/--h 를 지정하라")
    return args.p1, args.p2, args.h


def states(d, args, p1, p2, h):
    """φ, β [deg] + 속도 [deg/s] (스무딩 미분)"""
    phi = d["phi"]
    if args.alpha == "ank-phi":   alpha = d["ank"] - phi
    elif args.alpha == "ank+phi": alpha = d["ank"] + phi
    else:                          alpha = d["ank"]
    theta = alpha + d["dl"]
    beta = (p1 * alpha + p2 * theta) / (p1 + p2)
    t = d["t"]

    def dot(x):                     # 5점 이동평균 후 gradient
        k = np.ones(5) / 5
        xs = np.convolve(x, k, "same")
        return np.gradient(xs, t)
    return t, phi, beta, dot(phi), dot(beta)


def ev_times(d, name):
    return [t / 1000.0 for (t, n, a) in d["ev"] if n == name], \
           [a for (t, n, a) in d["ev"] if n == name]


# ---------------- [실험 0] 백래시 ----------------
def cmd_backlash(args):
    d = load(args.log)
    t, dl, ank = d["t"], d["dl"], d["ank"]
    # δ 방향 반전 구간에서 발목(=몸 반응)이 멈춰 있는 δ 폭을 잰다
    vd = np.gradient(np.convolve(dl, np.ones(9) / 9, "same"), t)
    va = np.gradient(np.convolve(ank, np.ones(9) / 9, "same"), t)
    moving = np.abs(vd) > 1.0                      # δ가 움직이는 중(>1°/s)
    dead = moving & (np.abs(va) < 0.15 * np.abs(vd))  # 몸은 안 따라옴
    # 연속 dead 구간별 δ 이동량 = 그 반전의 백래시 폭
    widths = []
    i = 0
    while i < len(dead):
        if dead[i]:
            j = i
            while j < len(dead) and dead[j]:
                j += 1
            if t[j - 1] - t[i] > 0.02:             # 20ms 이상 지속된 구간만
                widths.append(abs(dl[j - 1] - dl[i]))
            i = j
        else:
            i += 1
    widths = [w for w in widths if 0.02 < w < 2.0]
    if not widths:
        print("반전 데드존을 찾지 못함 — 스윕 속도/로그 확인")
        return
    bl = np.median(widths)
    print(f"\n=== 백래시(중앙값) = {bl:.3f}°  (구간 {len(widths)}개, 최대 {max(widths):.3f}°) ===")
    print("판정: < 0.10° → 실물이 시뮬 산포를 이길 가능성 있음(트리거 인하 후보)")
    print("      > 0.30° → 트리거 0.6° 아래로 내리지 말 것")


# ---------------- [실험 ②] 기울여 놓기 → λ_u, w ----------------
def cmd_exp2(args):
    d = load(args.log)
    p1, p2, h = body_params(args)
    t, phi, beta, phid, betad = states(d, args, p1, p2, h)

    # 시도 구간: MARK 짝(놓음, 잡음) 우선, 없으면 |β| 성장 구간 자동 탐지
    mk, _ = ev_times(d, "MARK")
    segs = []
    if len(mk) >= 2:
        for i in range(0, len(mk) - 1, 2):
            m = (t >= mk[i]) & (t <= mk[i + 1])
            if m.sum() > 20:
                segs.append(m)
    else:
        grow = np.abs(beta) > 0.15
        i = 0
        while i < len(t):
            if grow[i]:
                j = i
                while j < len(t) and abs(beta[j]) < 5.0 and (j == i or abs(beta[j]) >= abs(beta[i]) * 0.5):
                    j += 1
                if j - i > 30:
                    segs.append((np.arange(len(t)) >= i) & (np.arange(len(t)) < j))
                i = j
            else:
                i += 1
    if not segs:
        sys.exit("성장 구간을 찾지 못함 — 실험 중 m(놓음)·m(잡음)을 눌러 마커를 남겨라")
    print(f"[exp2] 시도 {len(segs)}개")

    # PCA → 불안정 방향 w (정규화 좌표에서 1주성분)
    X = np.vstack([np.concatenate([phi[m] for m in segs]),
                   np.concatenate([beta[m] for m in segs]),
                   np.concatenate([phid[m] for m in segs]),
                   np.concatenate([betad[m] for m in segs])]).T
    sd = X.std(axis=0); sd[sd == 0] = 1
    _, _, Vt = np.linalg.svd((X / sd) - (X / sd).mean(axis=0), full_matrices=False)
    w = Vt[0] / sd                                  # 물리 단위 좌고유벡터(비율만 의미)
    w = w / w[1] if abs(w[1]) > 1e-9 else w         # β성분=1 로 정규화 → A가 β-deg 단위
    lam_list = []
    for m in segs:
        a = np.abs(phi[m] * w[0] + beta[m] * w[1] + phid[m] * w[2] + betad[m] * w[3])
        ok = a > max(0.05, a.max() * 0.05)
        if ok.sum() > 15:
            c = np.polyfit(t[m][ok], np.log(a[ok]), 1)
            lam_list.append(c[0])
    lam = np.array(lam_list)
    print(f"\n=== λ_u = {lam.mean():.2f} ± {lam.std():.2f} /s  (시도별: {np.round(lam,2)}) ===")
    print(f"=== w (φ,β,φ̇,β̇; β=1 정규화) = {np.round(w,4)} ===")
    print("게이트: 모델 예측 λ_u(sim_check/params 기준, ~6.4/s)와 ±15% 이내인지 비교")
    print(f"exp4 에 쓸 인자:  --lam {lam.mean():.3f} --w {','.join(f'{v:.5f}' for v in w)}")


# ---------------- [실험 ③] 허리 관성·마찰 ----------------
def cmd_exp3(args):
    d = load(args.log)
    t, dl, vel, cur = d["t"], d["dl"], d["vel"], d["cur"]
    tau = cur / 1000.0 * args.kt                      # N·m
    # δ̈: PRESENT_VELOCITY(°/s) 미분(스무딩) → rad/s²
    k = np.ones(7) / 7
    acc = np.gradient(np.convolve(vel, k, "same"), t) * np.pi / 180.0
    veln = np.convolve(vel, k, "same") * np.pi / 180.0
    m = np.abs(veln) > 0.15                           # 정지 구간 제외
    A = np.vstack([acc[m], veln[m], np.sign(veln[m])]).T
    x, res, *_ = np.linalg.lstsq(A, tau[m], rcond=None)
    Iw, bv, cf = x
    pred = A @ x
    rms = np.sqrt(np.mean((tau[m] - pred) ** 2))
    print(f"\n=== 허리 유효관성 I_δ = {Iw*1e3:.2f}e-3 kg·m²  (armature 포함) ===")
    print(f"=== 점성 b = {bv*1e3:.2f}e-3 N·m·s/rad,  쿨롱 c = {cf*1e3:.1f}e-3 N·m ===")
    print(f"적합 RMS = {rms*1e3:.1f} mN·m ({m.sum()}점)")
    tau_avail = args.tau_avail
    print(f"→ 권장 Profile Acceleration ≈ 가용토크/I_δ = {tau_avail/Iw*180/np.pi:.0f} °/s²"
          f"  (가용토크 {tau_avail} N·m 가정, 8000과 비교)")
    print("params_v19 SERVO.* 갱신 후 compute_gains_v19 재실행")


# ---------------- [실험 ④] 사이클 지도 → Gc, gc, γ, γ(A), σ_A, 트리거 ----------------
def cmd_exp4(args):
    d = load(args.log)
    p1, p2, h = body_params(args)
    t, phi, beta, phid, betad = states(d, args, p1, p2, h)
    lam = args.lam
    if args.w:
        w = np.array([float(v) for v in args.w.split(",")])
    else:  # w 미지정 시 예측점 근사: A ∝ (β + β̇/λ) — 데모용, 실험②의 w 권장
        w = np.array([0.0, 1.0, 0.0, 1.0 / lam])
        print("[주의] --w 미지정 → A ≈ β + β̇/λ 근사 사용 (실험② w 권장)")
    A = phi * w[0] + beta * w[1] + phid * w[2] + betad * w[3]   # β-deg 단위

    tf, af = ev_times(d, "FOLD")      # 접기 시작 (인자 = 명령 접기각)
    te, _ = ev_times(d, "EDONE")      # 펴기 완료
    n = min(len(tf), len(te))
    rows = []
    for i in range(n):
        m_pre = (t > tf[i] - 0.06) & (t < tf[i])                 # 접기 직전 60ms
        m_post = (t > te[i] + 0.05) & (t < te[i] + 0.15)         # 휴지 후 100ms 창
        if m_pre.sum() < 3 or m_post.sum() < 3:
            continue
        rows.append((np.mean(A[m_pre]), af[i], np.mean(A[m_post])))
    if len(rows) < 6:
        sys.exit(f"유효 사이클 {len(rows)}개 — 최소 6개(권장 20개+) 필요")
    Am, df, Ap = map(np.array, zip(*rows))
    print(f"[exp4] 사이클 {len(rows)}개, |A-| 범위 {np.abs(Am).min():.2f}~{np.abs(Am).max():.2f}°")

    # 공선성 점검: δf 가 A- 에 단순 비례하면 Gc·gc 분리 불가
    r = np.corrcoef(Am, df)[0, 1]
    if abs(r) > 0.98:
        print(f"⚠ 공선성 경고: corr(A-, δf)={r:.3f} — 접기각을 기울기와 '독립적으로' 3단 이상 바꿔 재실험!")

    M = np.vstack([Am, -df]).T
    (Gc, gc), *_ = np.linalg.lstsq(M, Ap, rcond=None)
    resid = Ap - M @ np.array([Gc, gc])
    sig = resid.std()
    gam = Gc / gc
    print(f"\n=== Gc = {Gc:.2f},  gc = {gc:.4f},  γ = Gc/gc = {gam:.1f} ===")
    print(f"=== 잔여 산포 σ_A = {sig:.3f}° ===")
    print(f"=== 권장 트리거 = 1.2~1.5 × σ_A = {1.2*sig:.2f} ~ {1.5*sig:.2f}°  (현 설정 0.6°와 비교) ===")

    # γ(A) 진폭별 보간표: Gc 고정, 구간별 gc 재적합 → γ_bin
    bins = np.quantile(np.abs(Am), [0, 1/3, 2/3, 1.0])
    print("\n--- γ(A) 진폭별 보간표 (Gc 고정) ---")
    print(" |A-| 구간 [deg]    N    γ_bin")
    for lo, hi in zip(bins[:-1], bins[1:]):
        m = (np.abs(Am) >= lo) & (np.abs(Am) <= hi + 1e-9)
        if m.sum() >= 3 and np.abs(df[m]).sum() > 0:
            gcb = np.sum(df[m] * (Gc * Am[m] - Ap[m])) / np.sum(df[m] ** 2)
            print(f" {lo:5.2f} ~ {hi:5.2f}   {m.sum():3d}   {Gc/gcb:6.1f}")
    print("→ γ_bin 이 구간별로 5% 이상 다르면 펌웨어에 보간표로 탑재 (상수 γ 대체)")
    print("→ Gc 이론 점검: Gc ≟ e^{λ·(사이클시간)} — 로그의 FOLD~EDONE+휴지 시간으로 확인")


# ---------------- main ----------------
def main():
    ap = argparse.ArgumentParser(description="v19 자가 측정 실험 분석")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("backlash", "exp2", "exp3", "exp4"):
        s = sub.add_parser(name)
        s.add_argument("log")
        s.add_argument("--params", default=None, help="v19_bringup 경로 (params_v19 임포트)")
        s.add_argument("--p1", type=float); s.add_argument("--p2", type=float)
        s.add_argument("--h", type=float)
        s.add_argument("--alpha", default="ank-phi", choices=["ank-phi", "ank+phi", "ank"])
    sub.choices["exp3"].add_argument("--kt", type=float, default=1.304)
    sub.choices["exp3"].add_argument("--tau_avail", type=float, default=0.9)
    sub.choices["exp4"].add_argument("--lam", type=float, required=True, help="실험② λ_u [1/s]")
    sub.choices["exp4"].add_argument("--w", default=None, help="실험② w 4성분 'a,b,c,d'")
    args = ap.parse_args()
    dict(backlash=cmd_backlash, exp2=cmd_exp2, exp3=cmd_exp3, exp4=cmd_exp4)[args.cmd](args)


if __name__ == "__main__":
    main()
