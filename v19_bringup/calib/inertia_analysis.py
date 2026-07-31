# -*- coding: utf-8 -*-
"""
inertia_analysis.py — 허리 유효관성 I_delta 산출 (실험 ③)

입력: motor_inertia_test.ino 가 뱉은 CSV (serial_logger.py 로 저장)
      D,t_ms,delta_deg,cur_unit,phi_deg,ank_deg
      E,...,PULSE_START,<unit>,tau=<N.m>     ← 펄스 시작 마커
      E,...,PULSE_END,...                    ← 펄스 종료 마커

물리:
    I_delta * ddot(delta) = tau_cmd - m2*g*ell2*sin(delta) - tau_friction*sign(dot delta)

  각 펄스의 초기 구간을 2차식으로 적합해 alpha 를 얻고, 그 구간의 평균
  중력토크를 빼서 '순수 구동토크'를 만든 뒤, 여러 펄스의
      alpha  vs  (tau_cmd - tau_gravity)
  를 직선적합한다.  기울기 = 1/I_delta,  x절편 = 쿨롱 마찰토크.

사용:
    python inertia_analysis.py itest.csv
    python inertia_analysis.py itest.csv --m2 0.6406 --ell2 0.1917 --kt 1.304
    python inertia_analysis.py itest.csv --win 0.20      # 적합 구간 [s]
    python inertia_analysis.py itest.csv --nograv        # 힙축을 연직으로 세운 경우

주의:
  * 중력 보정은 '상체가 힙축 아래로 매달린' 자세(delta=0 = 연직 하향) 기준이다.
    힙 회전축을 연직으로 세워 중력토크가 0인 지그를 쓸 수 있으면 --nograv 로.
  * 적합 구간은 짧게(0.15~0.25 s). 길수록 중력·속도의존 마찰이 섞인다.
"""
import argparse
import math
import sys


def read_csv(path):
    rows, marks = [], []
    with open(path, encoding="utf-8", errors="ignore") as f:
        for line in f:
            s = line.strip().split(",")
            if not s:
                continue
            if s[0] == "D" and len(s) >= 4:
                try:
                    rows.append((int(s[1]) / 1000.0, float(s[2]), float(s[3])))
                except ValueError:
                    pass
            elif s[0] == "E" and len(s) >= 3:
                marks.append((int(s[1]) / 1000.0, ",".join(s[2:])))
    return rows, marks


def segments(rows, marks):
    """PULSE_START 마커마다 (tau_cmd, [(t,delta_rad)...]) 를 만든다."""
    starts = [(t, m) for t, m in marks if "PULSE_START" in m]
    ends = [t for t, m in marks if "PULSE_END" in m]
    out = []
    for i, (t0, m) in enumerate(starts):
        t_end = next((e for e in ends if e > t0), None)
        if t_end is None:
            t_end = starts[i + 1][0] if i + 1 < len(starts) else rows[-1][0]
        tau = None
        for tok in m.split(","):
            if tok.startswith("tau="):
                tau = float(tok[4:])
        if tau is None:                       # 옛 로그: unit 에서 환산
            try:
                tau = float(m.split(",")[1]) * 2.69e-3 * 1.304
            except Exception:
                continue
        seg = [(t - t0, math.radians(d)) for t, d, _ in rows if t0 <= t <= t_end]
        if len(seg) >= 8:
            out.append((tau, seg))
    return out


def fit_quad(seg, win):
    """delta(t) = a0 + a1 t + a2 t^2 최소제곱 -> alpha = 2*a2, 평균 delta"""
    pts = [(t, d) for t, d in seg if t <= win]
    n = len(pts)
    if n < 6:
        return None
    # 정규방정식 (3x3) 직접 풀이 — numpy 없이도 돌게
    S = [0.0] * 5
    T = [0.0] * 3
    for t, d in pts:
        for k in range(5):
            S[k] += t ** k
        for k in range(3):
            T[k] += d * t ** k
    A = [[S[0], S[1], S[2]],
         [S[1], S[2], S[3]],
         [S[2], S[3], S[4]]]
    # 가우스 소거
    M = [A[i][:] + [T[i]] for i in range(3)]
    for i in range(3):
        p = max(range(i, 3), key=lambda r: abs(M[r][i]))
        M[i], M[p] = M[p], M[i]
        if abs(M[i][i]) < 1e-18:
            return None
        for r in range(3):
            if r == i:
                continue
            f = M[r][i] / M[i][i]
            for c in range(i, 4):
                M[r][c] -= f * M[i][c]
    a = [M[i][3] / M[i][i] for i in range(3)]
    alpha = 2 * a[2]
    dmean = sum(d for _, d in pts) / n
    return alpha, dmean, n, pts[-1][0]


def impulse_fit(segs, m2, ell2, g=9.81):
    """충격량-운동량법: 펄스 전체에 대해 I·ω_end = ∫τ dt − mgl∫sinδ dt − τf·T·sgn.
    δ(t)에 지그·결합부 진동이 섞여도 적분 과정에서 대부분 상쇄된다.
    25° 컷으로 끝난 단조운동 펄스(T 0.15~0.8 s, |ω_end| 유의미)만 사용."""
    MGL = m2 * g * ell2
    rows = []
    for tau, seg in segs:
        T = seg[-1][0]
        tail = [(t, d) for t, d in seg if t >= T - 0.08]
        if len(tail) < 4:
            continue
        n = len(tail)
        mt = sum(t for t, _ in tail) / n
        md = sum(d for _, d in tail) / n
        sxx = sum((t - mt) ** 2 for t, _ in tail)
        if sxx == 0:
            continue
        w = sum((t - mt) * (d - md) for t, d in tail) / sxx
        if not (0.15 <= T <= 0.8) or abs(w) < 0.3:
            continue        # 중력 평형에 걸린 저토크 펄스 등은 제외
        Jg = 0.0
        for (t1, d1), (t2, d2) in zip(seg[:-1], seg[1:]):
            Jg += 0.5 * (math.sin(d1) + math.sin(d2)) * (t2 - t1)
        rows.append((tau, w, tau * T - MGL * Jg, T))
    if len(rows) < 3:
        return None
    # 최소제곱 2변수: J_net = I·w + tauf·T·sgn(tau)
    S11 = sum(r[1] ** 2 for r in rows)
    S12 = sum(r[1] * r[3] * (1 if r[0] > 0 else -1) for r in rows)
    S22 = sum(r[3] ** 2 for r in rows)
    b1 = sum(r[1] * r[2] for r in rows)
    b2 = sum(r[3] * (1 if r[0] > 0 else -1) * r[2] for r in rows)
    det = S11 * S22 - S12 * S12
    if abs(det) < 1e-12:
        return None
    I = (b1 * S22 - b2 * S12) / det
    tauf = (S11 * b2 - S12 * b1) / det
    ss = sum((r[2] - (I * r[1] + tauf * r[3] * (1 if r[0] > 0 else -1))) ** 2 for r in rows)
    my = sum(r[2] for r in rows) / len(rows)
    sst = sum((r[2] - my) ** 2 for r in rows)
    r2 = 1 - ss / sst if sst else 0.0
    return I, abs(tauf), r2, rows


def linfit(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx == 0:
        return None
    b = sxy / sxx
    a = my - b * mx
    ss_res = sum((y - (a + b * x)) ** 2 for x, y in zip(xs, ys))
    ss_tot = sum((y - my) ** 2 for y in ys)
    r2 = 1 - ss_res / ss_tot if ss_tot else 0.0
    return a, b, r2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--m2", type=float, default=0.6406, help="상체 링크 질량 [kg]")
    ap.add_argument("--ell2", type=float, default=0.1917, help="힙축->상체 CoM [m]")
    ap.add_argument("--kt", type=float, default=1.304, help="토크상수 [N.m/A] (로그의 tau= 를 우선 사용)")
    ap.add_argument("--win", type=float, default=0.20, help="적합 구간 [s]")
    ap.add_argument("--nograv", action="store_true", help="중력토크 0 지그 (힙축 연직)")
    ap.add_argument("--dir", choices=["both", "pos", "neg"], default="both",
                    help="분석할 펄스 방향 (기본 both). 한쪽이 이상할 때 pos/neg 로 분리 확인")
    ap.add_argument("--method", choices=["window", "impulse"], default="window",
                    help="window: 초기 0.2s 2차식 적합(기본). impulse: 펄스 전체 충격량-운동량 "
                         "(지그·결합부 진동이 δ(t)에 섞일 때 이쪽이 강건 — 진동이 적분되며 상쇄됨)")
    a = ap.parse_args()

    rows, marks = read_csv(a.csv)
    if not rows:
        print("D행이 없습니다. 로그 파일을 확인하세요."); sys.exit(1)
    segs = segments(rows, marks)
    if a.dir == "pos":
        segs = [(t, s) for t, s in segs if t > 0]
    elif a.dir == "neg":
        segs = [(t, s) for t, s in segs if t < 0]
    if len(segs) < 2:
        print(f"펄스 구간이 {len(segs)}개뿐입니다. 'i <unit>' 을 최소 4단계(예: 60/90/120/160/200) 실행하세요.")
        if not segs:
            sys.exit(1)

    if a.method == "impulse":
        r = impulse_fit(segs, a.m2, a.ell2)
        if r is None:
            print("impulse 법에 쓸 단조운동 펄스(0.15~0.8 s)가 3개 미만입니다."); sys.exit(1)
        I, tauf, r2, used = r
        print(f"{'tau_cmd':>9} {'T[s]':>6} {'w_end':>8} {'J_net':>9}")
        print("-" * 40)
        for tau, w, Jn, T in used:
            print(f"{tau:9.4f} {T:6.2f} {w:8.2f} {Jn:9.4f}")
        print("-" * 40)
        print(f"충격량 적합 (펄스 {len(used)}개)    R^2 = {r2:.4f}")
        print(f"\n  ★ I_delta   = {I:.5f} kg·m²   (armature 반영분 포함)")
        print(f"  ★ 마찰토크  = {tauf:.4f} N·m   (쿨롱 등가)")
        for cur, lbl in ((600, "CUR_SAFE 600u"), (855, "855u"), (1193, "하드상한 1193u")):
            tau = cur * 2.69e-3 * a.kt
            print(f"  권장 최대 각가속도 @{lbl}: {math.degrees(tau/I):8.0f} °/s²"
                  f"  (Profile Acceleration ≈ {math.degrees(tau/I)/21.4577:.0f} unit)")
        return

    g = 9.81
    xs, ys = [], []
    print(f"{'tau_cmd':>9} {'delta_avg':>10} {'tau_grav':>9} {'tau_net':>9} {'alpha':>9} {'n':>4}")
    print("-" * 56)
    for tau, seg in segs:
        r = fit_quad(seg, a.win)
        if r is None:
            continue
        alpha, dmean, n, tlast = r
        tg = 0.0 if a.nograv else a.m2 * g * a.ell2 * math.sin(dmean) * (1 if tau > 0 else -1)
        # 중력은 항상 복원 방향(원점으로) -> 구동 반대쪽으로 작용
        tnet = abs(tau) - abs(tg)
        print(f"{tau:9.4f} {math.degrees(dmean):9.2f}° {tg:9.4f} {tnet:9.4f} {abs(alpha):9.2f} {n:4d}")
        xs.append(tnet); ys.append(abs(alpha))

    if len(xs) < 2:
        print("\n적합 가능한 점이 부족합니다."); sys.exit(1)

    fit = linfit(xs, ys)
    a0, b, r2 = fit
    I = 1.0 / b if b else float("nan")
    tau_f = -a0 / b if b else float("nan")
    print("-" * 56)
    print(f"직선적합  alpha = {b:.3f} * tau_net + ({a0:.3f})    R^2 = {r2:.4f}")
    print(f"\n  ★ I_delta   = {I:.5f} kg·m²   (armature 반영분 포함)")
    print(f"  ★ 마찰토크  = {tau_f:.4f} N·m   (쿨롱 등가)")
    link = a.m2 * a.ell2 ** 2
    print(f"\n  참고: 상체 링크만의 힙축 관성(질량표 기준) = {link:.5f} + I2_cm")
    print(f"        차이가 크면 그 초과분이 회전자 armature 반영분입니다.")
    for cur, lbl in ((600, "CUR_SAFE 600u"), (855, "855u"), (1193, "하드상한 1193u")):
        tau = cur * 2.69e-3 * a.kt
        print(f"  권장 최대 각가속도 @{lbl}: {math.degrees(tau/I):8.0f} °/s²"
              f"  (Profile Acceleration ≈ {math.degrees(tau/I)/21.4577:.0f} unit)")


if __name__ == "__main__":
    main()
