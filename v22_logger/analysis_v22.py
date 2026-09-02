# -*- coding: utf-8 -*-
"""
analysis_v22.py — 측정 도구 (전부 "어떤 구간의 어떤 점으로 어떤 식을 적합했는가" 를 되돌려준다)
=============================================================================================
모든 도구는 dict 를 돌려준다:
    tool, ok, msg, params(쓴 값), window[t0,t1], used(쓴 표본 인덱스 구간들), n,
    steps(사람이 읽는 절차 설명), result(숫자), table(행 목록),
    overlay(스트립차트 위 그림: line/points/band/vline), plane(평면 위 그림), curves(작은 보조 그래프)
숫자는 numpy, 방법은 문서 70·64·53 의 오프라인 정본 스크립트(p2r_fit / lambda_fit / r_fit)와 같다.
"""
import math
import numpy as np

LN2 = math.log(2.0)


# ---------------------------------------------------------------- 공통
def _win(ds, t0, t1):
    t = ds.arr("t")
    if ds.n == 0:
        return t, 0, 0
    if t0 is None:
        t0 = float(t[0])
    if t1 is None:
        t1 = float(t[-1])
    i0 = int(np.searchsorted(t, t0, side="left"))
    i1 = int(np.searchsorted(t, t1, side="right"))
    return t, i0, i1


def _dec(t, y, cap=2500):
    n = len(t)
    if n <= cap:
        return [round(float(v), 5) for v in t], [None if not np.isfinite(v) else round(float(v), 5) for v in y]
    idx = np.linspace(0, n - 1, cap).astype(int)
    return [round(float(t[i]), 5) for i in idx], [None if not np.isfinite(y[i]) else round(float(y[i]), 5) for i in idx]


def _r(x, k=4):
    try:
        if x is None or not np.isfinite(x):
            return None
        return round(float(x), k)
    except TypeError:
        return None


def linreg(x, y):
    """y = a + b·x 최소제곱. (b, a, R², SE_b, SE_a, n, resid)"""
    x = np.asarray(x, float); y = np.asarray(y, float)
    n = len(x)
    if n < 2:
        return dict(b=float("nan"), a=float("nan"), r2=float("nan"), se_b=float("nan"),
                    se_a=float("nan"), n=n, resid=np.zeros(n))
    xm, ym = x.mean(), y.mean()
    sxx = ((x - xm) ** 2).sum()
    sxy = ((x - xm) * (y - ym)).sum()
    b = sxy / sxx if sxx > 0 else float("nan")
    a = ym - b * xm
    fit = a + b * x
    resid = y - fit
    sse = (resid ** 2).sum()
    sst = ((y - ym) ** 2).sum()
    r2 = 1 - sse / sst if sst > 0 else float("nan")
    se_b = math.sqrt(sse / (n - 2) / sxx) if n > 2 and sxx > 0 else float("nan")
    se_a = se_b * math.sqrt((x ** 2).sum() / n) if n > 2 and sxx > 0 else float("nan")
    return dict(b=b, a=a, r2=r2, se_b=se_b, se_a=se_a, n=n, resid=resid)


def _smooth(t, x, W):
    from dataset_v22 import centered_smooth_and_deriv
    if not W or W <= 0:
        return x.copy()
    return centered_smooth_and_deriv(t, x, W / 1000.0)[0]


# ---------------------------------------------------------------- 1. 구간 통계 / 잡음
def stats(ds, t0=None, t1=None, chans=None, hp_ms=100.0):
    t, i0, i1 = _win(ds, t0, t1)
    chans = chans or ["u_phi", "u_ank", "del", "a_alpha", "a_beta", "a_Ahat", "Ahat_fw"]
    rows = []
    for c in chans:
        y = ds.arr(c)[i0:i1]
        tt = t[i0:i1]
        m = np.isfinite(y)
        if m.sum() < 2:
            continue
        y, tt = y[m], tt[m]
        lr = linreg(tt, y)
        hp = y - _smooth(tt, y, hp_ms)
        rows.append(dict(ch=c, n=int(len(y)), mean=_r(y.mean()), std=_r(y.std(ddof=1)),
                         min=_r(y.min()), max=_r(y.max()), drift=_r(lr["b"]),
                         hp_rms=_r(np.sqrt((hp ** 2).mean()))))
    steps = [f"구간 [{t[i0]:.3f}, {t[max(i1-1,i0)]:.3f}] s 의 표본 {i1-i0} 개",
             "평균·표준편차(ddof=1)·최소·최대, 드리프트 = t 에 대한 직선 기울기",
             f"hp_rms = 중심 이동평균({hp_ms:.0f} ms)을 뺀 잔차의 rms — 잡음 바닥 (문서 54 방식)"]
    return dict(tool="stats", ok=True, window=[_r(t[i0]), _r(t[max(i1-1, i0)])], used=[[i0, i1]],
                n=i1 - i0, steps=steps, table=rows, result={}, overlay=[], plane=[], curves=[],
                params=dict(hp_ms=hp_ms, chans=chans))


# ---------------------------------------------------------------- 2. 선형 적합 (일반 / P2R)
def linfit(ds, t0=None, t1=None, xch="del", ych="a_alpha", origin=False):
    t, i0, i1 = _win(ds, t0, t1)
    x = ds.arr(xch)[i0:i1]; y = ds.arr(ych)[i0:i1]
    m = np.isfinite(x) & np.isfinite(y)
    x, y = x[m], y[m]
    if len(x) < 3:
        return dict(tool="linfit", ok=False, msg="표본 부족")
    lr = linreg(x, y)
    b0 = float((x * y).sum() / (x * x).sum()) if (x * x).sum() > 0 else float("nan")
    xs = np.linspace(x.min(), x.max(), 50)
    steps = [f"구간 표본 {len(x)} 개: x={xch}, y={ych}",
             "최소제곱 y = a + b·x,  b = Sxy/Sxx,  a = ȳ − b·x̄,  SE_b = √(SSE/(n−2)/Sxx)",
             f"원점 강제 기울기 b0 = Σxy/Σx² = {b0:.5f} (비교용)"]
    res = dict(slope=_r(lr["b"], 5), intercept=_r(lr["a"], 4), r2=_r(lr["r2"], 6), se_slope=_r(lr["se_b"], 5),
               se_intercept=_r(lr["se_a"], 4), n=int(lr["n"]), slope_origin=_r(b0, 5),
               resid_std=_r(np.std(lr["resid"], ddof=2) if lr["n"] > 2 else float("nan")))
    if xch == "del" and ych in ("a_alpha", "alpha_fw", "u_ank"):
        res["P2R"] = _r(-lr["b"], 5)
        steps.append("P2R = −b  (매달림 평형 β=0 ⇒ α = −P2R·δ, 문서 70 §3)")
    return dict(tool="linfit", ok=True, window=[_r(t[i0]), _r(t[max(i1-1, i0)])], used=[[i0, i1]],
                n=int(len(x)), steps=steps, result=res, table=[],
                params=dict(xch=xch, ych=ych),
                overlay=[], plane=[],
                curves=[dict(kind="xy", label=f"{ych} vs {xch}", x=[_r(v) for v in x[::max(1, len(x)//1500)]],
                             y=[_r(v) for v in y[::max(1, len(x)//1500)]],
                             fit_x=[_r(v) for v in xs], fit_y=[_r(lr["a"] + lr["b"] * v) for v in xs])])


def _segments_by_hold(ds, i0, i1, min_len_s):
    """hold(del_cmd) 가 일정한 평탄구간 [(ia, ib, hold), ...]"""
    h = ds.arr("hold")[i0:i1]
    t = ds.arr("t")[i0:i1]
    if not np.isfinite(h).any():
        return []
    segs = []
    a = 0
    for k in range(1, len(h) + 1):
        if k == len(h) or abs(h[k] - h[a]) > 0.05 or not np.isfinite(h[k]):
            if k - a > 1 and t[k - 1] - t[a] >= min_len_s and np.isfinite(h[a]):
                segs.append((i0 + a, i0 + k, float(h[a])))
            a = k
    return segs


def _segments_by_events(ds, i0, i1, min_len_s, names=("MOVE",)):
    t = ds.arr("t"); tms = ds.arr("t_ms")
    ev_t = []
    for e in ds.events:
        if e[1] in names:
            j = int(np.searchsorted(tms, e[0]))
            if 0 <= j < ds.n:
                ev_t.append(j)
    bounds = sorted(set([i0] + [j for j in ev_t if i0 < j < i1] + [i1]))
    segs = []
    for a, b in zip(bounds[:-1], bounds[1:]):
        if b - a > 1 and t[b - 1] - t[a] >= min_len_s:
            hv = ds.arr("hold")[a:b]
            segs.append((a, b, float(np.nanmean(hv)) if np.isfinite(hv).any() else float("nan")))
    return segs


def p2r_fit(ds, t0=None, t1=None, avg_s=2.0, seg_mode="auto", min_seg_s=1.5, xch="del", ych="a_alpha"):
    """실측① P2R — 문서 64 p2r_fit_0822.py 절차: 구간 자르기 → 각 구간 마지막 avg_s 평균 → α 대 δ 최소제곱."""
    t, i0, i1 = _win(ds, t0, t1)
    if i1 - i0 < 10:
        return dict(tool="p2r", ok=False, msg="표본 부족")
    mode = seg_mode
    segs = []
    if mode in ("auto", "hold"):
        segs = _segments_by_hold(ds, i0, i1, min_seg_s)
        if mode == "auto" and len(segs) < 3:
            segs = []
    if not segs and mode in ("auto", "events"):
        segs = _segments_by_events(ds, i0, i1, min_seg_s)
        if segs:
            mode = "events"
    else:
        mode = "hold" if segs else mode
    steps = []
    if len(segs) < 2:
        # 평탄구간을 못 나누면 전체 점별 적합으로 대체 (정직하게 표시)
        r = linfit(ds, t0, t1, xch, ych)
        if not r.get("ok"):
            return r
        r["tool"] = "p2r"
        r["steps"] = ["★평탄구간(hold 일정 / MOVE 이벤트)을 못 찾아 **전체 표본 점별 적합**으로 대체했다"] + r["steps"]
        r["result"]["P2R"] = _r(-r["result"]["slope"], 5)
        return r
    X = ds.arr(xch); Y = ds.arr(ych)
    pts = []
    for (a, b, hv) in segs:
        tb = t[b - 1]
        ja = int(np.searchsorted(t, tb - avg_s, side="left"))
        ja = max(ja, a)
        xs, ys = X[ja:b], Y[ja:b]
        m = np.isfinite(xs) & np.isfinite(ys)
        if m.sum() < 5:
            continue
        pts.append(dict(ia=int(ja), ib=int(b), hold=_r(hv, 3), t0=_r(t[ja], 3), t1=_r(tb, 3),
                        x=float(xs[m].mean()), y=float(ys[m].mean()), y_std=float(ys[m].std(ddof=1)),
                        x_std=float(xs[m].std(ddof=1)), n=int(m.sum())))
    if len(pts) < 2:
        return dict(tool="p2r", ok=False, msg="평균점 부족")
    x = np.array([p["x"] for p in pts]); y = np.array([p["y"] for p in pts])
    lr = linreg(x, y)
    P2R = -lr["b"]
    # 상행/하행 나누기 (이전 점보다 δ 가 커지면 상행)
    up = [k for k in range(1, len(pts)) if x[k] > x[k - 1] + 0.5]
    dn = [k for k in range(1, len(pts)) if x[k] < x[k - 1] - 0.5]
    res = dict(P2R=_r(P2R, 5), se=_r(lr["se_b"], 5), intercept=_r(lr["a"], 4), r2=_r(lr["r2"], 6), n_pts=len(pts),
               P2R_origin=_r(-(x * y).sum() / (x * x).sum(), 5) if (x * x).sum() > 0 else None)
    if len(up) >= 2:
        lu = linreg(x[up], y[up]); res["P2R_up"] = _r(-lu["b"], 5)
    if len(dn) >= 2:
        ld = linreg(x[dn], y[dn]); res["P2R_down"] = _r(-ld["b"], 5)
    # 인접차분 (오프셋 면역)
    adj = [-(y[k] - y[k - 1]) / (x[k] - x[k - 1]) for k in range(1, len(pts)) if abs(x[k] - x[k - 1]) > 2.0]
    if adj:
        res["P2R_adj_min"], res["P2R_adj_max"] = _r(min(adj), 5), _r(max(adj), 5)
    # 히스테리시스: 상행 적합선과 하행 적합선의 차 (δ = 20° 또는 δ 범위 중앙에서) — 문서 64 정의
    if len(up) >= 2 and len(dn) >= 2:
        lu = linreg(x[up], y[up]); ld = linreg(x[dn], y[dn])
        xm = 20.0 if x.min() <= 20.0 <= x.max() else float(0.5 * (x.min() + x.max()))
        res["hysteresis"] = _r(abs((lu["a"] + lu["b"] * xm) - (ld["a"] + ld["b"] * xm)), 4)
        res["hysteresis_at"] = _r(xm, 1)
    steps += [f"구간 나누기: {mode} 기준 {len(segs)} 구간 (최소 {min_seg_s} s)",
              f"각 구간 마지막 {avg_s} s 표본 평균 → (δ̄, ᾱ) 점 {len(pts)} 개 (표준편차는 표에)",
              "최소제곱 α = a₀ + b·δ,  P2R = −b,  SE = √(SSE/(n−2)/Sxx)",
              "교차검산: 상행만/하행만/원점강제/인접차분 (문서 64 §3 표와 같은 항목)",
              "★나누는 것은 명령각(hold)이 아니라 실제 δ(del_now) 다 — 처짐 6.8 % (문서 64 §4-3)"]
    for p in pts:
        p.update(x=_r(p["x"], 4), y=_r(p["y"], 4), y_std=_r(p["y_std"], 4), x_std=_r(p["x_std"], 4))
        p["resid"] = _r(p["y"] - (lr["a"] + lr["b"] * p["x"]), 4)
    xs = np.linspace(x.min(), x.max(), 20)
    overlay = [dict(kind="band", t0=p["t0"], t1=p["t1"], label=f"δ̄={p['x']}", color="rgba(6,214,160,.18)") for p in pts]
    return dict(tool="p2r", ok=True, window=[_r(t[i0]), _r(t[i1 - 1])], used=[[p["ia"], p["ib"]] for p in pts],
                n=int(sum(p["n"] for p in pts)), steps=steps, result=res, table=pts,
                params=dict(avg_s=avg_s, seg_mode=mode, min_seg_s=min_seg_s, xch=xch, ych=ych),
                overlay=overlay, plane=[],
                curves=[dict(kind="xy", label="ᾱ vs δ̄ (구간 평균점)", x=[_r(v) for v in x], y=[_r(v) for v in y],
                             fit_x=[_r(v) for v in xs], fit_y=[_r(lr["a"] + lr["b"] * v) for v in xs],
                             xlab="δ [°]", ylab="α [°]")])


# ---------------------------------------------------------------- 3. λ (발산율) 적합
def _rising_run(a, lo, hi, tol_frac=0.03, tol_abs=0.15):
    """|ψ| 배열에서 lo 를 위로 지나 hi 에 처음 닿기까지의 **상승 구간** (시작, 끝 exclusive, hi 도달여부, 최대).
       봉우리를 지나 내려가기 시작하면(잡힘·되돌아옴) 봉우리에서 끊는다 — '상승부만' (문서 70 §5)."""
    n = len(a)
    best = None
    k = 0
    while k < n:
        if a[k] >= lo and (k == 0 or a[k - 1] < lo):
            j = k
            peak = a[k]; jpeak = k
            reached = False
            end = None
            while j + 1 < n:
                j += 1
                if a[j] >= hi:
                    reached = True; end = j + 1; jpeak = j; peak = a[j]
                    break
                if a[j] < lo:
                    end = jpeak + 1
                    break
                if a[j] > peak:
                    peak = a[j]; jpeak = j
                elif a[j] < peak - max(tol_abs, tol_frac * peak):
                    end = jpeak + 1                      # 봉우리 지나 하강 → 상승부 끝
                    break
            if end is None:
                end = jpeak + 1
            cand = (k, end, reached, peak)
            if best is None or (reached and not best[2]) or (reached == best[2] and (end - k) > (best[1] - best[0])):
                best = cand
            k = max(j, k + 1)
        else:
            k += 1
    return best


def lambda_fit(ds, t0=None, t1=None, phi_eq=None, lo=2.0, hi=9.0, ch="u_phi", smooth_ms=0.0, levels=(2.0, 4.0, 8.0)):
    """실측③ λ — 문서 70 §5: ψ = φ − φ_eq, ln|ψ| 대 t 직선적합 (밴드 lo~hi, 상승부만). 관측 채널은 φ 하나."""
    t, i0, i1 = _win(ds, t0, t1)
    if i1 - i0 < 5:
        return dict(tool="lambda", ok=False, msg="표본 부족")
    if phi_eq is None:
        phi_eq = float(ds.pipe.get("phi_eq", 0.0))
    y = ds.arr(ch)[i0:i1]
    tt = t[i0:i1]
    ys = _smooth(tt, y, smooth_ms) if smooth_ms else y
    psi = ys - phi_eq
    a = np.abs(psi)
    run = _rising_run(a, lo, hi)
    if run is None:
        return dict(tool="lambda", ok=False, msg=f"|ψ| 가 {lo}° 를 위로 지나는 상승 구간이 없음 (φ_eq={phi_eq})",
                    window=[_r(tt[0]), _r(tt[-1])])
    k, e, reached, peak = run
    sgn = 1.0 if psi[e - 1] >= 0 else -1.0
    same = np.sign(psi[k:e]) == sgn
    idx = np.arange(k, e)[same & (a[k:e] > 0)]
    if len(idx) < 3:
        return dict(tool="lambda", ok=False, msg="부호가 일정한 상승 표본이 3개 미만")
    x = tt[idx]; ly = np.log(a[idx])
    lr = linreg(x, ly)
    lam = lr["b"]
    # 통과시각 (펌웨어 R행 방식 비교용): |ψ| = 2/4/8° 를 처음 넘는 시각 (선형보간)
    cross = {}
    for L in levels:
        j = None
        for q in range(k, e):
            if a[q] >= L:
                j = q; break
        if j is not None and j > 0 and a[j] != a[j - 1]:
            f = (L - a[j - 1]) / (a[j] - a[j - 1])
            cross[L] = float(tt[j - 1] + f * (tt[j] - tt[j - 1]))
        elif j is not None:
            cross[L] = float(tt[j])
    lam24 = LN2 / (cross[4.0] - cross[2.0]) if 2.0 in cross and 4.0 in cross and cross[4.0] > cross[2.0] else None
    lam48 = LN2 / (cross[8.0] - cross[4.0]) if 4.0 in cross and 8.0 in cross and cross[8.0] > cross[4.0] else None
    res = dict(lam=_r(lam, 4), se=_r(lr["se_b"], 4), T2_ms=_r(LN2 / lam * 1000, 1) if lam > 0 else None,
               r2=_r(lr["r2"], 5), n=int(len(idx)), dir=int(sgn), phi_eq=_r(phi_eq, 3),
               t_start=_r(tt[idx[0]], 4), t_end=_r(tt[idx[-1]], 4), reached_hi=bool(reached), peak=_r(peak, 3),
               lam24=_r(lam24, 3), lam48=_r(lam48, 3),
               t_cross={str(L): _r(v, 4) for L, v in cross.items()})
    steps = [f"채널 {ch}" + (f" (평활 {smooth_ms:.0f} ms)" if smooth_ms else " (평활 없음)") + f", ψ = φ − φ_eq, φ_eq = {phi_eq:.3f}°",
             f"밴드 |ψ| ∈ [{lo}, {hi}]°: |ψ| 가 {lo}° 를 위로 지난 뒤 {hi}° 에 " + ("닿을 때까지" if reached else f"못 닿고 최대 {peak:.2f}° 까지") + f" 의 상승 구간 → 표본 {len(idx)} 개, 방향 {'+' if sgn>0 else '−'}",
             "직선적합 ln|ψ| = a + λ·t  (최소제곱), SE_λ = √(SSE/(n−2)/Sxx), T₂ = ln2/λ",
             "비교: 펌웨어 R행 방식 lam24 = ln2/(t₄−t₂), lam48 = ln2/(t₈−t₄) (통과시각 선형보간)",
             "정본 대조: 문서 70 §5 λ = 5.44 ± 0.59 /s (φ_eq=+1.40°), 문서 70 §4 동정 5.66"]
    xs = np.linspace(tt[idx[0]], tt[idx[-1]], 60)
    fit = phi_eq + sgn * np.exp(lr["a"] + lam * xs)
    overlay = [dict(kind="line", ch=ch, t=[_r(v) for v in xs], y=[_r(v) for v in fit], label=f"λ={lam:.2f}", color="#ff006e"),
               dict(kind="hline", ch=ch, y=phi_eq + sgn * lo, label=f"|ψ|={lo}", color="rgba(255,190,11,.6)"),
               dict(kind="hline", ch=ch, y=phi_eq + sgn * hi, label=f"|ψ|={hi}", color="rgba(255,190,11,.6)"),
               dict(kind="hline", ch=ch, y=phi_eq, label="φ_eq", color="rgba(255,255,255,.4)"),
               dict(kind="points", ch=ch, t=[_r(v) for v in tt[idx][::max(1, len(idx)//300)]],
                    y=[_r(v) for v in ys[idx][::max(1, len(idx)//300)]], label="적합에 쓴 점", color="#ff006e")]
    for L, v in cross.items():
        overlay.append(dict(kind="vline", t=v, label=f"{L:g}°", color="rgba(255,255,255,.35)"))
    return dict(tool="lambda", ok=True, window=[_r(tt[0]), _r(tt[-1])], used=[[i0 + int(idx[0]), i0 + int(idx[-1]) + 1]],
                n=int(len(idx)), steps=steps, result=res, table=[],
                params=dict(phi_eq=phi_eq, lo=lo, hi=hi, ch=ch, smooth_ms=smooth_ms), overlay=overlay, plane=[],
                curves=[dict(kind="xy", label="ln|ψ| vs t", x=[_r(v) for v in x], y=[_r(v) for v in ly],
                             fit_x=[_r(v) for v in xs], fit_y=[_r(lr["a"] + lam * v) for v in xs], xlab="t [s]", ylab="ln|ψ|")])


# ---------------------------------------------------------------- 4. 시행 나누기
def _quiet_mask(phi, w, tol):
    """뒤쪽 w 표본 창의 최대−최소 < tol 이면 '정지' (창이 안 차는 앞부분은 False)."""
    n = len(phi)
    q = np.zeros(n, dtype=bool)
    if n < w or w < 2:
        return q
    from numpy.lib.stride_tricks import sliding_window_view
    win = sliding_window_view(phi, w)
    rng = win.max(axis=1) - win.min(axis=1)
    q[w - 1:] = rng < tol
    return q


def find_trials(ds, mode="auto", phi_eq=None, reldet=1.0, fcatch=8.5, quiet_s=0.5, quiet_tol=0.35, min_len_s=0.15,
                t0=None, t1=None, max_rise_s=2.0, min_peak=4.0, min_r2=0.9):
    """놓기 시행 자동 분할.
       mode auto: phase 열이 5(발산) 를 쓰면 phase 로, 아니면 rel(정지→이탈) 로.
       rel: quiet_s 동안 φ 의 최대−최소 < quiet_tol 이면 '손에 잡혀 정지' 로 보고, 그 정지 평균 φ_q 에서
            reldet 이상 벗어나는 순간이 놓기. ★놓기점 = 정지에서 벗어나기 시작한 표본 (손 뗀 순간의 자세).
            φ_q 가 0 이 아니어도 된다 — r 실험처럼 φ=±3° 에서 놓아도 잡는다.
       유효 판정 두 가지: dir_valid(방향이 확실: 놓기 뒤 max_rise_s 안에 min_peak 이상 벗어남) / lam_valid(+λ 적합 R²≥min_r2).
       놓기 경계(r·c₀)는 dir_valid, λ 평균·φ_eq·동정은 lam_valid 만 쓴다."""
    t, i0, i1 = _win(ds, t0, t1)
    if i1 - i0 < 10:
        return dict(tool="trials", ok=False, msg="표본 부족")
    if phi_eq is None:
        phi_eq = float(ds.pipe.get("phi_eq", 0.0))
    phi = ds.arr("u_phi"); ank = ds.arr("u_ank"); beta = ds.arr("a_beta"); A = ds.arr("a_Ahat")
    ph = ds.arr("phase")
    psi = phi - phi_eq
    trials = []                      # (rel, s0, e, phi_q)
    used_mode = mode
    if mode in ("auto", "phase") and np.isfinite(ph[i0:i1]).any() and np.any(ph[i0:i1] == 5):
        used_mode = "phase"
        k = i0
        while k < i1:
            if ph[k] == 5 and (k == i0 or ph[k - 1] != 5):
                e = k
                while e < i1 and ph[e] == 5:
                    e += 1
                rel = max(i0, k - 1)
                trials.append((rel, k, e, float(phi[rel])))
                k = e
            else:
                k += 1
    else:
        used_mode = "rel"
        dt = float(np.median(np.diff(t[i0:i1]))) if i1 - i0 > 2 else 0.01
        w = max(3, int(round(quiet_s / max(dt, 1e-4))))
        quiet = _quiet_mask(phi[i0:i1], w, quiet_tol)
        k = 0
        n = i1 - i0
        while k < n:
            if not quiet[k]:
                k += 1
                continue
            q_end = k
            while q_end + 1 < n and quiet[q_end + 1]:
                q_end += 1
            seg = phi[i0 + q_end - w + 1: i0 + q_end + 1]
            phi_q = float(seg.mean()); sig = float(seg.std())
            s0 = q_end + 1
            found = False
            while s0 < n:
                if abs(phi[i0 + s0] - phi_q) >= reldet:
                    found = True
                    break
                if quiet[s0] and (t[i0 + s0] - t[i0 + q_end]) > quiet_s:     # 다시 정지 — 놓기 아님
                    break
                s0 += 1
            if not found:
                k = max(s0, q_end + 1)
                continue
            thr = max(3.0 * sig, 0.15)
            rel = s0
            while rel - 1 > q_end - w and abs(phi[i0 + rel - 1] - phi_q) > thr:
                rel -= 1
            rel = max(rel - 1, 0)
            sgn = 1.0 if phi[i0 + s0] - phi_q >= 0 else -1.0
            e = s0; pk = abs(phi[i0 + s0] - phi_q)
            ended_quiet = False
            while e + 1 < n:
                e += 1
                d = phi[i0 + e] - phi_q; ad = abs(d); pk = max(pk, ad)
                if ad >= fcatch or (pk > 2 * reldet and ad < 0.5 * pk) or (np.sign(d) != sgn and ad > reldet):
                    break
                if quiet[e] and e - s0 >= w:          # 다른 자세로 옮겨 잡고 다시 정지 — 놓기가 아니라 이동
                    e = e - w + 1; ended_quiet = True
                    break
            if t[i0 + e] - t[i0 + rel] >= min_len_s:
                trials.append((i0 + rel, i0 + s0, i0 + e, phi_q))
            k = e if ended_quiet else e + 1
    rows = []
    for m, (rel, s0, e, phi_q) in enumerate(trials):
        ee = min(e, ds.n - 1)
        dep = phi[rel:ee + 1] - phi_q
        d = 1 if dep[-1] >= 0 else -1
        adep = np.abs(dep)
        pk = float(adep.max()) if len(adep) else 0.0
        reach = np.nonzero(adep >= min_peak)[0]
        t_reach = float(t[rel + reach[0]] - t[rel]) if len(reach) else None
        sub = lambda_fit(ds, float(t[rel]), float(t[ee]), phi_eq=phi_eq)
        dur = float(t[ee] - t[rel])
        why_dir, why_lam = [], []
        if pk < min_peak:
            why_dir.append(f"진폭 {pk:.1f}°<{min_peak}°")
        elif t_reach is not None and t_reach > max_rise_s:
            why_dir.append(f"느림 {t_reach:.1f}s>{max_rise_s}s")
        if not sub.get("ok"):
            why_lam.append("λ 적합 실패")
        elif sub["result"]["r2"] is not None and sub["result"]["r2"] < min_r2:
            why_lam.append(f"R² {sub['result']['r2']:.2f}<{min_r2}")
        dir_valid = not why_dir
        lam_valid = dir_valid and not why_lam
        rows.append(dict(k=m + 1, i0=int(rel), i1=int(e), t0=_r(t[rel], 3), t_thr=_r(t[s0], 3), t1=_r(t[ee], 3),
                         dur=_r(dur, 3), dir=d, dir_valid=dir_valid, valid=lam_valid, why=" · ".join(why_dir + why_lam),
                         phi_q=_r(phi_q, 3), phi0=_r(phi[rel], 3), ank0=_r(ank[rel], 3), beta0=_r(beta[rel], 3), A0=_r(A[rel], 3),
                         lam=sub["result"]["lam"] if sub.get("ok") else None,
                         lam_r2=sub["result"]["r2"] if sub.get("ok") else None,
                         lam_n=sub["result"]["n"] if sub.get("ok") else None,
                         peak=_r(pk, 2), t_reach=_r(t_reach, 3) if t_reach is not None else None))
    lam_p = [r["lam"] for r in rows if r["dir"] > 0 and r["lam"] and r["valid"]]
    lam_n = [r["lam"] for r in rows if r["dir"] < 0 and r["lam"] and r["valid"]]
    res = dict(n_trials=len(rows), n_dir_valid=int(sum(1 for r in rows if r["dir_valid"])),
               n_valid=int(sum(1 for r in rows if r["valid"])), mode=used_mode, phi_eq=_r(phi_eq, 3),
               lam_plus=_r(np.mean(lam_p), 3) if lam_p else None, n_plus=len(lam_p),
               lam_minus=_r(np.mean(lam_n), 3) if lam_n else None, n_minus=len(lam_n))
    if lam_p and lam_n:
        lo_, hi_ = min(res["lam_plus"], res["lam_minus"]), max(res["lam_plus"], res["lam_minus"])
        res["dir_split_pct"] = _r(100 * (hi_ - lo_) / lo_, 1) if lo_ > 0 else None
    steps = [f"모드 {used_mode}: " + ("phase==5(발산) 구간을 시행으로" if used_mode == "phase" else
             f"φ 가 {quiet_s}s 동안 {quiet_tol}° 안에 머물면 정지(손에 잡힘)로 보고, 정지 평균 φ_q 에서 {reldet}° 벗어나는 순간 = 놓기, {fcatch}° 이상 또는 되돌아오면 종료"),
             "★놓기점 (φ₀, ank₀, β₀, Â₀) = 정지에서 벗어나기 시작한 표본 — 손 뗀 순간의 자세 (t0). t_thr = 문턱 통과 시각",
             f"방향 유효(dir_valid): 놓기 뒤 {max_rise_s}s 안에 |φ−φ_q| ≥ {min_peak}° — 놓기 경계(r·c₀)는 이것만 본다",
             f"λ 유효(valid): 방향 유효 + λ 적합 R² ≥ {min_r2} — 방향별 λ 평균·φ_eq 훑기는 이것만 쓴다",
             "시행별 λ = lambda 도구(밴드 2~9°, 같은 φ_eq). 방향별 평균이 20 % 넘게 갈리면 φ_eq 의심 (문서 79 §3)"]
    overlay = []
    for r in rows:
        col = ("rgba(255,0,110,.10)" if r["dir"] > 0 else "rgba(58,134,255,.12)") if r["dir_valid"] else "rgba(255,255,255,.06)"
        overlay.append(dict(kind="band", t0=r["t0"], t1=r["t1"], label=f"시행 {r['k']} {'+' if r['dir']>0 else '−'}" + ("" if r["dir_valid"] else " ✗"), color=col))
        overlay.append(dict(kind="vline", t=r["t0"], label=f"놓기 {r['k']}", color="rgba(255,209,102,.8)"))
    vr = [r for r in rows if r["dir_valid"]]
    plane = [dict(kind="points", plane="pl1", x=[r["beta0"] for r in vr], y=[r["phi0"] for r in vr],
                  dir=[r["dir"] for r in vr], label="놓기점 (β₀, φ₀) 색=낙하 방향")]
    return dict(tool="trials", ok=True, window=[_r(t[i0]), _r(t[i1 - 1])], used=[[r["i0"], r["i1"]] for r in rows],
                n=len(rows), steps=steps, result=res, table=rows,
                params=dict(mode=used_mode, phi_eq=phi_eq, reldet=reldet, fcatch=fcatch, quiet_s=quiet_s, quiet_tol=quiet_tol,
                            max_rise_s=max_rise_s, min_peak=min_peak, min_r2=min_r2),
                overlay=overlay, plane=plane, curves=[])


def phi_eq_scan(ds, trials=None, lo=2.0, hi=9.0, grid_lo=-4.0, grid_hi=4.0, step=0.05, **kw):
    """φ_eq 훑기: 방향별 λ 가 일치하는 φ_eq 를 찾는다 (문서 70 §5 — '+낙하 3.94 / −낙하 7.53 → 1.40° 에서 만난다')."""
    tr = trials
    if tr is None:
        tres = find_trials(ds, **kw)
        if not tres.get("ok"):
            return tres
        tr = [r for r in tres["table"] if r.get("valid", True)]
    t = ds.arr("t")
    xs = np.arange(grid_lo, grid_hi + 1e-9, step)
    lp, ln_ = [], []
    for pe in xs:
        a, b = [], []
        for r in tr:
            sub = lambda_fit(ds, float(t[r["i0"]]) - 0.05, float(t[min(r["i1"], ds.n - 1)]), phi_eq=float(pe), lo=lo, hi=hi)
            if sub.get("ok") and sub["result"]["lam"] and sub["result"]["n"] >= 5:
                (a if sub["result"]["dir"] > 0 else b).append(sub["result"]["lam"])
        lp.append(np.mean(a) if a else np.nan)
        ln_.append(np.mean(b) if b else np.nan)
    lp, ln_ = np.array(lp), np.array(ln_)
    diff = lp - ln_
    best = None
    for k in range(1, len(xs)):
        if np.isfinite(diff[k - 1]) and np.isfinite(diff[k]) and np.sign(diff[k - 1]) != np.sign(diff[k]):
            f = diff[k - 1] / (diff[k - 1] - diff[k])
            pe = xs[k - 1] + f * (xs[k] - xs[k - 1])
            lam = lp[k - 1] + f * (lp[k] - lp[k - 1])
            best = (float(pe), float(lam))
            break
    if best is None and np.isfinite(diff).any():
        k = int(np.nanargmin(np.abs(diff)))
        best = (float(xs[k]), float(np.nanmean([lp[k], ln_[k]])))
    res = dict(phi_eq_best=_r(best[0], 3) if best else None, lam_at_best=_r(best[1], 3) if best else None,
               n_trials=len(tr), n_plus=int(sum(1 for r in tr if r["dir"] > 0)), n_minus=int(sum(1 for r in tr if r["dir"] < 0)))
    steps = [f"시행 {len(tr)} 개 각각에 대해 φ_eq 를 {grid_lo}~{grid_hi}° ({step}° 간격) 로 바꿔가며 λ 를 다시 적합",
             "방향별 평균 λ+(φ_eq), λ−(φ_eq) 곡선 → 두 곡선이 만나는 φ_eq 가 평형점 (선형보간)",
             "★방향을 섞어 놓은 시행이 없으면 이 도구는 답을 못 낸다 (문서 70 §5: 4회 모두 같은 방향이 φ_eq 를 숨겼다)"]
    return dict(tool="phi_eq", ok=best is not None, msg=None if best else "방향별 λ 곡선이 만나지 않음 (양방향 시행 필요)",
                window=None, used=[], n=len(tr), steps=steps, result=res, table=[],
                params=dict(lo=lo, hi=hi, grid=[grid_lo, grid_hi, step]), overlay=[], plane=[],
                curves=[dict(kind="lines", label="λ vs φ_eq", x=[_r(v, 3) for v in xs],
                             series=[dict(label="λ+ (＋낙하)", y=[_r(v) for v in lp], color="#ff006e"),
                                     dict(label="λ− (−낙하)", y=[_r(v) for v in ln_], color="#3a86ff")],
                             xlab="φ_eq [°]", ylab="λ [1/s]", vline=best[0] if best else None)])


# ---------------------------------------------------------------- 5. 감쇠 진동 적합 (ω, ζ)
def osc_fit(ds, t0=None, t1=None, ch="u_phi", smooth_ms=0.0, refine=True, I_r=4.5e-3):
    t, i0, i1 = _win(ds, t0, t1)
    if i1 - i0 < 20:
        return dict(tool="osc", ok=False, msg="표본 부족")
    tt = t[i0:i1]; y0 = ds.arr(ch)[i0:i1]
    m = np.isfinite(y0)
    tt, y0 = tt[m], y0[m]
    y = _smooth(tt, y0, smooth_ms) if smooth_ms else y0.copy()
    lr = linreg(tt, y)                     # 선형 추세(평형 오프셋+드리프트) 제거
    c = lr["a"] + lr["b"] * tt
    x = y - c
    # 영점 교차
    s = np.sign(x)
    zc = []
    for k in range(1, len(x)):
        if s[k - 1] != 0 and s[k] != 0 and s[k - 1] != s[k]:
            f = x[k - 1] / (x[k - 1] - x[k])
            zc.append((float(tt[k - 1] + f * (tt[k] - tt[k - 1])), int(s[k])))
    if len(zc) < 3:
        return dict(tool="osc", ok=False, msg="영점 교차 3회 미만 — 진동이 아니다")
    # 봉우리: 인접 교차 사이의 극값
    peaks = []
    for (ta, sa), (tb, sb) in zip(zc[:-1], zc[1:]):
        ia = int(np.searchsorted(tt, ta)); ib = int(np.searchsorted(tt, tb))
        if ib - ia < 2:
            continue
        seg = x[ia:ib]
        j = ia + (int(np.argmax(seg)) if sa > 0 else int(np.argmin(seg)))
        peaks.append((float(tt[j]), float(x[j])))
    # 주기: 같은 방향 교차 간격
    T_list = [zc[k + 2][0] - zc[k][0] for k in range(len(zc) - 2)]
    T = float(np.mean(T_list)); T_sd = float(np.std(T_list, ddof=1)) if len(T_list) > 1 else float("nan")
    # 대수감쇠율: 반주기 간 |봉우리| 비 → 한 주기 δ
    amps = np.array([abs(p[1]) for p in peaks])
    dh = [math.log(amps[k] / amps[k + 1]) for k in range(len(amps) - 1) if amps[k + 1] > 1e-6 and amps[k] > 1e-6]
    delta = 2.0 * float(np.mean(dh)) if dh else float("nan")
    zeta = delta / math.sqrt(4 * math.pi ** 2 + delta ** 2) if np.isfinite(delta) else float("nan")
    wd = 2 * math.pi / T
    wn = wd / math.sqrt(1 - zeta ** 2) if np.isfinite(zeta) and abs(zeta) < 1 else wd
    res = dict(T=_r(T, 4), T_sd=_r(T_sd, 4), omega_d=_r(wd, 4), log_dec=_r(delta, 4), zeta=_r(zeta, 5),
               omega_n=_r(wn, 4), n_cross=len(zc), n_peaks=len(peaks), A0=_r(amps[0], 3) if len(amps) else None,
               offset=_r(lr["a"] + lr["b"] * tt.mean(), 3), drift=_r(lr["b"], 4),
               c_phi_est=_r(2 * zeta * wn * I_r, 6) if np.isfinite(zeta) else None)
    steps = [f"채널 {ch}, 표본 {len(x)} 개, 직선 추세(오프셋·드리프트) 제거",
             f"영점 교차 {len(zc)} 회 (선형보간) → 같은 방향 교차 간격 평균 = 주기 T = {T:.4f} s, ω_d = 2π/T",
             f"교차 사이 극값 = 봉우리 {len(peaks)} 개 → 반주기 진폭비의 ln 평균 ×2 = 대수감쇠율 δ, ζ = δ/√(4π²+δ²), ω_n = ω_d/√(1−ζ²)",
             f"c_φ 추정 = 2ζω_n·I_r  (I_r = {I_r:g} kg·m², params_v19 실측)  — 문서 16 방식과 같은 정의"]
    overlay = [dict(kind="points", ch=ch, t=[_r(p[0]) for p in peaks], y=[_r(p[1] + (lr['a'] + lr['b'] * p[0])) for p in peaks], label="봉우리", color="#ffbe0b")]
    for zt, _ in zc:
        overlay.append(dict(kind="vline", t=_r(zt), label="", color="rgba(255,255,255,.18)"))
    curves = [dict(kind="xy", label="ln|봉우리| vs t", x=[_r(p[0]) for p in peaks], y=[_r(math.log(abs(p[1]))) for p in peaks if abs(p[1]) > 1e-6],
                   xlab="t [s]", ylab="ln|A|")]
    if refine:
        try:
            from scipy.optimize import curve_fit
            def model(tq, A, sig, w, ph, cc):
                return A * np.exp(-sig * (tq - tt[0])) * np.cos(w * (tq - tt[0]) + ph) + cc
            p0 = [amps[0] if len(amps) else x.std() * 1.4, zeta * wn if np.isfinite(zeta) else 0.05, wd, 0.0, 0.0]
            popt, _ = curve_fit(model, tt, x, p0=p0, maxfev=20000)
            A, sig, w, ph, cc = popt
            fit = model(tt, *popt) + c
            r2 = 1 - ((x - model(tt, *popt)) ** 2).sum() / ((x - x.mean()) ** 2).sum()
            zeta2 = sig / math.sqrt(sig ** 2 + w ** 2)
            wn2 = math.sqrt(sig ** 2 + w ** 2)
            res.update(refine=dict(A=_r(A, 4), sigma=_r(sig, 5), omega_d=_r(w, 4), zeta=_r(zeta2, 5), omega_n=_r(wn2, 4),
                                   r2=_r(r2, 5), c_phi_est=_r(2 * zeta2 * wn2 * I_r, 6)))
            steps.append("정밀화: A·e^(−σt)·cos(ω_d t + φ) + c 비선형 최소제곱 (scipy curve_fit, 초기값 = 위 봉우리 추정)")
            dt_, fy = _dec(tt, fit)
            overlay.append(dict(kind="line", ch=ch, t=dt_, y=fy, label="감쇠진동 적합", color="#ff006e"))
        except Exception as ex:      # 수렴 실패는 정직하게
            steps.append(f"정밀화 실패: {ex}")
    return dict(tool="osc", ok=True, window=[_r(tt[0]), _r(tt[-1])], used=[[i0, i1]], n=int(len(x)), steps=steps,
                result=res, table=[dict(k=k + 1, t=_r(p[0]), amp=_r(p[1], 4)) for k, p in enumerate(peaks)],
                params=dict(ch=ch, smooth_ms=smooth_ms, I_r=I_r), overlay=overlay, plane=[], curves=curves)


# ---------------------------------------------------------------- 6. 놓기 경계 (r, c₀) — 문서 70 §4-2 경로②
def boundary_fit(ds, trials=None, r_fixed=None, grid_lo=-3.0, grid_hi=-0.8, step=0.01, **kw):
    tr = trials
    if tr is None:
        tres = find_trials(ds, **kw)
        if not tres.get("ok"):
            return tres
        tr = [r for r in tres["table"] if r.get("dir_valid", r.get("valid", True))]
    pts = [(r["phi0"], r["beta0"], r["dir"]) for r in tr if r.get("phi0") is not None and r.get("beta0") is not None]
    if len(pts) < 3:
        return dict(tool="boundary", ok=False, msg="놓기점 3개 미만")
    phi0 = np.array([p[0] for p in pts]); beta0 = np.array([p[1] for p in pts]); d = np.array([p[2] for p in pts])
    # (a) 놓기점 자체의 회귀: β = s·φ + b → r = 1/s  (놓은 자리가 경계 근처일 때)
    lr = linreg(phi0, beta0)
    corr = float(np.corrcoef(phi0, beta0)[0, 1]) if len(pts) > 2 else float("nan")
    r_reg = 1.0 / lr["b"] if abs(lr["b"]) > 1e-9 else float("nan")

    def separate(rv):
        c = phi0 - rv * beta0                      # 선 φ = r·β + c 의 각 점 절편
        order = np.argsort(c)
        cs, ds_ = c[order], d[order]
        best = None
        for k in range(len(cs) + 1):               # 문턱 앞쪽(k개)이 한 방향, 뒤가 반대
            for sgn in (1, -1):
                err = int((ds_[:k] != -sgn).sum() + (ds_[k:] != sgn).sum())
                if k == 0:
                    cth = cs[0] - 0.5; gap = 0.0
                elif k == len(cs):
                    cth = cs[-1] + 0.5; gap = 0.0
                else:
                    cth = 0.5 * (cs[k - 1] + cs[k]); gap = cs[k] - cs[k - 1]
                key = (err, -gap)
                if best is None or key < best[0]:
                    best = (key, float(cth), sgn, float(gap))
        return best[0][0], best[1], best[2], best[3]

    have_both = (d > 0).any() and (d < 0).any()
    rows = []
    r_use = r_fixed if r_fixed is not None else float(ds.pipe.get("r", -1.506))
    err, c0, sgn, gap = separate(r_use)
    res = dict(n=len(pts), r_reg=_r(r_reg, 4), slope_reg=_r(lr["b"], 4), corr=_r(corr, 4),
               r_used=_r(r_use, 4), c0=_r(c0, 3), errors=int(err), margin=_r(gap, 3),
               plus_side="c_i > c0" if sgn > 0 else "c_i < c0", have_both_dirs=bool(have_both))
    curves = []
    if have_both:
        xs = np.arange(grid_lo, grid_hi + 1e-9, step)
        errs, gaps, cs = [], [], []
        for rv in xs:
            e_, c_, _, g_ = separate(float(rv))
            errs.append(e_); gaps.append(g_); cs.append(c_)
        errs = np.array(errs); gaps = np.array(gaps)
        kbest = int(np.lexsort((-gaps, errs))[0])
        res.update(r_grid_best=_r(xs[kbest], 3), c0_grid_best=_r(cs[kbest], 3), errors_grid=int(errs[kbest]),
                   margin_grid=_r(gaps[kbest], 3))
        curves.append(dict(kind="lines", label="분리 오차수·여유 vs r", x=[_r(v, 3) for v in xs],
                           series=[dict(label="오분류 수", y=[int(v) for v in errs], color="#ff006e"),
                                   dict(label="여유(margin) [°]", y=[_r(v) for v in gaps], color="#06d6a0")],
                           xlab="r", ylab="", vline=float(xs[kbest])))
    for (p, b, dd), cc in zip(pts, phi0 - r_use * beta0):
        rows.append(dict(phi0=_r(p, 3), beta0=_r(b, 3), dir=int(dd), c_i=_r(cc, 3)))
    steps = [f"놓기점 {len(pts)} 개 (φ₀, β₀ = 손 뗀 순간의 자세) 와 낙하 방향 — 미분·모델 미사용 (문서 70 §4-2 경로②). 방향 유효 시행만",
             f"(a) 놓기점 회귀 β₀ = s·φ₀ + b: s = {lr['b']:.4f}, 상관 {corr:.3f} → r = 1/s = {r_reg:.3f}  (놓은 자리가 경계 근처일 때만 의미)",
             f"(b) 기울기 r = {r_use:.4f} 고정, 각 점의 절편 c_i = φ₀ − r·β₀ 를 정렬 → 방향이 갈리는 문턱 = c₀ (오분류 {err}, 여유 {gap:.3f}°)",
             "(c) r 를 격자로 훑어 오분류 최소·여유 최대인 r 도 함께 (양방향 시행이 있을 때만)",
             "정본 대조: 문서 70 r = −1.506 ± 0.074, c₀ = −1.11 ± 0.86 (미확정 — 같은 φ 에서 ank ±3° 흩기 필요)"]
    bl, br_ = float(beta0.min()) - 1, float(beta0.max()) + 1
    plane = [dict(kind="points", plane="pl1", x=[_r(v) for v in beta0], y=[_r(v) for v in phi0], dir=[int(v) for v in d], label="놓기점"),
             dict(kind="line", plane="pl1", x=[bl, br_], y=[r_use * bl + c0, r_use * br_ + c0], label=f"경계 φ={r_use:.3f}β+{c0:.2f}", color="#ff9f43")]
    return dict(tool="boundary", ok=True, window=None, used=[], n=len(pts), steps=steps, result=res, table=rows,
                params=dict(r_fixed=r_fixed, grid=[grid_lo, grid_hi, step]), overlay=[], plane=plane, curves=curves)


# ---------------------------------------------------------------- 7. 시스템 동정 (4×4) — 문서 70 §4-2 경로①
def sysid(ds, windows=None, phi_max=5.0, smooth_ms=120.0, poly=3, t0=None, t1=None, **kw):
    from scipy.signal import savgol_filter
    t = ds.arr("t")
    if windows is None:
        if t0 is not None or t1 is not None:
            windows = [[t0, t1]]
        else:
            tres = find_trials(ds, **kw)
            if not tres.get("ok") or not tres["table"]:
                return dict(tool="sysid", ok=False, msg="시행을 못 찾음 — 구간을 직접 지정")
            windows = [[r["t0"], r["t1"]] for r in tres["table"] if r.get("valid", True)]
    phi_all = ds.arr("u_phi"); beta_all = ds.arr("a_beta")
    X, Y, used, n_raw = [], [], [], 0
    for (a, b) in windows:
        _, i0, i1 = _win(ds, a, b)
        if i1 - i0 < 12:
            continue
        tt = t[i0:i1]
        dt = float(np.median(np.diff(tt)))
        if dt <= 0:
            continue
        win = max(int(round(smooth_ms / 1000.0 / dt)) | 1, poly + 2)
        if win > i1 - i0:
            continue
        cols = []
        for src in (phi_all, beta_all):
            y = src[i0:i1]
            cols.append([savgol_filter(y, win, poly, deriv=k, delta=dt, mode="interp") for k in range(3)])
        (p, pd, pdd), (bb, bd, bdd) = cols
        m = np.abs(p) < phi_max
        n_raw += int(m.sum())
        X.append(np.column_stack([p[m], bb[m], pd[m], bd[m], np.ones(m.sum())]))
        Y.append(np.column_stack([pdd[m], bdd[m]]))
        used.append([i0, i1])
    if not X:
        return dict(tool="sysid", ok=False, msg="유효 표본 없음")
    X = np.vstack(X); Y = np.vstack(Y)
    if len(X) < 30:
        return dict(tool="sysid", ok=False, msg=f"표본 {len(X)} 개 — 부족")
    coef, *_ = np.linalg.lstsq(X, Y, rcond=None)      # 5×2
    fit = X @ coef
    r2 = [1 - ((Y[:, k] - fit[:, k]) ** 2).sum() / ((Y[:, k] - Y[:, k].mean()) ** 2).sum() for k in range(2)]
    F = np.zeros((4, 4)); F[0, 2] = 1; F[1, 3] = 1
    F[2, :] = coef[:4, 0]; F[3, :] = coef[:4, 1]
    g = np.array([0, 0, coef[4, 0], coef[4, 1]])
    ev, V = np.linalg.eig(F)
    kmax = int(np.argmax(ev.real))
    lam = ev[kmax]
    evl, W = np.linalg.eig(F.T)                       # 좌고유벡터 = Fᵀ 의 우고유벡터
    kl = int(np.argmin(np.abs(evl - lam)))
    w = W[:, kl].real
    if abs(w[1]) < 1e-12:
        return dict(tool="sysid", ok=False, msg="좌고유벡터 w_β ≈ 0")
    w = w / w[1]                                      # w_β = 1 정규화
    lam_r = float(lam.real)
    r = -1.0 / w[0] if abs(w[0]) > 1e-12 else float("nan")
    A_off = float(w @ g) / lam_r if abs(lam_r) > 1e-9 else float("nan")
    c0 = A_off * r
    res = dict(lam=_r(lam_r, 4), lam_imag=_r(float(lam.imag), 4), r=_r(r, 4), c0=_r(c0, 3), A_offset=_r(A_off, 4),
               w=[_r(v, 5) for v in w], wf=_r(w[2], 5), wb=_r(w[3], 5), r2_phidd=_r(r2[0], 4), r2_betadd=_r(r2[1], 4),
               n=int(len(X)), n_windows=len(used),
               eig=[[_r(e.real, 4), _r(e.imag, 4)] for e in ev],
               coef=dict(phidd=[_r(v, 5) for v in coef[:, 0]], betadd=[_r(v, 5) for v in coef[:, 1]]))
    steps = [f"자유비행 창 {len(used)} 개, Savitzky–Golay({smooth_ms:.0f} ms, {poly}차)로 φ, β 와 1·2차 미분, |φ| < {phi_max}° 표본 {len(X)} 개",
             "(φ̈, β̈) 를 (φ, β, φ̇, β̇, 1) 로 최소제곱 회귀 → 4×4 시스템행렬 F 와 상수 g",
             "F 의 고유값 중 실수부 최대 = λ (불안정), Fᵀ 의 그 고유벡터 = 좌고유벡터 w, w_β = 1 정규화",
             "r = −1/w_φ,  w_φ̇ w_β̇ = 속도 가중(wf, wb),  A_offset = (w·g)/λ,  c₀ = A_offset·r  (문서 70 §4·§7)",
             "정본 대조: r = −1.506 ± 0.074, λ = 5.66 ± 0.09, 고유값 +5.66 / −0.19±5.32j / −3.25, 선형영역 |φ|<5° (§4-4)"]
    return dict(tool="sysid", ok=True, window=None, used=used, n=int(len(X)), steps=steps, result=res, table=[],
                params=dict(phi_max=phi_max, smooth_ms=smooth_ms, poly=poly, windows=windows), overlay=[], plane=[], curves=[])


# ---------------------------------------------------------------- 디스패치
TOOLS = dict(stats=stats, linfit=linfit, p2r=p2r_fit, **{"lambda": lambda_fit}, trials=find_trials,
             phi_eq=phi_eq_scan, osc=osc_fit, boundary=boundary_fit, sysid=sysid)


def run(ds, tool, args=None):
    fn = TOOLS.get(tool)
    if fn is None:
        return dict(tool=tool, ok=False, msg=f"모르는 도구: {tool}")
    args = dict(args or {})
    try:
        out = fn(ds, **args)
    except Exception as ex:          # 분석 실패는 앱을 죽이지 않고 메시지로
        import traceback
        out = dict(tool=tool, ok=False, msg=f"{type(ex).__name__}: {ex}", trace=traceback.format_exc()[-800:])
    out.setdefault("args", args)
    return out
