# -*- coding: utf-8 -*-
"""문서 59 '접기 스파이크 = 물리' 재검증.
질문: 예측점 평면에서 참값 β_pred 가 접기 순간 -a -> +a 로 '정확히' 반전하는가?
  H1(물리): 반전은 β̇ 킥(δ̇ 유래)의 통과 현상 — 반전 전후 크기는 비대칭·산포.
  H2(버그): 어떤 프레임이 β_pred 대신 -β_pred 를 그린다 — 반전 전후 크기가 정확히 대칭(비율 -1).
검증 4종:
  [1] 재현: fold/REST 위상에서만 부호반전이 나는가 (문서 59: 79건/8s, idle 0건)
  [2] 원인분해: β_pred 에서 '직접 δ̇ 항'(P11·p2r·δ̇)을 빼면 반전이 사라지는가
  [3] 거울검사: 반전 전후 비율 after/before 의 분포 — -1 에 몰리면 H2, 산포하면 H1
  [4] 독립검산: MuJoCo 질량가중 CoM 수평위치로 x_CoM ≈ R·φ + h·β 확인 (β 정의·부호 무결)
"""
import sys, os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = "/home/user/slackline-balacing1"
sys.path.insert(0, os.path.join(REPO, "v19_bringup"))
sys.path.insert(0, os.path.join(REPO, "v21_pres"))
from sim_engine import SimEngine, REAL_DEFAULT, PH_IDLE, PH_FOLD, PH_REST
import params_v19 as P


def make_engine():
    # server.py 'fwe_async_preset_measured' 와 동일 (지터만 0 으로 고정 — 결정론)
    e = SimEngine()
    e.nominal = dict(P.MEASURED_SET); e.nominal["RHO"] = 0.95; e.nominal["DT"] = 0.005
    e.mismatch = dict(m1=0, m2=0, L1=0, L2=0, R=0, kt=0)
    e.real = dict(REAL_DEFAULT)
    e.real.update(enc_use_gyro=False, fwe_settle_s=0.0, fwe_adapt_T=False, fwe_ff=True,
                  fwe_fast_L=False, w_noload_dps=462.0, fwe_pos_mode=True, pos_vel_dps=420.0,
                  pos_acc_dps2=8000.0, fwe_async=True, fwe_async_gamma=10.0,
                  fwe_async_vret_dps=3.0, fwe_async_hold_min_deg=1.0, fwe_online=False,
                  enc_diff_ms=25.0, enc_tau_v=0.030, **P.MEASURED_REAL)
    e.kp_servo, e.kd_servo = 80.0, 1.3
    e.estimator = "enc"; e.ctrl_mode = "fwe3"
    e.x0 = dict(phi0=0.0, alpha0=0.5, theta0=0.5)
    e.build()
    e.reset()
    return e


def mj_com_x(e):
    """MuJoCo 질량가중 몸통(하체+상체) CoM 수평위치 [m] — 독립 검산용."""
    m1 = e.model.body_mass[e.uid_lower]; m2 = e.model.body_mass[e.uid_upper]
    x1 = e.data.xipos[e.uid_lower][0];   x2 = e.data.xipos[e.uid_upper][0]
    return (m1*x1 + m2*x2) / (m1 + m2)


def run(T=8.0):
    e = make_engine()
    g = e.g; pl = e._plane
    P11 = pl["P"][1][1]; P10 = pl["P"][1][0]; p2r = pl["p2r"]
    n = int(T / e.p["DT"])
    rows = []
    for i in range(n):
        # 문서 55 함정: A 는 control_step '이전' 값 — 상태도 스텝 전 기록
        x = e.get_state()
        b, f, bp, fp = e.plane_pt(x)
        xh = e.xenc
        _, _, bph, fph = e.plane_pt(xh)
        dd = x[5] - x[4]                      # 참 δ̇ [rad/s]
        phase = {PH_IDLE: 0, PH_FOLD: 1, PH_REST: 2}.get(e.fwe["phase"], 3)
        xcom = mj_com_x(e)
        rows.append((e.data.time, phase, b, f, bp, fp, bph, dd, x[3],
                     p2r*x[4] + (1-p2r)*0 , xcom, x[0]))
        e.control_step()
        if e.fallen:
            break
    return e, np.array(rows), P11, P10, p2r


def main():
    e, a, P11, P10, p2r = run()
    t, ph, b, f, bp, fp, bph, dd, fd, _, xcom, phi = a.T
    deg = np.rad2deg
    infold = (ph == 1) | (ph == 2)
    print(f"스텝 {len(a)}  (fold/REST {int(infold.sum())}스텝)  λ={e._plane['lam']:.4f}  "
          f"P11={P11:.4f} (1/λ={1/e._plane['lam']:.4f})  p2r={p2r:.4f}")

    # ---- [1] 부호반전 재현 (문서 59 와 같은 정의: 연속 스텝 부호 바뀜, 양쪽 |값|>0.2°)
    def reversals(v, mask):
        out = []
        for i in range(1, len(v)):
            if not mask[i]: continue
            if v[i-1]*v[i] < 0 and abs(deg(v[i-1])) > 0.2 and abs(deg(v[i])) > 0.2:
                out.append(i)
        return out
    rev_pred_fold = reversals(bp, infold)
    rev_pred_idle = reversals(bp, ~infold)
    rev_pos_all   = reversals(b, np.ones_like(infold, bool))
    print(f"\n[1] β_pred 참값 부호반전: fold/REST {len(rev_pred_fold)}건, idle {len(rev_pred_idle)}건")
    print(f"    원위치 β 참값 '점프성' 반전(|Δ|>1°): "
          f"{sum(1 for i in rev_pos_all if abs(deg(b[i]-b[i-1]))>1.0)}건")
    dstep_fold = deg(np.abs(np.diff(bp)))[infold[1:]]
    dstep_idle = deg(np.abs(np.diff(bp)))[~infold[1:]]
    dstep_est  = deg(np.abs(np.diff(bph)))
    print(f"    |Δβ_pred|/스텝 중앙값: 참 fold {np.median(dstep_fold):.2f}° / "
          f"참 idle {np.median(dstep_idle):.3f}° / 추정(전체) {np.median(dstep_est):.3f}°")

    # ---- [2] 원인분해: 직접 δ̇ 항 제거판
    bp_frozen = bp - P11 * p2r * dd          # β̇ 에서 p2r·δ̇ 기여 제거
    rev_frozen = reversals(bp_frozen, infold)
    # φ̇ 커플링(P10·φ̇)까지 보수적으로 보려면: 접기 반작용은 φ̇ 에도 실린다 — 참고 수치만
    corr = np.corrcoef(np.diff(bp)[infold[1:]], (P11*p2r*np.diff(dd))[infold[1:]])[0, 1]
    print(f"\n[2] δ̇ 항 제거 시 fold/REST 반전: {len(rev_pred_fold)} → {len(rev_frozen)}건")
    print(f"    fold 중 Δβ_pred vs P11·p2r·Δδ̇ 상관 = {corr:.3f}")

    # ---- [3] 거울검사: 반전 전후 비율
    if rev_pred_fold:
        ratio = np.array([bp[i]/bp[i-1] for i in rev_pred_fold])
        pairs = [(deg(bp[i-1]), deg(bp[i])) for i in rev_pred_fold[:8]]
        print(f"\n[3] 반전 전후 비율 after/before: 중앙값 {np.median(ratio):.2f}, "
              f"IQR [{np.percentile(ratio,25):.2f}, {np.percentile(ratio,75):.2f}], "
              f"범위 [{ratio.min():.2f}, {ratio.max():.2f}]")
        near_mirror = np.mean(np.abs(ratio + 1.0) < 0.05) * 100
        print(f"    '정확 거울'(비율 -1±0.05) 비중: {near_mirror:.0f}%")
        print("    예시(°):", ", ".join(f"{u:+.2f}→{v:+.2f}" for u, v in pairs))

    # ---- [4] 독립검산: x_CoM ≈ R·φ + h·β (MuJoCo 좌표 x 는 모델 x 와 부호 반대: qpos[phi]=-φ)
    aux = e.g["mdl"]["aux"]; R = e.p["R"]; h = aux["h"]
    pred_xcom = R*np.sin(phi)*0 + R*phi + h*b   # 소각 선형화 그대로 (모델 정의와 동일)
    err = np.abs(-xcom - pred_xcom)          # mj x축 부호 반전 규약
    err2 = np.abs(xcom - pred_xcom)
    use = err if np.median(err) < np.median(err2) else err2
    sgn = "-x_mj" if np.median(err) < np.median(err2) else "+x_mj"
    print(f"\n[4] CoM 독립검산 ({sgn} vs Rφ+hβ): 중앙값 오차 {np.median(use)*1000:.2f} mm, "
          f"최대 {use.max()*1000:.2f} mm, fold 중 최대 {use[infold].max()*1000:.2f} mm  (h={h:.4f} m)")

    # ---- [5] 스파이크의 해부 — 무게중심 보존이 성립하는 '내부 교환'인가
    #   ẍ_CoM = −gφ 는 접는 도중에도 성립(권위문서). 따라서 스텝당 Δẋ_CoM 은 fold 라고
    #   특별히 클 수 없다. 반면 hβ̇ 과 Rφ̇ 은 각각 크게 튀되 서로 반대로 튀어야 한다.
    DTs = e.p["DT"]
    xdot = np.gradient(-xcom, DTs)           # MuJoCo CoM 속도 (모델 부호로)
    dxd  = np.abs(np.diff(xdot))
    bd_arr = np.gradient(b, DTs); fd_arr = fd  # fd = x[3] (φ̇ 참값 기록)
    hb = h*np.gradient(b, DTs); rf = R*fd
    m_f = infold[1:]
    print(f"\n[5] 무게중심 보존 검사:")
    print(f"    스텝당 |Δẋ_CoM|: fold 중앙값 {np.median(dxd[m_f])*1000:.2f} mm/s ↔ "
          f"idle 중앙값 {np.median(dxd[~m_f])*1000:.2f} mm/s (비슷해야 함 — CoM 은 접기를 모른다)")
    cc = np.corrcoef(hb[infold], (xdot - rf)[infold])[0, 1]
    print(f"    fold 중 h·β̇ vs (ẋ_CoM − R·φ̇) 상관 = {cc:.4f} (1.0 이면 β̇ 스파이크 = "
          f"발던지기 반작용 그 자체)")
    print(f"    fold 중 |h·β̇|max = {np.abs(hb[infold]).max()*1000:.0f} mm/s, "
          f"|R·φ̇|max = {np.abs(rf[infold]).max()*1000:.0f} mm/s, "
          f"|ẋ_CoM|max = {np.abs(xdot[infold]).max()*1000:.0f} mm/s")

    # 최대 스파이크 크기 참고
    print(f"\n(참고) fold 중 |δ̇|max = {deg(np.abs(dd[infold])).max():.0f}°/s, "
          f"실측 |β_pred|max(fold) = {deg(np.abs(bp[infold])).max():.1f}°, "
          f"idle |β_pred|max = {deg(np.abs(bp[~infold])).max():.1f}°")


if __name__ == "__main__":
    main()
