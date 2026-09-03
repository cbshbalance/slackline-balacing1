# -*- coding: utf-8 -*-
"""v21 수용시험 (헤드리스) — 계획 문서 56 §4.
실행: python tests_v21.py

[1A 기준 — 8/17 밤 정정]
당초 기준 "선 위 = 발산 없음"은 선형 이상화였다. 전물리에서는 비선형 씨딩(진폭² 비례)이
λ=5.92/s 로 증폭되어 **완벽히 선 위에 놓아도 ~2 s 뒤 넘어진다** (수치 씨딩만으로도 ~3 s 상한).
실물도 같은 물리이므로 문서 46 이 실측② 합격을 "3회 이상 진동"으로 정한 것과 정합.
따라서 시연·시험 기준은 대조로 정의한다:
  선 위  = 왕복 2회 이상 + 오래 버팀 / 선 밖 = 왕복 없이(≤1회) 즉시 한쪽 낙하
  대조비 = (선위 최소 버팀) / (선밖 최대 버팀) ≥ 1.5
  제어 ON(트리거 0.6°) = 같은 10점 전부 8 s 생존 — "증분접기가 전부 잡는다"
"""
import sys, os
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "v19_bringup")); sys.path.insert(0, HERE)
import numpy as np
from sim_engine import SimEngine, REAL_DEFAULT
import params_v19 as P

def make_engine(trigger_deg=0.6):
    e = SimEngine()
    e.nominal = dict(P.MEASURED_SET); e.nominal["RHO"] = 0.95; e.nominal["DT"] = 0.005
    e.nominal["A_TRIGGER_DEG"] = trigger_deg
    e.mismatch = dict(m1=0, m2=0, L1=0, L2=0, R=0, kt=0)
    e.real = dict(REAL_DEFAULT)
    e.real.update(enc_use_gyro=False, fwe_settle_s=0.0, fwe_adapt_T=False, fwe_ff=True,
                  fwe_fast_L=False, w_noload_dps=462.0, fwe_pos_mode=True, pos_vel_dps=420.0,
                  pos_acc_dps2=8000.0, fwe_async=True, fwe_async_gamma=10.0,
                  fwe_async_vret_dps=3.0, fwe_async_hold_min_deg=1.0, fwe_online=False,
                  enc_diff_ms=25.0, enc_tau_v=0.030, **P.MEASURED_REAL)
    e.kp_servo, e.kd_servo = 80.0, 1.3
    e.estimator = "enc"; e.ctrl_mode = "fwe3"
    e.build()
    return e

def release(e, beta, phi, T=8.0):
    """(β,φ) 속도0 δ=0 놓기 → (낙하시각 or None, φ부호반전 수, |φ|max)"""
    e.x0 = dict(phi0=phi, alpha0=beta, theta0=beta)
    e.reset()
    n = int(T/e.p["DT"]); flips = 0; prev = np.sign(phi) or 1.0; pm = 0.0
    for i in range(n):
        e.control_step()
        x = e.get_state(); ph = np.rad2deg(x[0]); pm = max(pm, abs(ph))
        s = np.sign(ph)
        if s != 0 and s != prev and abs(ph) > 0.2: flips += 1; prev = s
        if e.fallen: return (i+1)*e.p["DT"], flips, pm
    return None, flips, pm

def points(r):
    onl  = [(b, r*b) for b in (-2.0, -1.0, 0.5, 1.0, 2.0)]
    offl = [(b + dA, r*b) for b, dA in
            ((-1.0, 1.5), (-0.5, -1.2), (0.5, 1.2), (1.0, -1.5), (0.0, 1.8))]  # Δβ=ΔA (w_β=1)
    return onl, offl

def test_1a():
    print("=== 1A 놓기 대조 (트리거 999 = 제어 봉쇄) ===")
    e = make_engine(999.0)
    r = e.build_info()["plane"]["r"]
    onl, offl = points(r)
    ton, toff, ok = [], [], True
    for b, f in onl:
        tf, fl, pm = release(e, b, f)
        t = tf if tf else 8.0; ton.append(t)
        good = (fl >= 2 and t >= 1.3)
        print(f"  선위 β={b:5.2f} φ={f:6.2f} → 버팀 {t:4.2f}s 왕복 {fl}회 {'OK' if good else 'NG'}")
        ok &= good
    for b, f in offl:
        tf, fl, pm = release(e, b, f)
        t = tf if tf else 8.0; toff.append(t)
        good = (t <= 1.0 and fl <= 1)
        print(f"  선밖 β={b:5.2f} φ={f:6.2f} → 버팀 {t:4.2f}s 왕복 {fl}회 {'OK' if good else 'NG'}")
        ok &= good
    ratio = min(ton)/max(toff)
    print(f"  대조비 min(선위)/max(선밖) = {ratio:.2f} (기준 ≥1.5)")
    ok &= ratio >= 1.5
    print("=== 1A-제어ON (트리거 0.6°) — 같은 10점 전부 생존 ===")
    e2 = make_engine(0.6)
    for b, f in onl + offl:
        tf, fl, pm = release(e2, b, f)
        alive = tf is None
        print(f"  β={b:5.2f} φ={f:6.2f} → {'8s 생존' if alive else f'낙하 {tf:.2f}s'} |φ|max={pm:4.1f}° 접기 {e2.fwe['cycles']}회")
        ok &= alive
    print("1A:", "PASS" if ok else "FAIL"); assert ok

def test_1b():
    print("=== 1B 4D 왕복 (속도 포함 x0 → 상태·평면점 일치) ===")
    e = make_engine()
    p2r = e.build_info()["plane"]["p2r"]; Pm = np.array(e.build_info()["plane"]["P"])
    ok = True
    for (b, f, bd, fd, dl) in [(1.0, -1.64, 0, 0, 0), (2.0, -3.0, 10, -15, 0),
                               (-1.5, 2.0, -8, 12, 20), (0.5, -0.8, 5, 5, -15)]:
        al = b - p2r*dl; th = al + dl
        e.x0 = dict(phi0=f, alpha0=al, theta0=th, phid0=fd, alphad0=bd, thetad0=bd)
        e.reset()
        x = e.get_state()
        bq, fq, bp, fp = e.plane_pt(x)
        q  = np.deg2rad([f, b]); qd = np.deg2rad([fd, bd])
        qp = q + Pm @ qd
        err = max(abs(np.rad2deg(bq)-b), abs(np.rad2deg(fq)-f),
                  abs(np.rad2deg(bp)-np.rad2deg(qp[1])), abs(np.rad2deg(fp)-np.rad2deg(qp[0])))
        dln = np.rad2deg(x[2]-x[1])
        print(f"  β={b} φ={f} β̇={bd} φ̇={fd} δ={dl} → 최대오차 {err:.1e}° δ재현 {dln:.3f}°")
        ok &= err < 1e-9 and abs(dln-dl) < 1e-9
    print("1B:", "PASS" if ok else "FAIL"); assert ok

def test_pose():
    print("=== pose 무결성 (set_pose → 복원) ===")
    e = make_engine()
    e.x0 = dict(phi0=0.0, alpha0=0.5, theta0=0.5); e.reset()
    for _ in range(400): e.control_step()
    s = e.snapshot(); x0 = e.get_state().copy(); n0 = e.step_count
    e.set_pose(5.0, -3.0, -3.0)
    xp = e.get_state()
    assert abs(np.rad2deg(xp[0])-5.0) < 1e-9 and abs(np.rad2deg(xp[1])+3.0) < 1e-9
    assert np.all(np.abs(xp[3:]) < 1e-12)
    e.restore(s)
    assert np.allclose(e.get_state(), x0) and e.step_count == n0
    for _ in range(100): e.control_step()
    print("pose: PASS")

if __name__ == "__main__":
    test_1a(); test_1b(); test_pose()
    print("\nALL PASS")
