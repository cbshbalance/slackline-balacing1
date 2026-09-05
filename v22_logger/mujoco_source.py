# -*- coding: utf-8 -*-
"""
mujoco_source.py — v22 로거: MuJoCo 가상 로봇 소스 (v21 SimEngine + v22_raw v2 펌웨어 흉내)
=============================================================================================
실기 대신 MuJoCo(v21_pres/sim_engine.py, 물리 무수정)를 실시간 200 Hz 로 돌리고,
그 위에 v22_raw v2 펌웨어의 로직(D/F/E 행, Â, 단일접기/증분접기, 파라미터 표, 명령 문법)을
파이썬으로 그대로 얹었다. 앱 쪽에서는 시리얼 포트 하나가 더 있는 것과 구별되지 않는다:

    D,t_ms,phi,ank,alpha,beta,dphi,dbeta,Ahat,hold,del_now,phase,cue,err,phi_raw,ank_raw,dxl_raw
    F,trial,A_pre,d0,dd_cmd,dd_act,A_post,lock_ms,fold_ms,goaln
    E,t_ms,event,value             (ZERO MOVE FOLD FOLDPOST GO STOP FALL)

사람이 하는 일(로봇을 잡고 옮기고 놓고 다시 잡는 것)은 'sim …' 지시문으로 넣는다 — 리허설
영상·E2E 용이다. 실기 명령 문법(z u k g h x mode N gam X fold X …)은 펌웨어와 같다.

    sim release B F [T]   (β=B°, φ=F°) 로 옮겨 잡고(T s, 기본 1.2) 놓는다 → |φ| 가 잡기각에 닿으면 잡는다
    sim hold B F          (β, φ) 로 옮겨 잡고 유지
    sim catch             지금 자세로 잡기
    sim free              지금 놓기
    sim seed N            난수 씨앗 (손떨림·놓기 킥·잡기각)
    sim tremor X          손떨림 진폭 [deg] (기본 0.02)
    sim catch_deg X       잡기각 [deg] (기본 8.8 ± 0.5 난수)

⚠ 엔진 자체의 FWE 트리거는 잠그고(armed=False, 트리거 999°) 위치 서보(프로파일 발생기)만 빌린다.
   접기 결정은 전부 이 파일의 '펌웨어' 가 한다 — γ 하나, ρ 없음, w = r·λ 닫힌형 (v22_raw v2 와 동일).
"""
import collections
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for pth in (os.path.join(HERE, "..", "v19_bringup"), HERE):
    if pth not in sys.path:
        sys.path.insert(0, pth)

import serial_bridge as sb

D_HEADER = "t_ms,phi,ank,alpha,beta,dphi,dbeta,Ahat,hold,del_now,phase,cue,err,phi_raw,ank_raw,dxl_raw"
F_HEADER = sb.FOLD_HEADER
IDLE, FOLD, REST, STOPPED, ARMED_OFF = 0, 1, 2, 3, 4
ENC_DEG = 360.0 / 16384.0            # AS5047P 14비트
DXL_DEG = 360.0 / 4096.0             # XM430 12비트
VEL_DPS, ACC_DPS2 = 1.374, 21.4577   # 프로파일 단위 → deg/s, deg/s²
DT_S = 0.005
VEL_N, EMA_A = 5, 0.15

# 펌웨어 파라미터 표 (이름, 기본, lo, hi, 설명) — v22_raw v2 와 같은 순서·범위
PARAM_TABLE = [
    ("gam", 8.0, 0.5, 60.0, "접기 이득 γ [°/°] (ρ 없음)"),
    ("trig", 0.6, 0.05, 5.0, "트리거 문턱 [deg]"),
    ("rel", 0.3, 0.02, 5.0, "펴기 게이트 [deg]"),
    ("vrel", 3.0, 0.0, 30.0, "펴기 속도 [deg/s]"),
    ("dead", 1.0, 0.0, 10.0, "펴기 데드밴드 [deg]"),
    ("dstep", 20.0, 1.0, 55.0, "접기 1회 상한 [deg]"),
    ("dlim", 55.0, 5.0, 80.0, "힙 기구한계 [deg]"),
    ("rest", 60.0, 0.0, 500.0, "REST [ms]"),
    ("alim", 30.0, 5.0, 90.0, "넘어짐 한계각 [deg]"),
    ("sgn", 1.0, -1.0, 1.0, "접기 방향 ±1"),
    ("fdeg", 0.0, 0.0, 55.0, "고정 접기량 (0=γ·Â)"),
    ("lock", 250.0, 50.0, 1000.0, "A_post 관측창 [ms]"),
    ("r", -1.506, -20.0, -0.1, "안정모드선 기울기"),
    ("lam", 5.44, 0.5, 30.0, "발산율 λ [1/s]"),
    ("c0", 0.0, -20.0, 20.0, "절편 c0 [deg]"),
    ("kv", 1.0, 0.2, 4.0, "닫힌형 속도항 배율"),
    ("wmode", 0.0, 0.0, 1.0, "0 닫힌형 / 1 수동 wf wb"),
    ("wf", 0.1945, 0.0, 2.0, "수동 w φ̇"),
    ("wb", 0.3049, 0.0, 2.0, "수동 w β̇"),
    ("p2r", 0.4285, 0.05, 0.95, "β = α + P2R·δ"),
    ("fphi", 0.0, 0.0, 1.0, "φ 직립 변환 +180 (가상 로봇은 무시)"),
    ("fank", 0.0, 0.0, 1.0, "발목 직립 변환 +180 (가상 로봇은 무시)"),
    ("loghz", 100.0, 1.0, 200.0, "CSV 주기 [Hz]"),
    ("ftol", 2.0, 0.1, 10.0, "접기 도착 판정 [deg]"),
    ("ftmax", 300.0, 20.0, 800.0, "FOLD 상한 [ms]"),
]


def build_engine(dt=DT_S, vel_unit=250, acc_unit=373):
    """v21 tests_v21.make_engine 과 같은 실측 프리셋. 엔진 트리거는 잠근다."""
    from sim_engine import SimEngine, REAL_DEFAULT
    import params_v19 as P
    e = SimEngine()
    e.nominal = dict(P.MEASURED_SET)
    e.nominal["RHO"] = 1.0
    e.nominal["DT"] = dt
    e.nominal["A_TRIGGER_DEG"] = 999.0          # 엔진 자체 접기 금지 — 접기는 MujocoSource 의 펌웨어가 한다
    e.mismatch = dict(m1=0.0, m2=0.0, L1=0.0, L2=0.0, R=0.0, kt=0.0)
    e.real = dict(REAL_DEFAULT)
    e.real.update(enc_use_gyro=False, fwe_settle_s=0.0, fwe_adapt_T=False, fwe_ff=True,
                  fwe_fast_L=False, w_noload_dps=462.0, fwe_pos_mode=True,
                  pos_vel_dps=vel_unit * VEL_DPS, pos_acc_dps2=acc_unit * ACC_DPS2,
                  fwe_async=True, fwe_async_gamma=10.0, fwe_async_vret_dps=0.0,
                  fwe_async_hold_min_deg=1.0, fwe_online=False,
                  enc_diff_ms=25.0, enc_tau_v=0.030, **P.MEASURED_REAL)
    e.kp_servo, e.kd_servo = 80.0, 1.3
    e.estimator = "enc"
    e.ctrl_mode = "fwe3"
    e.x0 = dict(phi0=0.0, alpha0=0.0, theta0=0.0)
    e.build()
    e.reset()
    e.fwe["armed"] = False
    return e


class MujocoSource(sb.LineSource):
    """MuJoCo 가상 로봇 + v22_raw v2 펌웨어 흉내. LineSource 규약(push/drain/write/close/describe)."""

    def __init__(self, seed=0, realtime=True, tremor_deg=0.02):
        super().__init__()
        self.realtime = bool(realtime)
        self.rng = np.random.default_rng(seed)
        self.tremor = float(tremor_deg)
        self.trem = [0.0, 0.0]                     # AR(1) 손떨림 상태 (φ, α) — 백색이 아니라 ~0.15 s 시정수의 느린 떨림
        self.n_settled = 0                         # 옮기기 완료 횟수 (시험용)
        self.catch_deg = 0.0                      # 0 = 8.8 ± 0.5 난수
        self.cmd_q = collections.deque()
        self.eng = None
        self.eng_err = None
        # ---- 펌웨어 상태 ----
        self.P = {n: float(v) for n, v, lo, hi, w in PARAM_TABLE}
        self.vel_unit, self.acc_unit, self.cur_limit = 250, 373, 350
        self.run_mode = 0
        self.running = False
        self.dry_run = False
        self.csv_on, self.out_on = True, True
        self.phase = ARMED_OFF
        self.hold = 0.0
        self.delta_now = 0.0
        self.once_done = False
        self.post_pending = False
        self.trial_n = 0
        self.zero_stage = 0
        self.phi_zero, self.ank_zero = 8192, 8192
        self.primed = False
        self.hist_phi = [0.0] * (VEL_N + 1)
        self.hist_beta = [0.0] * (VEL_N + 1)
        self.hist_i = 0
        self.dphi = self.dbeta = 0.0
        self.Ahat = 0.0
        self.phi_d = self.ank_d = self.alpha_d = self.beta_d = 0.0
        self.phi_raw = self.ank_raw = 8192
        self.dxl_raw = 2048.0
        self.f = dict(A_pre=0.0, d0=0.0, dd_cmd=0.0, goal=0.0, fold_ms=0, arrived=False, wait_ms=0)
        self.phase_t0 = self.fold_t0 = 0.0
        self.relax_thr = 0
        self.W = dict(phi=0.0, phid=0.0, betad=0.0, off=0.0)
        self._derive_w()
        # ---- 사람(무대 지시) ----
        self.stage = "held"          # held / moving / free
        self.pose = [0.0, 0.0]       # 잡고 있는 (φ, α) [deg]
        self.mv = None               # dict(a=(φ,α), b=(φ,α), t0, T, then, hold_T)
        self.free_t0 = 0.0
        self.n_release = self.n_catch = 0
        self.last_release = None
        # ---- 시각 ----
        self.k = 0
        self.t_ms = 0
        self.log_next_ms = 0
        self.hdr_next_ms = 0
        self.last_frame = None

    # ================= 펌웨어 보조 =================
    def _derive_w(self):
        P = self.P
        self.W["phi"] = -1.0 / P["r"]
        if P["wmode"] < 0.5:
            self.W["phid"] = P["kv"] * (-1.0 / (P["r"] * P["lam"]))
            self.W["betad"] = P["kv"] / P["lam"]
        else:
            self.W["phid"], self.W["betad"] = P["wf"], P["wb"]
        self.W["off"] = P["c0"] / P["r"]

    def _ms(self):
        return self.t_ms

    def _emitE(self, name, v, dec=0):
        self.push(f"E,{self._ms()},{name},{v:.{dec}f}")

    def _profile_ms(self, deg):
        deg = abs(deg)
        acc, vmx = self.acc_unit * ACC_DPS2, self.vel_unit * VEL_DPS
        if deg <= 0:
            return 20.0
        t = 2.0 * math.sqrt(deg / acc) if math.sqrt(deg * acc) <= vmx else (deg / vmx + vmx / acc)
        return t * 1000.0

    def _write_goal(self, deg):
        if self.eng is not None and not self.dry_run:
            self.eng.fwe["hold"] = float(np.deg2rad(deg))

    def _header(self):
        self.push("# D," + D_HEADER)
        self.push("# F," + F_HEADER)
        self.push("# v22_raw v2 (MuJoCo 가상): phase 0 IDLE/1 FOLD/2 REST/3 STOP/4 대기. cue=단일접기 완료. err = phi등급 + 4·ank등급 + 16·dxl")

    def _log_line(self):
        ph = self.phase if self.running else (STOPPED if self.phase == STOPPED else ARMED_OFF)
        self.push("D,%d,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.4f,%.2f,%.2f,%d,%d,%d,%d,%d,%d" % (
            self._ms(), self.phi_d, self.ank_d, self.alpha_d, self.beta_d, self.dphi, self.dbeta,
            self.Ahat, self.hold, self.delta_now, ph, 1 if self.once_done else 0, 0,
            self.phi_raw, self.ank_raw, int(self.dxl_raw)))

    def _status(self):
        P, W = self.P, self.W
        self.push("---- v22_raw v2 상태 (MuJoCo 가상 로봇) ----")
        self.push(f"모드 {self.run_mode} {'측정' if self.run_mode == 0 else ('단일접기' if self.run_mode == 1 else '증분접기')}"
                  f"   제어 {'RUN' if self.running else '정지'}{'  [DRY-RUN]' if self.dry_run else ''}   phase {self.phase}")
        self.push("모터: OK  torque=ON")
        self.push(f"w = [{W['phi']:.4f}, 1, {W['phid']:.4f}, {W['betad']:.4f}]  A_offset={W['off']:.4f}  (wmode {int(P['wmode'])}, kv {P['kv']:.2f})")
        self.push(f"gam {P['gam']:.2f}  trig {P['trig']:.2f}  fdeg {P['fdeg']:.1f}  lock {P['lock']:.0f}  sgn {P['sgn']:.0f}  r {P['r']:.3f}  lam {P['lam']:.2f}  c0 {P['c0']:.3f}  p2r {P['p2r']:.4f}")
        self.push(f"CSV {'ON' if self.csv_on else 'OFF'}  loghz {P['loghz']:.0f}  vel {self.vel_unit}  acc {self.acc_unit}  ilim {self.cur_limit}")
        self.push(f"영점 raw: phi {self.phi_zero}  ank {self.ank_zero}  (단계 {self.zero_stage}/2)  home_tick 2048")
        self.push(f"hold {self.hold:.2f}  delta {self.delta_now:.2f}  접기 시행 {self.trial_n}")
        self.push(f"[가상] 사람: {self.stage}  놓기 {self.n_release}회 · 잡기 {self.n_catch}회")

    def _print_params(self):
        for n, v, lo, hi, w in PARAM_TABLE:
            self.push(f"# {n} = {self.P[n]:.4f}   {w}")

    # ================= 펌웨어: 측정·접기 =================
    def _sense(self):
        x = self.eng.get_state()
        phi = math.degrees(x[0])
        alpha = math.degrees(x[1])
        theta = math.degrees(x[2])
        delta = theta - alpha
        # 엔코더 양자화 (14비트 / 12비트) — 실기와 같은 잡음 바닥
        self.phi_raw = int(round(phi / ENC_DEG)) % 16384
        self.ank_raw = int(round((alpha + phi) / ENC_DEG)) % 16384
        self.dxl_raw = 2048.0 + round(delta / DXL_DEG)
        self.phi_d = round(phi / ENC_DEG) * ENC_DEG
        self.ank_d = round((alpha + phi) / ENC_DEG) * ENC_DEG
        self.delta_now = round(delta / DXL_DEG) * DXL_DEG
        self.alpha_d = self.ank_d - self.phi_d
        self.beta_d = self.alpha_d + self.P["p2r"] * self.delta_now
        if not self.primed:
            self.hist_phi = [self.phi_d] * (VEL_N + 1)
            self.hist_beta = [self.beta_d] * (VEL_N + 1)
            self.dphi = self.dbeta = 0.0
            self.primed = True
        self.hist_phi[self.hist_i] = self.phi_d
        self.hist_beta[self.hist_i] = self.beta_d
        old = (self.hist_i + 1) % (VEL_N + 1)
        dphi_raw = (self.phi_d - self.hist_phi[old]) / (VEL_N * DT_S)
        dbeta_raw = (self.beta_d - self.hist_beta[old]) / (VEL_N * DT_S)
        self.hist_i = old
        self.dphi += EMA_A * (dphi_raw - self.dphi)
        self.dbeta += EMA_A * (dbeta_raw - self.dbeta)
        W = self.W
        self.Ahat = W["phi"] * self.phi_d + self.beta_d + W["phid"] * self.dphi + W["betad"] * self.dbeta + W["off"]
        self.last_frame = dict(phi=phi, alpha=alpha, theta=theta, delta=delta,
                               phid=math.degrees(x[3]), alphad=math.degrees(x[4]))

    def _do_fold(self, step, why):
        P = self.P
        step = max(-P["dstep"], min(P["dstep"], step))
        self.f.update(A_pre=self.Ahat, d0=self.delta_now, dd_cmd=step)
        self.hold = max(-P["dlim"], min(P["dlim"], self.hold + step))
        self.f["goal"] = self.hold
        self._write_goal(self.hold)
        fw = self._profile_ms(step) * 1.3 + 20.0
        self.f["wait_ms"] = min(fw, P["ftmax"])
        self.phase = FOLD
        self.phase_t0 = self.fold_t0 = self._ms()
        self.f.update(arrived=False, fold_ms=0)
        self.post_pending = True
        self.trial_n += 1
        self.push(f"E,{self._ms()},FOLD,{step:.2f}   # {why} A_pre={self.f['A_pre']:.3f}")

    def _control(self):
        P = self.P
        now = self._ms()
        if self.phase == IDLE:
            if not (self.run_mode == 1 and self.once_done):
                if self.run_mode >= 1 and abs(self.Ahat) > P["trig"]:
                    sgn = 1.0 if self.Ahat > 0 else -1.0
                    step = P["sgn"] * P["fdeg"] * sgn if P["fdeg"] > 0 else P["sgn"] * P["gam"] * self.Ahat
                    self._do_fold(step, "단일접기" if self.run_mode == 1 else "증분접기")
                    if self.run_mode == 1:
                        self.once_done = True
                elif self.run_mode == 2 and abs(self.Ahat) < P["rel"] and abs(self.hold) > P["dead"]:
                    self.hold += (-1 if self.hold > 0 else 1) * P["vrel"] * DT_S
                    self.relax_thr += 1
                    if self.relax_thr >= 10:
                        self.relax_thr = 0
                        self._write_goal(self.hold)
        elif self.phase == FOLD:
            if not self.f["arrived"] and abs(self.delta_now - self.hold) < P["ftol"]:
                self.f["arrived"] = True
                self.f["fold_ms"] = now - self.fold_t0
            if self.f["arrived"] or (now - self.phase_t0) >= self.f["wait_ms"]:
                if not self.f["arrived"]:
                    self.f["fold_ms"] = now - self.fold_t0
                self.phase = REST
                self.phase_t0 = now
        elif self.phase == REST:
            if now - self.phase_t0 >= P["rest"]:
                self.phase = IDLE
        if self.post_pending and now - self.fold_t0 >= P["lock"]:
            self.post_pending = False
            dd_act = self.delta_now - self.f["d0"]
            self.push("F,%d,%.4f,%.2f,%.2f,%.2f,%.4f,%d,%d,%.2f" % (
                self.trial_n, self.f["A_pre"], self.f["d0"], self.f["dd_cmd"], dd_act, self.Ahat,
                int(P["lock"]), int(self.f["fold_ms"]), self.f["goal"]))
            self._emitE("FOLDPOST", self.Ahat, 4)

    def _torque_off_all(self, why):
        self.running = False
        self.phase = STOPPED
        self.push(f"# !!! 토크 OFF — {why}")

    def _do_go(self):
        if self.zero_stage == 1:
            self.push("# ⚠ 영점이 1차만 기록된 상태 — z 를 마저 누르는 게 좋다")
        self.hold = self.delta_now
        self._write_goal(self.hold)
        self.primed = False
        self.dphi = self.dbeta = 0.0
        self.once_done = False
        self.post_pending = False
        self.phase = IDLE
        self.phase_t0 = self._ms()
        self.running = True
        what = {0: " 측정(접지 않음)", 1: " 단일접기 — 첫 |Â|>trig 에서 한 번 접고 δ 고정", 2: " 증분접기"}[self.run_mode]
        self.push(f"# GO  모드 {self.run_mode}{what}" + ("  (DRY-RUN: 모터 명령 안 나감)" if self.dry_run else ""))
        self._emitE("GO", float(self.run_mode), 0)

    # ================= 명령 =================
    def write(self, text):
        text = text.rstrip("\r\n")
        self.cmd_q.append(text)

    def _handle(self, s):
        s = s.strip()
        if not s:
            return
        low = s.lower()
        if low.startswith("sim"):
            self._handle_sim(s[3:].strip())
            return
        if s[0] == "x" or s[0] == "X":
            self._torque_off_all("사용자 x")
            self._emitE("STOP", -1, 0)
            return
        c = s[0]
        if c.isdigit() or c in "+-":
            if self.running:
                self.push("# 제어 중에는 수동 δ 금지 — h 로 멈추고")
                return
            try:
                v = int(float(s))
            except ValueError:
                self.push("# 모르는 명령: " + s)
                return
            v = max(-int(self.P["dlim"]), min(int(self.P["dlim"]), v))
            self.hold = float(v)
            self._write_goal(self.hold)
            self._emitE("MOVE", float(v), 0)
            return
        tok = ""
        i = 0
        while i < len(s) and s[i].isalnum() and len(tok) < 11:
            tok += s[i].lower()
            i += 1
        rest = s[i:].lstrip(" =")
        has = any(ch.isdigit() for ch in rest)
        try:
            val = float(rest.split()[0]) if has else 0.0
        except ValueError:
            val = 0.0
        P = self.P
        if tok == "hdr":
            self._header(); return
        if tok == "swap":
            self.push("# CS 교환: (가상 로봇 — 효과 없음)"); self._emitE("SWAP", 10, 0); return
        if tok == "mode":
            if self.running:
                self.push("# h 로 멈추고 모드를 바꿀 것"); return
            self.run_mode = max(0, min(2, int(val)))
            self.once_done = False
            self.push("# 모드 " + {0: "0 측정(접지 않음)", 1: "1 단일접기 (γ 실험)", 2: "2 증분접기"}[self.run_mode]); return
        if tok == "fold":
            if not has:
                self.push("# fold X  (X = 접기량 deg)"); return
            if not self.running:
                self.push("# g 로 무장한 뒤에 (Â·D행이 살아 있어야 성적표가 된다)"); return
            self._do_fold(P["sgn"] * val, "수동 fold"); return
        if tok == "vel":
            self.vel_unit = int(val); self.eng.real["pos_vel_dps"] = self.vel_unit * VEL_DPS
            self.push(f"# vel {self.vel_unit}"); return
        if tok == "acc":
            self.acc_unit = int(val); self.eng.real["pos_acc_dps2"] = self.acc_unit * ACC_DPS2
            self.push(f"# acc {self.acc_unit}"); return
        if tok == "ilim":
            self.cur_limit = int(val); self.push(f"# ilim {self.cur_limit}"); return
        for n, dv, lo, hi, what in PARAM_TABLE:
            if n == tok:
                if has:
                    P[n] = max(lo, min(hi, val))
                    self._derive_w()
                self.push(f"# {n} = {P[n]:.4f}   {what}")
                if n in ("r", "lam", "c0", "kv", "wmode"):
                    W = self.W
                    self.push(f"#   → w = [{W['phi']:.4f}, 1, {W['phid']:.4f}, {W['betad']:.4f}]  A_offset {W['off']:.4f}")
                return
        if len(tok) != 1:
            self.push("# 모르는 이름: " + tok); return
        if tok == "z":
            if self.running:
                self.push("# h 로 멈추고 영점을 잡을 것"); return
            pz, az = self.phi_raw, self.ank_raw
            if self.zero_stage == 0:
                self.zero_stage = 1; self.push("# ZERO 1차 기록 — 반대쪽에서 정착시킨 뒤 z 한 번 더 (지금은 1차값이 임시 영점)")
            elif self.zero_stage == 1:
                self.zero_stage = 2; self.push("# ZERO 완성 — 데드밴드 phi 0.02 deg (양쪽 정착 차)")
            else:
                self.zero_stage = 1; self.push("# ZERO 다시 시작 — 1차 기록")
            self.hold = 0.0; self._write_goal(0.0); self.primed = False; self.dphi = self.dbeta = 0.0
            self.push(f"E,{self._ms()},ZERO,{pz}/{az}")
        elif tok == "u":
            self.running = False; self.phase = ARMED_OFF; self.push("# 토크 해제")
        elif tok == "k":
            self.hold = 0.0; self._write_goal(0.0); self.phase = ARMED_OFF; self.push("# 토크 ON (현재 위치 유지, δ=0 재정의)")
        elif tok == "g":
            self._do_go()
        elif tok == "h":
            self.running = False; self.phase = ARMED_OFF; self.push("# 정지 (토크·자세 유지)"); self._emitE("STOP", 0, 0)
        elif tok == "y":
            if self.running:
                self.push("# 제어 중에는 전환 금지 — h 먼저"); return
            self.dry_run = not self.dry_run
            self.push("# dry-run " + ("ON (모터 명령 안 나감 — 부호 확인용)" if self.dry_run else "OFF"))
        elif tok == "m":
            self.csv_on = not self.csv_on
            if self.csv_on:
                self._header()
            self.push("# CSV " + ("ON" if self.csv_on else "OFF"))
        elif tok == "s":
            self.out_on = not self.out_on; self.push("# 출력 재개" if self.out_on else "# 출력 정지")
        elif tok == "p":
            self._log_line()
        elif tok == "t":
            self._status()
        elif tok == "w":
            self._print_params()
        elif tok == "e":
            self.push("# ---- 엔코더 진단 (가상 로봇: AGC 64 · MagL 0 · MagH 0 · ERRFL 0) ----")
            self.push(f"# 현재 raw: phi {self.phi_raw}  ank {self.ank_raw}   마지막 변화: phi 5 ms 전, ank 5 ms 전")
        else:
            self.push("# 모르는 명령: " + s)

    # ---- 사람(무대) ----
    def _pose_from_beta(self, beta, phi):
        alpha = beta - self.P["p2r"] * self.delta_now
        return [float(phi), float(alpha)]

    def _handle_sim(self, s):
        parts = s.split()
        if not parts:
            self.push("# [가상] sim release B F [T] · sim hold B F · sim catch · sim free · sim seed N · sim tremor X · sim catch_deg X")
            return
        op = parts[0].lower()
        nums = []
        for p in parts[1:]:
            try:
                nums.append(float(p))
            except ValueError:
                pass
        if op == "release" and len(nums) >= 2:
            beta, phi = nums[0], nums[1]
            T = nums[2] if len(nums) >= 3 else 1.2
            self._start_move(self._pose_from_beta(beta, phi), then="release", hold_T=T)
            self.push(f"# [가상] 사람: 로봇을 β={beta:.2f}° φ={phi:.2f}° 로 옮겨 {T:.1f} s 잡았다가 놓는다")
        elif op == "hold" and len(nums) >= 2:
            self._start_move(self._pose_from_beta(nums[0], nums[1]), then="hold", hold_T=0.0)
            self.push(f"# [가상] 사람: 로봇을 β={nums[0]:.2f}° φ={nums[1]:.2f}° 로 옮겨 잡고 있는다")
        elif op == "catch":
            self._catch("사람이 잡음")
        elif op == "free":
            self._release()
        elif op == "seed" and nums:
            self.rng = np.random.default_rng(int(nums[0])); self.push(f"# [가상] seed {int(nums[0])}")
        elif op == "tremor" and nums:
            self.tremor = abs(nums[0]); self.push(f"# [가상] 손떨림 {self.tremor:.3f}°")
        elif op == "catch_deg" and nums:
            self.catch_deg = abs(nums[0]); self.push(f"# [가상] 잡기각 {self.catch_deg:.1f}°")
        else:
            self.push("# [가상] 모르는 지시: sim " + s)

    def _start_move(self, target, then, hold_T):
        if self.stage == "free":
            self._catch("옮기려고 잡음")
        a = list(self.pose)
        dist = max(abs(target[0] - a[0]), abs(target[1] - a[1]))
        T = max(0.6, min(2.0, 0.5 + dist * 0.25))
        self.mv = dict(a=a, b=list(target), t0=self.k * DT_S, T=T, then=then, hold_T=hold_T)
        self.stage = "moving"

    def _shake(self):
        """손떨림: AR(1) (시정수 0.15 s, 정상 표준편차 tremor). 백색이면 25 ms 차분 속도가 크게 튀어 Â 트리거를 오발한다."""
        tau = 0.15
        a = math.exp(-DT_S / tau)
        sig = self.tremor * math.sqrt(1.0 - a * a)
        self.trem[0] = a * self.trem[0] + self.rng.normal(0.0, sig)
        self.trem[1] = a * self.trem[1] + self.rng.normal(0.0, sig)
        return self.trem

    def _pin(self, phi, alpha):
        e = self.eng
        f0, a0 = math.radians(phi), math.radians(alpha)
        e.data.qpos[e.qadr["phi"]] = -f0
        e.data.qpos[e.qadr["ankle"]] = a0 + f0
        e.data.qvel[e.vadr["phi"]] = 0.0
        e.data.qvel[e.vadr["ankle"]] = 0.0
        import mujoco
        mujoco.mj_forward(e.model, e.data)

    def _release(self):
        if self.stage == "free":
            return
        e = self.eng
        # 손 뗄 때 작은 킥 (실제 손은 완전 정지가 아니다)
        f0d = math.radians(self.rng.normal(0.0, 0.25))
        a0d = math.radians(self.rng.normal(0.0, 0.25))
        e.data.qvel[e.vadr["phi"]] = -f0d
        e.data.qvel[e.vadr["ankle"]] = a0d + f0d
        self.stage = "free"
        self.free_t0 = self.k * DT_S
        self.n_release += 1
        self.last_release = dict(phi=self.pose[0], alpha=self.pose[1], t_ms=self._ms(),
                                 beta=self.pose[1] + self.P["p2r"] * self.delta_now)
        self.push(f"# [가상] 사람: 놓음 #{self.n_release}  (φ {self.pose[0]:.2f}°, α {self.pose[1]:.2f}°, β {self.last_release['beta']:.2f}°, δ {self.delta_now:.1f}°)")

    def _catch(self, why):
        if self.stage != "free":
            return
        fr = self.last_frame or dict(phi=0.0, alpha=0.0)
        self.pose = [fr["phi"], fr["alpha"]]
        self._pin(*self.pose)
        self.stage = "held"
        self.mv = None
        self.n_catch += 1
        self.push(f"# [가상] 사람: {why}  (φ {self.pose[0]:.2f}°, α {self.pose[1]:.2f}°, 놓은 지 {self.k * DT_S - self.free_t0:.2f} s)")

    def _stage_tick(self):
        t = self.k * DT_S
        if self.stage == "moving":
            m = self.mv
            u = min(1.0, (t - m["t0"]) / m["T"])
            s = 0.5 - 0.5 * math.cos(math.pi * u)
            self.pose = [m["a"][0] + (m["b"][0] - m["a"][0]) * s, m["a"][1] + (m["b"][1] - m["a"][1]) * s]
            tr = self._shake()
            self._pin(self.pose[0] + tr[0], self.pose[1] + tr[1])
            if u >= 1.0:
                self.pose = list(m["b"])
                self.n_settled += 1
                if m["then"] == "release":
                    self.mv = dict(m, t0=t, T=max(0.05, m["hold_T"]), then="free")
                    self.stage = "settle"
                else:
                    self.stage = "held"
        elif self.stage == "settle":
            m = self.mv
            tr = self._shake()
            self._pin(self.pose[0] + tr[0], self.pose[1] + tr[1])
            if t - m["t0"] >= m["T"]:
                self._pin(self.pose[0] + tr[0], self.pose[1] + tr[1])
                self._release()
        elif self.stage == "held":
            tr = self._shake()
            self._pin(self.pose[0] + tr[0], self.pose[1] + tr[1])
        else:                                       # free — 잡기 판단은 감지 뒤에
            pass

    def _catch_check(self):
        if self.stage != "free" or self.last_frame is None:
            return
        cd = self.catch_deg if self.catch_deg > 0 else 8.8 + self.rng.uniform(-0.5, 0.5)
        fr = self.last_frame
        dphi = fr["phi"] - (self.last_release["phi"] if self.last_release else 0.0)
        if abs(dphi) >= cd or abs(fr["alpha"]) >= 25.0:      # 시간 상한은 없다 — 증분접기가 잡고 있으면 계속 놓아 둔다
            self._catch("잡음 (|Δφ| %.1f°)" % abs(dphi))

    # ================= 스레드 본체 =================
    def run(self):
        try:
            self.eng = build_engine()
        except Exception as ex:
            self.eng_err = f"{type(ex).__name__}: {ex}"
            self.error = "MuJoCo 엔진을 만들지 못했다: " + self.eng_err
            self.push("# !! " + self.error)
            self.stop_evt.set()
            return
        self._pin(0.0, 0.0)
        self.push("# 모터 OK (토크는 켜지 않았다 — 매달림 영점은 이대로, 잡으려면 k)   [MuJoCo 가상 로봇 — v21 SimEngine 실측 프리셋]")
        self.push("# v22_raw v2 — z u k <정수> m s p t w e swap hdr | mode N · g · h · x · y · fold X | 이름 값 (gam trig r lam c0 kv …)")
        self.push("# [가상] 사람 지시: sim release B F [T] · sim hold B F · sim catch · sim free")
        self._header()
        wall0 = time.time()
        self.k = 0
        self.hdr_next_ms = 20000
        self.log_next_ms = 0
        while not self.stop_evt.is_set():
            while self.cmd_q:
                try:
                    self._handle(self.cmd_q.popleft())
                except Exception as ex:
                    self.push(f"# !! 명령 오류: {type(ex).__name__}: {ex}")
            self.t_ms = int(round(self.k * DT_S * 1000.0))
            self._stage_tick()
            self._sense()
            if self.running:
                P = self.P
                if abs(self.phi_d) > P["alim"] or abs(self.alpha_d) > P["alim"]:
                    self._torque_off_all("넘어짐")
                    self._emitE("FALL", self.phi_d, 1)
                else:
                    self._control()
            self._catch_check()
            if self.csv_on and self.out_on and self.t_ms >= self.log_next_ms:
                self._log_line()
                self.log_next_ms += int(round(1000.0 / max(self.P["loghz"], 1.0)))
            if self.csv_on and self.out_on and self.t_ms >= self.hdr_next_ms:
                self.hdr_next_ms += 20000
                self._header()
            # 물리 1스텝 (서보는 엔진 프로파일 발생기 + 위치 PD 가 hold 를 추종)
            self.eng.control_step()
            self.eng.hist.clear()
            self.k += 1
            if self.realtime:
                due = wall0 + self.k * DT_S
                wait = due - time.time()
                if wait > 0:
                    self.stop_evt.wait(wait)
                elif wait < -0.5:                 # 많이 뒤처졌으면 벽시계를 다시 맞춘다 (폭주 방지)
                    wall0 = time.time() - self.k * DT_S

    def close(self):
        super().close()
        try:
            self.join(timeout=1.0)
        except Exception:
            pass

    def describe(self):
        fr = self.last_frame or {}
        return dict(kind="mujoco", speed=1.0, stage=self.stage, running=self.running, mode=self.run_mode,
                    n_release=self.n_release, n_catch=self.n_catch, n_settled=self.n_settled, phi=round(fr.get("phi", 0.0), 3),
                    alpha=round(fr.get("alpha", 0.0), 3), delta=round(self.delta_now, 2), hold=round(self.hold, 2),
                    trial_n=self.trial_n, t_ms=self.t_ms, err=self.eng_err)


if __name__ == "__main__":                       # 빠른 자가시험: 놓기 1회 + 단일접기 1회 (비실시간)
    src = MujocoSource(seed=1, realtime=False)
    src.start()
    time.sleep(0.5)
    src.write("z"); src.write("z"); src.write("sim release 1.0 0.0 0.6")
    time.sleep(3.0)
    src.write("mode 1"); src.write("g"); src.write("sim release 0.4 0.0 0.6")
    time.sleep(3.0)
    src.close()
    lines = [t for _, t in src.drain()]
    nd = sum(1 for l in lines if l.startswith("D,"))
    print(f"lines {len(lines)}  D {nd}")
    for l in lines:
        if not l.startswith("D,"):
            print("  ", l)
