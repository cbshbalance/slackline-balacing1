# -*- coding: utf-8 -*-
"""
dataset_v22.py — v22 로거의 데이터 정본: 원시 열 + 앱 파이프라인 파생 열
=======================================================================
펌웨어가 무엇을 계산했든 앱은 **원시 엔코더 열(phi, ank, del_now)** 에서 모든 것을 다시 만든다.
펌웨어 계산열(alpha, beta, dphi, dbeta, Ahat)은 이름 뒤에 `_fw` 를 붙여 **나란히** 보관하고,
앱 파생열은 `a_` (인과 = 로봇이 실시간에 볼 수 있는 값) / `s_` (비인과 평활 = 사후 준참값) 접두어를 쓴다.
차이가 곧 "펌웨어가 무엇을 다르게 했는가" 다.

파이프라인 (문서 70·73·45 규약, 전부 파라미터로 노출):
    u_phi, u_ank   ±180° 언랩 (+ 매달림→직립 감김수 스냅, 펌웨어 v8 과 같은 규칙)
    a_alpha = u_ank − u_phi          (alpha_mode: 'ank-phi' 정본 / 'ank+phi' 옛 부호 / 'fw' 펌웨어 열)
    a_theta = a_alpha + del
    a_beta  = a_alpha + P2R·del
    a_dphi, a_dbeta   diff_ms 기저차분 → EMA τ=tau_ms       (실기 파이프라인 한 묶음)
    a_Ahat  = (−1/r)·u_phi + a_beta + vg·wf·a_dphi + vg·wb·a_dbeta + c0/r   (finale8 방식)
    a_psi   = u_phi − phi_eq
    a_fp, a_bp   예측점 q + P·q̇  (P 는 시뮬 모델의 (λI−D̂)⁻¹, 없으면 I/λ)
    s_*      중심 이동평균(smooth_ms) + 중앙차분 — 지연 0 의 준참값 (분석용)
"""
import math
import numpy as np

PIPE_DEFAULT = dict(
    p2r=0.4285, r=-1.506, c0=0.0, lam=5.66, wf=0.1945, wb=0.3049, vg=1.0, phi_eq=0.0,
    diff_ms=25.0, tau_ms=28.0, smooth_ms=50.0, alpha_mode="ank-phi", unwrap=True, snap=True,
    phi_off=0.0, ank_off=0.0,
)
PIPE_DOC = {
    "p2r": "β = α + P2R·δ 의 질량배분비 (실측① 0.4285, 문서 70)",
    "r": "안정모드선 기울기 φ = r·β (실측② −1.506) → w_φ = −1/r",
    "c0": "안정모드선 절편 [°] (A 오프셋 = c0/r)",
    "lam": "발산율 λ [1/s] — Â 에 안 들어감. 예측점(P=I/λ 폴백)·배가시간 표시용",
    "wf": "동정 w 의 φ̇ 성분 (문서 73)", "wb": "동정 w 의 β̇ 성분 (문서 73)",
    "vg": "속도항 배율 (1.0 = 실측 그대로)", "phi_eq": "평형점 [°] — ψ = φ − φ_eq",
    "diff_ms": "속도 기저차분 창 [ms] (실기 25)", "tau_ms": "속도 EMA 시정수 [ms] (실기 28)",
    "smooth_ms": "비인과 평활 창 [ms] (사후 분석용)",
    "alpha_mode": "α 공식: ank-phi (정본, 문서 70 §2) / ank+phi (옛 부호) / fw (펌웨어 열 그대로)",
    "unwrap": "±180° 언랩", "snap": "감김수 스냅 (|u−w|>180 이고 |w|<90 이면 원가지로)",
    "phi_off": "φ 오프셋 [°] — 매달림 영점으로 세우면 φ≈±180 → 180 을 넣어 직립 0 으로 (펌웨어 fphi 대응)",
    "ank_off": "발목 오프셋 [°] (보통 0)",
}
ALIASES = {
    "t_ms": ("t_ms", "t", "time_ms"),
    "phi": ("phi", "phi_deg", "f"),
    "ank": ("ank", "ank_deg", "ankle", "ankle_deg", "k"),
    "del": ("del_now", "del_now_deg", "delta", "delta_deg", "d"),
    "hold": ("hold", "del_cmd", "del_cmd_deg", "dcmd"),
    "alpha_fw": ("alpha", "alpha_deg"),
    "beta_fw": ("beta", "beta_deg"),
    "dphi_fw": ("dphi", "phid", "dphi_dps"),
    "dbeta_fw": ("dbeta", "betad", "dbeta_dps"),
    "Ahat_fw": ("Ahat", "ahat", "A_hat", "A"),
    "phase": ("phase",), "cue": ("cue",), "err": ("err",),
}
DERIVED = ["t", "u_phi", "u_ank", "a_alpha", "a_theta", "a_beta", "a_dphi", "a_dbeta", "a_Ahat",
           "a_psi", "a_fp", "a_bp", "s_phi", "s_beta", "s_dphi", "s_dbeta", "s_fp", "s_bp"]
PHASE_NAMES = {0: "IDLE", 1: "FOLD", 2: "REST", 3: "STOP", 4: "대기", 5: "발산", 6: "종료"}


def wrap180(x):
    return (x + 180.0) % 360.0 - 180.0


class Dataset:
    """원시 D행 + 파생열. 라이브 append(인과 상태 유지)와 전체 rebuild 둘 다 된다."""

    MAX_ROWS = 600_000        # 200 Hz × 50 분. 넘으면 앞을 버린다 (기록 파일에는 남아 있다)

    def __init__(self, pipe=None, plane=None):
        self.pipe = dict(PIPE_DEFAULT)
        if pipe:
            self.pipe.update(pipe)
        self.plane = plane                 # 시뮬 모델 상수 (P, r, slopeA0 …) — 없어도 된다
        self.name = ""
        self.source = ""
        self.header = None                 # D행 열 이름 (펌웨어가 준 대로)
        self.cmap = {}                     # canonical -> column index in header
        self.raw = {}                      # header name -> list[float]
        self.der = {k: [] for k in DERIVED}
        self.events = []                   # [t_ms, event, value]
        self.trials = []                   # dict rows (R)
        self.folds = []                    # dict rows (F)
        self.trial_header = None
        self.fold_header = None
        self.notes = []                    # 앱이 붙이는 설명 (형식 감지 등)
        self._reset_state()
        self._smooth_valid = 0

    # ---------------- 헤더/형식 ----------------
    def set_header(self, names):
        names = [n.strip() for n in names]
        if self.header == names:
            return
        if self.header is not None and self.n > 0:
            # 열 구성이 다른 헤더가 오면 다른 펌웨어/형식이다 — 섞지 않고 새로 시작한다 (기록 파일은 영향 없음)
            note = f"헤더 변경으로 버퍼 새로 시작: {','.join(self.header)} → {','.join(names)}"
            self.clear()
            self.notes.append(note)
        self.header = names
        self.cmap = {}
        for canon, alist in ALIASES.items():
            for i, nm in enumerate(names):
                if nm in alist:
                    self.cmap[canon] = i
                    break
        self.raw = {nm: [] for nm in names}
        missing = [k for k in ("t_ms", "phi", "ank", "del") if k not in self.cmap]
        if missing:
            self.notes.append("필수 열 없음: " + ",".join(missing) + " (있는 열로만 계산)")

    def col(self, canon, i):
        j = self.cmap.get(canon)
        if j is None:
            return float("nan")
        return self.raw[self.header[j]][i]

    @property
    def n(self):
        return len(self.der["t"])

    # ---------------- 행 추가 ----------------
    def add_data_row(self, payload):
        if self.header is None:
            self.set_header(__import__("serial_bridge").DEFAULT_D_HEADER.split(","))
        parts = payload.split(",")
        if len(parts) < len(self.header):
            return False
        vals = []
        for k, nm in enumerate(self.header):
            try:
                vals.append(float(parts[k]))
            except ValueError:
                vals.append(float("nan"))
        if not math.isfinite(vals[self.cmap.get("t_ms", 0)]):
            return False
        for nm, v in zip(self.header, vals):
            self.raw[nm].append(v)
        i = self.n
        self._derive_one(i)
        if self.n > self.MAX_ROWS:
            self._trim(self.MAX_ROWS // 10)
        return True

    def _dict_row(self, header, payload):
        parts = [p.strip() for p in payload.split(",")]
        d = {}
        for k, nm in enumerate(header):
            if k < len(parts):
                try:
                    d[nm] = float(parts[k])
                except ValueError:
                    d[nm] = parts[k]
        return d

    def add_trial_row(self, payload, header=None):
        hdr = header or self.trial_header or __import__("serial_bridge").TRIAL_HEADER.split(",")
        self.trial_header = hdr
        d = self._dict_row(hdr, payload)
        d["_src"] = "R"
        self.trials.append(d)

    def add_fold_row(self, payload, header=None):
        hdr = header or self.fold_header or __import__("serial_bridge").FOLD_HEADER.split(",")
        self.fold_header = hdr
        d = self._dict_row(hdr, payload)
        self.folds.append(d)

    def add_event_row(self, payload):
        parts = [p.strip() for p in payload.split(",")]
        if len(parts) < 2:
            return
        try:
            t_ms = float(parts[0])
        except ValueError:
            return
        self.events.append([t_ms, parts[1], parts[2] if len(parts) > 2 else ""])

    def add_event(self, t_ms, name, value=""):
        self.events.append([float(t_ms), str(name), str(value)])

    def last_t_ms(self):
        if self.n == 0:
            return 0.0
        return self.col("t_ms", self.n - 1)

    # ---------------- 파일 로드 ----------------
    def load_text(self, text, name="", source="file"):
        """p2r_logger 형 CSV(접두어 없음) / D행 접두어 CSV / raw.txt 모두 읽는다."""
        import serial_bridge as sb
        sink = sb.LineSink()
        self.clear()
        self.name, self.source = name, source
        hdr = None
        for ln in text.splitlines():
            s = ln.strip()
            if not s:
                continue
            kind, prefix, payload = sink.classify(s)
            if kind == "header":
                if prefix == "D":
                    self.set_header(payload)
                elif prefix == "R":
                    self.trial_header = payload
                elif prefix == "F":
                    self.fold_header = payload
                continue
            if kind == "data":
                self.add_data_row(payload)
            elif kind == "trial":
                self.add_trial_row(payload)
            elif kind == "fold":
                self.add_fold_row(payload)
            elif kind == "event":
                self.add_event_row(payload)
            elif kind == "dev":
                if hdr is None and "t_ms" in s and not s.startswith("#"):
                    hdr = [c.strip() for c in s.split(",")]
                    self.set_header(hdr)
                    self.notes.append("접두어 없는 CSV (p2r_logger 형식) 로 읽음")
                elif hdr is not None and s[0] in "-+0123456789":
                    self.add_data_row(s)
                elif s.startswith("t_ms,event"):
                    hdr = "events"
                elif hdr == "events" and s[0] in "0123456789":
                    self.add_event_row(s)
        self.rebuild()
        return self.n

    def load_events_text(self, text):
        for ln in text.splitlines():
            s = ln.strip()
            if not s or s.startswith("t_ms") or s.startswith("#"):
                continue
            self.add_event_row(s)

    def clear(self):
        self.header = None
        self.cmap = {}
        self.raw = {}
        self.der = {k: [] for k in DERIVED}
        self.events, self.trials, self.folds = [], [], []
        self.notes = []
        self._reset_state()
        self._smooth_valid = 0

    def _trim(self, k):
        for nm in self.raw:
            del self.raw[nm][:k]
        for nm in self.der:
            del self.der[nm][:k]
        self._smooth_valid = max(0, self._smooth_valid - k)
        self._hist_j = max(0, self._hist_j - k)

    # ---------------- 파이프라인 (인과) ----------------
    def _reset_state(self):
        self._t_off = 0.0
        self._t0_ms = None
        self._prev_ms = None
        self._prev_phi = None
        self._prev_ank = None
        self._u_phi = 0.0
        self._u_ank = 0.0
        self._ema_dphi = 0.0
        self._ema_dbeta = 0.0
        self._hist_j = 0

    def set_pipe(self, **kw):
        changed = False
        for k, v in kw.items():
            if k in PIPE_DEFAULT and self.pipe.get(k) != v:
                self.pipe[k] = v
                changed = True
        if changed:
            self.rebuild()
        return changed

    def rebuild(self):
        """원시 열은 그대로, 파생열 전부 재계산 (파라미터 변경·파일 로드 뒤)."""
        n = len(self.raw[self.header[0]]) if self.header else 0
        self.der = {k: [] for k in DERIVED}
        self._reset_state()
        for i in range(n):
            self._derive_one(i)
        self._smooth_valid = 0
        self.smooth_update(force_all=True)

    def _P(self):
        if self.plane and self.plane.get("P"):
            return self.plane["P"]
        lam = max(float(self.pipe["lam"]), 1e-3)
        return [[1.0 / lam, 0.0], [0.0, 1.0 / lam]]

    def _derive_one(self, i):
        p = self.pipe
        t_ms = self.col("t_ms", i)
        if self._t0_ms is None:
            self._t0_ms = t_ms
        if self._prev_ms is not None and t_ms < self._prev_ms - 1000.0:
            # 펌웨어 재시작 등으로 t_ms 가 되감김 → 연속 시간축 유지
            self._t_off += (self._prev_ms - self._t0_ms) / 1000.0 + 0.01
            self._t0_ms = t_ms
            self.add_event(t_ms, "T_RESET", f"{self._prev_ms:.0f}->{t_ms:.0f}")
        t = self._t_off + (t_ms - self._t0_ms) / 1000.0
        self._prev_ms = t_ms

        phi = self.col("phi", i)
        ank = self.col("ank", i)
        dl = self.col("del", i)
        if not math.isfinite(dl):
            dl = 0.0
        # 오프셋(직립 변환) → 언랩 (+감김수 스냅)
        if math.isfinite(phi):
            phi = wrap180(phi + p["phi_off"])
        if math.isfinite(ank):
            ank = wrap180(ank + p["ank_off"])
        if not math.isfinite(phi):
            phi = self._prev_phi if self._prev_phi is not None else 0.0
        if not math.isfinite(ank):
            ank = self._prev_ank if self._prev_ank is not None else 0.0
        if self._prev_phi is None or not p["unwrap"]:
            self._u_phi, self._u_ank = phi, ank
        else:
            self._u_phi += wrap180(phi - self._prev_phi)
            self._u_ank += wrap180(ank - self._prev_ank)
            if p["snap"]:
                if abs(self._u_phi - phi) > 180.0 and abs(phi) < 90.0:
                    self._u_phi = phi
                if abs(self._u_ank - ank) > 180.0 and abs(ank) < 90.0:
                    self._u_ank = ank
        self._prev_phi, self._prev_ank = phi, ank
        u_phi, u_ank = self._u_phi, self._u_ank

        mode = p["alpha_mode"]
        if mode == "fw" and "alpha_fw" in self.cmap:
            alpha = self.col("alpha_fw", i)
        elif mode == "ank+phi":
            alpha = u_ank + u_phi
        else:
            alpha = u_ank - u_phi
        theta = alpha + dl
        beta = alpha + p["p2r"] * dl

        # 속도: diff_ms 기저차분 + EMA(tau_ms)
        tt = self.der["t"]
        if i == 0:
            dphi_raw = dbeta_raw = 0.0
            self._ema_dphi = self._ema_dbeta = 0.0
            dt = 0.0
        else:
            j = self._hist_j
            while j + 1 < i and t - tt[j + 1] >= p["diff_ms"] / 1000.0:
                j += 1
            self._hist_j = j
            dtj = t - tt[j]
            if dtj > 1e-6:
                dphi_raw = (u_phi - self.der["u_phi"][j]) / dtj
                dbeta_raw = (beta - self.der["a_beta"][j]) / dtj
            else:
                dphi_raw, dbeta_raw = self._ema_dphi, self._ema_dbeta
            dt = t - tt[i - 1]
            a = dt / (p["tau_ms"] / 1000.0 + dt) if dt > 0 else 0.0
            self._ema_dphi += a * (dphi_raw - self._ema_dphi)
            self._ema_dbeta += a * (dbeta_raw - self._ema_dbeta)
        dphi, dbeta = self._ema_dphi, self._ema_dbeta

        r = p["r"] if abs(p["r"]) > 1e-9 else -1e9
        Ahat = (-1.0 / r) * u_phi + beta + p["vg"] * p["wf"] * dphi + p["vg"] * p["wb"] * dbeta + p["c0"] / r
        psi = u_phi - p["phi_eq"]
        P = self._P()
        fp = u_phi + P[0][0] * dphi + P[0][1] * dbeta
        bp = beta + P[1][0] * dphi + P[1][1] * dbeta

        d = self.der
        d["t"].append(t); d["u_phi"].append(u_phi); d["u_ank"].append(u_ank)
        d["a_alpha"].append(alpha); d["a_theta"].append(theta); d["a_beta"].append(beta)
        d["a_dphi"].append(dphi); d["a_dbeta"].append(dbeta); d["a_Ahat"].append(Ahat)
        d["a_psi"].append(psi); d["a_fp"].append(fp); d["a_bp"].append(bp)
        for k in ("s_phi", "s_beta", "s_dphi", "s_dbeta", "s_fp", "s_bp"):
            d[k].append(float("nan"))

    # ---------------- 비인과 평활 ----------------
    def smooth_arrays(self, i0, i1, W=None):
        """[i0,i1) 구간의 평활값/중앙차분을 numpy 로 계산 (양옆 마진 포함해서 읽는다)."""
        n = self.n
        if n == 0 or i1 <= i0:
            return None
        W = float(self.pipe["smooth_ms"] if W is None else W) / 1000.0
        t = np.asarray(self.der["t"], dtype=float)
        m0 = max(0, np.searchsorted(t, t[i0] - W) - 1)
        m1 = min(n, np.searchsorted(t, t[min(i1, n) - 1] + W) + 1)
        tt = t[m0:m1]
        out = {}
        for src, dst in (("u_phi", "s_phi"), ("a_beta", "s_beta")):
            x = np.asarray(self.der[src][m0:m1], dtype=float)
            xs, xd = centered_smooth_and_deriv(tt, x, W)
            out[dst] = xs
            out["s_d" + dst[2:]] = xd
        P = self._P()
        out["s_fp"] = out["s_phi"] + P[0][0] * out["s_dphi"] + P[0][1] * out["s_dbeta"]
        out["s_bp"] = out["s_beta"] + P[1][0] * out["s_dphi"] + P[1][1] * out["s_dbeta"]
        a, b = i0 - m0, i1 - m0
        return {k: v[a:b] for k, v in out.items()}

    def smooth_update(self, force_all=False):
        """평활열을 최신 상태로. 라이브에서는 아직 안 채운 꼬리(+마진)만 다시 계산한다."""
        n = self.n
        if n == 0:
            return 0
        W = float(self.pipe["smooth_ms"]) / 1000.0
        if force_all:
            i0 = 0
        else:
            t = self.der["t"]
            tv = t[max(0, self._smooth_valid - 1)] - W
            i0 = max(0, int(np.searchsorted(np.asarray(t), tv)) - 1)
        res = self.smooth_arrays(i0, n)
        if res is None:
            return 0
        for k, arr in res.items():
            lst = self.der[k]
            lst[i0:n] = [float(v) for v in arr]
        self._smooth_valid = n
        return i0

    # ---------------- 배열 꺼내기 ----------------
    def columns(self):
        return list(self.header or []) + DERIVED

    def matrix(self, i0=0, i1=None, cols=None):
        """float32 [cols][rows] — WS 바이너리 전송용."""
        i1 = self.n if i1 is None else min(i1, self.n)
        cols = cols or self.columns()
        out = np.empty((len(cols), max(0, i1 - i0)), dtype=np.float32)
        for k, c in enumerate(cols):
            src = self.raw.get(c) if c in self.raw else self.der.get(c)
            if src is None:
                out[k, :] = np.nan
            else:
                out[k, :] = np.asarray(src[i0:i1], dtype=np.float32)
        return out

    def arr(self, name):
        if name in self.der:
            return np.asarray(self.der[name], dtype=float)
        if name in self.raw:
            return np.asarray(self.raw[name], dtype=float)
        j = self.cmap.get(name)
        if j is not None:
            return np.asarray(self.raw[self.header[j]], dtype=float)
        return np.full(self.n, np.nan)

    def aux(self):
        return dict(events=self.events[-2000:], trials=self.trials[-500:], folds=self.folds[-500:],
                    header=self.header, trial_header=self.trial_header, fold_header=self.fold_header,
                    notes=self.notes[-20:], name=self.name, source=self.source, n=self.n,
                    pipe=self.pipe)


# ---------------------------------------------------------------- 수치 보조
def centered_smooth_and_deriv(t, x, W):
    """중심 이동평균(창 W [s], 시간 기준)과 같은 창의 중앙차분. 양끝은 창을 줄인다 (지연 0)."""
    n = len(t)
    if n == 0:
        return x, x
    if n == 1:
        return x.copy(), np.zeros(1)
    lo = np.searchsorted(t, t - W / 2, side="left")
    hi = np.searchsorted(t, t + W / 2, side="right")           # exclusive
    cs = np.concatenate([[0.0], np.cumsum(x)])
    cnt = np.maximum(hi - lo, 1)
    xs = (cs[hi] - cs[lo]) / cnt
    hi2 = np.minimum(hi - 1, n - 1)
    lo2 = np.minimum(lo, n - 1)
    dt = t[hi2] - t[lo2]
    xd = np.where(dt > 1e-6, (x[hi2] - x[lo2]) / np.where(dt > 1e-6, dt, 1.0), 0.0)
    return xs, xd
