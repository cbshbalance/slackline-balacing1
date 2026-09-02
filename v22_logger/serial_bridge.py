# -*- coding: utf-8 -*-
"""
serial_bridge.py — v22 로거: 시리얼 소스 · 가짜 소스 · 행 분류 · 4파일 기록기
==============================================================================
hangcal_logger.py / fold_logger.py 가 하던 일(포트 자동탐지, D/R/E/F 행 분류, CSV 4파일
저장)을 v22 서버 안에서 쓰기 좋게 모듈로 옮긴 것이다. 규약은 그대로다:

    <이름>.csv          D행 (200/100 Hz 상태)   헤더는 펌웨어의 '# D,...' 주석을 그대로 채택
    <이름>.trials.csv   R행 (시행 요약)        '# R,...'
    <이름>.folds.csv    F행 (단일접기 요약)    '# F,...'
    <이름>.events.csv   E행 (이벤트)           t_ms,event,value  — 앱의 「마크」도 여기 들어간다
    <이름>.raw.txt      화면에 뜬 것 전부 (부팅 배너·상태 요약·명령 에코 포함)

★접속 직후 아무 글자도 먼저 보내지 않는다 (문서 77 §6). 명령은 사용자가 누를 때만 나간다.
★같은 이름이 이미 있으면 덮지 않고 _2, _3 을 붙인다.

장치 없이도 돌아가게 FakeSource 가 있다 — 기존 CSV/raw.txt 를 실시간 속도로 재생하거나
(파일 없으면) 감쇠 진동 신호를 합성한다. 개발·E2E·발표 리허설용.
"""
import collections
import math
import os
import threading
import time

PORT_HINTS = ("opencr", "stm", "arduino", "usb serial", "usbmodem",
              "ttyacm", "ch340", "cp210", "ftdi", "usb-serial")
DEFAULT_D_HEADER = "t_ms,phi,ank,alpha,beta,dphi,dbeta,Ahat,hold,del_now,phase,cue,err"
TRIAL_HEADER = "trial,dir,phi0,ank0,beta0,A0,t2_ms,t4_ms,t8_ms,lam24,lam48"
FOLD_HEADER = "trial,A_pre,d0,dd_cmd,dd_act,A_post,lock_ms,fold_ms,goaln"
EVENT_HEADER = "t_ms,event,value"
PREFIX_KIND = {"D": "data", "R": "trial", "F": "fold", "E": "event"}


# ---------------------------------------------------------------- 포트
def list_ports():
    """[{device, description, hwid, hint}] — pyserial 이 없으면 빈 목록 + 이유."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return [], "pyserial 이 없습니다:  pip install pyserial"
    out = []
    for p in list_ports.comports():
        text = (p.description + " " + p.device).lower()
        out.append(dict(device=p.device, description=p.description, hwid=p.hwid,
                        hint=any(h in text for h in PORT_HINTS)))
    return out, None


def autodetect_port():
    ps, err = list_ports()
    if err:
        return None, err
    if not ps:
        return None, "시리얼 포트가 없습니다 — USB 케이블·보드 전원을 확인하세요."
    if len(ps) == 1:
        return ps[0]["device"], f"포트 자동 선택: {ps[0]['device']} ({ps[0]['description']})"
    hits = [p for p in ps if p["hint"]]
    if len(hits) == 1:
        return hits[0]["device"], f"포트 자동 선택: {hits[0]['device']} ({hits[0]['description']})"
    return None, "포트가 여러 개입니다 — 목록에서 고르세요."


# ---------------------------------------------------------------- 행 분류
class LineSink:
    """수신 행을 header/data/trial/fold/event/dev 로 가른다. 장치 없이 단위시험 가능."""

    def __init__(self):
        self.headers = {"D": None, "R": None, "F": None, "E": None}
        self.counts = collections.Counter()

    def classify(self, line):
        s = line.rstrip("\r\n").strip()
        if not s:
            return None, None, None
        if s.startswith("#"):
            body = s.lstrip("# ").strip()
            if len(body) > 2 and body[0] in PREFIX_KIND and body[1] == ",":
                names = [c.strip() for c in body[2:].split(",")]
                if names and names[0]:
                    self.headers[body[0]] = names
                    self.counts["header"] += 1
                    return "header", body[0], names
            self.counts["dev"] += 1
            return "dev", None, s
        if len(s) > 2 and s[0] in PREFIX_KIND and s[1] == ",":
            kind = PREFIX_KIND[s[0]]
            self.counts[kind] += 1
            return kind, s[0], s[2:]
        self.counts["dev"] += 1
        return "dev", None, s

    def header_line(self, prefix):
        h = self.headers.get(prefix)
        if h:
            return ",".join(h)
        return {"D": DEFAULT_D_HEADER, "R": TRIAL_HEADER, "F": FOLD_HEADER,
                "E": EVENT_HEADER}[prefix]


# ---------------------------------------------------------------- 기록기
def resolve_stem(folder, name, force=False):
    """<folder>/<name> 을 기준으로 겹치지 않는 stem 을 돌려준다 (_2, _3 …)."""
    name = (name or "").strip()
    if name.lower().endswith(".csv"):
        name = name[:-4]
    if not name:
        name = time.strftime("%Y%m%d_%H%M%S")
    safe = "".join(c for c in name if c.isalnum() or c in "_-. ")
    safe = safe.strip() or time.strftime("%Y%m%d_%H%M%S")
    stem = os.path.join(folder, safe)
    if not force:
        k = 1
        base = stem
        while any(os.path.exists(stem + ext) for ext in (".csv", ".raw.txt")):
            k += 1
            stem = f"{base}_{k}"
    return stem


class Recorder:
    """4파일 기록기. 파일은 첫 행이 올 때 연다 (빈 파일을 남기지 않는다)."""
    FLUSH_S = 0.5

    def __init__(self, folder, name, sink, force=False, raw=True):
        os.makedirs(folder, exist_ok=True)
        self.folder = folder
        self.sink = sink
        self.stem = resolve_stem(folder, name, force)
        self.name = os.path.basename(self.stem)
        self.raw_on = raw
        self.files = {}          # key -> file
        self.header_done = set()
        self.n = collections.Counter()
        self.nbytes = 0
        self.t_open = time.time()
        self._last_flush = time.time()
        self.closed = False

    def _file(self, key):
        f = self.files.get(key)
        if f is None:
            ext = {"data": ".csv", "trial": ".trials.csv", "fold": ".folds.csv",
                   "event": ".events.csv", "raw": ".raw.txt"}[key]
            f = open(self.stem + ext, "a", encoding="utf-8", newline="")
            self.files[key] = f
        return f

    def write(self, kind, prefix, payload, raw_text):
        if self.closed:
            return
        if self.raw_on and raw_text is not None:
            self._file("raw").write(raw_text + "\n")
            self.nbytes += len(raw_text) + 1
        if kind in ("data", "trial", "fold", "event"):
            f = self._file(kind)
            if kind not in self.header_done:
                f.write(self.sink.header_line(prefix) + "\n")
                self.header_done.add(kind)
            f.write(payload + "\n")
            self.n[kind] += 1
            if kind != "data":
                f.flush()             # 시행·이벤트는 드물고 귀하다 — 바로 디스크에
        self._maybe_flush()

    def mark(self, t_ms, event, value=""):
        """앱에서 찍는 이벤트(마크)를 E 행으로 기록 + raw 에도 남긴다."""
        payload = f"{int(t_ms)},{event},{value}"
        self.write("event", "E", payload, "E," + payload + "   (앱 마크)")

    def _maybe_flush(self):
        now = time.time()
        if now - self._last_flush > self.FLUSH_S:
            self._last_flush = now
            for f in self.files.values():
                try:
                    f.flush()
                except Exception:
                    pass

    def close(self):
        if self.closed:
            return
        self.closed = True
        for f in self.files.values():
            try:
                f.close()
            except Exception:
                pass

    def info(self):
        files = [os.path.basename(self.stem) + ext for ext in
                 (".csv", ".trials.csv", ".folds.csv", ".events.csv", ".raw.txt")
                 if os.path.exists(self.stem + ext)]
        return dict(name=self.name, stem=self.stem, n_data=self.n["data"], n_trial=self.n["trial"],
                    n_event=self.n["event"], n_fold=self.n["fold"], nbytes=self.nbytes,
                    elapsed=round(time.time() - self.t_open, 1), files=files)


# ---------------------------------------------------------------- 소스
class LineSource(threading.Thread):
    """줄 단위 입력 소스의 공통 뼈대. 수신 스레드가 (호스트시각, 텍스트) 를 큐에 넣는다."""

    def __init__(self):
        super().__init__(daemon=True)
        self.q = collections.deque()
        self.lock = threading.Lock()
        self.stop_evt = threading.Event()
        self.error = None
        self.n_lines = 0
        self.t_start = time.time()

    def push(self, text):
        with self.lock:
            self.q.append((time.time(), text))
            self.n_lines += 1

    def drain(self):
        with self.lock:
            items = list(self.q)
            self.q.clear()
        return items

    def write(self, text):
        raise NotImplementedError

    def close(self):
        self.stop_evt.set()

    def describe(self):
        return dict(kind="none")


class SerialSource(LineSource):
    def __init__(self, port, baud=115200):
        super().__init__()
        import serial                       # pyserial
        self.port, self.baud = port, int(baud)
        self.ser = serial.Serial(port, self.baud, timeout=0.05)
        self.n_bytes = 0

    def run(self):
        buf = b""
        while not self.stop_evt.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception as e:
                self.error = f"포트가 끊겼습니다: {e}"
                self.push("# !! " + self.error)
                self.stop_evt.set()
                break
            if not chunk:
                continue
            self.n_bytes += len(chunk)
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.push(line.decode("utf-8", errors="replace").rstrip("\r"))
            if len(buf) > 4096:            # 줄바꿈 없는 폭주 방어
                self.push(buf.decode("utf-8", errors="replace"))
                buf = b""

    def write(self, text):
        data = (text.rstrip("\r\n") + "\n").encode("utf-8")
        self.ser.write(data)

    def close(self):
        super().close()
        try:
            self.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass

    def describe(self):
        return dict(kind="serial", port=self.port, baud=self.baud, n_bytes=self.n_bytes)


class FakeSource(LineSource):
    """장치 없이 쓰는 가짜 소스.
       file 이 CSV 면 D 행으로, raw.txt 면 줄 그대로, 없으면 합성 신호(감쇠 진동 + 놓기 발산)를 낸다.
       속도(speed)는 실시간 배율. 명령을 보내면 '# [fake] …' 로 에코한다."""

    def __init__(self, file=None, speed=1.0, loop=True, rate_hz=100.0):
        super().__init__()
        self.file, self.speed, self.loop, self.rate_hz = file, float(speed), bool(loop), float(rate_hz)
        self.lines = None
        if file:
            with open(file, encoding="utf-8", errors="replace") as fh:
                raw = [ln.rstrip("\r\n") for ln in fh]
            if file.lower().endswith(".csv"):
                hdr = None
                out = []
                for ln in raw:
                    if not ln.strip():
                        continue
                    if hdr is None:
                        if "t_ms" in ln:
                            hdr = ln
                            out.append("# D," + ln.strip())
                        continue
                    out.append("D," + ln.strip())
                self.lines = out
            else:
                self.lines = raw
        self.cmd_log = []

    @staticmethod
    def _t_of(line):
        if line.startswith("D,"):
            try:
                return float(line.split(",", 2)[1])
            except (ValueError, IndexError):
                return None
        return None

    def run(self):
        if self.lines is not None:
            self._run_file()
        else:
            self._run_synth()

    def _run_file(self):
        self.push("# [fake] 파일 재생 시작: " + os.path.basename(self.file) +
                  f"  ({len(self.lines)}줄, 속도 {self.speed}×)")
        while not self.stop_evt.is_set():
            t_prev = None
            host0 = time.time()
            ms0 = None
            for ln in self.lines:
                if self.stop_evt.is_set():
                    return
                tm = self._t_of(ln)
                if tm is not None:
                    if ms0 is None:
                        ms0 = tm
                        host0 = time.time()
                    if t_prev is not None and tm < t_prev - 1000:      # t_ms 리셋
                        ms0 = tm
                        host0 = time.time()
                    due = host0 + (tm - ms0) / 1000.0 / max(self.speed, 1e-6)
                    wait = due - time.time()
                    if wait > 0:
                        self.stop_evt.wait(wait)
                    t_prev = tm
                self.push(ln)
            if not self.loop:
                self.push("# [fake] 파일 끝")
                return
            self.push("# [fake] 파일 끝 — 처음부터 다시")

    def _run_synth(self):
        self.push("# [fake] 합성 신호 (감쇠 진동 φ, ank≈0.33φ, 20 s 마다 놓기 발산)")
        self.push("# D," + DEFAULT_D_HEADER)
        dt = 1.0 / self.rate_hz
        t0 = time.time()
        k = 0
        w, z = 4.86, 0.03
        phase = 0
        while not self.stop_evt.is_set():
            t = k * dt
            tc = t % 20.0
            if tc < 14.0:                       # 매달림 자유흔들기 (감쇠 진동)
                phi = 4.0 * math.exp(-z * w * tc) * math.cos(w * tc)
                phase = 0
            else:                               # 놓기 → 발산 (λ=5.5) → 8° 에서 잡기
                psi = 0.4 * math.exp(5.5 * (tc - 14.0))
                phi = min(psi, 8.5) if (int(t // 20) % 2 == 0) else -min(psi, 8.5)
                phase = 5 if abs(phi) < 8.4 else 6
            ank = 0.33 * phi + 0.05 * math.sin(37.0 * t)
            alpha = ank - phi
            delta = 0.0
            beta = alpha + 0.4285 * delta
            self.push("D,%d,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.4f,%.2f,%.2f,%d,0,0" % (
                int(t * 1000), phi, ank, alpha, beta, 0.0, 0.0, 0.0, 0.0, delta, phase))
            k += 1
            due = t0 + (k * dt) / max(self.speed, 1e-6)
            wait = due - time.time()
            if wait > 0:
                self.stop_evt.wait(wait)

    def write(self, text):
        text = text.rstrip("\r\n")
        self.cmd_log.append(text)
        self.push("# [fake] 명령 수신: " + text)
        if text.strip() == "t":
            self.push("---- 상태 ----")
            self.push("모드: 가짜 소스   제어: 정지")
        elif text.strip() == "p":
            self.push("f=0.00 k=0.00 d=0.00 | a=0.00 b=0.00 A=0.000 hold=0.0")

    def describe(self):
        return dict(kind="fake", file=os.path.basename(self.file) if self.file else None,
                    speed=self.speed)
