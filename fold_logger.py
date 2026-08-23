# -*- coding: utf-8 -*-
"""
fold_logger.py — 시리얼 모니터 + CSV 로거 (incremental_fold_finale5.ino 용)
============================================================================
아두이노 시리얼 모니터를 **대체한다.** 화면에 값이 그대로 흐르고, 같은 창에서
명령을 칠 수 있고, 그 사이 데이터는 CSV 로 저장된다.
(로거를 쓰려고 시리얼 모니터를 닫을 필요가 없다 — 명령을 그대로 친다.)

사용:
    python fold_logger.py 0823_run1           ← 포트는 알아서 찾는다
    python fold_logger.py                     ← 파일명 생략하면 날짜로 자동 생성
    python fold_logger.py r1.csv --port COM7  ← 포트를 직접 지정
    python fold_logger.py run1 --send m       ← 연결하자마자 보드의 CSV 를 켠다
    python fold_logger.py --list              ← 포트 목록만 보기
    python fold_logger.py --selftest          ← 장치 없이 파싱 로직 점검

화면에서:
    z  g  h  x  k  u  y  n  j  m  s  p  t  w  d  ?     ← 그대로 치면 로봇으로 전달된다
    lam 5.42   trig 0.8   gam 11   -20                 ← 값 바꾸기·수동 이동도 그대로
    #손으로 기울임                                      ← ★# 로 시작하면 보내지 않고 표시만 남긴다
    quit (또는 exit, Ctrl+C)                            ← 종료

만드는 파일 (셋 다 같은 이름 기준):
    <이름>.csv          D행 데이터        t_ms,phi,ank,alpha,beta,dphi,dbeta,Ahat,hold,del_now,phase,cue,err
    <이름>.events.csv   사건             t_ms,event,value   (READY·STOP·센서고장·접기상한·MARK…)
    <이름>.raw.txt      화면에 뜬 것 전부  (진단 메시지·부팅 배너·파라미터 덤프 포함)

  ★같은 이름이 이미 있으면 덮지 않고 _2, _3 을 붙인다 (--force 로 덮어쓰기).

  ★이 펌웨어는 E 행을 찍지 않는다. 그래서 events.csv 는 화면 메시지에서 뽑는다
    (>>> STOP, # READY, << phi 고장 >>, ?  접기량 상한 …). 값이 아니라 '언제 무슨 일이
    있었는지' 를 CSV 시간축에 맞춰 두기 위한 것이다.

필요 패키지:  pip install pyserial
"""
import argparse
import os
import sys
import threading
import time

try:                                    # 윈도우 cmd(cp949)에서 한글·° 가 깨지지 않게
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

DEFAULT_HEADER = ("t_ms,phi,ank,alpha,beta,dphi,dbeta,Ahat,hold,del_now,phase,cue,err")
EVENT_HEADER = "t_ms,event,value"
PORT_HINTS = ("opencr", "stm", "arduino", "usb serial", "usbmodem",
              "ttyacm", "ch340", "cp210", "ftdi", "usb-serial")

# 화면 메시지 -> 사건 이름.  (찾을 문자열, 사건 이름) 을 위에서부터 본다.
EVENT_RULES = (
    (">>> STOP",             "STOP"),          # 비상정지·한계각·센서고장·토크풀림
    ("# ZERO",               "ZERO"),
    ("# GO",                 "GO"),
    ("# 제어 정지",           "HALT"),
    ("# READY",              "READY"),
    ("# ready 해제",          "READY_OFF"),
    ("# 잡음 측정",           "NOISE_START"),
    ("Â 잡음 바닥",           "NOISE_REPORT"),
    ("접기량 상한",           "STEP_CLIP"),      # 잡음 한 샘플이 크게 커밋하려 한 것
    ("<< phi",               "FAULT_PHI"),
    ("<< ank",               "FAULT_ANK"),
    ("<< dxl",               "FAULT_DXL"),
    ("기구한계 밖",           "DELTA_OOR"),
    ("# 시작 정렬",           "ALIGN"),
    ("# dry-run",            "DRYRUN"),
    ("# 수동 delta",          "MANUAL"),
    ("# 1단계 성공",          "RECOVER"),
    ("# 2단계 성공",          "RECOVER_REBOOT"),
    ("# w 재계산",            "PARAM"),
)


def csv_quote(s):
    """events.csv 의 value 칸 — 쉼표가 들어 있어도 열이 안 밀리게."""
    s = s.replace('"', "'").strip()
    return '"' + s + '"' if ("," in s or '"' in s) else s


# ---------------------------------------------------------------- 포트 찾기
def comports():
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    return list(list_ports.comports())


def show_ports():
    ps = comports()
    if ps is None:
        print("  pyserial 이 없습니다:  pip install pyserial")
        return
    if not ps:
        print("  시리얼 포트가 하나도 없습니다 — USB 케이블·보드 전원을 확인하세요.")
        return
    for p in ps:
        print(f"    {p.device}\t{p.description}")


def autodetect_port():
    """포트를 알아서 고른다. (찾은 포트, 안내문) 을 돌려준다."""
    ps = comports()
    if not ps:
        return None, "시리얼 포트가 없습니다 — USB 케이블·보드 전원을 확인하세요."
    if len(ps) == 1:
        return ps[0].device, f"포트 자동 선택: {ps[0].device}  ({ps[0].description})"
    hits = [p for p in ps
            if any(h in (p.description + " " + p.device).lower() for h in PORT_HINTS)]
    if len(hits) == 1:
        return hits[0].device, f"포트 자동 선택: {hits[0].device}  ({hits[0].description})"
    lines = ["포트가 여러 개라 고를 수 없습니다. --port 로 지정하세요:"]
    for p in ps:
        lines.append(f"    {p.device}\t{p.description}")
    return None, "\n".join(lines)


# ---------------------------------------------------------------- 파일 이름
def resolve_names(name, force):
    if not name:
        name = time.strftime("fold_%m%d_%H%M%S.csv")
    if not name.lower().endswith(".csv"):
        name += ".csv"
    stem = name[:-4]
    if not force:
        k = 1
        while os.path.exists(stem + ".csv"):
            k += 1
            stem = f"{name[:-4]}_{k}"
    return stem + ".csv", stem + ".events.csv", stem + ".raw.txt"


# ---------------------------------------------------------------- 행 분류
class Sink:
    """수신 행을 데이터/사건/그 밖으로 가른다. 장치 없이 단위시험 가능."""

    def __init__(self, data_prefix="D"):
        self.dp = data_prefix + ","
        self.header = None
        self.n_data = 0
        self.n_event = 0
        self.last = None            # 마지막 D행 payload (헤더 뺀 나머지)
        self.t_ms = 0               # 보드 시각 — 사건에 붙일 시간축
        self.n_err = 0              # err != 0 인 D행
        self.err_mask = 0
        self.max_A = 0.0
        self.max_hold = 0.0

    def feed(self, line):
        line = line.rstrip("\r\n")
        s = line.strip()
        if not s:
            return None, None
        if s.startswith("#") and self.dp in s:          # 펌웨어가 찍는 헤더 주석
            self.header = s.lstrip("# ").split(",", 1)[1]
            return "header", None
        if s.startswith(self.dp):
            self.n_data += 1
            self.last = s[len(self.dp):]
            self._scan(self.last)
            return "data", self.last
        for pat, name in EVENT_RULES:                   # E 행이 없으므로 메시지에서 뽑는다
            if pat in s:
                self.n_event += 1
                return "event", f"{self.t_ms},{name},{csv_quote(s)}"
        return "dev", s

    def _scan(self, payload):
        """요약에 쓸 값만 훑는다. 파싱이 안 되면 조용히 넘어간다."""
        f = payload.split(",")
        try:
            self.t_ms = int(f[0])
        except (ValueError, IndexError):
            pass
        try:
            self.max_A = max(self.max_A, abs(float(f[7])))
            self.max_hold = max(self.max_hold, abs(float(f[8])))
        except (ValueError, IndexError):
            pass
        try:
            e = int(f[12])
            if e:
                self.n_err += 1
                self.err_mask |= e
        except (ValueError, IndexError):
            pass

    def err_channels(self):
        m, out = self.err_mask, []
        if m & 0x03:
            out.append("phi")
        if (m >> 2) & 0x03:
            out.append("ank")
        if m & 16:
            out.append("dxl")
        return "+".join(out) if out else "-"


def selftest():
    s = Sink()
    cases = [
        ("=== incremental_fold — 증분접기 제어 ===",                            "dev"),
        ("# D,t_ms,phi,ank,alpha,beta,dphi,dbeta,Ahat,hold,del_now,phase,cue,err", "header"),
        ("# Ahat 는 control_step 이전 값이다 (문서 46 §9)",                      "dev"),
        ("D,1240,0.031,-8.191,-8.160,-8.160,0.38,0.26,0.7312,5.70,5.68,1,0,0",  "data"),
        ("D,1260,0.031,-8.191,-8.160,-8.160,0.38,0.26,-0.9000,5.70,5.68,0,0,4", "data"),
        ("RUN IDLE | A=+0.731 | b=-8.16 f=+0.03 k=-8.19 | hold=+5.70",          "dev"),
        ("# READY  Ahat=0.104",                                                  "event"),
        ("<< ank 고장 raw=16383 정지 1520 ms >>",                                "event"),
        (">>> STOP (torque off) : 발목 엔코더 이상",                              "event"),
        ("?  접기량 상한 20 deg 로 잘림 (요청 34.2, Ahat=1.80)",                  "event"),
        ("",                                                                     None),
    ]
    ok = True
    for text, want in cases:
        got, _ = s.feed(text)
        good = got == want
        ok &= good
        print(f"  [{'OK ' if good else 'FAIL'}] {want!s:<7} <- {text[:46]!r}")
    for name, got, want in [("헤더 자동 채택", s.header, DEFAULT_HEADER),
                            ("데이터 수", s.n_data, 2),
                            ("사건 수", s.n_event, 4),
                            ("err 행 수", s.n_err, 1),
                            ("err 채널", s.err_channels(), "ank"),
                            ("|Ahat| 최대", round(s.max_A, 3), 0.9),
                            ("보드 시각", s.t_ms, 1260)]:
        good = got == want
        ok &= good
        print(f"  [{'OK ' if good else 'FAIL'}] {name}: {got!r}")
    print("\n  전부 통과" if ok else "\n  ★실패 있음")
    return 0 if ok else 1


# ---------------------------------------------------------------- 수신 스레드
class Reader(threading.Thread):
    def __init__(self, ser, sink, f_csv, f_evt, f_raw, quiet=False):
        super().__init__(daemon=True)
        self.ser, self.sink = ser, sink
        self.f_csv, self.f_evt, self.f_raw = f_csv, f_evt, f_raw
        self.quiet = quiet                              # D행은 화면에 안 찍기 (--quiet)
        self.stop = threading.Event()
        self.header_written = False
        self.error = None
        self._last_flush = time.time()
        self._last_beat = time.time()

    def run(self):
        while not self.stop.is_set():
            try:
                raw = self.ser.readline()
            except Exception as e:
                self.error = e
                self.stop.set()
                print(f"\n[logger] 포트가 끊겼습니다: {e}\n[logger] Enter 를 눌러 종료하세요.")
                break
            if not raw:
                self._maybe_flush()
                continue
            text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            kind, payload = self.sink.feed(text)

            if kind == "data":
                if not self.header_written:
                    self.f_csv.write((self.sink.header or DEFAULT_HEADER) + "\n")
                    self.header_written = True
                self.f_csv.write(payload + "\n")
            elif kind == "event":
                self.f_evt.write(payload + "\n")
                self.f_evt.flush()

            if self.f_raw:
                self.f_raw.write(text + "\n")

            if kind == "data" and self.quiet:           # 50 Hz 를 다 찍으면 << >> 를 놓친다
                self._beat()
            else:
                print(text)                             # ★시리얼 모니터처럼 그대로

            self._maybe_flush()

    def _beat(self):
        """--quiet 일 때 살아 있다는 표시만 1 초에 한 줄."""
        now = time.time()
        if now - self._last_beat < 1.0:
            return
        self._last_beat = now
        f = (self.sink.last or "").split(",")
        try:
            print(f"[rec {self.sink.n_data:6d}]  t={f[0]:>7}ms  A={float(f[7]):+.3f}"
                  f"  b={float(f[4]):+.2f}  f={float(f[2]):+.2f}"
                  f"  hold={float(f[8]):+.2f}  d={float(f[9]):+.2f}"
                  + ("  ★err" if f[12] != "0" else ""))
        except (ValueError, IndexError):
            print(f"[rec {self.sink.n_data:6d}]")

    def _maybe_flush(self):
        now = time.time()
        if now - self._last_flush > 0.5:                 # 뽑혀도 최근 0.5초만 잃도록
            self._last_flush = now
            for f in (self.f_csv, self.f_evt, self.f_raw):
                if f:
                    try:
                        f.flush()
                    except Exception:
                        pass


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(
        description="incremental_fold_finale5.ino 시리얼 모니터 + CSV 로거",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="예:  python fold_logger.py 0823_run1.csv --send m")
    ap.add_argument("csv", nargs="?", help="저장할 CSV 이름 (생략하면 날짜로 자동)")
    ap.add_argument("--port", help="COM7 / /dev/ttyACM0 (생략하면 자동 탐지)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--prefix", default="D")
    ap.add_argument("--send", help="연결 직후 장치로 보낼 명령 (예: --send m)")
    ap.add_argument("--force", action="store_true", help="같은 이름이 있으면 덮어쓰기")
    ap.add_argument("--no-raw", action="store_true", help="raw.txt 를 만들지 않음")
    ap.add_argument("--quiet", action="store_true",
                    help="D행은 화면에 안 찍고 1초 요약만 (진단줄이 흘러가지 않게)")
    ap.add_argument("--list", action="store_true", help="포트 목록만 출력")
    ap.add_argument("--selftest", action="store_true", help="장치 없이 파싱 점검")
    a = ap.parse_args()

    if a.selftest:
        print("[selftest] 행 분류 로직 점검")
        sys.exit(selftest())

    if a.list:
        print("사용 가능한 포트:")
        show_ports()
        sys.exit(0)

    try:
        import serial
    except ImportError:
        sys.exit("pyserial 이 없습니다.  설치:  pip install pyserial\n"
                 "  (윈도우에서 python 이 안 잡히면:  py -m pip install pyserial)")

    port = a.port
    if not port:
        port, msg = autodetect_port()
        print(msg)
        if not port:
            sys.exit(1)

    try:
        ser = serial.Serial(port, a.baud, timeout=0.2)
    except Exception as e:
        print(f"[오류] 포트 '{port}' 를 열 수 없습니다: {e}\n")
        print("  ★가장 흔한 원인: 아두이노 IDE 의 시리얼 모니터가 포트를 잡고 있음 — 닫고 다시 실행")
        print("  (이 로거가 시리얼 모니터를 대신하므로, 앞으로는 모니터를 열 필요가 없습니다)\n")
        print("사용 가능한 포트:")
        show_ports()
        sys.exit(1)

    p_csv, p_evt, p_raw = resolve_names(a.csv, a.force)
    f_csv = open(p_csv, "w", encoding="utf-8", newline="")
    f_evt = open(p_evt, "w", encoding="utf-8", newline="")
    f_evt.write(EVENT_HEADER + "\n")
    f_raw = None if a.no_raw else open(p_raw, "w", encoding="utf-8")

    print("=" * 72)
    print(f"  {port} @ {a.baud}")
    print(f"  데이터 -> {p_csv}")
    print(f"  사건   -> {p_evt}")
    if f_raw:
        print(f"  전체   -> {p_raw}")
    print("  이 창에서 그대로 명령을 치면 로봇으로 갑니다 (z / g / h / m / t / lam 5.42 ...)")
    print("  #으로 시작하면 보내지 않고 MARK 로 기록만 합니다  (예: #손으로 기울임)")
    print("  ★비상정지는 x — 보드는 x 를 받는 즉시 토크를 끊습니다")
    print("  종료: quit  또는  Ctrl+C")
    print("=" * 72)

    sink = Sink(a.prefix)
    rd = Reader(ser, sink, f_csv, f_evt, f_raw, quiet=a.quiet)
    rd.start()

    if a.send:
        time.sleep(2.0)                              # OpenCR 리셋·부팅 대기
        ser.write((a.send + "\n").encode())

    n_mark = 0
    try:
        while not rd.stop.is_set():
            try:
                line = input()
            except EOFError:
                break
            if line.strip().lower() in ("quit", "exit"):
                break
            if line.startswith("#"):                 # ★보내지 않는다 — 시간표시만 남긴다
                n_mark += 1
                note = line[1:].strip() or f"mark{n_mark}"
                f_evt.write(f"{sink.t_ms},MARK,{csv_quote(note)}\n")
                f_evt.flush()
                if f_raw:
                    f_raw.write(f"# MARK t={sink.t_ms} {note}\n")
                print(f"[mark] t={sink.t_ms} ms  {note}")
                continue
            try:
                ser.write((line + "\n").encode())
            except Exception as e:
                print(f"[logger] 전송 실패: {e}")
                break
    except KeyboardInterrupt:
        pass
    finally:
        rd.stop.set()
        rd.join(timeout=1.5)
        for f in (f_csv, f_evt, f_raw):
            if f:
                f.flush()
                f.close()
        try:
            ser.close()
        except Exception:
            pass

    print(f"\n[logger] 저장 완료")
    print(f"  {p_csv}          {sink.n_data} samples"
          + (f"   (보드시각 {sink.t_ms/1000.0:.1f} s 까지)" if sink.t_ms else ""))
    print(f"  {p_evt}   {sink.n_event} events" + (f" + MARK {n_mark}" if n_mark else ""))
    if f_raw:
        print(f"  {p_raw}")
    if sink.n_data:
        print(f"  |Ahat| 최대 {sink.max_A:.2f} deg   |hold| 최대 {sink.max_hold:.2f} deg")
        if sink.n_err:
            print(f"  ★err 행 {sink.n_err} / {sink.n_data}"
                  f" ({100.0*sink.n_err/sink.n_data:.1f} %)  채널 {sink.err_channels()}"
                  f"  → 그 구간은 버릴 것")
        else:
            print("  err 0 — 전 구간 정상")
    else:
        print("\n  ★D행이 0입니다. 확인할 것:")
        print("    1. 'm' 을 눌러 CSV 로그를 켜야 D행이 나옵니다 (기본은 250 ms 상태표시).")
        print("    2. 's' 로 출력을 꺼둔 상태는 아닌지 — 한 번 더 눌러 재개하세요.")
        print("    3. raw.txt 에는 화면에 뜬 게 전부 남아 있으니 거기서 원인을 볼 수 있습니다.")


if __name__ == "__main__":
    main()
