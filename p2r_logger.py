# -*- coding: utf-8 -*-
"""
p2r_logger.py — 시리얼 모니터 + CSV 로거 (p2r_test.ino 용)
============================================================================
아두이노 시리얼 모니터를 **대체한다.** 화면에 값이 그대로 흐르고, 같은 창에서
명령을 칠 수 있고, 그 사이 데이터는 CSV 로 저장된다.
(기존엔 로거를 쓰려면 시리얼 모니터를 닫아야 해서 명령을 못 쳤다 — 그게 없어진다.)
 
사용:
    python p2r_logger.py 0817_p2r_r2    ← 포트는 알아서 찾는다
    python p2r_logger.py                      ← 파일명 생략하면 날짜로 자동 생성
    python p2r_logger.py r1.csv --port COM7   ← 포트를 직접 지정
    python p2r_logger.py --list               ← 포트 목록만 보기
    python p2r_logger.py --selftest           ← 장치 없이 파싱 로직 점검
 
화면에서:
    20   z   s   m   w   q   e   p   t   x    ← 그대로 치면 로봇으로 전달된다
    quit (또는 exit, Ctrl+C)                   ← 종료
 
만드는 파일 (셋 다 같은 이름 기준):
    <이름>.csv          D행 데이터        t_ms,phi_deg,ank_deg,alpha_deg,del_cmd_deg,del_now_deg
    <이름>.events.csv   E행 이벤트        t_ms,event,value   (δ 를 언제 명령했는지)
    <이름>.raw.txt      화면에 뜬 것 전부  (진단 메시지·부팅 배너 포함, 사후 검토용)
 
  ★같은 이름이 이미 있으면 덮지 않고 _2, _3 을 붙인다 (--force 로 덮어쓰기).
 
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
 
DEFAULT_HEADER = "t_ms,phi_deg,ank_deg,alpha_deg,del_cmd_deg,del_now_deg"
EVENT_HEADER = "t_ms,event,value"
PORT_HINTS = ("opencr", "stm", "arduino", "usb serial", "usbmodem",
              "ttyacm", "ch340", "cp210", "ftdi", "usb-serial")
 
 
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
        name = time.strftime("p2r_%m%d_%H%M%S.csv")
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
    """수신 행을 데이터/이벤트/그 밖으로 가른다. 장치 없이 단위시험 가능."""
 
    def __init__(self, data_prefix="D", event_prefix="E"):
        self.dp = data_prefix + ","
        self.ep = event_prefix + ","
        self.header = None
        self.n_data = 0
        self.n_event = 0
        self.last = None
 
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
            return "data", self.last
        if s.startswith(self.ep):
            self.n_event += 1
            self.last = s[len(self.ep):]
            return "event", self.last
        return "dev", s
 
 
def selftest():
    s = Sink()
    cases = [
        ("=== p2r_test (P2R 실측 + 배선 진단) ===",                          "dev"),
        ("# D,t_ms,phi_deg,ank_deg,alpha_deg,del_cmd_deg,del_now_deg",     "header"),
        ("E,1234,MOVE,20",                                                  "event"),
        ("D,1240,0.031,-8.191,-8.160,20,19.980",                            "data"),
        ("dcmd=+20  d=+19.98 | phi=+0.03  ank=-8.19  alpha=-8.16",          "dev"),
        ("",                                                                 None),
        ("RAW phi=10649 ank=8123 | phi n=2410 rail=0 EF=0 par=0",           "dev"),
    ]
    ok = True
    for text, want in cases:
        got, _ = s.feed(text)
        good = got == want
        ok &= good
        print(f"  [{'OK ' if good else 'FAIL'}] {want!s:<7} <- {text[:44]!r}")
    for name, got, want in [("헤더 자동 채택", s.header, DEFAULT_HEADER),
                            ("데이터 수", s.n_data, 1),
                            ("이벤트 수", s.n_event, 1)]:
        good = got == want
        ok &= good
        print(f"  [{'OK ' if good else 'FAIL'}] {name}: {got!r}")
    print("\n  전부 통과" if ok else "\n  ★실패 있음")
    return 0 if ok else 1
 
 
# ---------------------------------------------------------------- 수신 스레드
class Reader(threading.Thread):
    def __init__(self, ser, sink, f_csv, f_evt, f_raw):
        super().__init__(daemon=True)
        self.ser, self.sink = ser, sink
        self.f_csv, self.f_evt, self.f_raw = f_csv, f_evt, f_raw
        self.stop = threading.Event()
        self.header_written = False
        self.error = None
        self._last_flush = time.time()
 
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
            print(text)                                  # ★시리얼 모니터처럼 그대로
            if self.f_raw:
                self.f_raw.write(text + "\n")
 
            kind, payload = self.sink.feed(text)
            if kind == "data":
                if not self.header_written:
                    self.f_csv.write((self.sink.header or DEFAULT_HEADER) + "\n")
                    self.header_written = True
                self.f_csv.write(payload + "\n")
            elif kind == "event":
                self.f_evt.write(payload + "\n")
                self.f_evt.flush()
            self._maybe_flush()
 
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
        description="p2r_test.ino 시리얼 모니터 + CSV 로거",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="예:  python p2r_logger.py 0817_p2r_r1.csv")
    ap.add_argument("csv", nargs="?", help="저장할 CSV 이름 (생략하면 날짜로 자동)")
    ap.add_argument("--port", help="COM7 / /dev/ttyACM0 (생략하면 자동 탐지)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--prefix", default="D")
    ap.add_argument("--event-prefix", default="E")
    ap.add_argument("--send", help="연결 직후 장치로 보낼 명령 (예: --send m)")
    ap.add_argument("--force", action="store_true", help="같은 이름이 있으면 덮어쓰기")
    ap.add_argument("--no-raw", action="store_true", help="raw.txt 를 만들지 않음")
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
 
    print("=" * 68)
    print(f"  {port} @ {a.baud}")
    print(f"  데이터 -> {p_csv}")
    print(f"  이벤트 -> {p_evt}")
    if f_raw:
        print(f"  전체   -> {p_raw}")
    print("  이 창에서 그대로 명령을 치면 로봇으로 갑니다 (20 / z / s / m / w / q ...)")
    print("  종료: quit  또는  Ctrl+C")
    print("=" * 68)
 
    sink = Sink(a.prefix, a.event_prefix)
    rd = Reader(ser, sink, f_csv, f_evt, f_raw)
    rd.start()
 
    if a.send:
        time.sleep(2.0)                              # OpenCR 리셋·부팅 대기
        ser.write((a.send + "\n").encode())
 
    try:
        while not rd.stop.is_set():
            try:
                line = input()
            except EOFError:
                break
            if line.strip().lower() in ("quit", "exit"):
                break
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
    print(f"  {p_csv}        {sink.n_data} samples")
    print(f"  {p_evt}   {sink.n_event} events")
    if f_raw:
        print(f"  {p_raw}")
    if sink.n_data == 0:
        print("\n  ★D행이 0입니다. 확인할 것:")
        print("    1. 'm' 을 눌러 CSV 모드로 바꿔야 D행이 나옵니다 (기본은 읽기용 4Hz 표시).")
        print("    2. 's' 로 출력을 꺼둔 상태는 아닌지 — 한 번 더 눌러 재개하세요.")
        print("    3. raw.txt 에는 화면에 뜬 게 전부 남아 있으니 거기서 원인을 볼 수 있습니다.")
 
 
if __name__ == "__main__":
    main()