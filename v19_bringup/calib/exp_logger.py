# -*- coding: utf-8 -*-
"""
exp_logger.py — 초보자용 시리얼 로거 (기록 + 명령 전송 동시 지원)
==================================================================
기존 calib/serial_logger.py 와 달리, CSV로 기록하는 '동시에' 키보드로
펌웨어 명령(z, s, m, B, 3, f 12 등)을 보낼 수 있다.

사용:
  pip install pyserial
  python exp_logger.py COM3 0727_exp2_r1.csv          (Windows)
  python exp_logger.py /dev/cu.usbmodem1101 log.csv    (Mac)

- 실행하면 바로 기록이 시작된다 (D/E 행만 파일에 저장, 나머지는 화면에만).
- 아래 입력줄에 명령을 치고 Enter: 예) z → s → m → f 12
- 종료: quit 입력 또는 Ctrl+C
※ Arduino IDE의 시리얼 모니터가 열려 있으면 포트를 못 잡는다 — 먼저 닫을 것!
"""
import sys, threading, time

try:
    import serial
except ImportError:
    sys.exit("pyserial이 없습니다. 먼저:  pip install pyserial")

if len(sys.argv) < 3:
    sys.exit("사용법: python exp_logger.py <포트> <저장파일.csv>")

port, path = sys.argv[1], sys.argv[2]
try:
    ser = serial.Serial(port, 115200, timeout=0.2)
except Exception as e:
    sys.exit(f"포트 열기 실패({e}) — 포트 이름, 시리얼 모니터 종료 여부 확인")

stop = False
n_rows = 0

def reader():
    global n_rows
    with open(path, "w", encoding="utf-8") as f:
        while not stop:
            try:
                ln = ser.readline().decode(errors="ignore").strip()
            except Exception:
                continue
            if not ln:
                continue
            if ln.startswith("D,") or ln.startswith("E,"):
                f.write(ln + "\n")
                n_rows += 1
                if ln.startswith("E,"):
                    print(f"  [기록됨] {ln}")
            else:
                print("  " + ln)

t = threading.Thread(target=reader, daemon=True)
t.start()
print(f"기록 중 → {path}   (명령 입력 후 Enter, 종료는 quit)")
try:
    while True:
        cmd = input()
        if cmd.strip().lower() in ("quit", "exit", "q!"):
            break
        ser.write((cmd + "\n").encode())
except (KeyboardInterrupt, EOFError):
    pass
stop = True
time.sleep(0.4)
ser.close()
print(f"종료 — 데이터/이벤트 {n_rows}행 저장: {path}")
