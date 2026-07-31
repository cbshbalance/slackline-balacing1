# -*- coding: utf-8 -*-
"""
exp_logger.py — v19 실험 로거 (실험절차서 6.2절)
============================================================
사용:  python exp_logger.py COM3 0727_test.csv  [--baud 115200]
       (COM3 은 Arduino IDE > Tools > Port 에서 확인한 이름으로)

- 이 창이 시리얼 모니터를 겸한다: 명령(z, s, m, f 12 ...)을 치고
  Enter 하면 로봇으로 전달된다.
- 'D,'(데이터)·'E,'(이벤트)로 시작하는 행은 자동으로 CSV에 저장된다.
- 끝낼 때 quit 입력 → '데이터/이벤트 N행 저장'이 뜨면 성공.
- 실험마다 새 파일 이름 사용: 날짜_실험_회차.csv (예: 0727_exp2_r1.csv)

필요 패키지:  pip install pyserial
"""
import argparse
import sys
import threading
import time


def main():
    ap = argparse.ArgumentParser(description="v19 experiment serial logger")
    ap.add_argument("port", help="시리얼 포트 (예: COM3, /dev/ttyACM0)")
    ap.add_argument("outfile", help="저장할 CSV 파일 이름 (예: 0727_exp2_r1.csv)")
    ap.add_argument("--baud", type=int, default=115200)
    a = ap.parse_args()

    try:
        import serial
    except ImportError:
        sys.exit("pyserial 필요:  pip install pyserial")

    try:
        ser = serial.Serial(a.port, a.baud, timeout=0.2)
    except Exception as e:
        sys.exit(
            f"포트 열기 실패({a.port}): {e}\n"
            "  ① 아두이노 시리얼 모니터가 열려 있으면 닫기\n"
            "  ② 포트 이름 확인 (Arduino IDE > Tools > Port)\n"
            "  ③ USB 케이블·보드 전원 확인"
        )

    counts = {"D": 0, "E": 0}
    stop = threading.Event()
    lock = threading.Lock()
    f = open(a.outfile, "w", encoding="utf-8", newline="")

    def reader():
        while not stop.is_set():
            try:
                line = ser.readline().decode(errors="ignore").strip()
            except Exception:
                break
            if not line:
                continue
            if line.startswith("D,") or line.startswith("E,"):
                with lock:
                    f.write(line + "\n")
                counts[line[0]] += 1
                if line.startswith("E,"):
                    print("  [이벤트]", line)
                elif counts["D"] % 1000 == 0:
                    print(f"  ...데이터 {counts['D']}행 저장 중")
            else:
                print("  [보드]", line)

    th = threading.Thread(target=reader, daemon=True)
    th.start()

    print(f"[로거] {a.port} @ {a.baud} → {a.outfile}")
    print("[로거] 명령을 치고 Enter (예: z, s, m, f 12).  종료: quit")

    try:
        while True:
            cmd = input()
            if cmd.strip().lower() == "quit":
                break
            if cmd.strip():
                ser.write((cmd.strip() + "\n").encode())
    except (KeyboardInterrupt, EOFError):
        pass

    stop.set()
    time.sleep(0.4)
    with lock:
        f.close()
    ser.close()
    print(f"[로거] 데이터 {counts['D']}행 / 이벤트 {counts['E']}행 저장 → {a.outfile}")


if __name__ == "__main__":
    main()
