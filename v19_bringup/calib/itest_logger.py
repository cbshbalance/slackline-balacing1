# -*- coding: utf-8 -*-
"""
itest_logger.py — motor_inertia_test 전용 '대화형' 로거 (실험 ③, M2)
====================================================================
이 창 하나로 ①로봇에 명령을 보내고 ②데이터를 CSV로 저장한다.

사용:
    python itest_logger.py COM6                (→ itest.csv 로 저장)
    python itest_logger.py COM6 mytest.csv
    (macOS/리눅스는 COM6 대신 /dev/ttyACM0 등)

- 이 창에 z / p / c 50 / i 60 / k / r / x 를 치고 Enter → 로봇으로 전달된다
- 로봇이 보내는 D,.../E,... 행은 그대로 CSV에 저장된다 (inertia_analysis.py 입력 형식)
- 그 외 안내 메시지(#..., !!...)는 화면에 표시된다
- 종료: quit 입력 (또는 Ctrl+C). 종료 시 안전을 위해 'x'(토크 OFF)를 자동 전송한다.

필요 패키지: pip install pyserial
⚠ Arduino IDE 시리얼 모니터가 열려 있으면 포트를 못 잡는다 — 먼저 닫을 것.
"""
import argparse
import re
import sys
import threading
import time

# 로봇이 이해하는 명령만 통과시킨다 (조각난 입력·오타가 그대로 전달되는 사고 방지)
CMD_OK = re.compile(r"^([zskrpx]|[ci]\s*-?\d+|t\s*-?\d*\.?\d+)$", re.IGNORECASE)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="예: COM6 (윈도우), /dev/ttyACM0 (리눅스)")
    ap.add_argument("out", nargs="?", default="itest.csv", help="저장 파일 (기본 itest.csv)")
    ap.add_argument("--baud", type=int, default=115200)
    a = ap.parse_args()

    try:
        import serial
    except ImportError:
        sys.exit("pyserial 필요:  pip install pyserial")

    try:
        ser = serial.Serial(a.port, a.baud, timeout=0.2)
    except Exception as e:
        sys.exit(f"포트를 못 열었습니다: {e}\n"
                 "  - Arduino 시리얼 모니터가 열려 있으면 닫으세요\n"
                 "  - 포트 이름(장치관리자/Arduino IDE 툴→포트) 확인")
    time.sleep(0.3)

    stop = threading.Event()

    def keyboard():
        while not stop.is_set():
            try:
                cmd = input()
            except (EOFError, KeyboardInterrupt):
                stop.set()
                break
            cmd = cmd.strip()
            if not cmd:
                continue
            if cmd.lower() in ("quit", "exit", "q"):
                stop.set()
                break
            if not CMD_OK.match(cmd):
                print(f"[logger] 무시함: '{cmd}'  (가능: z / p / c 50 / i 60 / k / r / x / quit)")
                continue
            try:
                ser.write((cmd + "\n").encode())
                print(f"[logger] 전송: {cmd}")
            except Exception:
                stop.set()
                break

    threading.Thread(target=keyboard, daemon=True).start()

    print(f"[logger] {a.port} @ {a.baud}  →  {a.out}")
    print("[logger] 이 창에 명령을 치고 Enter:  z / p / c 50 / i 60 / k / r / x")
    print("[logger] 종료: quit (종료 시 x 자동 전송으로 토크 OFF)")
    n = 0
    with open(a.out, "w", encoding="utf-8") as f:
        try:
            while not stop.is_set():
                try:
                    line = ser.readline().decode(errors="ignore").strip()
                except Exception:
                    break
                if not line:
                    continue
                if line.startswith("D,"):
                    f.write(line + "\n")            # 데이터는 조용히 저장만 (화면 도배 방지)
                    n += 1
                elif line.startswith("E,"):
                    f.write(line + "\n")
                    f.flush()
                    print("  " + line)              # 펄스 시작/종료 요약만 화면에
                else:
                    print("  " + line)              # 장치 안내 메시지
        except KeyboardInterrupt:
            pass

    stop.set()
    try:
        ser.write(b"x\n")                            # 안전: 나가면서 토크 OFF
        time.sleep(0.2)
    except Exception:
        pass
    ser.close()
    print(f"[logger] 저장 완료: {a.out} ({n} 행)")
    print(f"[logger] 다음:  python v19_bringup/calib/inertia_analysis.py {a.out}")


if __name__ == "__main__":
    main()
