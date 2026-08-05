# -*- coding: utf-8 -*-
"""
wiring_check.py — 실험 시작 전 배선·각도 3종 점검 (tilt_release_test.ino 용)
=============================================================================
φ 배선 헐거움 이력(값 고정/끊김) 때문에, 실험 전에 이 스크립트로
① 각도 3개(φ, 발목, δ)가 다 들어오는지 ② 값이 얼고(stuck) 있지 않은지
③ (선택) 부호 규약(--alpha 후보)까지 한 번에 확인한다.

사용:
  1) tilt_release_test.ino 업로드 (IDE 시리얼 모니터는 반드시 닫기!)
  2) python wiring_check.py COM6            ← 자동 건강 점검만 (로봇 가만히)
     python wiring_check.py COM6 --sign     ← 부호 점검(대화형)까지
  3) 전부 [OK]면 exp_logger.py 로 본 실험 진행

판정 기준:
  - 미수신     : D행이 안 들어옴 → 포트/업로드/보드 전원
  - RAW 0      : 부팅 메시지의 raw=0 → 그 엔코더 배선 자체가 죽음
  - 고정(stuck): 1초 이상 값이 완전히 동일 (다른 채널은 움직이는데) → 배선 헐거움 의심
  - 점프       : 인접 샘플 간 30° 이상 튐 → 접촉 불량/노이즈
  - 수신율     : 200Hz 기준 150Hz 미만이면 경고

pip install pyserial
"""
import argparse, sys, time


# ---------------- 파싱·판정 (오프라인 테스트 가능) ----------------

def parse_d(line):
    p = line.strip().split(",")
    if p[0] != "D" or len(p) < 5:
        return None
    try:
        return (float(p[1]) / 1000.0, float(p[2]), float(p[3]), float(p[4]))
    except ValueError:
        return None


def longest_freeze(vals, ts):
    """값이 '완전히 동일'하게 유지된 최장 구간 길이 [s]"""
    best, i = 0.0, 0
    for j in range(1, len(vals) + 1):
        if j == len(vals) or vals[j] != vals[i]:
            best = max(best, ts[j - 1] - ts[i])
            i = j
    return best


def analyze_health(rows, dur):
    """rows: [(t, phi, ank, dl)] → 채널별 진단 딕셔너리 리스트"""
    out = []
    if not rows:
        return None
    ts = [r[0] for r in rows]
    span = ts[-1] - ts[0] if len(ts) > 1 else 0.0
    rate = (len(rows) - 1) / span if span > 0 else 0.0
    for k, name in ((1, "phi(줄)"), (2, "ank(발목)"), (3, "delta(허리)")):
        v = [r[k] for r in rows]
        rng = max(v) - min(v)
        jumps = sum(1 for a, b in zip(v, v[1:]) if abs(b - a) > 30.0)
        frz = longest_freeze(v, ts)
        # δ는 서보 폴링이라 분해능이 굵어 미세 정지가 정상 → 기준 완화
        frz_lim = 2.0 if k == 3 else 1.0
        issues = []
        if all(x == 0.0 for x in v):
            issues.append("전부 0 — 미배선/영점 직후 무변화")
        if frz > frz_lim and rng > 0:
            issues.append(f"{frz:.1f}s 동안 값 고정 — 배선 헐거움 의심")
        if jumps:
            issues.append(f"{jumps}회 30°+ 점프 — 접촉 불량/노이즈")
        out.append(dict(name=name, n=len(v), rng=rng, frz=frz, jumps=jumps,
                        vmin=min(v), vmax=max(v), issues=issues))
    return dict(rate=rate, span=span, ch=out)


def print_health(h):
    print(f"\n  수신 {h['ch'][0]['n']}행 / {h['span']:.1f}s = {h['rate']:.0f} Hz "
          + ("[OK]" if h["rate"] >= 150 else "★경고: 200Hz 기준 미달 — USB/보드 확인"))
    ok_all = h["rate"] >= 150
    for c in h["ch"]:
        flag = "[OK]" if not c["issues"] else "★문제"
        print(f"  {c['name']:12s} {flag}  범위 [{c['vmin']:+8.3f}, {c['vmax']:+8.3f}]° "
              f"최장고정 {c['frz']:.2f}s 점프 {c['jumps']}회")
        for msg in c["issues"]:
            print(f"      → {msg}")
            ok_all = False
    return ok_all


# ---------------- 시리얼 유틸 ----------------

def open_port(port, baud):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial 필요:  pip install pyserial")
    try:
        return serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        sys.exit(f"포트 열기 실패({port}): {e}\n  ① IDE 시리얼 모니터 닫기 ② 포트 이름 ③ USB/전원")


def drain(ser, secs, echo_boot=False):
    """secs 동안 수신, D행 파싱 결과와 기타 행 반환"""
    rows, other = [], []
    t_end = time.time() + secs
    while time.time() < t_end:
        try:
            ln = ser.readline().decode(errors="ignore").strip()
        except Exception:
            break
        if not ln:
            continue
        d = parse_d(ln)
        if d:
            rows.append(d)
        else:
            other.append(ln)
            if echo_boot:
                print("   |", ln)
    return rows, other


def cmd(ser, ch, wait=0.3):
    ser.write(ch.encode()); ser.flush(); time.sleep(wait)


def collect(ser, secs):
    cmd(ser, "s")               # 스트리밍 ON
    rows, other = drain(ser, secs)
    cmd(ser, "s")               # OFF
    drain(ser, 0.3)
    return rows, other


def peak_dev(rows, k, base):
    """기준값 대비 '유지 구간'의 부호 있는 편차 — 드롭아웃에 강건한 판정.

    발목 엔코더는 간헐적으로 raw=0 드롭아웃이 나며, 이때 각도가 영점 기준
    ±180° 근처로 튄다(8/5 데이터에서 확인). 최대편차를 그대로 쓰면 이 한 샘플이
    결과를 지배하므로: |편차|>90°(살짝 밀기/기울이기에서 물리적으로 불가능)는
    글리치로 버리고, 남은 샘플 중 |편차| 상위 25%의 **중앙값**을 쓴다
    ('밀었다/기울였다 유지' 구간의 대표값). 글리치 개수도 함께 돌려준다."""
    dev = [r[k] - base for r in rows]
    glitch = sum(1 for d in dev if abs(d) > 90.0)
    clean = sorted((d for d in dev if abs(d) <= 90.0), key=abs)
    if not clean:
        return 0.0, glitch
    top = clean[-max(1, len(clean) // 4):]          # |편차| 큰 쪽 25% = 유지 구간
    top.sort()
    return top[len(top) // 2], glitch


# ---------------- 메인 ----------------

def main():
    ap = argparse.ArgumentParser(description="배선·각도 3종 점검 (tilt_release_test.ino)")
    ap.add_argument("port", help="예: COM6, /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--sign", action="store_true", help="부호 규약 점검(대화형)까지 수행")
    ap.add_argument("--selftest", help="(개발용) CSV 파일로 판정 로직만 시험")
    a = ap.parse_args()

    if a.selftest:                                     # 오프라인 검증 경로
        rows = [d for d in (parse_d(l) for l in open(a.selftest, encoding="utf-8", errors="ignore"))
                if d]
        h = analyze_health(rows, 0)
        sys.exit(0 if h and print_health(h) else 1)

    ser = open_port(a.port, a.baud)
    print(f"[1/3] 포트 {a.port} 연결. 부팅/상태 메시지 수신 중 (raw=0 경고 주시)...")
    _, boot = drain(ser, 2.0, echo_boot=True)
    for ln in boot:
        if "raw=0" in ln.replace(" ", "") or "경고" in ln:
            print("   ★부팅 경고 감지 — 해당 엔코더 배선부터 확인하라")
        if "응답없음" in ln:
            print("   ★DXL 응답 없음 — 배터리/RS-485 확인 (δ는 0으로 찍힌다)")

    print("\n[2/3] 자동 건강 점검 — 로봇을 건드리지 말 것 (5초). 영점(z) 후 측정.")
    cmd(ser, "z"); drain(ser, 0.3)
    rows, _ = collect(ser, 5.0)
    if not rows:
        sys.exit("★D행 미수신 — tilt_release_test.ino 업로드/포트/보레이트 확인")
    h = analyze_health(rows, 5.0)
    static_ok = print_health(h)
    # 정지 상태이므로 큰 범위 자체도 이상 신호
    for c in h["ch"][:2]:
        if c["rng"] > 2.0:
            print(f"  ★{c['name']}: 정지 상태인데 범위 {c['rng']:.2f}° — 실제로 움직였거나 노이즈")
            static_ok = False

    print("\n[2b/3] 흔들기 응답 점검 — 로봇을 손으로 잡고 '천천히 앞뒤로 살짝' 흔들어라 (8초).")
    input("  준비되면 Enter → ")
    rows2, _ = collect(ser, 8.0)
    h2 = analyze_health(rows2, 8.0)
    move_ok = True
    for c in h2["ch"][:2]:                              # phi·ank 는 반드시 반응해야
        if c["rng"] < 0.5:
            print(f"  ★{c['name']}: 흔들었는데 범위 {c['rng']:.2f}° < 0.5° — 안 움직임(배선/자석/조임)")
            move_ok = False
    print_health(h2)

    if not a.sign:
        print("\n[3/3] (부호 점검 생략 — 필요하면 --sign 로 재실행)")
    else:
        print("\n[3/3] 부호 규약 점검 — '+방향'을 하나 정하고(예: 특정 벽 쪽) 아래 3동작을 그 방향으로만.")
        cmd(ser, "z"); drain(ser, 0.3)

        input("  (a) 로봇은 세운 채 줄(크랭크)만 +방향으로 살짝 밀었다 유지 → Enter 후 4초 기록 ")
        r, _ = collect(ser, 4.0)
        dphi, g = peak_dev(r, 1, 0.0)
        if g: print(f"      (φ 드롭아웃 {g}샘플 제외)")
        print(f"      Δphi = {dphi:+.2f}° → 이 방향이 화면상 φ{'+' if dphi >= 0 else '−'} 이다")

        cmd(ser, "z"); drain(ser, 0.3)
        input("  (b) 줄은 가운데 두고 로봇 몸 전체를 같은 +방향으로 살짝 기울여 유지 → Enter 후 4초 기록 ")
        r, _ = collect(ser, 4.0)
        dank, g = peak_dev(r, 2, 0.0)
        if g: print(f"      (발목 드롭아웃 {g}샘플 제외 — 많으면 커넥터 재점검)")
        if abs(dank) < 0.3:
            print("      ★Δank가 너무 작음 — 기울임 유지 상태에서 다시 시도하라")
        s_phi = 1.0 if dphi >= 0 else -1.0
        # φ+ 방향으로 몸을 기울였을 때 α(=몸기울기)가 +가 되도록 하는 조합 추정
        agree = (dank * s_phi) >= 0
        print(f"      Δank = {dank:+.2f}° → 발목 부호가 φ와 {'일치' if agree else '반대'}")
        print(f"      ⇒ --alpha 후보: {'ank-phi (기본값 유지)' if agree else 'ank-phi 로 두되 분석 β 부호 확인 필요 — 정합 안 맞으면 ank+phi 시험'}")

        input("  (c) 허리 토크 끈 상태(u)에서 상체를 +방향으로 살짝 접기 → Enter 후 4초 기록 ")
        cmd(ser, "u"); drain(ser, 0.3)
        r, _ = collect(ser, 4.0)
        ddl, _g = peak_dev(r, 3, r[0][3] if r else 0.0)
        print(f"      Δdelta = {ddl:+.2f}° → +방향 접기가 δ{'+' if ddl >= 0 else '−'} "
              f"(MOTOR_DIR={'+1 유지' if ddl >= 0 else '-1 로 수정 검토(torque_cal 재확인)'})")
        print("      ★위 세 결과를 절차서 8.3 정합점검 기록란과 프로젝트 문서에 옮겨 적을 것")

    print("\n==== 종합 ====")
    if static_ok and move_ok:
        print("각도 3종 수신 정상. exp_logger.py 로 본 실험 진행 가능.")
    else:
        print("★문제 항목을 해결한 뒤 이 스크립트를 다시 실행하라. (φ 헐거움 이력: 커넥터 재삽입 후 케이블타이 고정 권장)")
    ser.close()


if __name__ == "__main__":
    main()
