# -*- coding: utf-8 -*-
"""v22 수용시험 (헤드리스). 실행: cd v22_logger && python tests_v22.py

 1. 행 분류(LineSink)·기록기(Recorder) — 장치 없이
 2. Dataset: 두 CSV 형식 로드, 언랩·파생열, 라이브 append == 전체 rebuild (결정론)
 3. 분석 정본 대조: P2R(문서 64/70: 0.4285 ± 0.0011, R² 0.99996) · λ(문서 70 §5: 5.44 ± 0.59, 방향 갈림 <20 %)
 4. FakeSource → LoggerHub 펌프 → 바이너리 프레임 왕복 (서버 경로)
"""
import io
import json
import os
import struct
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np

import serial_bridge as sb
from dataset_v22 import Dataset
import analysis_v22 as an

ROOT = os.path.join(HERE, "..")
LAMBDA_CSV = os.path.join(ROOT, "lambda test", "0822_lambda_test.csv")
P2R_CSV = os.path.join(ROOT, "p2r test", "0822_p2r_test.csv")
FAILS = []


def check(name, cond, detail=""):
    print(f"  [{'OK ' if cond else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))
    if not cond:
        FAILS.append(name)


def test_sink_recorder():
    print("=== 1. 행 분류 · 기록기 ===")
    s = sb.LineSink()
    cases = [("# D,t_ms,phi,ank,alpha,beta,dphi,dbeta,Ahat,hold,del_now,phase,cue,err", "header"),
             ("# R,trial,dir,phi0,ank0,beta0,A0,t2_ms,t4_ms,t8_ms,lam24,lam48", "header"),
             ("D,1240,0.031,-8.191,-8.160,-8.13,0.5,1.2,0.09,0,0.00,0,0,0", "data"),
             ("R,1,+1,0.52,-0.31,0.21,0.61,95,220,345,5.54,5.54", "trial"),
             ("F,1,0.7,0,12,11.2,0.1,150,90,3", "fold"),
             ("E,1234,FOLD,5.13", "event"), ("# zero set — deadband 0.4 deg", "dev"),
             ("★첫 접기 — 크랭크쪽 손을 y 축으로 빼라", "dev"), ("", None)]
    for text, want in cases:
        kind, _, _ = s.classify(text)
        check(f"classify {want!s:<7} <- {text[:30]!r}", kind == want, str(kind))
    check("D 헤더 채택", s.headers["D"][0] == "t_ms" and len(s.headers["D"]) == 13)
    with tempfile.TemporaryDirectory() as d:
        rec = sb.Recorder(d, "t1", s)
        for text, _ in cases:
            kind, prefix, payload = s.classify(text)
            if kind:
                rec.write(kind, prefix, payload, text)
        rec.mark(2000, "MARK", "손")
        rec.close()
        files = sorted(os.listdir(d))
        check("4+1 파일 생성", set(files) >= {"t1.csv", "t1.trials.csv", "t1.folds.csv", "t1.events.csv", "t1.raw.txt"}, str(files))
        csv = open(os.path.join(d, "t1.csv"), encoding="utf-8").read().splitlines()
        check("D 파일 헤더 = 펌웨어 주석", csv[0].startswith("t_ms,phi,ank"), csv[0])
        ev = open(os.path.join(d, "t1.events.csv"), encoding="utf-8").read().splitlines()
        check("E 파일에 마크 포함", any("MARK" in l for l in ev), str(ev))
        rec2 = sb.Recorder(d, "t1", s)
        check("이름 충돌 시 _2", rec2.name == "t1_2", rec2.name)
        rec2.close()


def test_dataset():
    print("=== 2. Dataset ===")
    ds = Dataset()
    n = ds.load_text(open(LAMBDA_CSV, encoding="utf-8").read(), "lambda")
    check("p2r_logger 형식 로드", n == 9087, str(n))
    al = ds.arr("a_alpha"); ank = ds.arr("u_ank"); phi = ds.arr("u_phi")
    check("α = ank − φ 재계산", np.allclose(al, ank - phi), f"max|Δ|={np.max(np.abs(al - (ank - phi))):.2e}")
    check("t 연속·단조", np.all(np.diff(ds.arr("t")) > 0))
    # 언랩
    ds2 = Dataset()
    ds2.set_header("t_ms,phi,ank,del_now".split(","))
    for k, ph in enumerate([170, 175, 179, -178, -172, -160, 0, 20]):
        ds2.add_data_row(f"{k*10},{ph},0,0")
    u = ds2.arr("u_phi")
    check("±180 언랩 (경계 넘어 연속)", abs(u[3] - 182) < 1e-6 and abs(u[5] - 200) < 1e-6, str(u[:6]))
    check("감김수 스냅 (|w|<90 이면 원가지)", abs(u[6]) < 1e-6 and abs(u[7] - 20) < 1e-6, str(u[6:]))
    # 라이브 append 와 전체 rebuild 결정론
    lines = open(LAMBDA_CSV, encoding="utf-8").read().splitlines()
    ds3 = Dataset(); ds3.set_header(lines[0].split(","))
    for ln in lines[1:2001]:
        ds3.add_data_row(ln)
    live = ds3.matrix(0, 2000, ["a_alpha", "a_beta", "a_dphi", "a_dbeta", "a_Ahat"]).copy()
    ds3.rebuild()
    full = ds3.matrix(0, 2000, ["a_alpha", "a_beta", "a_dphi", "a_dbeta", "a_Ahat"])
    check("라이브 append == rebuild (결정론)", np.allclose(live, full, atol=1e-5), f"max|Δ|={np.max(np.abs(live-full)):.2e}")
    # 파이프라인 변경 → 재계산
    b0 = ds3.arr("a_beta").copy()
    ds3.set_pipe(p2r=0.5)
    check("set_pipe 재계산", not np.allclose(b0, ds3.arr("a_beta")) or np.all(ds3.arr("del") == 0))
    m = ds3.matrix(0, 10)
    check("matrix float32 열우선", m.dtype == np.float32 and m.shape == (len(ds3.columns()), 10))


def test_analysis():
    print("=== 3. 분석 정본 대조 ===")
    ds = Dataset(); ds.load_text(open(P2R_CSV, encoding="utf-8").read(), "p2r")
    ds.load_events_text(open(P2R_CSV[:-4] + ".events.csv", encoding="utf-8").read())
    r = an.run(ds, "p2r", dict(avg_s=2.0))
    R = r["result"]
    check("P2R 도구 실행", r["ok"], r.get("msg"))
    check("P2R = 0.4285 ± 0.0011 (문서 70)", abs(R["P2R"] - 0.4285) < 0.0011, f"{R['P2R']} se {R['se']}")
    check("P2R R² > 0.9999", R["r2"] > 0.9999, str(R["r2"]))
    check("P2R 평균점 9개 · 구간 = hold", R["n_pts"] == 9 and r["params"]["seg_mode"] == "hold", f"{R['n_pts']} {r['params']['seg_mode']}")
    check("P2R 절편 ≈ +0.139°", abs(R["intercept"] - 0.139) < 0.02, str(R["intercept"]))
    r2 = an.run(ds, "p2r", dict(avg_s=2.0, seg_mode="events"))
    check("P2R MOVE 이벤트 분할도 0.428~0.434", r2["ok"] and 0.428 < r2["result"]["P2R"] < 0.434, str(r2.get("result", {}).get("P2R")))
    ds = Dataset(); ds.load_text(open(LAMBDA_CSV, encoding="utf-8").read(), "lambda")
    tr = an.run(ds, "trials", dict(phi_eq=1.40))
    T = tr["result"]
    check("시행 분할 (유효 ≥ 10, 정지→이탈 감지)", tr["ok"] and T["n_valid"] >= 10 and T["mode"] == "rel", str(T))
    k0 = tr["table"][1]
    check("놓기점 = 손 뗀 순간 (t0 < 문턱통과 t_thr, φ₀ 가 정지평균 근처)", k0["t0"] < k0["t_thr"] and abs(k0["phi0"] - k0["phi_q"]) < 0.6, str(k0))
    check("λ (φ_eq=1.40) 방향 평균 4.8~6.3 (문서 70: 5.33/5.54)", 4.8 < T["lam_plus"] < 6.3 and 4.8 < T["lam_minus"] < 6.3, f"{T['lam_plus']}/{T['lam_minus']}")
    check("방향 갈림 < 20 %", T["dir_split_pct"] < 20, str(T["dir_split_pct"]))
    tr0 = an.run(ds, "trials", dict(phi_eq=0.0))["result"]
    check("φ_eq=0 이면 방향 갈림 커짐 (문서 70 §5 재현)", tr0["dir_split_pct"] is None or tr0["dir_split_pct"] > T["dir_split_pct"], str(tr0))
    k = tr["table"][1]
    lam = an.run(ds, "lambda", dict(t0=k["t0"] - 0.05, t1=k["t1"], phi_eq=1.40))
    check("단일 시행 λ 적합 + 통과시각 + 오버레이", lam["ok"] and lam["result"]["lam"] > 3 and lam["result"]["t_cross"] and lam["overlay"], str(lam.get("result")))
    pe = an.run(ds, "phi_eq", dict(grid_lo=-1, grid_hi=4, step=0.1, phi_eq=1.4))
    check("φ_eq 훑기 0.5~2.5° (문서 70: 1.40)", pe["ok"] and 0.5 < pe["result"]["phi_eq_best"] < 2.5, str(pe.get("result")))
    bd = an.run(ds, "boundary", dict(phi_eq=1.4))
    check("놓기 경계 도구 (방향유효 시행 ≥ 10, 평면 오버레이)", bd["ok"] and bd["result"]["n"] >= 10 and bd["plane"], str(bd.get("result")))
    st = an.run(ds, "stats", dict(t0=0, t1=5))
    check("구간 통계", st["ok"] and any(r["ch"] == "u_phi" for r in st["table"]))
    sid = an.run(ds, "sysid", dict(phi_max=5.0, smooth_ms=200.0, phi_eq=1.4))
    check("시스템 동정 실행 (수치 확인용 — 정본은 64회 놓기)", sid["ok"] and sid["result"]["n"] > 100, str(sid.get("msg") or sid["result"].get("lam")))
    bad = an.run(ds, "osc", dict(t0=0, t1=0.05))
    check("실패는 메시지로 (앱 안 죽음)", bad["ok"] is False and bad.get("msg"))
    unk = an.run(ds, "nope", {})
    check("모르는 도구", unk["ok"] is False)
    # 합성 r 실험: φ=+3° 에서 잡고 있다가 놓아 발산 — 절대 문턱이 아니라 정지→이탈로 잡혀야 한다
    dsr = Dataset(); dsr.set_header("t_ms,phi,ank,del_now".split(","))
    rows = []
    for k in range(3000):
        tt = k * 0.01; c = tt % 10.0
        if c < 3.0: ph = 3.0 + 0.03 * np.sin(50 * tt)
        elif c < 5.0: ph = 3.0 + 0.5 * np.exp(5.5 * (c - 3.0)); ph = min(ph, 12.0)
        else: ph = 0.0 + 0.03 * np.sin(50 * tt)
        rows.append(f"{int(tt*1000)},{ph:.4f},{ph*0.33:.4f},0")
    for r in rows: dsr.add_data_row(r)
    trr = an.run(dsr, "trials", dict(phi_eq=0.0))
    check("φ=+3° 정지에서 놓기 감지 (놓기점 φ₀≈3.0)", trr["ok"] and trr["result"]["n_dir_valid"] >= 3 and abs(trr["table"][0]["phi0"] - 3.0) < 0.2, str(trr["result"]) + " " + str(trr["table"][:1]))
    # 합성 신호로 osc
    dsyn = Dataset(); dsyn.set_header("t_ms,phi,ank,del_now".split(","))
    w, z = 4.86, 0.03
    for k in range(1200):
        t = k * 0.01; ph = 4.0 * np.exp(-z * w * t) * np.cos(w * t)
        dsyn.add_data_row(f"{int(t*1000)},{ph:.4f},{0.33*ph:.4f},0")
    o = an.run(dsyn, "osc", dict(t0=0, t1=12))
    check("감쇠진동 ω_n ≈ 4.86 (±3 %)", o["ok"] and abs(o["result"]["omega_n"] - 4.86) / 4.86 < 0.03, str(o.get("result", {}).get("omega_n")))
    check("감쇠진동 ζ ≈ 0.03 (±30 %)", o["ok"] and abs(o["result"]["zeta"] - 0.03) < 0.01, str(o.get("result", {}).get("zeta")))
    json.dumps(o, default=str); json.dumps(lam); json.dumps(bd)
    check("결과 JSON 직렬화", True)


def test_hub():
    print("=== 4. FakeSource → LoggerHub → 프레임 ===")
    import server as sv
    hub = sv.LoggerHub()
    hub.autorec = True
    sv.LOGS = tempfile.mkdtemp()
    hub.connect_fake(P2R_CSV, speed=50.0, name="hubtest")
    time.sleep(1.2)
    n_new = hub.pump()
    check("펌프로 행 수신", hub.ds.n > 50, str(hub.ds.n))
    check("헤더 채택 (p2r_logger 열)", hub.ds.header and hub.ds.header[1] == "phi_deg", str(hub.ds.header))
    fr = hub.ds_full_frame()
    hl = struct.unpack("<I", fr[:4])[0]; hdr = json.loads(fr[4:4 + hl].decode("utf-8"))
    body = np.frombuffer(fr[4 + hl:], dtype=np.float32)
    check("ds_full 프레임 형식", hdr["type"] == "ds_full" and len(body) == hdr["n"] * len(hdr["cols"]), f"{hdr['n']}×{len(hdr['cols'])}")
    t_col = hdr["cols"].index("t")
    check("프레임 t 열 단조", np.all(np.diff(body[t_col * hdr["n"]:(t_col + 1) * hdr["n"]]) > 0))
    time.sleep(0.6); hub.pump()
    fr2 = hub.ds_append_frame()
    check("ds_append 프레임", fr2 is not None)
    hub.send("t")
    time.sleep(0.2); hub.pump()
    check("명령 에코 (콘솔)", any("명령 수신: t" in l[1] for l in hub.console))
    hub.mark("RELEASE 왼손")
    check("마크 → 이벤트", hub.ds.events and hub.ds.events[-1][1] == "RELEASE")
    info = hub.rec.info()
    hub.disconnect()
    check("기록 파일 생성", os.path.exists(os.path.join(sv.LOGS, info["name"] + ".csv")) and os.path.exists(os.path.join(sv.LOGS, info["name"] + ".events.csv")), str(info["files"]))
    rec_csv = open(os.path.join(sv.LOGS, info["name"] + ".csv"), encoding="utf-8").read().splitlines()
    check("기록 CSV 헤더 = 원본 열", rec_csv[0].startswith("t_ms,phi_deg"), rec_csv[0])
    res = an.run(hub.ds, "stats", {})
    check("허브 데이터로 분석", res["ok"])
    files = hub.files()
    check("파일 목록에 기존 CSV 포함", any(f["name"] == "0822_lambda_test.csv" for f in files))
    hub.load_file("0822_lambda_test.csv")
    check("파일 로드 → 버퍼 교체", hub.ds.n == 9087 and hub.sent_n == 0, str(hub.ds.n))
    hub.set_pipe(dict(p2r=0.5))
    check("파이프 변경 → 재전송 예약", hub.sent_n == 0 and hub.ds.pipe["p2r"] == 0.5)
    hub.connect_fake(None, speed=1.0)   # 합성
    time.sleep(0.5); hub.pump()
    check("합성 신호 소스", hub.ds.n > 10, str(hub.ds.n))
    hub.disconnect()


if __name__ == "__main__":
    test_sink_recorder()
    test_dataset()
    test_analysis()
    test_hub()
    print("\n" + ("ALL PASS" if not FAILS else "FAILED: " + ", ".join(FAILS)))
    sys.exit(1 if FAILS else 0)
