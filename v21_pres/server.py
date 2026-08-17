# -*- coding: utf-8 -*-
"""
server.py — v21 발표 시뮬레이터 (v19_state 확장) 로컬 서버
============================================
실행:  cd v19_sim && python server.py     (필요: pip install mujoco aiohttp scipy numpy)
접속:  http://localhost:8200

물리·제어는 전부 파이썬(MuJoCo)에서 돌고, 브라우저는 v17식 UI(3D 뷰·슬라이더·
재생속도·1장 스텝·그래프)만 담당한다. WebSocket 으로 30Hz 프레임 전송.
"""
import asyncio
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "v19_bringup"))
sys.path.insert(0, HERE)

import numpy as np
from aiohttp import web, WSMsgType

from sim_engine import SimEngine

PORT = int(os.environ.get("V21_PORT", "8210"))   # 병행 실행·테스트용 오버라이드
TICK = 1.0 / 30.0          # 프레임 전송 주기
GRAPH_DECIM_FAST = 4       # 실속도 재생 시 그래프 데시메이션 (125Hz)

# 리빌드(모델 재컴파일·게인 재계산)가 필요한 현실화 키
REAL_REBUILD = {"f_phi", "c_phi_extra", "f_ankle", "b_ankle",
                "armature", "b_gear", "f_hip", "mj_dt"}


class SimServer:
    def __init__(self):
        self.eng = SimEngine()
        self.eng.kp_servo, self.eng.kd_servo = 25.0, 0.4   # 전물리 튜닝 기본값
        self.running = False
        self.speed = 1.0
        self.acc = 0.0             # 실시간 누적(제어스텝 단위)
        self.clients = set()
        self.pending = []          # 이번 틱에 쌓인 그래프 포인트

    # ---------- 상태 패킷 ----------
    def sample(self):
        e = self.eng
        f = e.frame()
        x, xh = f["x"], f["xh"]
        return [f["t"],
                round(np.rad2deg(x[0]), 4), round(np.rad2deg(x[1]), 4),
                round(np.rad2deg(x[2]), 4), round(np.rad2deg(x[2]-x[1]), 4),
                round(np.rad2deg(f["A"]), 4), round(np.rad2deg(f["Ae"]), 4),
                round(f["tau_act"], 4), f["cur"],
                round(np.rad2deg(xh[1]), 4), round(np.rad2deg(xh[2]), 4)]  \
               + [round(np.rad2deg(v), 4) for v in e.plane_pt(x)] \
               + [round(np.rad2deg(v), 4) for v in e.plane_pt(xh)]
        # 인덱스: 0 t / 1 φ / 2 α / 3 θ / 4 δ / 5 A / 6 Ae / 7 τ / 8 cur / 9 α̂ / 10 θ̂
        #        11 β 12 φ 13 β_pred 14 φ_pred      (참)
        #        15 β̂ 16 φ̂ 17 β̂_pred 18 φ̂_pred     (추정)

    def view_msg(self):
        e = self.eng
        f = e.frame()
        return dict(type="frame", pts=self.pending, view=dict(
            t=f["t"], q=f["q"], fwe=f["fwe"], cyc=f["cyc"], fallen=f["fallen"],
            tau=f["tau_act"], tau_cmd=f["tau_cmd"], cur=f["cur"],
            x=[round(np.rad2deg(v), 3) for v in f["x"][:3]],
            xh=[round(np.rad2deg(v), 3) for v in f["xh"][:3]],
            A=round(np.rad2deg(f["A"]), 3), step=f["step"], running=self.running,
            pl=[round(np.rad2deg(v), 4) for v in e.plane_pt(f["x"])],
            plh=[round(np.rad2deg(v), 4) for v in e.plane_pt(f["xh"])],
            dcmd=round(np.rad2deg(e.fwe.get("hold", 0.0)), 3)))

    def init_msg(self):
        e = self.eng
        bi = e.build_info()
        p = e.p
        return dict(type="init",
                    build=dict(w_eff=round(bi["w_eff"], 3), lam_u=round(bi["lam_u"], 3),
                               eps_star=round(bi["eps_star"], 3),
                               gamma_fold=round(bi["gamma_fold"], 2),
                               gamma_cycle=round(bi["gamma_cycle"], 2),
                               K=[round(k, 1) for k in bi["K"]],
                               I_r_calc=round(bi["I_r_calc"], 5), I_r_mj=round(bi["I_r_mj"], 5),
                               R=p["R"], DT=p["DT"], plane=bi["plane"]),
                    geom=dict(R=p["R"]*(1+e.mismatch.get("R", 0)/100.0),
                              L1=p["L1"]*(1+e.mismatch.get("L1", 0)/100.0),
                              L2=p["L2"]*(1+e.mismatch.get("L2", 0)/100.0)),
                    settings=dict(mode=e.ctrl_mode, estimator=e.estimator,
                                  servo_aware=e.servo_aware, gain=e.gain_scale,
                                  kp=e.kp_servo, kd=e.kd_servo, speed=self.speed,
                                  real=e.real, mismatch=e.mismatch,
                                  x0=e.x0,
                                  nominal=dict(R=p["R"], m1=p["m1"], m2=p["m2"],
                                               L1=p["L1"], L2=p["L2"],
                                               crank_m_each=p["crank_m_each"],
                                               bottom_m=p["bottom_m"],
                                               c_phi=p["c_phi"], B_THETA=p["B_THETA"],
                                               RHO=p["RHO"],
                                               A_TRIGGER_DEG=p["A_TRIGGER_DEG"],
                                               T_FOLD=p["T_FOLD"], T_WAIT=p["T_WAIT"],
                                               T_EXTEND=p["T_EXTEND"], T_REST=p["T_REST"],
                                               CUR_SAFE_UNIT=p["CUR_SAFE_UNIT"],
                                               R_COST=p["R_COST"])))

    async def broadcast(self, msg):
        dead = []
        data = json.dumps(msg)
        for ws in self.clients:
            try:
                await ws.send_str(data)
            except ConnectionError:
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)

    # ---------- 시뮬 틱 ----------
    async def loop(self):
        while True:
            t0 = asyncio.get_event_loop().time()
            if self.running and self.clients:
                self.acc += self.speed * TICK / self.eng.p["DT"]
                n = int(self.acc)
                self.acc -= n
                n = min(n, 200)                     # 폭주 방지
                decim = GRAPH_DECIM_FAST if self.speed > 0.3 else 1
                for i in range(n):
                    self.eng.control_step()
                    if self.eng.step_count % decim == 0:
                        self.pending.append(self.sample())
                if self.eng.fallen and n > 0:
                    pass                            # 낙하해도 물리는 계속 (매달려 스윙)
            if self.clients and (self.pending or self.running):
                await self.broadcast(self.view_msg())
                self.pending = []
            dt = asyncio.get_event_loop().time() - t0
            await asyncio.sleep(max(0.001, TICK - dt))

    # ---------- 명령 처리 ----------
    async def handle(self, ws, m):
        e = self.eng
        cmd = m.get("cmd")
        if cmd == "hello":
            await ws.send_str(json.dumps(self.init_msg()))
            await ws.send_str(json.dumps(self.view_msg()))
        elif cmd == "run":
            self.running = True
        elif cmd == "pause":
            self.running = False
            await self.broadcast(self.view_msg())
        elif cmd == "reset":
            e.reset()
            self.pending = []
            await self.broadcast(dict(type="clear"))
            await self.broadcast(self.view_msg())
        elif cmd == "speed":
            self.speed = float(np.clip(m["v"], 0.01, 3.0))
        elif cmd == "step":
            self.running = False
            for _ in range(int(m.get("n", 1))):
                e.control_step()
                self.pending.append(self.sample())
            await self.broadcast(self.view_msg())
            self.pending = []
        elif cmd == "back":
            self.running = False
            for _ in range(int(m.get("n", 1))):
                if not e.step_back():
                    break
            await self.broadcast(dict(type="rewind", t=e.data.time))
            await self.broadcast(self.view_msg())
        elif cmd == "perturb":
            e.perturb(float(m["v"]))
        elif cmd == "mode":
            e.ctrl_mode = m["v"]
            e.fwe.update(phase="idle", t=0.0, armed=True, cycles=0)
        elif cmd == "estimator":
            e.estimator = m["v"]
        elif cmd == "gain":
            e.gain_scale = float(m["v"])
        elif cmd == "pd":
            e.kp_servo = float(m["kp"]); e.kd_servo = float(m["kd"])
        elif cmd == "x0":
            # [v21 1B] 속도 키 추가 (deg/s). 속도 키가 하나도 없으면 0 으로 리셋 —
            #   슬라이더·평면클릭(위치만 보냄) 뒤에 이전 4D 입력의 속도가 남는 것 방지.
            e.x0.update({k: float(m[k]) for k in
                         ("phi0", "alpha0", "theta0", "phid0", "alphad0", "thetad0") if k in m})
            if not any(k in m for k in ("phid0", "alphad0", "thetad0")):
                e.x0.update(phid0=0.0, alphad0=0.0, thetad0=0.0)
            e.reset()
            await self.broadcast(dict(type="clear"))
            await self.broadcast(self.view_msg())
        elif cmd == "pose":
            # [v21 1A] 자세 미리보기 — reset/clear 없음 (그래프·궤적 보존)
            e.set_pose(float(m.get("phi0", 0.0)), float(m.get("alpha0", 0.0)),
                       float(m.get("theta0", 0.0)))
            await self.broadcast(self.view_msg())
        elif cmd == "pose_begin":
            # hover 진입: 현 상태 저장 (이탈 시 pose_end 로 복원)
            self.running = False
            e._pose_saved = e.snapshot()
        elif cmd == "scene_save":
            # [v21 2] 오버랩 장면(카메라+기하+레이어) 저장 — scenes/<name>.json
            os.makedirs(os.path.join(HERE, "scenes"), exist_ok=True)
            name = "".join(c for c in str(m.get("name", "scene"))
                           if c.isalnum() or c in "_-") or "scene"
            with open(os.path.join(HERE, "scenes", name + ".json"), "w",
                      encoding="utf-8") as fh:
                json.dump(m.get("data", {}), fh, ensure_ascii=False, indent=1)
            await ws.send_str(json.dumps(dict(type="scene_saved", name=name)))
        elif cmd == "scene_load":
            name = "".join(c for c in str(m.get("name", ""))
                           if c.isalnum() or c in "_-")
            fp = os.path.join(HERE, "scenes", name + ".json")
            if os.path.isfile(fp):
                with open(fp, encoding="utf-8") as fh:
                    data = json.load(fh)
                await ws.send_str(json.dumps(dict(type="scene_data", name=name, data=data)))
            else:
                d = os.path.join(HERE, "scenes")
                names = sorted(os.path.splitext(f)[0] for f in os.listdir(d)
                               if f.endswith(".json")) if os.path.isdir(d) else []
                await ws.send_str(json.dumps(dict(type="scene_list", names=names)))
        elif cmd == "pose_end":
            s = getattr(e, "_pose_saved", None)
            if s is not None:
                e.restore(s); e._pose_saved = None
            await self.broadcast(self.view_msg())
        elif cmd == "real":
            k, v = m["k"], m["v"]
            e.real[k] = (bool(v) if k in ("servo_on", "sensor_on") else float(v))
            if k in REAL_REBUILD:
                await self.rebuild()
        elif cmd == "nominal":
            k, v = m["k"], m["v"]
            if k == "Q_DIAG":
                e.nominal[k] = [float(x) for x in v]
            else:
                e.nominal[k] = float(v)
            await self.rebuild()
        elif cmd == "mismatch":
            e.mismatch[m["k"]] = float(m["v"])
            await self.rebuild()
        elif cmd == "servo_aware":
            e.servo_aware = bool(m["v"])
            await self.rebuild()
        elif cmd == "fwe_async_preset":
            # ★비동기 FWE 시연 (2026-08-01 신챔피언): 펴기 생략 + 증분 접기 + 저속 복귀.
            #   기존 fwe_preset(완전 사이클)은 그대로 두고 별도 프리셋으로 추가한 것.
            #   차이 둘: ① 줄 마찰이 **실측값**(점성 3.3e-4·건마찰 3.1e-4 — 기존 프리셋의
            #   가정 0.008/0.002 가 아니다) ② 온라인 노브 없이 고정 γ_h=15.
            #   실측 마찰 20케이스 분포: 전 케이스 120s 상한 도달 (완전사이클 중앙값 49.3s).
            #   ⚠생존 레짐이 소각근사 밖(몸기울기 최대 ~19°, 유지각 표류 ±33°)이라
            #     '이론 검증'이 아니라 '시뮬 발견'으로 다룰 것.
            self.running = False
            self.pending = []
            from sim_engine import REAL_DEFAULT
            e.nominal = dict(c_phi=3.3e-4, RHO=0.95)      # 실측 줄 점성 (자유흔들기 07-28)
            e.mismatch = dict(m1=0.0, m2=0.0, L1=0.0, L2=0.0, R=0.0, kt=0.0)
            e.real = dict(REAL_DEFAULT)
            e.gain_scale = 1.0
            e.real.update(enc_use_gyro=False, fwe_settle_s=0.0, fwe_adapt_T=False,
                          fwe_ff=True, fwe_fast_L=False, w_noload_dps=462.0,
                          f_phi=3.1e-4,                    # 실측 줄 건마찰
                          fwe_pos_mode=True, pos_vel_dps=420.0, pos_acc_dps2=8000.0,
                          fwe_async=True, fwe_async_gamma=15.0,
                          fwe_async_vret_dps=3.0, fwe_async_hold_min_deg=1.0,
                          fwe_online=False)                # 고정 γ — 학습 없음
            e.kp_servo, e.kd_servo = 80.0, 1.3
            e.estimator = "enc"
            e.ctrl_mode = "fwe3"
            jit = float(np.random.default_rng().uniform(-0.02, 0.02))
            e.x0 = dict(phi0=0.0, alpha0=0.5 + jit, theta0=0.5 + jit)
            e.build()
            e.reset()
            await self.broadcast(dict(type="clear"))
            await self.broadcast(self.init_msg())
            await self.broadcast(self.view_msg())
            self.running = True
        elif cmd == "fwe_async_preset_measured":
            # ★★[2026-08-14] 비동기 FWE — **실측 정본 파라미터** 판.
            #   위 fwe_async_preset 은 params_v19.py 의 설계추정치(placeholder) 위에서 돈다.
            #   이 버튼은 2026-07-30~08-06 실측값(params_v19.MEASURED_SET)을 얹는다.
            #     · 질량표 실측 -> p1=0.17510 p2=0.12280 h=0.3972 (문서 38 파생값 재현)
            #     · I_r=4.50e-3(저울) / S_r=1.0835e-2(자유흔들기 omega_n=4.86 실측)
            #     · c_phi=3.33e-4, 줄 건마찰 f_phi=3.105e-4
            #   결과: lam_u 6.448(추정치) -> 5.922(실측). 배가시간이 느려져 deadbeat gamma 가
            #   내려가므로 **작동 gamma 가 15 -> 4~12 로 이동**한다(60s 4/4 스캔, 08-14).
            #   기본 gamma=10 (고원 중앙, 유지각 표류가 가장 작음 ~19deg).
            self.running = False
            self.pending = []
            from sim_engine import REAL_DEFAULT
            import params_v19 as _P
            e.nominal = dict(_P.MEASURED_SET); e.nominal["RHO"] = 0.95
            # ★[2026-08-15 정정] 제어주기 200Hz = 실기 펌웨어 fwe_async_v19.ino 와 동일
            #   (nx = now_us + 5000; const float dt = 0.005f).
            #   ⚠ 500Hz 로 두고 속도 차분만 25ms 로 맞추면 '13스텝 차분' 이라는 실기에도
            #   시뮬에도 없는 잡종이 되고, 거기서만 나타나는 늦은트리거·과접기를 실기
            #   현상으로 오인하게 된다. 200Hz 에서는 T_REST 60~120ms 가 사실상 무차별이라
            #   앞서 넣었던 T_REST=75ms 는 철회하고 기본값 60 을 쓴다. 근거: 46번 문서.
            e.nominal["DT"] = 0.005
            e.mismatch = dict(m1=0.0, m2=0.0, L1=0.0, L2=0.0, R=0.0, kt=0.0)
            e.real = dict(REAL_DEFAULT)
            e.gain_scale = 1.0
            e.real.update(enc_use_gyro=False, fwe_settle_s=0.0, fwe_adapt_T=False,
                          fwe_ff=True, fwe_fast_L=False, w_noload_dps=462.0,
                          fwe_pos_mode=True, pos_vel_dps=420.0, pos_acc_dps2=8000.0,
                          fwe_async=True, fwe_async_gamma=10.0,
                          fwe_async_vret_dps=3.0, fwe_async_hold_min_deg=1.0,
                          fwe_online=False,
                          enc_diff_ms=25.0, enc_tau_v=0.030,   # ★실기 파이프라인 기본 ON
                          **_P.MEASURED_REAL)
            e.kp_servo, e.kd_servo = 80.0, 1.3
            e.estimator = "enc"
            e.ctrl_mode = "fwe3"
            jit = float(np.random.default_rng().uniform(-0.02, 0.02))
            e.x0 = dict(phi0=0.0, alpha0=0.5 + jit, theta0=0.5 + jit)
            e.build()
            e.reset()
            await self.broadcast(dict(type="clear"))
            await self.broadcast(self.init_msg())
            await self.broadcast(self.view_msg())
            self.running = True
        elif cmd == "fwe_preset":
            # ★원클릭 FWE 시연: 스윕 최적 시간표(params 기본) + 발목엔코더 + c_φ=0.008
            #   + γ(T)×실행갭 + 온라인 노브. 설정 후 바로 재생 시작.
            #   ★재현성: 이전에 만진 편차/현실화 슬라이더를 전부 초기화하고 시작한다
            #   (안 그러면 예: kt −10% 가 남아 생존 56s → 10s 로 떨어져도 원인을 모름)
            self.running = False
            self.pending = []
            from sim_engine import REAL_DEFAULT
            e.nominal = dict(c_phi=0.008, RHO=0.95)
            e.mismatch = dict(m1=0.0, m2=0.0, L1=0.0, L2=0.0, R=0.0, kt=0.0)
            e.real = dict(REAL_DEFAULT)
            e.gain_scale = 1.0
            e.real.update(enc_use_gyro=False, fwe_settle_s=0.0, fwe_adapt_T=False,
                          fwe_ff=True, fwe_fast_L=False, w_noload_dps=462.0,
                          # ★챔피언 설정 (2026-07-18): 위치모드(사다리꼴 420°/s·8000°/s²)
                          #   + 1-노브 RLS (Gc 고정, gc 만 학습 — 2노브는 공선성 표류로 사망)
                          #   20판 지터 분포: 중앙값 70.4s (전류모드 43.4s, 고정γ 32.6s)
                          fwe_pos_mode=True, pos_vel_dps=420.0, pos_acc_dps2=8000.0,
                          fwe_gamma_cal=22.7, fwe_Gc_cal=15.8,
                          fwe_online=True, fwe_online_1p=True, fwe_online_ff=0.95,
                          fwe_online_rej_deg=2.0)
            e.kp_servo, e.kd_servo = 80.0, 1.3
            e.estimator = "enc"
            e.ctrl_mode = "fwe3"
            # ★누를 때마다 새 추첨: 초기 기울기 ±0.02° 지터 (혼돈계라 수명이 매번 다름).
            #   '리셋' 버튼은 이 x0 를 재사용 → 같은 판 리플레이(결정론 재현).
            jit = float(np.random.default_rng().uniform(-0.02, 0.02))
            e.x0 = dict(phi0=0.0, alpha0=0.5 + jit, theta0=0.5 + jit)
            e.build()   # γ*·Gc* 는 위에서 위치모드 캘리브값(22.7/15.8)으로 직접 지정됨
            e.reset()
            await self.broadcast(dict(type="clear"))
            await self.broadcast(self.init_msg())
            await self.broadcast(self.view_msg())
            self.running = True

    async def rebuild(self):
        self.running = False
        self.pending = []
        try:
            self.eng.build()
            await self.broadcast(dict(type="clear"))
            await self.broadcast(self.init_msg())
            await self.broadcast(self.view_msg())
        except Exception as ex:                     # 파라미터 조합 불능(비양정치 등)
            await self.broadcast(dict(type="error", msg=str(ex)))


SRV = SimServer()


async def ws_handler(request):
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)
    SRV.clients.add(ws)
    print(f"[ws] client connected ({len(SRV.clients)})", flush=True)
    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                try:
                    await SRV.handle(ws, json.loads(msg.data))
                except Exception as ex:
                    await ws.send_str(json.dumps(dict(type="error", msg=str(ex))))
            elif msg.type == WSMsgType.ERROR:
                break
    finally:
        SRV.clients.discard(ws)
    return ws


async def index(request):
    return web.FileResponse(os.path.join(HERE, "static", "index.html"))


def main():
    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/three.min.js",
                       lambda r: web.FileResponse(os.path.join(HERE, "static", "three.min.js")))
    media = os.path.join(HERE, "static", "media")
    os.makedirs(media, exist_ok=True)
    app.router.add_static("/media/", media)   # [v21 2] 오버랩 사진·영상

    async def on_startup(app):
        asyncio.get_event_loop().create_task(SRV.loop())
    app.on_startup.append(on_startup)

    print("=" * 60)
    print("  v21 발표 시뮬레이터 (v19_state 확장)")
    print(f"  브라우저에서 http://localhost:{PORT} 를 여세요")
    print("=" * 60)
    web.run_app(app, port=PORT, print=None)


if __name__ == "__main__":
    main()
