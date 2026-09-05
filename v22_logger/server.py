# -*- coding: utf-8 -*-
"""
server.py — v22 로거·측정 앱 서버 (v21 발표 시뮬레이터를 그대로 품고, 측정 앱을 얹었다)
=========================================================================================
실행:  cd v22_logger && python server.py            (기본 포트 8220, V22_PORT 로 변경)
       python server.py --fake                      (장치 없이: 합성 신호)
       python server.py --fake "../lambda test/0822_lambda_test.csv" --speed 2
       python server.py --mujoco                    (장치 없이: MuJoCo 가상 로봇 — 리허설·E2E)
접속:  http://localhost:8220/          측정 앱 (v22)
       http://localhost:8220/pres      v21 발표 시뮬레이터 (그대로)

구조:
    브라우저 ──WS /ws2──▶ LoggerHub ──▶ 시리얼(pyserial) / FakeSource
                            │  LineSink 로 D/R/F/E/# 분류
                            ├─ Recorder  (logs/<이름>.csv .trials.csv .folds.csv .events.csv .raw.txt)
                            ├─ Dataset   (원시 + 앱 파생열 — 정본)
                            └─ analysis  (측정 도구 — 구간·점·식·잔차를 되돌려준다)
    데이터는 바이너리 프레임으로 보낸다: [u32 헤더길이][JSON 헤더][float32 열우선 행렬]
"""
import argparse
import asyncio
import collections
import json
import os
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "v19_bringup"))
sys.path.insert(0, HERE)

import numpy as np
from aiohttp import web, WSMsgType

try:                                    # 윈도우 cmd(cp949)에서 한글 print 가 예외를 내지 않게 (hangcal_logger 와 동일)
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

import serial_bridge as sb
from dataset_v22 import Dataset, PIPE_DEFAULT, PIPE_DOC, DERIVED
import analysis_v22 as an

PORT = int(os.environ.get("V22_PORT", "8220"))
TICK = 1.0 / 30.0
LOGS = os.path.join(HERE, "logs")
EXTRA_LOG_DIRS = [os.path.join(HERE, ".."), os.path.join(HERE, "..", "lambda test"),
                  os.path.join(HERE, "..", "p2r test"), os.path.join(HERE, "..", "r test")]

# v21 시뮬 서버 — MuJoCo 가 없으면 없이 간다 (평면 상수는 폴백)
try:
    import sim_server_v21 as simsrv
    SIM = simsrv.SRV
    SIM_ERR = None
except Exception as ex:                      # ImportError(mujoco) 등
    simsrv, SIM, SIM_ERR = None, None, f"{type(ex).__name__}: {ex}"

PL_FALLBACK = dict(p1r=0.5715, p2r=0.4285, lam=5.66, P=[[1 / 5.66, 0.0], [0.0, 1 / 5.66]],
                   wq=[1 / 1.506, 1.0], slopeA0=-1.506, sCoM=0.8889, kFold=0.3517, r=-1.506,
                   _note="MuJoCo 없음 — 실측 정본(문서 70) 상수로 폴백")
GEOM_FALLBACK = dict(R=0.433, L1=0.259, L2=0.375)


def plane_and_geom():
    if SIM is not None:
        try:
            bi = SIM.eng.build_info()
            p = SIM.eng.p
            return bi["plane"], dict(R=p["R"], L1=p["L1"], L2=p["L2"]), None
        except Exception as ex:
            return PL_FALLBACK, GEOM_FALLBACK, str(ex)
    return PL_FALLBACK, GEOM_FALLBACK, SIM_ERR


def pack(header, mat):
    """바이너리 프레임: [u32 len][JSON][float32 col-major]"""
    h = json.dumps(header, ensure_ascii=False).encode("utf-8")
    body = np.ascontiguousarray(mat, dtype=np.float32).tobytes() if mat is not None else b""
    return struct.pack("<I", len(h)) + h + body


class LoggerHub:
    def __init__(self):
        self.plane, self.geom, self.sim_note = plane_and_geom()
        self.ds = Dataset(plane=self.plane)
        self.sink = sb.LineSink()
        self.src = None
        self.rec = None
        self.autorec = True
        self.clients = set()
        self.console = collections.deque(maxlen=600)      # [host_t, text, kind]
        self.console_new = []
        self.sent_n = 0
        self.aux_dirty = True
        self.link_dirty = True
        self.rate_win = collections.deque()                # 호스트 시각 (D행) — Hz 계산
        self.last_link = 0.0
        self.err = None
        self.scene = dict(name="twin", args={}, seq=0, t=0.0)   # 무대(/show) 장면 상태 — 조종석(/deck)이 바꾸고 모든 창에 방송
        self.scene_dirty = False
        self._nohdr_warned = False
        self.commands = self._load_commands()
        self.profile = self.commands.get("default")
        os.makedirs(LOGS, exist_ok=True)

    # ---------- 명령 팔레트 ----------
    def _load_commands(self):
        try:
            with open(os.path.join(HERE, "commands.json"), encoding="utf-8") as fh:
                return json.load(fh)
        except Exception as ex:
            return dict(default=None, profiles={}, _error=str(ex))

    # ---------- 콘솔 ----------
    def say(self, text, kind="app"):
        item = [round(time.time(), 3), text, kind]
        self.console.append(item)
        self.console_new.append(item)

    # ---------- 무대 장면 ----------
    def set_scene(self, name, args=None):
        self.scene = dict(name=str(name or "twin"), args=dict(args or {}), seq=int(self.scene.get("seq", 0)) + 1, t=round(time.time(), 3))
        self.scene_dirty = True
        self.say(f"무대: {self.scene['name']}" + (f" {json.dumps(self.scene['args'], ensure_ascii=False)}" if self.scene["args"] else ""), "app")

    # ---------- 소스 ----------
    def connect(self, port=None, baud=115200, name=None):
        if self.src is not None:
            self.disconnect()
        if not port:
            port, msg = sb.autodetect_port()
            self.say(msg, "app")
            if not port:
                raise RuntimeError(msg)
        self.src = sb.SerialSource(port, baud)
        self.src.start()
        self.say(f"연결: {port} @ {baud}  (접속 직후 아무것도 보내지 않는다 — 문서 77 §6)", "app")
        self._after_connect(name)

    def connect_fake(self, file=None, speed=1.0, name=None):
        if self.src is not None:
            self.disconnect()
        path = self._resolve_file(file) if file else None
        self.src = sb.FakeSource(path, speed=speed)
        self.src.start()
        self.say("가짜 소스 연결: " + (os.path.basename(path) if path else "합성 신호") + f" ×{speed}", "app")
        self._after_connect(name)

    def connect_mujoco(self, name=None, seed=0):
        """MuJoCo 가상 로봇 (v21 SimEngine + v22_raw v2 펌웨어 흉내) — 리허설·E2E 용."""
        if self.src is not None:
            self.disconnect()
        import mujoco_source as mjs
        self.src = mjs.MujocoSource(seed=seed)
        self.src.start()
        self.say("MuJoCo 가상 로봇 연결 (v21 실측 프리셋, 200 Hz · D행 100 Hz). 사람 동작은 'sim release β φ' 로 지시한다", "app")
        self._after_connect(name)

    def robot(self, text):
        """가상 로봇에 무대 지시(sim …)를 콘솔 에코 없이 보낸다 — 실기에는 해당 없음."""
        if self.src is None or self.src.describe().get("kind") != "mujoco":
            raise RuntimeError("MuJoCo 가상 로봇에 연결된 상태에서만 쓴다")
        self.src.write("sim " + text.strip())

    def _after_connect(self, name):
        self.err = None
        self._nohdr_warned = False
        self.sink = sb.LineSink()                # 새 연결 = 새 헤더 상태
        if self.ds.source in ("file", "upload") and self.ds.n:
            self.ds.clear()                      # 파일 분석 중이던 버퍼는 라이브와 섞지 않는다
            self.say("파일 버퍼 비움 → 라이브 버퍼 시작", "app")
        self.ds.source = "live"
        self.ds.name = ""
        self.sent_n = 0
        self.aux_dirty = True
        self.rate_win.clear()
        self.link_dirty = True
        if self.autorec:
            self.rec_new(name)

    def disconnect(self):
        if self.src is not None:
            try:
                self.src.close()
            except Exception:
                pass
            self.say("연결 끊음", "app")
        self.src = None
        self.rec_stop()
        self.link_dirty = True

    def send(self, text):
        if self.src is None:
            raise RuntimeError("연결되어 있지 않다")
        text = text.rstrip("\r\n")
        self.src.write(text)
        self.say("> " + text, "tx")
        if self.rec:
            self.rec.write("dev", None, None, "> " + text + "   (앱에서 보냄)")

    # ---------- 기록 ----------
    def rec_new(self, name=None):
        if self.src is None:
            raise RuntimeError("연결 상태에서만 기록한다")
        self.rec_stop()
        self.rec = sb.Recorder(LOGS, name or time.strftime("%Y%m%d_%H%M%S_auto"), self.sink)
        self.say(f"기록 시작: logs/{self.rec.name}.csv (+ .trials .folds .events .raw)", "app")
        self.link_dirty = True

    def rec_stop(self):
        if self.rec is not None:
            info = self.rec.info()
            self.rec.close()
            self.say(f"기록 종료: {info['name']}  D행 {info['n_data']} · R {info['n_trial']} · E {info['n_event']}", "app")
            self.rec = None
            self.link_dirty = True

    def mark(self, text):
        t_ms = self.ds.last_t_ms()
        name, _, value = (text or "MARK").partition(" ")
        name = name.strip() or "MARK"
        self.ds.add_event(t_ms, name, value.strip())
        if self.rec:
            self.rec.mark(t_ms, name, value.strip())
        self.say(f"마크 E,{int(t_ms)},{name},{value.strip()}", "app")
        self.aux_dirty = True

    # ---------- 파일 ----------
    def files(self):
        out = []
        seen = set()
        for d in [LOGS] + EXTRA_LOG_DIRS:
            if not os.path.isdir(d):
                continue
            for f in sorted(os.listdir(d)):
                if not f.lower().endswith(".csv") or f.endswith(".trials.csv") or f.endswith(".events.csv") or f.endswith(".folds.csv"):
                    continue
                p = os.path.join(d, f)
                key = os.path.abspath(p)
                if key in seen:
                    continue
                seen.add(key)
                try:
                    st = os.stat(p)
                except OSError:
                    continue
                out.append(dict(name=f, dir=os.path.relpath(d, HERE), size=st.st_size,
                                mtime=time.strftime("%m-%d %H:%M", time.localtime(st.st_mtime)),
                                has_events=os.path.exists(p[:-4] + ".events.csv")))
        out.sort(key=lambda r: (r["dir"] != "logs", r["name"]), reverse=False)
        return out

    def _resolve_file(self, name):
        name = os.path.basename(name) if name else ""
        for d in [LOGS] + EXTRA_LOG_DIRS:
            p = os.path.join(d, name)
            if os.path.isfile(p):
                return p
        raise FileNotFoundError(name)

    def load_file(self, name):
        if self.src is not None:
            raise RuntimeError("라이브 중에는 파일을 불러올 수 없다 — 먼저 끊기")
        p = self._resolve_file(name)
        with open(p, encoding="utf-8", errors="replace") as fh:
            txt = fh.read()
        n = self.ds.load_text(txt, os.path.basename(p), "file")
        ev = p[:-4] + ".events.csv" if p.lower().endswith(".csv") else None
        if ev and os.path.isfile(ev):
            with open(ev, encoding="utf-8", errors="replace") as fh:
                self.ds.load_events_text(fh.read())
        self.say(f"파일 로드: {os.path.basename(p)}  {n} 행" + (" + events" if ev and os.path.isfile(ev) else ""), "app")
        self.sent_n = 0
        self.aux_dirty = True
        return n

    def load_text(self, name, text):
        if self.src is not None:
            raise RuntimeError("라이브 중에는 파일을 불러올 수 없다 — 먼저 끊기")
        n = self.ds.load_text(text, name, "upload")
        self.say(f"업로드 로드: {name}  {n} 행", "app")
        self.sent_n = 0
        self.aux_dirty = True
        return n

    def clear(self):
        self.ds.clear()
        self.sent_n = 0
        self.aux_dirty = True
        self.say("버퍼 비움 (기록 파일은 그대로)", "app")

    # ---------- 파이프라인 ----------
    def set_pipe(self, params):
        clean = {}
        for k, v in params.items():
            if k not in PIPE_DEFAULT:
                continue
            if isinstance(PIPE_DEFAULT[k], bool):
                clean[k] = bool(v)
            elif isinstance(PIPE_DEFAULT[k], str):
                clean[k] = str(v)
            else:
                clean[k] = float(v)
        changed = self.ds.set_pipe(**clean)
        if changed:
            self.sent_n = 0
            self.aux_dirty = True
            self.say("파이프라인 갱신 → 파생열 전부 재계산: " + ", ".join(f"{k}={v}" for k, v in clean.items()), "app")
        return changed

    # ---------- 수신 처리 ----------
    def pump(self):
        if self.src is None:
            return 0
        items = self.src.drain()
        if self.src.error and self.err != self.src.error:
            self.err = self.src.error
            self.say("!! " + self.err, "err")
            self.link_dirty = True
        n0 = self.ds.n
        for host_t, text in items:
            kind, prefix, payload = self.sink.classify(text)
            if kind is None:
                continue
            if self.rec:
                self.rec.write(kind, prefix, payload, text)
            if kind == "data":
                if self.ds.header is None or self.ds.header != self.sink.headers.get("D", self.ds.header):
                    if self.sink.headers.get("D"):
                        self.ds.set_header(self.sink.headers["D"])
                if not self.sink.headers.get("D") and not self._nohdr_warned:
                    self._nohdr_warned = True
                    ncol = payload.count(",") + 1
                    self.say(f"!! D행이 헤더 없이 온다 ({ncol}열) — 보드가 이미 돌고 있어 '# D,…' 를 놓쳤다. "
                             + ("기본 13열 이름을 가정한다. " if ncol >= 13 else "★열 수가 기본(13)보다 적어 행을 버린다. ")
                             + "펌웨어에 hdr (v22_raw) 또는 m 두 번(hangcal: CSV 껐다 켜기)을 보내 헤더를 다시 받을 것", "err")
                if self.ds.add_data_row(payload):
                    self.rate_win.append(host_t)
            elif kind == "header":
                if prefix == "D":
                    self.ds.set_header(payload)
                elif prefix == "R":
                    self.ds.trial_header = payload
                elif prefix == "F":
                    self.ds.fold_header = payload
                self.console.append([round(host_t, 3), text, "rx"]); self.console_new.append(self.console[-1])
                self.aux_dirty = True
            elif kind == "trial":
                self.ds.add_trial_row(payload); self.aux_dirty = True
                self.console.append([round(host_t, 3), text, "rx"]); self.console_new.append(self.console[-1])
            elif kind == "fold":
                self.ds.add_fold_row(payload); self.aux_dirty = True
                self.console.append([round(host_t, 3), text, "rx"]); self.console_new.append(self.console[-1])
            elif kind == "event":
                self.ds.add_event_row(payload); self.aux_dirty = True
                self.console.append([round(host_t, 3), text, "rx"]); self.console_new.append(self.console[-1])
            else:
                self.console.append([round(host_t, 3), text, "rx"]); self.console_new.append(self.console[-1])
        now = time.time()
        while self.rate_win and now - self.rate_win[0] > 2.0:
            self.rate_win.popleft()
        return self.ds.n - n0

    def rate_hz(self):
        if len(self.rate_win) < 2:
            return 0.0
        span = self.rate_win[-1] - self.rate_win[0]
        return round((len(self.rate_win) - 1) / span, 1) if span > 0 else 0.0

    # ---------- 메시지 ----------
    def link_msg(self):
        src = self.src.describe() if self.src else dict(kind="none")
        return dict(type="link", connected=self.src is not None, src=src, err=self.err,
                    rec=self.rec.info() if self.rec else None, autorec=self.autorec,
                    rate_hz=self.rate_hz(), n=self.ds.n, last_t=round(self.ds.der["t"][-1], 3) if self.ds.n else 0.0,
                    counts=dict(self.sink.counts), profile=self.profile)

    def hello_msg(self):
        return dict(type="hello", plane=self.plane, geom=self.geom, sim_note=self.sim_note, scene=self.scene,
                    pipe=self.ds.pipe, pipe_doc=PIPE_DOC, commands=self.commands, profile=self.profile,
                    derived=DERIVED, console=list(self.console)[-300:], logs_dir=LOGS)

    def ds_full_frame(self):
        self.ds.smooth_update(force_all=True)
        cols = self.ds.columns()
        mat = self.ds.matrix(0, self.ds.n, cols)
        self.sent_n = self.ds.n
        return pack(dict(type="ds_full", cols=cols, n=self.ds.n, name=self.ds.name, source=self.ds.source,
                         pipe=self.ds.pipe), mat)

    def ds_append_frame(self):
        n = self.ds.n
        if n <= self.sent_n:
            return None
        i0 = self.sent_n
        j0 = self.ds.smooth_update()               # 꼬리 평활 갱신 (갱신된 행부터 다시 보낸다)
        i0 = min(i0, j0) if j0 is not None else i0
        cols = self.ds.columns()
        mat = self.ds.matrix(i0, n, cols)
        self.sent_n = n
        return pack(dict(type="ds_append", cols=cols, n0=i0, n=n), mat)

    async def broadcast(self, msg):
        if not self.clients:
            return
        data = json.dumps(msg, ensure_ascii=False) if isinstance(msg, dict) else msg
        dead = []
        for ws in list(self.clients):
            try:
                if isinstance(data, bytes):
                    await ws.send_bytes(data)
                else:
                    await ws.send_str(data)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)

    async def loop(self):
        while True:
            t0 = time.time()
            try:
                self.pump()
                if self.clients:
                    if self.ds.n > self.sent_n or self.sent_n > self.ds.n:
                        if self.sent_n == 0 or self.sent_n > self.ds.n:
                            await self.broadcast(self.ds_full_frame())
                        else:
                            fr = self.ds_append_frame()
                            if fr:
                                await self.broadcast(fr)
                    if self.console_new:
                        await self.broadcast(dict(type="console", lines=self.console_new[-200:]))
                        self.console_new = []
                    if self.aux_dirty:
                        await self.broadcast(dict(type="aux", **self.ds.aux()))
                        self.aux_dirty = False
                    if self.scene_dirty:
                        self.scene_dirty = False
                        await self.broadcast(dict(type="scene", **self.scene))
                    if self.link_dirty or time.time() - self.last_link > 0.5:
                        self.last_link = time.time(); self.link_dirty = False
                        await self.broadcast(self.link_msg())
            except Exception as ex:                # 루프는 절대 죽지 않는다
                self.say(f"!! 서버 루프 오류: {type(ex).__name__}: {ex}", "err")
            dt = time.time() - t0
            await asyncio.sleep(max(0.002, TICK - dt))

    # ---------- 클라이언트 명령 ----------
    async def handle(self, ws, m):
        cmd = m.get("cmd")
        if cmd == "hello":
            await ws.send_str(json.dumps(self.hello_msg(), ensure_ascii=False))
            await ws.send_bytes(self.ds_full_frame())
            await ws.send_str(json.dumps(dict(type="aux", **self.ds.aux()), ensure_ascii=False))
            await ws.send_str(json.dumps(self.link_msg(), ensure_ascii=False))
            await ws.send_str(json.dumps(dict(type="files", files=self.files()), ensure_ascii=False))
        elif cmd == "ports":
            ports, err = sb.list_ports()
            auto, msg = sb.autodetect_port()
            await ws.send_str(json.dumps(dict(type="ports", ports=ports, err=err, auto=auto, msg=msg), ensure_ascii=False))
        elif cmd == "connect":
            self.connect(m.get("port") or None, int(m.get("baud", 115200)), m.get("name") or None)
        elif cmd == "fake":
            self.connect_fake(m.get("file") or None, float(m.get("speed", 1.0)), m.get("name") or None)
        elif cmd == "scene":
            self.set_scene(m.get("name"), m.get("args"))
        elif cmd == "mujoco":
            self.connect_mujoco(m.get("name") or None, int(m.get("seed", 0)))
        elif cmd == "robot":
            self.robot(str(m.get("text", "")))
        elif cmd == "disconnect":
            self.disconnect()
        elif cmd == "send":
            self.send(str(m.get("text", "")))
        elif cmd == "rec":
            self.rec_new(m.get("name") or None)
        elif cmd == "rec_stop":
            self.rec_stop()
        elif cmd == "autorec":
            self.autorec = bool(m.get("on", True)); self.link_dirty = True
        elif cmd == "mark":
            self.mark(str(m.get("text", "MARK")))
        elif cmd == "files":
            await ws.send_str(json.dumps(dict(type="files", files=self.files()), ensure_ascii=False))
        elif cmd == "load":
            self.load_file(str(m.get("name", "")))
        elif cmd == "load_text":
            self.load_text(str(m.get("name", "upload.csv")), str(m.get("text", "")))
            if m.get("events"):
                self.ds.load_events_text(str(m["events"]))
        elif cmd == "clear":
            self.clear()
        elif cmd == "pipe":
            self.set_pipe(m.get("params") or {})
            await ws.send_str(json.dumps(dict(type="pipe", pipe=self.ds.pipe), ensure_ascii=False))
        elif cmd == "profile":
            self.profile = m.get("name"); self.link_dirty = True
        elif cmd == "analyze":
            tool, args = m.get("tool"), m.get("args") or {}
            loop = asyncio.get_event_loop()
            res = await loop.run_in_executor(None, an.run, self.ds, tool, args)
            res["req"] = m.get("req")
            await ws.send_str(json.dumps(dict(type="analysis", **res), ensure_ascii=False, default=_json_default))
        elif cmd == "sim":
            # v21 시뮬 명령 중계 (x0 등) — MuJoCo 가 있을 때만
            if SIM is None:
                raise RuntimeError("시뮬 없음: " + str(SIM_ERR))
            await SIM.handle(ws, m.get("m") or {})
        else:
            raise RuntimeError(f"모르는 명령: {cmd}")


def _json_default(o):
    if isinstance(o, (np.floating, np.integer)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


HUB = LoggerHub()


async def ws2_handler(request):
    ws = web.WebSocketResponse(heartbeat=20, max_msg_size=256 * 1024 * 1024)
    await ws.prepare(request)
    HUB.clients.add(ws)
    HUB.link_dirty = True
    print(f"[ws2] client connected ({len(HUB.clients)})", flush=True)
    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                try:
                    await HUB.handle(ws, json.loads(msg.data))
                except Exception as ex:
                    HUB.say(f"!! {type(ex).__name__}: {ex}", "err")
                    await ws.send_str(json.dumps(dict(type="error", msg=str(ex)), ensure_ascii=False))
            elif msg.type == WSMsgType.ERROR:
                break
    finally:
        HUB.clients.discard(ws)
    return ws


async def logger_index(request):
    return web.FileResponse(os.path.join(HERE, "static", "logger.html"))


async def pres_index(request):
    return web.FileResponse(os.path.join(HERE, "static", "index.html"))


def _static_page(name):
    async def h(request):
        return web.FileResponse(os.path.join(HERE, "static", name))
    return h


def build_app():
    app = web.Application()
    app.router.add_get("/", logger_index)
    app.router.add_get("/pres", pres_index)
    app.router.add_get("/lab", _static_page("lab.html"))      # 측정실 (엑셀식 표·차트·추세선)
    app.router.add_get("/show", _static_page("show.html"))    # 무대 (세로 모니터)
    app.router.add_get("/deck", _static_page("deck.html"))    # 조종석
    app.router.add_get("/ws2", ws2_handler)
    if simsrv is not None:
        app.router.add_get("/ws", simsrv.ws_handler)
    app.router.add_get("/three.min.js", lambda r: web.FileResponse(os.path.join(HERE, "static", "three.min.js")))
    app.router.add_get("/commands.json", lambda r: web.FileResponse(os.path.join(HERE, "commands.json")))
    media = os.path.join(HERE, "static", "media")
    os.makedirs(media, exist_ok=True)
    os.makedirs(LOGS, exist_ok=True)
    app.router.add_static("/media/", media)
    pmedia = os.path.join(HERE, "..", "presentation", "media")               # 실사 영상 (jultagi_6s_stab.webm)
    if os.path.isdir(pmedia):
        app.router.add_static("/pmedia/", pmedia)
    app.router.add_static("/repo/", os.path.abspath(os.path.join(HERE, "..")))   # 옛 버전 시뮬(v1~v18 index.html)을 갤러리로 — 로컬 발표용
    app.router.add_static("/logs/", LOGS, show_index=True)
    app.router.add_static("/static/", os.path.join(HERE, "static"))

    async def on_startup(app):
        loop = asyncio.get_event_loop()
        loop.create_task(HUB.loop())
        if SIM is not None:
            loop.create_task(SIM.loop())
    app.on_startup.append(on_startup)
    return app


def main(argv=None):
    ap = argparse.ArgumentParser(description="v22 로거·측정 앱 서버")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--fake", nargs="?", const="synth", default=None,
                    help="장치 없이 시작: 파일 이름(logs/ 또는 기존 CSV) 또는 생략=합성 신호")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--mujoco", action="store_true", help="장치 없이 시작: MuJoCo 가상 로봇 (v22_raw v2 흉내, 'sim release β φ' 로 사람 동작)")
    ap.add_argument("--no-autorec", action="store_true", help="연결 시 자동 기록 끄기")
    a = ap.parse_args(argv)
    HUB.autorec = not a.no_autorec
    if a.mujoco:
        HUB.connect_mujoco()
    elif a.fake:
        HUB.connect_fake(None if a.fake == "synth" else a.fake, a.speed)
    app = build_app()
    print("=" * 64)
    print("  v22 로거·측정 앱  →  http://localhost:%d/      (발표 시뮬: /pres)" % a.port)
    print("  시뮬 모델: " + ("MuJoCo OK" if SIM is not None else "없음 — " + str(SIM_ERR)))
    print("  로그 폴더: " + LOGS)
    print("=" * 64)
    web.run_app(app, port=a.port, print=None)


if __name__ == "__main__":
    main()
