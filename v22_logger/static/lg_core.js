// lg_core.js — v22 로거 앱 공통 상태 · WebSocket · 바이너리 데이터셋 저장소
// 모든 모듈은 window.LG 하나를 공유한다 (스크립트 순서: core → 3d → plane → chart → panels)
"use strict";
const LG = window.LG = {
  ws: null, wsOK: false,
  PL: null, GEOM: null, PIPE: null, PIPEDOC: null, CMDS: null, profile: null, simNote: null,
  ds: { cols: [], data: {}, n: 0, cap: 0, name: "", source: "" },
  aux: { events: [], trials: [], folds: [], header: null, notes: [], trial_header: null },
  link: { connected: false, rec: null, rate_hz: 0, src: {kind: "none"} },
  console: [],                 // [t, text, kind]
  cursor: -1,                  // -1 = 라이브 꼬리 따라감
  follow: true,
  playing: false, playSpeed: 1.0, _playT0: 0, _playI0: 0,
  sel: null,                   // {t0, t1} 분석 구간 선택
  analysis: null, results: [],
  showOverlay: true,
  trig: 0.6,
  listeners: {},
  on(ev, fn) { (this.listeners[ev] = this.listeners[ev] || []).push(fn); },
  emit(ev, ...a) { for (const f of (this.listeners[ev] || [])) { try { f(...a); } catch (e) { console.error(ev, e); } } },
};

// ---- 열 별칭 (dataset_v22.ALIASES 와 같다) ----
LG.ALIASES = {
  t_ms: ["t_ms", "t", "time_ms"], phi: ["phi", "phi_deg", "f"], ank: ["ank", "ank_deg", "ankle", "ankle_deg", "k"],
  del: ["del_now", "del_now_deg", "delta", "delta_deg", "d"], hold: ["hold", "del_cmd", "del_cmd_deg", "dcmd"],
  alpha_fw: ["alpha", "alpha_deg"], beta_fw: ["beta", "beta_deg"], dphi_fw: ["dphi", "phid", "dphi_dps"],
  dbeta_fw: ["dbeta", "betad", "dbeta_dps"], Ahat_fw: ["Ahat", "ahat", "A_hat", "A"], phase: ["phase"], cue: ["cue"], err: ["err"],
};
LG.col = function (name) {           // 파생열 이름이면 그대로, 원시 canonical 이면 별칭으로 찾는다
  const d = LG.ds.data;
  if (d[name]) return d[name];
  const al = LG.ALIASES[name];
  if (al) for (const a of al) if (d[a]) return d[a];
  return null;
};
LG.val = function (name, i) { const c = LG.col(name); return (c && i >= 0 && i < LG.ds.n) ? c[i] : NaN; };
LG.cur = function () { return LG.cursor >= 0 ? Math.min(LG.cursor, LG.ds.n - 1) : LG.ds.n - 1; };
LG.tOf = function (i) { const t = LG.ds.data.t; return (t && i >= 0 && i < LG.ds.n) ? t[i] : NaN; };
LG.idxOfT = function (tq) {          // t 에서 인덱스 (이분탐색, ≤)
  const t = LG.ds.data.t, n = LG.ds.n; if (!t || !n) return -1;
  let lo = 0, hi = n - 1;
  if (tq <= t[0]) return 0; if (tq >= t[n - 1]) return n - 1;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (t[m] <= tq) lo = m; else hi = m; }
  return lo;
};
LG.idxOfMs = function (ms) {         // t_ms 에서 인덱스 (되감김 전 구간은 무시 — 마지막 단조 구간 기준)
  const c = LG.col("t_ms"), n = LG.ds.n; if (!c || !n) return -1;
  let lo = 0, hi = n - 1;
  if (ms <= c[0]) return 0; if (ms >= c[n - 1]) return n - 1;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (c[m] <= ms) lo = m; else hi = m; }
  return lo;
};
LG.tOfMs = function (ms) { const i = LG.idxOfMs(ms); return i >= 0 ? LG.ds.data.t[i] : NaN; };

// ---- 데이터셋 저장소 (열마다 Float32Array, 용량 2배 증가) ----
function dsEnsure(cols, need) {
  const ds = LG.ds;
  if (need <= ds.cap && cols.every(c => ds.data[c])) return;
  const cap = Math.max(need, ds.cap * 2, 4096);
  for (const c of new Set([...ds.cols, ...cols])) {
    const old = ds.data[c];
    const a = new Float32Array(cap);
    if (old) a.set(old.subarray(0, ds.n)); else a.fill(NaN);
    if (!old) a.fill(NaN);
    ds.data[c] = a;
  }
  ds.cols = [...new Set([...ds.cols, ...cols])];
  ds.cap = cap;
}
function dsReset() { LG.ds = { cols: [], data: {}, n: 0, cap: 0, name: "", source: "" }; }
function dsApplyFrame(header, body) {
  const ds = LG.ds, cols = header.cols, m = cols.length;
  if (header.type === "ds_full") {
    dsReset(); LG.ds.name = header.name || ""; LG.ds.source = header.source || "";
    dsEnsure(cols, header.n);
    const n = header.n;
    for (let k = 0; k < m; k++) LG.ds.data[cols[k]].set(body.subarray(k * n, (k + 1) * n), 0);
    for (const c of LG.ds.cols) if (!cols.includes(c)) LG.ds.data[c].fill(NaN, 0, n);
    LG.ds.n = n;
    if (LG.cursor >= n) LG.cursor = n - 1;
    LG.emit("ds_full");
  } else {
    const n0 = header.n0, n = header.n, k0 = n - n0;
    dsEnsure(cols, n);
    for (let k = 0; k < m; k++) LG.ds.data[cols[k]].set(body.subarray(k * k0, (k + 1) * k0), n0);
    for (const c of LG.ds.cols) if (!cols.includes(c)) LG.ds.data[c].fill(NaN, n0, n);
    LG.ds.n = n;
    LG.emit("ds_append", n0, n);
  }
}

// ---- WebSocket ----
LG.send = function (m) { if (LG.wsOK) LG.ws.send(JSON.stringify(m)); else LG.toast("서버 연결 안 됨"); };
LG.connect = function () {
  const ws = new WebSocket((location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws2");
  ws.binaryType = "arraybuffer";
  LG.ws = ws;
  ws.onopen = () => { LG.wsOK = true; LG.emit("ws", true); ws.send(JSON.stringify({ cmd: "hello" })); };
  ws.onclose = () => { LG.wsOK = false; LG.emit("ws", false); setTimeout(LG.connect, 1500); };
  ws.onerror = () => {};
  ws.onmessage = ev => {
    if (ev.data instanceof ArrayBuffer) {
      const dv = new DataView(ev.data), hl = dv.getUint32(0, true);
      const header = JSON.parse(new TextDecoder().decode(new Uint8Array(ev.data, 4, hl)));
      const body = new Float32Array(ev.data.slice(4 + hl));
      dsApplyFrame(header, body);
      return;
    }
    const m = JSON.parse(ev.data);
    LG.handle(m);
  };
};
LG.handle = function (m) {
  switch (m.type) {
    case "hello":
      LG.PL = m.plane; LG.GEOM = m.geom; LG.PIPE = m.pipe; LG.PIPEDOC = m.pipe_doc; LG.CMDS = m.commands;
      LG.profile = m.profile; LG.simNote = m.sim_note; LG.console = m.console || [];
      LG.emit("hello", m); break;
    case "aux": LG.aux = m; LG.emit("aux", m); break;
    case "link": LG.link = m; LG.emit("link", m); break;
    case "console": for (const l of m.lines) LG.console.push(l); if (LG.console.length > 800) LG.console.splice(0, LG.console.length - 800); LG.emit("console", m.lines); break;
    case "ports": LG.emit("ports", m); break;
    case "files": LG.emit("files", m.files); break;
    case "pipe": LG.PIPE = m.pipe; LG.emit("pipe", m.pipe); break;
    case "analysis": LG.emit("analysis", m); break;
    case "error": LG.toast("오류: " + m.msg, true); LG.emit("error", m); break;
    default: LG.emit(m.type, m);
  }
};

// ---- 유틸 ----
LG.el = id => document.getElementById(id);
LG.fmt = (v, k = 2) => (v == null || !isFinite(v)) ? "—" : (+v).toFixed(k);
LG.toast = function (text, err) {
  const box = LG.el("toast"); if (!box) return;
  const d = document.createElement("div"); d.className = "toast" + (err ? " err" : ""); d.textContent = text;
  box.appendChild(d); setTimeout(() => d.remove(), err ? 6000 : 3000);
};
LG.PHASES = { 0: "IDLE", 1: "FOLD", 2: "REST", 3: "STOP", 4: "대기", 5: "발산", 6: "종료" };
LG.phaseName = function (v) {
  const p = LG.CMDS && LG.profile && LG.CMDS.profiles[LG.profile] && LG.CMDS.profiles[LG.profile].phases;
  if (p && p[String(v | 0)]) return p[String(v | 0)];
  return LG.PHASES[v | 0] || String(v);
};
LG.errDecode = function (e) {
  e = e | 0; if (!e) return "0";
  const out = []; const p = e & 3, a = (e >> 2) & 3, d = (e >> 4) & 1;
  if (p) out.push("φ" + p); if (a) out.push("ank" + a); if (d) out.push("dxl");
  return e + " (" + out.join("·") + ")";
};
LG.store = {
  get(k, d) { try { const v = localStorage.getItem("v22." + k); return v == null ? d : JSON.parse(v); } catch (e) { return d; } },
  set(k, v) { try { localStorage.setItem("v22." + k, JSON.stringify(v)); } catch (e) {} },
};
// 현재 위치의 (φ, α, θ) — 3D·판독 공용. 앱 파생열 우선, 없으면 펌웨어 열.
LG.poseAt = function (i) {
  const phi = LG.val("u_phi", i), al = LG.val("a_alpha", i), th = LG.val("a_theta", i);
  if (isFinite(phi) && isFinite(al) && isFinite(th)) return { phi, alpha: al, theta: th };
  const p2 = LG.val("phi", i), a2 = LG.val("alpha_fw", i), d2 = LG.val("del", i);
  return { phi: p2 || 0, alpha: a2 || 0, theta: (a2 || 0) + (d2 || 0) };
};
