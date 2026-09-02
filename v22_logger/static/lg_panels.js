// lg_panels.js — 오른쪽 패널(원본값·명령·기록·파이프라인·콘솔) · 트랜스포트(커서/재생) · 분석 패널 · 키보드 · 프레임 루프
"use strict";
(function () {
  const el = LG.el, fmt = LG.fmt;
  const PIPE_DEFAULT = { p2r: 0.4285, r: -1.506, c0: 0.0, lam: 5.66, wf: 0.1945, wb: 0.3049, vg: 1.0, phi_eq: 0.0, diff_ms: 25, tau_ms: 28, smooth_ms: 50, alpha_mode: "ank-phi", unwrap: true, snap: true, phi_off: 0, ank_off: 0 };

  // ================= 탭 =================
  document.querySelectorAll(".tabs button").forEach(b => b.onclick = () => {
    document.querySelectorAll(".tabs button").forEach(x => x.classList.toggle("on", x === b));
    document.querySelectorAll(".pane").forEach(p => p.classList.toggle("on", p.id === "pane_" + b.dataset.tab));
  });
  el("bHide").onclick = () => document.body.classList.toggle("hide");

  // ================= 헤더 칩 =================
  LG.on("ws", ok => { const c = el("hWs"); c.textContent = ok ? "서버 OK" : "서버 끊김 — 재접속 중"; c.className = "chip" + (ok ? " on" : " warn"); });
  LG.on("link", m => {
    const c = el("hConn");
    if (m.connected) { const s = m.src; c.textContent = (s.kind === "serial" ? `${s.port} @${s.baud}` : `가짜 ${s.file || "합성"} ×${s.speed}`) + ` · ${m.rate_hz} Hz`; c.className = "chip on"; }
    else { c.textContent = m.err ? "끊김: " + m.err : "연결 없음"; c.className = "chip" + (m.err ? " warn" : ""); }
    const r = el("hRec");
    if (m.rec) { r.textContent = `● REC ${m.rec.name} · ${m.rec.n_data}행 · ${m.rec.elapsed}s`; r.className = "chip rec"; el("recInfo").innerHTML = `<b style="color:#ff6b6b">● 기록 중</b> logs/${m.rec.name}.csv — D행 ${m.rec.n_data} · R ${m.rec.n_trial} · E ${m.rec.n_event} · ${(m.rec.nbytes / 1024).toFixed(0)} KB · ${m.rec.elapsed}s<br><span style="color:var(--dim)">${(m.rec.files || []).join(" · ")}</span>`; }
    else { r.textContent = "기록 없음"; r.className = "chip"; el("recInfo").textContent = m.connected ? "기록 정지 상태 — 「새 파일」로 다시 시작" : "기록 없음 (연결하면 자동 시작)"; }
    el("hBuf").textContent = `버퍼 ${m.n} 행 · ${fmt(m.last_t, 1)}s` + (LG.ds.name ? ` · ${LG.ds.name}` : "");
    el("cAutoRec").checked = !!m.autorec;
    el("bConnect").disabled = m.connected; el("bDisc").disabled = !m.connected;
    el("bFileLoad").disabled = m.connected;
    if (m.profile && m.profile !== LG.profile) { LG.profile = m.profile; buildPalette(); }
    updateMode();
  });

  // ================= 포트 · 연결 =================
  LG.on("ports", m => {
    const s = el("portSel"); const keep = s.value; s.innerHTML = `<option value="">포트 자동 ${m.auto ? "(" + m.auto + ")" : ""}</option>`;
    for (const p of m.ports) { const o = document.createElement("option"); o.value = p.device; o.textContent = (p.hint ? "★ " : "") + p.device + " — " + p.description; s.appendChild(o); }
    if (keep) s.value = keep;
    if (m.err) LG.toast(m.err, true);
  });
  el("bPorts").onclick = () => LG.send({ cmd: "ports" });
  el("bConnect").onclick = () => LG.send({ cmd: "connect", port: el("portSel").value, baud: +el("iBaud").value || 115200, name: el("iRecName").value.trim() });
  el("bDisc").onclick = () => LG.send({ cmd: "disconnect" });
  el("bFakeSynth").onclick = () => LG.send({ cmd: "fake" });
  el("bFakeFile").onclick = () => { const f = el("fileList").value; if (!f) { LG.toast("기록·파일 탭에서 파일을 고르세요"); return; } LG.send({ cmd: "fake", file: f, speed: +el("selSpeed").value || 1 }); };

  // ================= 명령 팔레트 =================
  function buildPalette() {
    const C = LG.CMDS; if (!C) return;
    const ps = el("profileSel"); ps.innerHTML = "";
    for (const k of Object.keys(C.profiles)) { const o = document.createElement("option"); o.value = k; o.textContent = k; ps.appendChild(o); }
    if (!LG.profile || !C.profiles[LG.profile]) LG.profile = C.default || Object.keys(C.profiles)[0];
    ps.value = LG.profile;
    const P = C.profiles[LG.profile]; const g = el("cmdGroups"); g.innerHTML = "";
    const t = document.createElement("div"); t.className = "hint"; t.textContent = P.title || ""; g.appendChild(t);
    for (const grp of P.groups || []) {
      const h = document.createElement("div"); h.className = "hint"; h.style.marginTop = "6px"; h.textContent = "▸ " + grp.name; g.appendChild(h);
      const grid = document.createElement("div"); grid.className = "cmdgrid";
      for (const it of grp.items) {
        const b = document.createElement("button"); b.className = "btn" + (it.accent ? " accent" : "") + (it.danger ? " danger" : ""); b.textContent = it.label; b.title = (it.help || "") + `  [${it.cmd}]`;
        b.onclick = () => { if (it.danger && !confirm(`${it.label} — ${it.help}\n보낼까요?`)) return; LG.send({ cmd: "send", text: it.cmd }); };
        grid.appendChild(b);
      }
      g.appendChild(grid);
    }
    el("deltaRow").style.display = P.delta_move ? "" : "none";
    if (P.delta_move) { el("deltaHelp").textContent = P.delta_move.help || ""; el("iDelta").min = P.delta_move.min; el("iDelta").max = P.delta_move.max; }
    const sel = el("paramSel"); sel.innerHTML = "";
    for (const p of P.params || []) { const o = document.createElement("option"); o.value = p.name; o.textContent = `${p.name} — ${p.what}${p.unit ? " [" + p.unit + "]" : ""}`; sel.appendChild(o); }
    el("paramRow").style.display = (P.params && P.params.length) ? "" : "none";
    el("paramHelp").textContent = P.err_bits || "";
  }
  el("profileSel").onchange = () => { LG.profile = el("profileSel").value; LG.send({ cmd: "profile", name: LG.profile }); buildPalette(); };
  el("bDeltaGo").onclick = () => { const v = Math.round(+el("iDelta").value || 0); LG.send({ cmd: "send", text: String(v) }); };
  el("bParamSet").onclick = () => { const n = el("paramSel").value, v = el("paramVal").value.trim(); if (!n || !v) return; LG.send({ cmd: "send", text: `${n} ${v}` }); };
  el("bParamGet").onclick = () => { const n = el("paramSel").value; if (n) LG.send({ cmd: "send", text: n }); };

  // ================= 콘솔 =================
  const con = el("console");
  function conAdd(lines) {
    const atBottom = con.scrollTop + con.clientHeight >= con.scrollHeight - 30;
    for (const l of lines) { const d = document.createElement("div"); d.className = l[2] || "rx"; d.textContent = l[1]; con.appendChild(d); }
    while (con.children.length > 600) con.removeChild(con.firstChild);
    if (atBottom) con.scrollTop = con.scrollHeight;
  }
  LG.on("hello", m => { con.innerHTML = ""; conAdd(m.console || []); });
  LG.on("console", conAdd);
  function sendLine() {
    const t = el("iLine").value; if (!t.trim()) return;
    if (t.trim().startsWith("#")) LG.send({ cmd: "mark", text: "MEMO " + t.trim().slice(1).trim() });
    else LG.send({ cmd: "send", text: t.trim() });
    el("iLine").value = "";
  }
  el("bLine").onclick = sendLine; el("iLine").addEventListener("keydown", e => { if (e.key === "Enter") sendLine(); });

  // ================= 기록 · 파일 =================
  el("bRecNew").onclick = () => LG.send({ cmd: "rec", name: el("iRecName").value.trim() });
  el("bRecStop").onclick = () => LG.send({ cmd: "rec_stop" });
  el("cAutoRec").onchange = () => LG.send({ cmd: "autorec", on: el("cAutoRec").checked });
  function mark() { const t = el("iMark").value.trim() || "MARK"; LG.send({ cmd: "mark", text: t }); }
  el("bMark").onclick = mark; el("iMark").addEventListener("keydown", e => { if (e.key === "Enter") mark(); });
  LG.on("files", files => {
    const s = el("fileList"); const keep = s.value; s.innerHTML = "";
    for (const f of files) { const o = document.createElement("option"); o.value = f.name; o.textContent = `${f.dir === "logs" ? "" : "[" + f.dir + "] "}${f.name}  (${(f.size / 1024).toFixed(0)} KB, ${f.mtime}${f.has_events ? ", +events" : ""})`; s.appendChild(o); }
    if (keep) s.value = keep;
  });
  el("bFiles").onclick = () => LG.send({ cmd: "files" });
  el("bFileLoad").onclick = () => { const f = el("fileList").value; if (f) LG.send({ cmd: "load", name: f }); };
  el("bClear").onclick = () => { if (confirm("버퍼를 비웁니다 (기록 파일은 그대로). 계속?")) LG.send({ cmd: "clear" }); };
  el("fUpload").onchange = async ev => {
    const files = [...ev.target.files]; if (!files.length) return;
    const main = files.find(f => !/\.events\.csv$/i.test(f.name)) || files[0];
    const evf = files.find(f => /\.events\.csv$/i.test(f.name));
    const text = await main.text(); const events = evf ? await evf.text() : null;
    LG.send({ cmd: "load_text", name: main.name, text, events });
    ev.target.value = "";
  };

  // ================= 파이프라인 =================
  const FORMULA = `u_phi, u_ank : (phi+phi_off, ank+ank_off) ±180° 언랩 (+감김수 스냅)
a_alpha = u_ank − u_phi            (alpha_mode)
a_theta = a_alpha + del
a_beta  = a_alpha + P2R·del
a_dphi, a_dbeta = diff_ms 기저차분 → EMA(tau_ms)
a_Ahat  = (−1/r)·u_phi + a_beta + vg·wf·a_dphi + vg·wb·a_dbeta + c0/r
a_psi   = u_phi − phi_eq
a_fp, a_bp = (φ, β) + P·(φ̇, β̇)      P = (λI−D̂)⁻¹ (시뮬) / I·(1/λ)
s_*     = 중심 이동평균(smooth_ms) + 중앙차분 (비인과, 지연 0)`;
  function buildPipeForm() {
    el("pipeFormula").textContent = FORMULA;
    const f = el("pipeForm"); f.innerHTML = "";
    const row = document.createElement("div"); row.className = "row";
    for (const k of Object.keys(PIPE_DEFAULT)) {
      const v = LG.PIPE ? LG.PIPE[k] : PIPE_DEFAULT[k];
      const lab = document.createElement("label"); lab.title = (LG.PIPEDOC && LG.PIPEDOC[k]) || "";
      let inp;
      if (typeof PIPE_DEFAULT[k] === "boolean") { inp = document.createElement("input"); inp.type = "checkbox"; inp.checked = !!v; inp.style.width = "auto"; }
      else if (k === "alpha_mode") { inp = document.createElement("select"); for (const o of ["ank-phi", "ank+phi", "fw"]) { const op = document.createElement("option"); op.value = o; op.textContent = o; inp.appendChild(op); } inp.value = v; }
      else { inp = document.createElement("input"); inp.type = "number"; inp.step = "any"; inp.value = v; }
      inp.dataset.k = k; lab.appendChild(document.createTextNode(k + (LG.PIPEDOC && LG.PIPEDOC[k] ? " — " + LG.PIPEDOC[k].split(" (")[0].slice(0, 26) : ""))); lab.appendChild(inp); row.appendChild(lab);
    }
    f.appendChild(row);
  }
  function readPipeForm() { const o = {}; el("pipeForm").querySelectorAll("[data-k]").forEach(i => { o[i.dataset.k] = i.type === "checkbox" ? i.checked : (i.tagName === "SELECT" ? i.value : +i.value); }); return o; }
  el("bPipeApply").onclick = () => LG.send({ cmd: "pipe", params: readPipeForm() });
  el("bPipeReset").onclick = () => { LG.PIPE = Object.assign({}, PIPE_DEFAULT); buildPipeForm(); LG.send({ cmd: "pipe", params: PIPE_DEFAULT }); };
  LG.on("pipe", () => { buildPipeForm(); LG.toast("파이프라인 적용 — 파생열 재계산됨"); });
  LG.on("hello", m => {
    buildPalette(); buildPipeForm(); buildTools();
    const P = LG.PL || {};
    el("simNote").textContent = (m.sim_note ? "⚠ " + m.sim_note + "  " : "MuJoCo 모델 상수 사용  ") + `· 모델 r=${fmt(P.r, 3)} slopeA0=${fmt(P.slopeA0, 3)} λ=${fmt(P.lam, 2)} sCoM=${fmt(P.sCoM, 3)} P=[[${fmt(P.P && P.P[0][0], 3)},${fmt(P.P && P.P[0][1], 3)}],[${fmt(P.P && P.P[1][0], 3)},${fmt(P.P && P.P[1][1], 3)}]]`;
    LG.send({ cmd: "ports" });
  });
  el("iTrig").onchange = () => { LG.trig = +el("iTrig").value || 0.6; };

  // ================= 원본값 · 파생값 =================
  let rawRows = [];
  function buildRawTable() {
    const tb = el("rawTable"); tb.innerHTML = ""; rawRows = [];
    const hdr = LG.aux.header || [];
    for (const nm of hdr) { const tr = document.createElement("tr"); tr.innerHTML = `<td class="k">${nm}</td><td class="v">—</td>`; tb.appendChild(tr); rawRows.push([nm, tr.children[1]]); }
    if (!hdr.length) tb.innerHTML = `<tr><td class="k">—</td><td class="v">데이터 없음</td></tr>`;
    el("dsInfo").textContent = (LG.aux.name ? LG.aux.name + " · " : "") + (LG.aux.source || "") + ` · ${LG.aux.n} 행`;
    el("dsNotes").textContent = (LG.aux.notes || []).join(" · ");
  }
  const DER = [["t", "t [s]"], ["u_phi", "φ 언랩"], ["u_ank", "ank 언랩"], ["a_alpha", "α = ank−φ", "alpha_fw"], ["a_theta", "θ = α+δ"], ["a_beta", "β = α+P2R·δ", "beta_fw"],
               ["a_dphi", "φ̇ [°/s]", "dphi_fw"], ["a_dbeta", "β̇ [°/s]", "dbeta_fw"], ["a_Ahat", "Â [°]", "Ahat_fw"], ["a_psi", "ψ = φ−φ_eq"], ["a_fp", "φ_pred"], ["a_bp", "β_pred"],
               ["s_phi", "φ 평활"], ["s_beta", "β 평활"], ["phase", "phase", null, "phase"], ["err", "err", null, "err"], ["cue", "cue"]];
  const derRows = [];
  (function () { const tb = el("derTable"); for (const d of DER) { const tr = document.createElement("tr"); tr.innerHTML = `<td class="k">${d[1]}</td><td class="v">—</td><td class="d"></td>`; if (d[0] === "a_Ahat" || d[0] === "u_phi" || d[0] === "a_beta") tr.className = "hl"; tb.appendChild(tr); derRows.push([d, tr.children[1], tr.children[2]]); } })();
  function stuck(a, i) {                        // 최근 2 s 동안 값이 전혀 안 변했으면 true (엔코더 고착 표시)
    if (!a || i < 10) return false;
    const t = LG.ds.data.t; const j0 = Math.max(0, LG.idxOfT(t[i] - 2.0));
    if (i - j0 < 10) return false;
    const v = a[i]; if (!isFinite(v)) return false;
    for (let k = j0; k < i; k += 2) if (a[k] !== v) return false;
    return true;
  }
  function updateTables() {
    const i = LG.cur(); if (i < 0) return;
    for (const [nm, td] of rawRows) {
      const a = LG.ds.data[nm]; td.textContent = a ? fmt(a[i], nm === "t_ms" ? 0 : 3) : "—";
      if (/^(phi|ank|phi_raw|ank_raw|del_now|dxl_raw|phi_deg|ank_deg|del_now_deg)$/.test(nm)) {
        const s = stuck(a, i); td.style.color = s ? "#ff8080" : ""; if (s) td.textContent += "  ⚠고착 2s"; }
    }
    for (const [d, tv, tdiff] of derRows) {
      const v = LG.val(d[0], i);
      if (d[3] === "phase") { tv.textContent = isFinite(v) ? `${v | 0} ${LG.phaseName(v)}` : "—"; continue; }
      if (d[3] === "err") { tv.textContent = isFinite(v) ? LG.errDecode(v) : "—"; tv.style.color = (v | 0) ? "#ff8080" : ""; continue; }
      tv.textContent = fmt(v, 3);
      if (d[2]) { const f = LG.val(d[2], i); tdiff.textContent = isFinite(f) ? `펌 ${fmt(f, 3)}  Δ${fmt(v - f, 3)}` : ""; }
    }
  }
  function buildTrialTable() {
    const tr = LG.aux.trials || []; const box = el("trialTable");
    if (!tr.length) { box.textContent = "없음 (자유비행 모드 R행이 오면 여기 쌓인다)"; el("trialInfo").textContent = ""; return; }
    const cols = ["trial", "dir", "phi0", "ank0", "beta0", "A0", "lam24", "lam48"];
    let h = `<table class="tb" style="border-collapse:collapse;font-family:Consolas;font-size:10.5px;width:100%"><tr>${cols.map(c => `<th style="color:var(--cyan);text-align:right;font-weight:normal">${c}</th>`).join("")}</tr>`;
    for (const r of tr.slice(-40)) h += `<tr>${cols.map(c => `<td style="text-align:right;padding:0 3px">${r[c] == null ? "" : (typeof r[c] === "number" ? (Number.isInteger(r[c]) ? r[c] : r[c].toFixed(2)) : r[c])}</td>`).join("")}</tr>`;
    h += "</table>";
    const byDir = {};
    for (const r of tr) { const l = +r.lam48; if (l > 0.5) (byDir[r.dir > 0 ? "+" : "−"] = byDir[r.dir > 0 ? "+" : "−"] || []).push(l); }
    const ms = Object.entries(byDir).map(([d, xs]) => `dir ${d}: lam48 평균 ${(xs.reduce((a, b) => a + b, 0) / xs.length).toFixed(2)} (n=${xs.length})`);
    let warn = "";
    if (Object.keys(byDir).length >= 2) { const m = Object.values(byDir).map(xs => xs.reduce((a, b) => a + b, 0) / xs.length); const lo = Math.min(...m), hi = Math.max(...m); warn = lo > 0 && (hi - lo) / lo > 0.2 ? ` ★방향별 λ ${(100 * (hi - lo) / lo).toFixed(0)}% 갈림 — phieq 의심` : " 방향별 λ 일치(20% 이내)"; }
    box.innerHTML = h + `<div class="hint">${ms.join(" · ")}${warn}</div>`;
    el("trialInfo").textContent = `${tr.length} 시행`;
  }
  LG.on("aux", () => { buildRawTable(); buildTrialTable(); });

  // ================= 트랜스포트 =================
  function updateMode() {
    const c = el("tMode");
    if (LG.follow) { c.textContent = LG.link.connected ? "LIVE" : "END"; c.className = "chip on"; }
    else { c.textContent = LG.playing ? "PLAY" : "REPLAY"; c.className = "chip warn"; }
    el("bFollow").classList.toggle("on", LG.follow); el("bPlay").textContent = LG.playing ? "⏸" : "▶";
  }
  LG.setCursor = function (i) {
    const n = LG.ds.n; if (!n) return;
    LG.cursor = Math.max(0, Math.min(n - 1, i | 0)); LG.follow = false; LG.playing = false;
    const t = LG.tOf(LG.cursor);
    if (LG.chart.view && (t < LG.chart.view.t0 || t > LG.chart.view.t1)) { const span = LG.chart.view.t1 - LG.chart.view.t0; LG.chart.view = { t0: t - span * 0.7, t1: t + span * 0.3 }; }
    if (!LG.chart.view) LG.chart.view = { t0: t - LG.chart.win * 0.7, t1: t + LG.chart.win * 0.3 };
    updateMode();
  };
  el("sIdx").oninput = () => LG.setCursor(+el("sIdx").value);
  el("bFollow").onclick = () => { LG.follow = true; LG.cursor = -1; LG.playing = false; LG.chart.view = null; updateMode(); };
  LG.on("follow", updateMode);
  function play() {
    if (!LG.ds.n) return;
    if (LG.playing) { LG.playing = false; updateMode(); return; }
    if (LG.follow || LG.cur() >= LG.ds.n - 1) { LG.setCursor(LG.chart.view ? LG.idxOfT(LG.chart.view.t0) : 0); }
    LG.playing = true; LG._playT0 = performance.now(); LG._playI0 = LG.cur(); LG.playSpeed = +el("selSpeed").value || 1; updateMode();
  }
  el("bPlay").onclick = play;
  el("selSpeed").onchange = () => { LG.playSpeed = +el("selSpeed").value || 1; LG._playT0 = performance.now(); LG._playI0 = LG.cur(); };
  function playTick() {
    if (!LG.playing) return;
    const t = LG.tOf(LG._playI0) + (performance.now() - LG._playT0) / 1000 * LG.playSpeed;
    const i = LG.idxOfT(t);
    if (i >= LG.ds.n - 1) { LG.cursor = LG.ds.n - 1; LG.playing = false; updateMode(); return; }
    LG.cursor = i;
    const v = LG.chart.view; const tt = LG.tOf(i);
    if (v && tt > v.t1 - (v.t1 - v.t0) * 0.1) { const span = v.t1 - v.t0; LG.chart.view = { t0: tt - span * 0.5, t1: tt + span * 0.5 }; }
  }
  const step = k => LG.setCursor(LG.cur() + k);
  el("bStepB").onclick = () => step(-1); el("bStepF").onclick = () => step(1); el("bStepB10").onclick = () => step(-10); el("bStepF10").onclick = () => step(10);
  el("iWin").value = LG.chart.win; el("iWin").onchange = () => { LG.chart.win = Math.max(0.2, +el("iWin").value || 10); LG.store.set("chartWin", LG.chart.win); };
  el("bSelClear").onclick = () => { LG.sel = null; LG.emit("sel"); };
  LG.on("sel", () => { el("tSel").textContent = LG.sel ? `선택 ${LG.sel.t0.toFixed(2)}–${LG.sel.t1.toFixed(2)}s` : ""; if (LG.sel) { el("anT0").value = LG.sel.t0.toFixed(3); el("anT1").value = LG.sel.t1.toFixed(3); } });
  LG.on("ds_full", () => { el("sIdx").max = Math.max(0, LG.ds.n - 1); if (LG.cursor >= LG.ds.n) LG.cursor = LG.ds.n - 1; LG.chartOverlay = []; LG.planeOverlay = []; buildRawTable(); });
  LG.on("ds_append", () => { el("sIdx").max = Math.max(0, LG.ds.n - 1); });

  // 채널 바
  (function () {
    const box = el("chanBox");
    for (const L of LG.chart.lanes) for (const ch of L.chans) {
      const lab = document.createElement("label"); const i = document.createElement("input"); i.type = "checkbox"; i.checked = ch.on;
      i.onchange = () => { ch.on = i.checked; LG.saveChans(); };
      const sw = document.createElement("span"); sw.className = "sw"; sw.style.background = ch.col; if (ch.dash) sw.style.opacity = .6;
      lab.appendChild(i); lab.appendChild(sw); lab.appendChild(document.createTextNode(ch.label)); box.appendChild(lab);
    }
  })();

  // ================= 분석 패널 =================
  const CH_OPTS = ["u_phi", "u_ank", "del", "a_alpha", "a_beta", "a_theta", "a_dphi", "a_dbeta", "a_Ahat", "a_psi", "s_phi", "s_beta", "alpha_fw", "beta_fw", "Ahat_fw", "dphi_fw", "dbeta_fw", "hold", "phase"];
  const TRIAL_P = [{ k: "mode", l: "시행 모드", v: "auto", t: "sel", o: ["auto", "phase", "rel"] }, { k: "phi_eq", l: "φ_eq (빈칸=파이프)", v: "" }, { k: "reldet", l: "놓기 이탈 문턱 °", v: 1.0 },
                   { k: "fcatch", l: "종료 |φ−φ_q|", v: 8.5 }, { k: "quiet_s", l: "직전 정지 s", v: 0.5 }, { k: "quiet_tol", l: "정지 허용폭 °", v: 0.35 }, { k: "max_rise_s", l: "최대 상승 s", v: 2.0 }, { k: "min_peak", l: "최소 진폭", v: 4.0 }, { k: "min_r2", l: "최소 R²", v: 0.9 }];
  const TOOLS = {
    stats: { label: "구간 통계 · 잡음 바닥", win: true, help: "평균·표준편차·드리프트, hp_rms = 이동평균을 뺀 잔차 rms (문서 54 잡음 바닥 방식)", p: [{ k: "hp_ms", l: "고역 창 ms", v: 100 }] },
    linfit: { label: "선형 적합 y = a + b·x", win: true, help: "구간 전 표본 점별 최소제곱. x=del, y=α 이면 P2R = −b 도 표시", p: [{ k: "xch", l: "x 열", v: "del", t: "chan" }, { k: "ych", l: "y 열", v: "a_alpha", t: "chan" }] },
    p2r: { label: "P2R — 실측① (평탄구간 평균 → α 대 δ)", win: true, help: "문서 64 p2r_fit 절차: hold 평탄구간(또는 MOVE 이벤트)으로 나눠 각 구간 끝 avg_s 평균점을 적합. 상행/하행/원점강제/인접차분 교차검산", p: [{ k: "avg_s", l: "구간 끝 평균 s", v: 2.0 }, { k: "seg_mode", l: "구간 나누기", v: "auto", t: "sel", o: ["auto", "hold", "events"] }, { k: "min_seg_s", l: "최소 구간 s", v: 1.5 }, { k: "ych", l: "y 열", v: "a_alpha", t: "chan" }] },
    lambda: { label: "λ — 실측③ (ln|ψ| 직선적합)", win: true, help: "ψ = φ − φ_eq. |ψ| 가 lo 를 위로 지나 hi 에 닿기까지의 상승 구간만 씀. lam24/lam48 = 펌웨어 R행 방식 비교", p: [{ k: "phi_eq", l: "φ_eq (빈칸=파이프)", v: "" }, { k: "lo", l: "밴드 하한 °", v: 2.0 }, { k: "hi", l: "밴드 상한 °", v: 9.0 }, { k: "ch", l: "채널", v: "u_phi", t: "chan" }, { k: "smooth_ms", l: "평활 ms (0=없음)", v: 0 }] },
    trials: { label: "시행 나누기 (놓기 → 발산 → 잡기)", win: "opt", help: "정지(손에 잡힘) → 이탈로 놓기를 찾는다. φ=±3° 에서 놓아도 된다. 놓기점 = 손 뗀 순간의 자세. dir_valid = 방향 확실(놓기 경계용), valid = λ 적합까지 좋음(λ 평균용)", p: TRIAL_P },
    phi_eq: { label: "φ_eq 훑기 (방향별 λ 일치점)", win: "opt", help: "문서 70 §5 — +낙하 λ 와 −낙하 λ 가 만나는 φ_eq. 양방향 시행이 있어야 한다", p: [{ k: "lo", l: "밴드 하한", v: 2.0 }, { k: "hi", l: "밴드 상한", v: 9.0 }, { k: "grid_lo", l: "격자 시작", v: -3 }, { k: "grid_hi", l: "격자 끝", v: 4 }, { k: "step", l: "격자 간격", v: 0.05 }, ...TRIAL_P.slice(2)] },
    osc: { label: "감쇠 진동 (ω, ζ, c_φ) — 매달림 자유흔들기", win: true, help: "영점 교차 → 주기, 봉우리 → 대수감쇠율, 정밀화 = 감쇠 사인 비선형 적합", p: [{ k: "ch", l: "채널", v: "u_phi", t: "chan" }, { k: "smooth_ms", l: "평활 ms", v: 0 }, { k: "refine", l: "정밀화", v: true, t: "bool" }, { k: "I_r", l: "I_r kg·m²", v: 0.0045 }] },
    recommend: { label: "★ 다음 놓기 추천 (r 적응 탐색)", win: "opt", help: "놓을 때마다 s = 방향×발산 초기진폭(선까지 거리) 를 얻어 놓기점에 회귀 → r̂·ĉ₀. 다음 점 = 점이 적은 β 열의 추정선 ± off°. 실행할 때마다 시행을 다시 찾는다", p: [{ k: "off", l: "선에서 벗어남 °", v: 0.5 }, { k: "beta_set", l: "β 열 (쉼표)", v: "2,-2,1,-1,0", t: "text" }, { k: "first_phi", l: "첫 시행 φ", v: 0.7 }, { k: "r_guess", l: "r 초기 가정 (빈칸=파이프)", v: "" }, { k: "lam_fixed", l: "λ 고정 (빈칸=파이프)", v: "" }, ...TRIAL_P.slice(1)] },
    boundary: { label: "놓기 경계 r·c₀ — 실측② 경로②", win: "opt", help: "놓기점 (φ₀, β₀) 와 낙하 방향만 사용 (미분·모델 없음). r 고정 시 절편 c₀ 분리 문턱, r 격자 훑기 포함", p: [{ k: "r_fixed", l: "r 고정 (빈칸=파이프)", v: "" }, { k: "grid_lo", l: "r 격자 시작", v: -3 }, { k: "grid_hi", l: "r 격자 끝", v: -0.8 }, { k: "step", l: "간격", v: 0.01 }, ...TRIAL_P.slice(1)] },
    sysid: { label: "시스템 동정 4×4 — 실측② 경로①", win: "opt", help: "(φ̈, β̈) 를 (φ, β, φ̇, β̇, 1) 로 회귀 → 고유값 λ, 좌고유벡터 → r, wf, wb, c₀. 표본이 적거나 잡음이 크면 흔들린다 — R² 와 고유값을 꼭 볼 것", p: [{ k: "phi_max", l: "|φ| 상한 °", v: 5.0 }, { k: "smooth_ms", l: "SG 창 ms", v: 120 }, { k: "poly", l: "SG 차수", v: 3 }, ...TRIAL_P.slice(1)] },
  };
  let curTool = "lambda", reqId = 0;
  function buildTools() {
    const s = el("anTool"); s.innerHTML = "";
    for (const [k, t] of Object.entries(TOOLS)) { const o = document.createElement("option"); o.value = k; o.textContent = t.label; s.appendChild(o); }
    s.value = curTool; renderParams();
  }
  function renderParams() {
    const T = TOOLS[curTool]; const box = el("anParams"); box.innerHTML = "";
    for (const p of T.p) {
      const lab = document.createElement("label"); lab.textContent = p.l; let inp;
      if (p.t === "chan") { inp = document.createElement("select"); for (const c of CH_OPTS) { const o = document.createElement("option"); o.value = c; o.textContent = c; inp.appendChild(o); } inp.value = p.v; }
      else if (p.t === "sel") { inp = document.createElement("select"); for (const c of p.o) { const o = document.createElement("option"); o.value = c; o.textContent = c; inp.appendChild(o); } inp.value = p.v; }
      else if (p.t === "bool") { inp = document.createElement("input"); inp.type = "checkbox"; inp.checked = !!p.v; inp.style.width = "auto"; }
      else if (p.t === "text") { inp = document.createElement("input"); inp.type = "text"; inp.value = p.v; }
      else { inp = document.createElement("input"); inp.type = "number"; inp.step = "any"; inp.value = p.v; inp.placeholder = "빈칸=기본"; }
      inp.dataset.k = p.k; inp.dataset.t = p.t || "num"; lab.appendChild(inp); box.appendChild(lab);
    }
    el("anHelp").textContent = T.help; el("anWinOptWrap").style.display = T.win === "opt" ? "" : "none";
  }
  el("anTool").onchange = () => { curTool = el("anTool").value; renderParams(); };
  el("bAnSel").onclick = () => { if (LG.sel) { el("anT0").value = LG.sel.t0.toFixed(3); el("anT1").value = LG.sel.t1.toFixed(3); } else LG.toast("차트에서 Shift+드래그로 구간을 먼저 고르세요"); };
  el("bAnAll").onclick = () => { el("anT0").value = ""; el("anT1").value = ""; LG.sel = null; LG.emit("sel"); };
  el("cOverlay").onchange = () => { LG.showOverlay = el("cOverlay").checked; };
  el("anTrial").onchange = () => {
    const k = +el("anTrial").value; const tr = (LG.lastTrials || []).find(r => r.k === k); if (!tr) return;
    const t0 = tr.t0 - 0.3, t1 = tr.t1 + 0.1; el("anT0").value = t0.toFixed(3); el("anT1").value = t1.toFixed(3);
    LG.sel = { t0, t1 }; LG.emit("sel"); LG.follow = false; LG.chart.view = { t0: t0 - 0.5, t1: t1 + 0.5 }; LG.cursor = LG.idxOfT(tr.t0); updateMode();
  };
  function runTool() {
    const T = TOOLS[curTool]; const args = {};
    el("anParams").querySelectorAll("[data-k]").forEach(i => {
      const k = i.dataset.k, t = i.dataset.t;
      if (t === "bool") args[k] = i.checked; else if (t === "chan" || t === "sel" || t === "text") { if (i.value !== "") args[k] = i.value; } else if (i.value !== "") args[k] = +i.value;
    });
    const t0 = el("anT0").value, t1 = el("anT1").value;
    if (T.win === true || (T.win === "opt" && el("cAnWin").checked)) { if (t0 !== "") args.t0 = +t0; if (t1 !== "") args.t1 = +t1; }
    reqId++; LG.send({ cmd: "analyze", tool: curTool, args, req: reqId });
    el("anOut").innerHTML = `<div class="hint">계산 중… (${T.label})</div>`;
  }
  el("bAnRun").onclick = runTool;
  const esc = s => String(s).replace(/[&<>]/g, c => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[c]));
  function fmtV(v) { if (v == null) return "—"; if (typeof v === "number") return Number.isInteger(v) ? String(v) : (Math.abs(v) >= 100 ? v.toFixed(1) : v.toFixed(4)); if (Array.isArray(v)) return "[" + v.map(x => Array.isArray(x) ? "(" + x.map(fmtV).join(",") + ")" : fmtV(x)).join(", ") + "]"; if (typeof v === "object") return ""; return esc(v); }
  const BIG = { lam: 1, P2R: 1, r: 1, c0: 1, next_beta: 1, next_phi: 1, phi_eq_best: 1, zeta: 1, omega_n: 1, slope: 1, T2_ms: 1, lam_plus: 1, lam_minus: 1, n_trials: 1, r_used: 1, r_grid_best: 1, c0_grid_best: 1 };
  function renderResult(res) {
    const T = TOOLS[res.tool] || { label: res.tool };
    let h = `<div><b style="color:${res.ok ? "#06d6a0" : "#ff8080"}">${esc(T.label)}</b> ${res.ok ? "" : "— 실패: " + esc(res.msg || "")}${res.window ? ` <span style="color:var(--dim)">구간 ${res.window[0]}–${res.window[1]} s</span>` : ""}${res.n != null ? ` <span style="color:var(--dim)">n=${res.n}</span>` : ""}</div>`;
    if (res.steps && res.steps.length) h += `<ol>${res.steps.map(s => `<li>${esc(s)}</li>`).join("")}</ol>`;
    if (res.result && Object.keys(res.result).length) {
      h += `<div class="res">`;
      for (const [k, v] of Object.entries(res.result)) {
        if (v && typeof v === "object" && !Array.isArray(v)) { for (const [k2, v2] of Object.entries(v)) h += `<span class="k">${esc(k)}.${esc(k2)}</span><span class="v">${fmtV(v2)}</span>`; }
        else h += `<span class="k">${esc(k)}</span><span class="v ${BIG[k] ? "big" : ""}">${fmtV(v)}</span>`;
      }
      h += `</div>`;
    }
    if (res.table && res.table.length) {
      const cols = [...new Set(res.table.flatMap(r => Object.keys(r)))].filter(c => !c.startsWith("i") || c === "ia" || c === "ib").filter(c => !["ia", "ib", "i0", "i1"].includes(c));
      h += `<table class="tb"><tr>${cols.map(c => `<th>${esc(c)}</th>`).join("")}</tr>`;
      for (const r of res.table) h += `<tr class="${(r.dir_valid === false || (r.valid === false && r.dir_valid === undefined)) ? "bad" : ""}">${cols.map(c => `<td>${fmtV(r[c])}</td>`).join("")}</tr>`;
      h += `</table>`;
    }
    if (res.trace) h += `<pre class="hint">${esc(res.trace)}</pre>`;
    if (res.next) {
      const nx = res.next;
      h = `<div style="border:1px solid var(--cyan);border-radius:8px;padding:8px 10px;margin-bottom:8px;background:rgba(76,201,240,.08)">
        <div style="font-size:11px;color:var(--cyan)">다음 놓기 (${esc(nx.reason || "")})</div>
        <div style="font-family:Consolas;font-size:15px;color:#fff;margin:3px 0">β <b>${(+nx.beta).toFixed(2)}</b>°   φ <b>${(+nx.phi).toFixed(2)}</b>°   (ank ${(+nx.ank).toFixed(2)}°)  ${nx.side > 0 ? "선 위쪽" : "선 아래쪽"}</div>
        <div style="font-size:11px;color:var(--dim)">현재 추정 r = ${fmtV(res.result.r)} ± ${fmtV(res.result.se_r)} · c₀ = ${fmtV(res.result.c0)} ± ${fmtV(res.result.se_c0)} · n = ${res.result.n}${res.result.enough ? '  <b style="color:#06d6a0">— 충분 (SE_r < 0.05)</b>' : ""}</div></div>` + h;
      if (el("cAutoTgt").checked) { el("cTgt").checked = true; el("iTgtB").value = (+nx.beta).toFixed(2); el("iTgtF").value = (+nx.phi).toFixed(2); }
    }
    el("anOut").innerHTML = h;
    LG.drawCurve(el("anCurve"), res.curves && res.curves[0]);
    LG.chartOverlay = res.overlay || []; LG.planeOverlay = res.plane || [];
    if (res.tool === "trials" && res.ok) {
      LG.lastTrials = res.table; const s = el("anTrial"); s.innerHTML = `<option value="">시행…</option>`;
      for (const r of res.table) { const o = document.createElement("option"); o.value = r.k; o.textContent = `시행 ${r.k} ${r.dir > 0 ? "+" : "−"} ${r.t0}s (β${r.beta0}, φ${r.phi0})${r.valid ? "" : (r.dir_valid ? " λ✗" : " ✗")}`; s.appendChild(o); }
    }
  }
  LG.on("analysis", res => {
    renderResult(res);
    const R = res.result || {};
    const key = Object.keys(BIG).filter(k => R[k] != null).slice(0, 3).map(k => `${k}=${fmtV(R[k])}`).join(" ");
    LG.results.unshift({ ts: new Date().toLocaleTimeString(), res, label: `${(TOOLS[res.tool] || {}).label || res.tool} ${res.ok ? key : "✗"}` });
    LG.results = LG.results.slice(0, 30);
    const hbox = el("anHist"); hbox.innerHTML = "결과 기록: "; LG.results.forEach((r, i) => { const s = document.createElement("span"); s.textContent = `[${r.ts}] ${r.label}`; s.onclick = () => renderResult(r.res); hbox.appendChild(s); });
  });

  // ================= 키보드 =================
  document.addEventListener("keydown", e => {
    if (["INPUT", "SELECT", "TEXTAREA"].includes(e.target.tagName)) return;
    if (e.code === "Space") { e.preventDefault(); play(); }
    else if (e.key === "f" || e.key === "F") el("bFollow").click();
    else if (e.key === "m" || e.key === "M") mark();
    else if (e.key === "ArrowLeft") step(e.shiftKey ? -10 : -1);
    else if (e.key === "ArrowRight") step(e.shiftKey ? 10 : 1);
    else if (e.key === "Escape") { LG.sel = null; LG.emit("sel"); }
    else if (e.key === "h" || e.key === "H") document.body.classList.toggle("hide");
    else if (e.key === "Enter") runTool();
  });
  el("bCamSide").onclick = () => LG.cam.side(); el("bCamIso").onclick = () => LG.cam.iso();
  // 아래 영역 높이 스플리터
  (function () {
    const sp = el("splitter"); let d0 = null;
    const saved = LG.store.get("bottomH", null); if (saved) document.documentElement.style.setProperty("--bottomH", saved + "px");
    sp.addEventListener("mousedown", e => { d0 = { y: e.clientY, h: el("splitter").nextElementSibling.getBoundingClientRect().height }; e.preventDefault(); });
    window.addEventListener("mousemove", e => { if (!d0) return; const h = Math.max(160, Math.min(window.innerHeight - 260, d0.h - (e.clientY - d0.y))); document.documentElement.style.setProperty("--bottomH", h + "px"); });
    window.addEventListener("mouseup", () => { if (d0) { LG.store.set("bottomH", parseInt(getComputedStyle(document.documentElement).getPropertyValue("--bottomH")) || null); d0 = null; } });
  })();

  // ================= 프레임 루프 =================
  let lastRead = 0;
  function frame() {
    requestAnimationFrame(frame);
    playTick();
    LG.render3d(); LG.renderPlanes(); LG.renderChart();
    const now = performance.now();
    if (now - lastRead > 66) {
      lastRead = now;
      const i = LG.cur();
      if (i >= 0) {
        const p = LG.poseAt(i), ph = LG.val("phase", i), A = LG.val("a_Ahat", i), Af = LG.val("Ahat_fw", i);
        let tgt = "";
        if (el("cTgt").checked) { const tb = +el("iTgtB").value || 0, tf = +el("iTgtF").value || 0, db = LG.val("a_beta", i) - tb, df = p.phi - tf;
          tgt = `\n목표 (β ${tb.toFixed(1)}, φ ${tf.toFixed(1)}) 까지  Δβ ${db >= 0 ? "+" : ""}${fmt(db)}  Δφ ${df >= 0 ? "+" : ""}${fmt(df)}  ${Math.abs(db) < 0.2 && Math.abs(df) < 0.2 ? "● 놓아도 됨" : "○"}`; }
        el("ov3d").textContent = `t = ${fmt(LG.tOf(i), 3)} s   [${isFinite(ph) ? LG.phaseName(ph) : "—"}]\nφ = ${fmt(p.phi)}°   ank = ${fmt(LG.val("u_ank", i))}°\nα = ${fmt(p.alpha)}°   θ = ${fmt(p.theta)}°   δ = ${fmt(LG.val("del", i))}°\nβ = ${fmt(LG.val("a_beta", i))}°   Â = ${fmt(A, 3)}° (펌 ${fmt(Af, 3)})   |Â|/trig = ${fmt(Math.abs(A) / LG.trig, 2)}\nφ̇ = ${fmt(LG.val("a_dphi", i), 1)}  β̇ = ${fmt(LG.val("a_dbeta", i), 1)} °/s` + tgt;
        el("tTime").textContent = `t = ${fmt(LG.tOf(i), 3)} s  [${i + 1}/${LG.ds.n}]`;
        if (LG.follow) el("sIdx").value = LG.ds.n - 1; else el("sIdx").value = i;
        updateTables();
      }
    }
  }
  buildTools(); buildPipeForm(); updateMode();
  frame();
  LG.connect();
})();
