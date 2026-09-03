// lg_deck.js — 조종석: 큐 목록(무대 장면 신호) · 로봇 명령 · 상태 판독 · 트윈 미리보기 · 질의응답 버튼
"use strict";
(function () {
  const el = LG.el, fmt = LG.fmt;
  // ================= 큐 (발표 시나리오 — 씬 순서 고정, 2026-09-03) =================
  const CUES = [
    { n: "1", t: "문제 — 장대 없이 어떻게?", v: "video play",
      say: "서양 줄타기는 장대로 기운 무게중심을 되돌린다. 한국 광대는 부채뿐이다. 어떻게 서나?",
      scene: { name: "video", args: { mode: "play", overlay: false, badge: "1 / 8  문제", caption: "장대가 없다. 어떻게 서는가?" } } },
    { n: "2", t: "발상 — 발을 옮긴다", v: "video loop + overlay",
      say: "줄이 흔들리니 발을 옮길 수 있다. 발을 무게중심 너머로 던지면 쓰러지는 방향이 뒤집힌다.",
      scene: { name: "video", args: { mode: "loop", overlay: true, badge: "2 / 8  발상", caption: "줄이 흔들린다 → 발을 옮길 수 있다", sub: "발을 무게중심 너머로 던지면 쓰러지는 방향이 뒤집힌다" } } },
    { n: "3", t: "수단 — 허리를 접는다", v: "twin (접기 시연)",
      say: "발은 허리를 접어서 옮긴다. 기운 만큼에 비례해 접자. 그 비율이 ε 이다.",
      scene: { name: "twin", args: { plane: "none", cam: "side", badge: "3 / 8  수단", caption: "허리를 접으면 발이 반대로 간다", sub: "기운 만큼에 비례해 접는다 — 그 비율 ε" } } },
    { n: "4", t: "균형의 재정의 — 줄과 함께 흔들린다", v: "twin + pl1",
      say: "최적 ε 을 찾으려면 균형을 다시 정의해야 했다. 한국 줄타기의 균형은 줄과 함께 흔들리는 것이다.",
      scene: { name: "twin", args: { plane: "pl1", cam: "side", badge: "4 / 8  균형의 재정의", caption: "균형 = 곧게 서는 것이 아니라 줄과 함께 흔들리는 것", sub: "안정모드선 위에 놓고 손을 뗀다 — 넘어지지 않고 흔들린다" } } },
    { n: "5", t: "모드 — 안정모드와 불안정모드", v: "pl1 크게",
      say: "φ 와 β 가 알맞은 비율로 흔들리면 구동 없이도 균형이다. 고유값 문제를 풀면 안정모드와 불안정모드가 나온다. 불안정모드를 지우는 것이 균형이다.",
      scene: { name: "plane", args: { which: "pl1", zoom: 6, badge: "5 / 8  모드", caption: "안정모드 = 원점을 지나는 직선 · 불안정모드 = 그 선에서 벗어나는 성분", sub: "불안정모드를 지우는 것이 한국 줄타기의 균형" } } },
    { n: "6", t: "★ 클라이막스 — 두 직선의 교점 ε*", v: "pl1 + ε* 기하 + 식",
      say: "허리 접기는 무게중심 불변 직선을 따라 상태를 옮긴다. 이 직선과 안정모드선의 교점이 곧 최적 접기비 ε* 이고, 닫힌 식으로 나온다.",
      scene: { name: "plane", args: { which: "pl1", epsGeom: true, zoom: 6, badge: "6 / 8  ★ 닫힌해", caption: "접기 = 무게중심 불변 직선을 따라 이동 → 안정모드선과의 교점이 ε*", sub: "ε* = −h / (rR + h)   (β₀ 에 무관 → 비례 제어)" } } },
    { n: "7", t: "한계와 답 — 말안장, 증분접기", v: "twin + pl2 (live)",
      say: "그런데 이 값을 넣어도 로봇은 서지 못한다. 말안장 곡면이라 불안정모드가 다시 자란다. 그래서 자란 만큼 다시 접는다.",
      scene: { name: "twin", args: { plane: "pl2", cam: "side", badge: "7 / 8  한계와 답", caption: "아무리 잘 접어도 불안정모드는 다시 자란다 → 자란 만큼 다시 접는다", sub: "예측점이 A=0 선을 벗어날 때마다 접어서 되돌린다" } } },
    { n: "8", t: "방점 — 어름이", v: "twin 크게 + HUD",
      say: "그 로봇이 '어름이' 다.",
      scene: { name: "twin", args: { plane: "none", cam: "side", badge: "8 / 8  어름이", caption: "장대 없는 줄타기 균형을 물리적으로 해석하고 로봇으로 실증했다" } } },
  ];
  const QA = [
    { l: "실사 반복+오버랩", scene: { name: "video", args: { mode: "loop", overlay: true, caption: "실사 위 모델 — 휘청 구간 반복" } } },
    { l: "실사 → 모델 변신", scene: { name: "video", args: { mode: "morph", overlay: true, caption: "사람 스케일 → 60 cm 로봇" } } },
    { l: "실사 정지", scene: { name: "video", args: { mode: "still", overlay: false } } },
    { l: "트윈 크게", scene: { name: "twin", args: { plane: "none", cam: "side" } } },
    { l: "트윈 + 원위치", scene: { name: "twin", args: { plane: "pl1", cam: "side" } } },
    { l: "트윈 + 예측점", scene: { name: "twin", args: { plane: "pl2", cam: "side" } } },
    { l: "원위치 평면 크게", scene: { name: "plane", args: { which: "pl1" } } },
    { l: "ε* 기하 (라이브)", scene: { name: "plane", args: { which: "pl1", epsGeom: true, zoom: 6, caption: "지금 접으면 상태는 흰 점선을 따라 초록 점으로 간다" } } },
    { l: "예측점 평면 크게", scene: { name: "plane", args: { which: "pl2" } } },
    { l: "v21 발표 시뮬 (/pres)", scene: { name: "frame", args: { url: "/pres" } } },
    { l: "측정실 차트", scene: { name: "frame", args: { url: "/lab" } } },
    { l: "ε* 카드", scene: { name: "card", args: { title: "최적 접기비 — 닫힌해", formula: "ε* = −h / (r·R + h)", lines: ["r = 안정모드 고유벡터비 (φ/β)", "R = 줄 처짐(진자 반지름), h = 무게중심 높이", "β₀ 에 무관 → 기운 만큼에 비례해 접으면 된다"] } } },
    { l: "검은 화면", scene: { name: "black", args: {} } },
  ];
  const VERSIONS = [
    ["v1_single_body", "V1 단일 몸체"], ["v2_two_body", "V2 2-Body 힙 토크"], ["v3_three_dof", "V3 3자유도"], ["v4_lqr", "V4 LQR"], ["v5_latency", "V5 지연"],
    ["v6_3d", "V6 3D"], ["v7_estimation", "V7 추정"], ["v8_observer", "V8 관측기"], ["v9_robustness", "V9 강건성"], ["v10_adaptive", "V10 적응"],
    ["v11_small", "V11 소형 (현 실물 파라미터)"], ["v16_fwe", "V16 FWE"], ["v17_fwe", "V17 FWE"], ["v18_optimized", "V18 최적화 (MuJoCo 포함)"],
  ];
  const ROBOT = [["z", "z 영점"], ["k", "k 토크ON"], ["u", "u 토크해제"], ["mode 0", "mode 0 측정"], ["mode 1", "mode 1 단일접기"], ["mode 2", "mode 2 증분접기"], ["g", "g 시작"], ["h", "h 정지"], ["x", "x 비상정지"], ["t", "t 상태"]];
  let cueIdx = -1, stage = null;

  // ================= 큐 목록 =================
  function buildCues() {
    const box = el("cues"); box.innerHTML = "";
    CUES.forEach((c, i) => { const d = document.createElement("div"); d.className = "cue"; d.innerHTML = `<div class="n">${c.n}</div><div><div class="t">${c.t}</div><div class="s">${c.say}</div></div><div class="v">${c.v}</div>`; d.onclick = () => go(i); box.appendChild(d); });
  }
  function go(i) {
    if (i < 0 || i >= CUES.length) return;
    cueIdx = i; const c = CUES[i];
    LG.send({ cmd: "scene", name: c.scene.name, args: Object.assign({ mirror: el("cMirror").checked }, c.scene.args) });
    el("say").textContent = c.say;
    document.querySelectorAll(".cue").forEach((d, k) => d.classList.toggle("on", k === i));
  }
  function sendScene(sc) { LG.send({ cmd: "scene", name: sc.name, args: Object.assign({ mirror: el("cMirror").checked }, sc.args) }); document.querySelectorAll(".cue").forEach(d => d.classList.remove("on")); cueIdx = -1; }
  LG.on("scene", m => { stage = m; el("hStage").textContent = `무대: ${m.name}${m.args && m.args.badge ? " · " + m.args.badge : ""}`; el("hStage").className = "chip on"; el("stage").textContent = `무대 #${m.seq} ${m.name} ${JSON.stringify(m.args || {}).slice(0, 120)}`; });
  LG.on("hello", m => { if (m.scene) { el("hStage").textContent = `무대: ${m.scene.name}`; } buildVersions(); });
  document.addEventListener("keydown", e => {
    if (["INPUT", "SELECT", "TEXTAREA"].includes(e.target.tagName)) return;
    if (e.key === "ArrowRight" || e.key === "PageDown" || e.code === "Space") { e.preventDefault(); go(Math.min(CUES.length - 1, cueIdx + 1)); }
    else if (e.key === "ArrowLeft" || e.key === "PageUp") { e.preventDefault(); go(Math.max(0, cueIdx - 1)); }
    else if (e.key >= "1" && e.key <= "9") { const i = +e.key - 1; if (i < CUES.length) go(i); }
    else if (e.key === "b" || e.key === "B") sendScene({ name: "black", args: {} });
    else if (e.key === "Home") go(0);
  });
  el("bBlack").onclick = () => sendScene({ name: "black", args: {} });
  el("bOpenShow").onclick = () => window.open("/show", "v22show", "width=608,height=1080");
  el("bOpenLab").onclick = () => window.open("/lab", "v22lab");
  el("cMirror").checked = !!LG.store.get("deck.mirror", false);
  el("cMirror").onchange = () => { LG.store.set("deck.mirror", el("cMirror").checked); if (stage) LG.send({ cmd: "scene", name: stage.name, args: Object.assign({}, stage.args || {}, { mirror: el("cMirror").checked }) }); };

  // ================= 질의응답 · 옛 버전 =================
  (function () { const g = el("qaBtns"); for (const q of QA) { const b = document.createElement("button"); b.className = "btn sm"; b.textContent = q.l; b.onclick = () => sendScene(q.scene); g.appendChild(b); } })();
  function buildVersions() { const s = el("selVer"); if (s.options.length) return; for (const [d, l] of VERSIONS) { const o = document.createElement("option"); o.value = `/repo/${d}/index.html`; o.textContent = l; s.appendChild(o); } }
  buildVersions();
  el("bVer").onclick = () => sendScene({ name: "frame", args: { url: el("selVer").value, caption: el("selVer").selectedOptions[0].textContent } });
  el("bVerHere").onclick = () => window.open(el("selVer").value, "v22ver");
  el("bScale").onclick = () => { const w = +el("iMonW").value, p = +el("iMonPx").value; if (!(w > 0 && p > 0)) { LG.toast("모니터 가로 mm 와 가로 px 를 넣으세요"); return; } LG.store.set("deck.scale", { mm_w: w, px_w: p }); el("hScale").textContent = `1 px = ${(w / p).toFixed(3)} mm`; LG.send({ cmd: "scene", name: (stage && stage.name) || "twin", args: Object.assign({}, (stage && stage.args) || { plane: "none" }, { scale: { mm_w: w, px_w: p } }) }); };
  (function () { const s = LG.store.get("deck.scale", null); if (s) { el("iMonW").value = s.mm_w; el("iMonPx").value = s.px_w; el("hScale").textContent = `1 px = ${(s.mm_w / s.px_w).toFixed(3)} mm`; } })();

  // ================= 로봇 =================
  LG.on("ws", ok => { const c = el("hWs"); c.textContent = ok ? "서버 OK" : "서버 끊김"; c.className = "chip" + (ok ? " on" : " warn"); });
  LG.on("link", m => {
    const c = el("hConn"), s = m.src || {};
    if (m.connected) { c.textContent = (s.kind === "serial" ? `${s.port} @${s.baud}` : s.kind === "mujoco" ? `MuJoCo 가상 · ${{ held: "손에", moving: "옮기는 중", settle: "놓기 직전", free: "자유" }[s.stage] || s.stage}` : `가짜`) + ` · ${m.rate_hz} Hz`; c.className = "chip on"; }
    else { c.textContent = m.err ? "끊김: " + m.err : "연결 없음"; c.className = "chip" + (m.err ? " warn" : ""); }
    const r = el("hRec"); if (m.rec) { r.textContent = `● REC ${m.rec.name} · ${m.rec.n_data}행`; r.className = "chip rec"; } else { r.textContent = "기록 없음"; r.className = "chip"; }
  });
  el("bConnect").onclick = () => LG.send({ cmd: "connect" }); el("bMj").onclick = () => LG.send({ cmd: "mujoco" }); el("bDisc").onclick = () => LG.send({ cmd: "disconnect" });
  const sendCmd = t => { if (!t) return; LG.send({ cmd: "send", text: t }); };
  el("bCmd").onclick = () => { sendCmd(el("iCmd").value.trim()); el("iCmd").value = ""; }; el("iCmd").addEventListener("keydown", e => { if (e.key === "Enter") { sendCmd(el("iCmd").value.trim()); el("iCmd").value = ""; } });
  (function () { const g = el("robotBtns"); for (const [c, l] of ROBOT) { const b = document.createElement("button"); b.className = "btn sm" + (c === "x" ? " danger" : c === "g" ? " acc" : ""); b.textContent = l; b.onclick = () => { if (c === "x" && !confirm("비상정지 (토크 OFF) — 보낼까요?")) return; sendCmd(c); }; g.appendChild(b); } })();
  const foldDemo = sgn => { sendCmd(String(20 * sgn)); setTimeout(() => sendCmd("0"), 1500); };
  el("bFoldDemo").onclick = () => foldDemo(1); el("bFoldDemoN").onclick = () => foldDemo(-1);

  // ================= 상태 판독 · 트윈 =================
  let goMs = null, folds = 0, evSeen = 0;
  LG.on("aux", m => { const ev = m.events || []; for (let k = evSeen; k < ev.length; k++) { const e = ev[k]; if (e[1] === "GO") { goMs = e[0]; folds = 0; } else if (e[1] === "FOLD") folds++; else if (e[1] === "STOP" || e[1] === "FALL") goMs = null; } evSeen = ev.length; });
  LG.on("ds_full", () => { evSeen = 0; goMs = null; folds = 0; });
  let last = 0;
  function frame() {
    requestAnimationFrame(frame);
    LG.render3d();
    const now = performance.now(); if (now - last < 100) return; last = now;
    const i = LG.cur(); if (i < 0) return;
    const p = LG.poseAt(i), tms = LG.val("t_ms", i); const up = goMs != null && tms >= goMs ? (tms - goMs) / 1000 : null;
    el("ro").innerHTML = `φ <b>${fmt(p.phi)}</b>°  β <b>${fmt(LG.val("a_beta", i))}</b>°  Â <b>${fmt(LG.val("a_Ahat", i))}</b>°  δ ${fmt(LG.val("del", i), 1)}°  hold ${fmt(LG.val("hold", i), 1)}°<br>` +
      `phase ${LG.phaseName(LG.val("phase", i))} · 균형 유지 <b>${up != null ? up.toFixed(1) + " s" : "—"}</b> · 접기 ${folds}회 · t ${fmt(LG.tOf(i), 1)} s`;
  }
  buildCues(); LG.cam.side(false); frame();
  LG.connect();
})();
