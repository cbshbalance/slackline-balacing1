// lg_show.js — 무대 (세로 모니터). 조종석(/deck)이 보내는 장면 신호(type=scene)대로 한 창 안에서 바꾼다.
//   video : 실사 영상 (재생 / 휘청 구간 반복 + 모델 동기 오버랩 / 정지 / 모델로 변신)
//   twin  : 3D 트윈 (+ 아래 상태공간 pl1|pl2) · HUD(유지 시간·접기 횟수·접기량)
//   plane : 상태공간 크게 (pl1|pl2, 라이브 ε* 기하)
//   frame : iframe (/pres 씬, 옛 버전 시뮬, /lab)
//   card  : 큰 글씨 카드 (식)
//   black : 검은 화면
// lg_core.js(WS·데이터) · lg_3d.js(트윈) · lg_plane.js(평면) 을 그대로 쓴다.
"use strict";
(function () {
  const el = LG.el, fmt = LG.fmt, RAD = Math.PI / 180;
  const ST = { view: "black", scene: null, mirror: LG.store.get("show.mirror", false), scale: LG.store.get("show.scale", null),
               vid: { mode: "still", overlay: false, loop: null, opa: 1.0, morph: null }, hud: { go_ms: null, folds: 0, last: null, flashT: 0 } };
  const vid = el("vid"), ovc = el("ovc");

  // ================= 장면 적용 =================
  function setView(name) {
    for (const v of ["vBlack", "vVideo", "vTwin", "vFrame", "vCard"]) el(v).classList.toggle("on", v === { black: "vBlack", video: "vVideo", twin: "vTwin", plane: "vTwin", frame: "vFrame", card: "vCard" }[name]);
    ST.view = name;
  }
  function caption(text, sub, badge) {
    const c = el("cap"); if (text) { c.innerHTML = text + (sub ? `<span class="sub">${sub}</span>` : ""); c.classList.add("on"); } else c.classList.remove("on");
    const b = el("badge"); if (badge) { b.textContent = badge; b.classList.add("on"); } else b.classList.remove("on");
  }
  function applyScene(sc) {
    ST.scene = sc; const a = sc.args || {};
    if (a.mirror != null) { ST.mirror = !!a.mirror; LG.store.set("show.mirror", ST.mirror); }
    if (a.scale) { ST.scale = a.scale; LG.store.set("show.scale", ST.scale); }
    if (a.trig) LG.trig = +a.trig;
    switch (sc.name) {
      case "video": setView("video"); videoMode(a); break;
      case "twin": setView("twin"); twinLayout(a.plane || "none", a); break;
      case "plane": setView("twin"); twinLayout(a.which || "pl1", Object.assign({ only: true }, a)); break;
      case "frame": setView("frame"); { const f = el("fr"); if (f.getAttribute("src") !== (a.url || "about:blank")) f.src = a.url || "about:blank"; } break;
      case "card": setView("card"); el("cardBox").innerHTML = `${a.title ? `<h1>${a.title}</h1>` : ""}${a.formula ? `<div class="f">${a.formula}</div>` : ""}${(a.lines || []).map(l => `<p>${l}</p>`).join("")}`; break;
      default: setView("black");
    }
    caption(a.caption, a.sub, a.badge);
    el("stat").textContent = `${sc.name} #${sc.seq || 0}`;
    if (sc.name !== "video") stopVideoLoop();
  }
  LG.on("hello", m => { if (m.scene) applyScene(m.scene); });
  LG.on("scene", m => applyScene(m));

  // ================= 트윈 + 평면 배치 =================
  function twinLayout(plane, a) {
    const box = el("vTwin"); box.classList.remove("split", "planeonly");
    if (a.only) box.classList.add("planeonly"); else if (plane !== "none") box.classList.add("split");
    el("pl1").classList.toggle("on", plane === "pl1"); el("pl2").classList.toggle("on", plane === "pl2");
    el("planeTitle").textContent = plane === "pl1" ? "원위치 평면 (β, φ) — 초록 점선 = 안정모드선 · 흰 점선 = 무게중심 불변 직선" : plane === "pl2" ? "예측점 평면 (β_pred, φ_pred) — 초록 = A=0 선 · 노랑 = 트리거 띠" : "";
    LG.PLOPT.epsGeom = !!a.epsGeom;
    LG.PLVIEW.pl1.on = false; LG.PLVIEW.pl2.on = false;
    if (a.zoom) { const V = LG.PLVIEW[plane]; if (V) { V.on = true; V.m = +a.zoom; V.b = 0; V.f = 0; } }
    const cam = a.cam || "side";
    if (cam === "iso") LG.cam.iso(ST.mirror); else LG.cam.side(ST.mirror);
    // 세로 화면: 프레임(1.22 m)이 위 칸을 채우게 가까이
    const tb = el("twinBox"); const portrait = tb.clientHeight > tb.clientWidth * 0.9;
    if (portrait) LG.cam.set({ dist: a.dist || (plane === "none" ? 1.45 : 1.6), tz: 0.78 });
    if (ST.scale && ST.scale.mm_w && ST.scale.px_w && !a.only) {
      // 1:1 축척 — 프레임 높이 1.22 m 가 화면에서 1220 mm 로 보이게 카메라 거리 조정 (원근 fov 그대로)
      const mmPerPx = ST.scale.mm_w / ST.scale.px_w, hPx = el("twinBox").clientHeight;
      const hVis = hPx * mmPerPx / 1000; const c = LG.cam.get(); const fov = c.fov * RAD;
      const dist = hVis / (2 * Math.tan(fov / 2)); LG.cam.set({ dist: Math.max(0.6, dist), tz: 0.72 });
    }
    el("hud").style.display = a.hud === false ? "none" : "";
  }
  // 위·아래 평면 캔버스 크기: 정사각형으로 가운데
  function fitPlanes() {
    const pb = el("planeBox"); if (!pb.clientWidth) return;
    const only = el("vTwin").classList.contains("planeonly");
    const capH = el("cap").classList.contains("on") ? el("cap").offsetHeight + 44 : 12;     // 자막이 켜져 있으면 그만큼 비운다
    const top = only ? 92 : 28;
    const W = pb.clientWidth - 24, H = pb.clientHeight - top - capH; const s = Math.max(10, Math.min(W, H));
    for (const id of ["pl1", "pl2"]) { const c = el(id); c.style.width = s + "px"; c.style.height = s + "px"; c.style.left = ((pb.clientWidth - s) / 2) + "px"; c.style.top = (top + Math.max(0, (H - s) / 2)) + "px"; }
  }

  // ================= HUD (유지 시간 · 접기) =================
  let evSeen = 0;
  LG.on("aux", m => {
    const ev = m.events || [];
    for (let k = evSeen; k < ev.length; k++) {
      const e = ev[k];
      if (e[1] === "GO") { ST.hud.go_ms = e[0]; ST.hud.folds = 0; ST.hud.last = null; }
      else if (e[1] === "FOLD") { ST.hud.folds++; ST.hud.last = parseFloat(e[2]); ST.hud.flashT = performance.now(); }
      else if (e[1] === "STOP" || e[1] === "FALL") { ST.hud.go_ms = null; }
    }
    evSeen = ev.length;
  });
  LG.on("ds_full", () => { evSeen = 0; ST.hud = { go_ms: null, folds: 0, last: null, flashT: 0 }; });
  function renderHud() {
    const i = LG.cur(); if (i < 0) { el("hud").innerHTML = ""; return; }
    const p = LG.poseAt(i), A = LG.val("a_Ahat", i), hold = LG.val("hold", i), ph = LG.val("phase", i) | 0;
    const tms = LG.val("t_ms", i); const up = ST.hud.go_ms != null && tms >= ST.hud.go_ms ? (tms - ST.hud.go_ms) / 1000 : null;
    el("hud").innerHTML = `<div class="big">${up != null ? up.toFixed(1) + " s" : "—"}</div><div class="k">균형 유지</div>` +
      `<div>접기 <b>${ST.hud.folds}</b>회 · hold ${fmt(hold, 1)}°</div>` +
      `<div>φ ${fmt(p.phi, 2)}°  β ${fmt(LG.val("a_beta", i), 2)}°  Â ${fmt(A, 2)}°</div>` +
      `<div class="k">${LG.link.connected ? (LG.link.src && LG.link.src.kind === "mujoco" ? "MuJoCo 가상 로봇" : "어름이 · 실기") : "기록 재생"} · ${LG.phaseName(ph)}</div>`;
    const f = el("flash"); const dt = performance.now() - ST.hud.flashT;
    if (ST.hud.last != null && dt < 900) { f.textContent = `접기 ${ST.hud.last >= 0 ? "+" : ""}${ST.hud.last.toFixed(1)}°`; f.classList.add("on"); } else f.classList.remove("on");
  }

  // ================= 실사 영상 + 오버랩 (three.js, /pres 의 정합 씬 그대로) =================
  const VM = { vw: 1080, vh: 1920, cy: 607, ch: 800, tEnd: 6.2 };          // 사진 = 영상 t* 의 y 607~1407 띠 (문서 8/17 정합)
  const CAM_J = { az: 140.61, el: -5.42, dist: 15.19, fov: 30.0, roll: -3.01, ty: 2.33, tz: 2.86 };
  const CAM_R = { az: 51.6, el: 20.1, dist: 2.6, fov: 42, roll: 0, ty: 0, tz: 0.85 };
  const G_HUM = { D: 11.59, Hs: 2.49, R: 0.70, L1: 0.703, L2: 1.017, footY: 2.75, sag: 0.149 };
  const G_ROB = { D: 0.60, Hs: 1.245, R: 0.433, L1: 0.259, L2: 0.375, footY: 0.0, sag: 0.433 };
  const OVS = { thk: 2.6, body: 1.7, morphT: 0, pose: [0, 0, 0], cam: Object.assign({}, CAM_J) };
  const jr = new THREE.WebGLRenderer({ canvas: ovc, antialias: true, alpha: true });
  jr.setClearColor(0x000000, 0);
  const jscene = new THREE.Scene();
  const jcam = new THREE.PerspectiveCamera(30, 1.35, 0.05, 200); jcam.up.set(0, 0, 1);
  jscene.add(new THREE.AmbientLight(0xffffff, 0.85));
  const jdl = new THREE.DirectionalLight(0xffffff, 0.7); jdl.position.set(2, 1, 4); jscene.add(jdl);
  const jG = new THREE.Group(); jscene.add(jG);
  const jtarget = new THREE.Vector3();
  function jcamUpdate() {
    const c = OVS.cam, az = c.az * RAD, elv = c.el * RAD, roll = c.roll * RAD;
    jtarget.set(0, c.ty, c.tz);
    jcam.position.set(jtarget.x + c.dist * Math.cos(elv) * Math.sin(az), jtarget.y - c.dist * Math.cos(elv) * Math.cos(az), jtarget.z + c.dist * Math.sin(elv));
    jcam.up.set(Math.sin(roll), 0, Math.cos(roll)); jcam.lookAt(jtarget);
    if (jcam.fov !== c.fov) { jcam.fov = c.fov; jcam.updateProjectionMatrix(); }
  }
  const jmat = (c, glow) => { const m = new THREE.MeshLambertMaterial({ color: c }); if (glow) m.emissive = new THREE.Color(c).multiplyScalar(0.28); return m; };
  function jrod(a, b, r, c, glow) {
    const A = new THREE.Vector3(...a), B = new THREE.Vector3(...b), d = B.clone().sub(A), len = Math.max(d.length(), 1e-6);
    const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 14), jmat(c, glow));
    m.position.copy(A.clone().add(B).multiplyScalar(0.5)); m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.clone().normalize()); return m;
  }
  function jClear() { for (let i = jG.children.length - 1; i >= 0; i--) { const o = jG.children[i]; if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); jG.remove(o); } }
  function jGeom() { const t = OVS.morphT, g = {}; for (const k in G_HUM) g[k] = G_HUM[k] + (G_ROB[k] - G_HUM[k]) * t; return g; }
  function jKin(g, x) {
    const f = x[0] * RAD, al = x[1] * RAD, th = x[2] * RAD;
    const footZ0 = g.Hs - g.sag, axZ = footZ0 + g.R;
    const foot = [g.R * Math.sin(f), g.footY, axZ - g.R * Math.cos(f)];
    const hip = [foot[0] + g.L1 * Math.sin(al), g.footY, foot[2] + g.L1 * Math.cos(al)];
    const head = [hip[0] + g.L2 * Math.sin(th), g.footY, hip[2] + g.L2 * Math.cos(th)];
    return { foot, hip, head, al, th };
  }
  function jBuild() {
    jClear();
    const g = jGeom(), k = jKin(g, OVS.pose), glow = OVS.morphT < 0.5;
    const pr = (0.010 + 0.008 * (g.Hs / 2.5)) * OVS.thk;
    jG.add(jrod([0, -g.D / 2, 0], [0, -g.D / 2, g.Hs], pr, 0xdfe6ee, glow));           // 기둥 (밝은 회백)
    jG.add(jrod([0, g.D / 2, 0], [0, g.D / 2, g.Hs], pr, 0xdfe6ee, glow));
    jG.add(jrod([0, -g.D / 2, g.Hs], k.foot, pr * 0.55, 0xff9f1c, glow));                 // 줄 (주황 — 실사 위에서 또렷하게)
    jG.add(jrod(k.foot, [0, g.D / 2, g.Hs], pr * 0.55, 0xff9f1c, glow));
    const jbox = (w, d, h, c) => new THREE.Mesh(new THREE.BoxGeometry(w, d, h), jmat(c, glow));
    const bw = OVS.body;
    const lb = jbox(0.147 * g.L1 * bw, 0.309 * g.L1 * bw, g.L1, 0x06b98c); lb.position.set((k.foot[0] + k.hip[0]) / 2, g.footY, (k.foot[2] + k.hip[2]) / 2); lb.rotation.y = k.al; jG.add(lb);
    const ub = jbox(0.101 * g.L2 * bw, 0.240 * g.L2 * bw, g.L2, 0x7a35d6); ub.position.set((k.hip[0] + k.head[0]) / 2, g.footY, (k.hip[2] + k.head[2]) / 2); ub.rotation.y = k.th; jG.add(ub);
    const hd = jbox(0.101 * g.L2 * bw, 0.261 * g.L2 * bw, 0.123 * g.L2, 0xf2c14e); hd.position.set(k.head[0] + 0.062 * g.L2 * Math.sin(k.th), g.footY, k.head[2] + 0.062 * g.L2 * Math.cos(k.th)); hd.rotation.y = k.th; jG.add(hd);
    const hpr = 0.062 * g.L1 * bw; const hp = new THREE.Mesh(new THREE.CylinderGeometry(hpr, hpr, 0.50 * g.L1 * bw, 14), jmat(0xffbe0b, glow)); hp.position.set(k.hip[0], g.footY, k.hip[2]); jG.add(hp);
    const ft = new THREE.Mesh(new THREE.SphereGeometry(0.05 * g.L1 * bw, 14, 10), jmat(0xe6a817, glow)); ft.position.set(k.foot[0], g.footY, k.foot[2]); jG.add(ft);
  }
  // 영상 배치: 세로 화면에 contain, 캔버스는 사진 띠(y 607~1407) 위에만 — 정합 카메라가 그대로 맞는다
  function layoutVideo() {
    const W = innerWidth, H = innerHeight; let w = W, h = W * VM.vh / VM.vw; if (h > H) { h = H; w = H * VM.vw / VM.vh; }
    const L = (W - w) / 2, T = (H - h) / 2;
    Object.assign(vid.style, { left: L + "px", top: T + "px", width: w + "px", height: h + "px" });
    const bt = T + h * VM.cy / VM.vh, bh = h * VM.ch / VM.vh;
    Object.assign(ovc.style, { left: L + "px", top: bt + "px", width: w + "px", height: bh + "px" });
    const pw = Math.round(w * devicePixelRatio), ph = Math.round(bh * devicePixelRatio);
    if (ovc.width !== pw || ovc.height !== ph) { jr.setSize(pw, ph, false); jcam.aspect = pw / ph; jcam.updateProjectionMatrix(); }
  }
  // ---- 휘청 추적 데이터 (tools/track_jultagi.py → /media/jultagi_trace.json) ----
  const TR = { ok: false, t: [], phi: [], beta: [], loop: [3.6, 5.8], sPhi: 1, sTilt: 1 };
  fetch("/media/jultagi_trace.json").then(r => r.ok ? r.json() : null).then(j => { if (j) buildTrace(j); }).catch(() => {});
  function buildTrace(j) {
    const n = j.n || j.t.length; const fx = j.foot_x, tl = j.tilt_deg;
    const lp = j.loop && isFinite(j.loop.t_a) ? [j.loop.t_a, j.loop.t_b] : TR.loop;
    // 발 x → φ: 줄 span 픽셀이 있으면 m/px 로, 없으면 반복 구간에서 최대 진폭 3° 로 정규화
    let mpp = null; if (j.rope && j.rope.span_px) mpp = G_HUM.D / j.rope.span_px;
    const idx = []; for (let k = 0; k < n; k++) if (j.t[k] >= lp[0] - 0.3 && j.t[k] <= lp[1] + 0.3) idx.push(k);
    const mean = arr => idx.reduce((s, k) => s + arr[k], 0) / Math.max(1, idx.length);
    const fx0 = mean(fx), tl0 = mean(tl);
    let phi = fx.map(v => mpp ? Math.asin(Math.max(-0.99, Math.min(0.99, (v - fx0) * mpp / G_HUM.R))) / RAD : (v - fx0));
    if (!mpp) { const amp = Math.max(1e-6, ...idx.map(k => Math.abs(phi[k]))); phi = phi.map(v => v / amp * 3.0); }
    const beta = tl.map(v => (v - tl0));
    TR.t = j.t; TR.phi = phi; TR.beta = beta; TR.loop = lp; TR.ok = true;
    calibSigns();
    LG.toast(`휘청 추적 ${n} 프레임, 반복 ${lp[0].toFixed(2)}–${lp[1].toFixed(2)} s`);
  }
  // 화면 방향 보정: 모델의 φ>0 · 기울기>0 가 영상에서 발·상체가 움직인 쪽과 같게
  function calibSigns() {
    const g = G_HUM, proj = p => { const v = new THREE.Vector3(p[0], p[1], p[2]).project(jcam); return v.x; };
    jcamUpdate();
    const k1 = jKin(g, [1, 0, 0]), k2 = jKin(g, [-1, 0, 0]);
    TR.sPhi = (proj(k1.foot) - proj(k2.foot)) >= 0 ? 1 : -1;          // 화면 x+ (오른쪽) 로 발이 가는 φ 부호
    const h1 = jKin(g, [0, 1, 1]), h2 = jKin(g, [0, -1, -1]);
    TR.sTilt = (proj(h1.head) - proj(h2.head)) >= 0 ? 1 : -1;
  }
  function tracePose(t) {
    if (!TR.ok) { const w = 2 * Math.PI / 1.15; const phi = 2.2 * Math.sin(w * t), beta = phi / -2.1; return [phi, beta, beta]; }
    const T = TR.t; let lo = 0, hi = T.length - 1; if (t <= T[0]) lo = hi = 0; else if (t >= T[hi]) lo = hi; else { while (hi - lo > 1) { const m = (lo + hi) >> 1; if (T[m] <= t) lo = m; else hi = m; } }
    const u = hi > lo ? (t - T[lo]) / (T[hi] - T[lo]) : 0;
    const phi = (TR.phi[lo] + (TR.phi[hi] - TR.phi[lo]) * u) * TR.sPhi, beta = (TR.beta[lo] + (TR.beta[hi] - TR.beta[lo]) * u) * TR.sTilt;
    return [phi, beta, beta];
  }
  // ---- 모드 ----
  let loopTimer = null, morphAnim = null;
  function stopVideoLoop() { ST.vid.loop = null; }
  function videoMode(a) {
    const mode = a.mode || "still"; ST.vid.mode = mode; ST.vid.overlay = !!a.overlay; ST.vid.opa = a.opa != null ? +a.opa : 1.0;
    if (a.thk) OVS.thk = +a.thk; if (a.body) OVS.body = +a.body;
    el("vidVeil").style.opacity = "0"; ovc.style.opacity = ST.vid.overlay ? "1" : "0"; vid.style.opacity = "1";
    morphAnim = null; OVS.morphT = 0; OVS.cam = Object.assign({}, CAM_J);
    if (mode === "play") { ST.vid.loop = null; try { vid.currentTime = 0; } catch (e) {} vid.play().catch(() => {}); }
    else if (mode === "loop") {
      const lp = a.loop && a.loop.length === 2 ? a.loop : TR.loop; ST.vid.loop = lp;
      if (vid.currentTime < lp[0] || vid.currentTime > lp[1]) { try { vid.currentTime = lp[0]; } catch (e) {} }
      vid.play().catch(() => {});
    }
    else if (mode === "still") { ST.vid.loop = null; vid.pause(); try { vid.currentTime = a.t != null ? +a.t : Math.max(0, (vid.duration || VM.tEnd) - 0.05); } catch (e) {} }
    else if (mode === "morph") {
      ST.vid.loop = null; vid.pause(); ST.vid.overlay = true; ovc.style.opacity = "1";
      morphAnim = { t0: performance.now(), dur: (a.dur || 5.0) * 1000, fade: (a.fade || 1.5) * 1000 };
    }
  }
  const easeIO = t => t < 0.5 ? 2 * t * t : 1 - 2 * (1 - t) * (1 - t);
  function renderVideo() {
    layoutVideo();
    if (ST.vid.loop) { const lp = ST.vid.loop; if (vid.currentTime >= lp[1] || vid.ended) { try { vid.currentTime = lp[0]; } catch (e) {} if (vid.paused) vid.play().catch(() => {}); } }
    if (!ST.vid.overlay) return;
    if (ST.vid.loop) OVS.pose = tracePose(vid.currentTime);
    else if (ST.vid.mode === "morph" && morphAnim) {
      const u = Math.min(1, (performance.now() - morphAnim.t0) / morphAnim.dur), s = easeIO(u);
      OVS.morphT = s; const c = {}; for (const k in CAM_J) c[k] = CAM_J[k] + (CAM_R[k] - CAM_J[k]) * s; OVS.cam = c;
      vid.style.opacity = String(Math.max(0, 1 - (performance.now() - morphAnim.t0) / morphAnim.fade));
      el("vidVeil").style.opacity = String(Math.min(1, (performance.now() - morphAnim.t0) / morphAnim.fade));
      const i = LG.cur(); if (i >= 0) { const p = LG.poseAt(i); OVS.pose = [p.phi, p.alpha, p.theta]; } else OVS.pose = tracePose(vid.currentTime);
    } else { const i = LG.cur(); if (i >= 0 && LG.link.connected) { const p = LG.poseAt(i); OVS.pose = [p.phi, p.alpha, p.theta]; } else OVS.pose = tracePose(vid.currentTime); }
    jcamUpdate(); jBuild(); jr.render(jscene, jcam);
  }

  // ================= 키 (비상용) =================
  document.addEventListener("keydown", e => {
    if (e.key === "f" || e.key === "F") { if (document.fullscreenElement) document.exitFullscreen(); else document.documentElement.requestFullscreen().catch(() => {}); }
    else if (e.key === "c" || e.key === "C") document.body.classList.toggle("cursor");
    else if (e.key === "b" || e.key === "B") applyScene({ name: "black", args: {} });
    else if (e.key === "m" || e.key === "M") { ST.mirror = !ST.mirror; LG.store.set("show.mirror", ST.mirror); if (ST.scene) applyScene(ST.scene); }
  });
  window.addEventListener("resize", fitPlanes);

  // ================= 프레임 루프 =================
  function frame() {
    requestAnimationFrame(frame);
    if (ST.view === "video") renderVideo();
    else if (ST.view === "twin" || ST.view === "plane") { fitPlanes(); LG.render3d(); LG.renderPlanes(); renderHud(); }
  }
  fitPlanes(); frame();
  LG.connect();
})();
