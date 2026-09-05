// lg_lab.js — 측정실: 로거 앱 전체 기능(lg_panels.js 가 그대로 돈다) 위에 엑셀식 원본 표 · 시간 차트 · XY 차트 · 추세선을 얹는다.
// 상태는 로거와 공유한다: 창 = LG.chart.win/view (스트립 차트와 같은 시간 창), 선택 = LG.sel, 커서 = LG.setCursor, 따라가기 = LG.follow.
// 동작 셋: 열 고르기 → 구간 드래그(왼쪽 버튼) → 추세선. 선형·지수는 이 자리에서, 감쇠사인·놓기점·추천은 서버 도구로.
"use strict";
(function () {
  const el = LG.el, fmt = LG.fmt;
  const COLS = [
    { k: "t", l: "t [s]", d: 3, col: "#555" }, { k: "u_phi", l: "φ", d: 3, col: "#1f6fe5" }, { k: "u_ank", l: "ank", d: 3, col: "#0f9d58" },
    { k: "del", l: "δ", d: 2, col: "#b8860b" }, { k: "a_alpha", l: "α", d: 3, col: "#7b3fe4" }, { k: "a_beta", l: "β", d: 3, col: "#e8590c" },
    { k: "a_Ahat", l: "Â", d: 3, col: "#d6336c" }, { k: "a_dphi", l: "φ̇", d: 1, col: "#5c7cfa" }, { k: "a_dbeta", l: "β̇", d: 1, col: "#f08c00" },
    { k: "hold", l: "hold", d: 2, col: "#2b8a3e" }, { k: "phase", l: "phase", d: 0, col: "#868e96" }, { k: "a_theta", l: "θ", d: 3, col: "#9c36b5" },
    { k: "a_fp", l: "φ_pred", d: 3, col: "#1098ad" }, { k: "a_bp", l: "β_pred", d: 3, col: "#e67700" }, { k: "a_psi", l: "ψ", d: 3, col: "#0b7285" },
    { k: "Ahat_fw", l: "Â(펌)", d: 3, col: "#c2255c" }, { k: "beta_fw", l: "β(펌)", d: 3, col: "#d9480f" },
  ];
  const COL = Object.fromEntries(COLS.map(c => [c.k, c]));
  const FIT_COLORS = ["#c92a2a", "#1864ab", "#087f5b", "#5f3dc4", "#d9480f", "#0b7285", "#a61e4d"];
  const tableOn = LG.store.get("lab.cols", { t: 1, u_phi: 1, u_ank: 1, del: 1, a_alpha: 1, a_beta: 1, a_Ahat: 1 });
  const tcOn = LG.store.get("lab.tc", { u_phi: 1, u_ank: 1, del: 1, a_beta: 1, a_Ahat: 0 });
  const S = { tcFits: [], xyFits: [], xyPts: null, anchor: -1, hover: null, reqs: {}, reqId: 0 };

  // ================= 공통 =================
  function niceStep(span, n) { const raw = span / Math.max(1, n); const p = Math.pow(10, Math.floor(Math.log10(raw))); const m = raw / p; return (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * p; }
  function decimals(step) { return step >= 1 ? 0 : step >= 0.1 ? 1 : step >= 0.01 ? 2 : 3; }
  function idxRange(t0, t1) { const n = LG.ds.n; if (!n) return [0, -1]; return [Math.max(0, LG.idxOfT(t0)), Math.min(n - 1, LG.idxOfT(t1) + 1)]; }
  function selIdx() { if (LG.sel) return idxRange(LG.sel.t0, LG.sel.t1); const [a, b] = LG.chartRange(); return idxRange(a, b); }
  function setSel(s) { LG.sel = s; LG.emit("sel"); }
  function freeze(v0) { if (LG.follow) { LG.follow = false; LG.playing = false; LG.chart.view = { t0: v0[0], t1: v0[1] }; LG.emit("follow"); } }
  function analyze(tool, args, cb) { const id = "lab" + (++S.reqId); S.reqs[id] = cb; LG.send({ cmd: "analyze", tool, args, req: id }); }
  LG.on("analysis", res => { const cb = S.reqs[res.req]; if (cb) { delete S.reqs[res.req]; cb(res); } });
  function linreg(x, y) {
    const n = x.length; if (n < 2) return null;
    let sx = 0, sy = 0; for (let i = 0; i < n; i++) { sx += x[i]; sy += y[i]; }
    const mx = sx / n, my = sy / n; let sxx = 0, sxy = 0, syy = 0;
    for (let i = 0; i < n; i++) { const dx = x[i] - mx, dy = y[i] - my; sxx += dx * dx; sxy += dx * dy; syy += dy * dy; }
    if (sxx < 1e-12) return null;
    const b = sxy / sxx, a = my - b * mx; let sse = 0;
    for (let i = 0; i < n; i++) { const r = y[i] - (a + b * x[i]); sse += r * r; }
    const r2 = syy > 0 ? 1 - sse / syy : NaN, se_b = n > 2 ? Math.sqrt(sse / (n - 2) / sxx) : NaN;
    return { a, b, r2, n, se_b };
  }
  // 창 길이: 트랜스포트의 iWin 과 엑셀 차트의 iXWin 이 같은 LG.chart.win 을 본다
  function setWin(w) { LG.chart.win = Math.max(0.2, Math.min(3600, w)); LG.store.set("chartWin", LG.chart.win); el("iWin").value = +LG.chart.win.toFixed(2); el("iXWin").value = +LG.chart.win.toFixed(2); }
  el("iXWin").value = +LG.chart.win.toFixed(2);
  el("iXWin").onchange = () => setWin(+el("iXWin").value || 10);
  el("iWin").addEventListener("change", () => { el("iXWin").value = +LG.chart.win.toFixed(2); });

  // ================= 열 선택 =================
  function buildColBox() {
    const box = el("colBox"); box.innerHTML = "";
    for (const c of COLS) { const lab = document.createElement("label"); lab.className = "ck"; const i = document.createElement("input"); i.type = "checkbox"; i.checked = !!tableOn[c.k]; i.onchange = () => { tableOn[c.k] = i.checked ? 1 : 0; LG.store.set("lab.cols", tableOn); buildTableHeader(); }; lab.appendChild(i); lab.appendChild(document.createTextNode(c.l)); box.appendChild(lab); }
    const ts = el("tcSeries"); ts.innerHTML = "";
    for (const c of COLS) { if (c.k === "t") continue; const lab = document.createElement("label"); lab.className = "ck"; const i = document.createElement("input"); i.type = "checkbox"; i.checked = !!tcOn[c.k]; i.onchange = () => { tcOn[c.k] = i.checked ? 1 : 0; LG.store.set("lab.tc", tcOn); }; const sw = document.createElement("span"); sw.className = "sw"; sw.style.background = c.col; lab.appendChild(i); lab.appendChild(sw); lab.appendChild(document.createTextNode(c.l)); ts.appendChild(lab); }
    for (const id of ["tcCol", "xyX", "xyY"]) { const s = el(id); s.innerHTML = ""; for (const c of COLS) { if (id === "tcCol" && c.k === "t") continue; const o = document.createElement("option"); o.value = c.k; o.textContent = c.l; s.appendChild(o); } }
    el("tcCol").value = "u_phi"; el("xyX").value = "del"; el("xyY").value = "a_alpha";
  }
  buildColBox();
  el("tcFit").onchange = () => { el("tcY0Wrap").style.display = el("tcFit").value === "exp" ? "" : "none"; };
  el("tcFit").onchange();

  // ================= 원본 표 (가상 스크롤) =================
  const ROW_H = 19; let tableCols = [];
  const tbl = el("tbl"), thdr = el("thdr"), tbody = el("tbody");
  function buildTableHeader() {
    tableCols = COLS.filter(c => tableOn[c.k]);
    const gt = tableCols.map(c => c.k === "t" ? "72px" : "minmax(56px,1fr)").join(" ");
    thdr.style.gridTemplateColumns = gt; thdr.innerHTML = tableCols.map(c => `<div>${c.l}</div>`).join("");
    tbody.innerHTML = ""; tbody._rows = []; tbody._gt = gt;
  }
  buildTableHeader();
  function renderTable() {
    const n = LG.ds.n; tbody.style.height = (n * ROW_H) + "px";
    const top = tbl.scrollTop, h = tbl.clientHeight;
    const i0 = Math.max(0, Math.floor(top / ROW_H) - 2), i1 = Math.min(n - 1, Math.ceil((top + h) / ROW_H) + 2);
    const rows = tbody._rows, need = Math.max(0, i1 - i0 + 1);
    while (rows.length < need) {
      const d = document.createElement("div"); d.className = "row2"; d.style.gridTemplateColumns = tbody._gt; d.innerHTML = tableCols.map(() => "<div></div>").join("");
      d.onclick = e => { const i = +d.dataset.i; if (e.shiftKey && S.anchor >= 0) { const a = Math.min(S.anchor, i), b = Math.max(S.anchor, i); setSel({ t0: LG.tOf(a), t1: LG.tOf(b) }); } else { S.anchor = i; LG.setCursor(i); } };
      tbody.appendChild(d); rows.push(d);
    }
    while (rows.length > need) rows.pop().remove();
    const [s0, s1] = LG.sel ? idxRange(LG.sel.t0, LG.sel.t1) : [1, 0];
    const cur = LG.cur();
    for (let k = 0; k < need; k++) {
      const i = i0 + k, d = rows[k]; d.dataset.i = i; d.style.top = (i * ROW_H) + "px";
      d.className = "row2" + (i >= s0 && i <= s1 ? " sel" : "") + (i === cur ? " cur" : "");
      const cells = d.children;
      for (let c = 0; c < tableCols.length; c++) { const col = tableCols[c]; const v = LG.val(col.k, i); cells[c].textContent = isFinite(v) ? v.toFixed(col.d) : "—"; }
    }
  }
  let lastScrollCur = -1;
  function tableFollowCursor() {
    const i = LG.cur(); if (i < 0 || i === lastScrollCur) return; lastScrollCur = i;
    const top = i * ROW_H, h = tbl.clientHeight;
    if (LG.follow) tbl.scrollTop = Math.max(0, top - h + ROW_H * 2);
    else if (top < tbl.scrollTop || top > tbl.scrollTop + h - ROW_H) tbl.scrollTop = Math.max(0, top - h / 2);
  }
  // 복사 (TSV — 엑셀에 그대로 붙는다)
  function copyRows(all) {
    const n = LG.ds.n; if (!n) { LG.toast("데이터 없음"); return; }
    const [a, b] = all ? [0, n - 1] : selIdx();
    const cols = tableCols.length ? tableCols : COLS.slice(0, 7);
    const lines = [cols.map(c => c.l).join("\t")];
    const step = Math.max(1, Math.floor((b - a + 1) / 200000));
    for (let i = a; i <= b; i += step) lines.push(cols.map(c => { const v = LG.val(c.k, i); return isFinite(v) ? v.toFixed(c.d) : ""; }).join("\t"));
    const text = lines.join("\n");
    const done = () => LG.toast(`${lines.length - 1} 행 × ${cols.length} 열 복사됨 — 엑셀에 붙여넣기`);
    if (navigator.clipboard && navigator.clipboard.writeText) navigator.clipboard.writeText(text).then(done, () => showText(text));
    else showText(text);
  }
  function showText(text) { el("dlgMsg").textContent = "클립보드에 못 넣었다 — 아래를 전체 선택해 복사"; el("dlgText").value = text; el("dlg").showModal(); el("dlgText").select(); }
  el("bCopy").onclick = () => copyRows(false); el("bCopyAll").onclick = () => copyRows(true);

  // ================= 시간 차트 (엑셀식, 스트립 차트와 같은 창) =================
  const tc = el("tc"), tctx = tc.getContext("2d");
  let TC = { X: null, Y: null, L: 0, T: 0, PW: 1, PH: 1, t0: 0, t1: 1 };
  function drawAxes(ctx, L, T, W, H, x0, x1, y0, y1, xlab, ylab, DPR) {
    ctx.fillStyle = "#fff"; ctx.fillRect(L, T, W, H);
    const sx = niceStep(x1 - x0, Math.max(3, W / (70 * DPR))), sy = niceStep(y1 - y0, Math.max(3, H / (40 * DPR)));
    const X = x => L + (x - x0) / (x1 - x0) * W, Y = y => T + H - (y - y0) / (y1 - y0) * H;
    ctx.strokeStyle = "#e6eaf0"; ctx.lineWidth = 1 * DPR; ctx.font = `${10.5 * DPR}px Consolas, monospace`; ctx.fillStyle = "#5c6675";
    const dx = decimals(sx), dy = decimals(sy);
    for (let v = Math.ceil(x0 / sx) * sx; v <= x1 + 1e-9; v += sx) { const x = X(v); ctx.beginPath(); ctx.moveTo(x, T); ctx.lineTo(x, T + H); ctx.stroke(); ctx.textAlign = "center"; ctx.fillText(v.toFixed(dx), x, T + H + 13 * DPR); }
    for (let v = Math.ceil(y0 / sy) * sy; v <= y1 + 1e-9; v += sy) { const y = Y(v); ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(L + W, y); ctx.stroke(); ctx.textAlign = "right"; ctx.fillText(v.toFixed(dy), L - 4 * DPR, y + 4 * DPR); }
    if (y0 < 0 && y1 > 0) { ctx.strokeStyle = "#9aa3b2"; ctx.beginPath(); ctx.moveTo(L, Y(0)); ctx.lineTo(L + W, Y(0)); ctx.stroke(); }
    if (x0 < 0 && x1 > 0) { ctx.strokeStyle = "#9aa3b2"; ctx.beginPath(); ctx.moveTo(X(0), T); ctx.lineTo(X(0), T + H); ctx.stroke(); }
    ctx.strokeStyle = "#b9c1cc"; ctx.strokeRect(L, T, W, H);
    ctx.fillStyle = "#3a4657"; ctx.textAlign = "left"; ctx.font = `${11 * DPR}px "Segoe UI", "Malgun Gothic", sans-serif`;
    if (xlab) ctx.fillText(xlab, L + W - ctx.measureText(xlab).width - 4 * DPR, T + H - 5 * DPR);
    if (ylab) ctx.fillText(ylab, L + 5 * DPR, T + 13 * DPR);
    return { X, Y };
  }
  function drawSeries(ctx, X, Y, tA, vA, i0, i1, Wpx, DPR, step) {
    const n = i1 - i0 + 1; if (n <= 0) return;
    ctx.beginPath();
    if (n > 2.5 * Wpx) {
      let px = -1, mn = 0, mx = 0;
      for (let i = i0; i <= i1; i++) { const v = vA[i]; if (!isFinite(v)) continue; const x = X(tA[i]) | 0; if (x !== px) { if (px >= 0) { ctx.moveTo(px, Y(mn)); ctx.lineTo(px, Y(mx)); } px = x; mn = mx = v; } else { if (v < mn) mn = v; if (v > mx) mx = v; } }
      if (px >= 0) { ctx.moveTo(px, Y(mn)); ctx.lineTo(px, Y(mx)); }
    } else {
      let first = true, py = 0;
      for (let i = i0; i <= i1; i++) { const v = vA[i]; if (!isFinite(v)) { first = true; continue; } const x = X(tA[i]), y = Y(v); if (first) { ctx.moveTo(x, y); first = false; } else { if (step) ctx.lineTo(x, py); ctx.lineTo(x, y); } py = y; }
    }
    ctx.stroke();
  }
  function polyline(ctx, X, Y, ts, ys) { ctx.beginPath(); let first = true; for (let k = 0; k < ts.length; k++) { const y = ys[k]; if (y == null || !isFinite(y)) { first = true; continue; } if (first) { ctx.moveTo(X(ts[k]), Y(y)); first = false; } else ctx.lineTo(X(ts[k]), Y(y)); } ctx.stroke(); }
  function renderTC() {
    const DPR = devicePixelRatio, W = tc.width = tc.clientWidth * DPR, H = tc.height = tc.clientHeight * DPR; if (!W || !H) return;
    const ctx = tctx; ctx.clearRect(0, 0, W, H); ctx.fillStyle = "#fafbfd"; ctx.fillRect(0, 0, W, H);
    const L = 52 * DPR, R = 12 * DPR, T = 8 * DPR, B = 22 * DPR, PW = W - L - R, PH = H - T - B;
    const n = LG.ds.n, tA = LG.ds.data.t;
    const [t0, t1] = LG.chartRange();
    let ymin = Infinity, ymax = -Infinity;
    const [i0, i1] = n ? idxRange(t0, t1) : [0, -1];
    const series = COLS.filter(c => c.k !== "t" && tcOn[c.k]);
    for (const c of series) { const a = LG.col(c.k); if (!a) continue; const st = Math.max(1, Math.floor((i1 - i0) / 4000)); for (let i = i0; i <= i1; i += st) { const v = a[i]; if (!isFinite(v)) continue; if (v < ymin) ymin = v; if (v > ymax) ymax = v; } }
    for (const f of S.tcFits) if (f.curve) for (const v of f.curve.y) { if (isFinite(v)) { if (v < ymin) ymin = v; if (v > ymax) ymax = v; } }
    if (!isFinite(ymin)) { ymin = -1; ymax = 1; }
    const pad = Math.max((ymax - ymin) * 0.08, 0.2); ymin -= pad; ymax += pad;
    const { X, Y } = drawAxes(ctx, L, T, PW, PH, t0, t1, ymin, ymax, "t [s]", "[°]", DPR);
    TC = { X, Y, L, T, PW, PH, t0, t1 };
    ctx.save(); ctx.beginPath(); ctx.rect(L, T, PW, PH); ctx.clip();
    // phase 음영 (접기 분홍 · 발산 주황) — 스트립 차트와 같은 뜻
    if (n && i1 >= i0) { const ph = LG.col("phase"); if (ph) { let a = -1, pv = 0; for (let i = i0; i <= i1 + 1; i++) { const v = i <= i1 ? (ph[i] | 0) : -2; if (v !== pv) { if (a >= 0 && (pv === 1 || pv === 5)) { ctx.fillStyle = pv === 1 ? "rgba(255,0,110,.10)" : "rgba(255,159,67,.13)"; ctx.fillRect(X(tA[a]), T, Math.max(1, X(tA[Math.min(i, i1)]) - X(tA[a])), PH); } a = i; pv = v; } } } }
    // 분석 오버레이 (측정 도구가 돌려준 구간·점·적합선) — 켜져 있으면 여기에도 겹친다
    const OV = LG.showOverlay ? (LG.chartOverlay || []) : [];
    for (const o of OV) if (o.kind === "band") { ctx.fillStyle = (o.color || "rgba(0,0,0,.08)").replace(/rgba\(([^)]+),\s*[\d.]+\)/, "rgba($1,.16)"); ctx.fillRect(X(o.t0), T, Math.max(1, X(o.t1) - X(o.t0)), PH); }
    // 선택 구간
    if (LG.sel) { ctx.fillStyle = "rgba(31,111,229,.10)"; ctx.fillRect(X(LG.sel.t0), T, X(LG.sel.t1) - X(LG.sel.t0), PH); ctx.strokeStyle = "rgba(31,111,229,.6)"; ctx.setLineDash([4 * DPR, 3 * DPR]); ctx.beginPath(); ctx.moveTo(X(LG.sel.t0), T); ctx.lineTo(X(LG.sel.t0), T + PH); ctx.moveTo(X(LG.sel.t1), T); ctx.lineTo(X(LG.sel.t1), T + PH); ctx.stroke(); ctx.setLineDash([]); }
    // 이벤트 (MARK · RELEASE · FOLD …)
    let lastLx = -1e9, lrow = 0;
    for (const e of (LG.aux.events || [])) { const te = LG.tOfMs(e[0]); if (!isFinite(te) || te < t0 || te > t1) continue; const xe = X(te); ctx.strokeStyle = "rgba(0,0,0,.18)"; ctx.setLineDash([2 * DPR, 3 * DPR]); ctx.beginPath(); ctx.moveTo(xe, T); ctx.lineTo(xe, T + PH); ctx.stroke(); ctx.setLineDash([]);
      lrow = (xe - lastLx < 40 * DPR) ? (lrow + 1) % 3 : 0; lastLx = xe;                       // 촘촘한 이벤트(FOLD 연속)는 세 줄로 어긋나게
      ctx.fillStyle = "#7a5c00"; ctx.font = `${10 * DPR}px Consolas`; ctx.textAlign = "left"; ctx.fillText(e[1] + (e[2] ? " " + e[2] : ""), xe + 2 * DPR, T + (12 + 11 * lrow) * DPR); }
    // 시리즈
    for (const c of series) { const a = LG.col(c.k); if (!a || i1 < i0) continue; ctx.strokeStyle = c.col; ctx.lineWidth = 1.6 * DPR; drawSeries(ctx, X, Y, tA, a, i0, i1, PW, DPR, c.k === "phase"); }
    // 도구 오버레이: 적합선 · 점 · 수평선 · 수직선
    for (const o of OV) {
      if (o.kind === "line") { ctx.strokeStyle = o.color || "#d6336c"; ctx.lineWidth = 2 * DPR; polyline(ctx, X, Y, o.t, o.y); }
      else if (o.kind === "points") { ctx.fillStyle = o.color || "#d6336c"; for (let k = 0; k < o.t.length; k++) { if (o.y[k] == null) continue; ctx.beginPath(); ctx.arc(X(o.t[k]), Y(o.y[k]), 2.2 * DPR, 0, 7); ctx.fill(); } }
      else if (o.kind === "hline") { ctx.strokeStyle = o.color || "#9aa3b2"; ctx.lineWidth = 1 * DPR; ctx.setLineDash([3 * DPR, 3 * DPR]); ctx.beginPath(); ctx.moveTo(L, Y(o.y)); ctx.lineTo(L + PW, Y(o.y)); ctx.stroke(); ctx.setLineDash([]); if (o.label) { ctx.fillStyle = o.color || "#3a4657"; ctx.font = `${9.5 * DPR}px Consolas`; ctx.textAlign = "right"; ctx.fillText(o.label, L + PW - 4 * DPR, Y(o.y) - 2 * DPR); } }
      else if (o.kind === "vline" && o.t >= t0 && o.t <= t1) { ctx.strokeStyle = o.color || "#3a4657"; ctx.lineWidth = 1 * DPR; ctx.beginPath(); ctx.moveTo(X(o.t), T); ctx.lineTo(X(o.t), T + PH); ctx.stroke(); if (o.label) { ctx.fillStyle = "#3a4657"; ctx.font = `${9.5 * DPR}px Consolas`; ctx.textAlign = "left"; ctx.fillText(o.label, X(o.t) + 2 * DPR, T + PH - 6 * DPR); } }
    }
    // 추세선 (점선)
    for (const f of S.tcFits) if (f.curve) { ctx.strokeStyle = f.color; ctx.lineWidth = 2.2 * DPR; ctx.setLineDash([7 * DPR, 4 * DPR]); polyline(ctx, X, Y, f.curve.t, f.curve.y); ctx.setLineDash([]); }
    // 커서
    if (n) { const cur = LG.cur(); const xc = X(tA[cur]); ctx.strokeStyle = "#c98a00"; ctx.lineWidth = 1.2 * DPR; ctx.beginPath(); ctx.moveTo(xc, T); ctx.lineTo(xc, T + PH); ctx.stroke(); }
    if (S.hover) { ctx.fillStyle = "#3a4657"; ctx.font = `${10.5 * DPR}px Consolas`; ctx.textAlign = "left"; ctx.fillText(`t=${S.hover.t.toFixed(3)}`, X(S.hover.t) + 4 * DPR, T + PH - 6 * DPR); }
    ctx.restore();
    // 범례 (현재값)
    let lx = L + 8 * DPR; ctx.font = `${11 * DPR}px Consolas`; ctx.textAlign = "left";
    const cur = LG.cur();
    for (const c of series) { const v = LG.val(c.k, cur); const txt = `${c.l} ${isFinite(v) ? v.toFixed(c.d) : "—"}`; ctx.fillStyle = c.col; ctx.fillText(txt, lx, T + 26 * DPR); lx += ctx.measureText(txt).width + 12 * DPR; }
    // 추세선 식 (차트 위 상자)
    let ly = T + 42 * DPR; ctx.font = `${11 * DPR}px Consolas`;
    for (const f of S.tcFits) { ctx.fillStyle = f.color; ctx.fillText(f.eq, L + 8 * DPR, ly); ly += 14 * DPR; }
    if (!LG.follow) { ctx.fillStyle = "#c98a00"; ctx.textAlign = "right"; ctx.fillText("REPLAY — 더블클릭 = 따라가기", L + PW - 6 * DPR, T + 13 * DPR); }
  }
  // 조작: 좌드래그 = 구간 선택 · 우드래그 = 이동 · 휠 = 시간 줌 · 클릭 = 커서 · 더블클릭 = 따라가기
  const tAt = e => { const r = tc.getBoundingClientRect(); const px = (e.clientX - r.left) * devicePixelRatio; return TC.t0 + (px - TC.L) / TC.PW * (TC.t1 - TC.t0); };
  let drag = null;
  tc.addEventListener("mousedown", e => { const t = tAt(e); drag = { btn: e.button, x: e.clientX, t, v0: [TC.t0, TC.t1], moved: false }; if (e.button === 0) LG.sel = { t0: t, t1: t }; });
  window.addEventListener("mousemove", e => {
    S.hover = tc.matches(":hover") && LG.ds.n ? { t: tAt(e) } : null;
    if (!drag) return; const t = tAt(e); if (Math.abs(e.clientX - drag.x) > 3) drag.moved = true;
    if (drag.btn === 0) { if (drag.moved) { freeze(drag.v0); LG.sel = { t0: Math.min(drag.t, t), t1: Math.max(drag.t, t) }; LG.emit("sel"); } }
    else if (drag.moved) { const span = drag.v0[1] - drag.v0[0]; const dt = (e.clientX - drag.x) * devicePixelRatio / TC.PW * span; freeze(drag.v0); LG.chart.view = { t0: drag.v0[0] - dt, t1: drag.v0[1] - dt }; }
  });
  window.addEventListener("mouseup", () => {
    if (!drag) return;
    if (drag.btn === 0 && !drag.moved && LG.ds.n) { LG.sel = null; LG.setCursor(LG.idxOfT(drag.t)); }
    if (LG.sel && LG.sel.t1 - LG.sel.t0 < 0.02) LG.sel = null;
    drag = null; LG.emit("sel");
  });
  tc.addEventListener("wheel", e => { e.preventDefault(); const t = tAt(e); const k = e.deltaY > 0 ? 1.2 : 1 / 1.2; if (LG.follow) setWin(LG.chart.win * k); else LG.chart.view = { t0: t + (TC.t0 - t) * k, t1: t + (TC.t1 - t) * k }; }, { passive: false });
  tc.addEventListener("dblclick", () => { el("bFollow").click(); });
  tc.addEventListener("contextmenu", e => e.preventDefault());
  function updateSelInfo() { el("fSel").textContent = LG.sel ? `선택 ${LG.sel.t0.toFixed(3)} – ${LG.sel.t1.toFixed(3)} s  (${(LG.sel.t1 - LG.sel.t0).toFixed(2)} s, ${(() => { const [a, b] = idxRange(LG.sel.t0, LG.sel.t1); return b - a + 1; })()} 행)` : ""; }
  LG.on("sel", updateSelInfo);

  // ---- 시간 추세선 ----
  function fitLinear(col, i0, i1) {
    const t = LG.ds.data.t, a = LG.col(col); if (!a) return null; const xs = [], ys = [];
    for (let i = i0; i <= i1; i++) { const v = a[i]; if (isFinite(v)) { xs.push(t[i]); ys.push(v); } }
    const lr = linreg(xs, ys); if (!lr) return null;
    const c = COL[col];
    return { eq: `${c.l} = ${lr.b.toFixed(4)}·t ${lr.a >= 0 ? "+" : "−"} ${Math.abs(lr.a).toFixed(3)}   R²=${lr.r2.toFixed(4)}  n=${lr.n}  [${t[i0].toFixed(2)}–${t[i1].toFixed(2)} s]`,
             curve: { t: [t[i0], t[i1]], y: [lr.a + lr.b * t[i0], lr.a + lr.b * t[i1]] }, res: lr };
  }
  function fitExp(col, i0, i1, y0) {
    const t = LG.ds.data.t, a = LG.col(col); if (!a) return null; const xs = [], zs = []; let sgn = 0; const tb = t[i0];
    for (let i = i0; i <= i1; i++) { const v = a[i] - y0; if (isFinite(v) && Math.abs(v) >= 0.02) { xs.push(t[i] - tb); zs.push(Math.log(Math.abs(v))); sgn += v; } }
    const lr = linreg(xs, zs); if (!lr) return null;
    const lam = lr.b, A = Math.exp(lr.a), s = sgn >= 0 ? 1 : -1, c = COL[col];
    const ts = [], ys = []; for (let k = 0; k <= 60; k++) { const tt = t[i0] + (t[i1] - t[i0]) * k / 60; ts.push(tt); ys.push(y0 + s * A * Math.exp(lam * (tt - tb))); }
    return { eq: `|${c.l}−${y0}| = ${A.toFixed(3)}·e^(${lam.toFixed(3)}·(t−${tb.toFixed(2)}))   λ=${lam.toFixed(3)} /s  T₂=${(1000 * Math.LN2 / Math.abs(lam)).toFixed(0)} ms  R²=${lr.r2.toFixed(4)}  n=${lr.n}  (ln|y−y₀| 직선적합, |y−y₀|≥0.02°)`,
             curve: { t: ts, y: ys }, res: { lam, A, r2: lr.r2, n: lr.n, se: lr.se_b } };
  }
  function addTcFit() {
    const n = LG.ds.n; if (!n) { LG.toast("데이터 없음"); return; }
    if (!LG.sel) { LG.toast("차트에서 구간을 드래그해 고르세요 (엑셀에서 셀 범위 고르듯)", true); return; }
    const [i0, i1] = idxRange(LG.sel.t0, LG.sel.t1); if (i1 - i0 < 3) { LG.toast("구간이 너무 짧다", true); return; }
    const type = el("tcFit").value, col = el("tcCol").value, color = FIT_COLORS[S.tcFits.length % FIT_COLORS.length];
    if (!tcOn[col]) { tcOn[col] = 1; LG.store.set("lab.tc", tcOn); buildColBox(); el("tcCol").value = col; }
    if (type === "lin") { const f = fitLinear(col, i0, i1); if (!f) { LG.toast("적합 실패", true); return; } S.tcFits.push({ type, col, color, ...f }); }
    else if (type === "exp") { const y0 = +el("tcY0").value || 0; const f = fitExp(col, i0, i1, y0); if (!f) { LG.toast("적합 실패 (|y−y₀| 가 0.02° 이상인 점이 2개 이상이어야)", true); return; } S.tcFits.push({ type, col, color, ...f }); }
    else if (type === "osc") {
      const t = LG.ds.data.t; const entry = { type, col, color, eq: "감쇠 사인 적합 중…", curve: null }; S.tcFits.push(entry);
      analyze("osc", { t0: t[i0], t1: t[i1], ch: col, refine: true }, res => {
        if (!res.ok) { entry.eq = "감쇠 사인 실패: " + (res.msg || ""); renderFits(); return; }
        const r = res.result, rf = r.refine || {}; const wn = rf.omega_n ?? r.omega_n, ze = rf.zeta ?? r.zeta, wd = rf.omega_d ?? r.omega_d;
        entry.eq = `${COL[col].l} = A·e^(−σt)·cos(ω_d t+φ)+c   ω_n=${fmt(wn, 3)} rad/s  ζ=${fmt(ze, 4)}  T=${wd ? (2 * Math.PI / wd).toFixed(3) : "—"} s  R²=${fmt(rf.r2 ?? r.r2, 4)}  n=${res.n}` + (rf.c_phi_est != null ? `  c_φ≈${(+rf.c_phi_est).toExponential(2)}` : "");
        const ln = (res.overlay || []).find(o => o.kind === "line"); if (ln) entry.curve = { t: ln.t, y: ln.y };
        entry.res = r; renderFits();
      });
    }
    renderFits();
  }
  function renderFits() {
    const box = el("fits"); box.innerHTML = "";
    S.tcFits.forEach((f, k) => { const d = document.createElement("div"); d.className = "fit"; d.innerHTML = `<span class="sw" style="background:${f.color}"></span><span class="eq" title="${f.eq}">${f.eq}</span><button class="btn sm">삭제</button>`; d.querySelector("button").onclick = () => { S.tcFits.splice(k, 1); renderFits(); }; box.appendChild(d); });
    if (!S.tcFits.length) box.innerHTML = `<span class="hint">추세선 없음 — 구간을 드래그하고 「추가」. 지수 = λ (ln|y−y₀| 기울기), 감쇠 사인 = ω·ζ (서버 osc 도구)</span>`;
  }
  el("bTcFit").onclick = addTcFit; el("bTcClear").onclick = () => { S.tcFits = []; renderFits(); }; renderFits();

  // ================= XY 차트 =================
  const xy = el("xy"), xctx = xy.getContext("2d");
  function xyPoints() {
    const xk = el("xyX").value, yk = el("xyY").value, ax = LG.col(xk), ay = LG.col(yk); const n = LG.ds.n;
    const out = { sel: [], all: [] }; if (!ax || !ay || !n) return out;
    const [a, b] = selIdx();
    const st = Math.max(1, Math.floor((b - a + 1) / 6000));
    for (let i = a; i <= b; i += st) { const x = ax[i], y = ay[i]; if (isFinite(x) && isFinite(y)) out.sel.push([x, y]); }
    if (el("cXyAll").checked) { const st2 = Math.max(1, Math.floor(n / 6000)); for (let i = 0; i < n; i += st2) { const x = ax[i], y = ay[i]; if (isFinite(x) && isFinite(y)) out.all.push([x, y]); } }
    return out;
  }
  function renderXY() {
    const DPR = devicePixelRatio, W = xy.width = xy.clientWidth * DPR, H = xy.height = xy.clientHeight * DPR; if (!W || !H) return;
    const ctx = xctx; ctx.clearRect(0, 0, W, H); ctx.fillStyle = "#fafbfd"; ctx.fillRect(0, 0, W, H);
    const L = 52 * DPR, R = 12 * DPR, T = 8 * DPR, B = 22 * DPR, PW = W - L - R, PH = H - T - B;
    const P = S.xyPts || xyPoints();
    const xs = [], ys = [];
    for (const p of P.sel) { xs.push(p[0]); ys.push(p[1]); } for (const p of P.all) { xs.push(p[0]); ys.push(p[1]); }
    for (const f of S.xyFits) { if (f.pts) for (const p of f.pts) { xs.push(p[0]); ys.push(p[1]); } if (f.next) { xs.push(f.next[0]); ys.push(f.next[1]); } }
    let x0 = Math.min(...xs, -0.5), x1 = Math.max(...xs, 0.5), y0 = Math.min(...ys, -0.5), y1 = Math.max(...ys, 0.5);
    if (!xs.length) { x0 = -1; x1 = 1; y0 = -1; y1 = 1; }
    const px = Math.max((x1 - x0) * 0.08, 0.1), py = Math.max((y1 - y0) * 0.08, 0.1); x0 -= px; x1 += px; y0 -= py; y1 += py;
    const xl = COL[el("xyX").value] ? COL[el("xyX").value].l : el("xyX").value, yl = COL[el("xyY").value] ? COL[el("xyY").value].l : el("xyY").value;
    const { X, Y } = drawAxes(ctx, L, T, PW, PH, x0, x1, y0, y1, S.xyPts ? "β₀ [°]" : xl, S.xyPts ? "φ₀ [°]" : yl, DPR);
    ctx.save(); ctx.beginPath(); ctx.rect(L, T, PW, PH); ctx.clip();
    ctx.fillStyle = "rgba(120,130,150,.25)"; for (const p of P.all) { ctx.beginPath(); ctx.arc(X(p[0]), Y(p[1]), 1.6 * DPR, 0, 7); ctx.fill(); }
    if (!S.xyPts) { ctx.fillStyle = "#1f6fe5"; for (const p of P.sel) { ctx.beginPath(); ctx.arc(X(p[0]), Y(p[1]), 2.2 * DPR, 0, 7); ctx.fill(); } }
    for (const f of S.xyFits) {
      if (f.pts) for (const p of f.pts) { ctx.fillStyle = p[2] > 0 ? "#d6336c" : (p[2] < 0 ? "#1f6fe5" : "#868e96"); ctx.beginPath(); ctx.arc(X(p[0]), Y(p[1]), 4.5 * DPR, 0, 7); ctx.fill(); ctx.strokeStyle = "#fff"; ctx.lineWidth = 1 * DPR; ctx.stroke(); }
      if (f.line) { ctx.strokeStyle = f.color; ctx.lineWidth = 2.2 * DPR; ctx.setLineDash([7 * DPR, 4 * DPR]); ctx.beginPath(); ctx.moveTo(X(x0), Y(f.line.a + f.line.b * x0)); ctx.lineTo(X(x1), Y(f.line.a + f.line.b * x1)); ctx.stroke(); ctx.setLineDash([]); }
      if (f.next) { ctx.strokeStyle = "#087f5b"; ctx.lineWidth = 2 * DPR; ctx.beginPath(); ctx.arc(X(f.next[0]), Y(f.next[1]), 8 * DPR, 0, 7); ctx.stroke(); ctx.beginPath(); ctx.moveTo(X(f.next[0]) - 13 * DPR, Y(f.next[1])); ctx.lineTo(X(f.next[0]) + 13 * DPR, Y(f.next[1])); ctx.moveTo(X(f.next[0]), Y(f.next[1]) - 13 * DPR); ctx.lineTo(X(f.next[0]), Y(f.next[1]) + 13 * DPR); ctx.stroke(); ctx.fillStyle = "#087f5b"; ctx.font = `${11 * DPR}px Consolas`; ctx.textAlign = "left"; ctx.fillText("다음 놓기", X(f.next[0]) + 10 * DPR, Y(f.next[1]) - 10 * DPR); }
    }
    // 현재점 (마름모)
    if (LG.ds.n) { const i = LG.cur(); const cx = S.xyPts ? LG.val("a_beta", i) : LG.val(el("xyX").value, i), cy = S.xyPts ? LG.val("u_phi", i) : LG.val(el("xyY").value, i); if (isFinite(cx) && isFinite(cy)) { ctx.fillStyle = "#c98a00"; ctx.beginPath(); ctx.moveTo(X(cx), Y(cy) - 6 * DPR); ctx.lineTo(X(cx) + 6 * DPR, Y(cy)); ctx.lineTo(X(cx), Y(cy) + 6 * DPR); ctx.lineTo(X(cx) - 6 * DPR, Y(cy)); ctx.closePath(); ctx.fill(); } }
    ctx.restore();
    let ly = T + 26 * DPR; ctx.font = `${11 * DPR}px Consolas`; ctx.textAlign = "left";
    for (const f of S.xyFits) { ctx.fillStyle = f.color; ctx.fillText(f.eq, L + 8 * DPR, ly); ly += 14 * DPR; }
    ctx.fillStyle = "#5c6675"; ctx.fillText(S.xyPts ? `놓기점 ${S.xyPts.sel.length}개 (분홍 +낙하 · 파랑 −낙하 · 마름모 = 지금)` : `${LG.sel ? "선택 구간" : "보이는 창"} ${P.sel.length} 점`, L + 8 * DPR, T + PH - 8 * DPR);
  }
  function addXyFit() {
    const type = el("xyFit").value, color = FIT_COLORS[(S.xyFits.length + 3) % FIT_COLORS.length];
    if (!LG.ds.n) { LG.toast("데이터 없음"); return; }
    if (type === "lin") {
      if (S.xyPts) { S.xyPts = null; }
      const P = xyPoints(); const xs = P.sel.map(p => p[0]), ys = P.sel.map(p => p[1]); const lr = linreg(xs, ys); if (!lr) { LG.toast("점이 부족하거나 x 가 변하지 않는다", true); return; }
      const xl = COL[el("xyX").value].l, yl = COL[el("xyY").value].l;
      let extra = ""; if (el("xyX").value === "del" && el("xyY").value === "a_alpha") extra = `   → P2R = −a = ${(-lr.b).toFixed(4)}`;
      if (el("xyX").value === "a_beta" && el("xyY").value === "u_phi") extra = `   → r = ${lr.b.toFixed(3)}, c₀ = ${lr.a.toFixed(3)}`;
      S.xyFits.push({ type, color, line: lr, eq: `${yl} = ${lr.b.toFixed(4)}·${xl} ${lr.a >= 0 ? "+" : "−"} ${Math.abs(lr.a).toFixed(4)}   R²=${lr.r2.toFixed(4)}  n=${lr.n}${extra}` });
    } else if (type === "rel" || type === "rec") {
      const entry = { type, color, eq: type === "rel" ? "놓기점 찾는 중… (시행 나누기 → 경계선)" : "다음 놓기 추천 계산 중…" }; S.xyFits.push(entry); renderXyFits();
      const phi_eq = LG.PIPE ? LG.PIPE.phi_eq : 0;
      analyze("trials", { phi_eq }, res => {
        if (!res.ok) { entry.eq = "시행 나누기 실패: " + (res.msg || ""); renderXyFits(); return; }
        const pts = res.table.filter(r => r.dir_valid).map(r => [r.beta0, r.phi0, r.dir]);
        S.xyPts = { sel: pts, all: [] }; entry.pts = pts;
        el("xyX").value = "a_beta"; el("xyY").value = "u_phi";
        entry.eq = `놓기점 ${pts.length}개 (방향 유효; 전체 ${res.table.length} 시행) · ${type === "rel" ? "경계선" : "추천"} 계산 중…`;
        if (type === "rel") analyze("boundary", { phi_eq }, rb => {
          if (!rb.ok) { entry.eq += " · 경계선 실패: " + (rb.msg || ""); renderXyFits(); return; }
          const r = rb.result; const rr = r.r_grid_best ?? r.r_used ?? r.r, c0 = r.c0_grid_best ?? r.c0;
          if (isFinite(rr) && isFinite(c0)) entry.line = { a: c0, b: rr };
          entry.eq = `놓기 경계  φ = ${fmt(rr, 3)}·β ${c0 >= 0 ? "+" : "−"} ${fmt(Math.abs(c0), 3)}   (오분류 ${r.errors_grid ?? r.errors ?? "—"}, 점 ${pts.length})`; renderXyFits();
        });
        else analyze("recommend", { phi_eq }, rc => {
          if (!rc.ok) { entry.eq += " · 추천 실패: " + (rc.msg || ""); renderXyFits(); return; }
          const r = rc.result, nx = rc.next || {};
          if (isFinite(r.r) && isFinite(r.c0)) entry.line = { a: r.c0, b: r.r };
          if (isFinite(nx.beta) && isFinite(nx.phi)) entry.next = [nx.beta, nx.phi];
          entry.eq = `r̂ = ${fmt(r.r, 3)} ± ${fmt(r.se_r, 3)}, ĉ₀ = ${fmt(r.c0, 2)}, ω̂ = ${fmt(r.om_hat)}  (n=${r.n})  →  다음 놓기 β ${fmt(nx.beta)}°, φ ${fmt(nx.phi)}°`; renderXyFits();
        });
        renderXyFits();
      });
    }
    renderXyFits();
  }
  function renderXyFits() { const box = el("xyFits"); box.innerHTML = S.xyFits.length ? "" : `<span class="hint">추세선 없음 — 선형(x=δ, y=α 면 P2R), 놓기점→경계선(r·c₀), 다음 놓기 추천</span>`; S.xyFits.forEach((f, k) => { const d = document.createElement("div"); d.className = "fit"; d.innerHTML = `<span class="sw" style="background:${f.color}"></span><span class="eq" title="${f.eq}">${f.eq}</span><button class="btn sm">삭제</button>`; d.querySelector("button").onclick = () => { S.xyFits.splice(k, 1); if (!S.xyFits.some(x => x.pts)) S.xyPts = null; renderXyFits(); }; box.appendChild(d); }); }
  el("bXyFit").onclick = addXyFit; el("bXyClear").onclick = () => { S.xyFits = []; S.xyPts = null; renderXyFits(); }; renderXyFits();
  el("xyX").onchange = el("xyY").onchange = () => { S.xyPts = null; };

  LG.on("ds_full", () => { S.tcFits = []; S.xyFits = []; S.xyPts = null; renderFits(); renderXyFits(); updateSelInfo(); });

  // ================= 프레임 루프 (엑셀 차트·표만; 트윈·평면·스트립차트는 lg_panels 의 루프) =================
  let lastTbl = 0;
  function frame() {
    requestAnimationFrame(frame);
    renderTC(); renderXY();
    const now = performance.now();
    if (now - lastTbl > 50) {
      lastTbl = now; tableFollowCursor(); renderTable();
      const i = LG.cur();
      if (i >= 0) el("fCur").innerHTML = `<span class="k">t</span> ${fmt(LG.tOf(i), 3)} s  <span class="k">φ</span> ${fmt(LG.val("u_phi", i), 3)}  <span class="k">ank</span> ${fmt(LG.val("u_ank", i), 3)}  <span class="k">δ</span> ${fmt(LG.val("del", i))}  <span class="k">α</span> ${fmt(LG.val("a_alpha", i), 3)}  <span class="k">β</span> ${fmt(LG.val("a_beta", i), 3)}  <span class="k">Â</span> ${fmt(LG.val("a_Ahat", i), 3)}  <span class="k">hold</span> ${fmt(LG.val("hold", i))}  <span class="k">[${i + 1}/${LG.ds.n}]</span>`;
    }
  }
  frame();
  LG.lab = { S, COLS, addTcFit, addXyFit, copyRows };
})();
