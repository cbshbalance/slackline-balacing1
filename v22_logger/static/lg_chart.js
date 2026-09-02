// lg_chart.js — 스트립 차트 (레인 3개: 각도 / 속도·Â / phase·hold·err) + 선택 구간 + 분석 오버레이 + 보조 곡선
// 조작: 휠=시간 줌 · 좌드래그=이동 · 클릭=커서(재생 위치) · Shift+드래그=분석 구간 선택 · 더블클릭=라이브 따라가기 복귀
"use strict";
(function () {
  const cv = LG.el("chart"), ctx = cv.getContext("2d");
  LG.chart = { win: LG.store.get("chartWin", 10), view: null, lanes: null, hover: null };
  LG.chartOverlay = [];
  const DEF_LANES = [
    { id: "ang", label: "각도 [°]", w: 3, chans: [
      { c: "u_phi", col: "#3a86ff", on: true, label: "φ" }, { c: "u_ank", col: "#06d6a0", on: true, label: "ank" },
      { c: "del", col: "#ffbe0b", on: true, label: "δ" }, { c: "a_alpha", col: "#8338ec", on: false, label: "α(앱)" },
      { c: "alpha_fw", col: "#8338ec", dash: true, on: false, label: "α(펌)" },
      { c: "a_beta", col: "#ff9f43", on: true, label: "β(앱)" }, { c: "beta_fw", col: "#ff9f43", dash: true, on: false, label: "β(펌)" },
      { c: "a_theta", col: "#c77dff", on: false, label: "θ(앱)" }, { c: "s_phi", col: "#e0e6ff", dash: true, on: false, label: "φ 평활" },
      { c: "s_beta", col: "#ffd6a5", dash: true, on: false, label: "β 평활" }, { c: "a_psi", col: "#8ecae6", on: false, label: "ψ=φ−φ_eq" } ] },
    { id: "vel", label: "속도 [°/s] · Â [°]", w: 2, chans: [
      { c: "a_dphi", col: "#3a86ff", on: false, label: "φ̇(앱)" }, { c: "dphi_fw", col: "#3a86ff", dash: true, on: false, label: "φ̇(펌)" },
      { c: "a_dbeta", col: "#ff9f43", on: false, label: "β̇(앱)" }, { c: "dbeta_fw", col: "#ff9f43", dash: true, on: false, label: "β̇(펌)" },
      { c: "s_dphi", col: "#e0e6ff", dash: true, on: false, label: "φ̇ 평활" },
      { c: "a_Ahat", col: "#ff006e", on: true, label: "Â(앱)" }, { c: "Ahat_fw", col: "#ff006e", dash: true, on: true, label: "Â(펌)" } ] },
    { id: "st", label: "phase · hold · err", w: 1, chans: [
      { c: "phase", col: "#ffd166", on: true, step: true, label: "phase" }, { c: "hold", col: "#06d6a0", on: true, label: "hold" },
      { c: "err", col: "#ef4444", on: true, step: true, label: "err" }, { c: "cue", col: "#8ecae6", on: false, step: true, label: "cue" } ] },
  ];
  const saved = LG.store.get("chans", null);
  LG.chart.lanes = DEF_LANES;
  if (saved) for (const L of DEF_LANES) for (const ch of L.chans) if (saved[ch.c] != null) ch.on = saved[ch.c];
  LG.saveChans = function () { const o = {}; for (const L of DEF_LANES) for (const ch of L.chans) o[ch.c] = ch.on; LG.store.set("chans", o); };
  LG.laneOf = function (c) { for (const L of DEF_LANES) if (L.chans.some(x => x.c === c)) return L; return DEF_LANES[0]; };

  function timeRange() {
    const n = LG.ds.n, t = LG.ds.data.t;
    if (!n) return [0, LG.chart.win];
    if (LG.chart.view && !LG.follow) return [LG.chart.view.t0, LG.chart.view.t1];
    const t1 = LG.follow ? t[n - 1] : t[LG.cur()];
    return [t1 - LG.chart.win, t1 + (LG.follow ? 0 : LG.chart.win * 0.15)];
  }
  LG.chartRange = timeRange;

  function drawSeries(X, Y, tA, vA, i0, i1, W, step) {
    // 픽셀당 표본이 많으면 min/max 로 묶는다
    const n = i1 - i0 + 1; if (n <= 0) return;
    if (n > 2.5 * W) {
      let px = -1, mn = 0, mx = 0, first = true;
      ctx.beginPath();
      for (let i = i0; i <= i1; i++) {
        const v = vA[i]; if (!isFinite(v)) continue;
        const x = X(tA[i]) | 0;
        if (x !== px) {
          if (px >= 0) { ctx.moveTo(px, Y(mn)); ctx.lineTo(px, Y(mx)); if (mn === mx) ctx.lineTo(px + 1, Y(mx)); }
          px = x; mn = mx = v;
        } else { if (v < mn) mn = v; if (v > mx) mx = v; }
      }
      if (px >= 0) { ctx.moveTo(px, Y(mn)); ctx.lineTo(px, Y(mx)); }
      ctx.stroke();
    } else {
      ctx.beginPath(); let first = true, py = 0;
      for (let i = i0; i <= i1; i++) {
        const v = vA[i]; if (!isFinite(v)) { first = true; continue; }
        const x = X(tA[i]), y = Y(v);
        if (first) { ctx.moveTo(x, y); first = false; } else { if (step) ctx.lineTo(x, py); ctx.lineTo(x, y); }
        py = y;
      }
      ctx.stroke();
    }
  }

  LG.renderChart = function () {
    const W = cv.width = cv.clientWidth * devicePixelRatio, H = cv.height = cv.clientHeight * devicePixelRatio;
    if (!W || !H) return;
    const DPR = devicePixelRatio, LPAD = 44 * DPR, RPAD = 8 * DPR, TPAD = 6 * DPR;
    ctx.clearRect(0, 0, W, H); ctx.fillStyle = "#0f131b"; ctx.fillRect(0, 0, W, H);
    const n = LG.ds.n, tA = LG.ds.data.t;
    const [t0, t1] = timeRange();
    const X = t => LPAD + (t - t0) / (t1 - t0) * (W - LPAD - RPAD);
    LG.chart.X = X; LG.chart.t0 = t0; LG.chart.t1 = t1; LG.chart.W = W; LG.chart.H = H; LG.chart.LPAD = LPAD;
    const i0 = n ? Math.max(0, LG.idxOfT(t0) - 1) : 0, i1 = n ? Math.min(n - 1, LG.idxOfT(t1) + 1) : -1;
    const cur = LG.cur();
    const wsum = DEF_LANES.reduce((s, L) => s + L.w, 0);
    let y0 = TPAD;
    const laneBox = {};
    for (const L of DEF_LANES) {
      const h = (H - TPAD - 16 * DPR) * L.w / wsum, yb = y0 + h;
      laneBox[L.id] = { y0, y1: yb };
      // 범위
      let ymax = 0.5, ymin = -0.5, any = false;
      const fixed = L.id === "st";
      if (n && i1 >= i0) for (const ch of L.chans) {
        if (!ch.on) continue; const a = LG.col(ch.c); if (!a) continue;
        const stp = Math.max(1, Math.floor((i1 - i0) / 3000));
        for (let i = i0; i <= i1; i += stp) { const v = a[i]; if (!isFinite(v)) continue; if (!any) { ymax = ymin = v; any = true; } else { if (v > ymax) ymax = v; if (v < ymin) ymin = v; } }
      }
      if (fixed) { ymin = Math.min(0, ymin); ymax = Math.max(ymax, 6); }
      else { const pad = Math.max((ymax - ymin) * 0.12, 0.3); ymax += pad; ymin -= pad; if (ymax - ymin < 1) { const c = (ymax + ymin) / 2; ymax = c + 0.5; ymin = c - 0.5; } }
      const Y = v => yb - (v - ymin) / (ymax - ymin) * (h - 4 * DPR) - 2 * DPR;
      laneBox[L.id].Y = Y; laneBox[L.id].ymin = ymin; laneBox[L.id].ymax = ymax;
      // 배경·0선
      ctx.fillStyle = "#10141d"; ctx.fillRect(LPAD, y0, W - LPAD - RPAD, h);
      ctx.strokeStyle = "rgba(255,255,255,.12)"; ctx.lineWidth = 1 * DPR;
      if (ymin < 0 && ymax > 0) { ctx.beginPath(); ctx.moveTo(LPAD, Y(0)); ctx.lineTo(W - RPAD, Y(0)); ctx.stroke(); }
      // phase 음영 (레인 공통): fold 분홍 · 발산 주황 · REST 회색
      if (n && i1 >= i0 && L.id === "ang") {
        const ph = LG.col("phase");
        if (ph) {
          let a = -1, pv = 0;
          for (let i = i0; i <= i1 + 1; i++) {
            const v = i <= i1 ? (ph[i] | 0) : -2;
            if (v !== pv) {
              if (a >= 0 && pv > 0) {
                ctx.fillStyle = pv === 1 ? "rgba(255,0,110,.12)" : pv === 2 ? "rgba(255,255,255,.06)" : pv === 5 ? "rgba(255,159,67,.14)" : pv === 6 ? "rgba(239,68,68,.10)" : "rgba(255,209,102,.06)";
                ctx.fillRect(X(tA[a]), TPAD, Math.max(1, X(tA[Math.min(i, i1)]) - X(tA[a])), H - TPAD - 16 * DPR);
              }
              a = i; pv = v;
            }
          }
        }
      }
      // 선택 구간 · 분석 밴드
      if (LG.sel) { ctx.fillStyle = "rgba(76,201,240,.10)"; ctx.fillRect(X(LG.sel.t0), y0, X(LG.sel.t1) - X(LG.sel.t0), h); }
      if (LG.showOverlay) for (const o of LG.chartOverlay) if (o.kind === "band") { ctx.fillStyle = o.color || "rgba(255,255,255,.08)"; ctx.fillRect(X(o.t0), y0, Math.max(1, X(o.t1) - X(o.t0)), h); }
      // 시리즈
      ctx.save(); ctx.beginPath(); ctx.rect(LPAD, y0, W - LPAD - RPAD, h); ctx.clip();
      if (n && i1 >= i0) for (const ch of L.chans) {
        if (!ch.on) continue; const a = LG.col(ch.c); if (!a) continue;
        ctx.strokeStyle = ch.col; ctx.lineWidth = (ch.dash ? 1.1 : 1.5) * DPR; ctx.setLineDash(ch.dash ? [4 * DPR, 3 * DPR] : []);
        drawSeries(X, Y, tA, a, i0, i1, W, ch.step);
        ctx.setLineDash([]);
      }
      // 분석 오버레이 (이 레인 채널 것)
      if (LG.showOverlay) for (const o of LG.chartOverlay) {
        const lane = o.ch ? LG.laneOf(o.ch) : DEF_LANES[0]; if (lane !== L) continue;
        if (o.kind === "line") { ctx.strokeStyle = o.color || "#ff006e"; ctx.lineWidth = 2 * DPR; ctx.beginPath(); let f = true; for (let k = 0; k < o.t.length; k++) { if (o.y[k] == null) { f = true; continue; } if (f) { ctx.moveTo(X(o.t[k]), Y(o.y[k])); f = false; } else ctx.lineTo(X(o.t[k]), Y(o.y[k])); } ctx.stroke(); }
        else if (o.kind === "points") { ctx.fillStyle = o.color || "#ff006e"; for (let k = 0; k < o.t.length; k++) { if (o.y[k] == null) continue; ctx.beginPath(); ctx.arc(X(o.t[k]), Y(o.y[k]), 2.2 * DPR, 0, 7); ctx.fill(); } }
        else if (o.kind === "hline") { ctx.strokeStyle = o.color || "rgba(255,255,255,.4)"; ctx.lineWidth = 1 * DPR; ctx.setLineDash([3 * DPR, 3 * DPR]); ctx.beginPath(); ctx.moveTo(LPAD, Y(o.y)); ctx.lineTo(W - RPAD, Y(o.y)); ctx.stroke(); ctx.setLineDash([]); if (o.label) { ctx.fillStyle = o.color || "#fff"; ctx.font = `${9.5 * DPR}px Consolas`; ctx.fillText(o.label, W - RPAD - 60 * DPR, Y(o.y) - 2 * DPR); } }
      }
      ctx.restore();
      // 라벨
      ctx.font = `${10 * DPR}px Consolas`; ctx.fillStyle = "rgba(255,255,255,.5)";
      ctx.fillText(ymax.toFixed(1), 3 * DPR, y0 + 10 * DPR); ctx.fillText(ymin.toFixed(1), 3 * DPR, yb - 3 * DPR);
      let lx = LPAD + 6 * DPR;
      ctx.fillStyle = "rgba(255,255,255,.55)"; ctx.fillText(L.label, lx, y0 + 11 * DPR); lx += ctx.measureText(L.label).width + 14 * DPR;
      for (const ch of L.chans) {
        if (!ch.on) continue; const v = LG.val(ch.c, cur);
        const txt = `${ch.label}=${isFinite(v) ? v.toFixed(2) : "—"}`; ctx.fillStyle = ch.col; ctx.fillText(txt, lx, y0 + 11 * DPR); lx += ctx.measureText(txt).width + 10 * DPR;
      }
      y0 = yb + 3 * DPR;
    }
    // 이벤트 · 마크 (수직선)
    const ev = LG.aux.events || [];
    ctx.font = `${9.5 * DPR}px Consolas`;
    for (const e of ev) {
      const te = LG.tOfMs(e[0]); if (!isFinite(te) || te < t0 || te > t1) continue;
      const x = X(te);
      ctx.strokeStyle = e[1] === "MARK" || e[1] === "T_RESET" ? "rgba(255,209,102,.7)" : "rgba(255,255,255,.35)"; ctx.lineWidth = 1 * DPR; ctx.setLineDash([2 * DPR, 3 * DPR]);
      ctx.beginPath(); ctx.moveTo(x, TPAD); ctx.lineTo(x, H - 16 * DPR); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = "rgba(255,209,102,.9)"; ctx.fillText(e[1] + (e[2] ? " " + e[2] : ""), x + 2 * DPR, TPAD + 22 * DPR);
    }
    if (LG.showOverlay) for (const o of LG.chartOverlay) if (o.kind === "vline" && o.t >= t0 && o.t <= t1) { const x = X(o.t); ctx.strokeStyle = o.color || "rgba(255,255,255,.35)"; ctx.lineWidth = 1 * DPR; ctx.beginPath(); ctx.moveTo(x, TPAD); ctx.lineTo(x, H - 16 * DPR); ctx.stroke(); if (o.label) { ctx.fillStyle = "#fff"; ctx.fillText(o.label, x + 2 * DPR, H - 20 * DPR); } }
    // 커서
    if (n) {
      const xc = X(tA[cur]);
      ctx.strokeStyle = "#ffd166"; ctx.lineWidth = 1.2 * DPR; ctx.beginPath(); ctx.moveTo(xc, TPAD); ctx.lineTo(xc, H - 16 * DPR); ctx.stroke();
    }
    // 시간축
    ctx.fillStyle = "rgba(255,255,255,.5)"; ctx.font = `${10 * DPR}px Consolas`;
    const span = t1 - t0, stp = span > 60 ? 10 : span > 20 ? 5 : span > 8 ? 2 : span > 3 ? 1 : span > 1 ? 0.5 : 0.1;
    for (let tt = Math.ceil(t0 / stp) * stp; tt <= t1; tt += stp) { const x = X(tt); ctx.fillText(tt.toFixed(stp < 1 ? 1 : 0) + "s", x - 8 * DPR, H - 3 * DPR); ctx.fillRect(x, H - 16 * DPR, 1, 3 * DPR); }
    if (LG.sel) { ctx.fillStyle = "#4cc9f0"; ctx.fillText(`선택 ${LG.sel.t0.toFixed(2)}–${LG.sel.t1.toFixed(2)} s (${(LG.sel.t1 - LG.sel.t0).toFixed(2)} s)`, X(LG.sel.t0) + 3 * DPR, TPAD + 34 * DPR); }
    if (LG.chart.hover) { const hv = LG.chart.hover; ctx.fillStyle = "rgba(255,255,255,.7)"; ctx.fillText(`t=${hv.t.toFixed(3)} s`, X(hv.t) + 4 * DPR, H - 20 * DPR); }
  };

  // ---- 조작 ----
  const tAt = e => { const r = cv.getBoundingClientRect(); const px = (e.clientX - r.left) * devicePixelRatio; const c = LG.chart; return c.t0 + (px - c.LPAD) / (c.W - c.LPAD - 8 * devicePixelRatio) * (c.t1 - c.t0); };
  let drag = null;
  cv.addEventListener("mousedown", e => {
    if (e.button !== 0) return;
    const t = tAt(e);
    drag = { x: e.clientX, t, shift: e.shiftKey, moved: false, v0: [LG.chart.t0, LG.chart.t1] };
    if (e.shiftKey) LG.sel = { t0: t, t1: t };
  });
  window.addEventListener("mousemove", e => {
    if (cv.matches(":hover") && LG.ds.n) LG.chart.hover = { t: tAt(e) }; else LG.chart.hover = null;
    if (!drag) return;
    const t = tAt(e);
    if (Math.abs(e.clientX - drag.x) > 3) drag.moved = true;
    if (drag.shift) { LG.sel = { t0: Math.min(drag.t, t), t1: Math.max(drag.t, t) }; LG.emit("sel"); }
    else if (drag.moved) {
      const span = drag.v0[1] - drag.v0[0]; const dt = (e.clientX - drag.x) * devicePixelRatio / (LG.chart.W - LG.chart.LPAD - 8 * devicePixelRatio) * span;
      LG.follow = false; LG.chart.view = { t0: drag.v0[0] - dt, t1: drag.v0[1] - dt }; LG.emit("follow");
    }
  });
  window.addEventListener("mouseup", e => {
    if (!drag) return;
    if (!drag.moved && !drag.shift && LG.ds.n) { LG.setCursor(LG.idxOfT(drag.t)); }
    if (drag.shift && LG.sel && LG.sel.t1 - LG.sel.t0 < 0.02) LG.sel = null;
    drag = null; LG.emit("sel");
  });
  cv.addEventListener("wheel", e => {
    e.preventDefault(); const t = tAt(e); const [a, b] = [LG.chart.t0, LG.chart.t1];
    const k = e.deltaY > 0 ? 1.2 : 1 / 1.2;
    if (LG.follow) { LG.chart.win = Math.max(0.2, Math.min(3600, LG.chart.win * k)); LG.store.set("chartWin", LG.chart.win); }
    else { LG.chart.view = { t0: t + (a - t) * k, t1: t + (b - t) * k }; }
  }, { passive: false });
  cv.addEventListener("dblclick", () => { LG.follow = true; LG.cursor = -1; LG.chart.view = null; LG.emit("follow"); });
  cv.addEventListener("contextmenu", e => e.preventDefault());

  // ---- 보조 곡선 (분석 패널) ----
  LG.drawCurve = function (canvas, cur) {
    const c = canvas.getContext("2d"), W = canvas.width = canvas.clientWidth * devicePixelRatio, H = canvas.height = canvas.clientHeight * devicePixelRatio;
    c.clearRect(0, 0, W, H); c.fillStyle = "#10141d"; c.fillRect(0, 0, W, H);
    if (!cur) return;
    const DPR = devicePixelRatio, L = 40 * DPR, R = 8 * DPR, T = 16 * DPR, B = 18 * DPR;
    const xs = [], ys = [];
    if (cur.kind === "xy") { for (let k = 0; k < cur.x.length; k++) if (cur.x[k] != null && cur.y[k] != null) { xs.push(cur.x[k]); ys.push(cur.y[k]); } if (cur.fit_x) for (let k = 0; k < cur.fit_x.length; k++) { xs.push(cur.fit_x[k]); ys.push(cur.fit_y[k]); } }
    else { for (const x of cur.x) if (x != null) xs.push(x); for (const s of cur.series) for (const y of s.y) if (y != null && isFinite(y)) ys.push(y); }
    if (!xs.length || !ys.length) return;
    let x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
    if (x1 - x0 < 1e-9) { x0 -= 1; x1 += 1; } if (y1 - y0 < 1e-9) { y0 -= 1; y1 += 1; }
    const px = (x1 - x0) * 0.05, py = (y1 - y0) * 0.1; x0 -= px; x1 += px; y0 -= py; y1 += py;
    const X = x => L + (x - x0) / (x1 - x0) * (W - L - R), Y = y => H - B - (y - y0) / (y1 - y0) * (H - T - B);
    c.strokeStyle = "rgba(255,255,255,.15)"; c.lineWidth = 1 * DPR; c.strokeRect(L, T, W - L - R, H - T - B);
    c.font = `${9.5 * DPR}px Consolas`; c.fillStyle = "rgba(255,255,255,.55)";
    c.fillText(y1.toFixed(2), 2 * DPR, T + 8 * DPR); c.fillText(y0.toFixed(2), 2 * DPR, H - B); c.fillText(x0.toFixed(2), L, H - 4 * DPR); c.fillText(x1.toFixed(2), W - R - 40 * DPR, H - 4 * DPR);
    c.fillStyle = "#cfe0ff"; c.fillText((cur.label || "") + (cur.xlab ? `   x: ${cur.xlab}` : "") + (cur.ylab ? `  y: ${cur.ylab}` : ""), L + 4 * DPR, T - 4 * DPR);
    if (cur.kind === "xy") {
      c.fillStyle = "#ffd166"; for (let k = 0; k < cur.x.length; k++) { if (cur.x[k] == null || cur.y[k] == null) continue; c.beginPath(); c.arc(X(cur.x[k]), Y(cur.y[k]), 2.2 * DPR, 0, 7); c.fill(); }
      if (cur.fit_x) { c.strokeStyle = "#ff006e"; c.lineWidth = 1.8 * DPR; c.beginPath(); for (let k = 0; k < cur.fit_x.length; k++) { if (k === 0) c.moveTo(X(cur.fit_x[k]), Y(cur.fit_y[k])); else c.lineTo(X(cur.fit_x[k]), Y(cur.fit_y[k])); } c.stroke(); }
    } else {
      let ly = T + 10 * DPR;
      for (const s of cur.series) {
        c.strokeStyle = s.color || "#fff"; c.lineWidth = 1.6 * DPR; c.beginPath(); let f = true;
        for (let k = 0; k < cur.x.length; k++) { const y = s.y[k]; if (y == null || !isFinite(y)) { f = true; continue; } if (f) { c.moveTo(X(cur.x[k]), Y(y)); f = false; } else c.lineTo(X(cur.x[k]), Y(y)); }
        c.stroke(); c.fillStyle = s.color || "#fff"; c.fillText(s.label, W - R - 110 * DPR, ly); ly += 11 * DPR;
      }
      if (cur.vline != null) { c.strokeStyle = "#ffd166"; c.setLineDash([3 * DPR, 3 * DPR]); c.beginPath(); c.moveTo(X(cur.vline), T); c.lineTo(X(cur.vline), H - B); c.stroke(); c.setLineDash([]); c.fillStyle = "#ffd166"; c.fillText(cur.vline.toFixed(2), X(cur.vline) + 3 * DPR, T + 20 * DPR); }
    }
  };
})();
