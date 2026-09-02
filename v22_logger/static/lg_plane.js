// lg_plane.js — 상태공간 평면 두 장 (원위치 (β, φ) · 예측점 (β_pred, φ_pred)) — v21 drawPlane 의 측정용 판
// 그리는 것: CoM 등고선 · 축 · 모델선(초록: pl1 안정모드선 φ=r·β / pl2 A=0 선 + 트리거 띠) · 실측선(주황, 파이프라인 r·c0)
//           · 궤적(금색 = 앱 인과값, 흰 점선 = 비인과 평활) · 현재점(마름모) · 분석 오버레이(놓기점·경계선)
"use strict";
(function () {
  const TRAIL = LG.store.get("trail", 2.5);
  LG.PLVIEW = { pl1: { on: false, m: 3, b: 0, f: 0 }, pl2: { on: false, m: 3, b: 0, f: 0 } };
  LG.PLSCALE = {};
  LG.planeOverlay = [];           // 분석 결과의 plane 항목
  const ctxs = { pl1: LG.el("pl1").getContext("2d"), pl2: LG.el("pl2").getContext("2d") };

  function seriesFor(kind) {
    const d = LG.ds.data;
    if (kind === "pl1") return { x: d.a_beta, y: d.u_phi, xs: d.s_beta, ys: d.s_phi, xf: LG.col("beta_fw"), yf: LG.col("phi") };
    return { x: d.a_bp, y: d.a_fp, xs: d.s_bp, ys: d.s_fp, xf: null, yf: null };
  }

  function draw(kind) {
    const ctx = ctxs[kind], cv = ctx.canvas;
    const W = cv.width = cv.clientWidth * devicePixelRatio, H = cv.height = cv.clientHeight * devicePixelRatio;
    if (!W || !H) return;
    const DPR = devicePixelRatio, PAD = 8 * DPR;
    ctx.clearRect(0, 0, W, H); ctx.fillStyle = "#10141d"; ctx.fillRect(0, 0, W, H);
    const PL = LG.PL; if (!PL) return;
    const n = LG.ds.n, cur = LG.cur();
    const S = seriesFor(kind);
    const tArr = LG.ds.data.t;
    const showSm = LG.el("cPlSmooth") ? LG.el("cPlSmooth").checked : true;
    const showFw = LG.el("cPlFw") ? LG.el("cPlFw").checked : false;
    // 꼬리 구간
    let i0 = 0;
    if (n > 0 && S.x) { const t1 = tArr[cur]; i0 = Math.max(0, LG.idxOfT(t1 - TRAIL)); }
    // 범위: 꼬리의 85 백분위 × 1.6, 최소 2°, 트리거 띠 보이게
    let m = 2.0;
    if (n > 0 && S.x) {
      const vals = [];
      const step = Math.max(1, Math.floor((cur - i0) / 400));
      for (let i = i0; i <= cur; i += step) { const a = Math.abs(S.x[i]), b = Math.abs(S.y[i]); if (isFinite(a) && isFinite(b)) { vals.push(a, b); } }
      vals.sort((a, b) => a - b);
      const p85 = vals.length ? vals[Math.floor(vals.length * 0.85)] : 1.0;
      m = Math.max(p85 * 1.6, 2.0);
      if (kind === "pl2") m = Math.max(m, 3.0 * LG.trig / Math.abs(PL.wq[0] || 1));
    }
    const V = LG.PLVIEW[kind].on ? LG.PLVIEW[kind] : null;
    let vb = 0, vf = 0; if (V) { m = V.m; vb = V.b; vf = V.f; }
    const cx = W / 2, cy = H / 2, R = Math.min(W, H) / 2 - PAD - 12 * DPR;
    const X = b => cx + ((b - vb) / m) * R, Y = f => cy - ((f - vf) / m) * R;
    LG.PLSCALE[kind] = { m, cx, cy, R, vb, vf };
    const bL = vb - m, bR = vb + m;
    ctx.save(); ctx.beginPath(); ctx.rect(cx - R, cy - R, 2 * R, 2 * R); ctx.clip();
    // CoM 등고선
    const sC = PL.sCoM;
    ctx.strokeStyle = "rgba(255,255,255,.10)"; ctx.lineWidth = 1 * DPR;
    const ci0 = Math.round((vf + sC * vb) / (m / 4));
    for (let i = ci0 - 24; i <= ci0 + 24; i++) { const c = i * (m / 4); ctx.beginPath(); ctx.moveTo(X(bL), Y(-sC * bL + c)); ctx.lineTo(X(bR), Y(-sC * bR + c)); ctx.stroke(); }
    // 축
    ctx.strokeStyle = "rgba(255,255,255,.22)";
    ctx.beginPath(); ctx.moveTo(cx - R, Y(0)); ctx.lineTo(cx + R, Y(0)); ctx.moveTo(X(0), cy - R); ctx.lineTo(X(0), cy + R); ctx.stroke();
    // 모델선 (초록)
    const sl = kind === "pl1" ? PL.r : PL.slopeA0;
    if (kind === "pl2") {
      const off = LG.trig / Math.abs(PL.wq[0] || 1);
      ctx.fillStyle = "rgba(255,190,11,.12)"; ctx.beginPath();
      ctx.moveTo(X(bL), Y(sl * bL + off)); ctx.lineTo(X(bR), Y(sl * bR + off)); ctx.lineTo(X(bR), Y(sl * bR - off)); ctx.lineTo(X(bL), Y(sl * bL - off)); ctx.closePath(); ctx.fill();
      ctx.strokeStyle = "rgba(255,190,11,.9)"; ctx.lineWidth = 1.6 * DPR; ctx.setLineDash([6 * DPR, 4 * DPR]);
      for (const s of [1, -1]) { ctx.beginPath(); ctx.moveTo(X(bL), Y(sl * bL + s * off)); ctx.lineTo(X(bR), Y(sl * bR + s * off)); ctx.stroke(); }
      ctx.setLineDash([]);
    }
    ctx.strokeStyle = "#06d6a0"; ctx.lineWidth = 2.0 * DPR; if (kind === "pl1") ctx.setLineDash([7 * DPR, 5 * DPR]);
    ctx.beginPath(); ctx.moveTo(X(bL), Y(sl * bL)); ctx.lineTo(X(bR), Y(sl * bR)); ctx.stroke(); ctx.setLineDash([]);
    // 실측선 (주황): φ = r·β + c0 (파이프라인 값)
    if (LG.PIPE && isFinite(LG.PIPE.r) && LG.PIPE.r !== 0) {
      const rM = LG.PIPE.r, cM = LG.PIPE.c0 || 0;
      ctx.strokeStyle = "#ff9f43"; ctx.lineWidth = 1.8 * DPR; ctx.setLineDash([2.5 * DPR, 3 * DPR]);
      ctx.beginPath(); ctx.moveTo(X(bL), Y(rM * bL + cM)); ctx.lineTo(X(bR), Y(rM * bR + cM)); ctx.stroke();
      if (kind === "pl2") {
        const offM = Math.abs(LG.trig * rM); ctx.lineWidth = 1.0 * DPR; ctx.globalAlpha = 0.7;
        for (const s of [1, -1]) { ctx.beginPath(); ctx.moveTo(X(bL), Y(rM * bL + cM + s * offM)); ctx.lineTo(X(bR), Y(rM * bR + cM + s * offM)); ctx.stroke(); }
        ctx.globalAlpha = 1;
      }
      ctx.setLineDash([]);
    }
    // 궤적
    if (n > 0 && S.x) {
      const step = Math.max(1, Math.floor((cur - i0) / 1500));
      if (showFw && S.xf && S.yf && kind === "pl1") {
        ctx.strokeStyle = "rgba(76,201,240,.45)"; ctx.lineWidth = 1.2 * DPR; ctx.setLineDash([4 * DPR, 3 * DPR]); ctx.beginPath(); let f = true;
        for (let i = i0; i <= cur; i += step) { const x = S.xf[i], y = S.yf[i]; if (!isFinite(x) || !isFinite(y)) { f = true; continue; } if (f) { ctx.moveTo(X(x), Y(y)); f = false; } else ctx.lineTo(X(x), Y(y)); }
        ctx.stroke(); ctx.setLineDash([]);
      }
      ctx.strokeStyle = "rgba(255,209,102,.9)"; ctx.lineWidth = 2.0 * DPR; ctx.beginPath(); let first = true;
      for (let i = i0; i <= cur; i += step) { const x = S.x[i], y = S.y[i]; if (!isFinite(x) || !isFinite(y)) { first = true; continue; } if (first) { ctx.moveTo(X(x), Y(y)); first = false; } else ctx.lineTo(X(x), Y(y)); }
      ctx.stroke();
      if (showSm && S.xs) {
        ctx.strokeStyle = "rgba(240,245,255,.75)"; ctx.lineWidth = 1.1 * DPR; ctx.setLineDash([3 * DPR, 3 * DPR]); ctx.beginPath(); first = true;
        for (let i = i0; i <= cur; i += step) { const x = S.xs[i], y = S.ys[i]; if (!isFinite(x) || !isFinite(y)) { first = true; continue; } if (first) { ctx.moveTo(X(x), Y(y)); first = false; } else ctx.lineTo(X(x), Y(y)); }
        ctx.stroke(); ctx.setLineDash([]);
        const xs = S.xs[cur], ys = S.ys[cur];
        if (isFinite(xs)) { ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 1.3 * DPR; ctx.beginPath(); ctx.arc(X(xs), Y(ys), 4.2 * DPR, 0, 7); ctx.stroke(); }
      }
      const px = X(S.x[cur]), py = Y(S.y[cur]);
      if (isFinite(px) && isFinite(py)) {
        const ph = LG.val("phase", cur) | 0;
        ctx.fillStyle = (ph === 1) ? "#ff006e" : (ph === 5 ? "#ff9f43" : "#ffd166");
        ctx.beginPath(); ctx.moveTo(px, py - 6 * DPR); ctx.lineTo(px + 6 * DPR, py); ctx.lineTo(px, py + 6 * DPR); ctx.lineTo(px - 6 * DPR, py); ctx.closePath(); ctx.fill();
      }
    }
    // 분석 오버레이 (놓기점·경계선)
    if (LG.showOverlay) for (const o of LG.planeOverlay) {
      if (o.plane !== kind) continue;
      if (o.kind === "points") {
        for (let k = 0; k < o.x.length; k++) {
          if (o.x[k] == null || o.y[k] == null) continue;
          const d = o.dir ? o.dir[k] : 0;
          ctx.fillStyle = d > 0 ? "#ff006e" : (d < 0 ? "#3a86ff" : "#ffffff");
          ctx.beginPath(); ctx.arc(X(o.x[k]), Y(o.y[k]), 4 * DPR, 0, 7); ctx.fill();
        }
      } else if (o.kind === "line") {
        ctx.strokeStyle = o.color || "#ff9f43"; ctx.lineWidth = 2 * DPR; ctx.beginPath();
        ctx.moveTo(X(o.x[0]), Y(o.y[0])); ctx.lineTo(X(o.x[1]), Y(o.y[1])); ctx.stroke();
      }
    }
    ctx.restore();
    ctx.strokeStyle = "rgba(255,255,255,.14)"; ctx.lineWidth = 1 * DPR; ctx.strokeRect(cx - R, cy - R, 2 * R, 2 * R);
    ctx.font = `${10.5 * DPR}px Consolas`; ctx.fillStyle = "rgba(255,255,255,.45)";
    ctx.fillText(`±${m.toFixed(1)}°`, cx + R - 42 * DPR, cy - R - 6 * DPR);
    ctx.fillText(kind === "pl1" ? "β" : "β_pred", cx + R - 40 * DPR, cy - 6 * DPR);
    ctx.fillText(kind === "pl1" ? "φ" : "φ_pred", cx + 6 * DPR, cy - R + 13 * DPR);
    if (V) ctx.fillText("🔍 수동 뷰 — 더블클릭 복귀", cx - R + 4 * DPR, cy - R + 13 * DPR + 13 * DPR);
    ctx.fillStyle = "#06d6a0";
    ctx.fillText(kind === "pl1" ? "초록 모델 φ=r·β  · 주황 실측 φ=r·β+c0" : `초록 A=0 · 노랑 트리거 ±${LG.trig}° · 주황 실측`, 6 * DPR, H - 8 * DPR);
    for (const o of LG.planeOverlay) if (o.plane === kind && o.label) { ctx.fillStyle = "#ff9f43"; ctx.fillText(o.label, 6 * DPR, H - 22 * DPR); break; }
  }
  // 줌·팬·복귀
  function attach(kind) {
    const cv = ctxs[kind].canvas;
    let pan = null;
    const toPlane = (e) => { const S = LG.PLSCALE[kind]; if (!S) return null; const r = cv.getBoundingClientRect(); const px = (e.clientX - r.left) * devicePixelRatio, py = (e.clientY - r.top) * devicePixelRatio; return { b: S.vb + (px - S.cx) / S.R * S.m, f: S.vf - (py - S.cy) / S.R * S.m }; };
    cv.addEventListener("wheel", e => {
      e.preventDefault(); const S = LG.PLSCALE[kind]; if (!S) return;
      const p = toPlane(e); const V = LG.PLVIEW[kind]; if (!V.on) { V.on = true; V.m = S.m; V.b = S.vb; V.f = S.vf; }
      const k = e.deltaY > 0 ? 1.15 : 1 / 1.15; V.b = p.b + (V.b - p.b) * k; V.f = p.f + (V.f - p.f) * k; V.m *= k;
    }, { passive: false });
    cv.addEventListener("contextmenu", e => e.preventDefault());
    cv.addEventListener("mousedown", e => { if (e.button === 2) { const S = LG.PLSCALE[kind]; if (!S) return; const V = LG.PLVIEW[kind]; if (!V.on) { V.on = true; V.m = S.m; V.b = S.vb; V.f = S.vf; } pan = { x: e.clientX, y: e.clientY, b: V.b, f: V.f, m: V.m, R: S.R }; } });
    window.addEventListener("mousemove", e => { if (!pan) return; const V = LG.PLVIEW[kind]; V.b = pan.b - (e.clientX - pan.x) * devicePixelRatio / pan.R * pan.m; V.f = pan.f + (e.clientY - pan.y) * devicePixelRatio / pan.R * pan.m; });
    window.addEventListener("mouseup", () => pan = null);
    cv.addEventListener("dblclick", () => { LG.PLVIEW[kind].on = false; });
  }
  attach("pl1"); attach("pl2");
  LG.renderPlanes = function () { draw("pl1"); draw("pl2"); };
})();
