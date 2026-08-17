/* =====================================================================
 * model.js — 줄타기 (φ, β, δ) 선형 모델 + 궤적 베이커
 * ---------------------------------------------------------------------
 * v19_bringup/model_v19.py 의 JS 이식. 하드코딩된 파생값 없음 —
 * 파라미터를 주면 M, G, D, μ, 모드, λ_u, w, 평면 상수를 전부 런타임 계산한다.
 *
 * 검산 (params SL.PARAMS.robot_meas 기준, B_THETA=0):
 *   λ_u = 5.9219,  A=0 선 기울기 = −1.6451,  CoM 등고선 기울기 = −0.8889,
 *   접기 1°당 (Δβ, Δφ) = (−0.3517°, +0.3126°)
 *   → v19_state/README_v19_state.md 「평면 상수」 빨강 버튼 열과 일치
 * ===================================================================== */
var SL = SL || {};
(function () {
'use strict';

var G_GRAV = 9.81;

/* ---------- 파라미터 세트 ---------------------------------------- */
SL.PARAMS = {
  // 실측 정본 (params_v19.py MEASURED_SET, 2026-07-30~08-06)
  // B_THETA = 0 : v19_state 실측 버튼과 같은 설정 (미실측 항목이라 0으로 둔다)
  robot_meas: {
    name: '로봇 · 실측정본', badge: null,
    m1: 0.1094, m2: 0.6406, L1: 0.259, L2: 0.375,
    ell1: 0.084, ell2: 0.1917, I1_cm: 8.85e-4, I2_cm: 2.116e-2,
    I_r: 4.50e-3, S_r: 1.0835e-2, c_phi: 3.33e-4, B_THETA: 0.0,
    R: 0.433, D_span: 0.60,
    T_FOLD: 0.06, T_WAIT: 0.02, T_EXTEND: 0.09, T_REST: 0.075,
    A_TRIG_DEG: 0.6, RHO: 0.9, DELTA_MAX_DEG: 55
  },
  // 사람 스케일 (이론부 표 2-1)
  human: {
    name: '사람 스케일', badge: '이론',
    m1: 30.0, m2: 45.0, L1: 0.85, L2: 0.85,
    ell1: 0.30, ell2: 0.40, I1_cm: 30.0 * 0.85 * 0.85 / 12, I2_cm: 45.0 * 0.85 * 0.85 / 12,
    I_r: 0.0, S_r: 0.0, c_phi: 0.0, B_THETA: 0.0,
    R: 0.70, D_span: 4.0,
    T_FOLD: 0.25, T_WAIT: 0.30, T_EXTEND: 0.15, T_REST: 0.0,
    A_TRIG_DEG: 1.0, RHO: 0.9, DELTA_MAX_DEG: 60
  }
};

/* ---------- 작은 선형대수 ----------------------------------------- */
function inv2(A) {
  var d = A[0][0] * A[1][1] - A[0][1] * A[1][0];
  return [[A[1][1] / d, -A[0][1] / d], [-A[1][0] / d, A[0][0] / d]];
}
function mul2(A, B) {
  return [[A[0][0] * B[0][0] + A[0][1] * B[1][0], A[0][0] * B[0][1] + A[0][1] * B[1][1]],
          [A[1][0] * B[0][0] + A[1][1] * B[1][0], A[1][0] * B[0][1] + A[1][1] * B[1][1]]];
}
function mul3(A, B) {
  var C = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], i, j, k;
  for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) { var s = 0;
    for (k = 0; k < 3; k++) s += A[i][k] * B[k][j]; C[i][j] = s; }
  return C;
}
function tr3(A) { return [[A[0][0], A[1][0], A[2][0]], [A[0][1], A[1][1], A[2][1]], [A[0][2], A[1][2], A[2][2]]]; }

/* 2×2 대칭이 아닌 일반 행렬의 고유값·고유벡터 (실수근 전제) */
function eig2(A) {
  var tr = A[0][0] + A[1][1], det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
  var disc = Math.sqrt(Math.max(0, tr * tr / 4 - det));
  var l1 = tr / 2 - disc, l2 = tr / 2 + disc;      // l1 < l2
  function vec(l) {                                 // (A − lI)v = 0
    var a = A[0][0] - l, b = A[0][1], c = A[1][0], d = A[1][1] - l;
    var v = (Math.abs(b) + Math.abs(a) > Math.abs(c) + Math.abs(d)) ? [-b, a] : [-d, c];
    var n = Math.hypot(v[0], v[1]); return [v[0] / n, v[1] / n];
  }
  return { lam: [l1, l2], vec: [vec(l1), vec(l2)] };
}

/* ---------- 플랜트 -------------------------------------------------- */
SL.build = function (p) {
  var Mt = p.m1 + p.m2;
  var p1 = p.m1 * p.ell1 + p.m2 * p.L1;
  var p2 = p.m2 * p.ell2;
  var p3 = p.m2 * p.L1 * p.ell2;
  var ps = p1 + p2, h = ps / Mt, g = G_GRAV, R = p.R;

  var M = [[Mt * R * R + p.I_r, R * p1, R * p2],
           [R * p1, p.m1 * p.ell1 * p.ell1 + p.I1_cm + p.m2 * p.L1 * p.L1, p3],
           [R * p2, p3, p.m2 * p.ell2 * p.ell2 + p.I2_cm]];
  var G = [[-(Mt * R + p.S_r) * g, 0, 0], [0, p1 * g, 0], [0, 0, p2 * g]];
  var D = [[-p.c_phi, 0, 0], [0, 0, 0], [0, 0, -p.B_THETA]];

  // (φ,α,θ) = J (φ,β,δ)
  var J = [[1, 0, 0], [0, 1, -p2 / ps], [0, 1, p1 / ps]];
  var Jt = tr3(J);
  var Mp = mul3(Jt, mul3(M, J)), Gp = mul3(Jt, mul3(G, J)), Dp = mul3(Jt, mul3(D, J));

  // 구조 검산 (이론): 접기 δ̈ 는 φ행에 안 들어오고, 접힌 자세 중력은 (φ,β)행에 안 들어온다
  var chk = { Mphidelta: Mp[0][2] / Mp[0][0], Gphidelta: Gp[0][2], Gbetadelta: Gp[1][2] };

  var M2 = [[Mp[0][0], Mp[0][1]], [Mp[1][0], Mp[1][1]]];
  var G2 = [[Gp[0][0], Gp[0][1]], [Gp[1][0], Gp[1][1]]];
  var D2 = [[Dp[0][0], Dp[0][1]], [Dp[1][0], Dp[1][1]]];
  var mu = Mp[1][2];

  var Mi2 = inv2(M2);
  var A2 = mul2(Mi2, G2);       // q̈ = A2 q + Dd q̇ + b δ̈
  var Dd = mul2(Mi2, D2);
  var bvec = [Mi2[0][1] * (-mu), Mi2[1][1] * (-mu)];   // δ̈ 입력 (β행에만 μ)

  // 무감쇠 모드
  var e = eig2(A2);
  var iu = e.lam[1] > e.lam[0] ? 1 : 0, is = 1 - iu;
  var omega_eff = Math.sqrt(Math.max(0, -e.lam[is]));
  var lam_eff = Math.sqrt(Math.max(0, e.lam[iu]));
  var r = e.vec[is][0] / e.vec[is][1];               // 안정모드 φ/β 비

  /* 감쇠 포함 불안정 λ_u : det(λ²I − λDd − A2) = 0 의 양의 실근 */
  function detf(l) {
    var a = l * l - l * Dd[0][0] - A2[0][0], b = -l * Dd[0][1] - A2[0][1];
    var c = -l * Dd[1][0] - A2[1][0], d = l * l - l * Dd[1][1] - A2[1][1];
    return a * d - b * c;
  }
  var lo = lam_eff * 0.5, hi = lam_eff * 1.6, i;
  if (detf(lo) * detf(hi) > 0) { lo = 1e-4; hi = lam_eff * 4; }
  for (i = 0; i < 200; i++) { var mid = 0.5 * (lo + hi);
    if (detf(lo) * detf(mid) <= 0) hi = mid; else lo = mid; }
  var lam_u = 0.5 * (lo + hi);

  // 좌고유벡터: w_v (N = λ²I − λDd − A2 의 좌영벡터), w_q = w_v (λI − Dd)
  var N = [[lam_u * lam_u - lam_u * Dd[0][0] - A2[0][0], -lam_u * Dd[0][1] - A2[0][1]],
           [-lam_u * Dd[1][0] - A2[1][0], lam_u * lam_u - lam_u * Dd[1][1] - A2[1][1]]];
  // w_v N = 0  ⇔  Nᵀ w_vᵀ = 0
  var wv = (Math.abs(N[0][0]) + Math.abs(N[1][0]) > Math.abs(N[0][1]) + Math.abs(N[1][1]))
         ? [N[1][0], -N[0][0]] : [N[1][1], -N[0][1]];
  var Pm = inv2([[lam_u - Dd[0][0], -Dd[0][1]], [-Dd[1][0], lam_u - Dd[1][1]]]); // (λI−Dd)⁻¹
  var Lm = [[lam_u - Dd[0][0], -Dd[0][1]], [-Dd[1][0], lam_u - Dd[1][1]]];
  var wq = [wv[0] * Lm[0][0] + wv[1] * Lm[1][0], wv[0] * Lm[0][1] + wv[1] * Lm[1][1]];
  var nrm = wq[1]; wq = [wq[0] / nrm, wq[1] / nrm]; wv = [wv[0] / nrm, wv[1] / nrm];

  /* 접기 기하 */
  var s = M2[0][1] / M2[0][0];                       // Δφ = s·Δ(발밀침)
  var detM2 = M2[0][0] * M2[1][1] - M2[0][1] * M2[1][0];
  var k = mu * M2[0][0] / detM2;                      // Δq = k δ_f (s, −1)

  var m = {
    p: p, Mt: Mt, p1: p1, p2: p2, ps: ps, h: h, R: R, g: g,
    M2: M2, G2: G2, D2: D2, Mi2: Mi2, A2: A2, Dd: Dd, b: bvec, mu: mu,
    omega_eff: omega_eff, lam_eff: lam_eff, r: r, lam_u: lam_u,
    wq: wq, wv: wv, Ppred: Pm, s: s, k: k, check: chk,
    T_osc: 2 * Math.PI / omega_eff,
    slopeA0: -wq[1] / wq[0],          // φ_pred = slopeA0 · β_pred
    slopeCoM: -s,                     // 접기가 미는 방향 (β 가로, φ 세로)
    eps_star: (-wq[1] / (wq[0] * s - wq[1])) - 1
  };
  m.A = function (x) { // x = [φ, β, φ̇, β̇]  →  A [rad, β환산]
    return wq[0] * x[0] + wq[1] * x[1] + wv[0] * x[2] + wv[1] * x[3];
  };
  m.pred = function (x) { // 예측점 (φ_pred, β_pred)
    return [x[0] + Pm[0][0] * x[2] + Pm[0][1] * x[3],
            x[1] + Pm[1][0] * x[2] + Pm[1][1] * x[3]];
  };
  m.deltaStar = function (x) {  // 지금 A=0 으로 보내는 접기각 [rad]
    return -m.A(x) / (k * (wq[0] * s - wq[1]));
  };

  /* 불안정 우고유벡터 v_u = [v_q, λ v_q] — 상태의 불안정 성분을 정확히 뽑아낸다 */
  var vq = (Math.abs(N[0][0]) + Math.abs(N[0][1]) > Math.abs(N[1][0]) + Math.abs(N[1][1]))
         ? [N[0][1], -N[0][0]] : [N[1][1], -N[1][0]];
  var vu = [vq[0], vq[1], lam_u * vq[0], lam_u * vq[1]];
  var wvu = wq[0] * vu[0] + wq[1] * vu[1] + wv[0] * vu[2] + wv[1] * vu[3];
  m.vu = vu; m.w_dot_vu = wvu;
  /* 불안정 성분을 0으로 — 이론의 '안정모드 정렬' 이상화 (수치 잔차 제거) */
  m.projectStable = function (x) {
    var c = m.A(x) / wvu;
    return [x[0] - c * vu[0], x[1] - c * vu[1], x[2] - c * vu[2], x[3] - c * vu[3]];
  };
  /* 초기상태 만들기: 시각 tAt 에서 A = A_at [rad] 이 되도록, 안정 진동 osc 를 얹어서 */
  m.makeX0 = function (A_at, tAt, osc) {
    osc = osc || 0;
    var xs = [m.r * osc, osc, 0, 0];              // 안정모드 방향 위치 오프셋
    var A_target = A_at * Math.exp(-lam_u * (tAt || 0));
    var cu = (A_target - m.A(xs)) / wvu;
    return [xs[0] + cu * vu[0], xs[1] + cu * vu[1], xs[2] + cu * vu[2], xs[3] + cu * vu[3]];
  };

  SL.calibrateFWE(m, 2e-5);
  return m;
};

/* ---------- FWE 맵 수치 캘리브레이션 -------------------------------- */
function triProfile(t, T, dmax) {   // 삼각 가감속: (δ, δ̇, δ̈)
  if (t <= 0) return [0, 0, 0];
  if (t >= T) return [dmax, 0, 0];
  var a = 4 * dmax / (T * T);
  if (t < T / 2) return [0.5 * a * t * t, a * t, a];
  var tr = T - t; return [dmax - 0.5 * a * tr * tr, a * tr, -a];
}
SL.triProfile = triProfile;

/* 4D RK4 한 스텝. x = [φ, β, φ̇, β̇], ddel = δ̈ */
function step4(m, x, ddel, dt) {
  var A2 = m.A2, Dd = m.Dd, b = m.b;
  function f(y) {
    return [y[2], y[3],
      A2[0][0] * y[0] + A2[0][1] * y[1] + Dd[0][0] * y[2] + Dd[0][1] * y[3] + b[0] * ddel,
      A2[1][0] * y[0] + A2[1][1] * y[1] + Dd[1][0] * y[2] + Dd[1][1] * y[3] + b[1] * ddel];
  }
  var k1 = f(x);
  var k2 = f([x[0] + dt / 2 * k1[0], x[1] + dt / 2 * k1[1], x[2] + dt / 2 * k1[2], x[3] + dt / 2 * k1[3]]);
  var k3 = f([x[0] + dt / 2 * k2[0], x[1] + dt / 2 * k2[1], x[2] + dt / 2 * k2[2], x[3] + dt / 2 * k2[3]]);
  var k4 = f([x[0] + dt * k3[0], x[1] + dt * k3[1], x[2] + dt * k3[2], x[3] + dt * k3[3]]);
  return [0, 1, 2, 3].map(function (i) {
    return x[i] + dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
  });
}
SL.step4 = step4;

/* γ 는 '실행 눈금'에서 잰다 — 적분 스텝이 다르면 다시 잰다(문서 16의 원칙).
   같은 dt 로 캘리브레이션하지 않으면 deadbeat 잔차가 e^{λt} 로 자라 그림이 무너진다. */
SL.calibrateFWE = function (m, dt) {
  var p = m.p, x, n, i, t;
  dt = dt || 2e-5;
  if (m._calDt === dt) return m;
  m._calDt = dt;
  // fold-hold: A⁻=0, δf=1rad → g_fold
  x = [0, 0, 0, 0]; n = Math.round(p.T_FOLD / dt);
  for (i = 0; i < n; i++) { t = (i + 0.5) * dt; x = step4(m, x, triProfile(t, p.T_FOLD, 1)[2], dt); }
  var g_fold = -m.A(x);
  m.G_fold = Math.exp(m.lam_u * p.T_FOLD);
  m.g_fold = g_fold;
  m.GAMMA_FOLD = m.G_fold / g_fold;
  // full cycle
  x = [0, 0, 0, 0];
  n = Math.round(p.T_FOLD / dt);
  for (i = 0; i < n; i++) { t = (i + 0.5) * dt; x = step4(m, x, triProfile(t, p.T_FOLD, 1)[2], dt); }
  n = Math.round(p.T_WAIT / dt);
  for (i = 0; i < n; i++) x = step4(m, x, 0, dt);
  n = Math.round(p.T_EXTEND / dt);
  for (i = 0; i < n; i++) { t = (i + 0.5) * dt; x = step4(m, x, -triProfile(t, p.T_EXTEND, 1)[2], dt); }
  m.g_cyc = -m.A(x);
  m.G_cyc = Math.exp(m.lam_u * (p.T_FOLD + p.T_WAIT + p.T_EXTEND + p.T_REST));
  m.GAMMA_CYCLE = m.G_cyc / m.g_cyc;
  return m;
};

/* =====================================================================
 * 궤적 베이커 — 발표 앱의 핵심.
 * 물리는 여기서 한 번만 돌고, 화면은 결과 배열을 시간으로 훑는다(scrub).
 * 반환: {dt, n, phi[], beta[], dphi[], dbeta[], delta[], A[], phase[]}
 *   phase: 0 자유 / 1 접기 / 2 대기 / 3 펴기 / 4 휴지·복귀
 * ===================================================================== */
SL.bake = function (m, opts) {
  var dt = opts.dt || 5e-4, T = opts.T || 6.0, n = Math.round(T / dt);
  SL.calibrateFWE(m, dt);                       // ★ 실행 눈금에서 γ 재측정
  var startAt0 = opts.startAt !== undefined ? opts.startAt : 0.35;
  var x = opts.x0 ? opts.x0.slice()
        : m.makeX0((opts.A_at !== undefined ? opts.A_at : 0.6 * Math.PI / 180),
                   startAt0, opts.osc !== undefined ? opts.osc : 0.4 * Math.PI / 180);
  var out = { dt: dt, n: n + 1, T: T, phi: new Float64Array(n + 1), beta: new Float64Array(n + 1),
              dphi: new Float64Array(n + 1), dbeta: new Float64Array(n + 1),
              delta: new Float64Array(n + 1), A: new Float64Array(n + 1),
              phase: new Uint8Array(n + 1), events: [] };
  var mode = opts.mode || 'free';
  var p = m.p;
  var delta = 0, ddel = 0, phase = 0;
  var st = 'IDLE', st_t = 0, foldAmp = 0, foldFrom = 0, cycles = 0;
  var tauRet = opts.tauRet || 1.2;       // 느린 복귀 시정수 [s] (펴기 생략 전략)
  var trig = (p.A_TRIG_DEG || 0.6) * Math.PI / 180;
  var rho = opts.rho !== undefined ? opts.rho : p.RHO;
  var dmax = (p.DELTA_MAX_DEG || 55) * Math.PI / 180;
  var startAt = startAt0;
  var exact = opts.deadbeat === 'exact';   // 접기 종료 시 불안정 성분을 정확히 0 (이론 이상화)

  for (var i = 0; i <= n; i++) {
    var t = i * dt;
    out.phi[i] = x[0]; out.beta[i] = x[1]; out.dphi[i] = x[2]; out.dbeta[i] = x[3];
    out.delta[i] = delta; out.A[i] = m.A(x); out.phase[i] = phase;
    if (i === n) break;

    var tm = t + 0.5 * dt;                       // 스텝 중점에서 δ̈ 샘플 (문서 42 (d))
    ddel = 0;

    if (mode === 'free') { phase = 0; }

    else if (mode === 'fold') {                  // ε*(=γ_fold) 단일 접기 후 유지
      if (st === 'IDLE' && t >= startAt) {
        foldAmp = m.GAMMA_FOLD * m.A(x); foldFrom = delta;
        st = 'FOLD'; st_t = t; out.events.push({ t: t, kind: 'fold', amp: foldAmp });
      }
      if (st === 'FOLD') {
        var lf = tm - st_t;
        if (lf >= p.T_FOLD) {
          st = 'HOLD'; delta = foldFrom + foldAmp; phase = 0;
          if (exact) x = m.projectStable(x);
        }
        else { var pr = triProfile(lf, p.T_FOLD, foldAmp); delta = foldFrom + pr[0]; ddel = pr[2]; phase = 1; }
      }
    }

    else if (mode === 'cycle') {                 // 완전 사이클 접기-대기-펴기
      if (st === 'IDLE' && t >= startAt) {
        foldAmp = m.GAMMA_CYCLE * m.A(x); foldFrom = delta;
        st = 'FOLD'; st_t = t; out.events.push({ t: t, kind: 'fold', amp: foldAmp });
      }
      if (st === 'FOLD') {
        var l1 = tm - st_t;
        if (l1 >= p.T_FOLD) { st = 'WAIT'; st_t = st_t + p.T_FOLD; delta = foldFrom + foldAmp; }
        else { var q1 = triProfile(l1, p.T_FOLD, foldAmp); delta = foldFrom + q1[0]; ddel = q1[2]; phase = 1; }
      }
      if (st === 'WAIT') {
        if (tm - st_t >= p.T_WAIT) { st = 'EXT'; st_t = st_t + p.T_WAIT; out.events.push({ t: t, kind: 'extend' }); }
        else phase = 2;
      }
      if (st === 'EXT') {
        var l3 = tm - st_t;
        if (l3 >= p.T_EXTEND) {
          st = 'DONE'; delta = foldFrom; phase = 0;
          if (exact) x = m.projectStable(x);
        }
        else { var q3 = triProfile(l3, p.T_EXTEND, foldAmp); delta = foldFrom + foldAmp - q3[0]; ddel = -q3[2]; phase = 3; }
      }
    }

    else if (mode === 'incr') {                  // 증분 접기 + 느린 복귀 (펴기 생략)
      if (st === 'IDLE') {
        if (Math.abs(m.A(x)) > trig && t > startAt) {
          var want = rho * m.GAMMA_FOLD * m.A(x);
          var tgt = Math.max(-dmax, Math.min(dmax, delta + want));
          foldAmp = tgt - delta; foldFrom = delta;
          st = 'FOLD'; st_t = t; cycles++;
          out.events.push({ t: t, kind: 'fold', amp: foldAmp, A: m.A(x) });
        } else {
          ddel = delta / (tauRet * tauRet); var dd = -delta / tauRet;   // δ = δ0 e^(−t/τ)
          delta += dd * dt; phase = 4;
          if (Math.abs(delta) < 1e-4) phase = 0;
        }
      }
      if (st === 'FOLD') {
        var lf2 = tm - st_t;
        if (lf2 >= p.T_FOLD) { st = 'REST'; st_t = st_t + p.T_FOLD; delta = foldFrom + foldAmp; }
        else { var q2 = triProfile(lf2, p.T_FOLD, foldAmp); delta = foldFrom + q2[0]; ddel = q2[2]; phase = 1; }
      }
      if (st === 'REST') {
        phase = 4;
        if (tm - st_t >= p.T_REST) st = 'IDLE';
      }
    }

    x = step4(m, x, ddel, dt);
    if (Math.abs(x[1]) > 1.2) { // 낙하 — 남은 구간을 마지막 값으로 채운다
      for (var j = i + 1; j <= n; j++) {
        out.phi[j] = x[0]; out.beta[j] = x[1]; out.delta[j] = delta;
        out.A[j] = m.A(x); out.phase[j] = 5;
      }
      out.fellAt = t; break;
    }
  }
  out.cycles = cycles;
  return out;
};

/* 궤적을 임의 시각에서 선형보간해 상태를 뽑는다 (스크럽의 핵심) */
SL.sample = function (tr, t) {
  var f = Math.max(0, Math.min(tr.n - 1.001, t / tr.dt));
  var i = Math.floor(f), a = f - i, j = i + 1;
  function L(arr) { return arr[i] * (1 - a) + arr[j] * a; }
  return { phi: L(tr.phi), beta: L(tr.beta), dphi: L(tr.dphi), dbeta: L(tr.dbeta),
           delta: L(tr.delta), A: L(tr.A), phase: tr.phase[i], t: t };
};

/* (φ,β,δ) → (φ,α,θ) */
SL.toPAT = function (m, phi, beta, delta) {
  return { phi: phi, alpha: beta - (m.p2 / m.ps) * delta, theta: beta + (m.p1 / m.ps) * delta };
};

})();
