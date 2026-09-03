// lg_3d.js — 3D 디지털 트윈 (v21 로봇 형상 그대로, 자세는 로그/실기 각도로 직접 구동 — 서버 왕복 없음)
"use strict";
(function () {
  const AXIS_Z = 1.245, PIV = 0.30, BARH = 0.05;
  const canvas = LG.el("c3d");
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
  const scene = new THREE.Scene(); scene.background = new THREE.Color(0x0d1017);
  const camera = new THREE.PerspectiveCamera(42, 2, 0.05, 200); camera.up.set(0, 0, 1);
  let az = 0.9, elv = 0.35, dist = 2.4;
  const target = new THREE.Vector3(0, 0, 0.85);
  function camUpdate() {
    camera.position.set(target.x + dist * Math.cos(elv) * Math.sin(az), target.y - dist * Math.cos(elv) * Math.cos(az), target.z + dist * Math.sin(elv));
    camera.lookAt(target);
  }
  camUpdate();
  scene.add(new THREE.AmbientLight(0xffffff, 0.55));
  const dl = new THREE.DirectionalLight(0xffffff, 0.8); dl.position.set(2, 1, 4); scene.add(dl);
  const grid = new THREE.GridHelper(6, 30, 0x2a3242, 0x1c2230); grid.rotation.x = Math.PI / 2; scene.add(grid);
  const mat = c => new THREE.MeshLambertMaterial({ color: c });
  const box = (w, d, h, c) => new THREE.Mesh(new THREE.BoxGeometry(w, d, h), mat(c));
  function rod(p1, p2, r, c) {
    const a = new THREE.Vector3(...p1), b = new THREE.Vector3(...p2), d = b.clone().sub(a), len = d.length();
    const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 10), mat(c));
    m.position.copy(a.clone().add(b).multiplyScalar(0.5));
    m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.normalize());
    return m;
  }
  const frameG = new THREE.Group();
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) { const p = box(0.02, 0.02, 1.22, 0x555a63); p.position.set(sx * 0.2, sy * 0.3, 0.61); frameG.add(p); }
  for (const sy of [-1, 1]) { const b1 = box(0.42, 0.02, 0.02, 0x555a63); b1.position.set(0, sy * 0.3, 1.22); frameG.add(b1); const b2 = box(0.42, 0.02, 0.02, 0x555a63); b2.position.set(0, sy * 0.3, 0.13); frameG.add(b2); }
  for (const sy of [-1, 1]) { const m = box(0.05, 0.03, 0.05, 0xb8bcc6); m.position.set(0, sy * PIV, AXIS_Z); frameG.add(m); }
  scene.add(frameG);
  let crankG = null, lowerG = null, upperG = null;
  function rebuild() {
    if (crankG) scene.remove(crankG);
    const g = LG.GEOM || { R: 0.433, L1: 0.259, L2: 0.375 };
    const R = g.R, L1 = g.L1, L2 = g.L2;
    crankG = new THREE.Group(); crankG.position.set(0, 0, AXIS_Z);
    crankG.add(rod([0, PIV, 0], [0, BARH, -R], 0.004, 0x23262b));
    crankG.add(rod([0, -PIV, 0], [0, -BARH, -R], 0.004, 0x23262b));
    crankG.add(rod([0, -BARH, -R], [0, BARH, -R], 0.006, 0xe6a817));
    for (const sy of [-1, 1]) { const s = new THREE.Mesh(new THREE.SphereGeometry(0.013, 12, 8), mat(0x4c9fe8)); s.position.set(0, sy * PIV, -0.02); crankG.add(s); }
    lowerG = new THREE.Group(); lowerG.position.set(0, 0, -R);
    const lb = box(0.038, 0.080, L1, 0x06b98c); lb.position.set(0, 0, L1 / 2); lowerG.add(lb);
    const hip = new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.016, 0.13, 14), mat(0xffbe0b)); hip.position.set(0, 0, L1); lowerG.add(hip);
    upperG = new THREE.Group(); upperG.position.set(0, 0, L1);
    const ub = box(0.038, 0.090, L2, 0x7a35d6); ub.position.set(0, 0, L2 / 2); upperG.add(ub);
    const hd = box(0.038, 0.098, 0.046, 0xf2c14e); hd.position.set(0, 0, L2 + 0.023); upperG.add(hd);
    const imu = new THREE.Mesh(new THREE.SphereGeometry(0.009, 10, 8), mat(0xff2d2d)); imu.position.set(0, 0, 0.2005); upperG.add(imu);
    lowerG.add(upperG); crankG.add(lowerG); scene.add(crankG);
  }
  rebuild();
  LG.on("hello", rebuild);
  // 마우스: 좌드래그 회전 · 우드래그/Shift 이동 · 휠 줌
  let drag = false, btn = 0, lx = 0, ly = 0;
  canvas.addEventListener("mousedown", e => { drag = true; btn = e.button; lx = e.clientX; ly = e.clientY; });
  window.addEventListener("mouseup", () => drag = false);
  canvas.addEventListener("contextmenu", e => e.preventDefault());
  window.addEventListener("mousemove", e => {
    if (!drag) return;
    const dx = e.clientX - lx, dy = e.clientY - ly;
    if (btn === 2 || btn === 1 || e.shiftKey) {
      const s = dist * 0.0011;
      const right = new THREE.Vector3().setFromMatrixColumn(camera.matrix, 0), upv = new THREE.Vector3().setFromMatrixColumn(camera.matrix, 1);
      target.addScaledVector(right, -dx * s); target.addScaledVector(upv, dy * s);
    } else { az -= dx * 0.008; elv = Math.min(1.5, Math.max(-0.3, elv + dy * 0.008)); }
    lx = e.clientX; ly = e.clientY; camUpdate();
  });
  canvas.addEventListener("wheel", e => { e.preventDefault(); dist = Math.min(20, Math.max(0.5, dist * (e.deltaY > 0 ? 1.025 : 0.976))); camUpdate(); }, { passive: false });
  LG.cam = {
    side() { az = 0; elv = 0.02; dist = 2.2; target.set(0, 0, 0.85); camUpdate(); },
    iso() { az = 0.9; elv = 0.35; dist = 2.4; target.set(0, 0, 0.85); camUpdate(); },
  };
  const RAD = Math.PI / 180;
  LG.render3d = function () {
    const w = canvas.clientWidth * devicePixelRatio, h = canvas.clientHeight * devicePixelRatio;
    if (!w || !h) return;
    if (canvas.width !== w || canvas.height !== h) { renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix(); }
    const i = LG.cur();
    if (i >= 0) {
      const p = LG.poseAt(i);
      // MuJoCo qpos 규약: crank = −φ, lower(발목) = α + φ, upper(힙) = θ − α  (sim_engine.set_pose 와 동일)
      crankG.rotation.y = -p.phi * RAD; lowerG.rotation.y = (p.alpha + p.phi) * RAD; upperG.rotation.y = (p.theta - p.alpha) * RAD;
    }
    renderer.render(scene, camera);
  };
})();
