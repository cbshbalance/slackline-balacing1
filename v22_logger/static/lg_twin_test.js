// v22 hanging hip test: real serial commands; twin remains driven by telemetry.
(() => {
  "use strict";
  const L = window.LG;
  if (!L || document.getElementById("hipTest")) return;
  const box = document.createElement("div");
  box.id = "hipTest";
  box.style.cssText = "position:fixed;bottom:12px;left:12px;z-index:80;background:#152331;color:#eef5ff;padding:12px;border:1px solid #53718b;border-radius:8px;max-width:420px;font-size:13px";
  box.innerHTML = '<b>매달림 · 허리 왕복 테스트</b><div style="margin:6px 0">시작 위치를 δ=0으로 설정 · 토크 ON</div><div style="margin:6px 0"><label>진폭 ±<input id="hipAmplitude" type="number" min="1" max="40" step="1" value="10" style="width:60px">°</label> <label>전환 간격 <input id="hipInterval" type="number" min="0.5" max="10" step="0.1" value="2" style="width:65px">초</label><br><small>±20°는 양 끝 사이 40° · 1초 간격이면 한 왕복 2초</small></div><button class="btn accent" id="hipStart">왕복 시작</button> <button class="btn" id="hipStop">반복 정지</button> <button class="btn" id="hipOff">비상 · 토크 OFF</button><div id="hipStatus" style="margin-top:6px">대기 — 정지 시 마지막 목표 유지</div>';
  document.body.appendChild(box);
  const start = document.getElementById("hipStart"), stop = document.getElementById("hipStop");
  const status = document.getElementById("hipStatus");
  const originalSend = L.send.bind(L);
  let state = "idle", lastData = -Infinity, deadline = 0, due = 0, sign = 1;
  let ack = false, armedAt = 0, amplitude = 10, intervalMs = 2000;
  const amplitudeInput = document.getElementById("hipAmplitude"), intervalInput = document.getElementById("hipInterval");
  const now = () => performance.now();
  const connected = () => L.wsOK && L.link.connected && L.link.src.kind === "serial";
  const valid = () => {
    const i = L.ds.n - 1, err = L.val("err", i);
    return connected() && L.profile === "v22_raw" && now() - lastData < 1500 &&
      Number.isFinite(L.val("del", i)) && Number.isFinite(err) && !(err & 16);
  };
  function send(text) { originalSend({cmd: "send", text}); }
  function finish(reason, hold = true) {
    const wasActive = state !== "idle";
    state = "idle";
    if (wasActive && hold && connected()) send("h");
    status.textContent = reason;
    start.disabled = false; stop.disabled = true;
    amplitudeInput.disabled = intervalInput.disabled = false;
  }
  function step() {
    send(String(sign * amplitude));
    status.textContent = `왕복 중 · 목표 ${sign > 0 ? "+" : "−"}${amplitude}° · ${intervalMs / 1000}초 간격`;
    sign *= -1;
    due = now() + intervalMs;
  }
  start.onclick = () => {
    if (state !== "idle") return;
    if (!valid()) { L.toast("v22_raw 실물 연결과 최신 모터 측정값이 필요합니다.", true); return; }
    const a = Number(amplitudeInput.value), seconds = Number(intervalInput.value);
    if (!Number.isInteger(a) || a < 1 || a > 40 || !Number.isFinite(seconds) || seconds < 0.5 || seconds > 10) {
      L.toast("진폭은 1~40° 정수, 전환 간격은 0.5~10초로 입력하세요.", true); return;
    }
    amplitude = a; intervalMs = Math.round(seconds * 1000);
    amplitudeInput.disabled = intervalInput.disabled = true;
    L.follow = true; L.cursor = -1; L.playing = false;
    state = "arming"; ack = false; sign = 1;
    armedAt = now(); deadline = armedAt + 3000;
    start.disabled = true; stop.disabled = false;
    status.textContent = "시작 위치 설정·토크 ON 확인 중…";
    send("h\nk");
  };
  stop.onclick = () => finish("반복 정지 · 마지막 목표 유지");
  document.getElementById("hipOff").onclick = () => {
    finish("비상정지 · 토크 OFF 요청", false);
    if (connected()) send("x");
  };
  // Cancel before forwarding another control, disconnect or data-source change.
  L.send = function (m) {
    if (state !== "idle" && ["send", "disconnect", "connect", "fake", "mujoco", "robot", "load", "load_text", "clear", "profile", "scene"].includes(m.cmd)) {
      finish("다른 명령으로 왕복 취소", m.cmd !== "send");
    }
    return originalSend(m);
  };
  L.on("ds_append", () => { lastData = now(); });
  L.on("ds_full", () => { lastData = -Infinity; if (state !== "idle") finish("데이터 교체로 왕복 정지"); });
  L.on("console", lines => {
    if (state === "arming" && lines.some(line => String(line[1]).includes("# 토크 ON (현재 위치 유지"))) ack = true;
  });
  L.on("ws", ok => { if (!ok) finish("서버 연결 끊김 · 반복 종료", false); });
  L.on("link", () => { if (state !== "idle" && !connected()) finish("장치 연결 끊김 · 반복 종료", false); });
  L.on("error", () => { if (state !== "idle") finish("서버 오류로 왕복 정지"); });
  setInterval(() => {
    if (state === "idle") return;
    if (!valid()) { finish("측정값 또는 연결 이상 · 반복 정지"); return; }
    if (state === "arming") {
      if (now() >= deadline) { finish("토크 ON 확인 시간 초과 · 반복 정지"); return; }
      if (ack && lastData > armedAt) { state = "running"; step(); }
    } else if (now() >= due) step();
  }, 50);
  document.addEventListener("visibilitychange", () => { if (document.hidden && state !== "idle") finish("화면을 떠나 왕복 정지"); });
  window.addEventListener("pagehide", () => finish("페이지 종료 · 반복 정지"));
  stop.disabled = true;
})();
