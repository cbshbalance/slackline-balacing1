/* =====================================================================
 * render.mjs — 발표 앱을 프레임 단위로 훑어 mp4 로 굽는다
 * ---------------------------------------------------------------------
 * 앱이 노출하는 계약은 window.__seek(t) 하나뿐이다. 실시간 재생을 하지 않고
 * t = n/fps 를 직접 넣으므로 컴퓨터 성능과 무관하게 항상 같은 그림이 나온다.
 *
 *   node render.mjs                          # 전체, 1280×720, 24fps
 *   node render.mjs --w 1920 --h 1080 --fps 30
 *   node render.mjs --from 90 --to 140 --out s6.mp4
 *   node render.mjs --ui                     # 조작 UI 를 켠 채로 (설명용)
 *
 * 필요: playwright(크로미움), ffmpeg
 * ===================================================================== */
import { spawn, execSync } from 'node:child_process';
import { mkdir, rm } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createRequire } from 'node:module';
const HERE = path.dirname(fileURLToPath(import.meta.url));

/* playwright 를 로컬 → 전역 순서로 찾는다 (npm i -D playwright 든 npm i -g 든 동작) */
const req = createRequire(import.meta.url);
let chromium;
try { ({ chromium } = req('playwright')); }
catch {
  try { ({ chromium } = req(path.join(execSync('npm root -g').toString().trim(), 'playwright'))); }
  catch { console.error('playwright 가 없습니다:  npm i -g playwright  후 다시 실행하세요.'); process.exit(1); }
}

/* ---- 인자 ---- */
const A = {};
for (let i = 2; i < process.argv.length; i++) {
  const a = process.argv[i];
  if (a.startsWith('--')) {
    const k = a.slice(2);
    const v = process.argv[i + 1];
    if (v === undefined || v.startsWith('--')) A[k] = true; else { A[k] = v; i++; }
  }
}
const W = +(A.w || 1280), H = +(A.h || 720), FPS = +(A.fps || 24);
const OUT = A.out || 'rough_cut.mp4';
const SHOW_UI = !!A.ui;
const TMP = path.join(HERE, '.frames');

/* ---- 브라우저 ---- */
const browser = await chromium.launch({
  args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader',
         '--hide-scrollbars', '--force-device-scale-factor=1']
});
const page = await browser.newPage({ viewport: { width: W, height: H }, deviceScaleFactor: 1 });
const errors = [];
page.on('pageerror', e => errors.push(e.message));
page.on('console', m => { if (m.type() === 'error') errors.push(m.text()); });
await page.goto('file://' + path.join(HERE, 'index.html'));
await page.waitForFunction(() => typeof window.__seek === 'function', null, { timeout: 30000 });
await page.waitForTimeout(1200);                 // 궤적 베이크 + 첫 렌더

if (!SHOW_UI) {
  await page.evaluate(() => {
    document.body.classList.add('hideui');
    ['#note'].forEach(s => { const e = document.querySelector(s); if (e) e.style.display = 'none'; });
  });
  await page.waitForTimeout(300);
}
const total = await page.evaluate(() => window.__total);
const scenes = await page.evaluate(() => window.__scenes);
const t0 = A.from !== undefined ? +A.from : 0;
const t1 = A.to !== undefined ? +A.to : total;
const n0 = Math.round(t0 * FPS), n1 = Math.round(t1 * FPS);
const N = n1 - n0;

console.log(`씬 ${scenes.length}개 · 전체 ${total.toFixed(1)}s`);
scenes.forEach(s => console.log(`  ${s.id} ${s.name.padEnd(12)} ${s.t0.toFixed(1)}s ~ ${(s.t0 + s.dur).toFixed(1)}s`));
console.log(`\n렌더 ${t0.toFixed(1)}~${t1.toFixed(1)}s · ${W}×${H} · ${FPS}fps · ${N} 프레임 → ${OUT}`);

await rm(TMP, { recursive: true, force: true });
await mkdir(TMP, { recursive: true });

const started = Date.now();
for (let n = n0; n < n1; n++) {
  const t = n / FPS;
  await page.evaluate(tt => window.__seek(tt), t);
  await page.screenshot({
    path: path.join(TMP, 'f' + String(n - n0).padStart(6, '0') + '.jpg'),
    type: 'jpeg', quality: 88
  });
  const done = n - n0 + 1;
  if (done % 25 === 0 || done === N) {
    const el = (Date.now() - started) / 1000, eta = el / done * (N - done);
    process.stdout.write(`\r  ${done}/${N}  (${(done / N * 100).toFixed(1)}%)  경과 ${el.toFixed(0)}s  남은 ${eta.toFixed(0)}s   `);
  }
}
console.log('\n프레임 완료 — ffmpeg 인코딩');
await browser.close();

await new Promise((res, rej) => {
  const ff = spawn('ffmpeg', ['-y', '-framerate', String(FPS),
    '-i', path.join(TMP, 'f%06d.jpg'),
    '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '19',
    '-preset', 'medium', '-movflags', '+faststart',
    path.join(HERE, OUT)], { stdio: ['ignore', 'ignore', 'inherit'] });
  ff.on('close', c => c === 0 ? res() : rej(new Error('ffmpeg exit ' + c)));
});
await rm(TMP, { recursive: true, force: true });

console.log(`\n✅ ${OUT}`);
if (errors.length) console.log('⚠ 브라우저 오류 ' + errors.length + '건:\n  ' + errors.slice(0, 8).join('\n  '));
