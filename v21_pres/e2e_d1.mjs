// v21 D1 E2E: 전체화면·별도 창·예측점 평면 줌/팬/집기 — JS 오류 0 + 동작 확인
import { chromium } from 'playwright';

const errs = [];
const br = await chromium.launch({ executablePath: '/opt/pw-browsers/chromium' });
const ctx = await br.newContext({ viewport: { width: 1500, height: 950 } });
const page = await ctx.newPage();
page.on('pageerror', e => errs.push('pageerror: ' + e.message));
page.on('console', m => { if (m.type() === 'error') errs.push('console: ' + m.text()); });

await page.goto('http://localhost:8213/');
await page.waitForFunction(() => (typeof PL!=="undefined" && PL!==null) && !!PLSCALE.pl2, null, { timeout: 15000 });
console.log('로드 + init 수신 OK');

// --- 팝업(별도 창) ---
const [pop] = await Promise.all([ctx.waitForEvent('page'), page.click('#bPop2')]);
await pop.waitForLoadState();
await page.waitForTimeout(600);
const popW = await pop.evaluate(() => document.getElementById('c').width);
const popOk = await page.evaluate(() => !!PLPOP.pl2 && !PLPOP.pl2.win.closed);
console.log(`별도 창: canvas.width=${popW} (>0 이면 미러 렌더 중), PLPOP 등록=${popOk}`);
if (!(popW > 0 && popOk)) errs.push('popup mirror not rendering');

// --- 전체화면 (headless 에서도 fullscreenElement 세팅 확인) ---
await page.click('#bFs2');
await page.waitForTimeout(400);
const fsId = await page.evaluate(() => document.fullscreenElement && document.fullscreenElement.id);
console.log(`전체화면: fullscreenElement=${fsId}`);
await page.keyboard.press('Escape');
await page.waitForTimeout(300);

// --- 줌 (휠) ---
const box = await page.locator('#pl2').boundingBox();
const cxp = box.x + box.width / 2, cyp = box.y + box.height / 2;
const m0 = await page.evaluate(() => PLSCALE.pl2.m);
await page.mouse.move(cxp, cyp);
await page.mouse.wheel(0, -240);          // 줌인
await page.waitForTimeout(300);
const v1 = await page.evaluate(() => ({ on: PLVIEW.pl2.on, m: PLVIEW.pl2.m }));
console.log(`줌: auto m=${m0.toFixed(2)} → 수동뷰 on=${v1.on}, m=${v1.m.toFixed(2)}`);
if (!(v1.on && v1.m < m0)) errs.push('wheel zoom failed');

// --- 팬 (우클릭 드래그) ---
await page.mouse.move(cxp, cyp);
await page.mouse.down({ button: 'right' });
await page.mouse.move(cxp + 60, cyp + 40, { steps: 4 });
await page.mouse.up({ button: 'right' });
const v2 = await page.evaluate(() => ({ b: PLVIEW.pl2.b, f: PLVIEW.pl2.f }));
console.log(`팬: 뷰중심 (β=${v2.b.toFixed(3)}, φ=${v2.f.toFixed(3)})`);
if (Math.abs(v2.b) < 1e-6 && Math.abs(v2.f) < 1e-6) errs.push('pan failed');

// --- 더블클릭 = 뷰 초기화 ---
await page.mouse.dblclick(cxp, cyp);
const v3 = await page.evaluate(() => PLVIEW.pl2.on);
console.log(`더블클릭 초기화: on=${v3}`);
if (v3) errs.push('dblclick reset failed');

// --- 좌클릭 집기: 클릭 위치 (β,φ) → pose(속도0) → lastView.pl 이 그 점으로 ---
const sc = await page.evaluate(() => PLSCALE.pl2);
const dpr = await page.evaluate(() => devicePixelRatio);
const dx = 40, dy = -25;                   // CSS px 오프셋
const expB = (sc.vb || 0) + (dx * dpr) / sc.R * sc.m;
const expF = (sc.vf || 0) + (dy * dpr) / sc.R * sc.m * -1 * -1; // f = vf - (py-cy)/R*m; dy<0 → f>0
const expF2 = (sc.vf || 0) - (dy * dpr) / sc.R * sc.m;
await page.mouse.move(cxp + dx, cyp + dy);
await page.mouse.down();
await page.waitForTimeout(200);
await page.mouse.move(cxp + dx + 8, cyp + dy, { steps: 2 });  // 드래그도 동작
await page.mouse.move(cxp + dx, cyp + dy, { steps: 2 });
await page.waitForTimeout(200);
await page.mouse.up();
await page.waitForTimeout(500);
const pl = await page.evaluate(() => lastView && lastView.pl);
console.log(`집기: 기대 (β=${expB.toFixed(2)}, φ=${expF2.toFixed(2)}) ↔ lastView.pl=[${pl ? pl.map(v => v.toFixed(2)).join(', ') : '—'}]`);
if (!pl || Math.abs(pl[0] - expB) > 0.15 || Math.abs(pl[1] - expF2) > 0.15) errs.push('pl2 pick pose mismatch');
if (pl && (Math.abs(pl[2] - pl[0]) > 1e-6 || Math.abs(pl[3] - pl[1]) > 1e-6)) errs.push('velocity not zero (pred != pos)');

// --- 실행 → 그 점에서 물리 시작 (러닝 전환 + 점 이동) ---
await page.keyboard.press('Space');
await page.waitForTimeout(700);
const pl2 = await page.evaluate(() => lastView.pl);
const moved = Math.hypot(pl2[0] - pl[0], pl2[1] - pl[1]);
console.log(`실행 후 0.7s: 점 이동량 ${moved.toFixed(3)}° (물리 살아있음)`);
if (moved < 1e-4) errs.push('run after pick did not move');

// --- 원위치 평면 집기(pl1) 회귀 확인: PLSCALE.pl1 이 팝업에 안 덮였는지 ---
const s1 = await page.evaluate(() => PLSCALE.pl1 && PLSCALE.pl1.R > 0);
if (!s1) errs.push('PLSCALE.pl1 broken');

console.log(errs.length ? '\n❌ 실패:\n' + errs.join('\n') : '\n✅ E2E 전부 통과, JS 오류 0');
await br.close();
process.exit(errs.length ? 1 : 0);
