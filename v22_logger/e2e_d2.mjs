// v21 D2 E2E: 두 평면 조작 통일 + 전체화면(로봇 미러) + 별도 창(조작 동일) — JS 오류 0
import { chromium } from 'playwright';

const errs = [];
const br = await chromium.launch({ executablePath: '/opt/pw-browsers/chromium' });
const ctx = await br.newContext({ viewport: { width: 1600, height: 900 } });
const page = await ctx.newPage();
page.on('pageerror', e => errs.push('pageerror: ' + e.message));
page.on('console', m => { if (m.type() === 'error' && !m.text().includes('404')) errs.push('console: ' + m.text()); });

await page.goto('http://localhost:8213/');
await page.waitForFunction(() => (typeof PL !== 'undefined' && PL !== null) && !!PLSCALE.pl1 && !!PLSCALE.pl2, null, { timeout: 15000 });
console.log('로드 OK');

async function planeCenter(id) {
  const b = await page.locator('#' + id).boundingBox();
  return { x: b.x + b.width / 2, y: b.y + b.height / 2 };
}

// ---- 두 평면 공통: 줌 → 팬 → 더블클릭 복귀 ----
for (const k of ['pl1', 'pl2']) {
  const c = await planeCenter(k);
  await page.mouse.move(c.x, c.y);
  await page.mouse.wheel(0, -240);
  await page.waitForTimeout(250);
  let v = await page.evaluate(kk => ({ on: PLVIEW[kk].on, m: PLVIEW[kk].m }), k);
  if (!v.on) errs.push(k + ' zoom failed');
  await page.mouse.move(c.x, c.y);
  await page.mouse.down({ button: 'right' });
  await page.mouse.move(c.x + 50, c.y + 30, { steps: 3 });
  await page.mouse.up({ button: 'right' });
  const v2 = await page.evaluate(kk => ({ b: PLVIEW[kk].b, f: PLVIEW[kk].f }), k);
  if (Math.abs(v2.b) < 1e-9 && Math.abs(v2.f) < 1e-9) errs.push(k + ' pan failed');
  await page.mouse.dblclick(c.x, c.y);
  const v3 = await page.evaluate(kk => PLVIEW[kk].on, k);
  if (v3) errs.push(k + ' dblclick reset failed');
  console.log(`${k}: 줌 m=${v.m.toFixed(2)} · 팬 (${v2.b.toFixed(2)},${v2.f.toFixed(2)}) · 복귀 OK`);
}

// ---- 두 평면 공통: 좌클릭 = 자세만 (자동 실행 없음), Space = 시작 ----
for (const k of ['pl1', 'pl2']) {
  const c = await planeCenter(k);
  await page.mouse.move(c.x + 30, c.y - 20);
  await page.mouse.down();
  await page.waitForTimeout(150);
  await page.mouse.move(c.x + 36, c.y - 20, { steps: 2 });
  await page.waitForTimeout(150);
  await page.mouse.up();
  await page.waitForTimeout(400);
  const st = await page.evaluate(() => ({ run: running, pl: lastView && lastView.pl }));
  if (st.run) errs.push(k + ' click auto-ran (should pose only)');
  if (!st.pl || Math.abs(st.pl[2] - st.pl[0]) > 1e-6) errs.push(k + ' pose velocity not zero');
  console.log(`${k}: 클릭=자세만 (running=${st.run}) pred==pos OK`);
}
await page.keyboard.press('Space');
await page.waitForTimeout(500);
const ran = await page.evaluate(() => running);
if (!ran) errs.push('Space run failed');
await page.keyboard.press('Space'); // pause
console.log('Space 실행/정지 OK');

// ---- Shift 스냅 (pl1): 드래그 점이 모드선 위 ----
{
  const c = await planeCenter('pl1');
  await page.keyboard.down('Shift');
  await page.mouse.move(c.x + 40, c.y + 10);
  await page.mouse.down();
  await page.waitForTimeout(200);
  const p = await page.evaluate(() => PICK.cur);
  await page.mouse.up();
  await page.keyboard.up('Shift');
  const r = await page.evaluate(() => PL.r);
  if (!p || Math.abs(p.f - r * p.b) > 1e-9) errs.push('pl1 Shift snap failed');
  console.log(`pl1 Shift 스냅: φ−r·β = ${p ? (p.f - r * p.b).toExponential(1) : '—'} OK`);
}

// ---- 전체화면: 그리드 + 로봇 미러 캔버스 렌더 ----
await page.click('#bFs2');
await page.waitForTimeout(500);
const fs = await page.evaluate(() => {
  const b = document.fullscreenElement;
  const fr = b && b.querySelector('.fsrobot');
  return { id: b && b.id, robot: fr && fr.width > 0 && fr.clientWidth > 0 };
});
if (fs.id !== 'box_pl2' || !fs.robot) errs.push('fullscreen robot mirror failed: ' + JSON.stringify(fs));
console.log(`전체화면: ${fs.id}, 로봇 미러 렌더=${fs.robot}`);
// 전체화면 안에서도 조작(휠 줌) 동작
{
  const c = await planeCenter('pl2');
  await page.mouse.move(c.x, c.y);
  await page.mouse.wheel(0, -240);
  await page.waitForTimeout(250);
  const on = await page.evaluate(() => PLVIEW.pl2.on);
  if (!on) errs.push('fullscreen wheel zoom failed');
  await page.mouse.dblclick(c.x, c.y);
  console.log('전체화면 내 줌·복귀 OK');
}
await page.keyboard.press('Escape');
await page.waitForTimeout(300);

// ---- 별도 창: 로봇 미러 + 조작 (휠 줌이 공유 뷰에 반영, 클릭=pose) ----
const [pop] = await Promise.all([ctx.waitForEvent('page'), page.click('#bPop2')]);
await pop.waitForLoadState();
await pop.waitForTimeout(700);
const pw = await pop.evaluate(() => ({
  plane: document.getElementById('c').width,
  robot: document.getElementById('r').width,
}));
if (!(pw.plane > 0 && pw.robot > 0)) errs.push('popup canvases not rendering: ' + JSON.stringify(pw));
console.log(`별도 창: 평면 ${pw.plane}px · 로봇 미러 ${pw.robot}px 렌더 중`);
const pcb = await pop.locator('#c').boundingBox();
await pop.mouse.move(pcb.x + pcb.width / 2, pcb.y + pcb.height / 2);
await pop.mouse.wheel(0, -240);
await pop.waitForTimeout(300);
const shared = await page.evaluate(() => PLVIEW.pl2.on);
if (!shared) errs.push('popup wheel did not affect shared view');
console.log(`별도 창 휠 줌 → 본창 공유 뷰 반영=${shared}`);
await pop.mouse.dblclick(pcb.x + pcb.width / 2, pcb.y + pcb.height / 2);
// 별도 창에서 클릭 = pose
await pop.mouse.move(pcb.x + pcb.width / 2 + 25, pcb.y + pcb.height / 2 - 15);
await pop.mouse.down(); await pop.waitForTimeout(150); await pop.mouse.up();
await pop.waitForTimeout(400);
const st2 = await page.evaluate(() => ({ run: running, pl: lastView.pl }));
if (st2.run) errs.push('popup click auto-ran');
if (Math.abs(st2.pl[2] - st2.pl[0]) > 1e-6) errs.push('popup pose velocity not zero');
console.log(`별도 창 클릭=자세만 OK (β_pred=${st2.pl[2].toFixed(2)})`);
// 별도 창에서 Space = 실행
await pop.bringToFront();
await pop.keyboard.press('Space');
await pop.waitForTimeout(400);
const ran2 = await page.evaluate(() => running);
if (!ran2) errs.push('popup Space run failed');
console.log('별도 창 Space 실행 OK');

console.log(errs.length ? '\n❌ 실패:\n' + errs.join('\n') : '\n✅ D2 E2E 전부 통과, JS 오류 0');
await br.close();
process.exit(errs.length ? 1 : 0);
