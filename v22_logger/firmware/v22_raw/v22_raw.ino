/*
 * v22_raw.ino — v22 로거·측정 앱 전용 펌웨어 (원시값 스트리밍 + 접기 실험) — 2026-09-03 v2 ★미컴파일·실물검증 대기
 * ============================================================================
 *  v22 앱과 붙어서 측정·접기·기록을 전부 한다. 별도 아두이노 앱 없음.
 *
 *  【모드】 mode 0  측정: 제어 없음, δ 고정 (λ·r 놓기 실험, 영점, P2R)
 *          mode 1  단일접기: g 로 무장 → |Â| > trig 인 첫 순간 한 번 접고 δ 고정 → 이후 자유 발산 (γ 실험)
 *          mode 2  증분접기: |Â| > trig 마다 hold += sgn·γ·Â, |Â| < rel 이면 천천히 펴기 (본 제어)
 *      ρ 는 없다. 접기량 = γ·Â 그대로 (사용자 결정 9/3).  fdeg > 0 이면 γ 대신 고정 접기량 sgn(Â)·fdeg.
 *
 *  【판정량】 Â = W_PHI·φ + β + W_PHIDOT·φ̇ + W_BETADOT·β̇ + A_OFFSET
 *      wmode 0 (기본): 닫힌형 w = [−1/r, 1, kv·(−1/(rλ)), kv/λ], A_OFFSET = c0/r   (문서 46; kv = 속도항 보정, 문서 70 §7 κ)
 *      wmode 1        : 속도 가중 수동 wf, wb (finale8 동정값)
 *      α = ank − φ (문서 70 §2), β = α + P2R·δ, φ̇ β̇ = 25 ms 차분 + EMA τ≈28 ms (한 묶음, 문서 46 §7)
 *
 *  【CSV】 '# D,…' 헤더 뒤 D행 (LOG_HZ). 앱·hangcal_logger 가 헤더를 그대로 열 이름으로 쓴다.
 *      D,t_ms,phi,ank,alpha,beta,dphi,dbeta,Ahat,hold,del_now,phase,cue,err,phi_raw,ank_raw,dxl_raw
 *      phase 0 IDLE(무장) / 1 FOLD / 2 REST / 3 STOP(넘어짐·비상) / 4 대기(비무장)   cue = 단일접기 완료 1
 *      E행: ZERO / MOVE / SWAP / FOLD,Δδ_cmd / FOLDPOST,A_post / FALL / GO / STOP     F행: 단일접기 요약(아래)
 *      # F,trial,A_pre,d0,dd_cmd,dd_act,A_post,lock_ms,fold_ms,goaln
 *
 *  【명령】 115200. 줄바꿈 또는 200 ms 조용하면 실행.
 *      z 영점(2단, 문서 66)  u 토크해제  k 토크ON(δ=0 재정의)  <정수> δ 명령  m CSV  s 출력  p 한 줄  t 상태
 *      e 엔코더 진단  swap CS 교환  hdr 헤더
 *      mode N   모드 0/1/2        g 시작(무장)   h 정지(토크·자세 유지)   x 비상정지(토크 OFF)   y dry-run 토글
 *      fold X   지금 즉시 X° 접기 (제어와 무관, γ 실험의 고정 Δδ 주입용)
 *      이름 값  파라미터 (w 로 목록): gam trig rel vrel dead dstep dlim rest alim sgn fdeg lock r lam c0 kv wmode wf wb p2r loghz vel acc ilim
 *  I/O 코드는 incremental_fold_min.ino(실기 검증됨)에서 그대로 가져왔다.
 * ============================================================================
 */
#include <SPI.h>
#include <Dynamixel2Arduino.h>
#include <math.h>

// ---- 실측 상수 · 판정 (전부 런타임 변경 가능) ----
float P2R = 0.4285f, LAMBDA = 5.44f, R_SLOPE = -1.506f, LINE_C = 0.0f;
float KV = 1.0f;                       // [kv] 닫힌형 속도항 배율 (문서 70 §7 κ≈1.83 을 쓰려면 1.83)
float WMODE = 0.0f;                    // [wmode] 0 닫힌형 / 1 수동 wf·wb
float WF_MAN = 0.1945f, WB_MAN = 0.3049f;
const float PHI_SIGN = +1.0f, ANK_SIGN = +1.0f;                        // 문서 65: 센서 읽는 줄에서만

// ---- 제어 노브 (ρ 없음) ----
float GAMMA = 8.0f;                    // [gam ] 접기 이득 γ [°접기/°Â] — 접기 성적표로 잰다
float A_TRIG = 0.6f;                   // [trig] 트리거 문턱 [deg]
float A_RELAX = 0.3f;                  // [rel ] 펴기 게이트 [deg]
float RELAX_RATE = 3.0f;               // [vrel] 펴기 속도 [deg/s]
float HOLD_DEAD = 1.0f;                // [dead] 이보다 작은 유지각은 안 편다
float STEP_LIM = 20.0f;                // [dstep] 접기 1회 상한
float DELTA_LIM = 55.0f;               // [dlim] 힙 기구한계
float T_REST = 60.0f;                  // [rest] REST [ms]
float ANG_LIMIT = 30.0f;               // [alim] |φ| 나 |α| 초과 = 넘어짐 → 토크 OFF
float FOLD_SIGN = +1.0f;               // [sgn ] 접기 방향 ±1 — 바닥 시험(y + g)으로 확인
float FDEG = 0.0f;                     // [fdeg] >0 이면 고정 접기량 (γ 실험용)
float LOCK_MS = 250.0f;                // [lock] 접기 뒤 A_post 관측창 [ms]
float FOLD_TOL = 2.0f, FOLD_TMAX = 300.0f;
float LOG_HZ = 100.0f;
int   VEL_UNIT = 250, ACC_UNIT = 373, CUR_LIMIT = 350;

// ---- 파이프라인 (한 묶음 — 문서 46 §7) ----
const uint32_t DT_US = 5000;  const float DT_S = 0.005f;   // 200 Hz
const int   VEL_N = 5;  const float EMA_A = 0.15f;

// ---- 하드웨어 (문서 17·52 배선) ----
#define DXL_SERIAL  Serial3
#define DXL_DIR_PIN 84
const uint8_t DXL_ID = 1;
uint8_t PHI_CS = 10, ANK_CS = 9;                 // swap 명령으로 런타임 교환 가능 (진단용)
const int   MOTOR_DIR = +1;
const float TICK_PER_DEG = 4096.0f / 360.0f;
const float VEL_DPS = 1.374f, ACC_DPS2 = 21.4577f;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// ---- 상태 ----
enum Phase { IDLE = 0, FOLD = 1, REST = 2, STOPPED = 3, ARMED_OFF = 4 };
Phase phase = ARMED_OFF;
int   run_mode = 0;                    // 0 측정 / 1 단일접기 / 2 증분접기
bool  motor_ok = false, csv_on = true, out_on = true, running = false, dry_run = false, once_done = false;
float home_tick = 0;  uint16_t phi_zero = 0, ank_zero = 0, phi_raw = 0, ank_raw = 0;
float dxl_raw = 0;
uint16_t phi_last = 0xFFFF, ank_last = 0xFFFF;  uint32_t phi_chg_ms = 0, ank_chg_ms = 0;
const uint32_t STUCK_MS = 2000;
int   zero_stage = 0;  uint16_t phi_z1 = 0, ank_z1 = 0;
float hold = 0, delta_now = 0;
float phi_d = 0, ank_d = 0, alpha_d = 0, beta_d = 0, dphi = 0, dbeta = 0, Ahat = 0;
float W_PHI, W_PHIDOT, W_BETADOT, A_OFFSET;
float phi_hist[VEL_N + 1], beta_hist[VEL_N + 1];
int   hist_i = 0;  bool primed = false;
uint32_t t0 = 0, next_us = 0, log_next_ms = 0, mon_next = 0, hdr_next_ms = 0;
uint32_t phase_t0 = 0, fold_wait_ms = 0, fold_t0 = 0;
bool  post_pending = false;  int trial_n = 0;
float f_A_pre = 0, f_d0 = 0, f_dd_cmd = 0, f_goal = 0;  uint32_t f_fold_ms = 0;  bool f_arrived = false;
uint8_t relax_thr = 0;
char  linebuf[48]; uint8_t linelen = 0; uint32_t last_rx_ms = 0;
int   err_code = 0;

uint16_t circMean14(uint16_t a, uint16_t b) {
  int16_t d = (int16_t)((b - a) & 0x3FFF); if (d > 8191) d -= 16384;
  return (uint16_t)((a + d / 2) & 0x3FFF);
}
void deriveW() {
  W_PHI = -1.0f / R_SLOPE;
  if (WMODE < 0.5f) { W_PHIDOT = KV * (-1.0f / (R_SLOPE * LAMBDA)); W_BETADOT = KV / LAMBDA; }
  else              { W_PHIDOT = WF_MAN; W_BETADOT = WB_MAN; }
  A_OFFSET = LINE_C / R_SLOPE;
}
void emitE(const char* name, float v, int dec) {
  Serial.print("E,"); Serial.print(millis() - t0); Serial.print(','); Serial.print(name); Serial.print(','); Serial.println(v, dec);
}

// ---- AS5047P (SPI 모드1, 2회 읽기) ----
uint16_t as5047_raw(uint8_t cs) {
  uint16_t v;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(cs, LOW);  delayMicroseconds(1);
  SPI.transfer16(0xFFFF);
  digitalWrite(cs, HIGH); delayMicroseconds(1);
  digitalWrite(cs, LOW);  delayMicroseconds(1);
  v = SPI.transfer16(0xFFFF);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  return v & 0x3FFF;
}
uint16_t as5047_frame(uint8_t cs, uint16_t frame) {
  uint16_t v;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(cs, LOW);  delayMicroseconds(1);
  v = SPI.transfer16(frame);
  digitalWrite(cs, HIGH); delayMicroseconds(1);
  SPI.endTransaction();
  return v;
}
uint16_t as5047_parity(uint16_t v) { uint16_t x = v & 0x7FFF; int n = 0; while (x) { n += x & 1; x >>= 1; } return (n & 1) ? 0x8000 : 0; }
uint16_t as5047_reg(uint8_t cs, uint16_t addr, bool* ef) {
  uint16_t cmd = 0x4000 | (addr & 0x3FFF);  cmd |= as5047_parity(cmd);
  as5047_frame(cs, cmd);
  uint16_t r = as5047_frame(cs, 0x0000);
  if (ef) *ef = (r & 0x4000) != 0;
  return r & 0x3FFF;
}
void encDiag(const char* name, uint8_t cs) {
  bool ef1, ef2, ef3, ef4;
  uint16_t com  = as5047_reg(cs, 0x3FFF, &ef1);
  uint16_t unc  = as5047_reg(cs, 0x3FFE, &ef2);
  uint16_t diag = as5047_reg(cs, 0x3FFC, &ef3);
  uint16_t errf = as5047_reg(cs, 0x0001, &ef4);
  Serial.print("# ENC "); Serial.print(name); Serial.print(" (CS D"); Serial.print(cs); Serial.print("): ");
  Serial.print("ANGLECOM="); Serial.print(com); Serial.print(" ANGLEUNC="); Serial.print(unc);
  Serial.print(" | DIAAGC raw="); Serial.print(diag);
  Serial.print(" AGC="); Serial.print(diag & 0xFF); Serial.print(" LF="); Serial.print((diag >> 8) & 1);
  Serial.print(" COF="); Serial.print((diag >> 9) & 1); Serial.print(" MagH="); Serial.print((diag >> 10) & 1);
  Serial.print(" MagL="); Serial.print((diag >> 11) & 1);
  Serial.print(" | ERRFL="); Serial.print(errf); Serial.print(" EF="); Serial.println((ef1 || ef2 || ef3 || ef4) ? 1 : 0);
  if ((diag == 0 && com == 0) || (diag == 0x3FFF && com == 0x3FFF)) Serial.println("#    → 응답 없음(전부 0 또는 16383): MISO/CS/전원 배선·커넥터 (문서 52)");
  else if ((diag >> 11) & 1 || (diag & 0xFF) == 0xFF) Serial.println("#    → MagL/AGC=255: 자석이 멀거나 없음 — 자석 마운트·갭 확인");
  else if ((diag >> 10) & 1 || (diag & 0xFF) == 0) Serial.println("#    → MagH/AGC=0: 자석이 너무 가까움");
  else if (!((diag >> 8) & 1)) Serial.println("#    → LF=0: 내부 보정 루프 미완 — 전원 직후거나 자석 불안정");
  else Serial.println("#    → 센서는 정상 응답 (자석·통신 OK). 값이 안 변하면 자석이 관절과 같이 돌지 않는 기구 문제");
}
float rawToDeg(uint16_t raw, uint16_t zero) {
  int16_t d = (int16_t)((raw - zero) & 0x3FFF);
  if (d > 8191) d -= 16384;
  return d * (360.0f / 16384.0f);
}

// ---- 모터 ----
void writeGoal(float deg) {
  if (motor_ok && !dry_run) dxl.setGoalPosition(DXL_ID, home_tick + MOTOR_DIR * deg * TICK_PER_DEG);
}
float readDelta() {
  if (!motor_ok) return hold;
  float t = dxl.getPresentPosition(DXL_ID);
  if (t == 0.0f) { err_code |= 16; return delta_now; }
  dxl_raw = t;
  return MOTOR_DIR * (t - home_tick) / TICK_PER_DEG;
}
float profileMs(float deg) {                     // 프로파일 소요시간 — 도착을 기다리지 않는다
  deg = fabsf(deg);
  float acc = ACC_UNIT * ACC_DPS2, vmx = VEL_UNIT * VEL_DPS;
  if (deg <= 0.0f) return 20.0f;
  float t = (sqrtf(deg * acc) <= vmx) ? (2.0f * sqrtf(deg / acc)) : (deg / vmx + vmx / acc);
  return t * 1000.0f;
}
void torqueOffAll(const char* why) {
  running = false; phase = STOPPED;
  if (motor_ok) dxl.torqueOff(DXL_ID);
  Serial.print("# 토크 OFF: "); Serial.println(why);
}

// ---- 읽기 + 파생값 ----
void readState() {
  phi_raw = as5047_raw(PHI_CS);  ank_raw = as5047_raw(ANK_CS);
  uint32_t nowms = millis();
  if (phi_raw != phi_last) { phi_last = phi_raw; phi_chg_ms = nowms; }
  if (ank_raw != ank_last) { ank_last = ank_raw; ank_chg_ms = nowms; }
  err_code = 0;
  if (phi_raw == 0 || phi_raw == 0x3FFF) err_code |= 1;
  else if (nowms - phi_chg_ms > STUCK_MS) err_code |= 2;
  if (ank_raw == 0 || ank_raw == 0x3FFF) err_code |= 4;
  else if (nowms - ank_chg_ms > STUCK_MS) err_code |= 8;
  phi_d = PHI_SIGN * rawToDeg(phi_raw, phi_zero);
  ank_d = ANK_SIGN * rawToDeg(ank_raw, ank_zero);
  delta_now = readDelta();
  alpha_d = ank_d - phi_d;
  beta_d  = alpha_d + P2R * delta_now;
  if (!primed) {
    for (int i = 0; i <= VEL_N; i++) { phi_hist[i] = phi_d; beta_hist[i] = beta_d; }
    dphi = dbeta = 0; primed = true;
  }
  phi_hist[hist_i] = phi_d;  beta_hist[hist_i] = beta_d;
  int old = (hist_i + 1) % (VEL_N + 1);
  float dphi_raw  = (phi_d  - phi_hist[old])  / (VEL_N * DT_S);
  float dbeta_raw = (beta_d - beta_hist[old]) / (VEL_N * DT_S);
  hist_i = old;
  dphi  += EMA_A * (dphi_raw  - dphi);
  dbeta += EMA_A * (dbeta_raw - dbeta);
  Ahat = W_PHI * phi_d + beta_d + W_PHIDOT * dphi + W_BETADOT * dbeta + A_OFFSET;
}

// ---- 접기 ----
void doFold(float step, const char* why) {
  if (step >  STEP_LIM) step =  STEP_LIM;
  if (step < -STEP_LIM) step = -STEP_LIM;
  f_A_pre = Ahat; f_d0 = delta_now; f_dd_cmd = step;
  hold += step;
  if (hold >  DELTA_LIM) hold =  DELTA_LIM;
  if (hold < -DELTA_LIM) hold = -DELTA_LIM;
  f_goal = hold;
  writeGoal(hold);
  float fw = profileMs(step) * 1.3f + 20.0f;
  fold_wait_ms = (uint32_t)((fw > FOLD_TMAX) ? FOLD_TMAX : fw);
  phase = FOLD; phase_t0 = fold_t0 = millis(); f_arrived = false; f_fold_ms = 0;
  post_pending = true; trial_n++;
  Serial.print("E,"); Serial.print(millis() - t0); Serial.print(",FOLD,"); Serial.print(step, 2);
  Serial.print("   # "); Serial.print(why); Serial.print(" A_pre="); Serial.println(f_A_pre, 3);
}
void controlStep() {
  uint32_t now = millis();
  switch (phase) {
    case IDLE:
      if (run_mode == 1 && once_done) break;
      if (run_mode >= 1 && fabsf(Ahat) > A_TRIG) {
        float step = (FDEG > 0.0f) ? FOLD_SIGN * FDEG * (Ahat > 0 ? 1.0f : -1.0f) : FOLD_SIGN * GAMMA * Ahat;
        doFold(step, run_mode == 1 ? "단일접기" : "증분접기");
        if (run_mode == 1) once_done = true;
      } else if (run_mode == 2 && fabsf(Ahat) < A_RELAX && fabsf(hold) > HOLD_DEAD) {
        hold += (hold > 0 ? -1 : 1) * RELAX_RATE * DT_S;
        if (++relax_thr >= 10) { relax_thr = 0; writeGoal(hold); }
      }
      break;
    case FOLD:
      if (!f_arrived && fabsf(delta_now - hold) < FOLD_TOL) { f_arrived = true; f_fold_ms = now - fold_t0; }
      if (f_arrived || (uint32_t)(now - phase_t0) >= fold_wait_ms) { phase = REST; phase_t0 = now; if (!f_arrived) f_fold_ms = now - fold_t0; }
      break;
    case REST:
      if ((uint32_t)(now - phase_t0) >= (uint32_t)T_REST) phase = IDLE;
      break;
    default: break;
  }
  if (post_pending && (uint32_t)(now - fold_t0) >= (uint32_t)LOCK_MS) {   // 접기 뒤 관측창 끝 — F행 + E행
    post_pending = false;
    float dd_act = delta_now - f_d0;
    Serial.print("F,"); Serial.print(trial_n); Serial.print(',');
    Serial.print(f_A_pre, 4); Serial.print(','); Serial.print(f_d0, 2); Serial.print(',');
    Serial.print(f_dd_cmd, 2); Serial.print(','); Serial.print(dd_act, 2); Serial.print(',');
    Serial.print(Ahat, 4); Serial.print(','); Serial.print((int)LOCK_MS); Serial.print(',');
    Serial.print(f_fold_ms); Serial.print(','); Serial.println(f_goal, 2);
    emitE("FOLDPOST", Ahat, 4);
  }
}

// ---- 출력 ----
void logHeader() {
  Serial.println("# D,t_ms,phi,ank,alpha,beta,dphi,dbeta,Ahat,hold,del_now,phase,cue,err,phi_raw,ank_raw,dxl_raw");
  Serial.println("# F,trial,A_pre,d0,dd_cmd,dd_act,A_post,lock_ms,fold_ms,goaln");
  Serial.println("# v22_raw v2: phase 0 IDLE/1 FOLD/2 REST/3 STOP/4 대기. cue=단일접기 완료. err = phi등급 + 4·ank등급 + 16·dxl");
}
void logLine() {
  Serial.print("D,");
  Serial.print(millis() - t0); Serial.print(',');
  Serial.print(phi_d, 3);      Serial.print(',');
  Serial.print(ank_d, 3);      Serial.print(',');
  Serial.print(alpha_d, 3);    Serial.print(',');
  Serial.print(beta_d, 3);     Serial.print(',');
  Serial.print(dphi, 2);       Serial.print(',');
  Serial.print(dbeta, 2);      Serial.print(',');
  Serial.print(Ahat, 4);       Serial.print(',');
  Serial.print(hold, 2);       Serial.print(',');
  Serial.print(delta_now, 2);  Serial.print(',');
  Serial.print((int)(running ? phase : (phase == STOPPED ? STOPPED : ARMED_OFF))); Serial.print(',');
  Serial.print(once_done ? 1 : 0); Serial.print(',');
  Serial.print(err_code);      Serial.print(',');
  Serial.print(phi_raw);       Serial.print(',');
  Serial.print(ank_raw);       Serial.print(',');
  Serial.println((long)dxl_raw);
}
void printStatus() {
  Serial.println("---- v22_raw v2 상태 ----");
  Serial.print("모드 "); Serial.print(run_mode == 0 ? "0 측정" : (run_mode == 1 ? "1 단일접기" : "2 증분접기"));
  Serial.print("   제어 "); Serial.print(running ? "RUN" : "정지"); Serial.print(dry_run ? "  [DRY-RUN]" : "");
  Serial.print("   phase "); Serial.println((int)phase);
  Serial.print("모터: "); Serial.print(motor_ok ? "OK" : "응답 없음");
  if (motor_ok) { Serial.print("  torque="); Serial.print(dxl.getTorqueEnableStat(DXL_ID) ? "ON" : "OFF"); }
  Serial.println();
  Serial.print("w = ["); Serial.print(W_PHI, 4); Serial.print(", 1, "); Serial.print(W_PHIDOT, 4); Serial.print(", "); Serial.print(W_BETADOT, 4);
  Serial.print("]  A_offset="); Serial.print(A_OFFSET, 4); Serial.print("  (wmode "); Serial.print((int)WMODE); Serial.print(", kv "); Serial.print(KV, 2); Serial.println(")");
  Serial.print("gam "); Serial.print(GAMMA, 2); Serial.print("  trig "); Serial.print(A_TRIG, 2); Serial.print("  fdeg "); Serial.print(FDEG, 1);
  Serial.print("  lock "); Serial.print(LOCK_MS, 0); Serial.print("  sgn "); Serial.print(FOLD_SIGN, 0);
  Serial.print("  r "); Serial.print(R_SLOPE, 3); Serial.print("  lam "); Serial.print(LAMBDA, 2); Serial.print("  c0 "); Serial.print(LINE_C, 3); Serial.print("  p2r "); Serial.println(P2R, 4);
  Serial.print("CSV "); Serial.print(csv_on ? "ON" : "OFF"); Serial.print("  loghz "); Serial.print(LOG_HZ, 0);
  Serial.print("  vel "); Serial.print(VEL_UNIT); Serial.print("  acc "); Serial.print(ACC_UNIT); Serial.print("  ilim "); Serial.println(CUR_LIMIT);
  Serial.print("영점 raw: phi "); Serial.print(phi_zero); Serial.print("  ank "); Serial.print(ank_zero);
  Serial.print("  (단계 "); Serial.print(zero_stage); Serial.print("/2)  home_tick "); Serial.println(home_tick, 0);
  Serial.print("hold "); Serial.print(hold, 2); Serial.print("  delta "); Serial.print(delta_now, 2); Serial.print("  접기 시행 "); Serial.println(trial_n);
}

// ---- 파라미터 표 ----
struct Param { const char* name; float* p; float lo, hi; const char* what; };
const Param PARAMS[] = {
  {"gam",  &GAMMA,      0.5f,  60.0f, "접기 이득 γ [°/°] (ρ 없음)"},
  {"trig", &A_TRIG,     0.05f,  5.0f, "트리거 문턱 [deg]"},
  {"rel",  &A_RELAX,    0.02f,  5.0f, "펴기 게이트 [deg]"},
  {"vrel", &RELAX_RATE, 0.0f,  30.0f, "펴기 속도 [deg/s]"},
  {"dead", &HOLD_DEAD,  0.0f,  10.0f, "펴기 데드밴드 [deg]"},
  {"dstep",&STEP_LIM,   1.0f,  55.0f, "접기 1회 상한 [deg]"},
  {"dlim", &DELTA_LIM,  5.0f,  80.0f, "힙 기구한계 [deg]"},
  {"rest", &T_REST,     0.0f, 500.0f, "REST [ms]"},
  {"alim", &ANG_LIMIT,  5.0f,  90.0f, "넘어짐 한계각 [deg]"},
  {"sgn",  &FOLD_SIGN, -1.0f,   1.0f, "접기 방향 ±1"},
  {"fdeg", &FDEG,       0.0f,  55.0f, "고정 접기량 (0=γ·Â)"},
  {"lock", &LOCK_MS,   50.0f, 1000.0f,"A_post 관측창 [ms]"},
  {"r",    &R_SLOPE,  -20.0f,  -0.1f, "안정모드선 기울기"},
  {"lam",  &LAMBDA,     0.5f,  30.0f, "발산율 λ [1/s]"},
  {"c0",   &LINE_C,   -20.0f,  20.0f, "절편 c0 [deg]"},
  {"kv",   &KV,         0.2f,   4.0f, "닫힌형 속도항 배율"},
  {"wmode",&WMODE,      0.0f,   1.0f, "0 닫힌형 / 1 수동 wf wb"},
  {"wf",   &WF_MAN,     0.0f,   2.0f, "수동 w φ̇"},
  {"wb",   &WB_MAN,     0.0f,   2.0f, "수동 w β̇"},
  {"p2r",  &P2R,        0.05f,  0.95f,"β = α + P2R·δ"},
  {"loghz",&LOG_HZ,     1.0f, 200.0f, "CSV 주기 [Hz]"},
  {"ftol", &FOLD_TOL,   0.1f,  10.0f, "접기 도착 판정 [deg]"},
  {"ftmax",&FOLD_TMAX, 20.0f, 800.0f, "FOLD 상한 [ms]"},
};
const int N_PARAMS = sizeof(PARAMS) / sizeof(PARAMS[0]);
int findParam(const char* n) { for (int i = 0; i < N_PARAMS; i++) if (!strcmp(PARAMS[i].name, n)) return i; return -1; }
void printAllParams() { for (int i = 0; i < N_PARAMS; i++) { Serial.print("# "); Serial.print(PARAMS[i].name); Serial.print(" = "); Serial.print(*PARAMS[i].p, 4); Serial.print("   "); Serial.println(PARAMS[i].what); } }

// ---- 명령 ----
void doGo() {
  if (phase == STOPPED && !dry_run && motor_ok && !dxl.getTorqueEnableStat(DXL_ID)) { home_tick = dxl.getPresentPosition(DXL_ID); hold = 0; dxl.setGoalPosition(DXL_ID, home_tick); dxl.torqueOn(DXL_ID); Serial.println("# 토크 복구 (δ=0 재정의)"); }
  if (zero_stage == 1) Serial.println("# ⚠ 영점이 1차만 기록된 상태 — z 를 마저 누르는 게 좋다");
  if (motor_ok && !dry_run) delta_now = readDelta();
  hold = delta_now; primed = false; dphi = dbeta = 0;
  once_done = false; post_pending = false; phase = IDLE; phase_t0 = millis();
  running = true;
  Serial.print("# GO  모드 "); Serial.print(run_mode); Serial.print(run_mode == 1 ? " 단일접기 — 첫 |Â|>trig 에서 한 번 접고 δ 고정" : (run_mode == 2 ? " 증분접기" : " 측정(접지 않음)"));
  Serial.println(dry_run ? "  (DRY-RUN: 모터 명령 안 나감)" : "");
  emitE("GO", (float)run_mode, 0);
}
void handleLine(char* s) {
  while (*s == ' ') s++;
  if (!*s) return;
  char c = *s;
  if ((c >= '0' && c <= '9') || c == '+' || c == '-') {
    if (running) { Serial.println("# 제어 중에는 수동 δ 금지 — h 로 멈추고"); return; }
    long v = atol(s);
    if (v >  (long)DELTA_LIM) v =  (long)DELTA_LIM;
    if (v < -(long)DELTA_LIM) v = -(long)DELTA_LIM;
    if (!motor_ok) { Serial.println("# 모터 무응답"); return; }
    if (!dxl.getTorqueEnableStat(DXL_ID)) { home_tick = dxl.getPresentPosition(DXL_ID); dxl.torqueOn(DXL_ID); }
    hold = (float)v;  writeGoal(hold);
    emitE("MOVE", (float)v, 0);
    return;
  }
  // 이름 값
  char tok[12]; int tl = 0;
  while (s[tl] && ((s[tl] >= 'a' && s[tl] <= 'z') || (s[tl] >= 'A' && s[tl] <= 'Z') || (s[tl] >= '0' && s[tl] <= '9')) && tl < 11) { tok[tl] = (s[tl] >= 'A' && s[tl] <= 'Z') ? s[tl] + 32 : s[tl]; tl++; }
  tok[tl] = 0;
  const char* rest = s + tl; while (*rest == ' ' || *rest == '=') rest++;
  bool has = false; for (const char* q = rest; *q; q++) if (*q >= '0' && *q <= '9') { has = true; break; }
  float val = has ? atof(rest) : 0.0f;
  if (!strcmp(tok, "hdr"))  { logHeader(); return; }
  if (!strcmp(tok, "swap")) { uint8_t t = PHI_CS; PHI_CS = ANK_CS; ANK_CS = t; primed = false;
                              Serial.print("# CS 교환: phi←D"); Serial.print(PHI_CS); Serial.print("  ank←D"); Serial.println(ANK_CS);
                              emitE("SWAP", (float)PHI_CS, 0); return; }
  if (!strcmp(tok, "mode")) { if (running) { Serial.println("# h 로 멈추고 모드를 바꿀 것"); return; }
                              run_mode = (int)val; if (run_mode < 0) run_mode = 0; if (run_mode > 2) run_mode = 2; once_done = false;
                              Serial.print("# 모드 "); Serial.println(run_mode == 0 ? "0 측정(접지 않음)" : (run_mode == 1 ? "1 단일접기 (γ 실험)" : "2 증분접기")); return; }
  if (!strcmp(tok, "fold")) { if (!has) { Serial.println("# fold X  (X = 접기량 deg)"); return; }
                              if (!running) { Serial.println("# g 로 무장한 뒤에 (Â·D행이 살아 있어야 성적표가 된다)"); return; }
                              doFold(FOLD_SIGN * val, "수동 fold"); return; }
  if (!strcmp(tok, "vel"))  { VEL_UNIT = (int)val; if (motor_ok) dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, VEL_UNIT); Serial.print("# vel "); Serial.println(VEL_UNIT); return; }
  if (!strcmp(tok, "acc"))  { ACC_UNIT = (int)val; if (motor_ok) dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, ACC_UNIT); Serial.print("# acc "); Serial.println(ACC_UNIT); return; }
  if (!strcmp(tok, "ilim")) { CUR_LIMIT = (int)val; if (motor_ok) { bool on = dxl.getTorqueEnableStat(DXL_ID); dxl.torqueOff(DXL_ID); dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID, CUR_LIMIT); if (on) dxl.torqueOn(DXL_ID); } Serial.print("# ilim "); Serial.println(CUR_LIMIT); return; }
  int pi = findParam(tok);
  if (pi >= 0) {
    if (has) { if (val < PARAMS[pi].lo) val = PARAMS[pi].lo; if (val > PARAMS[pi].hi) val = PARAMS[pi].hi; *PARAMS[pi].p = val; deriveW(); }
    Serial.print("# "); Serial.print(PARAMS[pi].name); Serial.print(" = "); Serial.print(*PARAMS[pi].p, 4); Serial.print("   "); Serial.println(PARAMS[pi].what);
    if (pi == findParam("r") || pi == findParam("lam") || pi == findParam("c0") || pi == findParam("kv") || pi == findParam("wmode"))
      { Serial.print("#   → w = ["); Serial.print(W_PHI, 4); Serial.print(", 1, "); Serial.print(W_PHIDOT, 4); Serial.print(", "); Serial.print(W_BETADOT, 4); Serial.print("]  A_offset "); Serial.println(A_OFFSET, 4); }
    return;
  }
  if (tl != 1) { Serial.print("# 모르는 이름: "); Serial.println(tok); return; }
  switch (tok[0]) {
    case 'z': {
      if (running) { Serial.println("# h 로 멈추고 영점을 잡을 것"); break; }
      uint16_t pz = as5047_raw(PHI_CS), az = as5047_raw(ANK_CS);
      if (zero_stage == 0) { phi_z1 = pz; ank_z1 = az; phi_zero = pz; ank_zero = az; zero_stage = 1;
        Serial.println("# ZERO 1차 기록 — 반대쪽에서 정착시킨 뒤 z 한 번 더 (지금은 1차값이 임시 영점)"); }
      else if (zero_stage == 1) { phi_zero = circMean14(phi_z1, pz); ank_zero = circMean14(ank_z1, az); zero_stage = 2;
        int16_t dd = (int16_t)((pz - phi_z1) & 0x3FFF); if (dd > 8191) dd -= 16384;
        Serial.print("# ZERO 완성 — 데드밴드 phi "); Serial.print(fabsf(dd) * 360.0f / 16384.0f, 2); Serial.println(" deg (양쪽 정착 차)"); }
      else { phi_z1 = pz; ank_z1 = az; phi_zero = pz; ank_zero = az; zero_stage = 1; Serial.println("# ZERO 다시 시작 — 1차 기록"); }
      if (motor_ok) { home_tick = dxl.getPresentPosition(DXL_ID); if (home_tick == 0.0f) Serial.println("# !! 영점 순간 모터가 안 읽혔다 — z 다시"); }
      hold = 0; delta_now = 0; primed = false; dphi = dbeta = 0;
      Serial.print("E,"); Serial.print(millis() - t0); Serial.print(",ZERO,"); Serial.print(pz); Serial.print('/'); Serial.println(az);
      break;
    }
    case 'u': running = false; phase = ARMED_OFF; if (motor_ok) dxl.torqueOff(DXL_ID); Serial.println("# 토크 해제"); break;
    case 'k': if (motor_ok) { home_tick = dxl.getPresentPosition(DXL_ID); hold = 0; dxl.setGoalPosition(DXL_ID, home_tick); dxl.torqueOn(DXL_ID); } phase = ARMED_OFF; Serial.println("# 토크 ON (현재 위치 유지, δ=0 재정의)"); break;
    case 'g': doGo(); break;
    case 'h': running = false; phase = ARMED_OFF; Serial.println("# 정지 (토크·자세 유지)"); emitE("STOP", 0, 0); break;
    case 'y': if (running) { Serial.println("# 제어 중에는 전환 금지 — h 먼저"); break; } dry_run = !dry_run; Serial.print("# dry-run "); Serial.println(dry_run ? "ON (모터 명령 안 나감 — 부호 확인용)" : "OFF"); break;
    case 'm': csv_on = !csv_on; if (csv_on) logHeader(); Serial.print("# CSV "); Serial.println(csv_on ? "ON" : "OFF"); break;
    case 's': out_on = !out_on; Serial.println(out_on ? "# 출력 재개" : "# 출력 정지"); break;
    case 'p': logLine(); break;
    case 't': printStatus(); break;
    case 'w': printAllParams(); break;
    case 'e':
      Serial.println("# ---- 엔코더 진단 (AS5047P 레지스터) ----");
      encDiag("phi", PHI_CS);  encDiag("ank", ANK_CS);
      Serial.print("# 현재 raw: phi "); Serial.print(phi_raw); Serial.print("  ank "); Serial.print(ank_raw);
      Serial.print("   마지막 변화: phi "); Serial.print(millis() - phi_chg_ms); Serial.print(" ms 전, ank "); Serial.print(millis() - ank_chg_ms); Serial.println(" ms 전");
      break;
    default:  Serial.print("# 모르는 명령: "); Serial.println(s); break;
  }
}
void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == 'x' || ch == 'X') { torqueOffAll("사용자 x"); emitE("STOP", -1, 0); linelen = 0; continue; }   // 비상정지는 줄 끝을 안 기다린다
    if (ch == '\n' || ch == '\r') { if (linelen) { linebuf[linelen] = 0; linelen = 0; handleLine(linebuf); } continue; }
    if (linelen < sizeof(linebuf) - 1) linebuf[linelen++] = ch; else linelen = 0;
    last_rx_ms = millis();
  }
  if (linelen && (millis() - last_rx_ms) >= 200) { linebuf[linelen] = 0; linelen = 0; handleLine(linebuf); }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(PHI_CS, OUTPUT); digitalWrite(PHI_CS, HIGH);
  pinMode(ANK_CS, OUTPUT); digitalWrite(ANK_CS, HIGH);
#ifdef BDPIN_DXL_PWR_EN
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT); digitalWrite(BDPIN_DXL_PWR_EN, HIGH); delay(300);
#endif
  SPI.begin();
  deriveW();
  dxl.setPortProtocolVersion(2.0);
  const uint32_t bauds[] = {1000000, 57600};
  for (int b = 0; b < 2 && !motor_ok; b++) {
    dxl.begin(bauds[b]);
    for (int i = 0; i < 3 && !motor_ok; i++) { if (dxl.ping(DXL_ID)) motor_ok = true; else delay(200); }
  }
  if (motor_ok) {
    dxl.torqueOff(DXL_ID); delay(50);
    dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION); delay(50);
    dxl.writeControlTableItem(RETURN_DELAY_TIME,    DXL_ID, 0);
    dxl.writeControlTableItem(PROFILE_VELOCITY,     DXL_ID, VEL_UNIT);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, ACC_UNIT);
    dxl.writeControlTableItem(CURRENT_LIMIT,        DXL_ID, CUR_LIMIT); delay(80);
    home_tick = dxl.getPresentPosition(DXL_ID);
    Serial.println("# 모터 OK (토크는 켜지 않았다 — 매달림 영점은 이대로, 잡으려면 k)");
  } else Serial.println("# !!! 모터 응답 없음 — 배터리/RS-485 확인 (엔코더 스트리밍은 계속)");
  Serial.println("# v22_raw v2 — z u k <정수> m s p t w e swap hdr | mode N · g · h · x · y · fold X | 이름 값 (gam trig r lam c0 kv …)");
  logHeader();
  t0 = millis();  next_us = micros();
}

void loop() {
  pollSerial();
  uint32_t now = micros();
  if ((int32_t)(now - next_us) < 0) return;                // 200 Hz 페이싱
  next_us += DT_US;
  if ((int32_t)(micros() - next_us) >= 0) next_us = micros() + DT_US;
  readState();
  if (running && (fabsf(phi_d) > ANG_LIMIT || fabsf(alpha_d) > ANG_LIMIT)) {
    torqueOffAll("한계각 초과 (넘어짐) — k 또는 g 로 복구");
    emitE("FALL", phi_d, 2);
  }
  if (running) controlStep();
  uint32_t ms = millis();
  if (csv_on && out_on && (int32_t)(ms - hdr_next_ms) >= 0) { hdr_next_ms = ms + 20000; logHeader(); }
  if (csv_on && out_on) {
    if ((int32_t)(ms - log_next_ms) >= 0) {
      log_next_ms += (uint32_t)(1000.0f / LOG_HZ);
      if ((int32_t)(ms - log_next_ms) > 100) log_next_ms = ms;
      logLine();
    }
  } else if (out_on && (int32_t)(ms - mon_next) >= 0) {
    mon_next = ms + 250;
    Serial.print("f="); Serial.print(phi_d, 2); Serial.print(" k="); Serial.print(ank_d, 2);
    Serial.print(" d="); Serial.print(delta_now, 2); Serial.print(" | a="); Serial.print(alpha_d, 2);
    Serial.print(" b="); Serial.print(beta_d, 2); Serial.print(" A="); Serial.println(Ahat, 3);
  }
}
