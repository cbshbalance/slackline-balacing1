/*
 * v22_raw.ino — 측정 전용 최소 펌웨어 (원시값 스트리밍) — 2026-09-02 초안 ★미컴파일·실물검증 대기
 * ============================================================================
 *  v22 로거·측정 앱(README_v22 §다음 단계)이 기대하는 형식. 제어(접기)는 없다.
 *  하는 일: 엔코더 2개(φ, 발목) + 서보 δ 를 200 Hz 로 읽어 **원시값 위주**로 한 줄씩 찍는다.
 *  α·β·속도·Â 도 같이 찍지만(펌웨어 열 _fw 로 앱이 나란히 보인다) 정본은 원시 열이다 — 앱이 다시 계산한다.
 *
 *  용도: 매달림 영점(문서 66) · P2R(δ 계단, 문서 64) · 자유비행 λ·φ_eq·c₀(δ 고정, 문서 70) · 매달림 자유흔들기(ω, ζ)
 *
 *  【CSV】  '# D,…' 헤더 주석 뒤 D행. 앱과 hangcal_logger 가 이 헤더를 그대로 열 이름으로 쓴다.
 *      D,t_ms,phi,ank,alpha,beta,dphi,dbeta,Ahat,hold,del_now,phase,cue,err,phi_raw,ank_raw,dxl_raw
 *      phi_raw/ank_raw = AS5047 14비트 원각(영점 전), dxl_raw = 서보 present position tick — 영점을 사후에 검토할 수 있다.
 *
 *  【명령】 115200. 줄바꿈 또는 200 ms 조용하면 실행.
 *      z        영점 — 두 번 눌러 완성 (문서 66: 왼쪽에서 정착 → z, 오른쪽에서 정착 → z → 두 원각의 원형 평균).
 *               세 번째 z 는 처음부터 다시. 한 번만 누르고 쓰면 1차값이 임시 영점이다.
 *      u / k    토크 해제 / 토크 켜기(현재 위치 유지)
 *      <정수>   δ 명령 [°] (dlim 클립)          m  CSV 토글     s  화면출력 토글
 *      p        한 줄 출력                       t  상태        hdr  헤더 다시 출력 (앱이 늦게 붙았을 때)
 *      e        ★엔코더 진단 — 두 AS5047 의 ERRFL·DIAAGC(AGC, MagL/MagH/COF/LF)·ANGLEUNC·ANGLECOM 을 읽어 판독
 *      swap     ★CS 핀 교환 (φ↔발목) — 재업로드 없이 "채널(배선·센서) 문제 vs 코드" 를 가른다
 *      err 열:  phi 등급 + 4·ank 등급 + 16·dxl,  등급 1 = 0/16383 고착(레일), 2 = 2 s 이상 값 불변(고착)
 *      (헤더는 20 s 마다 자동으로 다시 찍는다 — 앱은 같은 헤더를 무시한다)
 *      loghz N  CSV 주기 (115200 에서 100 권장)  vel N / acc N  프로파일   ilim N  전류제한
 *  I/O 코드는 incremental_fold_min.ino(실기 검증됨)에서 그대로 가져왔다. 파이프라인 상수도 같다.
 * ============================================================================
 */
#include <SPI.h>
#include <Dynamixel2Arduino.h>
#include <math.h>

// ---- 실측 상수 (앱 파이프라인 기본값과 같다 — 여기 값은 _fw 열에만 영향) ----
const float P2R = 0.4285f, LAMBDA = 5.66f, R_SLOPE = -1.506f, LINE_C = 0.0f;
const float WV_PHI = 0.1945f, WV_BETA = 0.3049f, VGAIN = 1.0f;        // finale8 방식 (문서 73)
const float PHI_SIGN = +1.0f, ANK_SIGN = +1.0f;                        // 문서 65: 센서 읽는 줄에서만

// ---- 런타임 조정값 ----
float LOG_HZ = 100.0f;
int   VEL_UNIT = 250, ACC_UNIT = 373, CUR_LIMIT = 350;
const float DELTA_LIM = 55.0f;

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
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// ---- 상태 ----
bool  motor_ok = false, csv_on = true, out_on = true;
float home_tick = 0;  uint16_t phi_zero = 0, ank_zero = 0, phi_raw = 0, ank_raw = 0;
int   zero_stage = 0;  uint16_t phi_z1 = 0, ank_z1 = 0;          // 2단 영점 (문서 66 양방향 정착 평균)
uint16_t circMean14(uint16_t a, uint16_t b) {                      // 14비트 원각 두 개의 원형 평균
  int16_t d = (int16_t)((b - a) & 0x3FFF); if (d > 8191) d -= 16384;
  return (uint16_t)((a + d / 2) & 0x3FFF);
}
float dxl_raw = 0;
uint16_t phi_last = 0xFFFF, ank_last = 0xFFFF;  uint32_t phi_chg_ms = 0, ank_chg_ms = 0;   // 고착 판정
const uint32_t STUCK_MS = 2000;
float hold = 0, delta_now = 0;
float phi_d = 0, ank_d = 0, alpha_d = 0, beta_d = 0, dphi = 0, dbeta = 0, Ahat = 0;
float phi_hist[VEL_N + 1], beta_hist[VEL_N + 1];
int   hist_i = 0;  bool primed = false;
uint32_t t0 = 0, next_us = 0, log_next_ms = 0, mon_next = 0, hdr_next_ms = 0;
char  linebuf[48]; uint8_t linelen = 0; uint32_t last_rx_ms = 0;
int   err_code = 0;

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
// ---- AS5047P 레지스터 읽기 (진단) — 명령 프레임: bit15 짝수패리티, bit14 읽기=1, bit13..0 주소. 응답은 다음 프레임에 온다.
uint16_t as5047_frame(uint8_t cs, uint16_t frame) {
  uint16_t v;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(cs, LOW);  delayMicroseconds(1);
  v = SPI.transfer16(frame);
  digitalWrite(cs, HIGH); delayMicroseconds(1);
  SPI.endTransaction();
  return v;
}
uint16_t as5047_parity(uint16_t v) {              // bit14..0 의 1 개수가 홀수면 bit15 세움
  uint16_t x = v & 0x7FFF; int n = 0;
  while (x) { n += x & 1; x >>= 1; }
  return (n & 1) ? 0x8000 : 0;
}
uint16_t as5047_reg(uint8_t cs, uint16_t addr, bool* ef) {   // 레지스터 값 (14비트) + 응답 EF 비트
  uint16_t cmd = 0x4000 | (addr & 0x3FFF);  cmd |= as5047_parity(cmd);
  as5047_frame(cs, cmd);
  uint16_t r = as5047_frame(cs, 0x0000);        // NOP 로 응답을 클럭아웃
  if (ef) *ef = (r & 0x4000) != 0;
  return r & 0x3FFF;
}
void encDiag(const char* name, uint8_t cs) {
  bool ef1, ef2, ef3, ef4;
  uint16_t com  = as5047_reg(cs, 0x3FFF, &ef1);  // ANGLECOM (보정각)
  uint16_t unc  = as5047_reg(cs, 0x3FFE, &ef2);  // ANGLEUNC (미보정각)
  uint16_t diag = as5047_reg(cs, 0x3FFC, &ef3);  // DIAAGC
  uint16_t errf = as5047_reg(cs, 0x0001, &ef4);  // ERRFL (읽으면 지워짐)
  Serial.print("# ENC "); Serial.print(name); Serial.print(" (CS D"); Serial.print(cs); Serial.print("): ");
  Serial.print("ANGLECOM="); Serial.print(com); Serial.print(" ANGLEUNC="); Serial.print(unc);
  Serial.print(" | DIAAGC raw="); Serial.print(diag);
  Serial.print(" AGC="); Serial.print(diag & 0xFF);
  Serial.print(" LF="); Serial.print((diag >> 8) & 1);
  Serial.print(" COF="); Serial.print((diag >> 9) & 1);
  Serial.print(" MagH="); Serial.print((diag >> 10) & 1);
  Serial.print(" MagL="); Serial.print((diag >> 11) & 1);
  Serial.print(" | ERRFL="); Serial.print(errf);
  Serial.print(" EF="); Serial.println((ef1 || ef2 || ef3 || ef4) ? 1 : 0);
  if ((diag == 0 && com == 0) || (diag == 0x3FFF && com == 0x3FFF))
    Serial.println("#    → 응답 없음(전부 0 또는 16383): MISO/CS/전원 배선·커넥터 (문서 52)");
  else if ((diag >> 11) & 1 || (diag & 0xFF) == 0xFF)
    Serial.println("#    → MagL/AGC=255: 자석이 멀거나 없음 — 자석 마운트·갭 확인");
  else if ((diag >> 10) & 1 || (diag & 0xFF) == 0)
    Serial.println("#    → MagH/AGC=0: 자석이 너무 가까움");
  else if (!((diag >> 8) & 1))
    Serial.println("#    → LF=0: 내부 보정 루프 미완 — 전원 직후거나 자석 불안정");
  else
    Serial.println("#    → 센서는 정상 응답 (자석·통신 OK). 값이 안 변하면 자석이 관절과 같이 돌지 않는 기구 문제");
}
float rawToDeg(uint16_t raw, uint16_t zero) {
  int16_t d = (int16_t)((raw - zero) & 0x3FFF);
  if (d > 8191) d -= 16384;
  return d * (360.0f / 16384.0f);
}

// ---- 모터 ----
void writeGoal(float deg) {
  if (motor_ok) dxl.setGoalPosition(DXL_ID, home_tick + MOTOR_DIR * deg * TICK_PER_DEG);
}
float readDelta() {
  if (!motor_ok) return hold;
  float t = dxl.getPresentPosition(DXL_ID);
  if (t == 0.0f) { err_code |= 16; return delta_now; }
  dxl_raw = t;
  return MOTOR_DIR * (t - home_tick) / TICK_PER_DEG;
}

// ---- 읽기 + 펌웨어 파생값 (_fw 열) ----
void readState() {
  phi_raw = as5047_raw(PHI_CS);  ank_raw = as5047_raw(ANK_CS);
  uint32_t nowms = millis();
  if (phi_raw != phi_last) { phi_last = phi_raw; phi_chg_ms = nowms; }
  if (ank_raw != ank_last) { ank_last = ank_raw; ank_chg_ms = nowms; }
  err_code = 0;
  if (phi_raw == 0 || phi_raw == 0x3FFF) err_code |= 1;      // 레일 고착 = MISO 배선 (문서 52)
  else if (nowms - phi_chg_ms > STUCK_MS) err_code |= 2;     // 값 불변 고착
  if (ank_raw == 0 || ank_raw == 0x3FFF) err_code |= 4;
  else if (nowms - ank_chg_ms > STUCK_MS) err_code |= 8;
  phi_d = PHI_SIGN * rawToDeg(phi_raw, phi_zero);
  ank_d = ANK_SIGN * rawToDeg(ank_raw, ank_zero);
  delta_now = readDelta();
  alpha_d = ank_d - phi_d;                       // 문서 70 §2
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
  Ahat = (-1.0f / R_SLOPE) * phi_d + beta_d + VGAIN * WV_PHI * dphi + VGAIN * WV_BETA * dbeta + LINE_C / R_SLOPE;
}

void logHeader() {
  Serial.println("# D,t_ms,phi,ank,alpha,beta,dphi,dbeta,Ahat,hold,del_now,phase,cue,err,phi_raw,ank_raw,dxl_raw");
  Serial.println("# v22_raw: phase 0 고정(제어 없음). err = phi고착1 + ank고착4 + dxl16. phi_raw/ank_raw 는 영점 전 14비트");
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
  Serial.print("0,0,");        Serial.print(err_code); Serial.print(',');
  Serial.print(phi_raw);       Serial.print(',');
  Serial.print(ank_raw);       Serial.print(',');
  Serial.println((long)dxl_raw);
}
void printStatus() {
  Serial.println("---- v22_raw 상태 ----");
  Serial.print("모터: "); Serial.print(motor_ok ? "OK" : "응답 없음");
  if (motor_ok) { Serial.print("  torque="); Serial.print(dxl.getTorqueEnableStat(DXL_ID) ? "ON" : "OFF"); }
  Serial.println();
  Serial.print("CSV "); Serial.print(csv_on ? "ON" : "OFF"); Serial.print("  loghz "); Serial.print(LOG_HZ, 0);
  Serial.print("  vel "); Serial.print(VEL_UNIT); Serial.print("  acc "); Serial.print(ACC_UNIT);
  Serial.print("  ilim "); Serial.println(CUR_LIMIT);
  Serial.print("영점 raw: phi "); Serial.print(phi_zero); Serial.print("  ank "); Serial.print(ank_zero);
  Serial.print("  (단계 "); Serial.print(zero_stage); Serial.print("/2)");
  Serial.print("  home_tick "); Serial.println(home_tick, 0);
}

// ---- 명령 ----
void handleLine(char* s) {
  while (*s == ' ') s++;
  if (!*s) return;
  char c = *s;
  if ((c >= '0' && c <= '9') || c == '+' || c == '-') {
    long v = atol(s);
    if (v >  (long)DELTA_LIM) v =  (long)DELTA_LIM;
    if (v < -(long)DELTA_LIM) v = -(long)DELTA_LIM;
    if (!motor_ok) { Serial.println("# 모터 무응답"); return; }
    if (!dxl.getTorqueEnableStat(DXL_ID)) { home_tick = dxl.getPresentPosition(DXL_ID); dxl.torqueOn(DXL_ID); }
    hold = (float)v;  writeGoal(hold);
    Serial.print("E,"); Serial.print(millis() - t0); Serial.print(",MOVE,"); Serial.println(v);   // 앱·p2r_fit 이 구간 경계로 쓴다
    return;
  }
  char* rest = s + 1;  while (*rest == ' ') rest++;
  if (!strncmp(s, "hdr", 3))   { logHeader(); return; }
  if (!strncmp(s, "swap", 4))  { uint8_t t = PHI_CS; PHI_CS = ANK_CS; ANK_CS = t; primed = false;
                                 Serial.print("# CS 교환: phi←D"); Serial.print(PHI_CS); Serial.print("  ank←D"); Serial.println(ANK_CS);
                                 Serial.println("#   지금 'phi' 열이 예전 발목 채널이다. 12000 고정이 phi 로 옮겨가면 채널(배선·센서) 문제, 그대로면 코드 쪽");
                                 Serial.print("E,"); Serial.print(millis() - t0); Serial.print(",SWAP,"); Serial.println(PHI_CS); return; }
  if (!strncmp(s, "loghz", 5)) { LOG_HZ = atof(s + 5); if (LOG_HZ < 1) LOG_HZ = 1; if (LOG_HZ > 200) LOG_HZ = 200; Serial.print("# loghz "); Serial.println(LOG_HZ, 0); return; }
  if (!strncmp(s, "vel", 3))   { VEL_UNIT = atoi(s + 3); if (motor_ok) dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, VEL_UNIT); Serial.print("# vel "); Serial.println(VEL_UNIT); return; }
  if (!strncmp(s, "acc", 3))   { ACC_UNIT = atoi(s + 3); if (motor_ok) dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, ACC_UNIT); Serial.print("# acc "); Serial.println(ACC_UNIT); return; }
  if (!strncmp(s, "ilim", 4))  { CUR_LIMIT = atoi(s + 4); if (motor_ok) { dxl.torqueOff(DXL_ID); dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID, CUR_LIMIT); dxl.torqueOn(DXL_ID); } Serial.print("# ilim "); Serial.println(CUR_LIMIT); return; }
  switch (c) {
    case 'z': {
      uint16_t pz = as5047_raw(PHI_CS), az = as5047_raw(ANK_CS);
      if (zero_stage == 0) {                                    // 1차: 임시 영점으로 바로 쓴다
        phi_z1 = pz; ank_z1 = az; phi_zero = pz; ank_zero = az; zero_stage = 1;
        Serial.println("# ZERO 1차 기록 — 반대쪽에서 정착시킨 뒤 z 한 번 더 (지금은 1차값이 임시 영점)");
      } else if (zero_stage == 1) {                             // 2차: 두 정착의 원형 평균
        phi_zero = circMean14(phi_z1, pz); ank_zero = circMean14(ank_z1, az); zero_stage = 2;
        Serial.print("# ZERO 완성 — 데드밴드 phi "); Serial.print(fabsf((int16_t)((pz - phi_z1) & 0x3FFF) > 8191 ? 0 : (int16_t)((pz - phi_z1) & 0x3FFF)) * 360.0f / 16384.0f, 2);
        Serial.println(" deg (양쪽 정착 차)");
      } else {                                                  // 3차: 처음부터
        phi_z1 = pz; ank_z1 = az; phi_zero = pz; ank_zero = az; zero_stage = 1;
        Serial.println("# ZERO 다시 시작 — 1차 기록");
      }
      if (motor_ok) { home_tick = dxl.getPresentPosition(DXL_ID); if (home_tick == 0.0f) Serial.println("# !! 영점 순간 모터가 안 읽혔다 — z 다시"); }
      hold = 0; delta_now = 0; primed = false; dphi = dbeta = 0;
      Serial.print("E,"); Serial.print(millis() - t0); Serial.print(",ZERO,"); Serial.print(pz); Serial.print('/'); Serial.println(az);
      break;
    }
    case 'u': if (motor_ok) dxl.torqueOff(DXL_ID); Serial.println("# 토크 해제"); break;
    case 'k': if (motor_ok) { home_tick = dxl.getPresentPosition(DXL_ID); hold = 0; dxl.setGoalPosition(DXL_ID, home_tick); dxl.torqueOn(DXL_ID); } Serial.println("# 토크 ON (현재 위치 유지, δ=0 재정의)"); break;
    case 'm': csv_on = !csv_on; if (csv_on) logHeader(); Serial.print("# CSV "); Serial.println(csv_on ? "ON" : "OFF"); break;
    case 's': out_on = !out_on; Serial.println(out_on ? "# 출력 재개" : "# 출력 정지"); break;
    case 'p': logLine(); break;
    case 'e':
      Serial.println("# ---- 엔코더 진단 (AS5047P 레지스터) ----");
      encDiag("phi", PHI_CS);  encDiag("ank", ANK_CS);
      Serial.print("# 현재 raw: phi "); Serial.print(phi_raw); Serial.print("  ank "); Serial.print(ank_raw);
      Serial.print("   마지막 변화: phi "); Serial.print(millis() - phi_chg_ms); Serial.print(" ms 전, ank "); Serial.print(millis() - ank_chg_ms); Serial.println(" ms 전");
      break;
    case 't': printStatus(); break;
    default:  Serial.print("# 모르는 명령: "); Serial.println(s); break;
  }
}
void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
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
  Serial.println("# v22_raw — z 영점 / u 토크해제 / k 토크ON / <정수> δ / m CSV / s 출력 / p / t / e 엔코더진단 / swap CS교환 / hdr / loghz vel acc ilim");
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
  uint32_t ms = millis();
  if (csv_on && out_on && (int32_t)(ms - hdr_next_ms) >= 0) { hdr_next_ms = ms + 20000; logHeader(); }
  if (csv_on && out_on) {
    if ((int32_t)(ms - log_next_ms) >= 0) {
      log_next_ms += (uint32_t)(1000.0f / LOG_HZ);
      if ((int32_t)(ms - log_next_ms) > 100) log_next_ms = ms;
      logLine();
    }
  } else if (out_on && (int32_t)(ms - mon_next) >= 0) {   // CSV 꺼진 동안 4 Hz 모니터
    mon_next = ms + 250;
    Serial.print("f="); Serial.print(phi_d, 2); Serial.print(" k="); Serial.print(ank_d, 2);
    Serial.print(" d="); Serial.print(delta_now, 2); Serial.print(" | a="); Serial.print(alpha_d, 2);
    Serial.print(" b="); Serial.print(beta_d, 2); Serial.print(" A="); Serial.println(Ahat, 3);
  }
}
