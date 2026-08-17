/*
 * p2r_test.ino — P2R 실측 + 배선 진단 (매달고 δ 명령 → 발목각 α 읽기)
 * ============================================================================
 * 문서 46 「실측 ① — P2R」용 스케치.  2026-08-17 진단 기능 추가판.
 *
 *   매달린 상태의 평형 = 무게중심이 발목축 바로 아래 = β = 0
 *   β = P1R·α + P2R·θ = 0,   θ = α + δ,   P1R + P2R = 1
 *      →   α = −P2R · δ        ★ α–δ 직선의 기울기가 −P2R
 *   예측 기울기 = −0.4122
 *
 * ----------------------------------------------------------------------------
 * [조작]  시리얼 모니터 115200 baud.  ★정수만 치고 Enter 하면 그 각도로 간다.
 *      20 → δ=+20°,   -20 → δ=−20°,   0 → 영점 복귀   (z 영점 기준 절대각)
 *   줄바꿈(Newline) 설정을 안 해도 동작 — 마지막 글자 후 200 ms 지나면 자동 실행.
 *
 * [측정 키]
 *   s   출력 정지 / 재개 (정지 중에도 명령은 먹음)
 *   z   영점: 엔코더 2개 + 모터 홈(δ=0).  ★완전히 멎은 상태에서만
 *   p   현재값 1회 출력      m  출력 형식: 읽기용 4Hz ↔ 100Hz CSV  
 *   v 30 / a 10  프로파일 속도·가속도       u / k  토크 해제 / 복구
 *   x   비상정지 (즉시)      t  설정·상태 요약
 *
 * [★진단 키]  — 배선 끊김·통신 오류를 잡을 때
 *   w   진단 표시 ON/OFF — 각도 대신 **raw 카운트와 오류 카운터**를 띄운다.
 *       진단 중에는 엔코더를 500 Hz 로 두들겨 읽으므로 순간 끊김도 잡힌다.
 *       ★하네스를 흔들거나 허리를 접어 가며 카운터가 오르는지 본다.
 *   e   엔코더 자가진단 레지스터 1회 읽기 (ERRFL + DIAAGC)
 *       ERRFL  → 센서가 받은 명령이 깨졌나 (MOSI·SCLK·CS 쪽)
 *       DIAAGC → 자석이 너무 멀거나(MAGL) 가깝나(MAGH), AGC 값
 *   r   오류 카운터 리셋
 *   b   RS-485 보레이트 전환 (1 M ↔ 57600) — ★1 M 이 노이즈에 약하다
 *   c   SPI 클럭 순환 (1 MHz → 500 kHz → 250 kHz) — ★길고 안 감싼 하네스면 낮출 것
 *
 * ----------------------------------------------------------------------------
 * [★진단 원리 — 왜 값이 '126°' 로 고정되는가]
 *   AS5047P 가 응답을 못 하면 MISO 가 붕 떠서 0xFFFF 가 읽히고, 각도부는 16383.
 *   그런데 화면은 (raw − 영점) 을 보여주므로 16383 이 엉뚱한 각도로 위장된다.
 *   phi_zero = 10649 이면 railed 값이 정확히 +126.0° 로 보인다 — 우연이 아니다.
 *   그래서 이 판은 **raw 를 같이 띄우고, 레일·EF·패리티를 따로 센다.**
 *
 *   판정 요령
 *     raw 가 16383 고정  → 센서가 응답 못 함 (CS 끊김 / MISO 끊김 / 전원 순단)
 *     raw 가 0 고정      → MISO 가 로우로 붙음 (단락 / 센서 무응답)
 *     ERRFL 에 오류 有   → 명령이 깨져서 도착 (MOSI·SCLK·CS 쪽)
 *     ERRFL 깨끗 + 레일  → 응답 경로가 끊김 (MISO 쪽 / CS 가 선택을 못 함)
 *     DIAAGC 에 MAGL     → 자석이 멀다 (공극 1~2 mm 확인)
 *
 *   ★어느 선인지 가르는 법: 두 엔코더는 SCLK·MISO·MOSI 를 공유하고 CS 만 따로다.
 *     φ 와 발목이 **같이** 이상해지면  → 공유선 (D11·D12·D13) 또는 공통 전원
 *     φ 만 이상해지면                  → φ 의 CS(D10) / φ 쪽 가지 / 센서·자석
 *
 * ----------------------------------------------------------------------------
 * [배선] φ CS=D10, 발목 CS=D9, SCLK=D13, MISO=D12, MOSI=D11
 *        모터 XM430-W210-R, RS-485 4핀, ID 1.  ★배터리 필수
 * [부호] 문서 37 (8/5 확정): α = ank + phi,  MOTOR_DIR = +1
 * ============================================================================
 */
#include <SPI.h>
#include <Dynamixel2Arduino.h>

// ---------------- 하드웨어 ----------------
#define DXL_SERIAL   Serial3
#define DXL_DIR_PIN  84
const uint8_t DXL_ID  = 1;

const uint8_t PHI_CS  = 10;
const uint8_t ANK_CS  = 9;

// ---------------- 규약·상수 ----------------
const int   MOTOR_DIR       = +1;
const float TICK_PER_DEG    = 4096.0f / 360.0f;
const float VEL_UNIT_DPS    = 1.374f;
const float ACC_UNIT_DPS2   = 21.4577f;
const long  DELTA_LIMIT_DEG = 55;
const uint32_t PERIOD_US    = 10000;     // 100 Hz CSV
const uint32_t MON_MS       = 250;       // 4 Hz 표시
const uint32_t DIAG_US      = 2000;      // 500 Hz 진단 샘플링
const uint32_t LINE_TIMEOUT_MS = 200;

// AS5047P 레지스터
const uint16_t AS_ERRFL  = 0x0001;
const uint16_t AS_DIAAGC = 0x3FFC;
const uint16_t AS_ANGLE  = 0x3FFF;       // ANGLECOM

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// ---------------- 상태 ----------------
uint32_t dxl_baud = 0;
bool     motor_ok = false;
float    home_tick = 0;
long     delta_cmd = 0;
uint16_t phi_zero = 0, ank_zero = 0;
int      vel_unit = 30, acc_unit = 10;
uint32_t spi_hz = 1000000;

bool     out_on   = true;
bool     csv_mode = false;
bool     diag_on  = false;
uint32_t t0 = 0, next_us = 0, next_mon = 0, next_diag = 0;

char     linebuf[24];
uint8_t  linelen = 0;
uint32_t last_rx_ms = 0;

// 진단 카운터
struct EncStat {
  uint32_t n, rail, ef, par;
  uint16_t last_ok, last_raw;
  bool     seeded, fault;
};
EncStat st_phi = {0,0,0,0,0,0,false,false};
EncStat st_ank = {0,0,0,0,0,0,false,false};
uint32_t dxl_n = 0, dxl_fail = 0;
bool     dxl_fault = false;
float    dxl_last_ok = 0;

// ---------------- AS5047P ----------------
bool parityEven(uint16_t v) {             // 16비트 전체의 1 개수가 짝수여야 정상
  v ^= v >> 8; v ^= v >> 4; v ^= v >> 2; v ^= v >> 1;
  return (v & 1) == 0;
}

uint16_t as5047_cmd(uint16_t addr) {      // 읽기 명령 + 짝수 패리티
  uint16_t c = 0x4000 | (addr & 0x3FFF);
  if (!parityEven(c)) c |= 0x8000;
  return c;
}

uint16_t as5047_frame(uint8_t cs, uint16_t cmd) {
  uint16_t v;
  SPI.beginTransaction(SPISettings(spi_hz, MSBFIRST, SPI_MODE1));
  digitalWrite(cs, LOW);  delayMicroseconds(1);
  SPI.transfer16(cmd);                    // 명령 전송
  digitalWrite(cs, HIGH); delayMicroseconds(1);
  digitalWrite(cs, LOW);  delayMicroseconds(1);
  v = SPI.transfer16(as5047_cmd(AS_ANGLE));   // 응답 수신 (다음 명령과 겸함)
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  return v;
}

// 각도 읽기 + 검사. 이상하면 마지막 정상값을 유지하고 카운터를 올린다.
uint16_t encRead(uint8_t cs, EncStat& s) {
  uint16_t f = as5047_frame(cs, as5047_cmd(AS_ANGLE));
  uint16_t a = f & 0x3FFF;
  s.n++;
  s.last_raw = a;
  s.fault = false;
  if (!parityEven(f))            { s.par++;  s.fault = true; }
  else if (f & 0x4000)           { s.ef++;   s.fault = true; }   // EF 비트
  if (a == 0x3FFF || a == 0x0000){ s.rail++; s.fault = true; }   // ★배선 끊김 신호
  if (!s.fault) { s.last_ok = a; s.seeded = true; return a; }
  return s.seeded ? s.last_ok : a;
}

float rawToDeg(uint16_t raw, uint16_t zero) {
  int16_t d = (int16_t)((raw - zero) & 0x3FFF);
  if (d > 8191) d -= 16384;
  return d * (360.0f / 16384.0f);
}

float phiDeg() { return rawToDeg(encRead(PHI_CS, st_phi), phi_zero); }
float ankDeg() { return rawToDeg(encRead(ANK_CS, st_ank), ank_zero); }

// ---------------- 모터 ----------------
float deltaNow() {                        // 실패하면 마지막 정상값 유지 (3회 재시도)
  if (!motor_ok) { dxl_fault = true; return 0; }
  for (int i = 0; i < 3; i++) {
    float p = dxl.getPresentPosition(DXL_ID);
    dxl_n++;
    if (dxl.getLastLibErrCode() == 0) {
      dxl_fault = false;
      dxl_last_ok = MOTOR_DIR * (p - home_tick) / TICK_PER_DEG;
      return dxl_last_ok;
    }
    dxl_fail++;
  }
  dxl_fault = true;                       // ★3회 다 실패 — 값 대신 직전 값을 쓴다
  return dxl_last_ok;
}

void torqueRestoreHere() {
  if (!motor_ok) return;
  dxl.setGoalPosition(DXL_ID, dxl.getPresentPosition(DXL_ID));
  dxl.torqueOn(DXL_ID);
}

void goToDelta(long deg) {
  if (!motor_ok) { Serial.println("# 모터 연결 안 됨 — 이동 불가"); return; }
  if (deg > DELTA_LIMIT_DEG || deg < -DELTA_LIMIT_DEG) {
    Serial.print("# 범위 초과 — ±"); Serial.print(DELTA_LIMIT_DEG); Serial.println("° 로 잘림");
    deg = (deg > 0) ? DELTA_LIMIT_DEG : -DELTA_LIMIT_DEG;
  }
  if (!dxl.getTorqueEnableStat(DXL_ID)) {
    torqueRestoreHere();
    Serial.println("# 토크 복구됨 (홈은 z 시점 그대로 — 크게 움직일 수 있으니 주의)");
  }
  delta_cmd = deg;
  dxl.setGoalPosition(DXL_ID, home_tick + (float)(MOTOR_DIR * deg) * TICK_PER_DEG);
  Serial.print("E,"); Serial.print(millis() - t0);
  Serial.print(",MOVE,"); Serial.println(deg);
}

// ---------------- 출력 ----------------
void printSigned(float v, int nd) { if (v >= 0) Serial.print('+'); Serial.print(v, nd); }

void printState() {
  float phi = phiDeg();
  float ank = ankDeg();
  bool  fp = st_phi.fault, fa = st_ank.fault;
  float alpha = ank + phi;
  float dn = deltaNow();

  Serial.print("dcmd="); printSigned((float)delta_cmd, 0);
  Serial.print("  d=");  printSigned(dn, 2); if (dxl_fault) Serial.print("!");
  Serial.print(" | phi="); printSigned(phi, 2); if (fp) Serial.print("!");
  Serial.print("  ank=");  printSigned(ank, 2); if (fa) Serial.print("!");
  Serial.print("  alpha="); printSigned(alpha, 2);
  if (dn > 0.5f || dn < -0.5f) {           // 명령이 아니라 실제 δ 로 나눈다
    Serial.print(" | alpha/d="); Serial.print(alpha / dn, 3);
  }
  if (fp || fa || dxl_fault) {             // ★값이 아니라 '못 읽었다'는 표시
    Serial.print("   <<");
    if (fp) { Serial.print(" phi raw="); Serial.print(st_phi.last_raw); }
    if (fa) { Serial.print(" ank raw="); Serial.print(st_ank.last_raw); }
    if (dxl_fault) Serial.print(" dxl 통신실패");
    Serial.print(" >>");
  }
  Serial.println();
}

void printDiag() {
  Serial.print("RAW phi="); Serial.print(st_phi.last_raw);
  Serial.print(" ank=");    Serial.print(st_ank.last_raw);
  Serial.print(" | phi n="); Serial.print(st_phi.n);
  Serial.print(" rail=");    Serial.print(st_phi.rail);
  Serial.print(" EF=");      Serial.print(st_phi.ef);
  Serial.print(" par=");     Serial.print(st_phi.par);
  Serial.print(" | ank n="); Serial.print(st_ank.n);
  Serial.print(" rail=");    Serial.print(st_ank.rail);
  Serial.print(" EF=");      Serial.print(st_ank.ef);
  Serial.print(" par=");     Serial.print(st_ank.par);
  Serial.print(" | dxl n="); Serial.print(dxl_n);
  Serial.print(" fail=");    Serial.println(dxl_fail);
}

void printEncRegs(uint8_t cs, const char* name) {
  uint16_t err = as5047_frame(cs, as5047_cmd(AS_ERRFL))  & 0x3FFF;
  uint16_t dia = as5047_frame(cs, as5047_cmd(AS_DIAAGC)) & 0x3FFF;
  Serial.print(name);
  Serial.print("  ERRFL=0x"); Serial.print(err, HEX);
  if (err & 0x01) Serial.print(" [FRERR 프레이밍]");
  if (err & 0x02) Serial.print(" [INVCOMM 잘못된명령]");
  if (err & 0x04) Serial.print(" [PARERR 패리티]");
  if (!(err & 0x07)) Serial.print(" (깨끗 — 명령은 잘 도착함)");
  Serial.print("   DIAAGC=0x"); Serial.print(dia, HEX);
  Serial.print(" AGC="); Serial.print(dia & 0xFF);
  if (dia & (1 << 11)) Serial.print(" [MAGL 자석 멀다]");
  if (dia & (1 << 10)) Serial.print(" [MAGH 자석 가깝다]");
  if (dia & (1 <<  9)) Serial.print(" [COF 연산오버플로]");
  if (!(dia & 0x0E00)) Serial.print(" (자석 정상)");
  Serial.println();
}

void printStatus() {
  Serial.println("---- 상태 ----");
  Serial.print("모터: "); Serial.println(motor_ok ? "OK" : "응답 없음");
  if (motor_ok) {
    Serial.print("  RS485 baud="); Serial.print(dxl_baud);
    Serial.print("  torque="); Serial.println(dxl.getTorqueEnableStat(DXL_ID) ? "ON" : "OFF");
    Serial.print("  home_tick="); Serial.println(home_tick, 1);
  }
  Serial.print("SPI 클럭="); Serial.print(spi_hz / 1000); Serial.println(" kHz");
  Serial.print("프로파일: v="); Serial.print(vel_unit);
  Serial.print(" (~"); Serial.print(vel_unit * VEL_UNIT_DPS, 0); Serial.print(" deg/s)  a=");
  Serial.print(acc_unit);
  Serial.print(" (~"); Serial.print(acc_unit * ACC_UNIT_DPS2, 0); Serial.println(" deg/s^2)");
  Serial.print("영점 raw: phi_zero="); Serial.print(phi_zero);
  Serial.print("  ank_zero=");         Serial.println(ank_zero);
  Serial.print("출력="); Serial.print(out_on ? "ON" : "정지(s로 재개)");
  Serial.print("  형식="); Serial.print(csv_mode ? "100Hz CSV" : "읽기용 4Hz");
  Serial.print("  진단="); Serial.println(diag_on ? "ON" : "OFF");
  printDiag();
  Serial.println("--------------");
  printState();
}

void printHelp() {
  Serial.println("정수 입력 = 그 각도로 이동 (예: 20 / -20 / 0)");
  Serial.println("측정: s 출력정지/재개 | z 영점 | p 1회 | m 4Hz<->CSV | v<n> a<n> | u k | x 비상정지 | t 상태");
  Serial.println("진단: w 진단표시 | e 엔코더레지스터 | r 카운터리셋 | b RS485보레이트 | c SPI클럭");
}

// ---------------- 명령 ----------------
long argInt(const char* s, bool* found) {
  const char* q = s + 1;
  while (*q == ' ' || *q == '\t' || *q == '=') q++;
  if (*q == '\0') { *found = false; return 0; }
  *found = true;
  return atol(q);
}

void resetCounters() {
  st_phi.n = st_phi.rail = st_phi.ef = st_phi.par = 0;
  st_ank.n = st_ank.rail = st_ank.ef = st_ank.par = 0;
  dxl_n = dxl_fail = 0;
}

void switchDxlBaud() {
  if (!motor_ok) { Serial.println("# 모터 연결 안 됨"); return; }
  uint32_t nb = (dxl_baud == 1000000) ? 57600 : 1000000;
  bool torq = dxl.getTorqueEnableStat(DXL_ID);
  dxl.begin(nb);
  bool ok = false;
  for (int i = 0; i < 5 && !ok; i++) { ok = dxl.ping(DXL_ID); delay(100); }
  if (ok) {
    dxl_baud = nb;
    Serial.print("# RS485 보레이트 전환 -> "); Serial.println(nb);
    Serial.println("  (모터 쪽 설정과 맞아야 함. 응답 없으면 b 로 되돌리세요)");
    if (torq) torqueRestoreHere();
  } else {
    dxl.begin(dxl_baud);                  // 되돌리기
    Serial.print("# 전환 실패 — "); Serial.print(nb);
    Serial.print(" 에서 응답 없음. "); Serial.print(dxl_baud); Serial.println(" 유지");
  }
  resetCounters();
}

void cycleSpiClock() {
  if      (spi_hz == 1000000) spi_hz = 500000;
  else if (spi_hz == 500000)  spi_hz = 250000;
  else                        spi_hz = 1000000;
  Serial.print("# SPI 클럭 -> "); Serial.print(spi_hz / 1000); Serial.println(" kHz");
  Serial.println("  (낮출수록 노이즈에 강함. 낮췄더니 rail 이 줄면 신호 품질 문제)");
  resetCounters();
}

void handleLine(char* s) {
  while (*s == ' ' || *s == '\t') s++;
  if (*s == '\0') return;

  char c = *s;
  if ((c >= '0' && c <= '9') || c == '+' || c == '-') {
    bool digit = false;
    for (const char* q = s; *q; q++) if (*q >= '0' && *q <= '9') { digit = true; break; }
    if (!digit) { Serial.println("# 숫자가 없음 — 무시"); return; }
    goToDelta(atol(s));
    return;
  }
  if (c >= 'A' && c <= 'Z') c += 32;

  bool has; long n;
  switch (c) {
    case 's':
      out_on = !out_on;
      next_mon = millis(); next_us = micros();
      if (out_on) Serial.println("# 출력 재개 (s 로 다시 정지)");
      else { printState(); Serial.println("# 출력 정지 — s 로 재개 (명령은 계속 먹음)"); }
      break;

    case 'z':
      phi_zero = encRead(PHI_CS, st_phi);
      ank_zero = encRead(ANK_CS, st_ank);
      if (st_phi.fault || st_ank.fault)
        Serial.println("!!! 영점을 이상한 읽기로 잡았습니다 — w 로 진단 후 다시 z");
      if (motor_ok) home_tick = dxl.getPresentPosition(DXL_ID);
      delta_cmd = 0;
      Serial.print("# ZERO — phi_zero="); Serial.print(phi_zero);
      Serial.print(" ank_zero="); Serial.println(ank_zero);
      printState();
      break;

    case 'p': printState(); break;

    case 'm':
      csv_mode = !csv_mode;
      next_mon = millis(); next_us = micros();
      if (csv_mode) {
        Serial.println("# D,t_ms,phi_deg,ank_deg,alpha_deg,del_cmd_deg,del_now_deg");
        Serial.println("# 형식: 100Hz CSV (로거용)");
      } else Serial.println("# 형식: 읽기용 4Hz");
      break;

    case 'w':
      diag_on = !diag_on;
      next_mon = millis(); next_diag = micros();
      if (diag_on) {
        resetCounters();
        Serial.println("# 진단 ON — 엔코더를 500Hz 로 읽으며 오류를 셉니다.");
        Serial.println("#   하네스를 흔들거나 허리를 접어 가며 rail/EF/par 이 오르는지 보세요.");
        Serial.println("#   φ 와 발목이 같이 오르면 공유선(D11/D12/D13), φ 만이면 CS(D10)·φ 가지.");
      } else Serial.println("# 진단 OFF");
      break;

    case 'e':
      Serial.println("---- 엔코더 자가진단 ----");
      printEncRegs(PHI_CS, "phi(D10)");
      printEncRegs(ANK_CS, "ank(D9) ");
      Serial.println("ERRFL 오류 有 = 명령이 깨져 도착 (MOSI/SCLK/CS 쪽)");
      Serial.println("ERRFL 깨끗 + raw 레일 = 응답 경로 끊김 (MISO 쪽 / CS 선택 실패)");
      Serial.println("-------------------------");
      break;

    case 'r': resetCounters(); Serial.println("# 오류 카운터 리셋"); break;
    case 'b': switchDxlBaud(); break;
    case 'c': cycleSpiClock(); break;

    case 'v':
      n = argInt(s, &has);
      if (has) {
        vel_unit = (int)constrain(n, 1, 200);
        if (motor_ok) dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, vel_unit);
      }
      Serial.print("# vel="); Serial.print(vel_unit);
      Serial.print(" (~"); Serial.print(vel_unit * VEL_UNIT_DPS, 0); Serial.println(" deg/s)");
      break;

    case 'a':
      n = argInt(s, &has);
      if (has) {
        acc_unit = (int)constrain(n, 1, 200);
        if (motor_ok) dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, acc_unit);
      }
      Serial.print("# acc="); Serial.print(acc_unit);
      Serial.print(" (~"); Serial.print(acc_unit * ACC_UNIT_DPS2, 0); Serial.println(" deg/s^2)");
      break;

    case 'u':
      if (motor_ok) dxl.torqueOff(DXL_ID);
      Serial.println("# 토크 해제 — 손으로 돌려도 됨 (k 로 복구)");
      break;

    case 'k': torqueRestoreHere(); Serial.println("# 토크 복구 (현재 자리 유지)"); break;
    case 't': printStatus(); break;
    case '?': case 'h': printHelp(); break;

    default:
      Serial.print("# 모르는 명령: "); Serial.println(s);
      printHelp();
      break;
  }
}

void emergencyStop() {
  if (motor_ok) dxl.torqueOff(DXL_ID);
  linelen = 0;
  Serial.println(">>> EMERGENCY STOP (torque off) — k 로 복구");
}

void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == 'x' || ch == 'X') { emergencyStop(); continue; }
    if (ch == '\n' || ch == '\r') {
      if (linelen) { linebuf[linelen] = '\0'; linelen = 0; handleLine(linebuf); }
      continue;
    }
    if (ch == ' ' && linelen == 0) continue;
    if (linelen < sizeof(linebuf) - 1) linebuf[linelen++] = ch;
    else { linelen = 0; Serial.println("# 입력이 너무 김 — 버림"); }
    last_rx_ms = millis();
  }
  if (linelen && (millis() - last_rx_ms) >= LINE_TIMEOUT_MS) {
    linebuf[linelen] = '\0'; linelen = 0; handleLine(linebuf);
  }
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(PHI_CS, OUTPUT); digitalWrite(PHI_CS, HIGH);
  pinMode(ANK_CS, OUTPUT); digitalWrite(ANK_CS, HIGH);
  SPI.begin();

  Serial.println("=== p2r_test (P2R 실측 + 배선 진단) ===");

  dxl.setPortProtocolVersion(2.0);
  const uint32_t bauds[] = {1000000, 57600};
  for (int b = 0; b < 2 && !motor_ok; b++) {
    dxl.begin(bauds[b]);
    for (int i = 0; i < 3; i++) {
      if (dxl.ping(DXL_ID)) { motor_ok = true; dxl_baud = bauds[b]; break; }
      delay(200);
    }
  }

  if (motor_ok) {
    Serial.print("모터 OK @ "); Serial.println(dxl_baud);
    dxl.torqueOff(DXL_ID); delay(50);
    dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION); delay(50);
    dxl.writeControlTableItem(PROFILE_VELOCITY,     DXL_ID, vel_unit);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, acc_unit);
    home_tick = dxl.getPresentPosition(DXL_ID);
    dxl_last_ok = 0;
    dxl.setGoalPosition(DXL_ID, home_tick); delay(50);
    dxl.torqueOn(DXL_ID);
  } else {
    Serial.println("!!! 모터 응답 없음 (배터리/RS-485/보레이트 확인) — 엔코더만 동작");
  }

  uint16_t r1 = encRead(PHI_CS, st_phi), r2 = encRead(ANK_CS, st_ank);
  Serial.print("phi raw="); Serial.print(r1);
  Serial.print("  ank raw="); Serial.println(r2);
  if (st_phi.fault || st_ank.fault)
    Serial.println("!!! 부팅부터 엔코더 읽기 이상 — e 로 레지스터 확인, w 로 진단");
  printEncRegs(PHI_CS, "phi(D10)");
  printEncRegs(ANK_CS, "ank(D9) ");
  resetCounters();

  printHelp();
  Serial.println("순서: 매달고 멎으면 z -> 10 -> 20 -> 30 -> 40 -> 30 -> 20 -> 10 -> 0");
  Serial.println("기대: alpha/d = -0.412 근처   (화면이 빠르면 s 로 정지)");

  t0 = millis();
  next_mon = millis();
  next_us = next_diag = micros();
}

void loop() {
  pollSerial();

  // ★진단 모드: 표시와 무관하게 500 Hz 로 두들겨 순간 끊김을 잡는다
  if (diag_on) {
    uint32_t nowu = micros();
    if ((int32_t)(nowu - next_diag) >= 0) {
      next_diag += DIAG_US;
      encRead(PHI_CS, st_phi);
      encRead(ANK_CS, st_ank);
    }
  }

  if (!out_on) return;

  if (csv_mode) {
    uint32_t now = micros();
    if ((int32_t)(now - next_us) >= 0) {
      next_us += PERIOD_US;
      float phi = phiDeg(), ank = ankDeg();
      Serial.print("D,");
      Serial.print(millis() - t0);   Serial.print(",");
      Serial.print(phi, 3);          Serial.print(",");
      Serial.print(ank, 3);          Serial.print(",");
      Serial.print(ank + phi, 3);    Serial.print(",");
      Serial.print(delta_cmd);       Serial.print(",");
      Serial.println(deltaNow(), 3);
    }
  } else {
    uint32_t now = millis();
    if ((int32_t)(now - next_mon) >= 0) {
      next_mon += MON_MS;
      if (diag_on) printDiag();
      else         printState();
    }
  }
}
