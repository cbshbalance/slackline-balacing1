/*
 * motor_inertia_test.ino — 허리 유효관성 I_delta 실측 (실험 ③)
 * ============================================================
 * 목적: 허리 모터(XM430, ID1)에 **알려진 토크**를 계단으로 걸고, 그때의
 *       **각가속도**를 100Hz로 재서 tau = I*alpha + (마찰) + (중력) 로부터
 *       I_delta 를 구한다. 기어 너머 회전자 관성(armature x 45,200)이
 *       자동으로 포함되는 것이 이 방법의 핵심이다.
 *
 * 원리:
 *   자세를 '상체가 힙축 아래로 매달린' 안정 자세(delta=0)로 두고 시작한다.
 *   계단 토크 tau 를 걸면
 *        I_delta * delta_ddot = tau  -  m2*g*ell2*sin(delta)  -  tau_friction
 *   초기 구간(|delta| 작음)에서는 중력항이 작아 delta(t)가 거의 포물선이고,
 *   기울기 적합으로 alpha 가 나온다. tau 를 여러 단계로 바꿔
 *   alpha vs tau 직선적합 → 기울기 1/I_delta, 절편에서 마찰토크.
 *   (정밀 처리는 calib/inertia_analysis.py 가 중력항까지 넣어 적합한다)
 *
 * ★ 반드시 발이 바닥·줄에서 떠 있어야 한다(하체를 지그에 고정).
 *   바닥에 닿으면 반력이 섞여 무효.
 *
 * 배선: 모터 RS-485 4핀, ID 1, 12V 배터리 필수.
 *       엔코더(옵션): phi CS=D10, 발목 CS=D9, SCLK=D13, MISO=D12, MOSI=D11
 *
 * [시리얼 명령] (115200 baud)
 *   z         : 현재 자세를 delta=0 홈으로 (매달려 정지한 상태에서 치기)
 *   s         : 100Hz 스트리밍 토글  -> D,t_ms,delta_deg,cur_unit,phi_deg,ank_deg
 *   c 150     : Goal Current 를 150 unit 으로 (부호 가능)
 *   t 0.30    : 토크 0.30 N.m 명령 (공칭 Kt=1.304 로 환산)
 *   i 150     : ★자동 펄스 시험 — 홈에서 +150unit 을 걸고 |delta|>25도 또는
 *               1.2초까지 100Hz 기록 후 전류 0. E행에 개략 alpha 출력.
 *   k         : 자유 감쇠 시험 — 전류 0 상태로 스트리밍만 (손으로 20도 밀었다 놓기)
 *   r         : 전류 0 (정지)
 *   p         : 상태 1회 출력
 *   x         : 비상정지 (토크 OFF)
 *
 * 안전: |delta| > DELTA_LIMIT(45도) 이면 자동으로 전류 0.
 *       명령 전류는 +-CUR_SAFE(600unit, 약 1.6A)로 제한.
 */
#include <SPI.h>
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DXL_DIR_PIN  84
const uint8_t DXL_ID = 1;

const uint8_t PHI_CS = 10;
const uint8_t ANK_CS = 9;
const bool    USE_ENCODERS = true;   // 엔코더 미배선이면 false

const float CUR_UNIT    = 2.69f;     // mA/unit
const float KT_NOMINAL  = 1.304f;    // N.m/A (공칭)
const int   CUR_SAFE    = 600;       // unit, 약 1.6 A
const float DELTA_LIMIT = 45.0f;     // deg, 안전 각도 한계
const uint32_t PULSE_MS = 1200;      // 자동 펄스 최대 시간
const float PULSE_DEG   = 25.0f;     // 자동 펄스 목표 각도

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

uint32_t dxl_baud = 0;
bool  motor_ok = false;
float home_deg = 0;
int   goal_cur = 0;
bool  streaming = false;
bool  pulsing = false;
uint32_t pulse_t0 = 0;
float pulse_start_deg = 0;
uint32_t t0 = 0, next_us = 0;
const uint32_t PERIOD_US = 10000;    // 100 Hz

uint16_t phi_zero = 0, ank_zero = 0;

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

float rawToDeg(uint16_t raw, uint16_t zero) {
  int16_t d = (int16_t)((raw - zero) & 0x3FFF);
  if (d > 8191) d -= 16384;
  return d * (360.0f / 16384.0f);
}

float deltaDeg() {
  if (!motor_ok) return 0;
  return dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) - home_deg;
}

void setCur(int unit) {
  unit = constrain(unit, -CUR_SAFE, CUR_SAFE);
  goal_cur = unit;
  if (motor_ok) dxl.setGoalCurrent(DXL_ID, unit);
}

void printOnce() {
  Serial.print("delta="); Serial.print(deltaDeg(), 2);
  Serial.print(" deg  goalCur="); Serial.print(goal_cur);
  Serial.print("u (tau_cmd="); Serial.print(goal_cur*CUR_UNIT/1000.0f*KT_NOMINAL, 3);
  Serial.print(" N.m)  presentCur=");
  Serial.print(motor_ok ? (int)dxl.readControlTableItem(PRESENT_CURRENT, DXL_ID) : 0);
  Serial.println("u");
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  if (USE_ENCODERS) {
    pinMode(PHI_CS, OUTPUT); digitalWrite(PHI_CS, HIGH);
    pinMode(ANK_CS, OUTPUT); digitalWrite(ANK_CS, HIGH);
    SPI.begin();
  }

  Serial.println("=== motor_inertia_test (실험3: 허리 유효관성) ===");

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
    dxl.torqueOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_CURRENT);      // ★전류(토크) 제어
    dxl.torqueOn(DXL_ID);
    setCur(0);                                     // 시작은 무토크
    home_deg = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
    Serial.println("전류제어 모드, Goal Current = 0 (자유). 매달려 멈춘 뒤 'z'.");
  } else {
    Serial.println("!!! 모터 응답 없음 (배터리/RS-485 확인)");
  }

  if (USE_ENCODERS) {
    phi_zero = as5047_raw(PHI_CS);
    ank_zero = as5047_raw(ANK_CS);
  }
  Serial.println("명령: z s c<u> t<Nm> i<u> k r p x");
  t0 = millis();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z' || c == 'Z') {
      if (motor_ok) home_deg = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
      if (USE_ENCODERS) { phi_zero = as5047_raw(PHI_CS); ank_zero = as5047_raw(ANK_CS); }
      Serial.println("# zero set (delta=0)");
    }
    else if (c == 's' || c == 'S') {
      streaming = !streaming; next_us = micros();
      Serial.println(streaming ? "# STREAM ON (D,t_ms,delta,cur,phi,ank)" : "# STREAM OFF");
    }
    else if (c == 'c' || c == 'C') { setCur((int)Serial.parseInt());
      Serial.print("# cur="); Serial.println(goal_cur); }
    else if (c == 't' || c == 'T') {
      float tau = Serial.parseFloat();
      setCur((int)(tau / KT_NOMINAL * 1000.0f / CUR_UNIT));
      Serial.print("# tau="); Serial.print(tau, 3);
      Serial.print(" -> cur="); Serial.println(goal_cur);
    }
    else if (c == 'i' || c == 'I') {
      int u = (int)Serial.parseInt();
      if (!motor_ok) { Serial.println("# 모터 없음"); }
      else {
        pulse_start_deg = deltaDeg();
        if (fabsf(pulse_start_deg) > 3.0f)
          Serial.println("# 주의: 홈에서 3도 이상 벗어나 있음 - 멈춘 뒤 z 권장");
        streaming = true; next_us = micros();
        pulsing = true; pulse_t0 = millis();
        setCur(u);
        Serial.print("E,"); Serial.print(millis()-t0);
        Serial.print(",PULSE_START,"); Serial.print(u);
        Serial.print(",tau="); Serial.println(u*CUR_UNIT/1000.0f*KT_NOMINAL, 4);
      }
    }
    else if (c == 'k' || c == 'K') {
      setCur(0); streaming = true; next_us = micros();
      Serial.println("# FREE (cur=0) 스트리밍 - 손으로 20도 밀었다 놓기");
    }
    else if (c == 'r' || c == 'R') { setCur(0); pulsing = false; Serial.println("# cur=0"); }
    else if (c == 'p' || c == 'P') printOnce();
    else if (c == 'x' || c == 'X') {
      setCur(0); pulsing = false;
      if (motor_ok) dxl.torqueOff(DXL_ID);
      Serial.println(">>> EMERGENCY STOP (torque off)");
    }
  }

  float d = deltaDeg();

  // ---- 안전 한계 ----
  if (motor_ok && fabsf(d) > DELTA_LIMIT && goal_cur != 0) {
    setCur(0); pulsing = false;
    Serial.print("!! ANGLE LIMIT "); Serial.print(d, 1); Serial.println(" deg -> cur=0");
  }

  // ---- 자동 펄스 종료 판정 ----
  if (pulsing) {
    uint32_t el = millis() - pulse_t0;
    float dd = d - pulse_start_deg;
    if (fabsf(dd) >= PULSE_DEG || el >= PULSE_MS) {
      setCur(0); pulsing = false;
      float t = el / 1000.0f;
      float alpha = (t > 0.01f) ? (2.0f * radians(fabsf(dd)) / (t*t)) : 0;  // rad/s^2 개략
      Serial.print("E,"); Serial.print(millis()-t0);
      Serial.print(",PULSE_END,d_deg="); Serial.print(dd, 2);
      Serial.print(",t_s="); Serial.print(t, 3);
      Serial.print(",alpha_rough="); Serial.print(alpha, 1);
      Serial.println(" rad/s2  (정밀값은 inertia_analysis.py)");
    }
  }

  // ---- 100Hz 스트리밍 ----
  if (streaming) {
    uint32_t now = micros();
    if ((int32_t)(now - next_us) >= 0) {
      next_us += PERIOD_US;
      int cur = motor_ok ? (int)dxl.readControlTableItem(PRESENT_CURRENT, DXL_ID) : 0;
      Serial.print("D,");
      Serial.print(millis() - t0);  Serial.print(",");
      Serial.print(d, 3);           Serial.print(",");
      Serial.print(cur);            Serial.print(",");
      if (USE_ENCODERS) {
        Serial.print(rawToDeg(as5047_raw(PHI_CS), phi_zero), 3); Serial.print(",");
        Serial.println(rawToDeg(as5047_raw(ANK_CS), ank_zero), 3);
      } else {
        Serial.println("0,0");
      }
    }
  }
}
