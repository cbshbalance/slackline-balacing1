/*
 * motor_triangle_test.ino — 각도 명령 + 삼각형(가속-감속, 등속 없음) 이동 + 엔코더 2개 동시 수집
 * ============================================================
 * motor_profile_test(사다리꼴)와 같지만, 이동 프로파일이 삼각형이다:
 *   가속 → (등속 없음) → 감속.  거리의 절반까지 가속했다가 바로 감속.
 *
 * 원리: 이동 거리 d와 가속도 a에서 꼭짓점 속도 v_peak=√(a·d)를 계산해
 *       Profile Velocity를 그 값으로 매번 자동 설정 → 등속 구간이 생길 틈이 없다.
 *       총 이동 시간 T = 2·√(d/a).
 *
 * 배선(문서 5.2절): φ CS=D10, 발목 CS=D9, SCLK=D13, MISO=D12, MOSI=D11
 * 모터: RS-485 4핀, ID 1, 배터리 필수.
 *
 * [시리얼 명령] (115200 baud)
 *   g 30    : 홈 기준 +30도로 삼각형 프로파일 이동 (±90도 제한)
 *   a 20    : 가속도 설정 (unit=214.577rev/min² ≈ 21.5도/초², 기본 20 ≈ 429도/초²)
 *   z       : 엔코더 2개 영점 + 모터 홈(0도) 설정
 *   s       : 스트리밍 시작/정지 (100Hz, D,t_ms,phi,ank,motor)
 *   p       : 현재 값 1회 출력
 *   x       : 비상정지 (토크 OFF)
 *
 * 출력 D행은 exp_logger.py 로 그대로 CSV 저장 가능.
 */
#include <SPI.h>
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DXL_DIR_PIN  84
const uint8_t DXL_ID = 1;

const uint8_t PHI_CS = 10;
const uint8_t ANK_CS = 9;

// 단위 환산
const float ACC_UNIT_DPS2 = 21.4577f;   // PROFILE_ACCELERATION 1unit = 21.4577 도/초²
const float VEL_UNIT_DPS  = 1.374f;     // PROFILE_VELOCITY     1unit = 0.229rpm = 1.374 도/초
const int   VEL_CAP_UNIT  = 200;        // 안전 상한 (≈275 도/초)

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

uint32_t dxl_baud = 0;
bool  motor_ok = false;
float home_deg = 0;
int   acc_unit = 20;                    // 기본 가속도 ≈ 429 도/초²
uint16_t phi_zero = 0, ank_zero = 0;

bool streaming = false;
uint32_t t0 = 0, next_us = 0;
const uint32_t PERIOD_US = 10000;       // 100 Hz

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

float motorDeg() {
  if (!motor_ok) return 0;
  return dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) - home_deg;
}

void moveTriangle(float target_deg) {
  if (!dxl.getTorqueEnableStat(DXL_ID)) {                 // x 이후 자동 토크 복구
    dxl.setGoalPosition(DXL_ID, dxl.getPresentPosition(DXL_ID, UNIT_DEGREE), UNIT_DEGREE);
    dxl.torqueOn(DXL_ID);
  }
  float cur = motorDeg();
  float d = fabsf(target_deg - cur);            // 이동 거리 [도]
  if (d < 0.5f) { Serial.println("# 이동 거리 너무 작음"); return; }

  float a_dps2  = acc_unit * ACC_UNIT_DPS2;     // 가속도 [도/초²]
  float v_peak  = sqrtf(a_dps2 * d);            // 삼각형 꼭짓점 속도 [도/초]
  int   v_unit  = (int)(v_peak / VEL_UNIT_DPS) + 1;   // 살짝 여유(올림)
  v_unit = constrain(v_unit, 1, VEL_CAP_UNIT);
  float T = 2.0f * sqrtf(d / a_dps2);           // 예상 소요 시간 [초]

  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, acc_unit);
  dxl.writeControlTableItem(PROFILE_VELOCITY,     DXL_ID, v_unit);
  dxl.setGoalPosition(DXL_ID, home_deg + target_deg, UNIT_DEGREE);

  Serial.print("E,"); Serial.print(millis() - t0);
  Serial.print(",TRI_MOVE,"); Serial.print(target_deg, 1);
  Serial.print(" (d=");   Serial.print(d, 1);
  Serial.print("deg, v_peak="); Serial.print(v_peak, 0);
  Serial.print("dps, T=");      Serial.print(T, 2); Serial.println("s)");
}

void printOnce() {
  Serial.print("phi=");     Serial.print(rawToDeg(as5047_raw(PHI_CS), phi_zero), 2);
  Serial.print("  ank=");   Serial.print(rawToDeg(as5047_raw(ANK_CS), ank_zero), 2);
  Serial.print("  motor="); Serial.print(motorDeg(), 2);
  Serial.print("  acc_unit="); Serial.println(acc_unit);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(PHI_CS, OUTPUT); digitalWrite(PHI_CS, HIGH);
  pinMode(ANK_CS, OUTPUT); digitalWrite(ANK_CS, HIGH);
  SPI.begin();

  Serial.println("=== motor_triangle_test (가속-감속 삼각형) ===");

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
    dxl.setOperatingMode(DXL_ID, OP_POSITION);
    home_deg = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID, home_deg, UNIT_DEGREE);   // 제자리 목표 → 토크 켤 때 안 움직임
    dxl.torqueOn(DXL_ID);
  } else {
    Serial.println("!!! 모터 응답 없음 (배터리/RS-485/속도 확인) — 엔코더만 동작");
  }

  uint16_t r1 = as5047_raw(PHI_CS), r2 = as5047_raw(ANK_CS);
  Serial.print("phi enc raw="); Serial.print(r1);
  Serial.print("  ank enc raw="); Serial.println(r2);
  Serial.println("(0 또는 16383 고정이면 배선 확인)");

  Serial.println("명령: g<deg> a<n> z s p x");
  t0 = millis();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'g' || c == 'G') {
      float deg = Serial.parseFloat();
      deg = constrain(deg, -90.0f, 90.0f);
      if (motor_ok) moveTriangle(deg);
      else Serial.println("# 모터 연결 안 됨");
    }
    else if (c == 'a' || c == 'A') {
      acc_unit = constrain((int)Serial.parseInt(), 1, 200);
      Serial.print("# acc_unit="); Serial.print(acc_unit);
      Serial.print(" ("); Serial.print(acc_unit * ACC_UNIT_DPS2, 0); Serial.println(" deg/s^2)");
    }
    else if (c == 'z' || c == 'Z') {
      phi_zero = as5047_raw(PHI_CS);
      ank_zero = as5047_raw(ANK_CS);
      if (motor_ok) home_deg = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
      Serial.println("# zero set");
    }
    else if (c == 's' || c == 'S') {
      streaming = !streaming;
      next_us = micros();
      Serial.println(streaming ? "# STREAM ON (D,t_ms,phi,ank,motor)" : "# STREAM OFF");
    }
    else if (c == 'p' || c == 'P') printOnce();
    else if (c == 'x' || c == 'X') {
      if (motor_ok) dxl.torqueOff(DXL_ID);
      Serial.println(">>> EMERGENCY STOP (torque off)");
    }
  }

  if (streaming) {
    uint32_t now = micros();
    if ((int32_t)(now - next_us) >= 0) {
      next_us += PERIOD_US;
      Serial.print("D,");
      Serial.print(millis() - t0);                              Serial.print(",");
      Serial.print(rawToDeg(as5047_raw(PHI_CS), phi_zero), 3);  Serial.print(",");
      Serial.print(rawToDeg(as5047_raw(ANK_CS), ank_zero), 3);  Serial.print(",");
      Serial.println(motorDeg(), 3);
    }
  }
}
