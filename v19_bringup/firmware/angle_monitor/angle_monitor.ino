/*
 * angle_monitor.ino — 발목·φ·모터 세 각도 동시 출력 (모니터링 전용, 모터 토크 OFF)
 * ============================================================
 * 배선(wiring diagram 그대로): φ CS=D10, 발목 CS=D9, SCLK=D13, MISO=D12, MOSI=D11
 * 모터: XM430 (RS-485 4핀, ID 1). 배터리 필수(모터 각도를 읽으려면).
 *
 * 모터 토크는 항상 OFF — 손으로 허리를 움직이며 세 각도가 따라오는지 확인용.
 *
 * [시리얼 명령] (115200 baud)
 *   z : 세 각도 모두 영점 (현재 자세 = 0도)
 *   s : 빠른 스트리밍 시작/정지 (100Hz, "D,t_ms,phi,ank,motor" — exp_logger 저장 가능)
 *   (기본 상태에서는 사람이 읽기 좋게 10Hz로 이름 붙여 출력)
 */
#include <SPI.h>
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DXL_DIR_PIN  84
const uint8_t DXL_ID = 1;

const uint8_t PHI_CS = 10;   // φ(줄 각도) 엔코더
const uint8_t ANK_CS = 9;    // 발목 엔코더

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

bool  motor_ok = false;
uint32_t dxl_baud = 0;
float motor_zero = 0;
uint16_t phi_zero = 0, ank_zero = 0;

bool fast_stream = false;    // false=10Hz 읽기용, true=100Hz D행
uint32_t t0 = 0, next_us = 0;

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
  if (!motor_ok) return NAN;
  return dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) - motor_zero;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(PHI_CS, OUTPUT); digitalWrite(PHI_CS, HIGH);
  pinMode(ANK_CS, OUTPUT); digitalWrite(ANK_CS, HIGH);
  SPI.begin();

  Serial.println("=== angle_monitor (토크 OFF, 읽기 전용) ===");

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
    dxl.torqueOff(DXL_ID);              // 항상 토크 OFF — 손으로 움직여도 됨
    motor_zero = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
    Serial.print("모터 OK @ "); Serial.println(dxl_baud);
  } else {
    Serial.println("!!! 모터 응답 없음 — 모터 각도는 nan으로 표시");
  }

  uint16_t r1 = as5047_raw(PHI_CS), r2 = as5047_raw(ANK_CS);
  Serial.print("phi enc raw="); Serial.print(r1);
  Serial.print("  ank enc raw="); Serial.println(r2);
  Serial.println("(0 또는 16383 고정이면 해당 엔코더 배선 확인)");
  Serial.println("명령: z(영점)  s(100Hz 스트리밍 전환)");
  t0 = millis();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z' || c == 'Z') {
      phi_zero = as5047_raw(PHI_CS);
      ank_zero = as5047_raw(ANK_CS);
      if (motor_ok) motor_zero = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
      Serial.println("# zero set (phi, ank, motor)");
    }
    else if (c == 's' || c == 'S') {
      fast_stream = !fast_stream;
      next_us = micros();
      Serial.println(fast_stream ? "# 100Hz D행 모드 (D,t_ms,phi,ank,motor)"
                                 : "# 10Hz 읽기 모드");
    }
  }

  uint32_t period = fast_stream ? 10000UL : 100000UL;   // 100Hz / 10Hz
  uint32_t now = micros();
  if ((int32_t)(now - next_us) >= 0) {
    next_us += period;
    float phi = rawToDeg(as5047_raw(PHI_CS), phi_zero);
    float ank = rawToDeg(as5047_raw(ANK_CS), ank_zero);
    float mot = motorDeg();
    if (fast_stream) {
      Serial.print("D,");
      Serial.print(millis() - t0); Serial.print(",");
      Serial.print(phi, 3);        Serial.print(",");
      Serial.print(ank, 3);        Serial.print(",");
      Serial.println(mot, 3);
    } else {
      Serial.print("phi=");    Serial.print(phi, 2);
      Serial.print("\tank=");  Serial.print(ank, 2);
      Serial.print("\tmotor="); Serial.println(mot, 2);
    }
  }
}
