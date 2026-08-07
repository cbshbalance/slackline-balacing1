/*
 * motor_profile_test.ino — 각도 명령 + 사다리꼴(가속-등속-감속) 이동 + 엔코더 2개 동시 수집
 * ============================================================
 * 동작:
 *  - 시리얼로 목표 각도를 입력하면 허리 모터(XM430, ID1)가
 *    "가속 → 등속 → 감속" 사다리꼴 프로파일로 이동한다.
 *    (모터 내장 Profile Velocity / Profile Acceleration 기능 사용)
 *  - 이동 중에도 φ 엔코더(D10)·발목 엔코더(D9)를 100Hz로 계속 읽어
 *    "D,t_ms,phi_deg,ank_deg,motor_deg" 형식으로 출력한다.
 *    → exp_logger.py 로 그대로 CSV 저장 가능 (D행 자동 저장).
 *
 * 배선(문서 5.2절 그대로): φ CS=D10, 발목 CS=D9, SCLK=D13, MISO=D12, MOSI=D11
 * 모터: RS-485 4핀, ID 1. 배터리 필수(USB만으로는 모터가 못 움직임).
 *
 * [시리얼 명령] (115200 baud)
 *   g 30    : 홈 기준 +30도로 이동 (범위 ±90도 제한)
 *   g -15   : 홈 기준 -15도로 이동
 *   v 60    : 최고 속도 설정 (unit=0.229rpm, 기본 60 ≈ 82도/초)
 *   a 20    : 가속도 설정   (unit=214.577rev/min², 기본 20 ≈ 429도/초²)
 *   z       : 엔코더 2개 영점 + 현재 모터 위치를 홈(0도)으로
 *   s       : 스트리밍 시작/정지 (100Hz D행 출력)
 *   p       : 현재 값 1회 출력
 *   x       : 비상정지 (토크 OFF)
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

uint32_t dxl_baud = 0;       // 실제 연결된 속도 (1M 우선, 57600 폴백)
bool  motor_ok   = false;
float home_deg   = 0;        // 'z' 시점의 모터 위치 = 0도 기준
uint16_t phi_zero = 0, ank_zero = 0;

bool streaming = false;
uint32_t t0 = 0, next_us = 0;
const uint32_t PERIOD_US = 10000;   // 100 Hz

// ---------- AS5047P 읽기 (SPI 모드1, ANGLECOM 0x3FFF) ----------
uint16_t as5047_raw(uint8_t cs) {
  uint16_t v;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(cs, LOW);  delayMicroseconds(1);
  SPI.transfer16(0xFFFF);                    // 각도 읽기 명령 (패리티 포함)
  digitalWrite(cs, HIGH); delayMicroseconds(1);
  digitalWrite(cs, LOW);  delayMicroseconds(1);
  v = SPI.transfer16(0xFFFF);                // 이전 명령의 응답 = 각도
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  return v & 0x3FFF;                         // 14비트
}

float rawToDeg(uint16_t raw, uint16_t zero) {
  int16_t d = (int16_t)((raw - zero) & 0x3FFF);
  if (d > 8191) d -= 16384;                  // ±180도로 감기
  return d * (360.0f / 16384.0f);
}

float motorDeg() {
  if (!motor_ok) return 0;
  return dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) - home_deg;
}

void printOnce() {
  Serial.print("phi=");   Serial.print(rawToDeg(as5047_raw(PHI_CS), phi_zero), 2);
  Serial.print("  ank="); Serial.print(rawToDeg(as5047_raw(ANK_CS), ank_zero), 2);
  Serial.print("  motor="); Serial.print(motorDeg(), 2);
  Serial.print("  (baud="); Serial.print(dxl_baud); Serial.println(")");
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(PHI_CS, OUTPUT); digitalWrite(PHI_CS, HIGH);
  pinMode(ANK_CS, OUTPUT); digitalWrite(ANK_CS, HIGH);
  SPI.begin();

  Serial.println("=== motor_profile_test ===");

  // 모터 연결: 1Mbps 우선, 안 되면 57600
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
    dxl.setOperatingMode(DXL_ID, OP_POSITION);          // 위치 모드 (0~360도)
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 20);  // 가속-감속 기울기
    dxl.writeControlTableItem(PROFILE_VELOCITY,     DXL_ID, 60);  // 등속 구간 속도
    home_deg = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID, home_deg, UNIT_DEGREE);   // 제자리 목표 → 토크 켤 때 안 움직임
    dxl.torqueOn(DXL_ID);
    Serial.println("사다리꼴 프로파일 설정: a=20, v=60 (v/a 명령으로 변경 가능)");
  } else {
    Serial.println("!!! 모터 응답 없음 (배터리/RS-485/속도 확인) — 엔코더만 동작");
  }

  // 엔코더 확인
  uint16_t r1 = as5047_raw(PHI_CS), r2 = as5047_raw(ANK_CS);
  Serial.print("phi enc raw="); Serial.print(r1);
  Serial.print("  ank enc raw="); Serial.println(r2);
  Serial.println("(0 또는 16383 고정이면 배선 확인)");

  Serial.println("명령: g<deg> v<n> a<n> z s p x");
  t0 = millis();
}

void loop() {
  // ---------- 명령 처리 ----------
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'g' || c == 'G') {
      float deg = Serial.parseFloat();
      deg = constrain(deg, -90.0f, 90.0f);              // 안전 제한
      if (motor_ok) {
        if (!dxl.getTorqueEnableStat(DXL_ID)) {           // x 이후 자동 토크 복구
          dxl.setGoalPosition(DXL_ID, dxl.getPresentPosition(DXL_ID, UNIT_DEGREE), UNIT_DEGREE);
          dxl.torqueOn(DXL_ID);
        }
        dxl.setGoalPosition(DXL_ID, home_deg + deg, UNIT_DEGREE);
        Serial.print("E,"); Serial.print(millis() - t0);
        Serial.print(",MOVE,"); Serial.println(deg, 1);
      } else Serial.println("# 모터 연결 안 됨");
    }
    else if (c == 'v' || c == 'V') {
      int v = Serial.parseInt();
      if (motor_ok) { dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, v);
        Serial.print("# vel="); Serial.println(v); }
    }
    else if (c == 'a' || c == 'A') {
      int a = Serial.parseInt();
      if (motor_ok) { dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, a);
        Serial.print("# acc="); Serial.println(a); }
    }
    else if (c == 'z' || c == 'Z') {
      phi_zero = as5047_raw(PHI_CS);
      ank_zero = as5047_raw(ANK_CS);
      if (motor_ok) home_deg = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
      Serial.println("# zero set (엔코더 2개 + 모터 홈)");
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

  // ---------- 100Hz 스트리밍 (이동 중에도 계속 돈다) ----------
  if (streaming) {
    uint32_t now = micros();
    if ((int32_t)(now - next_us) >= 0) {
      next_us += PERIOD_US;
      Serial.print("D,");
      Serial.print(millis() - t0);                       Serial.print(",");
      Serial.print(rawToDeg(as5047_raw(PHI_CS), phi_zero), 3); Serial.print(",");
      Serial.print(rawToDeg(as5047_raw(ANK_CS), ank_zero), 3); Serial.print(",");
      Serial.println(motorDeg(), 3);
    }
  }
}
