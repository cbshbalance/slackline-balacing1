/*
 * mounted_swing_log.ino — [실험 ① 10.2] 로봇 장착 상태 φ 자유진동 로깅 (v19)
 * =====================================================================
 * 10.1(크랭크 단독)과 같되, 로봇이 줄에 올라탄 상태이므로 허리 모터로 δ를
 * 잠가(상·하체를 한 막대로 고정) φ 자유진동만 순수하게 남긴다.
 * 목적: 장착 상태 ω_eff 실측 → 모델(줄 관성 + 로봇 질량 포함) 예측과 ±5% 대조.
 *
 * 필요: OpenCR + XM430(ID1, 허리) + φ 엔코더(AS5047P, CS=D10). ★배터리 연결 필수(모터).
 *
 * ★확인: PHI_CS(기본 D10), ENC_PHI_DIR(브링업 확정), MOTOR_DIR(torque_cal 확정).
 *
 * [명령] (115200 baud, PC 로깅은 exp_logger.py)
 *   z : φ 영점 (로봇 정지 상태에서)
 *   k : 허리 잠금 ON  — 현재 δ 위치를 목표로 위치제어 유지(강체화). ★측정 전 필수
 *   u : 허리 잠금 해제(토크 OFF)
 *   s : 스트리밍 시작/정지 (200Hz, "D,t_ms,phi_deg")
 *   t : 상태 출력(잠금여부·δ·φ)
 *
 * 절차: k(잠금) → z(영점) → s → 로봇+줄을 살짝 밀었다 놓기 → 10~20회 왕복 → s → quit
 */
#include <SPI.h>
#include <Dynamixel2Arduino.h>

#define PHI_CS 10
int ENC_PHI_DIR = +1;      // ★브링업 확정값
int MOTOR_DIR   = +1;      // ★torque_cal 확정값 (여기선 잠금 유지에만 쓰임)

#define DXL_SERIAL  Serial3
#define DXL_DIR_PIN 84
#define DXL_ID      1
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// ---------- φ 엔코더 (AS5047P/AS5048A 공통 SPI) ----------
static SPISettings ENC_SPI(1000000, MSBFIRST, SPI_MODE1);
uint32_t enc_err = 0;
float phi_zero = 0;
uint16_t encParity(uint16_t v){ v^=v>>8; v^=v>>4; v^=v>>2; v^=v>>1; return v&1; }
uint16_t encXfer(uint16_t f){
  SPI.beginTransaction(ENC_SPI); digitalWrite(PHI_CS, LOW); delayMicroseconds(1);
  uint16_t r = SPI.transfer16(f);
  digitalWrite(PHI_CS, HIGH); SPI.endTransaction(); delayMicroseconds(1); return r;
}
bool encRead(uint16_t &raw){
  uint16_t cmd = 0x3FFF | 0x4000; cmd |= encParity(cmd) << 15;
  encXfer(cmd);
  uint16_t resp = encXfer(cmd);
  if (resp & 0x4000) { enc_err++; return false; }
  raw = resp & 0x3FFF; return true;
}
float wrapPi(float a){ while(a> PI)a-=2*PI; while(a<-PI)a+=2*PI; return a; }
float phiDeg(){
  uint16_t raw; if(!encRead(raw)) return NAN;
  return ENC_PHI_DIR * wrapPi(raw*(2*PI/16384.0f) - phi_zero) * 180.0f/PI;
}

bool streaming = false, locked = false;
int32_t lock_pos = 0;

void hipLock(){
  int32_t here = dxl.getPresentPosition(DXL_ID);
  dxl.torqueOff(DXL_ID); delay(20);
  dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION); delay(20);
  // 프로파일 넉넉히(잠금 유지용), 목표=현재
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 60);
  dxl.torqueOn(DXL_ID);
  lock_pos = here; dxl.setGoalPosition(DXL_ID, lock_pos);
  locked = true;
}

void setup(){
  Serial.begin(115200); delay(400);
  pinMode(PHI_CS, OUTPUT); digitalWrite(PHI_CS, HIGH);
  SPI.begin();
  Serial.println("=== mounted_swing_log (v19) ===");
  // φ 자가진단(전부 0이면 배선 의심)
  uint16_t r; int ok=0; uint16_t first=0; bool vary=false;
  for(int i=0;i<20;i++){ if(encRead(r)){ if(ok==0)first=r; else if(r!=first)vary=true; ok++; } delay(5); }
  if(ok==0)                     Serial.println("phi enc: FAIL — 배선/CS/전원");
  else if(!vary && first==0)    Serial.println("phi enc: 의심! raw 전부 0 — MISO(D12)/전원 확인");
  else                          { Serial.print("phi enc: OK raw="); Serial.println(first); }
  // DXL 1Mbps 승격
  dxl.begin(57600); dxl.setPortProtocolVersion(2.0);
  if (dxl.ping(DXL_ID)) { dxl.torqueOff(DXL_ID); dxl.writeControlTableItem(BAUD_RATE, DXL_ID, 3); delay(50); }
  dxl.begin(1000000);
  Serial.println(dxl.ping(DXL_ID) ? "DXL 1Mbps OK" : "DXL 응답없음 — 배터리/배선 확인");
  Serial.println("명령: k(허리잠금) z(영점) s(스트리밍) u(해제) t(상태)");
}

void loop(){
  static uint32_t next = 0;
  if (streaming && (int32_t)(micros() - next) >= 0) {
    next = micros() + 5000;
    Serial.print("D,"); Serial.print(millis());
    Serial.print(","); Serial.println(phiDeg(), 4);
  }
  if (!Serial.available()) return;
  char c = Serial.read();
  if      (c=='z'){ uint16_t r; if(encRead(r)) phi_zero=r*(2*PI/16384.0f); Serial.println("E,0,ZERO"); }
  else if (c=='k'){ hipLock(); Serial.println("E,0,HIP_LOCK"); }
  else if (c=='u'){ dxl.torqueOff(DXL_ID); locked=false; Serial.println("E,0,HIP_FREE"); }
  else if (c=='s'){ streaming=!streaming; Serial.println(streaming?"E,0,STREAM_ON":"E,0,STREAM_OFF"); }
  else if (c=='t'){
    Serial.print("locked="); Serial.print(locked);
    Serial.print(" phi="); Serial.print(phiDeg(),3);
    Serial.print(" enc_err="); Serial.println(enc_err);
  }
}
