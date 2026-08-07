/*
 * tilt_release_test.ino — [실험 ② 11장] 기울여 놓기 (넘어짐 속도 λ_u · 위험가중치 w)
 * =====================================================================
 * 로봇을 줄에 올리고 허리를 잠근 뒤, 살짝 기울여 놓고 넘어지는 궤적을 기록한다.
 * φ(줄)·발목(α)·δ(허리) 세 각을 200Hz로 찍어 β(쓰러짐)를 계산할 수 있게 한다.
 *   불안정 성분이 e^{λt}로 성장 → λ_u,  여러 시도의 공통 성장방향 → w.
 * 검증판 SPI 읽기(0.22 고정 버그 없음). φ·발목 둘 다 살아있어야 실험 성립.
 *
 * ★필요: OpenCR + XM430(ID1) + 배터리. φ 엔코더(CS=D10) + 발목 엔코더(CS=D9) 둘 다.
 * ★확인: MOTOR_DIR(torque_cal). 분석 시 --alpha(8.3 정합점검값) 사용.
 *
 * [명령] (115200 baud, PC 로깅은 exp_logger.py)
 *   k : 허리 잠금 ON (현재 δ 유지 = 상·하체 강체화). ★측정 전 필수
 *   z : φ·발목 영점 (로봇 직립 정지 상태에서)
 *   s : 스트리밍 시작/정지 (200Hz)
 *   m : 마커 (놓는 순간 / 잡는 순간에 눌러 시도 구간 표시) ★핵심
 *   u : 허리 잠금 해제   t : 상태 출력
 *
 * CSV: D,t_ms,phi_deg,ank_deg,del_deg,vel_dps,cur_mA  (exp_analysis_v19.py exp2 호환)
 *   (vel·cur는 이 실험서 미사용, 포맷 유지용 0)
 */
#include <SPI.h>
#include <Dynamixel2Arduino.h>

#define PHI_CS 10
#define ANK_CS 9
int MOTOR_DIR = +1;                 // ★torque_cal 확정값

#define DXL_SERIAL  Serial3
#define DXL_DIR_PIN 84
#define DXL_ID      1
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

int32_t lock_raw = 0;
bool locked=false, streaming=false;

// ---------- 검증판 SPI 엔코더 ----------
static SPISettings ENC_SPI(1000000, MSBFIRST, SPI_MODE1);
uint16_t encParity(uint16_t v){ v^=v>>8; v^=v>>4; v^=v>>2; v^=v>>1; return v&1; }
uint16_t encXfer(int cs, uint16_t f){
  SPI.beginTransaction(ENC_SPI); digitalWrite(cs, LOW); delayMicroseconds(1);
  uint16_t r = SPI.transfer16(f);
  digitalWrite(cs, HIGH); SPI.endTransaction(); delayMicroseconds(1); return r;
}
uint16_t encRaw(int cs){
  uint16_t cmd = 0x3FFF | 0x4000; cmd |= encParity(cmd)<<15;
  encXfer(cs, cmd);
  return encXfer(cs, cmd) & 0x3FFF;
}
uint16_t phi_zero=0, ank_zero=0;
float wrapDeg(uint16_t raw, uint16_t zero){
  int16_t d=(int16_t)((raw-zero)&0x3FFF); if(d>8191)d-=16384;
  return d*(360.0f/16384.0f);
}

// ---------- 모터 잠금 ----------
void hipLock(){
  int32_t here=dxl.getPresentPosition(DXL_ID);
  dxl.torqueOff(DXL_ID); delay(20);
  dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION); delay(20);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 60);
  dxl.torqueOn(DXL_ID);
  lock_raw=here; dxl.setGoalPosition(DXL_ID, lock_raw); locked=true;
}
float delDeg(){ return MOTOR_DIR*(dxl.getPresentPosition(DXL_ID)-lock_raw)*(360.0f/4096.0f); }

void row(){
  float phi=wrapDeg(encRaw(PHI_CS), phi_zero);
  float ank=wrapDeg(encRaw(ANK_CS), ank_zero);
  Serial.print("D,"); Serial.print(millis());
  Serial.print(","); Serial.print(phi,3);
  Serial.print(","); Serial.print(ank,3);
  Serial.print(","); Serial.print(delDeg(),3);
  Serial.print(",0,0");                       // vel,cur 미사용
  Serial.println();
}

void setup(){
  Serial.begin(115200); delay(500);
  pinMode(PHI_CS,OUTPUT); digitalWrite(PHI_CS,HIGH);
  pinMode(ANK_CS,OUTPUT); digitalWrite(ANK_CS,HIGH);
  SPI.begin();
  Serial.println("=== tilt_release_test (v19) ===");
  uint16_t rp=encRaw(PHI_CS), ra=encRaw(ANK_CS);
  Serial.print("phi raw="); Serial.print(rp);
  Serial.print("  ank raw="); Serial.println(ra);
  if(rp==0||ra==0) Serial.println("★경고: raw 0 = 그 엔코더 배선 확인(실험②는 둘 다 필요!)");
  dxl.begin(57600); dxl.setPortProtocolVersion(2.0);
  if(dxl.ping(DXL_ID)){ dxl.torqueOff(DXL_ID); dxl.writeControlTableItem(BAUD_RATE,DXL_ID,3); delay(50);}
  dxl.begin(1000000);
  Serial.println(dxl.ping(DXL_ID)?"DXL 1Mbps OK":"DXL 응답없음 — 배터리/RS-485 확인");
  lock_raw=dxl.getPresentPosition(DXL_ID);
  Serial.println("절차: k(잠금) z(영점) s → [기울여놓고 m, 잡고 m] 좌우10회 → s quit");
  Serial.println("명령: k z s m u t");
}

void loop(){
  static uint32_t nx=0;
  if(streaming && (int32_t)(micros()-nx)>=0){ row(); nx=micros()+5000; }  // 200Hz
  if(!Serial.available()) return;
  char c=Serial.read();
  if(c=='k'){ hipLock(); Serial.println("E,0,HIP_LOCK"); }
  else if(c=='z'){ phi_zero=encRaw(PHI_CS); ank_zero=encRaw(ANK_CS); Serial.println("E,0,ZERO"); }
  else if(c=='s'){ streaming=!streaming; Serial.println(streaming?"E,0,STREAM_ON":"E,0,STREAM_OFF"); }
  else if(c=='m'){ Serial.print("E,"); Serial.print(millis()); Serial.println(",MARK"); }
  else if(c=='u'){ dxl.torqueOff(DXL_ID); locked=false; Serial.println("E,0,HIP_FREE"); }
  else if(c=='t'){ Serial.print("lock="); Serial.print(locked);
                   Serial.print(" phi="); Serial.print(wrapDeg(encRaw(PHI_CS),phi_zero),2);
                   Serial.print(" ank="); Serial.println(wrapDeg(encRaw(ANK_CS),ank_zero),2); }
}
