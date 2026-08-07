/*
 * hip_swing_test.ino — [실험 ③ 12장] 매달고 허리 흔들기 (허리 유효관성 I_δ·마찰)
 * =====================================================================
 * 발이 안 닿게 매단 상태에서 허리 모터에 계단·삼각 명령을 주고,
 * 서보 전류(=토크)·각도·각속도를 100Hz로 기록한다.
 *   τ = I_δ·δ̈ + b·δ̇ + c·sign(δ̇)  → 최소제곱으로 I_δ, b, c 추출.
 * 검증판 SPI 엔코더 읽기 사용(0.22 고정 버그 없음). φ·발목도 함께 기록(포맷 유지).
 *
 * ★필요: OpenCR + XM430(ID1) + 배터리(모터 전원). 엔코더는 있어도/없어도 실험 성립.
 * ★확인: MOTOR_DIR(torque_cal), 이후 분석에 Kt 실측값 사용.
 *
 * [명령] (115200 baud, PC 로깅은 exp_logger.py)
 *   z : 모터 홈(현재 위치=0°) + 엔코더 영점
 *   3 : 자동 시퀀스 실행 — 계단(±5/10/20°) + 삼각 왕복(약 25초), 자동 기록
 *   a <dps2> : 프로파일 가속도 설정(기본 8000°/s²)
 *   s : 수동 스트리밍 토글   x : 비상정지(토크 OFF)   t : 상태
 *
 * CSV: D,t_ms,phi_deg,ank_deg,del_deg,vel_dps,cur_mA   (exp_analysis_v19.py exp3 호환)
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

const float CUR_mA_UNIT = 2.69f;                 // PRESENT_CURRENT 단위
const float VEL_DPS_UNIT = 0.229f * 6.0f;        // 0.229rpm → deg/s
float prof_dps2 = 8000.0f;
int32_t home_raw = 0;
bool torque_on = false, streaming = false;

// ---------- 검증판 SPI 엔코더 (CS 유지 + 0x3FFF+패리티) ----------
static SPISettings ENC_SPI(1000000, MSBFIRST, SPI_MODE1);
uint16_t encParity(uint16_t v){ v^=v>>8; v^=v>>4; v^=v>>2; v^=v>>1; return v&1; }
uint16_t encXfer(int cs, uint16_t f){
  SPI.beginTransaction(ENC_SPI); digitalWrite(cs, LOW); delayMicroseconds(1);
  uint16_t r = SPI.transfer16(f);
  digitalWrite(cs, HIGH); SPI.endTransaction(); delayMicroseconds(1); return r;
}
uint16_t encRaw(int cs){
  uint16_t cmd = 0x3FFF | 0x4000; cmd |= encParity(cmd) << 15;
  encXfer(cs, cmd);
  return encXfer(cs, cmd) & 0x3FFF;
}
uint16_t phi_zero=0, ank_zero=0;
float wrapDeg(uint16_t raw, uint16_t zero){
  int16_t d = (int16_t)((raw - zero) & 0x3FFF);
  if (d > 8191) d -= 16384;
  return d * (360.0f/16384.0f);
}

// ---------- 모터 ----------
void setProfile(float dps2){
  float rev_min2 = dps2/360.0f*3600.0f;
  int32_t a = (int32_t)(rev_min2/214.577f); if(a<1)a=1;
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, a);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 0);   // 0 = 최대
}
void motorOn(){
  dxl.torqueOff(DXL_ID); delay(20);
  dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION); delay(20);
  setProfile(prof_dps2);
  dxl.torqueOn(DXL_ID); torque_on = true;
}
void gotoDeg(float d){
  int32_t raw = home_raw + MOTOR_DIR*(int32_t)(d/360.0f*4096.0f);
  dxl.setGoalPosition(DXL_ID, raw);
}
float delDeg(){ return MOTOR_DIR*(dxl.getPresentPosition(DXL_ID)-home_raw)*(360.0f/4096.0f); }

// ---------- 기록 ----------
float last_vel=0, last_cur=0; uint8_t slow=0;
void row(){
  float phi = encRaw(PHI_CS)==0?0:wrapDeg(encRaw(PHI_CS), phi_zero);
  float ank = wrapDeg(encRaw(ANK_CS), ank_zero);
  if (slow==0){
    last_cur = ((int16_t)dxl.readControlTableItem(PRESENT_CURRENT, DXL_ID))*CUR_mA_UNIT;
    last_vel = ((int32_t)dxl.readControlTableItem(PRESENT_VELOCITY, DXL_ID))*VEL_DPS_UNIT*MOTOR_DIR;
  }
  slow ^= 1;
  Serial.print("D,"); Serial.print(millis());
  Serial.print(","); Serial.print(phi,3);
  Serial.print(","); Serial.print(ank,3);
  Serial.print(","); Serial.print(delDeg(),3);
  Serial.print(","); Serial.print(last_vel,2);
  Serial.print(","); Serial.println(last_cur,1);
}
void wait(uint32_t ms){                    // 대기 중에도 100Hz 기록
  uint32_t t0=millis(), nx=micros();
  while(millis()-t0<ms){ if((int32_t)(micros()-nx)>=0){ row(); nx+=10000; } }
}
void evt(const char*n,float a){ Serial.print("E,");Serial.print(millis());Serial.print(",");Serial.print(n);Serial.print(",");Serial.println(a,1);}

void seq3(){                                // [실험 ③ 자동 시퀀스]
  if(!torque_on) motorOn();
  bool w=streaming; streaming=true;
  evt("EXP3_START", prof_dps2);
  float amp[3]={5,10,20};
  for(int k=0;k<3;k++){                     // 계단 ±amp (관성·마찰)
    evt("STEP",amp[k]);
    gotoDeg(+amp[k]); wait(600);
    gotoDeg(0);       wait(600);
    gotoDeg(-amp[k]); wait(600);
    gotoDeg(0);       wait(600);
  }
  evt("TRI",15);                            // 삼각 왕복 ×4 (마찰 부호반전)
  for(int k=0;k<4;k++){ gotoDeg(+15); wait(450); gotoDeg(-15); wait(450); }
  gotoDeg(0); wait(600);
  evt("EXP3_END",0); streaming=w;
}

void setup(){
  Serial.begin(115200); delay(500);
  pinMode(PHI_CS,OUTPUT); digitalWrite(PHI_CS,HIGH);
  pinMode(ANK_CS,OUTPUT); digitalWrite(ANK_CS,HIGH);
  SPI.begin();
  Serial.println("=== hip_swing_test (v19) ===");
  Serial.print("phi raw="); Serial.print(encRaw(PHI_CS));
  Serial.print("  ank raw="); Serial.println(encRaw(ANK_CS));
  dxl.begin(57600); dxl.setPortProtocolVersion(2.0);
  if(dxl.ping(DXL_ID)){ dxl.torqueOff(DXL_ID); dxl.writeControlTableItem(BAUD_RATE,DXL_ID,3); delay(50);}
  dxl.begin(1000000);
  Serial.println(dxl.ping(DXL_ID)?"DXL 1Mbps OK":"DXL 응답없음 — 배터리/RS-485 확인");
  home_raw = dxl.getPresentPosition(DXL_ID);
  Serial.println("★발이 아무데도 안 닿게 매단 뒤: z → 3");
  Serial.println("명령: z  3  a<dps2>  s  x  t");
}

void loop(){
  static uint32_t nx=0;
  if(streaming && (int32_t)(micros()-nx)>=0){ row(); nx=micros()+10000; }
  if(!Serial.available()) return;
  char c=Serial.read();
  if(c=='z'){ phi_zero=encRaw(PHI_CS); ank_zero=encRaw(ANK_CS);
              home_raw=dxl.getPresentPosition(DXL_ID); if(!torque_on)motorOn(); gotoDeg(0);
              Serial.println("E,0,ZERO"); }
  else if(c=='3'){ seq3(); }
  else if(c=='a'){ float v=Serial.parseFloat(); if(v>=500&&v<=20000){prof_dps2=v; setProfile(v); evt("ACC",v);} }
  else if(c=='s'){ streaming=!streaming; Serial.println(streaming?"E,0,STREAM_ON":"E,0,STREAM_OFF"); }
  else if(c=='x'){ dxl.torqueOff(DXL_ID); torque_on=false; Serial.println(">>> STOP"); }
  else if(c=='t'){ Serial.print("del=");Serial.print(delDeg(),2);Serial.print(" on=");Serial.println(torque_on); }
}
