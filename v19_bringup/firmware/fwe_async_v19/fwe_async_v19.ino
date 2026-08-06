/*
 * fwe_async_v19.ino — 비동기 FWE 실기 1차 (줄태우기 시험용, 2026-08-05)
 * =====================================================================
 * ★아두이노(OpenCR)에 업로드하는 펌웨어. PC 로깅은 exp_logger.py 그대로.
 *
 * 근거: 프로젝트 문서 28(펴기 생략+증분 접기+저속 복귀 = 시뮬 신챔피언),
 *       30(비동기 FWE: F=사건/W=기본상태/E=배경과정). 규칙은 단 두 개다:
 *   ① 위험도 |A| > 트리거(기본 0.6°) → 기운 쪽으로 허리 "증분" 접기  Δδ = K_FOLD × A
 *   ② 조용하면 → δ를 저속(기본 3°/s, 상한 6°/s — 회복속도 상한 규칙)으로 0에 복귀
 *
 * 위험도 A [β-deg] = W0·φ + W1·β + W2·φ̇ + W3·β̇
 *   ★8/6 실험② 실측 w·λ_u 반영 완료 (λ_u=6.68/s 중앙값 6.43, 문서 38)
 *   β = P1R·α + P2R·θ,  α = ank + φ (★8/5 정합점검 확정: --alpha ank+phi),
 *   θ = α + δ,  MOTOR_DIR=+1 (8/5 확정), δ+ = 앞접기 = 기운 쪽(+)으로 접기.
 *
 * [명령] (115200)
 *   z : 영점 (로봇 직립·정지 잡고) — δ 기준도 현 위치로
 *   g : 제어 ON/OFF (켜기 전 반드시 z. 켜면 E,CTRL_ON)
 *   s : 200Hz 로그 스트리밍 ON/OFF   x : 비상 — 토크 컷 + 제어 OFF
 *   r : δ 목표 즉시 0                 t : 상태 한 줄
 *   + - : K_FOLD ±1   [ ] : 트리거 ±0.1°   v : 복귀속도 1.5/3/6°/s 순환
 * CSV: D,t_ms,phi_deg,ank_deg,del_deg,A_deg,dcmd_deg
 *      E,t_ms,FOLD,<Δδ> | E,t_ms,FALL | E,0,CTRL_ON/OFF | E,0,ZERO
 *
 * 안전: 잡을 사람 상시 배치. |α|>25° → 자동 토크 컷(E,FALL). δ 명령 ±40° 제한,
 *       증분 1회 ±12° 제한, 증분 후 100ms 잠금(접기 실행 시간).
 */
#include <SPI.h>
#include <Dynamixel2Arduino.h>

// ---------------- 핀·모터 ----------------
#define PHI_CS 10
#define ANK_CS 9
#define DXL_SERIAL  Serial3
#define DXL_DIR_PIN 84
#define DXL_ID      1
const int MOTOR_DIR = +1;          // ★8/5 정합점검 확정 (+δ = 앞접기)
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// ---------------- 모델 상수 (실측 질량표 7/30-31 기준) ----------------
// p1 = m1·ell1 + m2·L1 = 0.1094*0.084 + 0.6406*0.259 = 0.17510
// p2 = m2·ell2 = 0.6406*0.1917 = 0.12280   → P1R=p1/(p1+p2), P2R=p2/(p1+p2)
const float P1R = 0.5878f;
const float P2R = 0.4122f;
// ★2026-08-06 실험② 실측 반영 (0806_exp2_tilt_r1.csv, 유효 14시도, 문서 38):
//   λ_u = 6.68 ± 2.90 /s (중앙값 6.43 — 모델 6.4 대비 +0.5%, 게이트 통과)
//   w   = (φ,β,φ̇,β̇; β=1) = (-0.3655, 1, -0.0690, 0.1287)
//   β̇계수 0.129 ≈ 1/λ — 예측점(A=β+β̇/λ) 이론 구조와 정합.
float LAMBDA_U = 6.68f;            // [1/s] 실측 (실험② 8/6)
float W0 = -0.3655f, W1 = 1.0f, W2 = -0.0690f, W3 = 0.1287f;   // 실측 w

// ---------------- 제어 상수 (문서 28 시뮬 확정값 기반) ----------------
float A_TRIG_DEG   = 0.6f;         // 트리거 문턱 [β-deg]
float K_FOLD       = 12.0f;        // 증분 환산비 Δδ=K·A (시뮬 고원 8~15)
float RET_DPS      = 3.0f;         // 저속 복귀 [°/s] (상한 6 — v<λ·A여유/킥계수)
float DCMD_CAP     = 40.0f;        // δ 명령 상한 [°] (기구한계 55 이내 안전)
float STEP_CAP     = 12.0f;        // 증분 1회 상한 [°]
uint32_t FOLD_LOCK_MS = 100;       // 증분 후 잠금(접기 실행 시간)
float FALL_ALPHA_DEG  = 25.0f;     // |α| 초과 시 낙하 판정 → 토크 컷
int PROF_ACC_UNIT = 120;           // ≈2570°/s² (M2 잠정 안전권)
int PROF_VEL_UNIT = 200;           // ≈275°/s

// ---------------- 검증판 SPI 엔코더 (tilt_release_test와 동일) ----------------
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

// ---------------- 상태 ----------------
bool streaming=false, ctrl_on=false;
int32_t d_zero_raw=0;              // δ=0 기준 서보 raw
float dcmd=0;                      // δ 명령 [°]
uint32_t lock_until=0;
float phi_f=0, ank_f=0, dphi=0, dbeta=0, beta_prev=0, phi_prev=0;
bool est_init=false;
uint32_t drop_cnt=0;

float delDeg(){ return MOTOR_DIR*(dxl.getPresentPosition(DXL_ID)-d_zero_raw)*(360.0f/4096.0f); }
void sendDelta(float d){
  int32_t goal = d_zero_raw + MOTOR_DIR * (int32_t)(d * 4096.0f/360.0f);
  dxl.setGoalPosition(DXL_ID, goal);
}
void ev(const char* name, float val=NAN, uint32_t t=0){
  Serial.print("E,"); Serial.print(t?t:millis()); Serial.print(","); Serial.print(name);
  if(!isnan(val)){ Serial.print(","); Serial.print(val,3); }
  Serial.println();
}

void setup(){
  Serial.begin(115200); delay(500);
  pinMode(PHI_CS,OUTPUT); digitalWrite(PHI_CS,HIGH);
  pinMode(ANK_CS,OUTPUT); digitalWrite(ANK_CS,HIGH);
  SPI.begin();
  Serial.println("=== fwe_async_v19 — 비동기 FWE (증분 접기 + 저속 복귀) ===");
  uint16_t rp=encRaw(PHI_CS), ra=encRaw(ANK_CS);
  Serial.print("phi raw="); Serial.print(rp);
  Serial.print("  ank raw="); Serial.println(ra);
  if(rp==0||ra==0) Serial.println("★경고: raw 0 — 엔코더 배선 확인 (제어 금지!)");
  dxl.begin(57600); dxl.setPortProtocolVersion(2.0);
  if(dxl.ping(DXL_ID)){ dxl.torqueOff(DXL_ID); dxl.writeControlTableItem(BAUD_RATE,DXL_ID,3); delay(50);}
  dxl.begin(1000000);
  Serial.println(dxl.ping(DXL_ID)?"DXL 1Mbps OK":"DXL 응답없음 — 배터리/RS-485 확인");
  dxl.torqueOff(DXL_ID); delay(20);
  dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION); delay(20);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, PROF_ACC_UNIT);
  dxl.writeControlTableItem(PROFILE_VELOCITY,     DXL_ID, PROF_VEL_UNIT);
  d_zero_raw = dxl.getPresentPosition(DXL_ID);
  Serial.println("절차: 로봇 직립 잡고 z → s(로그) → g(제어 시작) → 손 떼기");
  Serial.println("명령: z g s x r t + - [ ] v");
}

void loop(){
  static uint32_t nx=0;
  uint32_t now_us = micros();
  if((int32_t)(now_us-nx) < 0){ pollSerial(); return; }
  nx = now_us + 5000;              // 200 Hz 고정 루프
  const float dt = 0.005f;

  // ---- 측정 (드롭아웃/글리치 가드: raw=0 또는 90°+ 점프면 이전값 유지) ----
  uint16_t rp=encRaw(PHI_CS), ra=encRaw(ANK_CS);
  float phi = (rp==0)? phi_f : wrapDeg(rp, phi_zero);
  float ank = (ra==0)? ank_f : wrapDeg(ra, ank_zero);
  if(est_init && fabs(phi-phi_f)>90){ phi=phi_f; drop_cnt++; }
  if(est_init && fabs(ank-ank_f)>90){ ank=ank_f; drop_cnt++; }
  float del = delDeg();

  // ---- 추정: α=ank+φ(확정), θ=α+δ, β, 속도(EMA 미분) ----
  float alpha = ank + phi;
  float theta = alpha + del;
  float beta  = P1R*alpha + P2R*theta;
  if(!est_init){ phi_prev=phi; beta_prev=beta; est_init=true; }
  dphi  = 0.8f*dphi  + 0.2f*((phi -phi_prev )/dt);
  dbeta = 0.8f*dbeta + 0.2f*((beta-beta_prev)/dt);
  phi_prev=phi; beta_prev=beta; phi_f=phi; ank_f=ank;
  float A = W0*phi + W1*beta + W2*dphi + W3*dbeta;   // [β-deg]

  // ---- 제어 ----
  if(ctrl_on){
    if(fabs(alpha) > FALL_ALPHA_DEG){
      dxl.torqueOff(DXL_ID); ctrl_on=false; ev("FALL", alpha);
    } else if(fabs(A) > A_TRIG_DEG && millis() > lock_until){
      float step = K_FOLD * A;                       // 기운 쪽(+A→+δ 앞접기)
      step = constrain(step, -STEP_CAP, STEP_CAP);
      dcmd = constrain(dcmd + step, -DCMD_CAP, DCMD_CAP);
      sendDelta(dcmd);
      lock_until = millis() + FOLD_LOCK_MS;
      ev("FOLD", step);
    } else if(millis() > lock_until && fabs(dcmd) > 0.01f){
      float dd = RET_DPS * dt;                       // 저속 복귀 (배경과정 E)
      dcmd += (dcmd > 0)? -min(dd, dcmd) : min(dd, -dcmd);
      sendDelta(dcmd);
    }
  }

  // ---- 로그 ----
  if(streaming){
    Serial.print("D,"); Serial.print(millis());
    Serial.print(","); Serial.print(phi,3);
    Serial.print(","); Serial.print(ank,3);
    Serial.print(","); Serial.print(del,3);
    Serial.print(","); Serial.print(A,3);
    Serial.print(","); Serial.println(dcmd,3);
  }
  pollSerial();
}

void pollSerial(){
  if(!Serial.available()) return;
  char c=Serial.read();
  if(c=='z'){
    phi_zero=encRaw(PHI_CS); ank_zero=encRaw(ANK_CS);
    d_zero_raw=dxl.getPresentPosition(DXL_ID);
    dcmd=0; est_init=false; ev("ZERO",NAN,0);
  }
  else if(c=='g'){
    ctrl_on=!ctrl_on;
    if(ctrl_on){
      dxl.torqueOff(DXL_ID); delay(10);
      dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION); delay(10);
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, PROF_ACC_UNIT);
      dxl.writeControlTableItem(PROFILE_VELOCITY,     DXL_ID, PROF_VEL_UNIT);
      dxl.torqueOn(DXL_ID);
      dcmd = delDeg();                 // 현 자세에서 무점프 시작
      lock_until = millis()+200;
      ev("CTRL_ON",NAN,0);
    } else { ev("CTRL_OFF",NAN,0); }
  }
  else if(c=='s'){ streaming=!streaming; ev(streaming?"STREAM_ON":"STREAM_OFF",NAN,0); }
  else if(c=='x'){ dxl.torqueOff(DXL_ID); ctrl_on=false; ev("ESTOP",NAN,0); }
  else if(c=='r'){ dcmd=0; if(ctrl_on) sendDelta(0); ev("RET0",NAN,0); }
  else if(c=='+'){ K_FOLD+=1; ev("KFOLD",K_FOLD,0); }
  else if(c=='-'){ K_FOLD-=1; if(K_FOLD<1)K_FOLD=1; ev("KFOLD",K_FOLD,0); }
  else if(c=='['){ A_TRIG_DEG=max(0.2f,A_TRIG_DEG-0.1f); ev("TRIG",A_TRIG_DEG,0); }
  else if(c==']'){ A_TRIG_DEG+=0.1f; ev("TRIG",A_TRIG_DEG,0); }
  else if(c=='v'){ RET_DPS = (RET_DPS>=6.0f)?1.5f:(RET_DPS*2.0f); ev("RETDPS",RET_DPS,0); }
  else if(c=='t'){
    Serial.print("# ctrl="); Serial.print(ctrl_on);
    Serial.print(" K="); Serial.print(K_FOLD,1);
    Serial.print(" trig="); Serial.print(A_TRIG_DEG,2);
    Serial.print(" ret="); Serial.print(RET_DPS,1);
    Serial.print(" dcmd="); Serial.print(dcmd,2);
    Serial.print(" drop="); Serial.println(drop_cnt);
  }
}
