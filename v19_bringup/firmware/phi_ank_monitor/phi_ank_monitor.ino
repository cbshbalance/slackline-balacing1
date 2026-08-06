/*
 * phi_ank_monitor.ino v2 — 엔코더 2개 배선 점검 + 고장 부위 추적 (2026-08-06)
 * ==========================================================================
 * ★아두이노(OpenCR)에 업로드하는 펌웨어. 모터·배터리 없이 USB만으로 동작.
 *
 * v2: 8/6 밤 발목 엔코더 재고장(간헐 점프·고정값) 원인 부위를 찾기 위한 판.
 *     200Hz로 감시하면서 1초마다 채널별 통계를 찍는다:
 *       raw0 = 드롭아웃(응답 없음 → 배선 단선/전원)
 *       jump = 한 샘플에 30° 이상 튐(접촉 불량/노이즈)
 *       FRZ  = 1초 내내 값 완전 동일(고착)
 *
 * 사용법 — "흔들면서 본다":
 *  1) 업로드 → 시리얼 모니터 115200
 *  2) 로봇은 가만히 두고, 케이블·커넥터를 한 부위씩 손으로 흔든다
 *     (발목 보드 커넥터 → 중간 하네스 → OpenCR 쪽 순서로)
 *  3) 어느 부위를 만질 때 raw0/jump 카운터가 올라가면 = 거기가 범인
 *  4) 수리 후 다시 흔들어 카운터 0 유지 확인. 자석 갭도 확인
 *     (축 끝 자석이 칩 위 1~2mm, 접착 헐겁지 않게)
 *
 * 명령: z=영점  c=누적 카운터 리셋  f=출력 50Hz 토글(기본 10Hz)
 */
#include <SPI.h>

#define PHI_CS 10
#define ANK_CS 9

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

struct Ch {
  const char* name; int cs;
  float last; bool has_last;
  uint32_t raw0_1s, jump_1s, raw0_tot, jump_tot;
  float minv, maxv;
  void reset1s(){ raw0_1s=jump_1s=0; minv=1e9; maxv=-1e9; }
};
Ch chs[2] = {{"PHI",PHI_CS,0,false,0,0,0,0,1e9,-1e9},
             {"ANK",ANK_CS,0,false,0,0,0,0,1e9,-1e9}};

uint32_t show_ms=100;

void setup(){
  Serial.begin(115200); delay(500);
  pinMode(PHI_CS,OUTPUT); digitalWrite(PHI_CS,HIGH);
  pinMode(ANK_CS,OUTPUT); digitalWrite(ANK_CS,HIGH);
  SPI.begin();
  Serial.println("=== phi_ank_monitor v2 — 배선 점검·고장 부위 추적 ===");
  Serial.println("케이블을 부위별로 흔들며 raw0/jump 카운터를 본다. z=영점 c=카운터리셋 f=빠른출력");
  phi_zero=encRaw(PHI_CS); ank_zero=encRaw(ANK_CS);
  chs[0].reset1s(); chs[1].reset1s();
}

void loop(){
  // ---- 200Hz 감시 ----
  static uint32_t nx=0;
  if((int32_t)(micros()-nx) >= 0){
    nx = micros()+5000;
    for(int i=0;i<2;i++){
      Ch &c=chs[i];
      uint16_t r=encRaw(c.cs);
      if(r==0){ c.raw0_1s++; c.raw0_tot++; continue; }
      float v=wrapDeg(r, i==0?phi_zero:ank_zero);
      if(c.has_last && fabs(v-c.last)>30.0f){ c.jump_1s++; c.jump_tot++; }
      c.last=v; c.has_last=true;
      if(v<c.minv)c.minv=v; if(v>c.maxv)c.maxv=v;
    }
  }

  // ---- 값 표시 (10Hz 기본) ----
  static uint32_t nshow=0;
  if(millis()>=nshow){
    nshow=millis()+show_ms;
    Serial.print("phi=");
    Serial.print(chs[0].has_last?chs[0].last:0.0f,2);
    Serial.print("  ank=");
    Serial.print(chs[1].has_last?chs[1].last:0.0f,2);
    Serial.println();
  }

  // ---- 1초 통계 ----
  static uint32_t nstat=0;
  if(millis()>=nstat){
    nstat=millis()+1000;
    Serial.print("# ");
    for(int i=0;i<2;i++){
      Ch &c=chs[i];
      bool frz = c.has_last && (c.maxv-c.minv)<1e-6 && c.raw0_1s==0;
      Serial.print(c.name);
      Serial.print("[raw0:"); Serial.print(c.raw0_1s);
      Serial.print(" jump:"); Serial.print(c.jump_1s);
      if(frz) Serial.print(" FRZ");
      bool bad = c.raw0_1s>0 || c.jump_1s>0;
      Serial.print(bad?" ★]  ":" OK]  ");
      c.reset1s();
    }
    Serial.print("누적 raw0 "); Serial.print(chs[0].raw0_tot); Serial.print("/");
    Serial.print(chs[1].raw0_tot);
    Serial.print("  jump "); Serial.print(chs[0].jump_tot); Serial.print("/");
    Serial.println(chs[1].jump_tot);
  }

  if(Serial.available()){
    char ch=Serial.read();
    if(ch=='z'){ phi_zero=encRaw(PHI_CS); ank_zero=encRaw(ANK_CS); Serial.println("# zero set"); }
    else if(ch=='c'){ chs[0].raw0_tot=chs[0].jump_tot=0; chs[1].raw0_tot=chs[1].jump_tot=0; Serial.println("# counters cleared"); }
    else if(ch=='f'){ show_ms=(show_ms==100)?20:100; Serial.println(show_ms==20?"# 50Hz":"# 10Hz"); }
  }
}
