/*
 * phi_ank_monitor.ino — φ(줄)·발목 엔코더만 읽어서 출력 (배선 확인용 최소 스케치)
 * =============================================================================
 * ★이 파일은 아두이노(OpenCR)에 업로드하는 펌웨어다. 모터·배터리 없이 USB만으로 동작.
 *
 * 용도: 시리얼 모니터에서 눈으로 직접 φ·발목 값이 살아있는지 본다.
 *  - raw=0  → 그 엔코더 배선 사망(전원/CS/MISO 확인)
 *  - 값이 안 변함(손으로 움직여도) → 배선 헐거움/자석 문제 (8/5 발목 증상)
 *  - 값이 마구 튐 → 접촉 불량/노이즈
 *
 * 사용:
 *  1) 업로드 → 시리얼 모니터 115200
 *  2) 10Hz로 두 각도가 계속 찍힌다. 크랭크/발목을 손으로 천천히 움직여 값이 따라오는지 확인
 *  3) 명령: z = 현재 위치를 0°로 (영점),  f = 빠른 출력(50Hz) 토글
 *
 * 배선(tilt_release_test와 동일): φ 엔코더 CS=D10, 발목 엔코더 CS=D9, SPI 공유.
 */
#include <SPI.h>

#define PHI_CS 10
#define ANK_CS 9

// ---------- 검증판 SPI 읽기 (tilt_release_test.ino와 동일) ----------
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

uint32_t period_ms = 100;      // 기본 10Hz (모니터로 읽기 편한 속도)

void setup(){
  Serial.begin(115200); delay(500);
  pinMode(PHI_CS,OUTPUT); digitalWrite(PHI_CS,HIGH);
  pinMode(ANK_CS,OUTPUT); digitalWrite(ANK_CS,HIGH);
  SPI.begin();
  Serial.println("=== phi_ank_monitor — 엔코더 2개 배선 확인 ===");
  Serial.println("명령: z=영점  f=빠른출력(50Hz) 토글");
  phi_zero = encRaw(PHI_CS);
  ank_zero = encRaw(ANK_CS);   // 시작 위치를 0°로
}

void loop(){
  if (Serial.available()) {
    char c = Serial.read();
    if (c=='z'){ phi_zero=encRaw(PHI_CS); ank_zero=encRaw(ANK_CS); Serial.println("# zero set"); }
    else if (c=='f'){ period_ms = (period_ms==100) ? 20 : 100;
                      Serial.println(period_ms==20 ? "# 50Hz" : "# 10Hz"); }
  }

  static uint32_t next=0;
  if (millis() < next) return;
  next = millis() + period_ms;

  uint16_t rp = encRaw(PHI_CS);
  uint16_t ra = encRaw(ANK_CS);

  Serial.print("phi= ");
  if (rp==0) Serial.print("---- ★배선확인(raw 0)");
  else { Serial.print(wrapDeg(rp,phi_zero), 2); Serial.print("\xC2\xB0 (raw "); Serial.print(rp); Serial.print(")"); }
  Serial.print("   |   ank= ");
  if (ra==0) Serial.print("---- ★배선확인(raw 0)");
  else { Serial.print(wrapDeg(ra,ank_zero), 2); Serial.print("\xC2\xB0 (raw "); Serial.print(ra); Serial.print(")"); }
  Serial.println();
}
