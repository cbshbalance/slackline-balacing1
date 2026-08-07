/*
 * crank_swing_log.ino — [실험 ① 10.1] 크랭크(줄) 단독 자유진동 로깅 (v19)
 * ======================================================================
 * 로봇 없이 줄의 진동수 ω 와 감쇠 c_φ 를 재기 위한 최소 스케치.
 * φ 엔코더 하나만 사용 — 모터·배터리·발목 엔코더 필요 없음 (USB 전원만!).
 *
 * ★확인: PHI_CS 핀 번호(기본 D10), ENC_PHI_DIR(브링업 확정값).
 *
 * [명령]  z : 영점 (크랭크 완전 정지 상태에서)
 *         s : 스트리밍 시작/정지 (200Hz, "D,t_ms,phi_deg")
 * PC 기록: python exp_logger.py COM3 0727_crank_r1.csv
 */
#include <SPI.h>

#define PHI_CS 10
int ENC_PHI_DIR = +1;          // ★브링업 확정값

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

bool streaming = false;

void setup(){
  Serial.begin(115200); delay(400);
  pinMode(PHI_CS, OUTPUT); digitalWrite(PHI_CS, HIGH);
  SPI.begin();
  uint16_t r;
  Serial.println("=== crank_swing_log (v19) ===");
  Serial.println(encRead(r) ? "phi enc: OK" : "phi enc: FAIL — 배선/CS 확인");
  Serial.println("명령: z(영점)  s(스트리밍)");
}

void loop(){
  static uint32_t next = 0;
  if (streaming && (int32_t)(micros() - next) >= 0) {
    next = micros() + 5000;                        // 200 Hz
    Serial.print("D,"); Serial.print(millis());
    Serial.print(","); Serial.println(phiDeg(), 4);
  }
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') { uint16_t r; if(encRead(r)) phi_zero = r*(2*PI/16384.0f);
                    Serial.println("E,0,ZERO"); }
    else if (c == 's') { streaming = !streaming;
                    Serial.println(streaming ? "E,0,STREAM_ON" : "E,0,STREAM_OFF"); }
  }
}
