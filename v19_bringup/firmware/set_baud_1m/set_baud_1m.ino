/*
 * set_baud_1m.ino — 다이나믹셀 통신 속도를 57600 → 1Mbps로 변경
 * ============================================================
 * 사용법:
 *  1) 폴더 이름 = 파일 이름 규칙: set_baud_1m 폴더 안에 이 파일을 둔다
 *  2) 배터리 전원 ON 상태에서 OpenCR에 업로드
 *  3) 시리얼 모니터(115200)를 열면 자동으로 진행 과정이 출력된다
 *  4) "SUCCESS" 가 뜨면 exp_selftest_v19 를 다시 업로드 → 'DXL 1Mbps OK' 확인
 *
 * 모터를 57600에서 못 찾으면: 자동으로 전체 속도·전체 ID 스캔을 돌려
 * 모터가 실제로 어디에 있는지(어떤 ID, 어떤 속도) 알려준다.
 */
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DXL_DIR_PIN  84
const uint8_t DXL_ID = 1;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

void fullScan() {
  const uint32_t bauds[] = {9600, 57600, 115200, 1000000, 2000000, 3000000};
  Serial.println("--- 전체 스캔 시작 (1~2분 걸림) ---");
  for (uint8_t bi = 0; bi < 6; bi++) {
    Serial.print("scan @ "); Serial.println(bauds[bi]);
    dxl.begin(bauds[bi]);
    for (int id = 0; id <= 252; id++) {
      if (dxl.ping(id)) {
        Serial.print("  >>> 발견! ID="); Serial.print(id);
        Serial.print("  baud="); Serial.println(bauds[bi]);
      }
    }
  }
  Serial.println("--- 스캔 끝. 아무것도 안 나왔으면 배선·배터리 문제 ---");
}

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println("=== set_baud_1m: 57600 -> 1Mbps ===");
  dxl.setPortProtocolVersion(2.0);

  dxl.begin(57600);
  if (dxl.ping(DXL_ID)) {
    Serial.println("1) 모터 발견 @57600 (ID 1)");
    dxl.torqueOff(DXL_ID);
    Serial.println("2) 토크 OFF");
    if (dxl.writeControlTableItem(BAUD_RATE, DXL_ID, 3)) {   // 3 = 1Mbps (X시리즈)
      Serial.println("3) 속도 설정 쓰기 OK (1Mbps)");
    } else {
      Serial.println("3) 속도 설정 쓰기 실패 — 전원 껐다 켜고 리셋해서 재시도");
      return;
    }
    delay(800);
    dxl.begin(1000000);
    if (dxl.ping(DXL_ID)) {
      Serial.println("4) SUCCESS: 모터가 이제 1Mbps로 응답!");
      Serial.println("   -> exp_selftest_v19 를 다시 업로드하세요.");
    } else {
      Serial.println("4) 1Mbps 확인 실패 — 전원 껐다 켠 뒤 리셋 버튼을 눌러 재시도");
    }
  } else {
    Serial.println("모터가 57600(ID 1)에서 응답 없음 — 실제 위치를 찾는다:");
    fullScan();
  }
}

void loop() {}
