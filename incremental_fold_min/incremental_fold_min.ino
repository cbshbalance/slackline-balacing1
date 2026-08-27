/*
 * incremental_fold_min.ino — 증분접기 최소 펌웨어 (2026-08-27)
 * ============================================================================
 * finale6(1736줄)에서 제어에 꼭 필요한 것만 남긴 판. 물리·제어 경로는 동일하다.
 *
 *  루프(200 Hz): 엔코더 3개 읽기 → φ,β 와 25ms차분+EMA 속도 → Â → 상태기계 → 서보
 *  판정량:  Â = w·(φ, β, φ̇, β̇) + c0/r,   w = [−1/r, 1, −1/(rλ), 1/λ]   (문서 46)
 *  상태:    α = ank − φ   (★문서 69 정정 — + 가 아니다),   β = α + P2R·δ
 *  상태기계: IDLE ─|Â|>trig→ hold += sgn·ρ·γ·Â → FOLD(프로파일 시간만 대기) → REST 60ms → IDLE
 *           IDLE 에서 |Â|<rel 이고 |hold|>1° 면 3°/s 로 천천히 편다
 *
 *  명령(115200): z 영점(완전히 멎은 뒤)  |  g 시작  |  h 정지(토크 유지)
 *               1 단발접기 모드 토글 — 첫 트리거에서 한 번만 접고 δ 고정(펴기도 없음), 로그는 계속
 *                 γ·λ 검증용: 접기 직전→직후 Â 비율이 γ 의 성적표(0 근처면 deadbeat,
 *                 같은 부호 남으면 γ 부족, 부호 넘어가 크면 과대), 그 뒤 ln|Â| 의
 *                 기울기가 λ 실측치다 (E1 재생·smoothed 참값으로 적합)
 *  로그: 실행 중 50 Hz CSV — finale6 과 같은 형식(fold_logger·E1 재생 호환)
 *
 *  ⚠⚠ 줄 위에 올리기 전에 FOLD_SIGN 을 눈으로 확인할 것 (finale6 헤더의 바닥 시험).
 *     부호가 반대면 넘어지는 쪽으로 접는다 — 한 번에 부러진다.
 *  ⚠ 200 Hz + 25 ms 차분 + EMA τ≈28 ms 는 한 묶음이다. 따로 바꾸지 말 것 (문서 46 §7).
 *  ⚠ 이 판에는 감시·복구·런타임 튜닝이 없다. 상수는 여기서 고치고 재컴파일한다.
 * ============================================================================
 */
#include <SPI.h>
#include <Dynamixel2Arduino.h>
#include <math.h>

// ---- 실측 3 + 부호 (문서 69) ----
const float P2R       = 0.433f;    // 실측① δ→발목각 기울기
const float LAMBDA    = 5.66f;     // 실측③ 발산율 [1/s]
const float R_SLOPE   = -1.506f;   // 실측② 안정모드선 기울기
const float LINE_C    = 0.0f;      // 실측② 절편 [deg] — 매달림 영점(문서 66)이 0 을 만든다
const float FOLD_SIGN = +1.0f;     // ★바닥 시험으로 확인

// ---- 노브 4 ----
const float GAMMA      = 11.0f;    // 접기 이득 γ (고원 4~12)
const float T_REST     = 60.0f;    // REST [ms]
const float A_TRIG     = 0.6f;     // 트리거 문턱 [deg]
const float A_RELAX    = 0.3f;     // 펴기 게이트 [deg] + 펴기 속도 RELAX_RATE

// ---- 나머지 상수 ----
const float RHO        = 0.95f;    // 감쇠계수
const float RELAX_RATE = 3.0f;     // 펴기 속도 [deg/s]
const float HOLD_DEAD  = 1.0f;     // 이보다 작은 유지각은 안 편다 [deg]
const float DELTA_LIM  = 55.0f;    // 힙 기구한계 [deg]
const float STEP_LIM   = 20.0f;    // 접기 1회 상한 [deg] — 잡음 한 샘플의 과대 커밋 방지
const float FOLD_TOL   = 2.0f;     // 도착 판정 [deg] — 서보 처짐(1~3°)보다 크게
const float FOLD_TMAX  = 300.0f;   // FOLD 상한 [ms]
const float ANG_LIMIT  = 30.0f;    // |φ| 나 |α| 초과 = 넘어짐 → 토크 OFF (지우려면 loop 의 한 줄)
const float VEL_UNIT   = 250.0f;   // PROFILE_VELOCITY  ≈344 deg/s
const float ACC_UNIT   = 373.0f;   // PROFILE_ACCELERATION ≈8000 deg/s²
const int   CUR_LIMIT  = 350;      // 전류 제한 [unit] — 접기 순간 전압 붕괴 방지 (finale6 §전원)

// ---- 파이프라인 (한 묶음 — 문서 46 §7) ----
const uint32_t DT_US = 5000;  const float DT_S = 0.005f;   // 200 Hz
const int   VEL_N = 5;                                     // 기저차분 25 ms
const float EMA_A = 0.15f;                                 // τ ≈ 28 ms

// ---- 하드웨어 (문서 17·52 배선) ----
#define DXL_SERIAL  Serial3
#define DXL_DIR_PIN 84
const uint8_t DXL_ID = 1, PHI_CS = 10, ANK_CS = 9;
const int   MOTOR_DIR    = +1;                  // 문서 37 확정
const float TICK_PER_DEG = 4096.0f / 360.0f;
const float VEL_DPS = 1.374f, ACC_DPS2 = 21.4577f;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// ---- 유도 상수 ----
const float W_PHI     = -1.0f / R_SLOPE;
const float W_PHIDOT  = -1.0f / (R_SLOPE * LAMBDA);
const float W_BETADOT =  1.0f / LAMBDA;
const float A_OFFSET  =  LINE_C / R_SLOPE;

// ---- 상태 ----
enum Phase { IDLE = 0, FOLD = 1, REST = 2 };
Phase phase = IDLE;
bool  running = false, motor_ok = false;
bool  once_mode = false, once_done = false;   // 1: 단발접기 — 첫 접기 후 δ 고정 (γ·λ 검증)
float home_tick = 0;  uint16_t phi_zero = 0, ank_zero = 0;
float hold = 0, delta_now = 0;
float phi_d = 0, ank_d = 0, alpha_d = 0, beta_d = 0, dphi = 0, dbeta = 0, Ahat = 0;
float phi_hist[VEL_N + 1], beta_hist[VEL_N + 1];
int   hist_i = 0;  bool primed = false;
uint32_t t0 = 0, next_us = 0, phase_t0 = 0, fold_wait_ms = 0, log_next_ms = 0;
uint8_t  relax_thr = 0;

// ---- AS5047P (SPI 모드1, 2회 읽기) ----
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

// ---- 모터 ----
void writeGoal(float deg) {
  if (motor_ok) dxl.setGoalPosition(DXL_ID, home_tick + MOTOR_DIR * deg * TICK_PER_DEG);
}
float readDelta() {
  if (!motor_ok) return hold;
  float t = dxl.getPresentPosition(DXL_ID);
  if (t == 0.0f) return delta_now;               // 읽기 실패(라이브러리가 0 반환) — 직전 값 유지
  return MOTOR_DIR * (t - home_tick) / TICK_PER_DEG;
}
float profileMs(float deg) {                     // 프로파일 소요시간 — 도착을 기다리지 않는다 (finale6 정정)
  deg = fabsf(deg);
  float acc = ACC_UNIT * ACC_DPS2, vmx = VEL_UNIT * VEL_DPS;
  if (deg <= 0.0f) return 20.0f;
  float t = (sqrtf(deg * acc) <= vmx) ? (2.0f * sqrtf(deg / acc)) : (deg / vmx + vmx / acc);
  return t * 1000.0f;
}

// ---- 상태 읽기 → Â ----
void readState() {
  phi_d = rawToDeg(as5047_raw(PHI_CS), phi_zero);
  ank_d = rawToDeg(as5047_raw(ANK_CS), ank_zero);
  delta_now = readDelta();
  alpha_d = ank_d - phi_d;                       // ★문서 69 정정: −
  beta_d  = alpha_d + P2R * delta_now;

  if (!primed) {
    for (int i = 0; i <= VEL_N; i++) { phi_hist[i] = phi_d; beta_hist[i] = beta_d; }
    dphi = dbeta = 0; primed = true;
  }
  phi_hist[hist_i] = phi_d;  beta_hist[hist_i] = beta_d;
  int old = (hist_i + 1) % (VEL_N + 1);
  float dphi_raw  = (phi_d  - phi_hist[old])  / (VEL_N * DT_S);
  float dbeta_raw = (beta_d - beta_hist[old]) / (VEL_N * DT_S);
  hist_i = old;
  dphi  += EMA_A * (dphi_raw  - dphi);
  dbeta += EMA_A * (dbeta_raw - dbeta);

  Ahat = W_PHI * phi_d + beta_d + W_PHIDOT * dphi + W_BETADOT * dbeta + A_OFFSET;
}

// ---- 상태기계 ----
void controlStep() {
  uint32_t now = millis();
  switch (phase) {
    case IDLE:
      if (once_mode && once_done) break;               // 단발접기 완료 — δ 고정, 관찰만
      if (fabsf(Ahat) > A_TRIG) {
        float step = FOLD_SIGN * RHO * GAMMA * Ahat;
        if (step >  STEP_LIM) step =  STEP_LIM;
        if (step < -STEP_LIM) step = -STEP_LIM;
        hold += step;
        if (hold >  DELTA_LIM) hold =  DELTA_LIM;
        if (hold < -DELTA_LIM) hold = -DELTA_LIM;
        writeGoal(hold);
        float fw = profileMs(step) * 1.3f + 20.0f;
        fold_wait_ms = (uint32_t)((fw > FOLD_TMAX) ? FOLD_TMAX : fw);
        phase = FOLD;  phase_t0 = now;
        if (once_mode) { once_done = true; Serial.println("# 단발접기 실행 — 이후 delta 고정"); }
      } else if (fabsf(Ahat) < A_RELAX && fabsf(hold) > HOLD_DEAD) {
        hold += (hold > 0 ? -1 : 1) * RELAX_RATE * DT_S;   // 천천히 펴기 — 안전할 때만
        if (++relax_thr >= 10) { relax_thr = 0; writeGoal(hold); }
      }
      break;
    case FOLD:
      if (fabsf(delta_now - hold) < FOLD_TOL || (uint32_t)(now - phase_t0) >= fold_wait_ms) {
        phase = REST;  phase_t0 = now;
      }
      break;
    case REST:
      if ((uint32_t)(now - phase_t0) >= (uint32_t)T_REST) phase = IDLE;
      break;
  }
}

// ---- 로그 (finale6 과 같은 형식 — fold_logger·E1 호환. cue·err 열은 0 고정) ----
void logLine() {
  Serial.print("D,");
  Serial.print(millis() - t0); Serial.print(',');
  Serial.print(phi_d, 3);      Serial.print(',');
  Serial.print(ank_d, 3);      Serial.print(',');
  Serial.print(alpha_d, 3);    Serial.print(',');
  Serial.print(beta_d, 3);     Serial.print(',');
  Serial.print(dphi, 2);       Serial.print(',');
  Serial.print(dbeta, 2);      Serial.print(',');
  Serial.print(Ahat, 4);       Serial.print(',');
  Serial.print(hold, 2);       Serial.print(',');
  Serial.print(delta_now, 2);  Serial.print(',');
  Serial.print((int)phase);    Serial.println(",0,0");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(PHI_CS, OUTPUT); digitalWrite(PHI_CS, HIGH);
  pinMode(ANK_CS, OUTPUT); digitalWrite(ANK_CS, HIGH);
#ifdef BDPIN_DXL_PWR_EN
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT); digitalWrite(BDPIN_DXL_PWR_EN, HIGH); delay(300);
#endif
  SPI.begin();

  dxl.setPortProtocolVersion(2.0);
  const uint32_t bauds[] = {1000000, 57600};
  for (int b = 0; b < 2 && !motor_ok; b++) {
    dxl.begin(bauds[b]);
    for (int i = 0; i < 3 && !motor_ok; i++) { if (dxl.ping(DXL_ID)) motor_ok = true; else delay(200); }
  }
  if (motor_ok) {
    dxl.torqueOff(DXL_ID); delay(50);
    dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION); delay(50);
    dxl.writeControlTableItem(RETURN_DELAY_TIME,    DXL_ID, 0);
    dxl.writeControlTableItem(PROFILE_VELOCITY,     DXL_ID, (int)VEL_UNIT);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, (int)ACC_UNIT);
    dxl.writeControlTableItem(CURRENT_LIMIT,        DXL_ID, CUR_LIMIT); delay(80);
    home_tick = dxl.getPresentPosition(DXL_ID);
    dxl.setGoalPosition(DXL_ID, home_tick); delay(50);
    dxl.torqueOn(DXL_ID);
    Serial.println("# 모터 OK");
  } else Serial.println("# !!! 모터 응답 없음 — 배터리/RS-485 확인");

  Serial.println("# incremental_fold_min — z 영점 / g 시작 / h 정지 / 1 단발접기 토글");
  Serial.println("# D,t_ms,phi,ank,alpha,beta,dphi,dbeta,Ahat,hold,del_now,phase,cue,err");
  t0 = millis();  next_us = micros();
}

void loop() {
  // 명령: z / g / h  한 글자
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') {                                        // ★완전히 멎은 상태에서만
      phi_zero = as5047_raw(PHI_CS);  ank_zero = as5047_raw(ANK_CS);
      if (motor_ok) {
        home_tick = dxl.getPresentPosition(DXL_ID);
        if (home_tick == 0.0f) Serial.println("# !! 영점 순간 모터가 안 읽혔다 — z 다시");
        dxl.setGoalPosition(DXL_ID, home_tick);            // 넘어짐 컷 후에도 z 로 토크 복구
        dxl.torqueOn(DXL_ID);
      }
      hold = 0; delta_now = 0; primed = false; dphi = dbeta = 0; Ahat = 0; phase = IDLE;
      once_done = false;
      Serial.println("# ZERO");
    }
    else if (c == 'g') { phase = IDLE; once_done = false; running = true;
                         Serial.println(once_mode ? "# GO (단발접기 모드)" : "# GO"); }
    else if (c == 'h') { running = false;               Serial.println("# STOP (토크 유지)"); }
    else if (c == '1') { once_mode = !once_mode; once_done = false;
                         Serial.println(once_mode ? "# 단발접기 모드 ON — 첫 트리거에서 한 번만 접는다"
                                                  : "# 단발접기 모드 OFF (연속 증분접기)"); }
  }

  uint32_t now = micros();
  if ((int32_t)(now - next_us) < 0) return;                // 200 Hz 페이싱
  next_us += DT_US;
  if ((int32_t)(micros() - next_us) >= 0) next_us = micros() + DT_US;

  readState();                                             // Â 는 제어 이전 값 (문서 46 §9)

  if (running && (fabsf(phi_d) > ANG_LIMIT || fabsf(alpha_d) > ANG_LIMIT)) {
    running = false;  if (motor_ok) dxl.torqueOff(DXL_ID); // 넘어짐 — 서보 보호. 최소판의 유일한 안전줄
    Serial.println("# 한계각 초과 — 토크 OFF. z 부터 다시");
  }

  if (running) {
    controlStep();
    uint32_t ms = millis();
    if ((int32_t)(ms - log_next_ms) >= 0) {                // 50 Hz CSV
      log_next_ms += 20;
      if ((int32_t)(ms - log_next_ms) > 100) log_next_ms = ms;
      logLine();
    }
  } else {                                                 // 정지 중: 4 Hz 모니터 (부호·영점 눈 확인용)
    static uint32_t mon_next = 0;
    uint32_t ms = millis();
    if ((int32_t)(ms - mon_next) >= 0) {
      mon_next = ms + 250;
      Serial.print("f=");    Serial.print(phi_d, 2);       // φ (피벗)
      Serial.print(" k=");   Serial.print(ank_d, 2);       // 발목
      Serial.print(" d=");   Serial.print(delta_now, 2);   // δ (모터)
      Serial.print(" | a="); Serial.print(alpha_d, 2);
      Serial.print(" b=");   Serial.print(beta_d, 2);
      Serial.print(" A=");   Serial.print(Ahat, 3);
      Serial.print(" hold="); Serial.println(hold, 1);
    }
  }
}
