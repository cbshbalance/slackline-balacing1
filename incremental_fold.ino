/*
 * incremental_fold.ino — 증분접기 제어 (2026-08-20)
 * ============================================================================
 *
 *  【제어의 한 문장】
 *      예측점이 안정모드선에서 문턱 이상 벗어나면, 벗어난 만큼(γ배) 접어 다시 선 쪽으로 보낸다.
 *      선 위에 있으면 아무것도 하지 않는다.
 *
 *  【판정량 A — 넘어지는 쪽 성분의 크기, 단위는 β환산 도(°)】
 *      A = w·(φ, β, φ̇, β̇) + A_OFFSET
 *      w = [ −1/r , 1 , −1/(r·λ) , 1/λ ]        ← r 과 λ 만으로 결정된다 (문서 46)
 *      A = 0  ⟺  예측점 (β+β̇/λ, φ+φ̇/λ) 이 안정모드선 위에 있다
 *
 *      ★ 로봇이 볼 수 있는 것은 Â(추정값)뿐이다. 참 A 는 시뮬에만 있다.
 *        트리거는 Â 로 걸린다 — 그래서 Â 의 잡음이 문턱보다 작아야 한다 (문서 54).
 *
 *  【상태 만들기】
 *      α = ank - phi                (문서 37, 8/5 확정 규약)
 *      β = P1R·α + P2R·θ,  θ = α+δ,  P1R+P2R = 1   →   β = α + P2R·δ
 *      φ̇, β̇ : 25 ms 기저차분 → τ≈28 ms EMA        (실기 파이프라인, 한 묶음)
 *
 *  【상태기계 — γ 비례】
 *      IDLE  ─ |Â| > A_TRIG(0.6°) ?
 *              예   → hold += FOLD_SIGN·ρ·γ·Â   (ρ=0.95, γ=10, ±55° 클립) → FOLD
 *              아니오 → |Â| < A_RELAX(0.3°) 이고 |hold| > 1° 이면
 *                       hold 를 3°/s 로 중립쪽 감소 (= 천천히 펴기)
 *      FOLD  ─ 서보 프로파일이 δ 를 hold 로 이동. |δ−hold| < 0.5° (또는 0.6 s) → REST
 *      REST  ─ 60 ms 대기, 판정 없음 → IDLE
 *
 *      ★ 접는 양은 Â 에 비례한다 — 벗어난 만큼 되돌린다.
 *        평시에는 늘 문턱에서 발동하므로 ρ·γ·0.6 ≈ 5.7° 로 거의 일정하고,
 *        큰 외란이 왔을 때만 한 번에 크게 접는다. 그것이 γ 를 쓰는 이유다.
 *      ★ 펴기는 "안전할 때만" 한다 — 펴기 자체가 불안정 성분을 주입하기 때문 (문서 46 §7).
 *
 * ----------------------------------------------------------------------------
 * ⚠⚠ 줄 위에 올리기 전에 반드시 할 것 — FOLD_SIGN 확인 ⚠⚠
 *
 *   부호가 반대면 로봇은 넘어지는 쪽으로 스스로 접는다. 한 번에 부러진다.
 *
 *   [바닥 시험]  y  (dry-run: 판정·로그는 다 하고 모터만 안 움직임)
 *     1. 로봇을 손으로 들고 줄 위 자세로 세운다. z 로 영점.
 *     2. g 로 제어 시작 (dry-run 이므로 모터는 가만히 있다).
 *     3. 로봇을 한쪽으로 천천히 기울인다 → 화면의 Ahat 부호를 본다.
 *     4. 같은 방향으로 기울인 채, 손으로 허리를 화면이 말하는 dcmd 방향으로 접어 본다.
 *        ★ 그 접기가 로봇을 "기울어진 반대쪽으로" 되돌려야 한다.
 *        되돌리지 않고 더 넘어뜨리면 →  sgn -1  을 치면 된다 (재컴파일 불필요).
 *
 *   [교차검산]  매달기 시험(P2R)에서 δ 를 +로 주면 발목각 α 가 −로 갔다 (기울기 −0.433).
 *     즉 +δ 는 하체를 − 방향으로 돌린다. 줄 위에서 β>0 (오른쪽 기울어짐, A>0) 일 때
 *     필요한 것은 발이 무게중심을 지나쳐 오른쪽으로 가는 것 → +δ. 그래서 FOLD_SIGN = +1
 *     이 기본값이다. 하지만 이건 예측이지 측정이 아니다. 위 바닥 시험으로 눈으로 확인할 것.
 *
 * ----------------------------------------------------------------------------
 * 【배선】 문서 17·52 그대로 — 바꾸지 않았다
 *     φ CS = D10,  발목 CS = D9,  SCLK = D13, MISO = D12, MOSI = D11
 *     모터 XM430-W210-R, RS-485, ID 1, Serial3 / DIR = 84
 *     ★배터리 필수 — USB 전원만으로는 모터가 안 움직인다.
 *     (선택) 놓기신호 LED/부저 = D8
 *
 * 【명령】 115200 baud. 줄바꿈 설정 무관 (200 ms 조용하면 실행)
 *
 *   [동작]
 *     z        영점 — 엔코더 2개 + 모터 홈(δ=0). ★완전히 멎은 상태에서만
 *     g        제어 시작 (GO)          |  h  제어 정지 (토크는 유지, δ 그 자리)
 *     x        비상정지 — 토크 즉시 OFF |  k  토크 복구  |  u  토크 해제
 *     y        dry-run 토글 — 판정·로그만, 모터 명령 안 나감
 *     n        Â 잡음 측정 20 s (문서 54 방법). ★가진이 있는 상태에서 (흔들리는 중)
 *     j        발목 3점 중앙값 필터 토글 (기본 OFF — n 결과 보고 정한다)
 *     m        CSV 로그 ON/OFF   |  s 출력 정지/재개  |  p 1회 출력  |  t 상태 요약
 *     w        ★파라미터 전체 목록   |  d  현재 값을 소스 코드로 덤프  |  ? 도움말
 *     <정수>   δ 수동 이동 [°] — 제어 정지 중에만 먹는다
 *
 *   [값 바꾸기]  이름 값   — 값을 빼면 현재 값을 읽는다
 *     실측:  p2r 0.433 | lam 5.66 | r -1.506 | c0 -1.11 | sgn 1    ← 바꾸면 w 자동 재계산
 *     제어:  gam 10 | rho 0.95 | trig 0.6 | rel 0.3 | vrel 3 | dead 1
 *            dlim 55 | rest 60 | alim 30 | cue 0.3 | cuems 500
 *     기록:  loghz 50 | vel 250 | acc 373
 *     별칭:  f=gam  c=trig  e=rel  o=rho  l=loghz  v=vel  a=acc
 *
 *   ⚠ 값은 전원을 끄면 이 파일의 기본값으로 돌아간다.
 *     확정되면 `d` 를 쳐서 나온 블록을 아래 상수 블록에 붙여넣고 다시 컴파일할 것.
 *
 * ----------------------------------------------------------------------------
 * 【브링업 순서】
 *   1. z            영점 (똑바로 선 자세, 완전히 멎은 뒤)
 *   2. n            Â 잡음 20 s — ★가진이 있는 상태에서. 0.15° 이하가 아니면 여기서 멈추고
 *                   발목 센서부터 고친다 (자석 갭 → 배선 → 접지).
 *                   ⚠ 문턱(trig)을 올려 피하려 하지 말 것 — 축약모형에서 σ=0.41° 는 문턱을
 *                     2.5° 까지 올려도 12회 중 최대 3회만 살았다. γ 비례에서는 잡음이
 *                     트리거뿐 아니라 '접기량'에도 그대로 들어가기 때문이다.
 *   3. y → g        dry-run 으로 FOLD_SIGN 확인 (위 바닥 시험)
 *   4. h → y        dry-run 해제
 *   5. m, loghz 50  로그 켜기
 *   6. z → g        줄 위에서 놓기. READY(부저/LED) 뜨면 놓는다.
 *
 * 【γ 노브 맞추는 법】  (f 10  또는  gam 10)
 *   너무 작으면: 접어도 A 의 부호가 안 바뀌어 δ 가 한 방향으로 쌓이다 ±55° 에 붙는다.
 *                → 로그의 hold 가 단조증가하면 γ 를 키운다.
 *   너무 크면:   반대편으로 튕겨 스윙이 커지고 접기 횟수가 는다.
 *                → φ 스윙 rms 가 커지면 γ 를 줄인다.
 *   문서 46 의 작동 고원은 γ = 4~12 (3배 범위) 이고 실측 deadbeat 는 γ ≈ 7~8.
 *   기본 10 은 그보다 조금 크게 잡아 둔 것이다 (유한시간·비선형 손실을 흡수).
 * ============================================================================
 */
#include <SPI.h>
#include <Dynamixel2Arduino.h>
#include <math.h>

// ============================================================================
// ★★★ 조정 가능한 값 — 전부 런타임에 바꿀 수 있다 ★★★
//
//   여기 적힌 것은 '전원을 켰을 때의 기본값'일 뿐이다.
//   시리얼에서  이름 값   으로 언제든 바꾼다.   예:  lam 5.42   /   trig 0.8
//   `v` 로 전체 목록,  `d` 로 여기에 붙여넣을 수 있는 C 코드 블록을 출력한다.
//   ⚠ 전원을 끄면 이 파일의 값으로 돌아간다. 확정되면 `d` 출력을 여기에 붙여넣을 것.
// ============================================================================

// ---- 실측 상수 (제어 정지 중에만 바뀐다. 바꾸면 w 가 자동 재계산된다) ----
float P2R      = 0.433f;    // [p2r] 실측① 매달고 δ→발목각 직선의 기울기 (문서 64 재실측)
float LAMBDA   = 5.66f;     // [lam] 실측③ 발산율 [1/s] — 배가 122 ms (문서 69 교차검증)
float R_SLOPE  = -1.506f;   // [r  ] 실측② 안정모드선 기울기 ±0.074 (문서 69 실측)
float LINE_C   = -1.11f;    // [c0 ] 실측② 안정모드선 절편 [deg] ±0.86 ⚠미확정 (문서 69 §6)
float FOLD_SIGN = +1.0f;    // [sgn] 접기 방향 ±1 — ⚠ 헤더의 바닥 시험으로 확인할 것

// ---- 제어 ----
float GAMMA       = 10.0f;  // [gam ] ★접기 이득 γ — 줄 위 노브. Δδ = ρ·γ·Â
float RHO         = 0.95f;  // [rho ] 감쇠계수 ρ
float A_TRIG      = 0.6f;   // [trig] ★트리거 문턱 [deg]
float A_RELAX     = 0.3f;   // [rel ] 펴기 게이트 [deg]
float RELAX_RATE  = 3.0f;   // [vrel] 펴기 속도 [deg/s]
float HOLD_DEADBAND = 1.0f; // [dead] 이보다 작은 유지각은 안 편다 [deg]
float DELTA_LIMIT = 55.0f;  // [dlim] 힙 기구한계 [deg] (정지 중에만)
float T_REST      = 60.0f;  // [rest] REST 대기 [ms]
float ANG_LIMIT   = 30.0f;  // [alim] |φ| 또는 |α| 가 넘으면 토크 OFF [deg]
float CUE_TH      = 0.3f;   // [cue ] 놓기신호 문턱 [deg] (문서 50)
float CUE_HOLD    = 500.0f; // [cuems] 놓기신호 유지시간 [ms]

// ---- 기록·모터 ----
float LOG_HZ      = 50.0f;  // [loghz] CSV 로그 [Hz]  (115200 baud 에서 100 이 한계)
float VEL_UNIT    = 250.0f; // [vel ] PROFILE_VELOCITY     [unit] ≈344 deg/s
float ACC_UNIT    = 373.0f; // [acc ] PROFILE_ACCELERATION [unit] ≈8000 deg/s^2

// ---- 안 바꾸는 것 ----
const float FOLD_TOL = 0.5f;              // 접기 도착 판정 [deg]
const uint32_t FOLD_TIMEOUT_MS = 600;     // 접기 포기 [ms]

// ============================================================================
// 실기 파이프라인 — ⚠ 셋은 한 묶음이다. 따로 바꾸면 잡종이 된다 (문서 45의 교훈)
//   그래서 런타임 변경 대상에서 일부러 뺐다. 바꿀 일이 있으면 셋을 함께 고칠 것.
// ============================================================================
const uint32_t DT_US   = 5000;      // 200 Hz
const float    DT_S    = 0.005f;
const int      VEL_N   = 5;         // 기저차분 5샘플 = 25 ms
const float    EMA_A   = 0.15f;     // τ = dt(1−a)/a ≈ 28 ms

// ============================================================================
// 하드웨어
// ============================================================================
#define DXL_SERIAL   Serial3
#define DXL_DIR_PIN  84
const uint8_t DXL_ID  = 1;
const uint8_t PHI_CS  = 10;
const uint8_t ANK_CS  = 9;
const uint8_t CUE_PIN = 8;          // 놓기신호 LED/부저 (없어도 무해)

const int   MOTOR_DIR     = +1;                  // 문서 37 확정
const float TICK_PER_DEG  = 4096.0f / 360.0f;
const float VEL_UNIT_DPS  = 1.374f;
const float ACC_UNIT_DPS2 = 21.4577f;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// ============================================================================
// 유도 상수 — setup() 에서 위 실측값으로부터 계산된다 (하드코딩 없음)
// ============================================================================
float W_PHI, W_BETA, W_PHIDOT, W_BETADOT, A_OFFSET;

void deriveConstants() {
  //   A = −(1/r)·(φ − r·β − c) + 속도항
  //     = (−1/r)·φ + 1·β + (−1/(rλ))·φ̇ + (1/λ)·β̇ + c/r
  W_PHI     = -1.0f / R_SLOPE;
  W_BETA    =  1.0f;
  W_PHIDOT  = -1.0f / (R_SLOPE * LAMBDA);
  W_BETADOT =  1.0f / LAMBDA;
  A_OFFSET  =  LINE_C / R_SLOPE;
}

// ============================================================================
// 상태
// ============================================================================
enum Phase { IDLE = 0, FOLD = 1, REST = 2, FALLEN = 3 };
Phase phase = IDLE;

bool running   = false;   // g / h
bool dry_run   = false;   // y  — 모터 명령을 내보내지 않는다
bool motor_ok  = false;
bool out_on    = true;
bool csv_on    = false;
bool ank_med3  = false;   // j  — 발목 3점 중앙값 필터

uint32_t dxl_baud = 0;
float    home_tick = 0;
uint16_t phi_zero = 0, ank_zero = 0;

float hold      = 0.0f;   // 목표 유지각 δ [deg] — 누적된다
float delta_now = 0.0f;   // 서보가 읽는 실제 δ
float phi_d = 0, ank_d = 0, alpha_d = 0, beta_d = 0;
float dphi = 0, dbeta = 0;
float Ahat = 0;           // ★ control_step 이전 값 (문서 46 §9)

float phi_hist[VEL_N + 1], beta_hist[VEL_N + 1];
int   hist_i = 0;
bool  primed = false;

float ank_m[3] = {0, 0, 0};
int   ank_mi = 0;
bool  ank_m_primed = false;

uint32_t t0 = 0, next_us = 0, phase_t0 = 0;
uint32_t fold_count = 0, overrun = 0, dropped = 0;
uint32_t cycle_max_us = 0;

uint32_t cue_since = 0;
bool     cue_on = false;

// 로그
uint32_t log_next_ms = 0;

// 수동 δ 명령용 (제어 정지 중)
long manual_cmd = 0;

// 잡음 측정
bool     noise_on = false;
uint32_t noise_end = 0;
double   ns_phi = 0, ns_ank = 0, ns_del = 0, ns_A = 0;
uint32_t ns_n = 0;
float    sl_phi = 0, sl_ank = 0, sl_del = 0, sl_A = 0;
bool     ns_primed = false;
const float SLOW_A = 0.05f;   // τ = 100 ms @200 Hz — 이 위의 성분만 '잡음'으로 본다

// 시리얼 입력
char     linebuf[24];
uint8_t  linelen = 0;
uint32_t last_rx_ms = 0;

// ============================================================================
// AS5047P (SPI 모드1, ANGLECOM)
// ============================================================================
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

float med3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// ============================================================================
// 모터
// ============================================================================
float readDelta() {
  if (!motor_ok) return hold;                    // 모터 없으면 명령값으로 대신 (벤치 시험용)
  float t = dxl.getPresentPosition(DXL_ID);
  float d = MOTOR_DIR * (t - home_tick) / TICK_PER_DEG;
  if (fabsf(d) > 120.0f) return delta_now;       // 통신 사고 — 직전 값 유지
  return d;
}

void writeGoal(float deg) {
  if (!motor_ok || dry_run) return;
  if (deg >  DELTA_LIMIT) deg =  DELTA_LIMIT;
  if (deg < -DELTA_LIMIT) deg = -DELTA_LIMIT;
  dxl.setGoalPosition(DXL_ID, home_tick + MOTOR_DIR * deg * TICK_PER_DEG);
}

void torqueRestoreHere() {
  if (!motor_ok) return;
  dxl.setGoalPosition(DXL_ID, dxl.getPresentPosition(DXL_ID));
  dxl.torqueOn(DXL_ID);
}

void emergencyStop(const char* why) {
  running = false;
  phase   = FALLEN;
  if (motor_ok) dxl.torqueOff(DXL_ID);
  linelen = 0;
  Serial.print(">>> STOP (torque off) : "); Serial.println(why);
  Serial.println("    k 토크복구 → z 영점 → g 재시작");
}

// ============================================================================
// 상태 추정 — 한 사이클
// ============================================================================
void readState() {
  float phi = rawToDeg(as5047_raw(PHI_CS), phi_zero);
  float ank = rawToDeg(as5047_raw(ANK_CS), ank_zero);

  if (ank_med3) {                                 // 발목만 3점 중앙값 (문서 54: 0.223→0.138°)
    if (!ank_m_primed) { ank_m[0] = ank_m[1] = ank_m[2] = ank; ank_m_primed = true; }
    ank_m[ank_mi] = ank;
    ank_mi = (ank_mi + 1) % 3;
    ank = med3(ank_m[0], ank_m[1], ank_m[2]);
  }

  delta_now = readDelta();

  phi_d   = phi;
  ank_d   = ank;
  alpha_d = ank - phi;                            // 문서 37
  beta_d  = alpha_d + P2R * delta_now;            // β = α + P2R·δ

  if (!primed) {
    for (int i = 0; i <= VEL_N; i++) { phi_hist[i] = phi_d; beta_hist[i] = beta_d; }
    dphi = dbeta = 0; primed = true;
  }

  phi_hist[hist_i]  = phi_d;
  beta_hist[hist_i] = beta_d;
  int old = (hist_i + 1) % (VEL_N + 1);

  float dphi_raw  = (phi_d  - phi_hist[old])  / (VEL_N * DT_S);   // 25 ms 기저차분
  float dbeta_raw = (beta_d - beta_hist[old]) / (VEL_N * DT_S);
  hist_i = old;

  dphi  += EMA_A * (dphi_raw  - dphi);                            // τ ≈ 28 ms
  dbeta += EMA_A * (dbeta_raw - dbeta);

  // ★ 이 값이 판정에 쓰이는 Â 이고, 로그에 남는 것도 이 값이다 (제어 동작 이전)
  Ahat = W_PHI * phi_d + W_BETA * beta_d
       + W_PHIDOT * dphi + W_BETADOT * dbeta + A_OFFSET;
}

// ============================================================================
// 상태기계
// ============================================================================
void controlStep() {
  uint32_t now = millis();

  switch (phase) {
    case IDLE: {
      if (fabsf(Ahat) > A_TRIG) {
        float step = FOLD_SIGN * RHO * GAMMA * Ahat;    // ★벗어난 만큼 되돌린다
        hold += step;
        if (hold >  DELTA_LIMIT) hold =  DELTA_LIMIT;
        if (hold < -DELTA_LIMIT) hold = -DELTA_LIMIT;
        writeGoal(hold);
        fold_count++;
        phase = FOLD;
        phase_t0 = now;
      }
      else if (fabsf(Ahat) < A_RELAX && fabsf(hold) > HOLD_DEADBAND) {
        float d = RELAX_RATE * DT_S;                    // 천천히 펴기 — 안전할 때만
        hold += (hold > 0 ? -d : d);
        if (fabsf(hold) < 0.05f) hold = 0;
        static uint8_t thr = 0;                         // 목표 갱신은 20 Hz 로 솎는다
        if (++thr >= 10) { thr = 0; writeGoal(hold); }
      }
      break;
    }

    case FOLD: {
      if (dry_run ||                                    // 모터가 안 움직이므로 도착을 기다리지 않는다
          fabsf(delta_now - hold) < FOLD_TOL ||
          (uint32_t)(now - phase_t0) >= FOLD_TIMEOUT_MS) {
        phase = REST;
        phase_t0 = now;
      }
      break;
    }

    case REST: {
      if ((uint32_t)(now - phase_t0) >= (uint32_t)T_REST) phase = IDLE;   // 히스테리시스 없음
      break;
    }

    case FALLEN: default: break;
  }
}

// ============================================================================
// 놓기 신호 (문서 50) — |Â| < 0.3° 가 500 ms 유지되면 점등
// ============================================================================
void updateCue() {
  uint32_t now = millis();
  if (fabsf(Ahat) < CUE_TH) {
    if (cue_since == 0) cue_since = now;
    if (!cue_on && (uint32_t)(now - cue_since) >= (uint32_t)CUE_HOLD) {
      cue_on = true;
      digitalWrite(CUE_PIN, HIGH);
      Serial.print("# READY  Ahat="); Serial.println(Ahat, 3);
    }
  } else {
    cue_since = 0;
    if (cue_on) { cue_on = false; digitalWrite(CUE_PIN, LOW); }
  }
}

// ============================================================================
// Â 잡음 바닥 측정 (문서 54 방법의 온보드판)
//   느린 성분(τ=100 ms EMA)을 뺀 고주파 잔차의 rms = 센서 잡음
// ============================================================================
void noiseStart() {
  noise_on = true; ns_primed = false;
  ns_phi = ns_ank = ns_del = ns_A = 0; ns_n = 0;
  noise_end = millis() + 20000;
  Serial.println("# 잡음 측정 20 s 시작 — 제어는 멈춘다.");
  Serial.println("# ⚠ 반드시 '가진이 있는 상태'에서. 정지 상태로 재면 항상 깨끗하게 나와 의미가 없다.");
  Serial.println("#   (줄 위에서 흔들리는 중, 또는 손으로 잡고 계속 흔들면서)");
}

void noiseAccum() {
  if (!ns_primed) {
    sl_phi = phi_d; sl_ank = ank_d; sl_del = delta_now; sl_A = Ahat;
    ns_primed = true; return;
  }
  sl_phi += SLOW_A * (phi_d     - sl_phi);
  sl_ank += SLOW_A * (ank_d     - sl_ank);
  sl_del += SLOW_A * (delta_now - sl_del);
  sl_A   += SLOW_A * (Ahat      - sl_A);

  double a = phi_d - sl_phi;      ns_phi += a * a;
  double b = ank_d - sl_ank;      ns_ank += b * b;
  double c = delta_now - sl_del;  ns_del += c * c;
  double d = Ahat - sl_A;         ns_A   += d * d;
  ns_n++;
}

void noiseReport() {
  noise_on = false;
  if (ns_n < 100) { Serial.println("# 표본 부족 — 다시"); return; }
  float r_phi = sqrt(ns_phi / ns_n), r_ank = sqrt(ns_ank / ns_n);
  float r_del = sqrt(ns_del / ns_n), r_A   = sqrt(ns_A   / ns_n);

  Serial.println("==== Â 잡음 바닥 (고주파 rms, 20 s) ====");
  Serial.print("  phi  = "); Serial.print(r_phi, 4); Serial.println(" deg");
  Serial.print("  ank  = "); Serial.print(r_ank, 4); Serial.print(" deg");
  Serial.print("   <- 문서 54 기준 0.223 (조치 전) / 0.05 이하 (목표)");
  Serial.println();
  Serial.print("  del  = "); Serial.print(r_del, 4); Serial.println(" deg");
  Serial.print("  Ahat = "); Serial.print(r_A, 4);   Serial.print(" deg");
  Serial.print("   <- 문턱 "); Serial.print(A_TRIG, 2); Serial.println("");
  Serial.print("  Ahat / 문턱 = "); Serial.println(r_A / A_TRIG, 2);
  Serial.println("---- 판정 ----");
  if (r_A < 0.25f * A_TRIG) {
    Serial.println("  합격. 헛트리거 걱정 없음. 필터 추가 불필요.");
  } else if (r_A < 0.5f * A_TRIG) {
    Serial.println("  경계. 돌려는 보되 헛트리거 빈도를 세어 볼 것.");
  } else {
    Serial.println("  ★불합격 — 가만히 있어도 접는다.");
    if (r_ank > 3.0f * r_phi)
      Serial.println("   원인은 발목 채널이다 (phi 대비 과다). 순서: 자석 갭·정렬 → 배선 → 접지.");
    Serial.println("   임시방편: j 로 발목 3점 중앙값 필터 ON (문서 54 실측 −38%).");
    Serial.println("   그래도 모자라면 문턱을 올리는 대신 하드웨어를 고칠 것 —");
    Serial.println("   문턱을 올리면 회복 천장(2.8~4.0°)까지의 여유를 그만큼 버린다.");
  }
  Serial.println("=========================================");
}

// ============================================================================
// 출력
// ============================================================================
void ps(float v, int nd) { if (v >= 0) Serial.print('+'); Serial.print(v, nd); }

const char* phaseName() {
  switch (phase) { case IDLE: return "IDLE"; case FOLD: return "FOLD";
                   case REST: return "REST"; default: return "STOP"; }
}

void printState() {
  Serial.print(running ? (dry_run ? "DRY " : "RUN ") : "off ");
  Serial.print(phaseName());
  Serial.print(" | A="); ps(Ahat, 3);
  Serial.print(" | b=");   ps(beta_d, 2);
  Serial.print(" f=");     ps(phi_d, 2);
  Serial.print(" | db=");  ps(dbeta, 1);
  Serial.print(" df=");    ps(dphi, 1);
  Serial.print(" | hold="); ps(hold, 2);
  Serial.print(" d=");      ps(delta_now, 2);
  Serial.print(" | n=");    Serial.print(fold_count);
  if (cue_on) Serial.print("  READY");
  Serial.println();
}

void logHeader() {
  Serial.println("# D,t_ms,phi,ank,alpha,beta,dphi,dbeta,Ahat,hold,del_now,phase,cue");
  Serial.println("# Ahat 는 control_step 이전 값이다 (문서 46 §9)");
}

void logLine() {
  // 시리얼이 밀리면 제어루프가 멈춘다 — 자리 없으면 그냥 버린다
  if (Serial.availableForWrite() < 80) { dropped++; return; }
  Serial.print("D,");
  Serial.print(millis() - t0);  Serial.print(',');
  Serial.print(phi_d, 3);       Serial.print(',');
  Serial.print(ank_d, 3);       Serial.print(',');
  Serial.print(alpha_d, 3);     Serial.print(',');
  Serial.print(beta_d, 3);      Serial.print(',');
  Serial.print(dphi, 2);        Serial.print(',');
  Serial.print(dbeta, 2);       Serial.print(',');
  Serial.print(Ahat, 4);        Serial.print(',');
  Serial.print(hold, 2);        Serial.print(',');
  Serial.print(delta_now, 2);   Serial.print(',');
  Serial.print((int)phase);     Serial.print(',');
  Serial.println(cue_on ? 1 : 0);
}

void printStatus() {
  Serial.println("---- 상태 ----");
  Serial.print("모터: "); Serial.print(motor_ok ? "OK @" : "응답 없음");
  if (motor_ok) { Serial.print(dxl_baud);
    Serial.print("  torque="); Serial.print(dxl.getTorqueEnableStat(DXL_ID) ? "ON" : "OFF"); }
  Serial.println();
  Serial.print("제어: "); Serial.print(running ? "RUN" : "정지");
  Serial.print(dry_run ? "  [DRY-RUN — 모터 명령 안 나감]" : "");
  Serial.print("   발목중앙값: "); Serial.println(ank_med3 ? "ON" : "OFF");

  Serial.println("-- 실측 상수 (w 를 만드는 값) --");
  Serial.print("  p2r="); Serial.print(P2R, 4);
  Serial.print("  lam="); Serial.print(LAMBDA, 3);
  Serial.print(" (T2="); Serial.print(693.1f / LAMBDA, 0); Serial.print(" ms)");
  Serial.print("  r=");   Serial.print(R_SLOPE, 4);
  Serial.print("  c0=");  Serial.print(LINE_C, 3);
  Serial.print("  sgn="); Serial.println(FOLD_SIGN, 0);

  Serial.println("-- 유도된 w (r, lam 에서 계산 — 직접 못 바꾼다) --");
  Serial.print("  w = ["); Serial.print(W_PHI, 5);    Serial.print(", ");
  Serial.print(W_BETA, 5);    Serial.print(", ");
  Serial.print(W_PHIDOT, 5);  Serial.print(", ");
  Serial.print(W_BETADOT, 5); Serial.print("]   A_offset=");
  Serial.println(A_OFFSET, 5);
  Serial.println("  환율: beta 1deg=1.00 | phi 1deg=w0 | betadot 1dps=1/lam");

  Serial.println("-- 제어 --");
  Serial.print("  gam="); Serial.print(GAMMA, 2);
  Serial.print("  rho="); Serial.print(RHO, 3);
  Serial.print("  trig="); Serial.print(A_TRIG, 2);
  Serial.print("  rel="); Serial.print(A_RELAX, 2);
  Serial.print("  -> 문턱에서 접기량 "); Serial.print(RHO * GAMMA * A_TRIG, 2);
  Serial.print(" deg / 프로파일 ");
  Serial.print(2000.0f * sqrtf(RHO * GAMMA * A_TRIG / (ACC_UNIT * ACC_UNIT_DPS2)), 0);
  Serial.println(" ms");

  Serial.println("-- 실행 --");
  Serial.print("  접기 "); Serial.print(fold_count);
  Serial.print("회  overrun="); Serial.print(overrun);
  Serial.print("  로그버림="); Serial.print(dropped);
  Serial.print("  cycle_max="); Serial.print(cycle_max_us); Serial.println(" us");
  Serial.print("  엔코더 raw: phi="); Serial.print(as5047_raw(PHI_CS));
  Serial.print("  ank=");            Serial.println(as5047_raw(ANK_CS));
  Serial.println("  (전체 목록은 w, 소스로 덤프는 d)");
  Serial.println("--------------");
  printState();
}

// ============================================================================
// 파라미터 표 — 이름 하나로 읽고 쓴다
// ============================================================================
struct Param {
  const char* name;
  float*      p;
  float       lo, hi;
  bool        derive;   // 바꾸면 w 재계산 + 추정기 리셋
  bool        lock;     // 제어 중에는 못 바꾼다
  const char* unit;
  const char* what;
};

const Param PARAMS[] = {
  // 실측 상수
  {"p2r",  &P2R,          0.05f,  0.95f,  true,  true,  "",      "실측(1) 매달기 기울기"},
  {"lam",  &LAMBDA,       0.5f,  30.0f,   true,  true,  "1/s",   "실측(3) 발산율"},
  {"r",    &R_SLOPE,    -20.0f,  20.0f,   true,  true,  "",      "실측(2) 안정모드선 기울기"},
  {"c0",   &LINE_C,     -20.0f,  20.0f,   true,  true,  "deg",   "실측(2) 안정모드선 절편"},
  {"sgn",  &FOLD_SIGN,   -1.0f,   1.0f,   false, true,  "",      "접기 방향 +-1"},
  // 제어
  {"gam",  &GAMMA,        1.0f,  40.0f,   false, false, "",      "★접기 이득 gamma"},
  {"rho",  &RHO,          0.1f,   1.5f,   false, false, "",      "감쇠계수 rho"},
  {"trig", &A_TRIG,       0.05f,  5.0f,   false, false, "deg",   "★트리거 문턱"},
  {"rel",  &A_RELAX,      0.02f,  5.0f,   false, false, "deg",   "펴기 게이트"},
  {"vrel", &RELAX_RATE,   0.0f,  30.0f,   false, false, "deg/s", "펴기 속도"},
  {"dead", &HOLD_DEADBAND,0.0f,  10.0f,   false, false, "deg",   "펴기 데드밴드"},
  {"dlim", &DELTA_LIMIT,  5.0f,  80.0f,   false, true,  "deg",   "힙 기구한계"},
  {"rest", &T_REST,       0.0f, 500.0f,   false, false, "ms",    "REST 대기"},
  {"alim", &ANG_LIMIT,    5.0f,  90.0f,   false, false, "deg",   "안전 한계각"},
  {"cue",  &CUE_TH,       0.02f,  5.0f,   false, false, "deg",   "놓기신호 문턱"},
  {"cuems",&CUE_HOLD,    50.0f, 5000.0f,  false, false, "ms",    "놓기신호 유지"},
  // 기록·모터
  {"loghz",&LOG_HZ,       1.0f, 200.0f,   false, false, "Hz",    "CSV 로그 주기"},
  {"vel",  &VEL_UNIT,     1.0f,1023.0f,   false, false, "unit",  "프로파일 속도"},
  {"acc",  &ACC_UNIT,     1.0f,32767.0f,  false, false, "unit",  "프로파일 가속"},
};
const int N_PARAM = sizeof(PARAMS) / sizeof(PARAMS[0]);

// 한 글자 별칭 — 손에 익은 것만 남긴다
struct Alias { char c; const char* name; };
const Alias ALIASES[] = {
  {'f', "gam"}, {'c', "trig"}, {'e', "rel"}, {'o', "rho"},
  {'l', "loghz"}, {'v', "vel"}, {'a', "acc"},
};
const int N_ALIAS = sizeof(ALIASES) / sizeof(ALIASES[0]);

// 소수를 문자열로 — Arduino 의 일부 코어는 sprintf 의 %f 를 지원하지 않는다
void fmtF(char* out, float v, int nd) {
  bool neg = v < 0; if (neg) v = -v;
  long scale = 1; for (int i = 0; i < nd; i++) scale *= 10;
  long n = (long)(v * scale + 0.5f);
  long ip = n / scale, fp = n % scale;
  char* o = out;
  if (neg) *o++ = '-';
  // 정수부
  char tmp[12]; int t = 0;
  if (ip == 0) tmp[t++] = '0';
  while (ip > 0) { tmp[t++] = '0' + (ip % 10); ip /= 10; }
  while (t > 0) *o++ = tmp[--t];
  if (nd > 0) {
    *o++ = '.';
    for (int i = nd - 1; i >= 0; i--) { long d = fp; for (int k = 0; k < i; k++) d /= 10; *o++ = '0' + (d % 10); }
  }
  *o = '\0';
}

int findParam(const char* name) {
  for (int i = 0; i < N_PARAM; i++) if (!strcmp(PARAMS[i].name, name)) return i;
  return -1;
}

void printParam(int i, bool bare = false) {
  const Param& q = PARAMS[i];
  if (!bare) Serial.print("# ");
  Serial.print(q.name);
  for (int k = strlen(q.name); k < 6; k++) Serial.print(' ');
  Serial.print("= ");
  float v = *q.p;
  char nb[20];
  fmtF(nb, v, (fabsf(v) >= 100.0f) ? 1 : 4);
  Serial.print(nb);
  for (int k = strlen(nb); k < 10; k++) Serial.print(' ');
  Serial.print(q.unit);
  for (int k = strlen(q.unit); k < 6; k++) Serial.print(' ');
  Serial.print(q.what);
  if (q.lock)   Serial.print("   [정지 중에만]");
  if (q.derive) Serial.print(" [w 재계산]");
  Serial.println();
}

// 값이 이상하면 그냥 막지 말고 말해 준다 — 실측이 예상과 다를 수도 있으므로
void sanityWarn(int i) {
  const Param& q = PARAMS[i];
  if (q.p == &R_SLOPE) {
    if (R_SLOPE > -0.05f && R_SLOPE < 0.05f) Serial.println("!! r 이 0 근처 — w 가 폭발한다");
    else if (R_SLOPE > 0) Serial.println("!! r 이 양수다. 이론상 phi 와 beta 는 반대부호 (실측 -1.506). 확인할 것");
    else if (R_SLOPE < -4.0f || R_SLOPE > -0.5f) Serial.println("?  r 이 실측(-1.506 +-0.074) 에서 많이 벗어났다");
  }
  if (q.p == &LAMBDA) {
    Serial.print("   -> 배가시간 "); Serial.print(693.1f / LAMBDA, 0); Serial.println(" ms");
    if (LAMBDA < 3.0f || LAMBDA > 10.0f) Serial.println("?  lam 이 실측 범위(5.0~6.4) 밖이다");
  }
  if (q.p == &A_RELAX && A_RELAX >= A_TRIG)
    Serial.println("!! 펴기 게이트 >= 문턱 이면 게이트가 없는 것과 같다 (문서 46 §7)");
  if (q.p == &A_TRIG || q.p == &GAMMA || q.p == &RHO) {
    Serial.print("   -> 문턱에서의 접기량 "); Serial.print(RHO * GAMMA * A_TRIG, 2);
    Serial.print(" deg, 프로파일 ");
    Serial.print(2000.0f * sqrtf(RHO * GAMMA * A_TRIG / (ACC_UNIT * ACC_UNIT_DPS2)), 0);
    Serial.println(" ms");
  }
  if (q.p == &LOG_HZ && LOG_HZ > 100.0f)
    Serial.println("!! 115200 baud 로는 100 Hz 가 한계 — 넘으면 줄이 버려진다 (t 의 '로그버림')");
  if (q.p == &VEL_UNIT) {
    Serial.print("   -> "); Serial.print(VEL_UNIT * VEL_UNIT_DPS, 0); Serial.println(" deg/s");
  }
  if (q.p == &ACC_UNIT) {
    Serial.print("   -> "); Serial.print(ACC_UNIT * ACC_UNIT_DPS2, 0); Serial.println(" deg/s^2");
  }
}

bool setParam(int i, float v) {
  const Param& q = PARAMS[i];
  if (q.lock && running) {
    Serial.print("# 제어 중에는 못 바꾼다 (h 로 멈추고): "); Serial.println(q.name);
    return false;
  }
  if (v < q.lo || v > q.hi) {
    Serial.print("# 범위 "); Serial.print(q.lo, 3); Serial.print(" ~ "); Serial.print(q.hi, 3);
    Serial.println(" 로 잘림");
    v = (v < q.lo) ? q.lo : q.hi;
  }
  if (q.p == &FOLD_SIGN) v = (v >= 0) ? +1.0f : -1.0f;
  *q.p = v;

  if (q.derive) {
    deriveConstants();
    primed = false; dphi = dbeta = 0;        // w 가 바뀌면 추정기를 다시 채운다
    Serial.println("# w 재계산 + 추정기 리셋");
  }
  if (q.p == &VEL_UNIT && motor_ok)
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, (int)VEL_UNIT);
  if (q.p == &ACC_UNIT && motor_ok)
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, (int)ACC_UNIT);

  printParam(i);
  sanityWarn(i);
  return true;
}

void printAllParams() {
  Serial.println("==== 파라미터 (이름 값 으로 바꾼다.  예: lam 5.42) ====");
  Serial.println("-- 실측 상수 --");
  for (int i = 0; i < 5; i++) printParam(i, true);
  Serial.println("-- 제어 --");
  for (int i = 5; i < 16; i++) printParam(i, true);
  Serial.println("-- 기록·모터 --");
  for (int i = 16; i < N_PARAM; i++) printParam(i, true);
  Serial.println("한 글자 별칭: f=gam  c=trig  e=rel  o=rho  l=loghz  v=vel  a=acc");
  Serial.println("=====================================================");
}

// 현재 값을 소스에 붙여넣을 수 있는 형태로 — 전원을 꺼도 남기는 방법
void dumpSource() {
  Serial.println();
  Serial.println("// ----8<---- incremental_fold.ino 의 상수 블록에 붙여넣기 ----8<----");
  for (int i = 0; i < N_PARAM; i++) {
    const Param& q = PARAMS[i];
    // 소스의 변수명
    const char* var =
      (q.p == &P2R) ? "P2R" : (q.p == &LAMBDA) ? "LAMBDA" :
      (q.p == &R_SLOPE) ? "R_SLOPE" : (q.p == &LINE_C) ? "LINE_C" :
      (q.p == &FOLD_SIGN) ? "FOLD_SIGN" : (q.p == &GAMMA) ? "GAMMA" :
      (q.p == &RHO) ? "RHO" : (q.p == &A_TRIG) ? "A_TRIG" :
      (q.p == &A_RELAX) ? "A_RELAX" : (q.p == &RELAX_RATE) ? "RELAX_RATE" :
      (q.p == &HOLD_DEADBAND) ? "HOLD_DEADBAND" : (q.p == &DELTA_LIMIT) ? "DELTA_LIMIT" :
      (q.p == &T_REST) ? "T_REST" : (q.p == &ANG_LIMIT) ? "ANG_LIMIT" :
      (q.p == &CUE_TH) ? "CUE_TH" : (q.p == &CUE_HOLD) ? "CUE_HOLD" :
      (q.p == &LOG_HZ) ? "LOG_HZ" : (q.p == &VEL_UNIT) ? "VEL_UNIT" : "ACC_UNIT";
    Serial.print("float "); Serial.print(var);
    for (int k = strlen(var); k < 14; k++) Serial.print(' ');
    char nb[20]; fmtF(nb, *q.p, 4);
    Serial.print("= "); Serial.print(nb); Serial.print("f;");
    for (int k = strlen(nb); k < 11; k++) Serial.print(' ');
    Serial.print("// "); Serial.println(q.what);
  }
  Serial.println("// ----8<---------------------------------------------------8<----");
  Serial.println();
}

// ============================================================================
// 명령
// ============================================================================
void doZero() {
  phi_zero = as5047_raw(PHI_CS);
  ank_zero = as5047_raw(ANK_CS);
  if (motor_ok) home_tick = dxl.getPresentPosition(DXL_ID);
  hold = 0; delta_now = 0; manual_cmd = 0;
  primed = false; ank_m_primed = false;
  dphi = dbeta = 0; Ahat = 0;
  cue_since = 0; cue_on = false; digitalWrite(CUE_PIN, LOW);
  Serial.println("# ZERO — 엔코더 2개 영점 + 모터 홈(delta=0)");
  Serial.println("#   ⚠ 이 영점이 A 오프셋의 기준이다. c0=-1.11 은 문서 69 의 영점 기준이므로");
  Serial.println("#     '똑바로 선 자세'에서 누를 것. (자동트림은 폐기 — 문서 46 §8)");
  Serial.print  ("#   c0/r = "); Serial.print(A_OFFSET, 3);
  Serial.println(" deg 가 A 에 상수로 더해진다 (c0 오차 +-0.86 -> +-0.57 deg, 문서 69 §6)");
}

void printHelp() {
  Serial.println("[동작] z 영점 | g 시작 | h 정지 | x 비상정지 | k 토크복구 | u 토크해제");
  Serial.println("       y dry-run | n 잡음측정20s | j 발목중앙값 | m CSV | s 출력정지");
  Serial.println("       t 상태 | p 1회출력 | w 파라미터목록 | d 소스로 덤프 | ? 도움말");
  Serial.println("[값]   이름 값   으로 바꾼다.   예: lam 5.42 / r -1.58 / trig 0.8 / gam 11");
  Serial.println("       별칭 f=gam c=trig e=rel o=rho l=loghz v=vel a=acc");
  Serial.println("[수동] <정수> = delta 이동 [deg] — 제어 정지 중에만");
}

void handleLine(char* s) {
  while (*s == ' ' || *s == '\t') s++;
  if (*s == '\0') return;

  char c0c = *s;
  // --- 숫자 = 수동 delta 이동 ---
  if ((c0c >= '0' && c0c <= '9') || c0c == '+' || c0c == '-') {
    bool digit = false;
    for (const char* q = s; *q; q++) if (*q >= '0' && *q <= '9') { digit = true; break; }
    if (!digit) { Serial.println("# 숫자가 없음 — 무시"); return; }
    if (running) { Serial.println("# 제어 중에는 수동 이동 금지 — h 로 멈추고"); return; }
    manual_cmd = atol(s);
    if (manual_cmd >  (long)DELTA_LIMIT) manual_cmd =  (long)DELTA_LIMIT;
    if (manual_cmd < -(long)DELTA_LIMIT) manual_cmd = -(long)DELTA_LIMIT;
    if (motor_ok && !dxl.getTorqueEnableStat(DXL_ID)) torqueRestoreHere();
    hold = (float)manual_cmd;
    writeGoal(hold);
    Serial.print("# 수동 delta -> "); Serial.println(manual_cmd);
    return;
  }

  // --- 토큰(영숫자 연속) 잘라내기 ---
  char tok[12]; int tl = 0;
  while (s[tl] && ((s[tl] >= 'a' && s[tl] <= 'z') || (s[tl] >= 'A' && s[tl] <= 'Z') ||
                   (s[tl] >= '0' && s[tl] <= '9')) && tl < (int)sizeof(tok) - 1) {
    tok[tl] = (s[tl] >= 'A' && s[tl] <= 'Z') ? s[tl] + 32 : s[tl];
    tl++;
  }
  if (tl == 0) { tok[0] = *s; tok[1] = '\0'; tl = 1; }   // '?' 같은 기호 명령
  else tok[tl] = '\0';
  const char* rest = s + tl;
  while (*rest == ' ' || *rest == '\t' || *rest == '=') rest++;
  bool has = false;
  for (const char* q = rest; *q; q++) if ((*q >= '0' && *q <= '9')) { has = true; break; }
  float val = has ? atof(rest) : 0.0f;

  // --- 한 글자 별칭을 이름으로 펼친다 ---
  const char* name = tok;
  if (tl == 1) for (int i = 0; i < N_ALIAS; i++)
    if (ALIASES[i].c == tok[0]) { name = ALIASES[i].name; break; }

  // --- 파라미터? ---
  int pi = findParam(name);
  if (pi >= 0) {
    if (has) setParam(pi, val);
    else { printParam(pi); sanityWarn(pi); }
    return;
  }

  // --- 동작 명령 (한 글자) ---
  if (tl != 1) { Serial.print("# 모르는 이름: "); Serial.println(tok); printHelp(); return; }

  switch (tok[0]) {
    case 'z': doZero(); printState(); break;

    case 'g':
      if (!motor_ok && !dry_run) { Serial.println("# 모터 응답 없음 — y 로 dry-run 하거나 배선 확인"); break; }
      if (phase == FALLEN) { Serial.println("# STOP 상태 — k 로 토크 복구 후 z 부터"); break; }
      if (motor_ok && !dry_run && !dxl.getTorqueEnableStat(DXL_ID)) torqueRestoreHere();
      primed = false; dphi = dbeta = 0;
      fold_count = 0; overrun = 0; dropped = 0; cycle_max_us = 0;
      phase = IDLE; phase_t0 = millis();
      running = true;
      Serial.print("# GO"); Serial.println(dry_run ? "  (DRY-RUN — 모터 명령 안 나감)" : "");
      break;

    case 'h': running = false; Serial.println("# 제어 정지 (토크·자세 유지)"); break;

    case 'y':
      if (running) { Serial.println("# 제어 중에는 전환 금지 — h 먼저"); break; }
      dry_run = !dry_run;
      Serial.print("# dry-run "); Serial.println(dry_run ? "ON (모터 명령 안 나감)" : "OFF");
      break;

    case 'k':
      torqueRestoreHere();
      if (phase == FALLEN) phase = IDLE;
      Serial.println("# 토크 복구 (현재 자리 유지)");
      break;

    case 'u':
      running = false;
      if (motor_ok) dxl.torqueOff(DXL_ID);
      Serial.println("# 토크 해제 (k 로 복구)");
      break;

    case 'n':
      if (running) { Serial.println("# h 로 제어를 멈추고 재야 한다"); break; }
      primed = false; noiseStart();
      break;

    case 'j':
      ank_med3 = !ank_med3; ank_m_primed = false;
      Serial.print("# 발목 3점 중앙값 필터 "); Serial.print(ank_med3 ? "ON" : "OFF");
      Serial.println("   (ON 이면 발목이 5 ms 더 늦는다)");
      break;

    case 'm':
      csv_on = !csv_on;
      if (csv_on) logHeader();
      Serial.print("# CSV 로그 "); Serial.println(csv_on ? "ON" : "OFF");
      break;

    case 's':
      out_on = !out_on;
      Serial.println(out_on ? "# 출력 재개" : "# 출력 정지 (s 로 재개, 명령은 계속 먹음)");
      break;

    case 'w': printAllParams(); break;
    case 'd': dumpSource();     break;
    case 'p': printState();     break;
    case 't': printStatus();    break;
    case '?': printHelp();      break;

    default:
      Serial.print("# 모르는 명령: "); Serial.println(tok);
      printHelp();
      break;
  }
}

void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == 'x' || ch == 'X') { emergencyStop("사용자 x"); continue; }
    if (ch == '\n' || ch == '\r') {
      if (linelen) { linebuf[linelen] = '\0'; linelen = 0; handleLine(linebuf); }
      continue;
    }
    if (ch == ' ' && linelen == 0) continue;
    if (linelen < sizeof(linebuf) - 1) linebuf[linelen++] = ch;
    else { linelen = 0; Serial.println("# 입력이 너무 김 — 버림"); }
    last_rx_ms = millis();
  }
  if (linelen && (millis() - last_rx_ms) >= 200) {
    linebuf[linelen] = '\0'; linelen = 0; handleLine(linebuf);
  }
}

// ============================================================================
// setup / loop
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(PHI_CS, OUTPUT); digitalWrite(PHI_CS, HIGH);
  pinMode(ANK_CS, OUTPUT); digitalWrite(ANK_CS, HIGH);
  pinMode(CUE_PIN, OUTPUT); digitalWrite(CUE_PIN, LOW);
  SPI.begin();

  deriveConstants();

  Serial.println("=== incremental_fold — 증분접기 제어 ===");
  Serial.println("예측점이 안정모드선에서 문턱 이상 벗어나면 고정량만큼 접는다.");

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
    dxl.torqueOff(DXL_ID); delay(50);
    dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION); delay(50);
    dxl.writeControlTableItem(RETURN_DELAY_TIME,    DXL_ID, 0);   // 200 Hz 왕복을 위해
    dxl.writeControlTableItem(PROFILE_VELOCITY,     DXL_ID, (int)VEL_UNIT);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, (int)ACC_UNIT);
    home_tick = dxl.getPresentPosition(DXL_ID);
    dxl.setGoalPosition(DXL_ID, home_tick);
    delay(50);
    dxl.torqueOn(DXL_ID);
  } else {
    Serial.println("!!! 모터 응답 없음 — 엔코더만 동작 (y dry-run 으로 판정만 볼 수 있다)");
  }

  uint16_t r1 = as5047_raw(PHI_CS), r2 = as5047_raw(ANK_CS);
  Serial.print("phi raw="); Serial.print(r1);
  Serial.print("  ank raw="); Serial.println(r2);
  Serial.println("(0 또는 16383 고정이면 배선 확인)");

  printStatus();
  Serial.println();
  Serial.println("⚠ 줄 위에 올리기 전: y 로 dry-run → 손으로 기울여 Ahat 부호와");
  Serial.println("  FOLD_SIGN 방향이 맞는지 확인할 것 (헤더 주석 참조).");
  Serial.println("순서:  z → n(잡음) → y+g(부호확인) → h,y → z → g");

  t0 = millis();
  next_us = micros();
  log_next_ms = millis();
}

void loop() {
  pollSerial();

  uint32_t now = micros();
  if ((int32_t)(now - next_us) < 0) return;
  next_us += DT_US;
  if ((int32_t)(micros() - next_us) >= 0) {       // 한 주기를 통째로 놓쳤다
    overrun++;
    next_us = micros() + DT_US;
  }
  uint32_t c0 = micros();

  readState();                                     // ★ Â 는 여기서 확정 — 제어 이전 값

  // 안전
  if (running && (fabsf(phi_d) > ANG_LIMIT || fabsf(alpha_d) > ANG_LIMIT)) {
    emergencyStop("한계각 초과 (넘어짐)");
  }

  if (noise_on) {
    noiseAccum();
    if ((int32_t)(millis() - noise_end) >= 0) noiseReport();
  } else {
    updateCue();
    if (running && phase != FALLEN) controlStep();
  }

  if (csv_on && out_on) {
    uint32_t ms = millis();
    if ((int32_t)(ms - log_next_ms) >= 0) {
      log_next_ms += (uint32_t)(1000.0f / LOG_HZ);
      if ((int32_t)(ms - log_next_ms) > 100) log_next_ms = ms;   // 밀렸으면 재동기
      logLine();
    }
  } else if (!csv_on && out_on) {
    static uint32_t mon = 0;
    uint32_t ms = millis();
    if ((int32_t)(ms - mon) >= 0) { mon = ms + 250; printState(); }
  }

  uint32_t used = micros() - c0;
  if (used > cycle_max_us) cycle_max_us = used;
}
