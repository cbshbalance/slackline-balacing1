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
 *      α = ank − phi                ★문서 69 §2 정정 (2026-08-22, 사용자 육안 확인)
 *          ⚠ 문서 37·52 의 「α = ank + phi」 는 틀렸다. 이 한 줄이 이틀을 잡아먹었다:
 *            + 로 적합하면 r = +0.749 (모델과 부호 반대, R²(β̈) 0.571)
 *            − 로 적합하면 r = −1.506 (모델과 8 % 일치,  R²(β̈) 0.804)
 *            r = −1.506 은 − 규약에서 나온 값이므로 둘은 반드시 함께 간다.
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
 *     놓기신호·경보 = 온보드 USER LED 1 (pin 22).  (선택) 외부 LED = D8
 *
 * 【명령】 115200 baud. 줄바꿈 설정 무관 (200 ms 조용하면 실행)
 *
 *   [동작]
 *     z        영점 — 엔코더 2개 + 모터 홈(δ=0). ★완전히 멎은 상태에서만
 *     g        제어 시작 (GO)          |  h  제어 정지 (토크는 유지, δ 그 자리)
 *     x        비상정지 — 토크 즉시 OFF |  u  토크 해제
 *     k        토크 복구 — 1 켜보기 → 2 리부트 → 3 ★버스 전원 껐다 켜기 (선 뽑기와 같다)
 *              (다이나믹셀은 과부하로 셧다운되면 torqueOn 만으로는 절대 안 풀린다.
 *               리부트하면 home_tick 이 새로 잡히므로 반드시 z 를 다시 누를 것)
 *     y        dry-run 토글 — 판정·로그만, 모터 명령 안 나감
 *     n        Â 잡음 측정 20 s (문서 54 방법). ★가진이 있는 상태에서 (흔들리는 중)
 *     j        발목 3점 중앙값 필터 토글 (기본 OFF — n 결과 보고 정한다)
 *     m        CSV 로그 ON/OFF   |  s 출력 정지/재개  |  p 1회 출력  |  t 상태 요약
 *     b        ★전원 부하시험 — 부하가 걸린 순간의 전압을 재서 강하 원인을 가른다
 *     w        ★파라미터 전체 목록   |  d  현재 값을 소스 코드로 덤프  |  ? 도움말
 *     <정수>   δ 수동 이동 [°] — 제어 정지 중에만 먹는다
 *
 *   [값 바꾸기]  이름 값   — 값을 빼면 현재 값을 읽는다
 *     실측:  p2r 0.433 | lam 5.66 | r -1.506 | c0 -1.11 | sgn 1    ← 바꾸면 w 자동 재계산
 *     제어:  gam 10 | rho 0.95 | trig 0.6 | rel 0.3 | vrel 3 | dead 1
 *            dlim 55 | rest 60 | alim 30 | cue 0.3 | cuems 500
 *     기록:  loghz 50 | vel 250 | acc 373
 *     감시:  ewarn 300 | efail 1500        ← 0 으로 두면 그 감시를 끈다
 *     전원:  ilim 350 | vmin 10.5           ← 하드웨어 에러 '입력전압/엔코더/전기충격' 대책
 *     별칭:  f=gam  c=trig  e=rel  o=rho  l=loghz  v=vel  a=acc
 *
 *   ⚠ 값은 전원을 끄면 이 파일의 기본값으로 돌아간다.
 *     확정되면 `d` 를 쳐서 나온 블록을 아래 상수 블록에 붙여넣고 다시 컴파일할 것.
 *
 * ----------------------------------------------------------------------------
 * 【★센서 이상 표시 — 문서 66·67 의 규약 그대로】
 *
 *   화면:  값 뒤의  !     그 채널이 이상하다   (예:  f=+10.81!  k=-3.20  d=+0.00!)
 *          << ... >>      1 초에 한 줄, 원 카운트와 정지시간까지 (예: << phi 고장 raw=16383 … >>)
 *   불:    ★온보드 USER LED 1 (pin 22). 배선 필요 없다. 소리는 쓰지 않는다.
 *              READY  = 계속 켜짐
 *              해제   = 꺼짐
 *              ★고장 = 5 Hz 점멸      ← 계속 켜짐과 눈으로 구분된다
 *            부팅할 때 두 번 깜빡 → 표시등이 살아 있다는 뜻
 *            외부 D8 도 같이 움직인다 (선택 — 보드상 D8 에는 아무것도 없다)
 *   로그:  err 열         phi등급 + 4·ank등급 + 16·dxl  (0 정상 / 1 경고 / 2 고장)
 *
 *   무엇을 잡아내나
 *     엔코더  ① raw 가 0 또는 16383 고착 (MISO — 문서 52)
 *             ② raw 가 ewarn(300 ms) 동안 한 카운트도 안 바뀜 → ! ,  efail(1.5 s) → 고장
 *     모터    응답이 20 ms 넘게 없으면 실패로 세고, 3 연속이면 폴링을 끊는다
 *             ★이게 없으면 매 샘플 100 ms 타임아웃으로 루프가 3 Hz 로 떨어진다 (문서 66 §1)
 *             복구는  k  (재핑). 통신이 죽어도 로그는 계속 산다
 *
 *   고장 판정이 나면 — 제어 중이면 즉시 정지(토크 OFF)하고, g 로 다시 시작되지 않는다.
 *     ★센서가 죽은 채로 접으면 값이 틀린 게 아니라 접을 근거가 없다. 아무 쪽으로나 접는다.
 *
 *   ⚠ 오경보가 걱정될 때: ②는 '정말로 완전히 안 움직이는' 상황에서도 걸린다. 줄 위에서는
 *     잡음만으로 늘 1 LSB(0.022°)가 흔들리므로 안 걸리지만, 책상에 가만히 두고 오래 보면
 *     뜰 수 있다. 그때는  efail 0  (자동정지만 끄기) 또는  ewarn 0  (전부 끄기).
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
 *   6. z → g        줄 위에서 놓기. READY(LED 계속 켜짐) 뜨면 놓는다.
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
float LINE_C   = 0.0f;    // [c0 ] 실측② 안정모드선 절편 [deg] ±0.86 ⚠미확정 (문서 69 §6)
float FOLD_SIGN = +1.0f;    // [sgn] 접기 방향 ±1 — ⚠ 헤더의 바닥 시험으로 확인할 것

// ---- 제어 ----
float GAMMA       = 11.0f;  // [gam ] ★접기 이득 γ — 줄 위 노브. Δδ = ρ·γ·Â
float RHO         = 0.95f;  // [rho ] 감쇠계수 ρ
float A_TRIG      = 0.6f;   // [trig] ★트리거 문턱 [deg]
float A_RELAX     = 0.3f;   // [rel ] 펴기 게이트 [deg]
float RELAX_RATE  = 3.0f;   // [vrel] 펴기 속도 [deg/s]
float HOLD_DEADBAND = 1.0f; // [dead] 이보다 작은 유지각은 안 편다 [deg]
float DELTA_LIMIT = 55.0f;  // [dlim] 힙 기구한계 [deg] (정지 중에만)
float T_REST      = 60.0f;  // [rest] REST 대기 [ms]
float ANG_LIMIT   = 30.0f;  // [alim] |φ| 또는 |α| 가 넘으면 토크 OFF [deg]
float CUE_TH      = 1.0f;   // [cue ] 놓기신호 문턱 [deg] (문서 50)
float CUE_HOLD    = 500.0f; // [cuems] 놓기신호 유지시간 [ms]

// ---- 기록·모터 ----
float LOG_HZ      = 50.0f;  // [loghz] CSV 로그 [Hz]  (115200 baud 에서 100 이 한계)
float VEL_UNIT    = 250.0f; // [vel ] PROFILE_VELOCITY     [unit] ≈344 deg/s
float ACC_UNIT    = 373.0f; // [acc ] PROFILE_ACCELERATION [unit] ≈8000 deg/s^2

// ---- 센서 감시 (문서 66 §6 · 문서 67 §6) ----
float ENC_WARN_MS = 300.0f;  // [ewarn] 원값이 이만큼 안 바뀌면 ! 경고   (0 = 감시 끔)
float ENC_FAIL_MS = 1500.0f; // [efail] 이만큼이면 고장 — 경보 + 제어정지 (0 = 감시 끔)

// ---- 전원 감시 (하드웨어 에러 '입력전압·엔코더·전기충격' 대책) ----
//   XM430 은 입력전압이 9.5 V 아래로 떨어지면 Input Voltage Error 를 낸다.
//   '전기충격(Electrical Shock)' 비트는 ROBOTIS 정의상 '모터를 돌릴 전력 부족' 도 포함한다.
//   무부하 12.4 V 인데 접기 순간 무너진다면 원인은 전류 급증이다 → 두 방향으로 막는다.
float CUR_LIMIT = 350.0f;    // [ilim] 전류 제한 [unit] 1unit=2.69 mA, 최대 1193(3.2A)
float VOLT_MIN  = 10.5f;     // [vmin] 이 아래로 떨어지면 스스로 정지 [V] (0 = 감시 끔)

// ---- 접기 완료 판정 ----
//   ★도착을 기다리면 안 된다. 상체 무게 때문에 서보가 명령각에 1~3.4 deg 못 미친다 (문서 64 §4-3).
//     도착 판정 0.5 deg 로는 참이 되는 일이 없어서 매번 타임아웃(600 ms)을 다 기다렸고,
//     그러면 한 사이클이 660 ms = 5.4 배가시간이 되어 '증분'접기가 아니게 된다.
//   ⇒ 기다릴 것은 도착이 아니라 '프로파일이 끝나는 시간' 이고, 그건 접기량에서 계산된다.
float FOLD_TOL  = 2.0f;                   // [ftol ] 도착 판정 [deg] — 처짐보다 커야 한다
float FOLD_TMAX = 300.0f;                 // [ftmax] FOLD 상한 [ms] — 계산된 프로파일 시간의 상한

// ★한 번에 접는 양의 상한 [deg]
//   문서 46: "감마 비례에서는 잡음이 트리거뿐 아니라 '접기량' 에도 그대로 들어간다."
//   Ahat 는 속도항을 포함하므로, 손 흔들림이나 스파이크 한 샘플이 40 deg 를 커밋할 수 있다.
//   이 상한은 Ahat 을 왜곡하지 않는다 — 구동기 쪽 제한이다.
float STEP_LIMIT = 20.0f;                 // [dstep] 접기 1회 상한 [deg]

// 모터 통신 감시 (문서 66 §6 보강 1 — 실패해도 폴링을 계속하면 루프가 죽는다)
const uint32_t DXL_SLOW_US = 20000;       // 이보다 오래 걸린 읽기 = 무응답 (라이브러리 타임아웃 100 ms)
const uint8_t  DXL_FAIL_N  = 3;           // 연속 이만큼 실패하면 폴링을 끊는다

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
const uint8_t CUE_PIN = 8;          // (선택) 외부 LED — D8 에는 보드상 아무것도 없다

// ---- OpenCR 온보드 표시등 — 배선이 필요 없다 ----
//   USER LED 22~25, ★active-LOW (LOW 가 켜짐)
//   ⚠ 내장 LED(13)는 우리 배선의 SPI SCLK 와 겹치므로 쓰지 않는다
//   ⚠ 소리는 쓰지 않는다 — 표시는 전부 눈으로 본다 (LED + 화면)
#ifndef BDPIN_LED_USER_1
  #define BDPIN_LED_USER_1 22
#endif
const uint8_t LED_PIN = BDPIN_LED_USER_1;
const uint8_t LED_ON  = LOW;        // active-LOW
const uint8_t LED_OFF = HIGH;

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

// ---- 센서·통신 감시 상태 ----
//   등급 0 = 정상,  1 = 경고(!),  2 = 고장(경보 + 제어정지)
uint8_t  phi_err = 0, ank_err = 0;
uint16_t phi_raw = 0, ank_raw = 0;          // 마지막 원값 (진단줄에 찍는다)
uint32_t phi_chg_ms = 0, ank_chg_ms = 0;    // 원값이 마지막으로 바뀐 시각
bool     dxl_err = false;                   // 통신 끊겨 폴링을 멈춘 상태
uint8_t  dxl_fail_n = 0;
uint32_t err_next_ms = 0;                   // 진단줄 1 Hz 솎기
uint32_t fold_wait_ms = 0;                  // 이번 접기의 FOLD 예정 시간 (접기량에서 계산)
bool     delta_primed = false;              // 첫 읽기는 점프 판정에서 제외
uint32_t delta_jump = 0;                    // 버린 점프 횟수 (통신 잡음의 척도)
uint32_t dlim_warn_ms = 0;
float    v_now = 0, v_min = 0, i_peak = 0;  // 전원 감시
uint32_t pw_next = 0; uint8_t pw_phase = 0;
uint32_t vlow_warn_ms = 0;

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
// 표시등 — 온보드 USER LED 1 (active-LOW). 외부 D8 도 같이 움직인다
// ============================================================================
void cueLamp(bool on) {
  digitalWrite(LED_PIN, on ? LED_ON : LED_OFF);
  digitalWrite(CUE_PIN, on ? HIGH : LOW);
}

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

  // ★ 문서 66 §6 보강 1 — 무응답인데 계속 물어보면 매 샘플 100 ms 타임아웃을 기다려
  //   제어루프가 3 Hz 로 떨어진다. 연속 실패하면 폴링을 끊고 로그만 살린다.
  uint32_t t_us = micros();
  float t = dxl.getPresentPosition(DXL_ID);
  //   실패는 두 모양으로 온다: ① 타임아웃(느림) ② 곧바로 0.0 반환(빠름)
  //   ②는 확장위치 모드에서 원위치가 정확히 0 tick 일 확률이 없으므로 실패로 본다
  bool slow = ((uint32_t)(micros() - t_us) > DXL_SLOW_US);
  if (slow || t == 0.0f) {
    if (dxl_fail_n < 255) dxl_fail_n++;
    if (dxl_fail_n >= DXL_FAIL_N && !dxl_err) {
      dxl_err  = true;
      motor_ok = false;                                    // 이후 폴링 안 함
      Serial.print("<< dxl 통신실패 (");
      Serial.print(slow ? "타임아웃" : "0 반환");
      Serial.println(") — 폴링 중단. 배터리/RS-485 확인. k 로 재시도 >>");
    }
    return delta_now;                                      // 직전 값 유지
  }
  dxl_fail_n = 0;

  float d = MOTOR_DIR * (t - home_tick) / TICK_PER_DEG;

  // 통신 사고는 '한 사이클에 물리적으로 불가능한 점프' 로 나타난다.
  //   200 Hz 에서 서보 최고속도(344 deg/s)로도 5 ms 에 1.7 deg 밖에 못 간다.
  // ★예전에는 |d| 자체를 120 deg 로 잘랐는데, 그러면 토크를 풀고 손으로 관절을 크게
  //   돌렸을 때 표시가 통째로 얼어붙는다 (범위 밖이라 모든 읽기가 거부되므로 손을 멈춰도
  //   안 풀린다). 엔코더는 토크와 무관하게 잘 읽히는데 코드가 버리고 있었던 것이다.
  if (delta_primed && fabsf(d - delta_now) > 30.0f) { delta_jump++; return delta_now; }
  delta_primed = true;

  // 한계를 벗어난 것은 '버릴 값' 이 아니라 '알려야 할 사실' 이다
  if (fabsf(d) > DELTA_LIMIT + 15.0f && (int32_t)(millis() - dlim_warn_ms) >= 0) {
    dlim_warn_ms = millis() + 2000;
    Serial.print("?  delta 가 기구한계 밖이다: "); Serial.print(d, 1);
    Serial.println(" deg  — 영점이 틀렸거나 관절이 한계를 넘었다 (z 로 다시 잡을 것)");
  }
  return d;
}

// 하드웨어 에러 비트를 사람 말로
void printHwError(int32_t e) {
  Serial.print("   원인: ");
  if (e & 0x01) Serial.print("입력전압 ");
  if (e & 0x04) Serial.print("과열 ");
  if (e & 0x08) Serial.print("엔코더 ");
  if (e & 0x10) Serial.print("전기충격 ");
  if (e & 0x20) Serial.print("★과부하 ");
  Serial.println();
}

// 위치를 믿을 수 있을 때까지 읽는다. 리부트 직후에는 실패가 잦고,
// 실패값 0 을 목표로 써 버리면 서보가 tick 0 으로 달려 그 자리에서 다시 과부하가 난다.
bool readPositionTrusted(float* out) {
  float a = 0, b = 0;
  for (int i = 0; i < 6; i++) {
    a = dxl.getPresentPosition(DXL_ID);
    delay(40);
    b = dxl.getPresentPosition(DXL_ID);
    if (a != 0.0f && b != 0.0f && fabsf(a - b) < 60.0f) { *out = b; return true; }
    delay(60);
  }
  return false;
}

// 토크가 실제로 걸렸는지 확인한다 (명령을 보낸 것과 걸린 것은 다르다)
bool torqueIsOn() {
  for (int i = 0; i < 3; i++) { if (dxl.getTorqueEnableStat(DXL_ID)) return true; delay(60); }
  return false;
}

// 재시작(리부트·전원차단) 뒤 공통 브링업. 성공하면 true.
//   RAM 값(프로파일)은 날아가므로 다시 넣는다. OPERATING_MODE·CURRENT_LIMIT 은 EEPROM 이라 남는다.
bool bringUpAfterRestart() {
  motor_ok = false; dxl_fail_n = 0;
  for (int i = 0; i < 12 && !motor_ok; i++) {
    if (dxl.ping(DXL_ID)) motor_ok = true; else delay(200);
  }
  if (!motor_ok) return false;
  dxl_err = false;

  dxl.torqueOff(DXL_ID); delay(60);
  dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION); delay(60);
  dxl.writeControlTableItem(RETURN_DELAY_TIME,    DXL_ID, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY,     DXL_ID, (int)VEL_UNIT);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, (int)ACC_UNIT);

  float pos;
  if (!readPositionTrusted(&pos)) {
    Serial.println("<< 살아났는데 위치를 못 읽는다 — 토크는 켜지 않는다 (튈 수 있다) >>");
    return false;
  }
  home_tick = pos;                                 // ★영점이 새로 잡힌다
  hold = 0; delta_now = 0; delta_primed = false;
  dxl.setGoalPosition(DXL_ID, home_tick); delay(60);
  dxl.torqueOn(DXL_ID); delay(150);
  return torqueIsOn();
}

// ★토크 복구 — 진단이 아니라 '결과' 를 보고 단계를 올린다.
//   1 그냥 켜보기 → 2 리부트 명령 → 3 ★버스 전원 차단(선을 뽑는 것과 같다) → 4 사람에게.
//   3 단계가 있는 이유: 리부트 명령으로 안 지워지는 래치 상태가 있고, 그럴 때
//   지금까지는 사람이 케이블을 뽑았다 꽂아야 했다. OpenCR 은 그걸 핀 하나로 할 수 있다.
void motorRecover() {
  if (!motor_ok) { Serial.println("<< 모터 무응답 — 배터리·RS-485 부터 >>"); return; }

  int32_t e = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, DXL_ID);
  Serial.print("# 복구 시작. hw_error=0x"); Serial.print(e < 0 ? 0 : e, HEX);
  Serial.print("  torque="); Serial.println(torqueIsOn() ? "ON" : "OFF");
  if (e > 0) printHwError(e);

  // --- 1단계: 지금 자리를 목표로 잡고 그냥 켜본다 ---
  float pos;
  if (readPositionTrusted(&pos)) {
    dxl.setGoalPosition(DXL_ID, pos);
    dxl.torqueOn(DXL_ID);
    delay(150);
    if (torqueIsOn()) {
      hold = delta_now = MOTOR_DIR * (pos - home_tick) / TICK_PER_DEG;
      Serial.println("# 1단계 성공 — 그냥 켜져서 복구됨 (영점 그대로)");
      return;
    }
  }
  Serial.println("# 1단계 실패 (에러 플래그가 토크를 막고 있다) -> 리부트 명령");

  // --- 2단계: 리부트 명령 ---
  dxl.reboot(DXL_ID, 500);
  delay(1500);
  if (bringUpAfterRestart()) {
    Serial.println("# 2단계 성공 — 리부트로 복구됨");
    Serial.println("#   ★영점이 새로 잡혔다. z 를 다시 누를 것");
    return;
  }
  Serial.println("# 2단계 실패 -> ★버스 전원을 껐다 켠다");

  // --- 3단계: 다이나믹셀 버스 전원 차단 = 케이블을 뽑았다 꽂는 것과 같다 ---
#ifdef BDPIN_DXL_PWR_EN
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  Serial.println("#   전원 OFF ...");
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
  delay(1500);                                     // 커패시터가 완전히 빠질 때까지
  Serial.println("#   전원 ON  ...");
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  delay(2000);                                     // 서보가 부팅할 시간

  if (bringUpAfterRestart()) {
    Serial.println("# 3단계 성공 — 전원 재인가로 복구됨 (케이블 만질 필요 없음)");
    Serial.println("#   ★영점이 새로 잡혔다. z 를 다시 누를 것");
    return;
  }
#else
  Serial.println("<< 이 보드에서는 버스 전원 제어를 못 한다 >>");
#endif

  // --- 4단계: 여기까지 왔으면 사람이 봐야 한다 ---
  e = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, DXL_ID);
  Serial.print("<< 전원 재인가로도 안 된다. hw_error=0x"); Serial.println(e < 0 ? 0 : e, HEX);
  if (e > 0) printHwError(e);
  Serial.println("   ★이제는 접점이다 — DXL 4핀 커넥터를 뽑아 접점을 확인하고 다시 꽂을 것");
  Serial.println("     (전원은 이미 껐다 켰으므로, 그래도 안 되면 남은 것은 접촉 아니면 서보 고장)");
}

// 통신이 끊긴 뒤 다시 살려 보기 (k 명령·g 명령에서 부른다)
bool motorRetry() {
  dxl_fail_n = 0;
  for (int i = 0; i < 3; i++) {
    if (dxl.ping(DXL_ID)) {
      motor_ok = true; dxl_err = false;
      Serial.println("# 모터 응답 복구");
      return true;
    }
    delay(100);
  }
  Serial.println("<< dxl 여전히 무응답 >>");
  return false;
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
  hold    = delta_now;              // ★남은 목표가 다음 시작에서 도약이 되지 않게

  // ★에러가 '토크를 내리기 전' 에 있었는지 '내린 뒤' 에 생겼는지를 남긴다.
  //   넘어진 뒤 토크를 끊으면 관절이 자유가 되어 무게에 휘둘린다 →
  //   되감김 역기전력(과전압) 이나 케이블 순간단선으로 하드웨어 에러가 '새로' 생길 수 있다.
  //   이 두 줄이 원인과 결과의 방향을 갈라 준다.
  int32_t e0 = -1, v0 = -1;
  if (motor_ok) {
    e0 = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, DXL_ID);
    v0 = dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, DXL_ID);
    dxl.torqueOff(DXL_ID);
  }
  linelen = 0;
  Serial.print(">>> STOP (torque off) : "); Serial.println(why);

  if (motor_ok) {
    Serial.print("    [정지 직전] hw_error=0x"); Serial.print(e0 < 0 ? 0 : e0, HEX);
    Serial.print("  전압 "); Serial.print(v0 > 0 ? v0 / 10.0f : 0.0f, 1); Serial.println(" V");
    delay(400);                                     // 되감김이 끝날 때까지 기다렸다 다시 본다
    int32_t e1 = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, DXL_ID);
    int32_t v1 = dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, DXL_ID);
    Serial.print("    [0.4 s 뒤 ] hw_error=0x"); Serial.print(e1 < 0 ? 0 : e1, HEX);
    Serial.print("  전압 "); Serial.print(v1 > 0 ? v1 / 10.0f : 0.0f, 1); Serial.println(" V");

    if (e1 > 0 && e0 <= 0) {
      Serial.println("    ★에러는 토크를 내린 뒤에 생겼다.");
      Serial.println("      = 되감김(역기전력·과전압) 또는 케이블 순간단선.  전원이 약한 것이 아니다.");
      Serial.println("      대책: alim 을 낮춰 더 일찍 멈추기 / 케이블 스트레인릴리프 / 넘어질 때 받치기");
      printHwError(e1);
    } else if (e0 > 0) {
      Serial.println("    ★에러가 넘어지기 전부터 있었다 — 넘어짐의 원인일 수 있다.");
      printHwError(e0);
    }
    if (v1 > 160) Serial.println("    ★전압이 16 V 를 넘었다 = 되감김 과전압이 확실하다");
  }
  Serial.println("    k 토크복구 → z 영점 → g 재시작");
}

// ============================================================================
// 전원 — 전류를 덜 뽑고, 전압이 무너지기 전에 멈춘다
// ============================================================================

// CURRENT_LIMIT 은 EEPROM 이다. 값이 다를 때만 쓴다 (수명 아끼기). 쓰려면 토크를 내려야 한다.
void applyCurrentLimit() {
  if (!motor_ok) return;
  int32_t want = (int32_t)CUR_LIMIT;
  int32_t cur  = dxl.readControlTableItem(CURRENT_LIMIT, DXL_ID);
  if (cur == want) return;

  bool was_on = dxl.getTorqueEnableStat(DXL_ID);
  dxl.torqueOff(DXL_ID); delay(60);
  dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID, want); delay(80);
  int32_t got = dxl.readControlTableItem(CURRENT_LIMIT, DXL_ID);
  Serial.print("# 전류 제한 = "); Serial.print(got);
  Serial.print(" unit ("); Serial.print(got * 2.69f / 1000.0f, 2); Serial.println(" A)");
  if (got != want) Serial.println("!! 전류 제한이 적용되지 않았다 — 토크를 내리고 다시 시도할 것");
  if (was_on) {
    float pos = dxl.getPresentPosition(DXL_ID);
    if (pos != 0.0f) dxl.setGoalPosition(DXL_ID, pos);
    dxl.torqueOn(DXL_ID);
  }
}

// 전압·전류를 번갈아 10 Hz 로 본다. 한 번에 하나만 읽어 루프 부담을 나눈다.
void powerWatch() {
  if (!motor_ok || dxl_err) return;
  uint32_t ms = millis();
  if ((int32_t)(ms - pw_next) < 0) return;
  pw_next = ms + 50;

  pw_phase ^= 1;
  if (pw_phase) {
    int32_t v = dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, DXL_ID);
    if (v <= 0) return;
    v_now = v / 10.0f;
    if (v_min <= 0.0f || v_now < v_min) v_min = v_now;

    if (VOLT_MIN > 0.0f && v_now < VOLT_MIN) {
      if (running) {
        emergencyStop("입력전압 강하 — 배터리/전원배선");
        Serial.print("    최저 "); Serial.print(v_min, 1);
        Serial.print(" V (기준 "); Serial.print(VOLT_MIN, 1);
        Serial.println(" V). XM430 은 9.5 V 아래에서 에러를 낸다");
      } else if ((int32_t)(ms - vlow_warn_ms) >= 0) {
        vlow_warn_ms = ms + 3000;
        Serial.print("!! 입력전압 "); Serial.print(v_now, 1); Serial.println(" V — 배터리 확인");
      }
    }
  } else {
    int32_t i = dxl.readControlTableItem(PRESENT_CURRENT, DXL_ID);
    float a = fabsf((float)(int16_t)i) * 2.69f / 1000.0f;
    if (a < 10.0f && a > i_peak) i_peak = a;      // 말도 안 되는 값은 버린다
  }
}

// ============================================================================
// 전원 부하시험 (b) — '부하가 걸린 순간의 전압' 하나를 재려고 만든 것
//   배터리·배선이 멀쩡한데도 입력전압 에러가 나면, 강하가 어디서 생기는지를 갈라야 한다.
//   PRESENT_INPUT_VOLTAGE 는 ★모터 안에서 잰 값이므로, 이 시험의 최저값이
//   전원 경로(커넥터·케이블·OpenCR DXL 포트) 의 상태를 그대로 말해 준다.
// ============================================================================
void powerStressTest() {
  if (running)  { Serial.println("# h 로 제어를 멈추고 하세요"); return; }
  if (dry_run)  { Serial.println("# dry-run 중에는 못 한다 — y 로 해제"); return; }
  if (!motor_ok){ Serial.println("# 모터 무응답 — k 부터"); return; }

  float amp = 10.0f;
  if (amp > DELTA_LIMIT * 0.5f) amp = DELTA_LIMIT * 0.5f;

  Serial.println("==== 전원 부하시험 시작 ====");
  Serial.print  ("  delta 를 +-"); Serial.print(amp, 0);
  Serial.println(" deg 로 4 회 왕복한다. ★로봇을 손으로 잡고 있을 것 (약 3 초)");

  if (!dxl.getTorqueEnableStat(DXL_ID)) torqueRestoreHere();
  int32_t e0 = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, DXL_ID);
  int32_t v0 = dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, DXL_ID);

  float vmn = 99.0f, imx = 0.0f;
  uint32_t n = 0;
  bool tripped = false;

  for (int k = 0; k < 4 && !tripped; k++) {
    for (int dir = 0; dir < 2 && !tripped; dir++) {
      float tgt = dir ? amp : -amp;
      hold = tgt; writeGoal(tgt);
      uint32_t t_end = millis() + 400;
      while ((int32_t)(millis() - t_end) < 0) {
        int32_t v = dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, DXL_ID);
        if (v > 0) { float vv = v / 10.0f; if (vv < vmn) vmn = vv; n++; }
        int32_t i = dxl.readControlTableItem(PRESENT_CURRENT, DXL_ID);
        float a = fabsf((float)(int16_t)i) * 2.69f / 1000.0f;
        if (a < 10.0f && a > imx) imx = a;
      }
      if (dxl.readControlTableItem(HARDWARE_ERROR_STATUS, DXL_ID) > e0) {
        tripped = true;
        Serial.println("  !! 시험 중에 에러가 났다 — 여기서 멈춘다");
      }
    }
  }
  hold = 0; writeGoal(0); delay(300);
  int32_t e1 = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, DXL_ID);

  float vidle = (v0 > 0) ? v0 / 10.0f : 0.0f;
  float drop  = (vidle > 0 && vmn < 90.0f) ? (vidle - vmn) : 0.0f;

  Serial.println("---- 결과 ----");
  Serial.print("  무부하 전압   "); Serial.print(vidle, 1); Serial.println(" V");
  Serial.print("  ★최저 전압   "); Serial.print(vmn, 1);
  Serial.print(" V   (강하 ");      Serial.print(drop, 1);
  Serial.print(" V, 표본 ");        Serial.print(n); Serial.println(")");
  Serial.print("  최대 전류     ");  Serial.print(imx, 2); Serial.print(" A");
  if (imx > CUR_LIMIT * 2.69f / 1000.0f * 0.9f)
    Serial.print("   <- ilim 에 붙었다 (제한이 걸리는 중)");
  Serial.println();
  Serial.print("  hw_error  전 0x"); Serial.print(e0 < 0 ? 0 : e0, HEX);
  Serial.print("  ->  후 0x");       Serial.println(e1 < 0 ? 0 : e1, HEX);
  if (e1 > 0) printHwError(e1);

  Serial.println("---- 판정 ----");
  if (vmn > 90.0f) {
    Serial.println("  전압을 못 읽었다 — 통신부터 확인 (k)");
  } else if (drop > 1.5f) {
    Serial.println("  ★강하가 크다 (>1.5 V). 배터리가 멀쩡하다면 원인은 ★전원 경로다:");
    Serial.println("     DXL 4핀 커넥터 접점 → 케이블 → OpenCR 의 DXL 포트 → 전원 커넥터");
    Serial.println("     ※ 케이블을 자주 뽑았다 꽂았다면 그 접점이 가장 의심스럽다");
    Serial.println("     완화: acc 250 (가속 낮추기) / ilim 낮추기");
  } else if (e1 > e0) {
    Serial.println("  ★전압은 안 무너졌는데 에러가 났다 = 전원 부족이 아니다.");
    Serial.println("     남는 후보: 접지·EMI, DXL 케이블 신호선, 또는 서보 내부 고장");
    Serial.println("     → Dynamixel Wizard 2.0 으로 모터만 따로 물려 재현되는지 볼 것");
  } else {
    Serial.println("  이 조건에서는 정상. 접기(더 큰 각·더 빠른 가속)에서만 난다면");
    Serial.println("  amp 를 키운 셈인 실제 접기량으로 다시 볼 것 — dstep 을 낮춰 완화 가능");
  }
  Serial.println("=============================");
}

// ============================================================================
// 상태 추정 — 한 사이클
// ============================================================================
// ---- 엔코더 한 채널 등급 매기기 ----
//   판정 근거 두 가지 (문서 52·67 에서 실제로 나온 고장 모양 그대로)
//     ① 레일 고착: raw = 0 또는 16383 (0x3FFF) — MISO 문제
//     ② 정지 고착: raw 가 한 카운트도 안 바뀐 채 오래 감
//   ★ ②는 '진짜로 완전히 안 움직일 때' 도 걸린다. 줄 위에서는 잡음만으로도 1 LSB(0.022°)
//     는 늘 흔들리므로 문제없지만, 책상에 가만히 두면 뜰 수 있다. 그때는  efail 0  으로 끈다.
uint8_t encGrade(uint16_t raw, uint16_t* last, uint32_t* chg_ms, uint32_t now) {
  bool rail = (raw == 0 || raw == 0x3FFF);
  if (raw != *last && !rail) *chg_ms = now;
  *last = raw;

  if (ENC_WARN_MS <= 0.0f) return 0;                     // 감시 끔
  uint32_t still = now - *chg_ms;
  // ⚠ raw 가 0/16383 을 '지나가는' 것은 정상이다 (엔코더 절대영점이 동작점 근처면 그렇다).
  //   고장은 거기 '멈춰 있는' 것이므로 레일도 시간으로 판정한다 — 대신 더 빨리 확정한다.
  if (rail && still >= (uint32_t)ENC_WARN_MS) return 2;
  if (ENC_FAIL_MS > 0.0f && still >= (uint32_t)ENC_FAIL_MS) return 2;
  if (still >= (uint32_t)ENC_WARN_MS) return 1;
  return 0;
}

bool sensorFault() { return (phi_err >= 2 || ank_err >= 2); }   // 제어를 멈춰야 하는 등급
bool anyFault()    { return (phi_err >= 2 || ank_err >= 2 || dxl_err); }

void readState() {
  uint16_t raw_p = as5047_raw(PHI_CS);
  uint16_t raw_a = as5047_raw(ANK_CS);
  uint32_t now_ms = millis();
  phi_err = encGrade(raw_p, &phi_raw, &phi_chg_ms, now_ms);
  ank_err = encGrade(raw_a, &ank_raw, &ank_chg_ms, now_ms);

  float phi = rawToDeg(raw_p, phi_zero);
  float ank = rawToDeg(raw_a, ank_zero);

  if (ank_med3) {                                 // 발목만 3점 중앙값 (문서 54: 0.223→0.138°)
    if (!ank_m_primed) { ank_m[0] = ank_m[1] = ank_m[2] = ank; ank_m_primed = true; }
    ank_m[ank_mi] = ank;
    ank_mi = (ank_mi + 1) % 3;
    ank = med3(ank_m[0], ank_m[1], ank_m[2]);
  }

  delta_now = readDelta();

  phi_d   = phi;
  ank_d   = ank;
  alpha_d = ank - phi;                            // ★문서 69 §2 정정 (문서 37·52 의 + 는 틀렸다)
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
// 사다리꼴/삼각 프로파일이 deg 만큼 움직이는 데 걸리는 시간 [ms]
// ============================================================================
float profileMs(float deg) {
  deg = fabsf(deg);
  float acc = ACC_UNIT * ACC_UNIT_DPS2;          // deg/s^2
  float vmx = VEL_UNIT * VEL_UNIT_DPS;           // deg/s
  if (acc <= 1.0f || vmx <= 1.0f || deg <= 0.0f) return 20.0f;
  float t = (sqrtf(deg * acc) <= vmx) ? (2.0f * sqrtf(deg / acc))    // 삼각
                                      : (deg / vmx + vmx / acc);     // 사다리꼴
  return t * 1000.0f;
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
        if (fabsf(step) > STEP_LIMIT) {                 // 잡음 한 샘플이 크게 커밋하지 못하게
          static uint32_t sl_next = 0;
          if ((int32_t)(now - sl_next) >= 0) {
            sl_next = now + 1000;
            Serial.print("?  접기량 상한 "); Serial.print(STEP_LIMIT, 0);
            Serial.print(" deg 로 잘림 (요청 "); Serial.print(step, 1);
            Serial.print(", Ahat="); Serial.print(Ahat, 2); Serial.println(")");
          }
          step = (step > 0) ? STEP_LIMIT : -STEP_LIMIT;
        }
        hold += step;
        if (hold >  DELTA_LIMIT) hold =  DELTA_LIMIT;
        if (hold < -DELTA_LIMIT) hold = -DELTA_LIMIT;
        writeGoal(hold);
        fold_count++;
        // ★이번 접기가 실제로 걸릴 시간 (여유 30% + 통신 지연 20 ms). 도착을 기다리지 않는다.
        float fw = profileMs(step) * 1.3f + 20.0f;
        if (fw > FOLD_TMAX) fw = FOLD_TMAX;
        fold_wait_ms = (uint32_t)fw;
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
          fabsf(delta_now - hold) < FOLD_TOL ||         // 일찍 도착하면 일찍 나간다
          (uint32_t)(now - phase_t0) >= fold_wait_ms) { // ★보통은 이쪽 — 프로파일 예정 시간
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

  // ★ 경보가 놓기신호보다 우선한다 (문서 67 §6 — 화면을 안 보고 있으면 ! 가 안 보인다)
  //   READY = 계속 켜짐 / 고장 = 5 Hz 점멸.  눈으로도 귀로도 구분된다.
  if (anyFault()) {
    if (cue_on) { cue_on = false; Serial.println("# READY 취소 — 센서 이상"); }
    cue_since = 0;
    cueLamp(((now / 100) % 2) != 0);                  // 5 Hz 점멸 — READY(계속 켜짐)와 구분된다
    return;
  }
  if (noise_on) return;                      // 잡음 측정 중에는 경보만 본다

  // ★히스테리시스 — 켜지는 문턱과 꺼지는 문턱이 다르다.
  //   같은 값이면 Â 가 문턱을 오갈 때마다 신호가 깜빡여서 '지금!' 을 못 읽는다.
  //   꺼짐 문턱은 접기 트리거(A_TRIG)에 맞춘다 — "실제로 접어야 할 만큼 벗어나면 그때 꺼진다".
  float cue_off = (A_TRIG > CUE_TH) ? A_TRIG : (CUE_TH * 1.5f);
  float a = fabsf(Ahat);

  if (cue_on) {
    if (a > cue_off) {                       // 켜져 있을 때는 이 문턱까지 버틴다
      cue_on = false; cue_since = 0;
      cueLamp(false);
      Serial.print("# ready 해제  Ahat="); Serial.println(Ahat, 3);
    }
  } else {
    if (a < CUE_TH) {
      if (cue_since == 0) cue_since = now;
      if ((uint32_t)(now - cue_since) >= (uint32_t)CUE_HOLD) {
        cue_on = true;
        cueLamp(true);
        Serial.print("# READY  Ahat="); Serial.println(Ahat, 3);
      }
    } else {
      cue_since = 0;                         // 아직 안 켜졌을 때는 한 번만 넘어도 다시 센다
    }
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

// ---- 로그 백프레셔에 관하여 ----
//   원래는 Serial.availableForWrite() 로 송신버퍼 여유를 보고 자리가 없으면 로그 한 줄을
//   버렸다. OpenCR 의 USBSerial 에는 이 함수가 없어서 그대로는 컴파일이 안 된다.
//   (템플릿 SFINAE 로 우회하면 Arduino IDE 의 자동 프로토타입 생성이 template 줄을
//    빠뜨려 "'s' was not declared in this scope" 가 난다 — 그래서 그 방법도 못 쓴다.)
//   ⇒ 검사를 없앴다. 대신 '애초에 밀리지 않게' 하는 것이 방어책이다:
//      loghz 50 (기본값) 이하 유지.  t 의 overrun / cycle_max 가 커지면 loghz 를 더 낮출 것.
//   ⚠ 그래서 t 의 '로그버림' 은 이 보드에서 항상 0 이다 (기능이 없는 것이지 안 버린 것이 아님).

const char* phaseName() {
  switch (phase) { case IDLE: return "IDLE"; case FOLD: return "FOLD";
                   case REST: return "REST"; default: return "STOP"; }
}

// 이상이 있는 값 뒤에 ! 를 붙인다 (문서 66·67 의 표시 규약 그대로)
void bang(uint8_t g) { if (g) Serial.print('!'); }

void printState() {
  // ★dry-run 은 제어가 멎어 있을 때도 보여야 한다 — 안 보이면 왜 안 움직이는지 알 수가 없다
  Serial.print(running ? "RUN " : "off ");
  Serial.print(phaseName());
  if (dry_run) Serial.print(" [DRY]");
  Serial.print(" | A="); ps(Ahat, 3);   bang(phi_err | ank_err);
  Serial.print(" | b=");   ps(beta_d, 2);
  Serial.print(" f=");     ps(phi_d, 2);   bang(phi_err);
  Serial.print(" k=");     ps(ank_d, 2);   bang(ank_err);
  Serial.print(" | db=");  ps(dbeta, 1);
  Serial.print(" df=");    ps(dphi, 1);
  Serial.print(" | hold="); ps(hold, 2);
  Serial.print(" d=");      ps(delta_now, 2);  bang(dxl_err ? 1 : 0);
  Serial.print(" | n=");    Serial.print(fold_count);
  if (cue_on) Serial.print("  READY");
  Serial.println();
}

// 이상 진단줄 — 1 초에 한 번만. 원 카운트를 같이 찍는다 (문서 67 §1 형식)
void reportFault() {
  if (!(phi_err || ank_err || dxl_err)) return;
  uint32_t now = millis();
  if ((int32_t)(now - err_next_ms) < 0) return;
  err_next_ms = now + 1000;

  if (phi_err) {
    Serial.print("<< phi ");  Serial.print(phi_err >= 2 ? "고장" : "의심");
    Serial.print(" raw="); Serial.print(phi_raw);
    Serial.print(" 정지 "); Serial.print(now - phi_chg_ms); Serial.println(" ms >>");
  }
  if (ank_err) {
    Serial.print("<< ank ");  Serial.print(ank_err >= 2 ? "고장" : "의심");
    Serial.print(" raw="); Serial.print(ank_raw);
    Serial.print(" 정지 "); Serial.print(now - ank_chg_ms); Serial.println(" ms >>");
  }
  if (dxl_err) Serial.println("<< dxl 통신실패 — 폴링 중단 상태 (k 로 재시도) >>");
  if (phi_raw == 0 || phi_raw == 0x3FFF || ank_raw == 0 || ank_raw == 0x3FFF)
    Serial.println("<< raw 가 0/16383 고착 = MISO 배선·커넥터 (문서 52) >>");
}

void logHeader() {
  Serial.println("# D,t_ms,phi,ank,alpha,beta,dphi,dbeta,Ahat,hold,del_now,phase,cue,err");
  Serial.println("# Ahat 는 control_step 이전 값이다 (문서 46 §9)");
  Serial.println("# err = phi등급 + 4*ank등급 + 16*dxl   (등급 0 정상 / 1 경고 / 2 고장)");
  Serial.println("#   ★err 열이 0 이 아닌 구간은 버릴 것. 화면에는 ! 와 << >> 로도 나온다");
}

void logLine() {
  // (송신버퍼 여유 검사는 위 주석의 이유로 제거 — loghz 를 낮게 유지하는 것이 방어책)
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
  Serial.print(cue_on ? 1 : 0); Serial.print(',');
  Serial.println((int)phi_err + 4 * (int)ank_err + (dxl_err ? 16 : 0));
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
  Serial.print("  ftol="); Serial.print(FOLD_TOL, 1);
  Serial.print("  ftmax="); Serial.print(FOLD_TMAX, 0);
  Serial.print(" ms  -> 문턱 접기의 한 사이클 = FOLD ");
  { float fw = profileMs(RHO*GAMMA*A_TRIG)*1.3f + 20.0f;
    if (fw > FOLD_TMAX) fw = FOLD_TMAX;
    Serial.print(fw, 0); Serial.print(" + REST "); Serial.print(T_REST, 0);
    Serial.print(" = "); Serial.print(fw + T_REST, 0); Serial.print(" ms  (배가시간 ");
    Serial.print(693.1f/LAMBDA, 0); Serial.println(" ms)"); }
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
  Serial.print("  로그버림="); Serial.print(dropped); Serial.print("(미지원)");
  Serial.print("  delta점프버림="); Serial.print(delta_jump);
  Serial.print("  cycle_max="); Serial.print(cycle_max_us); Serial.println(" us");
  if (motor_ok) {
    int32_t e = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, DXL_ID);
    int32_t tp = dxl.readControlTableItem(PRESENT_TEMPERATURE,   DXL_ID);
    int32_t v  = dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, DXL_ID);
    Serial.println("-- 모터 건강 --");
    Serial.print("  hw_error=0x"); Serial.print(e < 0 ? 0 : e, HEX);
    if (e > 0) { Serial.println("  ★에러 — k 로 리부트 복구"); printHwError(e); }
    else Serial.println("  (0 = 정상)");
    Serial.print("  온도 "); Serial.print(tp); Serial.print(" C   전압 ");
    Serial.print(v / 10.0f, 1); Serial.print(" V (무부하)");
    Serial.print("   ★실행 중 최저 "); Serial.print(v_min, 1);
    Serial.print(" V   최대전류 "); Serial.print(i_peak, 2); Serial.println(" A");
    Serial.print("  전류제한 ilim="); Serial.print(CUR_LIMIT, 0);
    Serial.print(" ("); Serial.print(CUR_LIMIT * 2.69f / 1000.0f, 2);
    Serial.print(" A)   저전압정지 vmin="); Serial.print(VOLT_MIN, 1); Serial.println(" V");
    if (v_min > 0 && v_min < 11.0f)
      Serial.println("  ★강하가 크다 — 배터리 충전 / 전원선 굵기·커넥터 / acc 낮추기");
  }

  Serial.println("-- 센서 감시 --");
  Serial.print("  phi: raw="); Serial.print(phi_raw);
  Serial.print(" 정지 ");      Serial.print(millis() - phi_chg_ms); Serial.print(" ms  ");
  Serial.println(phi_err >= 2 ? "★고장" : (phi_err ? "의심(!)" : "정상"));
  Serial.print("  ank: raw="); Serial.print(ank_raw);
  Serial.print(" 정지 ");      Serial.print(millis() - ank_chg_ms); Serial.print(" ms  ");
  Serial.println(ank_err >= 2 ? "★고장" : (ank_err ? "의심(!)" : "정상"));
  Serial.print("  dxl: ");     Serial.println(dxl_err ? "★통신실패 (폴링 중단, k 로 재시도)" : "정상");
  Serial.print("  기준: ewarn="); Serial.print(ENC_WARN_MS, 0);
  Serial.print(" ms  efail=");    Serial.print(ENC_FAIL_MS, 0);
  Serial.println(" ms  (0 이면 감시 끔)");
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
  {"ftol", &FOLD_TOL,     0.1f,  10.0f,   false, false, "deg",   "접기 도착 판정 (처짐보다 크게)"},
  {"ftmax",&FOLD_TMAX,   20.0f, 800.0f,   false, false, "ms",    "★FOLD 상한 — 사이클 길이를 정한다"},
  {"dstep",&STEP_LIMIT,   1.0f,  55.0f,   false, false, "deg",   "★접기 1회 상한 (잡음 방어)"},
  // 기록·모터
  {"loghz",&LOG_HZ,       1.0f, 200.0f,   false, false, "Hz",    "CSV 로그 주기"},
  {"vel",  &VEL_UNIT,     1.0f,1023.0f,   false, false, "unit",  "프로파일 속도"},
  {"acc",  &ACC_UNIT,     1.0f,32767.0f,  false, false, "unit",  "프로파일 가속"},
  // 센서 감시
  {"ewarn",&ENC_WARN_MS,  0.0f,10000.0f,  false, false, "ms",    "엔코더 ! 경고 기준 (0=끔)"},
  {"efail",&ENC_FAIL_MS,  0.0f,10000.0f,  false, false, "ms",    "엔코더 고장 기준 (0=끔)"},
  {"ilim", &CUR_LIMIT,   50.0f, 1193.0f,  false, true,  "unit",  "★전류 제한 1u=2.69mA"},
  {"vmin", &VOLT_MIN,     0.0f,   14.0f,  false, false, "V",     "★저전압 자동정지 (0=끔)"},
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
  if (q.p == &ENC_WARN_MS || q.p == &ENC_FAIL_MS) {
    if (ENC_WARN_MS <= 0.0f) Serial.println("!! ewarn 0 — 센서 감시 전체가 꺼진다 (! 도 경보도 없음)");
    else if (ENC_FAIL_MS <= 0.0f) Serial.println("?  efail 0 — ! 경고만 뜨고 자동정지·경보는 안 한다");
    else if (ENC_FAIL_MS < ENC_WARN_MS) Serial.println("!! efail < ewarn — 경고 없이 곧바로 고장 처리된다");
  }
  if (q.p == &A_RELAX && A_RELAX >= A_TRIG)
    Serial.println("!! 펴기 게이트 >= 문턱 이면 게이트가 없는 것과 같다 (문서 46 §7)");
  if (q.p == &FOLD_TMAX || q.p == &T_REST || q.p == &GAMMA || q.p == &A_TRIG || q.p == &RHO) {
    float fw = profileMs(RHO*GAMMA*A_TRIG)*1.3f + 20.0f;
    if (fw > FOLD_TMAX) fw = FOLD_TMAX;
    float cyc = fw + T_REST, t2 = 693.1f/LAMBDA;
    Serial.print("   -> 한 사이클 "); Serial.print(cyc, 0);
    Serial.print(" ms = 배가시간의 "); Serial.print(cyc/t2, 2); Serial.println(" 배");
    if (cyc > 1.5f*t2) Serial.println("!! 사이클이 배가시간의 1.5배를 넘는다 — 증분접기가 성립하지 않는다");
  }
  if (q.p == &STEP_LIMIT || q.p == &GAMMA || q.p == &RHO) {
    float a_sat = STEP_LIMIT / (RHO * GAMMA);
    Serial.print("   -> Ahat "); Serial.print(a_sat, 2);
    Serial.print(" deg 까지는 온전히 접고, 그 위는 포화한다 (회복 천장 2.8~4.0)");
    Serial.println();
  }
  if (q.p == &A_TRIG || q.p == &GAMMA || q.p == &RHO) {
    Serial.print("   -> 문턱에서의 접기량 "); Serial.print(RHO * GAMMA * A_TRIG, 2);
    Serial.print(" deg, 프로파일 ");
    Serial.print(2000.0f * sqrtf(RHO * GAMMA * A_TRIG / (ACC_UNIT * ACC_UNIT_DPS2)), 0);
    Serial.println(" ms");
  }
  if (q.p == &CUR_LIMIT) {
    Serial.print("   -> "); Serial.print(CUR_LIMIT * 2.69f / 1000.0f, 2);
    Serial.println(" A 로 제한. 낮출수록 전압 강하가 줄지만 접기 토크도 준다");
  }
  if (q.p == &ACC_UNIT) {
    Serial.println("   ※ 전압 강하의 주범은 가속도다. 에러가 계속 나면 acc 를 먼저 낮출 것");
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
  if (q.p == &CUR_LIMIT) applyCurrentLimit();

  printParam(i);
  sanityWarn(i);
  return true;
}

void printAllParams() {
  Serial.println("==== 파라미터 (이름 값 으로 바꾼다.  예: lam 5.42) ====");
  Serial.println("-- 실측 상수 --");
  for (int i = 0; i < 5; i++) printParam(i, true);
  Serial.println("-- 제어 --");
  for (int i = 5; i < 19; i++) printParam(i, true);
  Serial.println("-- 기록·모터·감시 --");
  for (int i = 19; i < N_PARAM; i++) printParam(i, true);
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
      (q.p == &FOLD_TOL) ? "FOLD_TOL" : (q.p == &FOLD_TMAX) ? "FOLD_TMAX" :
      (q.p == &STEP_LIMIT) ? "STEP_LIMIT" :
      (q.p == &LOG_HZ) ? "LOG_HZ" : (q.p == &VEL_UNIT) ? "VEL_UNIT" :
      (q.p == &ACC_UNIT) ? "ACC_UNIT" : (q.p == &ENC_WARN_MS) ? "ENC_WARN_MS" : "ENC_FAIL_MS";
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
  if (motor_ok) {
    home_tick = dxl.getPresentPosition(DXL_ID);
    // ★읽기가 실패하면 라이브러리가 0 을 준다. 그 0 을 영점으로 삼으면 delta 가 통째로 거짓이 된다
    if (home_tick == 0.0f) {
      Serial.println("!! ★영점을 잡는 순간 모터가 안 읽혔다 (home_tick=0)");
      Serial.println("   이 영점은 무효다 — delta 표시가 전부 거짓이 된다.");
      Serial.println("   배터리·RS-485 를 고치고 k 로 복구한 뒤 z 를 다시 누를 것");
    }
  }
  hold = 0; delta_now = 0; manual_cmd = 0;
  primed = false; ank_m_primed = false; delta_primed = false;
  dphi = dbeta = 0; Ahat = 0;
  cue_since = 0; cue_on = false; cueLamp(false);
  phi_chg_ms = ank_chg_ms = millis();            // 감시 타이머도 여기서 다시 센다
  phi_err = ank_err = 0;
  Serial.println("# ZERO — 엔코더 2개 영점 + 모터 홈(delta=0)");
  Serial.println("#   ⚠ 이 영점이 A 오프셋의 기준이다. c0=-1.11 은 문서 69 의 영점 기준이므로");
  Serial.println("#     '똑바로 선 자세'에서 누를 것. (자동트림은 폐기 — 문서 46 §8)");
  Serial.print  ("#   c0/r = "); Serial.print(A_OFFSET, 3);
  Serial.println(" deg 가 A 에 상수로 더해진다 (c0 오차 +-0.86 -> +-0.57 deg, 문서 69 §6)");
}

void printHelp() {
  Serial.println("[동작] z 영점 | g 시작 | h 정지 | x 비상정지 | k 토크복구(에러시 리부트) | u 해제");
  Serial.println("       y dry-run | n 잡음측정20s | j 발목중앙값 | m CSV | s 출력정지");
  Serial.println("       t 상태 | p 1회출력 | b 전원부하시험 | w 목록 | d 덤프 | ? 도움말");
  Serial.println("[값]   이름 값   으로 바꾼다.   예: lam 5.42 / r -1.58 / trig 0.8 / gam 11");
  Serial.println("       별칭 f=gam c=trig e=rel o=rho l=loghz v=vel a=acc");
  Serial.println("[이상] 값 뒤 ! = 그 채널 이상 | << >> 진단줄 | CUE 5Hz 점멸 = 고장");
  Serial.println("       감시 끄기: efail 0 (자동정지만) / ewarn 0 (전부)");
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
    // ★조용히 무시하지 않는다 — 명령이 안 나가는 이유를 반드시 말해 준다
    if (dry_run) {
      Serial.print("!! dry-run ON — 모터 명령이 안 나갔다 (요청 ");
      Serial.print(manual_cmd); Serial.println("). y 로 해제하고 다시 칠 것");
      return;
    }
    if (!motor_ok) {
      Serial.println("!! 모터 무응답 — 명령이 안 나갔다. k 로 재시도 / 배터리·RS-485 확인");
      return;
    }
    if (!dxl.getTorqueEnableStat(DXL_ID)) torqueRestoreHere();
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
      if (dxl_err && !dry_run) motorRetry();               // 끊겼던 통신 한 번 살려 본다
      if (!motor_ok && !dry_run) { Serial.println("# 모터 응답 없음 — y 로 dry-run 하거나 배선 확인"); break; }
      if (phase == FALLEN) { Serial.println("# STOP 상태 — k 로 토크 복구 후 z 부터"); break; }
      if (sensorFault()) { Serial.println("# ★센서 고장 상태에서는 시작하지 않는다 — t 로 확인, 고친 뒤 z"); break; }
      if (motor_ok && !dry_run && !dxl.getTorqueEnableStat(DXL_ID)) torqueRestoreHere();

      // ★시작할 때 hold 를 '지금 실제 각도' 로 맞춘다.
      //   이걸 안 하면 지난 실행에서 쌓인 hold(±55 일 수 있다)가 그대로 남아,
      //   g 를 누르는 순간 서보가 거기까지 전속력으로 달려 기구한계에 처박힌다 → 과부하 셧다운.
      //   판정은 아무 잘못이 없는데 모터만 튀는, 찾기 어려운 고장이었다.
      if (motor_ok && !dry_run) delta_now = readDelta();
      if (fabsf(hold - delta_now) > 1.0f) {
        Serial.print("# 시작 정렬: hold "); Serial.print(hold, 1);
        Serial.print(" -> "); Serial.print(delta_now, 1);
        Serial.println(" deg (지금 자리에서 시작한다 — 튀지 않게)");
      }
      hold = delta_now;
      if (fabsf(hold) > HOLD_DEADBAND)
        Serial.println("?  접힌 자세에서 시작한다. 똑바로 편 자세에서 z 부터 하는 편이 낫다");

      primed = false; dphi = dbeta = 0;
      fold_count = 0; overrun = 0; dropped = 0; cycle_max_us = 0; delta_jump = 0;
      v_min = 0; i_peak = 0;
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
      if (running) { running = false; Serial.println("# (제어 정지하고 복구한다)"); }
      if (dxl_err || !motor_ok) motorRetry();
      motorRecover();                             // 1단계 켜보기 → 안 되면 2단계 리부트
      if (phase == FALLEN) phase = IDLE;
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

    case 'b': powerStressTest(); break;

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
  pinMode(CUE_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
#ifdef BDPIN_DXL_PWR_EN
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);   // DXL 버스 전원 ON (k 3단계에서 여기를 껐다 켠다)
  delay(300);
#endif
  cueLamp(false);
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
    applyCurrentLimit();                         // ★전압 강하 대책 — 전류부터 묶는다
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

  // 부팅 확인 — USER LED 1 이 두 번 깜빡이면 표시등이 살아 있다는 뜻
  for (int i = 0; i < 2; i++) { cueLamp(true); delay(120); cueLamp(false); delay(120); }

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
  powerWatch();                      // 전압·전류 10 Hz 감시 (무너지기 전에 멈춘다)

  // ★ 토크가 몰래 풀렸는지 1 Hz 로 확인한다 — 과부하 셧다운은 조용히 온다
  if (running && !dry_run && motor_ok) {
    static uint32_t tq_next = 0; static uint8_t tq_off_n = 0;
    uint32_t ms = millis();
    if ((int32_t)(ms - tq_next) >= 0) {
      tq_next = ms + 1000;
      if (!dxl.getTorqueEnableStat(DXL_ID)) {
        if (++tq_off_n >= 2) { tq_off_n = 0; emergencyStop("모터 토크가 풀렸다 (과부하 셧다운? k 로 확인)"); }
      } else tq_off_n = 0;
    }
  }

  // ★ 센서가 죽은 채로 접으면 엉뚱한 쪽으로 접는다 — 값이 아니라 근거가 없어진 것이다
  if (running && sensorFault()) {
    emergencyStop(phi_err >= 2 ? "phi 엔코더 이상" : "발목 엔코더 이상");
  }

  updateCue();                       // 경보는 잡음 측정 중에도 돈다 (안은 문서 67 §6)
  reportFault();                     // << ... >> 진단줄, 1 Hz

  if (noise_on) {
    noiseAccum();
    if ((int32_t)(millis() - noise_end) >= 0) noiseReport();
  } else {
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
