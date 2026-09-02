/*
 * hangcal_fold.ino — 매달림 영점 + 자유비행 실측(λ·φ_eq·c₀) + 증분접기 (2026-08-26)
 * ============================================================================
 *
 *  【무엇인가】
 *      v21 시뮬(sim_engine._fwe_async)의 증분접기 동작을 그대로 옮긴 실기 펌웨어에,
 *      ① 매달림 영점 절차(문서 66)  ② 자유비행 측정 모드(문서 67 놓기 프로토콜)를 얹었다.
 *      이번 세션의 측정 대상은 셋뿐이다 — ★λ(발산율), ★φ_eq(평형점), ★c₀(절편).
 *      r·P2R·w_v 는 문서 70·73 확정값으로 고정한다 (영점과 무관한 값들이다).
 *
 *  【판정량 Â — finale8 방식 (문서 73): 속도항은 동정값 직접】
 *      Â = (−1/r)·φ + 1·β + vg·wf·φ̇ + vg·wb·β̇ + c₀/r
 *      λ 는 Â 에 들어가지 않는다 — 배가시간 표시와 사이클 판정에만 쓴다.
 *      α = ank − φ (문서 70 §2 정정),  β = α + P2R·δ
 *      φ̇, β̇ : 25 ms 기저차분 → τ≈28 ms EMA (실기 파이프라인, 한 묶음 — 문서 45)
 *
 *  【상태기계 — v21 시뮬 _fwe_async 그대로】
 *      IDLE  ─ |Â| > trig ?  예 → hold += sgn·ρ·γ·Â (dstep 상한, ±dlim 클립) → FOLD
 *              아니오 → |Â| < rel 이고 |hold| > dead 이면 hold 를 vrel[°/s]로 0 쪽 복귀
 *      FOLD  ─ 프로파일 예정시간(접기량에서 계산×1.3+20ms) 또는 도착 → REST
 *              (시뮬은 '도착'으로 나가지만 실기는 처짐 1~3.4° 때문에 시간으로 나간다 — 문서 64 §4-3)
 *      REST  ─ rest[ms] 대기 → IDLE
 *
 * ----------------------------------------------------------------------------
 *  【★매달림 영점 — 문서 66 절차 그대로】
 *
 *      로봇을 줄에 완전히 늘어뜨린다 (서보 토크 OFF: u).  φ=0, β=180 인 자세다.
 *      중력이 CoM 축을 연직에 정렬하므로, 눈대중 세우기(±1°)와 달리 이것이 '맞는 기준'이다.
 *
 *      절차 (건마찰 데드밴드 대응 — 양방향 정착 평균):
 *        1. 왼쪽에서 접근시켜 완전히 멎으면  →  z   (1차 기록)
 *        2. 오른쪽에서 접근시켜 완전히 멎으면 →  z   (2차 기록 → 중간값이 영점이 된다)
 *      z 를 세 번째 누르면 처음부터 다시 시작한다.
 *
 *      ★직립 변환은 upFlip*() 한 곳뿐이다 (문서 66 §4 — ±180° 래핑이 사는 자리).
 *        실측 확정(08-26): 도는 것은 φ 채널 (fphi 1 기본). 발목은 안 돈다 (fank 0).
 *        영점 직후(매달림): f=±180 / k=0 / a=±180.   세운 뒤(직립): f·k·a 전부 0 근처.
 *        틀리게 보이는 채널이 있으면 fphi/fank 토글 — 재영점 불필요 (플립은 영점 뒤에 적용).
 *
 *  【★자유비행 측정 모드 — q 로 전환】
 *
 *      제어(접기)를 하지 않고 δ 를 그 자리에 물고만 있는다. 놓기 한 번이 한 시행.
 *      시행마다 자동으로: 놓기점 (φ, ank, β, Â) 기록 + |ψ| 2°/4°/8° 통과시각 → λ 즉석추정.
 *      ψ = φ − phieq.  잡기(|φ|>fcatch)면 시행 종료, R 줄 출력. 다시 조용해지면 다음 시행.
 *
 *      ┌ 이 데이터에서 무엇이 나오나 (오프라인 정본 분석은 기존 스크립트) ┐
 *      │ λ    : ln|ψ| 대 t 적합 (즉석값은 배가시간 2→4°, 4→8°)              │
 *      │ φ_eq : +방향 λ 와 −방향 λ 가 같아지는 phieq (t 요약이 힌트를 준다)  │
 *      │        ★매달림 영점이 맞다면 0 근처여야 한다 — 그 확인이 목적이다  │
 *      │ c₀   : 놓기점 (φ, β) 경계선의 절편 (문서 70 §4-2 경로②)            │
 *      │        ★같은 φ 에서 ank 를 ±3° 흩어서 놓아야 절편이 잡힌다 (§8①)   │
 *      └──────────────────────────────────────────────┘
 *      ⚠ 방향을 섞어서 놓을 것 (문서 70 §5 — 4회 전부 같은 방향이 φ_eq 를 숨겼다)
 *      ⚠ 접기 시행과 λ 시행은 같은 시행에서 동시에 안 된다 — 제어가 켜져 있으면
 *        Â=0.6° 에서 접어버려 적합 밴드(|ψ| 2~9°)에 들어가기 전에 자유비행이 끝난다.
 *
 *  【놓기 프로토콜 — 문서 67 (바 없이 손으로)】
 *      손을 스토퍼처럼 '한쪽 면만' 대고 있다가 y축(줄 방향)으로 뺀다. 잡지 않는다.
 *      순서: 머리쪽 손 먼저 → (제어 모드) 첫 접기 알림이 나오면 크랭크쪽 손 즉시 빼기.
 *      첫 접기 알림 = 화면 ★줄 + LED 3연점멸.
 *
 * ----------------------------------------------------------------------------
 * ⚠⚠ 줄 위에 올리기 전 FOLD_SIGN 바닥 시험 (finale6 헤더와 동일) ⚠⚠
 *   y (dry-run) → g → 손으로 기울여 Ahat 부호 확인 → 화면 dcmd 방향으로 손 접기 →
 *   기울어진 반대쪽으로 되돌아와야 한다. 아니면 sgn -1.
 *
 * 【배선】 문서 17·52 그대로
 *     φ CS = D10, 발목 CS = D9, SCLK = D13, MISO = D12, MOSI = D11
 *     모터 XM430-W210-R, RS-485, ID 1, Serial3 / DIR = 84. ★배터리 필수
 *     표시등 = 온보드 USER LED 1 (pin 22). (선택) 외부 LED = D8
 *
 * 【명령】 115200 baud. 줄바꿈 무관 (200 ms 조용하면 실행)
 *   [동작]
 *     z        ★매달림 영점 — 두 번 눌러 완성 (1차: 왼쪽 정착, 2차: 오른쪽 정착)
 *     q        ★모드 순환: 제어 → 자유비행(접지 않음) → 단일접기(1회 접고 기록)
 *     g / h    시작 / 정지        x 비상정지 (토크 OFF)
 *     k        토크 복구 (켜보기→리부트→버스 전원 재인가)     u 토크 해제
 *     y        dry-run 토글       n  Â 잡음 20 s       j  발목 3점 중앙값
 *     m CSV    s 출력정지        p 1회 출력            t 상태 (측정 모드면 λ 요약 포함)
 *     b        전원 부하시험      w 목록      d 소스 덤프      ? 도움말
 *     <정수>   δ 수동 이동 [°] — 제어 정지 중에만
 *   [값 바꾸기]  이름 값  (값 생략 = 읽기)
 *     실측(고정): p2r 0.4285 | r -1.506 | wf 0.1945 | wb 0.3049 | vg 1.0 | lam 5.66
 *     측정 대상: c0 0 | phieq 0        ← 이번 세션이 채울 값
 *     영점/변환: fphi 0|1 (직립에서 f 가 ±180 이면 1)
 *     제어: gam 10 | rho 0.95 | trig 0.6 | rel 0.3 | vrel 3 | dead 1 | dstep 20
 *           dlim 55 | rest 60 | alim 30 | armms 200 | cue 0.3 | cuems 500
 *     측정: reldet 1.0 (놓기 감지 문턱) | fcatch 8.5 (시행 종료 |φ|)
 *     기록: loghz 100 | vel 250 | acc 373     감시: ewarn 300 | efail 1500
 *     전원: ilim 350 | vmin 10.5
 *
 * 【CSV】 D 행은 finale 계열과 열이 같다 — 기존 fold_logger.py·해석 스크립트 그대로 먹는다.
 *     D,t_ms,phi,ank,alpha,beta,dphi,dbeta,Ahat,hold,del_now,phase,cue,err
 *     R,trial,dir,phi0,ank0,beta0,A0,t2_ms,t4_ms,t8_ms,lam24,lam48     ← 시행 요약
 *     phase: 0 IDLE / 1 FOLD / 2 REST / 3 STOP / 4 대기(측정) / 5 발산중 / 6 시행종료
 *
 * 【브링업 순서】
 *   1. u → 매달아 완전히 멎힘 → z → 반대쪽에서 멎힘 → z     (영점 완성)
 *   2. 세운다 → p 로 k·a 가 0 근처인지 확인 (f 가 ±180 이면 fphi 1)
 *   3. n   잡음 20 s (가진 상태에서)
 *   4. y → g   dry-run 부호 확인 → h → y
 *   5. m → q → g   자유비행 30~40회 (방향 섞기, 같은 φ 에서 ank ±3° 흩기)
 *   6. (오프라인 회귀 → c0·phieq 갱신) → q → g   접기 시행
 * ============================================================================
 */
#include <SPI.h>
#include <Dynamixel2Arduino.h>
#include <math.h>

// ============================================================================
// ★★★ 조정 가능한 값 — 전부 런타임에 바꿀 수 있다 (d 로 덤프해 여기 붙여넣기) ★★★
// ============================================================================

// ---- 실측 상수 (문서 70·73 정본 — 이번 세션에서는 고정) ----
float P2R      = 0.4285f;   // [p2r] 실측① 기울기 ±0.0011 (문서 70 §3 — 0.433 은 구판)
float R_SLOPE  = -1.506f;   // [r  ] 실측② 안정모드선 기울기 ±0.074 (3경로 확인 — 고정)
float WV_PHI   = 0.1945f;   // [wf ] 동정 w 의 φ̇ 성분 (문서 73)
float WV_BETA  = 0.3049f;   // [wb ] 동정 w 의 β̇ 성분 (문서 73)
float VGAIN    = 1.0f;      // [vg ] 속도항 배율. 실측 범위 밖(0.88~1.45)은 튜닝이다
float LAMBDA   = 5.66f;     // [lam] ★표시·사이클 판정 전용. Â 에 안 들어간다 (문서 73 §2)

// ---- ★이번 세션의 측정 대상 ----
float LINE_C   = 0.0f;      // [c0   ] 절편 [deg]. 매달림 영점 후 재실측 — 그때까지 0
float PHI_EQ   = 0.0f;      // [phieq] 평형점 [deg]. ψ=φ−phieq. 새 영점이 맞다면 0 근처 (문서 70 §8③)

// ---- 영점/직립 변환 ----
//   ★2026-08-26 실측 확정: 매달림→직립에서 도는 것은 φ 채널이다 (연속값 f=+193.5 관측).
//   로봇 전체가 줄 축(크랭크 피벗)을 중심으로 돌므로 φ 가 180° 를 보고,
//   발목은 몸통-크랭크 상대각이라 그대로다. 문서 66 의 α+180 은 φ 플립으로 자동 성립.
float FLIP_PHI = 1.0f;      // [fphi] 1 = φ 직립 변환(+180°). 직립에서 f 가 ±180 으로 보이면 조정
float FLIP_ANK = 0.0f;      // [fank] 1 = 발목도 +180. 직립에서 k 가 ±180 으로 보일 때만 1

// ---- 제어 (v21 시뮬 _fwe_async 의 노브들) ----
float FOLD_SIGN = -1.0f;    // [sgn ] 접기 방향 ±1 — 바닥 시험으로 확인
float GAMMA     = 12.0f;    // [gam ] 접기 이득 γ (시뮬 고원 8~15)
float RHO       = 0.95f;    // [rho ] 감쇠계수 ρ
float A_TRIG    = 0.6f;     // [trig] 트리거 문턱 [deg]
float A_RELAX   = 0.3f;     // [rel ] 복귀 게이트 [deg] (시뮬의 0.5·trig 에 해당)
float RELAX_RATE = 3.0f;    // [vrel] 저속 복귀 [deg/s] (시뮬 상한 ~6)
float HOLD_DEADBAND = 1.0f; // [dead] 이보다 작은 유지각은 안 되돌린다 [deg]
float STEP_LIMIT = 20.0f;   // [dstep] 접기 1회 상한 [deg] (시뮬 step_cap)
float DELTA_LIMIT = 55.0f;  // [dlim] 힙 기구한계 [deg]
float T_REST    = 60.0f;    // [rest] REST 대기 [ms]
float ANG_LIMIT = 30.0f;    // [alim] |φ| 또는 |α| 초과 시 토크 OFF [deg]
float ARM_MS    = 200.0f;   // [armms] g 직후 트리거 유예 [ms] (문서 76 §3② — g 는 놓기 전에 누른다)
float CUE_TH    = 0.3f;     // [cue ] 놓기신호 문턱 [deg] (문서 50)
float CUE_HOLD  = 500.0f;   // [cuems] 놓기신호 유지시간 [ms]

// ---- 자유비행/단일접기 측정 ----
float REL_DET  = 1.0f;      // [reldet] |ψ| 가 이 값을 밖으로 넘으면 '놓았다' [deg]
float F_CATCH  = 8.5f;      // [fcatch] |φ| 가 이 값을 넘으면 시행 종료(잡기) [deg] (문서 70: >8° 폐기)
float LOCK_MS  = 120.0f;    // [lock  ] 단일접기 A⁺ 확정 창 [ms] (문서 76 §9 — 스윕 중 절대 바꾸지 말 것)

// ---- 기록·모터 ----
float LOG_HZ    = 100.0f;   // [loghz] CSV [Hz] — 측정 모드는 100 권장 (실측②③이 100 Hz 였다)
float VEL_UNIT  = 250.0f;   // [vel ] PROFILE_VELOCITY [unit] ≈344 deg/s
float ACC_UNIT  = 373.0f;   // [acc ] PROFILE_ACCELERATION [unit] ≈8000 deg/s²

// ---- 감시 ----
float ENC_WARN_MS = 300.0f;  // [ewarn] (0=끔)
float ENC_FAIL_MS = 1500.0f; // [efail] (0=끔)
float CUR_LIMIT = 350.0f;    // [ilim] 1unit=2.69 mA
float VOLT_MIN  = 10.5f;     // [vmin] (0=끔)

// ---- 접기 완료 판정 (문서 64 §4-3 — 도착이 아니라 시간을 기다린다) ----
float FOLD_TOL  = 2.0f;      // [ftol ] 도착 판정 [deg] — 처짐보다 크게
float FOLD_TMAX = 300.0f;    // [ftmax] FOLD 상한 [ms]

const uint32_t DXL_SLOW_US = 20000;
const uint8_t  DXL_FAIL_N  = 3;

// ============================================================================
// 실기 파이프라인 — 셋은 한 묶음 (문서 45). 런타임 변경 불가.
// ============================================================================
const uint32_t DT_US   = 5000;      // 200 Hz
const float    DT_S    = 0.005f;
const int      VEL_N   = 5;         // 25 ms 기저차분
const float    EMA_A   = 0.15f;     // τ ≈ 28 ms

// ============================================================================
// 하드웨어
// ============================================================================
#define DXL_SERIAL   Serial3
#define DXL_DIR_PIN  84
const uint8_t DXL_ID  = 1;
const uint8_t PHI_CS  = 10;
const uint8_t ANK_CS  = 9;
const uint8_t CUE_PIN = 8;
#ifndef BDPIN_LED_USER_1
  #define BDPIN_LED_USER_1 22
#endif
const uint8_t LED_PIN = BDPIN_LED_USER_1;
const uint8_t LED_ON  = LOW;        // active-LOW
const uint8_t LED_OFF = HIGH;

const int   MOTOR_DIR     = +1;     // 문서 37 확정
const float TICK_PER_DEG  = 4096.0f / 360.0f;
const float VEL_UNIT_DPS  = 1.374f;
const float ACC_UNIT_DPS2 = 21.4577f;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// ============================================================================
// 유도 상수 — finale8 방식 (문서 73): 속도항은 동정값 직접, λ 는 안 들어간다
// ============================================================================
float W_PHI, W_BETA, W_PHIDOT, W_BETADOT, A_OFFSET;

void deriveConstants() {
  W_PHI     = -1.0f / R_SLOPE;
  W_BETA    =  1.0f;
  W_PHIDOT  =  VGAIN * WV_PHI;
  W_BETADOT =  VGAIN * WV_BETA;
  A_OFFSET  =  LINE_C / R_SLOPE;
}

// ============================================================================
// 상태
// ============================================================================
enum Phase { IDLE = 0, FOLD = 1, REST = 2, FALLEN = 3 };
Phase phase = IDLE;

// 자유비행 시행 상태 (측정 모드)
enum TState { T_QUIET = 4, T_DIVERGE = 5, T_DONE = 6 };
TState tstate = T_QUIET;

bool running   = false;
// q 가 순환하는 세 모드
//   0 제어(증분접기)  1 자유비행(접지 않음)  2 ★단일접기(한 번 접고 넘어지는 것을 기록)
// 단일접기 모드의 한 시행: 놓기 → 문턱에서 접기 1회(F행: A⁻·Δδ·A⁺) → 이후는 순수
// 자유비행 발산(R행: λ) → 잡기. λ 밴드(2~9°)는 접기 뒤에 지나가므로 λ 가 같이 나온다.
// ⚠ 접힌 δ 가 평형을 옮기므로 phieq 판정만은 자유비행(모드 1) 시행으로 볼 것.
uint8_t run_mode = 0;
bool dry_run   = false;
bool motor_ok  = false;
bool out_on    = true;
bool csv_on    = false;
bool ank_med3  = false;

uint32_t dxl_baud = 0;
float    home_tick = 0;
uint16_t phi_zero = 0, ank_zero = 0;

// ---- 매달림 영점 2단계 ----
uint8_t  zero_stage = 0;            // 0 = 대기, 1 = 1차 기록됨
uint16_t z1_phi = 0, z1_ank = 0;
float    z1_tick = 0;

float hold      = 0.0f;
float delta_now = 0.0f;
float phi_d = 0, ank_d = 0, alpha_d = 0, beta_d = 0;
float dphi = 0, dbeta = 0;
float Ahat = 0;
// ★연속화(unwrap) 앵커 — v2 수정.
//   매달림 자세는 α·β = ±180, 즉 wrap180 의 불연속점 '위' 다. 잡음 1 LSB(0.022°)로도
//   β 가 +179.99 ↔ −179.99 로 넘나들면 차분이 ±360° 점프 → 속도항이 수천 °/s →
//   Â 가 ±1200 까지 튄다 (실측 재현됨). 그래서 판정·속도용 φ·β 는 '연속값'으로 들고 가고,
//   래핑은 표시(ank_d)에만 남긴다.
float prev_phi_w = 0, prev_beta_w = 0;

float phi_hist[VEL_N + 1], beta_hist[VEL_N + 1];
int   hist_i = 0;
bool  primed = false;

float ank_m[3] = {0, 0, 0};
int   ank_mi = 0;
bool  ank_m_primed = false;

uint32_t t0 = 0, next_us = 0, phase_t0 = 0;
uint32_t fold_count = 0, overrun = 0, delta_jump = 0;
uint32_t cycle_max_us = 0;
uint32_t arm_until = 0;             // 트리거 유예 (armms)
uint32_t arm_note_ms = 0;

uint32_t cue_since = 0;
bool     cue_on = false;
uint32_t blink_until = 0;           // 첫 접기 LED 3연점멸

// ---- 자유비행 시행 기록 ----
uint32_t trial_n = 0;
int8_t   tr_dir = 0;
uint32_t tr_t0 = 0;                 // 놓기 감지 시각 [ms]
float    tr_phi0 = 0, tr_ank0 = 0, tr_beta0 = 0, tr_A0 = 0;
uint32_t tr_t2 = 0, tr_t4 = 0, tr_t8 = 0;   // |ψ| 2/4/8° 통과시각 (놓기 기준 ms)
uint32_t tr_quiet_ms = 0;
// 방향별 λ 통계 (밴드 4→8°)
double   lam_sum_p = 0, lam_sum_n = 0;
uint32_t lam_n_p = 0, lam_n_n = 0;

// ---- 단일접기 시행 기록 (모드 2) ----
bool     sf_folded = false;         // 이번 시행에서 이미 접었나 (한 번만!)
bool     sf_reported = false;       // F 행을 이미 냈나
float    sf_A_pre = 0, sf_d0 = 0, sf_dd_cmd = 0;
uint32_t sf_fold_ms = 0;            // 접기 시각 (놓기 기준 ms)
uint32_t sf_lock_t0 = 0;            // LOCK 창 시작 (절대 ms)
uint8_t  sf_goaln = 0;              // 이번 시행의 접기 명령 횟수 — 1 이어야 한다 (문서 76 §2)

// ---- 센서·통신 감시 (finale6 그대로) ----
uint8_t  phi_err = 0, ank_err = 0;
uint16_t phi_raw = 0, ank_raw = 0;
uint32_t phi_chg_ms = 0, ank_chg_ms = 0;
bool     dxl_err = false;
uint8_t  dxl_fail_n = 0;
uint32_t err_next_ms = 0;
uint32_t dxl_err_note_ms = 0;       // dxl 메시지 반복 스로틀 (10 s)
uint8_t  dly_skip = 0;              // 유휴/자유비행 폴링 감속용
uint32_t stall_n = 0;               // 루프 정지(>50 ms) 횟수 — t 에 찍는다
uint32_t fold_wait_ms = 0;
bool     delta_primed = false;
uint32_t dlim_warn_ms = 0;
float    v_now = 0, v_min = 0, i_peak = 0;
uint32_t pw_next = 0; uint8_t pw_phase = 0;
uint32_t vlow_warn_ms = 0;
uint32_t v_bad = 0;                 // 말이 안 되는 전압 판독 횟수 (t 에 찍는다)
uint8_t  v_low_n = 0;               // 저전압 연속 판독 수 — 2 연속이어야 경보

uint32_t log_next_ms = 0;
long     manual_cmd = 0;

// 잡음 측정 (finale6 그대로)
bool     noise_on = false;
uint32_t noise_end = 0;
double   ns_phi = 0, ns_ank = 0, ns_del = 0, ns_A = 0;
uint32_t ns_n = 0;
float    sl_phi = 0, sl_ank = 0, sl_del = 0, sl_A = 0;
bool     ns_primed = false;
const float SLOW_A = 0.05f;

char     linebuf[24];
uint8_t  linelen = 0;
uint32_t last_rx_ms = 0;

// ============================================================================
// 작은 도구
// ============================================================================
float wrap180(float x) {
  while (x >  180.0f) x -= 360.0f;
  while (x < -180.0f) x += 360.0f;
  return x;
}

void cueLamp(bool on) {
  digitalWrite(LED_PIN, on ? LED_ON : LED_OFF);
  digitalWrite(CUE_PIN, on ? HIGH : LOW);
}

// ============================================================================
// AS5047P (SPI 모드1, ANGLECOM) — finale6 그대로
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

// 두 원값의 원형 중간값 (양방향 정착 평균 — 문서 66 §2)
uint16_t rawMid(uint16_t a, uint16_t b) {
  int16_t d = (int16_t)((b - a) & 0x3FFF);
  if (d > 8191) d -= 16384;
  return (uint16_t)((a + d / 2) & 0x3FFF);
}

float med3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// ============================================================================
// ★직립 변환 — 문서 66 §4. 이 함수가 ±180° 플립의 '단일 지점'이다.
//   영점은 매달림 자세(φ=0, α=180)에서 잡힌다. 서서 동작할 때는 발목 채널이
//   180° 돌아가 있으므로 +180° 를 더해 α≈0 기준으로 되돌린다.
//   φ 는 기구에 따라 도는 경우(fphi 1)와 아닌 경우(기본)가 있다 — p 로 확인.
// ============================================================================
float upFlipAnk(float ank_hang) {
  return (FLIP_ANK >= 0.5f) ? wrap180(ank_hang + 180.0f) : ank_hang;
}
float upFlipPhi(float phi_hang) {
  return (FLIP_PHI >= 0.5f) ? wrap180(phi_hang + 180.0f) : phi_hang;
}

// ============================================================================
// 모터 (finale6 그대로)
// ============================================================================
float readDelta() {
  if (!motor_ok) return hold;
  // ★v4 — δ 폴링 감속: 제어(모드 0·2)로 뛰는 중이 아니면 20 Hz 면 충분하다.
  //   (자유비행·유휴에서 δ 는 안 움직인다. 200 Hz 폴링은 버스만 괴롭히고,
  //    접촉 불량·전원 빈약이 있으면 그 부하가 그대로 '통신실패' 로 보인다)
  bool need_fast = running && (run_mode != 1);
  if (!need_fast) {
    if (++dly_skip < 10) return delta_now;               // 10 사이클에 1번만 실제로 읽는다
    dly_skip = 0;
  }
  uint32_t t_us = micros();
  float t = dxl.getPresentPosition(DXL_ID);
  uint32_t took = (uint32_t)(micros() - t_us);
  // ★v5 — 300 ms 를 넘게 '걸렸다' 면 서보가 느린 게 아니라 우리 쪽(USB 직렬 정체 등)이
  //   멈춘 것이다 (라이브러리 타임아웃은 ~100 ms). 그 시간을 서보 탓으로 세지 않는다.
  if (took > 300000UL) { return delta_now; }
  bool slow = (took > DXL_SLOW_US);
  if (slow || t == 0.0f) {
    if (dxl_fail_n < 255) dxl_fail_n++;
    if (dxl_fail_n >= DXL_FAIL_N && !dxl_err) {
      dxl_err  = true;
      motor_ok = false;
      Serial.print("<< dxl 통신실패 (");
      Serial.print(slow ? "타임아웃" : "0 반환");
      Serial.println(") — 폴링 중단 >>");
      Serial.println("   원인 순서: ①배터리 (USB 만으로는 서보 응답이 간헐적으로 죽는다)");
      Serial.println("              ②DXL 4핀 커넥터 접점  ③t 로 보드레이트 확인 (57600 이면 빠듯)");
      Serial.println("   복구는 k (또는 g 를 누를 때 한 번 재시도)");
    }
    return delta_now;
  }
  dxl_fail_n = 0;
  float d = MOTOR_DIR * (t - home_tick) / TICK_PER_DEG;
  if (delta_primed && fabsf(d - delta_now) > 30.0f) { delta_jump++; return delta_now; }
  delta_primed = true;
  if (fabsf(d) > DELTA_LIMIT + 15.0f && (int32_t)(millis() - dlim_warn_ms) >= 0) {
    dlim_warn_ms = millis() + 2000;
    Serial.print("?  delta 가 기구한계 밖이다: "); Serial.print(d, 1);
    Serial.println(" deg  — 영점이 틀렸거나 관절이 한계를 넘었다 (z 로 다시 잡을 것)");
  }
  return d;
}

void printHwError(int32_t e) {
  Serial.print("   원인: ");
  if (e & 0x01) Serial.print("입력전압 ");
  if (e & 0x04) Serial.print("과열 ");
  if (e & 0x08) Serial.print("엔코더 ");
  if (e & 0x10) Serial.print("전기충격 ");
  if (e & 0x20) Serial.print("★과부하 ");
  Serial.println();
}

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

bool torqueIsOn() {
  for (int i = 0; i < 3; i++) { if (dxl.getTorqueEnableStat(DXL_ID)) return true; delay(60); }
  return false;
}

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
  home_tick = pos;
  hold = 0; delta_now = 0; delta_primed = false;
  dxl.setGoalPosition(DXL_ID, home_tick); delay(60);
  dxl.torqueOn(DXL_ID); delay(150);
  return torqueIsOn();
}

void motorRecover() {
  if (!motor_ok) { Serial.println("<< 모터 무응답 — 배터리·RS-485 부터 >>"); return; }
  int32_t e = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, DXL_ID);
  Serial.print("# 복구 시작. hw_error=0x"); Serial.print(e < 0 ? 0 : e, HEX);
  Serial.print("  torque="); Serial.println(torqueIsOn() ? "ON" : "OFF");
  if (e > 0) printHwError(e);

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
  Serial.println("# 1단계 실패 -> 리부트 명령");
  dxl.reboot(DXL_ID, 500);
  delay(1500);
  if (bringUpAfterRestart()) {
    Serial.println("# 2단계 성공 — 리부트로 복구됨");
    Serial.println("#   ★모터 홈이 새로 잡혔다. 매달아 z 두 번을 다시 할 것");
    return;
  }
  Serial.println("# 2단계 실패 -> ★버스 전원을 껐다 켠다");
#ifdef BDPIN_DXL_PWR_EN
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  Serial.println("#   전원 OFF ...");
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
  delay(1500);
  Serial.println("#   전원 ON  ...");
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  delay(2000);
  if (bringUpAfterRestart()) {
    Serial.println("# 3단계 성공 — 전원 재인가로 복구됨");
    Serial.println("#   ★모터 홈이 새로 잡혔다. 매달아 z 두 번을 다시 할 것");
    return;
  }
#else
  Serial.println("<< 이 보드에서는 버스 전원 제어를 못 한다 >>");
#endif
  e = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, DXL_ID);
  Serial.print("<< 전원 재인가로도 안 된다. hw_error=0x"); Serial.println(e < 0 ? 0 : e, HEX);
  if (e > 0) printHwError(e);
  Serial.println("   ★이제는 접점이다 — DXL 4핀 커넥터를 뽑아 확인하고 다시 꽂을 것");
}

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
  hold    = delta_now;
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
    delay(400);
    int32_t e1 = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, DXL_ID);
    int32_t v1 = dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, DXL_ID);
    Serial.print("    [0.4 s 뒤 ] hw_error=0x"); Serial.print(e1 < 0 ? 0 : e1, HEX);
    Serial.print("  전압 "); Serial.print(v1 > 0 ? v1 / 10.0f : 0.0f, 1); Serial.println(" V");
    if (e1 > 0 && e0 <= 0) {
      Serial.println("    ★에러는 토크를 내린 뒤에 생겼다 (되감김·순간단선) — 전원이 약한 게 아니다");
      printHwError(e1);
    } else if (e0 > 0) {
      Serial.println("    ★에러가 넘어지기 전부터 있었다 — 넘어짐의 원인일 수 있다");
      printHwError(e0);
    }
    if (v1 > 160) Serial.println("    ★전압 16 V 초과 = 되감김 과전압");
  }
  Serial.println("    k 토크복구 → (필요시 매달아 z 두 번) → g 재시작");
}

// ============================================================================
// 전원 감시 (finale6 그대로)
// ============================================================================
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

void powerWatch() {
  if (!motor_ok || dxl_err) return;
  uint32_t ms = millis();
  if ((int32_t)(ms - pw_next) < 0) return;
  pw_next = ms + 50;
  pw_phase ^= 1;
  if (pw_phase) {
    int32_t v = dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, DXL_ID);
    if (v <= 0) return;
    // v6 — 타당성 게이트: 서보가 응답한다는 것 자체가 버스에 10 V 이상이 있다는 뜻이다.
    //   0.1 V 같은 판독은 물리적으로 불가능한 쓰레기(깨진 응답)이므로 버린다.
    //   실측: 멀티미터 정상인데 "입력전압 0.1 V" 경보 반복 — 판독 오염이 원인.
    if (v < 80 || v > 170) { v_bad++; return; }          // 8.0 ~ 17.0 V 만 믿는다
    v_now = v / 10.0f;
    if (v_min <= 0.0f || v_now < v_min) v_min = v_now;
    if (VOLT_MIN > 0.0f && v_now < VOLT_MIN) {
      if (++v_low_n < 2) return;                          // 2 연속이어야 진짜로 본다
      if (running) {
        emergencyStop("입력전압 강하 — 배터리/전원배선");
        Serial.print("    최저 "); Serial.print(v_min, 1);
        Serial.print(" V (기준 "); Serial.print(VOLT_MIN, 1); Serial.println(" V)");
      } else if ((int32_t)(ms - vlow_warn_ms) >= 0) {
        vlow_warn_ms = ms + 3000;
        Serial.print("!! 입력전압 "); Serial.print(v_now, 1); Serial.println(" V — 배터리 확인");
      }
    } else v_low_n = 0;
  } else {
    int32_t i = dxl.readControlTableItem(PRESENT_CURRENT, DXL_ID);
    float a = fabsf((float)(int16_t)i) * 2.69f / 1000.0f;
    if (a < 10.0f && a > i_peak) i_peak = a;
  }
}

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
        if (v >= 80 && v <= 170) { float vv = v / 10.0f; if (vv < vmn) vmn = vv; n++; }
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
  Serial.print("  최대 전류     ");  Serial.print(imx, 2); Serial.println(" A");
  Serial.print("  hw_error  전 0x"); Serial.print(e0 < 0 ? 0 : e0, HEX);
  Serial.print("  ->  후 0x");       Serial.println(e1 < 0 ? 0 : e1, HEX);
  if (e1 > 0) printHwError(e1);
  Serial.println("=============================");
}

// ============================================================================
// 센서 감시 + 상태 추정 (finale6 그대로 + 직립 변환 한 곳)
// ============================================================================
uint8_t encGrade(uint16_t raw, uint16_t* last, uint32_t* chg_ms, uint32_t now) {
  bool rail = (raw == 0 || raw == 0x3FFF);
  if (raw != *last && !rail) *chg_ms = now;
  *last = raw;
  if (ENC_WARN_MS <= 0.0f) return 0;
  uint32_t still = now - *chg_ms;
  if (rail && still >= (uint32_t)ENC_WARN_MS) return 2;   // 레일 고착(0/16383)은 언제나 고장
  // ★v2 — '정지 고착'의 고장 승격은 제어 중(running)에만.
  //   매달림 영점·책상 정지는 '정말로 완전히 안 움직이는' 상태라 원값이 한 카운트도
  //   안 바뀌는 것이 정상인데, 그걸 고장으로 승격해 << ank 고장 >> 을 찍고 있었다.
  //   (finale6 헤더의 오경보 주의를 기본 동작으로 반영. 줄 위에서는 잡음만으로
  //    1 LSB 가 늘 흔들리므로 제어 중 감시 능력은 그대로다.)
  if (running && ENC_FAIL_MS > 0.0f && still >= (uint32_t)ENC_FAIL_MS) return 2;
  if (still >= (uint32_t)ENC_WARN_MS) return 1;
  return 0;
}

bool sensorFault() { return (phi_err >= 2 || ank_err >= 2); }
bool anyFault()    { return (phi_err >= 2 || ank_err >= 2 || dxl_err); }

void readState() {
  // ★v5 — 직전 호출에서 50 ms 넘게 지났으면 (직렬 정체·명령 처리 등으로 루프가 섰던 것)
  //   기저차분 창이 그 공백을 속도로 오해한다 → 추정기를 다시 채운다 (문서 76 §3② 와 같은 병).
  static uint32_t last_rs_ms = 0;
  uint32_t rs_now = millis();
  if (primed && last_rs_ms != 0 && (uint32_t)(rs_now - last_rs_ms) > 50) {
    primed = false; stall_n++;
  }
  last_rs_ms = rs_now;

  uint16_t raw_p = as5047_raw(PHI_CS);
  uint16_t raw_a = as5047_raw(ANK_CS);
  uint32_t now_ms = millis();
  phi_err = encGrade(raw_p, &phi_raw, &phi_chg_ms, now_ms);
  ank_err = encGrade(raw_a, &ank_raw, &ank_chg_ms, now_ms);

  // 매달림 기준 원각
  float phi_h = rawToDeg(raw_p, phi_zero);
  float ank_h = rawToDeg(raw_a, ank_zero);

  // ★직립 변환 — 여기 한 곳 (문서 66 §4)
  float phi = upFlipPhi(phi_h);
  float ank = upFlipAnk(ank_h);

  if (ank_med3) {
    if (!ank_m_primed) { ank_m[0] = ank_m[1] = ank_m[2] = ank; ank_m_primed = true; }
    ank_m[ank_mi] = ank;
    ank_mi = (ank_mi + 1) % 3;
    ank = med3(ank_m[0], ank_m[1], ank_m[2]);
  }

  delta_now = readDelta();

  // 래핑된(±180) 순간값
  float phi_w   = phi;
  float alpha_w = ank - phi_w;                    // ★α = ank − φ (문서 70 §2)
  float beta_w  = alpha_w + P2R * delta_now;      // β = α + P2R·δ

  // ★v2 — 판정·속도용 φ·β 는 연속값. 래핑 경계(±180 = 매달림 자세!)에서
  //   ±360° 점프가 차분으로 들어가는 것을 막는다. 표시용 ank_d 만 래핑 유지.
  if (!primed) {
    phi_d = phi_w; beta_d = beta_w;
    prev_phi_w = phi_w; prev_beta_w = beta_w;
  } else {
    phi_d  += wrap180(phi_w  - prev_phi_w);
    beta_d += wrap180(beta_w - prev_beta_w);
    prev_phi_w = phi_w; prev_beta_w = beta_w;

    // ★v8 — 감김수 제거 (사용자 지적: 세운 뒤 f=+358, b=−355).
    //   뒤집어 세우는 동작의 회전 이력이 연속값에 ±360 오프셋으로 남는다.
    //   연속값과 래핑값의 차는 정확히 360 의 배수이므로, ★경계(±180)에서 멀 때만★
    //   원가지로 스냅한다 — 매달림(래핑값이 ±180 위)에서는 스냅하지 않아
    //   플러터 방어가 그대로 산다. 스냅 순간 히스토리를 다시 채워 속도 스파이크를 막는다.
    bool snapped = false;
    if (fabsf(phi_d  - phi_w)  > 180.0f && fabsf(phi_w)  < 90.0f) { phi_d  = phi_w;  snapped = true; }
    if (fabsf(beta_d - beta_w) > 180.0f && fabsf(beta_w) < 90.0f) { beta_d = beta_w; snapped = true; }
    if (snapped) {
      for (int i = 0; i <= VEL_N; i++) { phi_hist[i] = phi_d; beta_hist[i] = beta_d; }
      dphi = dbeta = 0;
    }
  }
  ank_d   = ank;                                  // 표시·로그용 (래핑)
  alpha_d = beta_d - P2R * delta_now;             // 연속 α — β=α+P2R·δ 일관 유지

  if (!primed) {
    for (int i = 0; i <= VEL_N; i++) { phi_hist[i] = phi_d; beta_hist[i] = beta_d; }
    dphi = dbeta = 0; primed = true;
  }
  phi_hist[hist_i]  = phi_d;
  beta_hist[hist_i] = beta_d;
  int old = (hist_i + 1) % (VEL_N + 1);
  float dphi_raw  = (phi_d  - phi_hist[old])  / (VEL_N * DT_S);
  float dbeta_raw = (beta_d - beta_hist[old]) / (VEL_N * DT_S);
  hist_i = old;
  dphi  += EMA_A * (dphi_raw  - dphi);
  dbeta += EMA_A * (dbeta_raw - dbeta);

  Ahat = W_PHI * phi_d + W_BETA * beta_d
       + W_PHIDOT * dphi + W_BETADOT * dbeta + A_OFFSET;
}

// ============================================================================
// 프로파일 시간 [ms] (finale6 그대로)
// ============================================================================
float profileMs(float deg) {
  deg = fabsf(deg);
  float acc = ACC_UNIT * ACC_UNIT_DPS2;
  float vmx = VEL_UNIT * VEL_UNIT_DPS;
  if (acc <= 1.0f || vmx <= 1.0f || deg <= 0.0f) return 20.0f;
  float t = (sqrtf(deg * acc) <= vmx) ? (2.0f * sqrtf(deg / acc))
                                      : (deg / vmx + vmx / acc);
  return t * 1000.0f;
}

// ============================================================================
// 상태기계 — 제어 모드 (v21 시뮬 _fwe_async 이식)
// ============================================================================
void controlStep() {
  uint32_t now = millis();

  switch (phase) {
    case IDLE: {
      // ★무장 유예 (문서 76 §3②) — g 직후 속도추정기가 채워지기 전의 스파이크 방어.
      //   g 를 '놓기 전에' 누르면 유예가 손에 들고 있는 동안 지나가 대가가 없다.
      bool armed = ((int32_t)(now - arm_until) >= 0);
      if (!armed && fabsf(Ahat) > A_TRIG) {
        if ((int32_t)(now - arm_note_ms) >= 0) {
          arm_note_ms = now + 300;
          Serial.print("(유예) 문턱을 넘었지만 아직 안 접는다 — 남은 ");
          Serial.print((int32_t)(arm_until - now));
          Serial.print(" ms   Ahat="); Serial.println(Ahat, 2);
        }
        break;
      }
      if (armed && fabsf(Ahat) > A_TRIG) {
        float step = FOLD_SIGN * RHO * GAMMA * Ahat;      // Δδ = ρ·γ·Â (시뮬 그대로)
        if (fabsf(step) > STEP_LIMIT) {                   // step_cap (시뮬 그대로)
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
        if (fold_count == 1) {
          // ★문서 67 §4 — 첫 접기 신호: 크랭크쪽 손을 y축으로 빼라
          Serial.println("★★ 첫 접기 — 크랭크쪽 손을 y축(줄 방향)으로 즉시 뺄 것 ★★");
          blink_until = now + 600;                       // LED 3연점멸이 신호다                        // LED 3연점멸
        }
        float fw = profileMs(step) * 1.3f + 20.0f;
        if (fw > FOLD_TMAX) fw = FOLD_TMAX;
        fold_wait_ms = (uint32_t)fw;
        phase = FOLD;
        phase_t0 = now;
      }
      else if (fabsf(Ahat) < A_RELAX && fabsf(hold) > HOLD_DEADBAND) {
        float d = RELAX_RATE * DT_S;                      // 저속 복귀 (시뮬 vret)
        hold += (hold > 0 ? -d : d);
        if (fabsf(hold) < 0.05f) hold = 0;
        static uint8_t thr = 0;
        if (++thr >= 10) { thr = 0; writeGoal(hold); }
      }
      break;
    }
    case FOLD: {
      if (dry_run ||
          fabsf(delta_now - hold) < FOLD_TOL ||
          (uint32_t)(now - phase_t0) >= fold_wait_ms) {
        phase = REST;
        phase_t0 = now;
      }
      break;
    }
    case REST: {
      if ((uint32_t)(now - phase_t0) >= (uint32_t)T_REST) phase = IDLE;
      break;
    }
    case FALLEN: default: break;
  }
}

// ============================================================================
// ★자유비행 측정 모드 — 한 시행 = 놓기 한 번 (문서 67 §실측②·③ 겸용)
//   놓기점 스냅샷(→c₀ 경계선)과 |ψ| 2/4/8° 통과시각(→λ 즉석값)을 자동 기록한다.
//   φ_eq 는 방향별 λ 가 갈리는 것으로 드러난다 — t 요약이 힌트를 준다 (문서 70 §5).
// ============================================================================
void printTrialRow() {
  float lam24 = (tr_t4 > tr_t2 && tr_t2 > 0) ? 693.1f / (float)(tr_t4 - tr_t2) : 0.0f;
  float lam48 = (tr_t8 > tr_t4 && tr_t4 > 0) ? 693.1f / (float)(tr_t8 - tr_t4) : 0.0f;
  Serial.print("R,");
  Serial.print(trial_n);           Serial.print(',');
  Serial.print((int)tr_dir);       Serial.print(',');
  Serial.print(tr_phi0, 3);        Serial.print(',');
  Serial.print(tr_ank0, 3);        Serial.print(',');
  Serial.print(tr_beta0, 3);       Serial.print(',');
  Serial.print(tr_A0, 3);          Serial.print(',');
  Serial.print(tr_t2);             Serial.print(',');
  Serial.print(tr_t4);             Serial.print(',');
  Serial.print(tr_t8);             Serial.print(',');
  Serial.print(lam24, 3);          Serial.print(',');
  Serial.println(lam48, 3);
  if (lam48 > 0.5f) {
    if (tr_dir > 0) { lam_sum_p += lam48; lam_n_p++; }
    else            { lam_sum_n += lam48; lam_n_n++; }
  }
  // 사람용 한 줄
  Serial.print("# 시행 "); Serial.print(trial_n);
  Serial.print("  방향 "); Serial.print(tr_dir > 0 ? "+" : "-");
  Serial.print("  놓기점 f="); Serial.print(tr_phi0, 2);
  Serial.print(" k="); Serial.print(tr_ank0, 2);
  Serial.print(" b="); Serial.print(tr_beta0, 2);
  if (lam24 > 0.5f) { Serial.print("  lam(2-4도)="); Serial.print(lam24, 2); }
  if (lam48 > 0.5f) { Serial.print("  lam(4-8도)="); Serial.print(lam48, 2); }
  if (tr_t8 == 0) Serial.print("  (8도 미도달 — 너무 일찍 잡았다)");
  Serial.println();
}

// F 행 — 단일접기 요약 (문서 76 의 R 줄에 해당): A⁻, Δδ 명령/실제, A⁺(LOCK 후)
void printFoldRow(float dd_act, float A_post) {
  Serial.print("F,");
  Serial.print(trial_n);       Serial.print(',');
  Serial.print(sf_A_pre, 3);   Serial.print(',');
  Serial.print(sf_d0, 2);      Serial.print(',');
  Serial.print(sf_dd_cmd, 2);  Serial.print(',');
  Serial.print(dd_act, 2);     Serial.print(',');
  Serial.print(A_post, 3);     Serial.print(',');
  Serial.print((int)LOCK_MS);  Serial.print(',');
  Serial.print(sf_fold_ms);    Serial.print(',');
  Serial.println((int)sf_goaln);
  Serial.print("# 접기: A-="); Serial.print(sf_A_pre, 2);
  Serial.print("  dd(명령/실제)="); Serial.print(sf_dd_cmd, 1);
  Serial.print("/");                Serial.print(dd_act, 1);
  Serial.print("  A+(");            Serial.print((int)LOCK_MS);
  Serial.print("ms)=");             Serial.print(A_post, 2);
  // g_est 부호 즉석 판정 (문서 76 §4): G 를 e^(lam·lock) 로 고정한 거친 값
  if (fabsf(dd_act) > 0.3f) {
    float G = expf(LAMBDA * LOCK_MS / 1000.0f);
    float g_est = (G * sf_A_pre - A_post) / dd_act;
    Serial.print("  g_est="); Serial.print(g_est, 3);
    Serial.print(g_est > 0 ? "  (sgn 맞다)" : "  (★sgn 뒤집을 것)");
  }
  if (sf_goaln != 1) Serial.print("  ⚠goaln!=1 — 이 시행은 버릴 것");
  Serial.println();
}

void measStep() {
  uint32_t now = millis();
  float psi = phi_d - PHI_EQ;

  switch (tstate) {
    case T_QUIET: {
      // 놓기 감지: |ψ| 가 문턱을 '바깥쪽으로' 넘는다 (성장 중)
      if (fabsf(psi) > REL_DET && dphi * (psi > 0 ? 1.0f : -1.0f) > 2.0f) {
        trial_n++;
        tr_dir  = (psi > 0) ? +1 : -1;
        tr_t0   = now;
        tr_phi0 = phi_d; tr_ank0 = ank_d; tr_beta0 = beta_d; tr_A0 = Ahat;
        tr_t2 = tr_t4 = tr_t8 = 0;
        sf_folded = false; sf_reported = false; sf_goaln = 0;
        tstate = T_DIVERGE;
        Serial.print("# [놓음] 시행 "); Serial.print(trial_n);
        Serial.println(tr_dir > 0 ? "  (+방향)" : "  (-방향)");
      }
      break;
    }
    case T_DIVERGE: {
      float a = fabsf(psi);
      uint32_t el = now - tr_t0;
      if (tr_t2 == 0 && a >= 2.0f) tr_t2 = el;
      if (tr_t4 == 0 && a >= 4.0f) tr_t4 = el;
      if (tr_t8 == 0 && a >= 8.0f) tr_t8 = el;

      // ---- ★단일접기 (모드 2): 문턱 돌파에 한 번만 접는다 ----
      if (run_mode == 2 && !sf_folded &&
          (int32_t)(now - arm_until) >= 0 && fabsf(Ahat) > A_TRIG) {
        float step = FOLD_SIGN * RHO * GAMMA * Ahat;
        if (fabsf(step) > STEP_LIMIT) step = (step > 0) ? STEP_LIMIT : -STEP_LIMIT;
        sf_A_pre  = Ahat;
        sf_d0     = delta_now;
        sf_dd_cmd = step;
        sf_fold_ms = el;
        hold += step;
        if (hold >  DELTA_LIMIT) hold =  DELTA_LIMIT;
        if (hold < -DELTA_LIMIT) hold = -DELTA_LIMIT;
        writeGoal(hold);
        sf_goaln++;
        fold_count++;
        sf_folded = true;
        sf_lock_t0 = now;
        Serial.println("★★ 접기(단일) — 크랭크쪽 손을 y축으로 즉시 뺄 것. 이후는 자유비행 ★★");
        blink_until = now + 600;                         // LED 3연점멸이 신호다
      }
      // LOCK 창이 끝나면 A⁺ 확정 → F 행 (이후 발산은 λ 데이터로 계속 쓴다)
      if (run_mode == 2 && sf_folded && !sf_reported &&
          (uint32_t)(now - sf_lock_t0) >= (uint32_t)LOCK_MS) {
        sf_reported = true;
        printFoldRow(delta_now - sf_d0, Ahat);
      }

      // 종료: 잡기(|φ| 초과) / 시간 초과 / 되돌아옴(놓기 실패 — 단, 접은 뒤의 복귀는
      //        접기가 일한 것이므로 취소가 아니라 정상 종료로 기록한다)
      bool caught  = fabsf(phi_d) > F_CATCH;
      bool timeout = el > 3000;
      bool aborted = (tr_t2 > 0) && (a < 1.0f) && !sf_folded;
      if (run_mode == 2 && sf_folded && sf_reported && a < 0.8f && fabsf(dphi) < 3.0f) {
        // 접기 한 방으로 되돌아와 조용해졌다 — 접기 성공 사례. λ 는 없지만 F 행은 유효.
        Serial.println("# (접기 후 복귀 — 시행 종료. 회복 사례로 기록됨)");
        printTrialRow();
        tstate = T_DONE; tr_quiet_ms = 0;
        break;
      }
      if (caught || timeout || aborted) {
        if (aborted) {
          Serial.println("# (시행 취소 — 되돌아왔다. 손이 양방향으로 잡고 있었는지 볼 것)");
          trial_n--;                                     // 취소는 번호를 되돌린다
        } else {
          if (run_mode == 2 && sf_folded && !sf_reported) {
            // LOCK 이 끝나기 전에 잡았다 — A⁺ 를 지금 값으로 내되 표시해 둔다
            sf_reported = true;
            printFoldRow(delta_now - sf_d0, Ahat);
            Serial.println("#   ⚠ LOCK 창이 다 지나기 전에 잡았다 — 이 A+ 는 조기값 (버릴 것)");
          }
          printTrialRow();
        }
        tstate = T_DONE;
        tr_quiet_ms = 0;
      }
      break;
    }
    case T_DONE: {
      // 단일접기 모드: 접힌 δ 를 천천히 0 으로 되돌린다 (다음 시행은 δ=0 에서)
      if (run_mode == 2 && fabsf(hold) > 0.05f) {
        float d = 3.0f * RELAX_RATE * DT_S;              // ~9°/s
        hold += (hold > 0 ? -d : d);
        if (fabsf(hold) < 0.05f) { hold = 0; Serial.println("# delta 복귀 완료 (0)"); }
        static uint8_t thr2 = 0;
        if (++thr2 >= 10) { thr2 = 0; writeGoal(hold); }
      }
      // 다시 조용해지면(잡아서 세웠으면) 다음 시행 대기
      if (fabsf(psi) < 0.8f && fabsf(dphi) < 3.0f) {
        if (tr_quiet_ms == 0) tr_quiet_ms = now;
        if (now - tr_quiet_ms > 300 && !(run_mode == 2 && fabsf(hold) > 0.05f)) {
          tstate = T_QUIET;
          Serial.println("# [준비] 다음 놓기 대기 — READY 불을 보고 놓을 것");
        }
      } else tr_quiet_ms = 0;
      break;
    }
  }
}

void measSummary() {
  Serial.println("---- 자유비행 측정 요약 ----");
  Serial.print("  시행 "); Serial.print(trial_n);
  Serial.print("회   (+방향 "); Serial.print(lam_n_p);
  Serial.print(" / -방향 ");    Serial.print(lam_n_n); Serial.println(")");
  float lp = lam_n_p ? (float)(lam_sum_p / lam_n_p) : 0.0f;
  float ln_ = lam_n_n ? (float)(lam_sum_n / lam_n_n) : 0.0f;
  Serial.print("  lam(4-8도 평균)  +방향 "); Serial.print(lp, 2);
  Serial.print("   -방향 ");                 Serial.print(ln_, 2);
  Serial.print("   (phieq=");                Serial.print(PHI_EQ, 2); Serial.println(")");
  if (lam_n_p >= 3 && lam_n_n >= 3) {
    float d = fabsf(lp - ln_) / ((lp + ln_) * 0.5f);
    if (d > 0.2f) {
      Serial.println("  ★방향별 lam 이 20% 넘게 갈린다 = phieq 가 틀렸다는 뜻 (문서 70 §5).");
      Serial.println("    phieq 를 조금씩 바꿔 두 방향이 만나는 값을 찾을 것.");
      Serial.println("    ★매달림 영점이 맞다면 그 값은 0 근처여야 한다 — 그 확인이 이 실험의 목적.");
    } else {
      Serial.println("  방향별 lam 일치 — phieq 이 값으로 쓸 수 있다.");
    }
  } else {
    Serial.println("  방향별 3회 이상씩 모이면 phieq 판정이 나온다. ★방향을 섞어 놓을 것.");
  }
  Serial.println("  ⚠ c0 는 온보드로 안 나온다 — 같은 phi 에서 ank 를 ±3° 흩어 30~40회 모은 뒤");
  Serial.println("    R 줄(놓기점)로 오프라인 회귀 (문서 70 §8① 절차 그대로).");
  Serial.println("  ⚠ 여기 lam 은 즉석값. 정본은 D행 로그의 ln|psi| 적합 (lambda_fit 스크립트).");
  Serial.println("----------------------------");
}

// ============================================================================
// 놓기 신호 (finale6 그대로 + 첫접기 점멸)
// ============================================================================
void updateCue() {
  uint32_t now = millis();
  if (anyFault()) {
    if (cue_on) { cue_on = false; Serial.println("# READY 취소 — 센서 이상"); }
    cue_since = 0;
    cueLamp(((now / 100) % 2) != 0);
    return;
  }
  if (noise_on) return;
  if ((int32_t)(now - blink_until) < 0) {                // 첫 접기 3연점멸이 우선
    cueLamp(((now / 100) % 2) != 0);
    return;
  }
  float cue_off = (A_TRIG > CUE_TH) ? A_TRIG : (CUE_TH * 1.5f);
  float a = fabsf(Ahat);
  if (cue_on) {
    if (a > cue_off) {
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
      cue_since = 0;
    }
  }
}

// ============================================================================
// Â 잡음 측정 (finale6 그대로)
// ============================================================================
void noiseStart() {
  noise_on = true; ns_primed = false;
  ns_phi = ns_ank = ns_del = ns_A = 0; ns_n = 0;
  noise_end = millis() + 20000;
  Serial.println("# 잡음 측정 20 s 시작 — 제어는 멈춘다.");
  Serial.println("# ⚠ 가진은 '천천히 크게' — 진자처럼 1 초에 한 번쯤 흔들리게만.");
  Serial.println("#   빠른 손떨기는 금물: 0.1 s 보다 빠른 성분은 전부 '잡음' 으로 집계된다.");
  Serial.println("#   (매달린 채 살짝 밀어 저절로 흔들리게 두는 것이 제일 좋다)");
}

void noiseAccum() {
  // v6 — ank 는 표시용 래핑값(ank_d)이 아니라 연속값으로 잰다.
  //   매달림에서 ank_d 는 ±180 경계 위라, 흔들면 +180 <-> -180 을 넘나들며
  //   360도 점프가 잡음으로 집계되어 rms 가 ~90도 로 나온다 (실측 재현).
  //   연속 ank = alpha(연속) + phi(연속).
  float ank_c = alpha_d + phi_d;
  if (!ns_primed) {
    sl_phi = phi_d; sl_ank = ank_c; sl_del = delta_now; sl_A = Ahat;
    ns_primed = true; return;
  }
  sl_phi += SLOW_A * (phi_d     - sl_phi);
  sl_ank += SLOW_A * (ank_c     - sl_ank);
  sl_del += SLOW_A * (delta_now - sl_del);
  sl_A   += SLOW_A * (Ahat      - sl_A);
  double a = phi_d - sl_phi;      ns_phi += a * a;
  double b = ank_c - sl_ank;      ns_ank += b * b;
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
  Serial.print("  ank  = "); Serial.print(r_ank, 4); Serial.println(" deg  <- 목표 0.05 이하 (문서 54)");
  Serial.print("  del  = "); Serial.print(r_del, 4); Serial.println(" deg");
  Serial.print("  Ahat = "); Serial.print(r_A, 4);
  Serial.print(" deg   / 문턱 = "); Serial.println(r_A / A_TRIG, 2);
  if (r_A < 0.25f * A_TRIG)      Serial.println("  합격.");
  else if (r_A < 0.5f * A_TRIG)  Serial.println("  경계 — 헛트리거 빈도를 셀 것.");
  else {
    Serial.println("  ★불합격 — 가만히 있어도 접는다. 문턱을 올리지 말고 하드웨어를 고칠 것.");
    if (r_ank > 3.0f * r_phi) Serial.println("   원인은 발목 채널 (자석 갭 → 배선 → 접지). 임시: j");
  }
  Serial.println("=========================================");
}

// ============================================================================
// 출력
// ============================================================================
void ps(float v, int nd) { if (v >= 0) Serial.print('+'); Serial.print(v, nd); }

const char* phaseName() {
  if (run_mode != 0 && running) {
    switch (tstate) { case T_QUIET: return "대기"; case T_DIVERGE: return "발산";
                      default: return "종료"; }
  }
  switch (phase) { case IDLE: return "IDLE"; case FOLD: return "FOLD";
                   case REST: return "REST"; default: return "STOP"; }
}

int phaseCode() {
  if (run_mode != 0 && running) return (int)tstate; // 4 대기 / 5 발산 / 6 종료
  return (int)phase;                                // 0/1/2/3
}

void bang(uint8_t g) { if (g) Serial.print('!'); }

void printState() {
  Serial.print(running ? "RUN " : "off ");
  Serial.print(run_mode == 1 ? "[자유] " : (run_mode == 2 ? "[단일] " : ""));
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
  Serial.print(" | n=");    Serial.print(run_mode != 0 ? trial_n : fold_count);
  if (cue_on) Serial.print("  READY");
  Serial.println();
}

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
  // dxl 은 10 초에 한 번만 (같은 말 1 Hz 반복이 "계속 오류" 로 보이게 했다 — v4)
  if (dxl_err && (int32_t)(now - dxl_err_note_ms) >= 0) {
    dxl_err_note_ms = now + 10000;
    Serial.println("<< dxl 통신실패 상태 — k 로 재시도 >>");
  }
  if (phi_raw == 0 || phi_raw == 0x3FFF || ank_raw == 0 || ank_raw == 0x3FFF)
    Serial.println("<< raw 0/16383 고착 = MISO 배선·커넥터 (문서 52) >>");
}

void logHeader() {
  Serial.println("# D,t_ms,phi,ank,alpha,beta,dphi,dbeta,Ahat,hold,del_now,phase,cue,err");
  Serial.println("# R,trial,dir,phi0,ank0,beta0,A0,t2_ms,t4_ms,t8_ms,lam24,lam48");
  Serial.println("# F,trial,A_pre,d0,dd_cmd,dd_act,A_post,lock_ms,fold_ms,goaln   (단일접기)");
  Serial.println("# phase: 0 IDLE/1 FOLD/2 REST/3 STOP/4 대기/5 발산/6 종료");
  Serial.println("# err = phi등급 + 4*ank등급 + 16*dxl. err!=0 구간은 버릴 것");
}

void logLine() {
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
  Serial.print(phaseCode());    Serial.print(',');
  Serial.print(cue_on ? 1 : 0); Serial.print(',');
  Serial.println((int)phi_err + 4 * (int)ank_err + (dxl_err ? 16 : 0));
}

void printStatus() {
  Serial.println("---- 상태 ----");
  Serial.print("모드: ");
  Serial.print(run_mode == 0 ? "제어(증분접기)" :
               (run_mode == 1 ? "★자유비행" : "★단일접기"));
  Serial.print("   제어: "); Serial.print(running ? "RUN" : "정지");
  Serial.println(dry_run ? "  [DRY-RUN]" : "");
  Serial.print("모터: "); Serial.print(motor_ok ? "OK @" : "응답 없음");
  if (motor_ok) { Serial.print(dxl_baud);
    Serial.print("  torque="); Serial.print(dxl.getTorqueEnableStat(DXL_ID) ? "ON" : "OFF");
    if (dxl_baud == 57600) Serial.print("   ⚠57600 폴백 — 200Hz 에 빠듯. set_baud_1m 권장"); }
  Serial.println();
  Serial.print("영점: ");
  if (zero_stage == 1) Serial.println("★1차만 기록됨 — 반대쪽에서 멎힌 뒤 z 한 번 더");
  else Serial.println("완료(또는 미실시). 직립에서 f·k·a 전부 0 근처가 정상");

  Serial.println("-- 판정식 (finale8 방식 — lam 은 Â 에 안 들어간다) --");
  Serial.print("  w = ["); Serial.print(W_PHI, 5);    Serial.print(", ");
  Serial.print(W_BETA, 5);    Serial.print(", ");
  Serial.print(W_PHIDOT, 5);  Serial.print(", ");
  Serial.print(W_BETADOT, 5); Serial.print("]   A_offset=");
  Serial.println(A_OFFSET, 5);
  Serial.print("  p2r="); Serial.print(P2R, 4);
  Serial.print("  r=");   Serial.print(R_SLOPE, 4);
  Serial.print("  c0=");  Serial.print(LINE_C, 3);
  Serial.print("  phieq="); Serial.print(PHI_EQ, 2);
  Serial.print("  lam="); Serial.print(LAMBDA, 2);
  Serial.print(" (T2="); Serial.print(693.1f / LAMBDA, 0); Serial.println(" ms)");

  Serial.println("-- 제어 --");
  Serial.print("  gam="); Serial.print(GAMMA, 1);
  Serial.print("  rho="); Serial.print(RHO, 2);
  Serial.print("  trig="); Serial.print(A_TRIG, 2);
  Serial.print("  sgn="); Serial.print(FOLD_SIGN, 0);
  Serial.print("  -> 문턱 접기량 "); Serial.print(RHO * GAMMA * A_TRIG, 2); Serial.println(" deg");
  { float fw = profileMs(RHO*GAMMA*A_TRIG)*1.3f + 20.0f;
    if (fw > FOLD_TMAX) fw = FOLD_TMAX;
    Serial.print("  한 사이클 = FOLD "); Serial.print(fw, 0);
    Serial.print(" + REST "); Serial.print(T_REST, 0);
    Serial.print(" = "); Serial.print(fw + T_REST, 0);
    Serial.print(" ms  (T2 의 "); Serial.print((fw + T_REST) / (693.1f / LAMBDA), 2);
    Serial.println(" 배)"); }

  Serial.println("-- 실행 --");
  Serial.print("  접기 "); Serial.print(fold_count);
  Serial.print("회  시행 "); Serial.print(trial_n);
  Serial.print("회  overrun="); Serial.print(overrun);
  Serial.print("  루프정지="); Serial.print(stall_n);
  Serial.print("  전압쓰레기판독="); Serial.print(v_bad);
  Serial.print("  delta점프="); Serial.print(delta_jump);
  Serial.print("  cycle_max="); Serial.print(cycle_max_us); Serial.println(" us");
  if (motor_ok) {
    int32_t e = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, DXL_ID);
    int32_t tp = dxl.readControlTableItem(PRESENT_TEMPERATURE,   DXL_ID);
    int32_t v  = dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, DXL_ID);
    Serial.print("  hw_error=0x"); Serial.print(e < 0 ? 0 : e, HEX);
    if (e > 0) printHwError(e);
    Serial.print("  온도 "); Serial.print(tp); Serial.print(" C  전압 ");
    Serial.print(v / 10.0f, 1); Serial.print(" V  최저 "); Serial.print(v_min, 1);
    Serial.print(" V  최대전류 "); Serial.print(i_peak, 2); Serial.println(" A");
  }
  Serial.print("  phi raw="); Serial.print(phi_raw);
  Serial.print(phi_err >= 2 ? " ★고장" : (phi_err ? " !" : " 정상"));
  Serial.print("   ank raw="); Serial.print(ank_raw);
  Serial.println(ank_err >= 2 ? " ★고장" : (ank_err ? " !" : " 정상"));
  if (run_mode != 0) measSummary();
  Serial.println("--------------");
  printState();
}

// ============================================================================
// 파라미터 표
// ============================================================================
struct Param {
  const char* name;
  float*      p;
  float       lo, hi;
  bool        derive;
  bool        lock;
  const char* unit;
  const char* what;
};

const Param PARAMS[] = {
  // 실측 상수 (고정) — [0..5]
  {"p2r",  &P2R,          0.05f,  0.95f,  true,  true,  "",      "실측(1) 기울기 (문서70 정본 0.4285)"},
  {"r",    &R_SLOPE,    -20.0f,  20.0f,   true,  true,  "",      "실측(2) 기울기 — 고정"},
  {"wf",   &WV_PHI,       0.0f,   2.0f,   true,  true,  "",      "동정 w 의 phi_dot 성분"},
  {"wb",   &WV_BETA,      0.0f,   2.0f,   true,  true,  "",      "동정 w 의 beta_dot 성분"},
  {"vg",   &VGAIN,        0.3f,   3.0f,   true,  true,  "",      "속도항 배율 (실측범위 0.88~1.45)"},
  {"lam",  &LAMBDA,       0.5f,  30.0f,   false, true,  "1/s",   "표시·사이클 전용 (A 에 안 들어감)"},
  // ★측정 대상 — [6..7]
  {"c0",   &LINE_C,     -20.0f,  20.0f,   true,  true,  "deg",   "★절편 — 이번에 잰다"},
  {"phieq",&PHI_EQ,     -10.0f,  10.0f,   false, false, "deg",   "★평형점 — 새 영점이면 0 근처 기대"},
  // 영점/변환 — [8]
  {"fphi", &FLIP_PHI,     0.0f,   1.0f,   false, true,  "",      "1=phi 직립 +180 (기본 — 실측 확정)"},
  {"fank", &FLIP_ANK,     0.0f,   1.0f,   false, true,  "",      "1=ank 직립 +180 (기본 0)"},
  // 제어 — [9..22]
  {"sgn",  &FOLD_SIGN,   -1.0f,   1.0f,   false, true,  "",      "접기 방향 +-1 (바닥 시험)"},
  {"gam",  &GAMMA,        1.0f,  40.0f,   false, false, "",      "접기 이득 gamma"},
  {"rho",  &RHO,          0.1f,   1.5f,   false, false, "",      "감쇠계수 rho"},
  {"trig", &A_TRIG,       0.05f,  5.0f,   false, false, "deg",   "트리거 문턱"},
  {"rel",  &A_RELAX,      0.02f,  5.0f,   false, false, "deg",   "복귀 게이트"},
  {"vrel", &RELAX_RATE,   0.0f,  30.0f,   false, false, "deg/s", "저속 복귀 (상한 ~6)"},
  {"dead", &HOLD_DEADBAND,0.0f,  10.0f,   false, false, "deg",   "복귀 데드밴드"},
  {"dstep",&STEP_LIMIT,   1.0f,  55.0f,   false, false, "deg",   "접기 1회 상한"},
  {"dlim", &DELTA_LIMIT,  5.0f,  80.0f,   false, true,  "deg",   "힙 기구한계"},
  {"rest", &T_REST,       0.0f, 500.0f,   false, false, "ms",    "REST 대기"},
  {"alim", &ANG_LIMIT,    5.0f,  90.0f,   false, false, "deg",   "안전 한계각"},
  {"armms",&ARM_MS,       0.0f,2000.0f,   false, false, "ms",    "g 직후 트리거 유예"},
  {"cue",  &CUE_TH,       0.02f,  5.0f,   false, false, "deg",   "놓기신호 문턱"},
  {"cuems",&CUE_HOLD,    50.0f,5000.0f,   false, false, "ms",    "놓기신호 유지"},
  // 측정 — [23..24]
  {"reldet",&REL_DET,     0.3f,   5.0f,   false, false, "deg",   "놓기 감지 |psi| 문턱"},
  {"fcatch",&F_CATCH,     5.0f,  20.0f,   false, false, "deg",   "시행 종료 |phi| (문서70: 8 초과 폐기)"},
  {"lock",  &LOCK_MS,    60.0f, 400.0f,   false, false, "ms",    "단일접기 A+ 확정 창 (스윕 중 고정!)"},
  // 접기 완료 — [25..26]
  {"ftol", &FOLD_TOL,     0.1f,  10.0f,   false, false, "deg",   "접기 도착 판정"},
  {"ftmax",&FOLD_TMAX,   20.0f, 800.0f,   false, false, "ms",    "FOLD 상한"},
  // 기록·모터·감시 — [27..]
  {"loghz",&LOG_HZ,       1.0f, 200.0f,   false, false, "Hz",    "CSV 주기 (115200 에서 100 이 한계)"},
  {"vel",  &VEL_UNIT,     1.0f,1023.0f,   false, false, "unit",  "프로파일 속도"},
  {"acc",  &ACC_UNIT,     1.0f,32767.0f,  false, false, "unit",  "프로파일 가속"},
  {"ewarn",&ENC_WARN_MS,  0.0f,10000.0f,  false, false, "ms",    "엔코더 경고 (0=끔)"},
  {"efail",&ENC_FAIL_MS,  0.0f,10000.0f,  false, false, "ms",    "엔코더 고장 (0=끔)"},
  {"ilim", &CUR_LIMIT,   50.0f, 1193.0f,  false, true,  "unit",  "전류 제한 1u=2.69mA"},
  {"vmin", &VOLT_MIN,     0.0f,   14.0f,  false, false, "V",     "저전압 자동정지 (0=끔)"},
};
const int N_PARAM = sizeof(PARAMS) / sizeof(PARAMS[0]);

struct Alias { char c; const char* name; };
const Alias ALIASES[] = {
  {'f', "gam"}, {'c', "trig"}, {'e', "rel"}, {'o', "rho"},
  {'l', "loghz"}, {'v', "vel"}, {'a', "acc"},
};
const int N_ALIAS = sizeof(ALIASES) / sizeof(ALIASES[0]);

void fmtF(char* out, float v, int nd) {
  bool neg = v < 0; if (neg) v = -v;
  long scale = 1; for (int i = 0; i < nd; i++) scale *= 10;
  long n = (long)(v * scale + 0.5f);
  long ip = n / scale, fp = n % scale;
  char* o = out;
  if (neg) *o++ = '-';
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
  for (int k = strlen(q.name); k < 7; k++) Serial.print(' ');
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

void sanityWarn(int i) {
  const Param& q = PARAMS[i];
  if (q.p == &R_SLOPE) {
    if (R_SLOPE > -0.05f && R_SLOPE < 0.05f) Serial.println("!! r 이 0 근처 — w 가 폭발한다");
    else if (R_SLOPE > 0) Serial.println("!! r 이 양수 — alpha 부호 규약부터 의심할 것 (문서 70 §2)");
  }
  if (q.p == &VGAIN && (VGAIN < 0.88f || VGAIN > 1.45f))
    Serial.println("?  vg 가 실측 범위(0.88~1.45) 밖 — 실측이 아니라 튜닝이다 (문서 73 §4)");
  if (q.p == &LINE_C) {
    Serial.print("   -> A_offset = c0/r = "); Serial.print(LINE_C / R_SLOPE, 3);
    Serial.println(" deg 가 A 에 상수로 더해진다");
  }
  if (q.p == &PHI_EQ)
    Serial.println("   -> psi = phi − phieq. 측정 모드의 λ 밴드 판정에만 쓴다 (A 에는 c0 가 담당)");
  if (q.p == &A_RELAX && A_RELAX >= A_TRIG)
    Serial.println("!! 복귀 게이트 >= 문턱 — 게이트가 없는 것과 같다");
  if (q.p == &STEP_LIMIT || q.p == &GAMMA || q.p == &RHO) {
    float a_sat = STEP_LIMIT / (RHO * GAMMA);
    Serial.print("   -> Ahat "); Serial.print(a_sat, 2);
    Serial.println(" deg 위는 포화 (회복 천장 2.8~4.0)");
  }
  if (q.p == &A_TRIG || q.p == &GAMMA || q.p == &RHO) {
    Serial.print("   -> 문턱 접기량 "); Serial.print(RHO * GAMMA * A_TRIG, 2);
    Serial.println(" deg");
  }
  if (q.p == &ARM_MS && ARM_MS > 0.0f) {
    float mult = expf(LAMBDA * ARM_MS / 1000.0f);
    Serial.print("   -> 유예 동안 A 는 e^(lam·t) = "); Serial.print(mult, 2);
    Serial.println(" 배 자란다. g 를 '놓기 전에' 누르면 대가가 없다 (문서 76 §3②)");
  }
  if (q.p == &LOG_HZ && LOG_HZ > 100.0f)
    Serial.println("!! 115200 baud 로는 100 Hz 가 한계");
  if (q.p == &F_CATCH && F_CATCH > 9.0f)
    Serial.println("?  8 도 넘는 구간은 분석에서 버린다 (문서 70 §4-4) — 더 키울 이유가 없다");
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
  if (q.p == &FLIP_PHI || q.p == &FLIP_ANK)  v = (v >= 0.5f) ? 1.0f : 0.0f;
  *q.p = v;
  if (q.derive) {
    deriveConstants();
    primed = false; dphi = dbeta = 0;
    Serial.println("# w 재계산 + 추정기 리셋");
  }
  if (q.p == &FLIP_PHI || q.p == &FLIP_ANK) {
    primed = false; Serial.println("# 직립 변환 변경 — 추정기 리셋");
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
  Serial.println("==== 파라미터 (이름 값. 예: c0 -0.4 / phieq 0.3) ====");
  Serial.println("-- 실측 상수 (고정) --");
  for (int i = 0; i <= 5; i++) printParam(i, true);
  Serial.println("-- ★측정 대상 --");
  for (int i = 6; i <= 7; i++) printParam(i, true);
  Serial.println("-- 영점/제어 --");
  for (int i = 8; i <= 22; i++) printParam(i, true);
  Serial.println("-- 측정/완료판정/기록/감시 --");
  for (int i = 23; i < N_PARAM; i++) printParam(i, true);
  Serial.println("별칭: f=gam c=trig e=rel o=rho l=loghz v=vel a=acc");
  Serial.println("=====================================================");
}

void dumpSource() {
  Serial.println();
  Serial.println("// ----8<---- hangcal_fold.ino 상수 블록에 붙여넣기 ----8<----");
  for (int i = 0; i < N_PARAM; i++) {
    const Param& q = PARAMS[i];
    const char* var =
      (q.p == &P2R) ? "P2R" : (q.p == &R_SLOPE) ? "R_SLOPE" :
      (q.p == &WV_PHI) ? "WV_PHI" : (q.p == &WV_BETA) ? "WV_BETA" :
      (q.p == &VGAIN) ? "VGAIN" : (q.p == &LAMBDA) ? "LAMBDA" :
      (q.p == &LINE_C) ? "LINE_C" : (q.p == &PHI_EQ) ? "PHI_EQ" :
      (q.p == &FLIP_PHI) ? "FLIP_PHI" : (q.p == &FLIP_ANK) ? "FLIP_ANK" :
      (q.p == &FOLD_SIGN) ? "FOLD_SIGN" :
      (q.p == &GAMMA) ? "GAMMA" : (q.p == &RHO) ? "RHO" :
      (q.p == &A_TRIG) ? "A_TRIG" : (q.p == &A_RELAX) ? "A_RELAX" :
      (q.p == &RELAX_RATE) ? "RELAX_RATE" : (q.p == &HOLD_DEADBAND) ? "HOLD_DEADBAND" :
      (q.p == &STEP_LIMIT) ? "STEP_LIMIT" : (q.p == &DELTA_LIMIT) ? "DELTA_LIMIT" :
      (q.p == &T_REST) ? "T_REST" : (q.p == &ANG_LIMIT) ? "ANG_LIMIT" :
      (q.p == &ARM_MS) ? "ARM_MS" : (q.p == &CUE_TH) ? "CUE_TH" :
      (q.p == &CUE_HOLD) ? "CUE_HOLD" : (q.p == &REL_DET) ? "REL_DET" :
      (q.p == &F_CATCH) ? "F_CATCH" : (q.p == &FOLD_TOL) ? "FOLD_TOL" :
      (q.p == &FOLD_TMAX) ? "FOLD_TMAX" : (q.p == &LOG_HZ) ? "LOG_HZ" :
      (q.p == &VEL_UNIT) ? "VEL_UNIT" : (q.p == &ACC_UNIT) ? "ACC_UNIT" :
      (q.p == &ENC_WARN_MS) ? "ENC_WARN_MS" : (q.p == &ENC_FAIL_MS) ? "ENC_FAIL_MS" :
      (q.p == &CUR_LIMIT) ? "CUR_LIMIT" : "VOLT_MIN";
    Serial.print("float "); Serial.print(var);
    for (int k = strlen(var); k < 14; k++) Serial.print(' ');
    char nb[20]; fmtF(nb, *q.p, 4);
    Serial.print("= "); Serial.print(nb); Serial.print("f;");
    for (int k = strlen(nb); k < 11; k++) Serial.print(' ');
    Serial.print("// "); Serial.println(q.what);
  }
  Serial.println("// ----8<-----------------------------------------------8<----");
  Serial.println();
}

// ============================================================================
// ★매달림 영점 — 2단계 (문서 66)
// ============================================================================
void doHangZero() {
  uint16_t rp = as5047_raw(PHI_CS);
  uint16_t ra = as5047_raw(ANK_CS);
  float tick = motor_ok ? dxl.getPresentPosition(DXL_ID) : 0.0f;

  if (zero_stage == 0) {
    z1_phi = rp; z1_ank = ra; z1_tick = tick;
    zero_stage = 1;
    Serial.println("# 영점 1/2 기록 (이쪽 방향 정착).");
    Serial.println("#   이제 반대쪽에서 살짝 밀어 완전히 멎힌 뒤 z 를 한 번 더.");
    Serial.println("#   (건마찰 데드밴드의 양끝을 재서 중간을 취한다 — 문서 66 §2)");
    return;
  }

  // 2차 — 원형 중간값으로 확정
  uint16_t mp = rawMid(z1_phi, rp);
  uint16_t ma = rawMid(z1_ank, ra);
  int16_t dp = (int16_t)((rp - z1_phi) & 0x3FFF); if (dp > 8191) dp -= 16384;
  int16_t da = (int16_t)((ra - z1_ank) & 0x3FFF); if (da > 8191) da -= 16384;
  float band_p = fabsf(dp) * (360.0f / 16384.0f);
  float band_a = fabsf(da) * (360.0f / 16384.0f);

  phi_zero = mp;
  ank_zero = ma;
  if (motor_ok) {
    if (tick == 0.0f || z1_tick == 0.0f) {
      home_tick = (tick != 0.0f) ? tick : z1_tick;
      if (home_tick == 0.0f)
        Serial.println("!! 모터 위치를 못 읽었다 — delta 영점 무효. k 복구 후 z 두 번 다시");
    } else {
      home_tick = 0.5f * (tick + z1_tick);
    }
  }
  zero_stage = 0;
  hold = 0; delta_now = 0; manual_cmd = 0;
  primed = false; ank_m_primed = false; delta_primed = false;
  dphi = dbeta = 0; Ahat = 0;
  cue_since = 0; cue_on = false; cueLamp(false);
  phi_chg_ms = ank_chg_ms = millis();
  phi_err = ank_err = 0;
  trial_n = 0; lam_sum_p = lam_sum_n = 0; lam_n_p = lam_n_n = 0;
  tstate = T_QUIET;

  Serial.println("# ★영점 완료 — 매달림 기준 (φ=0, α=180, δ=0)");
  Serial.println("#   매달린 채의 A 는 ±60 근처가 정상이고, 흔들리지 않아야 한다.");
  Serial.println("#   (phi=±180, beta=∓180 이 부분 상쇄: A = 0.664·180 − 180 = ∓60.5)");
  Serial.println("#   세우면 A 가 0 근처로 온다.");
  Serial.print  ("#   데드밴드 폭: phi "); Serial.print(band_p, 3);
  Serial.print  (" deg / ank ");           Serial.print(band_a, 3); Serial.println(" deg");
  if (band_p > 1.0f || band_a > 1.0f)
    Serial.println("!!  데드밴드가 1 deg 를 넘는다 — 두 정착이 정말 반대쪽이었나? 다시 할 것");
  Serial.println("#   지금(매달림) 화면: f=±180 / k=0 / a=±180 이어야 정상 (p 로 확인)");
  Serial.println("#   세운 뒤: f·k·a 전부 0 근처가 정상.");
  Serial.println("#   ★f 만 ±180 이면 fphi 토글, ★k 만 ±180 이면 fank 1");
  Serial.println("#   ⚠ 이 영점이 c0·phieq 의 기준이다 — 재조립하면 반드시 다시 (문서 66)");
  printState();
}

// ============================================================================
// 도움말
// ============================================================================
void printHelp() {
  Serial.println("[영점] z 매달림 영점 (두 번 눌러 완성 — 왼쪽 정착, 오른쪽 정착)");
  Serial.println("[모드] q 순환: 제어 -> 자유비행(접지 않음) -> 단일접기(1회 접고 자유낙하)");
  Serial.println("[동작] g 시작 | h 정지 | x 비상정지 | k 토크복구 | u 해제 | y dry-run");
  Serial.println("       n 잡음20s | j 발목중앙값 | m CSV | s 출력정지 | p 1회 | t 상태(λ요약)");
  Serial.println("       b 전원부하시험 | w 목록 | d 덤프 | ? 도움말 | <정수> delta 수동");
  Serial.println("[값]   이름 값.  측정 대상: c0 / phieq.  영점: fphi.  제어: gam trig 등");
  Serial.println("[순서] u→매달기→z,z → 세우기→p확인 → n → y+g 부호확인 → m,q,g 놓기 30~40회");
  Serial.println("       (방향 섞기 + 같은 phi 에서 ank ±3° 흩기) → c0·phieq 갱신 → q,g 접기");
}

// ============================================================================
// 명령 처리 (finale6 구조 + z/q 교체, kv 안내)
// ============================================================================
void handleLine(char* s) {
  while (*s == ' ' || *s == '\t') s++;
  if (*s == '\0') return;

  char c0c = *s;
  if ((c0c >= '0' && c0c <= '9') || c0c == '+' || c0c == '-') {
    bool digit = false;
    for (const char* q2 = s; *q2; q2++) if (*q2 >= '0' && *q2 <= '9') { digit = true; break; }
    if (!digit) { Serial.println("# 숫자가 없음 — 무시"); return; }
    if (running) { Serial.println("# 제어 중에는 수동 이동 금지 — h 로 멈추고"); return; }
    manual_cmd = atol(s);
    if (manual_cmd >  (long)DELTA_LIMIT) manual_cmd =  (long)DELTA_LIMIT;
    if (manual_cmd < -(long)DELTA_LIMIT) manual_cmd = -(long)DELTA_LIMIT;
    if (dry_run)  { Serial.println("!! dry-run ON — 명령 안 나감. y 로 해제"); return; }
    if (!motor_ok){ Serial.println("!! 모터 무응답 — k 로 재시도"); return; }
    if (!dxl.getTorqueEnableStat(DXL_ID)) torqueRestoreHere();
    hold = (float)manual_cmd;
    writeGoal(hold);
    Serial.print("# 수동 delta -> "); Serial.println(manual_cmd);
    return;
  }

  char tok[12]; int tl = 0;
  while (s[tl] && ((s[tl] >= 'a' && s[tl] <= 'z') || (s[tl] >= 'A' && s[tl] <= 'Z') ||
                   (s[tl] >= '0' && s[tl] <= '9')) && tl < (int)sizeof(tok) - 1) {
    tok[tl] = (s[tl] >= 'A' && s[tl] <= 'Z') ? s[tl] + 32 : s[tl];
    tl++;
  }
  if (tl == 0) { tok[0] = *s; tok[1] = '\0'; tl = 1; }
  else tok[tl] = '\0';
  const char* rest = s + tl;
  while (*rest == ' ' || *rest == '\t' || *rest == '=') rest++;
  bool has = false;
  for (const char* q2 = rest; *q2; q2++) if ((*q2 >= '0' && *q2 <= '9')) { has = true; break; }
  float val = has ? atof(rest) : 0.0f;

  // 옛 이름 안내 (문서 73 §6)
  if (!strcmp(tok, "kv")) {
    Serial.println("# kv 는 없어졌다 (finale8 방식). 속도항 배율은 vg (1.0 = 실측 그대로).");
    Serial.println("#   옛 kv 1.5 ≈ 지금 vg 0.90");
    return;
  }

  const char* name = tok;
  if (tl == 1) for (int i = 0; i < N_ALIAS; i++)
    if (ALIASES[i].c == tok[0]) { name = ALIASES[i].name; break; }

  int pi = findParam(name);
  if (pi >= 0) {
    if (has) setParam(pi, val);
    else { printParam(pi); sanityWarn(pi); }
    return;
  }

  if (tl != 1) { Serial.print("# 모르는 이름: "); Serial.println(tok); printHelp(); return; }

  switch (tok[0]) {
    case 'z':
      if (running) { Serial.println("# h 로 멈추고 영점을 잡을 것"); break; }
      doHangZero();
      break;

    case 'q':
      if (running) { Serial.println("# h 로 멈추고 모드를 바꿀 것"); break; }
      run_mode = (run_mode + 1) % 3;
      tstate = T_QUIET;
      Serial.print("# 모드: ");
      if (run_mode == 0) Serial.println("제어(증분접기 — 여러 번 접는다)");
      else if (run_mode == 1) {
        Serial.println("★자유비행 (접지 않는다 — 놓기점·λ 자동 기록)");
        Serial.println("#   phieq(영점 검증)는 이 모드의 시행으로 판정한다");
        Serial.println("#   방향 섞기 + 같은 phi 에서 ank ±3° 흩기(c0). m 으로 CSV 켤 것");
      } else {
        Serial.println("★단일접기 (한 번 접고 넘어지는 것을 기록 — 접기 F행 + λ R행)");
        Serial.println("#   놓기 → 문턱에서 1회 접기 → 이후 자유비행 발산 → 잡기");
        Serial.println("#   λ 밴드(2~9°)는 접기 뒤에 지나간다. g 는 놓기 '전에' 누를 것");
        Serial.println("#   ⚠ 이 모드의 λ 는 접힌 δ 상태의 값 — phieq 판정은 자유비행 모드로");
      }
      break;

    case 'g':
      if (dxl_err && !dry_run) motorRetry();
      if (!motor_ok && !dry_run) { Serial.println("# 모터 응답 없음 — y 로 dry-run 하거나 배선 확인"); break; }
      if (phase == FALLEN) { Serial.println("# STOP 상태 — k 로 복구 후 (필요시 z) 부터"); break; }
      if (sensorFault()) { Serial.println("# ★센서 고장 — 시작하지 않는다. t 로 확인"); break; }
      if (zero_stage == 1) { Serial.println("# ★영점이 1차만 기록된 상태다 — z 를 마저 누르거나 다시"); break; }
      if (motor_ok && !dry_run && !dxl.getTorqueEnableStat(DXL_ID)) torqueRestoreHere();
      if (motor_ok && !dry_run) delta_now = readDelta();
      if (fabsf(hold - delta_now) > 1.0f) {
        Serial.print("# 시작 정렬: hold "); Serial.print(hold, 1);
        Serial.print(" -> "); Serial.print(delta_now, 1); Serial.println(" deg");
      }
      hold = delta_now;
      primed = false; dphi = dbeta = 0;
      fold_count = 0; overrun = 0; cycle_max_us = 0; delta_jump = 0;
      v_min = 0; i_peak = 0;
      phase = IDLE; phase_t0 = millis();
      arm_until = millis() + (uint32_t)ARM_MS;      // ★무장 유예 — g 는 놓기 전에 누른다
      arm_note_ms = 0;
      tstate = T_QUIET;
      running = true;
      Serial.print("# GO");
      Serial.print(run_mode == 1 ? "  [자유비행 — 접지 않는다]" :
                   (run_mode == 2 ? "  [단일접기 — 한 번만 접는다]" : ""));
      Serial.println(dry_run ? "  (DRY-RUN)" : "");
      break;

    case 'h': running = false; Serial.println("# 정지 (토크·자세 유지)"); break;

    case 'y':
      if (running) { Serial.println("# 제어 중에는 전환 금지 — h 먼저"); break; }
      dry_run = !dry_run;
      Serial.print("# dry-run "); Serial.println(dry_run ? "ON (모터 명령 안 나감)" : "OFF");
      break;

    case 'k':
      if (running) { running = false; Serial.println("# (정지하고 복구한다)"); }
      if (dxl_err || !motor_ok) motorRetry();
      motorRecover();
      if (phase == FALLEN) phase = IDLE;
      break;

    case 'u':
      running = false;
      if (motor_ok) dxl.torqueOff(DXL_ID);
      Serial.println("# 토크 해제 — 매달림 영점은 이 상태에서 잡는다 (k 로 복구)");
      break;

    case 'n':
      if (running) { Serial.println("# h 로 멈추고 재야 한다"); break; }
      primed = false; noiseStart();
      break;

    case 'j':
      ank_med3 = !ank_med3; ank_m_primed = false;
      Serial.print("# 발목 3점 중앙값 "); Serial.println(ank_med3 ? "ON (+5 ms 지연)" : "OFF");
      break;

    case 'm':
      csv_on = !csv_on;
      if (csv_on) logHeader();
      Serial.print("# CSV "); Serial.println(csv_on ? "ON" : "OFF");
      break;

    case 's':
      out_on = !out_on;
      Serial.println(out_on ? "# 출력 재개" : "# 출력 정지 (s 로 재개)");
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
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  delay(300);
#endif
  cueLamp(false);
  SPI.begin();

  deriveConstants();

  Serial.println("=== hangcal_fold — 매달림 영점 + 자유비행 실측(λ·φ_eq·c₀) + 증분접기 ===");
  Serial.println("이번 세션의 측정 대상: c0(절편) · phieq(평형점) · lam(발산율)");
  Serial.println("r·P2R·w 는 문서 70·73 확정값으로 고정.");

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
    dxl.writeControlTableItem(RETURN_DELAY_TIME,    DXL_ID, 0);
    dxl.writeControlTableItem(PROFILE_VELOCITY,     DXL_ID, (int)VEL_UNIT);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, (int)ACC_UNIT);
    applyCurrentLimit();
    home_tick = dxl.getPresentPosition(DXL_ID);
    dxl.setGoalPosition(DXL_ID, home_tick);
    delay(50);
    dxl.torqueOn(DXL_ID);
  } else {
    Serial.println("!!! 모터 응답 없음 — 엔코더만 동작 (dry-run 판정은 볼 수 있다)");
  }

  uint16_t r1 = as5047_raw(PHI_CS), r2 = as5047_raw(ANK_CS);
  Serial.print("phi raw="); Serial.print(r1);
  Serial.print("  ank raw="); Serial.println(r2);
  Serial.println("(0 또는 16383 고정이면 배선 확인)");

  printStatus();
  Serial.println();
  Serial.println("★순서: u → 매달기 → z,z(양방향) → 세우기 → p 확인(fphi?) → n → y+g 부호");
  Serial.println("        → m,q,g 자유비행 30~40회 (방향·ank 흩기) → c0/phieq 갱신 → q,g 접기");

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
  if ((int32_t)(micros() - next_us) >= 0) {
    overrun++;
    next_us = micros() + DT_US;
  }
  uint32_t c0u = micros();

  readState();

  // 안전 — 측정 모드에서도 동일하게 작동한다
  if (running && (fabsf(phi_d) > ANG_LIMIT || fabsf(alpha_d) > ANG_LIMIT)) {
    emergencyStop("한계각 초과 (넘어짐)");
  }
  powerWatch();

  // (v5) 자동 재시도는 뺐다 — ping 이 버스가 죽어 있으면 ~3 초를 통째로 블로킹해서
  //   루프를 세우고, 그 정지가 다시 '통신실패' 와 Â 스파이크로 둔갑했다 (cycle_max 3.09 s 실측).
  //   복구는 k (수동) 또는 g (시작할 때 한 번) 뿐이다. 블로킹은 사람이 시킬 때만.

  if (running && !dry_run && motor_ok) {
    static uint32_t tq_next = 0; static uint8_t tq_off_n = 0;
    uint32_t ms = millis();
    if ((int32_t)(ms - tq_next) >= 0) {
      tq_next = ms + 1000;
      if (!dxl.getTorqueEnableStat(DXL_ID)) {
        if (++tq_off_n >= 2) { tq_off_n = 0; emergencyStop("모터 토크가 풀렸다 (과부하? k 로 확인)"); }
      } else tq_off_n = 0;
    }
  }

  if (running && sensorFault()) {
    emergencyStop(phi_err >= 2 ? "phi 엔코더 이상" : "발목 엔코더 이상");
  }

  updateCue();
  reportFault();

  if (noise_on) {
    noiseAccum();
    if ((int32_t)(millis() - noise_end) >= 0) noiseReport();
  } else if (running && phase != FALLEN) {
    if (run_mode != 0) measStep();    // 자유비행/단일접기: 시행 기록 (+모드2는 접기 1회)
    else               controlStep();  // 제어: v21 증분접기
  }

  if (csv_on && out_on) {
    uint32_t ms = millis();
    if ((int32_t)(ms - log_next_ms) >= 0) {
      log_next_ms += (uint32_t)(1000.0f / LOG_HZ);
      if ((int32_t)(ms - log_next_ms) > 100) log_next_ms = ms;
      logLine();
    }
  } else if (!csv_on && out_on) {
    static uint32_t mon = 0;
    uint32_t ms = millis();
    if ((int32_t)(ms - mon) >= 0) { mon = ms + 250; printState(); }
  }

  uint32_t used = micros() - c0u;
  if (used > cycle_max_us) cycle_max_us = used;
}
