// ================================================================
// SkeinMachine v3.19  
//
// ================================================================
// MCU: ATmega328P (Uno/Nano)
// Display: SSD1306 128×64 I²C @0x3C (SDA=A4, SCL=A5), display.setRotation(2)
// Motor driver: Cytron MD13S — DIR=D8, PWM=D9 (Timer1 @ ~31 kHz)
// Motor encoder: Quadrature on D2/D3 (INT0/INT1), push-pull (no internal pull-ups)
// Inputs: Foot pedal D5 (active-LOW), KY-040: CLK=D7, DT=D6 (PCINT), SW=D4 (active-LOW)
//
// File layout:
//   [1]  Feature toggles & version
//   [2]  CONFIG (all tunables)        ← adjust here during calibration
//   [3]  Includes & global state      (grouped by domain, minimal)
//   [4]  ISRs                         (short & deterministic)
//   [5]  Low-level drivers & utils    (motor, qdec, atomic helpers)
//   [6]  Services                     (RPM, rotary, pedal, persist, UI helpers)
//   [7]  UI rendering                 (compact & debug)
//   [8]  Finite State Machine         (IDLE, RAMP_UP, RUN, RAMP_DOWN)
//   [9]  Setup & Loop
//
// Safety: no motor motion without explicit pedal press.
// ================================================================


// ================================================================
// [1] FEATURE TOGGLES & VERSION (compile-time)
// ================================================================
#define MAINT_RESET_ON_BOOT 0
#define TOAST_TEST_MODE     0
#define DEBUG_SERIAL   0      // 0=uit, 1=aan
#define SERIAL_BAUD    115200 // match dit in de Serial Monitor

// ================================================================
// [2] CONFIG — ALL TUNEABLE VALUES (calibration area)
// ================================================================

// ---------------- Display / OLED ----------------
constexpr uint8_t  OLED_ADDR                    = 0x3C;
constexpr uint8_t  SCREEN_WIDTH                 = 128;
constexpr uint8_t  SCREEN_HEIGHT                = 64;
constexpr uint8_t  OLED_ROTATION                = 2;      // 0..3 (2 = upside down)

// ---------------- Pin mapping -------------------
constexpr uint8_t  PIN_PEDAL                    = 5;      // INPUT_PULLUP, active LOW
constexpr uint8_t  PIN_DIR                      = 8;      // Cytron MD13S DIR
constexpr uint8_t  PIN_PWM                      = 9;      // Cytron MD13S PWM (Timer1 OC1A)
constexpr uint8_t  ENC_A                        = 2;      // INT0
constexpr uint8_t  ENC_B                        = 3;      // INT1
constexpr uint8_t  ROT_CLK                      = 7;      // PCINT23
constexpr uint8_t  ROT_DT                       = 6;      // PCINT22
constexpr uint8_t  ROT_SW                       = 4;      // Rotary pushbutton (active LOW)

// --------------- Mechanics / scaling ------------
constexpr uint16_t CPR_NUM                      = 9300;   // encoder counts numerator  (≈3093.333 CPR)
constexpr uint16_t CPR_DEN                      = 3;      // encoder counts denominator

// --------------- Phase anchoring ----------------
// Anchor is set on MANUAL→AUTO; per-run median+deadband nudges a small bias in counts.
constexpr uint16_t PH_DB_T1000                  = 25;     // deadband in milli-turns (≈0.025 turn)
constexpr uint8_t  PH_STEP_MAX_COUNTS           = 12;     // max bias step per skein (counts)
constexpr uint16_t PH_BIAS_MAX_COUNTS           = 250;    // clamp total phase bias (±counts) ≈0.08 turn
constexpr uint8_t  PH_LEAK_DEN                  = 128;    // leak: bias -= bias/128 each skein (0=disable)

// Slow anchor adaptation (moves anchor itself, very small steps)
constexpr uint8_t  PH_ANCHOR_ADAPT_NUM          = 1;      // ≈ 1/32 → combined ≈ 1/64 per run
constexpr uint8_t  PH_ANCHOR_ADAPT_DEN          = 32;
constexpr uint8_t  PH_ANCHOR_MAX_STEP           = 3;      // max 3 mturn per skein (≈0.003 turn)
constexpr uint8_t  PH_ANCHOR_DB_T1000           = 10;     // deadband (mturn) ≈0.010 turn
constexpr uint8_t  PH_ANCHOR_MIN_STEP           = 1;      // at least 1 mturn when outside deadband

// Phase→Lead micro-correction after settle
constexpr uint8_t  PH_LEAD_DB_T1000             = 12;     // deadband (mturn) ≈0.012 turn
constexpr uint8_t  PH_LEAD_GAIN_NUM             = 1;      // scale numerator
constexpr uint8_t  PH_LEAD_GAIN_DEN             = 8;      // scale denominator (1/8 of e[mturn]→counts)
constexpr uint8_t  PH_LEAD_MIN_STEP_COUNTS      = 1;      // min lead delta (counts)
constexpr uint8_t  PH_LEAD_MAX_STEP_COUNTS      = 8;      // max lead delta (counts)

// ---------------- RPM estimation ----------------
constexpr unsigned long RPM_WINDOW_MS           = 100;    // sample window for rpm10SinceWindow()
constexpr int16_t       STALL_RPM10_THRESH      = 50;     // stall threshold in ×10 RPM (50 = 5.0 RPM)

// --------- Stall detection refinements ---------
constexpr bool          STALL_DISABLE_IN_CREEP  = false;  // ignore stalls inside creep band
constexpr int           STALL_MIN_PWM           = 50;     // only detect stall if PWM ≥ this duty
constexpr unsigned long STALL_MIN_RUN_MS        = 200;    // must be ≥X ms in RUN before stall can trigger

// ---------------- PWM & ramps -------------------
constexpr int           PWM_FAST                 = 255;    // main running duty
constexpr int           PWM_CREEP                = 70;     // base creep duty
constexpr int           START_KICK_PWM           = 60;     // start “kick” duty
constexpr unsigned long START_KICK_MS            = 35;     // start “kick” duration
constexpr unsigned long rampUpMs                 = 500;    // linear ramp-up time
constexpr unsigned long rampDownMs               = 250;    // linear ramp-down time
constexpr unsigned long CREEP_TRANSITION_MS      = 500;    // default PWM slew time (non-creep)
constexpr int           MIN_RAMP_PWM             = 5;      // stop early if computed duty < this

// ------------- Creep RPM control ----------------
// Closed-loop duty control in creep to maintain target low RPM.
constexpr int           CREEP_RPM10_TARGET       = 250;    // ×10 RPM target (250 = 25.0 RPM)
constexpr int           CREEP_RPM10_DEADBAND     = 15;     // ±1.5 RPM deadband — no correction
constexpr int           CREEP_KP_NUM             = 1;      // proportional gain numerator
constexpr int           CREEP_KP_DEN             = 3;      // proportional gain denominator (≈0.33)
constexpr int           CREEP_KI_NUM             = 1;      // integral gain numerator (small)
constexpr int           CREEP_KI_DEN             = 20;     // integral gain denominator (≈0.05)
constexpr int           CREEP_I_CLAMP            = 30;     // clamp |I| contribution in PWM steps
constexpr int           CREEP_PWM_MIN            = 70;     // never go below this duty in creep
constexpr int           CREEP_PWM_MAX            = 130;    // cap to avoid jerky behaviour
constexpr int           CREEP_ENTER_MIN_PWMDROP  = 0;      // on creep entry, do not drop below current PWM by more than this (0 = no drop)

// ------------- Creep slew behaviour ------------
constexpr unsigned long CREEP_SLEW_MS            = 180;    // shorter slew when inside creep band
constexpr bool          CREEP_BYPASS_SLEW_WHEN_RAISING = true; // jump up immediately in creep (down still slewed)

// --------- Creep controller stability ----------
constexpr int           CREEP_DPWM_MAX           = 8;      // max PWM delta per update (rate limit)
constexpr int           CREEP_I_FREEZE_ERR       = 60;     // freeze I when |err| > 6.0 RPM (transient)

// ---------------- UI timing --------------------
constexpr unsigned long UI_REFRESH_MS            = 100;
constexpr unsigned long LIFETIME_SPLASH_MS       = 3000;

// --------------- Toast defaults ----------------
constexpr unsigned long TOAST_DEFAULT_MS         = 1000;

// ------ Approach / adaptive stop (lead) --------
constexpr long          APPROACH_COUNTS          = 4177;   // ≈1.35 turns worth of counts
constexpr long          APPROACH_HYST_COUNTS     = 155;    // hysteresis when leaving creep band
constexpr long          STOP_LEAD_COUNTS_BASE    = 300;    // initial decel lead (≈0.10 turns)
constexpr long          STOPLEAD_MIN             = 30;     // clamp
constexpr long          STOPLEAD_MAX             = APPROACH_COUNTS - 1;
constexpr uint8_t       STOPLEAD_ADAPT_K_NUM     = 2;      // ≈0.2 * delta
constexpr uint8_t       STOPLEAD_ADAPT_K_DEN     = 5;

// ---------- Lead persist policy ----------------
constexpr long          LEAD_PERSIST_EPS_COUNTS  = 30;     // only persist if |Δ| ≥ this
constexpr uint8_t       LEAD_PERSIST_MIN_RUNS    = 3;      // and after at least N skeins

// --------------- Manual / jog ------------------
constexpr int           PWM_JOG                  = 85;     // manual jog duty
constexpr int           ROT_DETENTS_PER_REV      = 20;     // KY-040 ticks per rev
constexpr int           ROT_SIGN                 = -1;     // rotary → motor direction (+1/-1)
constexpr unsigned int  ROT_MIN_DETENT_US        = 1200;   // ignore bouncing faster than this

// -------- Rotary target step size --------------
constexpr int8_t        TARGET_STEP_TURNS10      = 10;     // rotary step = 1.0 turn (×10)

// -------- Rotary double-click timing ----------
constexpr unsigned long SW_DBLCLICK_MS           = 350;

// -------- Button / pedal debounce -------------
constexpr unsigned long SW_DEBOUNCE_MS           = 25;
constexpr unsigned long SW_LONG1_MS              = 500;    // long-press 1
constexpr unsigned long SW_LONG2_MS              = 1500;   // long-press 2
constexpr unsigned long PEDAL_DB_DOWN_MS         = 8;
constexpr unsigned long PEDAL_DB_UP_MS           = 15;
constexpr unsigned long PEDAL_RELEASE_IGNORE_MS  = 80;

// --------------- EEPROM / persist --------------
constexpr uint32_t      PERSIST_MAGIC            = 0x54574953UL;  // 'TWIS'
constexpr uint16_t      PERSIST_VER              = 4;             // v3.19: bias replaces I
constexpr uint32_t      SAVE_EVERY_SKEINS        = 5;             // persist every N skeins
constexpr unsigned long PERSIST_MIN_INTERVAL_MS  = 15000UL;       // min interval between saves

// ================================================================
// [3] INCLUDES & GLOBAL STATE
// ================================================================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <avr/interrupt.h>
#include <EEPROM.h>
#include <string.h>

// ---------- Forward declarations ----------
bool     pedalRawNow();                                     // raw pedal pin (active LOW)
void     pedalRawEdgeService();                             // raw edge monitor + LED

struct   PedalEdges;
enum class RunState : uint8_t;
enum class Mode     : uint8_t;

int16_t  rpm10SinceWindow();
void     rotaryServiceFromQueue();
PedalEdges pedalService();
void     persistLoad();
void     persistSave();
void     uiLifetimeSplashTopRightLogo();
void     drawUI_ManualSimple(int16_t rpm10, long encCounts);
void     drawUI_AutoSimple  (int16_t rpm10, RunState st, long goalCountsLocal, long encLocal, int16_t targetTurns10Local);
void     drawUI_Debug       (int16_t rpm10, long pulses, long turns10, int pwm, bool pedalPressed, RunState st, Mode m, long tgt10);

inline   void motorAnalogWrite(int duty);
inline   void motorStopHard();
inline   void updatePwmSlew(unsigned long nowMs);
inline   long encAtomicRead();
inline   void encAtomicWrite(long v);
inline   long encAbsAtomicRead();
inline   long turns10_to_counts(long t10);
inline   long counts_to_turns10(long counts);
inline   long counts_to_turns100(long counts);
inline   long counts_to_turns1000(long counts);
inline   long turns1000_to_counts(long t1000);
static   void pciSetup(uint8_t pin);
static   inline uint16_t phaseFromAbsCountsT1000(long absCounts);
static   inline int16_t  phaseErrorT1000(uint16_t phase, uint16_t anchor);
static   inline int16_t  median3_i16(int16_t a, int16_t b, int16_t c);
inline   void qdecMotorReadAB_Update();
void     ISR_encA();
void     ISR_encB();
void     updateFSM(const PedalEdges& ped, unsigned long now);
inline   void showToastC(const __FlashStringHelper* s, unsigned long dur = TOAST_DEFAULT_MS, uint8_t size = 2);
inline   void showToastDyn(const char* s, unsigned long dur = TOAST_DEFAULT_MS, uint8_t size = 2);
inline   void showSkeinToast(uint32_t skeins, unsigned long dur = 2000);

// ---------------- Display object ----------------
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------------- Modes & states ----------------
enum class RunState : uint8_t { IDLE, RAMP_UP, RUN, RAMP_DOWN };
enum class Mode     : uint8_t { MANUAL, AUTO };

// ---------------- Encoder (motor) ---------------
volatile long     g_enc_count                 = 0;          // incremental counts (resettable)
volatile long     g_enc_abs                   = 0;          // absolute counts (never reset)
volatile uint8_t  g_enc_prevState             = 0;          // last AB state for QDEC

// ---------------- RPM sampling ------------------
unsigned long     g_rpm_lastMs                = 0;          // last sample time
long              g_rpm_lastCounts            = 0;          // counts at last sample
int16_t           g_rpm_lastValid10           = INT16_MIN;  // last valid ×10 RPM
bool              g_stallLatched              = false;      // avoid repeated stall toasts

// ---------------- Rotary (KY-040) ---------------
volatile int16_t  g_rot_stepQueue             = 0;          // queued detent steps
volatile uint8_t  g_rot_prevAB                = 0;          // last AB
volatile int8_t   g_rot_accum                 = 0;          // sub-detent accumulator
constexpr int8_t  ROT_DETENT_STEPS            = 4;
volatile unsigned long g_rot_lastDetentUs     = 0;
bool              g_sw_clickAwaiting          = false;
unsigned long     g_sw_lastShortMs            = 0;

// ------------- Rotary push-button ---------------
int               g_sw_state                  = HIGH;
unsigned long     g_sw_lastChangeMs           = 0;
unsigned long     g_sw_pressedAtMs            = 0;

// ---------------------- UI ----------------------
unsigned long     g_ui_lastMs                 = 0;
unsigned long     g_toast_untilMs             = 0;
uint8_t           g_toast_size                = 2;
char              g_toast_buf[22];
char              g_small_buf[22];
bool              g_uiDebug                   = false;
bool              g_toastWithIcons            = false;

// --------------- Manual / jog -------------------
long              g_manualDispCounts          = 0;          // shown counts in MANUAL
long              g_manualRefEnc              = 0;          // reference counts for jog
long              g_jogTargetRel              = 0;          // jog delta from reference
const long        JOG_COUNTS_PER_DETENT       = (long)(((long)CPR_NUM / (long)CPR_DEN + (ROT_DETENTS_PER_REV/2)) / ROT_DETENTS_PER_REV);
const long        JOG_TOL                     = (long)((JOG_COUNTS_PER_DETENT/2) > 1 ? (JOG_COUNTS_PER_DETENT/2) : 1);
unsigned long     g_manualSettleUntilMs       = 0;

// --------------- Pedal debounce -----------------
bool              g_pedalRaw                  = false;      // instantaneous pin state
bool              g_pedalStable               = false;      // debounced state
bool              g_pedalPrevStable           = false;      // previous debounced state
unsigned long     g_pedalLastChangeMs         = 0;
unsigned long     g_pedalIgnoreReleaseUntilMs = 0;          // release ignore window

// Raw pedal diagnostics
bool              g_pedalRawVis               = false;      // raw pin for UI (LOW = pressed)
bool              g_pedalRawPrev              = false;
volatile uint32_t g_pedalRawEdges             = 0;          // counted edges

struct PedalEdges {
  bool risingEff;    // debounced rising, after ignore window
  bool fallingEff;   // debounced falling, after ignore window
  bool risingRaw;    // raw rising
  bool fallingRaw;   // raw falling
};

// ---------------- PWM / slew --------------------
int               g_pwmTarget                 = PWM_FAST;   // current target duty
int               g_desiredPwmTarget          = PWM_FAST;   // desired duty (pre-slew)
unsigned long     g_lastSlewMs                = 0;
bool              g_inCreepBand               = false;
int               g_pwmNow                    = 0;
int               g_slewFrom                  = 0;
int               g_prevDesired               = -1;
unsigned long     g_slewStartMs               = 0;
unsigned long     g_slewDurationMs            = CREEP_TRANSITION_MS; // current slew duration

// Creep controller state
int               g_creepPwm                  = PWM_CREEP;  // loop-controlled creep duty
int               g_creepPwmPrev              = PWM_CREEP;  // rate-limited previous output
int16_t           g_creepI_pwm                = 0;          // integral term (PWM steps)
bool              g_wasInCreepBand            = false;      // detect creep entry/exit

// --------------- Targets & FSM ------------------
int16_t           g_targetTurns10             = 100;        // 10.0 turns (×10)
long              g_goalCounts                = 0;
bool              g_dirForward                = true;
RunState          g_state                     = RunState::IDLE;
Mode              g_mode                      = Mode::AUTO;

long              g_stopLeadFwd               = STOP_LEAD_COUNTS_BASE;
long              g_stopLeadRev               = STOP_LEAD_COUNTS_BASE;
bool              g_countedThisRun            = false;
bool              g_rampDownPlanned           = false;
long              g_encAtRampDown             = 0;
unsigned long     g_stateStartMs              = 0;
int               g_startDuty                 = 0;
unsigned long     g_settleUntilMs             = 0;
bool              g_adaptPending              = false;

// -------------- Phase anchoring -----------------
uint16_t          g_phaseAnchorT1000          = 0xFFFF;     // 0..999, 0xFFFF = unset
long              g_phaseBiasCounts           = 0;          // slow bias in counts
int16_t           g_phErrHist[3]              = {0,0,0};    // last 3 phase errors (mturn)
uint8_t           g_phErrIdx                  = 0;

// Phase debug monitors
int16_t           g_dbgLastAnchorStepT1000    = 0;          // last applied anchor step (mturn)
int16_t           g_dbgLastPhaseErrT1000      = 0;          // last measured phase error (mturn)

// -------------- Lifetime / persist --------------
uint32_t          g_skeinSession              = 0;
uint32_t          g_lifetimeSkeins            = 0;
uint32_t          g_lastSavedSkeinMark        = 0;
long              g_lastEncForLife            = 0;
unsigned long     g_lastLoopMs                = 0;
bool              g_persistDirty              = false;
unsigned long     g_persistLastSaveMs         = 0;
long              g_leadPersistedFwd          = 0;
long              g_leadPersistedRev          = 0;
uint32_t          g_leadLastSavedSkein        = 0;

// Persist structure (dual-slot with checksum + sequence)
struct Persist {
  uint32_t magic;
  uint16_t version;
  uint32_t skeins;
  uint64_t totalCounts;
  uint64_t runtimeMs;
  uint32_t leadFwdCounts;
  uint32_t leadRevCounts;
  uint32_t writeCount;
  uint32_t stallCount;
  uint16_t lastTargetTurns10;   // ×10 turns
  // v4 additions (replace v3 phaseI with bias):
  uint16_t phaseAnchorT1000;    // 0..999, 0xFFFF = unset
  int32_t  phaseBiasCounts;     // signed bias in counts
  // ---
  uint32_t seq;                 // slot sequence
  uint32_t checksum;            // FNV32 over struct except checksum
};
Persist g_persist;

// EEPROM slot layout (wear-levelling)
constexpr int     EEPROM_SLOT_A               = 0;
constexpr int     EEPROM_SLOT_SIZE            = sizeof(Persist);
constexpr int     EEPROM_SLOT_B               = EEPROM_SLOT_A + EEPROM_SLOT_SIZE;
static    uint8_t g_activeSlot                = 0;

#if defined(E2END)
  #if (EEPROM_SLOT_B + EEPROM_SLOT_SIZE - 1) > E2END
    #error "Persist struct + slots exceed EEPROM."
  #endif
#endif

// ================================================================
// [4] INTERRUPT SERVICE ROUTINES (keep them tiny)
// ================================================================

inline void qdecMotorReadAB_Update();
void ISR_encA() { qdecMotorReadAB_Update(); }
void ISR_encB() { qdecMotorReadAB_Update(); }

// KY-040 PCINT2 handler (D6/D7), emits ±1 per detent (with min spacing)
ISR(PCINT2_vect) {
  uint8_t a = (PIND >> 7) & 0x01; // D7
  uint8_t b = (PIND >> 6) & 0x01; // D6
  uint8_t curr = (a << 1) | b;
  static const int8_t rotQdec[16] PROGMEM = {
    0,-1,+1, 0,
   +1, 0, 0,-1,
   -1, 0, 0,+1,
    0,+1,-1, 0
  };
  uint8_t idx = (g_rot_prevAB << 2) | curr;
  int8_t step = pgm_read_byte(&rotQdec[idx]);
  if (step) {
    int8_t acc = g_rot_accum + step;
    if (acc >= ROT_DETENT_STEPS || acc <= -ROT_DETENT_STEPS) {
      unsigned long nowUs = micros();
      if ((nowUs - g_rot_lastDetentUs) >= ROT_MIN_DETENT_US) {
        g_rot_stepQueue += (acc > 0) ? +1 : -1;   // ±1 per detent
        g_rot_lastDetentUs = nowUs;
      }
      acc = 0;
    }
    g_rot_accum = acc;
  }
  g_rot_prevAB = curr;
}

// ================================================================
// [5] LOW-LEVEL DRIVERS & UTILS
// ================================================================
inline void motorAnalogWrite(int duty) {
  if (duty < 0) duty = 0; if (duty > 255) duty = 255;
  g_pwmNow = duty; analogWrite(PIN_PWM, duty);
}
inline void motorStopHard() { motorAnalogWrite(0); }

inline void updatePwmSlew(unsigned long nowMs) {
  if (g_desiredPwmTarget != g_prevDesired) {
    g_slewFrom     = g_pwmTarget;
    g_slewStartMs  = nowMs;
    g_prevDesired  = g_desiredPwmTarget;
  }
  if (g_pwmTarget == g_desiredPwmTarget) return;
  if (g_slewDurationMs == 0UL) { g_pwmTarget = g_desiredPwmTarget; return; }

  unsigned long elapsed = nowMs - g_slewStartMs;
  if (elapsed >= g_slewDurationMs) { g_pwmTarget = g_desiredPwmTarget; }
  else {
    const long ONE_Q15 = 1L << 15;
    long tQ = (long)((elapsed << 15) / (long)g_slewDurationMs);
    if (tQ < 0) tQ = 0; if (tQ > ONE_Q15) tQ = ONE_Q15;
    long t2Q  = (tQ * tQ) >> 15;
    long term = (3L << 15) - (2L * tQ);
    long uQ   = (t2Q * term) >> 15;
    int  span = g_desiredPwmTarget - g_slewFrom;
    long inc  = ( (long)span * uQ + (1L<<14) ) >> 15;
    int  out  = g_slewFrom + (int)inc;
    if (out < 0) out = 0; if (out > 255) out = 255;
    g_pwmTarget = out;
  }
}

// Atomic helpers
inline long encAtomicRead() { long c; noInterrupts(); c = g_enc_count; interrupts(); return c; }
inline void encAtomicWrite(long v){ noInterrupts(); g_enc_count = v; interrupts(); }
inline long encAbsAtomicRead(){ long c; noInterrupts(); c = g_enc_abs; interrupts(); return c; }

// Counts ↔ turns conversions
inline long turns10_to_counts(long t10) {
  long num = t10 * (long)CPR_NUM + (5 * CPR_DEN);
  return num / (10L * (long)CPR_DEN);
}
inline long counts_to_turns10(long counts) {
  long num = counts * (10L * (long)CPR_DEN) + (CPR_NUM / 2);
  return num / (long)CPR_NUM;
}
inline long counts_to_turns100(long counts) {
  long num = counts * (100L * (long)CPR_DEN) + ((long)CPR_NUM / 2);
  return num / (long)CPR_NUM;
}
inline long counts_to_turns1000(long counts) {
  long num = counts * (1000L * (long)CPR_DEN) + ((long)CPR_NUM / 2);
  return num / (long)CPR_NUM;
}
inline long turns1000_to_counts(long t1000) {
  long num = t1000 * (long)CPR_NUM + (500 * (long)CPR_DEN);
  return num / (1000L * (long)CPR_DEN);
}

// QDEC (motor encoder)
static const int8_t qdecTable[16] PROGMEM = {
  0, -1, +1,  0,
 +1,  0,  0, -1,
 -1,  0,  0, +1,
  0, +1, -1,  0
};
inline void qdecMotorReadAB_Update() {
  uint8_t a = (PIND >> 2) & 0x01; // D2
  uint8_t b = (PIND >> 3) & 0x01; // D3
  uint8_t curr = (a << 1) | b;
  uint8_t idx = (g_enc_prevState << 2) | curr;
  int8_t step = pgm_read_byte(&qdecTable[idx]);
  if (step) { g_enc_count += step; g_enc_abs += step; }
  g_enc_prevState = curr;
}

// PCINT setup
static void pciSetup(uint8_t pin) {
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));
  PCIFR  |= bit(digitalPinToPCICRbit(pin));
  PCICR  |= bit(digitalPinToPCICRbit(pin));
}

// ------------------ PHASE HELPERS ------------------
static inline uint16_t phaseFromAbsCountsT1000(long absCounts) {
  long t1000 = counts_to_turns1000(absCounts);
  long r = t1000 % 1000L; if (r < 0) r += 1000L;
  return (uint16_t)r; // 0..999
}
static inline int16_t phaseErrorT1000(uint16_t phase, uint16_t anchor) {
  int16_t e = (int16_t)phase - (int16_t)anchor;
  if (e > 500)  e -= 1000;
  if (e < -500) e += 1000;
  return e; // [-500..+500] mturn
}
static inline int16_t median3_i16(int16_t a, int16_t b, int16_t c) {
  // median of 3 without branches explosion
  int16_t minab = (a<b)?a:b, maxab = (a>b)?a:b;
  int16_t minc  = (maxab<c)?maxab:c;
  int16_t maxc  = (minab>c)?minab:c;
  return (minc>maxc)?minc:maxc;
}
static inline uint16_t wrapPhase1000(int16_t x){
  int16_t r = x % 1000; if (r < 0) r += 1000;
  return (uint16_t)r; // 0..999
}

// ---------------------------------------------------


// ================================================================
// [6] SERVICES (RPM, rotary, pedal, persist, UI helpers)
// ================================================================
int16_t rpm10SinceWindow() {
  unsigned long now = millis();
  unsigned long dt = now - g_rpm_lastMs;
  if (dt < RPM_WINDOW_MS) return INT16_MIN;
  long cNow = encAtomicRead();
  long delta = cNow - g_rpm_lastCounts;
  g_rpm_lastCounts = cNow;
  g_rpm_lastMs = now;
  if (dt == 0) return INT16_MIN;
  long num = delta * 5625L;
  long den = (long)dt * 29L;
  long v;
  if (num >= 0) v = (num + den/2) / den;
  else          v = (num - den/2) / den;
  if (v >  32767) v =  32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}

// Toast helpers
inline void showToastC(const __FlashStringHelper* s, unsigned long dur, uint8_t size) {
  strncpy_P(g_toast_buf, (PGM_P)s, sizeof(g_toast_buf)); g_toast_buf[sizeof(g_toast_buf)-1] = 0;
  g_toast_untilMs = millis() + dur; g_toast_size = size; g_toastWithIcons = false;
}
inline void showToastDyn(const char* s, unsigned long dur, uint8_t size) {
  strncpy(g_toast_buf, s, sizeof(g_toast_buf)); g_toast_buf[sizeof(g_toast_buf)-1] = 0;
  g_toast_untilMs = millis() + dur; g_toast_size = size; g_toastWithIcons = false;
}
inline void showSkeinToast(uint32_t skeins, unsigned long dur) {
  snprintf_P(g_toast_buf, sizeof(g_toast_buf), PSTR("%lu\nskeins!"), (unsigned long)skeins);
  g_toast_untilMs = millis() + dur;
  g_toast_size = 3;
  g_toastWithIcons = true;
}

// Rotary + button service (incl. MANUAL→AUTO anchor)
void rotaryServiceFromQueue() {
  int16_t delta;
  noInterrupts(); delta = g_rot_stepQueue; g_rot_stepQueue = 0; interrupts();
  if (delta != 0) {
    int16_t t = g_targetTurns10 + delta * TARGET_STEP_TURNS10;
    if (t < 0) t = 0;
    int16_t q = (t + (TARGET_STEP_TURNS10/2)) / TARGET_STEP_TURNS10;
    g_targetTurns10 = q * TARGET_STEP_TURNS10;

    if (g_mode == Mode::MANUAL) {
      long deltaCounts = (long)delta * ROT_SIGN * JOG_COUNTS_PER_DETENT;
      g_jogTargetRel += deltaCounts;
    } else {
      g_persistDirty = true;
    }
  }

  unsigned long now = millis();
  int reading = digitalRead(ROT_SW); // active LOW

  if (reading != g_sw_state && (now - g_sw_lastChangeMs) > SW_DEBOUNCE_MS) {
    bool wasLow = (g_sw_state == LOW);
    g_sw_state = reading;
    g_sw_lastChangeMs = now;

    if (g_sw_state == LOW) {
      g_sw_pressedAtMs = now;
    } else if (wasLow && g_sw_state == HIGH) {
      unsigned long dur = now - g_sw_pressedAtMs;

      if (g_sw_clickAwaiting && (now - g_sw_lastShortMs) > 0) {
        if (dur >= SW_LONG1_MS) g_sw_clickAwaiting = false;
      }

      if (dur >= SW_LONG2_MS) {
        g_uiDebug = !g_uiDebug;
        showToastC(g_uiDebug ? F("DEBUG ON") : F("DEBUG OFF"), 700, 2);
      }
      else if (dur >= SW_LONG1_MS) {
        g_dirForward = !g_dirForward;
        showToastC(g_dirForward ? F("DIR: FWD") : F("DIR: REV"), 700, 2);
      }
      else {
        if (g_sw_clickAwaiting && (now - g_sw_lastShortMs) <= SW_DBLCLICK_MS) {
          g_sw_clickAwaiting = false;
          uiLifetimeSplashTopRightLogo();
        } else {
          g_sw_clickAwaiting = true;
          g_sw_lastShortMs = now;
        }
      }
    }
  }

  if (g_sw_clickAwaiting && (millis() - g_sw_lastShortMs) > SW_DBLCLICK_MS) {
    g_sw_clickAwaiting = false;

    Mode prev = g_mode;
    g_mode = (g_mode == Mode::MANUAL) ? Mode::AUTO : Mode::MANUAL;
    if (g_mode == Mode::MANUAL) {
      long encNow = encAtomicRead();
      g_manualRefEnc = encNow;
      g_jogTargetRel = 0;
      motorAnalogWrite(0);
      g_manualDispCounts = 0;
      g_manualSettleUntilMs = millis() + 120;
      g_state = RunState::IDLE;
    } else if (prev == Mode::MANUAL && g_mode == Mode::AUTO) {
      // ---- MANUAL→AUTO: set anchor to current absolute phase ----
      uint16_t ph = phaseFromAbsCountsT1000(encAbsAtomicRead());
      g_phaseAnchorT1000 = ph;
      g_phaseBiasCounts  = 0;          // start zonder offset
      g_phErrHist[0]=g_phErrHist[1]=g_phErrHist[2]=0;
      persistMarkDirty();
      showToastC(F("ANCHOR SET"), 700, 2);

      // Copy MANUAL result into AUTO target (nearest whole turn)
      long c = encAtomicRead();
      long t10 = counts_to_turns10(labs(c));
      long rounded10 = ((t10 + 5) / 10) * 10;
      if (rounded10 < 0)      rounded10 = 0;
      if (rounded10 > 65535)  rounded10 = 65535;
      g_targetTurns10 = (int16_t)rounded10;
      motorAnalogWrite(0);
      g_persistDirty = true;
    }
  }
}

// Pedal debounce
PedalEdges pedalService() {
  unsigned long now = millis();
  int pr = digitalRead(PIN_PEDAL); // LOW = pressed
  bool raw = (pr == LOW);

  if (raw != g_pedalRaw) { g_pedalRaw = raw; g_pedalLastChangeMs = now; }

  unsigned long need = g_pedalRaw ? PEDAL_DB_DOWN_MS : PEDAL_DB_UP_MS;

  bool rising = false, falling = false;
  if ((now - g_pedalLastChangeMs) >= need && g_pedalStable != g_pedalRaw) {
    g_pedalPrevStable = g_pedalStable;
    g_pedalStable = g_pedalRaw;
    rising  = (!g_pedalPrevStable && g_pedalStable);
    falling = ( g_pedalPrevStable && !g_pedalStable);
  }

  bool risingEff  = rising;
  bool fallingEff = falling && (now >= g_pedalIgnoreReleaseUntilMs);

  return { risingEff, fallingEff, rising, falling };

}

// Returns de directe pinstatus (LOW = pressed)
bool pedalRawNow() {
  return (digitalRead(PIN_PEDAL) == LOW);
}

// Detecteert raw flanken, klapt LED_BUILTIN mee en telt edges
void pedalRawEdgeService() {
  bool rawNow = pedalRawNow();
  g_pedalRawVis = rawNow;

  if (rawNow != g_pedalRawPrev) {
    g_pedalRawPrev = rawNow;
    g_pedalRawEdges++;
    // Onboard LED mee laten klappen: aan bij pressed (LOW), uit bij released (HIGH)
    digitalWrite(LED_BUILTIN, rawNow ? HIGH : LOW);
  }
}


// EEPROM / persist
uint32_t simpleChecksum32(const uint8_t* p, size_t n) {
  uint32_t s = 0; for (size_t i = 0; i < n; ++i) s = (s * 16777619u) ^ p[i]; return s;
}
inline void eepromReadRaw(int base, void* buf, size_t n) {
  uint8_t* b = (uint8_t*)buf; for (size_t i=0;i<n;++i) b[i] = EEPROM.read(base + (int)i);
}
inline void eepromWriteRaw(int base, const void* buf, size_t n) {
  const uint8_t* b = (const uint8_t*)buf; for (size_t i=0;i<n;++i) EEPROM.update(base + (int)i, b[i]);
}
inline void persistMarkDirty(){ g_persistDirty = true; }

void persistLoad() {
  Persist a, b; bool aValid=false, bValid=false;

  eepromReadRaw(EEPROM_SLOT_A, &a, sizeof(Persist));
  if (a.magic == PERSIST_MAGIC && a.version == PERSIST_VER) {
    uint32_t cs = simpleChecksum32((const uint8_t*)&a, sizeof(Persist) - sizeof(a.checksum));
    aValid = (cs == a.checksum);
  }
  eepromReadRaw(EEPROM_SLOT_B, &b, sizeof(Persist));
  if (b.magic == PERSIST_MAGIC && b.version == PERSIST_VER) {
    uint32_t cs = simpleChecksum32((const uint8_t*)&b, sizeof(Persist) - sizeof(b.checksum));
    bValid = (cs == b.checksum);
  }

  if (!aValid && !bValid) {
    memset(&g_persist, 0, sizeof(g_persist));
    g_persist.magic       = PERSIST_MAGIC;
    g_persist.version     = PERSIST_VER;
    g_persist.leadFwdCounts = STOP_LEAD_COUNTS_BASE;
    g_persist.leadRevCounts = STOP_LEAD_COUNTS_BASE;
    g_persist.lastTargetTurns10 = 100;
    g_persist.phaseAnchorT1000  = 0xFFFF;
    g_persist.phaseBiasCounts   = 0;
    g_persist.seq = 1;
    g_persist.checksum = simpleChecksum32((const uint8_t*)&g_persist, sizeof(Persist) - sizeof(g_persist.checksum));
    eepromWriteRaw(EEPROM_SLOT_A, &g_persist, sizeof(Persist));
    g_activeSlot = 0;
  } else {
    if (aValid && bValid) g_activeSlot = (b.seq > a.seq) ? 1 : 0;
    else g_activeSlot = aValid ? 0 : 1;
    g_persist = (g_activeSlot == 0) ? a : b;
  }

  g_leadPersistedFwd   = (long)g_persist.leadFwdCounts;
  g_leadPersistedRev   = (long)g_persist.leadRevCounts;
  g_leadLastSavedSkein = g_lifetimeSkeins;

  g_lifetimeSkeins     = g_persist.skeins;
  g_skeinSession       = 0;
  g_lastSavedSkeinMark = (g_lifetimeSkeins / SAVE_EVERY_SKEINS) * SAVE_EVERY_SKEINS;

  // Phase mirrors
  g_phaseAnchorT1000   = g_persist.phaseAnchorT1000;
  g_phaseBiasCounts    = g_persist.phaseBiasCounts;
  g_phErrHist[0]=g_phErrHist[1]=g_phErrHist[2]=0;
}

void persistSave() {
  g_persist.skeins        = g_lifetimeSkeins;
  g_persist.leadFwdCounts = (uint32_t)g_stopLeadFwd;
  g_persist.leadRevCounts = (uint32_t)g_stopLeadRev;
  g_persist.lastTargetTurns10 = (uint16_t) g_targetTurns10;

  // Phase
  g_persist.phaseAnchorT1000 = g_phaseAnchorT1000;
  g_persist.phaseBiasCounts  = g_phaseBiasCounts;

  g_persist.writeCount++;
  g_persist.seq = (g_persist.seq == 0xFFFFFFFFUL) ? 1UL : (g_persist.seq + 1UL);
  g_persist.checksum = simpleChecksum32((const uint8_t*)&g_persist, sizeof(Persist) - sizeof(g_persist.checksum));

  uint8_t nextSlot = (g_activeSlot == 0) ? 1 : 0;
  int base = (nextSlot == 0) ? EEPROM_SLOT_A : EEPROM_SLOT_B;
  eepromWriteRaw(base, &g_persist, sizeof(Persist));
  g_activeSlot = nextSlot;

  g_persistDirty = false;
  g_persistLastSaveMs = millis();
  g_lastSavedSkeinMark = (g_lifetimeSkeins / SAVE_EVERY_SKEINS) * SAVE_EVERY_SKEINS;
}

inline void creepControllerReset() {
  g_creepPwm   = PWM_CREEP;
  g_creepI_pwm = 0;
}

inline void creepControllerUpdate(int16_t rpm10_meas) {
  if (rpm10_meas == INT16_MIN) return;
  int16_t rpmAbs = (rpm10_meas < 0) ? -rpm10_meas : rpm10_meas;
  int16_t err    = CREEP_RPM10_TARGET - rpmAbs;
  if (err > -CREEP_RPM10_DEADBAND && err < CREEP_RPM10_DEADBAND) err = 0;

  // P-term
  long p = ((long)CREEP_KP_NUM * (long)err) / (long)CREEP_KP_DEN;

  // I-term: bevries bij grote fout (transient) om overshoot te beperken
  if (((err >= 0) ? err : -err) <= CREEP_I_FREEZE_ERR) {
    g_creepI_pwm += (int16_t)(((long)CREEP_KI_NUM * (long)err) / (long)CREEP_KI_DEN);
    if (g_creepI_pwm >  CREEP_I_CLAMP) g_creepI_pwm =  CREEP_I_CLAMP;
    if (g_creepI_pwm < -CREEP_I_CLAMP) g_creepI_pwm = -CREEP_I_CLAMP;
  }

  // Back-calculation anti-windup
  long unsat = (long)g_creepPwm + p + (long)g_creepI_pwm;
  long sat   = unsat;
  if (sat < CREEP_PWM_MIN) sat = CREEP_PWM_MIN;
  if (sat > CREEP_PWM_MAX) sat = CREEP_PWM_MAX;

  if (sat != unsat) {
    // Trek het verzadigingsverschil uit de I-term (beta=1)
    long corr = sat - unsat;           // negatief bij hoge kant
    g_creepI_pwm += (int16_t)corr;     // compenseer windup
  }

  // Rate-limit op uiteindelijke setpoint
  int next = (int)sat;
  int d    = next - g_creepPwmPrev;
  if (d >  CREEP_DPWM_MAX) d =  CREEP_DPWM_MAX;
  if (d < -CREEP_DPWM_MAX) d = -CREEP_DPWM_MAX;
  g_creepPwmPrev = g_creepPwm = (int)(g_creepPwmPrev + d);
}



// ================================================================
// [7] UI RENDERING (compact & debug)
// ================================================================
constexpr uint8_t LOGO_TOAST_W = 28;
constexpr uint8_t LOGO_TOAST_H = 24;

const uint8_t LOGO_SHEEP_28x24[] PROGMEM = {
	0x00,0x07,0x80,0x00,0x00,0x78,0x40,0x00,0x00,0x80,0x3c,0xc0,0x01,0x00,0x00,0x40,
	0x03,0x00,0x00,0x20,0x06,0x00,0x00,0x20,0x08,0x00,0x00,0x00,0x08,0x00,0x00,0x20,
	0x38,0x00,0x00,0x20,0x7e,0x00,0x00,0x20,0xff,0x00,0x00,0x20,0xcf,0x00,0x00,0x10,
	0xfe,0x00,0x00,0x10,0xfc,0x00,0x00,0x10,0x7c,0x00,0x00,0x20,0x3c,0x00,0x00,0xc0,
	0x1c,0x00,0x00,0x80,0x04,0x00,0x10,0x80,0x03,0x84,0x3f,0x00,0x01,0xfe,0x6e,0x00,
	0x01,0xc1,0xce,0x00,0x07,0xc0,0x3f,0x00,0x0f,0xc0,0x3f,0x00,0x04,0x80,0x00,0x00
};

void drawBitmapHFlip(int16_t x,int16_t y,const uint8_t* bm,int16_t w,int16_t h,uint16_t color){
  const int16_t bpr=(w+7)/8;
  for(int16_t r=0;r<h;++r){
    const uint8_t* rp=bm+(r*bpr);
    for(int16_t c=0;c<w;++c){
      int16_t sc=w-1-c;
      uint8_t b=pgm_read_byte(rp+(sc>>3));
      uint8_t m=(uint8_t)(0x80>>(sc&7));
      if(b&m) display.drawPixel(x+c,y+r,color);
    }
  }
}
int16_t textWidthPxC(const char* s,uint8_t sz=1){ return (int16_t)strlen(s)*6*sz; }
void drawCenteredC(const char* s,int16_t y,uint8_t sz=1){
  int16_t x=(SCREEN_WIDTH - textWidthPxC(s,sz))/2; if(x<0)x=0;
  display.setTextSize(sz); display.setCursor(x,y); display.print(s);
}
void drawRightC(const char* s,int16_t y,uint8_t sz=1){
  int16_t x=SCREEN_WIDTH - textWidthPxC(s,sz); if(x<0)x=0;
  display.setTextSize(sz); display.setCursor(x,y); display.print(s);
}
static inline bool stateIsIdle(RunState st){ return st == RunState::IDLE; }

void drawTopRowDynamic(uint32_t skeins, RunState st, Mode m, int16_t rpm10) {
  display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0); display.print(skeins); display.print(F(" skeins"));
  if (stateIsIdle(st)) {
    drawRightC((m==Mode::MANUAL)?"MANUAL":"AUTO",0,1);
  } else {
    int rpm=(rpm10==INT16_MIN)?-1:((rpm10<0?-rpm10:rpm10)+5)/10;
    char rb[20]; if(rpm<0) snprintf_P(rb,sizeof(rb),PSTR("RPM --")); else snprintf_P(rb,sizeof(rb),PSTR("RPM %d"),rpm);
    drawRightC(rb,0,1);
  }
}
static uint8_t chooseBigSizeForTurns10(long t10,bool showDecimal){
  long abs10=(t10<0)?-t10:t10; long whole=abs10/10; int digits=1; for(long x=whole;x>=10;x/=10)digits++;
  int extra=((t10<0)?1:0)+(showDecimal?2:0); int total=digits+extra; return (total<=5)?4:3;
}
void drawBigTwists(long t10,int16_t yTop,bool showUnit=true){
  bool showDecimal=false; uint8_t sz=chooseBigSizeForTurns10(t10,showDecimal);
  char buf[16];
  if(showDecimal){ long a=(t10<0)?-t10:t10; snprintf_P(buf,sizeof(buf),PSTR("%s%ld.%ld"),(t10<0)?"-":"",a/10,a%10); }
  else           { long v=(t10<0)?-t10:t10; long whole=v/10; snprintf_P(buf,sizeof(buf),PSTR("%s%ld"),(t10<0)?"-":"",whole); }
  drawCenteredC(buf,yTop,sz);
  if(showUnit){ int16_t unitY=yTop+(8*sz)+2; drawCenteredC("twists",unitY,1); }
}
constexpr int16_t PROG_X=2,PROG_Y=56,PROG_W=124,PROG_H=8;
void drawProgressBarBottom(long goalCountsLocal,long encLocal){
  if(goalCountsLocal<=0)return; long progressCounts=labs(encLocal);
  long p1000=(progressCounts*1000L+goalCountsLocal/2)/goalCountsLocal; if(p1000<0)p1000=0; if(p1000>1000)p1000=1000;
  display.drawRect(PROG_X,PROG_Y,PROG_W,PROG_H,SSD1306_WHITE);
  int16_t innerW=PROG_W-2; int16_t fillW=(int16_t)((innerW*p1000+500L)/1000L);
  if(fillW>0) display.fillRect(PROG_X+1,PROG_Y+1,fillW,PROG_H-2,SSD1306_WHITE);
}
bool drawToastIfAny() {
  if (millis() >= g_toast_untilMs) return false;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Count lines (supports up to 3 lines; centered vertically)
  uint8_t lines = 1;
  for (const char* p = g_toast_buf; *p; ++p) if (*p == '\n') ++lines;
  if (lines > 3) lines = 3;

  uint8_t sz = g_toast_size;
  int lineH = 8 * sz;
  int vGap  = 2;
  int totalH = (int)lines * lineH + (int)(lines - 1) * vGap;
  int16_t y = (SCREEN_HEIGHT - totalH) / 2;

  // Render each line
  const char* s = g_toast_buf;
  for (uint8_t i = 0; i < lines; ++i) {
    const char* e = s; while (*e && *e != '\n') ++e;

    char lineBuf[22];
    size_t len = (size_t)(e - s);
    if (len >= sizeof(lineBuf)) len = sizeof(lineBuf) - 1;
    memcpy(lineBuf, s, len);
    lineBuf[len] = '\0';

    if (i == 0 && g_toastWithIcons) {
      // Text metrics
      int16_t textW = textWidthPxC(lineBuf, sz);
      int16_t textH = lineH;

      // Icon geometry (fixed at screen edges)
      const int16_t iconW = LOGO_TOAST_W;
      const int16_t iconH = LOGO_TOAST_H;
      const int16_t margin = 1;
      const int16_t gap    = 4;

      const int16_t xLeftIcon  = margin;
      const int16_t xRightIcon = SCREEN_WIDTH - margin - iconW;
      int16_t yIcon = y + (textH - iconH) / 2; if (yIcon < 0) yIcon = 0;

      const int16_t innerX0 = xLeftIcon + iconW + gap;
      const int16_t innerX1 = xRightIcon - gap;
      int16_t innerW = innerX1 - innerX0; if (innerW < 0) innerW = 0;

      int16_t xText = (textW <= innerW) ? innerX0 + (innerW - textW) / 2 : innerX0;

      drawBitmapHFlip(xLeftIcon,  yIcon, LOGO_SHEEP_28x24, iconW, iconH, SSD1306_WHITE);
      display.setTextSize(sz); display.setCursor(xText, y); display.print(lineBuf);
      display.drawBitmap(xRightIcon, yIcon, LOGO_SHEEP_28x24, iconW, iconH, SSD1306_WHITE);
    } else {
      int16_t x = (SCREEN_WIDTH - textWidthPxC(lineBuf, sz)) / 2; if (x < 0) x = 0;
      display.setTextSize(sz); display.setCursor(x, y); display.print(lineBuf);
    }

    y += lineH + vGap;
    s = (*e == '\n') ? (e + 1) : e;
  }

  display.display();
  return true;
}

void drawUI_ManualSimple(int16_t rpm10,long /*encCounts*/){
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  drawTopRowDynamic(g_skeinSession,g_state,g_mode,rpm10);
  long shownT10=counts_to_turns10(g_manualDispCounts);
  drawBigTwists(shownT10,20,true);
  display.display();
}
void drawUI_AutoSimple(int16_t rpm10,RunState st,long goalCountsLocal,long encLocal,int16_t targetTurns10Local){
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  drawTopRowDynamic(g_skeinSession,st,g_mode,rpm10);
  long centerT10;
  if(st==RunState::IDLE || goalCountsLocal==0){
    centerT10 = targetTurns10Local; drawBigTwists(centerT10,20,true);
  }else{
    long remaining=goalCountsLocal-labs(encLocal); if(remaining<0) remaining=0;
    centerT10 = counts_to_turns10(remaining); drawBigTwists(centerT10,20,false);
  }
  if((st!=RunState::IDLE) && (goalCountsLocal>0)) drawProgressBarBottom(goalCountsLocal,encLocal);
  display.display();
}
void drawUI_Debug(int16_t rpm10,long pulses,long turns10,int pwm,bool pedalPressed,RunState st,Mode m,long tgt10){
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE); display.setTextSize(1);
  display.setCursor(0,0); display.print(F("Mode: ")); display.print(m==Mode::MANUAL?F("FOOT"):F("AUTO"));
  snprintf_P(g_small_buf,sizeof(g_small_buf),PSTR("Dir: %s"), g_dirForward?"FWD":"REV"); drawRightC(g_small_buf,0,1);
//  display.setCursor(0,10); display.print(F("Pedal: ")); display.print(pedalPressed?F("ON"):F("off"));
  display.setCursor(0,10);
  display.print(F("Pedal: "));
  display.print(pedalPressed ? F("ON") : F("off"));  // debounced (g_pedalStable)

  // Rechts: RAW en edge-counter
  snprintf_P(g_small_buf, sizeof(g_small_buf),
             PSTR("RAW:%c  E:%lu"),
             g_pedalRawVis ? 'L' : 'H',
             (unsigned long)g_pedalRawEdges);
  drawRightC(g_small_buf, 10, 1);

//  snprintf_P(g_small_buf,sizeof(g_small_buf),PSTR("RPM: %d"), (rpm10==INT16_MIN)?0:(rpm10/10)); drawRightC(g_small_buf,10,1);
  display.setCursor(0,20); display.print(F("Enc: ")); display.print(pulses);
  snprintf_P(g_small_buf,sizeof(g_small_buf),PSTR("Turns: %ld.%ld"), turns10/10, labs(turns10)%10); drawRightC(g_small_buf,20,1);
  display.setCursor(0,30); display.print(F("PWM: ")); display.print(pwm);
  display.setCursor(0,40); display.print(F("State: ")); switch(st){
    case RunState::IDLE: display.print(F("IDLE")); break;
    case RunState::RAMP_UP: display.print(F("RUP")); break;
    case RunState::RUN: display.print(F("RUN")); break;
    case RunState::RAMP_DOWN: display.print(F("RDN")); break;
  }
  {
    // Phase anchor/phase/last error+step on a single compact line
    uint16_t phNow = phaseFromAbsCountsT1000(encAbsAtomicRead());
    display.setCursor(0, 48);
 //   if (g_phaseAnchorT1000 == 0xFFFF) {
      // anchor unset
      snprintf_P(g_small_buf, sizeof(g_small_buf),
                PSTR("P:%3u  e:%+d  s:%+d"),
                phNow, g_dbgLastPhaseErrT1000, g_dbgLastAnchorStepT1000);
/**    } else {
      snprintf_P(g_small_buf, sizeof(g_small_buf),
                PSTR("A:%3u P:%3u e:%+d s:%+d"),
                g_phaseAnchorT1000, phNow,
                g_dbgLastPhaseErrT1000, g_dbgLastAnchorStepT1000);
    } */
    display.print(g_small_buf);
  } 

  long t1000F = counts_to_turns1000(g_stopLeadFwd);
  long t1000R = counts_to_turns1000(g_stopLeadRev);
  long aF = (t1000F<0)?-t1000F:t1000F;
  long aR = (t1000R<0)?-t1000R:t1000R;
  display.setCursor(0,56);
  snprintf_P(g_small_buf,sizeof(g_small_buf),PSTR("Leads: %ld.%03ldF / %ld.%03ldR"),
             aF/1000,aF%1000,aR/1000,aR%1000);
  display.print(g_small_buf);
  display.display();
}
void uiLifetimeSplashTopRightLogo(){
  unsigned long t0=millis();
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE); display.setTextSize(1);
  const int16_t xLogo=SCREEN_WIDTH-LOGO_TOAST_W-1, yLogo=1;
  display.drawBitmap(xLogo,yLogo,LOGO_SHEEP_28x24,LOGO_TOAST_W,LOGO_TOAST_H,SSD1306_WHITE);
  long lifetimeTurns10=counts_to_turns10((long)g_persist.totalCounts);
  unsigned long long ms=g_persist.runtimeMs; unsigned long totalSec=(unsigned long)(ms/1000ULL);
  unsigned long hh=totalSec/3600UL; unsigned long mm=(totalSec%3600UL)/60UL;
  display.setCursor(0,0);  display.println(F("SkeinMachine v3.19"));
  display.setCursor(0,12); display.print(F("Skeins: ")); display.println(g_lifetimeSkeins);
  display.setCursor(0,22); display.print(F("Twists: ")); display.print(lifetimeTurns10/10);
  display.setCursor(0,32); display.print(F("Stalls: ")); display.println(g_persist.stallCount);
  display.setCursor(0,42); display.print(F("Runtime: ")); display.print(hh); display.print(F("h "));
                                                   display.print(mm); display.println(F("m"));
  display.setCursor(0,52); display.print(F("EEPWrites: ")); display.print(g_persist.writeCount);
  display.display();
  while(millis()-t0 < LIFETIME_SPLASH_MS) { /* idle */ }
  display.clearDisplay(); display.display();
}


// ================================================================
// [8] FINITE STATE MACHINE
// ================================================================
void updateFSM(const PedalEdges& ped, unsigned long now) {
  // Lifetime integrators
  long encNowForLife = encAtomicRead();
  long diff = encNowForLife - g_lastEncForLife; if (diff != 0) { if (diff < 0) diff = -diff; g_persist.totalCounts += (uint64_t)diff; g_lastEncForLife = encNowForLife; }
  g_persist.runtimeMs += (unsigned long)(now - g_lastLoopMs); g_lastLoopMs = now;

  // Manual mode
  if (g_mode == Mode::MANUAL) {
    long encNow = encAtomicRead(); (void)encNow;

    if (ped.risingEff) {
      g_state = RunState::RAMP_UP; g_stateStartMs = now;
      g_startDuty = 0;
      digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);
      encAtomicWrite(0); g_manualRefEnc = 0;
      g_pedalIgnoreReleaseUntilMs = now + PEDAL_RELEASE_IGNORE_MS;
      g_manualDispCounts = 0;
    } else if (ped.fallingEff) {
      g_state = RunState::RAMP_DOWN; g_stateStartMs = now; g_startDuty = g_pwmNow;
    }

    switch (g_state) {
      case RunState::RAMP_UP: {
        g_manualDispCounts = encAtomicRead();
        digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);
        unsigned long elapsed = now - g_stateStartMs;
        if (elapsed <= START_KICK_MS) {
          motorAnalogWrite(START_KICK_PWM);
        } else {
          if (elapsed >= rampUpMs) { motorAnalogWrite(PWM_FAST); g_state = RunState::RUN; }
          else { int duty=(int)(((long)PWM_FAST*(long)elapsed)/(long)rampUpMs); motorAnalogWrite(duty); }
        }
      } break;

      case RunState::RUN: {
        g_manualDispCounts = encAtomicRead();
        digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);
        if (ped.fallingEff) { g_state = RunState::RAMP_DOWN; g_stateStartMs = now; g_startDuty = g_pwmNow; break; }
        motorAnalogWrite(PWM_FAST);
      } break;

      case RunState::RAMP_DOWN: {
        g_manualDispCounts = encAtomicRead();
        digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);
        unsigned long elapsed = now - g_stateStartMs;
        bool finishNow=false; long duty;
        if (elapsed >= rampDownMs) finishNow = true;
        else { duty = g_startDuty - ((long)g_startDuty*(long)elapsed)/(long)rampDownMs; if (duty < MIN_RAMP_PWM) finishNow = true; }
        if (finishNow) {
          motorStopHard();
          long encNow2 = encAtomicRead();
          g_manualRefEnc = encNow2; g_jogTargetRel = 0;
          g_manualSettleUntilMs = now + 120;
          g_state = RunState::IDLE;
          /*
          // ---- Slow anchor adaptation (no effect on goal counts) ----
          if (g_phaseAnchorT1000 != 0xFFFF) {
            uint16_t phNow = phaseFromAbsCountsT1000(encAbsAtomicRead());
            int16_t  e     = phaseErrorT1000(phNow, g_phaseAnchorT1000); // [-500..+500] mturn

            // stap = clamp( round(e * NUM / DEN), ±PH_ANCHOR_MAX_STEP )
            int16_t step = (int16_t)(( (long)e * PH_ANCHOR_ADAPT_NUM + (PH_ANCHOR_ADAPT_DEN/2) )
                                    / PH_ANCHOR_ADAPT_DEN);
            if (step >  (int16_t)PH_ANCHOR_MAX_STEP)  step =  (int16_t)PH_ANCHOR_MAX_STEP;
            if (step < -(int16_t)PH_ANCHOR_MAX_STEP)  step = -(int16_t)PH_ANCHOR_MAX_STEP;

            g_dbgLastPhaseErrT1000   = e;
            g_dbgLastAnchorStepT1000 = step;

            g_phaseAnchorT1000 = wrapPhase1000((int16_t)g_phaseAnchorT1000 + step);
            persistMarkDirty();  // mag meeschrijven met de normale policy
          } */

        } else motorAnalogWrite((int)duty);
      } break;

      case RunState::IDLE:
      default: {
        motorAnalogWrite(0);
        if (g_manualSettleUntilMs == 0 || now >= g_manualSettleUntilMs) {
          long encNowX = encAtomicRead();
          long targetEnc = g_manualRefEnc + g_jogTargetRel;
          long err = targetEnc - encNowX;
          if (err > JOG_TOL) { digitalWrite(PIN_DIR, HIGH); motorAnalogWrite(PWM_JOG); }
          else if (err < -JOG_TOL) { digitalWrite(PIN_DIR, LOW); motorAnalogWrite(PWM_JOG); }
          else { motorAnalogWrite(0); }
        } else motorAnalogWrite(0);
      } break;
    }
    return;
  }

  // AUTO mode
  switch (g_state) {
    case RunState::IDLE: {
      if (ped.risingEff) {
        // showToastC(F("PEDAL START"), 400, 2);   // <— tijdelijk
        digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);
        if (g_targetTurns10 > 0) {
          encAtomicWrite(0); g_lastEncForLife = 0;

          // Basisdoel + langzame fase-bias
          /*
          long baseCounts = turns10_to_counts(g_targetTurns10);
          long bias = g_phaseBiasCounts;
          if (bias >  PH_BIAS_MAX_COUNTS) bias =  PH_BIAS_MAX_COUNTS;
          if (bias < -PH_BIAS_MAX_COUNTS) bias = -PH_BIAS_MAX_COUNTS;
          g_goalCounts = baseCounts + bias;
          */
          // Safety: ignore phase-bias for now to prevent 3× target jump
          g_goalCounts = turns10_to_counts(g_targetTurns10);


          g_rpm_lastCounts = encAtomicRead(); g_rpm_lastMs = now;
          g_rpm_lastValid10 = INT16_MIN;

          long totalCounts = g_goalCounts;
          bool smallTarget = (totalCounts <= (APPROACH_COUNTS + STOP_LEAD_COUNTS_BASE + 309));
          if (smallTarget) { g_inCreepBand = true;  g_pwmTarget = PWM_CREEP; g_desiredPwmTarget = PWM_CREEP; }
          else             { g_inCreepBand = false; g_pwmTarget = PWM_FAST;  g_desiredPwmTarget = PWM_FAST;  }

          g_creepPwmPrev = g_creepPwm;
          creepControllerReset();  

          g_lastSlewMs = now; g_countedThisRun = false; g_rampDownPlanned = false;
          g_state = RunState::RAMP_UP; g_stateStartMs = now; motorAnalogWrite(0);
        }
      }
    } break;

    case RunState::RAMP_UP: {
      if (ped.risingEff) { g_startDuty = g_pwmNow; g_state = RunState::RAMP_DOWN; g_stateStartMs = now; g_rampDownPlanned = false; break; }
      unsigned long elapsed = now - g_stateStartMs;
      if (elapsed >= rampUpMs) { updatePwmSlew(now); motorAnalogWrite(g_pwmTarget); g_state = RunState::RUN; updatePwmSlew(now); }
      else {
        int duty = (int)(((long)g_pwmTarget * (long)elapsed) / (long)rampUpMs);
        motorAnalogWrite(duty);
      }
    } break;

    case RunState::RUN: {
      if (ped.risingEff) { g_startDuty = g_pwmNow; g_state = RunState::RAMP_DOWN; g_stateStartMs = now; g_rampDownPlanned = false; break; }

      long encNow = encAtomicRead();
      long progress = labs(encNow); long error = g_goalCounts - progress;
      long stopLeadNow = g_dirForward ? g_stopLeadFwd : g_stopLeadRev; (void)stopLeadNow;

      long approachEnter = APPROACH_COUNTS;
      long approachExit  = APPROACH_COUNTS + APPROACH_HYST_COUNTS;
      if (!g_inCreepBand) {
        if (error <= approachEnter) { g_inCreepBand = true;  g_desiredPwmTarget = PWM_CREEP; } else { g_desiredPwmTarget = PWM_FAST; }
      } else {
        if (error > approachExit)   { g_inCreepBand = false; g_desiredPwmTarget = PWM_FAST;  } else { g_desiredPwmTarget = PWM_CREEP; }
      }

      // Slew-keuze voor deze iteratie
      if (g_inCreepBand) {
        g_slewDurationMs = CREEP_SLEW_MS;
        if (CREEP_BYPASS_SLEW_WHEN_RAISING && g_desiredPwmTarget > g_pwmTarget) {
          // Direct omhoog; omlaag nog wel via korte slew
          g_pwmTarget = g_desiredPwmTarget;
          // reset startpunt voor eventuele volgende daling
          g_slewFrom    = g_pwmTarget;
          g_slewStartMs = now;
          g_prevDesired = g_desiredPwmTarget;
        }
      } else {
        g_slewDurationMs = CREEP_TRANSITION_MS;
      }

      // --- Detecteer overgang naar/uit creep ---
      bool enteringCreep = (g_inCreepBand && !g_wasInCreepBand);
      bool leavingCreep  = (!g_inCreepBand && g_wasInCreepBand);

      // Bij binnenkomst in creep: reset de regelaar en voorkom een duty-dip
      if (enteringCreep) {
        creepControllerReset();

        // Start het regeldoel op (ongeveer) de huidige PWM, zodat er geen inzinking is
        int minStart = (CREEP_ENTER_MIN_PWMDROP > 0)
                        ? (g_pwmNow - CREEP_ENTER_MIN_PWMDROP)
                        : g_pwmNow;
        if (minStart < CREEP_PWM_MIN) minStart = CREEP_PWM_MIN;
        if (minStart > CREEP_PWM_MAX) minStart = CREEP_PWM_MAX;

        g_creepPwm = minStart;
        g_creepPwmPrev = g_creepPwm;

        if (g_desiredPwmTarget < g_creepPwm) g_desiredPwmTarget = g_creepPwm;

        // Herstart de slew vanaf het actuele target om sneller te reageren
        g_slewFrom    = g_pwmTarget;
        g_slewStartMs = now;
        g_prevDesired = -1;  // forceer updatePwmSlew() om nieuwe desired te volgen
      }

      // Tijdens creep elke iteratie de RPM-regelaar laten sturen
      if (g_inCreepBand) {
        creepControllerUpdate(g_rpm_lastValid10);
        if (g_desiredPwmTarget < g_creepPwm) g_desiredPwmTarget = g_creepPwm;
      }

      // Bijhouden voor volgende iteratie
      g_wasInCreepBand = g_inCreepBand;

      // --- PUNT 5: RPM-gestuurde creep ---
      if (g_inCreepBand) {
        // Gebruik meest recente geldige RPM; val terug op deze iteratie
        creepControllerUpdate(g_rpm_lastValid10);
          // laat desired niet onder het geregelde creep-niveau zakken
        if (g_desiredPwmTarget < g_creepPwm) g_desiredPwmTarget = g_creepPwm;
      }

      if (error <= stopLeadNow && error > 0) {
        if (!g_countedThisRun) {
          g_countedThisRun = true; g_skeinSession++; g_lifetimeSkeins++;
          if (!g_uiDebug) showSkeinToast(g_skeinSession, 2000);
          if (g_lifetimeSkeins >= g_lastSavedSkeinMark + SAVE_EVERY_SKEINS) persistSave();
        }
        g_rampDownPlanned = true;
        g_encAtRampDown = progress;
        g_startDuty = g_pwmNow;
        g_state = RunState::RAMP_DOWN; g_stateStartMs = now;
      }
      else if (error <= 0) {
        motorAnalogWrite(0); g_state = RunState::IDLE;
      }
      else {
        updatePwmSlew(now); motorAnalogWrite(g_pwmTarget);
      }
    } break;

    case RunState::RAMP_DOWN: {
      digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);
      unsigned long elapsed = now - g_stateStartMs;

      bool finishNow = false; long duty;
      if (elapsed >= rampDownMs) finishNow = true;
      else {
        duty = g_startDuty - ((long)g_startDuty * (long)elapsed) / (long)rampDownMs;
        if (duty < MIN_RAMP_PWM) finishNow = true;
      }

      if (finishNow) {
        motorStopHard();
        if (g_rampDownPlanned) {
          g_settleUntilMs = now + (RPM_WINDOW_MS + 100);
          g_adaptPending  = true;
          g_rampDownPlanned = false;
        }
        g_state = RunState::IDLE;

        // ---- Phase nudge update at STOP ----
        if (g_phaseAnchorT1000 != 0xFFFF) {
          uint16_t phNow = phaseFromAbsCountsT1000(encAbsAtomicRead());
          int16_t e = phaseErrorT1000(phNow, g_phaseAnchorT1000); // mturns
          // push into history
          g_phErrHist[g_phErrIdx] = e; g_phErrIdx = (uint8_t)((g_phErrIdx + 1) % 3);
          int16_t em = median3_i16(g_phErrHist[0], g_phErrHist[1], g_phErrHist[2]);

          int16_t ae = (em<0)?-em:em;
          if (ae > (int16_t)PH_DB_T1000) {
            // step ~ linear with |error|, capped
            // Map |em| (PH_DB_T1000..500) → (1..PH_STEP_MAX_COUNTS)
            int32_t num = (int32_t)(ae - (int16_t)PH_DB_T1000) * (int32_t)PH_STEP_MAX_COUNTS +  (500- (int16_t)PH_DB_T1000)/2;
            int16_t step = (int16_t)(num / (500 - (int16_t)PH_DB_T1000));
            if (step < 1) step = 1;
            if (step > (int16_t)PH_STEP_MAX_COUNTS) step = (int16_t)PH_STEP_MAX_COUNTS;
            if (em < 0) step = -step;
            long nb = g_phaseBiasCounts + (long)step;

            // small leak toward zero to avoid accumulation
            if (PH_LEAK_DEN != 0) {
              long leak = (nb >= 0) ? (nb / PH_LEAK_DEN) : - ((-nb) / PH_LEAK_DEN);
              nb -= leak;
            }

            if (nb >  (long)PH_BIAS_MAX_COUNTS) nb =  (long)PH_BIAS_MAX_COUNTS;
            if (nb < -(long)PH_BIAS_MAX_COUNTS) nb = -(long)PH_BIAS_MAX_COUNTS;
            g_phaseBiasCounts = nb;
            persistMarkDirty();

            // Reset phase bias to avoid compounding until we re-enable nudger later
            g_phaseBiasCounts = 0;

          }
        }
      } else {
        motorAnalogWrite((int)duty);
      }
    } break;
  }
}


// ================================================================
// [9] SETUP & LOOP
// ================================================================
void setup() {

 
  Wire.begin();
  Wire.setClock(400000); // 400 kHz
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    pinMode(LED_BUILTIN, OUTPUT);
    while (true) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(100); }
  }
  display.setRotation(OLED_ROTATION); display.clearDisplay(); display.display();

  persistLoad(); g_lastLoopMs = millis(); g_persistLastSaveMs = millis();

  #if MAINT_RESET_ON_BOOT
    // ======= MAINTENANCE: update counters (adjust values below) =======
    const uint32_t NEW_SKEINS = 100;
    const uint32_t NEW_STALLS = 0;
    g_persist.skeins     = NEW_SKEINS;
    g_persist.stallCount = NEW_STALLS;
    g_lifetimeSkeins     = NEW_SKEINS;
    g_skeinSession       = 0;
    g_lastSavedSkeinMark = (g_lifetimeSkeins / SAVE_EVERY_SKEINS) * SAVE_EVERY_SKEINS;
    g_phaseAnchorT1000   = 0xFFFF;
    g_phaseBiasCounts    = 0;
    persistMarkDirty(); persistSave();
    showToastC(F("EEP updated"), 900, 2);
  #endif

  #if TOAST_TEST_MODE
    pinMode(A0, INPUT);
    randomSeed( (unsigned long)analogRead(A0) ^ (unsigned long)micros() );
  #endif

  // Load user prefs
  g_targetTurns10 = g_persist.lastTargetTurns10;
  g_stopLeadFwd = (g_persist.leadFwdCounts >= (uint32_t)STOPLEAD_MIN && g_persist.leadFwdCounts <= (uint32_t)STOPLEAD_MAX)
                      ? (long)g_persist.leadFwdCounts : STOP_LEAD_COUNTS_BASE;
  g_stopLeadRev = (g_persist.leadRevCounts >= (uint32_t)STOPLEAD_MIN && g_persist.leadRevCounts <= (uint32_t)STOPLEAD_MAX)
                      ? (long)g_persist.leadRevCounts : STOP_LEAD_COUNTS_BASE;

  // Splash
  uiLifetimeSplashTopRightLogo();

  // I/O
  pinMode(PIN_PEDAL, INPUT_PULLUP);
  pinMode(PIN_DIR, OUTPUT); pinMode(PIN_PWM, OUTPUT);
  digitalWrite(PIN_DIR, HIGH); g_dirForward = true;

  // pedal debug led
  pinMode(LED_BUILTIN, OUTPUT);
  g_pedalRawPrev = pedalRawNow();              // init met actuele raw staat
  digitalWrite(LED_BUILTIN, g_pedalRawPrev ? HIGH : LOW);

  // Timer1: Fast PWM 8-bit on OC1A (D9), non-inverting, prescaler=1 (~31 kHz)
  TCCR1A = (1 << WGM10) | (1 << COM1A1);
  TCCR1B = (1 << WGM12) | (1 << CS10);

  // Motor encoder
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  { uint8_t a = digitalRead(ENC_A), b = digitalRead(ENC_B); g_enc_prevState = (a << 1) | b; }
  attachInterrupt(digitalPinToInterrupt(ENC_A), ISR_encA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), ISR_encB, CHANGE);

  // KY-040 (own decoder via PCINT)
  pinMode(ROT_CLK, INPUT_PULLUP);
  pinMode(ROT_DT,  INPUT_PULLUP);
  { uint8_t ra = (PIND >> 7) & 0x01; uint8_t rb = (PIND >> 6) & 0x01; g_rot_prevAB = (ra << 1) | rb; }
  pinMode(ROT_SW, INPUT_PULLUP); g_sw_state = digitalRead(ROT_SW);
  pciSetup(ROT_CLK); pciSetup(ROT_DT);

  motorStopHard(); g_state = RunState::IDLE;

  long ec = encAtomicRead();
  drawUI_AutoSimple(g_rpm_lastValid10, g_state, g_goalCounts, ec, g_targetTurns10);
}

void loop() {

  pedalRawEdgeService();


  rotaryServiceFromQueue();
  unsigned long now = millis();

  // Pedal edges
  PedalEdges ped = pedalService();

/*
  #if DEBUG_SERIAL
    debugPedalLog(ped);
  #endif
*/ 

  // FSM
  updateFSM(ped, now);

  // RPM sample
  int16_t rpm10 = rpm10SinceWindow(); if (rpm10 != INT16_MIN) g_rpm_lastValid10 = rpm10;
  long pulses = encAtomicRead();

  // Stall detection (alleen in "echt rijden", niet in creep of ramp-down)
  if (g_state == RunState::RUN && rpm10 != INT16_MIN) {
    // Optioneel: negeer de eerste STALL_MIN_RUN_MS in RUN om valse triggers te voorkomen
    bool runLongEnough = (millis() - g_stateStartMs) >= STALL_MIN_RUN_MS;

    int16_t absrpm10 = (rpm10 < 0) ? -rpm10 : rpm10;

    bool allowStall =
      runLongEnough &&
      (g_pwmNow >= STALL_MIN_PWM) &&
      (!STALL_DISABLE_IN_CREEP || !g_inCreepBand);

    if (allowStall && absrpm10 < STALL_RPM10_THRESH && !g_stallLatched) {
      g_stallLatched = true;
      showToastC(F("STALL"), 900, 2);
      g_persist.stallCount++;
      persistMarkDirty();

      // graceful stop
      g_startDuty = g_pwmNow;
      g_state = RunState::RAMP_DOWN;
      g_stateStartMs = millis();
      g_rampDownPlanned = false; // dit was geen geplande stop
    }
  }

  // Reset latch zodra we echt stilstaan
  if (g_state == RunState::IDLE) g_stallLatched = false;

  // Adaptive lead update after settling
  if (g_adaptPending && millis() >= g_settleUntilMs) {
    g_adaptPending = false;

    // ---- (1) Bestaande decel/lead adaptatie (ongewijzigd) ----
    long encAfterAbs = labs(encAtomicRead());
    long decelMeasured = encAfterAbs - g_encAtRampDown; if (decelMeasured < 0) decelMeasured = 0;
    long* pLead = g_dirForward ? &g_stopLeadFwd : &g_stopLeadRev;
    long delta   = decelMeasured - *pLead;
    long adjust  = (delta * STOPLEAD_ADAPT_K_NUM + (STOPLEAD_ADAPT_K_DEN/2)) / STOPLEAD_ADAPT_K_DEN; // ~0.2*delta
    long newLead = *pLead + adjust;
    if (newLead < STOPLEAD_MIN) newLead = STOPLEAD_MIN; if (newLead > STOPLEAD_MAX) newLead = STOPLEAD_MAX;
    *pLead = newLead;
    long persisted = g_dirForward ? g_leadPersistedFwd : g_leadPersistedRev;
    bool farFromPersisted = (labs(newLead - persisted) >= LEAD_PERSIST_EPS_COUNTS);
    bool enoughRuns       = ((g_lifetimeSkeins - g_leadLastSavedSkein) >= LEAD_PERSIST_MIN_RUNS);
    if (farFromPersisted && enoughRuns) { persistMarkDirty(); }

    // ---- (2) Slow anchor adaptation (na settle) ----
    int16_t e = 0;           // laatste fasefout (mturn)
    int16_t step = 0;        // laatste ankerstap (mturn)
    if (g_phaseAnchorT1000 != 0xFFFF) {
      uint16_t phNow = phaseFromAbsCountsT1000(encAbsAtomicRead());
      e = phaseErrorT1000(phNow, g_phaseAnchorT1000);         // [-500..+500] mturn
      int16_t ae = (e >= 0) ? e : -e;

      if (ae > (int16_t)PH_ANCHOR_DB_T1000) {
        // min 1 mturn, lineair geschaald, geclamped
        long num    = (long)(ae - (int16_t)PH_ANCHOR_DB_T1000) * PH_ANCHOR_ADAPT_NUM + (PH_ANCHOR_ADAPT_DEN/2);
        int16_t mag = (int16_t)(num / PH_ANCHOR_ADAPT_DEN);
        if (mag < (int16_t)PH_ANCHOR_MIN_STEP) mag = (int16_t)PH_ANCHOR_MIN_STEP;
        if (mag > (int16_t)PH_ANCHOR_MAX_STEP) mag = (int16_t)PH_ANCHOR_MAX_STEP;
        step = (e >= 0) ? mag : -mag;

        g_phaseAnchorT1000 = wrapPhase1000((int16_t)g_phaseAnchorT1000 + step);
        persistMarkDirty();
      }
    }
    // Debugwaarden altijd bijwerken
    g_dbgLastPhaseErrT1000   = e;
    g_dbgLastAnchorStepT1000 = step;

    // ---- (3) Phase→Lead micro-correctie (na settle) ----
    if (g_phaseAnchorT1000 != 0xFFFF) {
      uint16_t phNow2 = phaseFromAbsCountsT1000(encAbsAtomicRead());
      int16_t  e2     = phaseErrorT1000(phNow2, g_phaseAnchorT1000); // [-500..+500] mturn
      int16_t  ae2    = (e2 >= 0) ? e2 : -e2;

      if (ae2 > (int16_t)PH_LEAD_DB_T1000) {
        // mturn → counts: 1 mturn = turns1000_to_counts(1)
        long cpm = turns1000_to_counts(1);
        long num2 = (long)(ae2) * cpm * PH_LEAD_GAIN_NUM;
        long mag2 = (num2 + (PH_LEAD_GAIN_DEN/2)) / PH_LEAD_GAIN_DEN;  // |Δlead| in counts
        if (mag2 < (long)PH_LEAD_MIN_STEP_COUNTS) mag2 = PH_LEAD_MIN_STEP_COUNTS;
        if (mag2 > (long)PH_LEAD_MAX_STEP_COUNTS) mag2 = PH_LEAD_MAX_STEP_COUNTS;

        long dlead = (e2 >= 0) ? mag2 : -mag2;  // e>0 ⇒ later gestopt ⇒ lead groter
        long* pL = g_dirForward ? &g_stopLeadFwd : &g_stopLeadRev;
        long nl = *pL + dlead;
        if (nl < STOPLEAD_MIN) nl = STOPLEAD_MIN; if (nl > STOPLEAD_MAX) nl = STOPLEAD_MAX;
        *pL = nl;

        persistMarkDirty();
        // optioneel: g_dbgLastLeadDelta = (int16_t)dlead;
      }
    }
  }

  // UI refresh
  if (now - g_ui_lastMs >= UI_REFRESH_MS) {
    if (!drawToastIfAny()) {
      if (g_uiDebug) {
        long t10 = counts_to_turns10(pulses);
        drawUI_Debug(g_rpm_lastValid10, pulses, t10, g_pwmNow, g_pedalStable, g_state, g_mode, g_targetTurns10);
      } else {
        if (g_mode == Mode::MANUAL) drawUI_ManualSimple(g_rpm_lastValid10, pulses);
        else                        drawUI_AutoSimple(g_rpm_lastValid10, g_state, g_goalCounts, pulses, g_targetTurns10);
      }
    }
    g_ui_lastMs = now;
  }

  // Persist policy
  if (g_lifetimeSkeins >= g_lastSavedSkeinMark + SAVE_EVERY_SKEINS) {
    persistSave();
  } else if (g_persistDirty && (millis() - g_persistLastSaveMs) >= PERSIST_MIN_INTERVAL_MS) {
    persistSave();
  }
}
