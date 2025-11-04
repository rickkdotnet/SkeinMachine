// ----------------------------------------------------------------
// SkeinMachine v3.26
// Smart yarn-twister controller with phase-anchored AUTO stops
// NOW WITH CLOSED-LOOP RPM CONTROL (FAST & CREEP BANDS)
// ----------------------------------------------------------------
// Controls a DC motor via Cytron MD13S for precise skein twisting.
// Uses encoder feedback and adaptive braking.
//
// NEW IN v3.26
// - Removed PwmSlew logic to avoid double filtering. 
//
// RATIONALE
//  Heavy skeins caused RPM sag in the former PWM_CREEP band. With a PI speed
//  loop, the controller boosts duty against load so RPM remains stable.
//
// SAFETY
//  - No motor motion without pedal.
//  - Watchdog enabled (~1s).
//
// Hardware:
//   MCU:     Arduino Nano / Uno (ATmega328P)
//   Driver:  Cytron MD13S (DIR=D8, PWM=D9 @31 kHz)
//   Encoder: Quadrature on D2/D3 (INT0/INT1)
//   Inputs:  Foot pedal (D5, active-LOW), KY-040 rotary (D7/D6, SW=D4)
//   Display: SSD1306 128×64 I²C @0x3C
// ----------------------------------------------------------------


// ================================================================
// [1] FEATURE TOGGLES & VERSION (compile-time)
// ================================================================
#define MAINT_RESET_ON_BOOT 0
#define TOAST_TEST_MODE     0
#define DEBUG_SERIAL        0      // 0=off, 1=on
#define SERIAL_BAUD         115200
// No phase-lead or slow-anchor features in this build


// ================================================================
// [2] CONFIG — ALL TUNEABLE VALUES (calibration area)
//   Tags:
//     [HW]   = hardware-specific, don't change unless hw changes
//     [TUNE] = primary "feel" / behavior tuning
//     [ADV]  = advanced tuning, shouldn't need changing
// ================================================================

// ---------------- Display / OLED [HW] ----------------
constexpr uint8_t  OLED_ADDR              = 0x3C;   // I2C address of SSD1306
constexpr uint8_t  SCREEN_WIDTH           = 128;    // OLED width in pixels
constexpr uint8_t  SCREEN_HEIGHT          = 64;     // OLED height in pixels
constexpr uint8_t  OLED_ROTATION          = 2;      // [TUNE] 0..3; rotate screen to match physical mounting


// ---------------- Pin mapping [HW] -------------------
constexpr uint8_t  PIN_PEDAL              = 5;      // Foot pedal input (active LOW, INPUT_PULLUP)
constexpr uint8_t  PIN_DIR                = 8;      // Cytron MD13S DIR
constexpr uint8_t  PIN_PWM                = 9;      // Cytron MD13S PWM (Timer1 OC1A @31 kHz)
constexpr uint8_t  ENC_A                  = 2;      // Motor encoder A (INT0)
constexpr uint8_t  ENC_B                  = 3;      // Motor encoder B (INT1)
constexpr uint8_t  ROT_CLK                = 7;      // Rotary encoder A (PCINT23)
constexpr uint8_t  ROT_DT                 = 6;      // Rotary encoder B (PCINT22)
constexpr uint8_t  ROT_SW                 = 4;      // Rotary pushbutton (active LOW)


// --------------- Mechanics / scaling [HW] ------------
constexpr uint16_t CPR_NUM                = 9300;   // Encoder counts numerator   (~3100 counts/rev)
constexpr uint16_t CPR_DEN                = 3;      // Encoder counts denominator (gives exact CPR = CPR_NUM / CPR_DEN)


// ---------------- RPM estimation [ADV] ---------------
constexpr unsigned long RPM_WINDOW_MS     = 60;     // Time window per RPM sample; shorter = faster response, noisier
constexpr int16_t       STALL_RPM10_THRESH= 50;     // Min measured speed (x10 RPM) before we call it a stall (5.0 RPM)


// --------- Stall detection behaviour [ADV] ----------
constexpr bool          STALL_DISABLE_IN_CREEP = false; // Ignore stalls while in creep band if true (prevents slow false positives)
constexpr int           STALL_MIN_PWM         = 50;     // Only treat low RPM as stall when PWM >= this (avoid stall at gentle starts)
constexpr unsigned long RUN_STALL_GRACE_MS    = 50;     // Extra time after entering RUN before stall detection is allowed


// ---------------- PWM & ramps (open-loop) -----------
// NOTE: PWM_FAST is mainly for MANUAL and as absolute clamp; AUTO uses RPM control.
constexpr int           PWM_FAST              = 255;    // [TUNE] Max allowed duty (hardware safe limit; 255 = full)
constexpr int           START_KICK_PWM        = 60;     // [TUNE] Initial "kick" duty to break static friction (MANUAL + AUTO RAMP_UP)
constexpr unsigned long START_KICK_MS         = 35;     // [TUNE] Duration of start kick; too long = harsh jolt, too short = no start
constexpr unsigned long rampUpMs              = 500;    // [TUNE] Time for open-loop PWM ramp during AUTO/MANUAL ramp-up
constexpr unsigned long rampDownMs            = 150;    // [TUNE] Time for open-loop PWM ramp-down (shorter = snappier stop)
constexpr int           MIN_RAMP_PWM          = 40;     // [ADV] Stop ramp early if computed duty drops below this (avoid weak tail)

// ------------- RPM setpoint slewing (AUTO) ----------
constexpr unsigned long CREEP_TRANSITION_MS   = 250;    // [TUNE] Time to glide RPM setpoint in FAST band (smoother changes)
constexpr unsigned long CREEP_SLEW_MS         = 220;    // [TUNE] Time to glide RPM setpoint when entering creep (gentle slowdown)


// ---------------- UI timing [ADV] --------------------
constexpr unsigned long UI_REFRESH_MS         = 100;    // UI update period; shorter = more responsive, more CPU
constexpr unsigned long LIFETIME_SPLASH_MS    = 3000;   // Duration of lifetime splash screen at boot


// --------------- Toast defaults [ADV] ----------------
constexpr unsigned long TOAST_DEFAULT_MS      = 1000;   // Default on-screen toast duration


// ------ Approach / adaptive stop (lead) [ADV] -------
// These control how far before the target we start braking and how that adapts.
constexpr long          APPROACH_COUNTS       = 3875;   // Distance (counts) before target where we enter creep band (~1.25 turns)
constexpr long          STOP_LEAD_COUNTS_BASE = 310;    // Initial decel lead: counts before target where ramp-down is planned
constexpr long          STOPLEAD_MIN          = 30;     // Minimum allowed lead; prevents braking too late
constexpr long          STOPLEAD_MAX          = APPROACH_COUNTS - 1; // Maximum allowed lead; prevents braking too early
constexpr uint8_t       STOPLEAD_ADAPT_K_NUM  = 2;      // Adaptation strength: numerator (~0.2 * error)
constexpr uint8_t       STOPLEAD_ADAPT_K_DEN  = 5;      // Adaptation strength: denominator


// ---------- Lead persist policy [ADV] ---------------
constexpr long          LEAD_PERSIST_EPS_COUNTS = 30;   // Required change in lead before we bother saving to EEPROM
constexpr uint8_t       LEAD_PERSIST_MIN_RUNS   = 3;    // Minimum skeins between lead saves (limits EEPROM wear)


// --------------- Manual / jog [TUNE] ----------------
constexpr int           PWM_JOG               = 85;     // Jog duty used to nudge to a rotary-selected position
constexpr int           ROT_DETENTS_PER_REV   = 20;     // Mechanical detents per full rotary turn (KY-040: 20)
constexpr int           ROT_SIGN              = -1;     // Direction mapping: +1/-1 to align rotary with motor direction
constexpr unsigned int  ROT_MIN_DETENT_US     = 1200;   // Min time between detents; filters mechanical bounce / double ticks


// -------- Rotary target step size [TUNE] ------------
constexpr int8_t        TARGET_STEP_TURNS10   = 10;     // Rotary step in AUTO: Δtarget in x10 turns (10 = 1 whole turn)
constexpr int16_t       TARGET_STEP_TURNS_LIMIT = 500;  // Max target in x10 turns that can be dialed via rotary


// -------- Rotary timing (button) [ADV] --------------
constexpr unsigned long SW_DBLCLICK_MS        = 350;    // Max delay between clicks for double-click actions
constexpr unsigned long SW_DEBOUNCE_MS        = 40;     // Debounce time for pushbutton edges
constexpr unsigned long SW_LONG1_MS           = 500;    // Threshold for "long press" (e.g., direction toggle)
constexpr unsigned long SW_LONG2_MS           = 1500;   // Threshold for "very long press" (e.g., debug toggle)


// -------- Pedal debounce [ADV] ----------------------
constexpr unsigned long PEDAL_DB_DOWN_MS      = 8;      // Debounce for pedal press (fast = responsive, too fast = bounce)
constexpr unsigned long PEDAL_DB_UP_MS        = 15;     // Debounce for pedal release
constexpr unsigned long PEDAL_RELEASE_IGNORE_MS = 80;   // Ignore quick pedal releases after a new press (prevents spurious stops)


// --------------- EEPROM / persist [ADV] -------------
constexpr uint32_t      PERSIST_MAGIC         = 0x54574953UL; // 'TWIS' magic for EEPROM validation
constexpr uint16_t      PERSIST_VER           = 5;            // Structure version for EEPROM layout
constexpr uint32_t      SAVE_EVERY_SKEINS     = 5;            // Save lifetime counters after every N skeins
constexpr unsigned long PERSIST_MIN_INTERVAL_MS = 15000UL;    // Minimum time between EEPROM writes (limits wear)


// ---------------- RPM CONTROL (AUTO) ----------------
// These are the tunables for AUTO-mode.

// --- Targets in ×10 RPM [TUNE] ----------------------
// Pick speeds that your motor can reach at ~200–230 PWM in FAST, and
// that still have comfortable torque and smoothness in CREEP.
constexpr int16_t       RPM_FAST_10           = 1650;   // Auto RUN speed outside approach band (x10 RPM ⇒ 165.0 RPM)
constexpr int16_t       RPM_CREEP_10          = 400;    // Auto RUN speed in approach band (x10 RPM ⇒ 40.0 RPM)


// --- PI gains (FAST band) [ADV] ---------------------
// Discrete-time PI loop for speed, sampled every RPM_WINDOW_MS (60 ms).
// Higher Kp = tighter tracking, more risk of oscillation.
// Higher Ki = better steady-state accuracy, more risk of hunting.
constexpr int16_t       Kp_Q15                = 3932;   // Proportional gain (Q15) for FAST band
constexpr int16_t       Ki_Q15                = 655;    // Integral gain (Q15) for FAST band


// --- PI gains (CREEP band) [ADV] --------------------
// Lower gains in creep to keep low-speed control smooth and non-jerky.
constexpr int16_t       Kp_CREEP_Q15          = 1638;   // Proportional gain (Q15) for creep band
constexpr int16_t       Ki_CREEP_Q15          = 164;    // Integral gain (Q15) for creep band


// --- Feedforward model [ADV] ------------------------
// Duty ≈ a * RPM + b, derived from two measured points (e.g., 40 RPM → ~70, 160 RPM → ~255).
// This gives the PI loop a good starting point so it only has to trim.
constexpr int16_t       FF_A_NUM              = 185;    // FF slope numerator (≈ 1.445)
constexpr int16_t       FF_A_DEN              = 128;    // FF slope denominator
constexpr int8_t        FF_B_DUTY             = 12;     // FF intercept duty (baseline offset)


// --- Deadband / smoothing [ADV] ---------------------
// Deadband freezes the integrator near setpoint to avoid "chattering".
// Smoothing shapes how aggressively PWM steps move towards the new value.
constexpr int8_t        RPM_DBAND_RPM         = 2;      // Error deadband in FAST band; larger = more stable, less precise
constexpr uint8_t       ALPHA_NUM             = 1;      // FAST band IIR smoothing numerator (e.g., 1/4 step per sample)
constexpr uint8_t       ALPHA_DEN             = 4;      // FAST band IIR smoothing denominator


// --- Controller limits & anti-windup [ADV] ----------
constexpr int           CTRL_PWM_MIN          = 40;     // Min PWM the controller will command (below this we assume no useful torque)
constexpr int           CTRL_PWM_MAX          = 255;    // Max PWM the controller will command (absolute safety clamp)
constexpr int32_t       ITERM_MIN             = -(1L<<18); // Lower bound for integral term (prevents extreme windup)
constexpr int32_t       ITERM_MAX             =  (1L<<18); // Upper bound for integral term

// ================================================================
// [3] INCLUDES & GLOBAL STATE
// ================================================================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
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
struct   PedalEdges pedalService();
void     persistLoad();
void     persistSave();
void     uiLifetimeSplashTopRightLogo();
void     drawUI_ManualSimple(int16_t rpm10, long encCounts);
void     drawUI_AutoSimple  (int16_t rpm10, RunState st, long goalCountsLocal, long encLocal, int16_t targetTurns10Local);
void     drawUI_Debug       (int16_t rpm10, long pulses, long turns10, int pwm, bool pedalPressed, RunState st, Mode m, long tgt10);

inline   void motorAnalogWrite(int duty);
inline   void motorStopHard();
inline   void updatePwmSlew(unsigned long nowMs);
inline   void updateRpmSlew(unsigned long nowMs);
void     speedControllerOnRpmSample(int16_t rpm10);         // NEW: PI speed control step
inline   long encAtomicRead();
inline   void encAtomicWrite(long v);
inline   long encAbsAtomicRead();
inline   long turns10_to_counts(long t10);
inline   long counts_to_turns10(long counts);
inline   long counts_to_turns1000(long counts);
inline   long turns1000_to_counts(long t1000);
static   void pciSetup(uint8_t pin);
static   inline uint16_t phaseFromAbsCountsT1000(long absCounts);
static   inline int16_t  phaseErrorT1000(uint16_t phase, uint16_t anchor);
inline   void qdecMotorReadAB_Update();
void     ISR_encA();
void     ISR_encB();
void     updateFSM(const PedalEdges& ped, unsigned long now);
inline   void showToastC(const __FlashStringHelper* s, unsigned long dur = TOAST_DEFAULT_MS, uint8_t size = 2);
inline   void showToastDyn(const char* s, unsigned long dur = TOAST_DEFAULT_MS, uint8_t size = 2);
inline   void showSkeinToast(uint32_t skeins, unsigned long dur = 2000);
inline   void persistMarkDirty();

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

// ----- NEW: RPM setpoint slewing (AUTO) ---------
int16_t           g_rpmSetpoint10             = 0;          // current RPM setpoint (×10)
int16_t           g_rpmDesiredSetpoint10      = 0;          // desired target (changes with bands)
int16_t           g_rpmSlewFrom10             = 0;          // base for setpoint slewing
unsigned long     g_rpmSlewStartMs            = 0;
unsigned long     g_rpmSlewDurationMs         = CREEP_TRANSITION_MS;
int16_t           g_rpmPrevDesired10          = INT16_MIN;  // change detector

// ----- PI controller state -----------------
int32_t           g_iTerm_Q15                 = 0;          // integrator in Q15 domain
int16_t           g_speedRpm10Lp      = 0;    // low-pass filtered RPM10 for FAST band
bool              g_speedLpInited     = false;
int               g_speedLastPwm      = 0;    // last smoothed PWM command
bool              g_speedLastPwmInit  = false;

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

// -------- MANUAL run tracking -------------------
bool              g_manualDidPedalRun         = false;   // set true on first pedal press while in MANUAL

// --------------- Pedal debounce -----------------
bool              g_pedalRaw                  = false;      // instantaneous pin state
bool              g_pedalStable               = false;      // debounced state
bool              g_pedalPrevStable           = false;      // previous debounced state
unsigned long     g_pedalLastChangeMs         = 0;
unsigned long     g_pedalIgnoreReleaseUntilMs = 0;          // release ignore window

// Raw pedal diagnostics
bool              g_pedalRawPrev              = false;
volatile uint32_t g_pedalRawEdges             = 0;          // counted edges

struct PedalEdges {
  bool risingEff;    // debounced rising, after ignore window  (pressed)
  bool fallingEff;   // debounced falling, after ignore window (released)
};

// ---------------- PWM ---------------------------
int               g_desiredPwmTarget          = 0;          // controller desired duty (pre-slew)
int               g_slewFrom                  = 0;
bool              g_inCreepBand               = false;  // true when in approach/creep band
int               g_pwmNow                    = 0;      // last PWM actually written to hardware


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

// -------------- Phase anchor (fixed) -----------
uint16_t          g_phaseAnchorT1000          = 0xFFFF;     // 0..999, 0xFFFF = unset

// Phase debug monitors (keep only error fields for UI)
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
unsigned long     g_lastRuntimeUpdateMs       = 0;

// Persist structure (dual-slot with checksum + sequence)
struct Persist {
  uint32_t magic;
  uint16_t version;
  uint32_t skeins;
  uint32_t totalTwists;
  uint32_t runtimeSec;
  uint32_t leadFwdCounts;
  uint32_t leadRevCounts;
  uint32_t writeCount;
  uint32_t stallCount;
  uint32_t wdtResetCount;        // NEW v5: Watchdog timeout counter
  uint32_t powerOnCount;         // NEW v5: Power-on / reset counter
  uint32_t totalStarts;          // NEW v5: Total motor starts (wear indicator)
  uint16_t lastTargetTurns10;    // ×10 turns
  uint16_t phaseAnchorT1000;     // 0..999, 0xFFFF = unset
  // REMOVED v5: int32_t phaseBiasCounts (was always 0, unused)
  uint32_t seq;                  // slot sequence
  uint32_t checksum;             // FNV32 over struct except checksum
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

// RPM setpoint slewing (acts on g_rpmSetpoint10)
inline void updateRpmSlew(unsigned long nowMs) {
  if (g_rpmDesiredSetpoint10 != g_rpmPrevDesired10) {
    g_rpmSlewFrom10   = g_rpmSetpoint10;
    g_rpmSlewStartMs  = nowMs;
    g_rpmPrevDesired10= g_rpmDesiredSetpoint10;
  }
  if (g_rpmSetpoint10 == g_rpmDesiredSetpoint10) return;
  unsigned long dur = g_rpmSlewDurationMs;
  if (dur == 0UL) { g_rpmSetpoint10 = g_rpmDesiredSetpoint10; return; }
  unsigned long elapsed = nowMs - g_rpmSlewStartMs;
  if (elapsed >= dur) { g_rpmSetpoint10 = g_rpmDesiredSetpoint10; return; }

  const long ONE_Q15 = 1L << 15;
  long tQ = (long)((elapsed << 15) / (long)dur);
  if (tQ < 0) tQ = 0; if (tQ > ONE_Q15) tQ = ONE_Q15;
  long t2Q  = (tQ * tQ) >> 15;
  long term = (3L << 15) - (2L * tQ);
  long uQ   = (t2Q * term) >> 15;

  int  span = (int)g_rpmDesiredSetpoint10 - (int)g_rpmSlewFrom10;
  long inc  = ((long)span * uQ + (1L<<14)) >> 15;
  int  out  = (int)g_rpmSlewFrom10 + (int)inc;
  g_rpmSetpoint10 = (int16_t)out;
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

// Rotary + button service (incl. MANUAL→AUTO anchor & mode toggle)
void rotaryServiceFromQueue() {
  int16_t delta;
  noInterrupts(); delta = g_rot_stepQueue; g_rot_stepQueue = 0; interrupts();
  if (delta != 0) {
    if (g_mode == Mode::MANUAL) {
      long deltaCounts = (long)delta * ROT_SIGN * JOG_COUNTS_PER_DETENT;
      g_jogTargetRel += deltaCounts;
    } else {
      int16_t t = g_targetTurns10 + delta * TARGET_STEP_TURNS10;
      if (t < 0) t = 0;
      if (t > TARGET_STEP_TURNS_LIMIT) t = TARGET_STEP_TURNS_LIMIT;
      int16_t q = (t + (TARGET_STEP_TURNS10/2)) / TARGET_STEP_TURNS10;
      g_targetTurns10 = q * TARGET_STEP_TURNS10;
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
      // --- ENTER MANUAL: display equals current AUTO target (jog-proof) ---
      long encNow = encAtomicRead();             // reference for jog
      g_manualRefEnc        = encNow;
      g_jogTargetRel        = 0;

      // Show twists in MANUAL based on AUTO target, not encoder.
      g_manualDispCounts    = turns10_to_counts(g_targetTurns10);

      g_manualSettleUntilMs = millis() + 120;
      motorAnalogWrite(0);
      g_state               = RunState::IDLE;

      g_manualDidPedalRun   = false;

      // Reset speed controller when leaving MANUAL
      resetSpeedControllerState();

    } else if (prev == Mode::MANUAL && g_mode == Mode::AUTO) {
      // Anchor always set when returning to AUTO
      uint16_t ph = phaseFromAbsCountsT1000(encAbsAtomicRead());
      g_phaseAnchorT1000 = ph;
      persistMarkDirty();
      showToastC(F("ANCHOR SET"), 700, 2);

      // Only update target if pedal-run occurred in MANUAL
      if (g_manualDidPedalRun) {
        long cDisp = labs(g_manualDispCounts);
        long t10   = counts_to_turns10(cDisp);
        long rounded10 = ((t10 + 5) / 10) * 10;         // round to whole twists
        if (rounded10 < 0)      rounded10 = 0;
        if (rounded10 > 65535)  rounded10 = 65535;
        g_targetTurns10 = (int16_t)rounded10;
        g_persistDirty  = true;
      }
      motorAnalogWrite(0);

      // Reset speed controller when entering AUTO
      resetSpeedControllerState();
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

  return { risingEff, fallingEff };
}

// Returns direct pin state (LOW = pressed)
bool pedalRawNow() {
  return (digitalRead(PIN_PEDAL) == LOW);
}

// Detect raw edges, mirror LED_BUILTIN and count edges
void pedalRawEdgeService() {
  bool rawNow = pedalRawNow();

  if (rawNow != g_pedalRawPrev) {
    g_pedalRawPrev = rawNow;
    digitalWrite(LED_BUILTIN, rawNow ? HIGH : LOW); // LED on when pressed (LOW)
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
  Persist a, b; 
  bool aValid=false, bValid=false;

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
    // Initialize new struct with defaults
    memset(&g_persist, 0, sizeof(g_persist));
    g_persist.magic       = PERSIST_MAGIC;
    g_persist.version     = PERSIST_VER;
    g_persist.leadFwdCounts = STOP_LEAD_COUNTS_BASE;
    g_persist.leadRevCounts = STOP_LEAD_COUNTS_BASE;
    g_persist.lastTargetTurns10 = 100;
    g_persist.phaseAnchorT1000  = 0xFFFF;
    // NEW v5 fields auto-initialized to 0 by memset
    g_persist.seq = 1;
    g_persist.checksum = simpleChecksum32((const uint8_t*)&g_persist, sizeof(Persist) - sizeof(g_persist.checksum));
    eepromWriteRaw(EEPROM_SLOT_A, &g_persist, sizeof(Persist));
    g_activeSlot = 0;
  } else {
    if (aValid && bValid) g_activeSlot = (b.seq > a.seq) ? 1 : 0;
    else g_activeSlot = aValid ? 0 : 1;
    g_persist = (g_activeSlot == 0) ? a : b;
  }

  // Lifetime skeins uit EEPROM halen
  g_lifetimeSkeins     = g_persist.skeins;
  g_skeinSession       = 0;
  g_lastSavedSkeinMark = (g_lifetimeSkeins / SAVE_EVERY_SKEINS) * SAVE_EVERY_SKEINS;

  // Mirrors voor lead-persist / adapt-heuristiek
  g_leadPersistedFwd   = (long)g_persist.leadFwdCounts;
  g_leadPersistedRev   = (long)g_persist.leadRevCounts;
  g_leadLastSavedSkein = g_lifetimeSkeins;      // “laatst gesavede skein” bij boot = huidige EEPROM-waarde

  // Phase anchor mirror
  g_phaseAnchorT1000   = g_persist.phaseAnchorT1000;
}

// ------------------------------------------------------------
// 3. PERSIST SAVE - Remove phaseBiasCounts assignment
// ------------------------------------------------------------
void persistSave() {
  g_persist.skeins        = g_lifetimeSkeins;
  g_persist.leadFwdCounts = (uint32_t)g_stopLeadFwd;
  g_persist.leadRevCounts = (uint32_t)g_stopLeadRev;
  g_persist.lastTargetTurns10 = (uint16_t) g_targetTurns10;

  // lead mirrors bijwerken 
  g_leadPersistedFwd    = g_stopLeadFwd;
  g_leadPersistedRev    = g_stopLeadRev;
  g_leadLastSavedSkein  = g_lifetimeSkeins;

  // Phase anchor (phaseBiasCounts removed in v5)
  g_persist.phaseAnchorT1000 = g_phaseAnchorT1000;

  g_persist.writeCount++;
  g_persist.seq = (g_persist.seq == 0xFFFFFFFFUL) ? 1UL : (g_persist.seq + 1UL);
  g_persist.checksum = simpleChecksum32((const uint8_t*)&g_persist, sizeof(Persist) - sizeof(g_persist.checksum));

  // Watchdog around EEPROM writes
  wdt_reset();
  uint8_t nextSlot = (g_activeSlot == 0) ? 1 : 0;
  int base = (nextSlot == 0) ? EEPROM_SLOT_A : EEPROM_SLOT_B;
  eepromWriteRaw(base, &g_persist, sizeof(Persist));
  g_activeSlot = nextSlot;
  wdt_reset();

  g_persistDirty = false;
  g_persistLastSaveMs = millis();
  g_lastSavedSkeinMark = (g_lifetimeSkeins / SAVE_EVERY_SKEINS) * SAVE_EVERY_SKEINS;
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
void drawUI_Debug(int16_t rpm10,long pulses,long turns10,int pwm,bool /*pedalPressed*/,RunState st,Mode m,long /*tgt10*/){
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE); display.setTextSize(1);
  // Row 0: Mode | State
  display.setCursor(0,0); display.print(F("Mode:")); display.print(m==Mode::MANUAL?F("MAN"):F("AUTO"));
  display.setCursor(64,0); display.print(F("State:"));
  switch(st){ case RunState::IDLE: display.print(F("IDLE")); break; case RunState::RAMP_UP: display.print(F("RUP")); break; case RunState::RUN: display.print(F("RUN")); break; case RunState::RAMP_DOWN: display.print(F("RDN")); break; }

  // Row 1: RPM actual & setpoint
  display.setCursor(0,10); display.print(F("RPM:")); display.print((rpm10==INT16_MIN)?0:(rpm10/10));
  display.setCursor(64,10); display.print(F("SP:")); display.print((int)(g_rpmSetpoint10/10));

  // Row 2: Encoder counts & turns
  //  display.setCursor(0,20); display.print(F("Enc:")); display.print(pulses);
  //  display.setCursor(64,20); display.print(F("Twists:")); display.print(turns10/10); display.print('.'); display.print(labs(turns10)%10);
  // Row 2: Controller state during transition
    //  display.setCursor(0,20); display.print(F("Des:")); display.print(g_desiredPwmTarget);
    //  display.setCursor(64,20); display.print(F("iTrm:")); display.print((int)(g_iTerm_Q15 >> 10));
    // Row 2: RPM tracking
    int16_t spRPM = g_rpmSetpoint10 / 10;
    int16_t actRPM = (rpm10 == INT16_MIN) ? 0 : (rpm10 / 10);
    int16_t errRPM = spRPM - actRPM;
    display.setCursor(0,20); display.print(F("SP:")); display.print(spRPM);
    display.setCursor(40,20); display.print(F("Act:")); display.print(actRPM);
    display.setCursor(80,20); display.print(F("Err:")); display.print(errRPM);


  // Row 3: PWM & Band
  display.setCursor(0,30); display.print(F("PWM:")); display.print(pwm);
  display.setCursor(64,30); display.print(F("Band:")); display.print(g_inCreepBand?F("CREEP"):F("FAST"));

  // Row 4: Phase / Anchor / Error
  uint16_t phNow = phaseFromAbsCountsT1000(encAbsAtomicRead());
  uint16_t anc   = g_phaseAnchorT1000;     // 0..999  (0xFFFF = unset)
  int16_t  errT  = (anc==0xFFFF) ? 0 : phaseErrorT1000(phNow, anc); // P - A
  display.setCursor(0,40);
  if (anc == 0xFFFF) {
    snprintf_P(g_small_buf, sizeof(g_small_buf), PSTR("P:%3u A:--- e:%+d"), phNow, g_dbgLastPhaseErrT1000);
  } else {
    snprintf_P(g_small_buf, sizeof(g_small_buf), PSTR("P:%3u A:%3u e:%+d"), phNow, anc, errT);
  }
  display.print(g_small_buf);

  // Row 5: Leads
  long t1000F = counts_to_turns1000(g_stopLeadFwd);
  long t1000R = counts_to_turns1000(g_stopLeadRev);
  long aF = (t1000F<0)?-t1000F:t1000F; long aR = (t1000R<0)?-t1000R:t1000R;
  display.setCursor(0,52);
  snprintf_P(g_small_buf,sizeof(g_small_buf),PSTR("Lead F/R %ld.%03ld/%ld.%03ld"), aF/1000,aF%1000,aR/1000,aR%1000);
  display.print(g_small_buf);
  display.display();
}

// boot screen shows lifetime health counters 
void uiLifetimeSplashTopRightLogo(){
  unsigned long t0=millis();
  display.clearDisplay(); 
  display.setTextColor(SSD1306_WHITE); 
  display.setTextSize(1);
  
  const int16_t xLogo=SCREEN_WIDTH-LOGO_TOAST_W-1, yLogo=1;
  display.drawBitmap(xLogo,yLogo,LOGO_SHEEP_28x24,LOGO_TOAST_W,LOGO_TOAST_H,SSD1306_WHITE);
  
  unsigned long totalSec = g_persist.runtimeSec;
  unsigned long hh = totalSec / 3600UL; 
  unsigned long mm = (totalSec % 3600UL) / 60UL;
  
  // Compact health-focused layout
  display.setCursor(0,0);  display.println(F("SkeinMachine v3.24"));
  
  display.setCursor(0,12); display.print(F("Skeins: ")); 
  display.println(g_lifetimeSkeins);
  
  display.setCursor(0,20); display.print(F("Writes: ")); 
  display.println(g_persist.writeCount);
  
  display.setCursor(0,30); display.print(F("Stalls: ")); 
  display.print(g_persist.stallCount);
  display.setCursor(72,30); display.print(F("WDT: ")); 
  display.println(g_persist.wdtResetCount);
  
  display.setCursor(0,40); display.print(F("Starts: ")); 
  display.print(g_persist.totalStarts);
  display.setCursor(72,40); display.print(F("Boots: ")); 
  display.println(g_persist.powerOnCount);
  
  display.setCursor(0,50); display.print(F("Runtime: ")); 
  display.print(hh); display.print(F("h ")); 
  display.print(mm); display.println(F("m"));
  
  display.display();
  while(millis()-t0 < LIFETIME_SPLASH_MS) {
    wdt_reset(); // keep WDT happy while we show the splash
  }
  display.clearDisplay(); 
  display.display();
}

// ================================================================
// [8] CLOSED-LOOP SPEED CONTROL (AUTO)
// ================================================================

// Reset all speed controller state variables so the next AUTO run starts "fresh".
void resetSpeedControllerState() {
  // Core PI state
  g_iTerm_Q15           = 0;
  g_rpmSetpoint10       = 0;
  g_rpmDesiredSetpoint10= 0;
  g_rpmPrevDesired10    = INT16_MIN;

  // Internal filters / smoothing
  g_speedRpm10Lp        = 0;
  g_speedLpInited       = false;
  g_speedLastPwm        = 0;
  g_speedLastPwmInit    = false;
}

// Closed-loop PI speed controller, called whenever a fresh RPM sample is available.
// - Only active in AUTO mode
// - Only drives PWM in RUN (RAMP_UP is open-loop)
// - Uses Q15 gains and a small deadband to avoid chattering around setpoint.
// - Smoothing of PWM output happens here via lastPwm; there is no extra PWM slew elsewhere.
void speedControllerOnRpmSample(int16_t rpm10) {
  if (g_mode != Mode::AUTO) return;
  if (g_state != RunState::RAMP_UP && g_state != RunState::RUN) return;
  if (rpm10 == INT16_MIN) return;

  bool inRampUp = (g_state == RunState::RAMP_UP);

  // ---- RPM filtering ----
  int16_t rpm10_f;
  if (g_inCreepBand) {
    rpm10_f = rpm10;  // raw in creep
  } else {
    if (!g_speedLpInited) {
      g_speedRpm10Lp = rpm10;
      g_speedLpInited = true;
    }
    int32_t d = (int32_t)rpm10 - (int32_t)g_speedRpm10Lp;
    g_speedRpm10Lp += (int16_t)((d + 1) / 3);  // ~1/3 step
    rpm10_f = g_speedRpm10Lp;
  }


  // ---- Setpoint ----
  int16_t sp10 = g_rpmSetpoint10;
  if (sp10 <= 0) {
    // No speed requested: reset controller state
    g_iTerm_Q15  = 0;
    return;
  }

  // Feedforward (duty ≈ a*RPM + b)
  int16_t spRPM = (sp10 + 5) / 10;
  int32_t u_ff  = FF_B_DUTY + ((int32_t)spRPM * FF_A_NUM + (FF_A_DEN / 2)) / FF_A_DEN;

  // ---- Error in integer RPM ----
  int16_t err10  = sp10 - rpm10_f;
  int16_t errRPM = (err10 >= 0)
                 ? ( (err10 + 5) / 10 )
                 : ( (err10 - 5) / 10 );

  // ---- Gains (FAST vs CREEP) ----
  int16_t Kp = Kp_Q15;
  int16_t Ki = Ki_Q15;
  if (g_inCreepBand) {
    Kp = Kp_CREEP_Q15;
    Ki = Ki_CREEP_Q15;
  }

  // ---- Deadband ----
  int8_t dband = g_inCreepBand ? 1 : RPM_DBAND_RPM;

  // ---- Proportional term (Q15) ----
  int32_t p_Q15 = 0;
  if (abs(errRPM) > dband) {
    p_Q15 = (int32_t)Kp * (int32_t)errRPM;
  }

  // ---- Conditional integration with anti-windup ----
  if (Ki > 0 && abs(errRPM) > dband) {
    int32_t tentative_Q15 = p_Q15 + g_iTerm_Q15;
    int32_t tentative_pwm = (tentative_Q15 >> 8) + u_ff;

    bool atUpper     = tentative_pwm >= (CTRL_PWM_MAX - 5);
    bool atLower     = tentative_pwm <= (CTRL_PWM_MIN + 5);
    bool drivesUpper = (errRPM > 0);
    bool drivesLower = (errRPM < 0);

    // Integrate only if we're not obviously pushing further into saturation
    if (!(atUpper && drivesUpper) && !(atLower && drivesLower)) {
      g_iTerm_Q15 += (int32_t)Ki * (int32_t)errRPM;
      if (g_iTerm_Q15 > ITERM_MAX) g_iTerm_Q15 = ITERM_MAX;
      if (g_iTerm_Q15 < ITERM_MIN) g_iTerm_Q15 = ITERM_MIN;
    }
  }

  // ---- Combine FF + PI ----
  int32_t u_Q15 = p_Q15 + g_iTerm_Q15;
  int32_t u_pwm = (u_Q15 >> 8) + u_ff;

  // ---- Clamp to controller limits ----
  if (u_pwm < CTRL_PWM_MIN) u_pwm = CTRL_PWM_MIN;
  if (u_pwm > CTRL_PWM_MAX) u_pwm = CTRL_PWM_MAX;

  // ---- Output smoothing (per RPM sample) ----
  if (!g_speedLastPwmInit) {
    g_speedLastPwm     = (int)u_pwm;
    g_speedLastPwmInit = true;
  } else {
    int delta = (int)u_pwm - g_speedLastPwm;
    if (g_inCreepBand) {
      g_speedLastPwm = g_speedLastPwm + ((delta + 2) / 4);
    } else {
      g_speedLastPwm += (delta * ALPHA_NUM) / ALPHA_DEN;
    }
  }

  if (!inRampUp) {
    g_desiredPwmTarget = g_speedLastPwm;
  }

}


// ================================================================
// [9] FINITE STATE MACHINE
// ================================================================
void updateFSM(const PedalEdges& ped, unsigned long now) {
  // Lifetime integrators
  long encNowForLife = encAtomicRead();
  long diff = encNowForLife - g_lastEncForLife; 

  // runtime (tellen in seconden)
  unsigned long dtMs = now - g_lastRuntimeUpdateMs;
  if (dtMs >= 1000UL) {
      g_persist.runtimeSec += dtMs / 1000UL;
      g_lastRuntimeUpdateMs += (dtMs / 1000UL) * 1000UL;
  }

  // Manual mode
  if (g_mode == Mode::MANUAL) {
    if (ped.risingEff) {
      g_manualDidPedalRun = true;

      g_state = RunState::RAMP_UP; 
      g_stateStartMs = now;
      g_startDuty = 0;

      digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);

      // ---- Count motor start (health monitoring) ----
      g_persist.totalStarts++;

      // Reset encoder and MANUAL display counter for this run
      encAtomicWrite(0);
      g_manualRefEnc      = 0;
      g_manualDispCounts  = 0;

      g_pedalIgnoreReleaseUntilMs = now + PEDAL_RELEASE_IGNORE_MS;
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
        } else {
          if (g_startDuty < MIN_RAMP_PWM) g_startDuty = MIN_RAMP_PWM;
          motorAnalogWrite((int)duty);
        }
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
        digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);
        if (g_targetTurns10 > 0) {
          encAtomicWrite(0); g_lastEncForLife = 0;

          // ---- Count this motor start for health monitoring ----
          g_persist.totalStarts++;

          // Target from turns
          g_goalCounts = turns10_to_counts(g_targetTurns10);

          // --- Phase start-bias toward anchor (no slow adaptation elsewhere) ---
          if (g_phaseAnchorT1000 != 0xFFFF) {
            uint16_t phNow = phaseFromAbsCountsT1000(encAbsAtomicRead());
            int16_t  deltaT1000 = phaseErrorT1000(g_phaseAnchorT1000, phNow);  // desired - current
            int dirSign = g_dirForward ? +1 : -1;
            long cpm = turns1000_to_counts(1);      // counts per mturn
            long biasCounts = (long)dirSign * (long)deltaT1000 * cpm;
            long halfTurn = turns1000_to_counts(500);
            if (biasCounts >  halfTurn) biasCounts =  halfTurn;
            if (biasCounts < -halfTurn) biasCounts = -halfTurn;
            g_goalCounts += biasCounts;
          }

          g_rpm_lastCounts = encAtomicRead(); g_rpm_lastMs = now;
          g_rpm_lastValid10 = INT16_MIN;

          // Fresh start for the speed controller for this new AUTO run
          resetSpeedControllerState();

          long totalCounts = g_goalCounts;
          bool smallTarget = (totalCounts <= (APPROACH_COUNTS + STOP_LEAD_COUNTS_BASE + 309));

          // Initialise RPM setpoint and band
          g_inCreepBand = smallTarget; // start in creep if tiny target
          g_rpmDesiredSetpoint10 = g_inCreepBand ? RPM_CREEP_10 : RPM_FAST_10;
          g_rpmSlewDurationMs    = g_inCreepBand ? CREEP_SLEW_MS : CREEP_TRANSITION_MS;
          g_rpmSetpoint10        = g_rpmDesiredSetpoint10; // start at target immediately
          g_rpmPrevDesired10     = g_rpmDesiredSetpoint10;
          g_iTerm_Q15            = 0; // reset controller integral

          // PWM side: start at zero, controller will raise it; keep slew medium
          g_desiredPwmTarget     = 0;

          g_countedThisRun = false; g_rampDownPlanned = false;
          g_state = RunState::RAMP_UP; g_stateStartMs = now; motorAnalogWrite(0);

          // Early kick to break static friction before closed-loop takes over
          g_startDuty = START_KICK_PWM;
        }
      }
    } break;

    case RunState::RAMP_UP: {
      if (ped.risingEff) { // pedal pressed while ramping up → ramp down
        g_startDuty = g_pwmNow; g_state = RunState::RAMP_DOWN; g_stateStartMs = now; g_rampDownPlanned = false; break; }

      unsigned long elapsed = now - g_stateStartMs;

      // Open-loop PWM ramp during startup
      if (elapsed <= START_KICK_MS) {
        int duty = START_KICK_PWM;
        g_desiredPwmTarget = duty;
//        g_pwmTarget = duty;
        motorAnalogWrite(duty);
      } else {
        unsigned long rampElapsed = elapsed - START_KICK_MS;
        
        // RAMP_UP transitioning to RUN:
        if (rampElapsed >= rampUpMs) {
          // Calculate the duty we've ramped up to
          int16_t targetRPM = (g_inCreepBand ? RPM_CREEP_10 : RPM_FAST_10) / 10;
          int32_t targetDuty = FF_B_DUTY + ((int32_t)targetRPM * FF_A_NUM + (FF_A_DEN/2)) / FF_A_DEN;
          if (targetDuty < 60) targetDuty = 60;
          if (targetDuty > 255) targetDuty = 255;
          
          // Initialize closed-loop controller to continue from current duty
          g_rpmSlewDurationMs = g_inCreepBand ? CREEP_SLEW_MS : CREEP_TRANSITION_MS;
          g_rpmDesiredSetpoint10 = g_inCreepBand ? RPM_CREEP_10 : RPM_FAST_10;
          g_rpmSetpoint10 = g_rpmDesiredSetpoint10;
          g_rpmPrevDesired10 = g_rpmDesiredSetpoint10;
          
          // Set all PWM variables to prevent glitch
          g_desiredPwmTarget = (int)targetDuty;
          
          // FIX: Correct integrator pre-loading
          int32_t ff = FF_B_DUTY + ((int32_t)targetRPM * FF_A_NUM + (FF_A_DEN/2)) / FF_A_DEN;
          g_iTerm_Q15 = ((int32_t)targetDuty - ff) << 8;  // Correct scaling
          
          // Clamp integrator
          if (g_iTerm_Q15 > ITERM_MAX) g_iTerm_Q15 = ITERM_MAX;
          if (g_iTerm_Q15 < ITERM_MIN) g_iTerm_Q15 = ITERM_MIN;
          
          g_state = RunState::RUN;
          g_stateStartMs = millis();  // Reset state timer for RUN
         } else {
          // Linear PWM ramp from START_KICK_PWM to estimated duty
          int16_t targetRPM = (g_inCreepBand ? RPM_CREEP_10 : RPM_FAST_10) / 10;
          int32_t targetDuty = FF_B_DUTY + ((int32_t)targetRPM * FF_A_NUM + (FF_A_DEN/2)) / FF_A_DEN;
          if (targetDuty < 60) targetDuty = 60;
          if (targetDuty > 255) targetDuty = 255;
          
          int duty = START_KICK_PWM + (int)(((long)(targetDuty - START_KICK_PWM) * (long)rampElapsed) / (long)rampUpMs);
          
          // Set all PWM variables to prevent the main loop from overwriting
          g_desiredPwmTarget = duty;
          motorAnalogWrite(duty);
        }
      }
    } break;

    case RunState::RUN: {
      if (ped.risingEff) {
        g_startDuty = g_pwmNow;
        g_state     = RunState::RAMP_DOWN;
        g_stateStartMs = now;
        g_rampDownPlanned = false;
        break;
      }

      long encNow   = encAtomicRead();
      long progress = labs(encNow);
      long error    = g_goalCounts - progress;
      long stopLeadNow = g_dirForward ? g_stopLeadFwd : g_stopLeadRev;
      (void)stopLeadNow;

      // Enter/exit creep band by position error; set RPM setpoint accordingly.
      long approachEnter = APPROACH_COUNTS;

      bool wasInCreep = g_inCreepBand;  // (optioneel debug)

      if (!g_inCreepBand) {
        if (error <= approachEnter) {
          g_inCreepBand          = true;
          g_rpmDesiredSetpoint10 = RPM_CREEP_10;
          g_rpmSlewDurationMs    = CREEP_SLEW_MS;
          // integrator wat kleiner in creep
          g_iTerm_Q15 = g_iTerm_Q15 / 3;
        } else {
          g_rpmDesiredSetpoint10 = RPM_FAST_10;
          g_rpmSlewDurationMs    = CREEP_TRANSITION_MS;
        }
      } else {
          g_rpmDesiredSetpoint10 = RPM_CREEP_10;
          g_rpmSlewDurationMs    = CREEP_SLEW_MS;
      }

      // Slew alleen het RPM-setpoint, niet de PWM
      updateRpmSlew(now);

      if (error <= stopLeadNow && error > 0) {
        if (!g_countedThisRun) {
          g_countedThisRun = true;
          g_skeinSession++;
          g_lifetimeSkeins++;

          long countsThisSkein  = labs(encAtomicRead());
          long twistsThisSkein  = countsThisSkein / (CPR_NUM / CPR_DEN);
          g_persist.totalTwists += (uint32_t)twistsThisSkein;

          if (!g_uiDebug) showSkeinToast(g_skeinSession, 2000);
          if (g_lifetimeSkeins >= g_lastSavedSkeinMark + SAVE_EVERY_SKEINS) {
            persistMarkDirty();
          }
        }
        g_rampDownPlanned = true;
        g_encAtRampDown   = progress;
        g_startDuty       = g_pwmNow;
        g_state           = RunState::RAMP_DOWN;
        g_stateStartMs    = now;
      }
      else if (error <= 0) {
        // Veiligheid: hard stop, dan naar IDLE
        motorStopHard();
        g_state = RunState::IDLE;
      }
    } break;

    case RunState::RAMP_DOWN: {
      // Disable speed loop during open-loop ramp-down (pre-existing adaptive scheme)
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

        // Tel twists van deze manual run
        long countsThisRun = labs(encAtomicRead());
        long twistsThisRun = countsThisRun / (CPR_NUM / CPR_DEN);
        g_persist.totalTwists += (uint32_t)twistsThisRun;

        if (g_rampDownPlanned) {
          g_settleUntilMs = now + (RPM_WINDOW_MS + 100);
          g_adaptPending  = true;
          g_rampDownPlanned = false;
        }
        g_state = RunState::IDLE;

        // Reset controller
        g_iTerm_Q15 = 0; g_rpmSetpoint10 = 0; g_rpmDesiredSetpoint10 = 0;
      } else {
        motorAnalogWrite((int)duty);
      }
    } break;
  }
}

// ================================================================
// [10] SETUP & LOOP
// ================================================================
void setup() {
  // ---- Early initialization: reset detection ----
  // Capture reset cause (incl. watchdog) very early, then clear
  uint8_t mcusr_m = MCUSR;
  MCUSR = 0;

  // ---- Display initialization ----
  Wire.begin();
  Wire.setClock(400000); // 400 kHz I²C for fast OLED updates
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    // Display init failed - blink LED forever as error indication
    pinMode(LED_BUILTIN, OUTPUT);
    while (true) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(100); }
  }
  display.setRotation(OLED_ROTATION); 
  display.clearDisplay(); 
  display.display();

  // ---- Load persistent data from EEPROM ----
  persistLoad(); 

   // Count every power-on
  g_persist.powerOnCount++;
  persistMarkDirty();


  g_lastLoopMs = millis(); 
  g_persistLastSaveMs = millis();

  // If watchdog caused the reset, count it separately
  if (mcusr_m & _BV(WDRF)) {
    g_persist.wdtResetCount++;
    persistMarkDirty();
  }

  #if MAINT_RESET_ON_BOOT
    // ======= MAINTENANCE: Migrate v4 → v5 while preserving data =======
    // Vul hier je HUIDIGE waarden in uit de lifetime splash:
    const uint32_t CURRENT_SKEINS       = 1009;   // <- Vul in wat je ziet
    const uint32_t CURRENT_STALLS       = 0;      // <- Vul in wat je ziet
    const uint32_t CURRENT_WRITES       = 1178;   // <- Vul in wat je ziet (EEPWrites)
    const uint32_t CURRENT_TOTAL_TWISTS = 12000;  // <- Vul in wat je ziet (Twists)
    const uint32_t CURRENT_RUNTIME_SEC  = 65940;  // <- 18h 19m = 65940 seconden
    
    // Kopieer bestaande waarden
    g_persist.skeins        = CURRENT_SKEINS;
    g_persist.stallCount    = CURRENT_STALLS;
    g_persist.writeCount    = CURRENT_WRITES;
    g_persist.totalTwists   = CURRENT_TOTAL_TWISTS;
    g_persist.runtimeSec    = CURRENT_RUNTIME_SEC;
    
    // Behoud bestaande adaptive leads (als ze geldig waren)
    // Deze blijven staan zoals ze waren na persistLoad()
    
    // Initialiseer NIEUWE v5 velden
    g_persist.wdtResetCount = 0;
    g_persist.powerOnCount  = 1;        // Deze boot telt als eerste
    g_persist.totalStarts   = 0;        // Start op 0
    
    // Update versie
    g_persist.version = PERSIST_VER;
    
    // Update runtime variabelen
    g_lifetimeSkeins     = CURRENT_SKEINS;
    g_skeinSession       = 0;
    g_lastSavedSkeinMark = (g_lifetimeSkeins / SAVE_EVERY_SKEINS) * SAVE_EVERY_SKEINS;
    
    // Reset phase anchor (of behoud als je die wilt)
    // g_phaseAnchorT1000   = 0xFFFF;  // Uncomment om te resetten
    
    persistMarkDirty(); 
    persistSave();
    showToastC(F("v5 migrated"), 1500, 2);
  #endif

  #if TOAST_TEST_MODE
    // Random seed for toast testing
    pinMode(A0, INPUT);
    randomSeed( (unsigned long)analogRead(A0) ^ (unsigned long)micros() );
  #endif

  // ---- Load user preferences from EEPROM ----
  g_targetTurns10 = g_persist.lastTargetTurns10;
  // Validate and load adaptive stop leads (or use defaults)
  g_stopLeadFwd = (g_persist.leadFwdCounts >= (uint32_t)STOPLEAD_MIN && g_persist.leadFwdCounts <= (uint32_t)STOPLEAD_MAX)
                      ? (long)g_persist.leadFwdCounts : STOP_LEAD_COUNTS_BASE;
  g_stopLeadRev = (g_persist.leadRevCounts >= (uint32_t)STOPLEAD_MIN && g_persist.leadRevCounts <= (uint32_t)STOPLEAD_MAX)
                      ? (long)g_persist.leadRevCounts : STOP_LEAD_COUNTS_BASE;

  // ---- Show startup splash screen ----
  uiLifetimeSplashTopRightLogo();

  // ---- Configure I/O pins ----
  // Pedal input (active LOW with internal pullup)
  pinMode(PIN_PEDAL, INPUT_PULLUP);
  
  // Motor driver outputs (Cytron MD13S)
  pinMode(PIN_DIR, OUTPUT); 
  pinMode(PIN_PWM, OUTPUT);
  digitalWrite(PIN_DIR, HIGH); 
  g_dirForward = true;

  // LED mirrors pedal raw state for diagnostics
  pinMode(LED_BUILTIN, OUTPUT);
  g_pedalRawPrev = pedalRawNow();
  digitalWrite(LED_BUILTIN, g_pedalRawPrev ? HIGH : LOW);

  // ---- Configure Timer1 for fast PWM (~31 kHz) ----
  // Fast PWM 8-bit on OC1A (D9), non-inverting, prescaler=1
  TCCR1A = (1 << WGM10) | (1 << COM1A1);
  TCCR1B = (1 << WGM12) | (1 << CS10);

  // ---- Motor encoder setup (quadrature on INT0/INT1) ----
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  // Initialize encoder state from current pin readings
  { uint8_t a = digitalRead(ENC_A), b = digitalRead(ENC_B); 
    g_enc_prevState = (a << 1) | b; }
  attachInterrupt(digitalPinToInterrupt(ENC_A), ISR_encA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), ISR_encB, CHANGE);

  // ---- Rotary encoder setup (KY-040 via PCINT) ----
  pinMode(ROT_CLK, INPUT_PULLUP);
  pinMode(ROT_DT,  INPUT_PULLUP);
  // Initialize rotary state
  { uint8_t ra = (PIND >> 7) & 0x01; 
    uint8_t rb = (PIND >> 6) & 0x01; 
    g_rot_prevAB = (ra << 1) | rb; }
  pinMode(ROT_SW, INPUT_PULLUP); 
  g_sw_state = digitalRead(ROT_SW);
  pciSetup(ROT_CLK); 
  pciSetup(ROT_DT);

  // ---- Initial state: motor stopped, IDLE ----
  motorStopHard(); 
  g_state = RunState::IDLE;

  // Draw initial UI
  long ec = encAtomicRead();
  drawUI_AutoSimple(g_rpm_lastValid10, g_state, g_goalCounts, ec, g_targetTurns10);

  // ---- Enable watchdog LAST (after all peripherals initialized) ----
  wdt_enable(WDTO_1S);
  wdt_reset();
}
  
void loop() {
  // ---- Watchdog: reset early to prevent timeout ----
  wdt_reset();

  // ---- Service user inputs ----
  pedalRawEdgeService();      // Track raw pedal edges & mirror to LED
  rotaryServiceFromQueue();   // Process rotary encoder steps & button
  unsigned long now = millis();

  // ---- Pedal debounce & edge detection ----
  PedalEdges ped = pedalService();

  // ---- Main state machine: handle motor control ----
  updateFSM(ped, now);

  // ---- RPM measurement & closed-loop speed control (AUTO mode) ----
  int16_t rpm10 = rpm10SinceWindow();  // Sample RPM every ~60ms
  if (rpm10 != INT16_MIN) {
    g_rpm_lastValid10 = rpm10;
    speedControllerOnRpmSample(rpm10); // PI controller updates g_desiredPwmTarget
  }

  // ---- Apply PWM directly from PI controller (AUTO/RUN only) ----
  // In other states/modes, the FSM drives the motor (manual, ramp-up/down).
  if (g_mode == Mode::AUTO && g_state == RunState::RUN) {
    motorAnalogWrite(g_desiredPwmTarget);
  }

  // ---- Stall detection ----
  if (g_state != RunState::IDLE && rpm10 != INT16_MIN) {
    int16_t absrpm10 = (rpm10 < 0) ? -rpm10 : rpm10;
    unsigned long runTime = millis() - g_stateStartMs;
    
    // Wait for grace period before any stall detection
    if (runTime >= RUN_STALL_GRACE_MS) {
      bool stallDetected = false;
      
      // Simple check: if we're applying power but RPM is too low = stall
      if (g_pwmNow >= STALL_MIN_PWM && absrpm10 < STALL_RPM10_THRESH) {
        stallDetected = true;
      }
      
      // Honor the creep disable flag if set
      if (STALL_DISABLE_IN_CREEP && g_inCreepBand) {
        stallDetected = false;
      }
      
      if (stallDetected && !g_stallLatched) {
        g_stallLatched = true;
        showToastC(F("STALL"), 900, 2);
        g_persist.stallCount++;
        persistMarkDirty();

        // Graceful stop via ramp-down  
        g_startDuty = g_pwmNow;
        g_state = RunState::RAMP_DOWN;
        g_stateStartMs = millis();
        g_rampDownPlanned = false;
      }
    }
  }
  if (g_state == RunState::IDLE) g_stallLatched = false;

  // ---- Adaptive parameter updates (after motor settles) ----
  if (g_adaptPending && millis() >= g_settleUntilMs) {
    g_adaptPending = false;

    // (1) Adaptive stop lead: adjust based on measured deceleration
    long encAfterAbs = labs(encAtomicRead());
    long decelMeasured = encAfterAbs - g_encAtRampDown; 
    if (decelMeasured < 0) decelMeasured = 0;
    long* pLead = g_dirForward ? &g_stopLeadFwd : &g_stopLeadRev;
    long delta   = decelMeasured - *pLead;
    long adjust  = (delta * STOPLEAD_ADAPT_K_NUM + (STOPLEAD_ADAPT_K_DEN/2)) / STOPLEAD_ADAPT_K_DEN; // ~0.2*delta
    long newLead = *pLead + adjust;
    if (newLead < STOPLEAD_MIN) newLead = STOPLEAD_MIN; 
    if (newLead > STOPLEAD_MAX) newLead = STOPLEAD_MAX;
    *pLead = newLead;
    
    // Mark for EEPROM save if lead changed significantly
    long persisted = g_dirForward ? g_leadPersistedFwd : g_leadPersistedRev;
    bool farFromPersisted = (labs(newLead - persisted) >= LEAD_PERSIST_EPS_COUNTS);
    bool enoughRuns       = ((g_lifetimeSkeins - g_leadLastSavedSkein) >= LEAD_PERSIST_MIN_RUNS);
    if (farFromPersisted && enoughRuns) { 
      persistMarkDirty(); 
    }

    // (2) Phase tracking 
    
    // Update debug phase error snapshot for UI
    if (g_phaseAnchorT1000 != 0xFFFF) {
      uint16_t phNow = phaseFromAbsCountsT1000(encAbsAtomicRead());
      g_dbgLastPhaseErrT1000 = phaseErrorT1000(phNow, g_phaseAnchorT1000);
    }
  }

  // ---- UI refresh (every 100ms) ----
  if (now - g_ui_lastMs >= UI_REFRESH_MS) {
    long pulses = encAtomicRead();  // Read encoder only when needed for UI
    if (!drawToastIfAny()) {  // Toast messages have priority
      if (g_uiDebug) {
        // Debug mode: show detailed system state
        long t10 = counts_to_turns10(pulses);
        drawUI_Debug(g_rpm_lastValid10, pulses, t10, g_pwmNow, g_pedalStable, g_state, g_mode, g_targetTurns10);
      } else {
        // Normal mode: mode-specific simple UI
        if (g_mode == Mode::MANUAL) drawUI_ManualSimple(g_rpm_lastValid10, pulses);
        else                        drawUI_AutoSimple(g_rpm_lastValid10, g_state, g_goalCounts, pulses, g_targetTurns10);
      }
    }
    g_ui_lastMs = now;
  }

  // ---- EEPROM persistence: save when IDLE and dirty flag set ----
  // Delayed writes avoid interfering with time-critical motor control
  if (g_state == RunState::IDLE && g_persistDirty && 
      (millis() - g_persistLastSaveMs) >= PERSIST_MIN_INTERVAL_MS) {
    persistSave();
  }
}