// ----------------------------------------------------------------
// SkeinMachine v3.28
// Smart yarn-twister controller with phase-anchored AUTO stops
// NOW WITH CLOSED-LOOP RPM CONTROL (FAST & CREEP BANDS)
// ----------------------------------------------------------------
// Controls a DC motor via Cytron MD13S for precise skein twisting.
// Uses encoder feedback and adaptive braking.
//
// NEW IN v3.27
// - Remove some magic numbers, other small fixes
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
#define ENABLE_DEBUG_UI     1
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
constexpr uint16_t      COUNTS_PER_REV = 3100;      // Effective encoder counts per mechanical revolution of the hook. 

// ---------------- RPM estimation [ADV] ---------------
constexpr unsigned long RPM_WINDOW_MS     = 64;     // Time window per RPM sample; shorter = faster response, noisier
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
constexpr unsigned long CREEP_SLEW_MS         = 250;    // [TUNE] Time to glide RPM setpoint when entering creep (gentle slowdown)


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
constexpr int16_t       RPM_FAST_10           = 1610;   // Auto RUN speed outside approach band (x10 RPM ⇒ 165.0 RPM)
constexpr int16_t       RPM_CREEP_10          = 400;    // Auto RUN speed in approach band (x10 RPM ⇒ 40.0 RPM)
constexpr int16_t       RPM_FILTER_DIV = 3; // Low-pass filter on RPM measurement: lpf += (input - lpf) / 3 - Time constant ≈ 3 samples = 180ms at 60ms sample rate


// --- PI gains (FAST band) [ADV] ---------------------
// Discrete-time PI loop for speed, sampled every RPM_WINDOW_MS (60 ms).
// PI gains in Q15 fixed-point format (divide by 32768 for real value)
// FAST band: Kp = 3932/32768 ≈ 0.12, Ki = 655/32768 ≈ 0.02
// CREEP band: Kp = 1638/32768 ≈ 0.05, Ki = 164/32768 ≈ 0.005
// Tuned for: 160 RPM target, 60ms sample time, 3100 counts/rev motor
// Higher Kp = tighter tracking, more risk of oscillation.
// Higher Ki = better steady-state accuracy, more risk of hunting.

constexpr int16_t       Kp_Q15                = 3932;   // Proportional gain (Q15) for FAST band
constexpr int16_t       Ki_Q15                = 655;    // Integral gain (Q15) for FAST band


// --- PI gains (CREEP band) [ADV] --------------------
// Lower gains in creep to keep low-speed control smooth and non-jerky.
constexpr int16_t       Kp_CREEP_Q15          = 1638;   // Proportional gain (Q15) for creep band
constexpr int16_t       Ki_CREEP_Q15          = 164;    // Integral gain (Q15) for creep band


// --- Feedforward model [ADV] ------------------------
// Duty ≈ a * RPM + b, derived from two measured points (e.g., 40 RPM → ~75, 160 RPM → ~255).
// Slope: a = 1.5 exactly (192/128)
// Intercept: b = 15
// This gives the PI loop a good starting point so it only has to trim.
constexpr int16_t       FF_A_NUM              = 192;    // FF slope numerator (≈ 1.445)
constexpr int16_t       FF_A_DEN              = 128;    // FF slope denominator
constexpr int8_t        FF_B_DUTY             = 15;     // FF intercept duty (baseline offset)

// --- Deadband / smoothing [ADV] ---------------------
// Deadband freezes the integrator near setpoint to avoid "chattering".
// Smoothing shapes how aggressively PWM steps move towards the new value.
constexpr int8_t        RPM_DBAND_RPM         = 2;      // Error deadband in FAST band; larger = more stable, less precise
// IIR smoothing for PWM command: new_pwm = old_pwm + (target - old_pwm) * ALPHA
// ALPHA = 1/4 means PWM changes by 25% of error each sample (60ms)
constexpr uint8_t       ALPHA_NUM             = 1;      // FAST band IIR smoothing numerator (e.g., 1/4 step per sample)
constexpr uint8_t       ALPHA_DEN             = 4;      // FAST band IIR smoothing denominator

// --- Controller limits & anti-windup [ADV] ----------
constexpr int           CTRL_PWM_MIN          = 40;     // Min PWM the controller will command (below this we assume no useful torque)
constexpr int           CTRL_PWM_MAX          = 255;    // Max PWM the controller will command (absolute safety clamp)

// Anti-windup limits in Q15 domain
// ±(1<<18) in Q15 = ±(1<<18)/32768 = ±8 in PWM units
// This allows integrator to accumulate ±8 PWM worth of error correction
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
bool     pedalRawNow();                                     // Read raw pedal pin (active LOW)
void     pedalRawEdgeService();                             // Track raw pedal edges and mirror to LED

struct   PedalEdges;
enum class RunState : uint8_t;
enum class Mode     : uint8_t;

int16_t  rpm10SinceWindow();                                // Compute RPM×10 over a sliding window
void     rotaryServiceFromQueue();                          // Handle rotary encoder steps and button actions
struct   PedalEdges pedalService();                         // Debounce pedal and report effective edges
void     persistLoad();                                     // Load persistent data from EEPROM
void     persistSave();                                     // Save persistent data to EEPROM
void     uiLifetimeSplashTopRightLogo();                    // Show lifetime/health splash at boot
void     drawUI_ManualSimple(int16_t rpm10, long encCounts);
void     drawUI_AutoSimple  (int16_t rpm10, RunState st, long goalCountsLocal, long encLocal, int16_t targetTurns10Local);
void     drawUI_Debug       (int16_t rpm10, long pulses, long turns10, int pwm, bool pedalPressed, RunState st, Mode m, long tgt10);

inline   void motorAnalogWrite(int duty);                   // Low-level PWM write with clamping, keeps g_pwmNow
inline   void motorStopHard();                              // Immediate PWM=0 (hard stop)
inline   void updateRpmSlew(unsigned long nowMs);           // Smoothly move RPM setpoint towards desired band
void     speedControllerOnRpmSample(int16_t rpm10);         // PI speed control step on each new RPM sample
void     resetSpeedControllerState();                       // Reset all speed-controller internal state

inline   long encAtomicRead();                              // Atomic read of incremental encoder count
inline   void encAtomicWrite(long v);                       // Atomic write of incremental encoder count
inline   long encAbsAtomicRead();                           // Atomic read of absolute encoder count
inline   long turns10_to_counts(long t10);                  // Convert ×10 turns to encoder counts
inline   long counts_to_turns10(long counts);               // Convert encoder counts to ×10 turns
inline   long counts_to_turns1000(long counts);             // Convert encoder counts to ×1000 turns (mturns)
inline   long turns1000_to_counts(long t1000);              // Convert ×1000 turns (mturns) to counts
static   void pciSetup(uint8_t pin);                        // Configure PCINT for a given pin
static   inline uint16_t phaseFromAbsCountsT1000(long absCounts);   // Phase (0..999) from absolute counts
static   inline int16_t  phaseErrorT1000(uint16_t phase, uint16_t anchor); // Signed phase error vs anchor
inline   void qdecMotorReadAB_Update();                     // Quadrature decode for motor encoder
void     ISR_encA();                                       // External interrupt ISR for encoder A
void     ISR_encB();                                       // External interrupt ISR for encoder B
void     updateFSM(const PedalEdges& ped, unsigned long now);       // Main finite state machine
inline   void showToastC(const __FlashStringHelper* s, unsigned long dur = TOAST_DEFAULT_MS, uint8_t size = 2);
inline   void showToastDyn(const char* s, unsigned long dur = TOAST_DEFAULT_MS, uint8_t size = 2);
inline   void showSkeinToast(uint32_t skeins, unsigned long dur = 2000);
inline   void persistMarkDirty();                          // Mark persist state dirty so it will be saved later


// ---------------- Display object -----------------------
// OLED display driver; shared object for all UI rendering.
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


// ---------------- Modes & states -----------------------
// High-level run state and mode of the machine.
enum class RunState : uint8_t { IDLE, RAMP_UP, RUN, RAMP_DOWN };
enum class Mode     : uint8_t { MANUAL, AUTO };


// ---------------- Encoder (motor) ----------------------
// Incremental/absolute encoder counts from motor shaft.
volatile long     g_enc_count          = 0;   // Incremental counts since last reset (used for skein progress)
volatile long     g_enc_abs            = 0;   // Absolute counts since power-on (used for phase anchoring)
volatile uint8_t  g_enc_prevState      = 0;   // Last AB state for quadrature decode table


// ---------------- RPM sampling -------------------------
// Timing and last measurement for RPM estimation.
unsigned long     g_rpm_lastMs         = 0;   // Timestamp of last RPM sample window
long              g_rpm_lastCounts     = 0;   // Encoder counts at last RPM sample
int16_t           g_rpm_lastValid10    = INT16_MIN;  // Last valid RPM×10 (for UI/debug)
bool              g_stallLatched       = false;      // Latched stall flag to avoid repeated stall toasts


// ----- RPM setpoint slewing (AUTO) ---------------------
// These track the current and desired RPM setpoints and how we slew between them.
int16_t           g_rpmSetpoint10      = 0;           // Current RPM setpoint (×10) used by PI controller
int16_t           g_rpmDesiredSetpoint10 = 0;         // Target RPM setpoint (×10) based on band (FAST/CREEP)
int16_t           g_rpmSlewFrom10      = 0;           // Starting setpoint for current slew
unsigned long     g_rpmSlewStartMs     = 0;           // When the current setpoint slew started
unsigned long     g_rpmSlewDurationMs  = CREEP_TRANSITION_MS; // Slew duration for setpoint changes
int16_t           g_rpmPrevDesired10   = INT16_MIN;   // Last desired setpoint, to detect changes


// ----- PI controller state -----------------------------
// Internal state of the discrete-time PI speed controller.
int32_t           g_iTerm_Q15          = 0;    // Integrator term in Q15 domain (anti-windup bounded)
int               g_ctrlOutPwm         = 0;    // Latest PI output PWM (before clamping to min/max elsewhere)

int16_t           g_speedRpm10Lp       = 0;    // Low-pass filtered RPM×10 (FAST band filtering)
bool              g_speedLpInited      = false;// True once low-pass filter has been initialized
int               g_speedLastPwm       = 0;    // Smoothed PWM command (after IIR smoothing inside PI)
bool              g_speedLastPwmInit   = false;// True once PWM smoothing has been initialized


// ---------------- Rotary (KY-040) ----------------------
// Quadrature rotary encoder used for jog/target selection.
volatile int16_t  g_rot_stepQueue      = 0;    // Accumulated ±1 detent steps to be handled in main loop
volatile uint8_t  g_rot_prevAB         = 0;    // Last AB state for rotary quadrature decode
volatile int8_t   g_rot_accum          = 0;    // Sub-detent accumulator (combines 4 qdec steps into one detent)
constexpr int8_t  ROT_DETENT_STEPS     = 4;    // Number of qdec steps per mechanical detent
volatile unsigned long g_rot_lastDetentUs = 0; // Timestamp of last accepted detent (for bounce filtering)
bool              g_sw_clickAwaiting   = false;// True while waiting to see if a second click forms a double-click
unsigned long     g_sw_lastShortMs     = 0;    // Timestamp of last short click (for double-click detection)


// ------------- Rotary push-button ----------------------
// State and timing for the rotary pushbutton.
int               g_sw_state           = HIGH; // Last debounced button state (HIGH=not pressed, LOW=pressed)
unsigned long     g_sw_lastChangeMs    = 0;    // Last time the button state changed (for debounce)
unsigned long     g_sw_pressedAtMs     = 0;    // Time when the button was pressed (for long-press detection)


// ---------------------- UI -----------------------------
// UI timing, toasts and debug flags.
unsigned long     g_ui_lastMs          = 0;    // Timestamp of last UI refresh
unsigned long     g_toast_untilMs      = 0;    // Toast expiry timestamp (0 = no toast)
uint8_t           g_toast_size         = 2;    // Toast text size
char              g_toast_buf[22];             // Toast text buffer (multi-line with '\n')
char              g_small_buf[22];             // Small reusable text buffer for debug/UI
bool              g_uiDebug            = false;// True when debug UI page is shown instead of normal UI
bool              g_toastWithIcons     = false;// If true, toast is rendered with sheep icons on both sides


// --------------- Manual / jog --------------------------
// Manual jog state and rotary-based target holding.
long              g_manualDispCounts   = 0;    // Displayed counts in MANUAL mode (virtual skein length)
long              g_manualRefEnc       = 0;    // Encoder reference position at start of jog run
long              g_jogTargetRel       = 0;    // Relative jog target (counts offset from g_manualRefEnc)
const long JOG_COUNTS_PER_DETENT =
  (long)((COUNTS_PER_REV + (ROT_DETENTS_PER_REV / 2)) / ROT_DETENTS_PER_REV); // Counts per rotary detent
const long JOG_TOL = max(1L, JOG_COUNTS_PER_DETENT / 2);   // Jog tolerance: half a detent step, minimum 1 count
unsigned long     g_manualSettleUntilMs = 0;   // Delay after stopping before jog corrections are allowed


// -------- MANUAL run tracking --------------------------
// Tracks whether MANUAL mode has actually driven the motor.
bool              g_manualDidPedalRun  = false;// Set true on first pedal press while in MANUAL (used when returning to AUTO)


// --------------- Pedal debounce ------------------------
// Debounced pedal state, separate from raw diagnostics.
bool              g_pedalRaw           = false;// Instantaneous (raw) pedal state (true = pressed)
bool              g_pedalStable        = false;// Debounced pedal state
bool              g_pedalPrevStable    = false;// Previous debounced state (for edge detection)
unsigned long     g_pedalLastChangeMs  = 0;    // Last time raw pedal state changed
unsigned long     g_pedalIgnoreReleaseUntilMs = 0; // Ignore pedal releases until this time (prevents quick glitches)

// Raw pedal diagnostics (LED + edge count)
bool              g_pedalRawPrev       = false;// Previous raw pedal state (for LED mirror)
volatile uint32_t g_pedalRawEdges      = 0;    // Count of raw pedal edges (for diagnosis / noise measurement)

struct PedalEdges {
  bool risingEff;    // Debounced effective "pedal pressed" edge (after ignore window)
  bool fallingEff;   // Debounced effective "pedal released" edge (after ignore window)
};


// ---------------- PWM / AUTO band flags ----------------
// Current PWM command and AUTO band state.
int               g_desiredPwmTarget   = 0;    // PI controller desired duty (AUTO/RUN); applied in loop()
bool              g_inCreepBand        = false;// True when position is inside approach region (creep band active)
int               g_pwmNow             = 0;    // Last PWM actually written to hardware (for stall detection & UI)


// --------------- Targets & FSM -------------------------
// High-level AUTO target and FSM bookkeeping.
int16_t           g_targetTurns10      = 100;  // AUTO target in ×10 turns (default 10.0 turns)
long              g_goalCounts         = 0;    // Target encoder counts for current AUTO skein (including phase bias)
bool              g_dirForward         = true; // Motor direction flag; true = forward, false = reverse
RunState          g_state              = RunState::IDLE; // Current finite state machine state
Mode              g_mode               = Mode::AUTO;     // Current mode: MANUAL or AUTO

long              g_stopLeadFwd        = STOP_LEAD_COUNTS_BASE; // Adaptive stop lead for forward direction
long              g_stopLeadRev        = STOP_LEAD_COUNTS_BASE; // Adaptive stop lead for reverse direction
bool              g_countedThisRun     = false; // True once this skein has been counted towards lifetime/session
bool              g_rampDownPlanned    = false; // True if a planned ramp-down is active for adaptive lead
long              g_encAtRampDown      = 0;     // Encoder count at start of ramp-down (for decel measurement)
unsigned long     g_stateStartMs       = 0;     // Timestamp when current FSM state was entered
int               g_startDuty          = 0;     // PWM at the start of ramp-down (used to shape ramp)
unsigned long     g_settleUntilMs      = 0;     // Time until which we wait before adaptive updates after a run
bool              g_adaptPending       = false; // True if adaptive lead/phase update is pending after settle


// -------------- Phase anchor (fixed) -------------------
// Anchor phase for aligning skeins to a fixed twist angle.
uint16_t          g_phaseAnchorT1000   = 0xFFFF; // Phase anchor in 0..999 mturn; 0xFFFF = unset / disabled

// Phase debug monitor used only for UI.
int16_t           g_dbgLastPhaseErrT1000 = 0;   // Last measured phase error vs anchor in mturns (for debug UI)


// -------------- Lifetime / persist ---------------------
// Session and lifetime counters and persist bookkeeping.
uint32_t          g_skeinSession       = 0;     // Skeins twisted in this power-on session
uint32_t          g_lifetimeSkeins     = 0;     // Skeins twisted over lifetime (loaded/stored via EEPROM)
uint32_t          g_lastSavedSkeinMark = 0;     // Last skein count at which we wrote to EEPROM
long              g_lastEncForLife     = 0;     // Last encoder reading used for runtime/twist integrators
unsigned long     g_lastLoopMs         = 0;     // Last recorded loop timestamp (for runtime integration)
bool              g_persistDirty       = false; // True if persist data changed and needs saving
unsigned long     g_persistLastSaveMs  = 0;     // Last time persistSave() was called
long              g_leadPersistedFwd   = 0;     // Last persisted forward lead (for adapt heuristics)
long              g_leadPersistedRev   = 0;     // Last persisted reverse lead (for adapt heuristics)
uint32_t          g_leadLastSavedSkein = 0;     // Skein count at which we last saved lead values
unsigned long     g_lastRuntimeUpdateMs= 0;     // Last time runtimeSec was updated (1s based)


// Persist structure (dual-slot with checksum + sequence)
// Stores lifetime counters, adaptive values and health metrics.
struct Persist {
  uint32_t magic;             // Magic marker for valid struct ('TWIS')
  uint16_t version;           // Persist struct version (for migrations)
  uint32_t skeins;            // Lifetime skein count
  uint32_t totalTwists;       // Total twist count over lifetime (rough estimate)
  uint32_t runtimeSec;        // Total runtime in seconds
  uint32_t leadFwdCounts;     // Persisted forward stop lead (counts)
  uint32_t leadRevCounts;     // Persisted reverse stop lead (counts)
  uint32_t writeCount;        // Total EEPROM writes for this struct
  uint32_t stallCount;        // Total stall events seen
  uint32_t wdtResetCount;     // Watchdog reset counter (health metric)
  uint32_t powerOnCount;      // Power-on/reset counter
  uint32_t totalStarts;       // Total motor start events (wear indicator)
  uint16_t lastTargetTurns10; // Last used target in ×10 turns
  uint16_t phaseAnchorT1000;  // Persisted phase anchor (0..999, 0xFFFF = unset)
  // int32_t phaseBiasCounts; // Removed in v5: was unused and always 0
  uint32_t seq;               // Slot sequence number (for choosing newest slot)
  uint32_t checksum;          // FNV32 checksum over struct (excluding this field)
};

Persist g_persist;            // Active persist data in RAM

// EEPROM slot layout (wear-levelling, implementation detail)
// Kept here (near Persist) instead of in config because this depends on sizeof(Persist)
// and should not be tweaked for tuning.
constexpr int     EEPROM_SLOT_A        = 0;                       // Base address of slot A
constexpr int     EEPROM_SLOT_SIZE     = sizeof(Persist);         // Size of each slot
constexpr int     EEPROM_SLOT_B        = EEPROM_SLOT_A + EEPROM_SLOT_SIZE; // Base address of slot B
static    uint8_t g_activeSlot         = 0;                       // Currently active EEPROM slot (0=A, 1=B)

#if defined(E2END)
  #if (EEPROM_SLOT_B + EEPROM_SLOT_SIZE - 1) > E2END
    #error "Persist struct + slots exceed EEPROM."
  #endif
#endif

// ================================================================
// [4] INTERRUPT SERVICE ROUTINES (keep them tiny)
// ================================================================

// Forward: shared quadrature decoder for motor encoder (D2/D3)
inline void qdecMotorReadAB_Update();

// External interrupts for motor encoder A/B
void ISR_encA() { qdecMotorReadAB_Update(); }
void ISR_encB() { qdecMotorReadAB_Update(); }

// KY-040 PCINT2 handler (D6/D7), emits ±1 per detent (with min spacing)
// - Uses a 4x4 lookup table to decode quadrature steps
// - Accumulates sub-detent steps until ROT_DETENT_STEPS is reached
// - Enforces a minimum time between detents to reject bounce/double-steps
ISR(PCINT2_vect) {
  // Read current AB state of rotary encoder on D7/D6
  uint8_t a    = (PIND >> 7) & 0x01;  // D7
  uint8_t b    = (PIND >> 6) & 0x01;  // D6
  uint8_t curr = (a << 1) | b;

  // 4-bit index: previous AB (2 bits) + current AB (2 bits)
  static const int8_t rotQdec[16] PROGMEM = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
  };

  uint8_t idx  = (g_rot_prevAB << 2) | curr;
  int8_t  step = pgm_read_byte(&rotQdec[idx]);  // -1, 0, or +1

  if (step) {
    int8_t acc = g_rot_accum + step;

    // Convert accumulated qdec steps into full detents
    if (acc >= ROT_DETENT_STEPS || acc <= -ROT_DETENT_STEPS) {
      unsigned long nowUs = micros();

      // Enforce minimum time between detents (debounce / anti-chatter)
      if ((nowUs - g_rot_lastDetentUs) >= ROT_MIN_DETENT_US) {
        g_rot_stepQueue += (acc > 0) ? +1 : -1;   // Queue one logical detent step
        g_rot_lastDetentUs = nowUs;
      }
      acc = 0;  // Reset accumulator after emitting a detent
    }
    g_rot_accum = acc;
  }

  g_rot_prevAB = curr;
}

// ================================================================
// [5] LOW-LEVEL DRIVERS & UTILS
// ================================================================

// ----------------------------------------------------------------
// Motor driver: PWM output and hard stop
// ----------------------------------------------------------------

// Write PWM duty to the Cytron driver (clamped 0..255) and remember it.
// This is the only place that should call analogWrite(PIN_PWM).
inline void motorAnalogWrite(int duty) {
  if (duty < 0)   duty = 0;
  if (duty > 255) duty = 255;
  g_pwmNow = duty;
  analogWrite(PIN_PWM, duty);
}

// Immediately stop the motor by setting PWM to 0 (no ramp).
inline void motorStopHard() {
  motorAnalogWrite(0);
}


// ----------------------------------------------------------------
// RPM setpoint slewing (AUTO mode)
// ----------------------------------------------------------------
// Smoothly slews the RPM setpoint g_rpmSetpoint10 towards g_rpmDesiredSetpoint10
// over g_rpmSlewDurationMs, using a smoothstep-like cubic profile.
// This prevents abrupt changes in requested speed when changing bands (FAST/CREEP).
inline void updateRpmSlew(unsigned long nowMs) {
  // Detect change in desired setpoint and (re)start slew
  if (g_rpmDesiredSetpoint10 != g_rpmPrevDesired10) {
    g_rpmSlewFrom10    = g_rpmSetpoint10;
    g_rpmSlewStartMs   = nowMs;
    g_rpmPrevDesired10 = g_rpmDesiredSetpoint10;
  }

  // Nothing to do if we are already at the desired setpoint
  if (g_rpmSetpoint10 == g_rpmDesiredSetpoint10) return;

  unsigned long dur = g_rpmSlewDurationMs;
  if (dur == 0UL) {
    // No slew requested: jump directly to desired setpoint
    g_rpmSetpoint10 = g_rpmDesiredSetpoint10;
    return;
  }

  unsigned long elapsed = nowMs - g_rpmSlewStartMs;
  if (elapsed >= dur) {
    // Slew finished, snap exactly to desired setpoint
    g_rpmSetpoint10 = g_rpmDesiredSetpoint10;
    return;
  }

  // Smoothstep cubic: u = 3 t^2 - 2 t^3 in Q15 (t in [0,1])
  const long ONE_Q15 = 1L << 15;
  long tQ = (long)((elapsed << 15) / (long)dur);
  if (tQ < 0)        tQ = 0;
  if (tQ > ONE_Q15)  tQ = ONE_Q15;

  long t2Q  = (tQ * tQ) >> 15;
  long term = (3L << 15) - (2L * tQ);
  long uQ   = (t2Q * term) >> 15;   // smoothstep(t) in Q15

  int  span = (int)g_rpmDesiredSetpoint10 - (int)g_rpmSlewFrom10;
  long inc  = ((long)span * uQ + (1L << 14)) >> 15;
  int  out  = (int)g_rpmSlewFrom10 + (int)inc;

  g_rpmSetpoint10 = (int16_t)out;
}


// ----------------------------------------------------------------
// Atomic encoder access helpers
// ----------------------------------------------------------------

// Atomically read incremental encoder count (g_enc_count).
inline long encAtomicRead() {
  long c;
  noInterrupts();
  c = g_enc_count;
  interrupts();
  return c;
}

// Atomically write incremental encoder count (g_enc_count).
inline void encAtomicWrite(long v) {
  noInterrupts();
  g_enc_count = v;
  interrupts();
}

// Atomically read absolute encoder count (g_enc_abs).
inline long encAbsAtomicRead() {
  long c;
  noInterrupts();
  c = g_enc_abs;
  interrupts();
  return c;
}


// ----------------------------------------------------------------
// Counts ↔ turns conversion utilities
// ----------------------------------------------------------------

// t10 = tenths of a turn, returns encoder counts (rounded)
inline long turns10_to_counts(long t10) {
  long num = t10 * (long)COUNTS_PER_REV + 5;  // +5 → round to nearest
  return num / 10L;
}

// counts → tenths of a turn (rounded)
inline long counts_to_turns10(long counts) {
  long num = counts * 10L + (COUNTS_PER_REV / 2);
  return num / (long)COUNTS_PER_REV;
}

// counts → milli-turns (0.001 turn) (rounded)
inline long counts_to_turns1000(long counts) {
  long num = counts * 1000L + (COUNTS_PER_REV / 2);
  return num / (long)COUNTS_PER_REV;
}

// milli-turns → counts (rounded)
inline long turns1000_to_counts(long t1000) {
  long num = t1000 * (long)COUNTS_PER_REV + 500L;
  return num / 1000L;
}


// ----------------------------------------------------------------
// QDEC (motor encoder, D2/D3 using lookup table)
// ----------------------------------------------------------------
// Quadrature decode using a 4×4 transition table in PROGMEM.
// Updates both incremental and absolute count.
static const int8_t qdecTable[16] PROGMEM = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

inline void qdecMotorReadAB_Update() {
  // Read current AB state of motor encoder on D2/D3
  uint8_t a    = (PIND >> 2) & 0x01; // D2
  uint8_t b    = (PIND >> 3) & 0x01; // D3
  uint8_t curr = (a << 1) | b;

  uint8_t idx  = (g_enc_prevState << 2) | curr;
  int8_t  step = pgm_read_byte(&qdecTable[idx]);  // -1, 0, or +1

  if (step) {
    g_enc_count += step;  // Incremental counts (can be reset)
    g_enc_abs   += step;  // Absolute counts (never reset)
  }
  g_enc_prevState = curr;
}


// ----------------------------------------------------------------
// PCINT setup helper
// ----------------------------------------------------------------
// Enable pin change interrupt for a given Arduino pin.
static void pciSetup(uint8_t pin) {
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));  // Enable mask bit
  PCIFR  |= bit(digitalPinToPCICRbit(pin));                   // Clear any pending flag
  PCICR  |= bit(digitalPinToPCICRbit(pin));                   // Enable PCINT group
}


// ----------------------------------------------------------------
// Phase helpers (for phase-anchored AUTO stops)
// ----------------------------------------------------------------
// Represent phase as 0..999 "milliturns" (0.000..0.999 of a full turn).

// Compute current phase (0..999) from absolute encoder counts.
static inline uint16_t phaseFromAbsCountsT1000(long absCounts) {
  long t1000 = counts_to_turns1000(absCounts);   // total milliturns
  long phase = t1000 % 1000L;                   // wrap to 0..999
  return (phase < 0) ? phase + 1000 : phase;
}

// Compute signed phase error between current phase and anchor.
// Result is in range [-500..+500] milliturns (shortest direction).
static inline int16_t phaseErrorT1000(uint16_t phase, uint16_t anchor) {
  // Compute shortest signed distance between two phases (0-999)
  // If error > 500 mturns, it's shorter to go the other direction
  int16_t e = (int16_t)phase - (int16_t)anchor;
  if (e >  500) e -= 1000; // Wrap: e.g., 600 → -400 (shorter)
  if (e < -500) e += 1000; // Wrap: e.g., -600 → 400
  return e;
}

// ================================================================
// [6] SERVICES (RPM, rotary, pedal, persist, UI helpers)
// ================================================================


// ----------------------------------------------------------------
// RPM estimation service
// ----------------------------------------------------------------
// Estimate motor speed in ×10 RPM using encoder counts over a fixed time window.
// Returns INT16_MIN when not enough time has passed since the last sample.
int16_t rpm10SinceWindow() {
    unsigned long now = millis();
    unsigned long dt = now - g_rpm_lastMs;
    if (dt < RPM_WINDOW_MS) return INT16_MIN;
    
    long cNow = encAtomicRead();
    long delta = cNow - g_rpm_lastCounts;
    g_rpm_lastCounts = cNow;
    g_rpm_lastMs = now;
    if (dt == 0) return INT16_MIN;
    
    // RPM×10 = (delta_counts / COUNTS_PER_REV) / (dt_ms / 60000_ms) × 10
    //        = (delta × 600000) / (COUNTS_PER_REV × dt)
    long num = delta * 600000L;  // 60000 ms/min × 10 scale factor
    long den = (long)COUNTS_PER_REV * (long)dt;
    long v = (num >= 0) ? ((num + den/2) / den) : ((num - den/2) / den);
    
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
}


// ----------------------------------------------------------------
// Toast helpers (UI overlays)
// ----------------------------------------------------------------

// Show a constant (flash-stored) message as a toast for a given duration.
inline void showToastC(const __FlashStringHelper* s, unsigned long dur, uint8_t size) {
  strncpy_P(g_toast_buf, (PGM_P)s, sizeof(g_toast_buf));
  g_toast_buf[sizeof(g_toast_buf) - 1] = 0;
  g_toast_untilMs   = millis() + dur;
  g_toast_size      = size;
  g_toastWithIcons  = false;
}

// Show a RAM string as a toast for a given duration.
inline void showToastDyn(const char* s, unsigned long dur, uint8_t size) {
  strncpy(g_toast_buf, s, sizeof(g_toast_buf));
  g_toast_buf[sizeof(g_toast_buf) - 1] = 0;
  g_toast_untilMs   = millis() + dur;
  g_toast_size      = size;
  g_toastWithIcons  = false;
}

// Show a skein-completed toast with big text and sheep icons.
inline void showSkeinToast(uint32_t skeins, unsigned long dur) {
  snprintf_P(g_toast_buf, sizeof(g_toast_buf), PSTR("%lu\nskeins!"), (unsigned long)skeins);
  g_toast_untilMs   = millis() + dur;
  g_toast_size      = 3;
  g_toastWithIcons  = true;
}


// ----------------------------------------------------------------
// Rotary encoder + button service
// ----------------------------------------------------------------
// Handles accumulated detent steps from ISR, mode-dependent behaviour,
// and the rotary pushbutton (click, double-click, long-press).
void rotaryServiceFromQueue() {
  // --- Handle rotary detent steps (position changes) ---
  int16_t delta;
  noInterrupts();
  delta = g_rot_stepQueue;
  g_rot_stepQueue = 0;
  interrupts();

  if (delta != 0) {
    if (g_mode == Mode::MANUAL) {
      // In MANUAL, rotary adjusts jog target around a reference point.
      long deltaCounts = (long)delta * ROT_SIGN * JOG_COUNTS_PER_DETENT;
      g_jogTargetRel  += deltaCounts;
    } else {
      // In AUTO, rotary adjusts target turns in coarse steps.
      int16_t t = g_targetTurns10 + delta * TARGET_STEP_TURNS10;
      if (t < 0)                   t = 0;
      if (t > TARGET_STEP_TURNS_LIMIT) t = TARGET_STEP_TURNS_LIMIT;

      // Quantize to full steps of TARGET_STEP_TURNS10.
      int16_t q = (t + (TARGET_STEP_TURNS10 / 2)) / TARGET_STEP_TURNS10;
      g_targetTurns10 = q * TARGET_STEP_TURNS10;
      g_persistDirty  = true;
    }
  }

  // --- Handle rotary pushbutton (click/double/long) ---
  unsigned long now     = millis();
  int           reading = digitalRead(ROT_SW); // active LOW

  if (reading != g_sw_state && (now - g_sw_lastChangeMs) > SW_DEBOUNCE_MS) {
    bool wasLow      = (g_sw_state == LOW);
    g_sw_state       = reading;
    g_sw_lastChangeMs = now;

    if (g_sw_state == LOW) {
      // Button pressed
      g_sw_pressedAtMs = now;
    } else if (wasLow && g_sw_state == HIGH) {
      // Button released
      unsigned long dur = now - g_sw_pressedAtMs;

      if (g_sw_clickAwaiting && (now - g_sw_lastShortMs) > 0) {
        // Second release; if it is a long press, cancel click sequence
        if (dur >= SW_LONG1_MS) g_sw_clickAwaiting = false;
      }

      if (dur >= SW_LONG2_MS) {
        // Very long press: toggle debug UI
        g_uiDebug = !g_uiDebug;
        showToastC(g_uiDebug ? F("DEBUG ON") : F("DEBUG OFF"), 700, 2);
      }
      else if (dur >= SW_LONG1_MS) {
        // Long press: toggle direction
        g_dirForward = !g_dirForward;
        showToastC(g_dirForward ? F("DIR: FWD") : F("DIR: REV"), 700, 2);
      }
      else {
        // Short press: single or double click
        if (g_sw_clickAwaiting && (now - g_sw_lastShortMs) <= SW_DBLCLICK_MS) {
          // Double-click: show lifetime splash (health stats)
          g_sw_clickAwaiting = false;
          uiLifetimeSplashTopRightLogo();
        } else {
          // Start waiting to see if a second click follows
          g_sw_clickAwaiting = true;
          g_sw_lastShortMs   = now;
        }
      }
    }
  }

  // --- Handle single-click timeout: mode toggle ---
  if (g_sw_clickAwaiting && (millis() - g_sw_lastShortMs) > SW_DBLCLICK_MS) {
    g_sw_clickAwaiting = false;

    Mode prev = g_mode;
    g_mode    = (g_mode == Mode::MANUAL) ? Mode::AUTO : Mode::MANUAL;

    if (g_mode == Mode::MANUAL) {
      // --- ENTER MANUAL: display equals current AUTO target (jog-proof) ---
      long encNow           = encAtomicRead();             // Reference for jog
      g_manualRefEnc        = encNow;
      g_jogTargetRel        = 0;

      // Show twists in MANUAL based on AUTO target, not encoder.
      g_manualDispCounts    = turns10_to_counts(g_targetTurns10);

      g_manualSettleUntilMs = millis() + 120;
      motorAnalogWrite(0);
      g_state               = RunState::IDLE;

      g_manualDidPedalRun   = false;

      // Reset speed controller when leaving MANUAL (keep AUTO loop clean)
      resetSpeedControllerState();

    } else if (prev == Mode::MANUAL && g_mode == Mode::AUTO) {
      // --- ENTER AUTO from MANUAL: set phase anchor and optionally update target ---

      // Phase anchor is always (re)set when returning to AUTO
      uint16_t ph = phaseFromAbsCountsT1000(encAbsAtomicRead());
      g_phaseAnchorT1000 = ph;
      persistMarkDirty();
      showToastC(F("ANCHOR SET"), 700, 2);

      // Only update AUTO target if there was a pedal-driven MANUAL run
      if (g_manualDidPedalRun) {
        long cDisp     = labs(g_manualDispCounts);
        long t10       = counts_to_turns10(cDisp);
        long rounded10 = ((t10 + 5) / 10) * 10;         // round to whole twists

        if (rounded10 < 0)      rounded10 = 0;
        if (rounded10 > 65535)  rounded10 = 65535;

        g_targetTurns10 = (int16_t)rounded10;
        g_persistDirty  = true;
      }

      motorAnalogWrite(0);

      // Reset speed controller when entering AUTO (fresh loop for next skein)
      resetSpeedControllerState();
    }
  }
}


// ----------------------------------------------------------------
// Pedal services: debounce, raw read, diagnostics
// ----------------------------------------------------------------

// Debounce the foot pedal and report effective rising/falling edges.
PedalEdges pedalService() {
  unsigned long now = millis();
  int           pr  = digitalRead(PIN_PEDAL); // LOW = pressed
  bool          raw = (pr == LOW);

  if (raw != g_pedalRaw) {
    g_pedalRaw         = raw;
    g_pedalLastChangeMs = now;
  }

  unsigned long need = g_pedalRaw ? PEDAL_DB_DOWN_MS : PEDAL_DB_UP_MS;

  bool rising  = false;
  bool falling = false;

  // State change only after required debounce time.
  if ((now - g_pedalLastChangeMs) >= need && g_pedalStable != g_pedalRaw) {
    g_pedalPrevStable = g_pedalStable;
    g_pedalStable     = g_pedalRaw;
    rising            = (!g_pedalPrevStable && g_pedalStable);
    falling           = ( g_pedalPrevStable && !g_pedalStable);
  }

  // Apply ignore window to releases (to prevent very quick dropouts).
  bool risingEff  = rising;
  bool fallingEff = falling && (now >= g_pedalIgnoreReleaseUntilMs);

  return { risingEff, fallingEff };
}

// Returns direct pedal pin state (true = pressed, LOW on pin).
bool pedalRawNow() {
  return (digitalRead(PIN_PEDAL) == LOW);
}

// Detect raw pedal edges, mirror to LED_BUILTIN and count edges for diagnostics.
void pedalRawEdgeService() {
  bool rawNow = pedalRawNow();

  if (rawNow != g_pedalRawPrev) {
    g_pedalRawPrev = rawNow;
    digitalWrite(LED_BUILTIN, rawNow ? HIGH : LOW); // LED on when pressed
    // (Optionally, raw edge counter could be incremented here if needed)
  }
}


// ----------------------------------------------------------------
// EEPROM / persist helpers
// ----------------------------------------------------------------

// Simple 32-bit FNV-like checksum over a byte buffer.
uint32_t simpleChecksum32(const uint8_t* p, size_t n) {
  uint32_t s = 0;
  for (size_t i = 0; i < n; ++i) {
    s = (s * 16777619u) ^ p[i];
  }
  return s;
}

// Raw EEPROM read helper.
inline void eepromReadRaw(int base, void* buf, size_t n) {
  uint8_t* b = (uint8_t*)buf;
  for (size_t i = 0; i < n; ++i) {
    b[i] = EEPROM.read(base + (int)i);
  }
}

// Raw EEPROM write helper using EEPROM.update() to reduce wear.
inline void eepromWriteRaw(int base, const void* buf, size_t n) {
  const uint8_t* b = (const uint8_t*)buf;
  for (size_t i = 0; i < n; ++i) {
    EEPROM.update(base + (int)i, b[i]);
  }
}

// Mark the in-RAM persist struct as dirty so loop() can save later.
inline void persistMarkDirty() {
  g_persistDirty = true;
}


// ----------------------------------------------------------------
// Persist load: read active slot, validate, or initialize defaults
// ----------------------------------------------------------------
void persistLoad() {
  Persist a, b;
  bool    aValid = false;
  bool    bValid = false;

  // Read and validate slot A
  eepromReadRaw(EEPROM_SLOT_A, &a, sizeof(Persist));
  if (a.magic == PERSIST_MAGIC && a.version == PERSIST_VER) {
    uint32_t cs = simpleChecksum32((const uint8_t*)&a, sizeof(Persist) - sizeof(a.checksum));
    aValid = (cs == a.checksum);
  }

  // Read and validate slot B
  eepromReadRaw(EEPROM_SLOT_B, &b, sizeof(Persist));
  if (b.magic == PERSIST_MAGIC && b.version == PERSIST_VER) {
    uint32_t cs = simpleChecksum32((const uint8_t*)&b, sizeof(Persist) - sizeof(b.checksum));
    bValid = (cs == b.checksum);
  }

  if (!aValid && !bValid) {
    // No valid slot: initialise new struct with defaults.
    memset(&g_persist, 0, sizeof(g_persist));
    g_persist.magic          = PERSIST_MAGIC;
    g_persist.version        = PERSIST_VER;
    g_persist.leadFwdCounts  = STOP_LEAD_COUNTS_BASE;
    g_persist.leadRevCounts  = STOP_LEAD_COUNTS_BASE;
    g_persist.lastTargetTurns10 = 100;
    g_persist.phaseAnchorT1000  = 0xFFFF;   // no anchor set
    // New v5 fields are zeroed by memset.
    g_persist.seq = 1;
    g_persist.checksum = simpleChecksum32((const uint8_t*)&g_persist,
                                          sizeof(Persist) - sizeof(g_persist.checksum));
    eepromWriteRaw(EEPROM_SLOT_A, &g_persist, sizeof(Persist));
    g_activeSlot = 0;
  } else {
    // Pick the newest valid slot (by seq) if both valid.
    if (aValid && bValid) {
      g_activeSlot = (b.seq > a.seq) ? 1 : 0;
    } else {
      g_activeSlot = aValid ? 0 : 1;
    }
    g_persist = (g_activeSlot == 0) ? a : b;
  }

  // Pull lifetime skeins and related mirrors to RAM
  g_lifetimeSkeins      = g_persist.skeins;
  g_skeinSession        = 0;
  g_lastSavedSkeinMark  = (g_lifetimeSkeins / SAVE_EVERY_SKEINS) * SAVE_EVERY_SKEINS;

  // Mirrors for lead-persist / adapt heuristics
  g_leadPersistedFwd    = (long)g_persist.leadFwdCounts;
  g_leadPersistedRev    = (long)g_persist.leadRevCounts;
  g_leadLastSavedSkein  = g_lifetimeSkeins;  // "last saved skein" at boot = current EEPROM value

  // Phase anchor mirror
  g_phaseAnchorT1000    = g_persist.phaseAnchorT1000;
}


// ----------------------------------------------------------------
// Persist save: update struct, checksum, and write to alternate slot
// ----------------------------------------------------------------
void persistSave() {
  // Update persistent counters and adaptive parameters from RAM mirrors.
  g_persist.skeins           = g_lifetimeSkeins;
  g_persist.leadFwdCounts    = (uint32_t)g_stopLeadFwd;
  g_persist.leadRevCounts    = (uint32_t)g_stopLeadRev;
  g_persist.lastTargetTurns10= (uint16_t)g_targetTurns10;

  // Update lead mirrors in RAM.
  g_leadPersistedFwd         = g_stopLeadFwd;
  g_leadPersistedRev         = g_stopLeadRev;
  g_leadLastSavedSkein       = g_lifetimeSkeins;

  // Phase anchor (phaseBiasCounts removed in v5).
  g_persist.phaseAnchorT1000 = g_phaseAnchorT1000;

  // Bump write counter and sequence number.
  g_persist.writeCount++;
  g_persist.seq = (g_persist.seq == 0xFFFFFFFFUL) ? 1UL : (g_persist.seq + 1UL);

  // Recompute checksum over struct excluding the checksum field itself.
  g_persist.checksum = simpleChecksum32((const uint8_t*)&g_persist,
                                        sizeof(Persist) - sizeof(g_persist.checksum));

  // Watchdog around EEPROM writes to avoid WDT reset during write.
  wdt_reset();
  uint8_t nextSlot = (g_activeSlot == 0) ? 1 : 0;
  int     base     = (nextSlot == 0) ? EEPROM_SLOT_A : EEPROM_SLOT_B;
  eepromWriteRaw(base, &g_persist, sizeof(Persist));
  g_activeSlot = nextSlot;
  wdt_reset();

  g_persistDirty        = false;
  g_persistLastSaveMs   = millis();
  g_lastSavedSkeinMark  = (g_lifetimeSkeins / SAVE_EVERY_SKEINS) * SAVE_EVERY_SKEINS;
}

// ================================================================
// [7] UI RENDERING (compact & debug)
// ================================================================

// Small sheep logo used in splash/toast screens.
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


// ----------------------------------------------------------------
// Basic text / bitmap helpers
// ----------------------------------------------------------------

// Draw a PROGMEM bitmap horizontally flipped at (x,y).
void drawBitmapHFlip(int16_t x, int16_t y,
                     const uint8_t* bm, int16_t w, int16_t h,
                     uint16_t color) {
  const int16_t bpr = (w + 7) / 8; // bytes per row
  for (int16_t r = 0; r < h; ++r) {
    const uint8_t* rp = bm + (r * bpr);
    for (int16_t c = 0; c < w; ++c) {
      int16_t sc = w - 1 - c;              // source column (flipped)
      uint8_t b  = pgm_read_byte(rp + (sc >> 3));
      uint8_t m  = (uint8_t)(0x80 >> (sc & 7));
      if (b & m) {
        display.drawPixel(x + c, y + r, color);
      }
    }
  }
}

// Compute pixel width of a string at a given text size.
int16_t textWidthPxC(const char* s, uint8_t sz = 1) {
  return (int16_t)strlen(s) * 6 * sz;  // 5px glyph + 1px spacing
}

// Draw centered text (horizontally) at given y and size.
void drawCenteredC(const char* s, int16_t y, uint8_t sz = 1) {
  int16_t x = (SCREEN_WIDTH - textWidthPxC(s, sz)) / 2;
  if (x < 0) x = 0;
  display.setTextSize(sz);
  display.setCursor(x, y);
  display.print(s);
}

// Draw right-aligned text at given y and size.
void drawRightC(const char* s, int16_t y, uint8_t sz = 1) {
  int16_t x = SCREEN_WIDTH - textWidthPxC(s, sz);
  if (x < 0) x = 0;
  display.setTextSize(sz);
  display.setCursor(x, y);
  display.print(s);
}

static inline bool stateIsIdle(RunState st) { return st == RunState::IDLE; }


// ----------------------------------------------------------------
// Top row: skeins + mode or RPM
// ----------------------------------------------------------------
// Shows "N skeins" on the left and either MODE or RPM on the right.
void drawTopRowDynamic(uint32_t skeins, RunState st, Mode m, int16_t rpm10) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print(skeins);
  display.print(F(" skeins"));

  if (stateIsIdle(st)) {
    // Show mode when idle
    drawRightC((m == Mode::MANUAL) ? "MANUAL" : "AUTO", 0, 1);
  } else {
    // Show RPM while running
    int rpm = (rpm10 == INT16_MIN)
              ? -1
              : ((rpm10 < 0 ? -rpm10 : rpm10) + 5) / 10;
    char rb[20];
    if (rpm < 0) {
      snprintf_P(rb, sizeof(rb), PSTR("RPM --"));
    } else {
      snprintf_P(rb, sizeof(rb), PSTR("RPM %d"), rpm);
    }
    drawRightC(rb, 0, 1);
  }
}


// ----------------------------------------------------------------
// Big twists display helpers
// ----------------------------------------------------------------

// Choose big font size depending on digits and optional decimal.
static uint8_t chooseBigSizeForTurns10(long t10, bool showDecimal) {
  long abs10 = (t10 < 0) ? -t10 : t10;
  long whole = abs10 / 10;
  int  digits = 1;
  for (long x = whole; x >= 10; x /= 10) digits++;

  int extra   = ((t10 < 0) ? 1 : 0) + (showDecimal ? 2 : 0); // sign + decimal '.' and one digit
  int total   = digits + extra;

  return (total <= 5) ? 4 : 3;  // 4x for up to 5 chars, else 3x
}

// Draw the main twist count in large font, optionally with "twists" label.
void drawBigTwists(long t10, int16_t yTop, bool showUnit = true) {
  bool    showDecimal = false;  // currently not used; whole twists only
  uint8_t sz          = chooseBigSizeForTurns10(t10, showDecimal);
  char    buf[16];

  if (showDecimal) {
    long a = (t10 < 0) ? -t10 : t10;
    snprintf_P(buf, sizeof(buf), PSTR("%s%ld.%ld"),
               (t10 < 0) ? "-" : "",
               a / 10,
               a % 10);
  } else {
    long v     = (t10 < 0) ? -t10 : t10;
    long whole = v / 10;
    snprintf_P(buf, sizeof(buf), PSTR("%s%ld"),
               (t10 < 0) ? "-" : "",
               whole);
  }

  drawCenteredC(buf, yTop, sz);

  if (showUnit) {
    int16_t unitY = yTop + (8 * sz) + 2;
    drawCenteredC("twists", unitY, 1);
  }
}


// ----------------------------------------------------------------
// Progress bar (AUTO mode)
// ----------------------------------------------------------------
constexpr int16_t PROG_X = 2;
constexpr int16_t PROG_Y = 56;
constexpr int16_t PROG_W = 124;
constexpr int16_t PROG_H = 8;

// Draw a bottom progress bar for AUTO mode, based on encoder vs goal.
void drawProgressBarBottom(long goalCountsLocal, long encLocal) {
  if (goalCountsLocal <= 0) return;

  long progressCounts = labs(encLocal);
  long p1000 = (progressCounts * 1000L + goalCountsLocal / 2) / goalCountsLocal;
  if (p1000 < 0)    p1000 = 0;
  if (p1000 > 1000) p1000 = 1000;

  display.drawRect(PROG_X, PROG_Y, PROG_W, PROG_H, SSD1306_WHITE);

  int16_t innerW = PROG_W - 2;
  int16_t fillW  = (int16_t)((innerW * p1000 + 500L) / 1000L);
  if (fillW > 0) {
    display.fillRect(PROG_X + 1, PROG_Y + 1, fillW, PROG_H - 2, SSD1306_WHITE);
  }
}


// ----------------------------------------------------------------
// Toast overlay renderer
// ----------------------------------------------------------------
// If a toast is active, draws it full-screen (optionally with sheep icons)
// and returns true. Otherwise returns false so normal UI can be drawn.
bool drawToastIfAny() {
  if (millis() >= g_toast_untilMs) return false;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Count lines (supports up to 3 lines; centered vertically).
  uint8_t lines = 1;
  for (const char* p = g_toast_buf; *p; ++p) {
    if (*p == '\n') ++lines;
  }
  if (lines > 3) lines = 3;

  uint8_t sz      = g_toast_size;
  int     lineH   = 8 * sz;
  int     vGap    = 2;
  int     totalH  = (int)lines * lineH + (int)(lines - 1) * vGap;
  int16_t y       = (SCREEN_HEIGHT - totalH) / 2;

  // Render each line
  const char* s = g_toast_buf;
  for (uint8_t i = 0; i < lines; ++i) {
    const char* e = s;
    while (*e && *e != '\n') ++e;

    char   lineBuf[22];
    size_t len = (size_t)(e - s);
    if (len >= sizeof(lineBuf)) len = sizeof(lineBuf) - 1;
    memcpy(lineBuf, s, len);
    lineBuf[len] = '\0';

    if (i == 0 && g_toastWithIcons) {
      // Text metrics for first line
      int16_t textW = textWidthPxC(lineBuf, sz);
      int16_t textH = lineH;

      // Icon geometry (fixed at screen edges)
      const int16_t iconW  = LOGO_TOAST_W;
      const int16_t iconH  = LOGO_TOAST_H;
      const int16_t margin = 1;
      const int16_t gap    = 4;

      const int16_t xLeftIcon  = margin;
      const int16_t xRightIcon = SCREEN_WIDTH - margin - iconW;
      int16_t       yIcon      = y + (textH - iconH) / 2;
      if (yIcon < 0) yIcon = 0;

      const int16_t innerX0 = xLeftIcon + iconW + gap;
      const int16_t innerX1 = xRightIcon - gap;
      int16_t       innerW  = innerX1 - innerX0;
      if (innerW < 0) innerW = 0;

      int16_t xText = (textW <= innerW)
                      ? innerX0 + (innerW - textW) / 2
                      : innerX0;

      drawBitmapHFlip(xLeftIcon, yIcon,
                      LOGO_SHEEP_28x24, iconW, iconH, SSD1306_WHITE);
      display.setTextSize(sz);
      display.setCursor(xText, y);
      display.print(lineBuf);
      display.drawBitmap(xRightIcon, yIcon,
                         LOGO_SHEEP_28x24, iconW, iconH, SSD1306_WHITE);
    } else {
      // Centered text line without icons
      int16_t x = (SCREEN_WIDTH - textWidthPxC(lineBuf, sz)) / 2;
      if (x < 0) x = 0;
      display.setTextSize(sz);
      display.setCursor(x, y);
      display.print(lineBuf);
    }

    y += lineH + vGap;
    s  = (*e == '\n') ? (e + 1) : e;
  }

  display.display();
  return true;
}


// ----------------------------------------------------------------
// Normal UI screens (MANUAL / AUTO / DEBUG)
// ----------------------------------------------------------------

// MANUAL mode screen: big twist count based on manual display counts.
void drawUI_ManualSimple(int16_t rpm10, long /*encCounts*/) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  drawTopRowDynamic(g_skeinSession, g_state, g_mode, rpm10);

  long shownT10 = counts_to_turns10(g_manualDispCounts);
  drawBigTwists(shownT10, 20, true);

  display.display();
}

// AUTO mode screen: big remaining twist count + progress bar.
void drawUI_AutoSimple(int16_t rpm10,
                       RunState st,
                       long goalCountsLocal,
                       long encLocal,
                       int16_t targetTurns10Local) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  drawTopRowDynamic(g_skeinSession, st, g_mode, rpm10);

  long centerT10;
  if (st == RunState::IDLE || goalCountsLocal == 0) {
    // Show target twists when idle or when no valid goal yet.
    centerT10 = targetTurns10Local;
    drawBigTwists(centerT10, 20, true);
  } else {
    // Show remaining twists during run.
    long remaining = goalCountsLocal - labs(encLocal);
    if (remaining < 0) remaining = 0;
    centerT10 = counts_to_turns10(remaining);
    drawBigTwists(centerT10, 20, false);
  }

  // Progress bar only during active run with valid goal.
  if ((st != RunState::IDLE) && (goalCountsLocal > 0)) {
    drawProgressBarBottom(goalCountsLocal, encLocal);
  }

  display.display();
}

#if ENABLE_DEBUG_UI
void drawUI_Debug(int16_t rpm10,
                  long pulses,
                  long turns10,
                  int pwm,
                  bool /*pedalPressed*/,
                  RunState st,
                  Mode m,
                  long tgt10) {
  (void)pulses;  // not shown in this layout

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // Row 0: Mode | State
  display.setCursor(0, 0);
  display.print(F("Mode:"));
  display.print(m == Mode::MANUAL ? F("MAN") : F("AUTO"));

  display.setCursor(64, 0);
  display.print(F("State:"));
  switch (st) {
    case RunState::IDLE:      display.print(F("IDLE")); break;
    case RunState::RAMP_UP:   display.print(F("RUP"));  break;
    case RunState::RUN:       display.print(F("RUN"));  break;
    case RunState::RAMP_DOWN: display.print(F("RDN"));  break;
  }

  // --- RPM values ---
  int16_t spRPM  = g_rpmSetpoint10 / 10;
  int16_t actRPM = (rpm10 == INT16_MIN) ? 0 : (rpm10 / 10);

  // Row 1: RPM setpoint / actual
  // "RPM S/A 165/162"
  display.setCursor(0, 10);
  display.print(F("RPM S/A "));
  display.print(spRPM);
  display.print('/');
  display.print(actRPM);

  // --- Twists S/A (1 decimal) ---
  long tS10 = tgt10;    // target ×10 turns
  long tA10 = turns10;  // actual ×10 turns

  long tS_abs = (tS10 < 0) ? -tS10 : tS10;
  long tA_abs = (tA10 < 0) ? -tA10 : tA10;

  long tS_whole = tS_abs / 10;
  long tS_frac  = tS_abs % 10;
  long tA_whole = tA_abs / 10;
  long tA_frac  = tA_abs % 10;

  // Row 2: "Tw S/A  xx.x/ yy.y"
  display.setCursor(0, 20);
  snprintf_P(g_small_buf, sizeof(g_small_buf),
             PSTR("Tw S/A %s%ld.%ld/%s%ld.%ld"),
             (tS10 < 0) ? "-" : "",
             tS_whole, tS_frac,
             (tA10 < 0) ? "-" : "",
             tA_whole, tA_frac);
  display.print(g_small_buf);

  // Row 3: PWM & band
  display.setCursor(0, 30);
  display.print(F("PWM:"));
  display.print(pwm);

  display.setCursor(64, 30);
  display.print(F("Band:"));
  display.print(g_inCreepBand ? F("CREEP") : F("FAST"));

  // Row 4: Phase / Anchor / Error
  uint16_t phNow = phaseFromAbsCountsT1000(encAbsAtomicRead());
  uint16_t anc   = g_phaseAnchorT1000;                     // 0..999 (0xFFFF = unset)
  int16_t  errT  = (anc == 0xFFFF) ? 0 : phaseErrorT1000(phNow, anc); // P - A

  display.setCursor(0, 40);
  if (anc == 0xFFFF) {
    snprintf_P(g_small_buf, sizeof(g_small_buf),
               PSTR("P:%3u A:--- e:%+d"),
               phNow, g_dbgLastPhaseErrT1000);
  } else {
    snprintf_P(g_small_buf, sizeof(g_small_buf),
               PSTR("P:%3u A:%3u e:%+d"),
               phNow, anc, errT);
  }
  display.print(g_small_buf);

  // Row 5: Leads F/R
  long t1000F = counts_to_turns1000(g_stopLeadFwd);
  long t1000R = counts_to_turns1000(g_stopLeadRev);
  long aF     = (t1000F < 0) ? -t1000F : t1000F;
  long aR     = (t1000R < 0) ? -t1000R : t1000R;

  display.setCursor(0, 52);
  snprintf_P(g_small_buf, sizeof(g_small_buf),
             PSTR("Lead F/R %ld.%03ld/%ld.%03ld"),
             aF / 1000, aF % 1000,
             aR / 1000, aR % 1000);
  display.print(g_small_buf);

  display.display();
}
#endif

// ----------------------------------------------------------------
// Lifetime splash screen (boot-time health overview)
// ----------------------------------------------------------------
// Boot screen shows lifetime counters and health metrics in a compact layout,
// with the sheep logo in the top-right corner.
void uiLifetimeSplashTopRightLogo() {
  unsigned long t0 = millis();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  const int16_t xLogo = SCREEN_WIDTH - LOGO_TOAST_W - 1;
  const int16_t yLogo = 1;
  display.drawBitmap(xLogo, yLogo,
                     LOGO_SHEEP_28x24, LOGO_TOAST_W, LOGO_TOAST_H, SSD1306_WHITE);

  unsigned long totalSec = g_persist.runtimeSec;
  unsigned long hh       = totalSec / 3600UL;
  unsigned long mm       = (totalSec % 3600UL) / 60UL;

  // Compact health-focused layout
  display.setCursor(0, 0);
  display.println(F("SkeinMachine v3.27"));

  display.setCursor(0, 12);
  display.print(F("Skeins: "));
  display.println(g_lifetimeSkeins);

  display.setCursor(0, 20);
  display.print(F("Writes: "));
  display.println(g_persist.writeCount);

  display.setCursor(0, 30);
  display.print(F("Stalls: "));
  display.print(g_persist.stallCount);
  display.setCursor(72, 30);
  display.print(F("WDT: "));
  display.println(g_persist.wdtResetCount);

  display.setCursor(0, 40);
  display.print(F("Starts: "));
  display.print(g_persist.totalStarts);
  display.setCursor(72, 40);
  display.print(F("Boots: "));
  display.println(g_persist.powerOnCount);

  display.setCursor(0, 50);
  display.print(F("Runtime: "));
  display.print(hh);
  display.print(F("h "));
  display.print(mm);
  display.println(F("m"));

  display.display();

  // Keep splash visible for configured time, petting the watchdog.
  while (millis() - t0 < LIFETIME_SPLASH_MS) {
    wdt_reset();
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
  g_iTerm_Q15            = 0;
  g_rpmSetpoint10        = 0;
  g_rpmDesiredSetpoint10 = 0;
  g_rpmPrevDesired10     = INT16_MIN;

  // Internal filters / smoothing
  g_speedRpm10Lp         = 0;
  g_speedLpInited        = false;
  g_speedLastPwm         = 0;
  g_speedLastPwmInit     = false;
}

// Closed-loop PI speed controller, called whenever a fresh RPM sample is available.
//
// - Only active in AUTO mode
// - Only processes samples in RAMP_UP and RUN
//   * RAMP_UP: PI runs but does NOT drive PWM yet (still open-loop ramp)
//   * RUN:     PI output (with smoothing) drives g_desiredPwmTarget
// - Uses Q15 gains and a small deadband to avoid chattering around setpoint
// - All PWM smoothing is handled here via g_speedLastPwm (no extra PWM slew layer)
void speedControllerOnRpmSample(int16_t rpm10) {
  // Only handle AUTO mode and active motion states
  if (g_mode != Mode::AUTO) return;
  if (g_state != RunState::RAMP_UP && g_state != RunState::RUN) return;
  if (rpm10 == INT16_MIN) return;  // invalid sample

  bool inRampUp = (g_state == RunState::RAMP_UP);

  // ----------------------------------------------------------------
  // RPM filtering
  // ----------------------------------------------------------------
  // In creep band: use raw RPM (we care about slow, precise behaviour).
  // In fast band: use a low-pass filtered RPM to reduce noise.
  int16_t rpm10_f;
  if (g_inCreepBand) {
    rpm10_f = rpm10;
  } else {
    if (!g_speedLpInited) {
      g_speedRpm10Lp  = rpm10;
      g_speedLpInited = true;
    }
    int32_t d = (int32_t)rpm10 - (int32_t)g_speedRpm10Lp;
    g_speedRpm10Lp += (int16_t)((d + 1) / RPM_FILTER_DIV);  // ~1/3 step update
    rpm10_f = g_speedRpm10Lp;
  }

  // ----------------------------------------------------------------
  // Setpoint handling
  // ----------------------------------------------------------------
  int16_t sp10 = g_rpmSetpoint10;
  if (sp10 <= 0) {
    // No speed requested: reset integrator and bail out.
    g_iTerm_Q15 = 0;
    return;
  }

  // Feedforward (duty ≈ a*RPM + b) from linear fit.
  int16_t spRPM = (sp10 + 5) / 10;
  int32_t u_ff  = FF_B_DUTY
                + ((int32_t)spRPM * FF_A_NUM + (FF_A_DEN / 2)) / FF_A_DEN;

  // ----------------------------------------------------------------
  // Error (in integer RPM units)
  // ----------------------------------------------------------------
  int16_t err10  = sp10 - rpm10_f;
  int16_t errRPM = (err10 >= 0)
                 ? ((err10 + 5) / 10)
                 : ((err10 - 5) / 10);

  // ----------------------------------------------------------------
  // Gains (FAST vs CREEP)
  // ----------------------------------------------------------------
  int16_t Kp = Kp_Q15;
  int16_t Ki = Ki_Q15;
  if (g_inCreepBand) {
    Kp = Kp_CREEP_Q15;
    Ki = Ki_CREEP_Q15;
  }

  // Deadband around zero error to reduce chatter.
  int8_t dband = g_inCreepBand ? 1 : RPM_DBAND_RPM;

  // ----------------------------------------------------------------
  // Proportional term (Q15 domain)
  // ----------------------------------------------------------------
  int32_t p_Q15 = 0;
  if (abs(errRPM) > dband) {
    p_Q15 = (int32_t)Kp * (int32_t)errRPM;
  }

  // ----------------------------------------------------------------
  // Conditional integration with anti-windup
  // ----------------------------------------------------------------
  if (Ki > 0 && abs(errRPM) > dband) {
    // Tentative PI output to check for saturation direction.
    int32_t tentative_Q15 = p_Q15 + g_iTerm_Q15;
    int32_t tentative_pwm = (tentative_Q15 >> 8) + u_ff;

    bool atUpper     = tentative_pwm >= (CTRL_PWM_MAX - 5);
    bool atLower     = tentative_pwm <= (CTRL_PWM_MIN + 5);
    bool drivesUpper = (errRPM > 0);
    bool drivesLower = (errRPM < 0);

    // Integrate only if we are not obviously pushing further into saturation.
    if (!(atUpper && drivesUpper) && !(atLower && drivesLower)) {
      g_iTerm_Q15 += (int32_t)Ki * (int32_t)errRPM;
      if (g_iTerm_Q15 > ITERM_MAX) g_iTerm_Q15 = ITERM_MAX;
      if (g_iTerm_Q15 < ITERM_MIN) g_iTerm_Q15 = ITERM_MIN;
    }
  }

  // ----------------------------------------------------------------
  // Combine FF + PI into PWM command
  // ----------------------------------------------------------------
  int32_t u_Q15 = p_Q15 + g_iTerm_Q15;
  int32_t u_pwm = (u_Q15 >> 8) + u_ff;

  // Clamp to controller limits.
  if (u_pwm < CTRL_PWM_MIN) u_pwm = CTRL_PWM_MIN;
  if (u_pwm > CTRL_PWM_MAX) u_pwm = CTRL_PWM_MAX;

  // ----------------------------------------------------------------
  // Output smoothing (per RPM sample)
  // ----------------------------------------------------------------
  // We smooth inside the controller instead of using a separate PwmSlew layer.
  if (!g_speedLastPwmInit) {
    g_speedLastPwm     = (int)u_pwm;
    g_speedLastPwmInit = true;
  } else {
    int delta = (int)u_pwm - g_speedLastPwm;
    if (g_inCreepBand) {
      // Heavier smoothing in creep band for less "tugging"
      g_speedLastPwm = g_speedLastPwm + ((delta + 2) / 4);
    } else {
      // Tunable smoothing in fast band via ALPHA_NUM / ALPHA_DEN
      g_speedLastPwm += (delta * ALPHA_NUM) / ALPHA_DEN;
    }
  }

  // ----------------------------------------------------------------
  // Apply controller output
  // ----------------------------------------------------------------
  // In RAMP_UP we run the PI internally but keep using the open-loop ramp.
  // In RUN we hand off PWM control to the PI controller.
  if (!inRampUp) {
    g_desiredPwmTarget = g_speedLastPwm;
  }
}

// ================================================================
// [9] FINITE STATE MACHINE
// ================================================================
void updateFSM(const PedalEdges& ped, unsigned long now) {
  // ----------------------------------------------------------------
  // Lifetime / runtime trackers
  // ----------------------------------------------------------------
  // Runtime is accumulated in seconds using a coarse 1s tick.
  unsigned long dtMs = now - g_lastRuntimeUpdateMs;
  if (dtMs >= 1000UL) {
    g_persist.runtimeSec      += dtMs / 1000UL;
    g_lastRuntimeUpdateMs     += (dtMs / 1000UL) * 1000UL;
  }

  // ----------------------------------------------------------------
  // MANUAL MODE
  // ----------------------------------------------------------------
  if (g_mode == Mode::MANUAL) {
    // --- Pedal edges in MANUAL ---
    if (ped.risingEff) {
      // Pedal pressed → start a MANUAL run (RAMP_UP)
      g_manualDidPedalRun   = true;

      g_state               = RunState::RAMP_UP;
      g_stateStartMs        = now;
      g_startDuty           = 0;

      digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);

      // Count this motor start for health monitoring.
      g_persist.totalStarts++;

      // Reset encoder and MANUAL display counter for this run.
      encAtomicWrite(0);
      g_manualRefEnc        = 0;
      g_manualDispCounts    = 0;

      // Ignore very quick pedal releases right after start.
      g_pedalIgnoreReleaseUntilMs = now + PEDAL_RELEASE_IGNORE_MS;
    }
    else if (ped.fallingEff) {
      // Pedal released → ramp down to stop.
      g_state        = RunState::RAMP_DOWN;
      g_stateStartMs = now;
      g_startDuty    = g_pwmNow;
    }

    // --- MANUAL FSM ---
    switch (g_state) {
      case RunState::RAMP_UP: {
        // Open-loop ramp to full MANUAL jog speed (PWM_FAST).
        g_manualDispCounts = encAtomicRead();
        digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);

        unsigned long elapsed = now - g_stateStartMs;

        if (elapsed <= START_KICK_MS) {
          // Short kick to overcome static friction.
          motorAnalogWrite(START_KICK_PWM);
        } else {
          if (elapsed >= rampUpMs) {
            motorAnalogWrite(PWM_FAST);
            g_state = RunState::RUN;
          } else {
            int duty = (int)(((long)PWM_FAST * (long)elapsed) / (long)rampUpMs);
            motorAnalogWrite(duty);
          }
        }
      } break;

      case RunState::RUN: {
        // Constant-speed MANUAL run (full PWM) until pedal release.
        g_manualDispCounts = encAtomicRead();
        digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);

        if (ped.fallingEff) {
          g_state        = RunState::RAMP_DOWN;
          g_stateStartMs = now;
          g_startDuty    = g_pwmNow;
          break;
        }
        motorAnalogWrite(PWM_FAST);
      } break;

      case RunState::RAMP_DOWN: {
        // Open-loop ramp down from last duty to zero.
        g_manualDispCounts = encAtomicRead();
        digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);

        unsigned long elapsed = now - g_stateStartMs;
        bool         finishNow = false;
        long         duty      = g_startDuty;

        if (elapsed >= rampDownMs) {
          finishNow = true;
        } else {
          duty = g_startDuty
               - ((long)g_startDuty * (long)elapsed) / (long)rampDownMs;
          if (duty < MIN_RAMP_PWM) finishNow = true;
        }

        if (finishNow) {
          motorStopHard();

          // After stop, update jog reference and allow fine positioning again.
          long encNow2        = encAtomicRead();
          g_manualRefEnc      = encNow2;
          g_jogTargetRel      = 0;
          g_manualSettleUntilMs = now + 120;
          g_state             = RunState::IDLE;
        } else {
          if (g_startDuty < MIN_RAMP_PWM) g_startDuty = MIN_RAMP_PWM;
          motorAnalogWrite((int)duty);
        }
      } break;

      case RunState::IDLE:
      default: {
        // IDLE in MANUAL: jog controller keeps position near jog target.
        motorAnalogWrite(0);

        if (g_manualSettleUntilMs == 0 || now >= g_manualSettleUntilMs) {
          long encNow    = encAtomicRead();
          long targetEnc = g_manualRefEnc + g_jogTargetRel;
          long err       = targetEnc - encNow;

          if (err > JOG_TOL) {
            digitalWrite(PIN_DIR, HIGH);
            motorAnalogWrite(PWM_JOG);
          } else if (err < -JOG_TOL) {
            digitalWrite(PIN_DIR, LOW);
            motorAnalogWrite(PWM_JOG);
          } else {
            motorAnalogWrite(0);
          }
        } else {
          motorAnalogWrite(0);
        }
      } break;
    }
    return;  // MANUAL handled completely
  }

  // ----------------------------------------------------------------
  // AUTO MODE
  // ----------------------------------------------------------------
  switch (g_state) {
    // --------------------------------------------------------------
    // AUTO: IDLE → RAMP_UP (pedal press starts skein)
    // --------------------------------------------------------------
    case RunState::IDLE: {
      if (ped.risingEff) {
        digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);

        if (g_targetTurns10 > 0) {
          // New AUTO skein: zero incremental encoder, reset lifetime marker.
          encAtomicWrite(0);
          g_lastEncForLife = 0;

          // Count this motor start for health monitoring.
          g_persist.totalStarts++;

          // Convert target twists → encoder counts.
          g_goalCounts = turns10_to_counts(g_targetTurns10);

          // Optional phase start-bias toward anchor (if set).
          if (g_phaseAnchorT1000 != 0xFFFF) {
            uint16_t phNow       = phaseFromAbsCountsT1000(encAbsAtomicRead());
            int16_t  deltaT1000  = phaseErrorT1000(g_phaseAnchorT1000, phNow); // desired - current
            int      dirSign     = g_dirForward ? +1 : -1;
            long     cpm         = turns1000_to_counts(1);                     // counts per mturn
            long     biasCounts  = (long)dirSign * (long)deltaT1000 * cpm;
            long     halfTurn    = turns1000_to_counts(500);

            if (biasCounts >  halfTurn) biasCounts =  halfTurn;
            if (biasCounts < -halfTurn) biasCounts = -halfTurn;
            g_goalCounts += biasCounts;
          }

          // Reset RPM sampling base.
          g_rpm_lastCounts  = encAtomicRead();
          g_rpm_lastMs      = now;
          g_rpm_lastValid10 = INT16_MIN;

          // Fresh start for the speed controller for this new AUTO run.
          resetSpeedControllerState();

          long totalCounts = g_goalCounts;
          bool smallTarget = (totalCounts <= (APPROACH_COUNTS
                                              + STOP_LEAD_COUNTS_BASE
                                              + 309));

          // Initial band & RPM setpoint:
          //  - small targets start directly in creep band
          //  - others start in fast band.
          g_inCreepBand          = smallTarget;
          g_rpmDesiredSetpoint10 = g_inCreepBand ? RPM_CREEP_10 : RPM_FAST_10;
          g_rpmSlewDurationMs    = g_inCreepBand ? CREEP_SLEW_MS
                                                 : CREEP_TRANSITION_MS;
          g_rpmSetpoint10        = g_rpmDesiredSetpoint10;  // start at target
          g_rpmPrevDesired10     = g_rpmDesiredSetpoint10;
          g_iTerm_Q15            = 0;                       // reset integrator

          // PWM side: start at zero, PI will ramp it.
          g_desiredPwmTarget     = 0;

          g_countedThisRun   = false;
          g_rampDownPlanned  = false;
          g_state            = RunState::RAMP_UP;
          g_stateStartMs     = now;
          motorAnalogWrite(0);

          // Early kick duty used as base for open-loop ramp.
          g_startDuty        = START_KICK_PWM;
        }
      }
    } break;

    // --------------------------------------------------------------
    // AUTO: RAMP_UP (open-loop PWM ramp to approximate duty)
    // --------------------------------------------------------------
    case RunState::RAMP_UP: {
      // Second pedal press during ramp-up → abort into ramp-down.
      if (ped.risingEff) {
        g_startDuty        = g_pwmNow;
        g_state            = RunState::RAMP_DOWN;
        g_stateStartMs     = now;
        g_rampDownPlanned  = false;
        break;
      }

      unsigned long elapsed = now - g_stateStartMs;

      // Early kick to break static friction.
      if (elapsed <= START_KICK_MS) {
        int duty = START_KICK_PWM;
        g_desiredPwmTarget = duty;
        motorAnalogWrite(duty);
      } else {
        unsigned long rampElapsed = elapsed - START_KICK_MS;

        // Compute approximate duty for target RPM in current band.
        int16_t targetRPM  = (g_inCreepBand ? RPM_CREEP_10 : RPM_FAST_10) / 10;
        int32_t targetDuty = FF_B_DUTY
                           + ((int32_t)targetRPM * FF_A_NUM + (FF_A_DEN / 2)) / FF_A_DEN;
        if (targetDuty <  60) targetDuty =  60;
        if (targetDuty > 255) targetDuty = 255;

        // End of RAMP_UP → transition to RUN with preloaded PI state.
        if (rampElapsed >= rampUpMs) {
          // Slew parameters for RPM setpoint (fast/creep band).
          g_rpmSlewDurationMs    = g_inCreepBand ? CREEP_SLEW_MS
                                                 : CREEP_TRANSITION_MS;
          g_rpmDesiredSetpoint10 = g_inCreepBand ? RPM_CREEP_10 : RPM_FAST_10;
          g_rpmSetpoint10        = g_rpmDesiredSetpoint10;
          g_rpmPrevDesired10     = g_rpmDesiredSetpoint10;

          // Set controller's output and requested duty to the ramp target.
          g_desiredPwmTarget     = (int)targetDuty;

          // Preload integrator so PI output matches current duty (no step).
          int32_t ff             = FF_B_DUTY
                                 + ((int32_t)targetRPM * FF_A_NUM + (FF_A_DEN / 2)) / FF_A_DEN;
          g_iTerm_Q15            = ((int32_t)targetDuty - ff) << 8;  // scaled to Q15

          // Clamp integrator to safety limits.
          if (g_iTerm_Q15 > ITERM_MAX) g_iTerm_Q15 = ITERM_MAX;
          if (g_iTerm_Q15 < ITERM_MIN) g_iTerm_Q15 = ITERM_MIN;

          g_state        = RunState::RUN;
          g_stateStartMs = millis();  // fresh RUN timer
        } else {
          // Still ramping: linear ramp from START_KICK_PWM to targetDuty.
          int duty = START_KICK_PWM
                   + (int)(((long)(targetDuty - START_KICK_PWM)
                            * (long)rampElapsed) / (long)rampUpMs);

          g_desiredPwmTarget = duty;
          motorAnalogWrite(duty);
        }
      }
    } break;

    // --------------------------------------------------------------
    // AUTO: RUN (closed-loop speed control + position-based band logic)
    // --------------------------------------------------------------
    case RunState::RUN: {
      // Second pedal press during RUN → early abort into ramp-down.
      if (ped.risingEff) {
        g_startDuty        = g_pwmNow;
        g_state            = RunState::RAMP_DOWN;
        g_stateStartMs     = now;
        g_rampDownPlanned  = false;
        break;
      }

      long encNow   = encAtomicRead();
      long progress = labs(encNow);
      long error    = g_goalCounts - progress;
      long stopLeadNow = g_dirForward ? g_stopLeadFwd : g_stopLeadRev;
      (void)stopLeadNow;

      // --- Position-based band switching and RPM setpoint selection ---
      long approachEnter = APPROACH_COUNTS;

      if (!g_inCreepBand) {
        // Still in FAST band: when we enter the approach zone we switch to creep.
        if (error <= approachEnter) {
          g_inCreepBand          = true;
          g_rpmDesiredSetpoint10 = RPM_CREEP_10;
          g_rpmSlewDurationMs    = CREEP_SLEW_MS;
          // Reduce integrator magnitude when entering creep for smoother behaviour.
          g_iTerm_Q15            = g_iTerm_Q15 / 3;
        } else {
          // Stay in FAST band.
          g_rpmDesiredSetpoint10 = RPM_FAST_10;
          g_rpmSlewDurationMs    = CREEP_TRANSITION_MS;
        }
      } else {
        // Once in creep band, we stay in creep until the run completes.
        g_rpmDesiredSetpoint10 = RPM_CREEP_10;
        g_rpmSlewDurationMs    = CREEP_SLEW_MS;
      }

      // Slew only the RPM setpoint (not PWM).
      updateRpmSlew(now);

      // --- Stop condition & adaptive stop lead ---
      if (error <= stopLeadNow && error > 0) {
        // We have reached the adaptive "start decel" point for this direction.
        if (!g_countedThisRun) {
          g_countedThisRun = true;
          g_skeinSession++;
          g_lifetimeSkeins++;

          long countsThisSkein  = labs(encAtomicRead());
          long twistsThisSkein  = countsThisSkein / COUNTS_PER_REV;
          g_persist.totalTwists += (uint32_t)twistsThisSkein;

          if (!g_uiDebug) {
            showSkeinToast(g_skeinSession, 2000);
          }
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
        // Safety: if we somehow pass goalCounts without planned ramp-down.
        motorStopHard();
        g_state = RunState::IDLE;
      }
    } break;

    // --------------------------------------------------------------
    // AUTO: RAMP_DOWN (open-loop decel + adaptive lead update)
    // --------------------------------------------------------------
    case RunState::RAMP_DOWN: {
      // Disable closed-loop speed control: open-loop PWM deceleration.
      digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);

      unsigned long elapsed = now - g_stateStartMs;
      bool          finishNow = false;
      long          duty      = g_startDuty;

      if (elapsed >= rampDownMs) {
        finishNow = true;
      } else {
        duty = g_startDuty
             - ((long)g_startDuty * (long)elapsed) / (long)rampDownMs;
        if (duty < MIN_RAMP_PWM) finishNow = true;
      }

      if (finishNow) {
        motorStopHard();

        // Count total twists for this run (for lifetime stats).
        long countsThisRun   = labs(encAtomicRead());
        long twistsThisRun   = countsThisRun / COUNTS_PER_REV;
        g_persist.totalTwists += (uint32_t)twistsThisRun;

        // Mark for adaptive stop-lead update after the motor has fully settled.
        if (g_rampDownPlanned) {
          g_settleUntilMs   = now + (RPM_WINDOW_MS + 100);
          g_adaptPending    = true;
          g_rampDownPlanned = false;
        }

        g_state = RunState::IDLE;

        // Reset speed controller for next run.
        resetSpeedControllerState();
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
        #if ENABLE_DEBUG_UI
        // Debug mode: show detailed system state
        long t10 = counts_to_turns10(pulses);      
        drawUI_Debug(g_rpm_lastValid10, pulses, t10, g_pwmNow, g_pedalStable, g_state, g_mode, g_targetTurns10);
        #endif 
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