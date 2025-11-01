/* ================================================================
 * Skein Calibrator v1.06 — pedal edge restore + UI pedal indicator
 * Fixes vs v1.05:
 *  - Restore original robust pedalService() (separate down/up debounce)
 *  - Show pedal state on UI ("Ped:UP/DN") for quick diagnostics
 * ================================================================ */

// ===============================
// [1] TUNABLES (adjust here)
// ===============================
constexpr uint8_t  OLED_ADDR         = 0x3C;
constexpr uint8_t  OLED_ROTATION     = 2;

// PWM & speed
constexpr int      PWM_CREEP         = 40;      // CAL slow spin duty   ← tune
constexpr int      PWM_TEST          = 180;     // TEST fast duty        ← tune
constexpr int      PWM_APPROACH      = 60;      // TEST slow near goal   ← tune
constexpr unsigned rampUpMs          = 350;     // ramp-up ms            ← tune
constexpr unsigned rampDownMs        = 25;      // ramp-down ms          ← tune
constexpr int      MIN_RAMP_PWM      = 5;

// Debounce / UI cadence
constexpr unsigned UI_REFRESH_MS     = 50;
constexpr unsigned SW_DEBOUNCE_MS    = 25;
constexpr unsigned SW_LONG_MS        = 1000;
constexpr unsigned PEDAL_DB_DOWN_MS  = 2;       // ← your original values
constexpr unsigned PEDAL_DB_UP_MS    = 12;      // ← your original values

// Test parameters
constexpr uint8_t  TEST_ROUNDS       = 5;

// Rotary timing
constexpr unsigned ROT_MIN_DETENT_US = 1200;
constexpr int8_t   ROT_DETENT_STEPS  = 4;

// ---- CAL outlier control ----
constexpr float    CAL_OUTLIER_PCT   = 0.12f;
constexpr long     CAL_MIN_DELTA     = 50;

// ---- Speed estimation & predictive stop ----
constexpr unsigned SPEED_SAMPLE_MS    = 20;
constexpr uint8_t  SPEED_ALPHA_Q8     = 64;     // 0.25
constexpr uint8_t  STOP_LOOK_Q8       = 256;    // 1.00x
constexpr long     MIN_LOOKAHEAD_PUL  = 5;
constexpr long     MAX_LOOKAHEAD_PUL  = 20000;

// ---- Approach & brake thresholds (fractions of CPR, Q8=value/256) ----
constexpr uint8_t  APPROACH_Q8        = 90;     // ~0.35×CPR remaining ← tune
constexpr uint8_t  BRAKE_TRIG_Q8      = 6;      // ~0.023×CPR remaining ← tune

// ---- Active brake parameters ----
constexpr unsigned BRAKE_MS           = 18;     // reverse pulse duration ← tune
constexpr int      BRAKE_PWM          = 70;     // reverse pulse PWM      ← tune
constexpr bool     BRAKE_ENABLE       = true;

// ---- Brake bias & final trim (unchanged vs v1.05) ----
constexpr int16_t  BRAKE_COMP_PULSES  = -40;    // ← voor jouw ~-47 pulsen ondershoot
constexpr bool     TRIM_ENABLE        = true;
constexpr int      TRIM_PWM           = 45;
constexpr unsigned TRIM_MAX_MS        = 350;
constexpr long     TRIM_STOP_BAND_PUL = 2;

// ===============================
// [2] PIN MAP
// ===============================
constexpr uint8_t PIN_PEDAL = 5;
constexpr uint8_t PIN_DIR   = 8;
constexpr uint8_t PIN_PWM   = 9;
constexpr uint8_t ENC_A     = 2;
constexpr uint8_t ENC_B     = 3;
constexpr uint8_t ROT_CLK   = 7;
constexpr uint8_t ROT_DT    = 6;
constexpr uint8_t ROT_SW    = 4;

// ===============================
// [3] INCLUDES & GLOBALS
// ===============================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <avr/interrupt.h>

Adafruit_SSD1306 display(128, 64, &Wire, -1);

enum class Mode : uint8_t { CAL, TEST };
enum class RunState : uint8_t { IDLE, RAMP_UP, RUN, APPROACH, RAMP_DOWN, BRAKE, TRIM, HOLD };

// ---- Define structs ----
struct PedalEdges { bool press; bool release; };
struct SWEvents   { bool shortClick; bool longHoldEdge; bool isHeld; };

// ---- Globals ----
volatile long    g_enc_count      = 0;
volatile uint8_t g_enc_prevState  = 0;

volatile int16_t g_rot_stepQueue  = 0;
volatile uint8_t g_rot_prevAB     = 0;
volatile int8_t  g_rot_accum      = 0;
volatile unsigned long g_rot_lastDetentUs = 0;

int              g_pwmNow         = 0;
RunState         g_state          = RunState::IDLE;
Mode             g_mode           = Mode::CAL;
bool             g_dirForward     = true;

unsigned long    g_stateStartMs   = 0;
int              g_startDuty      = 0;

// --- CAL ---
bool             g_cal_run        = false;
bool             g_cal_armed      = false;
long             g_cal_startCount = 0;
long             g_cal_curCount   = 0;
long             g_cal_prevCount  = 0;
uint32_t         g_cal_sum        = 0;
uint16_t         g_cal_n          = 0;
bool             g_cal_lastWasSkip = false;

// --- CPR ---
long             g_cpr_setting    = 3072;   // default; user can overwrite via CAL

// --- TEST bookkeeping ---
long             g_test_goalPulses = 0;
long             g_test_startCount = 0;
bool             g_test_pressedRotate = false;

// thresholds (from CPR)
long             g_approachPulses  = 0;
long             g_brakeTrigPulses = 0;

// UI & switches
unsigned long    g_ui_lastMs      = 0;
int              g_sw_state       = HIGH;
unsigned long    g_sw_lastChange  = 0;
unsigned long    g_sw_pressedAt   = 0;
bool             g_sw_longLatched = false;

// --- Pedal debounce state (as in your original) ---
bool             g_pedalRaw       = false;       // instantaneous (LOW = pressed)
bool             g_pedalStable    = false;       // debounced level
unsigned long    g_pedalLastChange= 0;

// speed estimation
unsigned long    g_spd_lastMs     = 0;
long             g_spd_lastCnt    = 0;
uint32_t         g_ppms_q15       = 0;

// brake/trim timing
unsigned long    g_brakeStartMs   = 0;
unsigned long    g_trimStartMs    = 0;

// ===============================
// [4] ISRs: motor & rotary
// ===============================
static const int8_t qdecTable[16] PROGMEM = {
  0,-1,+1,0,  +1,0,0,-1,  -1,0,0,+1,  0,+1,-1,0
};

inline void qdecMotorReadAB_Update() {
  uint8_t a = (PIND >> 2) & 0x01; // D2
  uint8_t b = (PIND >> 3) & 0x01; // D3
  uint8_t curr = (a << 1) | b;
  uint8_t idx = (g_enc_prevState << 2) | curr;
  int8_t step = pgm_read_byte(&qdecTable[idx]);
  if (step) g_enc_count += step;
  g_enc_prevState = curr;
}
void ISR_encA(){ qdecMotorReadAB_Update(); }
void ISR_encB(){ qdecMotorReadAB_Update(); }

// Rotary via PCINT on D6/D7
ISR(PCINT2_vect) {
  uint8_t a = (PIND >> 7) & 0x01; // D7
  uint8_t b = (PIND >> 6) & 0x01; // D6
  uint8_t curr = (a << 1) | b;
  static const int8_t rotQdec[16] PROGMEM = {
    0,-1,+1,0, +1,0,0,-1, -1,0,0,+1, 0,+1,-1,0
  };
  uint8_t idx = (g_rot_prevAB << 2) | curr;
  int8_t step = pgm_read_byte(&rotQdec[idx]);
  if (step) {
    int8_t acc = g_rot_accum + step;
    if (acc >= ROT_DETENT_STEPS || acc <= -ROT_DETENT_STEPS) {
      unsigned long nowUs = micros();
      if ((nowUs - g_rot_lastDetentUs) >= ROT_MIN_DETENT_US) {
        g_rot_stepQueue += (acc > 0) ? +1 : -1;
        g_rot_lastDetentUs = nowUs;
      }
      acc = 0;
    }
    g_rot_accum = acc;
  }
  g_rot_prevAB = curr;
}

// ===============================
// [5] LOW-LEVEL HELPERS
// ===============================
inline void motorAnalogWrite(int duty){
  if (duty < 0) duty = 0; if (duty > 255) duty = 255;
  g_pwmNow = duty; analogWrite(PIN_PWM, duty);
}
inline void motorStopHard(){ motorAnalogWrite(0); }
inline long encRead(){ long c; noInterrupts(); c = g_enc_count; interrupts(); return c; }

static void pciSetup(uint8_t pin){
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));
  PCIFR  |= bit(digitalPinToPCICRbit(pin));
  PCICR  |= bit(digitalPinToPCICRbit(pin));
}

inline void rampTo(int target, unsigned long now, unsigned upMs, unsigned downMs){
  static unsigned long tStart=0; static int start=0; static int tgt=0; static bool active=false;
  if (!active || tgt!=target){
    tStart = now; start = g_pwmNow; tgt = target; active=true;
  }
  unsigned dur = (target>=g_pwmNow) ? upMs : downMs;
  unsigned long dt = now - tStart;
  if (dt >= dur){ motorAnalogWrite(target); active=false; return; }
  long num = (long)(target - start) * (long)dt;
  int val = start + (int)( num / (long)dur );
  motorAnalogWrite(val);
}

// ===============================
// [6] INPUT & SPEED SERVICES
// ===============================
/* pedalService() — debounced edges (press/release) for active-LOW pedal
 * Uses separate debounce for down/up and returns one-shot edges.
 * Adjust PEDAL_DB_* at top if needed.
 */
PedalEdges pedalService(){
  unsigned long now = millis();
  bool raw = (digitalRead(PIN_PEDAL) == LOW);      // LOW = pressed
  if (raw != g_pedalRaw){                          // raw transition seen
    g_pedalRaw = raw;
    g_pedalLastChange = now;
  }
  bool press=false, release=false;
  // use asymmetric debounce: press=fast, release=slower
  if ((now - g_pedalLastChange) >= (g_pedalRaw ? PEDAL_DB_DOWN_MS : PEDAL_DB_UP_MS)){
    if (g_pedalStable != g_pedalRaw){
      press   = (!g_pedalStable && g_pedalRaw);
      release = ( g_pedalStable && !g_pedalRaw);
      g_pedalStable = g_pedalRaw;
    }
  }
  return {press, release};
}

SWEvents rotSwService(){
  unsigned long now = millis();
  int rd = digitalRead(ROT_SW); // LOW=pressed
  bool shortClick=false, longEdge=false;

  if (rd != g_sw_state && (now - g_sw_lastChange) > SW_DEBOUNCE_MS){
    bool wasLow = (g_sw_state==LOW);
    g_sw_state = rd; g_sw_lastChange = now;
    if (g_sw_state == LOW){
      g_sw_pressedAt = now;
      g_sw_longLatched = false;
    } else if (wasLow && g_sw_state == HIGH){
      unsigned long dur = now - g_sw_pressedAt;
      if (dur < SW_LONG_MS) shortClick = true;
      g_sw_longLatched = false;
    }
  }
  if (g_sw_state == LOW && !g_sw_longLatched && (now - g_sw_pressedAt) >= SW_LONG_MS){
    longEdge = true;
    g_sw_longLatched = true;
  }

  bool held = (g_sw_state == LOW);
  return {shortClick, longEdge, held};
}

void rotaryStepsServiceForTest(bool pressedHeld){
  int16_t delta;
  noInterrupts(); delta = g_rot_stepQueue; g_rot_stepQueue = 0; interrupts();
  if (delta == 0) return;
  long step = pressedHeld ? 10 : 1;
  g_cpr_setting += (long)delta * step;
  if (g_cpr_setting < 10) g_cpr_setting = 10;
}

// Estimate speed (ppms) as EMA in Q15
void speedService(unsigned long nowMs){
  if (g_spd_lastMs == 0){ g_spd_lastMs = nowMs; g_spd_lastCnt = encRead(); return; }
  unsigned long dms = nowMs - g_spd_lastMs;
  if (dms < SPEED_SAMPLE_MS) return;
  long cnt = encRead();
  long dp  = cnt - g_spd_lastCnt; if (dp < 0) dp = -dp;
  g_spd_lastCnt = cnt; g_spd_lastMs = nowMs;

  uint32_t inst_q15 = (dms > 0) ? ( ( (uint32_t)dp << 15 ) / dms ) : 0;
  int32_t diff = (int32_t)inst_q15 - (int32_t)g_ppms_q15;
  g_ppms_q15 = (uint32_t)((int32_t)g_ppms_q15 + ( (int32_t)SPEED_ALPHA_Q8 * diff >> 8 ));
  if (g_ppms_q15 > (uint32_t)(MAX_LOOKAHEAD_PUL << 15)) g_ppms_q15 = (uint32_t)(MAX_LOOKAHEAD_PUL << 15);
}

// ===============================
// [7] UI HELPERS
// ===============================
void drawCentered(const char* s, int16_t y, uint8_t sz=2){
  int16_t w = strlen(s) * 6 * sz;
  int16_t x = (128 - w)/2; if (x<0) x=0;
  display.setTextSize(sz); display.setCursor(x,y); display.print(s);
}

void uiCAL(){
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,0); display.print(F("CAL  CPR=")); display.print(g_cpr_setting);
  display.setCursor(88,0); display.print(g_cal_run ? F("RUN") : F("STOP"));

  long enc = encRead();
  g_cal_curCount = (g_cal_armed) ? (enc - g_cal_startCount) : 0;

  display.setCursor(0,12); display.print(F("CUR : ")); display.print(g_cal_curCount);
  display.setCursor(0,22); display.print(F("PREV: ")); display.print(g_cal_prevCount);

  long avg = (g_cal_n>0) ? (long)((g_cal_sum + g_cal_n/2) / g_cal_n) : g_cal_prevCount;
  display.setCursor(0,34); display.print(F("AVG : ")); display.print(avg);
  display.setCursor(0,44); display.print(F("N   : ")); display.print(g_cal_n);

  display.setCursor(88,44); display.print(g_cal_lastWasSkip ? F("SKIP") : F("OK"));

  display.setCursor(0,56);
  display.print(F("Ped:")); display.print(g_pedalStable ? F("DN ") : F("UP "));
  display.print(F("  SW:Mark(arm/rec)"));
  display.display();
}

void uiTEST(){
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,0); display.print(F("TEST CPR=")); display.print(g_cpr_setting);

  long enc = encRead();
  long prog = enc - g_test_startCount; if (prog < 0) prog = -prog;
  long goal = g_test_goalPulses;
  long rounds_x100 = (g_cpr_setting>0) ? ( (prog * 100L + g_cpr_setting/2) / g_cpr_setting ) : 0;

  display.setCursor(0,14); display.print(F("RND : "));
  display.print(rounds_x100/100); display.print('.'); int r = rounds_x100%100; if (r<10) display.print('0'); display.print(r);
  display.setCursor(0,26); display.print(F("PUL : ")); display.print(prog); display.print(F(" / ")); display.print(goal);

  display.setCursor(0,38); display.print(F("State: "));
  switch (g_state){
    case RunState::IDLE:      display.print(F("IDLE")); break;
    case RunState::RAMP_UP:   display.print(F("RUP")); break;
    case RunState::RUN:       display.print(F("RUN")); break;
    case RunState::APPROACH:  display.print(F("APR")); break;
    case RunState::RAMP_DOWN: display.print(F("RDN")); break;
    case RunState::BRAKE:     display.print(F("BRK")); break;
    case RunState::TRIM:      display.print(F("TRM")); break;
    case RunState::HOLD:      display.print(F("HOLD")); break;
  }

  display.setCursor(0,50);
  display.print(F("Ped:")); display.print(g_pedalStable ? F("DN ") : F("UP "));
  display.print(F(" Rot:+/-1 Hold:+/-10"));
  display.display();
}

// ===============================
// [8] FSM & LOGIC
// ===============================
static inline bool cal_isOutlier(long delta, long refAvg){
  if (delta < CAL_MIN_DELTA) return true;
  if (refAvg <= 0) return false;
  long diff = delta - refAvg; if (diff < 0) diff = -diff;
  long band = (long)(refAvg * CAL_OUTLIER_PCT + 0.5f);
  return (diff > band);
}

void fsmCAL(const PedalEdges& ped, const SWEvents& sw, unsigned long now){
  if (ped.press){
    g_cal_run = !g_cal_run;
    if (g_cal_run){
      g_cal_armed = false;
      g_cal_curCount = 0;
      g_cal_startCount = encRead();
    }
  }

  if (sw.longHoldEdge){
    g_cal_prevCount = (g_cpr_setting > 0) ? g_cpr_setting : g_cal_prevCount;
    g_cal_sum = 0; g_cal_n = 0;
    g_mode = Mode::TEST;
    motorStopHard(); g_state = RunState::IDLE;
    g_test_goalPulses = g_cpr_setting * (long)TEST_ROUNDS;
    g_test_startCount = encRead();
    return;
  }

  if (sw.shortClick){
    long nowEnc = encRead();
    if (!g_cal_armed){
      g_cal_armed = true;
      g_cal_startCount = nowEnc;
      g_cal_curCount = 0;
      g_cal_lastWasSkip = false;
    } else {
      long delta = nowEnc - g_cal_startCount; if (delta < 0) delta = -delta;
      long ref = (g_cal_n>0) ? (long)((g_cal_sum + g_cal_n/2) / g_cal_n) : g_cal_prevCount;
      bool isSkip = cal_isOutlier(delta, ref);
      g_cal_lastWasSkip = isSkip;

      if (!isSkip && delta > 0){
        g_cal_prevCount = delta;
        g_cal_sum += (uint32_t)delta;
        if (g_cal_n < 65535) g_cal_n++;
        long avg = (long)((g_cal_sum + g_cal_n/2) / g_cal_n);
        g_cpr_setting = avg;
      } else {
        g_cal_prevCount = delta;
      }
      g_cal_startCount = nowEnc;
      g_cal_curCount   = 0;
      g_cal_armed      = true;
    }
  }

  digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);
  int target = g_cal_run ? PWM_CREEP : 0;
  rampTo(target, now, rampUpMs, rampDownMs);
}

void fsmTEST(const PedalEdges& ped, const SWEvents& sw, unsigned long now){
  if (sw.longHoldEdge){
    g_cal_prevCount = (g_cpr_setting > 0) ? g_cpr_setting : g_cal_prevCount;
    g_cal_sum = 0; g_cal_n = 0;
    g_cal_run = false; g_cal_armed = false;
    g_cal_curCount = 0;
    g_cal_startCount = encRead();
    g_mode = Mode::CAL;
    motorStopHard();
    return;
  }

  g_test_pressedRotate = sw.isHeld;
  rotaryStepsServiceForTest(g_test_pressedRotate);
  g_test_goalPulses = g_cpr_setting * (long)TEST_ROUNDS;

  // thresholds from CPR
  g_approachPulses  = ( (long)g_cpr_setting * (long)APPROACH_Q8  ) >> 8;
  g_brakeTrigPulses = ( (long)g_cpr_setting * (long)BRAKE_TRIG_Q8) >> 8;
  if (g_brakeTrigPulses < 5) g_brakeTrigPulses = 5;
  if ((long)g_brakeTrigPulses + (long)BRAKE_COMP_PULSES > 1)
    g_brakeTrigPulses += BRAKE_COMP_PULSES;

  // speed update
  speedService(now);

  switch (g_state){
    case RunState::IDLE:
      if (ped.press){
        g_test_startCount = encRead();
        g_state = RunState::RAMP_UP; g_stateStartMs = now; g_startDuty = 0;
      }
      motorStopHard();
      break;

    case RunState::RAMP_UP: {
      digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);
      unsigned long dt = now - g_stateStartMs;
      if (dt >= rampUpMs){ motorAnalogWrite(PWM_TEST); g_state = RunState::RUN; }
      else {
        int duty = (int)((long)PWM_TEST * (long)dt / (long)rampUpMs);
        motorAnalogWrite(duty);
      }
      if (ped.press){ g_state = RunState::RAMP_DOWN; g_stateStartMs=now; g_startDuty=g_pwmNow; }
    } break;

    case RunState::RUN: {
      digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);
      long prog = encRead() - g_test_startCount; if (prog < 0) prog = -prog;
      long remaining = g_test_goalPulses - prog; if (remaining < 0) remaining = 0;

      // predictive lookahead
      uint32_t base = ((uint32_t)g_ppms_q15 * (uint32_t)rampDownMs) >> 1;
      long lookahead = (long)(base >> 15);
      lookahead = (long)(( (long)lookahead * (long)STOP_LOOK_Q8 ) >> 8);
      if (lookahead < MIN_LOOKAHEAD_PUL) lookahead = MIN_LOOKAHEAD_PUL;
      if (lookahead > MAX_LOOKAHEAD_PUL) lookahead = MAX_LOOKAHEAD_PUL;

      if (remaining <= g_approachPulses){
        g_state = RunState::APPROACH;
      } else if (remaining <= lookahead){
        g_state = RunState::RAMP_DOWN; g_stateStartMs = now; g_startDuty = g_pwmNow;
      } else {
        motorAnalogWrite(PWM_TEST);
      }
      if (ped.press){ g_state = RunState::RAMP_DOWN; g_stateStartMs=now; g_startDuty=g_pwmNow; }
    } break;

    case RunState::APPROACH: {
      digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);
      long prog = encRead() - g_test_startCount; if (prog < 0) prog = -prog;
      long remaining = g_test_goalPulses - prog; if (remaining < 0) remaining = 0;

      if (BRAKE_ENABLE && remaining <= g_brakeTrigPulses){
        g_state = RunState::BRAKE;
        g_brakeStartMs = now;
        digitalWrite(PIN_DIR, g_dirForward ? LOW : HIGH); // reverse
        motorAnalogWrite(BRAKE_PWM);
      } else if (remaining == 0){
        motorStopHard(); g_state = RunState::HOLD;
      } else {
        motorAnalogWrite(PWM_APPROACH);
      }
      if (ped.press){ g_state = RunState::RAMP_DOWN; g_stateStartMs=now; g_startDuty=g_pwmNow; }
    } break;

    case RunState::RAMP_DOWN: {
      digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);
      unsigned long dt = now - g_stateStartMs;
      bool finish = false; int duty=0;
      if (dt >= rampDownMs) finish = true;
      else {
        duty = g_startDuty - (int)((long)g_startDuty * (long)dt / (long)rampDownMs);
        if (duty < MIN_RAMP_PWM) finish = true;
      }
      if (finish){
        g_state = RunState::APPROACH;
        motorAnalogWrite(PWM_APPROACH);
      } else motorAnalogWrite(duty);
    } break;

    case RunState::BRAKE: {
      unsigned long dt = now - g_brakeStartMs;
      if (dt >= BRAKE_MS){
        motorStopHard();
        digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW); // restore fwd
        long prog = encRead() - g_test_startCount; if (prog < 0) prog = -prog;
        long remaining = g_test_goalPulses - prog;
        if (TRIM_ENABLE && remaining > TRIM_STOP_BAND_PUL){
          g_state = RunState::TRIM;
          g_trimStartMs = now;
          motorAnalogWrite(TRIM_PWM);
        } else {
          g_state = RunState::HOLD;
        }
      }
    } break;

    case RunState::TRIM: {
      digitalWrite(PIN_DIR, g_dirForward ? HIGH : LOW);
      long prog = encRead() - g_test_startCount; if (prog < 0) prog = -prog;
      long remaining = g_test_goalPulses - prog;

      if (remaining <= 0 || (millis() - g_trimStartMs) >= TRIM_MAX_MS){
        motorStopHard();
        g_state = RunState::HOLD;
      } else {
        motorAnalogWrite(TRIM_PWM);
      }
      if (ped.press){ motorStopHard(); g_state = RunState::HOLD; }
    } break;

    case RunState::HOLD:
    default:
      motorStopHard();
      if (ped.press){
        g_test_startCount = encRead();
        g_state = RunState::RAMP_UP; g_stateStartMs = now; g_startDuty=0;
      }
      break;
  }
}

// ===============================
// [9] SETUP & LOOP
// ===============================
void setup(){
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)){
    pinMode(LED_BUILTIN, OUTPUT);
    while (1){ digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN)); delay(100); }
  }
  display.setRotation(OLED_ROTATION);
  display.clearDisplay(); display.display();

  pinMode(PIN_PEDAL, INPUT_PULLUP);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  digitalWrite(PIN_DIR, HIGH); g_dirForward = true;

  // Timer1 Fast PWM 8-bit, non-inverting, prescaler=1 (~31kHz)
  TCCR1A = (1<<WGM10) | (1<<COM1A1);
  TCCR1B = (1<<WGM12) | (1<<CS10);

  // Motor encoder
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  { uint8_t a = digitalRead(ENC_A), b = digitalRead(ENC_B); g_enc_prevState = (a<<1)|b; }
  attachInterrupt(digitalPinToInterrupt(ENC_A), ISR_encA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), ISR_encB, CHANGE);

  // Rotary
  pinMode(ROT_CLK, INPUT_PULLUP);
  pinMode(ROT_DT,  INPUT_PULLUP);
  pinMode(ROT_SW,  INPUT_PULLUP);
  { uint8_t ra = (PIND >> 7) & 0x01; uint8_t rb = (PIND >> 6) & 0x01; g_rot_prevAB = (ra<<1)|rb; }
  pciSetup(ROT_CLK); pciSetup(ROT_DT);
  g_sw_state = digitalRead(ROT_SW);

  // CAL baseline
  g_cal_run = false; g_cal_armed = false; g_cal_curCount = 0; g_cal_startCount = encRead();

  // Splash
  drawCentered("Calibrator", 12, 2);
  display.display();
  delay(500);
  display.clearDisplay(); display.display();

  // Speed init
  g_spd_lastMs  = millis();
  g_spd_lastCnt = encRead();
}

void loop(){
  unsigned long now = millis();
  PedalEdges ped = pedalService();
  SWEvents sw = rotSwService();

  if (g_mode == Mode::CAL) fsmCAL(ped, sw, now);
  else                     fsmTEST(ped, sw, now);

  if (now - g_ui_lastMs >= UI_REFRESH_MS){
    if (g_mode == Mode::CAL) uiCAL();
    else                     uiTEST();
    g_ui_lastMs = now;
  }
}
