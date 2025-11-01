# 🧶 SkeinMachine v3.19 — Smart Yarn Twister Controller

An Arduino-based controller for an electric yarn twister (skein winder), inspired by the [Alpenglow SkeinTwister](https://www.alpenglowindustries.com/pages/skeintwister).   
It automates the twisting of yarn skeins to a precise number of turns tries to always stops at the same hook position.  
The system combines encoder feedback, closed-loop motor control, and a compact user interface for smooth, repeatable results.

---

## ✨ Features

### 🎯 Phase-anchored AUTO mode
- Each automatic run stops at the exact same mechanical phase, even if skein tension or weight changes.  
- Uses a *median-filtered nudge* algorithm with a small deadband to correct phase offset over time.  
- Bias and calibration values are persisted safely in EEPROM with wear-levelling.

### ⚙️ Closed-loop creep RPM control
- During the final approach, a float-less PI controller adjusts PWM duty to maintain a stable low speed.  
- Prevents stalls and overshoot on heavy skeins while keeping motion smooth and controlled.

### 🧩 Adaptive stop lead (“nudger”)
- Dynamically adjusts braking distance based on prior runs.  
- Compensates for inertia differences between skeins for consistent stopping accuracy.

### 👣 Dual-mode operation
- **Manual mode:** Motor runs only while the foot pedal is pressed.  
- **AUTO mode:** Twists to a preset number of turns and stops automatically at the anchor point.

### 🔘 Rotary encoder interface (KY-040)
- Adjusts target turn count and toggles between modes.  
- Supports push-button clicks and double-click actions for quick UI operations.

### 💾 EEPROM wear-levelling
- Dual-slot persistence ensures long EEPROM lifetime.  
- Saves lifetime skein counters, adaptive stop parameters, and phase anchor data.

### 🖥️ OLED user interface
- Compact SSD1306 128×64 display shows:
  - Real-time RPM
  - Turn progress bar
  - Mode (MANUAL / AUTO)
  - Optional debug overlay

### 🧠 Safety & reliability
- Motor cannot start without an explicit pedal press.  
- Stall detection halts the motor if the encoder stops moving.  
- Smooth PWM ramps up/down prevent mechanical shock and yarn tension spikes.

---

## 🛠️ Hardware Setup

| Component | Description |
|------------|--------------|
| **MCU** | Arduino Nano / Uno (ATmega328P) |
| **Motor** | 5840-36ZY 12V worm gear motor with encoder from AliExpress (note that the -36ZY is supposedly stronger than the -32ZY variant)
| **Motor driver** | Cytron MD13S |
| **Motor encoder** | Quadrature encoder (e.g. 16 PPR × 49 gear ≈ 9300 counts per rev) |
| **Inputs** | Foot pedal (active-LOW), KY-040 rotary encoder + push button |
| **Display** | SSD1306 128×64 I²C OLED |
| **Power** | 12 V DC (recommended), depending on motor spec |

---

## 🧾 Software Architecture

| Section | Purpose |
|----------|----------|
| `[1] Feature toggles & version` | Compile-time switches |
| `[2] CONFIG` | All tunable calibration parameters |
| `[3] Includes & global state` | Core variables, grouped by domain |
| `[4] ISRs` | Deterministic encoder interrupts |
| `[5] Low-level drivers & utils` | Motor PWM, encoder atomic access |
| `[6] Services` | RPM, pedal, rotary, EEPROM, UI helpers |
| `[7] UI rendering` | OLED display logic |
| `[8] Finite State Machine` | IDLE → RAMP_UP → RUN → RAMP_DOWN |
| `[9] Setup & loop` | Main runtime flow |

---

## ⚙️ Core Control Flow

1. **Pedal pressed** → motor ramps up (RAMP_UP)  
2. **Target turns reached** → transition to RAMP_DOWN  
3. **Final creep phase** uses RPM control for precision  
4. **Motor stops** exactly at anchor phase  
5. **Anchor offset** updated and persisted

---

## 📦 Version Highlights

**v3.19**
- Added phase-anchored AUTO stops  
- Introduced creep-phase RPM control  
- Added per-run adaptive lead correction  
- Optimized EEPROM wear-levelling and persist logic  
- Clean float-less math and compact UI  

---

## 🧶 Summary

> *SkeinMachine v3.19 turns a basic DC motor into a precise, repeatable, and intelligent yarn-twisting tool.*

- Phase-locked stopping position  
- Adaptive creep-speed control  
- Safe manual operation  
- Clean, responsive OLED interface  

Perfect for yarn dyers, spinners, and hobbyists who want consistent, professional skeins.

---

### 📜 License

GPL License © 2025 Rick Klaassen

