# Haptic Feedback System Documentation
## drone_e58.ino + Live Monitor (Current Implementation)

Last updated: 2026-05-04

---

## 1. Scope

This document describes the current, implemented behavior of:
- drone_e58/drone_e58.ino (Arduino Nano RP2040 Connect firmware)
- drone_e58/haptic_live_monitor.py (desktop serial visualizer)

It is written as an as-built reference, not a theoretical design.

---

## 2. System Summary

The system maps flight control state and action events to glove stimulation using:
- HV2701 switch matrix for electrode routing (position Mx)
- MAX5413 digital potentiometer for intensity (pot value 0..255)
- Pulse output pin for temporal pattern (single, burst, or train)

Two feedback families exist:
- Continuous control feedback (TRAIN): time-division multiplexed over active axes
- Discrete action feedback (BURST): one-shot bursts for takeoff/land/stop/zero/flip-arm

The firmware also emits debug lines over serial in a stable format that the Python monitor parses and renders.

---

## 3. Hardware Mapping

### 3.1 Haptic Pins (drone_e58.ino)

- HAPTIC_POT_CS = 6
- HAPTIC_DATA_PIN = 5
- HAPTIC_CLK_PIN = 4
- HAPTIC_HV_LE = 3
- HAPTIC_HV_CLR = 2
- HAPTIC_OUT_PIN = 12

Note:
- Pulse output is on pin 12 in current firmware.

### 3.2 Position Regions

- M4  -> YAW region (thumb)
- M8  -> PITCH region (index)
- M12 -> ROLL region (middle/ring)
- M20 -> THROTTLE region (palm)

Enum mapping:
- HAPTIC_POS_YAW = 4
- HAPTIC_POS_PITCH = 8
- HAPTIC_POS_ROLL = 12
- HAPTIC_POS_THROTTLE = 20

---

## 4. Pulse Modes and Timing

HapticPulseMode state machine:
- HPM_IDLE
- HPM_SINGLE
- HPM_BURST
- HPM_TRAIN

Default timing constants:
- HAPTIC_DEFAULT_FREQ_HZ = 100.0
- HAPTIC_DEFAULT_PW_US = 400
- HAPTIC_DEFAULT_TRAIN_MS = 2000
- HAPTIC_BURST_PULSE_MS = 50
- HAPTIC_BURST_PAUSE_MS = 100
- HAPTIC_ACTION_BURST_COUNT = 3

Behavior details:
- TRAIN is maintained while controls are off-neutral.
- Burst actions reserve haptic output with hapticActionLockUntilMs to avoid immediate overwrite by continuous TRAIN.
- hapticUpdatePulses() is non-blocking and must run every loop.

---

## 5. Continuous Control Feedback (TRAIN)

Function:
- updateHapticFeedback(stickYaw, stickPitch, stickRoll, stickThrottle)

Execution behavior:
- Skips feedback while flipInProgress is true.
- Returns early while action lock is active.
- Rate-limited by HAPTIC_FEEDBACK_UPDATE_MS (10 ms).

Per-axis activation:
- Axis active when normalized stick deviation > 0.
- If all axes neutral:
  - stop pulses
  - clear active states
  - reset mux cursor
  - emit idle debug line once when transitioning from active

Pot mapping:
- YAW: 16..25
- PITCH: 23..28
- ROLL: 28..35
- THROTTLE: 14..20

Interpolation direction:
- Pot decreases as magnitude increases in the current implementation.
- This is intentional as currently coded, and matches monitor display.

Multiplexing model:
- One active channel is selected per update tick.
- Round-robin fairness using hapticMuxCursor.
- For chosen channel:
  - route HV position
  - set pot
  - start/refresh TRAIN

Debug output format:
- [HDBG] TRAIN axis=<YAW|PITCH|ROLL|THROTTLE> pos=<M#> pot=<int> active=[YPRT]
- [HDBG] TRAIN idle (all controls neutral)

---

## 6. Discrete Action Feedback (BURST)

Function:
- triggerHapticAction(position, potValue)

Actions and mappings:
- TAKEOFF: M4, pot 20
- LAND: M8, pot 25
- STOP: M20, pot 18
- ZERO (serial O/ZERO): M12, pot 30
- FLIP ARMED (NN class 7): M20, pot 18

NN zero behavior fix (implemented):
- NN class 4 (zero) now triggers the same burst as manual zero:
  - triggerHapticAction(HAPTIC_POS_ROLL, 30)
- This ensures burst visibility and tactile parity across both zero paths.

Debug output format for bursts:
- [HDBG] BURST pos=<M#> pot=<int> count=<int> on=<ms>ms off=<ms>ms lock=<ms>ms

This line is emitted inside triggerHapticAction when hapticDebugEnabled is true.

---

## 7. TinyML Action Integration

NN classes used by firmware:
- 1 -> STOP
- 2 -> LAND
- 3 -> TAKEOFF
- 4 -> ZERO
- 7 -> FLIP ARMED

NN debounce logic:
- Requires class hold time (NN_HOLD_MS)
- Requires confidence margin (NN_MIN_MARGIN_Q, with special case for class 2)
- Enforces action cooldown (NN_ACTION_COOLDOWN_MS)

Flip workflow:
- Class 7 arms flip mode and gives palm burst confirmation.
- Directional stick deflection then starts actual flip sequence.

---

## 8. Live Monitor (haptic_live_monitor.py)

### 8.1 Serial behavior

- Default serial port is COM3.
- Default baud is 115200.
- On open, script sends HDBGON to enable debug lines.

CLI defaults:
- --port COM3
- --baud 115200
- --stale-ms 120
- --refresh-ms 50

### 8.2 Parsed line types

- TRAIN lines
- TRAIN idle line
- BURST lines

Regex support includes:
- TRAIN axis/pos/pot/active bitmap
- BURST pos/pot/count/on/off/lock

### 8.3 Visualization behavior

Bars:
- Four axes: YAW, PITCH, ROLL, THROTTLE
- Green when active TRAIN update is fresh
- Gray when inactive/stale
- Orange when recent BURST is mapped to that axis

Burst mapping in monitor:
- M4 -> YAW
- M8 -> PITCH
- M12 -> ROLL
- M20 -> THROTTLE

Status areas:
- Top-left: last update age + truncated last line
- Top-right: last burst summary and age
- Layout reserves top margin to avoid title overlap

---

## 9. Serial Command Reference (Current)

Flight/control commands:
- T / TAKEOFF
- L / LAND
- X / STOP
- C / CAL / CALIBRATE
- H / HEADLESS
- O / ZERO
- R / RECAL / RECALIBRATE
- CONNECT
- START
- D / DISCONNECT
- P / PYUDP
- A / ARDUDP
- FLIP:<FORWARD|BACKWARD|LEFT|RIGHT>
- FF / FB / FL / FR
- N / NN (toggle NN)

Haptic commands:
- HAPTIC_STOP
- HDBG / HDBGON / HDBGOFF
- P<0..255> (pot set)
- HS (single default)
- HSD<ms> (single custom duration)
- HB (burst default count)
- HBC<count> (burst custom count)
- HT (train with current settings)
- HF<Hz>
- HW<us>
- HD<ms>
- HSW<0..15> (toggle HV switch bit)

---

## 10. Firmware State and Priorities

Priority/interaction rules:
- Discrete action burst locks haptic path temporarily.
- Continuous TRAIN updates are paused during lock.
- Flip-in-progress suppresses continuous haptic update routine.
- Neutral controls stop TRAIN immediately and reset mux state.

Operational notes:
- hapticDebugEnabled controls whether HDBG lines are emitted.
- Monitor visibility depends on debug being on.

---

## 11. Validation Checklist

After flashing firmware and starting monitor:

1) Serial and monitor startup
- Confirm monitor connects on COM3.
- Confirm monitor receives HDBG lines.

2) Continuous TRAIN
- Move glove to activate yaw/pitch/roll/throttle.
- Confirm TRAIN lines arrive and bars animate.

3) Discrete BURST actions
- Trigger TAKEOFF/LAND/STOP/ZERO.
- Confirm [HDBG] BURST lines and orange burst highlighting.

4) NN zero parity check
- Trigger NN class 4 gesture.
- Confirm M12 burst with pot 30 appears in logs and monitor.

Expected burst text example:
- [HDBG] BURST pos=M12 pot=30 count=3 on=50ms off=100ms lock=...

---

## 12. Known Constraints

- Continuous haptic feedback is multiplexed across active channels (single channel per update tick), not simultaneous multi-channel output.
- Monitor depends on emitted serial debug lines; if HDBG is off, visualization will show stale/idle behavior.
- Live monitor currently assumes Windows-style COM naming defaults, with COM3 set as the default runtime port.

---

## 13. Change Log (Recent)

2026-05 updates:
- Added BURST debug parsing and visualization in live monitor.
- Set monitor default port to COM3 (no required launch arg).
- Fixed monitor status/title overlap by moving status to figure-level top text with reserved margin.
- Fixed NN class 4 zero path to emit haptic burst on M12 at pot 30, matching manual ZERO behavior.

---

## 14. Quick Reference

Region map:
- M4 = YAW
- M8 = PITCH
- M12 = ROLL
- M20 = THROTTLE

Action bursts:
- TAKEOFF -> M4 p20
- LAND -> M8 p25
- STOP -> M20 p18
- ZERO -> M12 p30
- FLIP ARM -> M20 p18

Debug lines consumed by monitor:
- [HDBG] TRAIN ...
- [HDBG] TRAIN idle ...
- [HDBG] BURST ...
# Haptic Feedback System Documentation
## Complete Architecture & Operating Manual

---

## 1. System Overview

The haptic feedback system provides real-time tactile feedback on a smart glove to inform the operator about drone control commands and ongoing flight status. It integrates seamlessly with the E58/K417 WiFi drone controllers running on Arduino Nano RP2040 Connect, using the glove's inherent properties as a communication channel.

### Key Principle
**All actions trigger stimulation on specific hand regions using configurable intensity (potentiometer) and pulse patterns (frequency/burst).**

---

## 2. Hardware Architecture

### 2.1 Control Hardware

```
┌─────────────────────────────────────────────────────┐
│       Arduino Nano RP2040 Connect                   │
│  ┌───────────────────────────────────────────────┐  │
│  │ IMU (LSM6DSOX) + WiFi (WiFiNINA) onboard     │  │
│  │                                               │  │
│  │ Control Pins:                                 │  │
│  │ • Pin 4  (CLK)      → HV2701/MAX5413 clock   │  │
│  │ • Pin 5  (DATA)     → SPI data line          │  │
│  │ • Pin 6  (POT_CS)   → MAX5413 chip select    │  │
│  │ • Pin 3  (HV_LE)    → HV2701 latch enable    │  │
│  │ • Pin 2  (HV_CLR)   → HV2701 clear signal    │  │
│  │ • Pin 13 (OUT)      → Pulse output to glove  │  │
│  └───────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────┘
```

### 2.2 Stimulation Hardware

#### MAX5413 Digital Potentiometer
- **Function**: Controls current intensity delivered to stimulators
- **Interface**: SPI (Serial Peripheral Interface)
- **Range**: 0-255 (mapped to potentiometer wiper position)
- **Intensity Range**: 0-10 kΩ resistance → variable current output

#### HV2701 High-Voltage Switch Matrix
- **Function**: Selects which electrodes stimulate (row/column matrix)
- **Output Channels**: 16 addressable channels (4-bit control word)
- **Latching**: Maintains state until updated
- **Control**: 16-bit SPI data frame

#### Stimulation Regions (Predefined Positions)
```
Position M4  (Yaw):      Thumb base region     (Ch 3)
Position M8  (Pitch):    Index finger region   (Ch 8)
Position M12 (Roll):     Middle/Ring region    (Ch 12/10)
Position M20 (Throttle): Palm region           (Ch 18/15)
```

---

## 3. Control Mapping (From PDF Specification)

### 3.1 Continuous Controls (Train Mode - 100 Hz frequency)

#### YAW Control (+/-)
```
Action:    Yaw +/-
Region:    M4 (Thumb, Ch 3)
Signal:    T1 (Train: 100 Hz, 400 µs pulses, 2 sec)
Pot Range: 
  • Negative (roll left):  20-16 (intensity increases as magnitude increases)
  • Positive (roll right): 25-21
Mapping:   Normalized angle (0-45°) → interpolated pot value
```

#### PITCH Control (+/-)
```
Action:    Pitch +/-
Region:    M8 (Index, Ch 8)
Signal:    T1 (Train: 100 Hz, 400 µs pulses, 2 sec)
Pot Range:
  • Negative (pitch back):  25-23
  • Positive (pitch fwd):   28-26
Mapping:   Normalized angle (0-45°) → interpolated pot value
```

#### ROLL Control (+/-)
```
Action:    Roll +/-
Region:    M12 (Middle/Ring, Ch 12/10)
Signal:    T1 (Train: 100 Hz, 400 µs pulses, 2 sec)
Pot Range:
  • Negative (roll left):  31-28
  • Positive (roll right): 35-32
Mapping:   Normalized angle (0-45°) → interpolated pot value
```

#### THROTTLE Control (+/-)
```
Action:    Throttle +/-
Region:    M20 (Palm, Ch 18/15)
Signal:    T1 (Train: 100 Hz, 400 µs pulses, 2 sec)
Pot Range:
  • Negative (throttle down): 16-14
  • Positive (throttle up):   20-17
Mapping:   Normalized stick deviation (0-1.0) → interpolated pot value
```

### 3.2 Discrete Actions (Burst Mode - 3 pulses × 50 ms on, 50 ms off)

#### Takeoff
```
Action:  Takeoff button
Region:  M4 (Thumb)
Signal:  B (Burst: 3 pulses, 50 ms on/off)
Pot:     20 (fixed intensity)
Trigger: When 'T' command sent or flagTakeoff is consumed
```

#### Landing
```
Action:  Landing button
Region:  M8 (Index)
Signal:  B (Burst: 3 pulses, 50 ms on/off)
Pot:     25 (fixed intensity)
Trigger: When 'L' command sent or flagLand is consumed
```

#### Zero (Orient Capture)
```
Action:  Zero button
Region:  M12 (Middle/Ring)
Signal:  B (Burst: 3 pulses, 50 ms on/off)
Pot:     30 (fixed intensity)
Trigger: When 'O' command sent or zero capture initiated
```

#### Stop (Emergency)
```
Action:  Stop button
Region:  M20 (Palm)
Signal:  B (Burst: 3 pulses, 50 ms on/off)
Pot:     18 (fixed intensity)
Trigger: When 'X' command sent or emergency stop initiated
```

#### Flip Mode (Multi-region)
```
Action:  Flip mode armed
Region:  M20 (Palm)
Signal:  B3 (Burst: 3 pulses, 50 ms on/off)
Pot:     18 (fixed intensity)
Trigger: When flip sequence initiates (first burst packet only)
```

---

## 4. Signal Processing Pipeline

### 4.1 Continuous Control Feedback Loop

```
┌─────────────────────────────────────────────────────────────┐
│ SENSOR INPUT STAGE                                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  IMU (LSM6DSOX)  →  Accelerometer (ax, ay, az)            │
│                  →  Gyroscope (gx, gy, gz)                 │
│  Flex Sensors    →  A0, A1, A2, A3 analog values          │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────┴────────────────────────────────────┐
│ ESTIMATION STAGE                                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Mahony AHRS Filter    →  (yaw, pitch, roll) angles [deg] │
│  Flex Deflection       →  (throttle) normalized [-1, 1]   │
│  Angle-to-Stick Map    →  Stick values [40, 220] uint8    │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────┴────────────────────────────────────┐
│ HAPTIC FEEDBACK STAGE (updateHapticFeedback)               │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. Check if action active (angle ≠ 0.0)                  │
│  2. Compute normalized magnitude: |angle| / MAX_ANGLE_DEG │
│  3. Interpolate pot value:                                 │
│     potValue = potMin + norm × (potMax - potMin)          │
│  4. Check direction change (sign flip)                     │
│  5. Trigger haptic feedback if active or direction changed│
│  6. Update feedback state                                  │
│                                                             │
│  ✓ YAW:      M4 region, ranges [20-25]                   │
│  ✓ PITCH:    M8 region, ranges [25-28]                   │
│  ✓ ROLL:     M12 region, ranges [31-35]                  │
│  ✓ THROTTLE: M20 region, ranges [16-20]                  │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────┴────────────────────────────────────┐
│ STIMULATION OUTPUT STAGE                                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  hapticSendToHV2701(M_position)  →  Select electrode set  │
│  hapticSetPot(potValue)          →  Set current intensity │
│  hapticStartTrain(...)           →  Start 100 Hz train    │
│                                                             │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────┴────────────────────────────────────┐
│ PULSE GENERATION (hapticUpdatePulses)                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Non-blocking state machine:                               │
│  • HPM_IDLE    - No output                                 │
│  • HPM_SINGLE  - One pulse                                 │
│  • HPM_BURST   - Repeated on/off cycles                   │
│  • HPM_TRAIN   - Continuous frequency train               │
│                                                             │
│  Output: Digital pulse on Pin 13 → Stimulator driver      │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 Discrete Action Feedback Path

```
Serial Command (e.g., "T")
    ↓
handleSerialCommandLine() parses command
    ↓
Set action flag (e.g., flagTakeoff = true)
    ↓
Main loop detects flag in control section
    ↓
triggerHapticAction(region, pot_intensity)
    ├─→ hapticSendToHV2701(positions[region])
    ├─→ hapticSetPot(pot_intensity)
    └─→ hapticStartBurst(3, 50, 50)  // 3 pulses
    ↓
hapticUpdatePulses() runs each loop iteration
    ├─→ Pulse output HIGH/LOW based on timing
    └─→ Complete after 3 on/off cycles
    ↓
Operator feels tactile feedback on glove
```

---

## 5. Software Implementation Details

### 5.1 Haptic Feedback Data Structure

```cpp
struct HapticFeedback {
  HapticPosition position;      // M4, M8, M12, M20
  float directionSign;          // +1.0 or -1.0
  int potMin;                   // Minimum pot value
  int potMax;                   // Maximum pot value
  bool isActive;                // Currently stimulating
  unsigned long lastTriggerMs;  // Timestamp of last trigger
};

// Global instances
HapticFeedback hapticYaw     = {HAPTIC_POS_YAW,      0.0f, 20, 25, false, 0};
HapticFeedback hapticPitch   = {HAPTIC_POS_PITCH,    0.0f, 25, 28, false, 0};
HapticFeedback hapticRoll    = {HAPTIC_POS_ROLL,     0.0f, 31, 35, false, 0};
HapticFeedback hapticThrottle= {HAPTIC_POS_THROTTLE, 0.0f, 16, 20, false, 0};
```

### 5.2 Core Functions

#### updateHapticFeedback() - Main Feedback Handler
```cpp
void updateHapticFeedback(float yaw, float pitch, float roll, uint8_t throttle)
```
- **Called**: Once per control loop cycle (~50 Hz for E58, ~40 Hz for K417)
- **Rate-Limited**: Only updates every 50 ms (HAPTIC_FEEDBACK_UPDATE_MS)
- **Logic**: 
  1. For each control (yaw, pitch, roll, throttle)
  2. Check if active (value ≠ 0 or ≠ STICK_MID)
  3. Normalize magnitude to 0-1 range
  4. Interpolate pot value across min/max range
  5. Detect direction changes
  6. Trigger feedback if needed

#### triggerHapticFeedback() - Continuous Stimulation
```cpp
void triggerHapticFeedback(HapticFeedback *feedback, int potValue)
```
- **Effect**: Start train mode (continuous frequency-based pulses)
- **Used For**: Real-time control feedback (yaw, pitch, roll, throttle)
- **Sequence**:
  1. Set HV2701 electrode region
  2. Set MAX5413 potentiometer
  3. Start 100 Hz train (400 µs pulses, 2 sec duration)

#### triggerHapticAction() - Discrete Action Stimulation
```cpp
void triggerHapticAction(HapticPosition position, int potValue)
```
- **Effect**: Start burst mode (3 discrete pulses)
- **Used For**: One-shot actions (Takeoff, Land, Zero, Stop)
- **Sequence**:
  1. Set HV2701 electrode region
  2. Set MAX5413 potentiometer
  3. Start burst (3 × 50 ms on/off cycles)

#### hapticUpdatePulses() - Non-Blocking Pulse Generator
```cpp
void hapticUpdatePulses()
```
- **Called**: Every loop iteration (non-blocking)
- **Maintains**: State machine for:
  - Single pulses
  - Burst sequences
  - Frequency trains
- **Output**: HIGH/LOW on Pin 13 based on timing

### 5.3 Position Constants (Hand Regions)

```cpp
enum HapticPosition {
  HAPTIC_POS_YAW       = 4,    // M4:  Thumb region
  HAPTIC_POS_PITCH     = 8,    // M8:  Index region
  HAPTIC_POS_ROLL      = 12,   // M12: Middle/Ring region
  HAPTIC_POS_THROTTLE  = 20    // M20: Palm region
};

// Map to 16-bit HV2701 control words
// drone_e58.ino: Uses positions[] array from est_fuante_pruebas.ino
// drone_k417.ino: Direct HV state mapping in trigger functions
```

---

## 6. Command Interface & Serial Protocol

### 6.1 Haptic-Specific Serial Commands

```
HAPTIC_STOP              - Stop all ongoing stimulation
Pxxx                     - Set potentiometer (0-255)
HS                       - Single pulse (default 1000 ms)
HSDxxx                   - Single pulse with custom duration (ms)
HB                       - Burst (default 5 pulses)
HBCx                     - Burst with custom count
HT                       - Train pulse (current freq/width/duration)
HFxx                     - Set train frequency (Hz)
HWxx                     - Set pulse width (µs)
HDxx                     - Set train duration (ms)
HSWx                     - Toggle HV2701 switch 0-15
```

### 6.2 Action Commands with Haptic Feedback

```
T / TAKEOFF              - Takeoff + haptic on M4 (pot 20)
L / LAND                 - Landing + haptic on M8 (pot 25)
X / STOP                 - Stop + haptic on M20 (pot 18)
O / ZERO                 - Zero orientation + haptic on M12 (pot 30)
```

### 6.3 Example Serial Session

```
> T
[HAPTIC] Takeoff feedback triggered
[CMD] TAKEOFF sent

> HF50
[HAPTIC] Frequency = 50.0 Hz

> HW200
[HAPTIC] Pulse width = 200 us

> P100
[HAPTIC] Pot = 100/255 (~3921.6 Ohms)
```

---

## 7. Operating Principles & Timing

### 7.1 Update Frequencies

| Component | Frequency | Period |
|-----------|-----------|--------|
| IMU Sensor Read | ~400 Hz | 2.5 ms |
| Control Loop | 50 Hz (E58), 40 Hz (K417) | 20/25 ms |
| Haptic Feedback Update | ~20 Hz | 50 ms |
| Haptic Pulse Generation | 100 Hz (train) | Variable |
| Telemetry | 30 Hz (E58), 25 Hz (K417) | 33/40 ms |

### 7.2 State Persistence

**Continuous Feedback (Yaw, Pitch, Roll, Throttle)**
```
• Starts when angle/throttle becomes non-zero
• Intensity increases with magnitude
• Direction changes trigger new stimulation
• Stops when value returns to zero
```

**Discrete Feedback (Takeoff, Land, Zero, Stop)**
```
• Triggered immediately on command
• Always delivers exactly 3 pulses
• Non-blocking (runs in background)
• Cannot be interrupted mid-sequence
```

### 7.3 Priority & Conflicts

- **Continuous feedback**: Lower priority, can be interrupted
- **Discrete feedback**: Higher priority, completes full sequence
- **Multiple simultaneous actions**: Serial execution (one after another)

---

## 8. Calibration & Setup

### 8.1 Potentiometer Intensity Ranges

| Action | Pot Min | Pot Max | Default |
|--------|---------|---------|---------|
| Yaw | 20 | 25 | 22.5 |
| Pitch | 25 | 28 | 26.5 |
| Roll | 31 | 35 | 33 |
| Throttle | 16 | 20 | 18 |
| Takeoff | - | - | 20 |
| Landing | - | - | 25 |
| Zero | - | - | 30 |
| Stop | - | - | 18 |

### 8.2 Pulse Parameters

```cpp
// Default Train Parameters (Continuous Feedback)
HAPTIC_DEFAULT_FREQ_HZ = 100.0          // 100 Hz frequency
HAPTIC_DEFAULT_PW_US = 400              // 400 µs pulse width
HAPTIC_DEFAULT_TRAIN_MS = 2000          // 2 second duration

// Burst Parameters (Discrete Feedback)
HAPTIC_BURST_COUNT = 3                  // 3 pulses per burst
HAPTIC_BURST_PULSE_MS = 50              // 50 ms on time
HAPTIC_BURST_PAUSE_MS = 50              // 50 ms off time
```

---

## 9. Troubleshooting & Diagnostics

### 9.1 No Stimulation Feedback

**Check List:**
1. Verify pins connected correctly (4, 5, 6, 3, 2, 13)
2. Confirm MAX5413 and HV2701 powered
3. Test with manual potentiometer: `P128`
4. Verify SPI communication with `HSW0` (toggle switch 0)

### 9.2 Weak Stimulation

**Solutions:**
1. Increase potentiometer value: `P255`
2. Verify electrode contact with skin
3. Check battery voltage to HV2701 (should be ~3.3V)
4. Increase pulse duration: `HW800`

### 9.3 Intermittent Feedback

**Causes:**
1. SPI bus interference - reduce control loop frequency
2. Loose connections on SPI pins
3. Potentiometer value out of range (> 255)
4. Multiple simultaneous pulse modes conflicting

### 9.4 Feedback Not Following Control

**Debug Steps:**
1. Verify control values being sent to drone
2. Check `yawDeg`, `pitchDeg`, `rollDeg` in telemetry
3. Confirm haptic feedback threshold is active
4. Test manual trigger: `HT` then tilt glove

---

## 10. System Architecture Diagram

```
┌──────────────────────────────────────────────────────────────┐
│                   ARDUINO CONTROL FLOW                       │
└──────────────────────────────────────────────────────────────┘

    SETUP PHASE
    ├─ Initialize pins (4, 5, 6, 3, 2, 13)
    ├─ Set initial pot (hapticSetPot(255))
    ├─ Set initial HV state (hapticSendToHV2701(0x0000))
    └─ Print initialization message

    MAIN LOOP (Repeating ~50 Hz)
    │
    ├─ [SENSOR STAGE]
    │  ├─ Read IMU (ax, ay, az, gx, gy, gz)
    │  ├─ Read Flex sensors (A0-A3)
    │  └─ Store in local variables
    │
    ├─ [ESTIMATION STAGE]
    │  ├─ Mahony AHRS filter
    │  │  └─ Output: (yaw, pitch, roll) degrees
    │  └─ Flex-to-throttle mapper
    │     └─ Output: stick throttle value
    │
    ├─ [CONTROL INTERVAL CHECK] (every 20-25 ms)
    │  │
    │  ├─ Process action flags
    │  │  ├─ flagTakeoff → triggerHapticAction(M4, 20)
    │  │  ├─ flagLand → triggerHapticAction(M8, 25)
    │  │  ├─ flagStop → triggerHapticAction(M20, 18)
    │  │  └─ ... other actions ...
    │  │
    │  ├─ Build stick values from IMU angles
    │  │
    │  ├─ Send UDP control packet to drone
    │  │
    │  ├─ [HAPTIC FEEDBACK STAGE] ⚠️ KEY STAGE
    │  │  └─ updateHapticFeedback(yaw, pitch, roll, throttle)
    │  │     ├─ Check if action active (non-zero)
    │  │     ├─ Normalize magnitude
    │  │     ├─ Map to pot range
    │  │     └─ Trigger if changed or first time
    │  │
    │  └─ Update telemetry if needed
    │
    ├─ [PULSE GENERATION STAGE] (every loop iteration)
    │  └─ hapticUpdatePulses()
    │     ├─ Check pulse mode (IDLE, SINGLE, BURST, TRAIN)
    │     ├─ Update timing counters
    │     └─ Set Pin 13 HIGH/LOW based on state
    │
    └─ [SERIAL HANDLER] (non-blocking)
       └─ Check for incoming commands
          ├─ Parse and execute
          └─ May trigger haptic feedback

┌──────────────────────────────────────────────────────────────┐
│                      HARDWARE LAYER                          │
└──────────────────────────────────────────────────────────────┘

SPI Bus (Clock, Data, CS)
    ↓
MAX5413 (Potentiometer)  ← Sets current intensity [0-255]
    ↓
Current Output
    ↓
Stimulator/Amplifier
    ↓
Electrodes on Glove

Parallel SPI (Latch, Clear)
    ↓
HV2701 (Switch Matrix)   ← Selects electrode region [M4/M8/M12/M20]
    ↓
Row/Column Selection
    ↓
Active Electrode Pair

Pin 13 (Pulse Output)
    ↓
Pulse Train Generator
    ↓
Driver Circuit
    ↓
Stimulation Output
```

---

## 11. Performance Characteristics

### 11.1 Latency Budget

```
IMU Read:                 ~2.5 ms
Mahony Filter:            ~1 ms
Control Computation:      ~5 ms
Haptic Feedback Logic:    ~2 ms (only every 50 ms)
UDP Transmission:         ~1 ms
Pulse Generation:         <0.5 ms
─────────────────────────────
Total Per Cycle:          ~12 ms (well under 20 ms budget)
```

### 11.2 Power Consumption (Estimated)

```
Arduino + IMU:            ~50 mA @ 3.3V
WiFi Transmission:        ~100 mA peak
HV2701 Standby:          ~5 mA
MAX5413 + Driver:        Variable (0-200 mA based on intensity)
─────────────────────────────
Total Typical:           150-250 mA
```

---

## 12. Verification & Testing

### 12.1 Functional Test Sequence

```
1. HARDWARE TEST
   > P0     # Should feel nothing
   > P255   # Should feel maximum intensity
   > HSW0   # Toggle electrode 0 (should hear relay)

2. SPI COMMUNICATION TEST
   > HF50   # Set frequency
   > HW200  # Set pulse width
   > HT     # Start train (should feel rhythmic pulse)

3. POSITION TEST
   > HS     # Single pulse on default position
   > HB     # Burst on default position
   > HSDxxx # Custom duration pulse

4. INTEGRATION TEST
   > T      # Takeoff - should feel M4 burst (3 pulses)
   > L      # Landing - should feel M8 burst
   > O      # Zero - should feel M12 burst
   > X      # Stop - should feel M20 burst

5. CONTINUOUS FEEDBACK TEST
   (Physically move wrist to generate IMU angles)
   > Yaw motion     → Feel M4 stimulation
   > Pitch motion   → Feel M8 stimulation
   > Roll motion    → Feel M12 stimulation
   > Flex fingers   → Feel M20 stimulation
```

### 12.2 Expected Behavior

✓ **During Continuous Control**
- Stimulation intensity increases as control magnitude increases
- Region changes when action direction reverses
- Feedback stops when control returns to zero
- No interference with drone control signals

✓ **During Discrete Actions**
- Exactly 3 distinct pulses felt
- Always on correct hand region
- Consistent timing (50 ms on, 50 ms off)
- Action completes before next command processed

---

## 13. Configuration Parameters

### 13.1 Adjustable Parameters (drone_e58.ino / drone_k417.ino)

```cpp
// Haptic pins
const int HAPTIC_POT_CS = 6;       // Adjust if using different pin
const int HAPTIC_DATA_PIN = 5;
const int HAPTIC_CLK_PIN = 4;
const int HAPTIC_HV_LE = 3;
const int HAPTIC_HV_CLR = 2;
const int HAPTIC_OUT_PIN = 13;

// Default pulse parameters
const float HAPTIC_DEFAULT_FREQ_HZ = 100.0;    // Train frequency
const unsigned long HAPTIC_DEFAULT_PW_US = 400; // Pulse width
const unsigned long HAPTIC_DEFAULT_TRAIN_MS = 2000; // Duration

// Feedback update rate
const unsigned long HAPTIC_FEEDBACK_UPDATE_MS = 50;

// Potentiometer ranges (per action)
HapticFeedback hapticYaw      = {..., 20, 25, ...};  // Yaw range
HapticFeedback hapticPitch    = {..., 25, 28, ...};  // Pitch range
HapticFeedback hapticRoll     = {..., 31, 35, ...};  // Roll range
HapticFeedback hapticThrottle = {..., 16, 20, ...};  // Throttle range
```

### 13.2 Tuning Guide

**Increase Stimulation Intensity:**
- Raise pot max values: `hapticYaw.potMax = 30;`
- Increase pulse width: `HW800`
- Increase frequency: `HF150`

**Decrease Stimulation Intensity:**
- Lower pot max values
- Decrease pulse width
- Decrease frequency

**Adjust Sensitivity:**
- Change pot min/max ranges
- Modify angle normalization (MAX_ANGLE_DEG constant)
- Change HAPTIC_FEEDBACK_UPDATE_MS for faster response

---

## 14. References & Documentation

- **Hardware**: Arduino Nano RP2040 Connect, LSM6DSOX, WiFiNINA
- **Drone Platforms**: E58 CAM8, Karuisrc K417
- **Protocols**: SPI (MAX5413, HV2701), UDP (drone commands)
- **Standards**: 100 Hz standard stimulation frequency
- **Safety**: Non-invasive surface electrodes, current-limited output

---

## Appendix A: Quick Reference Card

```
╔════════════════════════════════════════════════════════════╗
║              HAPTIC FEEDBACK QUICK REFERENCE              ║
╠════════════════════════════════════════════════════════════╣
║ REGION MAPPINGS                                            ║
║ • M4 (Ch 3):      YAW        | Thumb region              ║
║ • M8 (Ch 8):      PITCH      | Index region              ║
║ • M12 (Ch 12/10): ROLL       | Middle/Ring region        ║
║ • M20 (Ch 18/15): THROTTLE   | Palm region               ║
║                                                            ║
║ POT RANGES (Intensity Scaling)                            ║
║ • YAW:       20-25 (5 intensity levels)                   ║
║ • PITCH:     25-28 (3 intensity levels)                   ║
║ • ROLL:      31-35 (4 intensity levels)                   ║
║ • THROTTLE:  16-20 (4 intensity levels)                   ║
║                                                            ║
║ PULSE MODES                                               ║
║ • Single:  One pulse (default 1000 ms)                    ║
║ • Burst:   3 × 50ms on/off cycles (150 ms total)         ║
║ • Train:   100 Hz frequency (2 sec default)              ║
║                                                            ║
║ COMMAND EXAMPLES                                          ║
║ T          → Takeoff with M4 feedback                     ║
║ L          → Landing with M8 feedback                     ║
║ O          → Zero with M12 feedback                       ║
║ X          → Stop with M20 feedback                       ║
║ P128       → Set pot to mid-range (128/255)              ║
║ HT         → Start 100 Hz train pulse                     ║
║ HAPTIC_STOP → Immediately stop all stimulation           ║
╚════════════════════════════════════════════════════════════╝
```

---

**Document Version**: 1.0  
**Last Updated**: April 2026  
**System Status**: Production Ready  
**Compatibility**: drone_e58.ino, drone_k417.ino
