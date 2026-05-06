/**
 * K417_Drone_Controller.ino
 * ─────────────────────────────────────────────────────────────────────────────
 * Karuisrc K417 WiFi Drone — Direct Arduino Nano RP2040 Connect Controller
 * ─────────────────────────────────────────────────────────────────────────────
 *
 * Hardware:
 *   • Arduino Nano RP2040 Connect (LSM6DSOX IMU onboard, WiFiNINA module)
 *   • Flex sensors on A0 (ring/pinky-up), A1, A2 (index-up), A3 (thumb-down)
 *
 * Control mapping (mirrors control_video_v6.py / IMUAxisMapper):
 *   Pitch  →  forward/backward tilt of the wrist
 *   Roll   →  left/right tilt of the wrist
 *   Yaw    →  wrist rotation (supination/pronation)
 *   Throttle → A0 flex (up) vs A1 flex (down)
 *
 * Network:
 *   Connects to the drone's AP (SSID below) and sends UDP control packets
 *   at ~40 Hz to port 8800, using the exact same binary packet format
 *   as the Python reference implementation in control_video_v6.py.
 *
 * Dependencies (install via Library Manager or board package):
 *   Arduino_LSM6DSOX   (bundled with RP2040 Connect board package)
 *   WiFiNINA           (bundled with RP2040 Connect board package)
 *
 * Build: Board → "Arduino Nano RP2040 Connect", Upload Speed 921600
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <Arduino_LSM6DSOX.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <math.h>
#include <ctype.h>

// Optional TinyML gesture recognition (set ENABLE_GLOVE_NN to 1 to enable).
#define ENABLE_GLOVE_NN 0

#if ENABLE_GLOVE_NN
#include <eloquent_tensorflow_cortexm.h>
#include "neural/glove_fcnn_eloquent_inference/glove_fcnn_40_20_model_data.h"
#endif

// ═══════════════════════════════════════════════════════════════════════════════
//  USER CONFIGURATION — edit these before flashing
// ═══════════════════════════════════════════════════════════════════════════════

// Drone WiFi credentials (K417 default AP)
//const char* DRONE_SSID      = "Drone-BBF0B4";   // drone access-point name
const char* DRONE_SSID      = "WIFI_8K__bcc908";
const char* DRONE_PASSWORD  = "";                 // open network → leave empty

// Drone network address
const char* DRONE_IP        = "192.168.169.1";
const int   DRONE_PORT      = 8800;

// Control loop frequency (Hz)  — keep ≤ 80 to avoid overwhelming the drone
const int   CONTROL_HZ      = 40;

// -------- Haptic Stimulation Control Pins --------
const int HAPTIC_POT_CS   = 6;   // MAX5413 chip select
const int HAPTIC_DATA_PIN = 5;   // SPI data (MOSI)
const int HAPTIC_CLK_PIN  = 4;   // SPI clock
const int HAPTIC_HV_LE    = 3;   // HV2701 latch enable
const int HAPTIC_HV_CLR   = 2;   // HV2701 clear (active low)
const int HAPTIC_OUT_PIN  = 13;  // Pulse output

// -------- Haptic Configuration --------
const unsigned long HAPTIC_SINGLE_PULSE_MS = 1000;
const int HAPTIC_BURST_COUNT = 5;
const unsigned long HAPTIC_BURST_PULSE_MS = 50;
const unsigned long HAPTIC_BURST_PAUSE_MS = 100;
const float HAPTIC_DEFAULT_FREQ_HZ = 100.0;
const unsigned long HAPTIC_DEFAULT_PW_US = 400;
const unsigned long HAPTIC_DEFAULT_TRAIN_MS = 2000;

// Landing burst mirrors the former cam-up sequence behavior from control_video_v6.py.
const int   LAND_BURST_PACKETS = 8;
const int   LAND_BURST_DELAY_MS = 25;

// Telemetry print frequency over USB serial (Hz).
// Keep this lower than CONTROL_HZ to avoid serial blocking jitter.
const int   TELEMETRY_HZ    = 25;

// Mahony filter gains  (mirror the Python MahonyFilter defaults)
const float MAHONY_KP       = 3.5f;
const float MAHONY_KI       = 0.03f;

// Gyro calibration: number of static samples collected at startup
const int   GYRO_CALIB_N    = 250;

// Stick range — matches Python STICK_MIN / STICK_MID / STICK_MAX
const uint8_t STICK_MIN     = 40;
const uint8_t STICK_MID     = 128;
const uint8_t STICK_MAX     = 220;

// Control authority (IMUAxisMapper defaults from Python)
const float PR_DEADZONE     = 8.0f;   // pitch/roll deadzone  [degrees]
const float YAW_DEADZONE    = 8.0f;   // yaw deadzone          [degrees]
const float PR_SENSITIVITY  = 1.0f;
const float YAW_SENSITIVITY = 2.0f;
const float PR_EXPO         = 0.5f;
const float YAW_EXPO        = 0.5f;
const float THR_EXPO        = 0.1f;   // lower expo = earlier throttle saturation
const float MAX_ANGLE_DEG   = 45.0f;  // full-deflection angle

// Throttle flex-sensor parameters (mirror Python flex calibration)
const float FLEX_THRESH_STD_MULTIPLIER = 2.0f;
const float FLEX_NORM_SCALE            = 90.0f;
const float THROTTLE_ALPHA             = 0.12f; // EMA smoothing
const float THR_NET_DEADZONE           = 0.12f; // neutral zone on combined flex net [-1,1]
const float THR_NEUTRAL_SNAP_STICK     = 2.0f;  // snap-to-mid window in stick units

// Flex sensor calibration samples
const int   FLEX_CALIB_N    = 80;

// Throttle channel mapping (matches control_video_v6.py intent).
const int   THR_UP_PIN      = A2;
const int   THR_DOWN_PIN    = A3;

// TinyML gesture-recognition schedule and action filtering.
const unsigned long NN_PERIOD_MS = 80;
const unsigned long NN_HOLD_MS = 350;
const int NN_MIN_MARGIN_Q = 5;
const unsigned long NN_ACTION_COOLDOWN_MS = 900;

// ═══════════════════════════════════════════════════════════════════════════════
//  PACKET CONSTANTS  (from Python control_video_v6.py — do NOT edit)
// ═══════════════════════════════════════════════════════════════════════════════

// Protocol command bytes
const uint8_t CMD_NONE      = 0x00;
const uint8_t CMD_TAKEOFF   = 0x01;
const uint8_t CMD_LAND      = 0x02;
const uint8_t CMD_CALIBRATE = 0x04;
const uint8_t CMD_STOP      = 0x05;

// Headless mode bytes
const uint8_t HEADLESS_OFF  = 0x02;
const uint8_t HEADLESS_ON   = 0x03;
const uint8_t SOMERSAULT_FLAG = 0x08;

// Flip burst timing mirrors the known-good Python implementation.
const int FLIP_BURST_PACKETS = 20;
const int FLIP_RECOVER_PACKETS = 10;

// Fixed header prefix (12 bytes)
const uint8_t PKT_HDR[12] = {
  0xEF, 0x02, 0x7C, 0x00, 0x02, 0x02,
  0x00, 0x01, 0x02, 0x00, 0x00, 0x00
};

// C1 suffix  (6 bytes, after 2-byte counter c1)
const uint8_t C1_SUFFIX[6]  = { 0x00, 0x00, 0x14, 0x00, 0x66, 0x14 };

// Control pad (10 zero bytes, after 6 control bytes + 1 checksum byte)
// Checksum suffix: 0x99 + 44 zeros + 6 bytes
const uint8_t CKSUM_PREFIX  = 0x99;
const uint8_t CKSUM_TAIL[6] = { 0x32, 0x4B, 0x14, 0x2D, 0x00, 0x00 };

// C2 suffix  (18 bytes, after 2-byte counter c2)
const uint8_t C2_SUFFIX[18] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
  0x00, 0x00, 0x14, 0x00, 0x00, 0x00,
  0xFF, 0xFF, 0xFF, 0xFF
};

// C3 suffix  (14 bytes, after 2-byte counter c3)
const uint8_t C3_SUFFIX[14] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x03, 0x00, 0x00, 0x00, 0x10, 0x00,
  0x00, 0x00
};

// Total packet size:
//   12 (hdr) + 2 (c1) + 6 (c1sfx) + 6 (ctrl) + 10 (pad) + 1 (chk)
//   + 1 (0x99) + 44 (zeros) + 6 (tail)
//   + 2 (c2) + 18 (c2sfx)
//   + 2 (c3) + 14 (c3sfx)
//   = 12+2+6+6+10+1+1+44+6+2+18+2+14  = 124 bytes
const int PKT_SIZE = 124;

// ═══════════════════════════════════════════════════════════════════════════════
//  GLOBAL STATE
// ═══════════════════════════════════════════════════════════════════════════════

// --- WiFi & UDP ---
WiFiUDP udp;
IPAddress droneAddr;
bool wifiConnected = false;

// --- Packet counters (16-bit, roll over) ---
volatile uint16_t ctr1 = 0, ctr2 = 1, ctr3 = 2;

// --- Mahony AHRS quaternion & integral error ---
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float eIntX = 0.0f, eIntY = 0.0f, eIntZ = 0.0f;

// Quaternion snapshot of the "zero" orientation (captured after calibration)
float qRef0 = 1.0f, qRef1 = 0.0f, qRef2 = 0.0f, qRef3 = 0.0f;

// --- IMU calibration ---
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
bool  gyroCalibrated = false;
int   gyroCalibCount = 0;
float gyroSumX = 0.0f, gyroSumY = 0.0f, gyroSumZ = 0.0f;

// --- Flex calibration (4 channels) ---
// idx 2 and idx 3 are the throttle channels used by computeThrottle(),
// matching Python IMUAxisMapper semantics where throttle uses channels 2 and 3.
const int FLEX_PINS[4] = { A2, A3, THR_UP_PIN, THR_DOWN_PIN };
float flexMean[4]      = { 512.0f, 512.0f, 512.0f, 512.0f };
float flexStd[4]       = { 20.0f,  20.0f,  20.0f,  20.0f  };
bool  flexCalibrated   = false;
int   flexCalibCount   = 0;
float flexSumBuf[4]    = { 0.0f, 0.0f, 0.0f, 0.0f };
float flexSumSqBuf[4]  = { 0.0f, 0.0f, 0.0f, 0.0f };

// --- Live control values ---
float  throttleSmooth  = (float)STICK_MID;
float  yawDeg = 0.0f, pitchDeg = 0.0f, rollDeg = 0.0f;

// --- Timing ---
unsigned long lastImuMicros  = 0;
unsigned long lastCtrlMillis = 0;
unsigned long lastTelemMillis = 0;
const unsigned long CTRL_INTERVAL_MS = 1000UL / CONTROL_HZ;
const unsigned long TELEM_INTERVAL_MS = 1000UL / TELEMETRY_HZ;

// --- Orientation zero capture flag ---
bool  zeroOrientation = false;  // set true once after full calibration
bool  autoZeroAfterRecalib = false;  // run O-equivalent when recalibration finishes

// --- Flight arming state ---
// When false, throttle is forced to STICK_MIN and no active control is sent.
bool  flightArmed = false;

// --- UDP ownership mode ---
// true  -> Arduino sends flight-control UDP (default)
// false -> Python Video Mode owns UDP; Arduino keeps telemetry only.
bool  arduinoUdpEnabled = true;

// -------- Haptic Stimulation Globals --------
int hapticPotValue = 255;
uint16_t hapticHvState = 0x0000;
volatile bool haptic_spi_busy = false;

// Haptic pulse modes
enum HapticPulseMode { HPM_IDLE = 0, HPM_SINGLE, HPM_BURST, HPM_TRAIN };
HapticPulseMode hapticPulseMode = HPM_IDLE;

// SINGLE pulse timing
unsigned long haptic_single_start_ms = 0, haptic_single_duration_ms = 0;

// BURST pulse timing
int haptic_burst_total = 0, haptic_burst_index = 0;
unsigned long haptic_burst_on_ms = 0, haptic_burst_off_ms = 0, haptic_burst_last_ms = 0;
bool haptic_burst_state_on = false;

// TRAIN pulse timing
unsigned long haptic_train_start_ms = 0, haptic_train_duration_ms_running = 0;
unsigned long haptic_train_period_us = 0, haptic_train_pw_us = 0;
unsigned long haptic_train_next_toggle_us = 0;
bool haptic_train_state_on = false;

// Haptic configuration parameters
float hapticFreq_Hz = HAPTIC_DEFAULT_FREQ_HZ;
unsigned long hapticPulseWidth_us = HAPTIC_DEFAULT_PW_US;
unsigned long hapticTrainDuration_ms = HAPTIC_DEFAULT_TRAIN_MS;

// -------- Haptic Feedback Mapping (from PDF) --------
// Position presets corresponding to hand regions
enum HapticPosition {
  HAPTIC_POS_YAW = 4,        // M4: Thumb region (Channel 3)
  HAPTIC_POS_PITCH = 8,      // M8: Index region (Channel 8)
  HAPTIC_POS_ROLL = 12,      // M12: Middle/Ring region (Channel 12/10)
  HAPTIC_POS_THROTTLE = 20   // M20: Palm region (Channels 18/15)
};

// Haptic feedback state for continuous controls
struct HapticFeedback {
  HapticPosition position;      // Which hand region
  float directionSign;          // +1.0 or -1.0 (for +/- actions)
  int potMin;                   // Minimum pot value
  int potMax;                   // Maximum pot value
  bool isActive;
  unsigned long lastTriggerMs;
};

HapticFeedback hapticYaw = {HAPTIC_POS_YAW, 0.0f, 20, 25, false, 0};
HapticFeedback hapticPitch = {HAPTIC_POS_PITCH, 0.0f, 25, 28, false, 0};
HapticFeedback hapticRoll = {HAPTIC_POS_ROLL, 0.0f, 31, 35, false, 0};
HapticFeedback hapticThrottle = {HAPTIC_POS_THROTTLE, 0.0f, 16, 20, false, 0};

// Haptic feedback update interval (ms)
const unsigned long HAPTIC_FEEDBACK_UPDATE_MS = 50;
unsigned long lastHapticFeedbackMs = 0;

// --- Headless + flip state ---
bool headlessEnabled = false;
bool flipInProgress = false;
int flipBurstRemaining = 0;
int flipRecoverRemaining = 0;
uint8_t flipRoll = STICK_MID;
uint8_t flipPitch = STICK_MID;
uint8_t flipHoldThrottle = STICK_MID;
uint8_t flipHoldYaw = STICK_MID;
uint8_t lastStickThrottle = STICK_MID;
uint8_t lastStickYaw = STICK_MID;

#if ENABLE_GLOVE_NN
using Eloquent::CortexM::TensorFlow;
constexpr int kNNTensorArenaSize = 16 * 1024;
constexpr int kNNNumInputs = 2;
constexpr int kNNNumOutputs = 9;
constexpr int kNNNumOps = 10;
TensorFlow<kNNNumOps, kNNTensorArenaSize> tf;

float nnScalerMean[kNNNumInputs]  = {435.38202f, 400.79325f};
float nnScalerScale[kNNNumInputs] = {72.78391f, 84.44844f};

bool nnReady = false;
bool nnEnabled = false;
unsigned long lastNNMillis = 0;
unsigned long lastNNActionMillis = 0;
int nnLastClass = -1;
unsigned long nnClassStartMillis = 0;
int nnStablePosition = -1;
int nnLastActionClass = -1;
#endif

// ═══════════════════════════════════════════════════════════════════════════════
//  HAPTIC STIMULATION CONTROL
// ═══════════════════════════════════════════════════════════════════════════════

// -------- Haptic SPI Communication --------
void hapticPulseClock() {
  digitalWrite(HAPTIC_CLK_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(HAPTIC_CLK_PIN, LOW);
  delayMicroseconds(1);
}

void hapticShiftBits(uint32_t data, int count) {
  for (int i = count - 1; i >= 0; i--) {
    digitalWrite(HAPTIC_DATA_PIN, (data >> i) & 0x01);
    hapticPulseClock();
  }
}

void hapticSendToHV2701(uint16_t data) {
  haptic_spi_busy = true;
  digitalWrite(HAPTIC_POT_CS, HIGH);
  digitalWrite(HAPTIC_HV_LE, LOW);

  hapticShiftBits(data, 16);

  digitalWrite(HAPTIC_HV_LE, HIGH);
  delayMicroseconds(2);
  digitalWrite(HAPTIC_HV_LE, LOW);
  haptic_spi_busy = false;
}

void hapticSetPot(byte value) {
  haptic_spi_busy = true;

  digitalWrite(HAPTIC_HV_LE, HIGH);   // Avoid latch accident
  digitalWrite(HAPTIC_POT_CS, LOW);
  
  digitalWrite(HAPTIC_DATA_PIN, 1);   // Command bit for Wiper1
  hapticPulseClock();

  for (int i = 7; i >= 0; i--) {
    digitalWrite(HAPTIC_DATA_PIN, (value >> i) & 1);
    hapticPulseClock();
  }

  digitalWrite(HAPTIC_POT_CS, HIGH);
  haptic_spi_busy = false;
}

// -------- Haptic Pulse Control --------
void hapticStopPulses() {
  hapticPulseMode = HPM_IDLE;
  digitalWrite(HAPTIC_OUT_PIN, LOW);
  haptic_burst_index = 0;
  haptic_burst_state_on = false;
  haptic_train_state_on = false;
}

void hapticStartSingle(unsigned long d) {
  hapticStopPulses();
  haptic_single_start_ms = millis();
  haptic_single_duration_ms = d;
  digitalWrite(HAPTIC_OUT_PIN, HIGH);
  hapticPulseMode = HPM_SINGLE;
}

void hapticStartBurst(int n, unsigned long on_ms, unsigned long off_ms) {
  hapticStopPulses();
  haptic_burst_total = max(0, n);
  haptic_burst_index = 0;
  haptic_burst_on_ms = on_ms;
  haptic_burst_off_ms = off_ms;
  haptic_burst_last_ms = millis();
  haptic_burst_state_on = true;
  digitalWrite(HAPTIC_OUT_PIN, HIGH);
  hapticPulseMode = HPM_BURST;
}

void hapticStartTrain(float f, unsigned long pw_us, unsigned long total_ms) {
  if (f <= 0 || pw_us == 0) return;
  hapticStopPulses();
  haptic_train_period_us = (unsigned long)round(1000000.0f / f);
  haptic_train_pw_us = pw_us;
  haptic_train_start_ms = millis();
  haptic_train_duration_ms_running = total_ms;
  haptic_train_next_toggle_us = micros() + pw_us;
  haptic_train_state_on = true;
  digitalWrite(HAPTIC_OUT_PIN, HIGH);
  hapticPulseMode = HPM_TRAIN;
}

void hapticUpdatePulses() {
  unsigned long now_ms = millis();
  unsigned long now_us = micros();

  switch (hapticPulseMode) {
    case HPM_IDLE:
      break;

    case HPM_SINGLE:
      if (now_ms - haptic_single_start_ms >= haptic_single_duration_ms) {
        digitalWrite(HAPTIC_OUT_PIN, LOW);
        hapticPulseMode = HPM_IDLE;
      }
      break;

    case HPM_BURST:
      if (haptic_burst_index >= haptic_burst_total) {
        digitalWrite(HAPTIC_OUT_PIN, LOW);
        hapticPulseMode = HPM_IDLE;
        break;
      }
      if (haptic_burst_state_on) {
        if (now_ms - haptic_burst_last_ms >= haptic_burst_on_ms) {
          haptic_burst_state_on = false;
          digitalWrite(HAPTIC_OUT_PIN, LOW);
          haptic_burst_last_ms = now_ms;
          haptic_burst_index++;
        }
      } else {
        if (haptic_burst_index >= haptic_burst_total) {
          digitalWrite(HAPTIC_OUT_PIN, LOW);
          hapticPulseMode = HPM_IDLE;
        } else if (now_ms - haptic_burst_last_ms >= haptic_burst_off_ms) {
          haptic_burst_state_on = true;
          digitalWrite(HAPTIC_OUT_PIN, HIGH);
          haptic_burst_last_ms = now_ms;
        }
      }
      break;

    case HPM_TRAIN:
      if (now_ms - haptic_train_start_ms >= haptic_train_duration_ms_running) {
        digitalWrite(HAPTIC_OUT_PIN, LOW);
        hapticPulseMode = HPM_IDLE;
        haptic_train_state_on = false;
        break;
      }
      if (haptic_train_state_on) {
        unsigned long on_since = now_us - (haptic_train_next_toggle_us - haptic_train_pw_us);
        if (on_since >= haptic_train_pw_us) {
          digitalWrite(HAPTIC_OUT_PIN, LOW);
          haptic_train_state_on = false;
          unsigned long off_time_us = (haptic_train_period_us > haptic_train_pw_us) ?
                                      (haptic_train_period_us - haptic_train_pw_us) : 0;
          haptic_train_next_toggle_us = now_us + off_time_us;
        }
      } else {
        if ((long)(now_us - haptic_train_next_toggle_us) >= 0) {
          digitalWrite(HAPTIC_OUT_PIN, HIGH);
          haptic_train_state_on = true;
          haptic_train_next_toggle_us = now_us + haptic_train_pw_us;
        }
      }
      break;
  }
}

// -------- Haptic Feedback Triggers (from PDF mapping) --------
void triggerHapticFeedback(HapticFeedback *feedback, int potValue) {
  // K417 doesn't have position presets array, use direct HV state
  // Map position enum to HV2701 control word
  uint16_t hvState = 0x0000;
  switch (feedback->position) {
    case HAPTIC_POS_YAW:      hvState = 0x0101; break;   // Thumb region
    case HAPTIC_POS_PITCH:    hvState = 0x0102; break;   // Index region
    case HAPTIC_POS_ROLL:     hvState = 0x0104; break;   // Middle/Ring region
    case HAPTIC_POS_THROTTLE: hvState = 0x0108; break;   // Palm region
    default: hvState = 0x0000;
  }
  hapticSendToHV2701(hvState);
  hapticSetPot(potValue);
  hapticStartTrain(HAPTIC_DEFAULT_FREQ_HZ, HAPTIC_DEFAULT_PW_US, HAPTIC_DEFAULT_TRAIN_MS);
  feedback->isActive = true;
  feedback->lastTriggerMs = millis();
}

void triggerHapticAction(HapticPosition position, int potValue) {
  uint16_t hvState = 0x0000;
  switch (position) {
    case HAPTIC_POS_YAW:      hvState = 0x0101; break;
    case HAPTIC_POS_PITCH:    hvState = 0x0102; break;
    case HAPTIC_POS_ROLL:     hvState = 0x0104; break;
    case HAPTIC_POS_THROTTLE: hvState = 0x0108; break;
    default: hvState = 0x0000;
  }
  hapticSendToHV2701(hvState);
  hapticSetPot(potValue);
  hapticStartBurst(3, 50, 50);
}

// Update haptic feedback based on current control values (called from loop)
void updateHapticFeedback(float yaw, float pitch, float roll, uint8_t throttle) {
  // Skip all feedback while flip is in progress (gesture 7 action)
  if (flipInProgress) return;

  unsigned long now = millis();
  if (now - lastHapticFeedbackMs < HAPTIC_FEEDBACK_UPDATE_MS) return;
  lastHapticFeedbackMs = now;

  // YAW feedback (Train mode, continuous)
  if (yaw != 0.0f) {
    float normalized = constrain(fabsf(yaw) / MAX_ANGLE_DEG, 0.0f, 1.0f);
    int potVal = hapticYaw.potMin + (int)(normalized * (hapticYaw.potMax - hapticYaw.potMin));
    if (!hapticYaw.isActive || yaw > 0.0f != (hapticYaw.directionSign > 0.0f)) {
      triggerHapticFeedback(&hapticYaw, potVal);
      hapticYaw.directionSign = (yaw > 0.0f) ? 1.0f : -1.0f;
    }
  } else if (hapticYaw.isActive) {
    hapticYaw.isActive = false;
  }

  // PITCH feedback (Train mode, continuous)
  if (pitch != 0.0f) {
    float normalized = constrain(fabsf(pitch) / MAX_ANGLE_DEG, 0.0f, 1.0f);
    int potVal = hapticPitch.potMin + (int)(normalized * (hapticPitch.potMax - hapticPitch.potMin));
    if (!hapticPitch.isActive || pitch > 0.0f != (hapticPitch.directionSign > 0.0f)) {
      triggerHapticFeedback(&hapticPitch, potVal);
      hapticPitch.directionSign = (pitch > 0.0f) ? 1.0f : -1.0f;
    }
  } else if (hapticPitch.isActive) {
    hapticPitch.isActive = false;
  }

  // ROLL feedback (Train mode, continuous)
  if (roll != 0.0f) {
    float normalized = constrain(fabsf(roll) / MAX_ANGLE_DEG, 0.0f, 1.0f);
    int potVal = hapticRoll.potMin + (int)(normalized * (hapticRoll.potMax - hapticRoll.potMin));
    if (!hapticRoll.isActive || roll > 0.0f != (hapticRoll.directionSign > 0.0f)) {
      triggerHapticFeedback(&hapticRoll, potVal);
      hapticRoll.directionSign = (roll > 0.0f) ? 1.0f : -1.0f;
    }
  } else if (hapticRoll.isActive) {
    hapticRoll.isActive = false;
  }

  // THROTTLE feedback (Train mode, continuous)
  if (throttle != STICK_MID) {
    float normalized = constrain(fabsf((int)throttle - (int)STICK_MID) / (float)(STICK_MAX - STICK_MID), 0.0f, 1.0f);
    int potVal = hapticThrottle.potMin + (int)(normalized * (hapticThrottle.potMax - hapticThrottle.potMin));
    if (!hapticThrottle.isActive || (throttle > STICK_MID) != (hapticThrottle.directionSign > 0.0f)) {
      triggerHapticFeedback(&hapticThrottle, potVal);
      hapticThrottle.directionSign = (throttle > STICK_MID) ? 1.0f : -1.0f;
    }
  } else if (hapticThrottle.isActive) {
    hapticThrottle.isActive = false;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MAHONY AHRS
//  Ported faithfully from the Python MahonyFilter class in control_video_v6.py,
//  keeping identical gain semantics and the two-step (bias-then-quaternion)
//  update sequence.
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * mahonyUpdate()
 * Feed one IMU sample into the Mahony AHRS.
 *
 * @param ax,ay,az  accelerometer in g  (already bias-corrected)
 * @param gx,gy,gz  gyroscope in deg/s  (already bias-corrected + mapped)
 * @param dt        sample interval in seconds
 *
 * Internally normalises the accelerometer, computes the cross-product
 * error between the measured and estimated gravity vector, drives the
 * integral and proportional feedback terms into the gyroscope rates,
 * then integrates the quaternion using the corrected rates.
 */
void mahonyUpdate(float ax, float ay, float az,
                  float gx, float gy, float gz,
                  float dt)
{
  // Convert gyroscope to radians/s
  float gxR = gx * (float)(M_PI / 180.0);
  float gyR = gy * (float)(M_PI / 180.0);
  float gzR = gz * (float)(M_PI / 180.0);

  // Normalise accelerometer; if zero-norm skip (sensor stall / overflow)
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm < 1e-6f) return;
  ax /= norm; ay /= norm; az /= norm;

  // Estimated direction of gravity from quaternion (body frame)
  float vx = 2.0f * (q1 * q3 - q0 * q2);
  float vy = 2.0f * (q0 * q1 + q2 * q3);
  float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  // Cross-product error between measured and estimated gravity
  float ex = ay * vz - az * vy;
  float ey = az * vx - ax * vz;
  float ez = ax * vy - ay * vx;

  // Integral feedback (integral error in rad/s)
  eIntX += ex * MAHONY_KI * dt;
  eIntY += ey * MAHONY_KI * dt;
  eIntZ += ez * MAHONY_KI * dt;

  // Apply proportional + integral feedback to gyro rates
  gxR += MAHONY_KP * ex + eIntX;
  gyR += MAHONY_KP * ey + eIntY;
  gzR += MAHONY_KP * ez + eIntZ;

  // Integrate quaternion  (first-order Runge-Kutta)
  float hw  = 0.5f * dt;
  float qa  = q0, qb = q1, qc = q2;
  q0 += (-qb * gxR - qc * gyR - q3 * gzR) * hw;
  q1 += ( qa * gxR + qc * gzR - q3 * gyR) * hw;
  q2 += ( qa * gyR - qb * gzR + q3 * gxR) * hw;
  q3 += ( qa * gzR + qb * gyR - qc * gxR) * hw;

  // Re-normalise quaternion
  norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
}

/**
 * captureZero()
 * Record the current orientation as the reference ("zero") by storing the
 * conjugate of the current quaternion.  Subsequent calls to getRelativeEuler()
 * will return angles relative to this reference.
 */
void captureZero()
{
  // Store conjugate of current quaternion (= inverse for unit quaternion)
  qRef0 =  q0;
  qRef1 = -q1;
  qRef2 = -q2;
  qRef3 = -q3;
  Serial.println(F("[AHRS] Zero orientation captured."));
}

/**
 * getRelativeEuler()
 * Compute Yaw, Pitch, Roll (degrees) relative to the stored zero orientation.
 * Mirrors MahonyFilter.get_euler_relative() in Python.
 *
 * @param yaw_out, pitch_out, roll_out  output references (degrees)
 */
void getRelativeEuler(float &yaw_out, float &pitch_out, float &roll_out)
{
  // Quaternion product: qRef * q  (qRef is the conjugate of the zero quat)
  float w = qRef0*q0 - qRef1*q1 - qRef2*q2 - qRef3*q3;
  float x = qRef0*q1 + qRef1*q0 + qRef2*q3 - qRef3*q2;
  float y = qRef0*q2 - qRef1*q3 + qRef2*q0 + qRef3*q1;
  float z = qRef0*q3 + qRef1*q2 - qRef2*q1 + qRef3*q0;

  // Roll (x-axis rotation)
  roll_out  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y))
              * (float)(180.0 / M_PI);

  // Pitch (y-axis rotation) — clamped to avoid asinf domain errors
  float sinp = 2.0f*(w*y - z*x);
  sinp = constrain(sinp, -1.0f, 1.0f);
  pitch_out = asinf(sinp) * (float)(180.0 / M_PI);

  // Yaw (z-axis rotation)
  yaw_out = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z))
            * (float)(180.0 / M_PI);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  CONTROL MAPPING  (mirrors Python IMUAxisMapper)
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * angleToStick()
 * Convert a signed angle (degrees) to a drone stick byte [STICK_MIN, STICK_MAX].
 * Applies a deadzone then exponential curve, identical to Python _a2s().
 *
 * @param angle      measured angle in degrees
 * @param deadzone   deadzone width in degrees
 * @param sensitivity multiplier applied before clipping to 1.0
 * @param expo       expo factor [0=linear, 1=cubic]
 */
uint8_t angleToStick(float angle, float deadzone, float sensitivity, float expo)
{
  float sign = (angle >= 0.0f) ? 1.0f : -1.0f;
  float mag  = fabsf(angle);

  if (mag < deadzone) return STICK_MID;

  float norm   = (mag - deadzone) / (MAX_ANGLE_DEG - deadzone);
  norm = min(norm * sensitivity, 1.0f);
  float curved = norm * (1.0f - expo) + norm*norm*norm * expo;
  curved = constrain(curved, 0.0f, 1.0f);

  float raw = STICK_MID + sign * curved * (float)(STICK_MAX - STICK_MID);
  return (uint8_t)constrain((int)raw, STICK_MIN, STICK_MAX);
}

/**
 * flexDeflection()
 * Convert a raw ADC reading to a signed normalised deflection [-1, 1].
 * Values within FLEX_THRESH_STD_MULTIPLIER * std of the rest mean → 0.
 * Mirrors Python IMUAxisMapper._flex_def().
 *
 * @param raw   current ADC reading (0-1023)
 * @param idx   channel index (0-3) into flexMean / flexStd arrays
 */
float flexDeflection(int raw, int idx)
{
  float delta  = (float)raw - flexMean[idx];
  float thresh = FLEX_THRESH_STD_MULTIPLIER * flexStd[idx];

  if (fabsf(delta) < thresh) return 0.0f;

  float signed_excess = delta - (delta >= 0.0f ? thresh : -thresh);
  return constrain(signed_excess / FLEX_NORM_SCALE, -1.0f, 1.0f);
}

/**
 * computeThrottle()
 * Map flex sensor deflections on A0 (up) and A1 (down) to a throttle stick
 * value, with exponential curve and EMA smoothing.
 * Mirrors Python IMUAxisMapper.compute() throttle section.
 *
 * @param rawA2   ADC reading from A0 (up flex channel)
 * @param rawA3   ADC reading from A1 (down flex channel)
 * @return        smoothed stick byte [STICK_MIN, STICK_MAX]
 */
uint8_t computeThrottle(int rawA2, int rawA3)
{
  if (!flexCalibrated) return (uint8_t)throttleSmooth;

  float d2  = flexDeflection(rawA2, 2);   // channel idx 2 → A2
  float d3  = flexDeflection(rawA3, 3);   // channel idx 3 → A3
  float net = constrain(d2 - d3, -1.0f, 1.0f);

  // Deadzone around neutral, then re-normalize so full stick range is preserved.
  float s   = (net >= 0.0f) ? 1.0f : -1.0f;
  float m   = fabsf(net);
  float mapped = 0.0f;
  if (m > THR_NET_DEADZONE) {
    mapped = (m - THR_NET_DEADZONE) / (1.0f - THR_NET_DEADZONE);
  }

  float ct  = mapped * (1.0f - THR_EXPO) + mapped*mapped*mapped * THR_EXPO;

  float raw = STICK_MID + s * ct * (float)(STICK_MAX - STICK_MID);
  raw = constrain(raw, (float)STICK_MIN, (float)STICK_MAX);

  // EMA smoothing
  throttleSmooth += (raw - throttleSmooth) * THROTTLE_ALPHA;
  if (mapped == 0.0f && fabsf(throttleSmooth - (float)STICK_MID) <= THR_NEUTRAL_SNAP_STICK) {
    throttleSmooth = (float)STICK_MID;
  }
  return (uint8_t)constrain((int)throttleSmooth, STICK_MIN, STICK_MAX);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PACKET BUILDER  (exact port of Python build_packet())
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * buildPacket()
 * Assemble a 124-byte K417 control packet into buf[].
 *
 * Layout (all lengths in bytes):
 *   [12] Header
 *   [ 2] c1 (little-endian)
 *   [ 6] C1_SUFFIX
 *   [ 1] roll
 *   [ 1] pitch
 *   [ 1] throttle
 *   [ 1] yaw
 *   [ 1] command
 *   [ 1] headless
 *   [10] zero padding  (CTRL_PAD)
 *   [ 1] XOR checksum of the 6 control bytes
 *   [ 1] 0x99
 *   [44] zero bytes
 *   [ 6] CKSUM_TAIL
 *   [ 2] c2 (little-endian)
 *   [18] C2_SUFFIX
 *   [ 2] c3 (little-endian)
 *   [14] C3_SUFFIX
 *   ────
 *  124 bytes total
 */
void buildPacket(uint8_t *buf,
                 uint8_t roll, uint8_t pitch,
                 uint8_t throttle, uint8_t yaw,
                 uint8_t command, uint8_t headless,
                 uint16_t c1, uint16_t c2, uint16_t c3)
{
  int i = 0;

  // Header
  memcpy(buf + i, PKT_HDR, 12);  i += 12;

  // c1 little-endian
  buf[i++] = (uint8_t)(c1 & 0xFF);
  buf[i++] = (uint8_t)((c1 >> 8) & 0xFF);

  // C1 suffix
  memcpy(buf + i, C1_SUFFIX, 6);  i += 6;

  // 6 control bytes
  uint8_t ctrl[6] = { roll, pitch, throttle, yaw, command, headless };
  memcpy(buf + i, ctrl, 6);  i += 6;

  // 10 zero padding bytes
  memset(buf + i, 0x00, 10);  i += 10;

  // Checksum: XOR of the 6 control bytes
  uint8_t chk = 0;
  for (int j = 0; j < 6; j++) chk ^= ctrl[j];
  buf[i++] = chk;

  // 0x99 marker
  buf[i++] = CKSUM_PREFIX;

  // 44 zero bytes
  memset(buf + i, 0x00, 44);  i += 44;

  // Checksum tail
  memcpy(buf + i, CKSUM_TAIL, 6);  i += 6;

  // c2 little-endian
  buf[i++] = (uint8_t)(c2 & 0xFF);
  buf[i++] = (uint8_t)((c2 >> 8) & 0xFF);

  // C2 suffix
  memcpy(buf + i, C2_SUFFIX, 18);  i += 18;

  // c3 little-endian
  buf[i++] = (uint8_t)(c3 & 0xFF);
  buf[i++] = (uint8_t)((c3 >> 8) & 0xFF);

  // C3 suffix
  memcpy(buf + i, C3_SUFFIX, 14);  i += 14;

  // Safety check (will be optimised away in release builds)
  // assert(i == PKT_SIZE);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

/** Increment and return the current value of a 16-bit packet counter. */
uint16_t nextCounter(volatile uint16_t &ctr)
{
  uint16_t val = ctr;
  ctr = (ctr + 1) & 0xFFFF;
  return val;
}

/** Send one control packet over UDP. */
void sendPacket(uint8_t roll, uint8_t pitch,
                uint8_t throttle, uint8_t yaw,
                uint8_t command = CMD_NONE,
                uint8_t headless = HEADLESS_OFF)
{
  if (!wifiConnected || !arduinoUdpEnabled) return;

  uint16_t c1 = nextCounter(ctr1);
  uint16_t c2 = nextCounter(ctr2);
  uint16_t c3 = nextCounter(ctr3);

  uint8_t pkt[PKT_SIZE];
  buildPacket(pkt, roll, pitch, throttle, yaw, command, headless, c1, c2, c3);

  udp.beginPacket(droneAddr, DRONE_PORT);
  udp.write(pkt, PKT_SIZE);
  udp.endPacket();
}

/**
 * sendLandSequence()
 * Controlled landing trigger: send a short burst of CMD_LAND packets,
 * matching the former CAM-UP landing pattern used from Python.
 */
void sendLandSequence()
{
  for (int i = 0; i < LAND_BURST_PACKETS; ++i) {
    sendPacket(STICK_MID, STICK_MID, STICK_MID, STICK_MID, CMD_LAND);
    delay(LAND_BURST_DELAY_MS);
  }
}

bool decodeFlipDirection(const char* direction, uint8_t &outRoll, uint8_t &outPitch)
{
  if (!direction) return false;

  if (strcmp(direction, "FORWARD") == 0) {
    outPitch = STICK_MAX;
    outRoll = STICK_MID;
    return true;
  }
  if (strcmp(direction, "BACKWARD") == 0) {
    outPitch = STICK_MIN;
    outRoll = STICK_MID;
    return true;
  }
  if (strcmp(direction, "LEFT") == 0) {
    outPitch = STICK_MID;
    outRoll = STICK_MIN;
    return true;
  }
  if (strcmp(direction, "RIGHT") == 0) {
    outPitch = STICK_MID;
    outRoll = STICK_MAX;
    return true;
  }

  return false;
}

void startFlip(const char* direction, uint8_t stickThrottleSnapshot, uint8_t stickYawSnapshot)
{
  if (!flightArmed) {
    Serial.println(F("[FLIP] Ignored: drone not armed/airborne."));
    return;
  }
  if (flipInProgress) {
    Serial.println(F("[FLIP] Ignored: flip already in progress."));
    return;
  }

  uint8_t dirRoll = STICK_MID;
  uint8_t dirPitch = STICK_MID;
  if (!decodeFlipDirection(direction, dirRoll, dirPitch)) {
    Serial.println(F("[FLIP] Invalid direction. Use FORWARD/BACKWARD/LEFT/RIGHT."));
    return;
  }

  flipRoll = dirRoll;
  flipPitch = dirPitch;
  flipHoldYaw = stickYawSnapshot;
  flipHoldThrottle = max(stickThrottleSnapshot, (uint8_t)(STICK_MID + 6));
  flipBurstRemaining = FLIP_BURST_PACKETS;
  flipRecoverRemaining = FLIP_RECOVER_PACKETS;
  flipInProgress = true;

  Serial.print(F("[FLIP] START "));
  Serial.println(direction);
}

void handleSerialCommandLine(const char* cmdLine)
{
  if (!cmdLine || cmdLine[0] == '\0') return;

  char cmd[48];
  size_t n = strnlen(cmdLine, sizeof(cmd) - 1);
  memcpy(cmd, cmdLine, n);
  cmd[n] = '\0';

  for (size_t i = 0; cmd[i] != '\0'; i++) {
    cmd[i] = (char)toupper((unsigned char)cmd[i]);
  }

  if (strcmp(cmd, "T") == 0 || strcmp(cmd, "TAKEOFF") == 0) {
    flightArmed = true;
    sendPacket(STICK_MID, STICK_MID, STICK_MID, STICK_MID, CMD_TAKEOFF);
    // Haptic feedback: Takeoff on thumb region (M4) with pot 20
    triggerHapticAction(HAPTIC_POS_YAW, 20);
    Serial.println(F("[HAPTIC] Takeoff feedback triggered"));
    Serial.println(F("[CMD] TAKEOFF sent"));
    return;
  }

  if (strcmp(cmd, "L") == 0 || strcmp(cmd, "LAND") == 0) {
    sendLandSequence();
    flightArmed = false;
    flipInProgress = false;
    // Haptic feedback: Landing on index region (M8) with pot 25
    triggerHapticAction(HAPTIC_POS_PITCH, 25);
    Serial.println(F("[HAPTIC] Landing feedback triggered"));
    Serial.println(F("[CMD] LAND (controlled sequence) sent"));
    return;
  }

  if (strcmp(cmd, "X") == 0 || strcmp(cmd, "STOP") == 0) {
    flightArmed = false;
    flipInProgress = false;
    sendPacket(STICK_MID, STICK_MID, STICK_MID, STICK_MID, CMD_STOP);
    // Haptic feedback: Stop on palm region (M20) with pot 18
    triggerHapticAction(HAPTIC_POS_THROTTLE, 18);
    Serial.println(F("[HAPTIC] Stop feedback triggered"));
    Serial.println(F("[CMD] EMERGENCY STOP sent"));
    return;
  }

  if (strcmp(cmd, "C") == 0 || strcmp(cmd, "CAL") == 0 || strcmp(cmd, "CALIBRATE") == 0) {
    sendPacket(STICK_MID, STICK_MID, STICK_MID, STICK_MID, CMD_CALIBRATE);
    Serial.println(F("[CMD] CALIBRATE sent"));
    return;
  }

  if (strcmp(cmd, "O") == 0 || strcmp(cmd, "ZERO") == 0) {
    captureZero();
    autoZeroAfterRecalib = false;
    // Haptic feedback: Zero on middle region (M12) with pot 30
    triggerHapticAction(HAPTIC_POS_ROLL, 30);
    Serial.println(F("[HAPTIC] Zero feedback triggered"));
    return;
  }

  if (strcmp(cmd, "R") == 0 || strcmp(cmd, "RECAL") == 0 || strcmp(cmd, "RECALIBRATE") == 0) {
    triggerLocalRecalibration();
    return;
  }

  if (strcmp(cmd, "P") == 0 || strcmp(cmd, "PYUDP") == 0) {
    arduinoUdpEnabled = false;
    flightArmed = false;
    flipInProgress = false;
    Serial.println(F("[MODE] PYTHON_UDP ON (Arduino UDP paused)"));
    return;
  }

  if (strcmp(cmd, "A") == 0 || strcmp(cmd, "ARDUDP") == 0) {
    arduinoUdpEnabled = true;
    Serial.println(F("[MODE] ARDUINO_UDP ON"));
    return;
  }

  if (strcmp(cmd, "H") == 0 || strcmp(cmd, "HEADLESS") == 0) {
    headlessEnabled = !headlessEnabled;
    Serial.print(F("[MODE] HEADLESS "));
    Serial.println(headlessEnabled ? F("ON") : F("OFF"));
    return;
  }

  if (strncmp(cmd, "FLIP:", 5) == 0) {
    startFlip(cmd + 5, lastStickThrottle, lastStickYaw);
    return;
  }
  if (strcmp(cmd, "FF") == 0) { startFlip("FORWARD",  lastStickThrottle, lastStickYaw); return; }
  if (strcmp(cmd, "FB") == 0) { startFlip("BACKWARD", lastStickThrottle, lastStickYaw); return; }
  if (strcmp(cmd, "FL") == 0) { startFlip("LEFT",     lastStickThrottle, lastStickYaw); return; }
  if (strcmp(cmd, "FR") == 0) { startFlip("RIGHT",    lastStickThrottle, lastStickYaw); return; }

#if ENABLE_GLOVE_NN
  if (strcmp(cmd, "N") == 0 || strcmp(cmd, "NN") == 0) {
    nnEnabled = !nnEnabled;
    Serial.print(F("[NN] "));
    Serial.println(nnEnabled ? F("ENABLED") : F("DISABLED"));
    return;
  }
#endif

  // -------- Haptic Control Commands --------
  if (strcmp(cmd, "HAPTIC_STOP") == 0) {
    hapticStopPulses();
    Serial.println(F("[HAPTIC] Pulses stopped"));
    return;
  }

  if (cmd[0] == 'P' && cmd[1] >= '0' && cmd[1] <= '9') {
    int v = atoi(cmd + 1);
    if (v >= 0 && v <= 255) {
      hapticPotValue = v;
      hapticSetPot(v);
      float resistance_Ohms = (v / 255.0) * 10000.0;
      Serial.print(F("[HAPTIC] Pot = "));
      Serial.print(v);
      Serial.print(F("/255 (~"));
      Serial.print(resistance_Ohms, 1);
      Serial.println(F(" Ohms)"));
    }
    return;
  }

  if (strcmp(cmd, "HS") == 0) {
    hapticStartSingle(HAPTIC_SINGLE_PULSE_MS);
    Serial.println(F("[HAPTIC] Single pulse started"));
    return;
  }

  if (cmd[0] == 'H' && cmd[1] == 'S' && cmd[2] == 'D') {
    unsigned long duration = atol(cmd + 3);
    if (duration > 0) {
      hapticStartSingle(duration);
      Serial.print(F("[HAPTIC] Single pulse "));
      Serial.print(duration);
      Serial.println(F(" ms"));
    }
    return;
  }

  if (strcmp(cmd, "HB") == 0) {
    hapticStartBurst(HAPTIC_BURST_COUNT, HAPTIC_BURST_PULSE_MS, HAPTIC_BURST_PAUSE_MS);
    Serial.println(F("[HAPTIC] Burst started"));
    return;
  }

  if (cmd[0] == 'H' && cmd[1] == 'B' && cmd[2] == 'C') {
    int count = atoi(cmd + 3);
    if (count > 0) {
      hapticStartBurst(count, HAPTIC_BURST_PULSE_MS, HAPTIC_BURST_PAUSE_MS);
      Serial.print(F("[HAPTIC] Burst x"));
      Serial.println(count);
    }
    return;
  }

  if (cmd[0] == 'H' && cmd[1] == 'T') {
    hapticStartTrain(hapticFreq_Hz, hapticPulseWidth_us, hapticTrainDuration_ms);
    Serial.println(F("[HAPTIC] Train started"));
    return;
  }

  if (cmd[0] == 'H' && cmd[1] == 'F') {
    hapticFreq_Hz = atof(cmd + 2);
    if (hapticFreq_Hz > 0) {
      Serial.print(F("[HAPTIC] Frequency = "));
      Serial.print(hapticFreq_Hz, 1);
      Serial.println(F(" Hz"));
    }
    return;
  }

  if (cmd[0] == 'H' && cmd[1] == 'W') {
    hapticPulseWidth_us = atol(cmd + 2);
    if (hapticPulseWidth_us > 0) {
      Serial.print(F("[HAPTIC] Pulse width = "));
      Serial.print(hapticPulseWidth_us);
      Serial.println(F(" us"));
    }
    return;
  }

  if (cmd[0] == 'H' && cmd[1] == 'D') {
    hapticTrainDuration_ms = atol(cmd + 2);
    if (hapticTrainDuration_ms > 0) {
      Serial.print(F("[HAPTIC] Train duration = "));
      Serial.print(hapticTrainDuration_ms);
      Serial.println(F(" ms"));
    }
    return;
  }

  if (cmd[0] == 'H' && cmd[1] == 'S' && cmd[2] == 'W') {
    int sw = atoi(cmd + 3);
    if (sw >= 0 && sw <= 15) {
      hapticHvState ^= (1 << sw);
      hapticSendToHV2701(hapticHvState);
      Serial.print(F("[HAPTIC] SW"));
      Serial.print(sw);
      Serial.print(F(" toggled -> HV state: "));
      for (int i = 15; i >= 0; i--) Serial.print((hapticHvState >> i) & 1);
      Serial.println();
    }
    return;
  }

  if (strcmp(cmd, "?") == 0 || strcmp(cmd, "HELP") == 0) {
    Serial.println(F("Commands: T L X C O R H FLIP:<FORWARD|BACKWARD|LEFT|RIGHT> P A N ?"));
    Serial.println(F("Haptic: Pxxx(pot) HS(pulse) HB(burst) HT(train) HFxx(freq) HWxx(width) HDxx(duration) HSWx(switch) ?"));
    return;
  }

  Serial.print(F("[CMD] Unknown: "));
  Serial.println(cmd);
}

/**
 * emitTelemetry()
 * Compact, fixed-format telemetry line for telemetry_monitor.py parser:
 *   Y:<yaw> P:<pitch> R:<roll> T:<thr> A0:<raw> A1:<raw> A2:<raw> A3:<raw>
 * Uses direct Serial.print calls for maximum Arduino-core compatibility.
 */
void emitTelemetry(float yaw, float pitch, float roll,
                   uint8_t throttle, int rawA0, int rawA1, int rawA2, int rawA3)
{
  // Keep the exact tokens expected by telemetry_monitor.py regex parser.
  Serial.print(F("Y:"));  Serial.print(yaw, 1);
  Serial.print(F(" P:")); Serial.print(pitch, 1);
  Serial.print(F(" R:")); Serial.print(roll, 1);
  Serial.print(F(" T:")); Serial.print((int)throttle);
  Serial.print(F(" A0:")); Serial.print(rawA0);
  Serial.print(F(" A1:")); Serial.print(rawA1);
  Serial.print(F(" A2:")); Serial.print(rawA2);
  Serial.print(F(" A3:")); Serial.print(rawA3);
#if ENABLE_GLOVE_NN
  Serial.print(F(" POS:")); Serial.println(nnStablePosition);
#else
  Serial.print(F(" POS:")); Serial.println(-1);
#endif
}

void triggerLocalRecalibration()
{
  // Re-run full local gyro + flex calibration.
  flightArmed = false;
  gyroCalibrated  = false;
  flexCalibrated  = false;
  zeroOrientation = false;
  gyroCalibCount  = 0;
  flexCalibCount  = 0;
  gyroSumX = gyroSumY = gyroSumZ = 0.0f;
  memset(flexSumBuf,   0, sizeof(flexSumBuf));
  memset(flexSumSqBuf, 0, sizeof(flexSumSqBuf));
  eIntX = eIntY = eIntZ = 0.0f;
  q0=1.0f; q1=q2=q3=0.0f;
  throttleSmooth = (float)STICK_MID;
  autoZeroAfterRecalib = true;

#if ENABLE_GLOVE_NN
  nnStablePosition = -1;
  nnLastClass = -1;
  nnClassStartMillis = 0;
  nnLastActionClass = -1;
#endif

  Serial.println(F("[CALIB] Re-calibrating — keep STILL..."));
}

#if ENABLE_GLOVE_NN
const char* nnClassName(int cls)
{
  switch (cls) {
    case 0: return "neutral";
    case 1: return "land";
    case 2: return "stop";
    case 3: return "takeoff";
    case 4: return "zero";
    case 5: return "class5";
    case 6: return "class6";
    case 7: return "recal";
    case 8: return "class8";
    default: return "unknown";
  }
}

void applyNNAction(int cls)
{
  switch (cls) {
    case 2:
      flightArmed = false;
      sendPacket(STICK_MID, STICK_MID, STICK_MID, STICK_MID, CMD_STOP);
      Serial.println(F("[NN] STOP action"));
      break;

    case 3:
      if (!flightArmed) {
        flightArmed = true;
        sendPacket(STICK_MID, STICK_MID, STICK_MID, STICK_MID, CMD_TAKEOFF);
        Serial.println(F("[NN] TAKEOFF action"));
      }
      break;

    case 4:
      captureZero();
      Serial.println(F("[NN] ZERO action"));
      break;

    case 7:
      triggerLocalRecalibration();
      Serial.println(F("[NN] RE-CALIBRATE action (class 7)"));
      break;

    case 1:
      sendLandSequence();
      flightArmed = false;
      Serial.println(F("[NN] LAND action"));
      break;

    default:
      break;
  }
}

void updateNNRecognition(int rawA1, int rawA0)
{
  if (!nnReady || !nnEnabled) return;

  unsigned long now = millis();
  if (now - lastNNMillis < NN_PERIOD_MS) return;
  lastNNMillis = now;

  float x[kNNNumInputs];
  x[0] = ((float)rawA1 - nnScalerMean[0]) / nnScalerScale[0];
  x[1] = ((float)rawA0 - nnScalerMean[1]) / nnScalerScale[1];

  const float inScale = tf.input->params.scale;
  const int inZero = tf.input->params.zero_point;

  for (int i = 0; i < kNNNumInputs; i++) {
    int32_t q = (int32_t) roundf(x[i] / inScale) + inZero;
    if (q < -128) q = -128;
    if (q > 127) q = 127;
    tf.input->data.int8[i] = (int8_t) q;
  }

  if (tf.interpreter->Invoke() != kTfLiteOk) {
    Serial.println(F("[NN] Invoke error"));
    return;
  }

  int pred = 0;
  int8_t best = tf.output->data.int8[0];
  int8_t second = -128;
  for (int i = 1; i < kNNNumOutputs; i++) {
    int8_t v = tf.output->data.int8[i];
    if (v > best) {
      second = best;
      best = v;
      pred = i;
    }
    else if (v > second) {
      second = v;
    }
  }

  int margin = (int)best - (int)second;
  // Class 1 tends to be less separated on real glove data, so use a softer
  // margin threshold to reduce missed detections for this class.
  const int requiredMargin = (pred == 1) ? 4 : NN_MIN_MARGIN_Q;
  if (margin < requiredMargin) {
    nnLastClass = -1;
    nnClassStartMillis = 0;
    nnStablePosition = -1;
    return;
  }

  if (pred != nnLastClass) {
    nnLastClass = pred;
    nnClassStartMillis = now;
  }

  if (pred == 0) {
    if (now - nnClassStartMillis >= NN_HOLD_MS) {
      nnStablePosition = 0;
      nnLastActionClass = -1;  // neutral re-arms any future action class
    }
    return;
  }
  if (nnClassStartMillis == 0) return;
  if (now - nnClassStartMillis < NN_HOLD_MS) return;

  nnStablePosition = pred;
  if (pred == nnLastActionClass) return;
  if (now - lastNNActionMillis < NN_ACTION_COOLDOWN_MS) return;

  Serial.print(F("[NN] Gesture: "));
  Serial.print(nnClassName(pred));
  Serial.print(F(" (cls="));
  Serial.print(pred);
  Serial.print(F(", margin="));
  Serial.print(margin);
  Serial.println(F(")"));

  applyNNAction(pred);
  lastNNActionMillis = now;
  nnLastActionClass = pred;
}

void initNN()
{
  Serial.println(F("[NN] Initializing FCNN (2 -> 40 -> 20 -> 9)..."));

  tf.setNumInputs(kNNNumInputs);
  tf.setNumOutputs(kNNNumOutputs);
  tf.resolver.AddFullyConnected();
  tf.resolver.AddRelu();
  tf.resolver.AddSoftmax();
  tf.resolver.AddQuantize();
  tf.resolver.AddDequantize();

  if (!tf.begin(g_glove_model).isOk()) {
    nnReady = false;
    nnEnabled = false;
    Serial.print(F("[NN] TF begin error: "));
    Serial.println(tf.exception.toString());
    return;
  }

  nnReady = true;
  nnEnabled = true;
  Serial.println(F("[NN] Ready. Actions: 1=nothing 2=stop 3=takeoff 4=zero 6=land 7=recal"));
}
#endif

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup()
{
  Serial.begin(115200);
  // Give USB serial a moment to connect so boot messages are visible
  delay(2000);

  Serial.println(F("=== K417 Arduino WiFi Drone Controller ==="));

  // ── 1. IMU ────────────────────────────────────────────────────────────────
  Serial.print(F("[IMU] Initialising LSM6DSOX... "));
  if (!IMU.begin()) {
    Serial.println(F("FAILED — halting."));
    while (true) { delay(500); }
  }
  Serial.println(F("OK"));

  // ── 2. Haptic Control ────────────────────────────────────────────────────
  pinMode(HAPTIC_OUT_PIN, OUTPUT);
  digitalWrite(HAPTIC_OUT_PIN, LOW);
  pinMode(HAPTIC_CLK_PIN, OUTPUT);
  pinMode(HAPTIC_DATA_PIN, OUTPUT);
  pinMode(HAPTIC_POT_CS, OUTPUT);
  pinMode(HAPTIC_HV_LE, OUTPUT);
  pinMode(HAPTIC_HV_CLR, OUTPUT);

  digitalWrite(HAPTIC_CLK_PIN, LOW);
  digitalWrite(HAPTIC_DATA_PIN, LOW);
  digitalWrite(HAPTIC_POT_CS, HIGH);
  digitalWrite(HAPTIC_HV_LE, HIGH);
  digitalWrite(HAPTIC_HV_CLR, LOW);

  hapticSetPot(hapticPotValue);
  hapticSendToHV2701(hapticHvState);

  Serial.print(F("[HAPTIC] Initialized on pins: POT_CS="));
  Serial.print(HAPTIC_POT_CS);
  Serial.print(F(" DATA="));
  Serial.print(HAPTIC_DATA_PIN);
  Serial.print(F(" CLK="));
  Serial.print(HAPTIC_CLK_PIN);
  Serial.print(F(" HV_LE="));
  Serial.print(HAPTIC_HV_LE);
  Serial.print(F(" OUT="));
  Serial.println(HAPTIC_OUT_PIN);

  // ── 3. WiFi ───────────────────────────────────────────────────────────────
  Serial.print(F("[WiFi] Connecting to "));
  Serial.print(DRONE_SSID);
  Serial.print(F(" ... "));

  WiFi.begin(DRONE_SSID, DRONE_PASSWORD);
  int wifiRetry = 0;
  while (WiFi.status() != WL_CONNECTED && wifiRetry < 30) {
    delay(500);
    Serial.print('.');
    wifiRetry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.print(F("\n[WiFi] Connected. Local IP: "));
    Serial.println(WiFi.localIP());

    droneAddr.fromString(DRONE_IP);
    udp.begin(8802);  // local bind port; keep distinct from PC video socket port
    Serial.println(F("[CTRL] UDP socket open."));
  } else {
    wifiConnected = false;
    Serial.println(F("\n[WiFi] Connection FAILED — OFFLINE MODE (telemetry only)."));
  }

  Serial.println(F("[CALIB] Keep glove STILL — calibrating gyro + flex sensors..."));

  // ── 4. Neural Network ──────────────────────────────────────────────────────
#if ENABLE_GLOVE_NN
  initNN();
#else
  Serial.println(F("[NN] Disabled at compile time (set ENABLE_GLOVE_NN=1 to enable)."));
#endif

  // Initialise timing
  lastImuMicros  = micros();
  lastCtrlMillis = millis();
  lastTelemMillis = millis();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void loop()
{
  // ── IMU sample ─────────────────────────────────────────────────────────────
  float ax_r, ay_r, az_r, gx_r, gy_r, gz_r;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax_r, ay_r, az_r);
    IMU.readGyroscope(gx_r, gy_r, gz_r);

    // Apply axis remapping (copied from GloveController.on_sensor_data in Python):
    //   ax=ay_r, ay=-ax_r, az=az_r, gx=gy_r, gy=-gx_r, gz=gz_r
    float ax = ay_r, ay = -ax_r, az = az_r;
    float gx = gy_r, gy = -gx_r, gz = gz_r;

    // ── Gyro + flex calibration (matches control_video_v6.py semantics) ─────
    if (!gyroCalibrated || !flexCalibrated) {
      if (!gyroCalibrated) {
        gyroSumX += gx; gyroSumY += gy; gyroSumZ += gz;
        gyroCalibCount++;
        if (gyroCalibCount >= GYRO_CALIB_N) {
          // Gyro bias (mean)
          gyroBiasX = gyroSumX / (float)GYRO_CALIB_N;
          gyroBiasY = gyroSumY / (float)GYRO_CALIB_N;
          gyroBiasZ = gyroSumZ / (float)GYRO_CALIB_N;
          gyroCalibrated = true;
        }
      }

      if (!flexCalibrated) {
        // Flex accumulation over FLEX_CALIB_N samples (Python uses 80)
        for (int i = 0; i < 4; i++) {
          float v = (float)analogRead(FLEX_PINS[i]);
          flexSumBuf[i]   += v;
          flexSumSqBuf[i] += v * v;
        }
        flexCalibCount++;

        if (flexCalibCount >= FLEX_CALIB_N) {
          for (int i = 0; i < 4; i++) {
            float n    = (float)FLEX_CALIB_N;
            float mean = flexSumBuf[i] / n;
            float var  = (flexSumSqBuf[i] / n) - (mean * mean);
            flexMean[i] = mean;
            flexStd[i]  = max(sqrtf(var), 5.0f);  // floor at 5 ADC counts
          }
          flexCalibrated = true;
        }
      }

      if (gyroCalibrated && flexCalibrated && !zeroOrientation) {
        // Same moment as Python: only after both gyro + flex are calibrated.
        captureZero();
        zeroOrientation = true;

        if (autoZeroAfterRecalib) {
          // Make post-recalibration zero explicit in logs (O-equivalent action).
          Serial.println(F("[CALIB] Auto-zero (O) applied after re-calibration."));
          autoZeroAfterRecalib = false;
        }

        Serial.print(F("[CALIB] Done.  GyroBias: "));
        Serial.print(gyroBiasX, 3); Serial.print(F(", "));
        Serial.print(gyroBiasY, 3); Serial.print(F(", "));
        Serial.println(gyroBiasZ, 3);
        Serial.print(F("[CALIB] FlexMean: A0="));
        Serial.print(flexMean[0], 1); Serial.print(F("  A1="));
        Serial.print(flexMean[1], 1); Serial.print(F("  A2="));
        Serial.print(flexMean[2], 1); Serial.print(F("  A3="));
        Serial.println(flexMean[3], 1);
        Serial.println(F("[CTRL] Flight control ACTIVE — press T for takeoff."));
      }

      // During calibration keep motors at minimum throttle for safety.
      if (millis() - lastCtrlMillis >= CTRL_INTERVAL_MS) {
        sendPacket(STICK_MID, STICK_MID, STICK_MIN, STICK_MID);
        lastCtrlMillis = millis();
      }

      // Telemetry visibility during calibration (angles not yet meaningful).
      if (millis() - lastTelemMillis >= TELEM_INTERVAL_MS) {
        lastTelemMillis = millis();
        emitTelemetry(yawDeg, pitchDeg, rollDeg, STICK_MIN,
                      analogRead(A0), analogRead(A1), analogRead(A2), analogRead(A3));
      }

      lastImuMicros = micros();
      return;  // skip AHRS update until calibrated
    }

    // ── Apply software gyro bias (Mahony inner bias on top of hardware offset) ─
    gx -= gyroBiasX;
    gy -= gyroBiasY;
    gz -= gyroBiasZ;

    // ── Compute dt ──────────────────────────────────────────────────────────
    unsigned long nowUs = micros();
    float dt = (float)(nowUs - lastImuMicros) * 1e-6f;
    dt = min(dt, 0.05f);   // matches Python: dt=min(now-last, 0.05)
    lastImuMicros = nowUs;

    // ── Run Mahony AHRS ─────────────────────────────────────────────────────
    mahonyUpdate(ax, ay, az, gx, gy, gz, dt);

    // ── Extract relative Euler angles ───────────────────────────────────────
    getRelativeEuler(yawDeg, pitchDeg, rollDeg);
  }

  // ── Flex sensor reads ──────────────────────────────────────────────────────
  int rawThrUp   = analogRead(THR_UP_PIN);
  int rawThrDown = analogRead(THR_DOWN_PIN);
  int rawA0      = analogRead(A0);
  int rawA1      = analogRead(A1);
  int rawA2      = analogRead(A2);
  int rawA3      = analogRead(A3);

#if ENABLE_GLOVE_NN
  updateNNRecognition(rawA1, rawA0);
#endif

  // ── Control packet at fixed rate ───────────────────────────────────────────
  if (gyroCalibrated && (millis() - lastCtrlMillis >= CTRL_INTERVAL_MS)) {
    lastCtrlMillis = millis();

    // Map IMU angles → stick bytes (mirrors Python IMUAxisMapper.compute())
    uint8_t stickYaw      = angleToStick(yawDeg,   YAW_DEADZONE, YAW_SENSITIVITY, YAW_EXPO);
    uint8_t stickPitch    = angleToStick(pitchDeg,  PR_DEADZONE,  PR_SENSITIVITY,  PR_EXPO);
    uint8_t stickRoll     = angleToStick(rollDeg,   PR_DEADZONE,  PR_SENSITIVITY,  PR_EXPO);
    uint8_t stickThrottle = computeThrottle(rawThrUp, rawThrDown);
    uint8_t telemThrottle = stickThrottle;

    // Drone pitch convention check:
    // In the Python code, pitch stick = forward tilt → pitch angle > 0 → STICK_MAX (forward).
    // The raw angleToStick already encodes sign correctly.

    lastStickThrottle = stickThrottle;
    lastStickYaw = stickYaw;

    uint8_t headlessByte = headlessEnabled ? HEADLESS_ON : HEADLESS_OFF;
    if (flipInProgress) {
      if (flipBurstRemaining > 0) {
        stickRoll = flipRoll;
        stickPitch = flipPitch;
        stickThrottle = flipHoldThrottle;
        stickYaw = flipHoldYaw;
        headlessByte |= SOMERSAULT_FLAG;
        flipBurstRemaining--;
      }
      else if (flipRecoverRemaining > 0) {
        stickRoll = STICK_MID;
        stickPitch = STICK_MID;
        stickThrottle = flipHoldThrottle;
        stickYaw = flipHoldYaw;
        flipRecoverRemaining--;
        if (flipRecoverRemaining == 0) {
          flipInProgress = false;
          Serial.println(F("[FLIP] DONE"));
        }
      }
      else {
        flipInProgress = false;
      }
    }

    sendPacket(stickRoll, stickPitch, stickThrottle, stickYaw, CMD_NONE, headlessByte);

    // Update haptic feedback based on continuous control values
    updateHapticFeedback(yawDeg, pitchDeg, rollDeg, stickThrottle);

    // ── Serial telemetry (rate-limited and non-blocking for smooth control) ─
    if (millis() - lastTelemMillis >= TELEM_INTERVAL_MS) {
      lastTelemMillis = millis();
      // Report live throttle command from the glove mapping.
      emitTelemetry(yawDeg, pitchDeg, rollDeg, telemThrottle,
                    rawA0, rawA1, rawA2, rawA3);
    }
  }

  // Update haptic pulse generation
  hapticUpdatePulses();

  // ── Serial command handler (non-blocking, line-based) ────────────────────
  static char cmdBuf[48];
  static int cmdLen = 0;

  while (Serial.available()) {
    char ch = (char)Serial.read();

    if (ch == '\r') continue;

    if (ch == '\n') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleSerialCommandLine(cmdBuf);
        cmdLen = 0;
      }
      continue;
    }

    if (cmdLen < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = ch;
    }
    else {
      // Drop oversized command safely.
      cmdLen = 0;
    }
  }
}
// ─────────────────────────────────────────────────────────────────────────────
//  END OF FILE
// ─────────────────────────────────────────────────────────────────────────────
