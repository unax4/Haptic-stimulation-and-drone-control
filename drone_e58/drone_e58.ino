/**
 * drone_e58.ino
 * -----------------------------------------------------------------------------
 * E58 WIFI CAM Drone - Direct Arduino Nano RP2040 Connect Controller
 * -----------------------------------------------------------------------------
 *
 * What this sketch implements (matching control_video_e58_v8.py non-video logic):
 * - Mahony AHRS on Arduino for yaw/pitch/roll
 * - Flex throttle mapping (A2 up, A3 down)
 * - E58 CAM8 UDP control packets on port 8090:
 *     66 roll pitch throttle yaw cmd chk 99
 * - E58 session handshake on port 8080:
 *     CONNECT    = 42 76
 *     DISCONNECT = 42 77
 * - START burst on control port 8090:
 *     AA 80 80 00 80 00 80 55
 * - Takeoff, land (smart timed ramp), stop, calibrate, headless pulse, flip burst
 * - USB serial telemetry output for monitoring
 * - Optional TinyML glove NN inference + action mapping
 *
 * NOTE:
 * - Video receive/decode is intentionally excluded, per request.
 * - Headless behavior mirrors Python e58 file: sends one-shot CMD 0x10 pulse.
 */

#include <Arduino_LSM6DSOX.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <math.h>
#include <ctype.h>
#include <string.h>

// Optional TinyML gesture recognition.
#define ENABLE_GLOVE_NN 1

#if ENABLE_GLOVE_NN
#include <eloquent_tensorflow_cortexm.h>
#include "neural/glove_fcnn_eloquent_inference/glove_fcnn_40_20_model_data.h"
#endif

// -----------------------------------------------------------------------------
// User configuration
// -----------------------------------------------------------------------------
const char* DRONE_SSID = "WIFI_8K__bcc908";
//const char* DRONE_SSID = "WIFI_8K_Wd02711";
const char* DRONE_PASSWORD = "";  // Usually open AP

const char* DRONE_IP = "192.168.4.153";
const int DRONE_SESSION_PORT = 8080;
const int DRONE_CONTROL_PORT = 8090;

const int CONTROL_HZ = 100;
const int TELEMETRY_HZ = 30;

// -------- Haptic Stimulation Control Pins --------
const int HAPTIC_POT_CS   = 6;   // MAX5413 chip select
const int HAPTIC_DATA_PIN = 5;   // SPI data (MOSI)
const int HAPTIC_CLK_PIN  = 4;   // SPI clock
const int HAPTIC_HV_LE    = 3;   // HV2701 latch enable
const int HAPTIC_HV_CLR   = 2;   // HV2701 clear (active low)
const int HAPTIC_OUT_PIN  = 12;  // Pulse output

// -------- Haptic Configuration --------
const unsigned long HAPTIC_SINGLE_PULSE_MS = 1000;
const int HAPTIC_BURST_COUNT = 5;
const unsigned long HAPTIC_BURST_PULSE_MS = 50;
const unsigned long HAPTIC_BURST_PAUSE_MS = 100;
const int HAPTIC_ACTION_BURST_COUNT = 3;
const float HAPTIC_DEFAULT_FREQ_HZ = 100.0;
const unsigned long HAPTIC_DEFAULT_PW_US = 400;
const unsigned long HAPTIC_DEFAULT_TRAIN_MS = 2000;

const float MAHONY_KP = 3.5f;
const float MAHONY_KI = 0.03f;
const int GYRO_CALIB_N = 200;
const int FLEX_CALIB_N = 80;

const uint8_t STICK_MIN = 40;
const uint8_t STICK_MID = 128;
const uint8_t STICK_MAX = 220;

const float PR_DEADZONE = 10.0f;
const float YAW_DEADZONE = 16.0f;
const float PR_SENSITIVITY = 1.0f;
const float YAW_SENSITIVITY = 1.5f;
const float PR_EXPO = 0.5f;
const float YAW_EXPO = 0.5f;
const float MAX_ANGLE_DEG = 45.0f;

const float FLEX_THRESH_STD_MULTIPLIER = 2.0f;
const float FLEX_NORM_SCALE = 90.0f;
const float THROTTLE_ALPHA = 0.12f;
const float THR_NET_DEADZONE = 0.2f;
const float THR_EXPO = 0.10f;
const float THR_NEUTRAL_SNAP_STICK = 2.0f;

const int THR_UP_PIN = A2;
const int THR_DOWN_PIN = A3;

const int START_BURST_COUNT = 6;
const int START_BURST_DELAY_MS = 30;

const int FLIP_BURST_PACKETS = 20;
const int FLIP_RECOVER_PACKETS = 10;
const float FLIP_POST_HOLD_S = 0.35f;
const uint8_t FLIP_THR_MIN = 165;
const uint8_t FLIP_THR_BURST_BOOST = 28;
const uint8_t FLIP_THR_RECOVER_BOOST = 14;
const uint8_t FLIP_THR_POST_BOOST = 12;

const float LAND_TIMED_RAMP_S = 4.0f;
const int LAND_FINAL_PACKETS = 10;
const int LAND_FINAL_DELAY_MS = 20;

#if ENABLE_GLOVE_NN
const unsigned long NN_PERIOD_MS = 80;
const unsigned long NN_HOLD_MS = 350;
const int NN_MIN_MARGIN_Q = 5;
const unsigned long NN_ACTION_COOLDOWN_MS = 900;
#endif

// -----------------------------------------------------------------------------
// E58 protocol constants
// -----------------------------------------------------------------------------
const uint8_t CMD_NONE = 0x00;
const uint8_t CMD_TAKEOFF = 0x01;
const uint8_t CMD_LAND = 0x02;
const uint8_t CMD_STOP = 0x04;
const uint8_t CMD_CALIBRATE = 0x80;
const uint8_t CMD_HEADLESS_PULSE = 0x10;
const uint8_t CMD_SOMERSAULT_FLAG = 0x08;

const uint8_t CONNECT_PKT[2] = {0x42, 0x76};
const uint8_t DISCONNECT_PKT[2] = {0x42, 0x77};
const uint8_t START_CONTROL_PKT[8] = {0xAA, 0x80, 0x80, 0x00, 0x80, 0x00, 0x80, 0x55};

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------
WiFiUDP udpCtrl;
WiFiUDP udpSession;
IPAddress droneAddr;
bool wifiConnected = false;
bool controlStarted = false;

bool arduinoUdpEnabled = true;
bool flightArmed = false;

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float eIntX = 0.0f, eIntY = 0.0f, eIntZ = 0.0f;

float qRef0 = 1.0f, qRef1 = 0.0f, qRef2 = 0.0f, qRef3 = 0.0f;

bool gyroCalibrated = false;
int gyroCalibCount = 0;
float gyroSumX = 0.0f, gyroSumY = 0.0f, gyroSumZ = 0.0f;
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;

const int FLEX_PINS[4] = {A0, A1, A2, A3};
float flexMean[4] = {512.0f, 512.0f, 512.0f, 512.0f};
float flexStd[4] = {20.0f, 20.0f, 20.0f, 20.0f};
bool flexCalibrated = false;
int flexCalibCount = 0;
float flexSumBuf[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float flexSumSqBuf[4] = {0.0f, 0.0f, 0.0f, 0.0f};

bool zeroOrientation = false;
bool autoZeroAfterRecalib = false;

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
unsigned long hapticActionLockUntilMs = 0;
int hapticMuxCursor = -1;  // Round-robin cursor across [yaw,pitch,roll,throttle]
bool hapticDebugEnabled = true;
bool hapticAnyActiveLast = false;

// -------- Haptic Feedback Mapping (from PDF) --------
// HV2701 position words copied from est_fuante_pruebas.ino
const uint16_t positions[21] = {
  0b0000000000000000,  // M0
  0b0100000000010101,  // M1
  0b0100000000011010,  // M2
  0b0100000000100101,  // M3
  0b0100000000101010,  // M4
  0b0100001000000101,  // M5
  0b0100001000001010,  // M6
  0b0100000001000101,  // M7
  0b0100000001001010,  // M8
  0b0100010000000101,  // M9
  0b0100010000001010,  // M10
  0b1000000010000101,  // M11
  0b1000000010001010,  // M12
  0b1000100000000101,  // M13
  0b1000100000001010,  // M14
  0b1000000100000101,  // M15
  0b1000000100001010,  // M16
  0b1001000000000101,  // M17
  0b1001000000001010,  // M18
  0b1110000000000101,  // M19
  0b1110000000001010   // M20
};

// Position presets corresponding to hand regions
enum HapticPosition {
  HAPTIC_POS_YAW = 4,        // M4: Thumb region (Channel 2)
  HAPTIC_POS_PITCH = 8,      // M8: Index region (Channel 3)
  HAPTIC_POS_ROLL = 12,      // M12: Middle/Ring region (Channel 4)
  HAPTIC_POS_THROTTLE = 20   // M20: Palm region (Channels 10)
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

HapticFeedback hapticYaw = {HAPTIC_POS_YAW, 0.0f, 16, 25, false, 0};
HapticFeedback hapticPitch = {HAPTIC_POS_PITCH, 0.0f, 23, 28, false, 0};
HapticFeedback hapticRoll = {HAPTIC_POS_ROLL, 0.0f, 28, 35, false, 0};
HapticFeedback hapticThrottle = {HAPTIC_POS_THROTTLE, 0.0f, 14, 20, false, 0};

// Haptic feedback update interval (ms)
const unsigned long HAPTIC_FEEDBACK_UPDATE_MS = 10;
unsigned long lastHapticFeedbackMs = 0;

float yawDeg = 0.0f, pitchDeg = 0.0f, rollDeg = 0.0f;
float throttleSmooth = (float)STICK_MID;

unsigned long lastImuMicros = 0;
unsigned long lastCtrlMillis = 0;
unsigned long lastTelemMillis = 0;
const unsigned long CTRL_INTERVAL_MS = 1000UL / CONTROL_HZ;
const unsigned long TELEM_INTERVAL_MS = 1000UL / TELEMETRY_HZ;

uint8_t lastStickThrottle = STICK_MID;
uint8_t lastStickYaw = STICK_MID;

// One-shot command flags (mirror Python DroneState.consume_flags behavior).
bool flagTakeoff = false;
bool flagLand = false;
bool flagStop = false;
bool flagCalibrate = false;
bool flagHeadlessPulse = false;

// Flip state.
bool flipInProgress = false;
int flipBurstRemaining = 0;
int flipRecoverRemaining = 0;
uint8_t flipRoll = STICK_MID;
uint8_t flipPitch = STICK_MID;
uint8_t flipHoldYaw = STICK_MID;
uint8_t flipBurstThrottle = STICK_MID;
uint8_t flipRecoverStartThrottle = STICK_MID;
uint8_t flipRecoverEndThrottle = STICK_MID;
uint8_t flipPostHoldThrottle = STICK_MID;
unsigned long flipPostHoldUntilMs = 0;

#if ENABLE_GLOVE_NN
using Eloquent::CortexM::TensorFlow;
constexpr int kNNTensorArenaSize = 16 * 1024;
constexpr int kNNNumInputs = 2;
constexpr int kNNNumOutputs = 9;
constexpr int kNNNumOps = 10;
TensorFlow<kNNNumOps, kNNTensorArenaSize> tf;


float nnScalerMean[kNNNumInputs] = {436.91032609f, 381.33152174f};
float nnScalerScale[kNNNumInputs] = {62.69738485f, 71.23006118f};

bool nnReady = false;
bool nnEnabled = false;
unsigned long lastNNMillis = 0;
unsigned long lastNNActionMillis = 0;
int nnLastClass = -1;
unsigned long nnClassStartMillis = 0;
int nnStablePosition = -1;
int nnLastActionClass = -1;
bool nnFlipModeEnabled = false;
bool nnFlipTriggerLatched = false;
unsigned long nnFlipModeSinceMillis = 0;
#endif

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

const char* hapticAxisName(int idx) {
  switch (idx) {
    case 0: return "YAW";
    case 1: return "PITCH";
    case 2: return "ROLL";
    case 3: return "THROTTLE";
    default: return "UNKNOWN";
  }
}

const char* hapticPosName(HapticPosition pos) {
  switch (pos) {
    case HAPTIC_POS_YAW: return "M4";
    case HAPTIC_POS_PITCH: return "M8";
    case HAPTIC_POS_ROLL: return "M12";
    case HAPTIC_POS_THROTTLE: return "M20";
    default: return "M?";
  }
}

// -------- Haptic Feedback Triggers (from PDF mapping) --------
void triggerHapticFeedback(HapticFeedback *feedback, int potValue) {
  // Set position on HV2701 switch matrix
  hapticSendToHV2701(positions[feedback->position]);
  // Set potentiometer intensity
  hapticSetPot(potValue);
  // Start train mode for continuous feedback
  hapticStartTrain(HAPTIC_DEFAULT_FREQ_HZ, HAPTIC_DEFAULT_PW_US, HAPTIC_DEFAULT_TRAIN_MS);
  feedback->isActive = true;
  feedback->lastTriggerMs = millis();
}

void triggerHapticAction(HapticPosition position, int potValue) {
  // Set position on HV2701 switch matrix
  hapticSendToHV2701(positions[position]);
  // Set potentiometer intensity
  hapticSetPot(potValue);
  // Burst for action feedback
  hapticStartBurst(HAPTIC_ACTION_BURST_COUNT, HAPTIC_BURST_PULSE_MS, HAPTIC_BURST_PAUSE_MS);
  // Reserve haptic output for action burst before continuous train resumes.
  unsigned long actionMs = (unsigned long)(HAPTIC_ACTION_BURST_COUNT * HAPTIC_BURST_PULSE_MS) +
                           (unsigned long)(max(0, HAPTIC_ACTION_BURST_COUNT - 1) * HAPTIC_BURST_PAUSE_MS);
  hapticActionLockUntilMs = millis() + actionMs;
  if (hapticDebugEnabled) {
    Serial.print(F("[HDBG] BURST pos="));
    Serial.print(hapticPosName(position));
    Serial.print(F(" pot="));
    Serial.print(potValue);
    Serial.print(F(" count="));
    Serial.print(HAPTIC_ACTION_BURST_COUNT);
    Serial.print(F(" on="));
    Serial.print(HAPTIC_BURST_PULSE_MS);
    Serial.print(F("ms off="));
    Serial.print(HAPTIC_BURST_PAUSE_MS);
    Serial.print(F("ms lock="));
    Serial.print(actionMs);
    Serial.println(F("ms"));
  }
}

// Update haptic feedback based on current control values (called from loop)
void updateHapticFeedback(uint8_t stickYaw, uint8_t stickPitch, uint8_t stickRoll, uint8_t stickThrottle) {
  // Skip all feedback while flip is in progress (gesture 7 action)
  if (flipInProgress) return;

  unsigned long now = millis();
  if ((long)(hapticActionLockUntilMs - now) > 0) return;
  if (now - lastHapticFeedbackMs < HAPTIC_FEEDBACK_UPDATE_MS) return;
  lastHapticFeedbackMs = now;

  float yawNorm = (stickYaw > STICK_MID) ?
    (float)(stickYaw - STICK_MID) / (float)(STICK_MAX - STICK_MID) :
    (float)(STICK_MID - stickYaw) / (float)(STICK_MID - STICK_MIN);
  yawNorm = constrain(yawNorm, 0.0f, 1.0f);

  float pitchNorm = (stickPitch > STICK_MID) ?
    (float)(stickPitch - STICK_MID) / (float)(STICK_MAX - STICK_MID) :
    (float)(STICK_MID - stickPitch) / (float)(STICK_MID - STICK_MIN);
  pitchNorm = constrain(pitchNorm, 0.0f, 1.0f);

  float rollNorm = (stickRoll > STICK_MID) ?
    (float)(stickRoll - STICK_MID) / (float)(STICK_MAX - STICK_MID) :
    (float)(STICK_MID - stickRoll) / (float)(STICK_MID - STICK_MIN);
  rollNorm = constrain(rollNorm, 0.0f, 1.0f);

  float throttleNorm = (stickThrottle > STICK_MID) ?
    (float)(stickThrottle - STICK_MID) / (float)(STICK_MAX - STICK_MID) :
    (float)(STICK_MID - stickThrottle) / (float)(STICK_MID - STICK_MIN);
  throttleNorm = constrain(throttleNorm, 0.0f, 1.0f);

  bool yawActive = yawNorm > 0.0f;
  bool pitchActive = pitchNorm > 0.0f;
  bool rollActive = rollNorm > 0.0f;
  bool throttleActive = throttleNorm > 0.0f;

  if (!yawActive && !pitchActive && !rollActive && !throttleActive) {
    hapticStopPulses();
    hapticYaw.isActive = false;
    hapticPitch.isActive = false;
    hapticRoll.isActive = false;
    hapticThrottle.isActive = false;
    hapticMuxCursor = -1;
    if (hapticDebugEnabled && hapticAnyActiveLast) {
      Serial.println(F("[HDBG] TRAIN idle (all controls neutral)"));
    }
    hapticAnyActiveLast = false;
    return;
  }

  int yawPot = hapticYaw.potMax - (int)(yawNorm * (hapticYaw.potMax - hapticYaw.potMin));
  int pitchPot = hapticPitch.potMax - (int)(pitchNorm * (hapticPitch.potMax - hapticPitch.potMin));
  int rollPot = hapticRoll.potMax - (int)(rollNorm * (hapticRoll.potMax - hapticRoll.potMin));
  int throttlePot = hapticThrottle.potMax - (int)(throttleNorm * (hapticThrottle.potMax - hapticThrottle.potMin));

  // Time-division multiplexing across active channels: one channel per update.
  bool chanActive[4] = {yawActive, pitchActive, rollActive, throttleActive};
  int chanPot[4] = {yawPot, pitchPot, rollPot, throttlePot};
  HapticPosition chanPos[4] = {hapticYaw.position, hapticPitch.position, hapticRoll.position, hapticThrottle.position};

  int chosen = -1;
  for (int step = 1; step <= 4; step++) {
    int idx = (hapticMuxCursor + step) % 4;
    if (chanActive[idx]) {
      chosen = idx;
      break;
    }
  }
  if (chosen < 0) return;
  hapticMuxCursor = chosen;

  hapticSendToHV2701(positions[(int)chanPos[chosen]]);
  hapticSetPot((byte)constrain(chanPot[chosen], 0, 255));

  // Keep train continuous while off-neutral and stop immediately when neutral.
  if (hapticPulseMode != HPM_TRAIN) {
    hapticStartTrain(HAPTIC_DEFAULT_FREQ_HZ, HAPTIC_DEFAULT_PW_US, 60000UL);
  } else {
    haptic_train_start_ms = now;
    haptic_train_duration_ms_running = 60000UL;
  }

  hapticYaw.isActive = yawActive;
  hapticPitch.isActive = pitchActive;
  hapticRoll.isActive = rollActive;
  hapticThrottle.isActive = throttleActive;

  hapticYaw.directionSign = (stickYaw >= STICK_MID) ? 1.0f : -1.0f;
  hapticPitch.directionSign = (stickPitch >= STICK_MID) ? 1.0f : -1.0f;
  hapticRoll.directionSign = (stickRoll >= STICK_MID) ? 1.0f : -1.0f;
  hapticThrottle.directionSign = (stickThrottle >= STICK_MID) ? 1.0f : -1.0f;

  hapticYaw.lastTriggerMs = now;
  hapticPitch.lastTriggerMs = now;
  hapticRoll.lastTriggerMs = now;
  hapticThrottle.lastTriggerMs = now;
  hapticAnyActiveLast = true;

  if (hapticDebugEnabled) {
    Serial.print(F("[HDBG] TRAIN axis="));
    Serial.print(hapticAxisName(chosen));
    Serial.print(F(" pos="));
    Serial.print(hapticPosName(chanPos[chosen]));
    Serial.print(F(" pot="));
    Serial.print(chanPot[chosen]);
    Serial.print(F(" active=["));
    if (yawActive) Serial.print(F("Y"));
    if (pitchActive) Serial.print(F("P"));
    if (rollActive) Serial.print(F("R"));
    if (throttleActive) Serial.print(F("T"));
    Serial.println(F("]"));
  }
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

// -----------------------------------------------------------------------------
// Mahony AHRS
// -----------------------------------------------------------------------------
void captureZero() {
  qRef0 = q0;
  qRef1 = -q1;
  qRef2 = -q2;
  qRef3 = -q3;
  Serial.println(F("[AHRS] Zero orientation captured."));
}

void mahonyUpdate(float ax, float ay, float az,
                  float gx, float gy, float gz,
                  float dt) {
  float gxR = gx * (float)(M_PI / 180.0);
  float gyR = gy * (float)(M_PI / 180.0);
  float gzR = gz * (float)(M_PI / 180.0);

  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm < 1e-6f) return;
  ax /= norm;
  ay /= norm;
  az /= norm;

  float vx = 2.0f * (q1 * q3 - q0 * q2);
  float vy = 2.0f * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  float ex = ay * vz - az * vy;
  float ey = az * vx - ax * vz;
  float ez = ax * vy - ay * vx;

  eIntX += ex * MAHONY_KI * dt;
  eIntY += ey * MAHONY_KI * dt;
  eIntZ += ez * MAHONY_KI * dt;

  gxR += MAHONY_KP * ex + eIntX;
  gyR += MAHONY_KP * ey + eIntY;
  gzR += MAHONY_KP * ez + eIntZ;

  float hw = 0.5f * dt;
  float qa = q0, qb = q1, qc = q2;
  q0 += (-qb * gxR - qc * gyR - q3 * gzR) * hw;
  q1 += (qa * gxR + qc * gzR - q3 * gyR) * hw;
  q2 += (qa * gyR - qb * gzR + q3 * gxR) * hw;
  q3 += (qa * gzR + qb * gyR - qc * gxR) * hw;

  norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= norm;
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;
}

void getRelativeEuler(float &yaw_out, float &pitch_out, float &roll_out) {
  float w = qRef0 * q0 - qRef1 * q1 - qRef2 * q2 - qRef3 * q3;
  float x = qRef0 * q1 + qRef1 * q0 + qRef2 * q3 - qRef3 * q2;
  float y = qRef0 * q2 - qRef1 * q3 + qRef2 * q0 + qRef3 * q1;
  float z = qRef0 * q3 + qRef1 * q2 - qRef2 * q1 + qRef3 * q0;

  roll_out = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * (float)(180.0 / M_PI);

  float sinp = 2.0f * (w * y - z * x);
  sinp = constrain(sinp, -1.0f, 1.0f);
  pitch_out = asinf(sinp) * (float)(180.0 / M_PI);

  yaw_out = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * (float)(180.0 / M_PI);
}

// -----------------------------------------------------------------------------
// Mapping helpers
// -----------------------------------------------------------------------------
uint8_t angleToStick(float angle, float deadzone, float sensitivity, float expo) {
  float sign = (angle >= 0.0f) ? 1.0f : -1.0f;
  float mag = fabsf(angle);
  if (mag < deadzone) return STICK_MID;

  float norm = (mag - deadzone) / (MAX_ANGLE_DEG - deadzone);
  norm = min(norm * sensitivity, 1.0f);
  float curved = norm * (1.0f - expo) + norm * norm * norm * expo;
  curved = constrain(curved, 0.0f, 1.0f);

  float raw = STICK_MID + sign * curved * (float)(STICK_MAX - STICK_MID);
  return (uint8_t)constrain((int)raw, STICK_MIN, STICK_MAX);
}

float flexDeflection(int raw, int idx) {
  float delta = (float)raw - flexMean[idx];
  float thresh = FLEX_THRESH_STD_MULTIPLIER * flexStd[idx];
  if (fabsf(delta) < thresh) return 0.0f;
  float signedExcess = delta - (delta >= 0.0f ? thresh : -thresh);
  return constrain(signedExcess / FLEX_NORM_SCALE, -1.0f, 1.0f);
}

uint8_t computeThrottle(int rawA2, int rawA3) {
  if (!flexCalibrated) return (uint8_t)throttleSmooth;

  float d2 = flexDeflection(rawA2, 2);
  float d3 = flexDeflection(rawA3, 3);
  float net = constrain(d2 - d3, -1.0f, 1.0f);

  float s = (net >= 0.0f) ? 1.0f : -1.0f;
  float m = fabsf(net);
  float mapped = 0.0f;
  if (m > THR_NET_DEADZONE) {
    mapped = (m - THR_NET_DEADZONE) / (1.0f - THR_NET_DEADZONE);
  }

  float ct = mapped * (1.0f - THR_EXPO) + mapped * mapped * mapped * THR_EXPO;
  float raw = STICK_MID + s * ct * (float)(STICK_MAX - STICK_MID);
  raw = constrain(raw, (float)STICK_MIN, (float)STICK_MAX);

  throttleSmooth += (raw - throttleSmooth) * THROTTLE_ALPHA;
  if (mapped == 0.0f && fabsf(throttleSmooth - (float)STICK_MID) <= THR_NEUTRAL_SNAP_STICK) {
    throttleSmooth = (float)STICK_MID;
  }

  return (uint8_t)constrain((int)throttleSmooth, STICK_MIN, STICK_MAX);
}

// -----------------------------------------------------------------------------
// E58 UDP helpers
// -----------------------------------------------------------------------------
void buildCam8Packet(uint8_t *out,
                     uint8_t roll,
                     uint8_t pitch,
                     uint8_t throttle,
                     uint8_t yaw,
                     uint8_t cmd,
                     bool somersaultFlag = false) {
  if (somersaultFlag) {
    cmd |= CMD_SOMERSAULT_FLAG;
  }
  uint8_t chk = roll ^ pitch ^ throttle ^ yaw ^ cmd;
  out[0] = 0x66;
  out[1] = roll;
  out[2] = pitch;
  out[3] = throttle;
  out[4] = yaw;
  out[5] = cmd;
  out[6] = chk;
  out[7] = 0x99;
}

void sendControlPacket(uint8_t roll,
                       uint8_t pitch,
                       uint8_t throttle,
                       uint8_t yaw,
                       uint8_t cmd = CMD_NONE,
                       bool somersaultFlag = false) {
  if (!wifiConnected || !arduinoUdpEnabled) return;

  uint8_t pkt[8];
  buildCam8Packet(pkt, roll, pitch, throttle, yaw, cmd, somersaultFlag);
  udpCtrl.beginPacket(droneAddr, DRONE_CONTROL_PORT);
  udpCtrl.write(pkt, sizeof(pkt));
  udpCtrl.endPacket();
}

void sendConnectSession() {
  if (!wifiConnected) return;
  udpSession.beginPacket(droneAddr, DRONE_SESSION_PORT);
  udpSession.write(CONNECT_PKT, sizeof(CONNECT_PKT));
  udpSession.endPacket();
  Serial.println(F("[WIFI_CAM] CONNECT sent"));
}

void sendDisconnectSession() {
  if (!wifiConnected) return;
  udpSession.beginPacket(droneAddr, DRONE_SESSION_PORT);
  udpSession.write(DISCONNECT_PKT, sizeof(DISCONNECT_PKT));
  udpSession.endPacket();
  Serial.println(F("[WIFI_CAM] DISCONNECT sent"));
}

void sendStartControlBurst(int burst = START_BURST_COUNT) {
  if (!wifiConnected || !arduinoUdpEnabled) return;
  if (burst < 1) burst = 1;
  for (int i = 0; i < burst; i++) {
    udpCtrl.beginPacket(droneAddr, DRONE_CONTROL_PORT);
    udpCtrl.write(START_CONTROL_PKT, sizeof(START_CONTROL_PKT));
    udpCtrl.endPacket();
    delay(START_BURST_DELAY_MS);
  }
  controlStarted = true;
  Serial.print(F("[WIFI_CAM] START burst x"));
  Serial.println(burst);
}

void sendCalibratePulse() {
  sendControlPacket(STICK_MID, STICK_MID, STICK_MID, STICK_MID, CMD_CALIBRATE);
}

void sendHeadlessPulse() {
  // Mirrors Python: one-shot command pulse (not persistent headless byte).
  sendControlPacket(lastStickYaw, STICK_MID, lastStickThrottle, STICK_MID, CMD_HEADLESS_PULSE);
}

// -----------------------------------------------------------------------------
// High-level actions
// -----------------------------------------------------------------------------
bool decodeFlipDirection(const char *direction, uint8_t &outRoll, uint8_t &outPitch) {
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

void startFlip(const char *direction, uint8_t throttleSnapshot, uint8_t yawSnapshot) {
  if (!flightArmed) {
    Serial.println(F("[FLIP] Ignored: drone not armed."));
    return;
  }
  if (flipInProgress) {
    Serial.println(F("[FLIP] Ignored: already in progress."));
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
  flipHoldYaw = yawSnapshot;
  uint8_t baseThrottle = (uint8_t)constrain(max((int)throttleSnapshot, (int)FLIP_THR_MIN), (int)STICK_MIN, (int)STICK_MAX);
  flipBurstThrottle = (uint8_t)constrain((int)baseThrottle + (int)FLIP_THR_BURST_BOOST, (int)STICK_MIN, (int)STICK_MAX);
  flipRecoverStartThrottle = (uint8_t)constrain((int)baseThrottle + (int)FLIP_THR_RECOVER_BOOST, (int)STICK_MIN, (int)STICK_MAX);
  flipRecoverEndThrottle = (uint8_t)constrain((int)baseThrottle + (int)FLIP_THR_POST_BOOST, (int)STICK_MIN, (int)STICK_MAX);
  flipPostHoldThrottle = flipRecoverEndThrottle;
  flipPostHoldUntilMs = 0;
  flipBurstRemaining = FLIP_BURST_PACKETS;
  flipRecoverRemaining = FLIP_RECOVER_PACKETS;
  flipInProgress = true;

  Serial.print(F("[FLIP] START "));
  Serial.println(direction);
}

void sendLandPacket(uint8_t yawSnapshot) {
  if (!wifiConnected || !arduinoUdpEnabled) {
    flightArmed = false;
    return;
  }

  sendControlPacket(STICK_MID, STICK_MID, STICK_MIN, yawSnapshot, CMD_LAND);
  flightArmed = false;
  flipInProgress = false;
  Serial.println(F("[LAND] Land packet sent"));
}

void triggerLocalRecalibration() {
  flightArmed = false;
  flipInProgress = false;

  gyroCalibrated = false;
  flexCalibrated = false;
  zeroOrientation = false;
  gyroCalibCount = 0;
  flexCalibCount = 0;

  gyroSumX = gyroSumY = gyroSumZ = 0.0f;
  memset(flexSumBuf, 0, sizeof(flexSumBuf));
  memset(flexSumSqBuf, 0, sizeof(flexSumSqBuf));

  eIntX = eIntY = eIntZ = 0.0f;
  q0 = 1.0f;
  q1 = q2 = q3 = 0.0f;
  throttleSmooth = (float)STICK_MID;
  autoZeroAfterRecalib = true;

#if ENABLE_GLOVE_NN
  nnStablePosition = -1;
  nnLastClass = -1;
  nnClassStartMillis = 0;
  nnLastActionClass = -1;
  nnFlipModeEnabled = false;
  nnFlipTriggerLatched = false;
  nnFlipModeSinceMillis = 0;
#endif

  Serial.println(F("[CALIB] Re-calibrating, keep still..."));
}

// -----------------------------------------------------------------------------
// Telemetry
// -----------------------------------------------------------------------------
void emitTelemetry(float yaw, float pitch, float roll,
                   uint8_t stickRoll,
                   uint8_t stickPitch,
                   uint8_t stickThrottle,
                   uint8_t stickYaw,
                   uint8_t cmd,
                   int rawA0, int rawA1, int rawA2, int rawA3) {
  Serial.print(F("Y:"));
  Serial.print(yaw, 1);
  Serial.print(F(" P:"));
  Serial.print(pitch, 1);
  Serial.print(F(" R:"));
  Serial.print(roll, 1);
  Serial.print(F(" SY:"));
  Serial.print((int)stickYaw);
  Serial.print(F(" SP:"));
  Serial.print((int)stickPitch);
  Serial.print(F(" SR:"));
  Serial.print((int)stickRoll);

  Serial.print(F(" ST:"));
  Serial.print((int)stickThrottle);

  Serial.print(F(" CMD:"));
  Serial.print((int)cmd);
  Serial.print(F(" A0:"));
  Serial.print(rawA0);
  Serial.print(F(" A1:"));
  Serial.print(rawA1);
  Serial.print(F(" A2:"));
  Serial.print(rawA2);
  Serial.print(F(" A3:"));
  Serial.print(rawA3);
#if ENABLE_GLOVE_NN
  Serial.print(F(" POS:"));
  Serial.println(nnStablePosition);
#else
  Serial.print(F(" POS:"));
  Serial.println(-1);
#endif
}

// -----------------------------------------------------------------------------
// TinyML NN
// -----------------------------------------------------------------------------
#if ENABLE_GLOVE_NN
const char* nnClassName(int cls) {
  switch (cls) {
    case 0: return "neutral";
    case 1: return "stop";
    case 2: return "land";
    case 3: return "takeoff";
    case 4: return "zero";
    case 5: return "class5";
    case 6: return "class6";
    case 7: return "flip_armed";
    case 8: return "class8";
    default: return "unknown";
  }
}

void applyNNAction(int cls) {
  switch (cls) {
    case 1:
      flagStop = true;
      Serial.println(F("[NN] STOP action"));
      break;

    case 3:
      flagTakeoff = true;
      Serial.println(F("[NN] TAKEOFF action"));
      break;

    case 4:
      captureZero();
      triggerHapticAction(HAPTIC_POS_ROLL, 30);  // Zero feedback on M12 at pot 30
      Serial.println(F("[HAPTIC] Zero feedback triggered"));
      Serial.println(F("[NN] ZERO action"));
      break;

    case 7:
      nnFlipModeEnabled = true;
      nnFlipTriggerLatched = false;
      nnFlipModeSinceMillis = millis();
      triggerHapticAction(HAPTIC_POS_THROTTLE, 18);  // 3 bursts on M20 (palm) to indicate flip mode armed
      Serial.println(F("[NN] FLIP ARMED (one-shot)"));
      break;

    case 2:
      flagLand = true;
      Serial.println(F("[NN] LAND action"));
      break;

    default:
      break;
  }
}

void updateNNRecognition(int rawA1, int rawA0) {
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
    int32_t q = (int32_t)roundf(x[i] / inScale) + inZero;
    if (q < -128) q = -128;
    if (q > 127) q = 127;
    tf.input->data.int8[i] = (int8_t)q;
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
  const int requiredMargin = (pred == 2) ? 4 : NN_MIN_MARGIN_Q;
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
      nnLastActionClass = -1;
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

void initNN() {
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
  Serial.println(F("[NN] Ready. Actions: 1=stop 2=land 3=takeoff 4=zero 7=flip_armed(one-shot)"));
}
#endif

// -----------------------------------------------------------------------------
// Serial commands
// -----------------------------------------------------------------------------
void handleSerialCommandLine(const char *cmdLine) {
  if (!cmdLine || cmdLine[0] == '\0') return;

  char cmd[64];
  size_t n = strnlen(cmdLine, sizeof(cmd) - 1);
  memcpy(cmd, cmdLine, n);
  cmd[n] = '\0';

  for (size_t i = 0; cmd[i] != '\0'; i++) {
    cmd[i] = (char)toupper((unsigned char)cmd[i]);
  }

  if (strcmp(cmd, "T") == 0 || strcmp(cmd, "TAKEOFF") == 0) {
    flagTakeoff = true;
    return;
  }
  if (strcmp(cmd, "L") == 0 || strcmp(cmd, "LAND") == 0) {
    flagLand = true;
    return;
  }
  if (strcmp(cmd, "X") == 0 || strcmp(cmd, "STOP") == 0) {
    flagStop = true;
    return;
  }
  if (strcmp(cmd, "C") == 0 || strcmp(cmd, "CAL") == 0 || strcmp(cmd, "CALIBRATE") == 0) {
    flagCalibrate = true;
    return;
  }
  if (strcmp(cmd, "H") == 0 || strcmp(cmd, "HEADLESS") == 0) {
    flagHeadlessPulse = true;
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

  if (strcmp(cmd, "CONNECT") == 0) {
    sendConnectSession();
    return;
  }
  if (strcmp(cmd, "START") == 0) {
    sendConnectSession();
    sendStartControlBurst(START_BURST_COUNT);
    return;
  }
  if (strcmp(cmd, "D") == 0 || strcmp(cmd, "DISCONNECT") == 0) {
    sendDisconnectSession();
    controlStarted = false;
    return;
  }

  if (strcmp(cmd, "P") == 0 || strcmp(cmd, "PYUDP") == 0) {
    arduinoUdpEnabled = false;
    flightArmed = false;
    flipInProgress = false;
    flipPostHoldUntilMs = 0;
    Serial.println(F("[MODE] PYTHON_UDP ON (Arduino UDP paused)"));
    return;
  }
  if (strcmp(cmd, "A") == 0 || strcmp(cmd, "ARDUDP") == 0) {
    arduinoUdpEnabled = true;
    Serial.println(F("[MODE] ARDUINO_UDP ON"));
    return;
  }

  if (strncmp(cmd, "FLIP:", 5) == 0) {
    startFlip(cmd + 5, lastStickThrottle, lastStickYaw);
    return;
  }
  if (strcmp(cmd, "FF") == 0) {
    startFlip("FORWARD", lastStickThrottle, lastStickYaw);
    return;
  }
  if (strcmp(cmd, "FB") == 0) {
    startFlip("BACKWARD", lastStickThrottle, lastStickYaw);
    return;
  }
  if (strcmp(cmd, "FL") == 0) {
    startFlip("LEFT", lastStickThrottle, lastStickYaw);
    return;
  }
  if (strcmp(cmd, "FR") == 0) {
    startFlip("RIGHT", lastStickThrottle, lastStickYaw);
    return;
  }

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

  if (strcmp(cmd, "HDBG") == 0) {
    hapticDebugEnabled = !hapticDebugEnabled;
    Serial.print(F("[HAPTIC] Debug "));
    Serial.println(hapticDebugEnabled ? F("ON") : F("OFF"));
    return;
  }
  if (strcmp(cmd, "HDBGON") == 0 || strcmp(cmd, "HDBG1") == 0) {
    hapticDebugEnabled = true;
    Serial.println(F("[HAPTIC] Debug ON"));
    return;
  }
  if (strcmp(cmd, "HDBGOFF") == 0 || strcmp(cmd, "HDBG0") == 0) {
    hapticDebugEnabled = false;
    Serial.println(F("[HAPTIC] Debug OFF"));
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
    Serial.println(F("Commands: CONNECT START D T L X C H O R FLIP:<FORWARD|BACKWARD|LEFT|RIGHT> P A N ?"));
    Serial.println(F("Haptic: Pxxx HS HB HT HFxx HWxx HDxx HSWx HDBG/HDBGON/HDBGOFF ?"));
    return;
  }

  Serial.print(F("[CMD] Unknown: "));
  Serial.println(cmd);
}

// -----------------------------------------------------------------------------
// setup / loop
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println(F("=== E58 Arduino WiFi Drone Controller ==="));

  Serial.print(F("[IMU] Initializing LSM6DSOX... "));
  if (!IMU.begin()) {
    Serial.println(F("FAILED - halting."));
    while (true) {
      delay(500);
    }
  }
  Serial.println(F("OK"));

  // -------- Initialize Haptic Control Pins --------
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
    udpCtrl.begin(8091);
    udpSession.begin(8092);

    sendConnectSession();
    sendStartControlBurst(START_BURST_COUNT);
  }
  else {
    wifiConnected = false;
    Serial.println(F("\n[WiFi] Connection FAILED - telemetry-only mode."));
  }

  Serial.println(F("[CALIB] Keep glove still - calibrating gyro + flex sensors..."));

#if ENABLE_GLOVE_NN
  initNN();
#else
  Serial.println(F("[NN] Disabled at compile time (set ENABLE_GLOVE_NN=1)."));
#endif

  lastImuMicros = micros();
  lastCtrlMillis = millis();
  lastTelemMillis = millis();
}

void loop() {
  float ax_r, ay_r, az_r, gx_r, gy_r, gz_r;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax_r, ay_r, az_r);
    IMU.readGyroscope(gx_r, gy_r, gz_r);

    // Match Python axis remap in GloveController.on_sensor_data().
    float ax = ay_r;
    float ay = -ax_r;
    float az = az_r;
    float gx = gy_r;
    float gy = -gx_r;
    float gz = gz_r;

    if (!gyroCalibrated || !flexCalibrated) {
      if (!gyroCalibrated) {
        gyroSumX += gx;
        gyroSumY += gy;
        gyroSumZ += gz;
        gyroCalibCount++;
        if (gyroCalibCount >= GYRO_CALIB_N) {
          gyroBiasX = gyroSumX / (float)GYRO_CALIB_N;
          gyroBiasY = gyroSumY / (float)GYRO_CALIB_N;
          gyroBiasZ = gyroSumZ / (float)GYRO_CALIB_N;
          gyroCalibrated = true;
        }
      }

      if (!flexCalibrated) {
        for (int i = 0; i < 4; i++) {
          float v = (float)analogRead(FLEX_PINS[i]);
          flexSumBuf[i] += v;
          flexSumSqBuf[i] += v * v;
        }
        flexCalibCount++;
        if (flexCalibCount >= FLEX_CALIB_N) {
          for (int i = 0; i < 4; i++) {
            float n = (float)FLEX_CALIB_N;
            float mean = flexSumBuf[i] / n;
            float var = (flexSumSqBuf[i] / n) - (mean * mean);
            flexMean[i] = mean;
            flexStd[i] = max(sqrtf(var), 5.0f);
          }
          flexCalibrated = true;
        }
      }

      if (gyroCalibrated && flexCalibrated && !zeroOrientation) {
        captureZero();
        zeroOrientation = true;
        if (autoZeroAfterRecalib) {
          Serial.println(F("[CALIB] Auto-zero applied."));
          autoZeroAfterRecalib = false;
        }
        Serial.println(F("[CALIB] Done. Flight control active."));
      }

      if (millis() - lastCtrlMillis >= CTRL_INTERVAL_MS) {
        sendControlPacket(STICK_MID, STICK_MID, STICK_MIN, STICK_MID, CMD_NONE);
        lastCtrlMillis = millis();
      }

      if (millis() - lastTelemMillis >= TELEM_INTERVAL_MS) {
        lastTelemMillis = millis();
        emitTelemetry(yawDeg, pitchDeg, rollDeg,
                STICK_MID, STICK_MID, STICK_MIN, STICK_MID, CMD_NONE,
                analogRead(A0), analogRead(A1), analogRead(A2), analogRead(A3));
      }

      lastImuMicros = micros();
      return;
    }

    gx -= gyroBiasX;
    gy -= gyroBiasY;
    gz -= gyroBiasZ;

    unsigned long nowUs = micros();
    float dt = (float)(nowUs - lastImuMicros) * 1e-6f;
    dt = min(dt, 0.05f);
    lastImuMicros = nowUs;

    mahonyUpdate(ax, ay, az, gx, gy, gz, dt);
    getRelativeEuler(yawDeg, pitchDeg, rollDeg);
  }

  int rawA0 = analogRead(A0);
  int rawA1 = analogRead(A1);
  int rawA2 = analogRead(A2);
  int rawA3 = analogRead(A3);

#if ENABLE_GLOVE_NN
  updateNNRecognition(rawA1, rawA0);
#endif

  if (gyroCalibrated && (millis() - lastCtrlMillis >= CTRL_INTERVAL_MS)) {
    lastCtrlMillis = millis();

    // Python E58 mapper inverts yaw sign before angle->stick conversion.
    uint8_t stickYaw = angleToStick(-yawDeg, YAW_DEADZONE, YAW_SENSITIVITY, YAW_EXPO);
    uint8_t stickPitch = angleToStick(pitchDeg, PR_DEADZONE, PR_SENSITIVITY, PR_EXPO);
    uint8_t stickRoll = angleToStick(rollDeg, PR_DEADZONE, PR_SENSITIVITY, PR_EXPO);
    uint8_t stickThrottle = computeThrottle(rawA2, rawA3);

    lastStickYaw = stickYaw;
    lastStickThrottle = stickThrottle;

    uint8_t cmd = CMD_NONE;
    bool sentFlipBurstPacket = false;
    bool sentLandPacket = false;

    if (flagTakeoff) {
      cmd = CMD_TAKEOFF;
      flagTakeoff = false;
      flightArmed = true;
      // Haptic feedback: Takeoff on thumb region (M4) with pot 20
      triggerHapticAction(HAPTIC_POS_YAW, 20);
      Serial.println(F("[HAPTIC] Takeoff feedback triggered"));
      if (!controlStarted) {
        sendConnectSession();
        sendStartControlBurst(START_BURST_COUNT);
      }
    }
    else if (flagStop) {
      cmd = CMD_STOP;
      flagStop = false;
      flightArmed = false;
      flipInProgress = false;
      flipPostHoldUntilMs = 0;
      // Haptic feedback: Stop on palm region (M20) with pot 18
      triggerHapticAction(HAPTIC_POS_THROTTLE, 18);
      Serial.println(F("[HAPTIC] Stop feedback triggered"));
    }
    else if (flagCalibrate) {
      cmd = CMD_CALIBRATE;
      flagCalibrate = false;
      stickRoll = STICK_MID;
      stickPitch = STICK_MID;
      stickThrottle = STICK_MID;
      stickYaw = STICK_MID;
    }
    else if (flagHeadlessPulse) {
      cmd = CMD_HEADLESS_PULSE;
      flagHeadlessPulse = false;
    }

    if (flagLand) {
      flagLand = false;
      flipPostHoldUntilMs = 0;
      // Haptic feedback: Landing on index region (M8) with pot 25
      triggerHapticAction(HAPTIC_POS_PITCH, 25);
      Serial.println(F("[HAPTIC] Landing feedback triggered"));
      sendLandPacket(stickYaw);
      sentLandPacket = true;
      cmd = CMD_LAND;
    }

    if (flipInProgress) {
      if (flipBurstRemaining > 0) {
        // Haptic feedback: Flip mode on palm region (M20, B3) with pot 18
        if (flipBurstRemaining == FLIP_BURST_PACKETS) {
          triggerHapticAction(HAPTIC_POS_THROTTLE, 18);
          Serial.println(F("[HAPTIC] Flip mode feedback triggered"));
        }
        stickRoll = flipRoll;
        stickPitch = flipPitch;
        stickThrottle = flipBurstThrottle;
        stickYaw = flipHoldYaw;
        cmd = CMD_NONE;
        sendControlPacket(stickRoll, stickPitch, stickThrottle, stickYaw, cmd, true);
        sentFlipBurstPacket = true;
        flipBurstRemaining--;
      }
      else if (flipRecoverRemaining > 0) {
        stickRoll = STICK_MID;
        stickPitch = STICK_MID;
        int recoverStep = FLIP_RECOVER_PACKETS - flipRecoverRemaining;
        int recoverDen = max(1, FLIP_RECOVER_PACKETS - 1);
        int thrDelta = (int)flipRecoverStartThrottle - (int)flipRecoverEndThrottle;
        int recoverThrottle = (int)flipRecoverStartThrottle - ((thrDelta * recoverStep) / recoverDen);
        stickThrottle = (uint8_t)constrain(recoverThrottle, (int)STICK_MIN, (int)STICK_MAX);
        stickYaw = flipHoldYaw;
        cmd = CMD_NONE;
        flipRecoverRemaining--;
        if (flipRecoverRemaining == 0) {
          flipInProgress = false;
          flipPostHoldUntilMs = millis() + (unsigned long)(FLIP_POST_HOLD_S * 1000.0f);
          Serial.println(F("[FLIP] DONE"));
        }
      }
      else {
        flipInProgress = false;
      }
    }

    if (!flipInProgress && flightArmed && millis() < flipPostHoldUntilMs) {
      stickThrottle = max(stickThrottle, flipPostHoldThrottle);
    }

#if ENABLE_GLOVE_NN
    if (nnFlipModeEnabled && nnFlipModeSinceMillis && (millis() - nnFlipModeSinceMillis) > 3000UL) {
      nnFlipModeEnabled = false;
      nnFlipTriggerLatched = false;
      nnFlipModeSinceMillis = 0;
      Serial.println(F("[NN] FLIP arm timeout -> canceled"));
    }

    if (nnFlipModeEnabled && flightArmed && !flipInProgress) {
      if (!nnFlipTriggerLatched) {
        if (stickRoll == STICK_MAX) {
          startFlip("RIGHT", stickThrottle, stickYaw);
          nnFlipTriggerLatched = true;
          nnFlipModeEnabled = false;
          nnFlipModeSinceMillis = 0;
        }
        else if (stickRoll == STICK_MIN) {
          startFlip("LEFT", stickThrottle, stickYaw);
          nnFlipTriggerLatched = true;
          nnFlipModeEnabled = false;
          nnFlipModeSinceMillis = 0;
        }
        else if (stickPitch == STICK_MAX) {
          startFlip("FORWARD", stickThrottle, stickYaw);
          nnFlipTriggerLatched = true;
          nnFlipModeEnabled = false;
          nnFlipModeSinceMillis = 0;
        }
        else if (stickPitch == STICK_MIN) {
          startFlip("BACKWARD", stickThrottle, stickYaw);
          nnFlipTriggerLatched = true;
          nnFlipModeEnabled = false;
          nnFlipModeSinceMillis = 0;
        }
      }

      if (nnFlipTriggerLatched &&
          abs((int)stickRoll - (int)STICK_MID) <= 8 &&
          abs((int)stickPitch - (int)STICK_MID) <= 8) {
        nnFlipTriggerLatched = false;
      }
    }
#endif

    if (!sentFlipBurstPacket && !sentLandPacket) {
      sendControlPacket(stickRoll, stickPitch, stickThrottle, stickYaw, cmd);
    }

    // Update haptic feedback based on continuous control values
    updateHapticFeedback(stickYaw, stickPitch, stickRoll, stickThrottle);

    if (millis() - lastTelemMillis >= TELEM_INTERVAL_MS) {
      lastTelemMillis = millis();
      emitTelemetry(yawDeg, pitchDeg, rollDeg,
            stickRoll, stickPitch, stickThrottle, stickYaw, cmd,
            rawA0, rawA1, rawA2, rawA3);
    }
  }

  // Update haptic pulse generation
  hapticUpdatePulses();

  static char cmdBuf[64];
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
      cmdLen = 0;
    }
  }
}
