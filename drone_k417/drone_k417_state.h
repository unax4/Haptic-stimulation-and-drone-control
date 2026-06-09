#pragma once

#include <Arduino.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

// K417 protocol constants
const uint8_t CMD_NONE = 0x00;
const uint8_t CMD_TAKEOFF = 0x01;
const uint8_t CMD_LAND = 0x02;
const uint8_t CMD_STOP = 0x02;
const uint8_t CMD_CAM_UP = 0x05;
const uint8_t CMD_CAM_DOWN = 0x06;
const uint8_t CMD_CALIBRATE = 0x04;
const uint8_t HEADLESS_OFF = 0x02;
const uint8_t HEADLESS_ON = 0x03;
const uint8_t SOMERSAULT_FLAG = 0x08;

const uint8_t PKT_HDR[12] = {
  0xEF, 0x02, 0x7C, 0x00, 0x02, 0x02,
  0x00, 0x01, 0x02, 0x00, 0x00, 0x00
};
const uint8_t C1_SUFFIX[6] = {0x00, 0x00, 0x14, 0x00, 0x66, 0x14};
const uint8_t CKSUM_PREFIX = 0x99;
const uint8_t CKSUM_TAIL[6] = {0x32, 0x4B, 0x14, 0x2D, 0x00, 0x00};
const uint8_t C2_SUFFIX[18] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
  0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF
};
const uint8_t C3_SUFFIX[14] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
  0x00, 0x00, 0x10, 0x00, 0x00, 0x00
};
const int PKT_SIZE = 124;

// WiFi & UDP
WiFiUDP udp;
IPAddress droneAddr;
bool wifiConnected = false;
bool controlStarted = false;

// K417-specific counters
volatile uint16_t ctr1 = 0, ctr2 = 1, ctr3 = 2;

bool arduinoUdpEnabled = true;
bool flightArmed = false;

// AHRS quaternions
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float eIntX = 0.0f, eIntY = 0.0f, eIntZ = 0.0f;
float qRef0 = 1.0f, qRef1 = 0.0f, qRef2 = 0.0f, qRef3 = 0.0f;

// Gyro calibration
bool gyroCalibrated = false;
int gyroCalibCount = 0;
float gyroSumX = 0.0f, gyroSumY = 0.0f, gyroSumZ = 0.0f;
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;

// Flex sensors
const int FLEX_PINS[4] = {A0, A1, A2, A3};
float flexMean[4] = {512.0f, 512.0f, 512.0f, 512.0f};
float flexStd[4] = {20.0f, 20.0f, 20.0f, 20.0f};
bool flexCalibrated = false;
int flexCalibCount = 0;
float flexSumBuf[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float flexSumSqBuf[4] = {0.0f, 0.0f, 0.0f, 0.0f};

bool zeroOrientation = false;
bool autoZeroAfterRecalib = false;

// Haptic state
int hapticPotValue = 255;
uint16_t hapticHvState = 0x0000;
volatile bool haptic_spi_busy = false;

enum HapticPulseMode { HPM_IDLE = 0, HPM_SINGLE, HPM_BURST, HPM_TRAIN, HPM_MULTI };
HapticPulseMode hapticPulseMode = HPM_IDLE;

unsigned long haptic_single_start_ms = 0, haptic_single_duration_ms = 0;
int haptic_burst_total = 0, haptic_burst_index = 0;
unsigned long haptic_burst_on_ms = 0, haptic_burst_off_ms = 0, haptic_burst_last_ms = 0;
bool haptic_burst_state_on = false;

unsigned long haptic_train_start_ms = 0, haptic_train_duration_ms_running = 0;
unsigned long haptic_train_period_us = 0, haptic_train_pw_us = 0;
unsigned long haptic_train_next_toggle_us = 0;
bool haptic_train_state_on = false;

// Safe time-division state for multi-channel stimulation on the reduced set of
// known-good HV routes.
bool hapticMultiActive[4] = {false, false, false, false};
int hapticMultiPot[4] = {255, 255, 255, 255};
int hapticMultiPos[4] = {2, 4, 12, 10};
int hapticMultiCursor = -1;
int hapticMultiActiveCount = 0;
unsigned long haptic_multi_next_slot_us = 0;
unsigned long haptic_multi_slot_period_us = 0;
unsigned long haptic_multi_pw_us = HAPTIC_DEFAULT_PW_US;
const unsigned long HAPTIC_MULTI_ROUTE_GUARD_US = 80UL;

float hapticFreq_Hz = HAPTIC_DEFAULT_FREQ_HZ;
unsigned long hapticPulseWidth_us = HAPTIC_DEFAULT_PW_US;
unsigned long hapticTrainDuration_ms = HAPTIC_DEFAULT_TRAIN_MS;
unsigned long hapticActionLockUntilMs = 0;
bool hapticDebugEnabled = false;
bool hapticAnyActiveLast = false;
unsigned long lastHapticDebugPrintMs = 0;

enum HapticPosition {
  HAPTIC_POS_YAW = 2,        // M2: Thumb region (Channel 1)
  HAPTIC_POS_PITCH = 4,      // M4: Index region (Channel 2)
  HAPTIC_POS_ROLL = 12,      // M12: Ring_top region (Channel 4)
  HAPTIC_POS_THROTTLE = 10   // M10: Middle_bottom region (Channel 7)
};

struct HapticFeedback {
  HapticPosition position;
  float directionSign;
  int potMin;
  int potMax;
  bool isActive;
  unsigned long lastTriggerMs;
};

HapticFeedback hapticYaw = {HAPTIC_POS_YAW, 0.0f, 18, 25, false, 0};
HapticFeedback hapticPitch = {HAPTIC_POS_PITCH, 0.0f, 15, 25, false, 0};
HapticFeedback hapticRoll = {HAPTIC_POS_ROLL, 0.0f, 15, 28, false, 0};
HapticFeedback hapticThrottle = {HAPTIC_POS_THROTTLE, 0.0f, 14, 20, false, 0};

const unsigned long HAPTIC_FEEDBACK_UPDATE_MS = 10;
unsigned long lastHapticFeedbackMs = 0;

float yawDeg = 0.0f, pitchDeg = 0.0f, rollDeg = 0.0f;
float throttleSmooth = (float)STICK_MID;
float gyroDpsX = 0.0f, gyroDpsY = 0.0f, gyroDpsZ = 0.0f;

unsigned long lastImuMicros = 0;
unsigned long lastCtrlMillis = 0;
unsigned long lastTelemMillis = 0;
const unsigned long CTRL_INTERVAL_MS = 1000UL / CONTROL_HZ;
const unsigned long TELEM_INTERVAL_MS = 1000UL / TELEMETRY_HZ;

uint8_t lastStickThrottle = STICK_MID;
uint8_t lastStickYaw = STICK_MID;

// Command flags
bool flagTakeoff = false;
bool flagLand = false;
bool flagCamUp = false;
bool flagStop = false;
bool flagCalibrate = false;
bool headlessEnabled = false;

// Flip state
enum FlipPhase : uint8_t {
  FLIP_PHASE_IDLE = 0,
  FLIP_PHASE_BURST,
  FLIP_PHASE_SETTLE
};

bool flipInProgress = false;
FlipPhase flipPhase = FLIP_PHASE_IDLE;
int flipPhaseRemaining = 0;
uint8_t flipRoll = STICK_MID;
uint8_t flipPitch = STICK_MID;
uint8_t flipHoldThrottle = STICK_MID;
uint8_t flipHoldYaw = STICK_MID;
uint8_t flipHeadlessBase = HEADLESS_OFF;

#if ENABLE_GLOVE_NN
using Eloquent::CortexM::TensorFlow;
constexpr int kNNTensorArenaSize = 16 * 1024;
constexpr int kNNNumInputs = 2;
constexpr int kNNNumOutputs = 9;
constexpr int kNNNumOps = 10;
TensorFlow<kNNNumOps, kNNTensorArenaSize> tf;

float nnScalerMean[kNNNumInputs] = {435.38202f, 400.79325f};
float nnScalerScale[kNNNumInputs] = {72.78391f, 84.44844f};

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
bool nnZeroLongHoldHeadlessTriggered = false;
#endif
