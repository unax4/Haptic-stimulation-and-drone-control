#pragma once

#include <Arduino.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

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

// -------- Haptic Feedback Mapping --------
enum HapticPosition {
  HAPTIC_POS_YAW = 4,
  HAPTIC_POS_PITCH = 8,
  HAPTIC_POS_ROLL = 12,
  HAPTIC_POS_THROTTLE = 20
};

struct HapticFeedback {
  HapticPosition position;
  float directionSign;
  int potMin;
  int potMax;
  bool isActive;
  unsigned long lastTriggerMs;
};

HapticFeedback hapticYaw = {HAPTIC_POS_YAW, 0.0f, HAPTIC_YAW_POT_MIN, HAPTIC_YAW_POT_MAX, false, 0};
HapticFeedback hapticPitch = {HAPTIC_POS_PITCH, 0.0f, HAPTIC_PITCH_POT_MIN, HAPTIC_PITCH_POT_MAX, false, 0};
HapticFeedback hapticRoll = {HAPTIC_POS_ROLL, 0.0f, HAPTIC_ROLL_POT_MIN, HAPTIC_ROLL_POT_MAX, false, 0};
HapticFeedback hapticThrottle = {HAPTIC_POS_THROTTLE, 0.0f, HAPTIC_THROTTLE_POT_MIN, HAPTIC_THROTTLE_POT_MAX, false, 0};

// Haptic feedback update interval (ms)
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

// One-shot command flags.
bool flagTakeoff = false;
bool flagLand = false;
bool flagStop = false;
bool flagCalibrate = false;
bool flagHeadlessPulse = false;
bool headlessEnabled = false;
float headlessRefYawDeg = 0.0f;

// Flip state.
bool flipInProgress = false;
int flipBurstRemaining = 0;
int flipRecoverRemaining = 0;
uint8_t flipRoll = STICK_MID;
uint8_t flipPitch = STICK_MID;
uint8_t flipHoldYaw = STICK_MID;

#if ENABLE_GLOVE_NN
using Eloquent::CortexM::TensorFlow;
constexpr int kNNTensorArenaSize = 16 * 1024;
constexpr int kNNNumInputs = 2;
constexpr int kNNNumOutputs = 9;
constexpr int kNNNumOps = 10;
TensorFlow<kNNNumOps, kNNTensorArenaSize> tf;

float nnScalerMean[kNNNumInputs] = {434.19621749f, 379.56973995f};
float nnScalerScale[kNNNumInputs] = {61.45421933f, 70.63745035f};

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
