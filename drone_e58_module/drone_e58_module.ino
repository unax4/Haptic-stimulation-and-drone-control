/**
 * drone_e58_module.ino
 * -----------------------------------------------------------------------------
 * E58 WIFI CAM Drone - Direct Arduino Nano RP2040 Connect Controller
 * Modularized version: configure here, logic split across .h modules.
 * -----------------------------------------------------------------------------
 * Current haptic behavior, clearly mapped:

- `TAKEOFF` command:
  - Position: `M4` (Yaw region / thumb)
  - Pot: `20`
  - Pattern: burst, `HAPTIC_ACTION_BURST_COUNT` (currently `1`)

- `LAND` command:
  - Position: `M8` (Pitch region / index)
  - Pot: `25`
  - Pattern: burst, `1`

- `STOP` command:
  - Position: `M20` (Throttle region / palm)
  - Pot: `18`
  - Pattern: burst, `1`

- `ZERO` command:
  - Position: `M12` (Roll region / middle-ring)
  - Pot: `30`
  - Pattern: burst, `1`

- `FLIP start`:
  - Position: `M20` (palm)
  - Pot: `18`
  - Pattern: burst, `1`

- `NN flip armed` (gesture class 7):
  - Position: `M20` (palm)
  - Pot: `18`
  - Pattern: burst, `HAPTIC_FLIP_ARMED_BURST_COUNT` (currently `3`)

Continuous control feedback (while flying controls are off-neutral):
- Yaw off-mid -> activates `M4`
- Pitch off-mid -> activates `M8`
- Roll off-mid -> activates `M12`
- Throttle off-mid -> activates `M20`

Continuous signal rules:
- Pattern: train waveform (`HAPTIC_DEFAULT_FREQ_HZ`, `HAPTIC_DEFAULT_PW_US`)
- Multiple active controls: channels are combined (simultaneous routing ON)
- Shared intensity pot for all active channels:
  - Computed from each axis displacement-from-mid
  - Uses the maximum requested pot among active axes
- If all controls return neutral: train stops immediately

Intensity ranges used for continuous mode:
- Yaw: `20..25`
- Pitch: `25..28`
- Roll: `31..35`
- Throttle: `16..20`
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
#include "neural/glove_fcnn_40_20_model_data.h"
#endif

// -----------------------------------------------------------------------------
// User configuration (edit this block)
// -----------------------------------------------------------------------------
const char* DRONE_SSID = "WIFI_8K__bcc908";
const char* DRONE_PASSWORD = "";

const char* DRONE_IP = "192.168.4.153";
const int DRONE_SESSION_PORT = 8080;
const int DRONE_CONTROL_PORT = 8090;

const int CONTROL_HZ = 50;
const int TELEMETRY_HZ = 30;

// -------- Haptic Stimulation Control Pins --------
const int HAPTIC_POT_CS   = 6;
const int HAPTIC_DATA_PIN = 5;
const int HAPTIC_CLK_PIN  = 4;
const int HAPTIC_HV_LE    = 3;
const int HAPTIC_HV_CLR   = 2;
const int HAPTIC_OUT_PIN  = 12;

// -------- Haptic Default Parameters --------
const unsigned long HAPTIC_SINGLE_PULSE_MS = 1000;
const int HAPTIC_BURST_COUNT = 5;
const unsigned long HAPTIC_BURST_PULSE_MS = 50;
const unsigned long HAPTIC_BURST_PAUSE_MS = 50;

const int HAPTIC_ACTION_BURST_COUNT = 1;
const int HAPTIC_FLIP_ARMED_BURST_COUNT = 3;
const float HAPTIC_DEFAULT_FREQ_HZ = 100.0f;
const unsigned long HAPTIC_DEFAULT_PW_US = 400;
const unsigned long HAPTIC_DEFAULT_TRAIN_MS = 1000;

// -------- Action Haptic Mapping --------
const int HAPTIC_ACTION_TAKEOFF_POS = 4;
const int HAPTIC_ACTION_TAKEOFF_POT = 20;

const int HAPTIC_ACTION_LAND_POS = 8;
const int HAPTIC_ACTION_LAND_POT = 25;

const int HAPTIC_ACTION_STOP_POS = 20;
const int HAPTIC_ACTION_STOP_POT = 18;

const int HAPTIC_ACTION_ZERO_POS = 12;
const int HAPTIC_ACTION_ZERO_POT = 30;

const int HAPTIC_ACTION_FLIP_START_POS = 20;
const int HAPTIC_ACTION_FLIP_START_POT = 18;

const int HAPTIC_ACTION_FLIP_ARMED_POS = 20;
const int HAPTIC_ACTION_FLIP_ARMED_POT = 18;
// -------- Haptic Feedback Mapping Parameters --------
const int HAPTIC_YAW_POT_MIN = 20;
const int HAPTIC_YAW_POT_MAX = 25;

const int HAPTIC_PITCH_POT_MIN = 25;
const int HAPTIC_PITCH_POT_MAX = 28;

const int HAPTIC_ROLL_POT_MIN = 31;
const int HAPTIC_ROLL_POT_MAX = 35;

const int HAPTIC_THROTTLE_POT_MIN = 16;
const int HAPTIC_THROTTLE_POT_MAX = 20;

const unsigned long HAPTIC_FEEDBACK_UPDATE_MS = 20;

// -------- Flight / Sensor Parameters --------
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

const int START_BURST_COUNT = 6;
const int START_BURST_DELAY_MS = 30;

const int FLIP_BURST_PACKETS = 20;
const int FLIP_RECOVER_PACKETS = 10;
const float FLIP_POST_HOLD_S = 0.25f;
const uint8_t FLIP_THR_MIN = 165;  //Minimum throttle used for flip
const uint8_t FLIP_THR_BURST_BOOST = 28; //Extra throttle during actual somersault packets
const uint8_t FLIP_THR_RECOVER_BOOST = 25; //Throttle at start of recovery phase.
const uint8_t FLIP_THR_POST_BOOST = 12;

#if ENABLE_GLOVE_NN
const unsigned long NN_PERIOD_MS = 80;
const unsigned long NN_HOLD_MS = 350;
const int NN_MIN_MARGIN_Q = 5;
const unsigned long NN_ACTION_COOLDOWN_MS = 900;
#endif

// -----------------------------------------------------------------------------
// Modules
// -----------------------------------------------------------------------------
#include "drone_state.h"
#include "drone_haptics.h"
#include "drone_ahrs.h"
#include "drone_protocol.h"
#include "drone_nn.h"
#include "drone_serial.h"

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
      triggerHapticAction((HapticPosition)HAPTIC_ACTION_TAKEOFF_POS, HAPTIC_ACTION_TAKEOFF_POT);
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
      triggerHapticAction((HapticPosition)HAPTIC_ACTION_STOP_POS, HAPTIC_ACTION_STOP_POT);
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
      triggerHapticAction((HapticPosition)HAPTIC_ACTION_LAND_POS, HAPTIC_ACTION_LAND_POT);
      Serial.println(F("[HAPTIC] Landing feedback triggered"));
      sendLandPacket(stickYaw);
      sentLandPacket = true;
      cmd = CMD_LAND;
    }

    if (flipInProgress) {
      if (flipBurstRemaining > 0) {
        if (flipBurstRemaining == FLIP_BURST_PACKETS) {
          triggerHapticAction((HapticPosition)HAPTIC_ACTION_FLIP_START_POS, HAPTIC_ACTION_FLIP_START_POT);
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

    updateHapticFeedback(stickYaw, stickPitch, stickRoll, stickThrottle);

    if (millis() - lastTelemMillis >= TELEM_INTERVAL_MS) {
      lastTelemMillis = millis();
      emitTelemetry(yawDeg, pitchDeg, rollDeg,
                    stickRoll, stickPitch, stickThrottle, stickYaw, cmd,
                    rawA0, rawA1, rawA2, rawA3);
    }
  }

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
