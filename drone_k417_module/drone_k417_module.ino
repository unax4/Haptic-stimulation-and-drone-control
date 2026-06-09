/**
 * drone_k417_module.ino
 * K417 WIFI Drone - Modularized Arduino Nano RP2040 Connect Controller
 */

#include <Arduino_LSM6DSOX.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <math.h>
#include <ctype.h>
#include <string.h>

#define ENABLE_GLOVE_NN 1

#if ENABLE_GLOVE_NN
#include <eloquent_tensorflow_cortexm.h>
#include "neural/glove_fcnn_eloquent_inference/glove_fcnn_40_20_model_data.h"
#endif

// User configuration
const char* DRONE_SSID = "Drone-BBF0B4";
const char* DRONE_PASSWORD = "";
const char* DRONE_IP = "192.168.169.1";
const int DRONE_PORT = 8800;

const int CONTROL_HZ = 40;
const int TELEMETRY_HZ = 25;

// Haptic pins
const int HAPTIC_POT_CS   = 10;
const int HAPTIC_DATA_PIN = 9;
const int HAPTIC_CLK_PIN  = 8;
const int HAPTIC_HV_LE    = 7;
const int HAPTIC_HV_CLR   = 6;
const int HAPTIC_OUT_PIN  = 12;

// Haptic configuration
const unsigned long HAPTIC_SINGLE_PULSE_MS = 1000;
const int HAPTIC_BURST_COUNT = 5;
const unsigned long HAPTIC_BURST_PULSE_MS = 50;
const unsigned long HAPTIC_BURST_PAUSE_MS = 100;
const int HAPTIC_ACTION_BURST_DEFAULT_COUNT = 1;
const int HAPTIC_ACTION_BURST_SPECIAL_COUNT = 3;
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

const float PR_DEADZONE = 8.0f;
const float YAW_DEADZONE = 8.0f;
const float PR_SENSITIVITY = 1.0f;
const float YAW_SENSITIVITY = 2.0f;
const float PR_EXPO = 0.5f;
const float YAW_EXPO = 0.5f;
const float MAX_ANGLE_DEG = 45.0f;

const float FLEX_THRESH_STD_MULTIPLIER = 2.0f;
const float FLEX_NORM_SCALE = 90.0f;
const float THROTTLE_ALPHA = 0.12f;
const float THR_NET_DEADZONE = 0.12f;
const float THR_EXPO = 0.10f;
const float THR_NEUTRAL_SNAP_STICK = 2.0f;

const int THR_UP_PIN = A2;
const int THR_DOWN_PIN = A3;

const int START_BURST_COUNT = 6;
const int START_BURST_DELAY_MS = 30;

const int FLIP_BURST_PACKETS = 20;
const int FLIP_SETTLE_PACKETS = 10;
const uint8_t FLIP_BURST_THROTTLE = 212;
const uint8_t FLIP_RECOVER_THROTTLE = 204;

#if ENABLE_GLOVE_NN
const unsigned long NN_PERIOD_MS = 80;
const unsigned long NN_HOLD_MS = 350;
const unsigned long NN_ZERO_TO_HEADLESS_HOLD_MS = 2000;
const int NN_MIN_MARGIN_Q = 5;
const unsigned long NN_ACTION_COOLDOWN_MS = 900;
#endif

// Include all module headers
#include "drone_k417_state.h"
#include "drone_k417_haptics.h"
#include "drone_k417_ahrs.h"
#include "drone_k417_protocol.h"
#include "drone_k417_serial.h"
#include "drone_k417_nn.h"

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println(F("=== K417 Arduino WiFi Drone Controller ==="));

  Serial.print(F("[IMU] Initializing LSM6DSOX... "));
  if (!IMU.begin()) {
    Serial.println(F("FAILED - halting."));
    while (true) {
      delay(500);
    }
  }
  Serial.println(F("OK"));

  // Initialize Haptic Control Pins
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
    udp.begin(8802);

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

  triggerLocalRecalibration();

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
    gyroDpsX = gx;
    gyroDpsY = gy;
    gyroDpsZ = gz;

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
      clearFlipState();
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
    if (flagLand) {
      flagLand = false;
      clearFlipState();
      triggerHapticAction(HAPTIC_POS_PITCH, 25);
      Serial.println(F("[HAPTIC] Landing feedback triggered"));
      sendLandPacket(stickYaw);
      sentLandPacket = true;
      cmd = CMD_LAND;
    }
    else if (flagCamUp) {
      flagCamUp = false;
      cmd = CMD_CAM_UP;
    }

    bool flipSomersaultFlag = false;
    if (flipInProgress && applyFlipStep(stickRoll, stickPitch, stickThrottle, stickYaw, cmd, flipSomersaultFlag)) {
      if (flipSomersaultFlag) {
        Serial.print(F("[FLIP] BURST send roll=")); Serial.print(stickRoll);
        Serial.print(F(" pitch=")); Serial.print(stickPitch);
        Serial.print(F(" thr=")); Serial.print(stickThrottle);
        Serial.print(F(" yaw=")); Serial.print(stickYaw);
        Serial.print(F(" rem=")); Serial.print(flipPhaseRemaining);
        Serial.print(F(" headless=0x"));
        if (flipHeadlessBase < 16) Serial.print(0);
        Serial.println(flipHeadlessBase, HEX);
      }
      
      uint8_t effectiveCmd = cmd;
      if (flipSomersaultFlag) {
        effectiveCmd = (uint8_t)(effectiveCmd | SOMERSAULT_FLAG);
      }
      
      sendControlPacketWithHeadless(stickRoll, stickPitch, stickThrottle, stickYaw, effectiveCmd, flipHeadlessBase, flipSomersaultFlag);
      sentFlipBurstPacket = true;
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
