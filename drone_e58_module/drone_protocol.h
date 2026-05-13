#pragma once

#include <Arduino.h>
#include <string.h>

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
  sendControlPacket(STICK_MID, STICK_MID, lastStickThrottle, lastStickYaw, CMD_HEADLESS_PULSE);
}

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

void clearFlipState() {
  flipInProgress = false;
  flipBurstRemaining = 0;
  flipRecoverRemaining = 0;
}

void startFlip(const char *direction, uint8_t yawSnapshot) {
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
  clearFlipState();
  Serial.println(F("[LAND] Land packet sent"));
}

void triggerLocalRecalibration() {
  flightArmed = false;
  clearFlipState();

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
  nnZeroLongHoldHeadlessTriggered = false;
#endif

  Serial.println(F("[CALIB] Re-calibrating, keep still..."));
}

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
