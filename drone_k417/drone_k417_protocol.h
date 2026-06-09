#pragma once

#include <Arduino.h>
#include <string.h>

void clearFlipState();

void buildPacket(uint8_t *buf,
                 uint8_t roll,
                 uint8_t pitch,
                 uint8_t throttle,
                 uint8_t yaw,
                 uint8_t command,
                 uint8_t headless,
                 uint16_t c1,
                 uint16_t c2,
                 uint16_t c3) {
  int i = 0;
  memcpy(buf + i, PKT_HDR, 12); i += 12;

  buf[i++] = (uint8_t)(c1 & 0xFF);
  buf[i++] = (uint8_t)((c1 >> 8) & 0xFF);
  memcpy(buf + i, C1_SUFFIX, 6); i += 6;

  uint8_t ctrl[6] = {roll, pitch, throttle, yaw, command, headless};
  memcpy(buf + i, ctrl, 6); i += 6;
  memset(buf + i, 0x00, 10); i += 10;

  uint8_t chk = 0;
  for (int j = 0; j < 6; j++) chk ^= ctrl[j];
  buf[i++] = chk;

  buf[i++] = CKSUM_PREFIX;
  memset(buf + i, 0x00, 44); i += 44;
  memcpy(buf + i, CKSUM_TAIL, 6); i += 6;

  buf[i++] = (uint8_t)(c2 & 0xFF);
  buf[i++] = (uint8_t)((c2 >> 8) & 0xFF);
  memcpy(buf + i, C2_SUFFIX, 18); i += 18;

  buf[i++] = (uint8_t)(c3 & 0xFF);
  buf[i++] = (uint8_t)((c3 >> 8) & 0xFF);
  memcpy(buf + i, C3_SUFFIX, 14);
}

uint16_t nextCounter(volatile uint16_t &ctr) {
  uint16_t val = ctr;
  ctr = (ctr + 1) & 0xFFFF;
  return val;
}

void sendControlPacket(uint8_t roll,
                       uint8_t pitch,
                       uint8_t throttle,
                       uint8_t yaw,
                       uint8_t cmd = CMD_NONE,
                       bool somersaultFlag = false) {
  if (!wifiConnected || !arduinoUdpEnabled) return;

  uint8_t headless = headlessEnabled ? HEADLESS_ON : HEADLESS_OFF;
  if (somersaultFlag) headless = (uint8_t)(headless | SOMERSAULT_FLAG);
  uint8_t pkt[PKT_SIZE];
  buildPacket(pkt, roll, pitch, throttle, yaw, cmd, headless,
              nextCounter(ctr1), nextCounter(ctr2), nextCounter(ctr3));

  udp.beginPacket(droneAddr, DRONE_PORT);
  udp.write(pkt, PKT_SIZE);
  udp.endPacket();
}

void sendControlPacketWithHeadless(uint8_t roll,
                                   uint8_t pitch,
                                   uint8_t throttle,
                                   uint8_t yaw,
                                   uint8_t cmd,
                                   uint8_t headlessBase,
                                   bool somersaultFlag = false) {
  if (!wifiConnected || !arduinoUdpEnabled) return;

  uint8_t headless = headlessBase;
  if (somersaultFlag) headless = (uint8_t)(headless | SOMERSAULT_FLAG);

  uint8_t pkt[PKT_SIZE];
  buildPacket(pkt, roll, pitch, throttle, yaw, cmd, headless,
              nextCounter(ctr1), nextCounter(ctr2), nextCounter(ctr3));

  udp.beginPacket(droneAddr, DRONE_PORT);
  udp.write(pkt, PKT_SIZE);
  udp.endPacket();
}

void sendConnectSession() {
  if (!wifiConnected) return;
  controlStarted = true;
  Serial.println(F("[K417] CONNECT not required"));
}

void sendDisconnectSession() {
  controlStarted = false;
  Serial.println(F("[K417] DISCONNECT not required"));
}

void sendStartControlBurst(int burst = START_BURST_COUNT) {
  if (!wifiConnected || !arduinoUdpEnabled) return;
  if (burst < 1) burst = 1;
  for (int i = 0; i < burst; i++) {
    sendControlPacket(STICK_MID, STICK_MID, STICK_MIN, STICK_MID, CMD_NONE);
    delay(START_BURST_DELAY_MS);
  }
  controlStarted = true;
  Serial.print(F("[K417] START neutral burst x"));
  Serial.println(burst);
}

void sendCalibratePulse() {
  sendControlPacket(STICK_MID, STICK_MID, STICK_MID, STICK_MID, CMD_CALIBRATE);
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
  flipPhase = FLIP_PHASE_IDLE;
  flipPhaseRemaining = 0;
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
  flipHoldThrottle = throttleSnapshot;
  flipHoldYaw = yawSnapshot;
  flipHeadlessBase = headlessEnabled ? HEADLESS_ON : HEADLESS_OFF;
  flipPhase = FLIP_PHASE_BURST;
  flipPhaseRemaining = FLIP_BURST_PACKETS;
  flipInProgress = true;

  triggerHapticAction(HAPTIC_POS_THROTTLE, 18, HAPTIC_ACTION_BURST_SPECIAL_COUNT);
  Serial.println(F("[HAPTIC] Flip mode feedback triggered"));
  Serial.print(F("[FLIP] START "));
  Serial.println(direction);
  Serial.print(F("[FLIP] throttleSnapshot=")); Serial.print(throttleSnapshot);
  Serial.print(F(" yawSnapshot=")); Serial.println(yawSnapshot);
}

bool applyFlipStep(uint8_t &stickRoll,
                   uint8_t &stickPitch,
                   uint8_t &stickThrottle,
                   uint8_t &stickYaw,
                   uint8_t &cmd,
                   bool &somersaultFlag) {
  if (!flipInProgress) return false;

  cmd = CMD_NONE;
  stickThrottle = flipHoldThrottle;
  stickYaw = flipHoldYaw;

  if (flipPhase == FLIP_PHASE_BURST) {
    stickRoll = flipRoll;
    stickPitch = flipPitch;
    somersaultFlag = true;
    stickThrottle = FLIP_BURST_THROTTLE;
    flipPhaseRemaining--;
    if (flipPhaseRemaining <= 0) {
      flipPhase = FLIP_PHASE_SETTLE;
      flipPhaseRemaining = FLIP_SETTLE_PACKETS;
    }
    return true;
  }

  if (flipPhase == FLIP_PHASE_SETTLE) {
    stickRoll = STICK_MID;
    stickPitch = STICK_MID;
    stickThrottle = FLIP_RECOVER_THROTTLE;
    somersaultFlag = false;
    flipPhaseRemaining--;
    if (flipPhaseRemaining <= 0) {
      clearFlipState();
      Serial.println(F("[FLIP] DONE"));
    }
    return true;
  }

  clearFlipState();
  return false;
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
