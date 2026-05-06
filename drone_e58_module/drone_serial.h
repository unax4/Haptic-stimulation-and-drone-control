#pragma once

#include <Arduino.h>
#include <ctype.h>
#include <string.h>

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
    triggerHapticAction((HapticPosition)HAPTIC_ACTION_ZERO_POS, HAPTIC_ACTION_ZERO_POT);
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
      float resistance_Ohms = (v / 255.0f) * 10000.0f;
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
    Serial.println(F("Haptic: Pxxx(pot) HS(pulse) HB(burst) HT(train) HFxx(freq) HWxx(width) HDxx(duration) HSWx(switch) ?"));
    return;
  }

  Serial.print(F("[CMD] Unknown: "));
  Serial.println(cmd);
}
