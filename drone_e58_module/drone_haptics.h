#pragma once

#include <Arduino.h>

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

inline uint16_t hapticHvStateForPosition(HapticPosition position) {
  return positions[(int)position];
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

  digitalWrite(HAPTIC_HV_LE, HIGH);
  digitalWrite(HAPTIC_POT_CS, LOW);

  digitalWrite(HAPTIC_DATA_PIN, 1);
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
      }
      else {
        if (haptic_burst_index >= haptic_burst_total) {
          digitalWrite(HAPTIC_OUT_PIN, LOW);
          hapticPulseMode = HPM_IDLE;
        }
        else if (now_ms - haptic_burst_last_ms >= haptic_burst_off_ms) {
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
      }
      else {
        if ((long)(now_us - haptic_train_next_toggle_us) >= 0) {
          digitalWrite(HAPTIC_OUT_PIN, HIGH);
          haptic_train_state_on = true;
          haptic_train_next_toggle_us = now_us + haptic_train_pw_us;
        }
      }
      break;
  }
}

// -------- Haptic Feedback Triggers --------
void triggerHapticFeedback(HapticFeedback *feedback, int potValue) {
  hapticSendToHV2701(hapticHvStateForPosition(feedback->position));
  hapticSetPot(potValue);
  hapticStartTrain(HAPTIC_DEFAULT_FREQ_HZ, HAPTIC_DEFAULT_PW_US, HAPTIC_DEFAULT_TRAIN_MS);
  feedback->isActive = true;
  feedback->lastTriggerMs = millis();
}

void triggerHapticAction(HapticPosition position, int potValue, int burstCount = HAPTIC_ACTION_BURST_DEFAULT_COUNT) {
  int n = max(1, burstCount);
  hapticSendToHV2701(hapticHvStateForPosition(position));
  hapticSetPot(potValue);
  hapticStartBurst(n, HAPTIC_BURST_PULSE_MS, HAPTIC_BURST_PAUSE_MS);
  unsigned long actionMs = (unsigned long)(n * HAPTIC_BURST_PULSE_MS) +
                           (unsigned long)(max(0, n - 1) * HAPTIC_BURST_PAUSE_MS);
  hapticActionLockUntilMs = millis() + actionMs;
  if (hapticDebugEnabled) {
    Serial.print(F("[HDBG] BURST pos="));
    Serial.print(hapticPosName(position));
    Serial.print(F(" pot="));
    Serial.print(potValue);
    Serial.print(F(" count="));
    Serial.print(n);
    Serial.print(F(" on="));
    Serial.print(HAPTIC_BURST_PULSE_MS);
    Serial.print(F("ms off="));
    Serial.print(HAPTIC_BURST_PAUSE_MS);
    Serial.print(F("ms lock="));
    Serial.print(actionMs);
    Serial.println(F("ms"));
  }
}

void updateHapticFeedback(uint8_t stickYaw, uint8_t stickPitch, uint8_t stickRoll, uint8_t stickThrottle) {
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

  if (hapticPulseMode != HPM_TRAIN) {
    hapticStartTrain(HAPTIC_DEFAULT_FREQ_HZ, HAPTIC_DEFAULT_PW_US, 60000UL);
  }
  else {
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
