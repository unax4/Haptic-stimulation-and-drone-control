#pragma once

#include <Arduino.h>

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
      triggerHapticAction(HAPTIC_POS_THROTTLE, 18);  // STOP: 1 burst on M20 (palm) with pot 18
      Serial.println(F("[HAPTIC] Stop feedback triggered"));
      Serial.println(F("[NN] STOP action"));
      break;

    case 3:
      flagTakeoff = true;
      triggerHapticAction(HAPTIC_POS_YAW, 20);  // TAKEOFF: 1 burst on M4 (thumb) with pot 20
      Serial.println(F("[HAPTIC] Takeoff feedback triggered"));
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
      triggerHapticAction(HAPTIC_POS_THROTTLE, 18, HAPTIC_ACTION_BURST_SPECIAL_COUNT);  // 3 bursts on M20 (palm) to indicate flip mode armed
      Serial.println(F("[NN] FLIP ARMED (one-shot)"));
      break;

    case 2:
      flagLand = true;
      triggerHapticAction(HAPTIC_POS_PITCH, 25);  // LAND: 1 burst on M8 (index) with pot 25
      Serial.println(F("[HAPTIC] Land feedback triggered"));
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
    nnZeroLongHoldHeadlessTriggered = false;
    return;
  }

  if (pred != nnLastClass) {
    nnLastClass = pred;
    nnClassStartMillis = now;
    nnZeroLongHoldHeadlessTriggered = false;
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

  if (pred == 4 &&
      !nnZeroLongHoldHeadlessTriggered &&
      (now - nnClassStartMillis >= NN_ZERO_TO_HEADLESS_HOLD_MS)) {
    headlessEnabled = !headlessEnabled;
    if (headlessEnabled) {
      headlessRefYawDeg = yawDeg;
    }
    flagHeadlessPulse = true;
    triggerHapticAction(HAPTIC_POS_ROLL, 30, HAPTIC_ACTION_BURST_SPECIAL_COUNT);  // Headless feedback: M12, 3-burst, pot 30
    Serial.println(F("[HAPTIC] Headless long-hold feedback triggered"));
    nnZeroLongHoldHeadlessTriggered = true;
    Serial.print(F("[NN] HEADLESS long-hold from ZERO -> "));
    Serial.println(headlessEnabled ? F("ON") : F("OFF"));
  }

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
