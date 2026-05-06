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
      Serial.println(F("[NN] STOP action"));
      break;

    case 3:
      flagTakeoff = true;
      Serial.println(F("[NN] TAKEOFF action"));
      break;

    case 4:
      captureZero();
      Serial.println(F("[NN] ZERO action"));
      break;

    case 7:
      nnFlipModeEnabled = true;
      nnFlipTriggerLatched = false;
      nnFlipModeSinceMillis = millis();
      triggerHapticAction((HapticPosition)HAPTIC_ACTION_FLIP_ARMED_POS,
                          HAPTIC_ACTION_FLIP_ARMED_POT,
                          HAPTIC_FLIP_ARMED_BURST_COUNT);
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
