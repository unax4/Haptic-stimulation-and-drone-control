#pragma once

#include <Arduino.h>
#include <math.h>

void captureZero() {
  qRef0 = q0;
  qRef1 = -q1;
  qRef2 = -q2;
  qRef3 = -q3;
  Serial.println(F("[AHRS] Zero orientation captured."));
}

void mahonyUpdate(float ax, float ay, float az,
                  float gx, float gy, float gz,
                  float dt) {
  float gxR = gx * (float)(M_PI / 180.0);
  float gyR = gy * (float)(M_PI / 180.0);
  float gzR = gz * (float)(M_PI / 180.0);

  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm < 1e-6f) return;
  ax /= norm;
  ay /= norm;
  az /= norm;

  float vx = 2.0f * (q1 * q3 - q0 * q2);
  float vy = 2.0f * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  float ex = ay * vz - az * vy;
  float ey = az * vx - ax * vz;
  float ez = ax * vy - ay * vx;

  eIntX += ex * MAHONY_KI * dt;
  eIntY += ey * MAHONY_KI * dt;
  eIntZ += ez * MAHONY_KI * dt;

  gxR += MAHONY_KP * ex + eIntX;
  gyR += MAHONY_KP * ey + eIntY;
  gzR += MAHONY_KP * ez + eIntZ;

  float hw = 0.5f * dt;
  float qa = q0, qb = q1, qc = q2;
  q0 += (-qb * gxR - qc * gyR - q3 * gzR) * hw;
  q1 += (qa * gxR + qc * gzR - q3 * gyR) * hw;
  q2 += (qa * gyR - qb * gzR + q3 * gxR) * hw;
  q3 += (qa * gzR + qb * gyR - qc * gxR) * hw;

  norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= norm;
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;
}

void getRelativeEuler(float &yaw_out, float &pitch_out, float &roll_out) {
  float w = qRef0 * q0 - qRef1 * q1 - qRef2 * q2 - qRef3 * q3;
  float x = qRef0 * q1 + qRef1 * q0 + qRef2 * q3 - qRef3 * q2;
  float y = qRef0 * q2 - qRef1 * q3 + qRef2 * q0 + qRef3 * q1;
  float z = qRef0 * q3 + qRef1 * q2 - qRef2 * q1 + qRef3 * q0;

  roll_out = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * (float)(180.0 / M_PI);

  float sinp = 2.0f * (w * y - z * x);
  sinp = constrain(sinp, -1.0f, 1.0f);
  pitch_out = asinf(sinp) * (float)(180.0 / M_PI);

  yaw_out = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * (float)(180.0 / M_PI);
}

uint8_t angleToStick(float angle, float deadzone, float sensitivity, float expo) {
  float sign = (angle >= 0.0f) ? 1.0f : -1.0f;
  float mag = fabsf(angle);
  if (mag < deadzone) return STICK_MID;

  float norm = (mag - deadzone) / (MAX_ANGLE_DEG - deadzone);
  norm = min(norm * sensitivity, 1.0f);
  float curved = norm * (1.0f - expo) + norm * norm * norm * expo;
  curved = constrain(curved, 0.0f, 1.0f);

  float raw = STICK_MID + sign * curved * (float)(STICK_MAX - STICK_MID);
  return (uint8_t)constrain((int)raw, STICK_MIN, STICK_MAX);
}

float wrapDeg180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void applyHeadlessTransform(uint8_t &stickRoll, uint8_t &stickPitch, float yawNowDeg) {
  float relDeg = wrapDeg180(yawNowDeg - headlessRefYawDeg);
  float th = relDeg * (float)(M_PI / 180.0);
  float x = (float)((int)stickRoll - (int)STICK_MID);
  float y = (float)((int)stickPitch - (int)STICK_MID);
  float xr = x * cosf(th) - y * sinf(th);
  float yr = x * sinf(th) + y * cosf(th);
  stickRoll = (uint8_t)constrain((int)roundf((float)STICK_MID + xr), (int)STICK_MIN, (int)STICK_MAX);
  stickPitch = (uint8_t)constrain((int)roundf((float)STICK_MID + yr), (int)STICK_MIN, (int)STICK_MAX);
}

float flexDeflection(int raw, int idx) {
  float delta = (float)raw - flexMean[idx];
  float thresh = FLEX_THRESH_STD_MULTIPLIER * flexStd[idx];
  if (fabsf(delta) < thresh) return 0.0f;
  float signedExcess = delta - (delta >= 0.0f ? thresh : -thresh);
  return constrain(signedExcess / FLEX_NORM_SCALE, -1.0f, 1.0f);
}

uint8_t computeThrottle(int rawA2, int rawA3) {
  if (!flexCalibrated) return (uint8_t)throttleSmooth;

  float d2 = flexDeflection(rawA2, 2);
  float d3 = flexDeflection(rawA3, 3);
  float net = constrain(d2 - d3, -1.0f, 1.0f);

  float s = (net >= 0.0f) ? 1.0f : -1.0f;
  float m = fabsf(net);
  float mapped = 0.0f;
  if (m > THR_NET_DEADZONE) {
    mapped = (m - THR_NET_DEADZONE) / (1.0f - THR_NET_DEADZONE);
  }

  float ct = mapped * (1.0f - THR_EXPO) + mapped * mapped * mapped * THR_EXPO;
  float raw = STICK_MID + s * ct * (float)(STICK_MAX - STICK_MID);
  raw = constrain(raw, (float)STICK_MIN, (float)STICK_MAX);

  throttleSmooth += (raw - throttleSmooth) * THROTTLE_ALPHA;
  if (mapped == 0.0f && fabsf(throttleSmooth - (float)STICK_MID) <= THR_NEUTRAL_SNAP_STICK) {
    throttleSmooth = (float)STICK_MID;
  }

  return (uint8_t)constrain((int)throttleSmooth, STICK_MIN, STICK_MAX);
}
