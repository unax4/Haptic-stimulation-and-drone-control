#include <Arduino_LSM6DSOX.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

/*
  K417 glove direct controller for Arduino Nano RP2040 Connect
  -------------------------------------------------------------
  - Reads IMU (LSM6DSOX) + flex sensors (A3/A2/A1/A0)
  - Runs Mahony AHRS onboard
  - Builds and sends K417 RC UDP packets via WiFiNINA
  - Mirrors core control features from control_video_v6.py (no video)

  Serial command examples:
    WIFI my_ssid my_password
    CONNECT
    DRONEIP 192.168.169.1 8800
    T / TAKEOFF
    L / LAND
    SPACE / STOP
    H / HEADLESS
    C / CAL
    O / ZERO
    F5 / RECAL
    PGUP / CAMUP
    PGDN / CAMDOWN
    FLIP FWD|BACK|LEFT|RIGHT
    IMU ON|OFF
    RATE 40
    STATUS
    HELP
*/

// ---------------- Protocol constants ----------------
const char *DEFAULT_DRONE_IP = "192.168.169.1";
const uint16_t DEFAULT_DRONE_PORT = 8800;
const uint16_t LOCAL_UDP_PORT = 8890;

const uint8_t STICK_MIN = 40;
const uint8_t STICK_MID = 128;
const uint8_t STICK_MAX = 220;

const uint8_t CMD_NONE = 0x00;
const uint8_t CMD_TAKEOFF = 0x01;
const uint8_t CMD_LAND = 0x02;
const uint8_t CMD_STOP = 0x02;
const uint8_t CMD_CALIBRATE = 0x04;
const uint8_t CMD_CAM_UP = 0x05;
const uint8_t CMD_CAM_DOWN = 0x06;

const uint8_t START_STREAM[4] = {0xEF, 0x00, 0x04, 0x00};

const uint8_t HEADLESS_OFF = 0x02;
const uint8_t HEADLESS_ON = 0x03;
const uint8_t SOMERSAULT_FLAG = 0x08;

const uint8_t HDR[12] = {
  0xEF, 0x02, 0x7C, 0x00, 0x02, 0x02,
  0x00, 0x01, 0x02, 0x00, 0x00, 0x00
};
const uint8_t C1_SUFFIX[6] = {0x00, 0x00, 0x14, 0x00, 0x66, 0x14};
const uint8_t CTRL_PAD[10] = {0};
const uint8_t CKSUM_SFX[51] = {
  0x99,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0x32, 0x4B, 0x14, 0x2D, 0x00, 0x00
};
const uint8_t C2_SUFFIX[18] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
  0x00, 0x00, 0x14, 0x00, 0x00, 0x00,
  0xFF, 0xFF, 0xFF, 0xFF
};
const uint8_t C3_SUFFIX[14] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x03, 0x00, 0x00, 0x00, 0x10, 0x00,
  0x00, 0x00
};

// 12 + 2 + 6 + 6 + 10 + 1 + 51 + 2 + 18 + 2 + 14 = 124 bytes
const uint16_t PACKET_SIZE = 124;

// ---------------- Pins ----------------
const int FLEX_A3 = A3; // finger down
const int FLEX_A2 = A2; // finger up
const int FLEX_A1 = A1;
const int FLEX_A0 = A0;

// ---------------- WiFi ----------------
WiFiUDP udp;
char wifiSsid[64] = "Drone-BB71A1";
char wifiPass[64] = "";
char droneIp[32] = "192.168.169.1";
uint16_t dronePort = DEFAULT_DRONE_PORT;
IPAddress droneAddr;
bool wifiConnected = false;
bool udpStarted = false;
unsigned long lastUdpErrorMs = 0;
bool droneAddrValid = false;
bool debugTxEnabled = false;
unsigned long txCount = 0;
unsigned long lastTxReportMs = 0;

// ---------------- Timing ----------------
unsigned long lastControlMs = 0;
unsigned long lastSensorMs = 0;
unsigned long lastStatusMs = 0;
float controlRateHz = 40.0f;

// ---------------- Command / mode state ----------------
bool imuEnabled = true;
bool streamingEnabled = true;   // Enable by default for telemetry_monitor.py
bool headlessEnabled = false;

bool flagTakeoff = false;
bool flagLand = false;
bool flagStop = false;
bool flagCalibrate = false;
bool flagCamUp = false;
bool flagCamDown = false;

// Repeat one-shot commands across several packets for UDP reliability.
uint8_t repeatCmd = CMD_NONE;
uint8_t repeatCmdLeft = 0;
const uint8_t ONE_SHOT_REPEAT_PACKETS = 8;

// Smart landing state
bool smartLandActive = false;
unsigned long smartLandStartMs = 0;
const unsigned long SMART_LAND_MS = 4000;

// Flip state
enum FlipDir { FLIP_NONE = 0, FLIP_FWD, FLIP_BACK, FLIP_LEFT, FLIP_RIGHT };
bool flipActive = false;
FlipDir flipDir = FLIP_NONE;
uint8_t flipPacketsLeft = 0;
uint8_t flipSettlePacketsLeft = 0;

// Counter fields used by K417 packet
uint16_t c1 = 0;
uint16_t c2 = 1;
uint16_t c3 = 2;

// ---------------- Sticks ----------------
float stickThrottle = STICK_MID;
float stickYaw = STICK_MID;
float stickPitch = STICK_MID;
float stickRoll = STICK_MID;

// ---------------- Flex calibration/mapping ----------------
const int FLEX_REST_SAMPLES = 80;
const float FLEX_THRESH_STD = 3.0f;
float flexNormScale = 150.0f;
float throttleAlpha = 0.12f;

long flexAcc[4] = {0, 0, 0, 0};
long flexSqAcc[4] = {0, 0, 0, 0};
int flexCalCount = 0;
bool flexCalibrated = false;
float flexMean[4] = {512, 512, 512, 512};
float flexStd[4] = {20, 20, 20, 20};

// ---------------- Mahony filter ----------------
class MahonyFilter {
public:
  float kp = 5.0f;
  float ki = 0.02f;

  float q0 = 1.0f;
  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;

  float eIntX = 0.0f;
  float eIntY = 0.0f;
  float eIntZ = 0.0f;

  float gyroBiasX = 0.0f;
  float gyroBiasY = 0.0f;
  float gyroBiasZ = 0.0f;

  bool gyroBiasCalibrated = false;
  int gyroCalCount = 0;
  float gyroCalAccX = 0.0f;
  float gyroCalAccY = 0.0f;
  float gyroCalAccZ = 0.0f;

  // offset quaternion inverse for zeroing
  float offW = 1.0f;
  float offX = 0.0f;
  float offY = 0.0f;
  float offZ = 0.0f;

  void resetCalibration() {
    gyroBiasCalibrated = false;
    gyroCalCount = 0;
    gyroCalAccX = 0.0f;
    gyroCalAccY = 0.0f;
    gyroCalAccZ = 0.0f;
    eIntX = eIntY = eIntZ = 0.0f;
  }

  void addGyroCalSample(float gx, float gy, float gz, int nSamples = 150) {
    if (gyroBiasCalibrated) return;
    gyroCalAccX += gx;
    gyroCalAccY += gy;
    gyroCalAccZ += gz;
    gyroCalCount++;
    if (gyroCalCount >= nSamples) {
      gyroBiasX = gyroCalAccX / gyroCalCount;
      gyroBiasY = gyroCalAccY / gyroCalCount;
      gyroBiasZ = gyroCalAccZ / gyroCalCount;
      gyroBiasCalibrated = true;
      captureOffset();
    }
  }

  void captureOffset() {
    // Inverse of current quaternion
    offW = q0;
    offX = -q1;
    offY = -q2;
    offZ = -q3;
  }

  void update(float ax, float ay, float az, float gxDeg, float gyDeg, float gzDeg, float dt) {
    if (dt <= 0.0f) return;

    float gx = gxDeg;
    float gy = gyDeg;
    float gz = gzDeg;

    if (gyroBiasCalibrated) {
      gx -= gyroBiasX;
      gy -= gyroBiasY;
      gz -= gyroBiasZ;
    }

    const float deg2rad = 0.01745329251994329577f;
    gx *= deg2rad;
    gy *= deg2rad;
    gz *= deg2rad;

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

    eIntX += ex * ki * dt;
    eIntY += ey * ki * dt;
    eIntZ += ez * ki * dt;

    gx += kp * ex + eIntX;
    gy += kp * ey + eIntY;
    gz += kp * ez + eIntZ;

    float halfDt = 0.5f * dt;
    float qa = q0;
    float qb = q1;
    float qc = q2;
    float qd = q3;

    q0 += (-qb * gx - qc * gy - qd * gz) * halfDt;
    q1 += (qa * gx + qc * gz - qd * gy) * halfDt;
    q2 += (qa * gy - qb * gz + qd * gx) * halfDt;
    q3 += (qa * gz + qb * gy - qc * gx) * halfDt;

    float qNorm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (qNorm < 1e-6f) return;
    q0 /= qNorm;
    q1 /= qNorm;
    q2 /= qNorm;
    q3 /= qNorm;
  }

  void getEulerRelative(float &yawDeg, float &pitchDeg, float &rollDeg) {
    // q_rel = q_offset_inv * q_current
    float w = offW * q0 - offX * q1 - offY * q2 - offZ * q3;
    float x = offW * q1 + offX * q0 + offY * q3 - offZ * q2;
    float y = offW * q2 - offX * q3 + offY * q0 + offZ * q1;
    float z = offW * q3 + offX * q2 - offY * q1 + offZ * q0;

    float roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    float sinp = 2.0f * (w * y - z * x);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    float pitch = asinf(sinp);
    float yaw = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));

    const float rad2deg = 57.295779513082320876f;
    yawDeg = yaw * rad2deg;
    pitchDeg = pitch * rad2deg;
    rollDeg = roll * rad2deg;
  }
};

MahonyFilter mahony;

// Axis mapping params from control_video_v6.py behavior
const float MAX_ANGLE = 45.0f;
float prDeadzone = 8.0f;
float prSensitivity = 1.0f;
float prExpo = 0.5f;
float yawDeadzone = 8.0f;
float yawSensitivity = 1.0f;
float yawExpo = 0.5f;

unsigned long lastImuUs = 0;

// ---------------- Utility ----------------
float constrainf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void trimLine(String &s) {
  s.trim();
  while (s.length() > 0 && (s[s.length() - 1] == '\r' || s[s.length() - 1] == '\n')) {
    s.remove(s.length() - 1);
  }
}

void uppercase(String &s) {
  for (size_t i = 0; i < s.length(); i++) s[i] = toupper(s[i]);
}

float angleToStick(float angleDeg, float deadzone, float sensitivity, float expo) {
  float sign = (angleDeg >= 0.0f) ? 1.0f : -1.0f;
  float mag = fabsf(angleDeg);
  if (mag < deadzone) return STICK_MID;

  float norm = (mag - deadzone) / (MAX_ANGLE - deadzone);
  norm = constrainf(norm, 0.0f, 1.0f);
  norm = constrainf(norm * sensitivity, 0.0f, 1.0f);

  float curved = norm * (1.0f - expo) + norm * norm * norm * expo;
  curved = constrainf(curved, 0.0f, 1.0f);

  return STICK_MID + sign * curved * (STICK_MAX - STICK_MID);
}

float flexDeflection(int raw, int idx) {
  float delta = raw - flexMean[idx];
  float thresh = FLEX_THRESH_STD * flexStd[idx];
  if (fabsf(delta) < thresh) return 0.0f;
  float signedDelta = delta - ((delta >= 0.0f) ? thresh : -thresh);
  return constrainf(signedDelta / flexNormScale, -1.0f, 1.0f);
}

void resetFlexCalibration() {
  for (int i = 0; i < 4; i++) {
    flexAcc[i] = 0;
    flexSqAcc[i] = 0;
    flexMean[i] = 512.0f;
    flexStd[i] = 20.0f;
  }
  flexCalCount = 0;
  flexCalibrated = false;
}

void updateFlexCalibration(int a3, int a2, int a1, int a0) {
  if (flexCalibrated) return;

  int vals[4] = {a3, a2, a1, a0};
  for (int i = 0; i < 4; i++) {
    flexAcc[i] += vals[i];
    flexSqAcc[i] += (long)vals[i] * (long)vals[i];
  }

  flexCalCount++;
  if (flexCalCount >= FLEX_REST_SAMPLES) {
    for (int i = 0; i < 4; i++) {
      float mean = (float)flexAcc[i] / (float)flexCalCount;
      float meanSq = (float)flexSqAcc[i] / (float)flexCalCount;
      float var = meanSq - mean * mean;
      if (var < 25.0f) var = 25.0f;
      flexMean[i] = mean;
      flexStd[i] = sqrtf(var);
    }
    flexCalibrated = true;
    Serial.println(F("FLEX CALIBRATION: done"));
  }
}

bool parseDroneIp() {
  droneAddrValid = droneAddr.fromString(droneIp);
  if (!droneAddrValid) {
    Serial.print(F("DRONE IP INVALID: "));
    Serial.println(droneIp);
  }
  return droneAddrValid;
}

bool startUdpIfNeeded() {
  if (udpStarted) return true;
  int ok = udp.begin(LOCAL_UDP_PORT);
  if (!ok) {
    if (millis() - lastUdpErrorMs > 1000) {
      lastUdpErrorMs = millis();
      Serial.println(F("UDP: begin failed"));
    }
    return false;
  }
  udpStarted = true;
  Serial.print(F("UDP: local port "));
  Serial.println(LOCAL_UDP_PORT);
  Serial.print(F("UDP: target "));
  Serial.print(droneIp);
  Serial.print(':');
  Serial.println(dronePort);

  // Harmless protocol poke used by WiFi-UAV apps before/with RC traffic.
  udp.beginPacket(droneAddr, dronePort);
  udp.write(START_STREAM, sizeof(START_STREAM));
  udp.endPacket();
  return true;
}

bool ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    return true;
  }

  wifiConnected = false;
  if (strlen(wifiSsid) == 0) return false;

  int status;
  if (strlen(wifiPass) > 0) status = WiFi.begin(wifiSsid, wifiPass);
  else status = WiFi.begin(wifiSsid);

  unsigned long t0 = millis();
  while (millis() - t0 < 12000) {
    if (status == WL_CONNECTED) {
      wifiConnected = true;
      Serial.print(F("WIFI: connected, IP="));
      Serial.println(WiFi.localIP());
      startUdpIfNeeded();
      return true;
    }
    delay(200);
    status = WiFi.status();
  }

  Serial.println(F("WIFI: connect failed"));
  return false;
}

void buildPacket(uint8_t *pkt,
                 uint8_t roll, uint8_t pitch, uint8_t throttle, uint8_t yaw,
                 uint8_t cmd, uint8_t headless,
                 uint16_t cc1, uint16_t cc2, uint16_t cc3,
                 bool somersault) {
  uint8_t controls[6];
  controls[0] = roll;
  controls[1] = pitch;
  controls[2] = throttle;
  controls[3] = yaw;
  controls[4] = cmd;
  controls[5] = somersault ? (uint8_t)(headless | SOMERSAULT_FLAG) : headless;

  uint8_t chk = 0;
  for (int i = 0; i < 6; i++) chk ^= controls[i];

  int idx = 0;
  memcpy(pkt + idx, HDR, sizeof(HDR)); idx += sizeof(HDR);

  pkt[idx++] = (uint8_t)(cc1 & 0xFF);
  pkt[idx++] = (uint8_t)((cc1 >> 8) & 0xFF);
  memcpy(pkt + idx, C1_SUFFIX, sizeof(C1_SUFFIX)); idx += sizeof(C1_SUFFIX);

  memcpy(pkt + idx, controls, sizeof(controls)); idx += sizeof(controls);
  memcpy(pkt + idx, CTRL_PAD, sizeof(CTRL_PAD)); idx += sizeof(CTRL_PAD);

  pkt[idx++] = chk;
  memcpy(pkt + idx, CKSUM_SFX, sizeof(CKSUM_SFX)); idx += sizeof(CKSUM_SFX);

  pkt[idx++] = (uint8_t)(cc2 & 0xFF);
  pkt[idx++] = (uint8_t)((cc2 >> 8) & 0xFF);
  memcpy(pkt + idx, C2_SUFFIX, sizeof(C2_SUFFIX)); idx += sizeof(C2_SUFFIX);

  pkt[idx++] = (uint8_t)(cc3 & 0xFF);
  pkt[idx++] = (uint8_t)((cc3 >> 8) & 0xFF);
  memcpy(pkt + idx, C3_SUFFIX, sizeof(C3_SUFFIX)); idx += sizeof(C3_SUFFIX);

  if (idx != PACKET_SIZE) {
    Serial.print(F("PACKET ERROR: size="));
    Serial.println(idx);
  }
}

void sendControlPacket(uint8_t roll, uint8_t pitch, uint8_t throttle, uint8_t yaw,
                       uint8_t cmd, bool somersault) {
  if (!ensureWiFi()) return;
  if (!droneAddrValid && !parseDroneIp()) return;
  if (!startUdpIfNeeded()) return;

  uint8_t pkt[PACKET_SIZE];
  uint8_t hless = headlessEnabled ? HEADLESS_ON : HEADLESS_OFF;
  buildPacket(pkt, roll, pitch, throttle, yaw, cmd, hless, c1, c2, c3, somersault);

  int ok = udp.beginPacket(droneAddr, dronePort);
  if (ok == 1) {
    udp.write(pkt, PACKET_SIZE);
    ok = udp.endPacket();
  }
  if (ok != 1 && millis() - lastUdpErrorMs > 1000) {
    lastUdpErrorMs = millis();
    Serial.println(F("UDP: send failed"));
  }

  if (ok == 1) {
    txCount++;
    if (debugTxEnabled && millis() - lastTxReportMs >= 1000) {
      lastTxReportMs = millis();
      Serial.print(F("TX/s packets total="));
      Serial.print(txCount);
      Serial.print(F(" cmd="));
      Serial.print(cmd);
      Serial.print(F(" sticks R/P/T/Y="));
      Serial.print(roll);
      Serial.print('/');
      Serial.print(pitch);
      Serial.print('/');
      Serial.print(throttle);
      Serial.print('/');
      Serial.println(yaw);
    }
  }

  c1++;
  c2++;
  c3++;
}

uint8_t consumeCmdFlag() {
  if (repeatCmdLeft > 0) {
    repeatCmdLeft--;
    return repeatCmd;
  }

  if (flagTakeoff) {
    flagTakeoff = false;
    repeatCmd = CMD_TAKEOFF;
    repeatCmdLeft = ONE_SHOT_REPEAT_PACKETS - 1;
    return CMD_TAKEOFF;
  }
  if (flagStop) {
    flagStop = false;
    repeatCmd = CMD_STOP;
    repeatCmdLeft = ONE_SHOT_REPEAT_PACKETS - 1;
    return CMD_STOP;
  }
  if (flagLand) {
    flagLand = false;
    repeatCmd = CMD_LAND;
    repeatCmdLeft = ONE_SHOT_REPEAT_PACKETS - 1;
    return CMD_LAND;
  }
  if (flagCalibrate) {
    flagCalibrate = false;
    repeatCmd = CMD_CALIBRATE;
    repeatCmdLeft = ONE_SHOT_REPEAT_PACKETS - 1;
    return CMD_CALIBRATE;
  }
  if (flagCamUp) {
    flagCamUp = false;
    repeatCmd = CMD_CAM_UP;
    repeatCmdLeft = ONE_SHOT_REPEAT_PACKETS - 1;
    return CMD_CAM_UP;
  }
  if (flagCamDown) {
    flagCamDown = false;
    repeatCmd = CMD_CAM_DOWN;
    repeatCmdLeft = ONE_SHOT_REPEAT_PACKETS - 1;
    return CMD_CAM_DOWN;
  }
  return CMD_NONE;
}

void startFlip(FlipDir dir) {
  if (flipActive) {
    Serial.println(F("FLIP: already in progress"));
    return;
  }
  flipDir = dir;
  flipActive = true;
  flipPacketsLeft = 20;
  flipSettlePacketsLeft = 10;
  Serial.println(F("FLIP: armed"));
}

void startSmartLand() {
  if (smartLandActive) return;
  smartLandActive = true;
  smartLandStartMs = millis();
  Serial.println(F("LAND: smart descent started"));
}

void applyImuAndFlex() {
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  int a3 = analogRead(FLEX_A3);
  int a2 = analogRead(FLEX_A2);
  int a1 = analogRead(FLEX_A1);
  int a0 = analogRead(FLEX_A0);

  updateFlexCalibration(a3, a2, a1, a0);

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    unsigned long nowUs = micros();
    float dt = 0.01f;
    if (lastImuUs != 0) {
      dt = (float)(nowUs - lastImuUs) / 1000000.0f;
      dt = constrainf(dt, 0.001f, 0.03f);
    }
    lastImuUs = nowUs;

    mahony.addGyroCalSample(gx, gy, gz, 150);
    mahony.update(ax, ay, az, gx, gy, gz, dt);
  }

  if (imuEnabled) {
    float yawDeg = 0, pitchDeg = 0, rollDeg = 0;
    mahony.getEulerRelative(yawDeg, pitchDeg, rollDeg);

    stickYaw = angleToStick(yawDeg, yawDeadzone, yawSensitivity, yawExpo);
    stickPitch = angleToStick(pitchDeg, prDeadzone, prSensitivity, prExpo);
    stickRoll = angleToStick(rollDeg, prDeadzone, prSensitivity, prExpo);

    if (flexCalibrated) {
      float d2 = flexDeflection(a2, 1);
      float d3 = flexDeflection(a3, 0);
      float net = constrainf(d2 - d3, -1.0f, 1.0f);
      float e = prExpo * 0.6f;
      float sgn = (net >= 0.0f) ? 1.0f : -1.0f;
      float mag = fabsf(net);
      float curved = mag * (1.0f - e) + mag * mag * mag * e;
      float raw = STICK_MID + sgn * curved * (STICK_MAX - STICK_MID);
      raw = constrainf(raw, STICK_MIN, STICK_MAX);
      stickThrottle += (raw - stickThrottle) * throttleAlpha;
    }
  }

  if (streamingEnabled && millis() - lastSensorMs >= 20) {
    lastSensorMs = millis();
    float yawDeg = 0, pitchDeg = 0, rollDeg = 0;
    mahony.getEulerRelative(yawDeg, pitchDeg, rollDeg);

    Serial.print(millis() / 1000.0f, 3);
    Serial.print(',');
    Serial.print(a3);
    Serial.print(',');
    Serial.print(a2);
    Serial.print(',');
    Serial.print(a1);
    Serial.print(',');
    Serial.print(a0);
    Serial.print(',');
    Serial.print(yawDeg, 2);
    Serial.print(',');
    Serial.print(pitchDeg, 2);
    Serial.print(',');
    Serial.print(rollDeg, 2);
    Serial.print(',');
    Serial.print(stickThrottle, 1);
    Serial.print(',');
    Serial.print(stickYaw, 1);
    Serial.print(',');
    Serial.print(stickPitch, 1);
    Serial.print(',');
    Serial.println(stickRoll, 1);
  }
}

void runControlTick() {
  unsigned long nowMs = millis();
  unsigned long intervalMs = (unsigned long)(1000.0f / controlRateHz);
  if (intervalMs < 5) intervalMs = 5;

  if (nowMs - lastControlMs < intervalMs) return;
  lastControlMs = nowMs;

  // Smart landing emulates gradual descent when no altitude telemetry is available.
  if (smartLandActive) {
    unsigned long elapsed = nowMs - smartLandStartMs;
    if (elapsed < SMART_LAND_MS) {
      float frac = (float)elapsed / (float)SMART_LAND_MS;
      float thr = stickThrottle - frac * (stickThrottle - STICK_MIN);
      sendControlPacket(STICK_MID, STICK_MID, (uint8_t)constrainf(thr, STICK_MIN, STICK_MAX),
                        (uint8_t)constrainf(stickYaw, STICK_MIN, STICK_MAX), CMD_NONE, false);
      return;
    }

    for (int i = 0; i < 8; i++) {
      sendControlPacket(STICK_MID, STICK_MID, STICK_MIN,
                        (uint8_t)constrainf(stickYaw, STICK_MIN, STICK_MAX), CMD_LAND, false);
      delay(20);
    }
    smartLandActive = false;
    Serial.println(F("LAND: completed"));
    return;
  }

  if (flipActive) {
    uint8_t fRoll = STICK_MID;
    uint8_t fPitch = STICK_MID;

    if (flipDir == FLIP_FWD) fPitch = STICK_MAX;
    else if (flipDir == FLIP_BACK) fPitch = STICK_MIN;
    else if (flipDir == FLIP_LEFT) fRoll = STICK_MIN;
    else if (flipDir == FLIP_RIGHT) fRoll = STICK_MAX;

    if (flipPacketsLeft > 0) {
      sendControlPacket(fRoll, fPitch,
                        (uint8_t)constrainf(stickThrottle, STICK_MIN, STICK_MAX),
                        (uint8_t)constrainf(stickYaw, STICK_MIN, STICK_MAX),
                        CMD_NONE, true);
      flipPacketsLeft--;
      return;
    }

    if (flipSettlePacketsLeft > 0) {
      sendControlPacket(STICK_MID, STICK_MID,
                        (uint8_t)constrainf(stickThrottle, STICK_MIN, STICK_MAX),
                        (uint8_t)constrainf(stickYaw, STICK_MIN, STICK_MAX),
                        CMD_NONE, false);
      flipSettlePacketsLeft--;
      return;
    }

    flipActive = false;
    flipDir = FLIP_NONE;
    Serial.println(F("FLIP: done"));
  }

  uint8_t cmd = consumeCmdFlag();
  sendControlPacket((uint8_t)constrainf(stickRoll, STICK_MIN, STICK_MAX),
                    (uint8_t)constrainf(stickPitch, STICK_MIN, STICK_MAX),
                    (uint8_t)constrainf(stickThrottle, STICK_MIN, STICK_MAX),
                    (uint8_t)constrainf(stickYaw, STICK_MIN, STICK_MAX),
                    cmd, false);
}

void printHelp() {
  Serial.println(F("\n=== K417 Direct Glove Controller ==="));
  Serial.println(F("WIFI <ssid> <pass>   : set WiFi credentials"));
  Serial.println(F("CONNECT              : connect/reconnect WiFi"));
  Serial.println(F("DRONEIP <ip> [port]  : set drone endpoint"));
  Serial.println(F("T/TAKEOFF            : takeoff"));
  Serial.println(F("L/LAND               : smart land"));
  Serial.println(F("SPACE/STOP           : emergency stop"));
  Serial.println(F("H/HEADLESS           : toggle headless"));
  Serial.println(F("C/CAL                : drone calibrate command"));
  Serial.println(F("O/ZERO               : set IMU zero offset"));
  Serial.println(F("R/RECAL/RECALIB      : reset gyro+flex calibration"));
  Serial.println(F("PGUP/CAMUP           : camera tilt up"));
  Serial.println(F("PGDN/CAMDOWN         : camera tilt down"));
  Serial.println(F("FLIP FWD|BACK|LEFT|RIGHT"));
  Serial.println(F("IMU ON|OFF           : enable/disable IMU stick updates"));
  Serial.println(F("STREAM ON|OFF        : serial sensor stream"));
  Serial.println(F("DEBUGTX ON|OFF       : packet tx debug (1 line/s)"));
  Serial.println(F("RATE <hz>            : packet rate (10..80)"));
  Serial.println(F("DZ <v>               : pitch/roll deadzone"));
  Serial.println(F("YDZ <v>              : yaw deadzone"));
  Serial.println(F("SENS <v> / YSENS <v> : sensitivities"));
  Serial.println(F("EXPO <v> / YEXPO <v> : expo 0..1"));
  Serial.println(F("THR_ALPHA <v>        : throttle smooth 0..1"));
  Serial.println(F("STATUS               : show state"));
  Serial.println(F("HELP                 : this help"));
}

void printStatus() {
  Serial.println(F("\n--- STATUS ---"));
  Serial.print(F("WiFi: "));
  Serial.println((WiFi.status() == WL_CONNECTED) ? F("CONNECTED") : F("DISCONNECTED"));
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("Local IP: "));
    Serial.println(WiFi.localIP());
  }
  Serial.print(F("Drone: "));
  Serial.print(droneIp);
  Serial.print(':');
  Serial.println(dronePort);
  Serial.print(F("Drone IP parsed: "));
  Serial.println(droneAddrValid ? F("YES") : F("NO"));

  Serial.print(F("Rate Hz: "));
  Serial.println(controlRateHz, 1);
  Serial.print(F("IMU: "));
  Serial.println(imuEnabled ? F("ON") : F("OFF"));
  Serial.print(F("Headless: "));
  Serial.println(headlessEnabled ? F("ON") : F("OFF"));
  Serial.print(F("Flex cal: "));
  Serial.println(flexCalibrated ? F("DONE") : F("RUNNING"));

  Serial.print(F("Sticks T/Y/P/R: "));
  Serial.print(stickThrottle, 1);
  Serial.print('/');
  Serial.print(stickYaw, 1);
  Serial.print('/');
  Serial.print(stickPitch, 1);
  Serial.print('/');
  Serial.println(stickRoll, 1);
  Serial.print(F("TX packets: "));
  Serial.println(txCount);
}

void handleCommand(const String &cmdRaw) {
  String cmd = cmdRaw;
  trimLine(cmd);
  if (cmd.length() == 0) return;

  String u = cmd;
  uppercase(u);

  if (u == "HELP" || u == "?") {
    printHelp();
    return;
  }

  if (u == "STATUS") {
    printStatus();
    return;
  }

  if (u == "CONNECT") {
    ensureWiFi();
    return;
  }

  if (u.startsWith("WIFI ")) {
    int p1 = cmd.indexOf(' ');
    int p2 = cmd.indexOf(' ', p1 + 1);
    if (p2 < 0) {
      Serial.println(F("WIFI format: WIFI <ssid> <pass>"));
      return;
    }
    String s = cmd.substring(p1 + 1, p2);
    String p = cmd.substring(p2 + 1);
    s.toCharArray(wifiSsid, sizeof(wifiSsid));
    p.toCharArray(wifiPass, sizeof(wifiPass));
    Serial.println(F("WIFI: credentials updated"));
    return;
  }

  if (u.startsWith("DRONEIP ")) {
    int p1 = cmd.indexOf(' ');
    int p2 = cmd.indexOf(' ', p1 + 1);

    String ipPart;
    if (p2 < 0) {
      ipPart = cmd.substring(p1 + 1);
    } else {
      ipPart = cmd.substring(p1 + 1, p2);
      String portPart = cmd.substring(p2 + 1);
      int prt = portPart.toInt();
      if (prt > 0 && prt <= 65535) dronePort = (uint16_t)prt;
    }

    ipPart.toCharArray(droneIp, sizeof(droneIp));
    parseDroneIp();

    Serial.print(F("DRONE ENDPOINT: "));
    Serial.print(droneIp);
    Serial.print(':');
    Serial.println(dronePort);
    return;
  }

  if (u.startsWith("DEBUGTX ")) {
    String v = u.substring(8);
    trimLine(v);
    if (v == "ON") debugTxEnabled = true;
    else if (v == "OFF") debugTxEnabled = false;
    Serial.print(F("DEBUGTX: "));
    Serial.println(debugTxEnabled ? F("ON") : F("OFF"));
    return;
  }

  if (u == "T" || u == "TAKEOFF") {
    flagTakeoff = true;
    Serial.println(F("CMD: TAKEOFF"));
    return;
  }

  if (u == "L" || u == "LAND") {
    startSmartLand();
    return;
  }

  if (u == "SPACE" || u == "STOP") {
    flagStop = true;
    smartLandActive = false;
    flipActive = false;
    Serial.println(F("CMD: STOP"));
    return;
  }

  if (u == "H" || u == "HEADLESS") {
    headlessEnabled = !headlessEnabled;
    Serial.print(F("HEADLESS: "));
    Serial.println(headlessEnabled ? F("ON") : F("OFF"));
    return;
  }

  if (u == "C" || u == "CAL") {
    flagCalibrate = true;
    Serial.println(F("CMD: CALIBRATE"));
    return;
  }

  if (u == "O" || u == "ZERO") {
    mahony.captureOffset();
    Serial.println(F("IMU: zero captured"));
    return;
  }

  if (u == "F5" || u == "R" || u == "RECAL" || u == "RECALIB") {
    mahony.resetCalibration();
    resetFlexCalibration();
    Serial.println(F("IMU/FLEX: recalibration started"));
    return;
  }

  if (u == "PGUP" || u == "CAMUP") {
    flagCamUp = true;
    Serial.println(F("CMD: CAM UP"));
    return;
  }

  if (u == "PGDN" || u == "CAMDOWN") {
    flagCamDown = true;
    Serial.println(F("CMD: CAM DOWN"));
    return;
  }

  if (u.startsWith("FLIP ")) {
    String d = u.substring(5);
    trimLine(d);
    if (d == "FWD" || d == "FORWARD") startFlip(FLIP_FWD);
    else if (d == "BACK" || d == "BACKWARD") startFlip(FLIP_BACK);
    else if (d == "LEFT") startFlip(FLIP_LEFT);
    else if (d == "RIGHT") startFlip(FLIP_RIGHT);
    else Serial.println(F("FLIP usage: FLIP FWD|BACK|LEFT|RIGHT"));
    return;
  }

  if (u.startsWith("IMU ")) {
    String v = u.substring(4);
    trimLine(v);
    if (v == "ON") imuEnabled = true;
    else if (v == "OFF") imuEnabled = false;
    Serial.print(F("IMU: "));
    Serial.println(imuEnabled ? F("ON") : F("OFF"));
    return;
  }

  if (u.startsWith("STREAM ")) {
    String v = u.substring(7);
    trimLine(v);
    if (v == "ON") streamingEnabled = true;
    else if (v == "OFF") streamingEnabled = false;
    Serial.print(F("STREAM: "));
    Serial.println(streamingEnabled ? F("ON") : F("OFF"));
    return;
  }

  if (u.startsWith("RATE ")) {
    float hz = cmd.substring(5).toFloat();
    if (hz >= 10.0f && hz <= 80.0f) {
      controlRateHz = hz;
      Serial.print(F("RATE: "));
      Serial.println(controlRateHz, 1);
    }
    return;
  }

  if (u.startsWith("DZ ")) {
    prDeadzone = constrainf(cmd.substring(3).toFloat(), 0.0f, 20.0f);
    Serial.print(F("DZ: "));
    Serial.println(prDeadzone, 2);
    return;
  }

  if (u.startsWith("YDZ ")) {
    yawDeadzone = constrainf(cmd.substring(4).toFloat(), 0.0f, 20.0f);
    Serial.print(F("YDZ: "));
    Serial.println(yawDeadzone, 2);
    return;
  }

  if (u.startsWith("SENS ")) {
    prSensitivity = constrainf(cmd.substring(5).toFloat(), 0.2f, 2.5f);
    Serial.print(F("SENS: "));
    Serial.println(prSensitivity, 2);
    return;
  }

  if (u.startsWith("YSENS ")) {
    yawSensitivity = constrainf(cmd.substring(6).toFloat(), 0.2f, 2.5f);
    Serial.print(F("YSENS: "));
    Serial.println(yawSensitivity, 2);
    return;
  }

  if (u.startsWith("EXPO ")) {
    prExpo = constrainf(cmd.substring(5).toFloat(), 0.0f, 1.0f);
    Serial.print(F("EXPO: "));
    Serial.println(prExpo, 2);
    return;
  }

  if (u.startsWith("YEXPO ")) {
    yawExpo = constrainf(cmd.substring(6).toFloat(), 0.0f, 1.0f);
    Serial.print(F("YEXPO: "));
    Serial.println(yawExpo, 2);
    return;
  }

  if (u.startsWith("THR_ALPHA ")) {
    throttleAlpha = constrainf(cmd.substring(10).toFloat(), 0.01f, 1.0f);
    Serial.print(F("THR_ALPHA: "));
    Serial.println(throttleAlpha, 3);
    return;
  }

  Serial.println(F("Unknown command. Use HELP"));
}

void readSerialCommands() {
  static String line = "";
  static unsigned long lastCharMs = 0;

  while (Serial.available()) {
    char ch = (char)Serial.read();
    lastCharMs = millis();

    if (ch == '\r' || ch == '\n') {
      if (line.length() > 0) {
        handleCommand(line);
        line = "";
      }
    } else {
      line += ch;
      if (line.length() > 120) {
        line = "";
      }
    }
  }

  // If monitor is set to "No line ending", flush after a short idle gap.
  if (line.length() > 0 && (millis() - lastCharMs) > 120) {
    handleCommand(line);
    line = "";
  }
}

void setup() {
  pinMode(FLEX_A3, INPUT);
  pinMode(FLEX_A2, INPUT);
  pinMode(FLEX_A1, INPUT);
  pinMode(FLEX_A0, INPUT);

  Serial.begin(115200);
  delay(1500);

  Serial.println(F("Booting K417 direct glove controller..."));

  if (!IMU.begin()) {
    Serial.println(F("ERROR: IMU init failed"));
  } else {
    Serial.println(F("IMU: ready"));
  }

  parseDroneIp();
  resetFlexCalibration();
  mahony.resetCalibration();

  printHelp();

  Serial.println(F("Tip: set WiFi with WIFI <ssid> <pass>, then CONNECT"));
}

void loop() {
  readSerialCommands();

  // Keep WiFi alive if credentials are known.
  if (strlen(wifiSsid) > 0 && (millis() - lastStatusMs > 3000)) {
    lastStatusMs = millis();
    ensureWiFi();
  }

  applyImuAndFlex();
  runControlTick();
}
