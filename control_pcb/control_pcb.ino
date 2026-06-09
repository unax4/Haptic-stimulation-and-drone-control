// Fusion CORREGIDO: control serial + streaming + MAX5413 + HV2701
// Solución: Pausa streaming durante operaciones SPI críticas

#include <Arduino_LSM6DSOX.h>

const int POT_CS   = 10;
const int DATA_PIN = 9;
const int CLK_PIN  = 8;
const int HV_LE    = 7;
const int HV_CLR   = 6;
const int OUT_PIN  = 12;

//const int OUT_PIN  = 12;
//const int POT_CS   = 10;
//const int DATA_PIN = 9;
//const int CLK_PIN  = 8;
//const int HV_LE    = 7;
//const int HV_CLR   = 6;

int potValue = 255;
uint16_t hvState = 0x0000;

// Flag para deshabilitar streaming temporalmente durante operaciones SPI
volatile bool spi_busy = false;

// Generador
unsigned long singlePulse_ms = 1000;
int burstCount = 5;
unsigned long burstPulse_ms = 50;
unsigned long burstPause_ms = 100;
float freq_Hz = 100.0;
unsigned long pulseWidth_us = 400;
unsigned long trainDuration_ms = 2000;

// Trenes preconfigurados
const float train1_freq_Hz = 100.0;
const unsigned long train1_pw_us = 400;
const unsigned long train1_duration_ms = 2000;

const float train2_freq_Hz = 80.0;
const unsigned long train2_pw_us = 800;
const unsigned long train2_duration_ms = 2000;

// Analógicos para streaming
const int analogPins[4] = {A3, A2, A1, A0};

// Control de streaming
bool streamingEnabled = true; // Permite activar/desactivar streaming

// Posiciones
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

const char* names[21] = {
  "initial (0000000000000000)",
  "thumb 1 (0100000000010101)",
  "thumb 2 (0100000000011010)",
  "index_top 1 (0100000000100101)",
  "index_top 2 (0100000000101010)",
  "index_bottom 1 (0100001000000101)",
  "index_bottom 2 (0100001000001010)",
  "middle_top 1 (0100000001000101)",
  "middle_top 2 (0100000001001010)",
  "middle_bottom 1 (0100010000000101)",
  "middle_bottom 2 (0100010000001010)",
  "ring_top 1 (1000000010000101)",
  "ring_top 2 (1000000010001010)",
  "ring_bottom 1 (1000100000000101)",
  "ring_bottom 2 (1000100000001010)",
  "pinky_top 1 (1000000100000101)",
  "pinky_top 2 (1000000100001010)",
  "pinky_bottom 1 (1001000000000101)",
  "pinky_bottom 2 (1001000000001010)",
  "palm 1 (1110000000000101)",
  "palm 2 (1110000000001010)"
};

// ---- Pulsos no bloqueantes ----
enum PulseMode { PM_IDLE=0, PM_SINGLE, PM_BURST, PM_TRAIN };
PulseMode pulseMode = PM_IDLE;

// SINGLE
unsigned long single_start_ms = 0, single_duration_ms = 0;

// BURST
int burst_total = 0, burst_index = 0;
unsigned long burst_on_ms = 0, burst_off_ms = 0, burst_last_ms = 0;
bool burst_state_on = false;

// TRAIN
unsigned long train_start_ms = 0, train_duration_ms_running = 0;
unsigned long train_period_us = 0, train_pw_us = 0;
unsigned long train_next_toggle_us = 0;
bool train_state_on = false;

// Prototipos
void sendToHV2701(uint16_t data);
void setPosition(int n);
void setPot(byte v);
float getPotResistance();
void printHelp();
void printStatus();
String getStimulatedOutputs(uint16_t state);

void startSingle(unsigned long d);
void startBurst(int n, unsigned long on_ms, unsigned long off_ms);
void startTrain(float f, unsigned long pw_us, unsigned long total_ms);
void stopPulses();
void updatePulses();

////Offset del Giroscopio
float xoff=0.39;
float yoff=-0.35;
float zoff=-0.36;

void setup() {
  pinMode(OUT_PIN, OUTPUT); digitalWrite(OUT_PIN, LOW);
  pinMode(CLK_PIN, OUTPUT); pinMode(DATA_PIN, OUTPUT);
  pinMode(POT_CS, OUTPUT); pinMode(HV_LE, OUTPUT); pinMode(HV_CLR, OUTPUT);

  digitalWrite(CLK_PIN, LOW); digitalWrite(DATA_PIN, LOW);
  digitalWrite(POT_CS, HIGH); digitalWrite(HV_LE, HIGH); digitalWrite(HV_CLR, LOW);

  Serial.begin(115200);
  delay(2000);
  
  // INICIALIZAR IMU
  if (!IMU.begin()) {
    Serial.println(F("ERROR: No se pudo inicializar IMU LSM6DSOX"));
  }
  
  Serial.println(F("\n=== Generador + Pot MAX5413 + HV2701 (CORREGIDO) ==="));
  printHelp();

  setPot(potValue);
  setPosition(0);
}

void loop() {
  // 1) Procesar comandos (PRIORIDAD MÁXIMA)
  if (Serial.available()) {
    spi_busy = true; // Bloquear streaming durante procesamiento de comando
    
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.length() > 0) {
      cmd.toUpperCase();

      if (cmd == "STOP") { 
        stopPulses(); 
        Serial.println(F("STOP -> Pulses stopped")); 
      }
      
      else if (cmd == "STREAM_ON") {
        streamingEnabled = true;
        Serial.println(F("Streaming ENABLED"));
      }
      
      else if (cmd == "STREAM_OFF") {
        streamingEnabled = false;
        Serial.println(F("Streaming DISABLED"));
      }

      else if (cmd.startsWith("SW")) {
        int sw = cmd.substring(2).toInt();
        if (sw >= 0 && sw <= 15) {
          hvState ^= (1 << sw);
          sendToHV2701(hvState);
          Serial.print(F("SW")); Serial.print(sw); Serial.print(F(" toggled -> HV: "));
          for (int i = 15; i >= 0; i--) Serial.print((hvState >> i) & 1);
          Serial.println();
        }
      }

      else if (cmd.startsWith("M") && cmd.length() >= 2) {
        int num = cmd.substring(1).toInt();
        if (num >= 0 && num <= 20) {
          setPosition(num);
          Serial.print(F("Posición M")); Serial.print(num); Serial.print(F(" -> ")); 
          Serial.println(names[num]);
          Serial.print(F("Stimulated: "));
          Serial.println(getStimulatedOutputs(hvState));
        }
      }

      else if (cmd.length() == 16) {
        bool ok = true; uint16_t val = 0;
        for (int i = 0; i < 16; i++) {
          if (cmd[i] == '1') val |= (1 << (15 - i));
          else if (cmd[i] != '0') { ok = false; break; }
        }
        if (ok) { 
          hvState = val; 
          sendToHV2701(hvState); 
          Serial.print(F("HV2701 -> ")); Serial.println(cmd); 
        }
        else Serial.println(F("Error: solo 0 y 1"));
      }

      else if (cmd == "?" || cmd == "HELP") printHelp();
      
      else if (cmd.startsWith("P")) {
        int v = cmd.substring(1).toInt();
        if (v >= 0 && v <= 255) { 
          potValue = v; 
          setPot(v);
          float intensidad_mA_real = 1/((0.0144 * potValue + 0.0815));
          Serial.print(F("Pot -> ")); Serial.print(potValue); Serial.print(F("/255 (~"));
          Serial.print(F("Ω) -> Intensidad entregada HV2701 ≈ "));
          Serial.print(intensidad_mA_real, 3); Serial.println(F(" mA"));
          
        }
      }
      
      else if (cmd == "S") { 
        startSingle(singlePulse_ms); 
        Serial.println(F("Single started")); 
      }
      else if (cmd.startsWith("SD")) { 
        singlePulse_ms = cmd.substring(2).toInt(); 
        Serial.print(F("SD=")); Serial.println(singlePulse_ms); 
      }
      
      else if (cmd == "B") { 
        startBurst(burstCount, burstPulse_ms, burstPause_ms); 
        Serial.println(F("Burst started")); 
      }
      else if (cmd.startsWith("BC")) { 
        burstCount = cmd.substring(2).toInt(); 
        Serial.print(F("BC=")); Serial.println(burstCount); 
      }
      else if (cmd.startsWith("BP")) { 
        burstPulse_ms = cmd.substring(2).toInt(); 
        Serial.print(F("BP=")); Serial.println(burstPulse_ms); 
      }
      else if (cmd.startsWith("BO")) { 
        burstPause_ms = cmd.substring(2).toInt(); 
        Serial.print(F("BO=")); Serial.println(burstPause_ms); 
      }
      
      else if (cmd == "T") { 
        startTrain(freq_Hz, pulseWidth_us, trainDuration_ms); 
        Serial.println(F("Train started")); 
      }
      else if (cmd == "T1") {
        startTrain(train1_freq_Hz, train1_pw_us, train1_duration_ms);
        Serial.println(F("Train T1 started"));
      }
      else if (cmd == "T2") {
        startTrain(train2_freq_Hz, train2_pw_us, train2_duration_ms);
        Serial.println(F("Train T2 started"));
      }
      else if (cmd.startsWith("F")) { 
        freq_Hz = cmd.substring(1).toFloat(); 
        Serial.print(F("F=")); Serial.println(freq_Hz); 
      }
      else if (cmd.startsWith("W")) { 
        pulseWidth_us = cmd.substring(1).toInt(); 
        Serial.print(F("W=")); Serial.println(pulseWidth_us); 
      }
      else if (cmd.startsWith("TD")) { 
        trainDuration_ms = cmd.substring(2).toInt(); 
        Serial.print(F("TD=")); Serial.println(trainDuration_ms); 
      }
      
      else if (cmd == "STATUS") printStatus();
      
      else Serial.println(F("Comando desconocido -> ? para ayuda"));
    }
    
    spi_busy = false; // Liberar streaming
    delay(5); // Pequeña pausa para estabilizar
  }

  // 2) Streaming SOLO si no hay operaciones SPI en curso
  if (streamingEnabled && !spi_busy) {
    static unsigned long lastStream = 0;
    if (millis() - lastStream >= 10) { // ~100 Hz streaming
      lastStream = millis();
      
      int analogRaw[4];
      for (int i = 0; i < 4; i++) analogRaw[i] = analogRead(analogPins[i]);
      
      float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
      if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        ax += 0.025;
        ay += 0.01;
        az += 0.01 + 0.01 * abs(ay);
        IMU.readGyroscope(gx, gy, gz);
        gx -= xoff;
        gy -= yoff;
        gz -= zoff;
      }
      
      unsigned long t = millis();
      Serial.print(t / 1000.0, 3); Serial.print(',');
      Serial.print(analogRaw[0]); Serial.print(','); 
      Serial.print(analogRaw[1]); Serial.print(',');
      Serial.print(analogRaw[2]); Serial.print(','); 
      Serial.print(analogRaw[3]); Serial.print(',');
      Serial.print(ax,3); Serial.print(','); 
      Serial.print(ay,3); Serial.print(','); 
      Serial.print(az,3); Serial.print(',');
      Serial.print(gx,3); Serial.print(','); 
      Serial.print(gy,3); Serial.print(','); 
      Serial.println(gz,3);
    }
  }

  // 3) Actualiza pulsos (independiente de SPI)
  updatePulses();
}


void pulseClock() {
  digitalWrite(CLK_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(CLK_PIN, LOW);
  delayMicroseconds(1);
}

void shiftBits(uint32_t data, int count) {
  for (int i = count - 1; i >= 0; i--) {
    digitalWrite(DATA_PIN, (data >> i) & 0x01);
    pulseClock();
  }
}

// --------------------
// sendToHV2701 con protección
// --------------------
void sendToHV2701(uint16_t data) {
  spi_busy = true;
  digitalWrite(POT_CS, HIGH); 
  digitalWrite(HV_LE, LOW); 
  
  shiftBits(data, 16); // Envía los 16 bits de un tirón

  digitalWrite(HV_LE, HIGH); delayMicroseconds(2); digitalWrite(HV_LE, LOW);
  spi_busy = false;
}

void setPot(byte value) {

  spi_busy = true;

  digitalWrite(HV_LE, HIGH);   // evita latch accidental del HV2701
  digitalWrite(POT_CS, LOW);
  // --- Bit de comando (P1) ---
  digitalWrite(DATA_PIN, 1);   // command = 1  -> Wiper1
  pulseClock();

  // --- 8 bits del valor ---
  for (int i = 7; i >= 0; i--) {
    digitalWrite(DATA_PIN, (value >> i) & 1);
    pulseClock();
  }

  digitalWrite(POT_CS, HIGH);

  spi_busy = false;
}

// --------------------
void setPosition(int n) {
  hvState = positions[n];
  sendToHV2701(hvState);
}


float getPotResistance() { 
  return (potValue / 255.0) * 10000.0; 
}

String getStimulatedOutputs(uint16_t state) {
  String out = "";

  // SW4..SW13 (interno) corresponden a Ch1..Ch10
  for (int i = 4; i <= 13; i++) {
    if ((state >> i) & 0x01) {
      if (out.length() > 0) out += "+";
      out += "Ch";
      out += String(i - 3);
    }
  }

  // SW14/SW15 (interno) -> Base1/Base2
  if ((state >> 14) & 0x01) {
    if (out.length() > 0) out += "+";
    out += "Base1";
  }
  if ((state >> 15) & 0x01) {
    if (out.length() > 0) out += "+";
    out += "Base2";
  }

  if (out.length() == 0) out = "none";
  return out;
}

void printHelp() {
  Serial.println(F("\n--- Comandos Rápidos ---"));
  Serial.println(F("M0..M20 -> posiciones"));
  Serial.println(F("SW0..SW15 -> toggle switch"));
  Serial.println(F("16 bits -> valor binario SW15..SW0"));
  Serial.println(F("P0..P255 -> pot (corriente)"));
  Serial.println(F("S, SDxxx -> single pulse"));
  Serial.println(F("B, BCnn, BPxx, BOxx -> burst"));
  Serial.println(F("T, Fxx, Wxx, TDxx -> train"));
  Serial.println(F("T1 -> 100Hz, 400us, 2000ms"));
  Serial.println(F("T2 -> 80Hz, 800us, 2000ms"));
  Serial.println(F("STOP -> corta pulsos"));
  Serial.println(F("STREAM_ON / STREAM_OFF -> control streaming"));
  Serial.println(F("STATUS, ?, HELP\n"));
}

void printStatus() {
  Serial.println(F("\n--- Estado ---"));
  int current = 0;
  for (int i = 0; i < 21; i++) if (hvState == positions[i]) current = i;
  Serial.print(F("Posición rápida: M")); Serial.println(current);
  Serial.print(F("HV bin: "));
  for (int i = 15; i >= 0; i--) Serial.print((hvState >> i) & 1);
  Serial.println();
  Serial.print(F("Pot: ")); Serial.print(potValue); Serial.print(F(" (~"));
  Serial.print(getPotResistance(),1); Serial.println(F("Ω)"));
  Serial.print(F("PulseMode: ")); Serial.println((int)pulseMode);
  Serial.print(F("Streaming: ")); Serial.println(streamingEnabled ? F("ON") : F("OFF"));
}

// ---------- Pulsos no bloqueantes ----------
void stopPulses() {
  pulseMode = PM_IDLE;
  digitalWrite(OUT_PIN, LOW);
  burst_index = 0; 
  burst_state_on = false; 
  train_state_on = false;
}

void startSingle(unsigned long d) {
  stopPulses();
  single_start_ms = millis(); 
  single_duration_ms = d;
  digitalWrite(OUT_PIN, HIGH); 
  pulseMode = PM_SINGLE;
}

void startBurst(int n, unsigned long on_ms, unsigned long off_ms) {
  stopPulses();
  burst_total = max(0, n);
  burst_index = 0; 
  burst_on_ms = on_ms; 
  burst_off_ms = off_ms;
  burst_last_ms = millis(); 
  burst_state_on = true;
  digitalWrite(OUT_PIN, HIGH); 
  pulseMode = PM_BURST;
}

void startTrain(float f, unsigned long pw_us, unsigned long total_ms) {
  if (f <= 0 || pw_us == 0) return;
  stopPulses();
  train_period_us = (unsigned long)round(1000000.0f / f);
  train_pw_us = pw_us;
  train_start_ms = millis(); 
  train_duration_ms_running = total_ms;
  train_next_toggle_us = micros() + pw_us; 
  train_state_on = true;
  digitalWrite(OUT_PIN, HIGH); 
  pulseMode = PM_TRAIN;
}

void updatePulses() {
  unsigned long now_ms = millis();
  unsigned long now_us = micros();

  switch (pulseMode) {
    case PM_IDLE: 
      break;

    case PM_SINGLE:
      if (now_ms - single_start_ms >= single_duration_ms) {
        digitalWrite(OUT_PIN, LOW); 
        pulseMode = PM_IDLE;
      }
      break;

    case PM_BURST:
      if (burst_index >= burst_total) { 
        digitalWrite(OUT_PIN, LOW); 
        pulseMode = PM_IDLE; 
        break; 
      }
      if (burst_state_on) {
        if (now_ms - burst_last_ms >= burst_on_ms) {
          burst_state_on = false;
          digitalWrite(OUT_PIN, LOW);
          burst_last_ms = now_ms;
          burst_index++;
        }
      } else {
        if (burst_index >= burst_total) { 
          digitalWrite(OUT_PIN, LOW); 
          pulseMode = PM_IDLE; 
        }
        else if (now_ms - burst_last_ms >= burst_off_ms) {
          burst_state_on = true;
          digitalWrite(OUT_PIN, HIGH);
          burst_last_ms = now_ms;
        }
      }
      break;

    case PM_TRAIN:
      if (now_ms - train_start_ms >= train_duration_ms_running) {
        digitalWrite(OUT_PIN, LOW); 
        pulseMode = PM_IDLE; 
        train_state_on = false; 
        break;
      }
      if (train_state_on) {
        unsigned long on_since = now_us - (train_next_toggle_us - train_pw_us);
        if (on_since >= train_pw_us) {
          digitalWrite(OUT_PIN, LOW);
          train_state_on = false;
          unsigned long off_time_us = (train_period_us > train_pw_us) ? 
                                      (train_period_us - train_pw_us) : 0;
          train_next_toggle_us = now_us + off_time_us;
        }
      } else {
        if ((long)(now_us - train_next_toggle_us) >= 0) {
          digitalWrite(OUT_PIN, HIGH);
          train_state_on = true;
          train_next_toggle_us = now_us + train_pw_us;
        }
      }
      break;
  }
}