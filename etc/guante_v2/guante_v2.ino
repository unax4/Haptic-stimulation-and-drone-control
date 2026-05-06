#include <Arduino_LSM6DSOX.h>

/* -------------------- Pines -------------------- */
enum { DATA_PIN=2, CLK_PIN=3, POT_CS=4, HV_LE=5, HV_CLR=6, OUT_PIN=13 };
const int analogPins[4] = {A3,A2,A1,A0};

/* -------------------- Estado -------------------- */
volatile bool spi_busy=false, streamingEnabled=true;
int potValue=255;
uint16_t hvState=0;

/* -------------------- Pulsos -------------------- */
enum PulseMode { IDLE, SINGLE, BURST, TRAIN } pulseMode=IDLE;

unsigned long t0_ms, dur_ms;
int burstTotal, burstIdx; 
unsigned long on_ms, off_ms, tBurst;
bool burstOn;

unsigned long trainStart_ms, trainDur_ms, period_us, pw_us, tNext_us;
bool trainOn;

/* -------------------- Parámetros -------------------- */
unsigned long single_ms=3000, burstPulse_ms=50, burstPause_ms=1000, train_ms=5000;
int burstCount=10; 
float freq_Hz=100;

/* -------------------- Posiciones -------------------- */
const uint16_t positions[21] = {
  0b0000000000000000,  // M0
  0b0100000000010101,  // M1
  0b0100000000011010,  // M2
  0b0100000000100101,  // M3
  0b0100000000101010,  // M4
  0b0100000001000101,  // M5
  0b0100000001001010,  // M6
  0b1000000010000101,  // M7
  0b1000000010001010,  // M8
  0b1000000100000101,  // M9
  0b1000000100001010,  // M10
  0b0100001000000101,  // M11
  0b0100001000001010,  // M12
  0b0100010000000101,  // M13
  0b0100010000001010,  // M14
  0b1000100000000101,  // M15
  0b1000100000001010,  // M16
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
  "middle_top 1 (0100000001000101)",
  "middle_top 2 (0100000001001010)",
  "ring_top 1 (1000000010000101)",
  "ring_top 2 (1000000010001010)",
  "pinky_top 1 (1000000100000101)",
  "pinky_top 2 (1000000100001010)",
  "index_bottom 1 (0100001000000101)",
  "index_bottom 2 (0100001000001010)",
  "middle_bottom 1 (0100010000000101)",
  "middle_bottom 2 (0100010000001010)",
  "ring_bottom 1 (1000100000000101)",
  "ring_bottom 2 (1000100000001010)",
  "pinky_bottom 1 (1001000000000101)",
  "pinky_bottom 2 (1001000000001010)",
  "palm 1 (1110000000000101)",
  "palm 2 (1110000000001010)"
};
/* ==================== UTILIDADES SPI ==================== */
inline void clkPulse(){
  digitalWrite(CLK_PIN,HIGH); delayMicroseconds(2);
  digitalWrite(CLK_PIN,LOW);  delayMicroseconds(1);
}

void sendHV(uint16_t v){
  spi_busy=true;
  digitalWrite(HV_LE,LOW); delayMicroseconds(2);
  for(int i=15;i>=0;i--){ digitalWrite(DATA_PIN,(v>>i)&1); clkPulse(); }
  digitalWrite(HV_LE,HIGH); delayMicroseconds(5);
  spi_busy=false;
}

void setPot(byte v){
  spi_busy=true;
  digitalWrite(POT_CS,LOW); delayMicroseconds(2);
  digitalWrite(DATA_PIN,LOW); clkPulse();     // dummy bit
  for(int i=7;i>=0;i--){ digitalWrite(DATA_PIN,(v>>i)&1); clkPulse(); }
  digitalWrite(POT_CS,HIGH);
  spi_busy=false;
}

/* ==================== POSICIONES ==================== */
inline void setPosition(int n){ hvState=positions[n]; sendHV(hvState); }

/* ==================== PULSOS ==================== */
void stopPulses(){
  pulseMode=IDLE; digitalWrite(OUT_PIN,LOW);
  burstIdx=0; burstOn=false; trainOn=false;
}

void startSingle(unsigned long d){
  stopPulses(); t0_ms=millis(); dur_ms=d;
  digitalWrite(OUT_PIN,HIGH); pulseMode=SINGLE;
}

void startBurst(int n,unsigned long on,unsigned long off){
  stopPulses();
  burstTotal=max(0,n); burstIdx=0;
  on_ms=on; off_ms=off; tBurst=millis();
  burstOn=true; digitalWrite(OUT_PIN,HIGH); pulseMode=BURST;
}

void startTrain(float f,unsigned long pw,unsigned long total){
  if(f<=0||pw==0) return;
  stopPulses();
  period_us=1e6/f; pw_us=pw;
  trainStart_ms=millis(); trainDur_ms=total;
  tNext_us=micros()+pw_us; trainOn=true;
  digitalWrite(OUT_PIN,HIGH); pulseMode=TRAIN;
}

void updatePulses(){
  unsigned long ms=millis(), us=micros();
  switch(pulseMode){
    case SINGLE:
      if(ms-t0_ms>=dur_ms){ digitalWrite(OUT_PIN,LOW); pulseMode=IDLE; }
      break;

    case BURST:
      if(burstIdx>=burstTotal){ stopPulses(); break; }
      if(burstOn && ms-tBurst>=on_ms){
        burstOn=false; digitalWrite(OUT_PIN,LOW); tBurst=ms; burstIdx++;
      } else if(!burstOn && ms-tBurst>=off_ms){
        burstOn=true; digitalWrite(OUT_PIN,HIGH); tBurst=ms;
      }
      break;

    case TRAIN:
      if(ms-trainStart_ms>=trainDur_ms){ stopPulses(); break; }
      if(trainOn && us>=tNext_us){
        digitalWrite(OUT_PIN,LOW); trainOn=false;
        tNext_us=us+(period_us>pw_us?period_us-pw_us:0);
      } else if(!trainOn && us>=tNext_us){
        digitalWrite(OUT_PIN,HIGH); trainOn=true; tNext_us=us+pw_us;
      }
      break;
    default: break;
  }
}

/* ==================== INFO ==================== */
float potR(){ return potValue*10000.0/255.0; }

void help(){
  Serial.println(F("\n--- Comandos Rápidos ---"));
  Serial.println(F("M0..M20 -> posiciones"));
  Serial.println(F("SW0..SW15 -> toggle switch"));
  Serial.println(F("16 bits -> valor binario SW15..SW0"));
  Serial.println(F("P0..P255 -> pot (corriente)"));
  Serial.println(F("S, SDxxx -> single pulse"));
  Serial.println(F("B, BCnn, BPxx, BOxx -> burst"));
  Serial.println(F("T, Fxx, Wxx, TDxx -> train"));
  Serial.println(F("STOP -> corta pulsos"));
  Serial.println(F("STREAM_ON / STREAM_OFF -> control streaming"));
  Serial.println(F("STATUS, ?, HELP\n"));
}

/* ==================== SETUP ==================== */
void setup(){
  pinMode(OUT_PIN,OUTPUT);
  pinMode(DATA_PIN,OUTPUT); pinMode(CLK_PIN,OUTPUT);
  pinMode(POT_CS,OUTPUT); pinMode(HV_LE,OUTPUT); pinMode(HV_CLR,OUTPUT);
  digitalWrite(POT_CS,HIGH); digitalWrite(HV_LE,HIGH);

  Serial.begin(115200); delay(1500);
  IMU.begin();

  setPot(potValue); setPosition(0);
  Serial.println(F("=== Generador + HV2701 + MAX5413 ==="));
  help();
}

/* ==================== LOOP ==================== */
void loop(){

  /* -------- Comandos -------- */
  if(Serial.available()){
    spi_busy=true;
    String c=Serial.readStringUntil('\n'); c.trim(); c.toUpperCase();

    if(c=="STOP") stopPulses();
    else if(c=="STREAM_ON") streamingEnabled=true;
    else if(c=="STREAM_OFF") streamingEnabled=false;

    else if(c.startsWith("SW")){
      int b=c.substring(2).toInt();
      if(b>=0&&b<16){ hvState^=1<<b; sendHV(hvState); }
    }

    else if(c.startsWith("M")){
      int n=c.substring(1).toInt();
      if(n>=0&&n<21){ setPosition(n); Serial.println(names[n]); }
    }

    else if(c.length()==16){
      uint16_t v=0; bool ok=true;
      for(int i=0;i<16;i++){
        if(c[i]=='1') v|=1<<(15-i);
        else if(c[i]!='0') ok=false;
      }
      if(ok){ hvState=v; sendHV(v); }
    }

    else if(c=="S") startSingle(single_ms);
    else if(c.startsWith("SD")) single_ms=c.substring(2).toInt();

    else if(c=="B") startBurst(burstCount,burstPulse_ms,burstPause_ms);
    else if(c.startsWith("BC")) burstCount=c.substring(2).toInt();
    else if(c.startsWith("BP")) burstPulse_ms=c.substring(2).toInt();
    else if(c.startsWith("BO")) burstPause_ms=c.substring(2).toInt();

    else if(c=="T") startTrain(freq_Hz,pw_us,train_ms);
    else if(c.startsWith("F")) freq_Hz=c.substring(1).toFloat();
    else if(c.startsWith("W")) pw_us=c.substring(1).toInt();
    else if(c.startsWith("TD")) train_ms=c.substring(2).toInt();

    else if(c.startsWith("P")){
      potValue=constrain(c.substring(1).toInt(),0,255);
      setPot(potValue);
      Serial.print(F("Pot ")); Serial.print(potValue);
      Serial.print(F(" ~")); Serial.print(potR(),0); Serial.println(F(" ohm"));
    }

    else if(c=="?"||c=="HELP") help();

    spi_busy=false; delay(3);
  }

  /* -------- Streaming -------- */
  if(streamingEnabled && !spi_busy){
    static unsigned long t=0;
    if(millis()-t>=10){
      t=millis();
      int a[4]; for(int i=0;i<4;i++) a[i]=analogRead(analogPins[i]);
      float ax,ay,az,gx,gy,gz;
      if(IMU.accelerationAvailable()) IMU.readAcceleration(ax,ay,az);
      if(IMU.gyroscopeAvailable()) IMU.readGyroscope(gx,gy,gz);

      Serial.print(t/1000.0,3); Serial.print(',');
      for(int i=0;i<4;i++){ Serial.print(a[i]); Serial.print(','); }
      Serial.print(ax,3); Serial.print(','); Serial.print(ay,3); Serial.print(',');
      Serial.print(az,3); Serial.print(','); Serial.print(gx,3); Serial.print(',');
      Serial.print(gy,3); Serial.print(','); Serial.println(gz,3);
    }
  }

  updatePulses();
}
