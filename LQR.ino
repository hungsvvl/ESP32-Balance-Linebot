#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ================= GIAO TIẾP VỚI ESP PHỤ =================
#define LINK_RX 16
#define LINK_TX 17
HardwareSerial SerialLink(2);

// ================= CẤU HÌNH OLED =================
#define OLED_SDA_PIN 25
#define OLED_SCL_PIN 26
TwoWire I2C_OLED = TwoWire(1);

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_OLED, OLED_RESET);
bool hasOLED = false;

// ================= CẤU HÌNH MOTOR & MPU =================
#define EN_PIN 27
#define DIR_LEFT_PIN 13
#define STEP_LEFT_PIN 12
#define DIR_RIGHT_PIN 15
#define STEP_RIGHT_PIN 4
#define MPU_INT_PIN 19

const uint8_t LEDC_CHAN_L = 0;
const uint8_t LEDC_CHAN_R = 1;
const uint32_t LEDC_RES_BITS = 10;
const uint32_t MIN_FREQ = 20; // Nếu bị reset thì thử 10

// ================= THAM SỐ (LQR STATE FEEDBACK) =================
// NOTE: giữ tên Kp/Ki/Kd để tương thích UI (cmd p/i/d)
// Nhưng ý nghĩa giờ là:
//   Kp = K_theta      (gain theo góc e, đơn vị: output/deg)
//   Kd = K_thetaDot   (gain theo tốc độ góc e_dot, đơn vị: output/(deg/s))
//   Ki = K_v          (gain theo tốc độ bánh v, đơn vị: output/(steps/s * 0.001))
float Kp = 70.0f;   // K_theta
float Ki = 50.0f;   // K_v (speed feedback)
float Kd = 1.6f;    // K_thetaDot

float targetAngle = 180.0f;
float speedScale  = 12.0f;    // scale output -> steps/s
long  maxSpeed    = 15000;

float killErrorDeg = 40.0f;
float deadbandDeg  = 0.0f;

// lọc cho e_dot (tương tự D-term filter)
float dAlpha = 0.90f; // 0.85..0.98 tuỳ nhiễu

MPU6050 mpu;
bool dmpReady = false;
volatile bool mpuInterrupt = false;
uint8_t mpuIntStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

void IRAM_ATTR dmpDataReady() { mpuInterrupt = true; }

float currentAngle = 180.0f;

// ====== LQR states (estimated) ======
float eDeg = 0.0f, lastEDeg = 0.0f;    // angle error (deg)
float eDotFilt = 0.0f;                 // filtered error rate (deg/s)
float vLP = 0.0f;                      // low-pass of motor command speed (steps/s)

// Motor command
volatile long motorSpeedL = 0;
volatile long motorSpeedR = 0;

// Timing
uint32_t lastCtrlUs = 0;
uint32_t lastDebugTime = 0;
uint32_t lastOledUpdate = 0;
bool isRobotActive = false;

// ===== [PATCH #2] TÁCH BUFFER CHO USB & UART LINK =====
char cmdBufUSB[64];
uint8_t cmdLenUSB = 0;

char cmdBufLink[64];
uint8_t cmdLenLink = 0;
// ======================================================

static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
static inline long  clampl(long v, long lo, long hi)    { return (v < lo) ? lo : (v > hi) ? hi : v; }

// ================= PWM & MOTOR =================
void setupPWM(uint8_t pin, uint8_t channel) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(pin, MIN_FREQ, LEDC_RES_BITS);
#else
  ledcSetup(channel, MIN_FREQ, LEDC_RES_BITS);
  ledcAttachPin(pin, channel);
#endif
}

void setPWMFreq(uint8_t pin, uint8_t channel, uint32_t freq) {
  if (freq < MIN_FREQ) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(pin, 0);
#else
    ledcWrite(channel, 0);
#endif
    return;
  }
  if (freq > 25000) freq = 25000;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWriteTone(pin, freq);
#else
  ledcWriteTone(channel, freq);
#endif
}

void applyMotorSpeed() {
  if (!isRobotActive) {
    setPWMFreq(STEP_LEFT_PIN, LEDC_CHAN_L, 0);
    setPWMFreq(STEP_RIGHT_PIN, LEDC_CHAN_R, 0);
    return;
  }

  long spL = motorSpeedL;
  long spR = motorSpeedR;

  if (spL != 0) digitalWrite(DIR_LEFT_PIN, (spL > 0) ? LOW : HIGH);
  if (spR != 0) digitalWrite(DIR_RIGHT_PIN, (spR > 0) ? HIGH : LOW);

  setPWMFreq(STEP_LEFT_PIN, LEDC_CHAN_L, abs(spL));
  setPWMFreq(STEP_RIGHT_PIN, LEDC_CHAN_R, abs(spR));
}

// ================= OLED & DEBUG =================
void handleOLED() {
  if (!hasOLED) return;
  if (millis() - lastOledUpdate < 250) return;
  lastOledUpdate = millis();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(isRobotActive ? "RUN" : "IDLE");

  display.setCursor(60, 0);
  display.print("K:"); display.print((int)Kp);

  display.setCursor(10, 12);
  display.setTextSize(2);
  display.printf("%.2f", currentAngle);
  display.display();
}

void printDebug() {
  if (millis() - lastDebugTime < 500) return;
  lastDebugTime = millis();

  String logMsg =
    "A:" + String(currentAngle, 2) +
    " T:" + String(targetAngle, 2) +
    " Kth(p):" + String(Kp, 2) +
    " Kv(i):" + String(Ki, 2) +
    " Kd(d):" + String(Kd, 2) +
    " v:" + String(vLP, 0) +
    " e:" + String(eDeg, 2) +
    " ed:" + String(eDotFilt, 1) +
    "\r\n";

  Serial.print(logMsg);
  SerialLink.print(logMsg);
}

// ================= LQR CONTROL =================
// LQR dạng state feedback (thực dụng, không cần encoder):
//   u = Kp*e + Kd*e_dot - Ki*v
//   sp = u * speedScale
// Trong đó v dùng motor command low-pass làm "proxy velocity feedback".
void computeLQR() {
  uint32_t nowUs = micros();
  if (lastCtrlUs == 0) { lastCtrlUs = nowUs; return; }
  float dt = (nowUs - lastCtrlUs) * 1e-6f;
  lastCtrlUs = nowUs;
  dt = clampf(dt, 0.002f, 0.020f);

  // error (deg) theo convention cũ để khỏi đảo chiều tuning:
  // PID cũ: error = target - current
  eDeg = targetAngle - currentAngle;

  // Safety kill
  if (fabsf(eDeg) > killErrorDeg) {
    isRobotActive = false;
    motorSpeedL = motorSpeedR = 0;
    vLP = 0;
    eDotFilt = 0;
    lastEDeg = eDeg;
    applyMotorSpeed();
    return;
  }

  // Arm logic
  if (!isRobotActive) {
    if (fabsf(eDeg) < 2.0f) {
      isRobotActive = true;
      vLP = 0;
      eDotFilt = 0;
      lastEDeg = eDeg;
    } else {
      return;
    }
  }

  // Deadband (optional)
  if (fabsf(eDeg) < deadbandDeg) {
    motorSpeedL = motorSpeedR = 0;
    applyMotorSpeed();
    return;
  }

  // e_dot estimation + filter
  float eDot = (eDeg - lastEDeg) / dt;             // deg/s
  eDotFilt = dAlpha * eDotFilt + (1.0f - dAlpha) * eDot;
  lastEDeg = eDeg;

  // velocity feedback proxy (steps/s)
  vLP = 0.9f * vLP + 0.1f * (float)motorSpeedL;

  // Control law
  // vLP scale: *0.001 để Ki ~ 0..200 trong UI vẫn hợp lý
  float u = (Kp * eDeg) + (Kd * eDotFilt) - (Ki * (vLP * 0.001f));

  long sp = (long)(u * speedScale);
  sp = clampl(sp, -maxSpeed, maxSpeed);

  motorSpeedL = motorSpeedR = sp;
  applyMotorSpeed();
}

// ================= CALIB =================
void performAutoCalibration() {
  if (hasOLED) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.print("CALIBRATING...");
    display.display();
  }

  mpu.resetFIFO();
  fifoCount = 0;

  long sumAngle = 0;
  int samples = 0;
  unsigned long startTime = millis();

  while (samples < 100) {
    if (millis() - startTime > 3000) { targetAngle = 180.0f; return; }

    if (!mpuInterrupt && fifoCount < packetSize) {
      mpuIntStatus = mpu.getIntStatus();
      fifoCount = mpu.getFIFOCount();
      if (fifoCount < packetSize) continue;
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) { mpu.resetFIFO(); continue; }

    if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      sumAngle += (long)(ypr[1] * 180.0f / M_PI + 180.0f);
      samples++;
    }
  }

  targetAngle = (float)sumAngle / 100.0f;
  SerialLink.print("CALIB_DONE: ");
  SerialLink.println(targetAngle);
}

// ===== [PATCH #1] DMP FIFO: ĐỌC HẾT PACKET, KHÔNG RESET KHI fifoCount>packetSize =====
void handleDMP() {
  if (!mpuInterrupt && fifoCount < packetSize) return;

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    return;
  }

  if (mpuIntStatus & 0x02) {
    // Đọc hết FIFO, giữ packet cuối (mới nhất)
    while (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    currentAngle = ypr[1] * 180.0f / M_PI + 180.0f;

    // gọi LQR
    computeLQR();
  }
}
// ================================================================================

// ================= NHẬN LỆNH =================
void parseCommand(char* str) {
  char cmd = str[0];
  float val = atof(str + 1);

  switch (cmd) {
    // p/i/d vẫn giữ để UI dùng như cũ, nhưng là LQR gains
    case 'p': Kp = val; break;           // K_theta
    case 'i': Ki = val; break;           // K_v
    case 'd': Kd = val; break;           // K_thetaDot

    case 't': targetAngle = val; break;
    case 's': speedScale = val; break;

    // Đồng bộ maxSpeed 25000
    case 'm': maxSpeed = clampl((long)val, 1000, 25000); break;

    case 'b': deadbandDeg = val; break;
    case 'c': performAutoCalibration(); break;
    case '?': lastDebugTime = 0; printDebug(); break;
  }

  // reset trạng thái LQR để khỏi “giật”
  eDotFilt = 0;
  vLP = 0;
  lastEDeg = eDeg;

  SerialLink.print("OK:");
  SerialLink.print(cmd);
  SerialLink.println(val);
}

// ===== [PATCH #2] HANDLE SERIAL: TÁCH BUFFER TRÁNH TRỘN LỆNH =====
static inline void handleOneStream(Stream& s, char* buf, uint8_t& len) {
  while (s.available()) {
    char c = (char)s.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[len] = 0;
      if (len) parseCommand(buf);
      len = 0;
    } else if (len < 63) {
      buf[len++] = c;
    }
  }
}

void handleSerial() {
  handleOneStream(Serial, cmdBufUSB, cmdLenUSB);
  handleOneStream(SerialLink, cmdBufLink, cmdLenLink);
}
// ==================================================================

// ================= SETUP & LOOP =================
void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_LEFT_PIN, OUTPUT); pinMode(STEP_LEFT_PIN, OUTPUT);
  pinMode(DIR_RIGHT_PIN, OUTPUT); pinMode(STEP_RIGHT_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  Serial.begin(115200);
  SerialLink.begin(115200, SERIAL_8N1, LINK_RX, LINK_TX);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  Wire.setTimeOut(3000);

  I2C_OLED.begin(OLED_SDA_PIN, OLED_SCL_PIN, 400000);
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    hasOLED = true;
    display.clearDisplay();
    display.display();
  }

  setupPWM(STEP_LEFT_PIN, LEDC_CHAN_L);
  setupPWM(STEP_RIGHT_PIN, LEDC_CHAN_R);

  mpu.initialize();
  uint8_t devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(120); mpu.setYGyroOffset(130);
  mpu.setZGyroOffset(-45); mpu.setZAccelOffset(707);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    pinMode(MPU_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;

    // Auto calib lúc khởi động
    if (hasOLED) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.setTextSize(1);
      display.print("DO NOT MOVE!");
      display.setCursor(0, 10);
      display.print("Auto-Calib in 2s...");
      display.display();
    }
    delay(2000);
    performAutoCalibration();
  }
}

void loop() {
  if (!dmpReady) return;
  handleSerial();
  handleDMP();
  handleOLED();
  printDebug();
}
