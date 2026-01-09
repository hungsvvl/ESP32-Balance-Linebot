#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_SDA_PIN 25
#define OLED_SCL_PIN 26
TwoWire I2C_OLED = TwoWire(1);

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_OLED, OLED_RESET);
bool hasOLED = false;

#define EN_PIN 27
#define DIR_LEFT_PIN 13
#define STEP_LEFT_PIN 12
#define DIR_RIGHT_PIN 15
#define STEP_RIGHT_PIN 4
#define MPU_INT_PIN 19

const uint8_t LEDC_CHAN_L = 0;
const uint8_t LEDC_CHAN_R = 1;
const uint32_t LEDC_RES_BITS = 10;
const uint32_t MIN_FREQ = 20;

float Kp = 80.0f;
float Ki = 120.0f;
float Kd = 1.3f;

float targetAngle = 180.0f;
float speedScale = 12.0f;
long maxSpeed = 15000;

float killErrorDeg = 40.0f;
float iZoneDeg = 4.0f;
float deadbandDeg = 0.05f;
float Kv = 0.015f;
float dAlpha = 0.9f;

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
float error = 0.0f, lastError = 0.0f;
float iTerm = 0.0f;
float pidOutput = 0.0f;
float dFilt = 0.0f, vLP = 0.0f;

volatile long motorSpeedL = 0;
volatile long motorSpeedR = 0;
uint32_t lastPidUs = 0;
uint32_t lastDebugTime = 0;
uint32_t lastOledUpdate = 0;
bool isRobotActive = false;

char cmdBuf[64];
uint8_t cmdLen = 0;

static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
static inline long clampl(long v, long lo, long hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

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
  display.printf("T:%.1f", targetAngle);
  display.setCursor(10, 12);
  display.setTextSize(2);
  display.printf("%.2f", currentAngle);
  display.display();
}

void printDebug() {
  if (millis() - lastDebugTime < 1000) return;
  lastDebugTime = millis();
  Serial.printf("A:%.2f T:%.2f E:%.2f SPD:%ld PID:%.2f\n", currentAngle, targetAngle, error, motorSpeedL, pidOutput);
}

void computePID() {
  uint32_t nowUs = micros();
  if (lastPidUs == 0) { lastPidUs = nowUs; return; }
  float dt = (nowUs - lastPidUs) * 1e-6f;
  lastPidUs = nowUs;
  dt = clampf(dt, 0.002f, 0.020f);

  error = targetAngle - currentAngle;

  if (fabsf(error) > killErrorDeg) {
    isRobotActive = false;
    pidOutput = 0;
    iTerm = 0;
    motorSpeedL = motorSpeedR = 0;
    applyMotorSpeed();
    return;
  }

  if (!isRobotActive) {
    if (fabsf(error) < 2.0f) {
      isRobotActive = true;
      iTerm = 0;
    } else {
      return;
    }
  }

  if (fabsf(error) < deadbandDeg) {
    iTerm *= 0.95f;
    pidOutput = 0;
  } else {
    if (fabsf(error) < iZoneDeg) {
      iTerm += (Ki * error * dt);
      float iLimit = ((float)maxSpeed) / speedScale;
      iTerm = clampf(iTerm, -iLimit, iLimit);
    } else {
      iTerm = 0;
    }

    float dRaw = (error - lastError) / dt;
    dFilt = dAlpha * dFilt + (1.0f - dAlpha) * dRaw;
    lastError = error;

    pidOutput = (Kp * error) + iTerm + (Kd * dFilt);
    vLP = 0.9f * vLP + 0.1f * (float)motorSpeedL;
    pidOutput -= Kv * vLP;
  }

  long sp = (long)(pidOutput * speedScale);
  sp = clampl(sp, -maxSpeed, maxSpeed);
  motorSpeedL = motorSpeedR = sp;
  applyMotorSpeed();
}

void performAutoCalibration() {
  if (hasOLED) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("CALIB...");
    display.display();
  }

  mpu.resetFIFO();
  fifoCount = 0;
  long sumAngle = 0;
  int samples = 0;
  unsigned long startTime = millis();

  while (samples < 100) {
    if (millis() - startTime > 3000) {
      targetAngle = 180.0f;
      return;
    }

    if (!mpuInterrupt && fifoCount < packetSize) {
      mpuIntStatus = mpu.getIntStatus();
      fifoCount = mpu.getFIFOCount();
      if (fifoCount < packetSize) continue;
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
      continue;
    }

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
}

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
    if (fifoCount > packetSize) { mpu.resetFIFO(); return; }
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    currentAngle = ypr[1] * 180.0f / M_PI + 180.0f;
    computePID();
  }
}

void parseCommand(char* str) {
  char cmd = str[0];
  float val = atof(str + 1);

  switch (cmd) {
    case 'p': Kp = val; break;
    case 'i': Ki = val; break;
    case 'd': Kd = val; break;
    case 't': targetAngle = val; break;
    case 's': speedScale = val; break;
    case 'm': maxSpeed = clampl((long)val, 1000, 20000); break;
    case 'b': deadbandDeg = val; break;
    case 'v': Kv = val; break;
    case 'c': performAutoCalibration(); break;
    case '?': lastDebugTime = 0; printDebug(); break;
  }
  iTerm = 0;
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { cmdBuf[cmdLen] = 0; parseCommand(cmdBuf); cmdLen = 0; }
    else if (cmdLen < 63) cmdBuf[cmdLen++] = c;
  }
}

void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_LEFT_PIN, OUTPUT);
  pinMode(STEP_LEFT_PIN, OUTPUT);
  pinMode(DIR_RIGHT_PIN, OUTPUT);
  pinMode(STEP_RIGHT_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  Serial.begin(115200);

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

  mpu.setXGyroOffset(120);
  mpu.setYGyroOffset(130);
  mpu.setZGyroOffset(-45);
  mpu.setZAccelOffset(707);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    pinMode(MPU_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
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
