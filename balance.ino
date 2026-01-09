/*
 * ROBOT CÂN BẰNG 2 BÁNH - ESP32 MAIN V7.2
 * - Đã bỏ tính năng Auto-Raise.
 * - Thêm tính năng: Khởi động -> Hiện "HOLDING" (1s) -> "CALIB" -> Chạy.
 */

#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================= CẤU HÌNH GIAO TIẾP =================
#define RX_AUX_PIN 16 
#define TX_AUX_PIN 17 
HardwareSerial SerialAux(2); 

// ================= CẤU HÌNH OLED =================
#define OLED_SDA_PIN 25
#define OLED_SCL_PIN 26
TwoWire I2C_OLED = TwoWire(1);
Adafruit_SSD1306 display(128, 32, &I2C_OLED, -1);
bool hasOLED = false;

// ================= PIN MOTOR =================
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

// ================= PID & TUNING =================
float Kp = 70.0f;
float Ki = 50.0f;
float Kd = 1.1f;

// Anti-Drift
float Kpos = 0.0f;       
float Kvel = 0.0f;       
float maxTiltOffset = 5.0f; 

float targetAngle = 180.0f;
float speedScale = 12.0f;
long maxSpeed = 15000;

// Các ngưỡng góc
float killErrorDeg = 45.0f; // Góc ngã (tắt motor)

float iZoneDeg = 4.0f;
float deadbandDeg = 0.0f;
float Kv = 0.015f;
float dAlpha = 0.9f;

// --- BIẾN ĐIỀU KHIỂN (JOYSTICK) ---
float throttleInput = 0; 
float steeringInput = 0; 

// ================= BIẾN HỆ THỐNG =================
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

// Anti-drift states
float pos_m = 0.0f;        
float vel_mps = 0.0f;      
float targetDyn = 180.0f; 

char cmdBuf[64];
uint8_t cmdLen = 0;

static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
static inline long clampl(long v, long lo, long hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

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
   
  long spL = motorSpeedL + steeringInput;
  long spR = motorSpeedR - steeringInput;

  spL = clampl(spL, -maxSpeed, maxSpeed);
  spR = clampl(spR, -maxSpeed, maxSpeed);

  if (spL != 0) digitalWrite(DIR_LEFT_PIN, (spL > 0) ? LOW : HIGH);
  if (spR != 0) digitalWrite(DIR_RIGHT_PIN, (spR > 0) ? HIGH : LOW);

  setPWMFreq(STEP_LEFT_PIN, LEDC_CHAN_L, abs(spL));
  setPWMFreq(STEP_RIGHT_PIN, LEDC_CHAN_R, abs(spR));
}

// ================= TÍNH TOÁN PID =================
void computePID() {
  uint32_t nowUs = micros();
  if (lastPidUs == 0) { lastPidUs = nowUs; return; }
  float dt = (nowUs - lastPidUs) * 1e-6f;
  lastPidUs = nowUs;
  dt = clampf(dt, 0.002f, 0.020f);

  error = targetAngle - currentAngle; 

  // --- LOGIC CÂN BẰNG ---
  // Anti-Drift Calculation
  if (isRobotActive && abs(throttleInput) < 10) {
    float estimatedSpeed = (float)motorSpeedL; 
    vel_mps = 0.95f * vel_mps + 0.05f * estimatedSpeed;
    pos_m += vel_mps * dt;
    
    float driftOffset = -(Kpos * pos_m + Kvel * vel_mps);
    driftOffset = clampf(driftOffset, -maxTiltOffset, maxTiltOffset);
    targetDyn = targetAngle + driftOffset + (throttleInput * 0.02f);
  } else {
    pos_m = 0; vel_mps = 0;
    targetDyn = targetAngle + (throttleInput * 0.02f);
  }

  float pidError = targetDyn - currentAngle;

  // Nếu ngã quá góc cho phép -> Tắt máy
  if (fabsf(pidError) > killErrorDeg) {
    isRobotActive = false;
    pidOutput = 0; iTerm = 0;
    motorSpeedL = motorSpeedR = 0;
    applyMotorSpeed();
    return;
  }

  // Auto Start (Nếu tay dựng xe lên nhẹ nhàng vào vùng cân bằng)
  if (!isRobotActive) {
    if (fabsf(pidError) < 2.0f) {
      isRobotActive = true;
      iTerm = 0; pos_m = 0;
    } else {
      return;
    }
  }

  // PID Computation
  if (fabsf(pidError) < deadbandDeg) {
    iTerm *= 0.95f;
    pidOutput = 0;
  } else {
    if (fabsf(pidError) < iZoneDeg) {
      iTerm += (Ki * pidError * dt);
      float iLimit = ((float)maxSpeed) / speedScale;
      iTerm = clampf(iTerm, -iLimit, iLimit);
    } else {
      iTerm = 0;
    }

    float dRaw = (pidError - lastError) / dt;
    dFilt = dAlpha * dFilt + (1.0f - dAlpha) * dRaw;
    lastError = pidError;

    pidOutput = (Kp * pidError) + iTerm + (Kd * dFilt);
    vLP = 0.9f * vLP + 0.1f * (float)motorSpeedL;
    pidOutput -= Kv * vLP;
  }

  long sp = (long)(pidOutput * speedScale);
  sp = clampl(sp, -maxSpeed, maxSpeed);
  motorSpeedL = sp; 
  motorSpeedR = sp;
  applyMotorSpeed();
}

// ================= COMMS PARSER =================
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
    
    case 'M': throttleInput = val; break;
    case 'T': steeringInput = val; break;
    
    case 'x': Kpos = val; break;
    case 'y': Kvel = val; break;
    
    case 'c': performAutoCalibration(); break;
  }
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { cmdBuf[cmdLen] = 0; parseCommand(cmdBuf); cmdLen = 0; }
    else if (cmdLen < 63) cmdBuf[cmdLen++] = c;
  }
  while (SerialAux.available()) {
    char c = SerialAux.read();
    if (c == '\n') { cmdBuf[cmdLen] = 0; parseCommand(cmdBuf); cmdLen = 0; }
    else if (cmdLen < 63) cmdBuf[cmdLen++] = c;
  }
}

// ================= AUTO CALIB (MỚI) =================
void performAutoCalibration() {
  // BƯỚC 1: Hiển thị HOLDING và chờ 1 giây
  if (hasOLED) {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(20, 10);
    display.print("HOLDING");
    display.display();
  }
  
  // Chờ 1s để người dùng giữ yên xe
  delay(1000); 

  // BƯỚC 2: Hiển thị CALIB và bắt đầu lấy mẫu
  if (hasOLED) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(20, 10);
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
  pos_m = 0; 
  
  // Hiển thị xong
  if (hasOLED) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(20, 10);
    display.print("DONE!");
    display.display();
    delay(500);
  }
}

// ================= SETUP =================
void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_LEFT_PIN, OUTPUT);
  pinMode(STEP_LEFT_PIN, OUTPUT);
  pinMode(DIR_RIGHT_PIN, OUTPUT);
  pinMode(STEP_RIGHT_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  Serial.begin(115200);
  SerialAux.begin(115200, SERIAL_8N1, RX_AUX_PIN, TX_AUX_PIN);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  Wire.setTimeOut(3000);

  I2C_OLED.begin(OLED_SDA_PIN, OLED_SCL_PIN, 400000);
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) hasOLED = true;

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
    
    // Gọi hàm calib lúc khởi động (đã có delay 1s)
    performAutoCalibration();
  }
}

void handleDMP() {
  if (!mpuInterrupt && fifoCount < packetSize) return;
  mpuInterrupt = false; mpuIntStatus = mpu.getIntStatus(); fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) { mpu.resetFIFO(); return; }
  if (mpuIntStatus & 0x02) {
    if (fifoCount > packetSize) { mpu.resetFIFO(); return; }
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize); fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer); mpu.dmpGetGravity(&gravity, &q); mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    currentAngle = ypr[1] * 180.0f / M_PI + 180.0f;
    computePID();
  }
}

void printDebug() {
  if (millis() - lastDebugTime < 200) return;
  lastDebugTime = millis();
  String log = "Angle:" + String(currentAngle, 2) + " ";
  log += "Target:" + String(targetAngle, 2) + "\n";
  Serial.print(log);
  SerialAux.print(log);
}

void loop() {
  if (!dmpReady) return;
  handleSerial();
  handleDMP();
  
  if(hasOLED) {
      if (millis() - lastOledUpdate > 250) {
        lastOledUpdate = millis();
        display.clearDisplay(); display.setTextColor(WHITE); display.setTextSize(1);
        display.setCursor(0,0); 
        display.print(isRobotActive?"RUN":"IDLE");
        display.setCursor(60,0); display.print("T:"); display.print(targetAngle,1);
        display.setCursor(10,12); display.setTextSize(2); display.print(currentAngle, 2);
        display.display();
      }
  }
  printDebug();
}
