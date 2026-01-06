/* 
 * ROBOT CÂN BẰNG 2 BÁNH - ESP32 V6.4.1 (KALMAN + UART LINK -> ESP PHỤ)
 * - Auto Calib
 * - Dual I2C: MPU6050 (21,22) & OLED (25,26)
 * - Loại bỏ HC-05
 * - Gửi REPORT mỗi 1 giây qua USB Serial + UART sang ESP32 phụ
 * - Nhận lệnh tuning từ USB Serial hoặc từ ESP32 phụ (UART)
 *
 * NEW:
 * - Thay DMP bằng Kalman Filter (Accel + Gyro)
 * - D-term dùng Gyro rate (mượt + nhanh)
 * - (Tùy chọn) Anti-drift vòng ngoài (giữ đứng yên, giảm trôi khi bị đẩy)
 */

#ifndef ARDUINO_ARCH_ESP32
#error "Loi: Hay chon board ESP32 trong Tools > Board"
#endif

#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================= 0. OPTION =================
#define USE_GYRO_DTERM     1     // 1: D = gyro rate (khuyên dùng)
#define ENABLE_ANTI_DRIFT  1     // 1: chống trôi (khuyên dùng), 0: giữ góc thuần

// Tần số đọc IMU (Hz)
#define IMU_HZ 250

// ================= 1. CẤU HÌNH DUAL I2C =================
#define OLED_SDA_PIN 25
#define OLED_SCL_PIN 26
TwoWire I2C_OLED = TwoWire(1);

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_OLED, OLED_RESET);
bool hasOLED = false;

// ================= 2. PINOUT & UART LINK (ESP PHỤ) =================
#define EN_PIN          27
#define DIR_LEFT_PIN    13
#define STEP_LEFT_PIN   12
#define DIR_RIGHT_PIN   15
#define STEP_RIGHT_PIN  4

// UART link sang ESP32 phụ
#define LINK_RX_PIN 16
#define LINK_TX_PIN 17
#define LINK_BAUD   115200
HardwareSerial LINK(2);

// ================= 3. LEDC PWM CONFIG =================
const uint8_t LEDC_CHAN_L = 0;
const uint8_t LEDC_CHAN_R = 1;
const uint32_t LEDC_RES_BITS = 10;
const uint32_t MIN_FREQ = 20;

// ================= 4. TUNING PARAMETERS =================
float Kp = 35.0f;
float Ki = 0.0f;
float Kd = 0.6f;

float targetAngle = 180.0f;     // hệ góc 0..360 (để giống code bạn)
float speedScale  = 12.0f;
long  maxSpeed    = 15000;

float killErrorDeg = 40.0f;
float iZoneDeg     = 4.0f;
float deadbandDeg  = 0.05f;
float Kv = 0.015f;
float dAlpha = 0.9f;

bool debugToLINK = true;

// ================= 4.1 IMU AXIS CONFIG (CỰC QUAN TRỌNG) =================
// Bạn phải chọn đúng trục ứng với "pitch" của xe.
// Gyro scale: nếu set full-scale 2000 dps => 16.4 LSB/(deg/s); 250 dps => 131
static const float GYRO_LSB_PER_DPS = 16.4f;  // sẽ đúng nếu set gyro FS = 2000 dps

// 0=X, 1=Y, 2=Z
#define GYRO_PITCH_AXIS  0
#define GYRO_PITCH_SIGN  (+1)

// Tính góc pitch từ accelerometer (deg). Bạn có thể phải đổi công thức.
// Option A (hay dùng): pitch = atan2(-ax, sqrt(ay^2+az^2))
// Option B (dựng dọc board khác): pitch = atan2(ax, az)
#define ACC_MODE  0  // 0=OptionA, 1=OptionB
#define ACC_PITCH_SIGN (+1)

// ================= 4.2 ANTI-DRIFT (OPTIONAL) =================
#if ENABLE_ANTI_DRIFT
const float STEPS_PER_REV = 200.0f * 16.0f;   // 1/16 microstep
const float WHEEL_D = 0.065f;                 // 65mm
const float WHEEL_C = 3.1415926f * WHEEL_D;
const float M_PER_STEP = (WHEEL_C / STEPS_PER_REV); // m/step

float vel_mps = 0.0f;   // m/s (ước lượng)
float pos_m   = 0.0f;   // m (ước lượng)

float Kpos = 10.0f;       // deg/m
float Kvel = 2.0f;        // deg/(m/s)
float maxTiltOffset = 3.0f; // deg

float targetDyn = 180.0f;
float tiltOffset = 0.0f;
#endif

// ================= 5. MPU + KALMAN =================
MPU6050 mpu;

static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
static inline long  clampl(long v, long lo, long hi)   { return (v < lo) ? lo : (v > hi) ? hi : v; }

// ----- Kalman 1D (angle + bias) -----
class Kalman1D {
public:
  Kalman1D() {
    Q_angle = 0.001f;
    Q_bias  = 0.003f;
    R_meas  = 0.03f;
    angle = 0.0f; bias = 0.0f; rate = 0.0f;
    P00 = P01 = P10 = P11 = 0.0f;
  }

  void setAngle(float a) { angle = a; }
  float getAngle(float newAngle, float newRate, float dt) {
    // Predict
    rate = newRate - bias;
    angle += dt * rate;

    // Update covariance
    P00 += dt * (dt*P11 - P01 - P10 + Q_angle);
    P01 -= dt * P11;
    P10 -= dt * P11;
    P11 += Q_bias * dt;

    // Innovation
    float S = P00 + R_meas;
    float K0 = P00 / S;
    float K1 = P10 / S;

    float y = newAngle - angle;

    // Update
    angle += K0 * y;
    bias  += K1 * y;

    float P00_temp = P00;
    float P01_temp = P01;

    P00 -= K0 * P00_temp;
    P01 -= K0 * P01_temp;
    P10 -= K1 * P00_temp;
    P11 -= K1 * P01_temp;

    return angle;
  }

  void setQangle(float q) { Q_angle = q; }
  void setQbias(float q)  { Q_bias  = q; }
  void setR(float r)      { R_meas  = r; }

  float getRate() const { return rate; }

private:
  float Q_angle, Q_bias, R_meas;
  float angle, bias, rate;
  float P00, P01, P10, P11;
};

Kalman1D kalman;
bool kalmanInited = false;

float accPitchDeg = 0.0f;     // -180..180
float gyroPitchDPS = 0.0f;    // deg/s
float kalPitchDeg  = 0.0f;    // -180..180

// ================= 6. STATE =================
float currentAngle = 180.0f;  // 0..360 để giống code bạn
float error = 0.0f;
float iTerm = 0.0f;
float pidOutput = 0.0f;
float dFilt = 0.0f, vLP = 0.0f;
volatile long motorSpeedL = 0;
volatile long motorSpeedR = 0;

uint32_t lastDebugTime = 0;
uint32_t lastOledUpdate = 0;
bool isRobotActive = false;

char cmdBuf[64];
uint8_t cmdLen = 0;

uint32_t lastImuUs = 0;
float lastDt = 0.0f;

// ================= 7. PWM HELPER =================
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
  if (spL != 0) digitalWrite(DIR_LEFT_PIN,  (spL > 0) ? LOW : HIGH);
  if (spR != 0) digitalWrite(DIR_RIGHT_PIN, (spR > 0) ? HIGH : LOW);
  setPWMFreq(STEP_LEFT_PIN, LEDC_CHAN_L, abs(spL));
  setPWMFreq(STEP_RIGHT_PIN, LEDC_CHAN_R, abs(spR));
}

// ================= 8. OLED =================
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
  display.print("T:");
  display.print(targetAngle, 1);

  display.setCursor(10, 12);
  display.setTextSize(2);
  display.print(currentAngle, 2);

  display.display();
}

// ================= 9. IMU HELPERS =================
static inline float gyroRawToDPS(int16_t gx, int16_t gy, int16_t gz) {
  int16_t raw = 0;
  #if (GYRO_PITCH_AXIS == 0)
    raw = gx;
  #elif (GYRO_PITCH_AXIS == 1)
    raw = gy;
  #else
    raw = gz;
  #endif
  return (GYRO_PITCH_SIGN * ((float)raw / GYRO_LSB_PER_DPS));
}

static inline float accelToPitchDeg(int16_t ax, int16_t ay, int16_t az) {
  float fax = (float)ax;
  float fay = (float)ay;
  float faz = (float)az;

  float pitch = 0.0f;

  #if (ACC_MODE == 0)
    // Option A: atan2(-ax, sqrt(ay^2 + az^2))
    float denom = sqrtf(fay*fay + faz*faz);
    if (denom < 1e-6f) denom = 1e-6f;
    pitch = atan2f(-fax, denom) * 57.2957795f;
  #else
    // Option B: atan2(ax, az)
    pitch = atan2f(fax, faz) * 57.2957795f;
  #endif

  return ACC_PITCH_SIGN * pitch;
}

// ================= 10. PID =================
void computePID(float dt) {
  dt = clampf(dt, 0.002f, 0.020f);

  // Error góc theo hệ 180-based để giống code bạn
  #if ENABLE_ANTI_DRIFT
    // Ước lượng vel/pos từ lệnh step (steps/s)
    float sp_steps_s = (float)motorSpeedL;
    float vel_now = sp_steps_s * M_PER_STEP;
    vel_mps = 0.90f * vel_mps + 0.10f * vel_now;
    pos_m  += vel_mps * dt;

    tiltOffset = -(Kpos * pos_m + Kvel * vel_mps);
    tiltOffset = clampf(tiltOffset, -maxTiltOffset, maxTiltOffset);
    targetDyn = targetAngle + tiltOffset;

    error = targetDyn - currentAngle;
  #else
    error = targetAngle - currentAngle;
  #endif

  // SAFETY
  if (fabsf(error) > killErrorDeg) {
    isRobotActive = false;
    pidOutput = 0; iTerm = 0;
    motorSpeedL = motorSpeedR = 0;
    dFilt = 0;
    #if ENABLE_ANTI_DRIFT
      vel_mps = 0; pos_m = 0;
      tiltOffset = 0; targetDyn = targetAngle;
    #endif
    applyMotorSpeed();
    return;
  }

  // AUTO START
  if (!isRobotActive) {
    if (fabsf(error) < 2.0f) {
      isRobotActive = true;
      iTerm = 0;
      dFilt = 0;
      #if ENABLE_ANTI_DRIFT
        vel_mps = 0; pos_m = 0;
        tiltOffset = 0; targetDyn = targetAngle;
      #endif
    } else {
      return;
    }
  }

  // Deadband: không set pidOutput=0 (để còn phanh trôi)
  float e = error;
  if (fabsf(e) < deadbandDeg) e = 0.0f;

  // I term
  if (fabsf(error) < iZoneDeg) {
    iTerm += (Ki * e * dt);
    float iLimit = ((float)maxSpeed) / speedScale;
    iTerm = clampf(iTerm, -iLimit, iLimit);
  } else {
    iTerm = 0.0f;
  }

  // D term
  #if USE_GYRO_DTERM
    // gyroPitchDPS ~ d(angle)/dt, còn d(error)/dt = -gyro
    dFilt = dAlpha * dFilt + (1.0f - dAlpha) * gyroPitchDPS;
    float dTerm = -Kd * dFilt;
  #else
    static float lastE = 0.0f;
    float dRaw = (e - lastE) / dt;
    dFilt = dAlpha * dFilt + (1.0f - dAlpha) * dRaw;
    lastE = e;
    float dTerm = Kd * dFilt;
  #endif

  pidOutput = (Kp * e) + iTerm + dTerm;

  // Damping kiểu cũ
  vLP = 0.9f * vLP + 0.1f * (float)motorSpeedL;
  pidOutput -= Kv * vLP;

  long sp = (long)(pidOutput * speedScale);
  sp = clampl(sp, -maxSpeed, maxSpeed);
  motorSpeedL = sp;
  motorSpeedR = sp;
  applyMotorSpeed();
}

// ================= 11. KALMAN UPDATE (thay handleDMP) =================
void handleIMU_Kalman() {
  const uint32_t periodUs = 1000000UL / IMU_HZ;
  uint32_t nowUs = micros();
  if (lastImuUs != 0 && (nowUs - lastImuUs) < periodUs) return;

  float dt = 0.0f;
  if (lastImuUs == 0) {
    lastImuUs = nowUs;
    dt = (1.0f / IMU_HZ);
  } else {
    dt = (nowUs - lastImuUs) * 1e-6f;
    lastImuUs = nowUs;
  }
  dt = clampf(dt, 0.001f, 0.03f);
  lastDt = dt;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  accPitchDeg = accelToPitchDeg(ax, ay, az);
  gyroPitchDPS = gyroRawToDPS(gx, gy, gz);

  if (!kalmanInited) {
    kalman.setAngle(accPitchDeg);
    kalPitchDeg = accPitchDeg;
    kalmanInited = true;
  }

  kalPitchDeg = kalman.getAngle(accPitchDeg, gyroPitchDPS, dt);

  // Quy về hệ 0..360 kiểu bạn đang dùng (quanh 180)
  currentAngle = kalPitchDeg + 180.0f;

  computePID(dt);
}

// ================= 12. AUTO CALIBRATION =================
void performAutoCalibration() {
  if (hasOLED) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("CALIB...");
    display.setCursor(0,15);
    display.setTextSize(2);
    display.println("HOLD");
    display.display();
  }

  Serial.println("Start Auto Calib (Kalman) ...");

  // Warm up Kalman 0.5s
  uint32_t t0 = millis();
  while (millis() - t0 < 500) {
    handleIMU_Kalman();
    delay(1);
  }

  // Lấy trung bình 1.0s
  float sum = 0.0f;
  int n = 0;
  t0 = millis();
  while (millis() - t0 < 1000) {
    handleIMU_Kalman();
    sum += currentAngle;
    n++;
    delay(1);
  }

  if (n < 10) {
    Serial.println("CALIB FAIL: not enough samples");
    targetAngle = 180.0f;
  } else {
    targetAngle = sum / (float)n;
  }

  #if ENABLE_ANTI_DRIFT
    vel_mps = 0; pos_m = 0;
    tiltOffset = 0; targetDyn = targetAngle;
  #endif

  if (hasOLED) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("DONE!");
    display.setTextSize(2);
    display.setCursor(0,15);
    display.print("T:");
    display.print(targetAngle, 1);
    display.display();
    delay(1200);
  }

  Serial.print("New Target: ");
  Serial.println(targetAngle, 3);
}

// ================= 13. DEBUG REPORT (1s) =================
void printDebug() {
  if (millis() - lastDebugTime < 1000) return;
  lastDebugTime = millis();

  String log = "";
  log += "====== STATUS ======\n";
  log += "RUN: " + String(isRobotActive ? 1 : 0) + " | ms: " + String(millis()) + "\n";
  log += "Angle: " + String(currentAngle, 2) + " / T:" + String(targetAngle, 2) + "\n";
  log += "accPitch: " + String(accPitchDeg, 2) + " | kalPitch: " + String(kalPitchDeg, 2) + "\n";
  log += "gyro: " + String(gyroPitchDPS, 2) + " dps | dt: " + String(lastDt, 4) + " s\n";
  log += "Error: " + String(error, 2) + "\n";
  log += "SpeedCmd: " + String(motorSpeedL) + "\n";
  log += "PID Out: " + String(pidOutput, 2) + "\n";

  #if ENABLE_ANTI_DRIFT
  log += "ANTI-DRIFT: pos=" + String(pos_m, 3) + "m vel=" + String(vel_mps, 3) + "m/s\n";
  log += "Kpos(x)=" + String(Kpos,2) + " Kvel(y)=" + String(Kvel,2) + " maxOff(o)=" + String(maxTiltOffset,2) + "\n";
  #endif

  log += "====== TUNING ======\n";
  log += "Kp: " + String(Kp, 2) + " Ki: " + String(Ki, 3) + " Kd: " + String(Kd, 2) + "\n";
  log += "Kv: " + String(Kv, 3) + " dead: " + String(deadbandDeg, 3) + " iZone: " + String(iZoneDeg,2) + "\n";
  log += "speedScale: " + String(speedScale, 2) + " maxSpeed: " + String(maxSpeed) + "\n";
  log += "====================\n";

  Serial.print(log);
  if (debugToLINK) LINK.print(log);
}

// ================= 14. COMMAND PARSER =================
void parseCommand(char* str) {
  char cmd = str[0];
  float val = atof(str + 1);

  switch(cmd) {
    case 'p': Kp = val; break;
    case 'i': Ki = val; break;
    case 'd': Kd = val; break;
    case 't': targetAngle = val; break;
    case 's': speedScale = val; break;
    case 'm': maxSpeed = clampl((long)val, 1000, 20000); break;
    case 'b': deadbandDeg = val; break;
    case 'v': Kv = val; break;
    case 'c': performAutoCalibration(); return;
    case 'q': debugToLINK = (val != 0); break;

    #if ENABLE_ANTI_DRIFT
    case 'x': Kpos = val; break;
    case 'y': Kvel = val; break;
    case 'o': maxTiltOffset = fabsf(val); break;
    case 'z':
      pos_m = 0; vel_mps = 0;
      tiltOffset = 0; targetDyn = targetAngle;
      iTerm = 0; dFilt = 0;
      break;
    #endif

    case '?':
      lastDebugTime = 0;
      printDebug();
      return;
  }

  Serial.println("OK");
  if (debugToLINK) LINK.println("OK");
  iTerm = 0;
}

static inline void feedChar(char c) {
  if (c == '\r') return;
  if (c == '\n') {
    cmdBuf[cmdLen] = 0;
    if (cmdLen > 0) parseCommand(cmdBuf);
    cmdLen = 0;
  } else if (cmdLen < 63) {
    cmdBuf[cmdLen++] = c;
  }
}

void handleSerial() {
  while (Serial.available()) feedChar((char)Serial.read());
  while (LINK.available())  feedChar((char)LINK.read());
}

// ================= 15. SETUP & LOOP =================
void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_LEFT_PIN, OUTPUT);
  pinMode(STEP_LEFT_PIN, OUTPUT);
  pinMode(DIR_RIGHT_PIN, OUTPUT);
  pinMode(STEP_RIGHT_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  Serial.begin(115200);
  LINK.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);

  // --- I2C 0: MPU ---
  Wire.begin(21, 22);
  Wire.setClock(400000);
  Wire.setTimeOut(3000);

  // --- I2C 1: OLED ---
  I2C_OLED.begin(OLED_SDA_PIN, OLED_SCL_PIN, 400000);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED NOT FOUND");
    hasOLED = false;
  } else {
    hasOLED = true;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("Boot Kalman...");
    display.display();
  }

  setupPWM(STEP_LEFT_PIN, LEDC_CHAN_L);
  setupPWM(STEP_RIGHT_PIN, LEDC_CHAN_R);

  Serial.println("Init MPU (NO DMP) ...");
  mpu.initialize();
  if (!mpu.testConnection()) Serial.println("MPU FAIL");

  // Tối ưu nhiễu: bật DLPF
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);

  // Set full-scale để đúng hệ số GYRO_LSB_PER_DPS
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);   // 2000 dps => 16.4
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);    // +-2g

  // Nếu bạn đã đo offset chuẩn thì giữ, không thì comment để tự tune sau
  mpu.setXGyroOffset(120);
  mpu.setYGyroOffset(130);
  mpu.setZGyroOffset(-45);
  mpu.setZAccelOffset(707);

  // Kalman init
  kalmanInited = false;
  lastImuUs = 0;

  performAutoCalibration();

  Serial.println("READY!");
  if (debugToLINK) LINK.println("READY!");
}

void loop() {
  handleSerial();
  handleIMU_Kalman();
  handleOLED();
  printDebug();
}
