/*
 * ROBOT CÂN BẰNG 2 BÁNH - ESP32
 * - Cảm biến: MPU6050 (DMP)
 * - Động cơ: Stepper Motor (Driver A4988/DRV8825)
 * - Hiển thị: OLED SSD1306
 * - Điều khiển: PID Controller
 * - Giao tiếp: UART2 với ESP32 phụ (Web Server)
 */

#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" // Thư viện DMP cho MPU6050
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================= 1. CẤU HÌNH GIAO TIẾP =================
// Cổng Serial2 để kết nối với ESP32 phụ (Remote Control qua Wifi)
#define LINK_RX 16
#define LINK_TX 17
HardwareSerial SerialLink(2); 

// ================= 2. CẤU HÌNH MÀN HÌNH OLED =================
#define OLED_SDA_PIN 25
#define OLED_SCL_PIN 26
// Tạo một bus I2C riêng (Bus 1) cho OLED nếu cần, hoặc dùng chung Bus 0
TwoWire I2C_OLED = TwoWire(1);

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_OLED, OLED_RESET);
bool hasOLED = false;

// ================= 3. CẤU HÌNH CHÂN (PINOUT) =================
// Chân điều khiển động cơ bước
#define EN_PIN 27          // Chân Enable (kích mức thấp để bật động cơ)
#define DIR_LEFT_PIN 13    // Chân hướng quay bánh trái
#define STEP_LEFT_PIN 12   // Chân xung bước bánh trái
#define DIR_RIGHT_PIN 15   // Chân hướng quay bánh phải
#define STEP_RIGHT_PIN 4   // Chân xung bước bánh phải
#define MPU_INT_PIN 19     // Chân ngắt từ MPU6050

// Kênh PWM (LEDC) cho ESP32
const uint8_t LEDC_CHAN_L = 0;
const uint8_t LEDC_CHAN_R = 1;
const uint32_t LEDC_RES_BITS = 10; // Độ phân giải PWM 10-bit
const uint32_t MIN_FREQ = 20;      // Tần số tối thiểu (tránh lỗi chia cho 0)

// ================= 4. THAM SỐ PID & ĐIỀU KHIỂN =================
// Các hệ số PID (Cần tinh chỉnh thực tế cho từng robot)
float Kp = 68.7f;  // Hệ số P: Phản ứng nhanh với sai số
float Ki = 20.0f;  // Hệ số I: Khắc phục sai số tĩnh (giúp đứng thẳng tuyệt đối)
float Kd = 1.6f;   // Hệ số D: Giảm rung lắc, làm mượt chuyển động

float targetAngle = 180.0f; // Góc cân bằng (sẽ được cập nhật sau khi Calib)
float speedScale = 12.0f;   // Hệ số quy đổi từ PID ra tốc độ động cơ
long maxSpeed = 15000;      // Tốc độ tối đa (steps/giây)

// Các tham số an toàn và lọc
float killErrorDeg = 40.0f; // Góc nghiêng tối đa cho phép (ngã quá thì tắt)
float iZoneDeg = 4.0f;      // Vùng cho phép tích phân I (chỉ tích I khi gần cân bằng)
float deadbandDeg = 0.0f;   // Vùng chết (bỏ qua sai số nhỏ)
float Kv = 0.015f;          // Hệ số phản hồi vận tốc (giúp xe không chạy vọt đi)
float dAlpha = 0.9f;        // Hệ số lọc thông thấp cho thành phần D

// ================= 5. BIẾN HỆ THỐNG MPU6050 =================
MPU6050 mpu;
bool dmpReady = false;          // Cờ báo DMP đã sẵn sàng
volatile bool mpuInterrupt = false; // Cờ báo ngắt
uint8_t mpuIntStatus;           // Trạng thái ngắt MPU
uint16_t packetSize;            // Kích thước gói dữ liệu DMP
uint16_t fifoCount;             // Số byte trong bộ đệm FIFO
uint8_t fifoBuffer[64];         // Bộ đệm chứa dữ liệu
Quaternion q;                   // Quaternion (đại số 4 chiều tính góc)
VectorFloat gravity;            // Vector trọng trường
float ypr[3];                   // Mảng chứa Yaw, Pitch, Roll (Góc)

// Hàm xử lý ngắt (Interrupt Service Routine)
void IRAM_ATTR dmpDataReady() { mpuInterrupt = true; }

// ================= 6. BIẾN TRẠNG THÁI ROBOT =================
float currentAngle = 180.0f;    // Góc hiện tại
float error = 0.0f;             // Sai số (Target - Current)
float lastError = 0.0f;         // Sai số lần trước (để tính D)
float iTerm = 0.0f;             // Thành phần tích phân
float pidOutput = 0.0f;         // Kết quả PID
float dFilt = 0.0f;             // Giá trị D đã lọc
float vLP = 0.0f;               // Vận tốc động cơ đã lọc (Low Pass)

volatile long motorSpeedL = 0;  // Tốc độ động cơ trái
volatile long motorSpeedR = 0;  // Tốc độ động cơ phải
uint32_t lastPidUs = 0;         // Thời gian tính PID lần cuối
uint32_t lastDebugTime = 0;     // Thời gian in Debug lần cuối
uint32_t lastOledUpdate = 0;    // Thời gian cập nhật màn hình
bool isRobotActive = false;     // Trạng thái hoạt động (True = Đang chạy)

// Bộ đệm nhận lệnh Serial (Tách riêng để tránh xung đột)
char cmdBufUSB[64];   // Cho cổng USB
uint8_t cmdLenUSB = 0;
char cmdBufLink[64];  // Cho cổng ESP phụ
uint8_t cmdLenLink = 0;

// Các hàm tiện ích kẹp giá trị (Clamp)
static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
static inline long clampl(long v, long lo, long hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

// ================= 7. HÀM CẤU HÌNH PWM & MOTOR =================
void setupPWM(uint8_t pin, uint8_t channel) {
  // Cấu hình LEDC cho ESP32 (Phiên bản mới 3.x và cũ 2.x)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(pin, MIN_FREQ, LEDC_RES_BITS);
#else
  ledcSetup(channel, MIN_FREQ, LEDC_RES_BITS);
  ledcAttachPin(pin, channel);
#endif
}

void setPWMFreq(uint8_t pin, uint8_t channel, uint32_t freq) {
  if (freq < MIN_FREQ) {
    // Tắt xung nếu tần số quá thấp
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(pin, 0);
#else
    ledcWrite(channel, 0);
#endif
    return;
  }
  // Giới hạn tần số tối đa để bảo vệ driver
  if (freq > 25000) freq = 25000;
  
  // Xuất xung
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWriteTone(pin, freq);
#else
  ledcWriteTone(channel, freq);
#endif
}

void applyMotorSpeed() {
  // Nếu robot chưa kích hoạt -> Tắt động cơ
  if (!isRobotActive) {
    setPWMFreq(STEP_LEFT_PIN, LEDC_CHAN_L, 0);
    setPWMFreq(STEP_RIGHT_PIN, LEDC_CHAN_R, 0);
    return;
  }
  
  long spL = motorSpeedL;
  long spR = motorSpeedR;

  // Điều khiển chiều quay (DIR Pin)
  if (spL != 0) digitalWrite(DIR_LEFT_PIN, (spL > 0) ? LOW : HIGH);
  if (spR != 0) digitalWrite(DIR_RIGHT_PIN, (spR > 0) ? HIGH : LOW);

  // Điều khiển tốc độ (STEP Pin)
  setPWMFreq(STEP_LEFT_PIN, LEDC_CHAN_L, abs(spL));
  setPWMFreq(STEP_RIGHT_PIN, LEDC_CHAN_R, abs(spR));
}

// ================= 8. HIỂN THỊ OLED =================
void handleOLED() {
  if (!hasOLED) return;
  // Cập nhật màn hình mỗi 250ms (4Hz) để không làm chậm Loop
  if (millis() - lastOledUpdate < 250) return;
  lastOledUpdate = millis();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Dòng 1: Trạng thái
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(isRobotActive ? "RUN" : "IDLE");
  
  // Dòng 1 (Góc phải): Giá trị P hiện tại
  display.setCursor(60, 0);
  display.print("P:"); display.print((int)Kp);
  
  // Dòng 2: Góc nghiêng hiện tại (Chữ to)
  display.setCursor(10, 12);
  display.setTextSize(2);
  display.printf("%.2f", currentAngle);
  display.display();
}

// ================= 9. GỬI DỮ LIỆU DEBUG =================
void printDebug() {
  if (millis() - lastDebugTime < 500) return; // 2Hz (500ms/lần)
  lastDebugTime = millis();
  
  // Tạo chuỗi thông tin: Góc, Target, Kp, Ki, Kd
  String logMsg = "A:" + String(currentAngle, 2) + 
                  " T:" + String(targetAngle, 2) + 
                  " Kp:" + String(Kp, 1) + 
                  " Ki:" + String(Ki, 1) + 
                  " Kd:" + String(Kd, 1) + "\r\n";
  
  // Gửi ra cả USB (cho người dùng xem) và ESP phụ (để hiển thị lên Web)
  Serial.print(logMsg);
  SerialLink.print(logMsg); 
}

// ================= 10. THUẬT TOÁN PID (TRÁI TIM CỦA ROBOT) =================
void computePID() {
  // Tính thời gian trôi qua (dt) chính xác bằng micros()
  uint32_t nowUs = micros();
  if (lastPidUs == 0) { lastPidUs = nowUs; return; }
  float dt = (nowUs - lastPidUs) * 1e-6f; // Chuyển sang giây
  lastPidUs = nowUs;
  dt = clampf(dt, 0.002f, 0.020f); // Giới hạn dt để tránh lỗi khi loop bị lag

  // 1. Tính sai số góc
  error = targetAngle - currentAngle;

  // 2. Kiểm tra an toàn: Nếu ngã quá 40 độ -> Tắt máy
  if (fabsf(error) > killErrorDeg) {
    isRobotActive = false;
    pidOutput = 0; iTerm = 0;
    motorSpeedL = motorSpeedR = 0;
    applyMotorSpeed();
    return;
  }

  // 3. Tự động kích hoạt (Auto Start): Nếu dựng xe đứng thẳng -> Bật máy
  if (!isRobotActive) {
    if (fabsf(error) < 2.0f) {
      isRobotActive = true; iTerm = 0;
    } else {
      return;
    }
  }

  // 4. Tính toán các thành phần PID
  // Deadband: Vùng chết (nếu sai số quá nhỏ thì bỏ qua để đỡ rung)
  if (fabsf(error) < deadbandDeg) {
    iTerm *= 0.95f; // Giảm dần I
    pidOutput = 0;
  } else {
    // I-Term (Tích phân): Chỉ tích lũy khi xe gần cân bằng (iZone)
    if (fabsf(error) < iZoneDeg) {
      iTerm += (Ki * error * dt);
      // Chống bão hòa (Anti-windup)
      float iLimit = ((float)maxSpeed) / speedScale;
      iTerm = clampf(iTerm, -iLimit, iLimit);
    } else {
      iTerm = 0;
    }

    // D-Term (Đạo hàm): Tốc độ thay đổi của sai số
    float dRaw = (error - lastError) / dt;
    // Lọc thông thấp cho D để giảm nhiễu kim (Spikes)
    dFilt = dAlpha * dFilt + (1.0f - dAlpha) * dRaw;
    lastError = error;

    // Tổng hợp PID
    pidOutput = (Kp * error) + iTerm + (Kd * dFilt);
    
    // Phản hồi vận tốc (Kv): Giảm overshoot
    vLP = 0.9f * vLP + 0.1f * (float)motorSpeedL;
    pidOutput -= Kv * vLP;
  }

  // 5. Chuyển đổi PID ra tốc độ động cơ
  long sp = (long)(pidOutput * speedScale);
  sp = clampl(sp, -maxSpeed, maxSpeed); // Giới hạn tốc độ tối đa
  motorSpeedL = motorSpeedR = sp;
  
  applyMotorSpeed();
}

// ================= 11. TỰ ĐỘNG CÂN CHỈNH (AUTO CALIBRATION) =================
void performAutoCalibration() {
  // Hiển thị thông báo đang Calib
  if (hasOLED) {
    display.clearDisplay(); display.setCursor(0, 0);
    display.print("CALIBRATING..."); display.display();
  }
  
  mpu.resetFIFO(); // Xóa bộ đệm cũ
  fifoCount = 0;
  long sumAngle = 0; int samples = 0; unsigned long startTime = millis();

  // Lấy 100 mẫu góc liên tục
  while (samples < 100) {
    if (millis() - startTime > 3000) { targetAngle = 180.0f; return; } // Timeout
    
    // Chờ dữ liệu MPU
    if (!mpuInterrupt && fifoCount < packetSize) {
      mpuIntStatus = mpu.getIntStatus(); fifoCount = mpu.getFIFOCount();
      if (fifoCount < packetSize) continue;
    }
    
    // Đọc và xử lý dữ liệu DMP
    mpuInterrupt = false; mpuIntStatus = mpu.getIntStatus(); fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) { mpu.resetFIFO(); continue; } // Xử lý tràn bộ đệm
    
    if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize); fifoCount -= packetSize;
      
      mpu.dmpGetQuaternion(&q, fifoBuffer); 
      mpu.dmpGetGravity(&gravity, &q); 
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
      // Cộng dồn góc
      sumAngle += (long)(ypr[1] * 180.0f / M_PI + 180.0f); samples++;
    }
  }
  // Tính trung bình để ra góc cân bằng chuẩn
  targetAngle = (float)sumAngle / 100.0f;
  
  // Gửi thông báo xong cho ESP phụ
  SerialLink.print("CALIB_DONE: "); SerialLink.println(targetAngle);
}

// ================= 12. HÀM XỬ LÝ DỮ LIỆU DMP =================
void handleDMP() {
  // Nếu chưa có ngắt hoặc chưa đủ dữ liệu thì thoát ngay
  if (!mpuInterrupt && fifoCount < packetSize) return;

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  // Kiểm tra lỗi tràn bộ đệm (Overflow)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    return;
  }

  // Nếu có dữ liệu mới
  if (mpuIntStatus & 0x02) {
    // [QUAN TRỌNG] Đọc hết bộ đệm đến gói cuối cùng để giảm độ trễ (Lag)
    while (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }

    // Giải mã dữ liệu Quaternion -> Góc Euler (Yaw, Pitch, Roll)
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Chuyển đổi Pitch (trục nghiêng) sang độ và gán vào currentAngle
    // +180 để đưa về dải dương (0..360) thay vì (-180..180) cho dễ xử lý
    currentAngle = ypr[1] * 180.0f / M_PI + 180.0f;
    
    // Gọi hàm tính toán PID ngay khi có góc mới
    computePID();
  }
}

// ================= 13. XỬ LÝ LỆNH SERIAL (PARSER) =================
void parseCommand(char* str) {
  char cmd = str[0];       // Ký tự đầu là lệnh (p, i, d, t...)
  float val = atof(str + 1); // Phần sau là giá trị số

  switch (cmd) {
    case 'p': Kp = val; break;
    case 'i': Ki = val; break;
    case 'd': Kd = val; break;
    case 't': targetAngle = val; break;
    case 's': speedScale = val; break;
    case 'm': maxSpeed = clampl((long)val, 1000, 25000); break;
    case 'b': deadbandDeg = val; break;
    case 'v': Kv = val; break;
    case 'c': performAutoCalibration(); break; // Lệnh 'c' để Calib lại
    case '?': lastDebugTime = 0; printDebug(); break; // Lệnh '?' để xem thông số
  }
  iTerm = 0; // Reset I khi thay đổi thông số để tránh giật
  
  // Phản hồi OK
  SerialLink.print("OK:"); SerialLink.print(cmd); SerialLink.println(val);
}

// Hàm đọc dữ liệu từ một Stream (USB hoặc UART2) vào bộ đệm riêng
static inline void handleOneStream(Stream& s, char* buf, uint8_t& len) {
  while (s.available()) {
    char c = (char)s.read();
    if (c == '\r') continue; // Bỏ qua ký tự xuống dòng CR
    if (c == '\n') {         // Gặp ký tự kết thúc LF -> Xử lý lệnh
      buf[len] = 0;
      if (len) parseCommand(buf);
      len = 0;
    } else if (len < 63) {
      buf[len++] = c;
    }
  }
}

void handleSerial() {
  handleOneStream(Serial, cmdBufUSB, cmdLenUSB);      // Xử lý lệnh từ USB
  handleOneStream(SerialLink, cmdBufLink, cmdLenLink); // Xử lý lệnh từ ESP phụ
}

// ================= 14. KHỞI TẠO (SETUP) =================
void setup() {
  // Cấu hình chân Output
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_LEFT_PIN, OUTPUT); pinMode(STEP_LEFT_PIN, OUTPUT);
  pinMode(DIR_RIGHT_PIN, OUTPUT); pinMode(STEP_RIGHT_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Bật Driver động cơ (LOW = Enable)

  // Khởi động Serial
  Serial.begin(115200); // USB
  SerialLink.begin(115200, SERIAL_8N1, LINK_RX, LINK_TX); // UART2 (ESP phụ)

  // Khởi động I2C
  Wire.begin(21, 22);
  Wire.setClock(400000); // Tốc độ cao 400kHz
  Wire.setTimeOut(3000);

  // Khởi động OLED và hiển thị thông báo "BOOTING"
  I2C_OLED.begin(OLED_SDA_PIN, OLED_SCL_PIN, 400000);
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    hasOLED = true; 
    display.clearDisplay(); 
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(10, 10);
    display.print("BOOTING.."); 
    display.display();
  }

  // Khởi động PWM cho động cơ
  setupPWM(STEP_LEFT_PIN, LEDC_CHAN_L);
  setupPWM(STEP_RIGHT_PIN, LEDC_CHAN_R);

  // Khởi động cảm biến MPU6050
  mpu.initialize();
  uint8_t devStatus = mpu.dmpInitialize();
  
  // Cài đặt Offset (Cần chỉnh lại cho từng cảm biến cụ thể)
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

    // === QUY TRÌNH AUTO CALIB KHI KHỞI ĐỘNG ===
    if(hasOLED) {
      // Đếm ngược 2 giây để người dùng chuẩn bị
      for(int i=2; i>0; i--) {
        display.clearDisplay(); display.setCursor(0, 0); 
        display.print("DONT MOVE"); 
        display.setCursor(50, 20); display.print(i);
        display.display(); 
        delay(1000);
      }
    }
    // Thực hiện cân chỉnh
    performAutoCalibration();
  }
}

// ================= 15. VÒNG LẶP CHÍNH (LOOP) =================
void loop() {
  if (!dmpReady) return; // Nếu MPU lỗi thì dừng
  handleSerial(); // Kiểm tra lệnh điều khiển
  handleDMP();    // Đọc cảm biến và chạy PID
  handleOLED();   // Cập nhật màn hình
  printDebug();   // Gửi dữ liệu debug
}
