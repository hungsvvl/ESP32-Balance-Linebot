# ESP32-Balance-Linebot
# ESP32 Balance LineBot (ICM-20948 + DRV8825 + 7 Line Sensors)

Xe 2 bánh tự cân bằng kết hợp dò line, dùng ESP32 điều khiển động cơ qua DRV8825, lấy góc/nghiêng từ IMU ICM-20948 và đọc 7 mắt line theo thứ tự Trái → Phải.

## Phần cứng chính
- ESP32 DevKit (khuyến nghị loại 30 pin)
- IMU ICM-20948 (I2C)
- 2x Driver DRV8825 (STEP/DIR) cho 2 bánh trái/phải
- Cảm biến dò line 7 mắt

---

## Sơ đồ chân (Pinout)

### 1) KHỐI ĐỘNG CƠ (DRIVER DRV8825)
> Giữ nguyên như cũ. Lưu ý tụ lọc nguồn.

**Gợi ý:** Xe 2 bánh thường dùng **2 driver DRV8825** (Trái + Phải).  
Các chân **VMOT/GND/RST+SLP/M0-M2/EN/GND logic** giống nhau, chỉ khác **DIR/STEP**.

#### Driver BÁNH TRÁI (LEFT)
| Chân Driver | Nối đi đâu? | Chức năng | Ghi chú quan trọng |
|---|---|---|---|
| VMOT (+) | 12V Pin | Nguồn động lực | ⚠️ Cần tụ **100uF/35V** song song gần VMOT-GND |
| GND (To) | (-) GND Pin | Mass động lực | Nối mass nguồn motor chắc chắn |
| 2B, 2A | Dây Motor (Cặp 1) | Ra động cơ | (Thường Xanh lá - Đen) |
| 1A, 1B | Dây Motor (Cặp 2) | Ra động cơ | (Thường Đỏ - Xanh dương) |
| RST + SLP | Nối tắt nhau | Bật Driver | Cắm jumper nối dính 2 chân này |
| M0, M1, M2 | (+) 3.3V | Microstepping | Kích hoạt chế độ **1/32 bước** |
| EN | GND | Enable | Luôn BẬT động cơ (EN active-low) |
| GND (Nhỏ) | GND ESP32 | Mass logic | **Bắt buộc nối chung** với ESP32 |
| DIR | GPIO 13 | Hướng bánh trái |  |
| STEP | GPIO 12 | Xung bước bánh trái |  |

#### Driver BÁNH PHẢI (RIGHT)
| Chân Driver | Nối đi đâu? | Chức năng | Ghi chú quan trọng |
|---|---|---|---|
| VMOT (+) | 12V Pin | Nguồn động lực | ⚠️ Cần tụ **100uF/35V** song song gần VMOT-GND |
| GND (To) | (-) GND Pin | Mass động lực | Nối mass nguồn motor chắc chắn |
| 2B, 2A | Dây Motor (Cặp 1) | Ra động cơ | (Thường Xanh lá - Đen) |
| 1A, 1B | Dây Motor (Cặp 2) | Ra động cơ | (Thường Đỏ - Xanh dương) |
| RST + SLP | Nối tắt nhau | Bật Driver | Cắm jumper nối dính 2 chân này |
| M0, M1, M2 | (+) 3.3V | Microstepping | Kích hoạt chế độ **1/32 bước** |
| EN | GND | Enable | Luôn BẬT động cơ (EN active-low) |
| GND (Nhỏ) | GND ESP32 | Mass logic | **Bắt buộc nối chung** với ESP32 |
| DIR | GPIO 15 | Hướng bánh phải |  |
| STEP | GPIO 4 | Xung bước bánh phải |  |

---

### 2) KHỐI CẢM BIẾN GÓC (IMU ICM-20948)
> Sử dụng giao tiếp I2C.

| Chân ICM-20948 | Chân ESP32 | Chức năng | Ghi chú |
|---|---:|---|---|
| VCC (VIN) | 3.3V | Nguồn | Không cắm 5V |
| GND | GND | Mass | Nối chung mass hệ thống |
| SCL | GPIO 22 | I2C Clock |  |
| SDA | GPIO 21 | I2C Data |  |
| AD0 / SDO | GND | Chọn địa chỉ | Set địa chỉ là **0x68** |

---

### 3) KHỐI DÒ LINE (7 MẮT)
> Thứ tự tính từ bên TRÁI xe sang bên PHẢI xe.

| Mắt Line | Chân ESP32 | Trọng số (PID) | Loại chân |
|---|---:|---:|---|
| Mắt 1 (Trái ngoài) | GPIO 36 | -3.0 | Input Only (VP) |
| Mắt 2 | GPIO 39 | -2.0 | Input Only (VN) |
| Mắt 3 | GPIO 34 | -1.0 | Input Only |
| Mắt 4 (Giữa) | GPIO 35 | 0.0 | Input Only |
| Mắt 5 | GPIO 32 | 1.0 | I/O Pin |
| Mắt 6 | GPIO 33 | 2.0 | I/O Pin |
| Mắt 7 (Phải ngoài) | GPIO 25 | 3.0 | I/O Pin |

**Note nhanh:**
- GPIO 34/35/36/39 là **input-only** (chuẩn cho cảm biến).
- GPIO 25 thuộc ADC2, nếu sau này dùng WiFi mà ADC “khó ở” thì cân nhắc chuyển mắt 7 sang ADC1 (32–39).

---

## SPI Bus (mặc định ESP32 SPI)
Dùng cho module SPI (nếu có gắn thêm: màn hình/LoRa/SD...):
- **SCK = GPIO18**
- **MISO = GPIO19**
- **MOSI = GPIO23**
> CS (Chip Select) tùy module, chọn chân khác.

---

## Pin defines (tham chiếu code)
```cpp
// --- MOTOR PINS (DRV8825) ---
#define DIR_LEFT_PIN   13
#define STEP_LEFT_PIN  12
#define DIR_RIGHT_PIN  15
#define STEP_RIGHT_PIN  4

// --- IMU PINS (I2C ICM-20948) ---
#define I2C_SDA_PIN    21
#define I2C_SCL_PIN    22
#define IMU_ADDRESS    0x68 // AD0 nối GND

// --- LINE SENSOR PINS (7 Eyes) ---
// Thứ tự: Trái -> Phải
const int lineSensorPins[7] = {36, 39, 34, 35, 32, 33, 25};

// SPI defaults: SCK=18, MISO=19, MOSI=23 (ESP32 SPI)
