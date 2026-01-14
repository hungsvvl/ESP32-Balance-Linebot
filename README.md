# ESP32 Balance LineBot (MPU-6050 + DRV8825)

Xe 2 bánh tự cân bằng , dùng ESP32 điều khiển động cơ qua DRV8825, lấy góc/nghiêng từ MPU-6050 .

## Phần cứng chính
- ESP32 DevKit
- MPU-6050 (I2C)
- 2x Driver DRV8825 (STEP/DIR) cho 2 bánh trái/phải

---

## Sơ đồ chân (Pinout)

### 1) KHỐI ĐỘNG CƠ (DRIVER DRV8825)

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

### 2) KHỐI CẢM BIẾN GÓC (MPU-6050)
> Sử dụng giao tiếp I2C.

| MPU-6050 | Chân ESP32 | Chức năng | Ghi chú |
|---|---:|---|---|
| VCC (VIN) | 3.3V | Nguồn | Không cắm 5V |
| GND | GND | Mass | Nối chung mass hệ thống |
| SCL | GPIO 22 | I2C Clock |  |
| SDA | GPIO 21 | I2C Data |  |
| AD0 / SDO | GND | Chọn địa chỉ | Set địa chỉ là **0x68** |

---


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

// --- MPU PINS (I2C MPU-6050) ---
#define I2C_SDA_PIN    21
#define I2C_SCL_PIN    22
#define IMU_ADDRESS    0x68 // AD0 nối GND

