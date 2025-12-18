

Hệ thống Smart Home đa phòng sử dụng ESP32, kết nối MQTT để giám sát và điều khiển các thiết bị trong nhà.

## 📋 Tổng quan

Dự án bao gồm 3 phòng được tự động hóa:
- **Bedroom** (Phòng ngủ) - Giám sát môi trường & điều khiển ánh sáng
- **Kitchen** (Nhà bếp) - Hệ thống cảnh báo gas & điều khiển thông gió
- **Living Room** (Phòng khách) - Hệ thống an ninh với keypad & servo door

## 🛠️ Công nghệ sử dụng

- **Hardware**: ESP32 DevKit
- **Platform**: PlatformIO
- **Communication**: MQTT (test.mosquitto.org:1883)
- **Sensors**: DHT22, Gas Sensor (MQ-2), Photoresistor
- **Displays**: LCD 16x2 (I2C), TM1637 7-segment
- **Actuators**: Servo Motor, LED, Buzzer

---

## 🛏️ BEDROOM - Phòng ngủ

### Hardware
- **ESP32** - Vi điều khiển chính
- **DHT22** (Pin 19) - Cảm biến nhiệt độ & độ ẩm
- **LDR** (Pin 34) - Cảm biến ánh sáng
- **LED** (Pin 26) - Đèn chiếu sáng
- **Button** (Pin 13) - Nút bấm điều khiển
- **LCD 16x2** (I2C: SDA=21, SCL=22) - Hiển thị thông tin
- **TM1637** (CLK=14, DIO=27) - Đồng hồ 7-segment

### Tính năng
- ⏰ Hiển thị thời gian thực (NTP sync GMT+7)
- 🌡️ Giám sát nhiệt độ & độ ẩm (DHT22)
- 💡 Đo cường độ ánh sáng (0-100%)
- 🔘 Điều khiển đèn bằng nút bấm hoặc MQTT
- 📊 LCD hiển thị: nhiệt độ, độ ẩm, trạng thái LED, ánh sáng

### MQTT Topics

**📥 Subscribe (Nhận lệnh):**
```
bedroom/command/light         # Điều khiển LED (ON/OFF/TOGGLE)
```

**📤 Publish (Gửi dữ liệu):**
```
bedroom/sensor/temperature    # Nhiệt độ (°C, thay đổi ≥0.1°C)
bedroom/sensor/humidity       # Độ ẩm (%, thay đổi ≥1%)
bedroom/sensor/light          # Ánh sáng (%, thay đổi ≥1%)
bedroom/status/led            # Trạng thái LED (ON/OFF)
```

---

## 🍳 KITCHEN - Nhà bếp

### Hardware
- **ESP32** - Vi điều khiển chính
- **DHT22** (Pin 27) - Cảm biến nhiệt độ & độ ẩm
- **Gas Sensor MQ-2** (Pin 33) - Cảm biến khí gas
- **LED White** (Pin 25) - Đèn chiếu sáng
- **LED Red** (Pin 26) - Đèn cảnh báo
- **Servo** (Pin 19) - Quạt/cửa sổ tự động
- **Buzzer** (Pin 18) - Còi báo động
- **Button** (Pin 12) - Nút bấm điều khiển
- **LCD 16x2** (I2C: SDA=21, SCL=22) - Hiển thị thông tin

### Tính năng
- 🔥 **Phát hiện gas tự động** với hệ thống cảnh báo đa cấp
- 🚨 **Auto Mode**: Khi phát hiện gas → Tự động bật đèn đỏ + còi + quạt
- 🌡️ Giám sát nhiệt độ & độ ẩm
- 💡 Điều khiển đèn chiếu sáng
- 🪟 Điều khiển quạt/cửa sổ (servo)
- 📊 LCD hiển thị: nhiệt độ, độ ẩm, gas, chế độ

### MQTT Topics

**📥 Subscribe (Nhận lệnh):**
```
kitchen/command/led           # Điều khiển đèn trắng (ON/OFF/TOGGLE)
kitchen/command/alarm         # Điều khiển đèn đỏ cảnh báo (ON/OFF/TOGGLE)
kitchen/command/fan           # Điều khiển quạt/cửa sổ (ON/OFF/TOGGLE)
kitchen/command/buzzer        # Điều khiển còi (ON/OFF)
kitchen/command/auto          # Chế độ tự động (ON/OFF/TOGGLE)
```

**📤 Publish (Gửi dữ liệu):**
```
kitchen/sensor/temperature    # Nhiệt độ (°C, thay đổi ≥0.5°C)
kitchen/sensor/humidity       # Độ ẩm (%, thay đổi ≥2%)
kitchen/sensor/gas            # Phát hiện gas (YES/NO)
kitchen/status/led            # Trạng thái đèn trắng (ON/OFF)
kitchen/status/alarm          # Trạng thái cảnh báo (ON/OFF)
kitchen/status/fan            # Trạng thái quạt (ON/OFF)
kitchen/status/auto           # Chế độ tự động (ON/OFF)
```

---

## 🛋️ LIVING ROOM - Phòng khách

### Hardware
- **ESP32** - Vi điều khiển chính
- **DHT22** (Pin 19) - Cảm biến nhiệt độ & độ ẩm
- **Keypad 4x4** (Rows: 4,5,16,17 | Cols: 32,33,23,0) - Bàn phím nhập mật khẩu
- **Servo** (Pin 25) - Khóa cửa điện tử
- **LED** (Pin 26) - Đèn chiếu sáng
- **Button** (Pin 13) - Nút bấm điều khiển
- **LCD 16x2** (I2C: SDA=21, SCL=22) - Hiển thị thông tin

### Tính năng
- 🔐 **Hệ thống khóa cửa thông minh** với mật khẩu
- ⌨️ **Keypad 4x4** để nhập mật khẩu (4-8 ký tự)
- 🚪 **Servo door**: LOCKED (0°) / UNLOCKED (90°)
- 🔑 **Đổi mật khẩu**: Nhấn 'D' trên keypad
- 🌡️ Giám sát nhiệt độ & độ ẩm
- 💡 Điều khiển đèn
- 💾 **Lưu mật khẩu vĩnh viễn** (Preferences/EEPROM)

### Keypad Commands
```
A         # Mở khóa/khóa cửa (nhập password)
D         # Đổi mật khẩu (nhập password cũ → password mới 2 lần)
#         # Xác nhận nhập
*         # Xóa/hủy
```

### MQTT Topics

**📥 Subscribe (Nhận lệnh):**
```
living-room/command/light     # Điều khiển LED (ON/OFF/TOGGLE)
living-room/command/door      # Điều khiển cửa (LOCK/UNLOCK)
living-room/command/password  # Đổi mật khẩu từ xa (4-8 ký tự)
```

**📤 Publish (Gửi dữ liệu):**
```
living-room/sensor/temperature    # Nhiệt độ (°C, thay đổi ≥0.5°C)
living-room/sensor/humidity       # Độ ẩm (%, thay đổi ≥2%)
living-room/status                # Trạng thái online
living-room/status/led            # Trạng thái LED (ON/OFF)
living-room/status/door           # Trạng thái cửa (LOCKED/UNLOCKED)
living-room/status/password       # Mật khẩu mới (khi đổi)
```

---

## 🚀 Cài đặt & Sử dụng

### 1. Yêu cầu
```bash
# Cài đặt PlatformIO
pip install platformio

# Clone project
git clone <repository-url>
cd Projects
```

### 2. Build & Upload

**Bedroom:**
```bash
cd bedroom
pio run --target upload
```

**Kitchen:**
```bash
cd kitchen
pio run --target upload
```

**Living Room:**
```bash
cd livingroom
pio run --target upload
```

### 3. Cấu hình WiFi & MQTT

Mặc định sử dụng:
- **WiFi**: `Wokwi-GUEST` (no password)
- **MQTT Broker**: `test.mosquitto.org:1883` (public)

Để thay đổi, chỉnh sửa trong `src/main.cpp`:
```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* mqtt_server = "YOUR_MQTT_BROKER";
```

### 4. Test MQTT

Sử dụng MQTT client (MQTT Explorer, mosquitto_pub/sub):

```bash
# Subscribe tất cả topics
mosquitto_sub -h test.mosquitto.org -t "#" -v

# Điều khiển đèn bedroom
mosquitto_pub -h test.mosquitto.org -t "bedroom/command/light" -m "ON"

# Phát hiện gas kitchen (test)
mosquitto_pub -h test.mosquitto.org -t "kitchen/command/auto" -m "ON"

# Mở cửa living room
mosquitto_pub -h test.mosquitto.org -t "living-room/command/door" -m "UNLOCK"
```

---

## 📊 Kiến trúc hệ thống

```
┌─────────────────────────────────────────────────────────┐
│             MQTT Broker (test.mosquitto.org)            │
└─────────────────────────────────────────────────────────┘
           ↑                  ↑                  ↑
           │                  │                  │
    ┌──────┴──────┐    ┌──────┴──────┐    ┌──────┴──────┐
    │   BEDROOM   │    │   KITCHEN   │    │ LIVING ROOM │
    │   ESP32     │    │   ESP32     │    │   ESP32     │
    ├─────────────┤    ├─────────────┤    ├─────────────┤
    │ DHT22       │    │ DHT22       │    │ DHT22       │
    │ LDR         │    │ Gas Sensor  │    │ Keypad 4x4  │
    │ LED         │    │ 2x LED      │    │ Servo Door  │
    │ Button      │    │ Servo Fan   │    │ LED         │
    │ LCD 16x2    │    │ Buzzer      │    │ Button      │
    │ TM1637      │    │ Button      │    │ LCD 16x2    │
    │             │    │ LCD 16x2    │    │             │
    └─────────────┘    └─────────────┘    └─────────────┘
```

---

## 📁 Cấu trúc thư mục

```
Projects/
├── README.md                    # File này
├── bedroom/
│   ├── diagram.json            # Wokwi simulation
│   ├── platformio.ini          # PlatformIO config
│   ├── wokwi.toml             # Wokwi config
│   └── src/
│       └── main.cpp           # Source code
├── kitchen/
│   ├── diagram.json
│   ├── platformio.ini
│   ├── wokwi.toml
│   └── src/
│       └── main.cpp
└── livingroom/
    ├── diagram.json
    ├── platformio.ini
    ├── wokwi.toml
    └── src/
        └── main.cpp
```

---

## 🔧 Thư viện sử dụng

### Bedroom
```ini
knolleary/PubSubClient@^2.8
marcoschwartz/LiquidCrystal_I2C@^1.1.4
adafruit/DHT sensor library@^1.4.4
adafruit/Adafruit Unified Sensor@^1.1.14
smougenot/TM1637@0.0.0-alpha+sha.9486982048
```

### Kitchen
```ini
knolleary/PubSubClient@^2.8
marcoschwartz/LiquidCrystal_I2C@^1.1.4
adafruit/DHT sensor library@^1.4.4
adafruit/Adafruit Unified Sensor@^1.1.14
ESP32Servo library
```

### Living Room
```ini
knolleary/PubSubClient@^2.8
marcoschwartz/LiquidCrystal_I2C@^1.1.4
adafruit/DHT sensor library@^1.4.4
adafruit/Adafruit Unified Sensor@^1.1.14
ESP32Servo library
Keypad library
```

---

## 🎯 MQTT Topic Pattern

Toàn bộ hệ thống tuân theo chuẩn topic pattern:

```
<room>/sensor/<sensor_name>      # Dữ liệu cảm biến
<room>/command/<device_name>     # Lệnh điều khiển thiết bị
<room>/status/<device_name>      # Trạng thái thiết bị
```

**Ví dụ:**
- `bedroom/sensor/temperature` → Dữ liệu nhiệt độ
- `kitchen/command/led` → Lệnh bật/tắt đèn
- `living-room/status/door` → Trạng thái khóa cửa

---

## 🐛 Debug & Monitor

```bash
# Monitor Serial output
pio device monitor -b 115200

# Monitor bedroom
cd bedroom && pio device monitor

# Monitor kitchen
cd kitchen && pio device monitor

# Monitor living room
cd livingroom && pio device monitor
```

---

## 📝 License

MIT License - Free to use & modify

---

## 👨‍💻 Author

Smart Home IoT System - ESP32 & MQTT
Built with PlatformIO & Arduino Framework

---

## 🔗 Links

- [PlatformIO](https://platformio.org/)
- [ESP32 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [MQTT.org](https://mqtt.org/)
- [Wokwi Simulator](https://wokwi.com/)

---

**Happy Coding! 🚀**
