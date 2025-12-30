#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <TM1637Display.h>
#include <time.h>
#include <ESP32Servo.h>

// ================= WiFi & MQTT =================
const char *ssid = "Wokwi-GUEST";
const char *password = "";
const char *mqtt_server = "switchback.proxy.rlwy.net";
const int mqtt_port = 35495;

// Cấu hình NTP
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600; // GMT+7 (Việt Nam)
const int daylightOffset_sec = 0;

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ================= ID Thiết bị =================
String tempHumidSensorId = "BR_DHT22_01";

// ================= Structs & Thiết bị =================
struct LightDevice
{
  String id;
  int pin;
  bool status;
  bool lastPublishLed;
  bool turnedOnByPIR;
};

struct DoorDevice
{
  String id;
  int pin;
  bool locked;
  bool lastPublishDoor;
  Servo servo;
};

LightDevice lights[] = {
    {"BR_Light_01", 26, false, false, false},
    {"BR_Light_02", 12, false, false, false},
};

DoorDevice doors[] = {
    {"BR_Door_01", 15, true, false, Servo()},
};

// ========== Chủ đề MQTT ==========
// Chủ đề dữ liệu cảm biến (pub)
String topicSensor = "bedroom/sensor-device";
String topicStatus = "bedroom/current-status";

// Chủ đề lệnh (sub)
String topicCommandLightPrefix = "bedroom/command/light/";
String topicCommandDoorPrefix = "bedroom/command/door/";
String topicGetSensorData = "bedroom/command/get-sensor-data";

// Chủ đề đăng ký thiết bị (pub)
String topicDeviceRegister = "bedroom/device-register";
// Chủ đề trạng thái (pub)
String topicLightStatusPrefix = "bedroom/device-status/light/";
String topicDoorStatusPrefix = "bedroom/device-status/door/";

// ========== Chân Phần cứng ==========
const int DHT_PIN = 19;    // Chân dữ liệu DHT22
const int PIR_PIN = 25;    // Cảm biến chuyển động PIR
const int BUTTON_PIN = 13; // Nút nhấn
const int TM1637_CLK = 14; // CLK 7 đoạn
const int TM1637_DIO = 27; // DIO 7 đoạn
const int LCD_SDA = 21;    // SDA I2C LCD
const int LCD_SCL = 22;    // SCL I2C LCD

// ========== Đối tượng ==========
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHT_PIN, DHT22);
TM1637Display display(TM1637_CLK, TM1637_DIO);

// ========== Biến ==========
// Dữ liệu cảm biến
float temperature = 0.0;
float humidity = 0.0;
bool pirMotionDetected = false;

// Giá trị đã xuất bản lần cuối
float lastPublishedTemp = -999.0;
float lastPublishedHum = -999.0;

// Hẹn giờ tự động tắt PIR
unsigned long ledAutoOffTime = 0;
const unsigned long LED_AUTO_OFF_DELAY = 3000;

// Theo dõi thời gian
struct tm timeinfo;
bool timeSynced = false;

// Thời gian
unsigned long lastDHTRead = 0;
unsigned long lastLCDUpdate = 0;
unsigned long lastTimeUpdate = 0;
const unsigned long DHT_INTERVAL = 2000;
const unsigned long LCD_INTERVAL = 1000;
const unsigned long TIME_INTERVAL = 1000;

// Xử lý nút nhấn
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Hẹn giờ tự động khóa cửa
unsigned long doorAutoLockTime = 0;
const unsigned long DOOR_AUTO_LOCK_DELAY = 3000;

// Tin nhắn LCD
unsigned long lcdMessageTimeout = 0;


// ========== HÀM HỖ TRỢ ==========

void showLCDMessage(const char *line1, const char *line2 = "", unsigned long duration = 1000)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  if (strlen(line2) > 0)
  {
    lcd.setCursor(0, 1);
    lcd.print(line2);
  }
  lcdMessageTimeout = millis() + duration;
}

void updateLCD()
{
  if (millis() < lcdMessageTimeout)
  {
    return;
  }

  lcd.clear();
  // Dòng 1: Nhiệt độ & Độ ẩm
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature, 1);
  lcd.print("C H:");
  lcd.print((int)humidity);
  lcd.print("%");

  // Dòng 2: LED: 00 Lock/Unlock
  lcd.setCursor(0, 1);
  lcd.print("LED:");
  lcd.print(lights[0].status ? "1" : "0");
  lcd.print(lights[1].status ? "1" : "0");
  lcd.print(" ");
  lcd.print(doors[0].locked ? "Lock" : "Unlock");
}

void readDHT()
{
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  if (!isnan(temp) && !isnan(hum))
  {
    bool tempChanged = abs(temp - lastPublishedTemp) > 0.1;
    bool humChanged = abs(hum - lastPublishedHum) > 1.0;

    temperature = temp;
    humidity = hum;

    // Chỉ xuất bản nếu nhiệt độ hoặc độ ẩm thay đổi
    if ((tempChanged || humChanged) && mqtt.connected())
    {
      String payload = "{";
      payload += "\"temperature\":" + String(temperature, 1) + ",";
      payload += "\"humidity\":" + String((int)humidity);
      payload += "}";

      mqtt.publish(topicSensor.c_str(), payload.c_str(), false);
      lastPublishedTemp = temperature;
      lastPublishedHum = humidity;

      Serial.print("Published temp-humid: ");
      Serial.println(payload);
    }
  }
  else
  {
    Serial.println("Failed to read DHT sensor!");
  }
}

void handlePIR()
{
  pirMotionDetected = digitalRead(PIR_PIN);

  if (pirMotionDetected)
  {
    // Phát hiện chuyển động - Bật đèn LED
    for (size_t i = 0; i < sizeof(lights) / sizeof(lights[0]); i++)
    {
      if (!lights[i].status)
      {
        lights[i].status = true;
        lights[i].turnedOnByPIR = true;
        digitalWrite(lights[i].pin, HIGH);

        mqtt.publish(
            (topicLightStatusPrefix + lights[i].id).c_str(),
            "ON",
            false);
        lights[i].lastPublishLed = lights[i].status;
      }
    }
    // Đặt lại hẹn giờ tự động tắt
    ledAutoOffTime = millis() + LED_AUTO_OFF_DELAY;
  }
}

void handleButton()
{
    int reading = digitalRead(BUTTON_PIN);

    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading == LOW && lastButtonState == HIGH) {
            // Nút nhấn được nhấn - Chuyển đổi LED đầu tiên
            lights[0].status = !lights[0].status;
            lights[0].turnedOnByPIR = false; // Tắt cờ PIR khi điều khiển thủ công
            digitalWrite(lights[0].pin, lights[0].status);
            ledAutoOffTime = 0; // Vô hiệu hóa tự động tắt khi được chuyển đổi thủ công

            Serial.print("Button: LED ");
            Serial.println(lights[0].status ? "ON" : "OFF");
            if (mqtt.connected())
            {
                mqtt.publish((topicLightStatusPrefix + lights[0].id).c_str(), lights[0].status ? "ON" : "OFF", false);
                lights[0].lastPublishLed = lights[0].status;
                Serial.println("Published led status!");
            }
            lcdMessageTimeout = 0;
            updateLCD();
        }
    }
    lastButtonState = reading;
}


// Tìm đèn theo id
LightDevice *getLightById(const String &id)
{
  for (size_t i = 0; i < sizeof(lights) / sizeof(lights[0]); i++)
    if (lights[i].id == id)
      return &lights[i];
  return nullptr;
}

// Tìm cửa theo id
DoorDevice *getDoorById(const String &id)
{
  for (size_t i = 0; i < sizeof(doors) / sizeof(doors[0]); i++)
  {
    if (doors[i].id == id)
      return &doors[i];
  }
  return nullptr;
}

// Xử lý lệnh đèn
void handleLightCommand(const String &topic, const String &message)
{
  int lastSlash = topic.lastIndexOf('/');
  String id = topic.substring(lastSlash + 1);
  LightDevice *light = getLightById(id);
  if (!light)
    return;

  if (message == "ON" || message == "1")
  {
    light->status = true;
    light->turnedOnByPIR = false;
  }
  else if (message == "OFF" || message == "0")
    light->status = false;
  else if (message == "TOGGLE")
    light->status = !light->status;

  digitalWrite(light->pin, light->status);
  ledAutoOffTime = 0; // Vô hiệu hóa tự động tắt PIR khi được điều khiển thủ công

  if (mqtt.connected())
  {
    String statusTopic = String(topicLightStatusPrefix) + light->id;
    mqtt.publish(statusTopic.c_str(), light->status ? "ON" : "OFF", false);
  }
  showLCDMessage(("LED " + light->id).c_str(), light->status ? "ON" : "OFF");
  Serial.println("LED " + light->id + ": " + (light->status ? "ON" : "OFF"));
}

void handleDoorCommand(const String &topic, const String &message)
{
  int lastSlash = topic.lastIndexOf('/');
  String id = topic.substring(lastSlash + 1);
  DoorDevice *door = getDoorById(id);
  if (!door)
    return;

  if (message == "LOCK")
  {
    door->locked = true;
    door->servo.write(0);
  }
  else if (message == "UNLOCK")
  {
    door->locked = false;
    door->servo.write(90);
    // Bắt đầu hẹn giờ tự động khóa lại
    doorAutoLockTime = millis() + DOOR_AUTO_LOCK_DELAY;
  }

  // Xuất bản trạng thái lên MQTT
  if (mqtt.connected())
  {
    String statusTopic = String(topicDoorStatusPrefix + door->id);
    mqtt.publish(statusTopic.c_str(), door->locked ? "LOCKED" : "UNLOCKED", false);
  }

  // Hiển thị trên LCD
  showLCDMessage(("Door: " + door->id).c_str(), door->locked ? "LOCKED" : "UNLOCKED");
  Serial.println("Door " + door->id + ": " + (door->locked ? "LOCKED" : "UNLOCKED"));
}

void display7SegmentTime()
{
  if (!getLocalTime(&timeinfo))
  {
    // Nếu thời gian không được đồng bộ, hiển thị "----"
    uint8_t data[] = {0x40, 0x40, 0x40, 0x40}; // Phân đoạn gạch ngang
    display.setSegments(data);
    return;
  }

  // Định dạng hiển thị: HH:MM (với dấu hai chấm nhấp nháy)
  int hour = timeinfo.tm_hour;
  int minute = timeinfo.tm_min;
  int second = timeinfo.tm_sec;

  // Hiển thị dấu hai chấm bằng cách sử dụng bit 7 của chữ số thứ hai
  bool showColon = (second % 2 == 0); // Nhấp nháy mỗi giây

  uint8_t data[4];
  data[0] = display.encodeDigit(hour / 10);
  data[1] = display.encodeDigit(hour % 10) | (showColon ? 0x80 : 0x00);
  data[2] = display.encodeDigit(minute / 10);
  data[3] = display.encodeDigit(minute % 10);

  display.setSegments(data);
}

void publishAllStatus()
{
  // Xuất bản dữ liệu cảm biến ban đầu
  String sensorPayload = "{";
  sensorPayload += "\"temperature\":" + String(temperature, 1) + ",";
  sensorPayload += "\"humidity\":" + String((int)humidity);
  sensorPayload += "}";
  mqtt.publish(topicSensor.c_str(), sensorPayload.c_str(), false);

  // Xuất bản trạng thái thiết bị ban đầu
  for (size_t i = 0; i < sizeof(lights) / sizeof(lights[0]); i++)
  {
    String statusTopic = String(topicLightStatusPrefix) + lights[i].id;
    mqtt.publish(statusTopic.c_str(), lights[i].status ? "ON" : "OFF", false);
    lights[i].lastPublishLed = lights[i].status;
  }

  for (size_t i = 0; i < sizeof(doors) / sizeof(doors[0]); i++)
  {
    String doorStatusTopic = String(topicDoorStatusPrefix) + doors[i].id;
    mqtt.publish(
        doorStatusTopic.c_str(),
        doors[i].locked ? "LOCKED" : "UNLOCKED",
        false);
    doors[i].lastPublishDoor = doors[i].locked;
  }

  lastPublishedTemp = temperature;
  lastPublishedHum = humidity;

  Serial.println("Initial status published");
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  String message = "";
  for (unsigned int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }

  Serial.print("MQTT [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  String topicStr = String(topic);

  if (topicStr == topicGetSensorData)
  {
    Serial.println("MQTT command: Get sensor data");
    lastPublishedTemp = -999.0; // Buộc xuất bản
    readDHT();
  }
  else if (topicStr.startsWith(topicCommandLightPrefix))
    handleLightCommand(topicStr, message);
  else if (topicStr.startsWith(topicCommandDoorPrefix))
    handleDoorCommand(topicStr, message);
}

void connectWiFi()
{
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30)
  {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("\nWiFi Connection Failed!");
  }
}

void connectMQTT()
{
  if (!mqtt.connected())
  {
    Serial.print("Connecting to MQTT...");
    String clientId = "ESP32_Bedroom_" + String(random(0xffff), HEX);

    if (mqtt.connect(
            clientId.c_str(),
            nullptr,
            nullptr,
            topicStatus.c_str(),
            1,
            false,
            "offline"
            ))
    {
      Serial.println("Connected!");

      mqtt.subscribe(topicGetSensorData.c_str());

      for (size_t i = 0; i < sizeof(doors) / sizeof(doors[0]); i++)
      {
        String doorCmdTopic = topicCommandDoorPrefix + doors[i].id;
        mqtt.subscribe(doorCmdTopic.c_str());
      }

      for (size_t i = 0; i < sizeof(lights) / sizeof(lights[0]); i++)
      {
        String cmdTopic = topicCommandLightPrefix + lights[i].id;
        mqtt.subscribe(cmdTopic.c_str());
      }

      mqtt.publish(topicStatus.c_str(), "online", false);

      String payload1 = "{";
      payload1 += "\"id\":\"" + tempHumidSensorId + "\",";
      payload1 += "\"name\":\"temperature and humidity sensor\",";
      payload1 += "\"type\":\"temp_humid_sensor\"";
      payload1 += "}";
      mqtt.publish(topicDeviceRegister.c_str(), payload1.c_str(), false);

      publishAllStatus();
    }
    else
    {
      Serial.print("Failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" retrying in 5s");
    }
  }
}

// ========== SETUP ==========
void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== BEDROOM SYSTEM STARTING ===");

  for (size_t i = 0; i < sizeof(lights) / sizeof(lights[0]); i++)
  {
    pinMode(lights[i].pin, OUTPUT);
    digitalWrite(lights[i].pin, LOW);
  }
  pinMode(PIR_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  for (size_t i = 0; i < sizeof(doors) / sizeof(doors[0]); i++)
  {
    doors[i].servo.attach(doors[i].pin);
    doors[i].servo.write(0); // Khóa cửa ban đầu
  }

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("BEDROOM SYSTEM");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  display.setBrightness(0x0f);
  display.clear();

  dht.begin();
  Serial.println("DHT22 initialized");

  connectWiFi();

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("Syncing time with NTP server...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    if (getLocalTime(&timeinfo))
    {
      timeSynced = true;
      Serial.println("\nTime synced!");
    }
    else
    {
      Serial.println("\nFailed to sync time");
    }
  }

  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(mqttCallback);

  delay(2000);
  readDHT();

  updateLCD();
  display7SegmentTime();

  Serial.println("=== BEDROOM SYSTEM READY ===");
}

// ========== MAIN LOOP ==========
void loop()
{
  unsigned long currentMillis = millis();

  if (WiFi.status() == WL_CONNECTED)
  {
    if (!mqtt.connected())
    {
      static unsigned long lastReconnectAttempt = 0;
      if (currentMillis - lastReconnectAttempt > 5000)
      {
        lastReconnectAttempt = currentMillis;
        connectMQTT();
      }
    }
    else
    {
      mqtt.loop();
    }
  }

  if (currentMillis - lastDHTRead >= DHT_INTERVAL)
  {
    lastDHTRead = currentMillis;
    readDHT();
  }

  handlePIR();

  if (ledAutoOffTime > 0 && currentMillis >= ledAutoOffTime)
  {
    Serial.println("PIR: Auto-off PIR lights");
    for (size_t i = 0; i < sizeof(lights) / sizeof(lights[0]); i++)
    {
      if (lights[i].status && lights[i].turnedOnByPIR)
      {
        lights[i].status = false;
        lights[i].turnedOnByPIR = false;
        digitalWrite(lights[i].pin, LOW);
        if (mqtt.connected())
        {
          String topic = topicLightStatusPrefix + lights[i].id;
          mqtt.publish(topic.c_str(), "OFF", false);
        }
      }
    }
    ledAutoOffTime = 0;
  }

  // Tự động khóa cửa sau khi hết thời gian
  if (doorAutoLockTime > 0 && currentMillis >= doorAutoLockTime)
  {
      for(size_t i = 0; i < sizeof(doors)/sizeof(doors[0]); ++i)
      {
          if(!doors[i].locked)
          {
              doors[i].locked = true;
              doors[i].servo.write(0);
              doorAutoLockTime = 0; // Đặt lại hẹn giờ
              showLCDMessage(("Door " + doors[i].id).c_str(), "AUTO LOCKED");
              if(mqtt.connected())
              {
                  mqtt.publish((topicDoorStatusPrefix + doors[i].id).c_str(), "LOCKED", false);
              }
              Serial.println("Door " + doors[i].id + ": AUTO-LOCKED");
          }
      }
  }


  if (currentMillis - lastLCDUpdate >= LCD_INTERVAL)
  {
    lastLCDUpdate = currentMillis;
    updateLCD();
  }

  if (currentMillis - lastTimeUpdate >= TIME_INTERVAL)
  {
    lastTimeUpdate = currentMillis;
    display7SegmentTime();
  }

  handleButton();

  delay(10);
}