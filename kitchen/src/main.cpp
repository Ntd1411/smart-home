#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <ESP32Servo.h>

#define BUZZER_CHANNEL 4 // khác channel servo
#define BUZZER_FREQ 1000 // 1kHz
#define BUZZER_RES 8     // 8-bit

// WiFi & MQTT Configuration
const char *ssid = "Wokwi-GUEST";
const char *password = "";
const char *mqtt_server = "switchback.proxy.rlwy.net";
const int mqtt_port = 35495;

WiFiClient espClient;
PubSubClient mqtt(espClient);

String tempHumidSensorId = "K_DHT22_01";
String gasSensorId = "K_GAS_01";

struct LightDevice
{
  String id;
  int pin;
  bool status;
  bool lastPublishLed;
};

struct WindowDevice
{
  String id;
  int pin;
  bool opened;
  Servo servo;
  bool lastPublishFan;
};

LightDevice lights[] = {
    {"K_Light_01", 25, false, false}, // white with button
    {"K_Light_02", 26, false, false}, // red warning light
                                      // thêm đèn ở đây
};

WindowDevice windows[] = {
    {"K_Window_01", 19, false, Servo()},
    // thêm cửa sổ ở đây
};

/* ================= MQTT Topics ================= */
// pub
String topicSensor = "kitchen/sensor-device";
String topicStatus = "kitchen/current-status";
String topicDeviceRegister = "kitchen/device-register";

// sub
String topicCommandLightPrefix = "kitchen/command/light/";
String topicCommandWindowPrefix = "kitchen/command/window/";
String topicCommandAuto = "kitchen/command/auto";

String topicGetSensorData = "kitchen/command/get-sensor-data";

// status pub
String topicLightStatusPrefix = "kitchen/device-status/light/";
String topicWindowStatusPrefix = "kitchen/device-status/window/";
String topicAutoStatus = "kitchen/device-status/auto";

#define BUTTON_PIN 12
#define DHT_PIN 27
#define GAS_PIN 33

// Hardware pins (theo diagram.json kitchen)
const int BUZZER_PIN = 18; // Buzzer
const int LCD_SDA = 21;    // LCD I2C SDA
const int LCD_SCL = 22;    // LCD I2C SCL

// Servo angles
const int SERVO_CLOSE_ANGLE = 0;  // Góc đóng quạt/cửa sổ
const int SERVO_OPEN_ANGLE = 180; // Góc mở quạt/cửa sổ

// Objects
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHT_PIN, DHT22);

// Variables
float temperature = 0.0;
float humidity = 0.0;
bool gasDetected = false; // Gas detected (digital)

bool buzzerStatus = false; // Buzzer cảnh báo
bool autoMode = true;      // Chế độ tự động

// Previous values for change detection
float lastPublishedTemp = -999.0;
float lastPublishedHum = -999.0;
bool lastPublishedGas = false;

// Button handling
bool lastButtonState = HIGH;
bool buttonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 20;

// Timing
unsigned long lastDHTRead = 0;
unsigned long lastLCDUpdate = 0;
unsigned long lastGasCheck = 0;
unsigned long systemStartTime = 0;

const unsigned long DHT_INTERVAL = 2000;
const unsigned long LCD_INTERVAL = 1000;
const unsigned long GAS_INTERVAL = 500;
const unsigned long ONLINE_INTERVAL = 1000;

bool isRemoteLightComand = false;
bool isRemoteWindowComand = false;

bool gasSensorReady = false;

unsigned long lcdMessageTimeout = 0;

void buzzerOn()
{
  ledcWrite(BUZZER_CHANNEL, 128); // 50% duty
  buzzerStatus = true;
}
void buzzerOff()
{
  ledcWrite(BUZZER_CHANNEL, 0);
  buzzerStatus = false;
}

// ========== HELPER FUNCTIONS ==========

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

void readDHT()
{
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  if (!isnan(temp) && !isnan(hum))
  {
    bool tempChanged = temp != lastPublishedTemp;
    bool humChanged = hum != lastPublishedHum;

    temperature = temp;
    humidity = hum;

    // Publish unified sensor data if any value changed
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

void readGasSensor()
{
  // Đọc giá trị digital (LOW = gas detected, HIGH = no gas)
  gasDetected = (analogRead(GAS_PIN) == 0);

  // Publish if gas status changed
  if (mqtt.connected() && gasDetected != lastPublishedGas)
  {
    String payload = "{";
    payload += "\"temperature\":" + String(temperature, 1) + ",";
    payload += "\"humidity\":" + String((int)humidity) + ",";
    payload += "\"gas\":" + String(gasDetected ? "true" : "false");
    payload += "}";
    mqtt.publish(topicSensor.c_str(), payload.c_str(), false);
    lastPublishedGas = gasDetected;
    Serial.print("Gas Detected: ");
    Serial.println(gasDetected ? "YES" : "NO");
  }
  // Auto mode: handle gas warning
  if (autoMode)
  {
    if (gasDetected)
    {
      // Gas phát hiện: Bật LED đỏ, buzzer, mở quạt
      if (!lights[1].status)
      {
        lights[1].status = true;
        digitalWrite(lights[1].pin, HIGH); // bật LED đỏ
      }
      if (!buzzerStatus)
      {
        buzzerOn(); // Bật buzzer với PWM
      }
      for (int i = 0; i < sizeof(windows) / sizeof(windows[0]); i++)
      {
        if (!windows[i].opened)
        {
          windows[i].opened = true;
          windows[i].servo.write(SERVO_OPEN_ANGLE); // Mở quạt/cửa sổ
          String windowTopic = String(topicWindowStatusPrefix + windows[i].id);
          mqtt.publish(windowTopic.c_str(), "OPENED");
        }
      }
    }
    else
    {
      // Không có gas: Tắt tất cả cảnh báo
      if (lights[1].status && !isRemoteLightComand)
      {
        lights[1].status = false;
        digitalWrite(lights[1].pin, LOW); // bật LED đỏ
        buzzerOff();
      }
      if (buzzerStatus)
      {
        buzzerOff(); // Tắt buzzer
      }
      for (int i = 0; i < sizeof(windows) / sizeof(windows[0]); i++)
      {
        if (windows[i].opened && !isRemoteWindowComand)
        {
          windows[i].opened = false;
          windows[i].servo.write(SERVO_CLOSE_ANGLE); // Đóng quạt/cửa sổ
          String windowTopic = String(topicWindowStatusPrefix + windows[i].id);
          mqtt.publish(windowTopic.c_str(), "CLOSED");
        }
      }
    }
  }
}

void updateLCD()
{
  lcd.clear();

  // Line 1: Temperature & Humidity
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature, 1);
  lcd.print("C H:");
  lcd.print((int)humidity);
  lcd.print("%");

  // Line 2: Gas status
  lcd.setCursor(0, 1);
  lcd.print("Gas:");

  if (gasDetected)
  {
    lcd.print("DETECTED!");
  }
  else
  {
    lcd.print("Safe");
  }
  if (autoMode)
  {
    lcd.setCursor(15, 1);
    lcd.print("A"); // Auto indicator
  }
}

void handleButton()
{
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState)
  {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if (reading != buttonState)
    {
      buttonState = reading;

      if (buttonState == LOW)
      {
        // Toggle LED white
        lights[0].status = !lights[0].status;
        digitalWrite(lights[0].pin, lights[0].status);
        Serial.print("Button: LED White ");
        Serial.println(lights[0].status ? "ON" : "OFF");

        // Publish LED status change
        if (mqtt.connected())
        {
          mqtt.publish((topicLightStatusPrefix + lights[0].id).c_str(), lights[0].status ? "ON" : "OFF", false);
          lights[0].lastPublishLed = lights[0].status;
          Serial.println("Published LED white status");
        }

        updateLCD();
      }
    }
  }

  lastButtonState = reading;
}

// Find light by id
LightDevice *getLightById(const String &id)
{
  for (int i = 0; i < sizeof(lights) / sizeof(lights[0]); i++)
    if (lights[i].id == id)
      return &lights[i];
  return nullptr;
}

// Tìm cửa theo id
WindowDevice *getWindowById(const String &id)
{
  for (int i = 0; i < sizeof(windows) / sizeof(windows[0]); i++)
  {
    if (windows[i].id == id)
      return &windows[i];
  }
  return nullptr;
}

void handleLightCommand(const String &topic, const String &message)
{
  int lastSlash = topic.lastIndexOf('/');
  String id = topic.substring(lastSlash + 1);

  LightDevice *light = getLightById(id);
  if (!light)
    return;

  if (message == "ON" || message == "1")
  {
    isRemoteLightComand = true;
    light->status = true;
  }
  else if (message == "OFF" || message == "0")
  {
    light->status = false;
    isRemoteLightComand = false;
  }
  else if (message == "TOGGLE")
    light->status = !light->status;

  digitalWrite(light->pin, light->status);

  // Publish trạng thái lên MQTT
  if (mqtt.connected())
  {
    String statusTopic = String(topicLightStatusPrefix + light->id);
    mqtt.publish(statusTopic.c_str(), light->status ? "ON" : "OFF", false);
  }

  // Hiển thị trên LCD và Serial
  showLCDMessage(("LED " + light->id).c_str(), light->status ? "ON" : "OFF");
  Serial.println("LED " + light->id + ": " + (light->status ? "ON" : "OFF"));
}

void handleWindowCommand(const String &topic, const String &message)
{
  int lastSlash = topic.lastIndexOf('/');
  String id = topic.substring(lastSlash + 1);

  WindowDevice *window = getWindowById(id);
  if (!window)
    return;

  if (message == "OPEN" || message == "1")
  {
    isRemoteWindowComand = true;
    window->opened = true;
    window->servo.write(SERVO_OPEN_ANGLE);
  }
  else if (message == "CLOSE" || message == "0")
  {
    isRemoteWindowComand = false;
    window->opened = false;
    window->servo.write(SERVO_CLOSE_ANGLE);
  }
  else if (message == "TOGGLE")
  {
    window->opened = !window->opened;
    window->servo.write(window->opened ? SERVO_OPEN_ANGLE : SERVO_CLOSE_ANGLE);
  }

  // Publish trạng thái lên MQTT
  if (mqtt.connected())
  {
    String statusTopic = String(topicWindowStatusPrefix + window->id);
    mqtt.publish(statusTopic.c_str(), window->opened ? "OPENED" : "CLOSED", false);
  }

  // Hiển thị trên LCD và Serial
  showLCDMessage(("Window: " + window->id).c_str(), window->opened ? "OPENED" : "CLOSED");
  Serial.println("Window " + window->id + ": " + (window->opened ? "OPENED" : "CLOSED"));
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  String message = "";
  for (int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }

  Serial.print("MQTT [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  String topicStr = String(topic);

  // Get sensor data command
  if (topicStr == topicGetSensorData)
  {
    Serial.println("MQTT command: Get sensor data");
    readDHT();
    readGasSensor();
    return;
  }

  // Control lights
  else if (topicStr.startsWith(topicCommandLightPrefix))
    handleLightCommand(topicStr, message);

  // Control windows
  else if (topicStr.startsWith(topicCommandWindowPrefix))
    handleWindowCommand(topicStr, message);

  // Control auto mode
  else if (topicStr == topicCommandAuto)
  {
    if (message == "ON" || message == "1")
    {
      autoMode = true;
    }
    else if (message == "OFF" || message == "0")
    {
      autoMode = false;
    }
    else if (message == "TOGGLE")
    {
      autoMode = !autoMode;
    }

    // mqtt.publish(topicAutoStatus.c_str(), autoMode ? "ON" : "OFF");
    Serial.print("Auto mode: ");
    Serial.println(autoMode ? "ON" : "OFF");
    updateLCD();
  }
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
    String clientId = "ESP32_Kitchen_" + String(random(0xffff), HEX);

    if (mqtt.connect(
            clientId.c_str(),
            nullptr,
            nullptr,
            topicStatus.c_str(), // LWT topic
            1,                   // QoS
            false,               // retained
            "offline"            // LWT message
            ))
    {
      Serial.println("Connected!");

      // Subscribe door commands & password responses per door
      for (int i = 0; i < sizeof(windows) / sizeof(windows[0]); i++)
      {
        String doorCmdTopic = topicCommandWindowPrefix + windows[i].id; // ví dụ: "living-room/command/door/LV_Door_01"
        mqtt.subscribe(doorCmdTopic.c_str());
      }

      // Subscribe light command topics
      for (int i = 0; i < sizeof(lights) / sizeof(lights[0]); i++)
      {
        String cmdTopic = topicCommandLightPrefix + lights[i].id;
        mqtt.subscribe(cmdTopic.c_str());
      }
      // mqtt.subscribe(topicCommandBuzzer.c_str());
      mqtt.subscribe(topicGetSensorData.c_str());
      mqtt.subscribe(topicCommandAuto.c_str());

      // Publish online status
      mqtt.publish(topicStatus.c_str(), "online", false);

      // Register temp/humidity sensor device
      String payload1 = "{";
      payload1 += "\"id\":\"" + tempHumidSensorId + "\",";
      payload1 += "\"name\":\"temperature and humidity sensor\",";
      payload1 += "\"type\":\"temp_humid_sensor\"";
      payload1 += "}";
      mqtt.publish(topicDeviceRegister.c_str(), payload1.c_str(), false);

      // Register gas sensor device
      String payload2 = "{";
      payload2 += "\"id\":\"" + gasSensorId + "\",";
      payload2 += "\"name\":\"gas sensor\",";
      payload2 += "\"type\":\"gas_sensor\"";
      payload2 += "}";
      mqtt.publish(topicDeviceRegister.c_str(), payload2.c_str(), false);

      // Publish unified sensor data
      String sensorPayload = "{";
      sensorPayload += "\"temperature\":" + String(temperature, 1) + ",";
      sensorPayload += "\"humidity\":" + String((int)humidity) + ",";
      sensorPayload += "\"gas\":" + String(gasDetected ? "true" : "false");
      sensorPayload += "}";
      mqtt.publish(topicSensor.c_str(), sensorPayload.c_str(), false);
      // mqtt.publish(topicAutoStatus.c_str(), autoMode ? "ON" : "OFF");

      // Publish device status
      for (int i = 0; i < sizeof(lights) / sizeof(lights[0]); i++)
      {
        String statusTopic = String(topicLightStatusPrefix) + lights[i].id;
        mqtt.publish(statusTopic.c_str(), lights[i].status ? "ON" : "OFF", false);
        lights[i].lastPublishLed = lights[i].status;
      }

      lastPublishedTemp = temperature;
      lastPublishedHum = humidity;
      lastPublishedGas = gasDetected;

      Serial.println("Initial status published to MQTT");
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
  Serial.println("\n\n=== KITCHEN SYSTEM STARTING ===");

  // Lưu thời điểm khởi động để tính warm-up
  systemStartTime = millis();

  // Initialize pins

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  for (int i = 0; i < sizeof(lights) / sizeof(lights[0]); i++)
  {
    pinMode(lights[i].pin, OUTPUT);
    digitalWrite(lights[i].pin, LOW);
  }
  pinMode(GAS_PIN, INPUT); // Digital input for gas sensor DOUT
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW); // Initialize buzzer off

  ledcSetup(BUZZER_CHANNEL, BUZZER_FREQ, BUZZER_RES);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  ledcWrite(BUZZER_CHANNEL, 0); // OFF

  // Initialize servo
  for (int i = 0; i < sizeof(windows) / sizeof(windows[0]); i++)
  {
    windows[i].servo.attach(windows[i].pin);
    windows[i].servo.write(0); // Lock door initially
  }

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("KITCHEN SYSTEM");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  // Initialize DHT sensor
  dht.begin();
  Serial.println("DHT22 initialized");

  // Connect to WiFi
  connectWiFi();

  // Setup MQTT
  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(mqttCallback);

  // Initial sensor readings
  readDHT();
  // Gas sensor sẽ tự động đọc sau warm-up time

  // Update display
  updateLCD();

  Serial.println("=== KITCHEN SYSTEM READY ===\n");
  Serial.println("Controls:");
  Serial.println("- Button: Toggle LED White ON/OFF");
  Serial.println("- Auto Mode: Automatic gas detection & response");
  Serial.println("- LCD: Shows temp, humidity, gas level");
  Serial.println("\nMQTT Topics:");
  Serial.println("  Subscribe:");
  Serial.println("    * kitchen/command/light (ON/OFF/TOGGLE)");
  Serial.println("    * kitchen/command/alarm (ON/OFF/TOGGLE)");
  Serial.println("    * kitchen/command/fan (ON/OFF/TOGGLE)");
  Serial.println("    * kitchen/command/buzzer (ON/OFF)");
  Serial.println("    * kitchen/command/auto (ON/OFF/TOGGLE)");
  Serial.println("  Publish:");
  Serial.println("    * kitchen/sensor/temperature (float)");
  Serial.println("    * kitchen/sensor/humidity (int)");
  Serial.println("    * kitchen/sensor/gas (YES/NO)");
  Serial.println("    * kitchen/status/light (ON/OFF)");
  Serial.println("    * kitchen/status/alarm (ON/OFF)");
  Serial.println("    * kitchen/status/fan (ON/OFF)");
  Serial.println("    * kitchen/status/auto (ON/OFF)");
  Serial.println();
}

// ========== MAIN LOOP ==========
void loop()
{
  unsigned long currentMillis = millis();

  // Maintain MQTT connection
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

  // Read DHT sensor periodically
  if (currentMillis - lastDHTRead >= DHT_INTERVAL)
  {
    lastDHTRead = currentMillis;
    readDHT();
  }

  // Read gas sensor frequently
  if (currentMillis - lastGasCheck >= GAS_INTERVAL)
  {
    lastGasCheck = currentMillis;
    readGasSensor();
  }

  // Update LCD display
  if (currentMillis - lastLCDUpdate >= LCD_INTERVAL)
  {
    lastLCDUpdate = currentMillis;
    updateLCD();
  }

  // Handle button input
  handleButton();

  delay(10);
}
