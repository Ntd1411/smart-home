#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <Keypad.h>
#include <Preferences.h>

// ================= WiFi & MQTT =================
const char *ssid = "Wokwi-GUEST";
const char *password = "";
const char *mqtt_server = "switchback.proxy.rlwy.net";
const int mqtt_port = 35495;

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ================= Device IDs =================
String tempHumidSensorId = "LV_DHT22_01";

// ================= Structs & Devices =================
struct LightDevice
{
  String id;
  int pin;
  bool status;
  bool lastPublishLed;
};

struct DoorDevice
{
  String id;
  int pin;
  bool locked;
  bool lastPublishDoor;
  String password;
  Servo servo;
};

LightDevice lights[] = {
    {"LV_Light_01", 26, false, false},
    {"LV_Light_02", 27, false, false},
    {"LV_Light_03", 14, false, false}
};

DoorDevice doors[] = {
    {"LV_Door_01", 25, true, false, "1234", Servo()},
};

// ========== MQTT Topics ==========
// sensor data topic (pub)
String topicSensor = "living-room/sensor-device";
String topicStatus = "living-room/current-status";

// command topics (sub)
String topicCommandLightPrefix = "living-room/command/light/";

String topicCommandDoorPrefix = "living-room/command/door/";

String topicGetSensorData = "living-room/command/get-sensor-data";
String topicRequestPasswordDoorPrefix = "living-room/request/password/";   // Yêu cầu lấy mật khẩu
String topicResponsePasswordDoorPrefix = "living-room/response/password/"; // Chỉ nhận mật khẩu từ server

// register device topic (pub)
String topicDeviceRegister = "living-room/device-register";
// status topic (pub)
String topicLightStatusPrefix = "living-room/device-status/light/";
String topicDoorStatusPrefix = "living-room/device-status/door/";

String topicPasswordStatusPrefix = "living-room/device-status/password/"; // Gửi mật khẩu lên server để lưu
String topicPasswordValidationPrefix = "living-room/password-validation/";  // Pub kết quả kiểm tra mật khẩu (SUCCESS/FAILED)

// ========== Hardware Pins ==========
// thêm đèn

const int BUTTON_PIN = 13; // Button
const int DHT_PIN = 19;    // DHT22

const int LCD_SDA = 21; // LCD I2C SDA
const int LCD_SCL = 22; // LCD I2C SCL

// ========== Objects ==========
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHT_PIN, DHT22);

// ========== Keypad 4x4 Configuration ==========
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};
byte rowPins[ROWS] = {4, 5, 16, 17};
byte colPins[COLS] = {32, 33, 23, 0};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ========== Variables ==========
// Sensor data
float temperature = 0.0;
float humidity = 0.0;

// Last published values
float lastPublishedTemp = -999.0;
float lastPublishedHum = -999.0;

// Timing
unsigned long lastDHTRead = 0;
unsigned long lastLCDUpdate = 0;
const unsigned long DHT_INTERVAL = 2000;
const unsigned long LCD_INTERVAL = 1000;
const unsigned long ONLINE_INTERVAL = 1000;

// Button handling
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Keypad handling
String keypadBuffer = "";
unsigned long keypadTimeout = 0;
const unsigned long KEYPAD_TIMEOUT = 5000;
bool enteringPassword = false;

// Change password handling
bool changingPassword = false;
int passwordChangeStep = 0; // 0=old, 1=new, 2=confirm
String newPassword = "";

// Door auto-lock
unsigned long doorAutoLockTime = 0;
const unsigned long DOOR_AUTO_LOCK_DELAY = 3000; // 3 seconds

// LCD message
unsigned long lcdMessageTimeout = 0;

// Password request
// sửa
bool passwordRequested = false;
bool passwordReceived = false;

// ========== Helper Functions ==========

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

  if (enteringPassword)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Enter Password:");
    lcd.setCursor(0, 1);
    for (size_t i = 0; i < keypadBuffer.length(); i++)
    {
      lcd.print('*');
    }
    return;
  }

  if (changingPassword)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    if (passwordChangeStep == 0)
    {
      lcd.print("Old Password:");
    }
    else if (passwordChangeStep == 1)
    {
      lcd.print("New Password:");
    }
    else
    {
      lcd.print("Confirm New:");
    }
    lcd.setCursor(0, 1);
    for (size_t i = 0; i < keypadBuffer.length(); i++)
    {
      lcd.print('*');
    }
    return;
  }

  lcd.clear();
  // Line 1: Temperature & Humidity (fits perfectly on 16 chars)
  // Format: "T:25.5C H:65%"
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature, 1);
  lcd.print("C H:");
  lcd.print((int)humidity);
  lcd.print("%");

  // Line 2: LED status (3 bits) + Door status
  // Format: "LED:001 Locked" or "LED:101 Unlocked"
  lcd.setCursor(0, 1);
  lcd.print("LED:");
  lcd.print(lights[0].status ? "1" : "0");
  lcd.print(lights[1].status ? "1" : "0");
  lcd.print(lights[2].status ? "1" : "0");
  lcd.print(" ");
  lcd.print(doors[0].locked ? "Locked" : "Unlocked");
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

    // Publish only if temperature or humidity changed
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

void handleButton()
{
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState)
  {
    lastDebounceTime = millis();
    lastButtonState = reading;

    if (reading == LOW)
    {
      // Button pressed - Toggle LED
      lights[0].status = !lights[0].status;
      digitalWrite(lights[0].pin, lights[0].status);

      Serial.print("Button: LED ");
      Serial.println(lights[0].status ? "ON" : "OFF");
      if (mqtt.connected())
      {
        mqtt.publish((topicLightStatusPrefix + lights[0].id).c_str(), lights[0].status ? "ON" : "OFF", false); // retained
        lights[0].lastPublishLed = lights[0].status;
        Serial.println("Published led status!");
      }

      // Update LCD immediately
      lcdMessageTimeout = 0;
      updateLCD();
    }
  }
}

void handleKeypad()
{
  char key = keypad.getKey();
  if (!key)
    return;

  Serial.print("Key pressed: ");
  Serial.println(key);

  if (key == 'A')
  {
    // Start entering password to unlock/lock door
    enteringPassword = true;
    keypadBuffer = "";
    keypadTimeout = millis() + KEYPAD_TIMEOUT;
    showLCDMessage("Enter Password:", "", 500);
    Serial.println("Enter password mode");
    return;
  }

  if (key == 'B')
  {
    // Lock door immediately
    doors[0].locked = true;
    doors[0].servo.write(0);
    
    showLCDMessage("Door LOCKED", "", 1000);
    
    if (mqtt.connected())
    {
      mqtt.publish((topicDoorStatusPrefix + doors[0].id).c_str(), "LOCKED", false);
      doors[0].lastPublishDoor = doors[0].locked;
    }
    
    Serial.println("Button B: Door LOCKED");
    return;
  }

  if (key == 'D')
  {
    // Start changing password
    changingPassword = true;
    passwordChangeStep = 0;
    keypadBuffer = "";
    newPassword = "";
    keypadTimeout = millis() + KEYPAD_TIMEOUT;
    showLCDMessage("Change Password", "Enter old pass", 800);
    Serial.println("Change password mode");
    return;
  }

  if (changingPassword)
  {
    if (key == '#')
    {
      // Process password change step
      if (passwordChangeStep == 0)
      {
        // Verify old password
        if (keypadBuffer == doors[0].password)
        {
          passwordChangeStep = 1;
          keypadBuffer = "";
          showLCDMessage("Correct!", "Enter new pass", 800);
          Serial.println("Old password correct, enter new");
        }
        else
        {
          showLCDMessage("Wrong Password!", "", 1000);
          Serial.println("Wrong old password!");
          changingPassword = false;
          keypadBuffer = "";
        }
      }
      else if (passwordChangeStep == 1)
      {
        // Save new password
        if (keypadBuffer.length() >= 4 && keypadBuffer.length() <= 8)
        {
          newPassword = keypadBuffer;
          passwordChangeStep = 2;
          keypadBuffer = "";
          showLCDMessage("Confirm new:", "", 800);
          Serial.println("Enter new password again to confirm");
        }
        else
        {
          showLCDMessage("4-8 digits only!", "", 1000);
          keypadBuffer = "";
        }
      }
      else
      {
        // Confirm new password
        if (keypadBuffer == newPassword)
        {
          doors[0].password = newPassword;

          // Lưu mật khẩu mới lên server
          if (mqtt.connected())
          {
            mqtt.publish((topicPasswordStatusPrefix + doors[0].id).c_str(), doors[0].password.c_str());
            Serial.println("Password saved to server");
          }

          showLCDMessage("Password", "Changed!", 1500);
          Serial.print("Password changed to: ");
          Serial.println(doors[0].password);
        }
        else
        {
          showLCDMessage("Not Match!", "Try again", 1000);
          Serial.println("Passwords don't match");
        }
        changingPassword = false;
        keypadBuffer = "";
        newPassword = "";
      }
      keypadTimeout = millis() + KEYPAD_TIMEOUT;
    }
    else if (key == '*')
    {
      // Cancel or clear
      if (keypadBuffer.length() > 0)
      {
        keypadBuffer = "";
        showLCDMessage("Cleared", "", 500);
      }
      else
      {
        showLCDMessage("Cancelled", "", 1000);
        changingPassword = false;
        keypadBuffer = "";
        newPassword = "";
        Serial.println("Password change cancelled");
      }
    }
    else if (key >= '0' && key <= '9')
    {
      // Add digit
      if (keypadBuffer.length() < 8)
      {
        keypadBuffer += key;
        updateLCD();
        keypadTimeout = millis() + KEYPAD_TIMEOUT;
      }
    }
  }
  else if (enteringPassword)
  {
    if (key == '#')
    {
      // Check password
      if (keypadBuffer == doors[0].password)
      {
        // Unlock door
        doors[0].locked = false;
        doors[0].servo.write(90);
        doorAutoLockTime = millis() + DOOR_AUTO_LOCK_DELAY;

        showLCDMessage("Door UNLOCKED", "", 500);

        if (mqtt.connected())
        {
          mqtt.publish((topicDoorStatusPrefix + doors[0].id).c_str(), "UNLOCKED", false); // retained
          doors[0].lastPublishDoor = doors[0].locked;
          
          // Publish password validation success
          mqtt.publish((topicPasswordValidationPrefix + doors[0].id).c_str(), "SUCCESS", false);
        }

        Serial.println("Door: UNLOCKED");
      }
      else
      {
        showLCDMessage("Wrong Password!", "");
        Serial.println("Wrong password!");
        
        // Publish password validation failed
        if (mqtt.connected())
        {
          mqtt.publish((topicPasswordValidationPrefix + doors[0].id).c_str(), "FAILED", false);
        }
      }

      enteringPassword = false;
      keypadBuffer = "";
    }
    else if (key == '*')
    {
      // Clear entry
      keypadBuffer = "";
      showLCDMessage("Cleared", "", 500);
    }
    else if (key >= '0' && key <= '9')
    {
      // Add digit
      if (keypadBuffer.length() < 8)
      {
        keypadBuffer += key;
        updateLCD();
        keypadTimeout = millis() + KEYPAD_TIMEOUT;
      }
    }
  }
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
DoorDevice *getDoorById(const String &id)
{
  for (int i = 0; i < sizeof(doors) / sizeof(doors[0]); i++)
  {
    if (doors[i].id == id)
      return &doors[i];
  }
  return nullptr;
}

// Handle light command
void handleLightCommand(const String &topic, const String &message)
{
  int lastSlash = topic.lastIndexOf('/');
  String id = topic.substring(lastSlash + 1);
  LightDevice *light = getLightById(id);
  if (!light)
    return;

  if (message == "ON" || message == "1")
    light->status = true;
  else if (message == "OFF" || message == "0")
    light->status = false;
  else if (message == "TOGGLE")
    light->status = !light->status;

  digitalWrite(light->pin, light->status);
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
  }

  // Publish trạng thái lên MQTT
  if (mqtt.connected())
  {
    String statusTopic = String(topicDoorStatusPrefix + door->id);
    mqtt.publish(statusTopic.c_str(), door->locked ? "LOCKED" : "UNLOCKED", false);
  }

  // Hiển thị trên LCD
  showLCDMessage(("Door: " + door->id).c_str(), door->locked ? "LOCKED" : "UNLOCKED");
  Serial.println("Door " + door->id + ": " + (door->locked ? "LOCKED" : "UNLOCKED"));
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
    // Publish current sensor data immediately
    Serial.println("MQTT command: Get sensor data");
    readDHT();
    return;
  }

  // Control lights
  else if (topicStr.startsWith(topicCommandLightPrefix))
    handleLightCommand(topicStr, message);
  // Control door
  else if (topicStr.startsWith(topicCommandDoorPrefix))
    handleDoorCommand(topicStr, message);

  // Receive and update password from server
  else if (topicStr.startsWith(topicResponsePasswordDoorPrefix))
  {
    // Lấy ID cửa từ topic, ví dụ: "living-room/response/password/LV_Door_01"
    int lastSlash = topicStr.lastIndexOf('/');
    String doorId = topicStr.substring(lastSlash + 1);

    DoorDevice *door = getDoorById(doorId);
    if (!door)
    {
      Serial.println("Unknown door ID in password response: " + doorId);
      return;
    }

    // Kiểm tra độ dài password hợp lệ
    if (message.length() >= 4 && message.length() <= 8)
    {
      door->password = message; // Lưu password riêng cho cửa này
      passwordReceived = true;
      showLCDMessage(("Password " + door->id).c_str(), "Loaded!");
      Serial.print("Password loaded for door ");
      Serial.print(door->id);
      Serial.print(": ");
      Serial.println(door->password);
    }
    else
    {
      Serial.println("Invalid password from server for door " + door->id);
    }
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
    String clientId = "ESP32_Livingroom_" + String(random(0xffff), HEX);

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

      // Subscribe to control topics
      mqtt.subscribe(topicGetSensorData.c_str());

      // Subscribe door commands & password responses per door
      for (int i = 0; i < sizeof(doors) / sizeof(doors[0]); i++)
      {
        String doorCmdTopic = topicCommandDoorPrefix + doors[i].id;          // ví dụ: "living-room/command/door/LV_Door_01"
        String doorPwdTopic = topicResponsePasswordDoorPrefix + doors[i].id; // ví dụ: "living-room/response/password/LV_Door_01"

        mqtt.subscribe(doorCmdTopic.c_str());
        mqtt.subscribe(doorPwdTopic.c_str());
      }

      // Subscribe light command topics
      for (int i = 0; i < sizeof(lights) / sizeof(lights[0]); i++)
      {
        String cmdTopic = topicCommandLightPrefix + lights[i].id;
        mqtt.subscribe(cmdTopic.c_str());
      }

      // Publish online status
      mqtt.publish(topicStatus.c_str(), "online", false);

      // đăng kí device sensor
      String payload = "{";
      payload += "\"id\":\"" + tempHumidSensorId + "\",";
      payload += "\"name\":\"temperature and humidity sensor\",";
      payload += "\"type\":\"temp_humid_sensor\"";
      payload += "}";

      mqtt.publish(
          topicDeviceRegister.c_str(),
          payload.c_str(),
          false // register KHÔNG retained
      );

      // Publish initial sensor data
      String sensorPayload = "{";
      sensorPayload += "\"temperature\":" + String(temperature, 1) + ",";
      sensorPayload += "\"humidity\":" + String((int)humidity);
      sensorPayload += "}";
      mqtt.publish(topicSensor.c_str(), sensorPayload.c_str(), false);

      // Publish initial device status
      for (int i = 0; i < sizeof(lights) / sizeof(lights[0]); i++)
      {
        String statusTopic = String(topicLightStatusPrefix) + lights[i].id;
        Serial.print(statusTopic);
        mqtt.publish(statusTopic.c_str(), lights[i].status ? "ON" : "OFF", false);
        lights[i].lastPublishLed = lights[i].status;
      } // retained

      // Publish door status (giống light)
      for (int i = 0; i < sizeof(doors) / sizeof(doors[0]); i++)
      {
        String doorStatusTopic = String(topicDoorStatusPrefix) + doors[i].id;
        mqtt.publish(
            doorStatusTopic.c_str(),
            doors[i].locked ? "LOCKED" : "UNLOCKED",
            false);
        doors[i].lastPublishDoor = doors[i].locked;
      } // retained

      lastPublishedTemp = temperature;
      lastPublishedHum = humidity;

      Serial.println("Initial status published");

      // Yêu cầu lấy mật khẩu từ server
      if (!passwordRequested)
      {
        for (int i = 0; i < sizeof(doors) / sizeof(doors[0]); i++)
        {
          String reqTopic = String(topicRequestPasswordDoorPrefix) + doors[i].id;

          Serial.print("Requesting password for ");
          Serial.println(doors[i].id);

          mqtt.publish(reqTopic.c_str(), "GET_PASSWORD", false);
        }
        passwordRequested = true;
      }
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
  Serial.println("\n\n=== LIVINGROOM SYSTEM STARTING ===");

  // Initialize pins
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  for (int i = 0; i < sizeof(lights) / sizeof(lights[0]); i++)
  {
    pinMode(lights[i].pin, OUTPUT);
    digitalWrite(lights[i].pin, LOW);
  }
  // Initialize servo

  for (int i = 0; i < sizeof(doors) / sizeof(doors[0]); i++)
  {
    doors[i].servo.attach(doors[i].pin);
    doors[i].servo.write(0); // Lock door initially
    doors[i].password = "1234";
    Serial.print("Default password: ");
    Serial.println(doors[i].password);
  }

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LIVINGROOM");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");

  // Initialize DHT
  dht.begin();
  Serial.println("DHT22 initialized");

  // Set default password

  // Connect WiFi
  connectWiFi();

  // Setup MQTT
  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(mqttCallback);

  // Initial sensor reading
  delay(2000);
  readDHT();
  updateLCD();
}

// ========== MAIN LOOP ==========
void loop()
{
  unsigned long currentMillis = millis();

  // Maintain WiFi & MQTT connection
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

  // Read DHT sensor
  if (currentMillis - lastDHTRead >= DHT_INTERVAL)
  {
    lastDHTRead = currentMillis;
    readDHT();
  }

  // Update LCD
  if (currentMillis - lastLCDUpdate >= LCD_INTERVAL)
  {
    lastLCDUpdate = currentMillis;
    updateLCD();
  }

  // Handle button
  handleButton();

  // Handle keypad
  handleKeypad();

  // Keypad timeout
  if (enteringPassword && currentMillis > keypadTimeout)
  {
    enteringPassword = false;
    keypadBuffer = "";
    Serial.println("Keypad entry timeout");
  }

  if (changingPassword && currentMillis > keypadTimeout)
  {
    changingPassword = false;
    keypadBuffer = "";
    newPassword = "";
    Serial.println("Password change timeout");
  }

  // // Auto-lock door after 3 seconds
  // if (!doorLocked && doorAutoLockTime > 0 && currentMillis >= doorAutoLockTime)
  // {
  //   doorLocked = true;
  //   doorServo.write(0);
  //   doorAutoLockTime = 0;

  //   showLCDMessage("Door:", "AUTO-LOCKED");

  //   if (mqtt.connected())
  //   {
  //     mqtt.publish(topicDoorStatus.c_str(), "LOCKED", false); // retained
  //     lastPublishedDoor = doorLocked;
  //   }

  //   Serial.println("Door: AUTO-LOCKED");
  // }

  delay(10);
}
