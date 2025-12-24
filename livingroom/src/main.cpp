#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <Keypad.h>
#include <Preferences.h>

// ========== WiFi & MQTT Configuration ==========
const char *ssid = "Wokwi-GUEST";
const char *password = "";
const char *mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ========== MQTT Topics ==========
// sensor data topic (pub)
String topicSensor = "living-room/sensor-device";
String topicStatus = "living-room/status";
// command topics (sub)
String topicCommandLight = "living-room/command/light";
String topicCommandDoor = "living-room/command/door";
String topicCommandPassword = "living-room/command/password";
String topicGetSensorData = "living-room/command/get-sensor-data";
// register device topic (pub)
String topicDeviceRegister = "living-room/device-register";
// status topic (pub)
String topicLightStatus = "living-room/device-status/light";
String topicDoorStatus = "living-room/device-status/door";
String topicPasswordStatus = "living-room/device-status/password";


String tempHumidSensorId = "LR_DHT22_01";

// ========== Hardware Pins ==========
const int LED_PIN = 26;    // LED
const int BUTTON_PIN = 13; // Button
const int DHT_PIN = 19;    // DHT22
const int SERVO_PIN = 25;  // Servo motor
const int LCD_SDA = 21;    // LCD I2C SDA
const int LCD_SCL = 22;    // LCD I2C SCL

// ========== Objects ==========
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHT_PIN, DHT22);
Servo doorServo;
Preferences prefs;

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

// Device status
bool ledStatus = false;
bool doorLocked = true;
String doorPassword = "1234";

// Last published values
float lastPublishedTemp = -999.0;
float lastPublishedHum = -999.0;
bool lastPublishedLed = false;
bool lastPublishedDoor = false;

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
  // Line 1: Temperature & Humidity
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature, 1);
  lcd.print("C H:");
  lcd.print((int)humidity);
  lcd.print("%");

  // Line 2: LED status & Door status
  lcd.setCursor(0, 1);
  lcd.print("LED:");
  lcd.print(ledStatus ? "ON " : "OFF");
  lcd.print(" Door:");
  lcd.print(doorLocked ? "L" : "U");
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
      ledStatus = !ledStatus;
      digitalWrite(LED_PIN, ledStatus);

      Serial.print("Button: LED ");
      Serial.println(ledStatus ? "ON" : "OFF");

      if (mqtt.connected())
      {
        mqtt.publish(topicLightStatus.c_str(), ledStatus ? "ON" : "OFF", true);
        lastPublishedLed = ledStatus;
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
        if (keypadBuffer == doorPassword)
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
          doorPassword = newPassword;

          // Publish new password to MQTT server
          if (mqtt.connected())
          {
            mqtt.publish(topicPasswordStatus.c_str(), doorPassword.c_str());
          }

          showLCDMessage("Password", "Changed!", 1500);
          Serial.print("Password changed to: ");
          Serial.println(doorPassword);
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
      if (keypadBuffer == doorPassword)
      {
        // Unlock door
        doorLocked = false;
        doorServo.write(90);
        doorAutoLockTime = millis() + DOOR_AUTO_LOCK_DELAY;

        showLCDMessage("Door UNLOCKED", "", 500);

        if (mqtt.connected())
        {
          mqtt.publish(topicDoorStatus.c_str(), "UNLOCKED", true);
          lastPublishedDoor = doorLocked;
        }

        Serial.println("Door: UNLOCKED");
      }
      else
      {
        showLCDMessage("Wrong Password!", "");
        Serial.println("Wrong password!");
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

  if(topicStr == topicGetSensorData)
  {
    // Publish current sensor data immediately
    Serial.println("MQTT command: Get sensor data");
    readDHT();
    return;
  }

  // Control LED
  if (topicStr == topicCommandLight)
  {
    if (message == "ON" || message == "1")
    {
      ledStatus = true;
    }
    else if (message == "OFF" || message == "0")
    {
      ledStatus = false;
    }
    else if (message == "TOGGLE")
    {
      ledStatus = !ledStatus;
    }
    digitalWrite(LED_PIN, ledStatus);

    if (mqtt.connected())
    {
      mqtt.publish(topicLightStatus.c_str(), ledStatus ? "ON" : "OFF", true);
    }
    showLCDMessage("LED:", ledStatus ? "ON" : "OFF");
  }

  // Control door
  else if (topicStr == topicCommandDoor)
  {
    if (message == "LOCK")
    {
      doorLocked = true;
      doorServo.write(0);
    }
    else if (message == "UNLOCK")
    {
      doorLocked = false;
      doorServo.write(90);
    }

    if (mqtt.connected())
    {
      mqtt.publish(topicDoorStatus.c_str(), doorLocked ? "LOCKED" : "UNLOCKED", true);
    }
    showLCDMessage("Door:", doorLocked ? "LOCKED" : "UNLOCKED");
  }

  // Change password
  else if (topicStr == topicCommandPassword)
  {
    if (message.length() >= 4 && message.length() <= 8)
    {
      doorPassword = message;
      showLCDMessage("Password", "Changed!");
      Serial.println("Password updated via MQTT");
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

    if (mqtt.connect(clientId.c_str()))
    {
      Serial.println("Connected!");

      // Subscribe to control topics
      mqtt.subscribe(topicCommandLight.c_str());
      mqtt.subscribe(topicCommandDoor.c_str());
      mqtt.subscribe(topicCommandPassword.c_str());
      mqtt.subscribe(topicGetSensorData.c_str());


      // Publish online status
      mqtt.publish(topicStatus.c_str(), "online", true);

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
      mqtt.publish(topicLightStatus.c_str(), ledStatus ? "ON" : "OFF", true);
      mqtt.publish(topicDoorStatus.c_str(), doorLocked ? "LOCKED" : "UNLOCKED", true);

      lastPublishedTemp = temperature;
      lastPublishedHum = humidity;
      lastPublishedLed = ledStatus;
      lastPublishedDoor = doorLocked;

      Serial.println("Initial status published");
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
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(LED_PIN, LOW);

  // Initialize servo
  doorServo.attach(SERVO_PIN);
  doorServo.write(0);

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
  doorPassword = "1234";
  Serial.print("Default password: ");
  Serial.println(doorPassword);

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

  // Auto-lock door after 3 seconds
  if (!doorLocked && doorAutoLockTime > 0 && currentMillis >= doorAutoLockTime)
  {
    doorLocked = true;
    doorServo.write(0);
    doorAutoLockTime = 0;

    showLCDMessage("Door:", "AUTO-LOCKED");

    if (mqtt.connected())
    {
      mqtt.publish(topicDoorStatus.c_str(), "LOCKED", true);
      lastPublishedDoor = doorLocked;
    }

    Serial.println("Door: AUTO-LOCKED");
  }

  unsigned long lastOnline = currentMillis;
  if(currentMillis - lastOnline >= ONLINE_INTERVAL) {
    mqtt.publish(topicStatus.c_str(), "online", true);
  }

  delay(10);
}
