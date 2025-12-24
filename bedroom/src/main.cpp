#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <TM1637Display.h>
#include <time.h>

// WiFi & MQTT Configuration
const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "test.mosquitto.org";  // Public MQTT broker
const int mqtt_port = 1883;

// NTP Configuration
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600;  // GMT+7 (Vietnam)
const int daylightOffset_sec = 0;     // No daylight saving

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ========== MQTT Topics ==========
// sensor data topic (pub)
String topicSensor = "bedroom/sensor-device";
String topicStatus = "bedroom/status";
// command topics (sub)
String topicCommandLight = "bedroom/command/light";
String topicGetSensorData = "bedroom/command/get-sensor-data";
// register device topic (pub)
String topicDeviceRegister = "bedroom/device-register";
// status topic (pub)
String topicLightStatus = "bedroom/device-status/light";

String tempHumidSensorId = "BR_DHT22_01";

// Hardware pins (theo diagram.json bedroom)
const int DHT_PIN = 19;          // DHT22 data pin
const int LED_PIN = 26;          // LED
const int PIR_PIN = 25;          // PIR motion sensor
const int BUTTON_PIN = 13;       // Button
const int TM1637_CLK = 14;       // 7-segment CLK
const int TM1637_DIO = 27;       // 7-segment DIO
const int LCD_SDA = 21;          // LCD I2C SDA
const int LCD_SCL = 22;          // LCD I2C SCL

// Objects
LiquidCrystal_I2C lcd(0x27, 16, 2);  // LCD 16x2
DHT dht(DHT_PIN, DHT22);
TM1637Display display(TM1637_CLK, TM1637_DIO);

// Variables
float temperature = 0.0;
float humidity = 0.0;
bool ledStatus = false;
bool pirMotionDetected = false;

// PIR auto-off timer
unsigned long ledAutoOffTime = 0;
const unsigned long LED_AUTO_OFF_DELAY = 3000;  

// Previous values for change detection
float lastPublishedTemp = -999.0;
float lastPublishedHum = -999.0;
bool lastPublishedLedStatus = false;

// Button handling
bool lastButtonState = HIGH;
bool buttonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 5;  

// Time tracking
struct tm timeinfo;
bool timeSynced = false;

// Timing
unsigned long lastDHTRead = 0;
unsigned long lastLCDUpdate = 0;
unsigned long lastMQTTPublish = 0;
unsigned long lastTimeUpdate = 0;

const unsigned long DHT_INTERVAL = 2000;      // Đọc DHT mỗi 2 giây
const unsigned long LCD_INTERVAL = 1000;      // Cập nhật LCD mỗi 1 giây
const unsigned long MQTT_INTERVAL = 5000;     // Gửi MQTT mỗi 5 giây
const unsigned long TIME_INTERVAL = 1000;     // Cập nhật thời gian mỗi 1 giây
const unsigned long ONLINE_INTERVAL = 1000;   // Cập nhật trạng thái online mỗi 1 giây

// ========== HELPER FUNCTIONS ==========

void readDHT() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  
  if (!isnan(temp) && !isnan(hum)) {
    bool tempChanged = (abs(temp - lastPublishedTemp) >= 0.1);
    bool humChanged = (abs(hum - lastPublishedHum) >= 1.0);
    
    temperature = temp;
    humidity = hum;

    // Publish unified sensor data if any value changed
    if ((tempChanged || humChanged) && mqtt.connected()) {
      Serial.print("DHT22 - Temp: ");
      Serial.print(temperature);
      Serial.print("°C, Humidity: ");
      Serial.print(humidity);
      Serial.println("%");
      
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
  } else {
    Serial.println("Failed to read DHT sensor!");
  }
}

void handlePIR() {
  pirMotionDetected = digitalRead(PIR_PIN);
  
  if (pirMotionDetected) {
    // Motion detected - Turn on LED
    if (!ledStatus) {
      ledStatus = true;
      digitalWrite(LED_PIN, HIGH);
      Serial.println("PIR: Motion detected - LED ON");
      
      if (mqtt.connected()) {
        mqtt.publish(topicLightStatus.c_str(), "ON", true);
        lastPublishedLedStatus = ledStatus;
      }
    }
    // Reset auto-off timer
    ledAutoOffTime = millis() + LED_AUTO_OFF_DELAY;
  }
}

void updateLCD() {
  lcd.clear();
  
  // Line 1: Temperature & Humidity
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature, 1);
  lcd.print("C H:");
  lcd.print((int)humidity);
  lcd.print("%");
  
  // Line 2: LED status & PIR motion
  lcd.setCursor(0, 1);
  lcd.print("LED:");
  lcd.print(ledStatus ? "ON " : "OFF");
  lcd.print(" PIR:");
  lcd.print(pirMotionDetected ? "Y" : "N");
}

void display7SegmentTime() {
  if (!getLocalTime(&timeinfo)) {
    // If time not synced, display "----"
    uint8_t data[] = {0x40, 0x40, 0x40, 0x40};  // Dash segments
    display.setSegments(data);
    return;
  }
  
  // Display format: HH:MM (with blinking colon)
  int hour = timeinfo.tm_hour;
  int minute = timeinfo.tm_min;
  int second = timeinfo.tm_sec;
  
  // Show colon by using bit 7 of second digit
  bool showColon = (second % 2 == 0);  // Blink every second
  
  uint8_t data[4];
  data[0] = display.encodeDigit(hour / 10);
  data[1] = display.encodeDigit(hour % 10) | (showColon ? 0x80 : 0x00);
  data[2] = display.encodeDigit(minute / 10);
  data[3] = display.encodeDigit(minute % 10);
  
  display.setSegments(data);
}

void handleButton() {
  int reading = digitalRead(BUTTON_PIN);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      
      if (buttonState == LOW) {  // Button pressed (active LOW)
        // Toggle LED
        ledStatus = !ledStatus;
        digitalWrite(LED_PIN, ledStatus);
        ledAutoOffTime = 0;  // Disable auto-off when manually toggled
        Serial.print("Button: LED ");
        Serial.println(ledStatus ? "ON" : "OFF");
        
        // Publish LED status change
        if (mqtt.connected()) {
          mqtt.publish(topicLightStatus.c_str(), ledStatus ? "ON" : "OFF", true);
          lastPublishedLedStatus = ledStatus;
          Serial.println("Published LED status");
        }
        
        updateLCD();  // Update LCD immediately
      }
    }
  }
  
  lastButtonState = reading;
}

void publishAllStatus() {
  // Publish all current values (called once on startup or reconnect)
  if (mqtt.connected()) {
    // Publish unified sensor data
    String sensorPayload = "{";
    sensorPayload += "\"temperature\":" + String(temperature, 1) + ",";
    sensorPayload += "\"humidity\":" + String((int)humidity);
    sensorPayload += "}";
    mqtt.publish(topicSensor.c_str(), sensorPayload.c_str(), false);
    
    // Publish device status
    mqtt.publish(topicLightStatus.c_str(), ledStatus ? "ON" : "OFF", true);
    
    lastPublishedTemp = temperature;
    lastPublishedHum = humidity;
    lastPublishedLedStatus = ledStatus;
    
    Serial.println("Initial status published to MQTT");
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("MQTT [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);
  
  String topicStr = String(topic);
  
  // Get sensor data command
  if (topicStr == topicGetSensorData) {
    Serial.println("MQTT command: Get sensor data");
    readDHT();
    return;
  }
  
  // Control commands from MQTT
  if (topicStr == topicCommandLight) {
    bool oldStatus = ledStatus;
    
    if (message == "ON" || message == "1") {
      ledStatus = true;
      digitalWrite(LED_PIN, HIGH);
      ledAutoOffTime = 0;  // Disable auto-off when manually controlled
    } else if (message == "OFF" || message == "0") {
      ledStatus = false;
      digitalWrite(LED_PIN, LOW);
      ledAutoOffTime = 0;  // Disable auto-off
    } else if (message == "TOGGLE") {
      ledStatus = !ledStatus;
      digitalWrite(LED_PIN, ledStatus);
      ledAutoOffTime = 0;  // Disable auto-off
    }
    
    // Publish status back if changed
    if (ledStatus != oldStatus) {
      mqtt.publish(topicLightStatus.c_str(), ledStatus ? "ON" : "OFF", true);
      lastPublishedLedStatus = ledStatus;
      Serial.print("LED changed to: ");
      Serial.println(ledStatus ? "ON" : "OFF");
    }
    
    updateLCD();
  }
}

void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi Connection Failed!");
  }
}

void connectMQTT() {
  if (!mqtt.connected()) {
    Serial.print("Connecting to MQTT...");
    String clientId = "ESP32_Bedroom_" + String(random(0xffff), HEX);
    
    if (mqtt.connect(clientId.c_str())) {
      Serial.println("Connected!");
      
      // Subscribe to control topics
      mqtt.subscribe(topicCommandLight.c_str());
      mqtt.subscribe(topicGetSensorData.c_str());
      
      // Publish online status
      mqtt.publish(topicStatus.c_str(), "online", true);
      
      // Register temp/humidity sensor device
      String payload1 = "{";
      payload1 += "\"id\":\"" + tempHumidSensorId + "\",";
      payload1 += "\"name\":\"temperature and humidity sensor\",";
      payload1 += "\"type\":\"temp_humid_sensor\"";
      payload1 += "}";
      mqtt.publish(topicDeviceRegister.c_str(), payload1.c_str(), false);
      
      // Publish all current status on connect
      publishAllStatus();
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" retrying in 5s");
    }
  }
}


// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== BEDROOM SYSTEM STARTING ===");
  
  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);         // PIR motion sensor
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Button with internal pullup
  digitalWrite(LED_PIN, LOW);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("BEDROOM SYSTEM");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  
  // Initialize 7-segment display
  display.setBrightness(0x0f);  // Max brightness
  display.clear();
  
  // Initialize DHT sensor
  dht.begin();
  Serial.println("DHT22 initialized");
  
  // Connect to WiFi
  connectWiFi();
  
  // Configure NTP time (GMT+7)
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Syncing time with NTP server...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    
    // Wait for time to be synced
    int attempts = 0;
    while (!getLocalTime(&timeinfo) && attempts < 10) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    
    if (getLocalTime(&timeinfo)) {
      timeSynced = true;
      Serial.println("\nTime synced!");
      Serial.print("Current time: ");
      Serial.println(&timeinfo, "%H:%M:%S %d-%m-%Y");
    } else {
      Serial.println("\nFailed to sync time");
    }
  }
  
  // Setup MQTT
  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(mqttCallback);
  
  // Initial sensor readings
  delay(2000);  // DHT needs time to stabilize
  readDHT();
  
  // Update displays
  updateLCD();
  display7SegmentTime();
  
  Serial.println("=== BEDROOM SYSTEM READY ===\n");
  Serial.println("Controls:");
  Serial.println("- Button: Toggle LED ON/OFF");
  Serial.println("- 7-Segment: Displays current time (HH:MM)");
  Serial.println("- LCD: Shows temp, humidity, LED status, light level");
  Serial.println("\nMQTT Topics:");
  Serial.println("  Subscribe:");
  Serial.println("    * bedroom/command/light (ON/OFF/TOGGLE)");
  Serial.println("  Publish:");
  Serial.println("    * bedroom/sensor/temperature");
  Serial.println("    * bedroom/sensor/humidity");
  Serial.println("    * bedroom/sensor/light");
  Serial.println("    * bedroom/status/led (ON/OFF)");
  Serial.println();
}

// ========== MAIN LOOP ==========
void loop() {
  unsigned long currentMillis = millis();
  
  // Maintain MQTT connection
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqtt.connected()) {
      static unsigned long lastReconnectAttempt = 0;
      if (currentMillis - lastReconnectAttempt > 5000) {
        lastReconnectAttempt = currentMillis;
        connectMQTT();
      }
    } else {
      mqtt.loop();
    }
  }
  
  // Read DHT sensor periodically
  if (currentMillis - lastDHTRead >= DHT_INTERVAL) {
    lastDHTRead = currentMillis;
    readDHT();
  }
  
  // Handle PIR motion sensor
  handlePIR();
  
  // Auto-off LED after PIR timer expires
  if (ledAutoOffTime > 0 && currentMillis >= ledAutoOffTime) {
    ledStatus = false;
    digitalWrite(LED_PIN, LOW);
    ledAutoOffTime = 0;
    Serial.println("PIR: LED auto-off");
    
    if (mqtt.connected()) {
      mqtt.publish(topicLightStatus.c_str(), "OFF", true);
      lastPublishedLedStatus = ledStatus;
    }
  }
  
  // Update LCD display
  if (currentMillis - lastLCDUpdate >= LCD_INTERVAL) {
    lastLCDUpdate = currentMillis;
    updateLCD();
  }
  
  // Update 7-segment time display
  if (currentMillis - lastTimeUpdate >= TIME_INTERVAL) {
    lastTimeUpdate = currentMillis;
    display7SegmentTime();
  }
  
  // Handle button input
  handleButton();

  unsigned long lastOnline = currentMillis;
  if(currentMillis - lastOnline >= ONLINE_INTERVAL) {
    mqtt.publish(topicStatus.c_str(), "online", true);
  }
  
  delay(10);  // Small delay for stability
}
