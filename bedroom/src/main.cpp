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

// 📥 Subscribe (Nhận lệnh):
// bedroom/command/light - Điều khiển LED (ON/OFF/TOGGLE)
// 📤 Publish (Gửi dữ liệu):
// bedroom/sensor/temperature - Nhiệt độ (°C, thay đổi ≥0.1°C)
// bedroom/sensor/humidity - Độ ẩm (%, thay đổi ≥1%)
// bedroom/sensor/light - Ánh sáng (%, thay đổi ≥1%)
// bedroom/status/led - Trạng thái LED (ON/OFF)

WiFiClient espClient;
PubSubClient mqtt(espClient);

// Hardware pins (theo diagram.json bedroom)
const int DHT_PIN = 19;          // DHT22 data pin
const int LED_PIN = 26;          // LED
const int LDR_PIN = 34;          // Photoresistor (Analog) - ADC1 to avoid WiFi conflict
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
int lightLevel = 0;              // 0-4095 (ADC 12-bit)
int lightPercent = 0;            // 0-100%
bool ledStatus = false;

// Previous values for change detection
float lastPublishedTemp = -999.0;
float lastPublishedHum = -999.0;
int lastPublishedLight = -1;
bool lastPublishedLedStatus = false;

// Button handling
bool lastButtonState = HIGH;
bool buttonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 20;  // Giảm từ 50ms xuống 20ms để nhạy hơn

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

// ========== HELPER FUNCTIONS ==========

void readDHT() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  
  if (!isnan(temp) && !isnan(hum)) {
    temperature = temp;
    humidity = hum;


    
    // Publish if temperature changed by 0.1°C or more
    if (mqtt.connected() && abs(temperature - lastPublishedTemp) >= 0.1) {
      Serial.print("DHT22 - Temp: ");
      Serial.print(temperature);
      Serial.print("°C, Humidity: ");
      Serial.print(humidity);
      Serial.println("%");
      mqtt.publish("bedroom/sensor/temperature", String(temperature, 1).c_str());
      lastPublishedTemp = temperature;
      Serial.println("  -> Published temperature");
    }
    
    // Publish if humidity changed by 1% or more
    if (mqtt.connected() && abs(humidity - lastPublishedHum) >= 1) {
      Serial.print("DHT22 - Temp: ");
      Serial.print(temperature);
      Serial.print("°C, Humidity: ");
      Serial.print(humidity);
      Serial.println("%");
      mqtt.publish("bedroom/sensor/humidity", String((int)humidity).c_str());
      lastPublishedHum = humidity;
      Serial.println("  -> Published humidity");
    }
  } else {
    Serial.println("Failed to read DHT sensor!");
  }
}

void readLightSensor() {
  lightLevel = analogRead(LDR_PIN);
  lightPercent = map(lightLevel, 0, 4095, 0, 100);
  
  // Publish if light changed by 1% or more
  if (mqtt.connected() && abs(lightPercent - lastPublishedLight) >= 1) {
    mqtt.publish("bedroom/sensor/light", String(lightPercent).c_str());
    lastPublishedLight = lightPercent;
    Serial.print("Light: ");
    Serial.print(lightPercent);
    Serial.println("% -> Published");
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
  
  // Line 2: LED status & Light level
  lcd.setCursor(0, 1);
  lcd.print("LED:");
  lcd.print(ledStatus ? "ON " : "OFF");
  lcd.print(" L:");
  lcd.print(lightPercent);
  lcd.print("%");
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
        Serial.print("Button: LED ");
        Serial.println(ledStatus ? "ON" : "OFF");
        
        // Publish LED status change
        if (mqtt.connected()) {
          mqtt.publish("bedroom/status/led", ledStatus ? "ON" : "OFF");
          lastPublishedLedStatus = ledStatus;
          Serial.println("  -> Published LED status");
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
    mqtt.publish("bedroom/sensor/temperature", String(temperature, 1).c_str());
    mqtt.publish("bedroom/sensor/humidity", String((int)humidity).c_str());
    mqtt.publish("bedroom/status/led", ledStatus ? "ON" : "OFF");
    mqtt.publish("bedroom/sensor/light", String(lightPercent).c_str());
    
    lastPublishedTemp = temperature;
    lastPublishedHum = humidity;
    lastPublishedLight = lightPercent;
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
  
  // Control commands from MQTT
  if (String(topic) == "bedroom/command/light") {
    bool oldStatus = ledStatus;
    
    if (message == "ON" || message == "1") {
      ledStatus = true;
      digitalWrite(LED_PIN, HIGH);
    } else if (message == "OFF" || message == "0") {
      ledStatus = false;
      digitalWrite(LED_PIN, LOW);
    } else if (message == "TOGGLE") {
      ledStatus = !ledStatus;
      digitalWrite(LED_PIN, ledStatus);
    }
    
    // Publish status back if changed
    if (ledStatus != oldStatus) {
      mqtt.publish("bedroom/status/led", ledStatus ? "ON" : "OFF");
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
      mqtt.subscribe("bedroom/command/light");
      
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
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Button with internal pullup
  pinMode(LDR_PIN, INPUT);
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
  readLightSensor();
  
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
  
  // Read light sensor continuously (faster response)
  readLightSensor();
  
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
  
  delay(10);  // Small delay for stability
}
