#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <ESP32Servo.h>

// WiFi & MQTT Configuration
const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "test.mosquitto.org";  // Public MQTT broker
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient mqtt(espClient);

// Hardware pins (theo diagram.json kitchen)
const int LED_WHITE = 25;        // LED trắng (led2)
const int LED_RED = 26;          // LED đỏ (led1)
const int BUTTON_PIN = 12;       // Button
const int DHT_PIN = 27;          // DHT22
const int GAS_PIN = 33;          // Gas sensor DOUT (digital)
const int SERVO_PIN = 19;        // Servo motor
const int BUZZER_PIN = 18;       // Buzzer
const int LCD_SDA = 21;          // LCD I2C SDA
const int LCD_SCL = 22;          // LCD I2C SCL

// Objects
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHT_PIN, DHT22);
Servo windowServo;

// Variables
float temperature = 0.0;
float humidity = 0.0;
bool gasDetected = false;        // Gas detected (digital)
bool ledWhiteStatus = false;     // LED trắng
bool ledRedStatus = false;       // LED đỏ cảnh báo
bool fanStatus = false;          // Servo (quạt/cửa sổ)
bool buzzerStatus = false;       // Buzzer cảnh báo
bool autoMode = true;           // Chế độ tự động

// Previous values for change detection
float lastPublishedTemp = -999.0;
float lastPublishedHum = -999.0;
bool lastPublishedGas = false;
bool lastPublishedLedWhite = false;
bool lastPublishedLedRed = false;
bool lastPublishedFan = false;

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
const unsigned long GAS_INTERVAL = 500;  // Kiểm tra gas thường xuyên hơn
const unsigned long GAS_WARMUP_TIME = 5000;  // 5 giây warm-up cho gas sensor

bool gasSensorReady = false;

// 📥 Subscribe (Nhận lệnh):
// kitchen/command/led - Điều khiển đèn trắng (ON/OFF/TOGGLE)
// kitchen/command/alarm - Điều khiển đèn đỏ cảnh báo (ON/OFF/TOGGLE)
// kitchen/command/fan - Điều khiển quạt/cửa sổ (ON/OFF/TOGGLE)
// kitchen/command/buzzer - Điều khiển còi (ON/OFF)
// kitchen/command/auto - Chế độ tự động (ON/OFF/TOGGLE)
// 📤 Publish (Gửi dữ liệu):
// kitchen/sensor/temperature - Nhiệt độ
// kitchen/sensor/humidity - Độ ẩm
// kitchen/sensor/gas - Phát hiện gas (YES/NO)
// kitchen/status/led - Trạng thái đèn trắng (ON/OFF)
// kitchen/status/alarm - Trạng thái cảnh báo (ON/OFF)
// kitchen/status/fan - Trạng thái quạt (ON/OFF)
// kitchen/status/auto - Trạng thái chế độ tự động (ON/OFF)

// ========== HELPER FUNCTIONS ==========

void readDHT() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  
  if (!isnan(temp) && !isnan(hum)) {
    temperature = temp;
    humidity = hum;
    Serial.print("DHT22 - Temp: ");
    Serial.print(temperature);
    Serial.print("°C, Humidity: ");
    Serial.print(humidity);
    Serial.println("%");
    
    // Publish if temperature changed by 0.5°C or more
    if (mqtt.connected() && abs(temperature - lastPublishedTemp) >= 0.5) {
      mqtt.publish("kitchen/sensor/temperature", String(temperature, 1).c_str());
      lastPublishedTemp = temperature;
      Serial.println("  -> Published temperature");
    }
    
    // Publish if humidity changed by 2% or more
    if (mqtt.connected() && abs(humidity - lastPublishedHum) >= 2.0) {
      mqtt.publish("kitchen/sensor/humidity", String((int)humidity).c_str());
      lastPublishedHum = humidity;
      Serial.println("  -> Published humidity");
    }
  } else {
    Serial.println("Failed to read DHT sensor!");
  }
}

void readGasSensor() {
  // Kiểm tra xem sensor đã warm-up chưa
  if (!gasSensorReady) {
    if (millis() - systemStartTime < GAS_WARMUP_TIME) {
      // Vẫn đang warm-up, bỏ qua
      return;
    } else {
      gasSensorReady = true;
      Serial.println("Gas sensor ready!");
    }
  }
  
  // Đọc giá trị digital (LOW = gas detected, HIGH = no gas)
  gasDetected = (digitalRead(GAS_PIN) == LOW);
  
  // Publish if gas status changed
  if (mqtt.connected() && gasDetected != lastPublishedGas) {
    mqtt.publish("kitchen/sensor/gas", gasDetected ? "YES" : "NO");
    lastPublishedGas = gasDetected;
    Serial.print("Gas Detected: ");
    Serial.println(gasDetected ? "YES" : "NO");
  }
  
  // Auto mode: handle gas warning
  if (autoMode) {
    if (gasDetected) {
      // Gas phát hiện: Bật LED đỏ, buzzer, mở quạt
      if (!ledRedStatus) {
        ledRedStatus = true;
        digitalWrite(LED_RED, HIGH);
        mqtt.publish("kitchen/status/alarm", "ON");
      }
      if (!buzzerStatus) {
        buzzerStatus = true;
        tone(BUZZER_PIN, 1000);  // 1kHz warning tone
      }
      if (!fanStatus) {
        fanStatus = true;
        windowServo.write(90);  // Mở quạt/cửa sổ
        mqtt.publish("kitchen/status/fan", "ON");
      }
    } else {
      // Không có gas: Tắt tất cả cảnh báo
      if (ledRedStatus) {
        ledRedStatus = false;
        digitalWrite(LED_RED, LOW);
        mqtt.publish("kitchen/status/alarm", "OFF");
      }
      if (buzzerStatus) {
        buzzerStatus = false;
        noTone(BUZZER_PIN);
      }
      if (fanStatus) {
        fanStatus = false;
        windowServo.write(0);  // Đóng quạt/cửa sổ
        mqtt.publish("kitchen/status/fan", "OFF");
      }
    }
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
  
  // Line 2: Gas status
  lcd.setCursor(0, 1);
  lcd.print("Gas:");
  
  if (gasDetected) {
    lcd.print("DETECTED!");
  } else {
    lcd.print("Safe");
  }
  
  if (autoMode) {
    lcd.setCursor(14, 1);
    lcd.print("A");  // Auto indicator
  }
}

void handleButton() {
  int reading = digitalRead(BUTTON_PIN);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      
      if (buttonState == LOW) {
        // Toggle LED white
        ledWhiteStatus = !ledWhiteStatus;
        digitalWrite(LED_WHITE, ledWhiteStatus);
        Serial.print("Button: LED White ");
        Serial.println(ledWhiteStatus ? "ON" : "OFF");
        
        // Publish LED status change
        if (mqtt.connected()) {
          mqtt.publish("kitchen/status/led", ledWhiteStatus ? "ON" : "OFF");
          lastPublishedLedWhite = ledWhiteStatus;
          Serial.println("  -> Published LED white status");
        }
        
        updateLCD();
      }
    }
  }
  
  lastButtonState = reading;
}

void publishAllStatus() {
  if (mqtt.connected()) {
    mqtt.publish("kitchen/sensor/temperature", String(temperature, 1).c_str());
    mqtt.publish("kitchen/sensor/humidity", String((int)humidity).c_str());
    mqtt.publish("kitchen/sensor/gas", gasDetected ? "YES" : "NO");
    mqtt.publish("kitchen/status/led", ledWhiteStatus ? "ON" : "OFF");
    mqtt.publish("kitchen/status/alarm", ledRedStatus ? "ON" : "OFF");
    mqtt.publish("kitchen/status/fan", fanStatus ? "ON" : "OFF");
    mqtt.publish("kitchen/status/auto", autoMode ? "ON" : "OFF");
    
    lastPublishedTemp = temperature;
    lastPublishedHum = humidity;
    lastPublishedGas = gasDetected;
    lastPublishedLedWhite = ledWhiteStatus;
    lastPublishedLedRed = ledRedStatus;
    lastPublishedFan = fanStatus;
    
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
  
  // Control LED white
  if (topicStr == "kitchen/command/led") {
    bool oldStatus = ledWhiteStatus;
    if (message == "ON" || message == "1") {
      ledWhiteStatus = true;
    } else if (message == "OFF" || message == "0") {
      ledWhiteStatus = false;
    } else if (message == "TOGGLE") {
      ledWhiteStatus = !ledWhiteStatus;
    }
    digitalWrite(LED_WHITE, ledWhiteStatus);
    
    if (ledWhiteStatus != oldStatus && mqtt.connected()) {
      mqtt.publish("kitchen/status/led", ledWhiteStatus ? "ON" : "OFF");
    }
    updateLCD();
  }
  
  // Control LED red
  else if (topicStr == "kitchen/command/alarm") {
    bool oldStatus = ledRedStatus;
    if (message == "ON" || message == "1") {
      ledRedStatus = true;
    } else if (message == "OFF" || message == "0") {
      ledRedStatus = false;
    } else if (message == "TOGGLE") {
      ledRedStatus = !ledRedStatus;
    }
    digitalWrite(LED_RED, ledRedStatus);
    
    if (ledRedStatus != oldStatus && mqtt.connected()) {
      mqtt.publish("kitchen/status/alarm", ledRedStatus ? "ON" : "OFF");
    }
  }
  
  // Control fan/servo
  else if (topicStr == "kitchen/command/fan") {
    bool oldStatus = fanStatus;
    if (message == "ON" || message == "1") {
      fanStatus = true;
      windowServo.write(90);
    } else if (message == "OFF" || message == "0") {
      fanStatus = false;
      windowServo.write(0);
    } else if (message == "TOGGLE") {
      fanStatus = !fanStatus;
      windowServo.write(fanStatus ? 90 : 0);
    }
    
    if (fanStatus != oldStatus && mqtt.connected()) {
      mqtt.publish("kitchen/status/fan", fanStatus ? "ON" : "OFF");
    }
  }
  
  // Control buzzer
  else if (topicStr == "kitchen/command/buzzer") {
    if (message == "ON" || message == "1") {
      buzzerStatus = true;
      tone(BUZZER_PIN, 1000);
    } else if (message == "OFF" || message == "0") {
      buzzerStatus = false;
      noTone(BUZZER_PIN);
    }
  }
  
  // Control auto mode
  else if (topicStr == "kitchen/command/auto") {
    if (message == "ON" || message == "1") {
      autoMode = true;
    } else if (message == "OFF" || message == "0") {
      autoMode = false;
    } else if (message == "TOGGLE") {
      autoMode = !autoMode;
    }
    
    mqtt.publish("kitchen/status/auto", autoMode ? "ON" : "OFF");
    Serial.print("Auto mode: ");
    Serial.println(autoMode ? "ON" : "OFF");
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
    String clientId = "ESP32_Kitchen_" + String(random(0xffff), HEX);
    
    if (mqtt.connect(clientId.c_str())) {
      Serial.println("Connected!");
      mqtt.subscribe("kitchen/command/led");
      mqtt.subscribe("kitchen/command/alarm");
      mqtt.subscribe("kitchen/command/fan");
      mqtt.subscribe("kitchen/command/buzzer");
      mqtt.subscribe("kitchen/command/auto");
      
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
  Serial.println("\n\n=== KITCHEN SYSTEM STARTING ===");
  
  // Lưu thời điểm khởi động để tính warm-up
  systemStartTime = millis();
  
  // Initialize pins
  pinMode(LED_WHITE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(GAS_PIN, INPUT);  // Digital input for gas sensor DOUT
  pinMode(BUZZER_PIN, OUTPUT);
  
  digitalWrite(LED_WHITE, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(BUZZER_PIN, LOW);  // Initialize buzzer off
  
  // Initialize servo
  windowServo.attach(SERVO_PIN);
  windowServo.write(0);  // Close position
  
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
  Serial.println("Gas sensor warming up (5 seconds)...");
  
  // Connect to WiFi
  connectWiFi();
  
  // Setup MQTT
  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(mqttCallback);
  
  // Initial sensor readings
  delay(2000);
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
  Serial.println("    * kitchen/command/led (ON/OFF/TOGGLE)");
  Serial.println("    * kitchen/command/alarm (ON/OFF/TOGGLE)");
  Serial.println("    * kitchen/command/fan (ON/OFF/TOGGLE)");
  Serial.println("    * kitchen/command/buzzer (ON/OFF)");
  Serial.println("    * kitchen/command/auto (ON/OFF/TOGGLE)");
  Serial.println("  Publish:");
  Serial.println("    * kitchen/sensor/temperature (float)");
  Serial.println("    * kitchen/sensor/humidity (int)");
  Serial.println("    * kitchen/sensor/gas (YES/NO)");
  Serial.println("    * kitchen/status/led (ON/OFF)");
  Serial.println("    * kitchen/status/alarm (ON/OFF)");
  Serial.println("    * kitchen/status/fan (ON/OFF)");
  Serial.println("    * kitchen/status/auto (ON/OFF)");
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
  
  // Read gas sensor frequently
  if (currentMillis - lastGasCheck >= GAS_INTERVAL) {
    lastGasCheck = currentMillis;
    readGasSensor();
  }
  
  // Update LCD display
  if (currentMillis - lastLCDUpdate >= LCD_INTERVAL) {
    lastLCDUpdate = currentMillis;
    updateLCD();
  }
  
  // Handle button input
  handleButton();
  
  delay(10);
}
