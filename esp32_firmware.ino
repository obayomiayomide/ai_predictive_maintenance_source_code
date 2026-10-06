#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include "SPIFFS.h"
#include <LiquidCrystal_I2C.h>

const char* WIFI_SSID     = "TECNO SPARK 40 Pro";
const char* WIFI_PASSWORD = "password";


const char* API_URL = "https://ac-health-monitor.onrender.com/predict";

const int SEND_INTERVAL = 2000;

LiquidCrystal_I2C lcd(0x27, 20, 4);

#define DHT_SUPPLY_PIN  4   // GPIO4 — DHT22 at AC vent outlet
#define DHT_ROOM_PIN    5   // GPIO5 — DHT22 for room ambient temp
#define DHT_TYPE        DHT22

#define ACS712_PIN      34  // GPIO34 — analog input (ADC1)
#define ACS712_SENSITIVITY 0.066  // 100mV per Amp for 20A version
#define ADC_VREF        3.3       // ESP32 ADC reference voltage
#define ADC_RESOLUTION  4095.0    // 12-bit ADC
#define VOLTAGE_DIVIDER_RATIO 0.5000 // Change to your resistor divider ratio if used (e.g., 0.66)

#define PRESSURE_LOW_PIN  35  // GPIO35 — low side (suction line)

#define WIFI_LED        15

DHT dht_supply(DHT_SUPPLY_PIN, DHT_TYPE);
DHT dht_room(DHT_ROOM_PIN, DHT_TYPE);
Adafruit_MPU6050 mpu;

#define VIB_SAMPLES 50        // samples to collect per cycle
float vib_mag_readings[VIB_SAMPLES];
int   vib_sample_idx = 0;
const int MPU_ADDR = 0x68;
float gyroZ_offset = 0;
float last_vib_magnitude = 0.0;

bool  compressor_was_on  = false;
unsigned long last_send_time = 0;

typedef struct {
  float supply_temp;      // DHT22 #1 — vent outlet (°C)
  float room_temp;        // DHT22 #2 — room ambient (°C)
  float temp_diff;        // room minus supply
  float vibration_mag;    // MPU6050 — magnitude
  float vibration_std;    // MPU6050 — std deviation
  float gyroscope;        // MPU6050 — gyro Z axis
  float current_A;        // ACS712 — compressor current
  float low_pressure;     // MPX5700AP — low side (PSI)
  int   compressor_on;    // 1 if compressor running, 0 if off
  bool  valid;            // false if any sensor read failed
} SensorData;

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n========================================");
  Serial.println("  AC HEALTH MONITOR — ESP32 Firmware");
  Serial.println("========================================\n");

  pinMode(WIFI_LED, OUTPUT);
  digitalWrite(WIFI_LED, LOW); // Start with LED off

  dht_supply.begin();
  dht_room.begin();
  Serial.println("✓ DHT22 sensors initialized");
 
  Wire.begin();
  Wire.setClock(100000);
  delay(500); 

  // --- Init LCD Display ---
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("AC Health Monitor");
  delay(2000);
  lcd.setCursor(0, 0);
  lcd.print("                    ");

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);     
  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.print("Hardware transmission failed with error code: ");
    Serial.println(error);
  }

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10); // 0x10 sets range to 8G
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08); // 0x08 sets range to 500 deg/s
  Wire.endTransmission();
  
  delay(500);

  // --- Connect WiFi ---
  connectWiFi();
}

void collectVibrationSample() {
  if (vib_sample_idx >= VIB_SAMPLES) return;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Accel X start register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);


  int16_t ax = 0, ay = 0, az = 0;
  if (Wire.available() >= 6) {
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
  }

  // Euclidean magnitude of 3-axis acceleration
  float mag = sqrt((float)ax*ax + (float)ay*ay + (float)az*az);
  vib_mag_readings[vib_sample_idx] = mag;
  vib_sample_idx++;

}

float computeVibrationMagnitude() {
  if (vib_sample_idx < VIB_SAMPLES) return last_vib_magnitude;
  float sum = 0;
  for (int i = 0; i < vib_sample_idx; i++) sum += vib_mag_readings[i];
  float mean = sum / vib_sample_idx;
  last_vib_magnitude = mean;
  return mean;
}

float computeVibrationStd() {
  if (vib_sample_idx < VIB_SAMPLES) return 0.0;
  float sum = 0;
  for (int i = 0; i < vib_sample_idx; i++) {
    sum += vib_mag_readings[i];
  }
  float true_mean = sum / vib_sample_idx;

  float sum_squared_diff = 0;
  for (int i = 0; i < vib_sample_idx; i++) {
    float diff = vib_mag_readings[i] - true_mean;
    sum_squared_diff += diff * diff;
  }

  return sqrt(sum_squared_diff / (vib_sample_idx - 1));
}

float readGyroData() {
  Wire.beginTransmission(MPU_ADDR);
  byte error = Wire.endTransmission(); 

 
  static float gyroZ_offset = 0;
  static bool is_calibrated = false;

  if (!is_calibrated && error == 0) {
    long total_raw = 0;
    int samples = 200;
    
    for(int i = 0; i < samples; i++) {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x47); 
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, 2);
      
      if (Wire.available() >= 2) {
        total_raw += (int16_t)((Wire.read() << 8) | Wire.read());
      }
      delay(5); // Fast sampling for calibration
    }
    gyroZ_offset = (float)total_raw / samples;
    is_calibrated = true; // Prevents this loop from ever running again
  }

  if(error == 0) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2);

    int16_t raw_gz = 0;
    if (Wire.available() >= 2) {
      raw_gz = (Wire.read() << 8) | Wire.read();
    }
    return ((float)raw_gz - gyroZ_offset) / 65.5;
  } else {
    return 0.0;;
  }
}

float readCurrent() {
  int maxValue = 0;          
  int minValue = 4095;       
  int readValue;
  
  uint32_t start_time = millis();
  while ((millis() - start_time) < 20) {
    readValue = analogRead(ACS712_PIN);

    if (readValue <= 15 || readValue >= 4080) {
     
      return -999.0; // Instantly exit the function with an error code
    }

    if (readValue > maxValue) {
      maxValue = readValue;
    }
    if (readValue < minValue) {
      minValue = readValue;
    }
  }

  int adcPeakToPeak = maxValue - minValue;
  float voltagePP = (adcPeakToPeak / ADC_RESOLUTION) * ADC_VREF;
  
  voltagePP = voltagePP / VOLTAGE_DIVIDER_RATIO; 
  
  float voltageRMS = (voltagePP / 2.0) * 0.7071;
  
  float currentRMS = voltageRMS / ACS712_SENSITIVITY;
 
  if (currentRMS < 0.15) {
    currentRMS = 0.0;
  }
  
  return abs(currentRMS);
}

float readPressurePSI(int pin) {
    long sum = 0;
    for (int i = 0; i < 20; i++) { sum += analogRead(pin); delay(1); }
    float avg_adc = sum / 20.0;

    if (avg_adc <= 15.0 || avg_adc >= 4080.0) {
       
        return -999.0; // Return error indicator code immediately
    }

    float vout = (avg_adc / 4095.0) * 3.3;
    vout = vout * (10.0 + 6.8) / 6.8;  // reverse divider → actual sensor voltage

    // Linear transfer function: 0.5V=0PSI, 4.5V=150PSI
    float p_psi = (vout - 0.5) / 4.0 * 150.0;
    p_psi = constrain(p_psi, 0.0, 150.0);
    return p_psi;
}

SensorData readAllSensors() {
  SensorData data;
  data.valid = true;

  data.supply_temp = dht_supply.readTemperature();
  data.room_temp   = dht_room.readTemperature();

  if (isnan(data.supply_temp) || isnan(data.room_temp)) {
    Serial.println("⚠ DHT22 read failed — using last valid reading");
  
    data.supply_temp = isnan(data.supply_temp) ? 0.0 : data.supply_temp;
    data.room_temp   = isnan(data.room_temp)   ? 0.0 : data.room_temp;
    data.valid = false;
  }
  data.temp_diff = data.room_temp - data.supply_temp;


  data.vibration_mag = computeVibrationMagnitude();
  data.vibration_std = computeVibrationStd();

  data.gyroscope = readGyroData();
  if (data.gyroscope == 0.0) {
    data.valid = false;
  }

  vib_sample_idx = 0;

  data.current_A = readCurrent();
  if (data.current_A == -999.0) {
    data.valid = false;
  }
 
  data.low_pressure  = readPressurePSI(PRESSURE_LOW_PIN);
  if (data.low_pressure == -999.0) {
    data.valid = false;
  }


  data.compressor_on = (data.current_A > 2.0 && data.vibration_mag > 4500) ? 1 : 0;

  return data;
}

void sendToAPI(SensorData data) {
  HTTPClient http;
  http.begin(API_URL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(5000);  // 5 second timeout

  StaticJsonDocument<256> doc;
  doc["device_id"]      = "unit1"; 
  doc["supply_temp"]    = round(data.supply_temp * 10) / 10.0;
  doc["room_temp"]      = round(data.room_temp * 10) / 10.0;
  doc["vibration_mag"]  = (int)data.vibration_mag;
  doc["vibration_std"]  = round(data.vibration_std * 10) / 10.0;
  doc["gyroscope"]      = round(data.gyroscope * 10) / 10.0;
  doc["current"]        = round(data.current_A * 100) / 100.0;
  doc["low_pressure"]   = round(data.low_pressure * 10) / 10.0;
  doc["compressor_on"]  = data.compressor_on;

  String jsonBody;
  serializeJson(doc, jsonBody);

  int responseCode = http.POST(jsonBody);

  if (responseCode == 200) {
    String response = http.getString();

    StaticJsonDocument<512> respDoc;
    DeserializationError error = deserializeJson(respDoc, response);
    if (!error) {
      Serial.println("\n--- PREDICTION RESULT ---");
      Serial.print("  Final prediction: ");
      Serial.println(respDoc["final_prediction"].as<String>());
      Serial.print("  RF:   ");
      Serial.print(respDoc["rf_prediction"].as<String>());
      Serial.print(" (");
      Serial.print(respDoc["rf_confidence"].as<float>(), 1);
      Serial.println("%)");
      Serial.print("  LSTM: ");
      Serial.print(respDoc["lstm_prediction"].as<String>());
      Serial.print(" (");
      Serial.print(respDoc["lstm_confidence"].as<float>(), 1);
      Serial.println("%)");
      Serial.print("  Alert: ");
      Serial.println(respDoc["alert_level"].as<String>());
      Serial.print("  Recommendation: ");
      Serial.println(respDoc["recommendation"].as<String>());
      Serial.println("-------------------------\n");
    }
  } else if (responseCode > 0) {
    Serial.print("⚠ API error — HTTP code: ");
    Serial.println(responseCode);
  } else {
    Serial.print("⚠ Connection failed — code: ");
    Serial.println(responseCode);
  }

  http.end();
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print("Connecting to WiFi: ");
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    digitalWrite(WIFI_LED, HIGH);
    delay(250);
    digitalWrite(WIFI_LED, LOW);
    delay(250);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi connected!");
    Serial.print("  IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("  Signal strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println("\n⚠ WiFi connection failed — will retry");
  }
}

void updateLCD(SensorData data) {

  lcd.clear();
  
  lcd.setCursor(0, 0);
  lcd.print("Temp: Rm:");
  if (data.room_temp == 0.0) {
    lcd.print("-- ");
  } else {
    lcd.print(data.room_temp, 0);
    lcd.print("C ");
  }
  lcd.print("Sp:");

  if (data.supply_temp == 0.0) {
    lcd.print("--");
  } else {
    lcd.print(data.supply_temp, 0);
    lcd.print("C  ");
  }

  lcd.setCursor(0, 1);
  lcd.print("Pressure:");
  if (data.low_pressure == -999.0) {
    lcd.print("--");
  } else {
    lcd.print(data.low_pressure, 2);
    lcd.print("PSI");
  }

  lcd.setCursor(0, 2);
  lcd.print("Current:");
  if (data.current_A == -999.0) {
    lcd.print("--");
  } else {
    lcd.print(data.current_A, 2);
    lcd.print("A ");
  }

  lcd.setCursor(0, 3);
  lcd.print("Vib Mag: ");
  if (data.gyroscope == 0.0) {
    lcd.print("--");
  } else {
    lcd.print(data.vibration_mag, 0);
  }
  lcd.print("   "); 
}

void printSensorData(SensorData data) {
  Serial.println("=== SENSOR READINGS ===");
  Serial.print("  Supply temp:    "); Serial.print(data.supply_temp, 1); Serial.println(" °C");
  Serial.print("  Room temp:      "); Serial.print(data.room_temp, 1);   Serial.println(" °C");
  Serial.print("  Temp diff:      "); Serial.print(data.temp_diff, 1);   Serial.println(" °C");
  Serial.print("  Vibration mag:  "); Serial.println((int)data.vibration_mag);
  Serial.print("  Vibration std:  "); Serial.println(data.vibration_std, 1);
  Serial.print("  Gyroscope Z:    "); Serial.print(data.gyroscope, 1);   Serial.println(" deg/s");
  Serial.print("  Current:        "); Serial.print(data.current_A, 2);   Serial.println(" A");
  Serial.print("  Low pressure:   "); Serial.print(data.low_pressure, 1); Serial.println(" PSI");
  Serial.print("  Compressor:     "); Serial.println(data.compressor_on ? "ON" : "OFF");
  if (!data.valid) Serial.println("  ⚠ Some sensor reads failed this cycle");
  Serial.println("=======================");
}

void loop() {
  unsigned long now = millis();

  if (vib_sample_idx < VIB_SAMPLES) {
    collectVibrationSample(); 
    return; 

  if (now - last_send_time >= SEND_INTERVAL) {
    last_send_time = now;

    SensorData data = readAllSensors();

    printSensorData(data);
    updateLCD(data);

    if (WiFi.status() == WL_CONNECTED) {
     
      digitalWrite(WIFI_LED, HIGH);
      if(data.valid){
        sendToAPI(data);
      } else {
        Serial.println("❌ Critical: Sensor hardware read failed. Dropping this cycle entirely.");
      }
      
    } else {
      Serial.println("⚠ WiFi disconnected");
      connectWiFi();
    }
  }
}

