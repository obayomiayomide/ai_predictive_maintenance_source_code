// ============================================================
// AC/HVAC PREDICTIVE MAINTENANCE SYSTEM
// ESP32 Firmware
//
// Sensors:
//   - DHT22 x2  : Supply air temp + Room ambient temp
//   - MPU6050   : Vibration (accelerometer + gyroscope)
//   - ACS712    : Compressor current draw
//   - MPX5700AP x2 : Low side + High side refrigerant pressure
//
// Flow:
//   Read sensors → Package JSON → POST to Flask API → Print response
//
// Libraries needed (install via Arduino Library Manager):
//   - DHT sensor library by Adafruit
//   - Adafruit Unified Sensor
//   - MPU6050 by Electronic Cats (or Adafruit MPU6050)
//   - ArduinoJson by Benoit Blanchon
//   - WiFi (built into ESP32 board package)
//   - HTTPClient (built into ESP32 board package)
// ============================================================

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

// ============================================================
// CONFIGURATION — CHANGE THESE TO MATCH YOUR SETUP
// ============================================================

// WiFi credentials
const char* WIFI_SSID     = "TECNO SPARK 40 Pro";
const char* WIFI_PASSWORD = "password";

// Flask API URL — replace with your Render URL after deployment
// For local testing use your PC's IP: "http://192.168.x.x:5000/predict"
const char* API_URL = "https://ac-health-monitor.onrender.com/predict";
// const char* API_URL = "http://10.177.98.173:5000/predict";

// Sending interval (milliseconds) — 2 seconds
const int SEND_INTERVAL = 2000;

LiquidCrystal_I2C lcd(0x27, 20, 4);

// ============================================================
// PIN DEFINITIONS
// ============================================================

// DHT22 sensors
#define DHT_SUPPLY_PIN  4   // GPIO4 — DHT22 at AC vent outlet
#define DHT_ROOM_PIN    5   // GPIO5 — DHT22 for room ambient temp
#define DHT_TYPE        DHT22

// ACS712 current sensor (analog)
#define ACS712_PIN      34  // GPIO34 — analog input (ADC1)
#define ACS712_SENSITIVITY 0.066  // 100mV per Amp for 20A version
#define ADC_VREF        3.3       // ESP32 ADC reference voltage
#define ADC_RESOLUTION  4095.0    // 12-bit ADC
#define VOLTAGE_DIVIDER_RATIO 0.5000 // Change to your resistor divider ratio if used (e.g., 0.66)
// #define ACS712_ZERO_VOLTAGE 2.5   // Voltage at 0A (Vcc/2 = 2.5V on 5V)

// Pressure sensors (analog)
#define PRESSURE_LOW_PIN  35  // GPIO35 — low side (suction line)
// MPX5700AP: Vout = Vs * (0.01294 * P + 0.00369)  where P in kPa
// Rearranged: P = (Vout/Vs - 0.00369) / 0.01294  in kPa
// 1 kPa = 0.14504 PSI
// #define MPX_VS          5.0   // sensor supply voltage (5V)
// #define MPX_MAX_KPA     700.0 // maximum rated pressure
#define WIFI_LED        15

// ============================================================
// SENSOR OBJECTS
// ============================================================
DHT dht_supply(DHT_SUPPLY_PIN, DHT_TYPE);
DHT dht_room(DHT_ROOM_PIN, DHT_TYPE);
Adafruit_MPU6050 mpu;

// ============================================================
// VIBRATION CALCULATION VARIABLES
// Window of readings used to compute vibration_magnitude
// and vibration_std for the current 2-second cycle
// ============================================================
#define VIB_SAMPLES 50        // samples to collect per cycle
float vib_mag_readings[VIB_SAMPLES];
int   vib_sample_idx = 0;
const int MPU_ADDR = 0x68;
float gyroZ_offset = 0;
float last_vib_magnitude = 0.0;

// ============================================================
// OFFLINE BUFFER
// Stores readings when WiFi is unavailable
// Saves to SPIFFS flash memory so data survives reboots
// ============================================================
#define BUFFER_FILE "/buffer.json"
#define MAX_BUFFER_SIZE 500   // max readings to store offline

// ============================================================
// STATE VARIABLES
// ============================================================
bool  compressor_was_on  = false;
unsigned long last_send_time = 0;

// ============================================================
// SENSOR DATA STRUCTURE
// ============================================================
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

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n========================================");
  Serial.println("  AC HEALTH MONITOR — ESP32 Firmware");
  Serial.println("========================================\n");

  pinMode(WIFI_LED, OUTPUT);
  digitalWrite(WIFI_LED, LOW); // Start with LED off

  // --- Init sensors ---
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

  // // 1. Manually ask the chip what its hidden internal ID is
  // Wire.beginTransmission(0x68);
  // Wire.write(0x75); // Request the "Who Am I" register
  // Wire.endTransmission(false);
  // Wire.requestFrom(0x68, 1);
  
  // if (Wire.available()) {
  //   byte chipID = Wire.read();
  //   Serial.print("-> Internal Chip ID reported by hardware: 0x");
  //   Serial.println(chipID, HEX);
  // }
  // Serial.println("Initializing MPU6050 with ID check disabled...");
  // // 2. FORCED INITIALIZATION
  // // We use standard initialization, but if it fails, we will force the engine to start anyway
  // if (!mpu.begin(0x68)) {
  //   Serial.println("Warning: Strict ID check failed. Forcing manual configuration...");
  // }
  // if (!mpu.begin(0x68)) {
  //   Serial.println("Initialization failed. Trying alternative approach...");
  //   // If it fails, try without an explicit address argument to let the library auto-manage it
  //   if (!mpu.begin()) { 
  //     Serial.println("CRITICAL ERROR: MPU6050 register initialization failed completely!");
  //     // Hard stop: If it ever drops connection, freeze here instead of reading fake noise
  //     while (1) { 
  //       delay(10); 
  //     }
  //   }
  // }

  // Serial.println("Success! MPU6050 initialized properly at address 0x68.");

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);     
  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.print("Hardware transmission failed with error code: ");
    Serial.println(error);
  }

  // Set Accelerometer configuration register (0x1C) to +/- 8G range
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10); // 0x10 sets range to 8G
  Wire.endTransmission();

  // Set Gyroscope configuration register (0x1B) to +/- 500 deg/s range
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08); // 0x08 sets range to 500 deg/s
  Wire.endTransmission();

  // Set safe ranges
  // mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  delay(500);

  // --- Init SPIFFS for offline buffering ---
  if (!SPIFFS.begin(true)) {
    Serial.println("⚠ SPIFFS init failed — offline buffering disabled");
  } else {
    Serial.println("✓ SPIFFS ready for offline buffering");
  }

  // --- Connect WiFi ---
  connectWiFi();
}

// ============================================================
// VIBRATION HELPERS
// Samples collected at high rate between send cycles
// giving a statistically meaningful window per 2-second period
// ============================================================
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

  // delayMicroseconds(500);  // ~2kHz sampling rate
}

float computeVibrationMagnitude() {
  if (vib_sample_idx < VIB_SAMPLES) return last_vib_magnitude;
  float sum = 0;
  for (int i = 0; i < vib_sample_idx; i++) sum += vib_mag_readings[i];
  float mean = sum / vib_sample_idx;
  last_vib_magnitude = mean;
  return mean;
}

// float computeVibrationStd() {
//   if (vib_sample_idx < 2) return 0.0;
//   float mean = computeVibrationMagnitude();
//   float variance = 0;
//   for (int i = 0; i < vib_sample_idx; i++) {
//     float diff = vib_mag_readings[i] - mean;
//     variance += diff * diff;
//   }
//   return sqrt(variance / vib_sample_idx);
// }
float computeVibrationStd() {
  if (vib_sample_idx < VIB_SAMPLES) return 0.0;

  // 1. Calculate the true mathematical mean of the current window
  float sum = 0;
  for (int i = 0; i < vib_sample_idx; i++) {
    sum += vib_mag_readings[i];
  }
  float true_mean = sum / vib_sample_idx;

  // 2. Calculate the variance using the sample size correction (N - 1)
  float sum_squared_diff = 0;
  for (int i = 0; i < vib_sample_idx; i++) {
    float diff = vib_mag_readings[i] - true_mean;
    sum_squared_diff += diff * diff;
  }

  // Dividing by (vib_sample_idx - 1) gives the standard engineering Sample Std Dev
  return sqrt(sum_squared_diff / (vib_sample_idx - 1));
}

// Assuming your function returns the data structure
float readGyroData() {
  Wire.beginTransmission(MPU_ADDR);
  byte error = Wire.endTransmission(); 

  // --- Static variables remember their values across function calls ---
  static float gyroZ_offset = 0;
  static bool is_calibrated = false;

  // --- Run Calibration ONCE on the very first function call ---
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

  // --- Packet 2: Read Gyroscope Z Data ---
  if(error == 0) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2);

    int16_t raw_gz = 0;
    if (Wire.available() >= 2) {
      raw_gz = (Wire.read() << 8) | Wire.read();
    }

    // Now subtracts the internally saved offset
    return ((float)raw_gz - gyroZ_offset) / 65.5;
  } else {
    return 0.0;;
  }
}


// ============================================================
// CURRENT READING (ACS712)
// Takes 50 ADC samples and averages to reduce noise
// ============================================================
// float readCurrent() {
//   long sum = 0;
//   int samples = 50;
//   for (int i = 0; i < samples; i++) {
//     sum += analogRead(ACS712_PIN);
//     delayMicroseconds(100);
//   }
//   float avg_adc = sum / (float)samples;

//   // Convert ADC reading to voltage
//   float voltage = (avg_adc / ADC_RESOLUTION) * ADC_VREF;

//   // Convert voltage to current (A)
//   // ACS712 zero current voltage is 2.5V on 5V supply
//   // But ESP32 runs at 3.3V so the midpoint shifts:
//   float zero_point = ADC_RESOLUTION / 2.0;  // ~2047.5 at 0A
//   float current = ((avg_adc - zero_point) / ADC_RESOLUTION * ADC_VREF)
//                   / ACS712_SENSITIVITY;

//   return abs(current);  // always positive
// }

float readCurrent() {
  int maxValue = 0;          
  int minValue = 4095;       
  int readValue;
  
  // Sample for 20 milliseconds to capture at least one full AC wave cycle
  uint32_t start_time = millis();
  while ((millis() - start_time) < 20) {
    readValue = analogRead(ACS712_PIN);

    if (readValue <= 15 || readValue >= 4080) {
      // Set global flag or reference flag here if you use one, e.g.:
      // data.current_error = true;
      return -999.0; // Instantly exit the function with an error code
    }

    if (readValue > maxValue) {
      maxValue = readValue;
    }
    if (readValue < minValue) {
      minValue = readValue;
    }
  }
  
  // 1. Calculate Peak-to-Peak ADC steps
  int adcPeakToPeak = maxValue - minValue;
  
  // 2. Convert ADC steps to Peak-to-Peak Voltage
  float voltagePP = (adcPeakToPeak / ADC_RESOLUTION) * ADC_VREF;
  
  // Adjust for a hardware voltage divider if you are using one
  voltagePP = voltagePP / VOLTAGE_DIVIDER_RATIO; 
  
  // 3. Convert Peak-to-Peak Voltage to RMS Voltage (Vrms = Vpp / 2 * 0.707)
  float voltageRMS = (voltagePP / 2.0) * 0.7071;
  
  // 4. Convert Voltage to RMS Current using your 20A sensitivity (0.100 V/A)
  float currentRMS = voltageRMS / ACS712_SENSITIVITY;
  
  // 5. Noise floor cutoff (removes ghost readings when compressor is completely off)
  if (currentRMS < 0.15) {
    currentRMS = 0.0;
  }
  
  return abs(currentRMS);
}

// ============================================================
// For generic HVAC transducer (0.5V-4.5V output, 5V supply):
// P_psi = (Vout - 0.5) / (4.5 - 0.5) * 500
// i.e., maps 0.5V→0 PSI and 4.5V→500 PSI linearly
// ============================================================
float readPressurePSI(int pin) {
    long sum = 0;
    for (int i = 0; i < 20; i++) { sum += analogRead(pin); delay(1); }
    float avg_adc = sum / 20.0;

    if (avg_adc <= 15.0 || avg_adc >= 4080.0) {
        // Set global flag here if you track it outside, e.g., data.pressure_error = true;
        return -999.0; // Return error indicator code immediately
    }

    // ADC → voltage (accounting for voltage divider)
    float vout = (avg_adc / 4095.0) * 3.3;
    vout = vout * (10.0 + 6.8) / 6.8;  // reverse divider → actual sensor voltage

    // Linear transfer function: 0.5V=0PSI, 4.5V=150PSI
    float p_psi = (vout - 0.5) / 4.0 * 150.0;
    p_psi = constrain(p_psi, 0.0, 150.0);
    return p_psi;
}
//p_kpa = p_psi / 0.14504

// NOTE: MPX5700AP runs on 5V but ESP32 ADC max is 3.3V
// Use a voltage divider (10kΩ + 6.8kΩ) to scale 5V → 3.3V
// before connecting to ESP32 pin. Account for that here:

// ============================================================
// READ ALL SENSORS
// ============================================================
SensorData readAllSensors() {
  SensorData data;
  data.valid = true;

  // --- DHT22 Temperature Sensors ---
  // DHT22 needs ~250ms between reads; readings are taken
  // here at 2-second intervals so timing is fine
  data.supply_temp = dht_supply.readTemperature();
  data.room_temp   = dht_room.readTemperature();

  if (isnan(data.supply_temp) || isnan(data.room_temp)) {
    Serial.println("⚠ DHT22 read failed — using last valid reading");
    // Keep last valid or set a safe default
    data.supply_temp = isnan(data.supply_temp) ? 0.0 : data.supply_temp;
    data.room_temp   = isnan(data.room_temp)   ? 0.0 : data.room_temp;
    data.valid = false;
  }
  data.temp_diff = data.room_temp - data.supply_temp;

  // --- MPU6050 Vibration ---
  // Use the window of samples collected since last send cycle
  data.vibration_mag = computeVibrationMagnitude();
  data.vibration_std = computeVibrationStd();

  // // Gyroscope Z axis (most relevant for compressor rotation wobble)
  // sensors_event_t a, g, temp;
  
  // // Try to grab data event maps
  // mpu.getEvent(&a, &g, &temp);
  
  // // 3. GYROSCOPE CONVERSION: Replaces 'data.gyroscope = gz / 65.5;'
  // // Converts the raw rad/s directly into degrees per second to match your old math
  // data.gyroscope = g.gyro.z * RAD_TO_DEG;  // convert rad/s to deg/s (±500 range)

  data.gyroscope = readGyroData();
  if (data.gyroscope == 0.0) {
    data.valid = false;
  }

  // Wire.beginTransmission(MPU_ADDR);
  // byte error = Wire.endTransmission(); 

  // // --- Packet 2: Read Gyroscope Z Data ---
  // if(error == 0) {
  //   Wire.beginTransmission(MPU_ADDR);
  //   Wire.write(0x47); // Gyro Z start register
  //   Wire.endTransmission(false);
  //   Wire.requestFrom(MPU_ADDR, 2);

  //   int16_t raw_gz = 0;
  //   if (Wire.available() >= 2) {
  //     raw_gz = (Wire.read() << 8) | Wire.read();
  //   }

  //   // Convert raw Gyro Z integer to standard degrees/sec scale
  //   // At +/- 500 deg/s configuration, the sensitivity is 65.5 LSB per deg/s
  //   data.gyroscope = (float)raw_gz / 65.5;
  // } else {
  //   data.gyroscope = 0.0;
  //   data.valid = false;
  // }
  

  // Reset vibration sample window for next cycle
  vib_sample_idx = 0;

  // --- ACS712 Current Sensor ---
  data.current_A = readCurrent();
  if (data.current_A == -999.0) {
    data.valid = false;
  }
  // --- Pressure Sensors ---
  data.low_pressure  = readPressurePSI(PRESSURE_LOW_PIN);
  if (data.low_pressure == -999.0) {
    data.valid = false;
  }

  // if (data.low_pressure == 0.0) {
  //   data.valid = false;
  // }

  // if (!data.valid && data.current_A == 0 && data.low_pressure != 0.0) {
  //   data.valid = false;
  // }
  
  // --- Determine Compressor State ---
  // Compressor is ON if current > 2A and vibration > 3000
  data.compressor_on = (data.current_A > 2.0 && data.vibration_mag > 4500) ? 1 : 0;

  return data;
}

// ============================================================
// SEND TO FLASK API
// ============================================================
void sendToAPI(SensorData data) {
  HTTPClient http;
  http.begin(API_URL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(5000);  // 5 second timeout

  // Build JSON payload
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

    // Parse and display the prediction
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
    // Save to offline buffer
    // saveToBuffer(data);
  }

  http.end();
}

// ============================================================
// OFFLINE BUFFERING (SPIFFS)
// Saves readings to flash when WiFi unavailable
// Flushes buffer when connection restored
// ============================================================
// void saveToBuffer(SensorData data) {
//   // Read existing buffer
//   File file = SPIFFS.open(BUFFER_FILE, "a");
//   if (!file) {
//     Serial.println("⚠ Could not open buffer file");
//     return;
//   }

//   StaticJsonDocument<256> doc;
//   doc["supply_temp"]   = data.supply_temp;
//   doc["room_temp"]     = data.room_temp;
//   doc["vibration_mag"] = data.vibration_mag;
//   doc["vibration_std"] = data.vibration_std;
//   doc["gyroscope"]     = data.gyroscope;
//   doc["current"]       = data.current_A;
//   doc["low_pressure"]  = data.low_pressure;
//   doc["compressor_on"] = data.compressor_on;

//   serializeJson(doc, file);
//   file.println();
//   file.close();

//   Serial.println("✓ Reading saved to offline buffer");
// }

// void flushOfflineBuffer() {
//   if (!SPIFFS.exists(BUFFER_FILE)) return;

//   File file = SPIFFS.open(BUFFER_FILE, "r");
//   if (!file) return;

//   int flushed = 0;
//   while (file.available() && WiFi.status() == WL_CONNECTED) {
//     String line = file.readStringUntil('\n');
//     line.trim();
//     if (line.length() == 0) continue;

//     HTTPClient http;
//     http.begin(API_URL);
//     http.addHeader("Content-Type", "application/json");
//     http.setTimeout(5000);
//     int code = http.POST(line);
//     http.end();

//     if (code == 200) flushed++;
//     delay(200);
//   }

//   file.close();

//   if (flushed > 0) {
//     SPIFFS.remove(BUFFER_FILE);
//     Serial.print("✓ Flushed ");
//     Serial.print(flushed);
//     Serial.println(" buffered readings to API");
//   }
// }

// ============================================================
// WIFI CONNECTION
// ============================================================
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
  // if (!data.valid) {
  //   lcd.setCursor(0, 1);
  //   lcd.print("Sensor Read Error! ");
  //   Serial.println("Sensor Read Error!");
  //   return;
  // }
  lcd.clear();
  
  // Line 1: Room & Supply Temperature
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

  // Line 2: Pressure (PSI) & Current (Amps)
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

  // Line 3: Vibration Magnitude
  lcd.setCursor(0, 3);
  lcd.print("Vib Mag: ");
  if (data.gyroscope == 0.0) {
    lcd.print("--");
  } else {
    lcd.print(data.vibration_mag, 0);
  }
  lcd.print("   "); 
}

// ============================================================
// SERIAL MONITOR DEBUGGING
// ============================================================
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

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  // Continuously collect vibration samples between send cycles
  if (vib_sample_idx < VIB_SAMPLES) {
    collectVibrationSample(); 
    return; // Skip the rest of the loop until we have all 50 samples!
  }

  // Send data every SEND_INTERVAL milliseconds
  if (now - last_send_time >= SEND_INTERVAL) {
    last_send_time = now;

    // Read all sensors
    SensorData data = readAllSensors();

    // Print to serial monitor for debugging
    printSensorData(data);
    updateLCD(data);

    // Send to API (or buffer if offline)
    if (WiFi.status() == WL_CONNECTED) {
      // First flush any buffered offline readings
      // flushOfflineBuffer();
      // Then send current reading
      digitalWrite(WIFI_LED, HIGH);
      if(data.valid){
        sendToAPI(data);
      } else {
        Serial.println("❌ Critical: Sensor hardware read failed. Dropping this cycle entirely.");
      }
      
    } else {
      Serial.println("⚠ WiFi disconnected");
      // saveToBuffer(data);
      // Attempt reconnect
      connectWiFi();
    }
  }
}

