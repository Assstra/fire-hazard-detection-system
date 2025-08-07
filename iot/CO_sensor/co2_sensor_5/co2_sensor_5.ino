#include <SoftwareSerial.h>

#define UART_RX 10
#define UART_TX 11

SoftwareSerial es920(UART_RX, UART_TX);

// Information
const String device_id = "005";

// PWM related
const int PWM_PIN = 3;
const float duty_value = 71.4;

// Measuring/Heating phases management
unsigned long startTime;
bool isMeasuring = false;
bool baselineInitialized = false;

// Store values in array for sliding mean
const int array_size = 10;
int sensor_prev_values[array_size];
int array_index = 0;
int readings_count = 0; // Track how many readings we have

// Trick to avoid too much printed values
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 1000;

// Baseline mean updated every 10 seconds
float mean = 0;
float baseline_mean = 0;
unsigned long baseline_time = 0;
const unsigned long baseline_update_interval = 10000;
const unsigned long stabilization_time = 10000; // 10 seconds to stabilize

// Fire detection threshold
const float fire_threshold = 2.0;

// Healthcheck timing
unsigned long lastHealthcheckTime = 0;
const unsigned long healthcheckInterval = 300000; // 5 Minutes

// Button (for tests only)
const int BUTTON_PIN = 13;
int buttonState = 0;

int reference_resistor_kOhm = 10;

// 5V - 60 seconds
void heating_phase() {
  digitalWrite(PWM_PIN, HIGH);
  Serial.println("Heating phase started");
}

// 1.4V - 90 seconds
void measuring_phase() {
  analogWrite(PWM_PIN, duty_value);
  Serial.println("Measuring phase started");
}

// LoRa message sender
void send_message(String type, unsigned int currentTime, String additional_info) {
  
  es920.print(type);
  es920.print("-");
  es920.print(device_id);
  es920.print("-");
  es920.print(currentTime);
  es920.print("-");
  es920.print(additional_info);
  es920.println("#");
  
  Serial.print("Message sent to ES920 N01:");
  Serial.print(type);
  Serial.print("-");
  Serial.print(device_id);
  Serial.print("-");
  Serial.print(currentTime);
  Serial.print("-");
  Serial.println(additional_info);
}

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  
  // Initialize sensor array with zeros
  for (int i = 0; i < array_size; i++) {
    sensor_prev_values[i] = 0;
  }

  // Hardware serial (via USB)
  Serial.begin(9600);
  Serial.println("-- CO2 Fire Detection System Init --");
  Serial.print("Fire threshold: ");
  Serial.println(fire_threshold);

  // Software serial (via digital pins)
  es920.begin(9600);

  heating_phase();
  startTime = millis();
  lastHealthcheckTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Measuring phase loop
  if (isMeasuring) {
    
    if (currentTime - startTime >= 90000) {
      Serial.println("Measuring -> Heating");
      startTime = millis();
      isMeasuring = false;
      baselineInitialized = false; // Reset baseline when switching phases
      readings_count = 0;
      heating_phase();
      
    } else {
      
      // Reading sensor phase loop
      int sensorValue = analogRead(A0);
      sensor_prev_values[array_index] = sensorValue;
      array_index = (array_index + 1) % array_size;
      readings_count = min(readings_count + 1, array_size);

      // Wait for stabilization and ensure we have enough readings
      if (currentTime - startTime >= stabilization_time && readings_count >= array_size) {
        
        // Calculate sliding mean
        mean = 0;
        for (int i = 0; i < array_size; i++) {
          mean += sensor_prev_values[i];
        }
        mean = mean / array_size;

        // Initialize baseline if not done yet
        if (!baselineInitialized) {
          baseline_mean = mean;
          baseline_time = currentTime;
          baselineInitialized = true;
          Serial.print("Baseline initialized: ");
          Serial.println(baseline_mean);
        }
  
        // Show readings every second
        if (currentTime - lastPrintTime >= printInterval) {
          Serial.print("Current: ");
          Serial.print(mean);
          Serial.print(", Baseline: ");
          Serial.print(baseline_mean);
          Serial.print(", Diff: ");
          Serial.println(mean - baseline_mean);
  
          // Update baseline periodically if conditions are stable
          if (currentTime - baseline_time >= baseline_update_interval) {
            float diff = abs(mean - baseline_mean);
            // Only update baseline if reading is stable (small change from current baseline)
            if (diff < 2.0) {
              baseline_mean = (baseline_mean * 0.9) + (mean * 0.1); // Smooth baseline update
              baseline_time = currentTime;
              Serial.println("Baseline updated (periodic)");
            }
          }
          
          // Update baseline if current reading is significantly lower (cleaner air)
          if (mean < baseline_mean - 2.0) {
            baseline_mean = mean;
            baseline_time = currentTime;
            Serial.println("Baseline updated (cleaner air detected)");
          }
  
          // Fire detection logic
          float spike = mean - baseline_mean;
          
          if (spike > fire_threshold) {
            send_message("A", currentTime, String(mean));
          }
  
          lastPrintTime = currentTime;
        }
      } else {
        // During stabilization period, show raw readings
        if (currentTime - lastPrintTime >= printInterval) {
          Serial.print("Stabilizing... Raw: ");
          Serial.print(sensorValue);
          Serial.print(", Readings collected: ");
          Serial.println(readings_count);
          lastPrintTime = currentTime;
        }
      }
    }
    
  // Heating phase
  } else {
    
    if (currentTime - startTime >= 60000) {
      Serial.println("Heating -> Measuring");
      startTime = millis();
      isMeasuring = true;
      measuring_phase();
    } else {
      // Show heating progress
      if (currentTime - lastPrintTime >= printInterval) {
        Serial.print("Heating... Time remaining: ");
        Serial.print((60000 - (currentTime - startTime)) / 1000);
        Serial.println(" seconds");
        lastPrintTime = currentTime;
      }
    }
  }

  // Healthcheck every 5 minutes
  if (currentTime - lastHealthcheckTime >= healthcheckInterval) {
    send_message("H", currentTime, String(mean));
    lastHealthcheckTime = currentTime;
  }

}
