#include <SoftwareSerial.h>

#define UART_RX 10
#define UART_TX 11

SoftwareSerial es920(UART_RX, UART_TX);

// PWM related
const int PWM_PIN = 3;
float duty_value = 71.4;

// Measuring/Heating phases management
unsigned long startTime;
bool isMeasuring = false;

// Store values in array for sliding mean
const int array_size = 10;
int sensor_prev_values[array_size];
int array_index = 0;

// Trick to avoid too much printed values
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 1000;

// Minimum mean every 10 seconds
float mean = 0;
float baseline_mean = 0;
unsigned long baseline_time = 0;
const unsigned long baseline_update_interval = 10000;

// 5V - 60 seconds
void heating_phase() {
  digitalWrite(PWM_PIN, HIGH);
}

// 1.4V - 90 seconds
void measuring_phase() {
  analogWrite(PWM_PIN, duty_value);
}

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(A0, INPUT);
  // pinMode(A1, INPUT); (Not used yet, for PWM confirmation)

  // Hardware serial (via USB)
  Serial.begin(9600);
  Serial.println("-- Init --");

  // Software serial (via digital pins)
  es920.begin(9600);

  heating_phase();
  startTime = millis();
}

void loop() {
  unsigned long currentTime = millis();

  // Measuring phase
  if (isMeasuring) {
    
    if (currentTime - startTime >= 90000) {
      Serial.println("Measuring -> Heating");
      startTime = millis();
      isMeasuring = false;
      heating_phase();
      
    } else {
      
      // Reading sensor
      int sensorValue = analogRead(A0);
      sensor_prev_values[array_index] = sensorValue;
      array_index = (array_index + 1) % array_size;

      if (currentTime - startTime >= 5000) { // Wait 5 sec to get correct readings when changing to measuring phase
        
        // Sliding mean
        mean = 0;
        for (int i = 0; i < array_size; i++) {
          mean += sensor_prev_values[i];
        }
        mean = mean / array_size;
  
        // Show mean only if 1 second have passed
        if (currentTime - lastPrintTime >= printInterval) {
          Serial.print("Mean: ");
          Serial.println(mean);
  
          // Update baseline periodically or when mean is decreasing
          if (currentTime - baseline_time >= baseline_update_interval || mean < baseline_mean) {
            baseline_mean = mean;
            baseline_time = currentTime;
          }
  
          // Test: write alert message if C02 spike detected
            if(mean - baseline_mean > 2.0) {
              Serial.println("Alert: C02 concentration spike detected");
              es920.println("Alert: C02 concentration spike detected");
            }
  
          // Reset values for next print
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
    }
    
  }
}
