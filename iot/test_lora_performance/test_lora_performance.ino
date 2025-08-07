#include <SoftwareSerial.h>
#define UART_RX 10
#define UART_TX 11
SoftwareSerial es920(UART_RX, UART_TX);

// Healthcheck timing
unsigned long lastHealthcheckTime = 0;
const unsigned long healthcheckInterval = 20000; // 20 seconds

void setup() {
  // Hardware serial (via USB)
  Serial.begin(9600);
  Serial.println("-- Healthcheck Program Started --");
  
  // Software serial (via digital pins)
  es920.begin(9600);
  
  lastHealthcheckTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Send healthcheck message every 5 seconds
  if (currentTime - lastHealthcheckTime >= healthcheckInterval) {
    es920.print("A");
    es920.print("-");
    es920.print("005");
    es920.print("-");
    es920.print(currentTime);
    es920.print("-");
    es920.print("208.80");
    es920.println("#");
    
    Serial.print(currentTime);
    Serial.println(": Alert sent to ES920");
    
    lastHealthcheckTime = currentTime;
  }
}
