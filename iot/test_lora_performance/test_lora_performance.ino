#include <SoftwareSerial.h>
#define UART_RX 10
#define UART_TX 11
SoftwareSerial es920(UART_RX, UART_TX);

// Healthcheck timing
unsigned long lastHealthcheckTime = 0;
const unsigned long healthcheckInterval = 5000; // 5 seconds

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
    es920.print(currentTime);
    es920.println(" : HEALTHCHECK_OK");
    Serial.print(currentTime);
    Serial.println(": Healthcheck sent to ES920");
    lastHealthcheckTime = currentTime;
  }
}
