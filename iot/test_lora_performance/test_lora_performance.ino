#include <SoftwareSerial.h>
#define UART_RX 10
#define UART_TX 11
SoftwareSerial es920(UART_RX, UART_TX);

// Healthcheck timing
unsigned long lastHealthcheckTime = 0;
const unsigned long healthcheckInterval = 120000; // 20 seconds

void setup() {
  // Hardware serial (via USB)
  Serial.begin(9600);
  Serial.println("-- Healthcheck Program Started --");
  
  // Software serial (via digital pins)
  es920.begin(9600);
  
  es920.print("A");
  es920.print("-");
  es920.print("002");
  es920.print("-");
  es920.print("0000000");
  es920.print("-");
  es920.print("208.80");
  es920.println("#");
}

void loop() {

}
