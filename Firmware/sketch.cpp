// Your program. Rename/edit freely - it's the Arduino setup()/loop() model.
#include "orion.h"

#define LED PA0            // no LED on the board: wire an LED + resistor to header pin PA0

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(115200);    // appears as a COM port / /dev/ttyACM0 on the PC
}

void loop() {
  static uint32_t last;
  if (millis() - last >= 500) {
    last = millis();
    digitalWrite(LED, !digitalRead(LED));
    Serial.print("uptime ms: ");
    Serial.println(millis());
  }
  while (Serial.available()) {          // echo anything you type back
    Serial.write((uint8_t)Serial.read());
  }
}
