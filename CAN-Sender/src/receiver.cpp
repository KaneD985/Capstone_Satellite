#include <Arduino.h>
#include <FlexCAN_T4.h>

const int ledPin = 13;
FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> CANbus;  // CAN0 is the CAN module to use

void setup() {
  pinMode(ledPin, OUTPUT);
  while (!Serial && millis() < 4000);
  CANbus.begin();
  CANbus.setBaudRate(500000);  // Set the CAN bus speed to 500 kbps (adjust as needed)
}

void loop() {
    CAN_message_t msg;
    if (CANbus.read(msg)) {
        if (msg.id == 0x123) { // Check if the received message ID matches the expected ID
            digitalWrite(ledPin, HIGH);  // LED on
            Serial.print("Received message: ");
            for (int i = 0; i < msg.len; i++) {
                Serial.print(msg.buf[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
            digitalWrite(ledPin, LOW);   // LED off
        }
    }
}