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
  msg.id = 0x123; // CAN message ID
  msg.len = 8;    // Message length (up to 8 bytes)
  msg.buf[0] = 0x11;
  msg.buf[1] = 0x22;
  msg.buf[2] = 0x33;
  msg.buf[3] = 0x44;
  msg.buf[4] = 0x55;
  msg.buf[5] = 0x66;
  msg.buf[6] = 0x77;
  msg.buf[7] = 0x88;
  
  digitalWrite(ledPin, HIGH);  // LED on
  CANbus.write(msg);
  delay(1000);
  digitalWrite(ledPin, LOW);   // LED off
  delay(1000);
  
  if (CANbus.read(msg)) {
    Serial.print("Received message with ID: ");
    Serial.println(msg.id, HEX);
    Serial.print("Message contents: ");
    for (int i = 0; i < msg.len; i++) {
      Serial.print(msg.buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}