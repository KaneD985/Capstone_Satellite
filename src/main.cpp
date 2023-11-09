#include <Arduino.h>
#include <FlexCAN_T4.h>

const int ledPin = 13;
FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> CANbus;  // CAN0 is the CAN module to use

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  while (!Serial && millis() < 4000);
  CANbus.begin();
  CANbus.setBaudRate(500000);  // Set the CAN bus speed to 500 kbps (adjust as needed)
}

void loop() {
  CAN_message_t msg;
  msg.id = 0x123; // CAN message ID
  msg.len = 8;    // Message length (up to 8 bytes)
  
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i] = random(0, 256);  // Generate a random number between 0 and 255
  }

  // Check if the last byte of the message is even
  if (msg.buf[msg.len - 1] % 2 == 0) {
    digitalWrite(ledPin, HIGH);  // LED on
  } else {
    digitalWrite(ledPin, LOW);   // LED off
  }

  CANbus.write(msg);
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