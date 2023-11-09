#include <Arduino.h>
#include <FlexCAN_T4.h>

const int ledPin = 13;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;  // CAN0 is the CAN module to use
CAN_message_t msg;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  while (!Serial && millis() < 4000);
  CANbus.begin();
  CANbus.setBaudRate(500000);  // Set the CAN bus speed to 500 kbps (adjust as needed)
}



void loop() {
  Serial.println("\n Loop Running... \n");
  msg.id = 0x123; // CAN message ID
  msg.len = 8;    // Message length (up to 8 bytes)
  

  for (int i = 0; i < msg.len; i++) {
    msg.buf[0] = 0x00;
    msg.buf[1] = 0x11;
    msg.buf[2] = 0x22;
    msg.buf[3] = 0x33;
    msg.buf[4] = 0x44;
    msg.buf[5] = 0x55;
    msg.buf[6] = 0x66;
    msg.buf[7] = random(0, 255);
  }

  // Check if the last byte of the message is even
  if (msg.buf[msg.len - 1] % 2 == 0) {
    digitalWrite(ledPin, HIGH);
    CANbus.write(msg);  // LED on
  } else {
    digitalWrite(ledPin, LOW);   // LED off
  }

  bool writeResult = CANbus.write(msg);  // Send the message
  Serial.print("Writing Message: ");
  Serial.println(writeResult);  // Print the result of the write operation

  delay(1000);  // Wait for a while before trying to read

  // Attempt to read a message and print if successful
  bool readResult = CANbus.read(msg);
  Serial.print("Reading Message: ");
  Serial.println(readResult);

  if (readResult) {
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