#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MLX90614.h>



const int ledPin = 13;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;  // CAN0 is the CAN module to use
CAN_message_t msg;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  while (!Serial && millis() < 4000);
  CANbus.begin();
  CANbus.setBaudRate(500000); 
  mlx.begin();
}



void loop() {
  Serial.println("\n Loop Running... \n");
  msg.id = 0x124; // CAN message ID
  msg.len = 1;    // Message length (up to 8 bytes)
  

  float tempC = mlx.readAmbientTempC();
  memcpy(msg.buf, &tempC, sizeof(tempC));


  bool writeResult = CANbus.write(msg);  // Send the message
  Serial.print("Writing Message: ");
  Serial.println(writeResult);  // Print the result of the write operation

  delay(1000);  // Wait for a while before trying to read

  // Attempt to read a message and print if successful
  bool readResult = CANbus.read(msg);
  Serial.print("Reading Message: ");
  Serial.println(readResult);

  if (readResult) {
    digitalWrite(ledPin, HIGH);
    Serial.print("Received message with ID: ");
    Serial.println(msg.id, HEX);
    Serial.print("Message contents: ");
    for (int i = 0; i < msg.len; i++) {
      Serial.print(msg.buf[i], DEC);
      Serial.print(" ");
    }
    Serial.println();
    digitalWrite(ledPin, LOW);
  }
}#include <Arduino.h>
#include <FlexCAN_T4.h>

const int ledPin = 13;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;  // CAN0 is the CAN module to use

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  while (!Serial && millis() < 4000);
  CANbus.begin();
  CANbus.setBaudRate(500000);  // Set the CAN bus speed to 500 kbps (adjust as needed)
}

void loop() {
  CAN_message_t msg;
  msg.id = 0x124; // CAN message ID
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