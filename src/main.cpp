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
  msg.len = 8;    // Message length (up to 8 bytes)
  
  float tempO = mlx.readObjectTempC();
  //Serial.print("Object = "); Serial.print(tempC); Serial.println("*C");
  float tempA = mlx.readAmbientTempC();

  int sendObjVal = tempO * 100;
  msg.buf[0] = sendObjVal >> 8;
  msg.buf[1] = sendObjVal & 0xFF;

  int sendAmbVal = tempA * 100;
  msg.buf[2] = sendAmbVal >> 8;
  msg.buf[3] = sendAmbVal & 0xFF;

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
    // for (int i = 0; i < msg.len; i++) {
    //   Serial.print(msg.buf[i], HEX);
    //   Serial.print(" ");
    // }
    Serial.println();
    float temp_object = ((msg.buf[0] << 8) | msg.buf[1]) / 100.;
    Serial.print("Temp: ");
    Serial.println(temp_object);

    float temp_ambient = ((msg.buf[2] << 8) | msg.buf[3]) / 100.;
    Serial.print("Ambient: ");
    Serial.println(temp_ambient);
    digitalWrite(ledPin, LOW);
  }
}