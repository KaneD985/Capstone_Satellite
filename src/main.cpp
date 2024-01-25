#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MLX90614.h>
#include <math.h>
// #include <BMI323-Sensor-API/bmi323.h>

const int ledPin = 13;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;  // CAN0 is the CAN module to use
CAN_message_t msg1;
Adafruit_MLX90614 mlx1 = Adafruit_MLX90614(0x5A);
Adafruit_MLX90614 mlx2 = Adafruit_MLX90614(0x5B);

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  while (!Serial && millis() < 4000);
  CANbus.begin();
  CANbus.setBaudRate(500000); 
  mlx1.begin(0x5A, &Wire);
  mlx2.begin(0X5B, &Wire1);
}

void loop() {
  // =================== BMI323 Logic ======================================================

  // =======================================================================================
  Serial.println("\n Loop Running... \n");
  msg1.id = 0x124; // CAN message ID
  msg1.len = 8;    // Message length (up to 8 bytes)
  
  float tempO1 = mlx1.readObjectTempC();
  //Serial.print("Object = "); Serial.print(tempC); Serial.println("*C");
  float tempA1 = mlx1.readAmbientTempC();

  int sendObjVal = tempO1 * 100;
  msg1.buf[0] = sendObjVal >> 8;
  msg1.buf[1] = sendObjVal & 0xFF;

  int sendAmbVal = tempA1 * 100;
  msg1.buf[2] = sendAmbVal >> 8;
  msg1.buf[3] = sendAmbVal & 0xFF;

  // bool writeResult = CANbus.write(msg1);  // Send the message
  // Serial.print("Writing Message: ");
  // Serial.println(writeResult);  // Print the result of the write operation

  // Additional MLX90614 sensor
  float tempO2 = mlx2.readObjectTempC();
  float tempA2 = mlx2.readAmbientTempC();

  int sendObjVal2 = tempO2 * 100;
  msg1.buf[4] = sendObjVal2 >> 8;
  msg1.buf[5] = sendObjVal2 & 0xFF;

  int sendAmbVal2 = tempA2 * 100;
  msg1.buf[6] = sendAmbVal2 >> 8;
  msg1.buf[7] = sendAmbVal2 & 0xFF;

  bool writeResult1 = CANbus.write(msg1);  // Send the message
  Serial.print("Writing Message: ");
  Serial.println(writeResult1);  // Print the result of the write operation

  delay(1000);  // Wait for a while before trying to read

  // Attempt to read a message and print if successful
  bool readResult = CANbus.read(msg1);
  Serial.print("Reading Message: ");
  Serial.println(readResult);

  if (readResult) {
    
    Serial.print("Received message with ID: ");
    Serial.println(msg1.id, HEX);
    Serial.print("Message contents: ");
    Serial.println();

    float temp_object = ((msg1.buf[0] << 8) | msg1.buf[1]) / 100.;
    Serial.print("Temp of Sensor 1: ");
    Serial.println(temp_object);
    float temp_ambient = ((msg1.buf[2] << 8) | msg1.buf[3]) / 100.;
    Serial.print("Ambient of Sensor 1: ");
    Serial.println(temp_ambient);

    float temp_object2 = ((msg1.buf[4] << 8) | msg1.buf[5]) / 100.;
    Serial.print("Temp of Sensor 2: ");
    Serial.println(temp_object2);
    float temp_ambient2 = ((msg1.buf[6] << 8) | msg1.buf[7]) / 100.;
    Serial.print("Ambient of Sensor 2: ");
    Serial.println(temp_ambient2);

    digitalWrite(ledPin, LOW);
  }
}