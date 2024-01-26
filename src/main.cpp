#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MLX90614.h>
#include <math.h>

#define INC_ADDRESS 0x69
#define ACC_CONF  0x20  //Page 91
#define GYR_CONF  0x21  //Page 93
#define CMD       0x7E  //Page 65

const int ledPin = 13;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;  // CAN0 is the CAN module to use
CAN_message_t msg;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
uint16_t gyr_x, gyr_y, gyr_z;

//Write data in 16 bits
void writeRegister16(uint16_t reg, uint16_t value) {
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(reg);
  //Low 
  Wire.write((uint16_t)value & 0xff);
  //High
  Wire.write((uint16_t)value >> 8);
  Wire.endTransmission();
}

//Read data in 16 bits
uint16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  int n = Wire.requestFrom(INC_ADDRESS, 4);  
  uint16_t data[4];
  int i =0;
  while(Wire.available()){
    data[i] = Wire.read();
    i++;
  }  
  return (data[3]   | data[2] << 8);
}

//Read all axis
void readAllAccel() {
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(INC_ADDRESS, 6);
  short data[6];
  int i =0;
  while(Wire.available()){
    data[i] = Wire.read();
    i++;
  }

  //Offset = 2 because the 2 first bytes are dummy (useless)
  int offset = 2;  
  gyr_x =         (data[offset + 6]   | (uint16_t)data[offset + 7] << 8);  //0x06
  gyr_y =         (data[offset + 8]   | (uint16_t)data[offset + 9] << 8);  //0x07
  gyr_z =         (data[offset + 10]  | (uint16_t)data[offset + 11] << 8); //0x08
}

void softReset(){  
    writeRegister16(CMD, 0xDEAF);
    delay(50);    
  }

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  while (!Serial && millis() < 4000);
  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz
  CANbus.begin();
  CANbus.setBaudRate(500000); 
  mlx.begin();
  softReset();
  writeRegister16(ACC_CONF, 0x708C); 
  writeRegister16(GYR_CONF, 0x708B);
}

void loop() {
  // =================== BMI323 Logic ======================================================
  
  readAllAccel();
  Serial.print("X: ");
  Serial.println(gyr_x);
  Serial.print("Y: ");
  Serial.println(gyr_y);
  Serial.print("Z: ");
  Serial.println(gyr_z);
  // =======================================================================================
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