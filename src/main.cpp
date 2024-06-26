#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_MLX90640.h>
#include <math.h>
#include <TeensyThreads.h>
#include <queue>

#define INC_ADDRESS 0x68
#define ACC_CONF  0x20  //Page 91
#define GYR_CONF  0x21  //Page 93
#define CMD       0x7E  //Page 65

const int ledPin = 13;
const int hallEffectPin = 41;
const double TIMER_MICROSECONDS = 150000;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;  // CAN0 is the CAN module to use
CAN_message_t msg;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_MLX90640 mlx2;
uint16_t gyr_x, gyr_y, gyr_z;
int16_t signed_gyr_x, signed_gyr_y, signed_gyr_z;
IntervalTimer halleffectTimer;
volatile  double last_rpm = 0;
volatile int numOfRotations = 0;
volatile int lastReadHallEffect = 0;
std::deque<double> past10readins;

//Convert Raw Data in 2's Complement to Normal
int16_t twosComplementToNormal(uint16_t raw) {
    if (raw & (1 << 15)) {
        return -((~raw + 1) & 0xFFFF);
    } else {
        return raw;
    }
}

///Write data in 16 bits
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
  uint16_t data[20];
  int i =0;
  while(Wire.available()){
    data[i] = Wire.read();
    i++;
  }  
  return (data[3]|data[2] << 8);
}

//Read all axis
void readAllAccel() {
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(INC_ADDRESS, 20);
  uint16_t data[20];
  int i =0;
  while(Wire.available()){
    data[i] = Wire.read();
    i++;
  }

  //Offset = 2 because the 2 first bytes are dummy (useless)
  int offset = 2;
  gyr_x = (data[offset + 6]   | (uint16_t)data[offset + 7] << 8);  //0x06
  gyr_y = (data[offset + 8]   | (uint16_t)data[offset + 9] << 8);  //0x07
  gyr_z = (data[offset + 10]  | (uint16_t)data[offset + 11] << 8); //0x08
}

void softReset(){  
  writeRegister16(CMD, 0xDEAF);
  delay(50);    
}

double calculateAVGRPM() {
  double newRPM = 0;
  int n = past10readins.size();
  for (int i = 0; i < n - 10; i++) {
    past10readins.pop_front();
  }
  for (int i = 0; i < 10; i++) {
    newRPM += past10readins[i];
  }
  return newRPM / 10.00;
}

void calculateRPM() {
  double rpm = numOfRotations / (TIMER_MICROSECONDS / 1000000.0);
  numOfRotations = 0;
  past10readins.push_back(rpm);
  last_rpm = calculateAVGRPM();    
}

void thread_func() {
  while(true) {
    uint8_t halleffectOuput = digitalRead(hallEffectPin);
    if (halleffectOuput == 0 && lastReadHallEffect == 1) {
      numOfRotations ++;
    }
    lastReadHallEffect = halleffectOutput;
  }
}

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(hallEffectPin, INPUT);
  threads.addThread(thread_func, 1);
  halleffectTimer.begin(calculateRPM, TIMER_MICROSECONDS);
  Serial.begin(115200);
  while (!Serial && millis() < 4000);
  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz
  Wire1.begin();
  Wire1.setClock(400000); //Increase I2C clock speed to 1MHz
  CANbus.begin();
  CANbus.setBaudRate(500000); 
  mlx.begin();
  mlx2.begin();
  mlx2.setRefreshRate(MLX90640_2_HZ);
  softReset();
  writeRegister16(ACC_CONF, 0x708B); 
  writeRegister16(GYR_CONF, 0x708B);
}

void loop() {
  // =================== CAN Message Setup =================================================

  msg.id = 0x124; // CAN message ID
  msg.len = 8;    // Message length (up to 8 bytes)
  Serial.println("\n Loop Running... \n");

  // =================== BMI323 Logic ======================================================
  readRegister16(0x02);
  if(readRegister16(0x02) == 0x00) {
    //Read ChipID
    Serial.print("ChipID:");
    Serial.print(readRegister16(0x00));    
    readAllAccel();             // read all accelerometer/gyroscope/temperature data  
    printf("gyr_x: %d\n", gyr_x);
    printf("gyr_y: %d\n", gyr_y);
    printf("gyr_z: %d\n", gyr_z);
    signed_gyr_x = twosComplementToNormal(gyr_x)/262.1;
    signed_gyr_y = twosComplementToNormal(gyr_y)/262.1;
    signed_gyr_z = twosComplementToNormal(gyr_z)/262.1;
    Serial.print(" \tgyr_x:");
    Serial.print(signed_gyr_x);
    Serial.print(" \tgyr_y:");
    Serial.print(signed_gyr_y);
    Serial.print(" \tgyr_z:");
    Serial.println(signed_gyr_z);   
  }

  // =======================================================================================

  //====================== MLX90640 Logic ==================================================
  float temp640[32*24];
  mlx2.getFrame(temp640);
  float sum = 0;
  for (uint8_t h=0; h<24; h++) {
    for (uint8_t w=0; w<32; w++) {
      float t = temp640[h*32 + w] - 273.0;
      Serial.print(t); Serial.print(" ");
      sum += t;
    }
    printf("\n");
  }
  
  float average = sum / (32*24);
  Serial.print("Average: "); 
  Serial.println(average);

  // delay(500);
  //=========================================================================================

  float tempO = mlx.readObjectTempC();
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

  delay(500);  // Wait for a while before trying to read

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
