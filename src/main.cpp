#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_MLX90640.h>
#include <math.h>
#include <TeensyThreads.h>
#include <queue>
#include <SD.h>

#define INC_ADDRESS 0x68
#define ACC_CONF  0x20  //Page 91
#define GYR_CONF  0x21  //Page 93
#define CMD       0x7E  //Page 65

//  =================== CAN Variables ===============================================
const int ledPin = 13;
const int hallEffectPin = 41;
const int chipSelect = BUILTIN_SDCARD;
const double TIMER_MICROSECONDS = 150000;
<<<<<<< Updated upstream
FlexCAN_T4FD<CAN2, RX_SIZE_256, TX_SIZE_16> CANbus;    // For custom PCB IT'S CAN 2 !!!!!! DONT FUCKING CHANGE IT 
// FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;    // For old PCB IT'S CAN 3 !!!!!!!!!!!!!!
CAN_message_t msg1;
CAN_message_t msg2;
CAN_message_t msg3;
CAN_message_t msg4;
=======
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CANbus;    // For custom PCB IT'S CAN 2 !!!!!! DONT FUCKING CHANGE IT 
// FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;    // For old PCB IT'S CAN 3 !!!!!!!!!!!!!!
// CAN_message_t msg1;
// CAN_message_t msg2;
// CAN_message_t msg3;
// CAN_message_t msg4;
>>>>>>> Stashed changes

CAN_message_t FR_ACC;
CAN_message_t FR_GYR;
CAN_message_t FR_BRAKE_TEMP;
CAN_message_t FR_TIRE_TEMP;
CAN_message_t FR_ANALOG;

CAN_message_t FL_ACC;
CAN_message_t FL_GYR;
CAN_message_t FL_BRAKE_TEMP;
CAN_message_t FL_TIRE_TEMP;
CAN_message_t FL_ANALOG;

CAN_message_t RR_ACC;
CAN_message_t RR_GYR;
CAN_message_t RR_BRAKE_TEMP;
CAN_message_t RR_TIRE_TEMP;
CAN_message_t RR_ANALOG;

CAN_message_t RL_ACC;
CAN_message_t RL_GYR;
CAN_message_t RL_BRAKE_TEMP;
CAN_message_t RL_TIRE_TEMP;
CAN_message_t RL_ANALOG;

// =================== MLX Variables ================================================
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_MLX90640 mlx2;

// =================== BMI Variables ================================================
uint16_t gyr_x, gyr_y, gyr_z;
int16_t signed_gyr_x, signed_gyr_y, signed_gyr_z;
uint16_t acc_x, acc_y, acc_z;
int16_t signed_acc_x, signed_acc_y, signed_acc_z;


// =================== BMI323 Helper Function Definitions ==============================================
// Write data in 16 bits
void writeRegister16(uint16_t reg, uint16_t value) {
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(reg);
  //Low 
  Wire.write((uint16_t)value & 0xff);
  //High
  Wire.write((uint16_t)value >> 8);
  Wire.endTransmission();
}

// Read data in 16 bits
uint16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  int n = Wire.requestFrom(INC_ADDRESS, 4);  
  uint16_t data[20];
  int i = 0;
  while(Wire.available()){
    data[i] = Wire.read();
    i++;
  }  
  return (data[3]|data[2] << 8);
}

// Read all axis
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

  // Offset = 2 because the 2 first bytes are dummy (useless)
  int offset = 2;
  acc_x = (data[offset + 0]  | (uint16_t)data[offset + 1] << 8); //0x00
  acc_y = (data[offset + 2]  | (uint16_t)data[offset + 3] << 8); //0x02
  acc_z = (data[offset + 4]  | (uint16_t)data[offset + 5] << 8); //0x04
  gyr_x = (data[offset + 6]   | (uint16_t)data[offset + 7] << 8);  //0x06
  gyr_y = (data[offset + 8]   | (uint16_t)data[offset + 9] << 8);  //0x07
  gyr_z = (data[offset + 10]  | (uint16_t)data[offset + 11] << 8); //0x08
}

void softReset(){  
  writeRegister16(CMD, 0xDEAF);
  delay(50);    
}

int16_t twosComplementToNormal(uint16_t raw) {
    if (raw & (1 << 15)) {
        return -((~raw + 1) & 0xFFFF);
    } else {
        return raw;
    }
}

// =================== Hall Effect Variables ========================================
IntervalTimer halleffectTimer;
<<<<<<< Updated upstream
volatile  double last_rpm = 0;
=======
volatile int last_rpm = 0;
>>>>>>> Stashed changes
volatile int numOfRotations = 0;
volatile int lastReadHallEffect = 0;
std::deque<double> past10readins;
uint16_t gyr_x, gyr_y, gyr_z;
int16_t signed_gyr_x, signed_gyr_y, signed_gyr_z;

// =================== HE Helper Function Definitions ===============================
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
    uint8_t halleffectOutput = digitalRead(hallEffectPin);
    if (halleffectOutput == 0 && lastReadHallEffect == 1) {
      numOfRotations ++;
    }
    lastReadHallEffect = halleffectOutput;
  }
}

// =================== Setup Function ===============================================
uint16_t readRegister16(uint8_t reg);
void readAllAccel(); 
int16_t twosComplementToNormal(uint16_t raw); 

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(hallEffectPin, INPUT);
  threads.addThread(thread_func, 1);
  halleffectTimer.begin(calculateRPM, TIMER_MICROSECONDS);

  Serial.begin(115200);
<<<<<<< Updated upstream
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");\
    return;
  }
  Serial.println("card initialized.");
=======
>>>>>>> Stashed changes
  
  while (!Serial && millis() < 4000);
  Wire.begin();
  Wire.setClock(400000); 
  Wire1.begin();
  Wire1.setClock(400000); 
  CANbus.begin();
<<<<<<< Updated upstream
  CANbus.setBaudRate(500000); 
=======
  CANbus.setBaudRate(1000000); 
>>>>>>> Stashed changes
  mlx.begin();
  mlx2.begin();
  mlx2.setRefreshRate(MLX90640_2_HZ);
}

void loop() {
  FR_ANALOG.id = 0x10F00;
  FR_ACC.id = 0x10F01;
  FR_GYR.id = 0x10F02;
  FR_BRAKE_TEMP.id = 0x10F03;
  FR_TIRE_TEMP.id = 0x10F04;
  
  FL_ANALOG.id = 0x10F05;
  FL_ACC.id = 0x10F06;
<<<<<<< Updated upstream
  FR_GYR.id = 0x10F07;
=======
  FL_GYR.id = 0x10F07;
>>>>>>> Stashed changes
  FL_BRAKE_TEMP.id = 0x10F08;
  FL_TIRE_TEMP.id = 0x10F09;

  RR_ANALOG.id = 0x10F10;
  RR_ACC.id = 0x10F11;
  RR_GYR.id = 0x10F12;
  RR_BRAKE_TEMP.id = 0x10F13;
  RR_TIRE_TEMP.id = 0x10F14;

  RL_ANALOG.id = 0x10F15;
  RL_ACC.id = 0x10F16;
  RL_GYR.id = 0x10F17;
  RL_BRAKE_TEMP.id = 0x10F18;
  RL_TIRE_TEMP.id = 0x10F19;
  // =================== CAN Message Setup ===========================================

  Serial.println("\n Loop Running... \n");

  //====================== MLX90640 Logic ============================================
  float temp640[32*24];
  mlx2.getFrame(temp640);
  float sum = 0;
  for (uint8_t h=0; h<24; h++) {
    for (uint8_t w=0; w<32; w++) {
<<<<<<< Updated upstream
      float t = temp640[h*32 + w] - 273.0;
      Serial.print(t); Serial.print(" ");
=======
      float t = temp640[h*32 + w] - 263.0;
>>>>>>> Stashed changes
      sum += t;
    }
    printf("\n");
  }
  int16_t Tire_temp = sum / (32*24);
  Serial.print("Average: "); 
  Serial.println(Tire_temp);
  delay(500);

  //===================== MLX90614 Logic =============================================
  float tempO = mlx.readObjectTempC();
  int Brake_temp = tempO * 100;
<<<<<<< Updated upstream

  //====================== BMI323 Logic =============================================
  readRegister16(0x02);
    if(readRegister16(0x02) == 0x00) {
      //Read ChipID
      Serial.print("ChipID:");
      Serial.print(readRegister16(0x00));    
      readAllAccel();             

      signed_acc_x = twosComplementToNormal(acc_x)/262.1;
      signed_acc_y = twosComplementToNormal(acc_y)/262.1;
      signed_acc_z = twosComplementToNormal(acc_z)/262.1;
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

  //===================== SD Card Data Logging ======================================
  File dataFile = SD.open("BrakeTemp.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(sendObjVal);
    dataFile.close();
  }  
  else {
    Serial.println("error opening datalog.txt");
  }   

  File dataFile = SD.open("TireTemp.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(temp640); //Change this to correct thing for Multipixel Sensor
    dataFile.close();
  }  
  else {
    Serial.println("error opening datalog.txt");
  }   

  File dataFile = SD.open("IMU.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(signed_gyr_x);
    dataFile.println(signed_gyr_y); 
    dataFile.println(signed_gyr_z);
    dataFile.close();
  }  
  else {
    Serial.println("error opening datalog.txt");
  }   
  //===================== Data Push ==================================================
  FR_BTSH.buf[0] = Brake_temp >> 8;
  FR_BTSH.buf[1] = Brake_temp & 0xFF;
  FR_BTSH.buf[2] = Tire_temp >> 8;
  FR_BTSH.buf[3] = Tire_temp & 0xFF;
  FR_BTSH.buf[4] = Suspension >> 8;
  RL_BTSH.buf[5] = Suspension & 0xFF;
  RL_BTSH.buf[6] = last_rpm >> 8;
  RL_BTSH.buf[7] = last_rpm & 0xFF;
  
  FR_ACC.buf[0] = (signed_acc_x >> 8) & 0xFF;
  FR_ACC.buf[1] = signed_acc_x & 0xFF;
  FR_ACC.buf[2] = (signed_acc_y >> 8) & 0xFF;
  FR_ACC.buf[3] = signed_acc_y & 0xFF;
  FR_ACC.buf[4] = (signed_acc_z >> 8) & 0xFF;
  FR_ACC.buf[5] = signed_acc_z & 0xFF;
  FR_GYR.buf[0] = (signed_gyr_x >> 8) & 0xFF;
  FR_GYR.buf[1] = signed_gyr_x & 0xFF;
  FR_GYR.buf[2] = (signed_gyr_y >> 8) & 0xFF;
  FR_GYR.buf[3] = signed_gyr_y & 0xFF;
  FR_GYR.buf[4] = (signed_gyr_z >> 8) & 0xFF;
  FR_GYR.buf[5] = signed_gyr_z & 0xFF;

  bool FR_ACC_send = CANbusFD.write(FR_ACC);
  bool FR_GYR_send = CANbusFD.write(FR_GYR);
=======
  Serial.print("Brake Temp: ");
  Serial.println(tempO);

  //====================== BMI323 Logic =============================================
  readRegister16(0x02);
    // if(readRegister16(0x02) == 0x00) {
  readAllAccel();             
  signed_acc_x = twosComplementToNormal(acc_x)/16384.;
  signed_acc_y = twosComplementToNormal(acc_y)/16384.;
  signed_acc_z = twosComplementToNormal(acc_z)/16384.;
  signed_gyr_x = twosComplementToNormal(gyr_x)/262.1;
  signed_gyr_y = twosComplementToNormal(gyr_y)/262.1;
  signed_gyr_z = twosComplementToNormal(gyr_z)/262.1;
  Serial.print(" gyr_x:");
  Serial.print(signed_gyr_x);
  Serial.print(" \tgyr_y:");
  Serial.print(signed_gyr_y);
  Serial.print(" \tgyr_z:");
  Serial.println(signed_gyr_z);   
    // }

  //===================== SD Card Data Logging ======================================
  // File dataFile = SD.open("BrakeTemp.txt", FILE_WRITE);
  // if (dataFile) {
  //   dataFile.println(sendObjVal);
  //   dataFile.close();
  // }  
  // else {
  //   Serial.println("error opening datalog.txt");
  // }   

  // File dataFile = SD.open("TireTemp.txt", FILE_WRITE);
  // if (dataFile) {
  //   dataFile.println(temp640); //Change this to correct thing for Multipixel Sensor
  //   dataFile.close();
  // }  
  // else {
  //   Serial.println("error opening datalog.txt");
  // }   

  // File dataFile = SD.open("IMU.txt", FILE_WRITE);
  // if (dataFile) {
  //   dataFile.println(signed_gyr_x);
  //   dataFile.println(signed_gyr_y); 
  //   dataFile.println(signed_gyr_z);
  //   dataFile.close();
  // }  
  // else {
  //   Serial.println("error opening datalog.txt");
  // }   
  //===================== Data Push ==================================================
  // int Suspension = 0;

  // FR_BRAKE_TEMP.buf[0] = Brake_temp >> 8;
  // FR_BRAKE_TEMP.buf[1] = Brake_temp & 0xFF;
  // FR_TIRE_TEMP.buf[0] = Tire_temp >> 8;
  // FR_TIRE_TEMP.buf[1] = Tire_temp & 0xFF;
  // FR_ANALOG.buf[0] = Suspension & 0xFF;
  // FR_ANALOG.buf[1] = last_rpm >> 8;
  // FR_ANALOG.buf[2] = last_rpm & 0xFF;
  
  // FR_ACC.buf[0] = (signed_acc_x >> 8) & 0xFF;
  // FR_ACC.buf[1] = signed_acc_x & 0xFF;
  // FR_ACC.buf[2] = (signed_acc_y >> 8) & 0xFF;
  // FR_ACC.buf[3] = signed_acc_y & 0xFF;
  // FR_ACC.buf[4] = (signed_acc_z >> 8) & 0xFF;
  // FR_ACC.buf[5] = signed_acc_z & 0xFF;
  // FR_GYR.buf[0] = (signed_gyr_x >> 8) & 0xFF;
  // FR_GYR.buf[1] = signed_gyr_x & 0xFF;
  // FR_GYR.buf[2] = (signed_gyr_y >> 8) & 0xFF;
  // FR_GYR.buf[3] = signed_gyr_y & 0xFF;
  // FR_GYR.buf[4] = (signed_gyr_z >> 8) & 0xFF;
  // FR_GYR.buf[5] = signed_gyr_z & 0xFF;

  // FR_ACC.flags.extended = 1;
  // FR_GYR.flags.extended = 1;
  // FR_BRAKE_TEMP.flags.extended = 1;
  // FR_TIRE_TEMP.flags.extended = 1;
  // FR_ANALOG.flags.extended = 1;
  // bool FR_ACC_send = CANbus.write(FR_ACC);
  // bool FR_GYR_send = CANbus.write(FR_GYR);
  // bool writeResult = CANbus.write(FR_BRAKE_TEMP) & CANbus.write(FR_TIRE_TEMP) & CANbus.write(FR_ANALOG) & FR_ACC_send & FR_GYR_send;

  // ==============FL Module=============
  int Suspension = 0;

  FL_BRAKE_TEMP.buf[0] = Brake_temp >> 8;
  FL_BRAKE_TEMP.buf[1] = Brake_temp & 0xFF;
  FL_TIRE_TEMP.buf[0] = Tire_temp >> 8;
  FL_TIRE_TEMP.buf[1] = Tire_temp & 0xFF;
  FL_ANALOG.buf[0] = Suspension & 0xFF;
  FL_ANALOG.buf[1] = last_rpm >> 8;
  FL_ANALOG.buf[2] = last_rpm & 0xFF;
  
  FL_ACC.buf[0] = (signed_acc_x >> 8) & 0xFF;
  FL_ACC.buf[1] = signed_acc_x & 0xFF;
  FL_ACC.buf[2] = (signed_acc_y >> 8) & 0xFF;
  FL_ACC.buf[3] = signed_acc_y & 0xFF;
  FL_ACC.buf[4] = (signed_acc_z >> 8) & 0xFF;
  FL_ACC.buf[5] = signed_acc_z & 0xFF;
  FL_GYR.buf[0] = (signed_gyr_x >> 8) & 0xFF;
  FL_GYR.buf[1] = signed_gyr_x & 0xFF;
  FL_GYR.buf[2] = (signed_gyr_y >> 8) & 0xFF;
  FL_GYR.buf[3] = signed_gyr_y & 0xFF;
  FL_GYR.buf[4] = (signed_gyr_z >> 8) & 0xFF;
  FL_GYR.buf[5] = signed_gyr_z & 0xFF;

  FL_ACC.flags.extended = 1;
  FL_GYR.flags.extended = 1;
  FL_BRAKE_TEMP.flags.extended = 1;
  FL_TIRE_TEMP.flags.extended = 1;
  FL_ANALOG.flags.extended = 1;
  bool FL_ACC_send = CANbus.write(FL_ACC);
  bool FL_GYR_send = CANbus.write(FL_GYR);
  bool writeResult = CANbus.write(FL_BRAKE_TEMP) & CANbus.write(FL_TIRE_TEMP) & CANbus.write(FL_ANALOG) & FL_ACC_send & FL_GYR_send;

  // ==============RL Module=============
  // int Suspension = 0;

  // RL_BRAKE_TEMP.buf[0] = Brake_temp >> 8;
  // RL_BRAKE_TEMP.buf[1] = Brake_temp & 0xFF;
  // RL_TIRE_TEMP.buf[0] = Tire_temp >> 8;
  // RL_TIRE_TEMP.buf[1] = Tire_temp & 0xFF;
  // RL_ANALOG.buf[0] = Suspension & 0xFF;
  // RL_ANALOG.buf[1] = last_rpm >> 8;
  // RL_ANALOG.buf[2] = last_rpm & 0xFF;
  
  // RL_ACC.buf[0] = (signed_acc_x >> 8) & 0xFF;
  // RL_ACC.buf[1] = signed_acc_x & 0xFF;
  // RL_ACC.buf[2] = (signed_acc_y >> 8) & 0xFF;
  // RL_ACC.buf[3] = signed_acc_y & 0xFF;
  // RL_ACC.buf[4] = (signed_acc_z >> 8) & 0xFF;
  // RL_ACC.buf[5] = signed_acc_z & 0xFF;

  // RL_GYR.buf[0] = (signed_gyr_x >> 8) & 0xFF;
  // RL_GYR.buf[1] = signed_gyr_x & 0xFF;
  // RL_GYR.buf[2] = (signed_gyr_y >> 8) & 0xFF;
  // RL_GYR.buf[3] = signed_gyr_y & 0xFF;
  // RL_GYR.buf[4] = (signed_gyr_z >> 8) & 0xFF;
  // RL_GYR.buf[5] = signed_gyr_z & 0xFF;

  // RL_ACC.flags.extended = 1;
  // RL_GYR.flags.extended = 1;
  // RL_BRAKE_TEMP.flags.extended = 1;
  // RL_TIRE_TEMP.flags.extended = 1;
  // RL_ANALOG.flags.extended = 1;

  // bool RL_ACC_send = CANbus.write(RL_ACC);
  // bool RL_GYR_send = CANbus.write(RL_GYR);
  // bool writeResult = CANbus.write(RL_BRAKE_TEMP) & CANbus.write(RL_TIRE_TEMP) & CANbus.write(RL_ANALOG) & RL_ACC_send & RL_GYR_send;

  // ==============RR Module=============
  // int Suspension = 0;

  // RR_BRAKE_TEMP.buf[0] = Brake_temp >> 8;
  // RR_BRAKE_TEMP.buf[1] = Brake_temp & 0xFF;
  // RR_TIRE_TEMP.buf[2] = Tire_temp >> 8;
  // RR_TIRE_TEMP.buf[3] = Tire_temp & 0xFF;
  // // RR_ANALOG.buf[0] = Suspension >> 8;
  // RR_ANALOG.buf[0] = Suspension & 0xFF;
  // RR_ANALOG.buf[1] = last_rpm >> 8;
  // RR_ANALOG.buf[2] = last_rpm & 0xFF;
  
  // RR_ACC.buf[0] = (signed_acc_x >> 8) & 0xFF;
  // RR_ACC.buf[1] = signed_acc_x & 0xFF;
  // RR_ACC.buf[2] = (signed_acc_y >> 8) & 0xFF;
  // RR_ACC.buf[3] = signed_acc_y & 0xFF;
  // RR_ACC.buf[4] = (signed_acc_z >> 8) & 0xFF;
  // RR_ACC.buf[5] = signed_acc_z & 0xFF;
  // RR_GYR.buf[0] = (signed_gyr_x >> 8) & 0xFF;
  // RR_GYR.buf[1] = signed_gyr_x & 0xFF;
  // RR_GYR.buf[2] = (signed_gyr_y >> 8) & 0xFF;
  // RR_GYR.buf[3] = signed_gyr_y & 0xFF;
  // RR_GYR.buf[4] = (signed_gyr_z >> 8) & 0xFF;
  // RR_GYR.buf[5] = signed_gyr_z & 0xFF;

  // RR_ACC.flags.extended = 1;
  // RR_GYR.flags.extended = 1;
  // RR_BRAKE_TEMP.flags.extended = 1;
  // RR_TIRE_TEMP.flags.extended = 1;
  // RR_ANALOG.flags.extended = 1;

  // bool RR_ACC_send = CANbus.write(RR_ACC);
  // bool RR_GYR_send = CANbus.write(RR_GYR);
  // bool writeResult = CANbus.write(RR_BRAKE_TEMP) & CANbus.write(RR_TIRE_TEMP) & CANbus.write(RR_ANALOG) & RR_ACC_send & RR_GYR_send;
>>>>>>> Stashed changes

  Serial.print("Writing Message: ");
  Serial.println(writeResult);  // Print the result of the write operation

<<<<<<< Updated upstream
}
=======
}
>>>>>>> Stashed changes
