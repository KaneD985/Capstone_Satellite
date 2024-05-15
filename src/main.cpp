#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_MLX90640.h>
#include <math.h>
#include <TeensyThreads.h>
#include <queue>
// #include <SD.h>

#define INC_ADDRESS 0x68
#define ACC_CONF 0x20 // Page 91
#define GYR_CONF 0x21 // Page 93
#define CMD 0x7E      // Page 65

//  =================== CAN Variables ===============================================
const int ledPin = 13;
const int hallEffectPin = 41;
// const int chipSelect = BUILTIN_SDCARD;
const double TIMER_MICROSECONDS = 150000;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CANbus; // For custom PCB IT'S CAN 2 !!!!!! DONT FUCKING CHANGE IT
// FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;    // For old PCB IT'S CAN 3 !!!!!!!!!!!!!!

CAN_message_t FL_BTSH; // BTSH stands for Brake_temp, Tire_temp, Suspension, Hall_Effect
CAN_message_t FL_ACC;
CAN_message_t FL_GYR;

// CAN_message_t FR_BTSH;
// CAN_message_t FR_ACC;
// CAN_message_t FR_GYR;

// CAN_message_t RL_BTSH;
// CAN_message_t RL_ACC;
// CAN_message_t RL_GYR;

// CAN_message_t RR_BTSH;
// CAN_message_t RR_ACC;
// CAN_message_t RR_GYR;

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
void writeRegister16(uint16_t reg, uint16_t value){
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(reg);
  // Low
  Wire.write((uint16_t)value & 0xff);
  // High
  Wire.write((uint16_t)value >> 8);
  Wire.endTransmission();
}

// Read data in 16 bits
uint16_t readRegister16(uint8_t reg){
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  int n = Wire.requestFrom(INC_ADDRESS, 4);
  uint16_t data[20];
  int i = 0;
  while (Wire.available())
  {
    data[i] = Wire.read();
    i++;
  }
  return (data[3] | data[2] << 8);
}

// Read all axis
void readAllAccel(){
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(INC_ADDRESS, 20);
  uint16_t data[20];
  int i = 0;
  while (Wire.available())
  {
    data[i] = Wire.read();
    i++;
  }

  // Offset = 2 because the 2 first bytes are dummy (useless)
  int offset = 2;
  acc_x = (data[offset + 0] | (uint16_t)data[offset + 1] << 8);   // 0x00
  acc_y = (data[offset + 2] | (uint16_t)data[offset + 3] << 8);   // 0x02
  acc_z = (data[offset + 4] | (uint16_t)data[offset + 5] << 8);   // 0x04
  gyr_x = (data[offset + 6] | (uint16_t)data[offset + 7] << 8);   // 0x06
  gyr_y = (data[offset + 8] | (uint16_t)data[offset + 9] << 8);   // 0x07
  gyr_z = (data[offset + 10] | (uint16_t)data[offset + 11] << 8); // 0x08
}

void softReset()
{
  writeRegister16(CMD, 0xDEAF);
  delay(50);
}

int16_t twosComplementToNormal(uint16_t raw){
  if (raw & (1 << 15))
  {
    return -((~raw + 1) & 0xFFFF);
  }
  else
  {
    return raw;
  }
}

// =================== Hall Effect Variables ========================================
IntervalTimer halleffectTimer;
volatile int16_t last_rpm = 0;
volatile int numOfRotations = 0;
volatile int lastReadHallEffect = 0;
std::deque<double> past10readins;

// =================== HE Helper Function Definitions ===============================
double calculateAVGRPM(){
  double newRPM = 0;
  int n = past10readins.size();
  for (int i = 0; i < n - 10; i++)
  {
    past10readins.pop_front();
  }
  for (int i = 0; i < 10; i++)
  {
    newRPM += past10readins[i];
  }
  return newRPM / 10.00;
}

void calculateRPM(){
  double rpm = numOfRotations / (TIMER_MICROSECONDS / 1000000.0);
  numOfRotations = 0;
  past10readins.push_back(rpm);
  last_rpm = calculateAVGRPM();
}

void thread_func(){
  while (true)
  {
    uint8_t halleffectOutput = digitalRead(hallEffectPin);
    if (halleffectOutput == 0 && lastReadHallEffect == 1)
    {
      numOfRotations++;
    }
    lastReadHallEffect = halleffectOutput;
  }
}

// =================== Setup Function ===============================================
uint16_t readRegister16(uint8_t reg);
void readAllAccel();
int16_t twosComplementToNormal(uint16_t raw);

void setup(){
  pinMode(ledPin, OUTPUT);
  pinMode(hallEffectPin, INPUT);
  threads.addThread(thread_func, 1);
  halleffectTimer.begin(calculateRPM, TIMER_MICROSECONDS);

  Serial.begin(115200);

  // Serial.print("Initializing SD card...");
  // if (!SD.begin(chipSelect))
  // {
  //   Serial.println("Card failed, or not present");
  //   return;
  // }
  // Serial.println("card initialized.");

  while (!Serial && millis() < 4000);
  Wire.begin();
  Wire.setClock(400000);
  softReset();
  writeRegister16(ACC_CONF,0x708C);
  writeRegister16(GYR_CONF,0x708B);
  CANbus.begin();
  CANbus.setBaudRate(500000);
  mlx.begin();
  mlx2.begin();
  mlx2.setRefreshRate(MLX90640_2_HZ);
}

void loop(){
  // =================== CAN Message Setup ===========================================
  FL_BTSH.id = 0x100;
  FL_BTSH.len = 8;
  FL_ACC.id = 0x101;
  FL_ACC.len = 8;
  FL_GYR.id = 0x102;
  FL_GYR.len = 8;

  // FR_BTSH.id = 0x200;
  // FR_BTSH.len = 8;
  // FR_ACC.id = 0x201;
  // FR_ACC.len = 8;
  // FR_GYR.id = 0x202;
  // FR_GYR.len = 8;

  // RL_BTSH.id = 0x300;
  // RL_BTSH.len = 8;
  // RL_ACC.id = 0x301;
  // RL_ACC.len = 8;
  // RL_GYR.id = 0x302;
  // RL_GYR.len = 8;

  // RR_BTSH.id = 0x400;
  // RR_BTSH.len = 8;
  // RR_ACC.id = 0x401;
  // RR_ACC.len = 8;
  // RR_GYR.id = 0x402;
  // RR_GYR.len = 8;

  Serial.println("\n Loop Running... \n");

  //====================== MLX90640 Logic ============================================
  float temp640[32 * 24];
  mlx2.getFrame(temp640);
  float sum = 0;
  for (uint8_t h = 0; h < 24; h++)
  {
    for (uint8_t w = 0; w < 32; w++)
    {
      float t = temp640[h * 32 + w] - 263.0;
      sum += t;
    }
  }
  int16_t Tire_temp = sum / (32 * 24);
  Serial.print("Average: ");
  Serial.println(Tire_temp);

  delay(500);

  //===================== MLX90614 Logic =============================================
  float tempO = mlx.readObjectTempC();
  int Brake_temp = tempO * 100;

  //====================== BMI323 Logic =============================================
  Serial.println(readRegister16(0x02));
  readRegister16(0x02);
  // if (readRegister16(0x02) == 0x00){
    // Read ChipID
    Serial.print("ChipID:");
    Serial.println(readRegister16(0x00));
    readAllAccel();

    signed_acc_x = twosComplementToNormal(acc_x) / 16384.;
    signed_acc_y = twosComplementToNormal(acc_y) / 16384.;
    signed_acc_z = twosComplementToNormal(acc_z) / 16384.;

    signed_gyr_x = twosComplementToNormal(gyr_x) / 262.1;
    signed_gyr_y = twosComplementToNormal(gyr_y) / 262.1;
    signed_gyr_z = twosComplementToNormal(gyr_z) / 262.1;

    Serial.print("Accel X: ");
    Serial.println(signed_acc_x);
    Serial.print("Accel Y: ");
    Serial.println(signed_acc_y);
    Serial.print("Accel Z: ");
    Serial.println(signed_acc_z);
    Serial.print("Gyro X: ");
    Serial.println(signed_gyr_x);
    Serial.print("Gyro Y: ");
    Serial.println(signed_gyr_y);
    Serial.print("Gyro Z: ");
    Serial.println(signed_gyr_z);

  // }

  delay(500);

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
  int16_t Suspension = 0;
    //=================== FL Module ================================================
  FL_BTSH.buf[0] = Brake_temp >> 8;
  FL_BTSH.buf[1] = Brake_temp & 0xFF;
  FL_BTSH.buf[2] = Tire_temp >> 8;
  FL_BTSH.buf[3] = Tire_temp & 0xFF;
  FL_BTSH.buf[4] = Suspension >> 8;
  FL_BTSH.buf[5] = Suspension & 0xFF;
  FL_BTSH.buf[6] = last_rpm >> 8;
  FL_BTSH.buf[7] = last_rpm & 0xFF;

  FL_ACC.buf[0] = signed_acc_x >> 8;
  FL_ACC.buf[1] = signed_acc_x & 0xFF;
  FL_ACC.buf[2] = signed_acc_y >> 8;
  FL_ACC.buf[3] = signed_acc_y & 0xFF;
  FL_ACC.buf[4] = signed_acc_z >> 8;
  FL_ACC.buf[5] = signed_acc_z & 0xFF;

  FL_GYR.buf[0] = signed_gyr_x >> 8;
  FL_GYR.buf[1] = signed_gyr_x & 0xFF;
  FL_GYR.buf[2] = signed_gyr_y >> 8;
  FL_GYR.buf[3] = signed_gyr_y & 0xFF;
  FL_GYR.buf[4] = signed_gyr_z >> 8;
  FL_GYR.buf[5] = signed_gyr_z & 0xFF;

    //=================== FR Module ================================================
  // FR_BTSH.buf[0] = Brake_temp >> 8;
  // FR_BTSH.buf[1] = Brake_temp & 0xFF;
  // FR_BTSH.buf[2] = Tire_temp >> 8;
  // FR_BTSH.buf[3] = Tire_temp & 0xFF;
  // FR_BTSH.buf[4] = Suspension >> 8;
  // FR_BTSH.buf[5] = Suspension & 0xFF;
  // FR_BTSH.buf[6] = last_rpm >> 8;
  // FR_BTSH.buf[7] = last_rpm & 0xFF;

  // FR_ACC.buf[0] = signed_acc_x >> 8;
  // FR_ACC.buf[1] = signed_acc_x & 0xFF;
  // FR_ACC.buf[2] = signed_acc_y >> 8;
  // FR_ACC.buf[3] = signed_acc_y & 0xFF;
  // FR_ACC.buf[4] = signed_acc_z >> 8;
  // FR_ACC.buf[5] = signed_acc_z & 0xFF;

  // FR_GYR.buf[0] = signed_gyr_x >> 8;
  // FR_GYR.buf[1] = signed_gyr_x & 0xFF;
  // FR_GYR.buf[2] = signed_gyr_y >> 8;
  // FR_GYR.buf[3] = signed_gyr_y & 0xFF;
  // FR_GYR.buf[4] = signed_gyr_z >> 8;
  // FR_GYR.buf[5] = signed_gyr_z & 0xFF;

    //=================== RL Module ================================================
  // RL_BTSH.buf[0] = Brake_temp >> 8;
  // RL_BTSH.buf[1] = Brake_temp & 0xFF;
  // RL_BTSH.buf[2] = Tire_temp >> 8;
  // RL_BTSH.buf[3] = Tire_temp & 0xFF;
  // RL_BTSH.buf[4] = Suspension >> 8;
  // RL_BTSH.buf[5] = Suspension & 0xFF;
  // RL_BTSH.buf[6] = last_rpm >> 8;
  // RL_BTSH.buf[7] = last_rpm & 0xFF;

  // RL_ACC.buf[0] = signed_acc_x >> 8;
  // RL_ACC.buf[1] = signed_acc_x & 0xFF;
  // RL_ACC.buf[2] = signed_acc_y >> 8;
  // RL_ACC.buf[3] = signed_acc_y & 0xFF;
  // RL_ACC.buf[4] = signed_acc_z >> 8;
  // RL_ACC.buf[5] = signed_acc_z & 0xFF;

  // RL_GYR.buf[0] = signed_gyr_x >> 8;
  // RL_GYR.buf[1] = signed_gyr_x & 0xFF;
  // RL_GYR.buf[2] = signed_gyr_y >> 8;
  // RL_GYR.buf[3] = signed_gyr_y & 0xFF;
  // RL_GYR.buf[4] = signed_gyr_z >> 8;
  // RL_GYR.buf[5] = signed_gyr_z & 0xFF;

    //=================== RR Module ================================================
  // RR_BTSH.buf[0] = Brake_temp >> 8;
  // RR_BTSH.buf[1] = Brake_temp & 0xFF;
  // RR_BTSH.buf[2] = Tire_temp >> 8;
  // RR_BTSH.buf[3] = Tire_temp & 0xFF;
  // RR_BTSH.buf[4] = Suspension >> 8;
  // RR_BTSH.buf[5] = Suspension & 0xFF;
  // RR_BTSH.buf[6] = last_rpm >> 8;
  // RR_BTSH.buf[7] = last_rpm & 0xFF;

  // RR_ACC.buf[0] = signed_acc_x >> 8;
  // RR_ACC.buf[1] = signed_acc_x & 0xFF;
  // RR_ACC.buf[2] = signed_acc_y >> 8;
  // RR_ACC.buf[3] = signed_acc_y & 0xFF;
  // RR_ACC.buf[4] = signed_acc_z >> 8;
  // RR_ACC.buf[5] = signed_acc_z & 0xFF;

  // RR_GYR.buf[0] = signed_gyr_x >> 8;
  // RR_GYR.buf[1] = signed_gyr_x & 0xFF;
  // RR_GYR.buf[2] = signed_gyr_y >> 8;
  // RR_GYR.buf[3] = signed_gyr_y & 0xFF;
  // RR_GYR.buf[4] = signed_gyr_z >> 8;
  // RR_GYR.buf[5] = signed_gyr_z & 0xFF;

  bool writeResult = CANbus.write(FL_BTSH) & CANbus.write(FL_ACC) & CANbus.write(FL_GYR);
  // bool writeResult = CANbus.write(FR_BTSH) & CANbus.write(FR_ACC) & CANbus.write(FR_GYR);
  // bool writeResult = CANbus.write(RL_BTSH) & CANbus.write(RL_ACC) & CANbus.write(RL_GYR);
  // bool writeResult = CANbus.write(RR_BTSH) & CANbus.write(RR_ACC) & CANbus.write(RR_GYR);

  Serial.print("Writing Message: ");
  Serial.println(writeResult); // Print the result of the write operation
}