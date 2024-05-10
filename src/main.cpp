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
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CANbus;    // For custom PCB IT'S CAN 2 !!!!!! DONT FUCKING CHANGE IT 
// FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;    // For old PCB IT'S CAN 3 !!!!!!!!!!!!!!
CAN_message_t msg1;
CAN_message_t msg2;
CAN_message_t msg3;
CAN_message_t msg4;

// =================== MLX Variables ================================================
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_MLX90640 mlx2;

// =================== BMI Variables ================================================
uint16_t gyr_x, gyr_y, gyr_z;
int16_t signed_gyr_x, signed_gyr_y, signed_gyr_z;

// =================== Hall Effect Variables ========================================
IntervalTimer halleffectTimer;
volatile  double last_rpm = 0;
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
void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(hallEffectPin, INPUT);
  threads.addThread(thread_func, 1);
  halleffectTimer.begin(calculateRPM, TIMER_MICROSECONDS);

  Serial.begin(115200);
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");\
    return;
  }
  Serial.println("card initialized.");
  
  while (!Serial && millis() < 4000);
  Wire.begin();
  Wire.setClock(400000); 
  Wire1.begin();
  Wire1.setClock(400000); 
  CANbus.begin();
  CANbus.setBaudRate(500000); 
  mlx.begin();
  mlx2.begin();
  mlx2.setRefreshRate(MLX90640_2_HZ);
}

void loop() {
  // =================== CAN Message Setup ===========================================
  // msg1.id = 0x100; 
  // msg1.len = 8; 

  msg2.id = 0x101; 
  msg2.len = 8; 

  // msg3.id = 0x102; 
  // msg3.len = 8;    

  // msg4.id = 0x103; 
  // msg4.len = 8;    

  Serial.println("\n Loop Running... \n");

  //====================== MLX90640 Logic ============================================
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

  delay(500);

  //===================== MLX90614 Logic =============================================
  float tempO = mlx.readObjectTempC();
  float tempA = mlx.readAmbientTempC();
  int sendObjVal = tempO * 100;
  int sendAmbVal = tempA * 100;

  //===================== SD Card Data Logging ======================================
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file is available, write the contents of datastring to it
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }   

  //===================== Data Push ==================================================
  // msg1.buf[0] = sendObjVal >> 8;
  // msg1.buf[1] = sendObjVal & 0xFF;
  // msg1.buf[2] = sendAmbVal >> 8;
  // msg1.buf[3] = sendAmbVal & 0xFF;

  msg2.buf[0] = sendObjVal >> 8;
  msg2.buf[1] = sendObjVal & 0xFF;
  msg2.buf[2] = sendAmbVal >> 8;
  msg2.buf[3] = sendAmbVal & 0xFF;

  // msg3.buf[0] = sendObjVal >> 8;
  // msg3.buf[1] = sendObjVal & 0xFF;
  // msg3.buf[2] = sendAmbVal >> 8;
  // msg3.buf[3] = sendAmbVal & 0xFF;

  // msg4.buf[0] = sendObjVal >> 8;
  // msg4.buf[1] = sendObjVal & 0xFF;
  // msg4.buf[2] = sendAmbVal >> 8;
  // msg4.buf[3] = sendAmbVal & 0xFF;

  // bool writeResult = CANbus.write(msg1) & CANbus.write(msg2) & CANbus.write(msg3) & CANbus.write(msg4);  // Send the message
  // bool writeResult = CANbus.write(msg1);
  bool writeResult = CANbus.write(msg2);
  // bool writeResult = CANbus.write(msg3);
  // bool writeResult = CANbus.write(msg4);

  Serial.print("Writing Message: ");
  Serial.println(writeResult);  // Print the result of the write operation

}