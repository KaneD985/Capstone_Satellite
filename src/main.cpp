#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MLX90614.h>
#include <math.h>
#include "bmi323.h"
#include "common.h"

const int ledPin = 13;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;  // CAN0 is the CAN module to use
CAN_message_t msg;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// =================== BMI323 Static Functions Declaration ==========================

static int8_t set_gyro_config(struct bmi3_dev *dev);

static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

// ==================================================================================

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  while (!Serial && millis() < 4000);
  CANbus.begin();
  CANbus.setBaudRate(500000); 
  mlx.begin();
}

void loop() {





  // =================== BMI323 Logic ======================================================

  /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Variable to define limit to print gyro data. */
    uint8_t limit = 100;

    float x = 0, y = 0, z = 0;
    uint8_t indx = 0;

    /* Sensor initialization configuration. */
    struct bmi3_dev dev = { 0 };

    /* Create an instance of sensor data structure. */
    struct bmi3_sensor_data sensor_data = { 0 };

    /* Initialize the interrupt status of gyro. */
    uint16_t int_status = 0;

    /* Structure to define gyro configuration. */
    struct bmi3_sens_config config = { 0 };

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated.
     * Interface reference is given as a parameter
     * For I2C : BMI3_I2C_INTF
     * For SPI : BMI3_SPI_INTF
     */
    rslt = bmi3_interface_init(&dev, BMI3_SPI_INTF);
    bmi3_error_codes_print_result("bmi3_interface_init", rslt);

    if (rslt == BMI323_OK)
    {
        /* Initialize bmi323. */
        rslt = bmi323_init(&dev);
        bmi3_error_codes_print_result("bmi323_init", rslt);

        if (rslt == BMI323_OK)
        {
            /* Gyro configuration settings. */
            rslt = set_gyro_config(&dev);

            if (rslt == BMI323_OK)
            {
                rslt = bmi323_get_sensor_config(&config, 1, &dev);
                bmi3_error_codes_print_result("bmi323_get_sensor_config", rslt);
            }

            if (rslt == BMI323_OK)
            {
                /* Select gyro sensor. */
                sensor_data.type = BMI323_GYRO;

                printf("\nData set, Range, Gyr_Raw_X, Gyr_Raw_Y, Gyr_Raw_Z, Gyr_dps_X, Gyr_dps_Y, Gyr_dps_Z\n\n");

                /* Loop to print gyro data when interrupt occurs. */
                while (indx <= limit)
                {
                    /* To get the data ready interrupt status of gyro. */
                    rslt = bmi323_get_int1_status(&int_status, &dev);
                    bmi3_error_codes_print_result("Get interrupt status", rslt);

                    /* To check the data ready interrupt status and print the status for 10 samples. */
                    if (int_status & BMI3_INT_STATUS_GYR_DRDY)
                    {
                        /* Get gyro data for x, y and z axis. */
                        rslt = bmi323_get_sensor_data(&sensor_data, 1, &dev);
                        bmi3_error_codes_print_result("Get sensor data", rslt);

                        /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
                        x = lsb_to_dps(sensor_data.sens_data.gyr.x, (float)2000, dev.resolution);
                        y = lsb_to_dps(sensor_data.sens_data.gyr.y, (float)2000, dev.resolution);
                        z = lsb_to_dps(sensor_data.sens_data.gyr.z, (float)2000, dev.resolution);

                        /* Print the data in dps. */
                        printf("%d, %d, %d, %d, %d, %4.2f, %4.2f, %4.2f\n",
                               indx,
                               config.cfg.gyr.range,
                               sensor_data.sens_data.gyr.x,
                               sensor_data.sens_data.gyr.y,
                               sensor_data.sens_data.gyr.z,
                               x,
                               y,
                               z);

                        indx++;
                    }
                }
            }
        }
    }

    bmi3_coines_deinit();

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