#ifndef AUTODRONEIMU_H
#define AUTODRONEIMU_H

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define AUTODRONE_I2C_ADDRESS_BASE 0x10
#define DEV_ID 0x01
#define AUTODRONE_I2C_ADDRESS AUTODRONE_I2C_ADDRESS_BASE+DEV_ID
#define REFRESH_RATE 100
#define DEBUG_EN

void initializeWire(void);
void initializeIMU(void);
void update_IMU(void);
void readFromMaster(int byteCount);
void writeToMaster(void);
void floatToByte(float input, byte *buff);

#endif
