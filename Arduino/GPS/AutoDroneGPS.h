#ifndef AUTODRONEGPS_H
#define AUTODRONEGPS_H

#include "Arduino.h"
#include <Wire.h>
#include <TinyGPS.h>

#define AUTODRONE_I2C_ADDRESS_BASE 0x20
#define DEV_ID 0x01
#define AUTODRONE_I2C_ADDRESS AUTODRONE_I2C_ADDRESS_BASE+DEV_ID

void initializeWire(void);
void updateGPS(void);
void readFromMaster(int byteCount);
void writeToMaster(void);
void feedGPSData(char c);
void floatToByte(float input, byte *buff);

#endif 
