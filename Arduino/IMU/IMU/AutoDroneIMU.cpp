#include "AutoDroneIMU.h"

static byte xaxis[4] = {0};
static byte yaxis[4] = {0};
static byte zaxis[4] = {0};
static byte refresh[4] = {0};

static int requested_register = 0x0;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void initializeWire(void)
{
  Wire1.begin(AUTODRONE_I2C_ADDRESS);
  Wire1.onRequest(writeToMaster);
  Wire1.onReceive(readFromMaster);

  #ifdef DEBUG_EN
  Serial.println("Wire Initialized");
  #endif
}

void initializeIMU(void)
{
  if(!bno.begin())
  {
    Serial.println("BNO055 Not Detected");
    while(1);
  }
}

void floatToByte(float input,byte* buff)
{
  int i=0;
  byte *intermediate = (byte*)&input;
  for(i=0;i<3;i++)
  {
    buff[i] = *(intermediate+i);
  }

  #ifdef DEBUG_EN
  Serial.print("F2B Conv");
  Serial.print(input,6);
  Serial.print(",");
  Serial.println(*buff);
  #endif
}

void update_IMU(void)
{
  sensors_event_t event;
  bno.getEvent(&event);
  floatToByte((float)event.orientation.x,xaxis);
  floatToByte((float)event.orientation.y,yaxis);
  floatToByte((float)event.orientation.z,zaxis);
  floatToByte((float)REFRESH_RATE,refresh);
}

void readFromMaster(int bytecount)
{
  while(1<Wire1.available())
  {
    requested_register=(int)Wire1.read();

    #ifdef DEBUG_EN
    Serial.print("I2C Read:");
    Serial.println(requested_register);
    #endif
  }
  int val = Wire1.read();
}

void writeToMaster(void)
{
  switch(requested_register)
  {
    case 0x01:Wire1.write(0x01);
              break;
    case 0x02:Wire1.write(xaxis,4);
              break;
    case 0x03:Wire1.write(yaxis,4);
              break;
    case 0x04:Wire1.write(zaxis,4);
              break;
    case 0x05:Wire1.write(refresh,4);
              break;
    default:Wire1.write("0");
            break;
  }
}
