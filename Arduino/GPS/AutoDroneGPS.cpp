#include "AutoDroneGPS.h"

static byte satellites_connected=0;
static byte latitude[4]={0};
static byte longitude[4]={0};
static byte altitude[4]={0};
static byte h_accuracy[4]={0};
static byte v_accuracy[4]={0};
static bool gpsUpdated=false;

static int requested_register=0x0;

TinyGPS gps;

void initializeWire(void)
{
	Wire.begin(AUTODRONE_I2C_ADDRESS);
	Wire.onRequest(writeToMaster);
	Wire.onReceive(readFromMaster);
}

void floatToByte(float input,byte* buff)
{
	int i=0;
	byte *intermediate = (byte*)&input;
	for(i=0;i<3;i++)
	{
		buff[i] = *(intermediate+i);
	}
}

void feedGPSData(char c)
{
	if(gps.encode(c))
		gpsUpdated=true;
}

void updateGPS(void)
{
	if(gpsUpdated)
	{
		float flat=0,flong=0,h_acc=0,v_acc=0,altitude=0;
		unsigned long age;
		int sats=0;
		gps.f_get_position(&flat,&flong,&age);
		flat=(flat==TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat);
		flong=(flong==TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flong);
		sats = (gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
		h_acc = (gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
		floatToByte(flat,latitude);
		floatToByte(flong,longitude);
		floatToByte(h_acc,h_accuracy);
		satellites_connected = (byte)sats;
		gpsUpdated=false;
	}
}

void readFromMaster(int bytecount)
{
	while(1<Wire.available())
	{
		requested_register=(int)Wire.read();
	}
	int val = Wire.read();
}

void writeToMaster(void)
{
	switch(requested_register)
	{
		case 0x01:Wire.write(satellites_connected);
		          break;
		case 0x02:Wire.write(latitude,4);
		          break;
		case 0x03:Wire.write(longitude,4);
		          break;
		case 0x04:Wire.write(altitude,4);
		          break;
		case 0x05:Wire.write(h_accuracy,4);
		          break;
		case 0x06:Wire.write(v_accuracy,4);
		          break;
		case 0x07:Wire.write("0");
		          break;
		default:Wire.write("0");
		        break;
	}
}
