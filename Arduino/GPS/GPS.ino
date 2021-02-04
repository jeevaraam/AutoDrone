#include "AutoDroneGPS.h"

void setup() {
  // put your setup code here, to run once:
initializeWire();
Serial.begin(115200);
Serial1.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
if(Serial1.available()>1)
{
  feedGPSData(Serial1.read());
}
updateGPS();
}
