#include "AutoDroneIMU.h"

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
initializeIMU();
initializeWire();
}

void loop() {
  // put your main code here, to run repeatedly:
update_IMU();
}
