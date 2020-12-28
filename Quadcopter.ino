// Include setup libraries.
#include "MotorSetup.h"
#include "OpticalSensorSetup.h"

// Define pin ports from Pins.h
#include "Pins.h"


void setup() {
  // Initialize the Motors.
  MotorSetup();

  // Initialize the Sensors.
  SensorSetup();
}

void loop() {
  // put your main code here, to run repeatedly:

}
