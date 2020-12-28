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
  // Run motors.
  Motor1.writeMicroseconds(180);
  Motor2.writeMicroseconds(180);
  Motor3.writeMicroseconds(180);
  Motor4.writeMicroseconds(180);

  // Introduce a delay on purpose.
  delay(100);  // [ms].

}
