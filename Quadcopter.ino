// Define pin ports from Pins.h
#include "Pins.h"

// Include setup libraries.
#include "MotorSetup.h"
#include "OpticalSensorSetup.h"
#include "IMU_QUAD.h"

// Declare Euler angles to non-sense values to check possible initial errors.
float roll_angle = -10000.0; float pitch_angle = 10000.0;
float yaw_angle = -10000.0;

// Declare Euler angle rates to non-sense values to check possible initial errors.
float roll_rate = -10000.0; float pitch_rate = 10000.0;
float yaw_rate = -10000.0;

// Declare acceleration measurements to non-sense values.
float x_accel = -999.0; float y_accel = -999.0; float z_accel = -999.0; 


void setup() {

  // Initialize the Serial Monitor.
  Serial.begin(9600);
  delay(1000);
  
  // Initialize the Motors.
  MotorSetup();

  // Arm motors to run at minimum pulse width.
  MotorArm();

  // Initialize IMU.
  initializeIMU();
  

  // Initialize the Sensors.
  SensorSetup();
}

void loop() {

  /*
  // Get Euler angles.
  getEulerAngles(roll_angle, pitch_angle, yaw_angle);
  Serial.print("\tRoll Angle = ");
  Serial.print(roll_angle);
  Serial.print(" |\tPitch Angle = ");
  Serial.print(pitch_angle);
  Serial.print(" |\tYaw Angle = ");
  Serial.println(yaw_angle);
  */

  /*
  // Get Angular velocities.
  getAngularVelocities(roll_rate, pitch_rate, yaw_rate);
  Serial.print("\tRoll Angle Rate = ");
  Serial.print(roll_rate);
  Serial.print(" |\tPitch Angle Rate = ");
  Serial.print(pitch_rate);
  Serial.print(" |\tYaw Angle Rate = ");
  Serial.println(yaw_rate);
  */

  // Get acceleration data.
  getAcceleration(x_accel, y_accel, z_accel);
  Serial.print("\tX acceleration = ");
  Serial.print(x_accel);
  Serial.print(" |\tY acceleration = ");
  Serial.print(y_accel);
  Serial.print(" |\tZ acceleration = ");
  Serial.println(z_accel);

  // Establish delay for new measurements.
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
