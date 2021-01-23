// Define pin ports from Pins.h
#include "Pins.h"

// Include setup libraries.
#include "MotorSetup.h"
#include "OpticalSensorSetup.h"
#include "IMU_QUAD.h"
#include "MeasureHeight.h";

// Declare Euler angles to non-sense values to check possible initial errors.
float roll_angle = -10000.0; float pitch_angle = 10000.0;
float yaw_angle = -10000.0;

// Declare Euler angle rates to non-sense values to check possible initial errors.
float roll_rate = -10000.0; float pitch_rate = 10000.0;
float yaw_rate = -10000.0;

// Declare temperature and height variables to non-sense values.
int8_t temperature = 100; float height = 100000.0;

// Declare acceleration measurements to non-sense values.
float x_accel = -999.0; float y_accel = -999.0; float z_accel = -999.0;


/*
 * This is the main script for the Quadcopter project. Before uploading the code,
 * read the instructions below:
 * 0. Modify pins at which motors, optical sensors and IMU will be connected in Pins.h
 * 1. If your motors require of calibration, change REQUIRE_CALIBRATION in MotorSetup.h
 *    to true.
 * 2. Either if your motors are calibrated or not, the user may change the
 *    minimum and maximum pulse widths as desired in MotorSetup.h by changing the
 *    values of MIN_PULSE_LENGTH and MAX_PULSE_LENGTH. If your motors are calibrated
 *    under some other values, change these variables as required. If your motors are
 *    not calibrated, these variables may be changed by user requirements.
 * 3. If calibration is required, follow the instructions given on screen. Otherwise,
 *    proceed with the execution.
 * 4. At this point, motors will start rotating in a given sequence at minimum
 *    throttle.
 * 5. Next, the IMU is initialized. At this point the serial monitor should already
 *    be opened to follow instructions.
 * 6. If no IMU calibration is found on Arduino's EEPROM, the user may follow the
 *    instructions given in:
 *    https://es.mathworks.com/help/supportpkg/arduinoio/ug/calibrate-sensors.html
 *    The code will automatically stop the calibration once the full process is done.
 *    The calibration profile will be saved. If a calibration profile is found,
 *    the IMU may require to slightly move the device to calibrate the magnetometer.
 * 7. Optical Sensors are initialized.
 */

void setup() {

  // Initialize the Serial Monitor.
  Serial.begin(9600);
  delay(1000);
  
  // Initialize the Motors.
  MotorSetup();

  // Initialize IMU.
  initializeIMU();

  // Arm motors to run at minimum pulse width.
  setIdle();
  
  // Initialize the Optical Sensors.
  OpticalSensorSetup();
  
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

  // Get Angular velocities.
  getAngularVelocities(roll_rate, pitch_rate, yaw_rate);
  Serial.print("\tRoll Angle Rate = ");
  Serial.print(roll_rate);
  Serial.print(" |\tPitch Angle Rate = ");
  Serial.print(pitch_rate);
  Serial.print(" |\tYaw Angle Rate = ");
  Serial.println(yaw_rate);

  /*
  // Get Height in centimeters from the ultrasonic sensor.
  getTemperature(temperature);
  MeasureHeight(temperature, height);
  Serial.print("\tAmbient Temperature [ºC] = ");
  Serial.print(temperature);
  Serial.print(" |\tHeight [centimetres] = ");
  Serial.println(height);
  */

  /*
  // Get acceleration data.
  getAcceleration(x_accel, y_accel, z_accel);
  Serial.print("\tX acceleration = ");
  Serial.print(x_accel);
  Serial.print(" |\tY acceleration = ");
  Serial.print(y_accel);
  Serial.print(" |\tZ acceleration = ");
  Serial.println(z_accel);
  */

  // Establish delay for new measurements.
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
