// Define pin ports from Pins.h
#include "Pins.h"

// Include setup libraries.
#include "Motors.h"
#include "OpticalSensorSetup.h"
#include "IMU_QUAD.h"
#include "MeasureHeight.h";
#include "PID.h";

// Declare Euler angles.
float roll_angle = 20.0, initial_roll_angle = 0, pitch_angle = 20.0, initial_pitch_angle = 0;
float yaw_angle = -10000.0, initial_yaw_angle = 0;

// Declare Euler angle rates [rad/s];
float roll_rate = 20.0; float pitch_rate = 20.0;
float yaw_rate = -10000.0;

// Declare temperature and height variables to non-sense values.
int8_t temperature = 100; float height = 100000.0;

// Declare acceleration measurements to non-sense values.
float x_accel = -999.0; float y_accel = -999.0; float z_accel = -999.0;

// Define outputs from the pitch and roll PID controllers.
int output_pitch = 0, output_roll = 0, output_pitch_rate = 0, output_roll_rate = 0;

// Define the maximum height at which the quadcopter can fly.
const int max_height = 10;  // [cm].

// Define PID thresholds.
const float pid_angle_threshold = 1.0, pid_angular_threshold = 0.25;

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

  // Get initial angles.
  getEulerAngles(initial_roll_angle, initial_pitch_angle, initial_yaw_angle);

  // Arm motors to run at minimum pulse width.
  setIdle();
  
  // Initialize the Optical Sensors.
  OpticalSensorSetup();

  // Initialize the pitch and roll PIDs.
  initializePitchPID(pitch_angle);
  initializeRollPID(roll_angle);
  initializePitchAngularPID(pitch_rate);
  initializeRollAngularPID(roll_rate);
  
}

void loop() {

  /**************************************************************************/
  /*
                    Get all information from the IMU.
      */
  /**************************************************************************/
  // Get Euler angles.
  getEulerAngles(roll_angle, pitch_angle, yaw_angle);
  pitch_angle -= initial_pitch_angle;
  roll_angle -= initial_roll_angle;
  
  // Get Angular velocities.
  getAngularVelocities(roll_rate, pitch_rate, yaw_rate);

  // Get Height in centimeters from the ultrasonic sensor.
  getTemperature(temperature);
  MeasureHeight(temperature, height);
  Serial.println(height);

  /*
  if (height < max_height){
    addMicroseconds(5);
  }
  else if (height > max_height){
    addMicroseconds(-5);
  }
  */
  

  // Get acceleration data.
  getAcceleration(x_accel, y_accel, z_accel);

    /**************************************************************************/
  /*
                                    PID
      */
  /**************************************************************************/
  // Run pitch and roll PIDs.
  output_pitch = runPitchPID(pitch_angle);
  output_pitch_rate = (runPitchAngularPID(pitch_rate));
  output_roll = runRollPID(roll_angle);
  output_roll_rate = (runRollAngularPID(roll_rate));

  // Decide which motors should be corrected by the sign of the pitch angle.
  
  if (pitch_angle > pid_angle_threshold){
    decreasePitch(output_pitch, pitch_angle);
  }
  else if (pitch_angle < -pid_angle_threshold){
    increasePitch(output_pitch, pitch_angle);
  }

  // Decide which motors should be corrected by the sign of the pitch angular velocity.
  if (pitch_rate > pid_angular_threshold){
    increasePitch(output_pitch_rate, pitch_angle);
  }
  else if (pitch_rate < -pid_angular_threshold){
    decreasePitch(output_pitch_rate, pitch_angle);
  }

  // Decide which motors should be corrected by the sign of the pitch angle.
  if (roll_angle > pid_angle_threshold){
    decreaseRoll(output_roll, roll_angle);
  }
  else if (roll_angle < -pid_angle_threshold){
    increaseRoll(output_roll, roll_angle);
  }

  // Decide which motors should be corrected by the sign of the pitch angular velocity.
  if (roll_rate > pid_angular_threshold){
    increaseRoll(output_roll_rate, roll_angle);
  }
  else if (roll_rate < -pid_angular_threshold){
    decreaseRoll(output_roll_rate, roll_angle);
  }

  // Establish delay for new measurements.
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
