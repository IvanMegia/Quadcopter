#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

/*
 * Main code of the IMU based on the example given in:
 * https://github.com/adafruit/Adafruit_BNO055/blob/master/examples/read_all_data/read_all_data.ino
 * This script will contain all the functions related with the IMU. These will be:
 *    - Obtain Euler angles: Roll, Pitch, Yaw.
 *    - Obtain angular velocities: omega_x, omega_y, omega_z.
 *    
 * Moreover, within this code the user will be able to calibrate the IMU if no configuration is
 * found within the EEPROM. For that purpose, the user will receive instructions on the serial
 * monitor to calibrate the gyroscope, magnetometer and accelerometer. Once full calibration is
 * finished, this setting will be saved in the EEPROM, so the user will no longer need to calibrate
 * the IMU.
 */

 // Set the refresh time between measurements.
 #define BNO055_SAMPLERATE_DELAY_MS (100)

 // Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void displayIMUDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("          IMU INFORMATION           ");
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}


/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displayIMUStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}


/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer offsets: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro offsets: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMagnetic offsets: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
    */
/**************************************************************************/
void initializeIMU(void)
{
    Serial.println("                                                            IMU INITIALIZATION                                                                 ");
    Serial.println("-----------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.println("Orientation Sensor Test"); Serial.println("");

    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }

    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    }
    else
    {
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        displaySensorOffsets(calibrationData);

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        Serial.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;
    }

    delay(1000);

    /* Display some basic information on this sensor */
    displayIMUDetails();

    /* Optional: Display current status */
    displayIMUStatus();

   /* Crystal must be configured AFTER loading calibration data into BNO055. */
    bno.setExtCrystalUse(true);

    sensors_event_t event;
    bno.getEvent(&event);
    /* always recal the mag as It goes out of calibration very often */
    if (foundCalib){
        Serial.println("Move sensor slightly to calibrate magnetometers");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }
    else
    {
        Serial.println("Please Calibrate Sensor: ");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);

            Serial.print("X: ");
            Serial.print(event.orientation.x, 4);
            Serial.print("\tY: ");
            Serial.print(event.orientation.y, 4);
            Serial.print("\tZ: ");
            Serial.print(event.orientation.z, 4);

            /* Optional: Display calibration status */
            displayCalStatus();

            /* New line for the next sample */
            Serial.println("");

            /* Wait the specified delay before requesting new data */
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }

    Serial.println("\nFully calibrated!");
    Serial.println("--------------------------------");
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

    Serial.println("\n\nStoring calibration data to EEPROM...");

    eeAddress = 0;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    Serial.println("Data stored to EEPROM.");

    Serial.println("\n--------------------------------\n");
    delay(500);
    Serial.println("-----------------------------------------------------------------------------------------------------------------------------------------------");
}

void getEulerAngles(float &roll_angle, float &pitch_angle, float &yaw_angle){
  /*
   * Obtain Euler angles (Roll, pitch and yaw) from the IMU. From tests it is shown
   * that a change on the "x" angle corresponds to a change in yaw; a change on the
   * "y" angle means a change in roll and pitch variation corresponds to a change in
   * "z" angle. The IMU expresses orientation in DEGREES.
   * X axis is oriented towards East, Y axis towards North.
   */

   // Define the orientation event from the Adafruit sensor library.
   sensors_event_t orientationData;

   // Based on the event, extract the corresponding info from the IMU.
   bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

   // Retrieve angles from the IMU event.
   yaw_angle = orientationData.orientation.x;
   roll_angle = orientationData.orientation.y;
   pitch_angle = orientationData.orientation.z;
}

void getAngularVelocities(float &roll_rate, float &pitch_rate, float &yaw_rate){
  /*
   * Get angular velocities for each of the axis. These will be related to the change
   * in the roll, pitch and yaw angles.
   * Units are RADIANS/SECONDS.
   */

   // Define the gyroscope event from the Adafruit sensor library.
   sensors_event_t angVelocityData;

   // Get angular velocities from the IMU.
   bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

   // Retrieve angular velocities.
   pitch_rate = angVelocityData.gyro.x;
   roll_rate = angVelocityData.gyro.y;
   yaw_rate = angVelocityData.gyro.z;
  
}

void getAcceleration(float &x_accel, float &y_accel, float &z_accel){
  /*
   * Get x, y and z components of the acceleration from the IMU.
   * Notice that the gravity acceleration is substracted to
   * eliminate gravity contribution.
   * Units are m/s^2.
   */

   // Define the acceleration event from the Adafruit sensor library.
   sensors_event_t linearAccelData;

   // Get the acceleration data from the IMU.
   bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

   // Retrieve acceleration data.
   x_accel = linearAccelData.acceleration.x;
   y_accel = linearAccelData.acceleration.y;
   z_accel = linearAccelData.acceleration.z;
}

void getTemperature(int8_t &temperature){
  /*
   * Get ambient temperature from the IMU.
   * Temperature is measured in DEGREES CELSIUS.
   */
   temperature = bno.getTemp();
}
