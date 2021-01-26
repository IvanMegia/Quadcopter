// Load the necessary library.
#include <PID_v2.h>;

// Pitch and Roll PID parameters.
// Pitch conservative and aggressive PID constants.
double aggKp = 0.935, aggKi = 0., aggKd = 0.025;
double consKp = 0.635, consKi = 0., consKd = 0.;

// Angular Roll & Pitch conservative and aggressive PID constants.
double aggKp_a = 1.025, aggKi_a = 0., aggKd_a = 0.025;
double consKp_a = 0.750, consKi_a = 0., consKd_a = 0.;

// Define output limits for the pitch and roll PIDs ({min, max}).
int pitch_limits[2] = {-100, 200}, roll_limits[2] = {-100, 200};

// Define output limits for the angular pitch and roll PIDs.
int angular_pitch_limits[2] = {-500, 500}, angular_roll_limits[2] = {-500, 500};

// Define the limit (in degrees) at which aggressive or conservative PID constants may be used.
/*
 * Within the controller, a gap is computed between the desired angle and its current value.
 * If that gap is bigger than the defined limit, aggressive constants may be used to restore
 * equilibrium faster. Otherwise, conservative values will be used.
 */
 int aggressive_limit_angles = 8;  // [degrees].
 int aggressive_limit_rates = 12;  // [rads/s].


// Create PID controllers for the pitch and roll.
PID_v2 pitchPID(consKp, consKi, consKd, PID::Direct);
PID_v2 pitchAngularPID(consKp_a, consKi_a, consKd_a, PID::Direct);
PID_v2 rollPID(consKp, consKi, consKd, PID::Direct);
PID_v2 rollAngularPID(consKp_a, consKi_a, consKd_a, PID::Direct);

void initializePitchPID(float pitch_angle){
  // Initialize the PID controller for the pitch.
  /*
   * In order to initialize the PID, the Start method from the PID lib
   * is used. The first input is the variable to be controlled, the
   * second is the current output and the third one is the setpoint
   * (the objective to be achieved by the PID controller).
   */
  pitchPID.Start(pitch_angle, 0, 0);
  pitchPID.SetOutputLimits(pitch_limits[0], pitch_limits[1]);
}

void initializeRollPID(float roll_angle){
  // Initialize the PID controller for the roll.
  /*
   * In order to initialize the PID, the Start method from the PID lib
   * is used. The first input is the variable to be controlled, the
   * second is the current output and the third one is the setpoint
   * (the objective to be achieved by the PID controller).
   */
  rollPID.Start(roll_angle, 0, 0);
  rollPID.SetOutputLimits(roll_limits[0], roll_limits[1]);
}

void initializePitchAngularPID(float pitch_rate){
  pitchAngularPID.Start(pitch_rate, 0, 0);
  pitchAngularPID.SetOutputLimits(angular_pitch_limits[0], angular_pitch_limits[1]);
  
}

void initializeRollAngularPID(float roll_rate){
  rollAngularPID.Start(roll_rate, 0, 0);
  rollAngularPID.SetOutputLimits(angular_roll_limits[0], angular_roll_limits[1]);
}

int runPitchPID(float pitch_angle){
  // Compute the gap between the desired pitch angle and its current value.
  float gap = abs(pitchPID.GetSetpoint() - pitch_angle);

  /*
   * Depending on the existing gap, the PID controller may use aggressive or conservative
   * PID constants.
   */
   if (gap >= aggressive_limit_angles){
    pitchPID.SetTunings(aggKp, aggKi, aggKd);
   }
   else {
    pitchPID.SetTunings(consKp, consKi, consKd);
   }

   // Run the PID.
   const int output = pitchPID.Run(pitch_angle);

   return output;
}

int runPitchAngularPID(float pitch_rate){
  // Compute the gap between the desired pitch angle rate and its current value.
  float gap = abs(pitchAngularPID.GetSetpoint() - pitch_rate);

  /*
   * Depending on the existing gap, the PID controller may use aggressive or conservative
   * PID constants.
   */
   if (gap >= aggressive_limit_rates){
    pitchAngularPID.SetTunings(aggKp_a, aggKi_a, aggKd_a);
   }
   else {
    pitchAngularPID.SetTunings(consKp_a, consKi_a, consKd_a);
   }

   // Run the PID.
   const int output = pitchAngularPID.Run(pitch_rate);

   return output;
}

int runRollPID(float roll_angle){
  // Compute the gap between the desired roll angle and its current value.
  float gap = abs(rollPID.GetSetpoint() - roll_angle);

  /*
   * Depending on the existing gap, the PID controller may use aggressive or conservative
   * PID constants.
   */
   if (gap >= aggressive_limit_angles){
    rollPID.SetTunings(aggKp, aggKi, aggKd);
   }
   else {
    rollPID.SetTunings(consKp, consKi, consKd);
   }

   // Run the PID.
   const int output = rollPID.Run(roll_angle);

   return output;
}

int runRollAngularPID(float roll_rate){
  // Compute the gap between the desired roll angle rate and its current value.
  float gap = abs(rollAngularPID.GetSetpoint() - roll_rate);

  /*
   * Depending on the existing gap, the PID controller may use aggressive or conservative
   * PID constants.
   */
   if (gap >= aggressive_limit_rates){
    rollAngularPID.SetTunings(aggKp_a, aggKi_a, aggKd_a);
   }
   else {
    pitchAngularPID.SetTunings(consKp_a, consKi_a, consKd_a);
   }

   // Run the PID.
   const int output = rollAngularPID.Run(roll_rate);

   return output;
}
