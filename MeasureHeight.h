// Obtain the necessary pins from Pins.h
extern int pin_Sensor5_trigger; extern int pin_Sensor5_echo;


float microseconds2cm(float microseconds, int8_t temperature)
{
  // Compute the speed of sound for the given ambient temperature.
  float a = sqrt(1.4 * 287 * (temperature+273.15));  // [m/s].

  // Convert speed of sound from m/s to microseconds/cm.
  float a_trans = 1 / ((a*100)/(1e6));
  
  return microseconds / a_trans / 2;
}

void MeasureHeight(int8_t temperature, float &height){
  /*
   * Measure height using the ultrasonic sensor. To do so, two pins are required:
   * one pin for the triggering of the sound wav and another pin for the echo
   * receiving task. Next, the time to receive the pulse is measured and translated
   * to centimeters.
   * Height is measured in CENTIMETERS.
   */
   
  // Define trigger pin as output and echo pin as input pin.
  pinMode(pin_Sensor5_trigger, OUTPUT);
  pinMode(pin_Sensor5_echo, INPUT);

  // Define variables to be used later in the code.
  float pulse_duration = 1111.1;

  // Turn off the trigger pin in case it was on before and introduce micro delay.
  digitalWrite(pin_Sensor5_trigger, LOW);
  delayMicroseconds(2);

  // Turn on the trigger so a sound wave is emitted, and wait for 10 microseconds.
  digitalWrite(pin_Sensor5_trigger, HIGH);
  delayMicroseconds(10);

  // Turn off the trigger pin and turn on the echo pin.
  digitalWrite(pin_Sensor5_trigger, LOW);

  // Measure the pulse received by the echo pin.
  pulse_duration = pulseIn(pin_Sensor5_echo, HIGH);  // [microseconds].
  
  // Transform the pulse duration to height (in cm).
  height = microseconds2cm(pulse_duration, temperature);
}
