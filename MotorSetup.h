#include <Servo.h>

// Define the Motor Objects as Servo Objects.
/*
Despite these engines are brushless dc, with this library we can
handle the esc by sending pwm pulses, which are then used by the
electronic speed controller.
*/
Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;

// Let compiler know that pins' variables are coming from external file.
extern int pin_Motor1; extern int pin_Motor2; extern int pin_Motor3;
extern int pin_Motor4;

// Define minimum width for the PWM. This value is used to arm ESC.
unsigned int minimum_width = 100; // [microseconds].

/*
 * Define the security delay, which is defined as the time ellapsed
 * between the arming sequence and the beginning of the rotation of
 * the motors.
 */
 unsigned int security_delay = 5000;  // [miliseconds].

 // Define ESC 1 (old ESC) minimum pulse width to start rotation.
 unsigned int minimum_rotation_old = 600;  // [microseconds].

  // Define ESC 2 (new ESC) minimum pulse width to start rotation.
 unsigned int minimum_rotation_new = 1060;  // [microseconds].

 // Define the delay between the beginning of rotation between motors.
 unsigned int start_rotation_delay = 1500;  // [miliseconds].


/**************************************************************************/
/*
    Define functions for the arming sequence and to start
                          minimum rotation.
    */
/**************************************************************************/

void MotorSetup() {
  // Run setup of ESC.

  // Attach motors to corresponding pins.
  Motor1.attach(pin_Motor1);
  Motor2.attach(pin_Motor2);
  Motor3.attach(pin_Motor3);
  Motor4.attach(pin_Motor4);

  // Initialize motors by sending minimum width pulse.
  Motor1.writeMicroseconds(minimum_width);
  Motor2.writeMicroseconds(minimum_width);
  Motor3.writeMicroseconds(minimum_width);
  Motor4.writeMicroseconds(minimum_width);

  // Introduce a security delay.
  delay(security_delay);

}

void MotorArm() {
  // Arm motors to run at minimum pulse width.
  Motor1.writeMicroseconds(minimum_rotation_old);
  delay(start_rotation_delay);
  Motor2.writeMicroseconds(minimum_rotation_new);
  delay(start_rotation_delay);
  Motor3.writeMicroseconds(minimum_rotation_old);
  delay(start_rotation_delay);
  Motor4.writeMicroseconds(minimum_rotation_new);
  delay(start_rotation_delay);
}
