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

int MotorSetup() {
  // Run setup of ESC.

  // Attach motors to corresponding pins.
  Motor1.attach(pin_Motor1);
  Motor2.attach(pin_Motor2);
  Motor3.attach(pin_Motor3);
  Motor4.attach(pin_Motor4);

  // Initialize motors by sending minimum width pulse.
  Motor1.writeMicroseconds(1000);
  // delay(5000);
  Motor2.writeMicroseconds(1000);
  // delay(5000);
  Motor3.writeMicroseconds(1000);
  // delay(5000);
  Motor4.writeMicroseconds(1000);
  delay(5000);

}
