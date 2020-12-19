#include <Servo.h>

// Define the motor object.
Servo Motor0;

void setup() {
  // put your setup code here, to run once:
  Motor0.attach(8);
  Motor0.writeMicroseconds(1000);
  delay(5000);

}

void loop() {
  // put your main code here, to run repeatedly:
  Motor0.write(180);
  delay(20);
}
