
// Declare sensor pins from Pins.h
extern int pin_Sensor1; extern int pin_Sensor2; extern int pin_Sensor3; extern int pin_Sensor4;

int SensorSetup() {
  Serial.println("Starting Optical Sensor...");
  
  // Define the defined pins as input pins.
  pinMode(pin_Sensor1, INPUT);
  pinMode(pin_Sensor2, INPUT);
  pinMode(pin_Sensor3, INPUT);
  pinMode(pin_Sensor4, INPUT);

  delay(500);
  Serial.println("Sensors were properly initialized!");
}
