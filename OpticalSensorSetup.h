
// Declare sensor pins from Pins.h
extern int pin_Sensor1; extern int pin_Sensor2; extern int pin_Sensor3; extern int pin_Sensor4;

void OpticalSensorSetup() {
  Serial.println("Starting Optical Sensor...");
  
  // Define the defined pins as input pins.
  pinMode(pin_Sensor1, INPUT);

  delay(500);
  Serial.println("Sensors were properly initialized!");
}

boolean detect_obstacles(){
  boolean detect = digitalRead(pin_Sensor1);
  if (detect == HIGH){
    Serial.println("No Obstacle detected");
  }
  else if (detect == LOW){
    Serial.println("Obstacle detected");
  }

  return detect;
}
