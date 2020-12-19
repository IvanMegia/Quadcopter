void setup() {
  // DEFINE THE FOUR SENSORS.
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  Serial.begin(9600);
  Serial.println("Starting Optical Sensor...");

}

void loop() {
  int detect1 = digitalRead(8);
  int detect2 = digitalRead(9);

  if (detect1 == HIGH && detect2 == HIGH) {
    Serial.println("All cleared");
  }
  else {
    Serial.println("Obstacle on the way!");
  }
   delay(300);

}
