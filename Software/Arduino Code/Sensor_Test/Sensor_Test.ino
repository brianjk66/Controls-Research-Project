// Simple program to test the Teensy's
// analogRead of the rotary position sensor 

int sensorVal;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
}

void loop() {
  sensorVal = analogRead(A0);
  Serial.print("Sensor Reading: ");
  Serial.print(sensorVal);
  Serial.print("\tAngle Reading: ");
  Serial.println((0.3656*sensorVal)-185.64, 2);
}
