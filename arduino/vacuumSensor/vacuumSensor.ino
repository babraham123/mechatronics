// test the vacuum sensor
// Bereket Abraham

const int vacuumPin = A0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(vacuumPin);
  Serial.println(sensorValue);
  delay(100);
}

