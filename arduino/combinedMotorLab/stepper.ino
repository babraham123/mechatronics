int stepPin = 3;
int dirPin = 4;

void setup() {
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
}

void loop() {

  digitalWrite(dirPin, HIGH);

  for(int x = 0; x < 400; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  digitalWrite(dirPin, LOW);

  delay(100);
  
  for(int x = 0; x < 400; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  delay(200);
}
