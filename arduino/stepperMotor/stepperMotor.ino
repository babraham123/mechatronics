// Motor Control lab
// Bereket Abraham

int maxSteps  = 200;
int stepPin = 9;
int stepperDirPin = 8;

void setup() 
{
  Serial.begin(9600);
  pinMode(stepPin, OUTPUT);
  pinMode(stepperDirPin, OUTPUT);
}

void loop()
{
  digitalWrite(stepperDirPin, HIGH);
  Serial.println("Loop 200 steps (1 rev)");
  for(int x = 0; x < 200; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(100);
  }
  delay(1000);
}
