/*
 * Ultrasonic Sensor Array
 * by Roy Seungwon Shin
 * for Carnegie Mechatronics WindowWasher
 */
#include <Servo.h>

Servo myservo;
// Pin Assignments
const int nPin = 13;
const int ePin = 10;
const int sPin = 7;
const int wPin = 4;
const int nLED = 11;
const int eLED = 8;
const int sLED = 5;
const int wLED = 2;
const int nEcho = 12;
const int eEcho = 9;
const int sEcho = 6;
const int wEcho = 3;
const int pressurePin = A0;
const int servoPin = 9;

// Variables
long nDuration, eDuration, sDuration, wDuration;
long nDistance, eDistance, sDistance, wDistance;
int analogPressure, voltage;
int servoAngle = 175;
long pressure;

// Function Declarations
long microsecondsToInches(long microseconds);
long microsecondsToCentimeters(long microseconds);

void setup() {
  Serial.begin(9600);
  myservo.attach(servoPin);
  pinMode(pressurePin, INPUT);
  pinMode(nLED, OUTPUT);
  pinMode(eLED, OUTPUT);
  pinMode(sLED, OUTPUT);
  pinMode(wLED, OUTPUT);
  digitalWrite(nLED, LOW);
  digitalWrite(eLED, LOW);
  digitalWrite(sLED, LOW);
  digitalWrite(wLED, LOW);
  pinMode(nPin, OUTPUT);
  pinMode(ePin, OUTPUT);
  pinMode(wPin, OUTPUT);
  pinMode(sPin, OUTPUT);
  pinMode(nEcho, INPUT);
  pinMode(eEcho, INPUT);
  pinMode(wEcho, INPUT);
  pinMode(sEcho, INPUT);
}

void loop() {
  analogPressure = analogRead(pressurePin);
  voltage = map(analogPressure, 0, 1023, 0, 5000);
  pressure = (voltage - 2820)/54;

  digitalWrite(nPin, LOW);
  delayMicroseconds(2);
  digitalWrite(nPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(nPin, LOW);
  nDuration = pulseIn(nEcho, HIGH);

  digitalWrite(ePin, LOW);
  delayMicroseconds(2);
  digitalWrite(ePin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ePin, LOW);
  eDuration = pulseIn(eEcho, HIGH);

  digitalWrite(sPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sPin, LOW);
  sDuration = pulseIn(sEcho, HIGH);

  digitalWrite(wPin, LOW);
  delayMicroseconds(2);
  digitalWrite(wPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(wPin, LOW);
  wDuration = pulseIn(wEcho, HIGH);

  nDistance = microsecondsToCentimeters(nDuration);
  eDistance = microsecondsToCentimeters(eDuration);
  sDistance = microsecondsToCentimeters(sDuration);
  wDistance = microsecondsToCentimeters(wDuration);

  Serial.print("n");
  Serial.print(String(nDistance));
  Serial.print("\n");
  Serial.print("e");
  Serial.print(String(eDistance));
  Serial.print("\n");
  Serial.print("s");
  Serial.print(String(sDistance));
  Serial.print("\n");
  Serial.print("w");
  Serial.print(String(wDistance));
  Serial.print("\n");

  if (nDistance < 10) {
    digitalWrite(nLED, HIGH);
  } else {
    digitalWrite(nLED, LOW);
  }
  if (eDistance < 10) {
    digitalWrite(eLED, HIGH);
  } else {
    digitalWrite(eLED, LOW);
  }
  if (sDistance < 10) {
    digitalWrite(sLED, HIGH);
  } else {
    digitalWrite(sLED, LOW);
  }
  if (wDistance < 10) {
    digitalWrite(wLED, HIGH);
  } else {
    digitalWrite(wLED, LOW);
  }

  while (Serial.available() > 0) {
    servoAngle = Serial.readStringUntil('\n').toInt();
  }
  servoAngle = constrain(servoAngle, 100,175);
  myservo.write(servoAngle);
  delay(100);
}

long microsecondsToInches(long microseconds) {
  return microseconds / 58.2;
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}
