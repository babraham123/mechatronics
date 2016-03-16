#include <Servo.h>
#include <Encoder.h>

bool xControl();
bool yControl();

/* PWM */
const int xPin1 = 2;
const int xPin2 = 3;
const int yPin1 = 4;
const int yPin2 = 5

/* Digital */
const int L1Pin = 22;
const int L2Pin = 23;
const int L3Pin = 24;
const int L4Pin = 25;

/* constants */
int xTarget = 0;
int yTarget = 0;
char command;
int number;

/* Rotary Encoders */
Encoder xEnc(xPin1, xPin2);
Encoder yEnc(yPin1, yPin2);

void setup() {
  pinMode(L1Pin, OUTPUT);
  pinMode(L2Pin, OUTPUT);
  pinMode(L3Pin, OUTPUT);
  pinMode(L4Pin, OUTPUT);
  xEnc.write(0);
  yEnc.write(0);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    command = Serial.read();
    if (Serial.available()) {
      number = Serial.parseInt();
      Serial.println("ok");
      Serial.print(command);
      Serial.print(number);
      switch(command) {
        case 'x':
          xTarget = number;
          break;
        case 'y':
          yTarget = number;
          break;
        case 'n':
          break;
        case 'e':
          break;
        case 's':
          break;
        case 'w':
          break;
      }
    }
  }
  xControl();
  yControl();
}

bool xControl() {
  int xPos = xEnc.read();
  if (xPos > xTarget) {
    digitalWrite(L1Pin, HIGH);
    digitalWrite(L2Pin, LOW);
    return true;
  } else if (xPos < xTarget) {
    digitalWrite(L1Pin, LOW);
    digitalWrite(L2Pin, HIGH);
    return true;
  } else {
    digitalWrite(L1Pin, LOW);
    digitalWrite(L2Pin, LOW);
    return true;
  }
  return false;
}

bool yControl() {
  int yPos = yEnc.read();
  if (yPos > yTarget) {
    digitalWrite(L3Pin, HIGH);
    digitalWrite(L4Pin, LOW);
    return true;
  } else if (yPos < yTarget) {
    digitalWrite(L3Pin, LOW);
    digitalWrite(L4Pin, HIGH);
    return true;
  } else {
    digitalWrite(L3Pin, LOW);
    digitalWrite(L4Pin, LOW);
    return true;
  }
  return false;
}

