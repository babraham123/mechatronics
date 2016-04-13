/*
   XYZ Control of Robot w/o Rotary Encoder
   by Roy S. Shin
   for CMU Mechatronics Window Washer S16

*/

/*
   Robot Control Scheme:
   wasd moves robot in x-y
   i raises feet in x-axis, k lowers feet
   o raises feet in y-axis, l lowers feet
   p prints pressure data
   invalid inputs stops all motion
*/

#include "FSM.h"
#include <NewPing.h>

volatile State state, nextState;

/* Limit Switch Pin Assignments */
const int pinLimLT = 19;
const int pinLimRT = 21;
const int pinLimUP = 20;
const int pinLimDN = 18;

/* Motor Driver 1 Pin Assignments */
const int X1 = 24;
const int X2 = 25;
const int Y1 = 22;
const int Y2 = 23;

/* Motor Driver 2 Pin Assignments */
const int L1 = 11;
const int L2 = 10;
const int R1 = 9;
const int R2 = 8;

/* Motor Driver 3 Pin Assignments */
const int U1 = 5;
const int U2 = 4;
const int D1 = 7;
const int D2 = 6;

/* Sensor Pin Assignments */
const int trigUP = 28;
const int echoUP = 29;
const int trigDN = 26;
const int echoDN = 27;
const int trigLT = 30;
const int echoLT = 31;
const int trigRT = 32;
const int echoRT = 33;
const int vacuum = A0;

/* Ultrasonic Sensor Declarations */
NewPing sonarUP(trigUP, echoUP, 8);
NewPing sonarDN(trigDN, echoDN, 8);
NewPing sonarLT(trigLT, echoLT, 8);
NewPing sonarRT(trigRT, echoRT, 8);

/* Global Variables */
volatile bool limitLT, limitRT, limitUP, limitDN;
long initial;
int count;

void setup() {
  Serial.begin(9600);

  state = stopALL;
  nextState = stopALL;

  pinMode(pinLimLT, INPUT);
  pinMode(pinLimRT, INPUT);
  pinMode(pinLimUP, INPUT);
  pinMode(pinLimDN, INPUT);

  digitalWrite(pinLimLT, HIGH);
  digitalWrite(pinLimRT, HIGH);
  digitalWrite(pinLimUP, HIGH);
  digitalWrite(pinLimDN, HIGH);

  attachInterrupt(digitalPinToInterrupt(pinLimLT), hitLT, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinLimRT), hitRT, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinLimUP), hitUP, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinLimDN), hitDN, FALLING);

  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(U1, OUTPUT);
  pinMode(U2, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(X1, OUTPUT);
  pinMode(X2, OUTPUT);
  pinMode(Y1, OUTPUT);
  pinMode(Y2, OUTPUT);

  stopAllMotors();
}

void loop() {
  int inByte;
  if (Serial.available() > 0) {
    inByte = Serial.read();
    switch (inByte) {
      case 97: // input character 'a'
        state = moveLT;
        break;
      case 100: // input character 'd'
        state = moveRT;
        break;
      case 119: // input character 'w'
        initial = sonarDN.ping_median(5);
        Serial.print("initial: ");
        Serial.print(initial);
        Serial.println("us");
        state = moveUPedge;
        nextState = stopALL;
        break;
      case 115: // input character 's'
        initial = sonarUP.ping_median(5);
        state = moveDNedge;
        nextState = stopALL;
        break;
      case 105: // input character 'i'
        state = switchXY;
        nextState = stopALL;
        break;
      case 111: // input character 'o'
        state = switchYX;
        nextState = stopALL;
        break;
      case 112: // input character 'p', display pressure data
        displayPressure();
        break;
      case 120: // x
        state = switchXY;
        nextState = moveUPedge;
        break;
      default:
        state = stopALL;
        break;
    }
  }

  long current;
  bool up, dn;

  switch (state) {
    case moveUP:
      if (digitalRead(pinLimUP) == HIGH) {
        limitUP = false;
      } else {
        limitUP = true;
      }

      if (!limitUP) {
        digitalWrite(Y1, LOW);
        digitalWrite(Y2, HIGH);
        limitDN = false;
        break;
      } else {
        digitalWrite(Y1, LOW);
        digitalWrite(Y2, LOW);
        break;
      }

    case moveUPedge:
      current = sonarDN.ping_median(5);
      Serial.print("current: ");
      Serial.print(current);
      Serial.println("us");
      if (digitalRead(pinLimUP) == HIGH) {
        limitUP = false;
      } else {
        limitUP = true;
      }

      if (!limitUP) { //&& (initial - current < 100) && (initial - current > -100)) {
        digitalWrite(Y1, LOW);
        digitalWrite(Y2, HIGH);
        limitDN = false;
        break;
      } else {
        digitalWrite(Y1, LOW);
        digitalWrite(Y2, LOW);
        state = nextState;
        nextState = stopALL;
        break;
      }

    case moveDN:
      if (digitalRead(pinLimDN) == HIGH) {
        limitDN = false;
      } else {
        limitDN = true;
      }

      if (!limitDN) {
        digitalWrite(Y1, HIGH);
        digitalWrite(Y2, LOW);
        limitUP = false;
        break;
      } else {
        digitalWrite(Y1, LOW);
        digitalWrite(Y2, LOW);
        break;
      }

    case moveDNedge:
      current = sonarUP.ping_median(5);
      Serial.print("current: ");
      Serial.print(current);
      Serial.println("us");
      if (digitalRead(pinLimDN) == HIGH) {
        limitDN = false;
      } else {
        limitDN = true;
      }

      if (!limitDN ) { //&& (initial - current < 100) && (initial - current > -100)) {
        digitalWrite(Y1, HIGH);
        digitalWrite(Y2, LOW);
        limitUP = false;
        break;
      } else {
        digitalWrite(Y1, LOW);
        digitalWrite(Y2, LOW);
        state = nextState;
        nextState = stopALL;
        break;
      }

    case moveLT:
      if (digitalRead(pinLimLT) == HIGH) {
        limitLT = false;
      } else {
        limitLT = true;
      }

      if (!limitLT) {
        digitalWrite(X1, HIGH);
        digitalWrite(X2, LOW);
        limitRT = false;
        break;
      } else {
        digitalWrite(X1, LOW);
        digitalWrite(X2, LOW);
        break;
      }

    case moveRT:
      if (digitalRead(pinLimRT) == HIGH) {
        limitRT = false;
      } else {
        limitRT = true;
      }

      if (!limitRT) {
        digitalWrite(X1, LOW);
        digitalWrite(X2, HIGH);
        limitLT = false;
        break;
      } else {
        digitalWrite(X1, LOW);
        digitalWrite(X2, LOW);
        break;
      }

    case switchXY:
      up = setUP(400);
      dn = setDN(400);
      if (up && dn) {
        state = nextState;
        nextState = switchYX;
      }
      break;

    case switchYX:
      up = setUP(235);
      dn = setDN(235);
      if (up && dn) {
        state = nextState;
        nextState = stopALL;
      }
      break;

    case stopALL:
      stopAllMotors();
      break;
  }
  delay(50);
}

void displayPressure() {
  int sensor = analogRead(A0);
  float voltage = sensor * (5.0 / 1023.0);
  float pressure = (voltage - 2.82) / 0.054;
  Serial.print("Pressure: ");
  Serial.println(pressure);
  return;
}

bool setUP(long target) {
  long echo = sonarUP.ping_median(5);
  long distance = sonarUP.convert_cm(echo);
  Serial.print("echoUP: ");
  Serial.print(echo);
  Serial.print("ms ");
  Serial.print("distanceUP: ");
  Serial.print(distance);
  Serial.println("cm");
  if (target - echo > 13) {
    digitalWrite(U1, HIGH);
    digitalWrite(U2, LOW);
    return false;
  } else if (target - echo < -18) {
    digitalWrite(U1, LOW);
    digitalWrite(U2, HIGH);
    return false;
  } else {
    digitalWrite(U1, LOW);
    digitalWrite(U2, LOW);
    return true;
  }
}

bool setDN(long target) {
  long echo = sonarDN.ping_median(5);
  long distance = sonarDN.convert_cm(echo);
  Serial.print("echoDN: ");
  Serial.print(echo);
  Serial.print("ms ");
  Serial.print("distanceDN: ");
  Serial.print(distance);
  Serial.println("cm");
  if (target - echo > 10) {
    digitalWrite(D1, HIGH);
    digitalWrite(D2, LOW);
    return false;
  } else if (target - echo < -20) {
    digitalWrite(D1, LOW);
    digitalWrite(D2, HIGH);
    return false;
  } else {
    digitalWrite(D1, LOW);
    digitalWrite(D2, LOW);
    return true;
  }
}

bool setLT(long target) {
  long echo = sonarLT.ping_median(3);
  long distance = sonarLT.convert_cm(echo);
  Serial.print("LT: ");
  Serial.print(distance);
  Serial.print("cm ");
  if (target > distance) {
    digitalWrite(L1, HIGH);
    digitalWrite(L2, LOW);
    return false;
  } else if (target < distance) {
    digitalWrite(L1, LOW);
    digitalWrite(L2, HIGH);
    return false;
  } else {
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
    return true;
  }
}

bool setRT(long target) {
  long echo = sonarRT.ping_median(3);
  long distance = sonarRT.convert_cm(echo);
  Serial.print(" RT: ");
  Serial.print(distance);
  Serial.println("cm ");
  if (target > distance) {
    digitalWrite(R1, HIGH);
    digitalWrite(R2, LOW);
    return false;
  } else if (target < distance) {
    digitalWrite(R1, LOW);
    digitalWrite(R2, HIGH);
    return false;
  } else {
    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
    return true;
  }
}

void stopAllMotors() {
  digitalWrite(L1, LOW);
  digitalWrite(L2, LOW);
  digitalWrite(R1, LOW);
  digitalWrite(R2, LOW);
  digitalWrite(U1, LOW);
  digitalWrite(U2, LOW);
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  digitalWrite(X1, LOW);
  digitalWrite(X2, LOW);
  digitalWrite(Y1, LOW);
  digitalWrite(Y2, LOW);
  return;
}

void hitLT() {
  digitalWrite(X1, LOW);
  digitalWrite(X2, LOW);
  state = stopALL;
  limitLT = true;
  return;
}

void hitRT() {
  digitalWrite(X1, LOW);
  digitalWrite(X2, LOW);
  state = stopALL;
  limitRT = true;
  return;
}

void hitUP() {
  digitalWrite(Y1, LOW);
  digitalWrite(Y2, LOW);
  limitUP = true;
  return;
}

void hitDN() {
  digitalWrite(Y1, LOW);
  digitalWrite(Y2, LOW);
  limitDN = true;
  return;
}
