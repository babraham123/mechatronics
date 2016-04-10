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
   invalid inputs stops all motion
*/

#include "FSM.h"
#include <NewPing.h>
volatile State state;

/* Limit Switch Pin Assignments */
const int pinLimLT = 18;
const int pinLimRT = 19;
const int pinLimUP = 20;
const int pinLimDN = 21;

/* Motor Driver 1 Pin Assignments */
const int X1 = 24;
const int X2 = 25;
const int Y1 = 22;
const int Y2 = 23;

/* Motor Driver 2 Pin Assignments */
const int L1 = 9;
const int L2 = 8;
const int R1 = 11;
const int R2 = 10;

/* Motor Driver 3 Pin Assignments */
const int U1 = 5;
const int U2 = 4;
const int D1 = 7;
const int D2 = 6;

/* Sensor Pin Assignments */
const int trigUP = 50;
const int echoUP = 51;
const int trigDN = 26;
const int echoDN = 27;
const int trigLT = 28;
const int echoLT = 29;
const int trigRT = 52;
const int echoRT = 53;
const int vacuum = A0;

/* Ultrasonic Sensor Declarations */
NewPing sonarUP(trigUP, echoUP, 50);
NewPing sonarDN(trigDN, echoDN, 50);
NewPing sonarLT(trigLT, echoLT, 50);
NewPing sonarRT(trigRT, echoRT, 50);

/* Global Variables */
volatile bool limitLT, limitRT, limitUP, limitDN;

int upHeight, dnHeight, ltHeight, rtHeight;

void setup() {
  Serial.begin(9600);

  state = stopALL;

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
        upHeight = sonarUP.ping_median(10);
        state = moveUP;
        break;
      case 115: // input character 's'
        state = moveDN;
        break;
      case 105: // input character 'i'
        state = switchXY;
        break;
      case 111: // input character 'o'
        state = switchYX;
        break;
      default:
        state = stopALL;
        break;
    }
  }

  int upHeightOffset;

  switch (state) {
    case moveUP:
      upHeightOffset = sonarUP.ping_median(10);
      if (digitalRead(pinLimUP) == HIGH) {
        limitUP = false;
      } else {
        limitUP = true;
      }

      if (!limitUP && (upHeight - upHeightOffset > 50)) {
        digitalWrite(Y1, HIGH);
        digitalWrite(Y2, LOW);
        limitDN = false;
        break;
      } else {
        digitalWrite(Y1, LOW);
        digitalWrite(Y2, LOW);
        break;
      }

    case moveDN:
      if (digitalRead(pinLimDN) == HIGH) {
        limitDN = false;
      } else {
        limitDN = true;
      }

      if (!limitDN) {
        digitalWrite(Y1, LOW);
        digitalWrite(Y2, HIGH);
        limitUP = false;
        break;
      } else {
        digitalWrite(Y1, LOW);
        digitalWrite(Y2, LOW);
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
      lowerCupsY();
      liftCupsX();
      state = stopALL;
      break;

    case switchYX:
      lowerCupsX();
      liftCupsY();
      state = stopALL;
      break;

    case stopALL:
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
  }
}

void hitLT() {
  digitalWrite(X1, LOW);
  digitalWrite(X2, LOW);
  state = stopALL;
  limitLT = true;
}

void hitRT() {
  digitalWrite(X1, LOW);
  digitalWrite(X2, LOW);
  state = stopALL;
  limitRT = true;
}

void hitUP() {
  digitalWrite(Y1, LOW);
  digitalWrite(Y2, LOW);
  state = stopALL;
  limitUP = true;
}

void hitDN() {
  digitalWrite(Y1, LOW);
  digitalWrite(Y2, LOW);
  state = stopALL;
  limitDN = true;
}

void liftCupsX() {
  int ltEcho = sonarLT.ping_median(2);
  int rtEcho = sonarRT.ping_median(2);
  Serial.print("lt: ");
  Serial.println(ltEcho);
  Serial.print("rt: ");
  Serial.println(rtEcho);
  while (ltEcho < 325 || rtEcho < 325) {
    if (ltEcho < 325) {
      digitalWrite(L1, HIGH);
      digitalWrite(L2, LOW);
      ltEcho = sonarLT.ping_median(2);
    } else {
      digitalWrite(L1, LOW);
      digitalWrite(L2, LOW);
    }
    if (rtEcho < 325) {
      digitalWrite(R1, HIGH);
      digitalWrite(R2, LOW);
      rtEcho = sonarRT.ping_median(2);
    } else {
      digitalWrite(R1, LOW);
      digitalWrite(R2, LOW);
    }
  }
  digitalWrite(L1, LOW);
  digitalWrite(L2, LOW);
  digitalWrite(R1, LOW);
  digitalWrite(R2, LOW);
  Serial.print("lt: ");
  Serial.println(ltEcho);
  Serial.print("rt: ");
  Serial.println(rtEcho);
  return;
}

void liftCupsY() {
  int upEcho = sonarUP.ping_median(2);
  int dnEcho = sonarDN.ping_median(2);
  Serial.print("up: ");
  Serial.print(upEcho);
  Serial.print(" dn: ");
  Serial.println(dnEcho);
  while (upEcho < 325 || dnEcho < 325) {
    if (upEcho < 325) {
      digitalWrite(U1, HIGH);
      digitalWrite(U2, LOW);
      upEcho = sonarUP.ping_median(2);
    } else {
      digitalWrite(U1, LOW);
      digitalWrite(U2, LOW);
    }
    if (dnEcho < 325) {
      digitalWrite(D1, HIGH);
      digitalWrite(D2, LOW);
      dnEcho = sonarDN.ping_median(2);
    } else {
      digitalWrite(D1, LOW);
      digitalWrite(D2, LOW);
    }
  }
  digitalWrite(U1, LOW);
  digitalWrite(U2, LOW);
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  Serial.print("up: ");
  Serial.print(upEcho);
  Serial.print(" dn: ");
  Serial.println(dnEcho);
  return;
}

void lowerCupsX() {
  int ltEcho = sonarLT.ping_median(2);
  int rtEcho = sonarRT.ping_median(2);
  Serial.print("lt: ");
  Serial.println(ltEcho);
  Serial.print("rt: ");
  Serial.println(rtEcho);
  while (ltEcho > 235 || rtEcho > 235) {
    if (ltEcho > 235) {
      digitalWrite(L1, LOW);
      digitalWrite(L2, HIGH);
      ltEcho = sonarLT.ping_median(2);
    } else {
      digitalWrite(L1, LOW);
      digitalWrite(L2, LOW);
    }
    if (rtEcho > 235) {
      digitalWrite(R1, LOW);
      digitalWrite(R2, HIGH);
      rtEcho = sonarRT.ping_median(2);
    } else {
      digitalWrite(R1, LOW);
      digitalWrite(R2, LOW);
    }
    Serial.print("lt: ");
    Serial.println(ltEcho);
    Serial.print("rt: ");
    Serial.println(rtEcho);
  }
  digitalWrite(L1, LOW);
  digitalWrite(L2, LOW);
  digitalWrite(R1, LOW);
  digitalWrite(R2, LOW);
  Serial.print("lt: ");
  Serial.println(ltEcho);
  Serial.print("rt: ");
  Serial.println(rtEcho);
  return;
}

void lowerCupsY() {
  int upEcho = sonarUP.ping_median(2);
  int dnEcho = sonarDN.ping_median(2);
  Serial.print("up: ");
  Serial.print(upEcho);
  Serial.print(" dn: ");
  Serial.println(dnEcho);
  while (upEcho > 235 || dnEcho > 235) {
    if (upEcho > 235) {
      digitalWrite(U1, LOW);
      digitalWrite(U2, HIGH);
      upEcho = sonarUP.ping_median(2);
    } else {
      digitalWrite(U1, LOW);
      digitalWrite(U2, LOW);
    }
    if (dnEcho > 235) {
      digitalWrite(D1, LOW);
      digitalWrite(D2, HIGH);
      dnEcho = sonarDN.ping_median(2);
    } else {
      digitalWrite(D1, LOW);
      digitalWrite(D2, LOW);
    }
  }
  digitalWrite(U1, LOW);
  digitalWrite(U2, LOW);
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  Serial.print("up: ");
  Serial.print(upEcho);
  Serial.print(" dn: ");
  Serial.println(dnEcho);
  return;
}