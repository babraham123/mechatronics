/*
   X-Y Control of Robot w/o Rotary Encoder
   by Roy S. Shin
   for CMU Mechatronics Window Washer S16
*/

/*
   Robot Control Scheme:
   wasd moves robot in x-y
   i raises feet in x-axis, k lowers feet
   o raises feet in y-axis, l lowers feet
   invalid inputs stops all x-y motion
*/

#include "FSM.h"
State state;

/* Pin Assignments */
const int pinLimLT = 18;
const int pinLimRT = 19;
const int pinLimUP = 20;
const int pinLimDN = 21;

const int X1 = 24;
const int X2 = 25;
const int Y1 = 22;
const int Y2 = 23;

const int L1 = 9;
const int L2 = 8;
const int R1 = 11;
const int R2 = 10;

const int U1 = 5;
const int U2 = 4;
const int D1 = 7;
const int D2 = 6;

/* Global Variables */
bool limitLT, limitRT, limitUP, limitDN;

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
    Serial.println(inByte);
    switch (inByte) {
      case 97: // input character 'a'
        state = moveLT;
        break;
      case 100: // input character 'd'
        state = moveRT;
        break;
      case 119: // input character 'w'
        state = moveUP;
        break;
      case 115: // input character 's'
        state = moveDN;
        break;
      case 105: // input character 'i'
        state = liftX;
        break;
      case 107: // input character 'k'
        state = lowerX;
        break;
      case 111: // input character 'o'
        state = liftY;
        break;
      case 108: // input character 'l'
        state = lowerY;
        break;
      default:
        state = stopALL;
        break;
    }
  }

  switch (state) {
    case moveUP:
      if (digitalRead(pinLimUP) == HIGH) {
        limitUP = false;
      } else {
        limitUP = true;
      }
      if (!limitUP) {
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

    case liftX:
      digitalWrite(L1, HIGH);
      digitalWrite(L2, LOW);
      digitalWrite(R1, HIGH);
      digitalWrite(R2, LOW);
      delay(500);
      digitalWrite(L1, LOW);
      digitalWrite(L2, LOW);
      digitalWrite(R1, LOW);
      digitalWrite(R2, LOW);
      state = stopALL;
      break;

    case lowerX:
      digitalWrite(L1, LOW);
      digitalWrite(L2, HIGH);
      digitalWrite(R1, LOW);
      digitalWrite(R2, HIGH);
      delay(500);
      digitalWrite(L1, LOW);
      digitalWrite(L2, LOW);
      digitalWrite(R1, LOW);
      digitalWrite(R2, LOW);
      state = stopALL;
      break;

    case liftY:
      digitalWrite(U1, HIGH);
      digitalWrite(U2, LOW);
      digitalWrite(D1, HIGH);
      digitalWrite(D2, LOW);
      delay(500);
      digitalWrite(U1, LOW);
      digitalWrite(U2, LOW);
      digitalWrite(D1, LOW);
      digitalWrite(D2, LOW);
      state = stopALL;
      break;

    case lowerY:
      digitalWrite(U1, LOW);
      digitalWrite(U2, HIGH);
      digitalWrite(D1, LOW);
      digitalWrite(D2, HIGH);
      delay(500);
      digitalWrite(U1, LOW);
      digitalWrite(U2, LOW);
      digitalWrite(D1, LOW);
      digitalWrite(D2, LOW);
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
