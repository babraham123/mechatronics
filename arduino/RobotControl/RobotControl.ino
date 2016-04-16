/*
 * Movement: WASD, x to center Y
 * X legs: i to raise, k to lower
 * Y legs: o to raise, l to lower
 * Pressure: p to display pressure
 * 
 */

#include <Encoder.h>
#include <NewPing.h>

const int pinLimLT = 19;
const int pinLimRT = 21;
const int pinLimUP = 20;
const int pinLimDN = 18;

const int X1 = 22;
const int X2 = 23;
const int Y1 = 24;
const int Y2 = 25;

const int L1 = 9;
const int L2 = 8;
const int R1 = 11;
const int R2 = 10;

const int U1 = 5;
const int U2 = 4;
const int D1 = 7;
const int D2 = 6;

const int trigLT = 52;
const int echoLT = 53;
const int trigRT = 50;
const int echoRT = 51;
const int trigUP = 28;
const int echoUP = 29;
const int trigDN = 26;
const int echoDN = 27;

NewPing sonarLT(trigLT, echoLT, 8);
NewPing sonarRT(trigRT, echoRT, 8);
NewPing sonarUP(trigUP, echoUP, 8);
NewPing sonarDN(trigDN, echoDN, 8);

Encoder yEncoder(3, 2);

volatile int prev, curr, next;
const int S_MOVELT = 0;
const int S_MOVERT = 1;
const int S_MOVEUP = 2;
const int S_MOVEDN = 3;
const int S_LOWERX = 4;
const int S_LIFT_X = 5;
const int S_LOWERY = 6;
const int S_LIFT_Y = 7;
const int S_STOPALL = 8;
const int S_CENTERY = 9;

const long loThresh = 5;
const long hiThresh = -20;
const long loTarget = 250;
const long hiTarget = 320;

/* Global Variables */
volatile bool limitLT, limitRT, limitUP, limitDN;
bool movingRight;
bool stateEnd;
long initial, current;
volatile int yExtreme, yPosition;

void setup() {
  Serial.begin(9600);

  movingRight = true;

  prev = S_STOPALL;
  curr = S_STOPALL;
  next = S_STOPALL;

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
      case 97: // 'a'
        prev = curr;
        curr = S_MOVELT;
        next = S_STOPALL;
        break;

      case 100: // 'd'
        prev = curr;
        curr = S_MOVERT;
        next = S_STOPALL;
        break;

      case 119: // 'w'
        prev = curr;
        curr = S_MOVEUP;
        next = S_STOPALL;
        break;

      case 115: // 's'
        prev = curr;
        curr = S_MOVEDN;
        next = S_STOPALL;
        break;

      case 105: // 'i'
        prev = curr;
        curr = S_LIFT_X;
        next = S_STOPALL;
        break;

      case 107: // 'k'
        prev = curr;
        curr = S_LOWERX;
        next = S_STOPALL;
        break;

      case 111: // 'o'
        prev = curr;
        curr = S_LIFT_Y;
        next = S_STOPALL;
        break;

      case 108: // 'l'
        prev = curr;
        curr = S_LOWERY;
        next = S_STOPALL;
        break;

      case 112: // 'p'
        displayPressure();
        break;

      case 120: // 'x'
        prev = curr;
        curr = S_CENTERY;
        next = S_STOPALL;
        break;

      case 122: // 'z'
        prev = curr;
        curr = S_LOWERX;
        next = S_LIFT_Y;
        break;

      default:
        prev = curr;
        curr = S_STOPALL;
        next = S_STOPALL;
        break;
    }
  }

  switch (curr) {
    case S_MOVELT:
      stateEnd = moveLT();
      if (stateEnd) {
        prev = curr;
        curr = next;
        switch (next) {
          case S_STOPALL:
            next = S_STOPALL;
            break;

          case S_LOWERX:
            next = S_LIFT_Y;
            break;
        }
      }
      break;

    case S_MOVERT:
      stateEnd = moveRT();
      if (stateEnd) {
        prev = curr;
        curr = next;
        switch (next) {
          case S_STOPALL:
            next = S_STOPALL;
            break;

          case S_LOWERY:
            next = S_LIFT_X;
            break;
        }
      }
      break;

    case S_MOVEUP:
      stateEnd = moveUP();
      if (stateEnd) {
        prev = curr;
        curr = next;
        next = S_STOPALL;
      }
      break;

    case S_MOVEDN:
      stateEnd = moveDN();
      if (stateEnd) {
        prev = curr;
        curr = next;
        next = S_STOPALL;
      }
      break;

    case S_LIFT_X:
      stateEnd = setX(hiTarget);
      if (stateEnd) {
        prev = curr;
        curr = next;
        switch (next) {
          case S_STOPALL:
            next = S_STOPALL;
            break;

          case S_MOVELT:
            next = S_LOWERX;
            break;
        }
      }
      break;

    case S_LOWERX:
      stateEnd = setX(loTarget);
      if (stateEnd) {
        prev = curr;
        curr = next;
        switch (next) {
          case S_STOPALL:
            next = S_STOPALL;
            break;

          case S_LIFT_Y:
            next = S_MOVERT;
            break;
        }
      }
      break;

    case S_LIFT_Y:
      stateEnd = setY(hiTarget);
      if (stateEnd) {
        prev = curr;
        curr = next;
        switch (next) {
          case S_STOPALL:
            next = S_STOPALL;
            break;

          case S_MOVERT:
            next = S_LOWERY;
            break;
        }
      }
      break;

    case S_LOWERY:
      stateEnd = setY(loTarget);
      if (stateEnd) {
        prev = curr;
        curr = next;
        switch (next) {
          case S_STOPALL:
            next = S_STOPALL;
            break;

          case S_LIFT_X:
            next = S_MOVELT;
            break;
        }
      }
      break;

    case S_STOPALL:
      stopAllMotors();
      prev = curr;
      curr = next;
      next = S_STOPALL;
      break;
  }
  delay(50);
}

void displayPressure() {
  int sensor1 = analogRead(A0);
  int sensor2 = analogRead(A1);
  float voltage1 = sensor1 * (5.0 / 1023.0);
  float pressure1 = (voltage1 - 2.82) / 0.054;
  float voltage2 = sensor2 * (5.0 / 1023.0);
  float pressure2 = (voltage2 - 2.82) / 0.054;
  Serial.print("Pressure1: ");
  Serial.print(pressure1);
  Serial.print(" Pressure2: ");
  Serial.print(pressure2);
  return;
}

bool moveLT() {
  if (digitalRead(pinLimLT) == HIGH) {
    digitalWrite(X1, HIGH);
    digitalWrite(X2, LOW);
    return false;
  } else {
    digitalWrite(X1, LOW);
    digitalWrite(X2, LOW);
    return true;
  }
}

bool moveRT() {
  if (digitalRead(pinLimRT) == HIGH) {
    digitalWrite(X1, LOW);
    digitalWrite(X2, HIGH);
    return false;
  } else {
    digitalWrite(X1, LOW);
    digitalWrite(X2, LOW);
    return true;
  }
}

bool moveUP() {
  if (digitalRead(pinLimUP) == HIGH) {
    digitalWrite(Y1, LOW);
    digitalWrite(Y2, HIGH);
    return false;
  } else {
    digitalWrite(Y1, LOW);
    digitalWrite(Y2, LOW);
    return true;
  }
}

bool moveDN() {
  if (digitalRead(pinLimDN) == HIGH) {
    digitalWrite(Y1, HIGH);
    digitalWrite(Y2, LOW);
    return false;
  } else {
    digitalWrite(Y1, LOW);
    digitalWrite(Y2, LOW);
    return true;
  }
}

bool moveMidY() {
  int target = yExtreme/2 -1;
  yPosition = yEncoder.read();

  if (target - yPosition > 1) {
    digitalWrite(Y1, LOW);
    digitalWrite(Y2, HIGH);
    return false;
  } else if (target - yPosition < -1) {
    digitalWrite(Y1, HIGH);
    digitalWrite(Y2, LOW);
    return false;
  } else {
    digitalWrite(Y1, LOW);
    digitalWrite(Y2, LOW);
    return true;
  }
}

bool setX(long target) {
  bool left, right;
  long echoLT = sonarLT.ping_median(5);
  long echoRT = sonarRT.ping_median(5);

  if (target - echoLT > loThresh) {
    digitalWrite(L1, HIGH);
    digitalWrite(L2, LOW);
    left = false;
  } else if (target - echoLT < hiThresh) {
    digitalWrite(L1, LOW);
    digitalWrite(L2, HIGH);
    left = false;
  } else {
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
    left = true;
  }

  if (target - echoRT > loThresh) {
    digitalWrite(R1, HIGH);
    digitalWrite(R2, LOW);
    right = false;
  } else if (target - echoRT < hiThresh) {
    digitalWrite(R1, LOW);
    digitalWrite(R2, HIGH);
    right = false;
  } else {
    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
    right = true;
  }

  return (left && right);
}

bool setY(long target) {
  bool up, down;
  long echoUP = sonarUP.ping_median(5);
  long echoDN = sonarDN.ping_median(5);

  if (target - echoUP > loThresh) {
    digitalWrite(U1, HIGH);
    digitalWrite(U2, LOW);
    up = false;
  } else if (target - echoUP < hiThresh) {
    digitalWrite(U1, LOW);
    digitalWrite(U2, HIGH);
    up = false;
  } else {
    digitalWrite(U1, LOW);
    digitalWrite(U2, LOW);
    up = true;
  }

  if (target - echoDN > loThresh) {
    digitalWrite(D1, HIGH);
    digitalWrite(D2, LOW);
    down = false;
  } else if (target - echoDN < hiThresh) {
    digitalWrite(D1, LOW);
    digitalWrite(D2, HIGH);
    down = false;
  } else {
    digitalWrite(D1, LOW);
    digitalWrite(D2, LOW);
    down = true;
  }

  return (up && down);
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
  return;
}

void hitRT() {
  digitalWrite(X1, LOW);
  digitalWrite(X2, LOW);
  return;
}

void hitUP() {
  digitalWrite(Y1, LOW);
  digitalWrite(Y2, LOW);
  yExtreme = yEncoder.read();
  return;
}

void hitDN() {
  digitalWrite(Y1, LOW);
  digitalWrite(Y2, LOW);
  yEncoder.write(0);
  return;
}
