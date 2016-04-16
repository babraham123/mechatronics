/*
   Movement: WASD, x to center Y
   X legs: i to raise, k to lower
   Y legs: o to raise, l to lower
   Pressure: p to display pressure

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

const int yValve = 40;
const int xValve = 41;

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
const int S_DISPLAY = 10;
const int S_WAITVACX = 11;
const int S_WAITVACY = 12;

const long hiThresh = 10;
const long loThresh = -25;
const long loTarget = 245;
const long hiTarget = 265;

/* Global Variables */
volatile bool limitLT, limitRT, limitUP, limitDN;
bool printStatus;
bool movingRight;
bool stateEnd;
long initial, current;
volatile int yExtreme, yPosition;

void setup() {
  Serial.begin(9600);

  movingRight = true;
  printStatus = false;

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

  pinMode(yValve, OUTPUT);
  pinMode(xValve, OUTPUT);

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
        next = S_WAITVACX;
        break;

      case 101: // 'e'
        prev = curr;
        curr = S_DISPLAY;
        next = S_STOPALL;
        break;

      case 113: // 'q'
        printStatus = !printStatus;
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
            next = S_WAITVACX;
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
            next = S_WAITVACY;
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

    case S_CENTERY:
      stateEnd = moveMidY();
      if (stateEnd) {
        prev = curr;
        curr = next;
        next = S_STOPALL;
      }
      break;

    case S_LIFT_X:
      digitalWrite(xValve, HIGH);
      stateEnd = liftCupsX();
      if (stateEnd) {
        digitalWrite(xValve, LOW);
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
      digitalWrite(xValve, LOW);
      stateEnd = lowerCupsX();
      if (stateEnd) {
        prev = curr;
        curr = next;
        switch (next) {
          case S_STOPALL:
            next = S_STOPALL;
            break;

          case S_WAITVACX:
            next = S_LIFT_Y;
            break;
        }
      }
      break;

    case S_LIFT_Y:
      digitalWrite(yValve, HIGH);
      stateEnd = liftCupsY();
      if (stateEnd) {
        digitalWrite(yValve, LOW);
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
      digitalWrite(yValve, LOW);
      stateEnd = lowerCupsY();
      if (stateEnd) {
        prev = curr;
        curr = next;
        switch (next) {
          case S_STOPALL:
            next = S_STOPALL;
            break;

          case S_WAITVACY:
            next = S_LIFT_X;
            break;
        }
      }
      break;

    case S_DISPLAY:
      displayUltrasonic();
      break;

    case S_STOPALL:
      stopAllMotors();
      prev = curr;
      curr = next;
      next = S_STOPALL;
      break;

    case S_WAITVACX:
      stateEnd = waitVacuumX();
      if (stateEnd) {
        prev = curr;
        curr = next;
        next = S_MOVERT;
      }
      break;

    case S_WAITVACY:
      stateEnd = waitVacuumY();
      if (stateEnd) {
        prev = curr;
        curr = next;
        next = S_MOVELT;
      }
      break;
  }
  delay(35);
}

bool waitVacuumX() {
  int sensor = analogRead(A1);
  float voltage = sensor * (5.0 / 1023.0);
  float pressure = (voltage - 2.82) / 0.054;
  if (pressure < 0) {
    return true;
  } else {
    return false;
  }
}

bool waitVacuumY() {
  int sensor = analogRead(A0);
  float voltage = sensor * (5.0 / 1023.0);
  float pressure = (voltage - 2.82) / 0.054;
  if (pressure < 0) {
    return true;
  } else {
    return false;
  }
}

void displayPressure() {
  int sensor1 = analogRead(A0);
  int sensor2 = analogRead(A1);
  float voltage1 = sensor1 * (5.0 / 1023.0);
  float pressure1 = (voltage1 - 2.82) / 0.054;
  float voltage2 = sensor2 * (5.0 / 1023.0);
  float pressure2 = (voltage2 - 2.82) / 0.054;
  Serial.print("PressureY: ");
  Serial.print(pressure1);
  Serial.print(" PressureX: ");
  Serial.println(pressure2);
  return;
}

void displayUltrasonic() {
  long echoLT = sonarLT.ping_median(5);
  long echoRT = sonarRT.ping_median(5);
  long echoUP = sonarUP.ping_median(5);
  long echoDN = sonarDN.ping_median(5);
  Serial.print("LT=");
  Serial.print(echoLT);
  Serial.print(" RT=");
  Serial.print(echoRT);
  Serial.print(" UP=");
  Serial.print(echoUP);
  Serial.print(" DN=");
  Serial.println(echoDN);
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
  int target = yExtreme / 2;
  yPosition = yEncoder.read();

  if (target - yPosition > 0) {
    digitalWrite(Y1, LOW);
    digitalWrite(Y2, HIGH);
    return false;
  } else if (target - yPosition < 0) {
    digitalWrite(Y1, HIGH);
    digitalWrite(Y2, LOW);
    return false;
  } else {
    digitalWrite(Y1, LOW);
    digitalWrite(Y2, LOW);
    return true;
  }
}

bool lowerCupsX() {
  bool left, right;

  long echoLT = sonarLT.ping_median(2);
  long echoRT = sonarRT.ping_median(2);

  if (printStatus) {
    Serial.print("LT:");
    Serial.print(echoLT);
    Serial.print("us RT:");
    Serial.print(echoRT);
    Serial.println("us");
  }

  if ((echoLT < 250) && (echoRT > 255)) {
    analogWrite(L1, 255 / 16);
    digitalWrite(L2, HIGH);
    analogWrite(R1, 255 / 4);
    digitalWrite(R2, LOW);
    return false;
  } else if (echoLT > 265) {
    analogWrite(L1, 255 / 16);
    digitalWrite(L2, HIGH);
    left = false;
  } else {
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
    left = true;
  }

  if ((echoRT < 250) && (echoLT > 255)) {
    analogWrite(L1, 255 / 4);
    digitalWrite(L2, LOW);
    analogWrite(R1, 255 / 16);
    digitalWrite(R2, HIGH);
    return false;
  } else if (echoRT > 265) {
    analogWrite(R1, 255 / 16);
    digitalWrite(R2, HIGH);
    right = false;
  } else {
    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
    right = true;
  }
  return (left && right);
}

bool liftCupsX() {
  bool left, right;

  long echoLT = sonarLT.ping_median(2);
  long echoRT = sonarRT.ping_median(2);

  if (printStatus) {
    Serial.print("LT:");
    Serial.print(echoLT);
    Serial.print("us RT:");
    Serial.print(echoRT);
    Serial.println("us");
  }

  if (echoLT - echoRT > 20) {
    analogWrite(L1, 255 / 4);
    digitalWrite(L2, LOW);
    analogWrite(R1, 255 / 4);
    digitalWrite(R2, HIGH);
    return false;
  } else if (echoLT < 265) {
    analogWrite(L1, 255 / 2);
    digitalWrite(L2, LOW);
    left = false;
  } else {
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
    left = true;
  }

  if (echoRT - echoLT > 20) {
    analogWrite(L1, 255 / 4);
    digitalWrite(L2, HIGH);
    analogWrite(R1, 255 / 4);
    digitalWrite(R2, LOW);
    return false;
  }
  else if (echoRT < 265) {
    analogWrite(R1, 255 / 2);
    digitalWrite(R2, LOW);
    right = false;
  } else {
    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
    right = true;
  }
  return (left && right);
}

bool lowerCupsY() {
  bool up, down;

  long echoUP = sonarUP.ping_median(2);
  long echoDN = sonarDN.ping_median(2);

  if (printStatus) {
    Serial.print("UP:");
    Serial.print(echoUP);
    Serial.print("us DN:");
    Serial.print(echoDN);
    Serial.println("us");
  }

  if ((echoUP < 230) && (echoDN > 230)) {
    analogWrite(U1, 255 / 16);
    digitalWrite(U2, HIGH);
    analogWrite(D1, 255 / 4);
    digitalWrite(D2, LOW);
    return false;
  } else if (echoUP > 250) {
    analogWrite(U1, 255 / 16);
    digitalWrite(U2, HIGH);
    up = false;
  } else {
    digitalWrite(U1, LOW);
    digitalWrite(U2, LOW);
    up = true;
  }

  if ((echoDN < 230) && (echoUP > 230)) {
    analogWrite(U1, 255 / 4);
    digitalWrite(U2, LOW);
    analogWrite(D1, 255 / 16);
    digitalWrite(D2, HIGH);
    return false;
  } else if (echoDN > 250) {
    analogWrite(D1, 255 / 16);
    digitalWrite(D2, HIGH);
    down = false;
  } else {
    digitalWrite(D1, LOW);
    digitalWrite(D2, LOW);
    down = true;
  }
  return (up && down);
}

bool liftCupsY() {
  bool up, down;

  long echoUP = sonarUP.ping_median(2);
  long echoDN = sonarDN.ping_median(2);

  if (printStatus) {
    Serial.print("UP:");
    Serial.print(echoUP);
    Serial.print("us DN:");
    Serial.print(echoDN);
    Serial.println("us");
  }

  if (echoUP < 270) {
    analogWrite(U1, 255 / 2);
    digitalWrite(U2, LOW);
    up = false;
  } else {
    digitalWrite(U1, LOW);
    digitalWrite(U2, LOW);
    up = true;
  }

  if (echoDN < 270) {
    analogWrite(D1, 255 / 2);
    digitalWrite(D2, LOW);
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
