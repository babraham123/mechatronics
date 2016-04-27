/* Icarus Window Washer, V6
   Mechatronic Design
   Roy Shin, Bereket Abraham
*/

/* Manual Control Scheme

   Movement: WASD, x/y centers X/Y
   X legs: i to raise, k to lower
   Y legs: o to raise, l to lower
   Pressure: p to display pressure
*/

#include <NewPing.h>

/********** Pin Definitions ***********/
const int pinLimLT = 19;
const int pinLimRT = 21;
const int pinLimUP = 20;
const int pinLimDN = 18;

const int pinLimN = 2;
const int pinLimS = 3;

const int X1 = 22;
const int X2 = 23;
const int Y1 = 24;
const int Y2 = 25;

const int L1 = 11;
const int L2 = 10;
const int R1 = 13;
const int R2 = 12;

const int U1 = 7;
const int U2 = 6;
const int D1 = 9;
const int D2 = 8;

const int trigLT = 52;
const int echoLT = 53;
const int trigRT = 50;
const int echoRT = 51;
const int trigUP = 28;
const int echoUP = 29;
const int trigDN = 26;
const int echoDN = 27;

const int trigWEST = 30;
const int echoWEST = 31;
const int trigNORTH = 32;
const int echoNORTH = 33;

const int xPump = 45;
const int yPump = 44;

const int yVacuum = A0;
const int xVacuum = A1;

NewPing sonarLT(trigLT, echoLT, 7);
NewPing sonarRT(trigRT, echoRT, 7);
NewPing sonarUP(trigUP, echoUP, 7);
NewPing sonarDN(trigDN, echoDN, 7);
NewPing sonarWEST(trigWEST, echoWEST, 15);
NewPing sonarNORTH(trigNORTH, echoNORTH, 15);

const float pressureThreshold = -1.0;
const bool longCupsVertical = true;
// records the orientation of the robot
// true means that the X railing (sponge holders) is vertical
// true when the divider is vertical

/********** States ***********/
// Performs actions and state transitions. State can override its manager
// in case of failure / unexpected conditions. State can transition to a
// different manager as well.
int currState;
unsigned long stateCounter; // # of cycles since currState started
const int S_MOVE_LT = 0;
const int S_MOVE_RT = 1;
const int S_MOVE_UP = 2;
const int S_MOVE_DN = 3;
const int S_LOWER_X = 4;
const int S_LIFT_X = 5;
const int S_LIFT_X_HI = 6;
const int S_LOWER_Y = 7;
const int S_LIFT_Y = 8;
const int S_LIFT_Y_HI = 9;
const int S_STOP_ALL = 10;
const int S_CENTER_X = 11;
const int S_CENTER_Y = 12;
const int S_WAIT_VAC_X = 13;
const int S_WAIT_VAC_Y = 14;
const int S_SET_CUPS = 15;

/********** Managers ***********/
// A sequence of states that repeats in a loop. Manager can be started in
// the middle of a sequence
int *currManager;
int currIndex;
// linear motion
int M_TRANSLATE_LT[10] = {S_CENTER_Y, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X, S_MOVE_RT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y, S_MOVE_LT, -1}; // loops back around
int M_TRANSLATE_RT[10] = {S_WAIT_VAC_Y, S_LIFT_X, S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y, S_MOVE_RT, S_CENTER_Y, S_LOWER_Y, -2};
int M_TRANSLATE_UP[10] = {S_CENTER_X, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y, S_MOVE_DN, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X, S_MOVE_UP, -3};
int M_TRANSLATE_DN[10] = {S_CENTER_X, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X, S_MOVE_DN, -4};
int M_DEBUG_LT[7] = {S_LOWER_Y, S_LIFT_X, S_MOVE_RT, S_LOWER_X, S_LIFT_Y, S_MOVE_LT, -5};

// turns, robot zigzags against divider
// when moving down, if barrier transition to linear motion
int M_TURN_X_LR[35] = {S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y, S_CENTER_X, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X,
                       S_MOVE_DN, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X, S_MOVE_DN,
                       S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X, S_MOVE_DN, S_LOWER_X,
                       S_WAIT_VAC_X, S_LIFT_Y, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X, S_CENTER_Y, -2
                      }; // M_TRANSLATE_RT

int M_TURN_X_RL[35] = {S_MOVE_RT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y, S_CENTER_X, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X,
                       S_MOVE_DN, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X, S_MOVE_DN,
                       S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X, S_MOVE_DN, S_LOWER_X,
                       S_WAIT_VAC_X, S_LIFT_Y, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X, S_CENTER_Y, -1
                      }; // M_TRANSLATE_LT

// turns, robot zigzags with divider (not used)
int M_TURN_Y_LR[35] = {S_MOVE_DN, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X, S_CENTER_Y, S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y,
                       S_MOVE_RT, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X, S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y, S_MOVE_RT,
                       S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X, S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y, S_MOVE_RT, S_LOWER_Y,
                       S_WAIT_VAC_Y, S_LIFT_X, S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y, S_CENTER_X, -3
                      }; // M_TRANSLATE_UP

int M_TURN_Y_RL[35] = {S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X, S_CENTER_Y, S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y,
                       S_MOVE_RT, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X, S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y, S_MOVE_RT,
                       S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X, S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y, S_MOVE_RT, S_LOWER_Y,
                       S_WAIT_VAC_Y, S_LIFT_X, S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y, S_CENTER_X, -4
                      }; // M_TRANSLATE_DN

// TODO: Ishit talk to Roy
int M_DIVIDER_LT[25] = {S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_HI, S_MOVE_LT, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_HI, S_MOVE_RT,
                        S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_HI, S_CENTER_X, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_HI, S_MOVE_RT,
                        S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_HI, S_MOVE_LT, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_HI, S_CENTER_X, -1
                       };
// divider, robot zigzags with divider (not used)
int M_DIVIDER_DN[21] = {S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_HI, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_HI, S_CENTER_Y,
                        S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_HI, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_HI, S_MOVE_DN,
                        S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_HI, S_CENTER_Y, -4
                       };

int M_CALIBRATE[9] = {S_SET_CUPS, S_LIFT_X, S_MOVE_DN, S_MOVE_UP, S_CENTER_Y, S_MOVE_RT, S_MOVE_LT, S_CENTER_X, -10};

// (robot starts in upper right??)
int M_STOP[2] = {S_STOP_ALL, -10};
// do not change order, only append to the end
int *MANAGERS[12] = {M_TRANSLATE_LT, M_TRANSLATE_RT, M_TRANSLATE_UP, M_TRANSLATE_DN, M_DEBUG_LT,
                     M_TURN_X_LR, M_TURN_X_RL, M_TURN_Y_LR, M_TURN_Y_RL, M_STOP, M_DIVIDER_LT, M_DIVIDER_DN
                    };

/********** Display Modes ***********/
// Determines what is outputted to the Serial port by the actions
int currDisplayMode;
const int D_NONE = 0;
const int D_GUI = 1;
const int D_DEBUG = 2;

/* Global Variables */
int xLow, xPos, xHigh, yLow, yPos, yHigh;
int ltLow, rtLow, upLow, dnLow;


void setup() {
  Serial.begin(9600);
  currState = S_STOP_ALL;
  currDisplayMode = D_GUI;
  currIndex = 0;
  stateCounter = 0;
  currManager = M_STOP;

  xLow = 999;
  xHigh = 0;
  yLow = 999;
  yHigh = 0;

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  pinMode(pinLimLT, INPUT_PULLUP);
  pinMode(pinLimRT, INPUT_PULLUP);
  pinMode(pinLimUP, INPUT_PULLUP);
  pinMode(pinLimDN, INPUT_PULLUP);
  pinMode(pinLimN, INPUT_PULLUP);
  pinMode(pinLimS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinLimLT), hitLT, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinLimRT), hitRT, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinLimUP), hitUP, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinLimDN), hitDN, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinLimN), hitN, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinLimS), hitS, FALLING);

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

  pinMode(xPump, OUTPUT);
  pinMode(yPump, OUTPUT);

  stopAllMotors();
}

void loop() {
  /********** Manual Control ***********/
  if (Serial.available() > 0) {
    switch (Serial.read()) {
      case 'a':
        currIndex = -1;
        currManager = M_TRANSLATE_LT;
        incrementState();
        break;

      case 'd':
        currIndex = -1;
        currManager = M_TRANSLATE_RT;
        incrementState();
        break;

      case 'w':
        currIndex = -1;
        currManager = M_TRANSLATE_UP;
        incrementState();
        break;

      case 's':
        currIndex = -1;
        currManager = M_TRANSLATE_DN;
        incrementState();
        break;

      case 'v':
        currIndex = -1;
        currManager = M_TURN_X_LR;
        incrementState();
        break;

      case 'b':
        currIndex = -1;
        currManager = M_TURN_X_RL;
        incrementState();
        break;

      case '1':
        Serial.println("Move Lt Manual");
        currState = S_MOVE_LT;
        currManager = M_STOP;
        break;

      case '2':
        Serial.println("Move Rt Manual");
        currState = S_MOVE_RT;
        currManager = M_STOP;
        break;

      case '3':
        Serial.println("Move Up Manual");
        currState = S_MOVE_UP;
        currManager = M_STOP;
        break;

      case '4':
        Serial.println("Move Dn Manual");
        currState = S_MOVE_DN;
        currManager = M_STOP;
        break;

      case 'i':
        currState = S_LIFT_X;
        currManager = M_STOP;
        break;

      case 'k':
        currState = S_LOWER_X;
        currManager = M_STOP;
        break;

      case 'o':
        currState = S_LIFT_Y;
        currManager = M_STOP;
        break;

      case 'l':
        currState = S_LOWER_Y;
        currManager = M_STOP;
        break;

      case 'x':
        currState = S_CENTER_X;
        currManager = M_STOP;
        break;

      case 'y':
        currState = S_CENTER_Y;
        currManager = M_STOP;
        break;

      case 'f':
        currDisplayMode = D_DEBUG;
        break;

      case 'g':
        displayPressure();
        break;

      case 'h':
        displayUltrasonic();
        break;

      case 'n':
        currIndex = -1;
        currManager = M_DEBUG_LT;
        incrementState();
        break;

      case 'c':
        currIndex = -1;
        currManager = M_CALIBRATE;
        incrementState();
        break;

      case '7':
        digitalWrite(xPump, HIGH);
        break;

      case '8':
        digitalWrite(xPump, LOW);
        break;

      case '9':
        digitalWrite(yPump, HIGH);
        break;

      case '0':
        digitalWrite(yPump, LOW);
        break;

      case '5':
        setCupHeight();
        break;

      default:
        currIndex = -1;
        currState = S_STOP_ALL;
        currManager = M_STOP;
        incrementState();
        break;
    }
  }

  /********** States and Transitions ***********/
  switch (currState) {
    case S_MOVE_LT:
      if (moveLT()) {
        incrementState();
      }
      break;

    case S_MOVE_RT:
      if (moveRT()) {
        incrementState();
      }
      break;

    case S_MOVE_UP:
      if (moveUP()) {
        incrementState();
      }
      break;

    case S_MOVE_DN:
      if (moveDN()) {
        incrementState();
      }
      break;

    case S_CENTER_X:
      if (SonarMidX()) {
        incrementState();
      }
      break;

    case S_CENTER_Y:
      if (SonarMidY()) {
        incrementState();
      }
      break;

    case S_LIFT_X:
      if (liftCupsX(false)) {
        incrementState();
      }
      break;

    case S_LIFT_X_HI:
      if (liftCupsX(true)) {
        incrementState();
      }
      break;

    case S_LOWER_X:
      if (lowerCupsX()) {
        incrementState();
      }
      break;

    case S_LIFT_Y:
      if (liftCupsY(false)) {
        incrementState();
      }
      break;

    case S_LIFT_Y_HI:
      if (liftCupsY(true)) {
        incrementState();
      }
      break;

    case S_LOWER_Y:
      if (lowerCupsY()) {
        incrementState();
      }
      break;

    case S_STOP_ALL:
      stopAllMotors();
      incrementState();
      break;

    case S_WAIT_VAC_X:
      if (waitVacuumX()) {
        incrementState();
      }
      break;

    case S_WAIT_VAC_Y:
      if (waitVacuumY()) {
        incrementState();
      }
      break;

  }
  stateCounter++;
  delay(35);
}

/********** State Machine Helpers ***********/
void incrementState() {
  currIndex++;
  currState = *(currManager + currIndex);
  if (currState < 0) {
    currIndex = 0;
    currManager = MANAGERS[(-1 * currState) - 1];
    currState = *currManager;
  }
  stateCounter = 0;
}

/********** Actions ***********/
bool waitVacuumX() {
  int sensor = analogRead(xVacuum);
  float voltage = sensor * (5.0 / 1023.0);
  float pressure = (voltage - 2.82) / 0.054;

  if (currDisplayMode == D_GUI) {
    Serial.print("u");
    Serial.println(pressure);
  }

  if (pressure < pressureThreshold) {
    return true;
  } else {
    return false;
  }
}

bool waitVacuumY() {
  int sensor = analogRead(yVacuum);
  float voltage = sensor * (5.0 / 1023.0);
  float pressure = (voltage - 2.82) / 0.054;

  if (currDisplayMode == D_GUI) {
    Serial.print("v");
    Serial.println(pressure);
  }

  if (pressure < pressureThreshold) {
    return true;
  } else {
    return false;
  }
}

bool moveLT() {
  int echo = sonarWEST.ping_median(3);
  if ((echo < xLow) && (echo != 0)) {
    xLow = echo;
  }

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
  int echo = sonarWEST.ping_median(3);
  if ((echo > xHigh) && (echo != 0)) {
    xHigh = echo;
  }

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
  int echo = sonarNORTH.ping_median(3);
  if ((echo < yLow) && (echo != 0)) {
    yLow = echo;
  }

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
  int echo = sonarNORTH.ping_median(3);
  if ((echo > yHigh) && (echo != 0)) {
    yHigh = echo;
  }

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

bool SonarMidX() {
  int echo = sonarWEST.ping_median(3);
  if (echo == 0) {
    digitalWrite(X1, LOW);
    digitalWrite(X2, HIGH);
    Serial.println("Error, 0 ping");
    return false;
  }
  int xSonarMid = (xHigh - xLow) / 2;
  if (echo - xSonarMid > 10) {
    digitalWrite(X1, HIGH);
    digitalWrite(X2, LOW);
    return false;
  } else if (echo - xSonarMid < -10) {
    digitalWrite(X1, LOW);
    digitalWrite(X2, HIGH);
    return false;
  } else {
    digitalWrite(X1, LOW);
    digitalWrite(X2, LOW);
    Serial.print("xLow:");
    Serial.print(xLow);
    Serial.print(" echo:");
    Serial.print(echo);
    Serial.print(" xHigh");
    Serial.println(xHigh);
    return true;
  }
  return false;
}

bool SonarMidY() {
  int echo = sonarNORTH.ping_median(3);
  Serial.print("yLow:");
  Serial.print(yLow);
  Serial.print(" echo:");
  Serial.print(echo);
  Serial.print(" yHigh:");
  Serial.println(yHigh);
  if (echo == 0) {
    digitalWrite(Y1, HIGH);
    digitalWrite(Y2, LOW);
    return false;
  }
  int ySonarMid = (yHigh - yLow) / 2 + 50;
  if (echo - ySonarMid > 5) {
    digitalWrite(Y1, LOW);
    digitalWrite(Y2, HIGH);
    return false;
  } else if (echo - ySonarMid < -5) {
    digitalWrite(Y1, HIGH);
    digitalWrite(Y2, LOW);
    return false;
  } else {
    digitalWrite(Y1, LOW);
    digitalWrite(Y2, LOW);
    return true;
  }
  return false;
}

bool setCupHeight() {
  ltLow = sonarLT.ping_median(3);
  rtLow = sonarRT.ping_median(3);
  upLow = sonarUP.ping_median(3);
  dnLow = sonarDN.ping_median(3);
  Serial.println("Cups set at: ");
  Serial.print("LT: ");
  Serial.print(ltLow);
  Serial.print(" RT: ");
  Serial.println(rtLow);
  Serial.print("UP: ");
  Serial.print(upLow);
  Serial.print(" DN: ");
  Serial.println(dnLow);
  return true;
}

bool lowerCupsX() {
  bool left, right;
  digitalWrite(xPump, HIGH);
  if (returnPressure(true) < -1.0) {
    stopMotorsLR();
    return true;
  }

  long echoLT = sonarLT.ping_median(3);
  long echoRT = sonarRT.ping_median(3);

  if ((echoLT < ltLow) && (echoRT > rtLow)) {
    digitalWrite(L1, LOW);
    digitalWrite(L2, HIGH);
    digitalWrite(R1, HIGH);
    digitalWrite(R2, LOW);
    return false;
  } else if (echoLT > (ltLow + 8)) {
    digitalWrite(L1, LOW);
    digitalWrite(L2, HIGH);
    left = false;
  } else {
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
    left = true;
  }

  if ((echoRT < rtLow) && (echoLT > ltLow)) {
    digitalWrite(L1, HIGH);
    digitalWrite(L2, LOW);
    digitalWrite(R1, LOW);
    digitalWrite(R2, HIGH);
    return false;
  } else if (echoRT > (rtLow + 8)) {
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

bool liftCupsX(bool high) {
  bool left, right;
  if (returnPressure(true) < 6.5) {
    stopMotorsLR();
    digitalWrite(xPump, LOW);
    return false;
  }

  long echoLT = sonarLT.ping_median(3);
  long echoRT = sonarRT.ping_median(3);

  if ((echoLT < (ltLow + 10)) && (echoLT != 0)) {
    digitalWrite(L1, HIGH);
    digitalWrite(L2, LOW);
    left = false;;
  } else {
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
    left = true;
  }

  if ((echoRT < (rtLow + 10)) && (echoRT != 0)) {
    digitalWrite(R1, HIGH);
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
  if (returnPressure(false) < -1.0) {
    stopMotorsUD();
    return true;
  }
  int switchN = digitalRead(pinLimN);
  int switchS = digitalRead(pinLimS);
  long echoUP = sonarUP.ping_median(3);
  long echoDN = sonarDN.ping_median(3);

  digitalWrite(yPump, HIGH);

  if ((echoUP < upLow) > (echoDN > dnLow)) {
    digitalWrite(U1, LOW);
    digitalWrite(U2, HIGH);
    digitalWrite(D1, HIGH);
    digitalWrite(D2, LOW);
    return false;
  } else if ((echoUP > (upLow + 8)) && (switchN == HIGH)) {
    digitalWrite(U1, LOW);
    digitalWrite(U2, HIGH);
    up = false;
  } else {
    digitalWrite(U1, LOW);
    digitalWrite(U2, LOW);
    up = true;
  }

  if ((echoDN < dnLow) && (echoUP > upLow)) {
    digitalWrite(U1, HIGH);
    digitalWrite(U2, LOW);
    digitalWrite(D1, LOW);
    digitalWrite(D2, HIGH);
    return false;
  } else if ((echoDN > (dnLow + 8)) && (switchS == HIGH)) {
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

bool liftCupsY(bool high) {
  bool up, down;

  if (returnPressure(false) < 6.5) {
    stopMotorsUD();
    digitalWrite(yPump, LOW);
    return false;
  }
  digitalWrite(U1, HIGH);
  digitalWrite(U2, LOW);
  digitalWrite(D1, HIGH);
  digitalWrite(D2, LOW);
  delay(500);
  stopMotorsUD();
  return true;
}

bool stopMotorsLR() {
  digitalWrite(L1, LOW);
  digitalWrite(L2, LOW);
  digitalWrite(R1, LOW);
  digitalWrite(R2, LOW);
  return true;
}

bool stopMotorsUD() {
  digitalWrite(U1, LOW);
  digitalWrite(U2, LOW);
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  return true;
}

bool stopMotorsXY() {
  digitalWrite(X1, LOW);
  digitalWrite(X2, LOW);
  digitalWrite(Y1, LOW);
  digitalWrite(Y2, LOW);
  return true;
}

bool stopAllMotors() {
  if (stopMotorsXY()) {
    if (stopMotorsLR()) {
      if (stopMotorsUD()) {
        return true;
      }
    }
  }
  return false;
}

/********** Interrupt Functions ***********/
void hitLT() {
  stopMotorsXY();
  return;
}

void hitRT() {
  stopMotorsXY();
  return;
}

void hitUP() {
  stopMotorsXY();
  return;
}

void hitDN() {
  stopMotorsXY();
  return;
}

void hitN() {
  digitalWrite(U1, LOW);
  digitalWrite(U2, LOW);
  return;
}

void hitS() {
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  return;
}

/********** miscellaneous ***********/
float returnPressure(bool x) {
  int sensor;
  if (x) {
    sensor = analogRead(xVacuum);
  } else {
    sensor = analogRead(yVacuum);
  }
  float voltage = sensor * (5.0 / 1023.0);
  float pressure = (voltage - 2.82) / 0.054;
  return pressure;
}

void displayPressure() {
  int sensor1 = analogRead(yVacuum);
  int sensor2 = analogRead(xVacuum);
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
  long echoLT = sonarLT.ping_median(3);
  long echoRT = sonarRT.ping_median(3);
  long echoUP = sonarUP.ping_median(3);
  long echoDN = sonarDN.ping_median(3);
  long echoN = sonarNORTH.ping_median(3);
  long echoW = sonarWEST.ping_median(3);
  Serial.print("LT = ");
  Serial.print(echoLT);
  Serial.print(" RT = ");
  Serial.println(echoRT);
  Serial.print("UP = ");
  Serial.print(echoUP);
  Serial.print(" DN = ");
  Serial.println(echoDN);
  Serial.print("N = ");
  Serial.print(echoN);
  Serial.print(" W = ");
  Serial.println(echoW);
  return;
}
