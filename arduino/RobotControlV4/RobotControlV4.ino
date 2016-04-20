/* Icarus Window Washer, V4.0
   Mechatronic Design
   Roy Shin, Bereket Abraham
*/

/* Manual Control Scheme

   Movement: WASD, y centers Y
   X legs: i to raise, k to lower
   Y legs: o to raise, l to lower
   Pressure: p to display pressure
*/

#include <Encoder.h>
#include <NewPing.h>

/********** Pin Definitions ***********/
const int pinLimLT = 19;
const int pinLimRT = 21;
const int pinLimUP = 20;
const int pinLimDN = 18;

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

const int yValve = 40;
const int xValve = 41;

const int yVacuum = A0;
const int xVacuum = A1;

NewPing sonarLT(trigLT, echoLT, 8);
NewPing sonarRT(trigRT, echoRT, 8);
NewPing sonarUP(trigUP, echoUP, 8);
NewPing sonarDN(trigDN, echoDN, 8);

Encoder yEncoder(2, 4);
Encoder xEncoder(3, 5);

const float pressureThreshold = -7.0;

const bool longCupsVertical = true;
// records the orientation of the robot
// true means that the X railing (sponge holders) is vertical
// true when the divider is vertical

/********** States ***********/
// Performs actions and state transitions. State can override its manager
// in case of failure / unexpected conditions. State can transition to a
// different manager as well.
volatile int currState;
volatile unsigned long stateCounter; // # of cycles since currState started
const int S_MOVE_LT = 0;
const int S_MOVE_RT = 1;
const int S_MOVE_UP = 2;
const int S_MOVE_DN = 3;
const int S_LOWER_X = 4;
const int S_LIFT_X_LO = 5;
const int S_LIFT_X_HI = 6;
const int S_LOWER_Y = 7;
const int S_LIFT_Y_LO = 8;
const int S_LIFT_Y_HI = 9;
const int S_STOP_ALL = 10;
const int S_CENTER_X = 11;
const int S_CENTER_Y = 12;
const int S_WAIT_VAC_X = 13;
const int S_WAIT_VAC_Y = 14;

/********** Managers ***********/
// A sequence of states that repeats in a loop. Manager can be started in
// the middle of a sequence
volatile int *currManager;
volatile int currIndex;
// linear motion
int M_TRANSLATE_LT[10] = {S_CENTER_Y, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_LO, S_MOVE_RT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_LO, S_MOVE_LT, -1}; // loops back around
int M_TRANSLATE_RT[10] = {S_CENTER_Y, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_LO, S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_LO, S_MOVE_RT, -2};
int M_TRANSLATE_UP[10] = {S_CENTER_X, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_LO, S_MOVE_DN, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_LO, S_MOVE_UP, -3};
int M_TRANSLATE_DN[10] = {S_CENTER_X, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_LO, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_LO, S_MOVE_DN, -4};
int M_DEBUG_LT[7] = {S_LOWER_Y, S_LIFT_X_LO, S_MOVE_RT, S_LOWER_X, S_LIFT_Y_LO, S_MOVE_LT, -5};

// turns, robot zigzags against divider
// when moving down, if barrier transition to linear motion
int M_TURN_X_LR[35] = {S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_LO, S_CENTER_X, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_LO,
                       S_MOVE_DN, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_LO, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_LO, S_MOVE_DN,
                       S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_LO, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_LO, S_MOVE_DN, S_LOWER_X,
                       S_WAIT_VAC_X, S_LIFT_Y_LO, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_LO, S_CENTER_Y, -2
                      }; // M_TRANSLATE_RT

int M_TURN_X_RL[35] = {S_MOVE_RT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_LO, S_CENTER_X, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_LO,
                       S_MOVE_DN, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_LO, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_LO, S_MOVE_DN,
                       S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_LO, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_LO, S_MOVE_DN, S_LOWER_X,
                       S_WAIT_VAC_X, S_LIFT_Y_LO, S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_LO, S_CENTER_Y, -1
                      }; // M_TRANSLATE_LT

// turns, robot zigzags with divider (not used)
int M_TURN_Y_LR[35] = {S_MOVE_DN, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_LO, S_CENTER_Y, S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_LO,
                       S_MOVE_RT, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_LO, S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_LO, S_MOVE_RT,
                       S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_LO, S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_LO, S_MOVE_RT, S_LOWER_Y,
                       S_WAIT_VAC_Y, S_LIFT_X_LO, S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_LO, S_CENTER_X, -3
                      }; // M_TRANSLATE_UP

int M_TURN_Y_RL[35] = {S_MOVE_UP, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_LO, S_CENTER_Y, S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_LO,
                       S_MOVE_RT, S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_LO, S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_LO, S_MOVE_RT,
                       S_LOWER_Y, S_WAIT_VAC_Y, S_LIFT_X_LO, S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_LO, S_MOVE_RT, S_LOWER_Y,
                       S_WAIT_VAC_Y, S_LIFT_X_LO, S_MOVE_LT, S_LOWER_X, S_WAIT_VAC_X, S_LIFT_Y_LO, S_CENTER_X, -4
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

int M_CALIBRATE[7] = {S_MOVE_DN, S_MOVE_UP, S_CENTER_Y, S_MOVE_RT, S_MOVE_LT, S_CENTER_X, -10};
//int M_CALIBRATE[4] = {S_MOVE_DN, S_MOVE_UP, S_CENTER_Y, -10};

// (robot starts in upper right??)
int M_STOP[2] = {S_STOP_ALL, -10};
// do not change order, only append to the end
int *MANAGERS[12] = {M_TRANSLATE_LT, M_TRANSLATE_RT, M_TRANSLATE_UP, M_TRANSLATE_DN, M_DEBUG_LT, 
                     M_TURN_X_LR, M_TURN_X_RL, M_TURN_Y_LR, M_TURN_Y_RL, M_STOP, M_DIVIDER_LT, M_DIVIDER_DN};

/********** Display Modes ***********/
// Determines what is outputted to the Serial port by the actions
volatile int currDisplayMode;
const int D_NONE = 0;
const int D_GUI = 1;
const int D_DEBUG = 2;

/* Global Variables */
volatile bool limitLT, limitRT, limitUP, limitDN;
volatile int yExtreme, yPosition, xExtreme, xPosition;
volatile bool edgeDetected;

void setup() {
  Serial.begin(9600);
  currState = S_STOP_ALL;
  currDisplayMode = D_GUI;
  currIndex = 0;
  stateCounter = 0;
  currManager = M_STOP;
  edgeDetected = false;

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
  stateCounter++;

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
        currState = S_MOVE_LT;
        currManager = M_STOP;
        break;

      case '2':
        currState = S_MOVE_RT;
        currManager = M_STOP;
        break;

      case '3':
        currState = S_MOVE_UP;
        currManager = M_STOP;
        break;

      case '4':
        currState = S_MOVE_DN;
        currManager = M_STOP;
        break;

      case 'i':
        currState = S_LIFT_X_HI;
        currManager = M_STOP;
        break;

      case 'k':
        currState = S_LOWER_X;
        currManager = M_STOP;
        break;

      case 'o':
        currState = S_LIFT_Y_LO;
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
      // debug
      case '7':
        digitalWrite(xValve, HIGH);
        break;
      case '8':
        digitalWrite(yValve, HIGH);
        break;
      case '9':
        digitalWrite(xValve, LOW);
        break;
      case '0':
        digitalWrite(yValve, LOW);
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
      if (moveMidX()) {
        incrementState();
      }
      break;

    case S_CENTER_Y:
      if (moveMidY()) {
        incrementState();
      }
      break;

    case S_LIFT_X_LO:
      digitalWrite(xValve, HIGH);
      if (liftCupsX(false)) {
        digitalWrite(xValve, LOW);
        incrementState();
      }
      break;

    case S_LIFT_X_HI:
      digitalWrite(xValve, HIGH);
      if (liftCupsX(true)) {
        digitalWrite(xValve, LOW);
        incrementState();
      }
      break;

    case S_LOWER_X:
      digitalWrite(xValve, LOW);
      if (lowerCupsX()) {
        incrementState();
      }
      break;

    case S_LIFT_Y_LO:
      digitalWrite(yValve, HIGH);
      if (liftCupsY(false)) {
        digitalWrite(yValve, LOW);
        incrementState();
      }
      break;

    case S_LIFT_Y_HI:
      digitalWrite(yValve, HIGH);
      if (liftCupsY(true)) {
        digitalWrite(yValve, LOW);
        incrementState();
      }
      break;

    case S_LOWER_Y:
      digitalWrite(yValve, LOW);
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

// 0 = nothing, 1 = edge, 2 = divider
int checkForBarrierRT() {
  int echo = sonarRT.ping_median(2);
  int barrier = 0;

  if (echo < 200) {
    edgeDetected = true;
    barrier = 0;
  } else if (edgeDetected) {
    edgeDetected = false;
    if (echo == 0) {
      barrier = 1;
    } else {
      barrier = 2;
    }
  } else {
    edgeDetected = false;
    barrier = 0;
  }

  if (currDisplayMode == D_GUI) {
    Serial.print("E");
    Serial.println(barrier);
  }
  return barrier;
}

int checkForBarrierLT() {
  int echo = sonarLT.ping_median(2);
  int barrier = 0;

  if (echo < 200) {
    edgeDetected = true;
    barrier = 0;
  } else if (edgeDetected) {
    edgeDetected = false;
    if (echo == 0) {
      barrier = 1;
    } else {
      barrier = 2;
    }
  } else {
    edgeDetected = false;
    barrier = 0;
  }

  if (currDisplayMode == D_GUI) {
    Serial.print("W");
    Serial.println(barrier);
  }
  return barrier;
}

int checkForBarrierUP() {
  int echo = sonarUP.ping_median(2);
  int barrier = 0;

  if (echo < 200) {
    edgeDetected = true;
    barrier = 0;
  } else if (edgeDetected) {
    edgeDetected = false;
    if (echo == 0) {
      barrier = 1;
    } else {
      barrier = 2;
    }
  } else {
    edgeDetected = false;
    barrier = 0;
  }

  if (currDisplayMode == D_GUI) {
    Serial.print("N");
    Serial.println(barrier);
  }
  return barrier;
}

int checkForBarrierDN() {
  int echo = sonarDN.ping_median(2);
  int barrier = 0;

  if (echo < 200) {
    edgeDetected = true;
    barrier = 0;
  } else if (edgeDetected) {
    edgeDetected = false;
    if (echo == 0) {
      barrier = 1;
    } else {
      barrier = 2;
    }
  } else {
    edgeDetected = false;
    barrier = 0;
  }

  if (currDisplayMode == D_GUI) {
    Serial.print("S");
    Serial.println(barrier);
  }
  return barrier;
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
  if (currDisplayMode == D_GUI) {
    xPosition = xEncoder.read();
    Serial.print("x");
    Serial.println(xPosition);
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
  if (currDisplayMode == D_GUI) {
    xPosition = xEncoder.read();
    Serial.print("x");
    Serial.println(xPosition);
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
  if (currDisplayMode == D_GUI) {
    yPosition = yEncoder.read();
    Serial.print("y");
    Serial.println(yPosition);
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
  if (currDisplayMode == D_GUI) {
    yPosition = yEncoder.read();
    Serial.print("y");
    Serial.println(yPosition);
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

bool moveMidX() {
  int target = xExtreme / 2;
  xPosition = xEncoder.read();

  if (currDisplayMode == D_GUI) {
    Serial.print("x");
    Serial.println(xPosition);
  }

  if (target - xPosition > 0) {
    digitalWrite(X1, HIGH);
    digitalWrite(X2, LOW);
    return false;
  } else if (target - xPosition < 0) {
    digitalWrite(X1, LOW);
    digitalWrite(X2, HIGH);
    return false;
  } else {
    digitalWrite(X1, LOW);
    digitalWrite(X2, LOW);

    if (currDisplayMode == D_GUI) {
      Serial.print("X");
      Serial.println(xExtreme);
    }
    return true;
  }
}

bool moveMidY() {
  int target = yExtreme / 2;
  yPosition = yEncoder.read();

  if (currDisplayMode == D_GUI) {
    Serial.print("y");
    Serial.println(yPosition);
  }

  if (target - yPosition > 0) {
    digitalWrite(Y1, HIGH);
    digitalWrite(Y2, LOW);
    return false;
  } else if (target - yPosition < 0) {
    digitalWrite(Y1, LOW);
    digitalWrite(Y2, HIGH);
    return false;
  } else {
    digitalWrite(Y1, LOW);
    digitalWrite(Y2, LOW);

    if (currDisplayMode == D_GUI) {
      Serial.print("Y");
      Serial.println(yExtreme);
    }
    return true;
  }
}

bool lowerCupsX() {
  bool left, right;

  long echoLT = sonarLT.ping_median(2);
  long echoRT = sonarRT.ping_median(2);

  if (currDisplayMode == D_DEBUG) {
    Serial.print("LT:");
    Serial.print(echoLT);
    Serial.print("us RT:");
    Serial.print(echoRT);
    Serial.println("us");
  } else if (currDisplayMode == D_GUI) {
    Serial.print("e");
    Serial.println(echoRT);
    Serial.print("w");
    Serial.println(echoLT);
  }

  if ((echoLT < 255) && (echoRT > 265)) {
    analogWrite(L1, 255 / 8);
    digitalWrite(L2, HIGH);
    analogWrite(R1, 255 / 8);
    digitalWrite(R2, LOW);
    return false;
  } else if (echoLT > 260) {
    analogWrite(L1, 255 / 16);
    digitalWrite(L2, HIGH);
    left = false;
  } else {
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
    left = true;
  }

  if ((echoRT < 255) && (echoLT > 265)) {
    analogWrite(L1, 255 / 8);
    digitalWrite(L2, LOW);
    analogWrite(R1, 255 / 8);
    digitalWrite(R2, HIGH);
    return false;
  } else if (echoRT > 260) {
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

bool liftCupsX(bool high) {
  bool left, right;

  long echoLT = sonarLT.ping_median(2);
  long echoRT = sonarRT.ping_median(2);

  if (currDisplayMode == D_DEBUG) {
    Serial.print("LT:");
    Serial.print(echoLT);
    Serial.print("us RT:");
    Serial.print(echoRT);
    Serial.println("us");
  } else if (currDisplayMode == D_GUI) {
    Serial.print("e");
    Serial.println(echoRT);
    Serial.print("w");
    Serial.println(echoLT);
  }
  digitalWrite(L1, HIGH);
  digitalWrite(L2, LOW);
  digitalWrite(R1, HIGH);
  digitalWrite(R2, LOW);
  if (high) {
    delay(1000);
  } else {
    delay(250);
  }
  digitalWrite(L1, LOW);
  digitalWrite(L2, LOW);
  digitalWrite(R1, LOW);
  digitalWrite(R2, LOW);
  return true;
}

bool lowerCupsY() {
  bool up, down;

  long echoUP = sonarUP.ping_median(2);
  long echoDN = sonarDN.ping_median(2);

  if (currDisplayMode == D_DEBUG) {
    Serial.print("UP:");
    Serial.print(echoUP);
    Serial.print("us DN:");
    Serial.print(echoDN);
    Serial.println("us");
  } else if (currDisplayMode == D_GUI) {
    Serial.print("n");
    Serial.println(echoUP);
    Serial.print("s");
    Serial.println(echoDN);
  }

  if ((echoUP < 240) && (echoDN > 250)) {
    analogWrite(U1, 255 / 8);
    digitalWrite(U2, HIGH);
    analogWrite(D1, 255 / 8);
    digitalWrite(D2, LOW);
    return false;
  } else if (echoUP > 245) {
    analogWrite(U1, 255 / 16);
    digitalWrite(U2, HIGH);
    up = false;
  } else {
    digitalWrite(U1, LOW);
    digitalWrite(U2, LOW);
    up = true;
  }

  if ((echoDN < 240) && (echoUP > 250)) {
    analogWrite(U1, 255 / 8);
    digitalWrite(U2, LOW);
    analogWrite(D1, 255 / 8);
    digitalWrite(D2, HIGH);
    return false;
  } else if (echoDN > 245) {
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

bool liftCupsY(bool high) {
  bool up, down;

  long echoUP = sonarUP.ping_median(2);
  long echoDN = sonarDN.ping_median(2);

  if (currDisplayMode == D_DEBUG) {
    Serial.print("UP:");
    Serial.print(echoUP);
    Serial.print("us DN:");
    Serial.print(echoDN);
    Serial.println("us");
  } else if (currDisplayMode == D_GUI) {
    Serial.print("n");
    Serial.println(echoUP);
    Serial.print("s");
    Serial.println(echoDN);
  }
  digitalWrite(U1, HIGH);
  digitalWrite(U2, LOW);
  digitalWrite(D1, HIGH);
  digitalWrite(D2, LOW);
  if (high) {
    delay(1000);
  } else {
    delay(400);
  }
  digitalWrite(U1, LOW);
  digitalWrite(U2, LOW);
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  return true;
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

/********** Interrupt Functions ***********/

void hitLT() {
  digitalWrite(X1, LOW);
  digitalWrite(X2, LOW);
  xExtreme = xEncoder.read();
  return;
}

void hitRT() {
  digitalWrite(X1, LOW);
  digitalWrite(X2, LOW);
  xEncoder.write(0);
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

/********** miscellaneous ***********/

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
  long echoLT = sonarLT.ping_median(2);
  long echoRT = sonarRT.ping_median(2);
  long echoUP = sonarUP.ping_median(2);
  long echoDN = sonarDN.ping_median(2);
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
