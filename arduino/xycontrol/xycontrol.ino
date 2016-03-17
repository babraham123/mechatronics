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

#include <Servo.h>

/* Servo Declarations */
Servo servoWest;
Servo servoEast;
Servo servoNorth;
Servo servoSouth;

/* Pin Assignments */
const int pinLimLT = 18;
const int pinLimRT = 19;
const int pinLimUP = 20;
const int pinLimDN = 21;

const int L1 = 22;
const int L2 = 23;
const int L3 = 24;
const int L4 = 25;

const int pinServoWest = 26;
const int pinServoEast = 27;
const int pinServoNorth = 28;
const int pinServoSouth = 29;

/* Global Variables */
bool limitLT, limitRT, limitUP, limitDN;

/* Global Constants */
int servoUP = 0;
int servoDN = 180;

void setup() {
  Serial.begin(9600);

  pinMode(pinLimLT, INPUT);
  pinMode(pinLimRT, INPUT);
  pinMode(pinLimUP, INPUT);
  pinMode(pinLimDN, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinLimLT), hitLT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinLimRT), hitRT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinLimUP), hitUP, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinLimDN), hitDN, CHANGE);

  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(L3, OUTPUT);
  pinMode(L4, OUTPUT);
  digitalWrite(L1, LOW);
  digitalWrite(L2, LOW);
  digitalWrite(L3, LOW);
  digitalWrite(L4, LOW);

  servoWest.attach(pinServoWest);
  servoEast.attach(pinServoEast);
  servoNorth.attach(pinServoNorth);
  servoSouth.attach(pinServoSouth);
}

void loop() {
  int inByte;
  bool success;
  if (Serial.available() > 0) {
    inByte = Serial.read();
    Serial.println(inByte);
    switch (inByte) {
      case 97: // input character 'a'
        success = moveLT();
        if (!success) {
          Serial.println("Can't move LT");
        }
        break;
      case 100: // input character 'd'
        success = moveRT();
        if (!success) {
          Serial.println("Can't move RT");
        }
        break;
      case 119: // input character 'w'
        success = moveUP();
        if (!success) {
          Serial.println("Can't move UP");
        }
        break;
      case 115: // input character 's'
        success = moveDN();
        if (!success) {
          Serial.println("Can't move DN");
        }
        break;
      case 105: // input character 'i'
        success = liftX();
        if (!success) {
          Serial.println("Can't lift xFeet");
        }
        break;
      case 107: // input character 'k'
        success = lowerX();
        if (!success) {
          Serial.println("Can't lower xFeet");
        }
        break;
      case 111: // input character 'o'
        success = liftY();
        if (!success) {
          Serial.println("Can't lift yFeet");
        }
        break;
      case 108: // input character 'l'
        success = lowerY();
        if (!success) {
          Serial.println("Can't lower yFeet");
        }
        break;
      default:
        stopX();
        stopY();
        break;
    }
  }
}

bool moveLT() {
  if (!limitLT) {
    digitalWrite(L1, HIGH);
    digitalWrite(L2, LOW);
    return true;
  } else {
    return false;
  }
}

bool moveRT() {
  if (!limitRT) {
    digitalWrite(L1, LOW);
    digitalWrite(L2, HIGH);
    return true;
  } else {
    return false;
  }
}

bool moveUP() {
  if (!limitUP) {
    digitalWrite(L3, HIGH);
    digitalWrite(L4, LOW);
    return true;
  } else {
    return false;
  }
}

bool moveDN() {
  if (!limitDN) {
    digitalWrite(L3, LOW);
    digitalWrite(L4, HIGH);
    return true;
  } else {
    return false;
  }
}

void stopX() {
  digitalWrite(L1, LOW);
  digitalWrite(L2, LOW);
}

void stopY() {
  digitalWrite(L3, LOW);
  digitalWrite(L4, LOW);
}

void hitLT() {
  int value = digitalRead(pinLimLT);
  if (value == HIGH) {
    limitLT = false;
  } else {
    stopX();
    limitLT = true;
  }
}

void hitRT() {
  int value = digitalRead(pinLimRT);
  if (value == HIGH) {
    limitRT = false;
  } else {
    stopX();
    limitRT = true;
  }
}

void hitUP() {
  int value = digitalRead(pinLimLT);
  if (value == HIGH) {
    limitLT = false;
  } else {
    stopY();
    limitLT = true;
  }
}

void hitDN() {
  int value = digitalRead(pinLimLT);
  if (value == HIGH) {
    limitRT = false;
  } else {
    limitRT = true;
    stopY();
  }
}

bool liftX() {
  int westStatus = servoWest.read();
  int eastStatus = servoEast.read();
  if (westStatus == servoUP && eastStatus != servoUP) {
    servoWest.write(servoUP);
    servoEast.write(servoUP);
    return true;
  }
  return false;
}

bool liftY() {
  int northStatus = servoNorth.read();
  int southStatus = servoSouth.read();
  if (northStatus != servoUP && southStatus != servoUP) {
    servoNorth.write(servoUP);
    servoSouth.write(servoUP);
    return true;
  }
  return false;
}

bool lowerX() {
  int eastStatus = servoEast.read();
  int westStatus = servoWest.read();
  if (eastStatus != servoDN && westStatus != servoDN) {
    servoWest.write(servoDN);
    servoEast.write(servoDN);
    return true;
  }
  return false;
}

bool lowerY() {
  int northStatus = servoNorth.read();
  int southStatus = servoSouth.read();
  if (northStatus != servoDN && southStatus != servoDN) {
    servoNorth.write(servoDN);
    servoSouth.write(servoDN);
    return true;
  }
  return false;
}
