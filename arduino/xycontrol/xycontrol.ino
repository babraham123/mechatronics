/*
   X-Y Control of Robot w/o Rotary Encoder
   by Roy S. Shin
   for CMU Mechatronics Window Washer S16
*/

/* Pin Assignments */
const int pinLimLT = 18;
const int pinLimRT = 19;
const int pinLimUP = 20;
const int pinLimDN = 21;

const int L1 = 22;
const int L2 = 23;
const int L3 = 24;
const int L4 = 25;

/* Global Variables */
bool limitLT, limitRT, limitUP, limitDN;

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
}

void loop() {
  int inByte;
  bool success;
  if (Serial.available() > 0) {
    inByte = Serial.read();
    Serial.println(inByte);
    switch (inByte) {
      case 1:
        success = moveLT();
        if (!success) {
          Serial.println("Can't move LT");
        }
        break;
      case 2:
        success = moveRT();
        if (!success) {
          Serial.println("Can't move RT");
        }
        break;
      case 3:
        success = moveUP();
        if (!success) {
          Serial.println("Can't move UP");
        }
        break;
      case 4:
        success = moveDN();
        if (!success) {
          Serial.println("Can't move DN");
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
