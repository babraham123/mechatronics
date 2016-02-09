// Motor Control lab
// Bereket Abraham

#include <Encoder.h>
#include <Bounce2.h>
#include <Servo.h>

int inputMode = 0;
// 0 = GUI, 1 = sensor
#define bufferLen 3
int buffer[bufferLen];
int curr = 0;

// uno: interrupt pins 2,3
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder hallEncoder(2, 3);
Encoder knobEncoder(12, 13);
volatile long hallPos = -999;
long knobPos = -999;

Bounce debouncer = Bounce(); 
const int limitPin = 7;
int switchMode = HIGH;

Servo servo;
const int servoPin = 9;
int servoAngle = 0;

// Declare pins for L298N Dual H-Bridge Motor Controller
// Motor 1
// const int motor1I1 = 7;
// const int motor1I2 = 8;
// const int speedPin1 = 9;
// Motor 2
const int motor2I3 = 4;
const int motor2I4 = 6;
const int speedPin2 = 5;

// IR proximity sensor
const int sensorPinIR = A0;
int valueIR = 0;
int voltageIR = 0;
int distanceIR = 0;

// Potentiometer
const int sensorPinPot = A1;
int valuePot = 0;
int degreePot = 0;

void setup() {
  Serial.begin(9600);

  // Set internal pull-up on switch
  pinMode(limitPin,INPUT_PULLUP);
  debouncer.attach(limitPin);
  debouncer.interval(5); // interval in ms
  servo.attach(servoPin);

  setupMotor();

  for (int i = 0; i < bufferLen; i++) {
    buffer[i] = 0;
  }
}

void loop() {
  hallPos = hallEncoder.read();
  updateMode();

  if (inputMode == 1) {
    readSensors();

  } else if (inputMode == 0) {
    // make sure not to send too many cmds at once
    // val only use first 4 bits
    // message terminated by 0xFF
    while (Serial.available() > 0) {
      int cByte = Serial.read();
      if (cByte == 255) {
        char t = (char)buffer[(curr+bufferLen-2) % bufferLen];
        int val = buffer[(curr+bufferLen-1) % bufferLen] << 2;
        val = val + buffer[curr];
        performAction(t, val);
        // print t,val?
      } else {
        curr = (curr + 1) % bufferLen;
        buffer[curr] = cByte;
      }
    }

  } else {
    Serial.println("Unknown mode");
  }
}

void performAction(char type, int value) {
  value = constrain(value, 0, 255);

  switch (type) {
    // Motor 2
    case 'f': // Motor 2 Forward
      digitalWrite(motor2I3, LOW);
      digitalWrite(motor2I4, HIGH);
      analogWrite(speedPin2, value);
      break;
    case 'b': // Motor 2 backward
      digitalWrite(motor2I3, HIGH);
      digitalWrite(motor2I4, LOW);
      analogWrite(speedPin2, value);
      break;
    case 'k': // Motor 2 brake
      digitalWrite(motor2I3, LOW);
      digitalWrite(motor2I4, LOW);
      analogWrite(speedPin2, 0);
      break;

    case 's': // Set servo angle (0-180)
      if (value > 180 || value < 0) {
        value = 0;
      }
      servo.write(value);
      break;

    case 'p': // Stepper motor
      // pass
      break;

    case 'r': // Reset encoders
      hallEncoder.write(0);
      knobEncoder.write(0);
      break;

    default:
      Serial.println("Unknown action type");
  }
}

void updateMode() {
  debouncer.update();
  int switchNew = debouncer.read();
  // LOW == depressed, on rising edge
  if (switchMode == LOW && switchNew == HIGH) {
    inputMode = (inputMode + 1) % 2;
  }
  switchMode = switchNew;
}

void readSensors() {
  long knobNew;
  int switchNew, irNew, potNew;

  knobNew = knobEncoder.read();
  if (knobNew != knobPos) {
    // TODO: calculate stepper steps
    performAction('p', knobNew - knobPos);
    knobPos = knobNew;
  }

  // IR Proximity Sensor
  valueIR = analogRead(sensorPinIR);
  voltageIR = mapAnalog(valueIR);
  irNew = transferIR(valueIR);
  if (irNew != distanceIR) {
    // TODO: calculate distance to speed
    if (irNew < 2) {
      performAction('k', 0);
    } else {
      performAction('f', irNew*20);
    }
    distanceIR = irNew;
  }
  
  // Potentiometer
  valuePot = analogRead(sensorPinPot);
  potNew = mapRotary(valuePot);
  if (potNew != degreePot) {
    // TODO: calculate servo angle
    performAction('s', potNew);
    degreePot = potNew;
  }
}

void setupMotor() {
  // L298N Dual H-Bridge Motor Controller Pins
  // pinMode(motor1I1,OUTPUT);
  // pinMode(motor1I2,OUTPUT);
  // pinMode(speedPin1,OUTPUT);
  pinMode(motor2I3,OUTPUT);
  pinMode(motor2I4,OUTPUT);
  pinMode(speedPin2,OUTPUT);
  // defaults
  // analogWrite(speedPin1, 0);
  analogWrite(speedPin2, 0);
  // digitalWrite(motor1I1, LOW);
  // digitalWrite(motor1I2, HIGH);
  digitalWrite(motor2I3, LOW);
  digitalWrite(motor2I4, HIGH);
}

int mapAnalog(int value) {
  return map(value, 0, 1023, 0, 5000);
}

int transferIR(int voltage) {
  return (1 / (0.00023914 * voltage - 0.0100251467)) + 1;
}

int mapRotary(int value) {
  return map(value, 0, 1023, 0, 300); 
}


