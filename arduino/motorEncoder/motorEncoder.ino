// Motor Control lab
// Bereket Abraham

#include <Encoder.h>
#include <Bounce2.h>

// uno: interrupt pins 2,3
// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder hallEncoder(2, 3);
Encoder knobEncoder(12, 13);
long hallPos = -999;
long knobPos = -999;

int limitPin = 7;

// Instantiate a Bounce object
Bounce debouncer = Bounce(); 

// Declare L298N Dual H-Bridge Motor Controller
// Motor 1
// int motor1I1 = 7;
// int motor1I2 = 8;
// int speedPin1 = 9;
// Motor 2
int motor2I3 = 4;
int motor2I4 = 6;
int speedPin2 = 5;

void setup() {
  Serial.begin(9600);

  // Setup the button with an internal pull-up :
  pinMode(limitPin,INPUT_PULLUP);
  // After setting up the button, setup the Bounce instance :
  debouncer.attach(limitPin);
  debouncer.interval(5); // interval in ms


  //Define L298N Dual H-Bridge Motor Controller Pins
  // pinMode(motor1I1,OUTPUT);
  // pinMode(motor1I2,OUTPUT);
  // pinMode(speedPin1,OUTPUT);
  pinMode(motor2I3,OUTPUT);
  pinMode(motor2I4,OUTPUT);
  pinMode(speedPin2,OUTPUT);
  // defaults
  // analogWrite(speedPin1, 0);
  analogWrite(speedPin2, 255);
  Serial.println("Motor 2 Forward");
  // digitalWrite(motor1I1, LOW);
  // digitalWrite(motor1I2, HIGH);
  digitalWrite(motor2I3, LOW);
  digitalWrite(motor2I4, HIGH);
}

void loop() {
  long knobNew;
  hallPos = hallEncoder.read();
  knobNew = knobEncoder.read();
  if (knobNew != knobPos) {
    Serial.print("Mechanical knob = ");
    Serial.print(knobNew);
    Serial.println();
    knobPos = knobNew;
  }

  debouncer.update();
  int value = debouncer.read();
  if ( value == LOW ) {
    Serial.println("Limit switch depressed");
  }
  
  if (Serial.available() > 0) {
    int inByte = Serial.read();

    switch (inByte) {
      // Motor 2
      case '4': // Motor 2 Forward
        digitalWrite(motor2I3, LOW);
        digitalWrite(motor2I4, HIGH);
        Serial.println("Motor 2 Forward");
        break;
      case '5': // Motor 2 backward
        digitalWrite(motor2I3, HIGH);
        digitalWrite(motor2I4, LOW);
        Serial.println("Motor 2 Backward");
        break;
      case '6': // Motor 2 brake
        digitalWrite(motor2I3, LOW);
        digitalWrite(motor2I4, LOW);
        Serial.println("Motor 2 Brake");
        break;

      case '0': // reset
        hallEncoder.write(0);
        knobEncoder.write(0);
        Serial.println("Reset both encoders");
        break;

      default:
        Serial.println("Unknown character");
    }
  }
}


