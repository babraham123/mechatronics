// Motor Control lab
// Bereket Abraham

#include <Encoder.h>
#include <PID_v1.h>

double setpoint, input, output;
double kp=3,ki=0,kd=0.0;
//Specify the links and initial tuning parameters
PID motorPID(&input, &output, &setpoint, kp,ki,kd, DIRECT);

Encoder hallEncoder(2, 3);
Encoder knobEncoder(12, 13);
volatile long hallPos = -999;
long knobPos = -999;
long previousMillis = 0;

const int motor2I3 = 4;
const int motor2I4 = 6;
const int speedPin2 = 5;

void setup() {
  Serial.begin(9600);

  pinMode(motor2I3,OUTPUT);
  pinMode(motor2I4,OUTPUT);
  pinMode(speedPin2,OUTPUT);
  // defaults
  analogWrite(speedPin2, 0);
  Serial.println("Motor 2 Forward");
  digitalWrite(motor2I3, LOW);
  digitalWrite(motor2I4, HIGH);

  input = analogRead(0);
  setpoint = 100;
  //turn the PID on
  motorPID.SetMode(AUTOMATIC);
}

void loop() {
  long knobNew;
  hallPos = hallEncoder.read();
  knobNew = knobEncoder.read();
  if (knobNew != knobPos) {
    knobPos = knobNew;
  }

  input = computeSpeed(hallPos);
  motorPID.Compute();
  analogWrite(speedPin2, output);
}

double computeSpeed(long hallPos) {
  //
  return 0.0;
}

