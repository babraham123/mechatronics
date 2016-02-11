// Motor Control lab
// Bereket Abraham

#include <Encoder.h>
#include <PID_v1.h>

double setpoint = 75; 
double input = 0, output = 0;
double kp = 10, ki = 0.1, kd = 0.001;

long oldPos = 0;
unsigned long oldTime = 0;

//Specify the links and initial tuning parameters
PID motorPID(&input, &output, &setpoint, kp,ki,kd, DIRECT);

Encoder hallEncoder(2, 3);
Encoder knobEncoder(12, 13);
volatile long hallPos = -999;
long knobPos = -999;

const int motor2I3 = 4;
const int motor2I4 = 6;
const int speedPin2 = 5;

void setup() {
  Serial.begin(9600);

  pinMode(motor2I3,OUTPUT);
  pinMode(motor2I4,OUTPUT);
  pinMode(speedPin2,OUTPUT);
  // defaults
  Serial.println("Motor 2 Forward");
  digitalWrite(motor2I3, HIGH);
  digitalWrite(motor2I4, LOW);
  analogWrite(speedPin2, 200);

  //turn the PID on
  motorPID.SetMode(AUTOMATIC);

  knobEncoder.write(0);
}

void loop() {
  long knobNew = knobEncoder.read();
  if (knobNew != knobPos) {
    knobPos = knobNew;
    analogWrite(speedPin2, constrain(knobNew*10, 0,255));
  }

  input = (double)computeSpeed();
  motorPID.Compute();
  analogWrite(speedPin2, output);
  
  Serial.print(input);
  Serial.print("  |  ");
  Serial.println(output);
  delay(100);
}

// speed will scale with the overall loop time :(
double computeSpeed() {
  unsigned long newTime = millis();
  hallPos = hallEncoder.read();
  //Serial.println(hallPos);
  long vel = (hallPos - oldPos)*1000 / (long)(newTime - oldTime);
  oldPos = hallPos;
  oldTime = newTime;
  return map(vel, 0,1000, 0,100);
}

