#include <Servo.h>

Servo myservo;
int potPin = A0;
int servoPin = 9;
int val;

void setup() {
  Serial.begin(9600);
  myservo.attach(servoPin);
}

void loop() {
  // put your main code here, to run repeatedly:
  val = analogRead(potPin);
  val = map(val, 0, 1023, 0, 180);
  val = constrain(val, 0,180);
  myservo.write(val);
  Serial.println(val);
  delay(15);
}
