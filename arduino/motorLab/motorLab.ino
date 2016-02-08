// Motor Control lab
// Bereket Abraham

//Keyboard Controls:
// 1 -Motor 1 Left
// 2 -Motor 1 Stop
// 3 -Motor 1 Right
//
// 4 -Motor 2 Left
// 5 -Motor 2 Stop
// 6 -Motor 2 Right

// Declare L298N Dual H-Bridge Motor Controller
// Motor 1
int motor1I1 = 7;
int motor1I2 = 8;
int speedPin1 = 9;
// Motor 2
int motor2I3 = 4;
int motor2I4 = 2;
int speedPin2 = 3;

void setup() {
  Serial.begin(9600);
  //Define L298N Dual H-Bridge Motor Controller Pins
  pinMode(motor1I1,OUTPUT);
  pinMode(motor1I2,OUTPUT);
  pinMode(speedPin1,OUTPUT);
  pinMode(motor2I3,OUTPUT);
  pinMode(motor2I4,OUTPUT);
  pinMode(speedPin2,OUTPUT);

  analogWrite(speedPin1, 0);
  analogWrite(speedPin2, 0);
  digitalWrite(motor1I1, LOW);
  digitalWrite(motor1I2, HIGH);
  digitalWrite(motor2I3, LOW);
  digitalWrite(motor2I4, HIGH);
}

void loop() {
  if (Serial.available() > 0) {
    int inByte = Serial.read();
    int speed; // Local variable

    switch (inByte) {
      // Motor 1
      case '1': // Motor 1 Forward
        analogWrite(speedPin1, 255);
        digitalWrite(motor1I1, LOW);
        digitalWrite(motor1I2, HIGH);
        Serial.println("Motor 1 Forward");
        break;

      case '2': // Motor 1 Stop (Freespin)
        analogWrite(speedPin1, 0);
        digitalWrite(motor1I1, LOW);
        digitalWrite(motor1I2, HIGH);
        Serial.println("Motor 1 Stop");
        break;

      case '3': // Motor 1 Reverse
        analogWrite(speedPin1, 255);
        digitalWrite(motor1I1, HIGH);
        digitalWrite(motor1I2, LOW);
        Serial.println("Motor 1 Reverse");
        break;

      // Motor 2
      case '4': // Motor 2 Forward
        analogWrite(speedPin2, 255);
        digitalWrite(motor2I3, LOW);
        digitalWrite(motor2I4, HIGH);
        Serial.println("Motor 2 Forward");
        break;

      case '5': // Motor 1 Stop (Freespin)
        analogWrite(speedPin2, 0);
        digitalWrite(motor2I3, LOW);
        digitalWrite(motor2I4, HIGH);
        Serial.println("Motor 2 Stop");
        break;

      case '6': // Motor 2 Reverse
        analogWrite(speedPin2, 255);
        digitalWrite(motor2I3, HIGH);
        digitalWrite(motor2I4, LOW);
        Serial.println("Motor 2 Reverse");
        break;

      default:
        Serial.println("Unknown character");
    }
  }
}

