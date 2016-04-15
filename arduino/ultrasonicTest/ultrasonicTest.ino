
#include <NewPing.h>

const int trigX = 12;
const int echoX = 11;
NewPing sonarX(trigX, echoX, 8);

void setup() {
  Serial.begin(9600);
}

void loop() {
  long echo = sonarX.ping_median(5);
  long distance = sonarX.convert_cm(echo);
  Serial.print("echoX: ");
  Serial.print(echo);
  Serial.print("ms ");
  Serial.print("distanceX: ");
  Serial.print(distance);
  Serial.println("cm");
  if (target - echo > 13) {
    // move down
  } else if (target - echo < -18) {
    // move up
  } else {
    // stop
  }
  delay(100);
}


