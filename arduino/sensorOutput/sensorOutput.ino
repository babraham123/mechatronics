int count = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(100);
  count = (count + 1) % 255;
  Serial.print(count);
  //Serial.println(outputValue);
}


