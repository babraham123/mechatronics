/*
  sensorLab
 */

const int sensorPinIR = A0;
const int sensorPinPo = A1;
const int sensorPin2  = A2;
int valueIR     = 0;
int valuePo     = 0;
int value2      = 0;
int voltageIR   = 0;
int degreePo    = 0;
int voltage2    = 0;
int distanceIR  = 0;

int mapAnalog  (int value);
int transferIR (int voltage);

void setup() {
  Serial.begin(9600);
}

void loop() {
  // read the value from the sensor:
  valueIR    = analogRead(sensorPinIR);
  voltageIR  = mapAnalog(valueIR);
  distanceIR = transferIR(valueIR);
  valuePo     = analogRead(sensorPinPo);
  degreePo   = mapRotary(valuePo);
  value2     = analogRead(sensorPin2);
  voltage2   = mapAnalog(value2);
  Serial.print("IR distance = ");
  Serial.print(distanceIR);
  Serial.print(" cm; ");
  Serial.print("Rotary Position = ");
  Serial.print(degreePo);
  Serial.print(" degrees");
  Serial.print(" sensor2 = ");
  Serial.println(voltage2);
  Serial.println();
  delay(100);
}

int mapAnalog(int value) {
  return map(value, 0, 1023, 0, 5000);
}

int mapRotary(int value) {
 return map(value, 0, 1023, 0, 300); 
}

int transferIR(int voltage) {
 return (1 / (0.0002391473 * voltage - 0.0100251467)) + 1;
}
