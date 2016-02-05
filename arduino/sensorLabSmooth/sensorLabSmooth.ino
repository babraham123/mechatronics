/*
  sensorLab
 */
#define LM_SIZE 10
int LM[LM_SIZE];      // LastMeasurements
byte index = 0;
long sum = 0;
byte count = 0;

const int sensorPinIR = A0;
const int sensorPinPo = A1;
const int echoPin     = 2;
const int triggerPin  = 3;
unsigned long pulseTime = 0;
unsigned long distance  = 0;
int valueIR     = 0;
int valuePo     = 0;
int voltageIR   = 0;
int degreePo    = 0;
int distanceIR  = 0;

int mapAnalog  (int value);
int transferIR (int voltage);

void setup() {
  pinMode(echoPin, INPUT);
  pinMode(triggerPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  // Ultrasound Range Finder
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pulseTime = pulseIn(echoPin, HIGH);
  distance = pulseTime/(58.2);
  
  // IR Proximity Sensor
  valueIR    = analogRead(sensorPinIR);
  voltageIR  = mapAnalog(valueIR);
  distanceIR = transferIR(valueIR);
  
  // Potentiometer
  valuePo     = analogRead(sensorPinPo);
  degreePo   = mapRotary(valuePo);
  
  // Print to serial monitor
  Serial.print("IR = ");
  distanceIR = runningAverage(distanceIR);
  Serial.print(distanceIR);
  Serial.print("cm; ");
  Serial.print("Potent = ");
  Serial.print(degreePo);
  Serial.print("degrees;");
  Serial.print(" Ultrasound = ");
  Serial.print(distance);
  Serial.println("cm");
  
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

long runningAverage(int M) {
  // keep sum updated to improve speed.
  sum -= LM[index];
  LM[index] = M;
  sum += LM[index];
  index = (index+1) % LM_SIZE;
  if (count < LM_SIZE) count++;

  return sum / count;
}

