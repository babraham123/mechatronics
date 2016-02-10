// motor - GUI communication test
// Bereket Abraham

#define bufferLen 3
unsigned int buffer[bufferLen];
int curr = 0;

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < bufferLen; i++) {
    buffer[i] = 0;
  }
}

void loop() {
  while (Serial.available() > 0) {
    unsigned int cByte = Serial.read();
    Serial.println(cByte);
    if (cByte == 255) {
      char t = (char)buffer[(curr+bufferLen-2) % bufferLen];
      unsigned int val = buffer[(curr+bufferLen-1) % bufferLen] << 4;
      val = val + buffer[curr];

      performAction(t, val);
    } else {
      curr = (curr + 1) % bufferLen;
      buffer[curr] = cByte;
    }
  }
}

void performAction(char type, unsigned int value) {
  Serial.print(type);
  Serial.print("  |  ");
  Serial.println(value);
}
