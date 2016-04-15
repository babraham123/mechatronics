// Motor Control lab
// Bereket Abraham

#include <Encoder.h>

// uno: interrupt pins 2,3
// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder knobEncoder(2, 3);
long knobPos = -999;


void setup() {
  Serial.begin(9600);
}

void loop() {
  long knobNew;
  knobNew = knobEncoder.read();
  if (knobNew != knobPos) {
    Serial.print("Mechanical knob = ");
    Serial.print(knobNew);
    Serial.println();
    knobPos = knobNew;
  }
}


