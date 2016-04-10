#include <EnableInterrupt.h>

// Refer to https://github.com/GreyGnome/EnableInterrupt/wiki/Usage#Summary
#define ARDUINOPIN A0

volatile uint16_t interruptCount=0; // The count will go back to 0 after hitting 65535.

void changeTrigger() {
  interruptCount++;
  Serial.println("Triggered!");
}

void setup() {
  Serial.begin(9600);
  pinMode(ARDUINOPIN, INPUT_PULLUP);  // See http://arduino.cc/en/Tutorial/DigitalPins
  enableInterrupt(ARDUINOPIN, changeTrigger, CHANGE);
}

void loop() {
  //
}

