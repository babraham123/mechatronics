// Interrupt test
// Bereket Abraham

int pin = 2;
volatile int state = LOW;

void setup() {
    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), changeTrigger, CHANGE);
    Serial.begin(9600);
}

void loop() {
    //
}

void changeTrigger() {
    Serial.println("Triggered!");
    state = !state;
}
