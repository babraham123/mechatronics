import processing.serial.*;

final int NUM_VALUES = 200;
final int FRAME_W = 600;
final int FRAME_H = 400;
final int maxValue = 255;
Serial myPort;
String val = "";
boolean firstContact = false;
// send bytes, receive strings

void settings() {
  size(FRAME_W, FRAME_H);
}

void setup() {
  ellipseMode(CENTER);
  String[] ports = Serial.list();
  for (int i = 0; i < ports.length; i++) {
    println(ports[i]);
  }
  myPort = new Serial(this, ports[1], 9600);
  myPort.bufferUntil('\n'); 
}

void draw() {
  background(102);
  // mouseX, mouseY
  fill(200);
  ellipse(width/2, height/2, 50, 50);   

  textSize(40);
  fill(25);
  text("Bereket", 50,50);
}

void sendMessage(char type, int value) {
  if (firstContact == true) {
    // write to serial the state(which can be f,b,k,p,q,s) 
    // and the value which is the angle or the speed according to the motor chosen.
    byte val1 = byte(value >> 4);
    byte val2 = byte(value & 0x0F);
  
    byte[] message = {byte(type), val1, val2, byte(0xff)};
    myPort.write(message);
  }
}

void serialEvent(Serial myPort) {
  val = myPort.readStringUntil('\n');
  if (val == null) { return; }
  
  val = trim(val);
  if (firstContact == false) {
    //trim whitespace, formatting chars
    val = trim(val);
    if (val.equals("A")) {
      myPort.clear();
      firstContact = true;
      myPort.write("A");
      println("contact");
    }
  }
  else { // contact established
    println(val);
  }
}