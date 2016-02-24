// Z Motion GUI
// Bereket Abraham
import processing.serial.*;
Serial myPort;

float robotXOffset = 0;
float robotYOffset = 0;
float footXOffset = 0;
float footYOffset = 0;
float robotScale = 14.0/10.0;
float robotAngle = 0;
float robotXJoint = 0; // [0,1]
float robotYJoint = 0; // [0,1]
float robotZJoint = 0; // [0,1]
float XYWidth = 0;
float XYHeight = 0;
float ZWidth = 0;
float ZHeight = 0;
boolean xFixed = false;
boolean yFixed = false;

void setup() {
  size(1200, 700);
  XYWidth = (float)width * (2.0/3.0);
  XYHeight = (float)height / 3.0;
  ZHeight = (float)height / 2.0;
  robotXOffset = XYWidth/2;
  robotYOffset = (float)height/2;
  footXOffset = (float)width * (5.0/6.0);
  footYOffset = (float)height/4;
  
  String[] ports = Serial.list();
  for (int i = 0; i < ports.length; i++) {
    println(ports[i]);
  }
  myPort = new Serial(this, ports[1], 9600);
}

void draw() {
  background(50);
  mouseInput();
  serialInput();
  drawRobot();
  serialOutput();
}

void serialInput() {
  while ( myPort.available() > 0) 
  {
    String val = myPort.readStringUntil('\n');
    if(val == null || val.length() < 2) {continue;};
    println(val);
    int nVal = Integer.parseInt(val.substring(1, val.length()-1));
    switch (val.charAt(0)) {
      case 'n':
        if (nVal < 10) {
          println("North near edge");
        }
        break;
      case 's':
        if (nVal < 10) {
          println("South near edge");
        }
        break;
      case 'w':
        if (nVal < 10) {
          println("West near edge");
        }
        break;
      case 'e':
        if (nVal < 10) {
          println("East near edge");
        }
        break;
      default:
        break;
    }
  }
}

void serialOutput() {
  myPort.write(str(robotZJoint*70 + 100));
}

void keyPressed() {
  switch (key) {
    case 'r':
      // reset offset to 0
      break;
    case 's':
      // send joint values to robot
      break;
    case 'x':
      // fix X rail
      break;
    case 'y':
      // fix y rail
      break;
    default:
      println("Unknown key");
      break;
  }
}

void mouseInput() {
  if (mouseX < XYWidth) {
    robotXJoint = (float)constrain(mouseX, 0, XYWidth) / XYWidth;
    robotYJoint = (float)constrain(mouseY, (height/2)-XYHeight, (height/2)+XYHeight);
    robotYJoint = robotYJoint / XYHeight / 2.0;
  } else if (mouseY < ZHeight) {
    robotZJoint = (float)constrain(mouseY, 0, ZHeight) / ZHeight;
  } else {
    // do something in the remaining space
  }
}

void drawRobot() {
  drawBounds();
  
  pushMatrix();
  translate(robotXOffset, robotYOffset);
  scale(robotScale);
  drawXYComponents();
  popMatrix();
  
  pushMatrix();
  translate(footXOffset, footYOffset);
  scale(robotScale*2.0);
  drawZComponents();
  popMatrix();
}

void drawXYComponents() {
  // railings
  pushMatrix();
  translate(0, jointRange(robotYJoint,70));
  drawYRail();
  popMatrix();
  
  pushMatrix();
  translate(jointRange(robotXJoint,70), 0);
  drawXRail();
  popMatrix();
  
  //center box
  fill(200);
  rect(-35,-35, 70,70);
  
  // draw any shape
  //beginShape();
  //vertex(sx, sy);
  //endShape(CLOSE);
}

void drawYRail() {
  fill(204, 102, 0); // orange
  rect(-26.25,-80, 3.75,160);
  rect(26.25,-80, 3.75,160);
  
  rect(-35,-80, 70,10);
  rect(-35,70, 70,10);
}

void drawXRail() {
  fill(200);
  rect(-85,-80, 15,160);
  rect(70,-80, 15,160);
  
  fill(204, 102, 0);
  rect(-80,-26.25, 160,3.75);
  rect(-80,26.25, 160,3.75);
  
  rect(-80,-35, 10,70);
  rect(70,-35, 10,70);
}

void drawZComponents() {
  // bottom part
  pushMatrix();
  translate(0, jointRange(robotZJoint, 15));
  drawLowerRail();
  popMatrix();
  
  //top part
  fill(200);
  ellipse(8,-8, 16,16);
  rect(-35,0, 70,10);
  
}

void drawLowerRail() {
  fill(200);
  rect(0,-16, 4,41);
  fill(204, 102, 0);
  rect(-20,0, 3.75,25);
  rect(20,0, 3.75,25);
  
  rect(-35,25, 70,5);
  triangle(-22.5,25, -23.5,27, -21.5,27);
  triangle(22.5,25, 21.5,27, 23.5,27);
}

void drawBounds() {
  fill(100);
  rect(0,0, 20,height);
  rect(0,0, XYWidth,20);
  rect(XYWidth-10,0, 20,height);
  rect(XYWidth-10,height/2, width/3,20);
  rect(0,height-20, XYWidth,20);
}

float jointRange(float val, float range) {
  val = (float)constrain(val*range, 0, range) - (range/2.0);
  return val;
}