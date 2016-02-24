// Z Motion GUI
// Bereket Abraham
import processing.serial.*;

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
}

void draw() {
  background(50);
  mouseInput();
  drawRobot();
  
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
  rect(0,0, 10,height);
  rect(0,0, XYWidth,10);
  rect(XYWidth-5,0, 10,height);
  rect(XYWidth-5,height/2, height/3,10);
  rect(0,height, XYHeight,10);
}

float jointRange(float val, float range) {
  val = (float)constrain(val*range, 0, range) - (range/2.0);
  return val;
}