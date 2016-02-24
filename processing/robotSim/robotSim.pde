// Z Motion GUI
// Bereket Abraham
import processing.serial.*;

float robotXOffset = 0;
float robotYOffset = 0;
float robotScale = 14.0/10.0;
float robotAngle = 0;
float robotXJoint = 0;
float robotYJoint = 0;
float robotZJoint = 0;

void setup() {
  size(1200, 700);
}

void draw() {
  background(50);
  robotXOffset = width/2;
  robotYOffset = height/2;
  robotXJoint = (float)mouseX/ (float)width;
  robotYJoint = (float)mouseY/ (float)height;
  
  pushMatrix();
  translate(robotXOffset, robotYOffset);
  scale(robotScale);
  drawRobot();
  popMatrix();
  
}

void drawRobot() {
  // railings
  pushMatrix();
  translate(0, constrain(robotYJoint*70,0,70)-35);
  drawYRail();
  popMatrix();
  
  pushMatrix();
  translate(constrain(robotXJoint*70,0,70)-35, 0);
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