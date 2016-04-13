// Z Motion GUI
// Bereket Abraham
import processing.serial.*;
Serial myPort;
Robot winBot;
Foot footBot;
Graph rtPlot;
final int NUM_VALUES = 200;

void setup() {
  size(1200, 700);
  float xyWidth = (float)width * (2.0/3.0);
  float zWidth = (float)width - xyWidth;
  float zHeight = (float)height / 2.0;

  // pass in display bounds
  winBot = new Robot(0, 0, xyWidth, (float)height);
  footBot = new Foot(xyWidth, 0, zWidth, zHeight);
  
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

  footBot.display();
  footBot.drawBounds();
  winBot.drawBounds();
  winBot.display();
  
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
  myPort.write(str(footBot.zJoint*70 + 100));
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
  if (winBot.inRange(mouseX, mouseY)) {
    winBot.xJoint = winBot.normalizeX(mouseX);
    winBot.yJoint = winBot.normalizeY(mouseY);
  } else if (footBot.inRange(mouseX, mouseY)) {
    footBot.zJoint = footBot.normalizeY(mouseY);
  } else {
    // do something in the remaining space
  }
}

abstract class Mechanism {
  float x, y, w, h;
  float scale = 1.0;
  float angle = 0;
  float xOffset = 0;
  float yOffset = 0;

  Mechanism(float xx, float yy, float ww, float hh) {
    x = xx;
    y = yy;
    w = ww;
    h = hh;
    xOffset = x + (w/2);
    yOffset = y + (h/2);
  }

  void display() {
    pushMatrix();
    translate(xOffset, yOffset);
    scale(scale);
    drawComponents();
    popMatrix();
  }

  abstract void drawComponents();

  void drawBounds() {
    fill(100);
    rect(x, y, 20, h);
    rect(x, y, w, 20);
    rect(x+w-20, y, 20, h);
    rect(x, y+h-20, w, 20);
  }

  float jointRange(float val, float range) {
    val = (float)constrain(val*range, 0, range) - (range/2.0);
    return val;
  }

  boolean inRange(int xi, int yi) {
    float xj = ((float)xi) - x;
    float yj = ((float)yi) - y;
    return ((xj >= 0) && (yj >= 0) && (xj < w) && (yj < h));
  }

  float normalizeX(int xi) {
    float xj = (float)constrain(xi - x - (w/2), -w/3, w/3);
    return xj / (w/3);
  }

  float normalizeY(int yi) {
    float yj = (float)constrain(yi - y - (h/2), -h/3, h/3);
    return yj / (h/3);
  }
}

class Robot extends Mechanism {
  float xJoint = 0; // [0,1]
  float yJoint = 0; // [0,1]
  boolean xFixed = false;
  boolean yFixed = false;

  Robot(float xx, float yy, float ww, float hh) {
    super(xx, yy, ww, hh);
    scale = ((float)min(width,height)) / 700;
  }

  @Override
  void drawComponents() {
    // railings
    pushMatrix();
    translate(0, jointRange(yJoint,70));
    drawYRail();
    popMatrix();
    
    pushMatrix();
    translate(jointRange(xJoint,70), 0);
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
}

class Foot extends Mechanism {
  float zJoint = 0; // [0,1]
  boolean fixed = false;

  Foot(float xx, float yy, float ww, float hh) {
    super(xx, yy, ww, hh);
    scale = ((float)min(width,height)) / 300;
  }

  @Override
  void drawComponents() {
    // bottom part
    pushMatrix();
    translate(0, jointRange(zJoint, 15));
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
    triangle(-22.5,25, -24.5,28, -20.5,28);
    triangle(22.5,25, 20.5,28, 24.5,28);
  }
}

class Graph {
  float x, y, w, h;
  float scale = 1.0;
  int[] serialVal = new int[NUM_VALUES];
  int curr = 0;
  int maxValue = 255;

  Mechanism(float xx, float yy, float ww, float hh) {
    x = xx;
    y = yy;
    w = ww;
    h = hh;
    for (int j = 0; j < NUM_VALUES; j++) {
      serialVal[j] = 0;
    }
  }

  void display() {
    pushMatrix();
    translate(xOffset, yOffset);
    scale(scale);
    drawGraph();
    popMatrix();
  }

  void drawGraph() {
    fill(0);
    for (int j = 0; j < NUM_VALUES; j++)
    {
      int i = (j+1+curr) % NUM_VALUES;
      fill(0);
      rect(j*width + 10, 15 + (maxValue-serialVal[i]), width, serialVal[i]);
    }
  }
}

