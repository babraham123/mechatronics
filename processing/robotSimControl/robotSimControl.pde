// Icarus Window Washer GUI
// Mechatronic Design
// Bereket Abraham

import processing.serial.*;
Serial myPort;
Robot winBot;
Foot footBot;
Graph vacuumX, vacuumY, positionX, positionY;
Graph ultrasonicN, ultrasonicS, ultrasonicE, ultrasonicW;
Graph currGraph;
final int NUM_VALUES = 200;
int xExtreme = -1;
int yExtreme = -1;

void setup() {
  size(1200, 700);
  float xyWidth = (float)width * (2.0/3.0);
  float zWidth = (float)width - xyWidth;
  float zHeight = (float)height / 2.0;
  float gHeight = (float)height - zHeight;

  // pass in display bounds
  winBot = new Robot(0, 0, xyWidth, (float)height);
  footBot = new Foot(xyWidth, 0, zWidth, zHeight);
  
  // initialize real time graphs
  vacuumX = new Graph(xyWidth, zHeight, zWidth, gHeight, "X Vacuum Pressure");
  vacuumY = new Graph(xyWidth, zHeight, zWidth, gHeight, "Y Vacuum Pressure");
  positionX = new Graph(xyWidth, zHeight, zWidth, gHeight, "X Railing Position");
  positionY = new Graph(xyWidth, zHeight, zWidth, gHeight, "Y Railing Position");
  ultrasonicN = new Graph(xyWidth, zHeight, zWidth, gHeight, "North Distance Sensor");
  ultrasonicS = new Graph(xyWidth, zHeight, zWidth, gHeight, "South Distance Sensor");
  ultrasonicE = new Graph(xyWidth, zHeight, zWidth, gHeight, "East Distance Sensor");
  ultrasonicW = new Graph(xyWidth, zHeight, zWidth, gHeight, "West Distance Sensor");
  currGraph = vacuumX;

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

  currGraph.display();
  footBot.display();
  footBot.drawBounds();
  winBot.drawBounds();
  winBot.display();
}

void serialInput() {
  int k = 0;
  while (myPort.available() > 0) 
  {
    String val = myPort.readStringUntil('\n');
    if (val == null || val.length() < 2) { continue; }
    println(val);
    int nVal = Integer.parseInt(val.substring(1, val.length()-1));
    switch (val.charAt(0)) {
      case 'u':
        vacuumX.update(nVal);
        break;
      case 'v':
        vacuumY.update(nVal);
        break;
      case 'x':
        positionX.update(nVal);
        break;
      case 'y':
        positionY.update(nVal);
        break;
      case 'X':
        xExtreme = nVal;
        break;
      case 'Y':
        yExtreme = nVal;
        break;
      case 'n':
        ultrasonicN.update(nVal);
        break;
      case 's':
        ultrasonicS.update(nVal);
        break;
      case 'e':
        ultrasonicE.update(nVal);
        break;
      case 'w':
        ultrasonicW.update(nVal);
        break;
      case 'N':
        if (nVal > 0) {
          winBot.edgeN = true;
        } else {
          winBot.edgeN = false;
        }
        break;
      case 'S':
        if (nVal > 0) {
          winBot.edgeS = true;
        } else {
          winBot.edgeS = false;
        }
        break;
      case 'E':
        if (nVal > 0) {
          winBot.edgeE = true;
        } else {
          winBot.edgeE = false;
        }
        break;
      case 'W':
        if (nVal > 0) {
          winBot.edgeW = true;
        } else {
          winBot.edgeW = false;
        }
        break;
      default:
        break;
    }
    k++;
    if (k > 5) { break; }
  }
}

void serialOutput(String msg) {
  myPort.write(msg);
}

void keyPressed() {
  switch (key) {
    case 'u':
      currGraph = vacuumX;
      break;
    case 'v':
      currGraph = vacuumY;
      break;
    case 'x':
      currGraph = positionX;
      break;
    case 'y':
      currGraph = positionY;
      break;
    case 'n':
      currGraph = ultrasonicN;
      break;
    case 's':
      currGraph = ultrasonicS;
      break;
    case 'e':
      currGraph = ultrasonicE;
      break;
    case 'w':
      currGraph = ultrasonicW;
      break;
    // testing
    case 'a':
      winBot.edgeN = !winBot.edgeN;
      currGraph.update(50);
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
  boolean edgeN = false;
  boolean edgeS = false;
  boolean edgeE = false;
  boolean edgeW = false;

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
    if (edgeW) { fill(255,0,0); }
    rect(x, y, 20, h);

    fill(100);
    if (edgeN) { fill(255,0,0); }
    rect(x, y, w, 20);

    fill(100);
    if (edgeE) { fill(255,0,0); }
    rect(x+w-20, y, 20, h);

    fill(100);
    if (edgeS) { fill(255,0,0); }
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
    scale = ((float)min(width,height)) / 350;
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
    
    fill(170);
    if (yFixed) { fill(255,0,0); }
    rect(-35,-80, 70,10);
    rect(-35,70, 70,10);
  }

  void drawXRail() {
    fill(170);
    if (xFixed) { fill(255,0,0); }
    rect(-85,-80, 15,160);
    rect(70,-80, 15,160);
    
    fill(204, 102, 0);
    rect(-80,-26.25, 160,3.75);
    rect(-80,26.25, 160,3.75);
    
    fill(200);
    rect(-80,-35, 10,70);
    rect(70,-35, 10,70);
  }
}

class Foot extends Mechanism {
  float zJoint = 0; // [0,1]
  boolean fixed = false;

  Foot(float xx, float yy, float ww, float hh) {
    super(xx, yy, ww, hh);
    scale = ((float)min(width,height)) / 200;
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
    
    fill(170);
    if (fixed) { fill(255,0,0); }
    rect(-35,25, 70,5);
    triangle(-22.5,25, -24.5,28, -20.5,28);
    triangle(22.5,25, 20.5,28, 24.5,28);
  }
}

class Graph {
  float x, y, w, h;
  float scale = 1.0;
  float barWidth = 10;
  int[] serialVal = new int[NUM_VALUES];
  int curr = 0;
  int maxValue = 255;
  int dataColor = 0;
  String title = "Sensor Value";

  Graph(float xx, float yy, float ww, float hh, String ttitle) {
    x = xx;
    y = yy;
    w = ww;
    h = hh;
    title = ttitle;
    barWidth = w / (float) NUM_VALUES;
    setMaxValue(255);

    for (int j = 0; j < NUM_VALUES; j++) {
      serialVal[j] = 0;
    }
  }

  void display() {
    pushMatrix();
    translate(x, y);
    drawGraph();
    popMatrix();
  }

  void drawGraph() {
    fill(dataColor);
    for (int j = 0; j < NUM_VALUES; j++)
    {
      int i = (j+1+curr) % NUM_VALUES;
      fill(0);
      rect(j*barWidth, 30 + scale*(maxValue-serialVal[i]), barWidth, scale*serialVal[i]);
    }

    fill(255,255,255);
    textSize(24);
    text(title, w/3, 20);
    text(str(maxValue), 5, 20);
  }

  void update(int reading) {
    serialVal[curr] = reading;
    curr = (curr + 1) % NUM_VALUES;
  }

  void setMaxValue(int maxVal) {
    maxValue = maxVal;
    scale = (h-30) / (float) maxValue;
  }
}

