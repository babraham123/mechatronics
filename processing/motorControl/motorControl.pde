// Motor Control lab, GUI
// Bereket Abraham

int []circlesX = {100, 200, 300, 400, 500};
String []motorText = {"RCServo", "Stepper", "DC", "Other1", "Other2"};
String centerText = "";
int circleY = 100;
int rectX = 50;
int rectY = 300;
int rectH = 50;
int rectW = 500;
int textX = 300;
int circleTextY = 20;
int circleDiameter = 50;
color rectColor = color(0);
color circleColor = color(255);
color baseColor = color(102);
color circleHighlight = color(204);
color rectHighlight = color(51);
color currentColor = baseColor;
boolean rectOver = false;
int circleOver = -1;

void setup() {
  size(640, 400);
  ellipseMode(CENTER);
  circleTextY = circleY-(circleDiameter/2)-20;
}

void draw() {
  update(mouseX, mouseY);
  background(baseColor);
  
  if (rectOver) {
    fill(rectHighlight);
  } else {
    fill(rectColor);
  }
  stroke(255);
  rect(rectX, rectY, rectW, rectH);

  textSize(40);
  fill(25);
  text(centerText, textX, rectY-50);
  textSize(24);

  for (int i = 0; i < circlesX.length; i++) {
    fill(25);
    text(motorText[i], circlesX[i]-(circleDiameter/2)-10, circleTextY);

    if (circleOver == i) {
      fill(circleHighlight);

    } else {
      fill(circleColor);
    }
    stroke(0);
    ellipse(circlesX[i], circleY, circleDiameter, circleDiameter);
  }
}

void update(int x, int y) {
  circleOver = -1;
  rectOver = false;
  for (int i = 0; i < circlesX.length; i++) {
    if ( overCircle(circlesX[i], circleY, circleDiameter) ) {
      circleOver = i;
      centerText = motorText[i];
      break;
    }
  }
  if (circleOver < 0 && overRect(rectX, rectY, rectW, rectH)) {
    rectOver = true;
  }
}

void mousePressed() {
  if (circleOver >= 0) {
    // switch motor
  }
  if (rectOver) {
    // send value
  }
}

boolean overRect(int x, int y, int width, int height)  {
  if (mouseX >= x && mouseX <= x+width && 
      mouseY >= y && mouseY <= y+height) {
    return true;
  } else {
    return false;
  }
}

boolean overCircle(int x, int y, int diameter) {
  float disX = x - mouseX;
  float disY = y - mouseY;
  if (sqrt(sq(disX) + sq(disY)) < diameter/2 ) {
    return true;
  } else {
    return false;
  }
}