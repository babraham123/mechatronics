// Motor Control lab, GUI
// Bereket Abraham
// Anchit Sood

import processing.serial.*;

int []circlesX = {100, 200, 300, 400, 500};
String []motorText = {"RCServo", "Stepper", "DC", "Other1", "Other2"};
String centerText = "";
int circleY = 100;
int rectX = 50;
int rectY = 300;
int rectH = 50;
int rectW = 530;
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
char state;
int value = 0;
int motornumb = -1;
int stepperLimit = 200;
int dcLimit = 255;
int servoLimit = 180;
int delaytime = 500;
Serial myPort;

void setup() {
  size(640, 400);
  ellipseMode(CENTER);
  circleTextY = circleY-(circleDiameter/2)-20;
  String[] ports = Serial.list();
  for (int i = 0; i < ports.length; i++) {
    println(ports[i]);
  }
  myPort = new Serial(this, ports[1], 9600);
}

// draw() function draws the current state of the GUI based on current mouse position
void draw() {
  update();
  printSerial();
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

  if (motornumb > 0) {
      line(rectX + dcLimit, rectY, rectX + dcLimit, rectY + rectH);
      line(rectX + dcLimit + 20, rectY, rectX + dcLimit + 20, rectY + rectH);
    }

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


// update() function updates the values of the variables circleOver and rectOver; to check which circle/rectangle is active.
void update() {
  circleOver = -1;
  rectOver = false;
  for (int i = 0; i < circlesX.length; i++) {
    if (overCircle(circlesX[i], circleY, circleDiameter) ) {
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
    motornumb = circleOver;
  } 
  if (rectOver) {
    switch (motornumb) {
      case 0:
        // servo: angle between 0-180
        // change state to s
        state = 's';
        // change value based on position of cursor on rectangle
        value = int((float(mouseX - rectX)/rectW)*180);
          // split rectangle into left, right and middle 'gray' area 
        break;
      case 1:
        // stepper: angle between 200-0-200
        // change state to p,q based on position of mouse on rectangle
        // change value based on position of cursor on rectangle
        if (mouseX > rectX && mouseX < rectX + dcLimit) {
          state = 'q';
          value = int((float((dcLimit - (mouseX - rectX)))/dcLimit)*stepperLimit);  // ranges from 1 thru 255
        } else if (mouseX > (rectX + dcLimit + 20) && mouseX < (rectX + rectW)) {
          state = 'p';
          value = int((float((mouseX - (rectX + dcLimit + 20)))/dcLimit)*stepperLimit);  // ranges from 1 thru 200
        } else {
          state = 'p';
          value = 0;
        }
        break;
      case 2:
        // dc: speed between 255-0-255
        // change state to f,b,k based on position of mouse on rectangle
        // change value based on position of cursor on rectangle
        if (mouseX > rectX && mouseX < rectX + dcLimit) {
          state = 'b';
          value = dcLimit - (mouseX - rectX);  // ranges from 1 thru 255
        } else if (mouseX > (rectX + dcLimit + 20) && mouseX < (rectX + rectW)) {
          state = 'f';
          value = mouseX - (rectX + dcLimit + 20);  // ranges from 1 thru 255
        } else {
          state = 'k';
          value = 0;
          // implement force brake functionality if needed; right now it's a simple coast stop situation
        }
        break;
      default:
        // do nothing, really
        value = 0;
        return;
    }
    println(str(motornumb) + " " + str(value));
    
    // write to serial the state(which can be f,b,k,p,q,s) 
    // and the value which is the angle or the speed according to the motor chosen.
    byte val1 = byte(value >> 4);
    byte val2 = byte(value & 0x0F);

    byte[] message = {byte(state), val1, val2, byte(0xff)};
    myPort.write(message);
    
    delay(delaytime);
  }
}

void printSerial() {
  if ( myPort.available() > 0) {
    String msg = myPort.readStringUntil('\n');
    println(msg);
  } 
}

// overRect checks if mouse is over a rectangle based on input x and y coordinates
boolean overRect(int x, int y, int width, int height)  {
  if (mouseX >= x && mouseX <= x+width && 
      mouseY >= y && mouseY <= y+height) {
    return true;
  } else {
    return false;
  }
}

// overRect checks if mouse is over a circle based on input x and y coordinates
boolean overCircle(int x, int y, int diameter) {
  float disX = x - mouseX;
  float disY = y - mouseY;
  if (sqrt(sq(disX) + sq(disY)) < diameter/2 ) {
    return true;
  } else {
    return false;
  }
}