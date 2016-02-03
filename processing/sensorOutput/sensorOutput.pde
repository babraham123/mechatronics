import processing.serial.*;

final int NUM_VALUES = 200;
final int FRAME_W = 400;
final int FRAME_H = 400;

int[] serialVal = new int[NUM_VALUES];
Serial myPort;
int curr = 0;
int width = 2;

void settings() {
  size(FRAME_W, FRAME_H);
}

void setup() {
  //String portName = "COM14";
  String portName = Serial.list()[1];
  print(portName);
  myPort = new Serial(this, portName, 9600);
  width = FRAME_W / NUM_VALUES;
  
  for (int j = 0; j < NUM_VALUES; j++)
  {
    serialVal[j] = 0;
  }
}

void draw() {
  background(130);
  while(true)
  {
    if(myPort.available()>0)
    {
      serialVal[curr] = myPort.read();
      curr++;
      if (curr % 5 == 0) {
        break;
      }
    }
  }
  
  for (int j = 0; j < NUM_VALUES; j++)
  {
    int i = (j+1+curr) % NUM_VALUES;
    fill(0);
    rect(j*width + 50, 75, width, serialVal[i]);
  }

}