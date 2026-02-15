import processing.serial.*;

Serial myPort;
float qW = 1.0, qX = 0.0, qY = 0.0, qZ = 0.0;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

void setup() {
  size(800, 600, P3D);
  // Change [0] to the correct index for your Arduino port
  println(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 115200);
  myPort.bufferUntil('\n');
}

void draw() {
  background(30);
  lights();
  
  translate(width/2, height/2, 0);
  
  roll_pitch();
  
  pushMatrix();
  rotateZ(pitch);
  rotateX(-roll);

  // Visualize the orientation
  draw_board();
  popMatrix();
}

/*
serial event:
each quaternion is separated by space and end of message is
indicated by newline '\n'
*/
void serialEvent(Serial myPort) {
  String inString = myPort.readStringUntil('\n');
  if (inString != null) {
    String[] data = split(trim(inString), ' ');
    if (data.length >= 4) {
      float qw = float(data[0]);
      float qx = float(data[1]);
      float qy = float(data[2]);
      float qz = float(data[3]);
      
      float magnitude = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
      
      qW = qw / magnitude;
      qX = qx / magnitude;
      qY = qy / magnitude;
      qZ = qz / magnitude;
      
      println(qW + " " + qX + " " + qY + " " + qZ); 
    }
  }
}

/*
calculate roll and pitch from quaternion
*/
void roll_pitch()
{
  roll = atan2(2*(qW*qX + qY*qZ),1 - 2*(qX*qX + qY*qY));
  pitch = asin(2*(qW*qY - qZ*qX));
  yaw = 0.0f;
}

/*
function to draw board geometry and features
*/
void draw_board()
{
  stroke(128);
  fill(0, 128, 4);
  box(150, 10, 150); // The "Body"
  fill(0,0,0);
  pushMatrix();
  translate(0, -5, 0);
  box(10, 5, 10);   // The IMU sensor on top
  popMatrix();
  
  pushMatrix();
  translate(40, -5, -30);
  box(50, 5, 25);   // The MCU on top
  popMatrix();
  
  pushMatrix();
  translate(40, -5, 50);
  box(15, 5, 20);   // The regulator on top
  popMatrix();
  
  pushMatrix();
  translate(70, -20, -70);
  box(10, 30, 10);   // The jumper pins on top
  translate(0, 0, 10);
  box(10, 30, 10);
  popMatrix();
  
  pushMatrix();
  translate(-70, -20, 70);
  box(10, 30, 10);   // The jumper pins on top
  translate(0, 0, -10);
  box(10, 30, 10);
  popMatrix();
}
