//Debug Constants
int WIREGRID_SIZE = 500; // The size of each cube of the wiregrid
int WIREGRID_DIST = 3; // The maximum number of cubes of the wiregrid in each direction

float timestep;
float maxtime;

float posX = 0;
float posY = 0;
float posH = 0;

float rotX = 0;
float rotY = 0;
float rotZ = 0;

BufferedReader reader;
String line;
PShape s;

void setup() {
  size(800, 800, P3D);
  s = loadShape("drone.obj");
  reader = createReader("../data.csv");
  try {
    line = reader.readLine();
  } catch (IOException e) {
    e.printStackTrace();
    line = null;
  }
  
  if (line == null) {
    exit(); 
  } else {
    String[] pieces = split(line, ',');
    timestep = float(pieces[0]);
    maxtime = float(pieces[1]);
  }
}

void draw() {
  delay(int(timestep * 1000));
  
  float real_posX = 0;
  float real_posY = 0;
  float real_posH = 0;
  float real_rotX = 0;
  float real_rotY = 0;
  float real_rotZ = 0;
  
  try {
    line = reader.readLine();
  } catch (IOException e) {
    e.printStackTrace();
    line = null;
  }
  
  if (line == null) {
    exit(); 
  } else {
    String[] pieces = split(line, ',');
    real_posX = float(pieces[0]);
    real_posY = float(pieces[1]);
    real_posH = float(pieces[2]);
    real_rotX = float(pieces[3]);
    real_rotY = float(pieces[4]);
    real_rotZ = float(pieces[5]);
  }
  
  posX = real_posX * 100;
  posY = real_posY * 100;
  posH = real_posH * 100;
  
  // ---- RENDERING ----
  background(84, 152, 255); //Refresh Screen
  //render 'drone'
  pushMatrix();
  translate(posY, -posH, -posX);
  rotateX(-real_rotY);
  rotateY(real_rotZ);
  rotateZ(real_rotX);
  fill(255);
  stroke(0);
  box(50,10,50);
  stroke(255, 0, 0);
  line(0, 0, 0, 0, 0, -100);
  stroke(0, 255, 0);
  line(0, 0, 0, -100, 0, 0);
  stroke(0, 0, 255);
  line(0, 0, 0, 0, 100, 0);
  popMatrix();
  camera(posY, -posH, -posX+800 , posY, -posH, -posX, 0, 1, 0);
  
  // --- DEBUG ---
  strokeWeight(2); // Debug stroke weight
  
  text("bank: " + real_rotX, posY + 10 - width/2, -posH + 10 - height/2, -posX);
  text("attitude: " + real_rotY, posY + 10 - width/2, -posH + 20 - height/2, -posX);
  text("heading: " + real_rotZ, posY + 10 - width/2, -posH + 30 - height/2, -posX);
  text("x: " + real_posY, posY + 200 - width/2, -posH + 10 - height/2, -posX);
  text("y: " + -real_posH, posY + 200 - width/2, -posH + 20 - height/2, -posX);
  text("h: " + -real_posX, posY + 200 - width/2, -posH + 30 - height/2, -posX);
  
  
  
  // Draw wiregrid to give sense of velocity
  noFill();
  int rx = round(posY/WIREGRID_SIZE) * WIREGRID_SIZE;
  int ry = round(-posH/WIREGRID_SIZE) * WIREGRID_SIZE;
  int rz = round(-posX/WIREGRID_SIZE) * WIREGRID_SIZE;
  stroke(255, 0, 0);
  for (int i = -WIREGRID_DIST; i < WIREGRID_DIST; i++) {
    for (int j = -WIREGRID_DIST; j < WIREGRID_DIST; j++) {
      for (int k = -WIREGRID_DIST; k < WIREGRID_DIST; k++) {
        pushMatrix();
        translate(rx + (i * WIREGRID_SIZE) , ry + (j * WIREGRID_SIZE), rz + (k * WIREGRID_SIZE));
        box(WIREGRID_SIZE);
        popMatrix();
      }
    }
  }
}
