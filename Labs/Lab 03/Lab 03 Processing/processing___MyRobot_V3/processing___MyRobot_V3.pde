import processing.serial.*;

Serial serialPort;
boolean connected = false;

// Choose the Arduino serial port here (see console output from Serial.list())
int portIndex = 0;      // <<< CHANGE THIS if needed
int baud = 115200;

void setup() {
  size(420, 220);
  textAlign(CENTER, CENTER);
  textSize(16);

  connectToArduino();
}

void draw() {
  background(25);
  fill(255);

  if (connected) {
    text("CONNECTED\nWASD to drive\nRelease to stop",
         width/2, height/2);
  } else {
    text("NOT CONNECTED\nCheck USB / port",
         width/2, height/2);
  }
}

void connectToArduino() {
  try {
    println("Available serial ports:");
    println(Serial.list());

    println("Connecting to Arduino...");
    String portName = Serial.list()[portIndex];
    serialPort = new Serial(this, portName, baud);

    // On many boards, opening Serial resets the Arduino.
    // Clear any boot text.
    serialPort.clear();
    delay(400);

    connected = true;
    println("Connected!");
    println("Port: " + portName);
  } catch (Exception e) {
    println("Connection failed");
    e.printStackTrace();
    connected = false;
  }
}

void keyPressed() {
  if (!connected) return;

  try {
    char k = Character.toLowerCase(key);

    if (k == 'w') serialPort.write('U');
    else if (k == 's') serialPort.write('D');
    else if (k == 'a') serialPort.write('L');
    else if (k == 'd') serialPort.write('R');

  } catch (Exception e) {
    connected = false;
  }
}

void keyReleased() {
  if (!connected) return;

  try {
    serialPort.write('S');
  } catch (Exception e) {
    connected = false;
  }
}
