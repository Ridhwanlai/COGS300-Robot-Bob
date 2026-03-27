import java.net.*;
import java.io.*;

Socket socket;
OutputStream out;
boolean connected = false;

String arduinoIP = "10.0.0.62"; // <<< PUT REAL IP HERE
int port = 5200;

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
  } 
  else {
    text("NOT CONNECTED\nCheck IP / WiFi",
         width/2, height/2);
  }
}

void connectToArduino() {
  try {
    println("Connecting to Arduino...");
    socket = new Socket(arduinoIP, port);
    out = socket.getOutputStream();
    connected = true;
    println("Connected!");
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

    if (k == 'w') out.write('U');
    else if (k == 's') out.write('D');
    else if (k == 'a') out.write('L');
    else if (k == 'd') out.write('R');

    out.flush();
  } catch (Exception e) {
    connected = false;
  }
}

void keyReleased() {
  if (!connected) return;

  try {
    out.write('S');
    out.flush();
  } catch (Exception e) {
    connected = false;
  }
}
