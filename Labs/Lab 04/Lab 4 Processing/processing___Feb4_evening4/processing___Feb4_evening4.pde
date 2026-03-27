import processing.serial.*;

Serial port;
PrintWriter out;

String lastLine = "";
char lastSent = '_';
int lineCount = 0;
void setup() {
  size(900, 200);

  // 1) Print ports clearly
  String[] ports = Serial.list();
  printArray(ports);

  // 2) Pick the Arduino port automatically (works on Mac/Win/Linux most of the time)
  int idx = findArduinoPortIndex(ports);
  if (idx < 0) {
    println("ERROR: Couldn't auto-find Arduino port.");
    println("Set idx manually by changing the code.");
    exit();
  }

  String portName = ports[idx];
  println("Using port: " + portName);

  // 3) Open serial
  port = new Serial(this, portName, 115200);
  port.clear();
  port.bufferUntil('\n');

  // 4) Arduino often resets when serial connects; give it time
  delay(1200);

  // Force manual mode + stop (so it's ready for WASD)
  sendCmd('m');
  sendCmd('x');

  // 5) Log file
  String fn = nf(year(),4)+nf(month(),2)+nf(day(),2)+"_"
            + nf(hour(),2)+nf(minute(),2)+nf(second(),2);
  out = createWriter("telemetry_"+fn+".csv");
  out.println("t_ms,photo,leftTicks,rightTicks,dLeft,dRight,mode,cmd,leftPWM,rightPWM");
}

int findArduinoPortIndex(String[] ports) {
  for (int i = 0; i < ports.length; i++) {
    String p = ports[i].toLowerCase();
    // Mac: cu.usbmodem / tty.usbmodem, Linux: ttyACM, Windows: COM
    if (p.contains("usbmodem") || p.contains("usbserial") || p.contains("ttyacm") || p.startsWith("com")) {
      return i;
    }
  }
  // fallback: if there's only one port, use it
  if (ports.length == 1) return 0;
  return -1;
}

void sendCmd(char c) {
  if (port == null) return;
  c = Character.toLowerCase(c);

  // Map space to stop
  if (c == ' ') c = 'x';   // space means STOP, not backward

  port.write(c);
  port.write('\n');   // Arduino ignores \n/\r anyway
  lastSent = c;
  println("Sent: " + c);
}

void serialEvent(Serial p) {
  try {
    String line = p.readStringUntil('\n');
    if (line == null) return;     // <-- must check BEFORE trim()
    line = trim(line);
    if (line.length() == 0) return;
    lineCount++;
    if (lineCount % 10 == 0) println(line);      // <-- ADD THIS (prints to Processing console)
    out.println(line);
    // don't flush every line (see below)
  } catch (Exception e) {
    println("serialEvent exception: " + e);
  }
}

void draw() {
  background(255);
  fill(0);
  text("CLICK THIS WINDOW FIRST, then use: m=manual, u=auto, w/a/s/d drive, space=stop, q=quit", 10, 30);
  text("Last sent: " + lastSent, 10, 60);
  text("Last telemetry: " + lastLine, 10, 90);
}

void keyPressed() {
  if (key == CODED) return;

  if (key == 'q') {
    sendCmd('x');
    out.flush();
    out.close();
    exit();
  }

  sendCmd(key);
}

void keyReleased() {
  // Stop when you release any movement key
  char c = Character.toLowerCase(key);
  if (c == 'w' || c == 'a' || c == 's' || c == 'd') {
    sendCmd('x');  // stop
  }
}
