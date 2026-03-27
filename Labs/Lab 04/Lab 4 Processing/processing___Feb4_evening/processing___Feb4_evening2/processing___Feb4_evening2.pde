import processing.serial.*;

Serial port;
PrintWriter out;

String lastLine = "";
int lastFlushMs = 0;
boolean ready = false;

void setup() {
  size(900, 200);

  // Print ports and pick one (change index if needed)
  String[] ports = Serial.list();
  printArray(ports);

  String portName = ports[0];   // <-- change this index if needed
  println("Using: " + portName);

  // Create the log file FIRST (so out is never null inside serialEvent)
  String fn = nf(year(),4)+nf(month(),2)+nf(day(),2)+"_"
            + nf(hour(),2)+nf(minute(),2)+nf(second(),2);
  out = createWriter("telemetry_"+fn+".csv");
  out.println("t_ms,photo,leftTicks,rightTicks,dLeft,dRight,mode,cmd,leftPWM,rightPWM");

  // Open serial
  port = new Serial(this, portName, 115200);
  port.bufferUntil('\n');

  // Arduino often resets when serial connects; wait, then clear junk
  delay(1200);
  port.clear();

  ready = true;
}

void serialEvent(Serial p) {
  if (!ready) return;

  try {
    String line = p.readStringUntil('\n');
    if (line == null) return;
    line = trim(line);
    if (line.length() == 0) return;

    // Expect 10 CSV fields => 9 commas. If not, ignore the line.
    if (countChar(line, ',') < 9) return;

    lastLine = line;
    out.println(line);

  } catch (Exception e) {
    // IMPORTANT: don’t let exceptions disable serialEvent()
    println("serialEvent exception: " + e);
  }
}

int countChar(String s, char c) {
  int n = 0;
  for (int i = 0; i < s.length(); i++) if (s.charAt(i) == c) n++;
  return n;
}

void draw() {
  background(255);
  fill(0);
  text("Logging... press q to quit (closes CSV).", 10, 30);
  text("Last line:", 10, 60);
  text(lastLine, 10, 90);

  // Flush occasionally instead of every line
  if (millis() - lastFlushMs > 500) {
    out.flush();
    lastFlushMs = millis();
  }
}

void keyPressed() {
  if (key == 'q') {
    out.flush();
    out.close();
    exit();
  }

  // send key to Arduino
  port.write(key);
  port.write('\n');
}
