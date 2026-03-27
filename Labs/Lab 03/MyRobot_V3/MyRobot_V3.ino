// ===== USB (Serial) version of your WiFi sketch =====
// Control chars come in over USB serial: U D L R S
// Telemetry now includes photocell info:
// t_ms,encL,encR,ticksL,ticksR,lastCmdAge_ms,photo,onTape

const int PHOTO_PIN = A0;
int photoThreshold = 500; // tune this

// ===== L298N pins (match your wiring) =====
const int ENA = 9;
const int ENB = 10;
const int IN1 = 7;
const int IN2 = 8;
const int IN3 = 6;
const int IN4 = 5;

int speedVal = 150;

unsigned long lastCmdTime = 0;
const unsigned long timeoutMs = 300; // dead-man stop

// ===== Encoder pins (match your build: D12/D13) =====
const int ENC_L = 12;   // left LM393 D0 -> D12
const int ENC_R = 13;   // right LM393 D0 -> D13

// Tick counters + last states (simple polling edge-detect like a "button")
volatile long ticksL = 0;
volatile long ticksR = 0;

int lastEncL = HIGH;
int lastEncR = HIGH;

// ===== Telemetry timing =====
unsigned long lastPrintMs = 0;
const unsigned long printEveryMs = 20; // ~50 Hz logging (adjust)

// Telemetry now includes photo + onTape
void printTelemetry(int encL, int encR, int photo, bool onTape) {
  // CSV-like: time_ms,encL,encR,ticksL,ticksR,lastCmdAge_ms,photo,onTape
  unsigned long now = millis();
  Serial.print(now); Serial.print(',');
  Serial.print(encL); Serial.print(',');
  Serial.print(encR); Serial.print(',');
  Serial.print(ticksL); Serial.print(',');
  Serial.print(ticksR); Serial.print(',');
  Serial.print(now - lastCmdTime); Serial.print(',');
  Serial.print(photo); Serial.print(',');
  Serial.println(onTape ? 1 : 0);
}

void setup() {
  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();

  // Encoders: use pullups, same idea as the lab example
  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);

  // Photocell
  pinMode(PHOTO_PIN, INPUT);

  Serial.begin(115200);

  // Print a header once so logs are self-describing
  Serial.println("t_ms,encL,encR,ticksL,ticksR,lastCmdAge_ms,photo,onTape");
  Serial.println("USB Serial control ready (send U/D/L/R/S).");

  lastCmdTime = millis(); // initialize
}

void loop() {
  // --- Always read encoders + update tick counts ---
  int encL = digitalRead(ENC_L);
  int encR = digitalRead(ENC_R);

  // --- Read photocell ---
  int photo = analogRead(PHOTO_PIN);
  bool onTape = (photo > photoThreshold);  // flip to (photo < photoThreshold) if needed

  // Count rising edges (LOW -> HIGH) to avoid double counting
  if (lastEncL == LOW && encL == HIGH) ticksL++;
  if (lastEncR == LOW && encR == HIGH) ticksR++;
  lastEncL = encL;
  lastEncR = encR;

  // Print telemetry at a steady rate (nice delimited output)
  unsigned long now = millis();
  if (now - lastPrintMs >= printEveryMs) {
    lastPrintMs = now;
    printTelemetry(encL, encR, photo, onTape);
  }

  // --- USB Serial handling (replaces WiFi client handling) ---
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd != '\n' && cmd != '\r') {
      handleCommand(cmd);
      lastCmdTime = millis();
    }
  }

  // Dead-man stop (same idea as before)
  if (millis() - lastCmdTime > timeoutMs) {
    stopMotors();
  }
}

void handleCommand(char cmd) {
  Serial.print("CMD,"); Serial.println(cmd);

  switch (cmd) {
    case 'U': forward(); break;
    case 'D': backward(); break;
    case 'L': left(); break;
    case 'R': right(); break;
    case 'S': stopMotors(); break;
  }
}

// Motor commands (unchanged)
void forward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void backward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void left() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void right() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}