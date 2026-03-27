#include <WiFiS3.h>

char ssid[] = "SHAW-F4E1";
char pass[] = "charge8277enter";

WiFiServer server(5200);

// L298N pins
const int ENA = 9;
const int ENB = 10;
const int IN1 = 7;
const int IN2 = 8;
const int IN3 = 6;
const int IN4 = 5;

int speedVal = 150;

unsigned long lastCmdTime = 0;
const unsigned long timeoutMs = 300; // dead-man stop

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();

  Serial.begin(115200);


  WiFi.begin(ssid, pass);
  delay(2000);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected!");
  Serial.print("Arduino IP: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("Server started on port 5200");
}

void loop() {
  WiFiClient client = server.available();

  if (client) {
    Serial.println("Client connected");

    while (client.connected()) {
      if (client.available()) {
        char cmd = client.read();
        handleCommand(cmd);
        lastCmdTime = millis();
      }

      if (millis() - lastCmdTime > timeoutMs) {
        stopMotors();
      }
    }

    client.stop();
    stopMotors();
    Serial.println("Client disconnected");
  }
}

void handleCommand(char cmd) {
  Serial.println(cmd);

  switch (cmd) {
    case 'U': forward(); break;
    case 'D': backward(); break;
    case 'L': left(); break;
    case 'R': right(); break;
    case 'S': stopMotors(); break;
  }
}
// W -> U
// A -> L
// S -> D
// D -> R
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