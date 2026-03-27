const int encLeftPin  = 12;  // your wiring: left D0 -> D12
const int encRightPin = 13;  // your wiring: right D0 -> D13

void setup() {
  Serial.begin(115200);
  pinMode(encLeftPin, INPUT_PULLUP);
  pinMode(encRightPin, INPUT_PULLUP);
}

void loop() {
  Serial.print(digitalRead(encLeftPin));
  Serial.print(',');
  Serial.println(digitalRead(encRightPin));
  delay(2);
}