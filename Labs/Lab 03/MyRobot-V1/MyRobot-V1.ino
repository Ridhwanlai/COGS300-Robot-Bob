// MyRobot-V1.ino
// This file must be named the same as your sketch folder
int enA = 9;   // Enable pin for Motor A — must be a PWM-capable pin
int in1 = 8;   // Direction control pin 1 for Motor A
int in2 = 7;   // Direction control pin 2 for Motor A

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
}

void loop() {
    // put your main code here, to run repeatedly:

    // Drive for 1 second
    logInfo(Serial, "Driving");
    drive(in1, in2, enA);
    delay(1000);

    // Stop for 1 second
    logInfo(Serial, "Stopping");
    stop(in1, in2, enA);
    delay(1000);

    // TODO: add your own driving functions here
}
