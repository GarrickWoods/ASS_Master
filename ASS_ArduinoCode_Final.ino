#include <Servo.h>

Servo servoPitch;
Servo servoYaw;
Servo servoTrigger;

int pitch = 85;
int yaw = 80;
int trigger = 0;

int minYaw = 20;
int maxYaw = 125;

int minPitch = 50;
int maxPitch = 95;

void setup() {
  pinMode(7, OUTPUT);  // Set pin 7 as an output
  servoPitch.attach(3);
  servoYaw.attach(5);
  servoTrigger.attach(6);

  servoPitch.write(85);  // Center pitch
  servoYaw.write(80);    // Center yaw
  servoTrigger.write(0); // Initialize at 0
  digitalWrite(7, HIGH);  // Set pin 7 high

  Serial.begin(9600);
}

void loop() {
  // Read the incoming data
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');

    String pitchStr = data.substring(0, commaIndex);
    String yawStr = data.substring(commaIndex + 1);

    int newPitch = pitchStr.toInt();
    int newYaw = yawStr.toInt();

    // Update the pitch and yaw values
    pitch = newPitch;
    yaw = newYaw;

    servoPitch.write(pitch);
    servoYaw.write(yaw);
  }

  // Check if yaw is not 80
  if (yaw != 80) {
    servoTrigger.write(90);
  } else {
    servoTrigger.write(0);
  }
}

