#include <Servo.h>

constexpr int servoPin = 9

Servo myServo;

void setup() {
  myServo.attach(servoPin);       // Servo signal wire to pin 9
  Serial.begin(9600);
  myServo.write(90);       // Start centered
}

void loop() {
  if (Serial.available() > 0) {
    uint8_t servoTarget = Serial.read();
    Serial.print("Received: ");
    Serial.println(cmd);

    myServo.write(servoTarget)
  }
}
