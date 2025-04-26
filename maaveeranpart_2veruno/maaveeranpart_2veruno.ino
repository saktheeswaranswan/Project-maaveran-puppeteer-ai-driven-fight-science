#include <Servo.h>

Servo leftServo;
Servo rightServo;
String input = "";

void setup() {
  Serial.begin(9600);
  leftServo.attach(8);  // Left arm
  rightServo.attach(9); // Right arm
  leftServo.write(90);  // Neutral position
  rightServo.write(90);
}

void loop() {
  if (Serial.available()) {
    input = Serial.readStringUntil('\n');
    input.trim(); // Remove whitespace

    int separator = input.indexOf(',');
    if (separator > 0) {
      String leftStr = input.substring(0, separator);
      String rightStr = input.substring(separator + 1);
      int leftAngle = leftStr.toInt();
      int rightAngle = rightStr.toInt();

      leftAngle = constrain(leftAngle, 0, 180);
      rightAngle = constrain(rightAngle, 0, 180);

      leftServo.write(leftAngle);
      rightServo.write(rightAngle);

      Serial.print("Left: ");
      Serial.print(leftAngle);
      Serial.print(" | Right: ");
      Serial.println(rightAngle);
    }
  }
}
