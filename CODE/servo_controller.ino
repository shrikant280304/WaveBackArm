#include <Servo.h>

const int servoPin = 9;  // Pin connected to servo motor
Servo servoMotor;         // Create a servo object

void setup() {
  servoMotor.attach(servoPin);  // Attach the servo to the specified pin
  Serial.begin(9600);           // Initialize serial communication
}

void loop() {
  while (Serial.available() > 0) {  // Check if data is available to read
    int servoPos = Serial.parseInt();  // Read the servo position from serial
    if (servoPos >= 0 && servoPos <= 180) {  // Validate servo position
      servoMotor.write(servoPos);  // Move servo to position
    }
  }
}
