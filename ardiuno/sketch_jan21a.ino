#include <Servo.h>

// Define servo objects for thrusters
Servo thrusters[5];

// Define PWM pins for 5 thrusters
const int thrusterPins[5] = {5, 6, 9, 10, 11};

// Default PWM values for thrusters
int pwmValues[5] = {1500, 1500, 1500, 1500, 1500};

void setup() {
  Serial.begin(115200); // Initialize serial communication

  // Attach each thruster to its respective pin
  for (int i = 0; i < 5; i++) {
    thrusters[i].attach(thrusterPins[i]);
    thrusters[i].writeMicroseconds(pwmValues[i]); // Set to default values
  }

  Serial.println("Thruster control initialized.");
}

void loop() {
  // Check if data is available on the serial port
  
  if (Serial.available()) {
    // Read the incoming command
    String command = Serial.readStringUntil('\n');
    command.trim();

    // Parse the PWM values from the command
    int index = 0;
    String value = "";

    for (int i = 0; i < command.length(); i++) {
      if (command[i] == ',' || i == command.length() - 1) {
        if (i == command.length() - 1) value += command[i]; // Add last character
        pwmValues[index++] = value.toInt();
        value = "";

        // Ensure we don't exceed 5 values
        if (index >= 5) break;
      } else {
        value += command[i];
      }
    }

    // Update thruster PWM values and display them
    Serial.print("Received PWM values: ");
    for (int i = 0; i < 5; i++) {
      thrusters[i].writeMicroseconds(int(pwmValues[i]));
      Serial.print(pwmValues[i]);
      if (i < 4) Serial.print(", ");
    }
    Serial.println();

    // Send acknowledgment back to Xavier
    //Serial.println("PWM values updated successfully.");
  }
}
