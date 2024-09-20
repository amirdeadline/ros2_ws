#include <Servo.h>

// Create Servo objects to control ESCs
Servo esc1;  // Motor 1 (Pin 9)
Servo esc2;  // Motor 2 (Pin 10)
Servo esc3;  // Motor 3 (Pin 11)

// ESC control pins
const int escPin1 = 9;  // Connected to ESC1 (motor 1)
const int escPin2 = 10; // Connected to ESC2 (motor 2)
const int escPin3 = 11; // Connected to ESC3 (motor 3)

String inputString = ""; // A string to hold incoming commands

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);
  inputString.reserve(10);  // Reserve some memory for the input string

  // Attach the ESCs to the corresponding pins
  esc1.attach(escPin1);
  esc2.attach(escPin2);
  esc3.attach(escPin3);

  // **Safely calibrate the ESCs** for motors 1, 2, and 3 (disconnect motors before starting)
  Serial.println("Calibrating ESCs for motors 1, 2, and 3 (disconnect motors)...");

  // Step 1: Send maximum throttle to all ESCs (ESC will register max throttle)
  esc1.writeMicroseconds(2000);
  esc2.writeMicroseconds(2000);
  esc3.writeMicroseconds(2000);
  delay(3000);  // Wait for 3 seconds

  // Step 2: Send minimum throttle to all ESCs (ESC will register min throttle)
  esc1.writeMicroseconds(800);
  esc2.writeMicroseconds(800);
  esc3.writeMicroseconds(800);
  delay(3000);  // Wait for 3 seconds

  Serial.println("ESC calibration completed for motors 1, 2, and 3.");

  // Wait for 5 seconds to allow ESCs to arm
  delay(5000);

  // Notify user that the ESCs are armed and ready
  Serial.println("ESCs are armed. Reconnect motors and enter commands in the format <motor_number:signal>");
}

void loop() {
  // Continue listening for commands and handling motor signals
  if (Serial.available()) {
    char inChar = (char)Serial.read();  // Read a character from the serial input

    if (inChar == '\n') {
      processCommand(inputString);
      inputString = "";  // Clear the string after processing
    } else {
      inputString += inChar;  // Append the character to the input string
    }
  }
}

// Function to process the command string
void processCommand(String input) {
  // Check if input contains a colon
  int separatorIndex = input.indexOf(':');
  if (separatorIndex != -1) {
    // Extract motor number and signal from the command
    String motorID = input.substring(0, separatorIndex);
    int signal = input.substring(separatorIndex + 1).toInt();

    // Handle the motor command
    handleMotorCommand(motorID, signal);
  } else {
    // If the command is not properly formatted, send an error message
    Serial.println("Invalid command format. Use <motor_number:signal>");
  }
}

// Function to handle motor commands
void handleMotorCommand(String motorID, int signal) {
  // Handle motor 1 (Pin 9)
  if (motorID == "1") {
    // Activate motor 1 with the given signal
    esc1.writeMicroseconds(signal);
    Serial.println("Motor 1 signal set to: " + String(signal));
  }
  // Handle motors 2 and 3 (both controlled together)
  else if (motorID == "23") {
    // Activate motors 2 and 3 with the same signal
    esc2.writeMicroseconds(signal);
    esc3.writeMicroseconds(signal);
    Serial.println("Motors 2 and 3 signal set to: " + String(signal));
  }
  else {
    Serial.println("Invalid motor number. Use 1 or 23.");
  }
}
