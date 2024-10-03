#include <Servo.h>
#define ARDUINO_NAME "Arduino2"  // Define a unique name for this Arduino

// Create Servo objects to control ESCs
Servo esc1;  // Motor 1 Dethacher (Pin 9)
Servo esc2;  // Motor 2 Mower Motor1 (Pin 10)
Servo esc3;  // Motor 3 Mower Motor2 (Pin 11)

// ESC control pins
const int escPin1 = 9;  // Connected to ESC1 (Motor 1 Dethacher)
const int escPin2 = 10; // Connected to ESC2 (Motor 2 Mower Motor1)
const int escPin3 = 11; // Connected to ESC3 (Motor 3 Mower Motor2)

// Motor Driver 3 pins (for linear actuators)
const int dirPin1 = 2;  // Direction control for Motor 31
const int pwmPin1 = 3;  // PWM control for Motor 31
const int dirPin2 = 4;  // Direction control for Motor 32
const int pwmPin2 = 5;  // PWM control for Motor 32

// Command input string
String inputString = ""; // A string to hold incoming commands

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);  // Use Serial for communication with Raspberry Pi over USB
  while (!Serial) {
    ;  // Wait for the serial port to connect
  }
  Serial.println("Arduino " ARDUINO_NAME " is ready");
  inputString.reserve(10);  // Reserve some memory for the input string

  // Attach the ESCs to the corresponding pins
  esc1.attach(escPin1);
  esc2.attach(escPin2);
  esc3.attach(escPin3);

  // Set all ESCs to the minimum throttle (800 microseconds) at startup
  esc1.writeMicroseconds(800);
  esc2.writeMicroseconds(800);
  esc3.writeMicroseconds(800);

  // Initialize motor driver pins for linear actuators
  pinMode(dirPin1, OUTPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(pwmPin2, OUTPUT);

  Serial.println("ESCs initialized at 800 signal. Enter commands like 1:950, 31:e, 32:r, or help with '?'");
}

void loop() {
  // Continuously listen for commands
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
    // Extract motor number and the command (signal or control)
    String motorID = input.substring(0, separatorIndex);
    String command = input.substring(separatorIndex + 1);

    // Handle the motor command or calibration
    handleMotorCommand(motorID, command);
  } else {
    // Handle special commands like help or name
    if (input == "?") {
      Serial.println("Available commands:");
      Serial.println("1:<signal> - Control Motor 1 (Dethacher)");
      Serial.println("23:<signal> - Control Motors 2 and 3 (Mower Motors)");
      Serial.println("31:e/r/s - Extend/Retract/Stop Motor 31");
      Serial.println("32:e/r/s - Extend/Retract/Stop Motor 32");
      Serial.println("name - Get Arduino name");
    } else if (input == "name") {
      Serial.println("Arduino Name: " + String(ARDUINO_NAME));
    } else {
      // If the command is not properly formatted, send an error message
      Serial.println("Invalid command format. Use ? for help.");
    }
  }
}

// Function to handle motor commands or calibration
void handleMotorCommand(String motorID, String command) {
  // Handle motor 1 (Pin 9)
  if (motorID == "1") {
    if (command == "cal") {
      // Calibrate motor 1
      calibrateESC(esc1, "Motor 1");
    } else {
      // Convert command to signal and apply it
      int signal = command.toInt();
      esc1.writeMicroseconds(signal);
      Serial.println("Motor 1 signal set to: " + String(signal));
    }
  }
  // Handle motors 2 and 3 (both controlled together)
  else if (motorID == "23") {
    if (command == "cal") {
      // Calibrate motors 2 and 3 together
      calibrateESC(esc2, "Motor 2");
      calibrateESC(esc3, "Motor 3");
    } else {
      // Convert command to signal and apply it to both motors
      int signal = command.toInt();
      esc2.writeMicroseconds(signal);
      esc3.writeMicroseconds(signal);
      Serial.println("Motors 2 and 3 signal set to: " + String(signal));
    }
  }
  // Handle motor 31 (linear actuator)
  else if (motorID == "31") {
    handleLinearActuator(command, dirPin1, pwmPin1, "Motor 31");
  }
  // Handle motor 32 (linear actuator)
  else if (motorID == "32") {
    handleLinearActuator(command, dirPin2, pwmPin2, "Motor 32");
  } else {
    Serial.println("Invalid motor number. Use 1, 23, 31, or 32.");
  }
}

// Function to handle linear actuators (extend, retract, stop)
void handleLinearActuator(String command, int dirPin, int pwmPin, String motorName) {
  if (command == "e") {
    digitalWrite(dirPin, HIGH);  // Set direction for extension
    analogWrite(pwmPin, 255);    // Full PWM signal for extension
    Serial.println(motorName + " extending.");
  } else if (command == "r") {
    digitalWrite(dirPin, LOW);   // Set direction for retraction
    analogWrite(pwmPin, 255);    // Full PWM signal for retraction
    Serial.println(motorName + " retracting.");
  } else if (command == "s") {
    analogWrite(pwmPin, 0);      // Stop motor
    Serial.println(motorName + " stopped.");
  } else {
    Serial.println("Invalid command for " + motorName + ". Use e (extend), r (retract), or s (stop).");
  }
}

// Function to calibrate a specific ESC
void calibrateESC(Servo &esc, String motorName) {
  Serial.println("Calibrating " + motorName + "...");

  // Step 1: Send maximum throttle
  esc.writeMicroseconds(2000);
  delay(3000);  // Wait for 3 seconds

  // Step 2: Send minimum throttle
  esc.writeMicroseconds(800);
  delay(3000);  // Wait for 3 seconds

  Serial.println(motorName + " calibration complete.");
}
