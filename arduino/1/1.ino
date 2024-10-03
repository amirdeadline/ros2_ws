#define ARDUINO_NAME "Arduino1"  // Define a unique name for this Arduino

// New motor pins for Cytron MDD20A Driver (adjusted for Arduino Uno)
const int motor11PWM = 3;    // Driver 1 PWM1
const int motor11DIR = 4;    // Driver 1 DIR1
const int motor12PWM = 5;    // Driver 1 PWM2
const int motor12DIR = 6;    // Driver 1 DIR2
const int motor21PWM = 9;    // Driver 2 PWM1
const int motor21DIR = 10;   // Driver 2 DIR1
const int motor22PWM = 11;   // Driver 2 PWM2
const int motor22DIR = 12;   // Driver 2 DIR2

// Initialize motor speeds
int speed11 = 0, speed12 = 0, speed21 = 0, speed22 = 0;
bool emergencyStop = false;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);  // Use Serial for communication with Raspberry Pi over USB
  while (!Serial) {
    ;  // Wait for the serial port to connect
  }
  Serial.println("Arduino " ARDUINO_NAME " is ready");
  
  // Initialize motor pins as outputs
  pinMode(motor11PWM, OUTPUT);
  pinMode(motor11DIR, OUTPUT);
  pinMode(motor12PWM, OUTPUT);
  pinMode(motor12DIR, OUTPUT);
  pinMode(motor21PWM, OUTPUT);
  pinMode(motor21DIR, OUTPUT);
  pinMode(motor22PWM, OUTPUT);
  pinMode(motor22DIR, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {  // Check if data is available on Serial (USB)
    String input = Serial.readStringUntil('\n');  // Read the incoming string

    Serial.println("Received command: " + input);

    if (input.startsWith("STOP")) {
      emergencyStop = true;
      stopMotors();
      Serial.println("Emergency Stop Activated");
    } else if (input.startsWith("RESUME")) {
      emergencyStop = false;
      Serial.println("Emergency Stop Deactivated");
    } else if (!emergencyStop) {
      parseMotorCommand(input);
      runMotors();
    }
  }

  delay(100);  // Reduce delay for faster response
}

void parseMotorCommand(String command) {
  // Example command format: "M11:30,M12:0,M21:0,M22:0"
  speed11 = extractSpeed(command, "M11");
  speed12 = extractSpeed(command, "M12");
  speed21 = extractSpeed(command, "M21");
  speed22 = extractSpeed(command, "M22");

  Serial.println("Parsed speeds: M11=" + String(speed11) + ", M12=" + String(speed12) + 
                 ", M21=" + String(speed21) + ", M22=" + String(speed22));
}

int extractSpeed(String command, String motorLabel) {
  int index = command.indexOf(motorLabel + ":");
  if (index != -1) {
    return command.substring(index + 4, command.indexOf(",", index)).toInt();
  }
  return 0;
}

void runMotors() {
  Serial.println("Running motors...");

  analogWrite(motor11PWM, abs(speed11) * 2.55);  // Set PWM speed for Motor11
  digitalWrite(motor11DIR, speed11 >= 0 ? HIGH : LOW);  // Set direction for Motor11

  analogWrite(motor12PWM, abs(speed12) * 2.55);  // Set PWM speed for Motor12
  digitalWrite(motor12DIR, speed12 >= 0 ? HIGH : LOW);  // Set direction for Motor12

  analogWrite(motor21PWM, abs(speed21) * 2.55);  // Set PWM speed for Motor21
  digitalWrite(motor21DIR, speed21 >= 0 ? HIGH : LOW);  // Set direction for Motor21

  analogWrite(motor22PWM, abs(speed22) * 2.55);  // Set PWM speed for Motor22
  digitalWrite(motor22DIR, speed22 >= 0 ? HIGH : LOW);  // Set direction for Motor22
}

void stopMotors() {
  Serial.println("Stopping all motors...");

  analogWrite(motor11PWM, 0);
  analogWrite(motor12PWM, 0);
  analogWrite(motor21PWM, 0);
  analogWrite(motor22PWM, 0);

  Serial.println("Motors stopped.");
}
