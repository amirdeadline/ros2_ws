#define ARDUINO_NAME "Mega1"  // Define a unique name for this Arduino

// Motor pins (adjust these to your wiring setup)
const int motor11PWM = 2;
const int motor11DIR = 22;
const int motor12PWM = 3;
const int motor12DIR = 23;
const int motor21PWM = 4;
const int motor21DIR = 24;
const int motor22PWM = 5;
const int motor22DIR = 25;
const int motor31PWM = 6;
const int motor31DIR = 26;
const int motor32PWM = 7;
const int motor32DIR = 27;

// Initialize motor speeds
int speed11 = 0, speed12 = 0, speed21 = 0, speed22 = 0, speed31 = 0, speed32 = 0;
bool emergencyStop = false;

void setup() {
  // Initialize serial communication
  Serial1.begin(9600);  // Use Serial1 for UART communication with the Raspberry Pi

  // Initialize motor pins as outputs
  pinMode(motor11PWM, OUTPUT);
  pinMode(motor11DIR, OUTPUT);
  pinMode(motor12PWM, OUTPUT);
  pinMode(motor12DIR, OUTPUT);
  pinMode(motor21PWM, OUTPUT);
  pinMode(motor21DIR, OUTPUT);
  pinMode(motor22PWM, OUTPUT);
  pinMode(motor22DIR, OUTPUT);
  pinMode(motor31PWM, OUTPUT);
  pinMode(motor31DIR, OUTPUT);
  pinMode(motor32PWM, OUTPUT);
  pinMode(motor32DIR, OUTPUT);
}

void loop() {
  if (Serial1.available() > 0) {  // Check if data is available on Serial1
    String input = Serial1.readStringUntil('\n');  // Read the incoming string

    if (input.startsWith("STOP")) {
      emergencyStop = true;
      stopMotors();
      Serial1.println("Emergency Stop Activated");
    } else if (input.startsWith("RESUME")) {
      emergencyStop = false;
      Serial1.println("Emergency Stop Deactivated");
    } else if (!emergencyStop) {
      parseMotorCommand(input);
      runMotors();
    }
  }

  delay(100);  // Reduce delay for faster response
}

void parseMotorCommand(String command) {
  // Example command format: "M11:30,M12:0,M21:0,M22:0,M31:0,M32:0"
  speed11 = extractSpeed(command, "M11");
  speed12 = extractSpeed(command, "M12");
  speed21 = extractSpeed(command, "M21");
  speed22 = extractSpeed(command, "M22");
  speed31 = extractSpeed(command, "M31");
  speed32 = extractSpeed(command, "M32");

  Serial1.println("Parsed speeds: " + String(speed11) + ", " + String(speed12) + ", " + String(speed21) + ", " + String(speed22) + ", " + String(speed31) + ", " + String(speed32));
}

int extractSpeed(String command, String motorLabel) {
  int index = command.indexOf(motorLabel + ":");
  if (index != -1) {
    return command.substring(index + 4, command.indexOf(",", index)).toInt();
  }
  return 0;
}

void runMotors() {
  analogWrite(motor11PWM, abs(speed11) * 2.55);
  digitalWrite(motor11DIR, speed11 >= 0 ? HIGH : LOW);

  analogWrite(motor12PWM, abs(speed12) * 2.55);
  digitalWrite(motor12DIR, speed12 >= 0 ? HIGH : LOW);

  analogWrite(motor21PWM, abs(speed21) * 2.55);
  digitalWrite(motor21DIR, speed21 >= 0 ? HIGH : LOW);

  analogWrite(motor22PWM, abs(speed22) * 2.55);
  digitalWrite(motor22DIR, speed22 >= 0 ? HIGH : LOW);

  analogWrite(motor31PWM, abs(speed31) * 2.55);
  digitalWrite(motor31DIR, speed31 >= 0 ? HIGH : LOW);

  analogWrite(motor32PWM, abs(speed32) * 2.55);
  digitalWrite(motor32DIR, speed32 >= 0 ? HIGH : LOW);
}

void stopMotors() {
  analogWrite(motor11PWM, 0);
  analogWrite(motor12PWM, 0);
  analogWrite(motor21PWM, 0);
  analogWrite(motor22PWM, 0);
  analogWrite(motor31PWM, 0);
  analogWrite(motor32PWM, 0);
  Serial1.println("Motors stopped.");
}
