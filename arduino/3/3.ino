#define ARDUINO_NAME "Arduino3"  // Define a unique name for this Arduino

// Define motor control pins for aerator, weed control, dethatcher, and stepper motor
#define AERATOR_MOTOR1_RPWM 2
#define AERATOR_MOTOR1_LPWM 7
#define AERATOR_MOTOR1_REN 5
#define AERATOR_MOTOR1_LEN 4
#define DETHATCHER_MOTOR1_PWM1 8
#define DETHATCHER_MOTOR1_DIR1 9
#define DETHATCHER_MOTOR2_PWM2 10
#define DETHATCHER_MOTOR2_DIR2 11
#define WEED_CONTROL_RPWM 12
#define WEED_CONTROL_LPWM 13
#define WEED_CONTROL_REN 44
#define WEED_CONTROL_LEN 45

// Stepper motor for weed control
#define WC_STEPPER_ENA 46
#define WC_STEPPER_DIR 22
#define WC_STEPPER_PUL 6

void setup() {
  // Initialize serial communication
  Serial.begin(9600);  // Use Serial for communication with Raspberry Pi over USB
  while (!Serial) {
    ;  // Wait for the serial port to connect
  }
  Serial.println("Arduino " ARDUINO_NAME " is ready");

  // Set up motor control pins as output for AERATOR
  pinMode(AERATOR_MOTOR1_RPWM, OUTPUT);
  pinMode(AERATOR_MOTOR1_LPWM, OUTPUT);
  pinMode(AERATOR_MOTOR1_REN, OUTPUT);
  pinMode(AERATOR_MOTOR1_LEN, OUTPUT);
  
  // Set up motor control pins as output for DETHATCHER
  pinMode(DETHATCHER_MOTOR1_PWM1, OUTPUT);
  pinMode(DETHATCHER_MOTOR1_DIR1, OUTPUT);
  pinMode(DETHATCHER_MOTOR2_PWM2, OUTPUT);
  pinMode(DETHATCHER_MOTOR2_DIR2, OUTPUT);
  
  // Set up motor control pins as output for WEED_CONTROL
  pinMode(WEED_CONTROL_RPWM, OUTPUT);
  pinMode(WEED_CONTROL_LPWM, OUTPUT);
  pinMode(WEED_CONTROL_REN, OUTPUT);
  pinMode(WEED_CONTROL_LEN, OUTPUT);
  
  // Set up stepper motor pins
  pinMode(WC_STEPPER_ENA, OUTPUT);
  pinMode(WC_STEPPER_DIR, OUTPUT);
  pinMode(WC_STEPPER_PUL, OUTPUT);

  // Initially disable the motors by setting REN and LEN pins LOW
  digitalWrite(AERATOR_MOTOR1_REN, LOW);
  digitalWrite(AERATOR_MOTOR1_LEN, LOW);
  digitalWrite(WEED_CONTROL_REN, LOW);
  digitalWrite(WEED_CONTROL_LEN, LOW);
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read the incoming string

    if (input == "name") {  // If the command is "name", return ARDUINO_NAME
      Serial.println(ARDUINO_NAME);
    } 
    else if (input.startsWith("aerator:")) {
      String command = input.substring(8); // Get the specific aerator command
      if (command == "e") { // Now "e" will retract
        Serial.println("Retracting aerator linear actuators");
        digitalWrite(AERATOR_MOTOR1_REN, HIGH); // Enable right motor
        digitalWrite(AERATOR_MOTOR1_LEN, HIGH); // Enable left motor
        digitalWrite(AERATOR_MOTOR1_RPWM, LOW); // Retract Motor1
        digitalWrite(AERATOR_MOTOR1_LPWM, HIGH);
      } 
      else if (command == "r") { // Now "r" will extend
        Serial.println("Extending aerator linear actuators");
        digitalWrite(AERATOR_MOTOR1_REN, HIGH); // Enable right motor
        digitalWrite(AERATOR_MOTOR1_LEN, HIGH); // Enable left motor
        digitalWrite(AERATOR_MOTOR1_RPWM, HIGH); // Extend Motor1
        digitalWrite(AERATOR_MOTOR1_LPWM, LOW);
      } 
      else if (command == "s") {
        Serial.println("Stopping aerator motors");
        digitalWrite(AERATOR_MOTOR1_RPWM, LOW); // Stop Motor1
        digitalWrite(AERATOR_MOTOR1_LPWM, LOW);
        digitalWrite(AERATOR_MOTOR1_REN, LOW);  // Disable motor
        digitalWrite(AERATOR_MOTOR1_LEN, LOW);
      }
    } 
    else if (input.startsWith("wc:")) {
      String command = input.substring(3); // Get the weed control command
      if (command == "e") { // Now "e" will retract
        Serial.println("Retracting weed control linear actuators");
        digitalWrite(WEED_CONTROL_REN, HIGH); // Enable right motor
        digitalWrite(WEED_CONTROL_LEN, HIGH); // Enable left motor
        digitalWrite(WEED_CONTROL_RPWM, LOW);
        digitalWrite(WEED_CONTROL_LPWM, HIGH);
      } 
      else if (command == "r") { // Now "r" will extend
        Serial.println("Extending weed control linear actuators");
        digitalWrite(WEED_CONTROL_REN, HIGH); // Enable right motor
        digitalWrite(WEED_CONTROL_LEN, HIGH); // Enable left motor
        digitalWrite(WEED_CONTROL_RPWM, HIGH);
        digitalWrite(WEED_CONTROL_LPWM, LOW);
      } 
      else if (command == "s") {
        Serial.println("Stopping weed control motors");
        digitalWrite(WEED_CONTROL_RPWM, LOW);
        digitalWrite(WEED_CONTROL_LPWM, LOW);
        digitalWrite(WEED_CONTROL_REN, LOW);  // Disable motor
        digitalWrite(WEED_CONTROL_LEN, LOW);
      }
    } 
    else if (input.startsWith("dethatcher:")) {
      String command = input.substring(11); // Get the dethatcher command
      if (command == "e") {
        Serial.println("Extending dethatcher linear actuators");
        digitalWrite(DETHATCHER_MOTOR1_PWM1, HIGH); // Extend Motor1
        digitalWrite(DETHATCHER_MOTOR2_PWM2, HIGH); // Extend Motor2
      } 
      else if (command == "r") {
        Serial.println("Retracting dethatcher linear actuators");
        digitalWrite(DETHATCHER_MOTOR1_DIR1, HIGH); // Retract Motor1
        digitalWrite(DETHATCHER_MOTOR2_DIR2, HIGH); // Retract Motor2
      } 
      else if (command == "s") {
        Serial.println("Stopping dethatcher motors");
        digitalWrite(DETHATCHER_MOTOR1_PWM1, LOW);
        digitalWrite(DETHATCHER_MOTOR2_PWM2, LOW);
        digitalWrite(DETHATCHER_MOTOR1_DIR1, LOW);
        digitalWrite(DETHATCHER_MOTOR2_DIR2, LOW);
      }
    } 
    else if (input.startsWith("wc_stepper:")) {
      int steps = input.substring(11).toInt();
      Serial.print("Moving stepper motor ");
      Serial.print(steps);
      Serial.println(" steps");

      // Enable stepper motor
      digitalWrite(WC_STEPPER_ENA, LOW);

      if (steps > 0) {
        digitalWrite(WC_STEPPER_DIR, HIGH); // Set direction
      } else {
        digitalWrite(WC_STEPPER_DIR, LOW);
      }

      for (int i = 0; i < abs(steps); i++) {
        digitalWrite(WC_STEPPER_PUL, HIGH);
        delay(5);
        digitalWrite(WC_STEPPER_PUL, LOW);
        delay(5);
      }

      // Disable stepper motor
      digitalWrite(WC_STEPPER_ENA, HIGH);
    } 
    else if (input == "?") {
      Serial.println("Commands available:");
      Serial.println("name - Get Arduino name");
      Serial.println("aerator:e - Retract aerator actuators");
      Serial.println("aerator:r - Extend aerator actuators");
      Serial.println("aerator:s - Stop aerator actuators");
      Serial.println("wc:e - Retract weed control actuators");
      Serial.println("wc:r - Extend weed control actuators");
      Serial.println("wc:s - Stop weed control actuators");
      Serial.println("dethatcher:e - Extend dethatcher actuators");
      Serial.println("dethatcher:r - Retract dethatcher actuators");
      Serial.println("dethatcher:s - Stop dethatcher actuators");
      Serial.println("wc_stepper:<steps> - Move stepper motor");
    }
  }

  // Add a small delay to avoid overwhelming the serial port
  delay(10);
}
