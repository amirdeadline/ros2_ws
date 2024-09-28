#define ARDUINO_NAME "3"  // Define a unique name for this Arduino

// Define motor control pins for each device (aerator, weed control, dethatcher, stepper motor)
#define AERATOR_MOTOR1_RPWM 2
#define AERATOR_MOTOR1_LPWM 7
#define AERATOR_MOTOR2_RPWM 5
#define AERATOR_MOTOR2_LPWM 4
#define DETHATCHER_MOTOR1_PWM1 8
#define DETHATCHER_MOTOR1_DIR1 9
#define DETHATCHER_MOTOR2_PWM2 10
#define DETHATCHER_MOTOR2_DIR2 11
#define WEED_CONTROL_RPWM 12
#define WEED_CONTROL_LPWM 13

// Stepper motor for weed control
#define WC_STEPPER_ENA 46
#define WC_STEPPER_DIR 22
#define WC_STEPPER_PUL 6

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Wait for serial port to connect. Needed for native USB
  while (!Serial) {
    ; // wait for serial port to connect
  }

  // Print the Arduino's name to the serial console
  Serial.print("Arduino Name: ");
  Serial.println(ARDUINO_NAME);

  // Set up motor control pins as output
  pinMode(AERATOR_MOTOR1_RPWM, OUTPUT);
  pinMode(AERATOR_MOTOR1_LPWM, OUTPUT);
  pinMode(AERATOR_MOTOR2_RPWM, OUTPUT);
  pinMode(AERATOR_MOTOR2_LPWM, OUTPUT);
  pinMode(DETHATCHER_MOTOR1_PWM1, OUTPUT);
  pinMode(DETHATCHER_MOTOR1_DIR1, OUTPUT);
  pinMode(DETHATCHER_MOTOR2_PWM2, OUTPUT);
  pinMode(DETHATCHER_MOTOR2_DIR2, OUTPUT);
  pinMode(WEED_CONTROL_RPWM, OUTPUT);
  pinMode(WEED_CONTROL_LPWM, OUTPUT);
  pinMode(WC_STEPPER_ENA, OUTPUT);
  pinMode(WC_STEPPER_DIR, OUTPUT);
  pinMode(WC_STEPPER_PUL, OUTPUT);
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read the incoming string

    if (input == "NAME") {
      Serial.println(ARDUINO_NAME);
    } 
    else if (input.startsWith("aerator:")) {
      String command = input.substring(8); // Get the specific aerator command
      if (command == "e") {
        Serial.println("Extending aerator linear actuators");
        digitalWrite(AERATOR_MOTOR1_RPWM, HIGH); // Extend Motor1
        digitalWrite(AERATOR_MOTOR2_RPWM, HIGH); // Extend Motor2
      } 
      else if (command == "r") {
        Serial.println("Retracting aerator linear actuators");
        digitalWrite(AERATOR_MOTOR1_LPWM, HIGH); // Retract Motor1
        digitalWrite(AERATOR_MOTOR2_LPWM, HIGH); // Retract Motor2
      } 
      else if (command == "s") {
        Serial.println("Stopping aerator motors");
        digitalWrite(AERATOR_MOTOR1_RPWM, LOW); // Stop Motor1
        digitalWrite(AERATOR_MOTOR2_RPWM, LOW); // Stop Motor2
        digitalWrite(AERATOR_MOTOR1_LPWM, LOW);
        digitalWrite(AERATOR_MOTOR2_LPWM, LOW);
      }
    } 
    else if (input.startsWith("wc:")) {
      String command = input.substring(3); // Get the weed control command
      if (command == "e") {
        Serial.println("Extending weed control linear actuators");
        digitalWrite(WEED_CONTROL_RPWM, HIGH);
      } 
      else if (command == "r") {
        Serial.println("Retracting weed control linear actuators");
        digitalWrite(WEED_CONTROL_LPWM, HIGH);
      } 
      else if (command == "s") {
        Serial.println("Stopping weed control motors");
        digitalWrite(WEED_CONTROL_RPWM, LOW);
        digitalWrite(WEED_CONTROL_LPWM, LOW);
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
        delay(10);
        digitalWrite(WC_STEPPER_PUL, LOW);
        delay(10);
      }

      // Disable stepper motor
      digitalWrite(WC_STEPPER_ENA, HIGH);
    } 
    else if (input == "?") {
      Serial.println("Commands available:");
      Serial.println("NAME - Get Arduino name");
      Serial.println("aerator:e - Extend aerator actuators");
      Serial.println("aerator:r - Retract aerator actuators");
      Serial.println("aerator:s - Stop aerator actuators");
      Serial.println("wc:e - Extend weed control actuators");
      Serial.println("wc:r - Retract weed control actuators");
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
