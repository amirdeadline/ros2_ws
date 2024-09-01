#define ARDUINO_NAME "2"  // Define a unique name for this Arduino

// Define the pins connected to the motor driver for linear actuators
const int dir1_pin = 2;   // Direction control for LA1
const int pwm1_pin = 3;   // PWM control for LA1
const int dir2_pin = 4;   // Direction control for LA2
const int pwm2_pin = 5;   // PWM control for LA2

// Timing variables
unsigned long previousMillis = 0;
const long interval = 5000;  // Interval for LED blinking (5 seconds)

bool actionInProgress = false; // To track if an action is in progress
int currentActuator = 0;  // Track which actuator is being controlled
char currentAction = 's'; // Track the current action: 'e' for extend, 'r' for retract, 's' for stop

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for the serial port to connect
  }

  // Print the Arduino's name to the serial console
  Serial.print("Arduino Name: ");
  Serial.println(ARDUINO_NAME);

  // Initialize the onboard LED pin as an output
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize the control pins for the linear actuators as outputs
  pinMode(dir1_pin, OUTPUT);
  pinMode(pwm1_pin, OUTPUT);
  pinMode(dir2_pin, OUTPUT);
  pinMode(pwm2_pin, OUTPUT);

  Serial.println("Setup complete. Ready to receive commands.");
}

void controlActuator(int actuator_id, char action) {
  if (action == 's') {  // Stop both actuators
    analogWrite(pwm1_pin, 0);
    analogWrite(pwm2_pin, 0);
    Serial.println("Stopping LA1 and LA2.");
    actionInProgress = false;
    return;
  }

  actionInProgress = true;  // Mark that an action is in progress
  currentActuator = actuator_id;  // Record the current actuator being controlled
  currentAction = action;  // Record the current action

  if (actuator_id == 1) {  // Control LA1
    if (action == 'e') {  // Extend
      digitalWrite(dir1_pin, HIGH);
      analogWrite(pwm1_pin, 255);
      Serial.println("Extending LA1 at full speed.");
    } else if (action == 'r') {  // Retract
      digitalWrite(dir1_pin, LOW);
      analogWrite(pwm1_pin, 255);
      Serial.println("Retracting LA1 at full speed.");
    }
  } else if (actuator_id == 2) {  // Control LA2
    if (action == 'e') {  // Extend
      digitalWrite(dir2_pin, HIGH);
      analogWrite(pwm2_pin, 255);
      Serial.println("Extending LA2 at full speed.");
    } else if (action == 'r') {  // Retract
      digitalWrite(dir2_pin, LOW);
      analogWrite(pwm2_pin, 255);
      Serial.println("Retracting LA2 at full speed.");
    }
  }
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read the incoming string

    // If the query is "NAME", respond with the Arduino's name
    if (input == "NAME") {
      Serial.println(ARDUINO_NAME);
    } else if (input == "STOP") {
      controlActuator(1, 's');  // Stop LA1
      controlActuator(2, 's');  // Stop LA2
    } else {
      // Parse input for actuator_id and action
      int delimiterIndex = input.indexOf(',');
      if (delimiterIndex > 0) {
        int actuator_id = input.substring(0, delimiterIndex).toInt();
        char action = input.charAt(delimiterIndex + 1);

        // Ensure action is valid (extend, retract, stop)
        if (action == 'e' || action == 'r' || action == 's') {
          controlActuator(actuator_id, action);
        }
      }
    }
  }

  // Handle LED blinking without blocking
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // Toggle LED
  }

  // Check if an action is in progress and handle it
  if (actionInProgress) {
    // No delay; the action continues until a STOP command is received
  }
}
