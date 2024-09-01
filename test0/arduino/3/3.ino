#define ARDUINO_NAME "3"  // Define a unique name for this Arduino

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

  // Initialize the onboard LED pin as an output
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read the incoming string

    // If the query is "NAME", respond with the Arduino's name
    if (input == "NAME") {
      Serial.println(ARDUINO_NAME);
    }
  }

  // Blink the onboard LED three times quickly
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED on (HIGH is the voltage level)
    delay(250);                       // Wait for 0.25 seconds
    digitalWrite(LED_BUILTIN, LOW);   // Turn the LED off by making the voltage LOW
    delay(250);                       // Wait for 0.25 seconds
  }

  // Wait for 5 seconds before repeating the pattern
  delay(5000);

  // Add a small delay to avoid overwhelming the serial port
  delay(10);
}
