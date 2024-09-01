void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud rate
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB
  }
  Serial.println("Arduino is ready!");
}

void loop() {
  if (Serial.available() > 0) {
    char received = Serial.read();
    Serial.print("Received: ");
    Serial.println(received);
  }
}
