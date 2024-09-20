import serial
import time

# Open the serial port connected to the Arduino
arduino = serial.Serial('/dev/serial0', 9600, timeout=1)  # Adjust '/dev/ttyUSB0' as needed
time.sleep(2)  # Wait for the serial connection to initialize

def send_pwm_value(pwm_value):
    # Send the PWM value to the Arduino
    command = f"{pwm_value}\n"  # Append a newline character
    arduino.write(command.encode())  # Encode and send the command
    print(f"Sent PWM value: {pwm_value}")

# Send the disarm signal (e.g., 1000 microseconds)
send_pwm_value(1000)

# Allow time for the signal to be processed
time.sleep(5)

# Test with other values if needed (e.g., 1500 for mid-throttle, 2000 for full throttle)
send_pwm_value(1500)
time.sleep(5)
send_pwm_value(2000)

arduino.close()
