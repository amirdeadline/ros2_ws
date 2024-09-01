import serial
import time
import argparse
from datetime import datetime

# Function to log messages with a timestamp
def log_message(message):
    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] {message}")

# Function to send motor command to Arduino
def send_motor_command(ser, m11, m12, m21, m22, duration):
    command = f"M11:{m11} M12:{m12} M21:{m21} M22:{m22}\n"
    log_message(f"Sending command: {command.strip()}")
    ser.write(command.encode())
    time.sleep(duration)
    ser.write("M11:0 M12:0 M21:0 M22:0\n".encode())  # Stop all motors after the duration
    log_message("Motors stopped.")

def main():
    # Setup argument parser
    parser = argparse.ArgumentParser(description="Control motors with specified speed and time.")
    parser.add_argument('-m11', type=int, default=0, help='Speed for Motor 11 (rear right motor)')
    parser.add_argument('-m12', type=int, default=0, help='Speed for Motor 12 (rear left motor)')
    parser.add_argument('-m21', type=int, default=0, help='Speed for Motor 21 (front right motor)')
    parser.add_argument('-m22', type=int, default=0, help='Speed for Motor 22 (front left motor)')
    parser.add_argument('-t', '--time', type=int, default=2, help='Time to run the motors (in seconds)')

    # Parse the arguments
    args = parser.parse_args()

    # Establish serial connection to Arduino
    try:
        ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Adjust port as necessary
        time.sleep(2)  # Give some time for Arduino to reset

        # Send motor command based on the parsed arguments
        send_motor_command(ser, args.m11, args.m12, args.m21, args.m22, args.time)

    except Exception as e:
        log_message(f"An error occurred: {e}")

    finally:
        # Close the serial connection
        if 'ser' in locals() and ser.is_open:
            ser.close()
        log_message("Serial connection closed.")

if __name__ == "__main__":
    main()
