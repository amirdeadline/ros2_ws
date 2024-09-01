"""
arduino_control.py

This Python script allows you to control linear actuators connected to an Arduino from a Raspberry Pi 
(or any other computer with a serial connection to the Arduino). The script sends commands over the 
serial port to extend, retract, or stop the linear actuators.

### How It Works

1. **Command-line Arguments**:
   - The script takes three command-line arguments:
     - `-d` or `--device`: Specifies which linear actuator to control ('la1' for actuator 1, 'la2' for actuator 2, or 'all' for both).
     - `-a` or `--action`: Specifies the action to perform ('e' for extend, 'r' for retract, 's' for stop).
     - `-c` or `--console`: Specifies the serial console port connected to the Arduino (default is '/dev/ttyACM0').

2. **Serial Communication**:
   - The script opens a serial connection to the Arduino and continuously reads output from the Arduino.
   - It sends commands formatted for the Arduino to control the linear actuators.

3. **Sending Commands**:
   - The script sends an initial command based on the input arguments and enters an interactive mode.
   - In interactive mode, the user can type additional commands to extend, retract, or stop the actuators or quit the script.

4. **Command Processing**:
   - The script validates user input commands and ensures they match the expected format.
   - Commands are sent to the Arduino over the serial connection, and the Arduino's responses are printed in real-time.

### Usage

Run the script from the command line with the appropriate arguments:

    python3 arduino_control.py -d la1 -a e

Example commands:
   - `python3 arduino_control.py -d la1 -a e` - Extend linear actuator 1.
   - `python3 arduino_control.py -d la2 -a r` - Retract linear actuator 2.
   - `python3 arduino_control.py -d all -a s` - Stop both actuators.

While running, you can interactively send commands like 'la1,e', 'la2,r', or 'STOP' to control the actuators.
Type 'q' to quit the script.
"""

import argparse
import serial
import threading
import time

def read_from_serial(ser):
    """Function to continuously read from the serial port and display output."""
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                print(f"Arduino response: {line}")
        except Exception as e:
            print(f"Error reading from serial: {e}")
            break

def send_command(ser, command):
    """Send a command to the Arduino."""
    ser.write((command + '\n').encode())
    ser.flush()  # Ensure the command is sent
    print(f"Sent command: {command}")

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Control Arduino linear actuators.')
    parser.add_argument('-d', '--device', type=str, required=True, choices=['la1', 'la2', 'all'], help='Device to control: la1, la2, or all')
    parser.add_argument('-a', '--action', type=str, required=True, choices=['e', 'r', 's'], help='Action to perform: e (extend), r (retract), s (stop)')
    parser.add_argument('-c', '--console', type=str, default='/dev/ttyACM0', help='Serial console port (default: /dev/ttyACM0)')

    args = parser.parse_args()

    try:
        # Open serial connection
        ser = serial.Serial(args.console, 9600, timeout=1)
        time.sleep(2)  # Wait for the serial connection to initialize
        print(f"Connected to Arduino on {args.console}.")

        # Start a thread to read from the serial port
        thread = threading.Thread(target=read_from_serial, args=(ser,))
        thread.daemon = True
        thread.start()

        # Construct command string based on arguments
        command = ""
        if args.device == 'all':
            command = "STOP"
        else:
            actuator_id = '1' if args.device == 'la1' else '2'
            command = f"{actuator_id},{args.action}"
        
        # Send the initial command to the Arduino
        send_command(ser, command)

        # Interactive mode to allow further commands
        while True:
            user_input = input("Enter command (e.g., 'la1,e', 'la2,r', 'STOP') or 'q' to quit: ")
            if user_input.lower() == 'q':
                break
            elif user_input in ["la1,e", "la1,r", "la1,s", "la2,e", "la2,r", "la2,s", "STOP"]:
                # Translate the user input to the correct command format for Arduino
                if user_input == "STOP":
                    command = "STOP"
                else:
                    parts = user_input.split(',')
                    actuator_id = '1' if parts[0] == 'la1' else '2'
                    action = parts[1]
                    command = f"{actuator_id},{action}"
                
                send_command(ser, command)
            else:
                print("Invalid command format. Please enter in the form 'la1,e', 'la2,r', or 'STOP'.")

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    except KeyboardInterrupt:
        print("Exiting...")

if __name__ == '__main__':
    main()
