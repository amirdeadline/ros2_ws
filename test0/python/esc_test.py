import serial
import argparse

def send_speed_command(port, esc_id, speed):
    # Open the serial port
    with serial.Serial(port, 9600, timeout=1) as ser:
        # Construct the command string with esc_id and speed
        command = f"{esc_id},{speed}\n"
        # Send the command to Arduino
        ser.write(command.encode())

if __name__ == "__main__":
    # Set up argument parsing
    parser = argparse.ArgumentParser(description="Control motor speed via Arduino.")
    parser.add_argument('-p', '--port', type=str, default='/dev/ttyACM1', help='Serial port of Arduino (e.g., /dev/ttyACM1)')
    parser.add_argument('-e', '--esc_id', type=int, choices=[1, 2], required=True, help='ESC ID to control (1 for mower_esc, 2 for dethatcher_esc)')
    parser.add_argument('-s', '--speed', type=int, default=20, choices=range(-100, 101), help='Speed value between -100 and 100')

    args = parser.parse_args()

    # Call the function to send the speed command
    send_speed_command(args.port, args.esc_id, args.speed)
