import sys
import argparse
from adafruit_servokit import ServoKit

# Set up argument parser
parser = argparse.ArgumentParser(description='Control a servo motor.')
parser.add_argument('-d', '--degrees', type=int, required=True, help='Degrees to rotate the servo (0 to 270)')
parser.add_argument('-s', '--servo', type=int, required=True, help='Servo channel number')
args = parser.parse_args()

# Initialize the servo kit
kit = ServoKit(channels=16)

# Get the servo channel number
servo_channel = args.servo

# Configure the servo with the correct range for a 270-degree servo
kit.servo[servo_channel].actuation_range = 270
kit.servo[servo_channel].set_pulse_width_range(500, 2500)  # Adjust these values for your specific servo

# Ensure the degree value is within the valid range
if args.degrees < 0 or args.degrees > 270:
    print("Error: Degrees must be between 0 and 270.")
    sys.exit(1)

# Move the servo to the specified angle
print(f"Rotating servo on channel {servo_channel} to {args.degrees} degrees...")
kit.servo[servo_channel].angle = args.degrees
print("Rotation complete.")
