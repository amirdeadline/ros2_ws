import sys
import argparse
from adafruit_servokit import ServoKit

# Set up argument parser
parser = argparse.ArgumentParser(description='Control a servo motor.')
parser.add_argument('-d', '--degrees', type=int, required=True, help='Degrees to rotate the servo (-180 to 180)')
parser.add_argument('-s', '--servo', type=int, required=True, help='Servo channel number')
args = parser.parse_args()

# Initialize the servo kit
kit = ServoKit(channels=16)

# Get the servo channel number
servo_channel = args.servo

# Ensure the degree value is within the valid range
if args.degrees < -180 or args.degrees > 180:
    print("Error: Degrees must be between -180 and 180.")
    sys.exit(1)

# Move the servo to the specified angle
print(f"Rotating servo on channel {servo_channel} to {args.degrees} degrees...")
kit.servo[servo_channel].angle = args.degrees
print("Rotation complete.")
