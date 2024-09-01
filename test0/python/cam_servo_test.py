import time
import argparse
from adafruit_servokit import ServoKit

# Initialize the PCA9685 using the default address (0x40)
kit = ServoKit(channels=16)

# Configure the servo min and max pulse lengths for 0 and 180 degrees
kit.servo[1].set_pulse_width_range(500, 2500)

# Function to rotate the servo to a specific angle
def rotate_servo_to_angle(servo_channel, angle):
    if angle < -180 or angle > 180:
        raise ValueError("Angle must be between -180 and 180 degrees.")

    print(f"Rotating servo to {angle} degrees...")
    # Normalize angle to 0-180 for the servo (if the servo supports -180 to 180)
    normalized_angle = (angle + 180) % 360  # Converting -180-180 to 0-360 range
    if normalized_angle > 180:
        normalized_angle = 360 - normalized_angle  # Bring back to 0-180 range
    kit.servo[servo_channel].angle = normalized_angle
    time.sleep(1)
    print("Rotation complete.")

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Rotate a servo to a specified angle.")
parser.add_argument('-d', '--degrees', type=int, required=True, help="Degrees to rotate the servo (-180 to 180).")
args = parser.parse_args()

# Rotate the servo connected to channel 1 (CAM Servo) to the specified angle
rotate_servo_to_angle(1, args.degrees)
