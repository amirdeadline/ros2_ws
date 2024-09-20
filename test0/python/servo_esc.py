import time
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# Create the I2C bus interface
i2c_bus = busio.I2C(SCL, SDA)

# Create a PCA9685 instance
pca = PCA9685(i2c_bus, address=0x40)  # Use the appropriate address if different
pca.frequency = 50  # Set frequency to 50 Hz, typical for ESCs

# Function to set the throttle
def set_throttle(channel, pulse_ms):
    # Calculate the PWM duty cycle corresponding to the pulse width
    pulse_length = int((pulse_ms / 1000.0) * pca.frequency * 4096)
    pca.channels[channel].duty_cycle = pulse_length

# Set a continuous disarm signal (800 microseconds) to channels 10, 11, and 12
disarm_pulse_ms = 0.8  # 800 microseconds is 0.8 milliseconds
print(f"Continuously sending disarm signal ({disarm_pulse_ms * 1000} us) to channels 10, 11, and 12...")

# Continuously send the disarm signal to the specified channels
while True:
    for ch in range(1,12):
    # for ch in [10, 11, 12]:
        set_throttle(ch, disarm_pulse_ms)
    time.sleep(0.02)  # Small delay to ensure the loop runs smoothly (50 Hz is 0.02 seconds per cycle)
