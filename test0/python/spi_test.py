import spidev
import time

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device (CS) 0
spi.max_speed_hz = 100000  # Set SPI speed to 100kHz
spi.mode = 0b00  # SPI mode

try:
    while True:
        # Send test data (0xAA) and read response from Arduino
        response = spi.xfer2([0xAA])
        print(f"Response from Arduino: {response}")

        time.sleep(1)  # Wait for 1 second between transmissions

except KeyboardInterrupt:
    print("Exiting...")
finally:
    spi.close()  # Close SPI connection when done
