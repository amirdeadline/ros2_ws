import serial

# Open serial connection
ser = serial.Serial('/dev/serial1', 9600, timeout=1)  # Change baud rate if necessary

print("Listening on /dev/serial1...")
while True:
    if ser.in_waiting > 0:  # Check if there is any data to read
        data = ser.readline().decode('utf-8').strip()
        if data:
            print("Received:", data)
