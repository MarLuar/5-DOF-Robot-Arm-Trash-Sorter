#!/usr/bin/env python3
import serial
import time

# Open at 9600
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=2)
time.sleep(2)

# Clear buffer
ser.reset_input_buffer()

# Send a simple servo command we know works
print("Sending: 0 90")
ser.write(b'0 90\n')
time.sleep(1)

# Read response
while ser.in_waiting > 0:
    line = ser.readline().decode('utf-8', errors='replace').strip()
    if line:
        print(f"Response: {line}")

ser.close()
