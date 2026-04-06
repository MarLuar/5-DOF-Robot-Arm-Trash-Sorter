#!/usr/bin/env python3
import serial
import time

try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    time.sleep(3)
    
    print("Reading all Arduino output for 3 seconds...")
    start = time.time()
    
    while time.time() - start < 3:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"  {line}")
    
    print("\nNow sending CAPACITY command...")
    ser.reset_input_buffer()
    ser.write(b'CAPACITY\n')
    
    print("Reading response for 5 seconds...")
    start = time.time()
    
    while time.time() - start < 5:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"  {line}")
                if 'CAP:' in line:
                    print("\n✅ Got capacity data!")
                    break
    
    ser.close()
except Exception as e:
    print(f'Error: {e}')
