#!/usr/bin/env python3
import serial
import time

try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    time.sleep(3)
    
    print("Reading Arduino output for 2 seconds...")
    start = time.time()
    while time.time() - start < 2:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"  {line}")
    
    print("\nSending CAPACITY...")
    ser.reset_input_buffer()
    ser.write(b'CAPACITY\n')
    
    print("Waiting for response (5 seconds)...")
    start = time.time()
    while time.time() - start < 5:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"  {line}")
                if 'CAP:' in line:
                    parts = line.split(':')
                    if len(parts) >= 5:
                        bio = int(parts[2])
                        nonbio = int(parts[4])
                        print(f"\n📊 BIO: {bio}% | NON-BIO: {nonbio}%")
                    break
    
    ser.close()
except Exception as e:
    print(f'Error: {e}')
