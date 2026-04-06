#!/usr/bin/env python3
import serial
import time

try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=3)
    time.sleep(3)
    
    ser.reset_input_buffer()
    time.sleep(0.5)
    
    print('Sending CAPACITY command (waiting up to 5 seconds)...')
    ser.write(b'CAPACITY\n')
    
    # Wait longer and read all output
    start_time = time.time()
    all_output = []
    
    while time.time() - start_time < 5:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            if line:
                all_output.append(line)
                print(f'  {line}')
                if 'CAP:' in line:
                    break
        time.sleep(0.1)
    
    # Parse capacity
    for line in all_output:
        if line.startswith('CAP:'):
            parts = line.split(':')
            if len(parts) >= 5:
                bio = int(parts[2])
                nonbio = int(parts[4])
                print(f'\n📊 Current Sensor Readings:')
                print(f'  🟢 BIO Bin: {bio}%')
                print(f'  🔴 NON-BIO Bin: {nonbio}%')
    
    ser.close()
except Exception as e:
    print(f'❌ Error: {e}')
