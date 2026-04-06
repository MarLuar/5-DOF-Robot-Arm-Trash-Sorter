#!/usr/bin/env python3
import serial
import time

try:
    ser = serial.Serial('/dev/ttyACM1', 115200, timeout=2)
    time.sleep(3)
    
    # Clear buffer
    ser.reset_input_buffer()
    time.sleep(0.5)
    
    # Test 1: Send CAPACITY
    print('Test 1: Sending CAPACITY command')
    ser.write(b'CAPACITY\n')
    time.sleep(1)
    
    responses = []
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        if line:
            responses.append(line)
            print(f'  {line}')
    
    if not responses:
        print('  No response to CAPACITY')
    
    # Test 2: Send a simple servo command
    print('\nTest 2: Sending servo command (0 90)')
    ser.write(b'0 90\n')
    time.sleep(1)
    
    responses = []
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        if line:
            responses.append(line)
            print(f'  {line}')
    
    if not responses:
        print('  No response to servo command')
    
    # Test 3: Just read any pending output
    print('\nTest 3: Reading any pending output')
    time.sleep(1)
    responses = []
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        if line:
            responses.append(line)
            print(f'  {line}')
    
    ser.close()
    print('\n✅ Serial communication test complete')
    
except Exception as e:
    print(f'❌ Error: {e}')
