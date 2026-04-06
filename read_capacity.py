#!/usr/bin/env python3
import serial
import time

try:
    ser = serial.Serial('/dev/ttyACM1', 115200, timeout=2)
    time.sleep(3)  # Wait for Arduino to fully initialize
    
    # Clear all buffered data
    ser.reset_input_buffer()
    time.sleep(0.5)
    
    print('Sending CAPACITY command...')
    ser.write(b'CAPACITY\n')
    time.sleep(1)
    
    # Read all responses
    responses = []
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        if line:
            responses.append(line)
            print(f'Response: {line}')
    
    # Parse capacity
    for resp in responses:
        if resp.startswith('CAP:'):
            parts = resp.split(':')
            if len(parts) >= 5:
                bio = int(parts[2])
                nonbio = int(parts[4])
                print(f'\n📊 Current Sensor Readings:')
                print(f'  🟢 BIO Bin: {bio}%')
                print(f'  🔴 NON-BIO Bin: {nonbio}%')
                
                if bio > 80 or nonbio > 80:
                    print('\n⚠️  WARNING: Bin(s) are nearly full!')
                elif bio > 50 or nonbio > 50:
                    print('\n⚡ Notice: Bin(s) are half full')
                else:
                    print('\n✅ Bin capacity levels are good')
    
    if not any('CAP:' in r for r in responses):
        print('\n⚠️  No capacity data received')
        print('This could mean:')
        print('  - Ultrasonic sensors are not connected')
        print('  - Sensors are not powered')
        print('  - Wiring issue')
        
    ser.close()
except Exception as e:
    print(f'❌ Error: {e}')
