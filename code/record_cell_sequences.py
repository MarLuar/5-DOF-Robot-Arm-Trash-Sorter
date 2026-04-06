#!/usr/bin/env python3
"""
Record pickup sequences for each grid cell (4x4 = 16 cells)
"""

import json
import time
import serial
import serial.tools.list_ports
import sys
from pathlib import Path

# Add project root to path for config import
sys.path.insert(0, str(Path(__file__).parent.parent))
import config

# Grid configuration
GRID_ROWS = 4
GRID_COLS = 4
CELL_NAMES = [f"{chr(ord('A')+row)}{col+1}" for row in range(GRID_ROWS) for col in range(GRID_COLS)]

# Sequence file
SEQUENCES_FILE = str(config.SEQUENCES_FILE)


def load_sequences():
    """Load existing sequences"""
    try:
        with open(SEQUENCES_FILE, 'r') as f:
            return json.load(f)
    except:
        return {}


def save_sequences(sequences):
    """Save sequences to file"""
    with open(SEQUENCES_FILE, 'w') as f:
        json.dump(sequences, f, indent=2)
    print(f"✓ Sequences saved to: {SEQUENCES_FILE}")


def connect_arduino():
    """Connect to Arduino"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'Arduino' in port.description or 'ttyACM' in port.device or 'CH340' in port.description:
            try:
                ser = serial.Serial(port.device, 115200, timeout=1)
                time.sleep(2)
                print(f"✓ Connected to {port.device}")
                return ser
            except:
                pass
    print("✗ Could not connect to Arduino")
    return None


def send_command(ser, servo_num, angle):
    """Send command to Arduino"""
    command = f"{servo_num} {angle}\n"
    ser.write(command.encode())
    time.sleep(0.1)


def send_multi_move(ser, angles):
    """Send multi-move command"""
    command = f"M {angles[0]} {angles[1]} {angles[2]} {angles[3]} {angles[4]}\n"
    ser.write(command.encode())
    time.sleep(0.1)


def record_sequence(cell_name, ser):
    """Record a sequence for a cell"""
    print(f"\n{'='*60}")
    print(f"📍 RECORDING SEQUENCE FOR CELL {cell_name}")
    print(f"{'='*60}")
    print("\n📋 Instructions:")
    print("1. Use the robotic arm controller to move the arm")
    print("2. Position the gripper above the trash in this cell")
    print("3. Lower to grab position")
    print("4. Note the angles for each position")
    print("\nOR use manual recording:")
    print("  Enter angles for each position (or press ENTER to skip)")
    
    sequence = []
    
    # Position 1: Approach (above trash)
    print("\n🔹 Position 1: Approach (above trash)")
    angles = input("  Enter angles [base shoulder elbow wrist gripper]: ").strip()
    if angles:
        try:
            angles = [int(a) for a in angles.split()]
            if len(angles) == 5:
                sequence.append({'name': 'Approach', 'angles': angles, 'delay': 1000})
                print(f"  ✓ Saved: {angles}")
                # Send to Arduino
                if ser:
                    send_multi_move(ser, angles)
                    time.sleep(1)
        except:
            print("  ✗ Invalid angles")
    
    # Position 2: Pickup (grab trash)
    print("\n🔹 Position 2: Pickup (grab trash)")
    angles = input("  Enter angles [base shoulder elbow wrist gripper]: ").strip()
    if angles:
        try:
            angles = [int(a) for a in angles.split()]
            if len(angles) == 5:
                sequence.append({'name': 'Pickup', 'angles': angles, 'delay': 500})
                print(f"  ✓ Saved: {angles}")
                if ser:
                    send_multi_move(ser, angles)
                    time.sleep(0.5)
        except:
            print("  ✗ Invalid angles")
    
    # Position 3: Close Gripper
    print("\n🔹 Position 3: Close Gripper")
    gripper_angle = input("  Enter gripper angle [0=closed, 140=open] (default: 0): ").strip()
    if gripper_angle:
        try:
            gripper_angle = int(gripper_angle)
            # Get current angles and just change gripper
            if sequence and 'angles' in sequence[-1]:
                angles = sequence[-1]['angles'].copy()
                angles[4] = gripper_angle
                sequence.append({'name': 'Close Gripper', 'angles': angles, 'delay': 500})
                print(f"  ✓ Saved: gripper={gripper_angle}")
                if ser:
                    send_command(ser, 4, gripper_angle)
                    time.sleep(0.5)
        except:
            print("  ✗ Invalid angle")
    
    # Position 4: Lift
    print("\n🔹 Position 4: Lift")
    angles = input("  Enter angles [base shoulder elbow wrist gripper]: ").strip()
    if angles:
        try:
            angles = [int(a) for a in angles.split()]
            if len(angles) == 5:
                sequence.append({'name': 'Lift', 'angles': angles, 'delay': 1000})
                print(f"  ✓ Saved: {angles}")
                if ser:
                    send_multi_move(ser, angles)
                    time.sleep(1)
        except:
            print("  ✗ Invalid angles")
    
    # Position 5: Return to Rest
    print("\n🔹 Position 5: Return to Rest")
    rest_angles = [150, 0, 70, 70, 140]  # Default rest position
    angles = input(f"  Enter angles [default: {rest_angles}]: ").strip()
    if angles:
        try:
            angles = [int(a) for a in angles.split()]
            if len(angles) == 5:
                rest_angles = angles
        except:
            pass
    sequence.append({'name': 'Rest', 'angles': rest_angles, 'delay': 1000})
    print(f"  ✓ Saved: {rest_angles}")
    if ser:
        send_multi_move(ser, rest_angles)
        time.sleep(1)
    
    return sequence


def main():
    print("="*60)
    print("🎯 CELL SEQUENCE RECORDER")
    print("="*60)
    print(f"\n📋 Grid: {GRID_COLS}x{GRID_ROWS} = {len(CELL_NAMES)} cells")
    print(f"   Cells: {', '.join(CELL_NAMES)}")
    
    # Load existing sequences
    sequences = load_sequences()
    recorded = len([c for c in CELL_NAMES if c in sequences])
    print(f"\n✓ Previously recorded: {recorded}/{len(CELL_NAMES)} cells")
    
    # Connect to Arduino
    ser = connect_arduino()
    
    # Record sequences
    print("\n" + "="*60)
    print("RECORDING SEQUENCES")
    print("="*60)
    
    for cell_name in CELL_NAMES:
        if cell_name in sequences:
            overwrite = input(f"\n⚠️  Cell {cell_name} already recorded. Overwrite? [y/N]: ").strip().lower()
            if overwrite != 'y':
                print(f"  ⊘ Skipping {cell_name}")
                continue
        
        sequence = record_sequence(cell_name, ser)
        
        if len(sequence) >= 2:
            sequences[cell_name] = {
                'cell': cell_name,
                'sequence': sequence,
                'recorded_at': time.time(),
            }
            save_sequences(sequences)
            print(f"\n✅ Cell {cell_name} recorded ({len(sequence)} steps)")
        else:
            print(f"\n⚠️  Cell {cell_name} needs at least 2 positions")
    
    # Summary
    print("\n" + "="*60)
    print("📊 RECORDING SUMMARY")
    print("="*60)
    recorded = len(sequences)
    print(f"   Recorded: {recorded}/{len(CELL_NAMES)} cells")
    print(f"   File: {SEQUENCES_FILE}")
    
    if recorded >= len(CELL_NAMES):
        print("\n✅ ALL CELLS RECORDED!")
        print("\n🚀 NEXT: Run vision pickup")
        print(f"   python3 {config.PROJECT_ROOT / 'code' / 'unified_control.py'}")
    else:
        print("\n⚠️  Some cells still need recording")
        print("   Run this script again to complete")
    
    print("="*60)
    
    if ser:
        ser.close()


if __name__ == "__main__":
    main()
