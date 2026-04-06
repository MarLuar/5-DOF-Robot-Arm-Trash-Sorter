#!/usr/bin/env python3
"""
Copy all BIO sequences (A1-C4) to their NON-BIO counterparts
and change Step 7 & 8 base angle to 0°
"""

import json
import copy
import sys
from pathlib import Path

# Add project root to path for config import
sys.path.insert(0, str(Path(__file__).parent))
import config

SEQUENCES_FILE = str(config.SEQUENCES_FILE)

# Load sequences
with open(SEQUENCES_FILE, 'r') as f:
    sequences = json.load(f)

# Cell names A1-C4 (rows A, B, C; columns 1-4)
CELL_NAMES = [f"{row}{col}" for row in ['A', 'B', 'C'] for col in range(1, 5)]

print("=" * 60)
print("COPYING BIO SEQUENCES TO NON-BIO (Steps 7-8 base = 0°)")
print("=" * 60)

copied_count = 0
modified_count = 0

for cell in CELL_NAMES:
    non_cell = f"{cell}_NON"
    
    if cell not in sequences:
        print(f"⚠ {cell}: No BIO sequence found, skipping")
        continue
    
    # Deep copy the BIO sequence
    bio_sequence = copy.deepcopy(sequences[cell])
    
    # Check if sequence has at least 8 steps
    if len(bio_sequence) < 8:
        print(f"⚠ {cell}: Only {len(bio_sequence)} steps, need 8+, skipping")
        continue
    
    # Modify steps 7 and 8 (indices 6 and 7) - set base to 0°
    for step_idx in [6, 7]:
        if 'angles' in bio_sequence[step_idx]:
            old_base = bio_sequence[step_idx]['angles'][0]
            bio_sequence[step_idx]['angles'][0] = 0  # Set to 0° for NON-BIO
            modified_count += 1
            print(f"  Step {step_idx + 1}: Base {old_base}° → 0°")
    
    # Save to NON-BIO cell
    sequences[non_cell] = bio_sequence
    copied_count += 1
    print(f"✓ {cell} → {non_cell} (Steps 7-8 base = 0°)")

# Save modified sequences
with open(SEQUENCES_FILE, 'w') as f:
    json.dump(sequences, f, indent=2)

print("\n" + "=" * 60)
print(f"COMPLETE!")
print(f"  Copied: {copied_count} sequences")
print(f"  Modified steps: {modified_count}")
print(f"  Saved to: {SEQUENCES_FILE}")
print("=" * 60)
