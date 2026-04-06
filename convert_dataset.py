#!/usr/bin/env python3
"""
Convert YOLO-format dataset to classification format
Reads labels and organizes images into class folders
"""

import os
import shutil
from pathlib import Path
import sys

# Add project root to path for config import
sys.path.insert(0, str(Path(__file__).parent))
import config

# Paths (using cross-platform config)
YOLO_DATASET = str(config.DATASET_DIR)
CLASS_DATASET = str(config.DATASET_CLASSIFICATION_DIR)

# Class mapping (from your classes.txt)
# 0 = biodegradable, 1 = non-biodegradable
CLASS_NAMES = ['biodegradable', 'non-biodegradable']

def convert_split(split_name):
    """Convert one split (train/val/test)"""
    print(f"\nConverting {split_name}...")
    
    # Create output directories
    for cls in CLASS_NAMES:
        os.makedirs(f"{CLASS_DATASET}/{split_name}/{cls}", exist_ok=True)
    
    # Read image paths from txt file
    txt_file = f"{YOLO_DATASET}/{split_name}.txt"
    if not os.path.exists(txt_file):
        print(f"  No {split_name}.txt found, skipping")
        return
    
    with open(txt_file, 'r') as f:
        lines = f.readlines()
    
    count = {0: 0, 1: 0}
    
    for line in lines:
        line = line.strip()
        if not line:
            continue
        
        # Line format: /content/final_dataset/images/train/class_img.jpg
        img_path = line.split()[0]
        
        # Extract just the filename
        img_name = os.path.basename(img_path)
        
        # Determine class from filename
        # Format: class_img.jpg (e.g., cardboard_123.jpg, plastic_456.jpg)
        class_prefix = img_name.split('_')[0].lower()
        
        # Map to biodegradable or non-biodegradable
        # Biodegradable: cardboard, paper, food, leaf, wood
        # Non-biodegradable: plastic, glass, metal, ewaste
        if class_prefix in ['cardboard', 'paper', 'food', 'leaf', 'wood']:
            class_idx = 0  # biodegradable
        elif class_prefix in ['plastic', 'glass', 'metal', 'ewaste']:
            class_idx = 1  # non-biodegradable
        else:
            print(f"  Unknown class: {class_prefix} in {img_name}")
            continue
        
        # Copy image to classification folder
        src = f"{YOLO_DATASET}/images/{split_name}/{img_name}"
        dst = f"{CLASS_DATASET}/{split_name}/{CLASS_NAMES[class_idx]}/{img_name}"
        
        if os.path.exists(src):
            shutil.copy2(src, dst)
            count[class_idx] += 1
    
    print(f"  Biodegradable: {count[0]} images")
    print(f"  Non-biodegradable: {count[1]} images")
    print(f"  Total: {count[0] + count[1]} images")

# Convert all splits
convert_split('train')
convert_split('val')
convert_split('test')

print("\n" + "="*60)
print("Dataset conversion complete!")
print(f"Output directory: {CLASS_DATASET}")
print("="*60)

# Verify
for split in ['train', 'val', 'test']:
    for cls in CLASS_NAMES:
        folder = f"{CLASS_DATASET}/{split}/{cls}"
        if os.path.exists(folder):
            img_count = len([f for f in os.listdir(folder) if f.endswith(('.jpg', '.jpeg', '.png'))])
            print(f"{split}/{cls}: {img_count} images")
