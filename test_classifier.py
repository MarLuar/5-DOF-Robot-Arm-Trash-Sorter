#!/usr/bin/env python3
"""
Test waste classifier on validation dataset
"""

import tensorflow as tf
import numpy as np
import os
import sys
from pathlib import Path

# Add project root to path for config import
sys.path.insert(0, str(Path(__file__).parent))
import config

from waste_classifier import WasteClassifier
import cv2

# Initialize classifier
print("Loading classifier...")
classifier = WasteClassifier()

# Test directories (using cross-platform config)
VAL_DIR = str(config.DATASET_CLASSIFICATION_DIR / 'val')
classes = ['biodegradable', 'non-biodegradable']

print("\n" + "=" * 60)
print("TESTING ON VALIDATION DATASET")
print("=" * 60)

correct = 0
total = 0
class_correct = {'biodegradable': 0, 'non-biodegradable': 0}
class_total = {'biodegradable': 0, 'non-biodegradable': 0}

for class_name in classes:
    class_dir = os.path.join(VAL_DIR, class_name)
    images = [f for f in os.listdir(class_dir) if f.endswith(('.jpg', '.png', '.jpeg'))]
    
    print(f"\nTesting {class_name}: {len(images)} images")
    
    for img_file in images[:50]:  # Test first 50 images per class
        img_path = os.path.join(class_dir, img_file)
        
        # Load image
        frame = cv2.imread(img_path)
        if frame is None:
            continue
        
        h, w = frame.shape[:2]
        bbox = (0, 0, w, h)  # Full image
        
        # Classify
        pred_class, confidence = classifier.classify(frame, bbox)
        
        # Check if correct
        total += 1
        class_total[class_name] += 1
        
        if pred_class == class_name:
            correct += 1
            class_correct[class_name] += 1
        else:
            print(f"  ✗ {img_file}: predicted {pred_class} ({confidence:.2f})")

# Results
print("\n" + "=" * 60)
print("RESULTS")
print("=" * 60)

for class_name in classes:
    acc = class_correct[class_name] / class_total[class_name] * 100 if class_total[class_name] > 0 else 0
    print(f"{class_name}: {class_correct[class_name]}/{class_total[class_name]} ({acc:.1f}%)")

overall_acc = correct / total * 100 if total > 0 else 0
print(f"\nOverall Accuracy: {correct}/{total} ({overall_acc:.1f}%)")
print("=" * 60)
