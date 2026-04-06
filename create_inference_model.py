#!/usr/bin/env python3
"""Create inference model without data augmentation"""

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
import numpy as np
import cv2
import os
import sys
from pathlib import Path

# Add project root to path for config import
sys.path.insert(0, str(Path(__file__).parent))
import config

# Load the trained model
print("Loading trained model...")
trained_model = tf.keras.models.load_model(str(config.MODELS_DIR / 'waste_classifier_best.keras'))

# Get MobileNetV2 base model (layer 2)
base_model = trained_model.layers[2]
base_model.trainable = False

# Create inference model without data augmentation
print("Creating inference model...")
inputs = keras.Input(shape=(160, 160, 3))
x = keras.applications.mobilenet_v2.preprocess_input(inputs)
x = base_model(x, training=False)
x = layers.Dropout(0.3)(x)
x = layers.Dense(128, activation='relu')(x)
x = layers.Dropout(0.2)(x)
outputs = layers.Dense(1, activation='sigmoid')(x)
inference_model = keras.Model(inputs, outputs)

# Copy weights from trained classifier head
print("Copying weights...")
inference_model.layers[2].set_weights(trained_model.layers[3].get_weights())  # dropout
inference_model.layers[3].set_weights(trained_model.layers[4].get_weights())  # dense
inference_model.layers[4].set_weights(trained_model.layers[5].get_weights())  # dropout_1
inference_model.layers[5].set_weights(trained_model.layers[6].get_weights())  # dense_1

# Copy MobileNetV2 weights
for i, layer in enumerate(inference_model.layers[1].layers):
    try:
        layer.set_weights(trained_model.layers[2].layers[i].get_weights())
    except:
        pass

# Save inference model
inference_model.save(str(config.MODELS_DIR / 'waste_inference.keras'))
print("Inference model saved!")

# Test
val_dir = str(config.DATASET_CLASSIFICATION_DIR / 'val')
print("\nTesting inference model:")
for cls in ['biodegradable', 'non-biodegradable']:
    imgs = os.listdir(os.path.join(val_dir, cls))[:5]
    correct = 0
    for img in imgs:
        frame = cv2.imread(os.path.join(val_dir, cls, img))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.resize(frame, (160, 160))
        frame = frame.astype(np.float32)
        pred = inference_model.predict(np.expand_dims(frame, 0), verbose=0)[0][0]
        is_correct = (pred > 0.5) == (cls == 'non-biodegradable')
        if is_correct:
            correct += 1
        status = "✓" if is_correct else "✗"
        print(f"  {cls[:3]}/{img[:15]}: {pred:.4f} {status}")
    print(f"  {cls}: {correct}/5 correct")
