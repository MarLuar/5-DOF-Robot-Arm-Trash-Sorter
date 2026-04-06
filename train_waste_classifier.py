#!/usr/bin/env python3
"""
Train waste classifier (MobileNetV2)
Biodegradable vs Non-biodegradable classification
"""

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
import numpy as np
import os
from datetime import datetime

# Configuration (optimized for CPU/low memory)
IMG_SIZE = (160, 160)  # Reduced from 224x224 for MobileNetV2
BATCH_SIZE = 16        # Reduced from 32 to save memory
EPOCHS = 15
LEARNING_RATE = 0.001

# Limit TensorFlow memory growth
tf.config.set_soft_device_placement(True)
gpus = tf.config.list_physical_devices('GPU')
if gpus:
    try:
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
    except RuntimeError as e:
        print(e)

# Paths
DATASET_DIR = '/home/koogs/Documents/5DOF_Robotic_Arm_Vision/dataset_classification'
MODEL_DIR = '/home/koogs/Documents/5DOF_Robotic_Arm_Vision/models'

# Create model directory
os.makedirs(MODEL_DIR, exist_ok=True)

print("=" * 60)
print("WASTE CLASSIFIER TRAINING")
print("=" * 60)

# Check GPU
print(f"\nTensorFlow version: {tf.__version__}")
print(f"GPU Available: {tf.config.list_physical_devices('GPU')}")

# Load data
print("\nLoading dataset...")
train_ds = tf.keras.utils.image_dataset_from_directory(
    f"{DATASET_DIR}/train",  # Fixed: removed /images
    image_size=IMG_SIZE,
    batch_size=BATCH_SIZE,
    shuffle=True
)

val_ds = tf.keras.utils.image_dataset_from_directory(
    f"{DATASET_DIR}/val",  # Fixed: removed /images
    image_size=IMG_SIZE,
    batch_size=BATCH_SIZE,
    shuffle=False
)

class_names = train_ds.class_names
print(f"Classes: {class_names}")
print(f"Training images: {len(train_ds) * BATCH_SIZE}")
print(f"Validation images: {len(val_ds) * BATCH_SIZE}")

# Optimize performance
AUTOTUNE = tf.data.AUTOTUNE
train_ds = train_ds.cache().shuffle(1000).prefetch(buffer_size=AUTOTUNE)
val_ds = val_ds.cache().prefetch(buffer_size=AUTOTUNE)

# Data augmentation (prevents overfitting)
data_augmentation = keras.Sequential([
    layers.RandomFlip("horizontal"),
    layers.RandomRotation(0.1),
    layers.RandomZoom(0.1),
    layers.RandomContrast(0.1),
], name="data_augmentation")

# Create model (MobileNetV2)
print("\nCreating MobileNetV2 model...")
base_model = keras.applications.MobileNetV2(
    input_shape=(IMG_SIZE[0], IMG_SIZE[1], 3),
    include_top=False,
    weights='imagenet',
    pooling='avg'
)
base_model.trainable = False  # Freeze base model initially

inputs = keras.Input(shape=(IMG_SIZE[0], IMG_SIZE[1], 3))
x = data_augmentation(inputs)
x = keras.applications.mobilenet_v2.preprocess_input(x)
x = base_model(x, training=False)
x = layers.Dropout(0.3)(x)
x = layers.Dense(128, activation='relu')(x)
x = layers.Dropout(0.2)(x)
outputs = layers.Dense(1, activation='sigmoid')(x)  # Binary classification

model = keras.Model(inputs, outputs)

model.compile(
    optimizer=keras.optimizers.Adam(learning_rate=LEARNING_RATE),
    loss='binary_crossentropy',
    metrics=['accuracy', keras.metrics.Precision(), keras.metrics.Recall()]
)

print("\nModel Summary:")
model.summary()

# Callbacks
checkpoint = keras.callbacks.ModelCheckpoint(
    f"{MODEL_DIR}/waste_classifier_best.keras",
    monitor='val_accuracy',
    save_best_only=True,
    mode='max',
    verbose=1
)

early_stop = keras.callbacks.EarlyStopping(
    monitor='val_loss',
    patience=5,
    restore_best_weights=True,
    verbose=1
)

# Train
print("\n" + "=" * 60)
print("STARTING TRAINING")
print("=" * 60)

history = model.fit(
    train_ds,
    validation_data=val_ds,
    epochs=EPOCHS,
    callbacks=[checkpoint, early_stop]
)

# Save final model
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
model.save(f"{MODEL_DIR}/waste_classifier_{timestamp}.keras")
model.save(f"{MODEL_DIR}/waste_classifier_latest.keras")
model.save(f"{MODEL_DIR}/waste_classifier_best.keras")  # Also save best as .keras

print("\n" + "=" * 60)
print("TRAINING COMPLETE")
print("=" * 60)
print(f"\nModels saved to: {MODEL_DIR}")
print(f"  - waste_classifier_best.h5 (best validation accuracy)")
print(f"  - waste_classifier_best.keras (Keras format)")
print(f"  - waste_classifier_latest.keras (latest)")
print(f"  - waste_classifier_{timestamp}.keras (timestamped)")

# Plot training history
import matplotlib.pyplot as plt

plt.figure(figsize=(15, 5))

plt.subplot(1, 3, 1)
plt.plot(history.history['accuracy'], label='Train Acc')
plt.plot(history.history['val_accuracy'], label='Val Acc')
plt.xlabel('Epoch')
plt.ylabel('Accuracy')
plt.legend()
plt.title('Accuracy')

plt.subplot(1, 3, 2)
plt.plot(history.history['loss'], label='Train Loss')
plt.plot(history.history['val_loss'], label='Val Loss')
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.legend()
plt.title('Loss')

plt.subplot(1, 3, 3)
plt.plot(history.history['precision'], label='Train Prec')
plt.plot(history.history['val_precision'], label='Val Prec')
plt.xlabel('Epoch')
plt.ylabel('Precision')
plt.legend()
plt.title('Precision')

plt.tight_layout()
plt.savefig(f"{MODEL_DIR}/training_history_{timestamp}.png", dpi=150)
print(f"\nTraining history plot saved: {MODEL_DIR}/training_history_{timestamp}.png")

# Evaluate on validation set
print("\n" + "=" * 60)
print("VALIDATION RESULTS")
print("=" * 60)
val_results = model.evaluate(val_ds, verbose=0)
print(f"Validation Loss: {val_results[0]:.4f}")
print(f"Validation Accuracy: {val_results[1]*100:.2f}%")
print(f"Validation Precision: {val_results[2]:.4f}")
print(f"Validation Recall: {val_results[3]:.4f}")

print("\n✅ Training complete! Ready to integrate with robotic arm system.")
