#!/usr/bin/env python3
"""
Waste Classification Module
Integrates with unified_control.py for automatic waste type detection
"""

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
import numpy as np
import cv2
import os

class WasteClassifier:
    """Biodegradable vs Non-biodegradable classifier"""

    def __init__(self, model_path=None):
        """Initialize classifier

        Args:
            model_path: Path to trained .h5 model file
        """
        if model_path is None:
            # Auto-find best model (prefer inference model without augmentation)
            model_dir = '/home/koogs/Documents/5DOF_Robotic_Arm_Vision/models'
            inf_path = f"{model_dir}/waste_inference.keras"
            keras_path = f"{model_dir}/waste_classifier_best.keras"
            h5_path = f"{model_dir}/waste_classifier_best.h5"

            if os.path.exists(inf_path):
                model_path = inf_path
            elif os.path.exists(keras_path):
                model_path = keras_path
            elif os.path.exists(h5_path):
                model_path = h5_path
            else:
                raise FileNotFoundError(f"No model found in {model_dir}")

        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model not found: {model_path}")

        print(f"Loading waste classifier from: {model_path}")

        # Load model directly (.keras format handles custom layers)
        self.model = tf.keras.models.load_model(model_path)

        self.classes = ['biodegradable', 'non-biodegradable']  # Match training: class 0 = bio, class 1 = non-bio
        self.img_size = (160, 160)  # Match training image size
        print(f"Classifier ready: {self.classes}")
    
    def classify(self, frame, bbox):
        """Classify object in bounding box

        Args:
            frame: Camera frame (BGR format from OpenCV)
            bbox: (x, y, w, h) bounding box tuple

        Returns:
            tuple: (class_name, confidence)
                - class_name: 'biodegradable' or 'non-biodegradable'
                - confidence: 0.0 to 1.0
        """
        try:
            # Extract ROI (region of interest)
            x, y, w, h = bbox

            # Safety check
            if w <= 0 or h <= 0:
                return 'unknown', 0.0

            # Crop and resize
            roi = frame[y:y+h, x:x+w]
            if roi.size == 0:
                return 'unknown', 0.0

            roi = cv2.resize(roi, self.img_size)

            # Convert BGR to RGB
            roi = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)

            # Preprocess (inference model expects raw 0-255 input, applies preprocess_input internally)
            roi = roi.astype(np.float32)
            roi = np.expand_dims(roi, axis=0)

            # Predict
            prediction = self.model.predict(roi, verbose=0)[0][0]

            # Convert to class and confidence
            # prediction > 0.5 → class 1 (non-biodegradable)
            # prediction < 0.5 → class 0 (biodegradable)
            class_idx = 1 if prediction > 0.5 else 0
            confidence = float(prediction) if prediction > 0.5 else float(1 - prediction)

            return self.classes[class_idx], confidence

        except Exception as e:
            print(f"Classification error: {e}")
            return 'unknown', 0.0
    
    def classify_batch(self, frame, bboxes):
        """Classify multiple objects efficiently
        
        Args:
            frame: Camera frame (BGR)
            bboxes: List of (x, y, w, h) bounding boxes
            
        Returns:
            list: [(class_name, confidence), ...]
        """
        results = []
        
        for bbox in bboxes:
            class_name, confidence = self.classify(frame, bbox)
            results.append((class_name, confidence))
        
        return results


# Test function
if __name__ == "__main__":
    """Test the classifier"""
    print("Testing Waste Classifier...")
    
    # Try to load model
    try:
        classifier = WasteClassifier()
        print("✓ Model loaded successfully")
        
        # Create test image
        test_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Test classification
        bbox = (100, 100, 200, 200)
        class_name, confidence = classifier.classify(test_img, bbox)
        
        print(f"✓ Test classification: {class_name} ({confidence:.2f})")
        print("\n✅ Classifier is working!")
        
    except Exception as e:
        print(f"✗ Error: {e}")
        print("\nMake sure to train the model first:")
        print("  python3 train_waste_classifier.py")
