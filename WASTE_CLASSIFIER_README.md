# Waste Classifier Training Guide

## Overview
Train a MobileNetV2 classifier to distinguish between **biodegradable** and **non-biodegradable** waste for automatic sorting with the robotic arm.

## Dataset
- **Location:** `/home/koogs/Documents/5DOF_Robotic_Arm_Vision/dataset/`
- **Classes:** 2 (biodegradable, non-biodegradable)
- **Total Images:** ~7000
- **Splits:**
  - Train: ~5600 images
  - Validation: ~1400 images
  - Test: ~700 images

## Quick Start

### 1. Install Dependencies
```bash
pip install tensorflow opencv-python numpy matplotlib
```

### 2. Train the Model
```bash
cd /home/koogs/Documents/5DOF_Robotic_Arm_Vision
python3 train_waste_classifier.py
```

**Training takes:** ~30-60 minutes (depends on GPU)

### 3. Verify Model
```bash
python3 waste_classifier.py
```

Expected output: `✅ Classifier is working!`

## Integration with Robotic Arm

The classifier automatically integrates with `unified_control.py`:

1. **Detection:** Camera detects object via background subtraction
2. **Classification:** Classifier determines biodegradable vs non-biodegradable
3. **Sequence Selection:**
   - Biodegradable → Uses `A1_BIO` sequence (base=180°)
   - Non-biodegradable → Uses `A1_NON` sequence (base=0°)
4. **Pickup:** Robotic arm executes appropriate sequence

## Model Files
After training, models are saved to:
- `/home/koogs/Documents/5DOF_Robotic_Arm_Vision/models/waste_classifier_latest.h5`
- `/home/koogs/Documents/5DOF_Robotic_Arm_Vision/models/waste_classifier_best.h5`

## Usage in Code

```python
from waste_classifier import WasteClassifier

# Initialize
classifier = WasteClassifier('/path/to/model.h5')

# Classify object in bounding box
frame = camera_frame  # BGR format
bbox = (x, y, w, h)   # Bounding box
waste_type, confidence = classifier.classify(frame, bbox)

print(f"Type: {waste_type}, Confidence: {confidence:.2%}")
```

## Expected Accuracy
With 7000 images, you should achieve:
- **Validation Accuracy:** 92-96%
- **Precision:** >0.90
- **Recall:** >0.90

## Troubleshooting

### Low Accuracy (<85%)
- Increase epochs to 20-25
- Check dataset balance (equal bio/non-bio images)
- Verify image quality

### Out of Memory
- Reduce `BATCH_SIZE` to 16 or 8
- Close other applications

### Slow Training
- Normal on CPU (no GPU)
- Consider using Google Colab (free GPU)

## Google Colab (Optional - Faster Training)

Upload dataset to Colab:
```python
from google.colab import drive
drive.mount('/content/drive')

# Upload your dataset
# Run training
!python3 train_waste_classifier.py

# Download model
from google.colab import files
files.download('models/waste_classifier_best.h5')
```

## Next Steps

1. ✅ Train model
2. ✅ Test classifier
3. ✅ Integrate with `unified_control.py`
4. ✅ Test with real objects
5. ✅ Fine-tune if needed

---

**Questions?** Check the code comments or review training logs in `/models/`
