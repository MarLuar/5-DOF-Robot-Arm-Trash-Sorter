#!/bin/bash
# Quick training script for waste classifier

echo "=========================================="
echo "WASTE CLASSIFIER TRAINING"
echo "=========================================="

# Check if TensorFlow is installed
if ! python3 -c "import tensorflow" 2>/dev/null; then
    echo ""
    echo "❌ TensorFlow not found!"
    echo ""
    echo "Install it with one of these commands:"
    echo ""
    echo "Option 1 (recommended - uses system packages):"
    echo "  sudo apt install python3-tensorflow python3-opencv python3-matplotlib"
    echo ""
    echo "Option 2 (pip with override):"
    echo "  pip install tensorflow opencv-python matplotlib --break-system-packages"
    echo ""
    echo "Option 3 (virtual environment):"
    echo "  python3 -m venv ~/tf_env"
    echo "  source ~/tf_env/bin/activate"
    echo "  pip install tensorflow opencv-python matplotlib"
    echo ""
    exit 1
fi

echo "✓ TensorFlow found!"
echo ""

# Run training
echo "Starting training..."
echo "This will take 30-60 minutes depending on your hardware."
echo ""

cd /home/koogs/Documents/5DOF_Robotic_Arm_Vision
python3 train_waste_classifier.py

echo ""
echo "=========================================="
echo "Training complete!"
echo "=========================================="
