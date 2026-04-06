@echo off
REM Quick training script for waste classifier (Windows)

echo ==========================================
echo WASTE CLASSIFIER TRAINING
echo ==========================================

REM Check if TensorFlow is installed
python -c "import tensorflow" 2>nul
if errorlevel 1 (
    echo.
    echo TensorFlow not found!
    echo.
    echo Install it with:
    echo.
    echo   pip install tensorflow opencv-python matplotlib
    echo.
    pause
    exit /b 1
)

echo TensorFlow found!
echo.

REM Run training
echo Starting training...
echo This will take 30-60 minutes depending on your hardware.
echo.

python train_waste_classifier.py

echo.
echo ==========================================
echo Training complete!
echo ==========================================
pause
