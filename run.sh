#!/bin/bash
# 5-DOF Robotic Arm Vision System - Quick Start Menu

PROJECT_DIR="/home/koogs/Documents/5DOF_Robotic_Arm_Vision"
CODE_DIR="$PROJECT_DIR/code"
TESTS_DIR="$PROJECT_DIR/tests"

echo "============================================================"
echo "  🤖 5-DOF ROBOTIC ARM VISION SYSTEM"
echo "============================================================"
echo ""
echo "Select an option:"
echo ""
echo "  CALIBRATION:"
echo "  1. Manual Marker Calibration (Most Accurate)"
echo "  2. Smart Calibration (Auto-detect)"
echo "  3. Test Grid Overlay (Verify Calibration)"
echo ""
echo "  SEQUENCE RECORDING:"
echo "  4. Record Cell Sequences (16 cells)"
echo ""
echo "  OPERATION:"
echo "  5. Vision Pickup (Auto)"
echo "  6. Manual Control (GUI)"
echo ""
echo "  TESTING:"
echo "  7. Analyze Camera"
echo "  8. Quick Vision Test"
echo ""
echo "  0. Exit"
echo ""
echo "============================================================"
read -p "Enter choice [0-8]: " choice

case $choice in
    1)
        echo "Running Manual Marker Calibration..."
        cd "$CODE_DIR" && python3 manual_marker_calibrate.py
        ;;
    2)
        echo "Running Smart Calibration..."
        cd "$CODE_DIR" && python3 smart_calibration.py
        ;;
    3)
        echo "Testing Grid Overlay..."
        cd "$TESTS_DIR" && python3 test_grid_overlay.py
        ;;
    4)
        echo "Recording Cell Sequences..."
        cd "$CODE_DIR" && python3 record_cell_sequences.py
        ;;
    5)
        echo "Running Vision Pickup..."
        cd "$CODE_DIR" && python3 vision_pickup_cell_based.py
        ;;
    6)
        echo "Starting Manual Control GUI..."
        cd "$CODE_DIR" && python3 robotic_arm_controller.py
        ;;
    7)
        echo "Analyzing Camera..."
        cd "$TESTS_DIR" && python3 analyze_camera.py
        ;;
    8)
        echo "Running Quick Vision Test..."
        cd "$TESTS_DIR" && python3 quick_vision_test.py
        ;;
    0)
        echo "Exiting..."
        exit 0
        ;;
    *)
        echo "Invalid option!"
        ;;
esac

echo ""
echo "============================================================"
echo "Done!"
echo "============================================================"
