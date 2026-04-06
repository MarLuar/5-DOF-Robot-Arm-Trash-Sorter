@echo off
REM 5-DOF Robotic Arm Vision System - Quick Start Menu (Windows)

echo ============================================================
echo   5-DOF ROBOTIC ARM VISION SYSTEM
echo ============================================================
echo.
echo Select an option:
echo.
echo   CALIBRATION:
echo   1. Manual Marker Calibration (Most Accurate)
echo   2. Smart Calibration (Auto-detect)
echo   3. Test Grid Overlay (Verify Calibration)
echo.
echo   SEQUENCE RECORDING:
echo   4. Record Cell Sequences (16 cells)
echo.
echo   OPERATION:
echo   5. Vision Pickup (Auto)
echo   6. Manual Control (GUI)
echo.
echo   TESTING:
echo   7. Analyze Camera
echo   8. Quick Vision Test
echo.
echo   0. Exit
echo.
echo ============================================================
set /p choice="Enter choice [0-8]: "

if "%choice%"=="1" (
    echo Running Manual Marker Calibration...
    python code\unified_control.py
) else if "%choice%"=="2" (
    echo Running Smart Calibration...
    python code\simple_grid_calib.py
) else if "%choice%"=="3" (
    echo Testing Grid Overlay...
    python tests\test_grid_overlay.py
) else if "%choice%"=="4" (
    echo Recording Cell Sequences...
    python code\record_cell_sequences.py
) else if "%choice%"=="5" (
    echo Running Vision Pickup...
    python code\unified_control.py
) else if "%choice%"=="6" (
    echo Starting Manual Control GUI...
    python code\robotic_arm_controller.py
) else if "%choice%"=="7" (
    echo Analyzing Camera...
    python tests\analyze_camera.py
) else if "%choice%"=="8" (
    echo Running Quick Vision Test...
    python tests\quick_vision_test.py
) else if "%choice%"=="0" (
    echo Exiting...
    exit /b
) else (
    echo Invalid option!
)

echo.
echo ============================================================
echo Done!
echo ============================================================
pause
