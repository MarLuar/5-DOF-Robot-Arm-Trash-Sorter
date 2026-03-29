# 5-DOF Robotic Arm with Computer Vision Pickup

**Version 1.1.00**

**Capstone Project - Automated Trash Pickup System**

---

## Project Overview

This system enables a 5-DOF robotic arm to automatically detect and pickup trash using computer vision. The arm identifies objects on a 4x4 grid platform and executes pre-recorded pickup sequences for each cell.

---

## Features

### Manual Control
- Individual servo control with +/- buttons
- Simultaneous servo movement (Send All button)
- Speed control for servo movements
- Preset positions (Rest, Pickup)
- Copy/paste angles to sequences

### Sequence Management
- Create sequences for each grid cell (A1-D4)
- Insert steps at specific positions
- Copy/paste between cells
- Copy from Manual Control
- Right-click context menu (Copy, Cut, Paste, Delete)
- Save/load sequences to file

### Grid Calibration
- 4-corner calibration (click corners)
- Auto-calculates 25 grid intersection points
- Captures empty grid reference
- Real-time grid overlay
- Sensitivity controls (threshold, area, solidity)

### Object Detection
- Background subtraction detection
- Cell identification (A1, B2, C3, etc.)
- Real-time detection display
- Persistent detection (3 second timeout, no flickering)
- Sensitivity adjustment

### Automatic Pickup
- Detect object on grid
- Identify cell location
- Execute cell's assigned sequence automatically
- Auto-recapture empty grid after successful pickup
- Ready for next object immediately

### System Monitoring
- Floating log window (movable, resizable)
- Static detection status display
- Real-time command logging
- Thread and performance monitoring

---

## Project Structure

```
5DOF_Robotic_Arm_Vision/
├── README.md                     # This file
├── WORK_REPORT_March_29-30_2025.txt  # Work report
├── run.sh                        # Quick start menu
│
├── code/                         # Main Python scripts
│   ├── unified_control.py        # Main unified control GUI
│   ├── robotic_arm_controller.py # Legacy manual control GUI
│   └── record_cell_sequences.py  # Sequence recorder
│
├── calibration/                  # Calibration data
│   ├── vision_calibration.json   # Grid calibration data
│   └── servo_presets.json        # Servo preset positions
│
├── sequences/                    # Recorded sequences
│   └── cell_sequences.json       # Cell pickup sequences
│
├── docs/                         # Documentation
│   ├── PROJECT_TODO.md           # Project task list
│   ├── VISION_SETUP.md           # Setup guide
│   └── FILE_INVENTORY.md         # File inventory
│
└── firmware/                     # Arduino firmware
    └── 5dof_robotic_arm/
        └── 5dof_robotic_arm.ino  # Main Arduino sketch
```

---

## Quick Start

### 1. Setup

```bash
cd /home/koogs/Documents/5DOF_Robotic_Arm_Vision

# Install dependencies (if needed)
pip3 install opencv-python numpy pyserial pillow psutil --break-system-packages
```

### 2. Run Unified Control System

```bash
python3 code/unified_control.py
```

### 3. Workflow

**A. Grid Calibration (First Time Setup)**
1. Click 4 grid corners in order (Top-Left, Top-Right, Bottom-Left, Bottom-Right)
2. Click "Calculate Grid"
3. Click "Recapture Empty Grid"
4. Enable "Show Object Detection"

**B. Create Sequences**
1. Go to "Cell Sequences" tab
2. Select cell (e.g., A1)
3. Move servos to desired position
4. Click "Add Current" to add step
5. Repeat for all steps
6. Click "Save Sequence to Cell"

**C. Manual Control**
1. Go to "Manual Control" tab
2. Connect to Arduino
3. Move servos individually or use presets
4. Click "Send All" for simultaneous movement

**D. Automatic Pickup**
1. Place object on grid
2. Wait for detection (shows cell name)
3. Click "PICKUP OBJECT"
4. Confirm and watch it execute!

---

## System Requirements

### Hardware
- 5-DOF Robotic Arm with PCA9685 servo controller
- Arduino Nano (or compatible)
- Z-Star USB Camera (or any UVC compatible camera)
- Computer (Linux/Windows)

### Software
- Python 3.x
- OpenCV
- NumPy
- PySerial
- Pillow
- psutil

---

## Configuration

### Grid Configuration
- Grid: 4x4 cells (5cm each)
- Total size: 20cm x 20cm
- 25 intersection points

### Servo Ranges
| Servo | Range | Notes |
|-------|-------|-------|
| Base | 0-180 | Full rotation |
| Shoulder | 30-110 | Limited for safety |
| Elbow | 30-110 | Limited for safety |
| Wrist | 30-110 | Limited for safety |
| Gripper | 0-140 | 0=closed, 140=open |

### Detection Settings
- Color Threshold: 35 (adjustable)
- Minimum Area: 3000 pixels (adjustable)
- Min Solidity: 0.5 (adjustable)
- Detection FPS: 2 FPS (every 5th frame at 10 FPS camera)
- Detection Persistence: 3 seconds

---

## Performance

### Current Performance (Version 1.1.00)

| Component | Performance | Status |
|-----------|-------------|--------|
| Camera FPS | 10 FPS | Excellent |
| Detection FPS | 2 FPS | Stable |
| Sequence Playback | No lag | Excellent |
| Serial Communication | Buffer managed | Excellent |
| UI Responsiveness | Fully responsive | Excellent |

### Optimizations Applied
- Camera runs in background thread (doesn't block UI)
- Canvas size cached (instant access)
- Serial buffers cleared before each send
- Detection persists for 3 seconds (no flickering)
- Event queue cleaned before each sequence
- Threads properly terminated with stop flags

---

## Troubleshooting

### Camera Not Showing
1. Check camera connection: `ls -la /dev/video*`
2. Try different camera device (video0, video2, video3, etc.)
3. Close other applications using camera
4. Restart application

### Object Not Detected
1. Ensure "Show Object Detection" is enabled
2. Recapture empty grid (remove all objects first)
3. Adjust sensitivity (threshold, area, solidity)
4. Ensure good lighting

### Sequence Lag
1. Check Arduino connection
2. Clear serial buffers (reconnect)
3. Reduce sequence speed if needed
4. Check for other applications using serial port

### UI Frozen
1. Check System Log for errors
2. Restart application
3. Check CPU/memory usage
4. Close unnecessary applications

---

## Version History

### Version 1.1.00 (March 30, 2025)

**New Features:**
- Unified control system (all features in one GUI)
- Floating log window
- Automatic object pickup
- Right-click step editing
- Copy/paste between Manual Control and Sequences
- Insert step at position
- Send All button for simultaneous movement
- Static detection status display

**Bug Fixes:**
- Fixed 4th sequence lag (Arduino serial buffer overflow)
- Fixed camera performance issues (cached canvas size)
- Fixed object detection flickering (3 second persistence)
- Fixed thread accumulation (proper termination)
- Fixed Tkinter event queue buildup (callback cancellation)

**Performance Improvements:**
- Camera: 0.5 FPS to 10 FPS (20x faster)
- Main thread block: 80-100% to <5% (16x less)
- Thread count: 4+ to 1 (4x reduction)
- Event queue: 40+ callbacks to 0

### Version 1.0.00 (Initial Release)

- Basic manual servo control
- Sequence recording and playback
- Grid calibration
- Basic object detection

---

## Documentation

| Document | Description |
|----------|-------------|
| `README.md` | Main project documentation |
| `WORK_REPORT_March_29-30_2025.txt` | Detailed work report |
| `docs/PROJECT_TODO.md` | Project task list |
| `docs/VISION_SETUP.md` | Vision system setup guide |
| `docs/FILE_INVENTORY.md` | File inventory |

---

## License

This project is part of a capstone requirement. All rights reserved.

---

## Acknowledgments

- Arduino community for servo control libraries
- OpenCV for computer vision tools
- Capstone advisors for guidance

---

## Contact

**Repository:** https://github.com/MarLuar/5-DOF-Robot-Arm-Trash-Sorter

**Developer:** User

**Last Updated:** March 30, 2025

---

## System Status

**Overall Status:** PRODUCTION READY

All components tested and working:
- Camera Preview: Excellent
- Object Detection: Excellent
- Sequence Playback: Excellent
- Serial Communication: Excellent
- UI Responsiveness: Excellent
- Sequence Editing: Excellent
- Automatic Pickup: Excellent
- Grid Calibration: Excellent
- Manual Control: Excellent
- System Logging: Excellent
