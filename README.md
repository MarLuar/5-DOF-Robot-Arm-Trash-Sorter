#  5-DOF Robotic Arm with Computer Vision Pickup

**Capstone Project - Automated Trash Pickup System**

---

##  Project Overview

This system enables a 5-DOF robotic arm to automatically detect and pickup trash using computer vision. The arm identifies colored objects on a 4x4 grid platform and executes pre-recorded pickup sequences for each cell.

---

##  Features

###  Completed
- [x] Manual robotic arm control GUI
- [x] Sequence builder (create, save, load, play sequences)
- [x] Speed control for servo movements
- [x] Preset management (save/load custom positions)
- [x] **Simple Grid Calibration** - Click 4 corners to calibrate
- [x] Digital grid overlay visualization
- [x] Camera integration (Z-Star 1080P USB camera)

---

##  Project Structure

```
5DOF_Robotic_Arm_Vision/
├── README.md                     # This file
├── run.sh                        # Quick start menu
│
├── code/                         # Main Python scripts
│   ├── robotic_arm_controller.py # Main control GUI
│   ├── simple_grid_calib.py      # ⭐ Grid calibration (4 clicks)
│   └── record_cell_sequences.py  # Record pickup sequences
│
├── calibration/                  # Calibration data
│   └── vision_calibration.json   # Grid calibration data
│
├── sequences/                    # Recorded sequences
│   └── cell_sequences.json       # Cell pickup sequences
│
├── docs/                         # Documentation
│   ├── PROJECT_TODO.md           # Project task list
│   ├── VISION_SETUP.md           # Setup guide
│   └── FILE_INVENTORY.md         # File inventory
│
└── images/                       # Reference images
```

---

##  Quick Start

### 1. Setup

```bash
cd /home/koogs/Documents/5DOF_Robotic_Arm_Vision

# Install dependencies (if needed)
pip3 install opencv-python numpy pyserial pillow --break-system-packages
```

### 2. Calibrate Grid (4 Clicks!)

```bash
cd code
python3 simple_grid_calib.py
```

**Click the 4 corners of your grid:**
1. Top-Left (Red)
2. Top-Right (Blue)
3. Bottom-Left (Green)
4. Bottom-Right (Yellow)

Click " Calculate Grid" → Verify overlay → Click " Save Calibration"

### 3. Record Cell Sequences

```bash
python3 record_cell_sequences.py
```

Record pickup sequence for each of the 16 cells (A1-D4)

### 4. Run Vision Pickup

```bash
python3 vision_pickup_cell_based.py
```

### 5. Manual Control (Optional)

```bash
python3 robotic_arm_controller.py
```

---

##  System Workflow

```
1. Calibrate Grid (4 clicks)
   └─> Click 4 corners
   └─> System calculates 25 intersection points
   └─> Save to calibration/vision_calibration.json

2. Record Sequences
   └─> For each cell (A1-D4)
   └─> Record: Approach → Pickup → Close → Lift → Rest
   └─> Save to sequences/cell_sequences.json

3. Vision Pickup
   └─> Capture camera image
   └─> Detect trash by color
   └─> Find nearest grid cell
   └─> Load and execute cell sequence
   └─> Trash picked up!
```

---

##  Hardware Requirements

| Component | Specification | Notes |
|-----------|---------------|-------|
| Robotic Arm | 5-DOF Servo Arm | Base, Shoulder, Elbow, Wrist, Gripper |
| Servo Controller | PCA9685 (16-channel PWM) | I2C interface |
| Camera | USB Webcam (720p/1080p) | Z-Star or Logitech C920 recommended |
| Microcontroller | Arduino Nano | For servo control |
| Grid Platform | 4x4 grid (5cm cells) | With 4 colored markers at corners |
| Computer | Linux/Windows | Python 3.x required |

---

## 📦 Firmware Upload

### Upload Arduino Sketch

```bash
# Using arduino-cli
arduino-cli compile --fqbn arduino:avr:nano firmware/5dof_robotic_arm/5dof_robotic_arm.ino
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:nano firmware/5dof_robotic_arm/5dof_robotic_arm.ino

# Or open in Arduino IDE
# File → Open → firmware/5dof_robotic_arm/5dof_robotic_arm.ino
# Then upload to Arduino
```

### Firmware Features
- Serial command parsing (`servo angle` format)
- Multi-move command for simultaneous servo movement (`M a1 a2 a3 a4 a5`)
- Speed control (`99 speed` command)
- Smooth interpolated servo movements
- Current position tracking

---

##  Configuration

### Grid Configuration (Default)
```python
GRID_ROWS = 4       # 4 rows (A, B, C, D)
GRID_COLS = 4       # 4 columns (1, 2, 3, 4)
CELL_SIZE_CM = 5    # Each cell is 5cm x 5cm
TOTAL_SIZE = "20cm x 20cm"
```

### Marker Positions (Default)
```
┌─────────────────┐
│  Red       Blue │  ← Top row (A)
│                 │
│                 │
│ Green     Purple│  ← Bottom row (D)
└─────────────────┘
```

### Servo Ranges
| Servo | Range | Notes |
|-------|-------|-------|
| Base | 0° - 180° | Full rotation |
| Shoulder | 30° - 110° | Limited for safety |
| Elbow | 30° - 110° | Limited for safety |
| Wrist | 30° - 110° | Limited for safety |
| Gripper | 0° - 140° | 0° = closed, 140° = open |

---

##  Testing

### Calibration Accuracy Test
```bash
cd tests
python3 test_grid_overlay.py
# Opens camera and shows grid overlay
# Verify grid lines align with physical grid
```

### Marker Detection Test
```bash
cd tests
python3 analyze_camera.py
# Detects all 4 markers and shows positions
```

### Pickup Accuracy Test
1. Place trash in center of cell A1
2. Run vision pickup
3. Record success/failure
4. Repeat for all 16 cells
5. Test edge cases (trash between cells)

---

## 📖 Documentation

| Document | Description |
|----------|-------------|
| `docs/PROJECT_TODO.md` | Complete project task list |
| `docs/VISION_SETUP.md` | Detailed vision system setup |
| `README.md` | This file - project overview |

---

## 🐛 Troubleshooting

### Camera Not Opening
```bash
# Check available cameras
ls -la /dev/video*

# Test camera
python3 -c "import cv2; c=cv2.VideoCapture(0); print(c.read()[0])"
```

### Calibration Failed
- Ensure all 4 markers are clearly visible
- Check lighting (even, diffused light works best)
- Clean camera lens
- Markers should be at grid corners

### Grid Overlay Misaligned
- Re-run calibration with accurate marker positions
- Use `manual_marker_calibrate.py` for precision
- Verify marker positions match your physical setup

### Servos Not Moving
- Check Arduino connection
- Verify serial port (usually `/dev/ttyACM0`)
- Check power supply to servo controller

---

## 📅 Project Timeline

| Week | Focus | Deliverables |
|------|-------|--------------|
| 1 | Vision Setup | Calibration working, sequences recorded |
| 2 | Vision Pickup | Working vision pickup script |
| 3 | GUI Integration | Unified control interface |
| 4 | Testing | Test results, optimized sequences |
| 5 | Documentation | Manual, README, video |
| 6 | Presentation | Slides, demo ready |

---

## 👥 Team

**Developer:** Mar Luar Igot

**Clients:** Garnet Garganza, Gene Adrian Capote, John Angelo Ayson, Jomari Tero, Vince Anos

---

##  License

This project is part of a capstone requirement. All rights reserved.

---

##  Acknowledgments

- Arduino community for servo control libraries
- OpenCV for computer vision tools
- Capstone advisors for guidance

---

**Last Updated:** March 29, 2025

**Status:** Phase 1 - Vision Setup In Progress
