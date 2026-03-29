# 5-DOF Robotic Arm - Computer Vision Setup

##  Prerequisites

```bash
pip3 install opencv-python numpy pillow --break-system-packages
```

##  Step-by-Step Guide

### Step 1: Adjust Camera Settings

First, find the right camera settings so the image isn't too bright:

1. **Run the Camera Settings GUI:**
   ```bash
   python3 /home/koogs/camera_settings_gui.py
   ```

2. **Adjust these settings:**
   - **Brightness:** -50 to 0 (reduce until grid is clear)
   - **Contrast:** 50-70 (increase for better marker detection)
   - **Saturation:** 50-70 (increase for vivid marker colors)
   - **Exposure:** -6 to -3 (lower = darker)
   - **Take snapshots** to test

3. **Note the good values** and update `CAMERA_SETTINGS` in `vision_calibration.py`

### Step 2: Prepare Your Grid Platform

1. **Print or draw a grid** with 4x6 cells (5cm each recommended)

2. **Place 4 colored markers at corners:**
   - 🔴 **Red** → Top-Left
   - 🟢 **Green** → Top-Right
   - 🟣 **Purple** → Bottom-Right
   - 🔵 **Blue** → Bottom-Left

3. **Mount camera overhead** pointing down at the grid

### Step 3: Run Calibration

```bash
python3 /home/koogs/vision_calibration.py
```

**Follow the prompts:**
1. Select your camera (0, 2, or 3)
2. Position the 4 markers at grid corners
3. Press ENTER to capture
4. System will detect markers and calculate homography

**If calibration fails:**
- Adjust lighting (even, diffused light works best)
- Make sure all 4 markers are visible
- Check camera settings (not too bright)
- Run again

### Step 4: Test Vision Pickup

```bash
python3 /home/koogs/vision_pickup.py
```

This will:
1. Capture an image
2. Detect trash by color
3. Convert to servo angles
4. Show the pickup sequence

### Step 5: Integrate with Robotic Arm

The vision system creates sequences that work with your existing `robotic_arm_controller.py`.

**Example integration:**
```python
from robotic_arm_controller import RoboticArmController
from vision_pickup import VisionPickup

arm = RoboticArmController(root)
vision = VisionPickup(camera_index=0)

# Auto pickup
vision.auto_pickup_sequence(arm_controller=arm)
```

##  Files Created

| File | Purpose |
|------|---------|
| `vision_calibration.py` | Calibrate camera and grid |
| `vision_pickup.py` | Auto-detect and pickup trash |
| `camera_settings_gui.py` | Adjust camera settings |
| `vision_calibration.json` | Saved calibration data |
| `calibration_capture.jpg` | Reference image from calibration |

## 🔧 Troubleshooting

### Camera not opening
```bash
# List cameras
ls -la /dev/video*

# Test camera
python3 -c "import cv2; cap=cv2.VideoCapture(0); print(cap.read()[1].shape)"
```

### Markers not detected
- Increase **saturation** in camera settings
- Use brighter/more saturated marker colors
- Improve lighting (add LED lamp)
- Make markers larger

### Image too bright
- Reduce **brightness** to -30 or lower
- Reduce **exposure** to -4 or lower
- Add diffuser over camera

### Calibration inaccurate
- Ensure markers are exactly at corners
- Measure actual grid cell size
- Update `GRID_CONFIG` in calibration script

##  Customizing Trash Detection

Edit `TRASH_COLORS` in `vision_pickup.py`:

```python
TRASH_COLORS = {
    'white': {
        'lower': np.array([0, 0, 200]),
        'upper': np.array([20, 20, 255]),
    },
    # Add custom colors:
    'yellow': {
        'lower': np.array([20, 100, 100]),
        'upper': np.array([35, 255, 255]),
    },
}
```

##  Tips

1. **Lighting is critical** - Use consistent, even lighting
2. **High contrast markers** - Bright colors on dark grid (or vice versa)
3. **Fixed camera position** - Don't move camera after calibration
4. **Test with one item first** - Before running full auto-pickup
5. **Save good camera settings** - Copy values from GUI to script

---

**Ready to start?** Run:
```bash
python3 /home/koogs/camera_settings_gui.py  # Adjust settings
python3 /home/koogs/vision_calibration.py   # Calibrate
python3 /home/koogs/vision_pickup.py        # Test pickup
```
