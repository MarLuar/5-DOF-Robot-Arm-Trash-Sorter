

## 1. Auto-Offset System (MAJOR FEATURE)

### What It Does
Automatically calculates and applies base angle corrections based on detected object position. Compensates for trash/debris between grid cells that causes the object to be slightly off-center.

### Features Added
- **Adjustable Ratio Control**: Set sensitivity (default: for every 2° of detected error, adjust base by 1°)
- **Live Suggestion Display**: Shows real-time offset suggestions with confidence percentage
- **Sample Averaging**: Collects 10 samples and averages them for stable, accurate offsets
- **Auto-Apply to Sequences**: Automatically applies offset to steps 3 and 4 (pickup and lift steps) only
- **Offset Range**: ±20° maximum correction (increased from ±10°)

### How It Works
1. Camera detects object position in cell
2. Compares detected center to expected cell center
3. Calculates pixel offset
4. Converts to angular offset using ratio setting
5. Averages last 10 samples for stability
6. Applies offset to base angle during sequence execution

### UI Components
- Auto-Offset Ratio control (Manual Control tab & Grid Calibration tab)
- Auto-Offset Suggestion panel with Apply/Analyze buttons
- Confidence display showing sample count and quality

---

## 2. Camera System Overhaul

### What It Does
Complete redesign of camera handling for better performance, reliability, and user control.

### Features Added
- **Camera Device Selector**: Dropdown to select camera device (0-7)
- **Find/Test Buttons**: Detect available cameras and test connectivity
- **Save/Load Camera Setting**: Persists camera selection across restarts
- **Camera Status Display**: Shows current camera configuration status
- **Direct Device Path Support**: Falls back to /dev/videoX paths if index fails
- **Multiple Fallback Methods**: Tries index, direct path, and scans all devices

### Performance Improvements
- Camera runs in separate thread (configurable FPS: 10 → 2 → 1 FPS)
- Cached canvas size prevents blocking
- Proper thread termination with stop flags

### UI Components (Manual Control Tab)
- Camera Configuration section with device selector
- Find/Test buttons for troubleshooting
- Save/Load Camera Setting buttons
- "Set Camera for Calibration" button
- Status label showing current configuration

---

## 3. Object Detection Flickering Fixes

### What It Does
Eliminates visual flickering and unstable detection readings in the camera preview.

### Problems Solved
- Detection box jumping between cells
- Rapid on/off detection flickering
- False positives from large objects
- Detection state resetting too quickly

### Solutions Implemented
- **Cell Hysteresis**: Must detect same cell 3 consecutive times before switching
- **Detection Persistence**: Keeps detection active for 8 seconds (increased from 3 seconds)
- **Camera Thread Drawing**: Detection box drawn in camera thread (not main thread)
- **Max Area Filter**: Ignores objects larger than 30,000 pixels (prevents false detections)
- **Solidity Calculation**: Filters based on shape solidity
- **Sample Clearing**: Clears offset samples when new cell detected

### UI Improvements
- Detection status display at top of log panel
- Detection info overlay on camera preview showing:
  - Cell name
  - Object area in pixels
  - Solidity score
  - Current threshold value

---

## 4. Auto-Pickup System

### What It Does
Automatically executes pickup sequence when object is detected, without manual confirmation.

### Features Added
- **4-Second Confirmation Delay**: Object must be present for 4 seconds before pickup triggers (prevents false triggers)
- **Live Offset Display**: Shows current offset value during confirmation countdown
- **Averaged Offset Usage**: Uses averaged offset from samples (not fresh calculation)
- **Background Execution**: Runs pickup sequence in background thread
- **Thread Cleanup**: Proper thread termination and callback cancellation

### How It Works
1. User enables "Auto Pickup" checkbox
2. System continuously monitors for objects
3. When object detected, starts 4-second countdown
4. Countdown displays: "Auto: A1 (3.2s) offset:-5°"
5. After 4 seconds, automatically executes sequence for that cell
6. Recaptures empty grid after successful pickup
7. Resets and ready for next object

### UI Components
- "Auto Pickup" checkbox (Grid Calibration tab)
- Status display during countdown
- Log messages showing confirmation progress

---

## 5. Step 3 & 4 Base Value Display

### What It Does
Shows real-time base angles being sent to Arduino, including offset calculations.

### Display Format
```
Step 3 (Pickup): 100° (base: 110° + offset: -10°)
Step 4 (Lift): 100° (base: 110° + offset: -10°)
```

### What It Shows
- **Final angle**: The actual angle being sent to Arduino
- **Base angle**: Original angle from sequence
- **Offset**: Auto-offset value applied (if enabled)

### Benefits
- Verify offset is being applied correctly
- Debug auto-offset calculation issues
- Confirm Arduino is receiving correct values

### UI Location
Grid Calibration tab, below Camera Settings section

---

## 6. Thread & Performance Fixes

### Problems Solved
- Sequence lag on 4th playback
- Arduino serial buffer buildup causing delays
- Event queue buildup from pending callbacks
- Thread termination issues

### Solutions Implemented
- **Removed Arduino Queue**: Eliminated command queue that was causing lag
- **Callback Tracking**: All `root.after()` callbacks tracked and can be cancelled
- **Serial Buffer Management**: Added `flush()` and `reset_input_buffer()` calls
- **Thread Stop Flags**: Proper thread termination with dedicated stop flags
- **Thread Reference Cleanup**: Clean up thread references after completion

### Performance Impact
- Eliminated sequence playback lag
- Prevented Arduino communication delays
- Reduced memory usage from callback buildup
- Improved overall system responsiveness

---

## 7. Copy/Paste Functionality

### What It Does
Allows copying and pasting servo angles between different parts of the application.

### Features Added
- **Copy Current Angles**: Copy current servo input box values
- **Paste to Manual Control**: Paste copied angles to servo input boxes
- **Copy Step**: Copy individual sequence step to clipboard
- **Copy All Steps**: Copy entire sequence to clipboard
- **Paste Step**: Paste step from clipboard to sequence
- **Paste to Insert Dialog**: Paste when inserting new step
- **Paste to Editor**: Paste step directly to sequence editor

### Clipboard Features
- Stores multiple steps (array of angle arrays)
- Persists across copy/paste operations
- Works between Manual Control and Sequences tabs

### UI Components
- "Copy Current Angles" / "Paste to Manual Control" buttons
- "Insert Step" button with paste option
- Right-click context menu: Copy Step, Copy All Steps, Paste Step
- "Paste Step to Editor" button in Sequences tab

---

## 8. UI Cleanup & Reorganization

### Manual Control Tab
- Moved System Log to left side (was at bottom)
- Added Detection Status display at top of log panel
- Moved Presets section to right side
- Added "Send All (Simultaneous)" button for sending all servos at once
- Removed emojis from most labels (cleaner appearance)

### Grid Calibration Tab
- Made right panel **scrollable** (width: 340px → 440px)
- Compact layout with smaller fonts (7pt instead of 9pt)
- Removed Instructions section (users familiar with system)
- Corner status moved to right panel
- All controls accessible via scrollbar

### Sequences Tab
- Shows both original cells (A1-D4) and _NON variants for non-biodegradable
- Added separator between biodegradable/non-biodegradable sections
- Changed checkmark (✓) to "[OK]" marker for better visibility
- Added "Clipboard" section for paste operations

---

## 9. Grid & Sequence Changes

### Grid Support
- Consistent 4×4 grid (16 cells): A1 through D4
- Support for _NON sequence variants (non-biodegradable with base=0°)
- Separate sections for biodegradable (base=180°) and non-biodegradable (base=0°)

### Legacy Sequences
- Basic Pick & Place
- LEFT sequence
- RIGHT sequence
- All loaded automatically on startup

---

## 10. Logging & Debug Improvements

### Added Features
- Manual event logging button for debugging
- Main thread ID tracking for thread safety verification
- Camera frame timing logs (detects slow frames)
- Structured log prefixes:
  - `[AUTO]` - Auto-pickup events
  - `[DEBUG]` - Debug information
  - `[Timer]` - Timing information
  - `[AUTO-OFFSET]` - Offset sampling events

### Log Cleanup
- Removed redundant detection logs
- Removed emoji prefixes from most logs
- Added detection count logging ("Found 1 object(s): ['A1']")
- Added offset calculation logging with sample details

---

## Configuration Files

### New Files Created
- `calibration/camera_config.json` - Stores camera device index and timestamp
- `calibration/servo_presets.json` - Stores user-modified servo presets

### Existing Files Enhanced
- `sequences/cell_sequences.json` - Now supports _NON variants
- `empty_grid_reference.jpg` - Recaptured after each pickup

---

## Performance Metrics

### Before Changes
- Camera: Blocking UI thread
- Detection: Flickering, unstable
- Sequences: Lag on 4th playback
- Arduino: Buffer buildup causing delays
- Offset: Manual calculation only

### After Changes
- Camera: Non-blocking, runs in separate thread
- Detection: Stable, 8-second persistence, hysteresis
- Sequences: No lag, proper thread cleanup
- Arduino: No buffer buildup, flush after each command
- Offset: Automatic calculation with ±20° range

---

## User Experience Improvements

1. **Faster Setup**: Camera auto-detects and saves setting
2. **More Reliable**: Detection doesn't flicker or jump between cells
3. **Better Visibility**: Real-time offset display shows what's being sent
4. **Automation**: Auto-pickup reduces manual intervention
5. **Flexibility**: Copy/paste speeds up sequence creation
6. **Stability**: Thread fixes prevent lag and crashes
7. **Accuracy**: Auto-offset compensates for grid misalignment

---

## Technical Debt Reduced

- Removed unused Arduino queue system
- Consolidated detection logic (removed duplicates)
- Standardized thread termination patterns
- Added proper error handling throughout
- Improved code organization and comments

---

