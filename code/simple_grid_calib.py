#!/usr/bin/env python3
"""
Simple Grid Calibration - Click 4 Corners
Fixed version with stable preview and accurate clicks
"""

import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import json
import time
import os
import sys
from pathlib import Path

# Add project root to path for config import
sys.path.insert(0, str(Path(__file__).parent.parent))
import config

# Grid configuration
GRID_ROWS = 4
GRID_COLS = 4
CELL_SIZE_CM = 5
CALIBRATION_FILE = str(config.CALIBRATION_FILE)

CAMERA_SETTINGS = {
    'brightness': -30,
    'contrast': 40,
    'saturation': 0,
}


class SimpleGridCalibrationGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Simple Grid Calibration - Click 4 Corners")
        self.root.geometry("1400x900")

        # Camera
        self.cap = None
        self.is_running = False

        # Corner points (TL, TR, BL, BR)
        self.corners = []
        self.all_points = []
        self.is_calibrated = False

        # Image tracking
        self.current_frame = None
        self.photo_image = None

        # Detection sensitivity controls - MUST BE BEFORE setup_ui()
        self.threshold_var = tk.IntVar(value=35)  # Color threshold
        self.min_area_var = tk.IntVar(value=3000)  # Minimum area
        self.solidity_var = tk.DoubleVar(value=0.5)  # Minimum solidity
        self.kernel_var = tk.IntVar(value=7)  # Kernel size

        # Background reference
        self.empty_grid = None
        self.bg_file = config.BG_FILE_LEGACY
        if os.path.exists(self.bg_file):
            self.empty_grid = cv2.imread(self.bg_file)
            print(f"Loaded existing empty grid reference")

        # Setup UI
        self.setup_ui()
        self.start_camera()
    
    def setup_ui(self):
        """Setup UI"""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Left: Preview
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        preview_frame = ttk.LabelFrame(left_frame, text="Click the 4 Grid Corners in Order", padding="10")
        preview_frame.pack(fill=tk.BOTH, expand=True)
        
        # Canvas for preview
        self.canvas = tk.Canvas(preview_frame, bg='black')
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind('<Button-1>', self.on_click)
        
        # Buttons
        btn_frame = ttk.Frame(preview_frame)
        btn_frame.pack(fill=tk.X, pady=(10, 0))

        self.fs_btn = ttk.Button(btn_frame, text="Fullscreen", command=self.toggle_fullscreen)
        self.fs_btn.pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Reset", command=self.reset).pack(side=tk.LEFT, padx=5)
        self.calc_btn = ttk.Button(btn_frame, text="Calculate Grid", command=self.calculate_grid, state='disabled')
        self.calc_btn.pack(side=tk.LEFT, padx=5)
        self.save_btn = ttk.Button(btn_frame, text="Save Calibration", command=self.save_calibration, state='disabled')
        self.save_btn.pack(side=tk.LEFT, padx=5)
        self.capture_bg_btn = ttk.Button(btn_frame, text="Recapture Empty Grid", command=self.capture_empty_grid)
        self.capture_bg_btn.pack(side=tk.LEFT, padx=5)
        self.detect_btn = ttk.Button(btn_frame, text="Detect Object Cell", command=self.detect_object, state='disabled')
        self.detect_btn.pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Quit", command=self.quit_app).pack(side=tk.LEFT, padx=5)
        
        # Right: Info
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        
        # Status
        status_frame = ttk.LabelFrame(right_frame, text="Status", padding="10")
        status_frame.pack(fill=tk.X)
        
        self.status_label = ttk.Label(status_frame, text="Click corner 1: Top-Left (Red)",
                                     font=('Helvetica', 11, 'bold'), foreground='blue')
        self.status_label.pack(anchor='w')
        
        self.progress_label = ttk.Label(status_frame, text="Progress: 0/4 corners",
                                       font=('Helvetica', 10))
        self.progress_label.pack(anchor='w', pady=(10, 0))
        
        # Instructions
        instr_frame = ttk.LabelFrame(right_frame, text="Instructions", padding="10")
        instr_frame.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        
        instructions = """
1. Click the 4 corners of your grid platform IN ORDER:

   Corner 1: TOP-LEFT     (Red)
   Corner 2: TOP-RIGHT    (Blue)
   Corner 3: BOTTOM-LEFT  (Green)
   Corner 4: BOTTOM-RIGHT (Yellow)

2. Click "Calculate Grid"
   - System calculates all 25 intersection points
   - Draws blue grid overlay

3. Verify overlay aligns with physical grid

4. Click "Save Calibration"

Tips:
- Click precisely on grid corners
- Ensure entire grid is visible
- Good lighting helps
"""
        
        ttk.Label(instr_frame, text=instructions, justify=tk.LEFT,
                 font=('Courier', 8)).pack(anchor='w')

        # Sensitivity Controls (variables defined in __init__)
        sens_frame = ttk.LabelFrame(right_frame, text="Detection Sensitivity", padding="10")
        sens_frame.pack(fill=tk.X, pady=(10, 0))

        # Color threshold
        ttk.Label(sens_frame, text="Color Threshold:").pack(anchor='w')
        threshold_scale = ttk.Scale(sens_frame, from_=10, to=100,
                                    variable=self.threshold_var, orient=tk.HORIZONTAL,
                                    command=lambda v: self.update_sensitivity_labels())
        threshold_scale.pack(fill=tk.X)
        self.threshold_label = ttk.Label(sens_frame, text="35 (higher = more strict)")
        self.threshold_label.pack(anchor='w')

        # Minimum area
        ttk.Label(sens_frame, text="Minimum Area:").pack(anchor='w', pady=(10, 0))
        area_scale = ttk.Scale(sens_frame, from_=1000, to=10000,
                               variable=self.min_area_var, orient=tk.HORIZONTAL,
                               command=lambda v: self.update_sensitivity_labels())
        area_scale.pack(fill=tk.X)
        self.area_label = ttk.Label(sens_frame, text="3000 px (higher = larger objects only)")
        self.area_label.pack(anchor='w')

        # Solidity
        ttk.Label(sens_frame, text="Min Solidity:").pack(anchor='w', pady=(10, 0))
        solidity_scale = ttk.Scale(sens_frame, from_=0.1, to=0.9,
                                   variable=self.solidity_var, orient=tk.HORIZONTAL,
                                   command=lambda v: self.update_sensitivity_labels())
        solidity_scale.pack(fill=tk.X)
        self.solidity_label = ttk.Label(sens_frame, text="0.5 (higher = more solid shapes)")
        self.solidity_label.pack(anchor='w')

        # Test button
        ttk.Button(sens_frame, text="Test Detection Now",
                  command=self.test_detection_live).pack(pady=(10, 0))

    def update_sensitivity_labels(self):
        """Update sensitivity control labels"""
        self.threshold_label.config(text=f"{self.threshold_var.get()} (higher = more strict)")
        self.area_label.config(text=f"{self.min_area_var.get()} px (higher = larger objects only)")
        self.solidity_label.config(text=f"{self.solidity_var.get():.2f} (higher = more solid shapes)")

    def test_detection_live(self):
        """Test detection with current sensitivity settings"""
        messagebox.showinfo("Testing", "Click 'Detect Object Cell' to test with current settings!")
        print(f"\nCurrent sensitivity settings:")
        print(f"  Color Threshold: {self.threshold_var.get()}")
        print(f"  Minimum Area: {self.min_area_var.get()}")
        print(f"  Min Solidity: {self.solidity_var.get():.2f}")
        print(f"  Kernel Size: {self.kernel_var.get()}x{self.kernel_var.get()}\n")
    
    def start_camera(self):
        """Start Z-Star camera (video2 ONLY)"""
        print("Opening Z-Star camera (video2)...")
        self.cap = cv2.VideoCapture(2)

        if not self.cap.isOpened():
            print("Failed to open video2, trying reset...")
            self.cap.release()
            import time
            time.sleep(2)
            self.cap = cv2.VideoCapture(2)

        if not self.cap.isOpened():
            messagebox.showerror("Error", "Cannot open Z-Star USB camera (video2)\n\nPlease check:\n1. USB connection\n2. Camera is not used by another program")
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.is_running = True
        print("Z-Star camera started")
        self.update_preview()
    
    def update_preview(self):
        """Update preview"""
        if not self.is_running or not self.cap:
            return
        
        ret, frame = self.cap.read()
        if not ret:
            self.root.after(30, self.update_preview)
            return
        
        # Apply adjustments
        frame = self.apply_adjustments(frame)
        self.original_height, self.original_width = frame.shape[:2]
        
        # Get canvas size
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        
        if canvas_width < 2 or canvas_height < 2:
            self.root.after(30, self.update_preview)
            return
        
        # Calculate scale
        scale = min(canvas_width / self.original_width, 
                   canvas_height / self.original_height)
        
        new_width = int(self.original_width * scale)
        new_height = int(self.original_height * scale)
        
        if new_width < 1 or new_height < 1:
            self.root.after(30, self.update_preview)
            return
        
        # Resize
        display_frame = cv2.resize(frame, (new_width, new_height))
        
        # Create background
        bg = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)
        
        # Center image
        y_offset = (canvas_height - new_height) // 2
        x_offset = (canvas_width - new_width) // 2
        
        bg[y_offset:y_offset+new_height, x_offset:x_offset+new_width] = display_frame
        
        # Store for click calculation
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.img_width = new_width
        self.img_height = new_height
        
        # Draw corners
        corner_colors = [(0, 0, 255), (255, 0, 0), (0, 255, 0), (0, 255, 255)]
        corner_names = ['1: TL', '2: TR', '3: BL', '4: BR']
        
        for i, (orig_x, orig_y) in enumerate(self.corners):
            disp_x = int(orig_x * (new_width / self.original_width)) + x_offset
            disp_y = int(orig_y * (new_height / self.original_height)) + y_offset
            
            cv2.circle(bg, (disp_x, disp_y), 15, corner_colors[i], -1)
            cv2.circle(bg, (disp_x, disp_y), 20, corner_colors[i], 2)
            cv2.putText(bg, corner_names[i], (disp_x+20, disp_y-20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, corner_colors[i], 2)
        
        # Draw grid if calibrated
        if self.is_calibrated and len(self.all_points) >= 25:
            # Calculate display points for all 25 intersections
            display_points = []
            for orig_x, orig_y in self.all_points:
                disp_x = int(orig_x * (new_width / self.original_width)) + x_offset
                disp_y = int(orig_y * (new_height / self.original_height)) + y_offset
                display_points.append((disp_x, disp_y))

            # Draw horizontal grid lines (5 rows)
            for row in range(GRID_ROWS + 1):
                start_idx = row * (GRID_COLS + 1)
                end_idx = start_idx + GRID_COLS
                if start_idx < len(display_points) and end_idx < len(display_points):
                    pt1 = display_points[start_idx]
                    pt2 = display_points[end_idx]
                    cv2.line(bg, pt1, pt2, (255, 0, 0), 2)  # Blue lines

            # Draw vertical grid lines (5 columns)
            for col in range(GRID_COLS + 1):
                col_points = [display_points[row * (GRID_COLS + 1) + col] for row in range(GRID_ROWS + 1)]
                for i in range(len(col_points) - 1):
                    cv2.line(bg, col_points[i], col_points[i+1], (255, 0, 0), 2)  # Blue lines

            # Draw intersection dots (magenta)
            for disp_x, disp_y in display_points:
                cv2.circle(bg, (disp_x, disp_y), 6, (255, 0, 255), -1)

            # Draw cell labels
            for row in range(GRID_ROWS):
                for col in range(GRID_COLS):
                    idx1 = row * (GRID_COLS + 1) + col
                    idx2 = idx1 + 1
                    idx3 = idx1 + (GRID_COLS + 1)
                    idx4 = idx3 + 1

                    if idx4 < len(display_points):
                        pt1 = display_points[idx1]
                        pt2 = display_points[idx2]
                        pt3 = display_points[idx3]
                        center_x = (pt1[0] + pt2[0]) // 2
                        center_y = (pt1[1] + pt3[1]) // 2

                        cell_name = f"{chr(ord('A') + row)}{col + 1}"
                        cv2.putText(bg, cell_name, (center_x - 15, center_y + 5),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # Convert to RGB
        bg = cv2.cvtColor(bg, cv2.COLOR_BGR2RGB)
        
        # Create PhotoImage
        img = Image.fromarray(bg)
        self.photo_image = ImageTk.PhotoImage(image=img)
        
        # Update canvas
        self.canvas.delete("all")
        self.canvas.create_image(0, 0, anchor='nw', image=self.photo_image)
        
        self.root.after(30, self.update_preview)
    
    def on_click(self, event):
        """Handle canvas click"""
        if len(self.corners) >= 4:
            return
        
        # Account for offsets
        click_x = event.x - self.x_offset
        click_y = event.y - self.y_offset
        
        # Check if within image
        if click_x < 0 or click_x > self.img_width or click_y < 0 or click_y > self.img_height:
            print(f"Click outside image: ({event.x}, {event.y}), offset: ({self.x_offset}, {self.y_offset})")
            return
        
        # Scale to original
        orig_x = int(click_x * (self.original_width / self.img_width))
        orig_y = int(click_y * (self.original_height / self.img_height))
        
        print(f"Click registered: ({orig_x}, {orig_y})")
        self.corners.append((orig_x, orig_y))
        
        corner_names = ['Top-Left (Red)', 'Top-Right (Blue)', 'Bottom-Left (Green)', 'Bottom-Right (Yellow)']
        
        if len(self.corners) < 4:
            next_corner = len(self.corners) + 1
            self.status_label.config(text=f"Click corner {next_corner}: {corner_names[len(self.corners)]}")
            self.progress_label.config(text=f"Progress: {len(self.corners)}/4 corners")
        else:
            self.status_label.config(text="All 4 corners clicked! Click 'Calculate Grid'")
            self.progress_label.config(text="Progress: 4/4 corners - Complete!")
            self.calc_btn.config(state='normal')
    
    def apply_adjustments(self, frame):
        """Apply camera adjustments"""
        b = CAMERA_SETTINGS.get('brightness', 0)
        c = CAMERA_SETTINGS.get('contrast', 0)
        s = CAMERA_SETTINGS.get('saturation', 0)
        
        if b: frame = cv2.convertScaleAbs(frame, alpha=1, beta=b)
        if c: frame = cv2.convertScaleAbs(frame, alpha=1+(c/100), beta=0)
        if s:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            h, sv = cv2.split(hsv)
            sv_new = cv2.multiply(sv, 1+(s/100))
            sv_new = np.clip(sv_new, 0, 255).astype(np.uint8)
            hsv = cv2.merge([h, sv_new])
            frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        return frame
    
    def toggle_fullscreen(self):
        """Toggle fullscreen"""
        self.is_fullscreen = not self.is_fullscreen
        if self.is_fullscreen:
            self.root.attributes('-fullscreen', True)
            self.fs_btn.config(text="Exit Fullscreen")
        else:
            self.root.attributes('-fullscreen', False)
            self.root.geometry("1400x900")
            self.fs_btn.config(text="Fullscreen")
    
    def reset(self):
        """Reset all corners and grid"""
        self.corners = []
        self.all_points = []
        self.is_calibrated = False
        self.status_label.config(text="Click corner 1: Top-Left (Red)")
        self.progress_label.config(text="Progress: 0/4 corners")
        self.calc_btn.config(state='disabled')
        self.save_btn.config(state='disabled')
        print("Reset - Click 4 corners to start over")
    
    def calculate_grid(self):
        """Calculate grid from 4 corners AND capture empty grid reference"""
        if len(self.corners) < 4:
            return

        # First, capture empty grid reference BEFORE calculating
        ret, frame = self.cap.read()
        if ret:
            # Save as empty grid reference
            self.empty_grid = frame.copy()
            cv2.imwrite(self.bg_file, self.empty_grid)
            print(f"✓ Empty grid captured and saved to: {self.bg_file}")
        else:
            print("Warning: Could not capture empty grid reference")

        tl = np.array(self.corners[0], dtype=np.float32)
        tr = np.array(self.corners[1], dtype=np.float32)
        bl = np.array(self.corners[2], dtype=np.float32)
        br = np.array(self.corners[3], dtype=np.float32)

        self.all_points = []
        for row in range(GRID_ROWS + 1):
            for col in range(GRID_COLS + 1):
                t = col / GRID_COLS
                u = row / GRID_ROWS
                top = tl + (tr - tl) * t
                bottom = bl + (br - bl) * t
                point = top + (bottom - top) * u
                self.all_points.append((int(point[0]), int(point[1])))

        self.is_calibrated = True
        self.status_label.config(text="Grid calculated! Verify overlay, then save")
        self.save_btn.config(state='normal')
        self.detect_btn.config(state='normal')  # Enable detection button

        print(f"Calculated {len(self.all_points)} points")
        messagebox.showinfo("Success",
            f"✓ Grid calculated!\n\n"
            f"{len(self.all_points)} points generated\n\n"
            f"✓ Empty grid reference captured!\n\n"
            f"Verify blue grid LINES align, then save.\n"
            f"Now place objects and click 'Detect Object Cell'")
    
    def capture_empty_grid(self):
        """Capture and save empty grid as background reference"""
        ret, frame = self.cap.read()
        if not ret:
            messagebox.showerror("Error", "Failed to capture frame")
            return

        # Save reference
        self.empty_grid = frame.copy()
        cv2.imwrite(self.bg_file, self.empty_grid)

        print(f"✓ Empty grid captured and saved to: {self.bg_file}")
        messagebox.showinfo("Success",
            f"✓ Empty grid captured!\n\n"
            f"Saved to: {self.bg_file}\n\n"
            f"Now place objects on the grid and click 'Detect Object Cell'")

    def detect_object(self):
        """Detect object using background subtraction with lighting compensation"""
        if not self.is_calibrated:
            messagebox.showwarning("Warning", "Calculate grid first!")
            return

        ret, frame = self.cap.read()
        if not ret:
            messagebox.showerror("Error", "Failed to capture frame")
            return

        frame_area = frame.shape[0] * frame.shape[1]

        # Check if we have background reference
        if self.empty_grid is None:
            messagebox.showwarning("No Reference",
                "No empty grid reference!\n\n"
                "Please click 'Recapture Empty Grid' first\n"
                "with no objects on the grid.")
            return

        # Convert to LAB color space (better for lighting changes)
        ref_lab = cv2.cvtColor(self.empty_grid, cv2.COLOR_BGR2LAB)
        curr_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

        # Extract L (lightness), A, B channels
        ref_l, ref_a, ref_b = cv2.split(ref_lab)
        curr_l, curr_a, curr_b = cv2.split(curr_lab)

        # Method 1: Color difference (A and B channels - ignores lighting)
        diff_a = cv2.absdiff(ref_a, curr_a)
        diff_b = cv2.absdiff(ref_b, curr_b)
        color_diff = cv2.addWeighted(diff_a, 0.5, diff_b, 0.5, 0)
        _, color_thresh = cv2.threshold(color_diff, 35, 255, cv2.THRESH_BINARY)

        # Method 2: Lightness difference with higher threshold (ignores subtle lighting)
        diff_l = cv2.absdiff(ref_l, curr_l)
        _, light_thresh = cv2.threshold(diff_l, 50, 255, cv2.THRESH_BINARY)

        # Method 3: Structural similarity (only solid objects)
        diff = cv2.absdiff(self.empty_grid, frame)
        diff_gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        _, diff_thresh = cv2.threshold(diff_gray, 40, 255, cv2.THRESH_BINARY)

        # Combine: Color difference OR (Light AND Diff)
        # This ignores pure lighting changes (no color change)
        combined = cv2.bitwise_or(color_thresh,
                         cv2.bitwise_and(light_thresh, diff_thresh))

        # Aggressive noise removal
        kernel = np.ones((7, 7), np.uint8)  # Larger kernel
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel, iterations=3)
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel, iterations=2)

        # Remove small blobs (lighting artifacts are usually small)
        combined = cv2.erode(combined, kernel, iterations=1)
        combined = cv2.dilate(combined, kernel, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter and score contours
        candidates = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = w / float(h) if h > 0 else 0
            cx, cy = x + w//2, y + h//2

            # STRICHER filtering for lighting
            # Skip background (>20% of frame)
            if area > frame_area * 0.2:
                continue

            # Skip noise (<3000 pixels - lighting artifacts are small)
            if area < 3000:
                continue

            # Skip very thin or very wide (lighting streaks)
            if aspect_ratio < 0.3 or aspect_ratio > 4.0:
                continue

            # Solidity check (real objects are more solid than lighting)
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            solidity = float(area) / hull_area if hull_area > 0 else 0
            if solidity < 0.5:  # Real objects have solidity > 0.5
                continue

            # Score: prefer larger, more central, more solid objects
            center_dist = np.sqrt((cx - frame.shape[1]/2)**2 + (cy - frame.shape[0]/2)**2)
            score = area * solidity * (1000 / (center_dist + 1))

            candidates.append({
                'contour': cnt,
                'area': area,
                'cx': cx, 'cy': cy,
                'x': x, 'y': y, 'w': w, 'h': h,
                'score': score,
                'solidity': solidity,
                'method': 'background_color'
            })

        # Sort by score
        candidates.sort(key=lambda c: c['score'], reverse=True)

        # Save debug visualization
        debug = frame.copy()
        color_thresh_vis = cv2.cvtColor(color_thresh, cv2.COLOR_GRAY2BGR)
        combined_vis = cv2.cvtColor(combined, cv2.COLOR_GRAY2BGR)

        # Resize for display
        color_thresh_vis = cv2.resize(color_thresh_vis, (200, 150))
        combined_vis = cv2.resize(combined_vis, (200, 150))

        # Draw on main frame
        cv2.putText(debug, f"Objects: {len(candidates)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.putText(debug, f"Method: Color+Background Subtraction", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(debug, f"(Ignores lighting changes)", (10, 85),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        for i, obj in enumerate(candidates[:5]):
            color = (0, 255, 0) if i == 0 else (0, 255, 255)
            cv2.rectangle(debug, (obj['x'], obj['y']), (obj['x']+obj['w'], obj['y']+obj['h']), color, 3)
            cv2.circle(debug, (obj['cx'], obj['cy']), 10, color, -1)
            label = f"#{i+1} A:{obj['area']:.0f} S:{obj['solidity']:.2f}"
            cv2.putText(debug, label, (obj['x'], obj['y']-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Add debug insets
        h, w, _ = debug.shape
        debug[h-150:h, w-410:w-210] = color_thresh_vis
        debug[h-150:h, w-200:w] = combined_vis

        cv2.putText(debug, "Color Diff", (w-400, h-160),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        cv2.putText(debug, "Final", (w-190, h-160),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        cv2.imwrite(str(config.PROJECT_ROOT / 'object_detection_color.jpg'), debug)

        if not candidates:
            messagebox.showinfo("No Object",
                "No object detected!\n\n"
                "Tips:\n"
                "• Make sure object has different color from background\n"
                "• Ensure object is large enough (>3000 pixels)\n"
                "• Try recapturing empty grid if needed")
            return

        # Get best candidate
        best = candidates[0]

        # Find cell
        cell_name = self.find_cell_for_point(best['cx'], best['cy'])

        # Show result
        if cell_name:
            msg = f"✓ Object detected!\n\n"
            msg += f"Grid cell: {cell_name}\n\n"
            msg += f"Position: ({best['cx']}, {best['cy']})\n"
            msg += f"Size: {best['w']}x{best['h']} pixels\n"
            msg += f"Area: {best['area']:.0f} pixels\n"
            msg += f"Solidity: {best['solidity']:.2f}\n"
            msg += f"Confidence: {best['score']:.0f}\n\n"
            msg += f"Method: Color+Background Subtraction\n"
            msg += f"(Ignores lighting changes)\n\n"
            msg += f"Debug saved:\nobject_detection_color.jpg"

            # Print to console
            print(f"\n{'='*60}")
            print(f"OBJECT DETECTED (Color+Background)")
            print(f"{'='*60}")
            print(f"Cell: {cell_name}")
            print(f"Position: ({best['cx']}, {best['cy']})")
            print(f"Size: {best['w']}x{best['h']}")
            print(f"Area: {best['area']:.0f}")
            print(f"Solidity: {best['solidity']:.2f}")
            print(f"Score: {best['score']:.0f}")
            if len(candidates) > 1:
                print(f"\nOther candidates (likely noise/lighting):")
                for i, c in enumerate(candidates[1:5], 2):
                    print(f"  #{i}: Area:{c['area']:.0f} Solidity:{c['solidity']:.2f}")
            print(f"{'='*60}\n")

            messagebox.showinfo("Object Detected", msg)
        else:
            msg = f"Object found at ({best['cx']}, {best['cy']})\n\n"
            msg += f"Size: {best['w']}x{best['h']} pixels\n\n"
            msg += f"Outside grid area"
            messagebox.showinfo("Object Detected", msg)

    def find_cell_for_point(self, pixel_x, pixel_y):
        """Find which grid cell contains the given pixel coordinates"""
        if len(self.all_points) < 25:
            return None

        # Convert pixel coordinates to real-world coordinates using homography
        pixel_point = np.array([[pixel_x, pixel_y]], dtype=np.float32)
        H = np.array(self.all_points)  # We need the actual homography matrix

        # For simplicity, use point-in-polygon test on each cell
        # Get display coordinates for all points
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()

        scale_x = canvas_width / self.original_width
        scale_y = canvas_height / self.original_height

        display_x = pixel_x * scale_x
        display_y = pixel_y * scale_y

        # Check each cell
        for row in range(GRID_ROWS):
            for col in range(GRID_COLS):
                # Get 4 corners of this cell
                idx1 = row * (GRID_COLS + 1) + col
                idx2 = idx1 + 1
                idx3 = idx1 + (GRID_COLS + 1)
                idx4 = idx3 + 1

                if idx4 >= len(self.all_points):
                    continue

                # Get original coordinates
                x1, y1 = self.all_points[idx1]
                x2, y2 = self.all_points[idx2]
                x3, y3 = self.all_points[idx3]
                x4, y4 = self.all_points[idx4]

                # Simple bounding box check (approximate)
                min_x = min(x1, x2, x3, x4)
                max_x = max(x1, x2, x3, x4)
                min_y = min(y1, y2, y3, y4)
                max_y = max(y1, y2, y3, y4)

                if min_x <= pixel_x <= max_x and min_y <= pixel_y <= max_y:
                    cell_name = f"{chr(ord('A') + row)}{col + 1}"
                    return cell_name

        return None

    def save_calibration(self):
        """Save calibration"""
        if len(self.all_points) < 4:
            messagebox.showwarning("Warning", "Calculate grid first!")
            return

        print(f"\nSaving {len(self.all_points)} points...")
        real_points = []
        for row in range(GRID_ROWS + 1):
            for col in range(GRID_COLS + 1):
                real_points.append([col * CELL_SIZE_CM, row * CELL_SIZE_CM])
        real_points = np.array(real_points, dtype=np.float32)
        
        H, _ = cv2.findHomography(pixel_points, real_points)
        
        print("Homography:")
        for row in H:
            print(f"  [{row[0]:10.6f}  {row[1]:10.6f}  {row[2]:10.6f}]")
        
        grid_dict = {}
        for i, (x, y) in enumerate(self.all_points):
            row = i // (GRID_COLS + 1)
            col = i % (GRID_COLS + 1)
            grid_dict[f"{chr(ord('A')+row)}{col+1}"] = {'x': int(x), 'y': int(y)}
        
        data = {
            'homography': H.tolist(),
            'grid_intersections': grid_dict,
            'grid_config': {
                'rows': GRID_ROWS,
                'cols': GRID_COLS,
                'cell_size_cm': CELL_SIZE_CM,
            },
            'timestamp': time.time(),
            'calibration_method': 'simple_4_corner',
        }
        
        with open(CALIBRATION_FILE, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"\nSAVED: {CALIBRATION_FILE}")
        self.status_label.config(text="Calibration saved!")
        messagebox.showinfo("Success", f"Calibration saved!\n\nFile: {CALIBRATION_FILE}")
    
    def quit_app(self):
        """Quit"""
        if self.cap:
            self.cap.release()
        self.root.quit()
        self.root.destroy()


def main():
    print("="*60)
    print("SIMPLE GRID CALIBRATION")
    print("="*60)
    print("\nGrid: 4x4 cells (5cm each)")
    print("Just click the 4 corners!")
    print("="*60)
    
    root = tk.Tk()
    app = SimpleGridCalibrationGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
