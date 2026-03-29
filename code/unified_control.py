#!/usr/bin/env python3
"""
5-DOF Robotic Arm - Unified Control System v1.00.02
Integrates:
- Manual servo control
- Grid calibration with object detection
- Sequence assignment per cell
- Auto-execution when object detected
- Comprehensive logging
"""

import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from PIL import Image, ImageTk
import serial
import serial.tools.list_ports
import threading
import json
import time
import os
from datetime import datetime

# Configuration
BAUD_RATE = 115200
DEFAULT_SPEED = 15
MIN_ANGLE = 0
MAX_ANGLE = 180

# Servo safe ranges (avoid problematic angles)
BASE_SAFE_MIN = 0
BASE_SAFE_MAX = 105  # Avoid 110-120° dead zone
BASE_WARNING_MIN = 100
BASE_WARNING_MAX = 125

# Default preset positions (user-modifiable)
DEFAULT_REST = [80, 70, 50, 125, 0]  # Base:80, Shoulder:70, Elbow:50, Wrist:125, Gripper:0
DEFAULT_PICKUP = [80, 74, 50, 5, 140]

# Legacy preset positions from original robotic_arm_controller.py
LEGACY_PRESETS = {
    'Rest': DEFAULT_REST.copy(),  # User-modifiable
    'Pickup': DEFAULT_PICKUP.copy(),  # User-modifiable
    'LEFT DROP': [170, 100, 70, 30, 140],
    'DOWN': [80, 60, 30, 30, 0],
    'GRAB': [80, 74, 50, 5, 0],
    'left': [170, 84, 60, 35, 0],
    'ENGAGE': [80, 84, 60, 35, 0],
    'RIGHT': [0, 84, 60, 35, 0],
    'RIGHT DROP': [0, 84, 60, 15, 140],
    'UP': [80, 124, 10, 25, 0],
}

# Legacy sequences from original robotic_arm_controller.py
LEGACY_SEQUENCES = {
    'Basic Pick & Place': [
        ['Pickup', 1500],
        ['Rest', 1000]
    ],
    'LEFT': [
        ['Rest', 1000],
        ['Pickup', 1000],
        ['GRAB', 1000],
        ['ENGAGE', 1000],
        ['left', 1000],
        ['LEFT DROP', 1000],
        ['left', 1000],
        ['Rest', 1000]
    ],
    'RIGHT': [
        ['Rest', 1000],
        ['Pickup', 1000],
        ['GRAB', 1000],
        ['ENGAGE', 1000],
        ['RIGHT', 1000],
        ['RIGHT DROP', 1000],
        ['RIGHT', 1000],
        ['Rest', 1000]
    ]
}
DEFAULT_REST = [150, 0, 70, 70, 140]
DEFAULT_PICKUP = [80, 100, 20, 0, 0]

JOINT_NAMES = ["Base", "Shoulder", "Elbow", "Wrist", "Gripper"]
CELL_NAMES = [f"{chr(ord('A')+row)}{col+1}" for row in range(4) for col in range(4)]

# File paths
CALIBRATION_FILE = '/home/koogs/Documents/5DOF_Robotic_Arm_Vision/calibration/vision_calibration.json'
SEQUENCES_FILE = '/home/koogs/Documents/5DOF_Robotic_Arm_Vision/sequences/cell_sequences.json'
PRESETS_FILE = '/home/koogs/Documents/5DOF_Robotic_Arm_Vision/calibration/servo_presets.json'
BG_FILE = '/home/koogs/empty_grid_reference.jpg'


class UnifiedControlSystem:
    def __init__(self, root):
        self.root = root
        self.root.title("5-DOF Robotic Arm - Unified Control System v1.00.02")
        self.root.geometry("1400x900")
        
        # Serial connection
        self.serial_conn = None
        self.is_connected = False
        
        # Camera
        self.cap = None
        self.is_running = False
        self.empty_grid = None
        
        # Calibration
        self.all_points = []
        self.is_calibrated = False
        self.corners = []
        
        # Sequences
        self.sequences = {}
        self.current_cell = None
        self.load_sequences()
        
        # Detection settings
        self.threshold_var = tk.IntVar(value=35)
        self.min_area_var = tk.IntVar(value=3000)
        self.solidity_var = tk.DoubleVar(value=0.5)

        # Auto-detection
        self.auto_detect_enabled = False
        self.detection_count = 0
        self.last_cell = "N/A"

        # Track detected objects for pickup
        self.detected_objects = []  # List of detected objects with cell info
        self.current_detection_cell = None  # Cell with detected object

        # Detection state persistence (prevents flickering)
        self.last_detection_time = 0
        self.detection_timeout = 3.0  # Keep detection for 3 seconds
        self.last_detected_cell = None

        # Performance optimizations
        self.serial_lock = threading.Lock()  # Thread-safe serial access
        self.frame_count = 0  # For frame skipping
        self.camera_running = False  # Camera thread control
        self.canvas_width = 640  # Cache canvas size
        self.canvas_height = 480  # Cache canvas size

        # Sequence control flags
        self._stop_sequence_flag = False
        self._current_sequence_thread = None
        self._current_pickup_thread = None

        # Debug: Track thread activity
        self.main_thread_id = threading.current_thread().ident
        self.last_command_time = 0

        # Setup UI
        self.setup_ui()
        self.refresh_ports()
        self.load_calibration()
        self.load_presets()  # Load user-modified presets

        # Start logging
        self.log("System initialized - v1.00.02")
        self.log(f"🧵 Main thread: {threading.current_thread().name}")

        # Load sequences (after UI is ready)
        self.load_sequences()

        # Start camera thread
        self.start_camera_thread()

        # Update canvas size cache periodically (from main thread)
        self.root.after(1000, self.update_canvas_size_cache)
        self.root.after(5000, self.update_canvas_size_cache)  # Update again after 5s

        # Auto-go to rest position after short delay (when connected)
        self.root.after(2000, self.auto_go_to_rest_on_startup)
    
    def setup_ui(self):
        """Setup main UI with notebook tabs"""
        main_frame = ttk.Frame(self.root, padding="5")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Top bar with floating log button
        top_bar = ttk.Frame(main_frame)
        top_bar.pack(fill=tk.X, pady=(0, 5))

        ttk.Label(top_bar, text="5-DOF Robotic Arm - Unified Control System v1.00.02",
                 font=('Helvetica', 12, 'bold')).pack(side=tk.LEFT)

        ttk.Button(top_bar, text="🪟 Open Floating Log Window",
                  command=self.open_floating_log).pack(side=tk.RIGHT, padx=5)

        # Analysis mode button (for debugging)
        ttk.Button(top_bar, text="🔍 Log Event",
                  command=self.log_user_event).pack(side=tk.RIGHT, padx=5)

        # Create notebook for tabs
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # Tab 1: Manual Control
        self.manual_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.manual_tab, text="Manual Control")
        self.setup_manual_tab()
        
        # Tab 2: Grid Calibration
        self.calib_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.calib_tab, text="Grid Calibration")
        self.setup_calibration_tab()
        
        # Tab 3: Sequences
        self.seq_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.seq_tab, text="Cell Sequences")
        self.setup_sequences_tab()

        # Log panel (shared across all tabs)
        self.setup_log_panel(main_frame)
    
    def setup_manual_tab(self):
        """Setup manual control tab"""
        # Left: Servo controls
        left_frame = ttk.LabelFrame(self.manual_tab, text="Servo Control", padding="10")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.input_boxes = []
        for i, name in enumerate(JOINT_NAMES):
            frame = ttk.Frame(left_frame)
            frame.pack(fill=tk.X, pady=3)

            ttk.Label(frame, text=name, width=10).pack(side=tk.LEFT, padx=5)

            var = tk.StringVar(value=str(DEFAULT_REST[i]))
            entry = ttk.Entry(frame, textvariable=var, width=6, justify='center')
            entry.pack(side=tk.LEFT, padx=5)
            entry.bind('<Return>', lambda e, idx=i: self.send_servo(idx))
            self.input_boxes.append(var)

            # Add warning for base servo
            if i == 0:  # Base
                ttk.Label(frame, text="(⚠ 110-120° risky)", foreground='orange', font=('Helvetica', 8)).pack(side=tk.LEFT, padx=3)

            btn_frame = ttk.Frame(frame)
            btn_frame.pack(side=tk.LEFT)
            ttk.Button(btn_frame, text="-10", width=3,
                      command=lambda idx=i, d=-10: self.adjust_servo(idx, d)).pack(side=tk.LEFT, padx=1)
            ttk.Button(btn_frame, text="-1", width=3,
                      command=lambda idx=i, d=-1: self.adjust_servo(idx, d)).pack(side=tk.LEFT, padx=1)
            ttk.Button(btn_frame, text="+1", width=3,
                      command=lambda idx=i, d=1: self.adjust_servo(idx, d)).pack(side=tk.LEFT, padx=1)
            ttk.Button(btn_frame, text="+10", width=3,
                      command=lambda idx=i, d=10: self.adjust_servo(idx, d)).pack(side=tk.LEFT, padx=1)
            ttk.Button(frame, text="Send", width=5,
                      command=lambda idx=i: self.send_servo(idx)).pack(side=tk.LEFT, padx=5)
        
        # Presets
        preset_frame = ttk.LabelFrame(left_frame, text="Quick Presets", padding="10")
        preset_frame.pack(fill=tk.X, pady=10)

        ttk.Button(preset_frame, text="Rest Position", command=self.go_to_rest).pack(side=tk.LEFT, padx=5)
        ttk.Button(preset_frame, text="Pickup Position", command=self.go_to_pickup).pack(side=tk.LEFT, padx=5)
        ttk.Button(preset_frame, text="📤 Send All (Simultaneous)", command=self.send_all_servos).pack(side=tk.LEFT, padx=5)
        
        # Right: Connection, Presets & Speed
        right_frame = ttk.Frame(self.manual_tab)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)

        # Connection
        conn_frame = ttk.LabelFrame(right_frame, text="Serial Connection", padding="5")
        conn_frame.pack(fill=tk.X, pady=5)

        port_frame = ttk.Frame(conn_frame)
        port_frame.pack(fill=tk.X)

        ttk.Label(port_frame, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(port_frame, textvariable=self.port_var, width=20)
        self.port_combo.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)

        self.connect_btn = ttk.Button(port_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=5)

        self.refresh_btn = ttk.Button(port_frame, text="Refresh", command=self.refresh_ports)
        self.refresh_btn.pack(side=tk.LEFT, padx=5)

        self.recover_btn = ttk.Button(port_frame, text="🔄 Recover", command=self.recover_connection)
        self.recover_btn.pack(side=tk.LEFT, padx=5)

        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected", foreground='red')
        self.status_label.pack(pady=5)

        # Presets
        preset_frame = ttk.LabelFrame(right_frame, text="Quick Presets", padding="5")
        preset_frame.pack(fill=tk.X, pady=5)

        ttk.Button(preset_frame, text="Load Rest", command=self.go_to_rest).pack(fill=tk.X, pady=2)
        ttk.Button(preset_frame, text="Load Pickup", command=self.go_to_pickup).pack(fill=tk.X, pady=2)

        ttk.Separator(preset_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=5)

        # Copy/Paste angles
        ttk.Label(preset_frame, text="Current Angles:", font=('Helvetica', 9, 'bold')).pack(anchor='w')
        ttk.Button(preset_frame, text="📋 Copy Current Angles", command=self.copy_current_angles).pack(fill=tk.X, pady=2)
        ttk.Button(preset_frame, text="📌 Paste to Manual Control", command=self.paste_to_manual).pack(fill=tk.X, pady=2)

        # Speed
        speed_frame = ttk.LabelFrame(right_frame, text="Movement Speed", padding="5")
        speed_frame.pack(fill=tk.X, pady=5)

        self.speed_var = tk.StringVar(value=str(DEFAULT_SPEED))
        ttk.Label(speed_frame, text="Speed (ms/deg):").pack(anchor='w')
        ttk.Entry(speed_frame, textvariable=self.speed_var, width=8, justify='center').pack(pady=5)
        ttk.Button(speed_frame, text="Set Speed", command=self.set_speed).pack(pady=5)

        # Sequence control
        seq_frame = ttk.LabelFrame(right_frame, text="Sequence Control", padding="5")
        seq_frame.pack(fill=tk.X, pady=5)

        self.seq_listbox = tk.Listbox(seq_frame, height=8, width=25)
        self.seq_listbox.pack(fill=tk.X, pady=5)
        self.refresh_seq_list()

        ttk.Button(seq_frame, text="Play Selected", command=self.play_sequence).pack(pady=2)
        ttk.Button(seq_frame, text="Stop", command=self.stop_sequence).pack(pady=2)
    
    def setup_calibration_tab(self):
        """Setup calibration tab"""
        # Left: Camera preview
        left_frame = ttk.LabelFrame(self.calib_tab, text="Camera Preview - Click 4 Corners", padding="10")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.calib_canvas = tk.Canvas(left_frame, bg='black', width=640, height=480)
        self.calib_canvas.pack(fill=tk.BOTH, expand=True)
        self.calib_canvas.bind('<Button-1>', self.on_calib_click)
        
        # Start calibration preview
        self.start_calib_preview()
        
        # Buttons
        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(fill=tk.X, pady=10)

        ttk.Button(btn_frame, text="Reset", command=self.reset_calibration).pack(side=tk.LEFT, padx=5)
        self.calc_btn = ttk.Button(btn_frame, text="Calculate Grid", command=self.calculate_grid, state='disabled')
        self.calc_btn.pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Recapture Empty Grid", command=self.capture_empty_grid).pack(side=tk.LEFT, padx=5)

        # Object detection toggle
        self.detect_in_calib_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(btn_frame, text="Show Object Detection",
                       variable=self.detect_in_calib_var,
                       command=self.toggle_detection_in_calib).pack(side=tk.LEFT, padx=10)

        # Pickup button (initially disabled)
        self.pickup_btn = ttk.Button(btn_frame, text="🤖 PICKUP OBJECT", command=self.pickup_detected_object, state='disabled')
        self.pickup_btn.pack(side=tk.LEFT, padx=10)

        # Detection status label
        self.detection_status_label = ttk.Label(btn_frame, text="No object detected", foreground='gray')
        self.detection_status_label.pack(side=tk.LEFT, padx=10)

        # Right: Instructions & Sensitivity
        right_frame = ttk.Frame(self.calib_tab)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)

        # Instructions
        instr_frame = ttk.LabelFrame(right_frame, text="Instructions", padding="10")
        instr_frame.pack(fill=tk.X, pady=5)

        instructions = """1. Click 4 grid corners in order:
   - Top-Left (Red)
   - Top-Right (Blue)
   - Bottom-Left (Green)
   - Bottom-Right (Yellow)

2. Click "Calculate Grid"
   - Calculates 25 points
   - Captures empty grid reference

3. Enable "Show Object Detection"
   - Detects objects in real-time
   - Shows which cell object is in
   - Logs position coordinates

4. Place object on grid (Cells A-C)
   - See which cell it's detected in
   - Yellow boxes show detections
   - Check System Log for coordinates

5. Click "🤖 PICKUP OBJECT"
   - Executes sequence for that cell
   - Automatically picks up object
   - Status shows pickup progress
"""
        ttk.Label(instr_frame, text=instructions, justify=tk.LEFT).pack(anchor='w')

        self.corner_label = ttk.Label(instr_frame, text="Corners clicked: 0/4", foreground='blue')
        self.corner_label.pack(pady=5)
        
        # Sensitivity
        sens_frame = ttk.LabelFrame(right_frame, text="Detection Sensitivity", padding="10")
        sens_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(sens_frame, text="Color Threshold:").pack(anchor='w')
        ttk.Scale(sens_frame, from_=10, to=100, variable=self.threshold_var,
                 orient=tk.HORIZONTAL, command=lambda v: self.update_sensitivity_labels()).pack(fill=tk.X)
        
        ttk.Label(sens_frame, text="Minimum Area:").pack(anchor='w', pady=(10,0))
        ttk.Scale(sens_frame, from_=1000, to=10000, variable=self.min_area_var,
                 orient=tk.HORIZONTAL, command=lambda v: self.update_sensitivity_labels()).pack(fill=tk.X)
        
        ttk.Label(sens_frame, text="Min Solidity:").pack(anchor='w', pady=(10,0))
        ttk.Scale(sens_frame, from_=0.1, to=0.9, variable=self.solidity_var,
                 orient=tk.HORIZONTAL, command=lambda v: self.update_sensitivity_labels()).pack(fill=tk.X)
        
        self.sens_label = ttk.Label(sens_frame, text="")
        self.sens_label.pack(pady=5)
        self.update_sensitivity_labels()
    
    def setup_sequences_tab(self):
        """Setup sequences tab"""
        # Left: Cell list
        left_frame = ttk.LabelFrame(self.seq_tab, text="Grid Cells", padding="10")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.cell_listbox = tk.Listbox(left_frame, height=16, width=15)
        self.cell_listbox.pack(fill=tk.BOTH, expand=True)
        self.cell_listbox.bind('<<ListboxSelect>>', self.on_cell_select)
        
        for cell in CELL_NAMES:
            has_seq = "✓" if cell in self.sequences else ""
            self.cell_listbox.insert(tk.END, f"{cell} {has_seq}")
        
        # Middle: Sequence editor
        mid_frame = ttk.LabelFrame(self.seq_tab, text="Sequence Editor", padding="10")
        mid_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        ttk.Label(mid_frame, text="Selected Cell:").pack(anchor='w')
        self.selected_cell_label = ttk.Label(mid_frame, text="None", font=('Helvetica', 12, 'bold'))
        self.selected_cell_label.pack(anchor='w', pady=5)
        
        ttk.Label(mid_frame, text="Steps (servo angles: Base Shoulder Elbow Wrist Gripper):").pack(anchor='w', pady=(10,0))

        self.steps_listbox = tk.Listbox(mid_frame, height=12, width=55)
        self.steps_listbox.pack(fill=tk.BOTH, expand=True)

        # Setup right-click context menu for steps
        self.setup_step_context_menu()

        step_btn_frame = ttk.Frame(mid_frame)
        step_btn_frame.pack(fill=tk.X, pady=5)

        ttk.Button(step_btn_frame, text="Add Current", command=self.add_current_step).pack(side=tk.LEFT, padx=2)
        ttk.Button(step_btn_frame, text="Add Rest", command=self.add_rest_step).pack(side=tk.LEFT, padx=2)
        ttk.Button(step_btn_frame, text="Insert Step", command=self.insert_step).pack(side=tk.LEFT, padx=2)
        ttk.Button(step_btn_frame, text="Remove", command=self.remove_step).pack(side=tk.LEFT, padx=2)
        ttk.Button(step_btn_frame, text="Clear", command=self.clear_steps).pack(side=tk.LEFT, padx=2)
        
        ttk.Button(mid_frame, text="Save Sequence to Cell", command=self.save_current_sequence).pack(pady=5)

        # Edit saved sequences
        edit_frame = ttk.LabelFrame(mid_frame, text="Edit Saved Sequences", padding="5")
        edit_frame.pack(fill=tk.X, pady=(10, 0))

        ttk.Label(edit_frame, text="Select sequence from list, then:", font=('Helvetica', 8)).pack(anchor='w')
        ttk.Button(edit_frame, text="Load to Editor", command=self.load_selected_sequence_to_editor).pack(fill=tk.X, pady=2)
        ttk.Button(edit_frame, text="Delete Sequence", command=self.delete_selected_sequence).pack(fill=tk.X, pady=2)
        ttk.Button(edit_frame, text="Duplicate to Another Cell", command=self.duplicate_sequence).pack(fill=tk.X, pady=2)

        # Legacy sequences info
        legacy_frame = ttk.LabelFrame(mid_frame, text="Legacy Sequences (Loaded)", padding="5")
        legacy_frame.pack(fill=tk.X, pady=(10, 0))

        ttk.Label(legacy_frame, text="• Basic Pick & Place\n• LEFT Sequence\n• RIGHT Sequence",
                 font=('Helvetica', 9, 'bold'), foreground='blue').pack(anchor='w')

        # Right: Presets
        right_frame = ttk.LabelFrame(self.seq_tab, text="Quick Presets", padding="10")
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)

        ttk.Button(right_frame, text="Load Rest Position", command=self.load_rest_preset).pack(fill=tk.X, pady=2)
        ttk.Button(right_frame, text="Load Pickup Position", command=self.load_pickup_preset).pack(fill=tk.X, pady=2)

        ttk.Separator(right_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)

        ttk.Label(right_frame, text="Set Custom Presets:", font=('Helvetica', 9, 'bold')).pack(anchor='w')
        ttk.Label(right_frame, text="Move servos to desired position,", font=('Helvetica', 8)).pack(anchor='w')
        ttk.Label(right_frame, text="then click to save:", font=('Helvetica', 8)).pack(anchor='w')
        ttk.Button(right_frame, text="Set Current as Rest", command=self.set_current_as_rest).pack(fill=tk.X, pady=2)
        ttk.Button(right_frame, text="Set Current as Pickup", command=self.set_current_as_pickup).pack(fill=tk.X, pady=2)

        ttk.Button(right_frame, text="Play Sequence", command=self.test_sequence).pack(fill=tk.X, pady=10)
    
    def setup_log_panel(self, parent):
        """Setup shared log panel"""
        log_frame = ttk.LabelFrame(parent, text="System Log", padding="5")
        log_frame.pack(fill=tk.X, padx=5, pady=5)

        # Detection status display at top (static)
        self.detection_display_label = ttk.Label(log_frame, text="📍 Detection Status: No object detected",
                                                  font=('Helvetica', 10, 'bold'), foreground='gray',
                                                  relief=tk.SUNKEN, padding=5)
        self.detection_display_label.pack(fill=tk.X, pady=(0, 5))

        self.log_text = scrolledtext.ScrolledText(log_frame, height=6, width=120, state='disabled')
        self.log_text.pack(fill=tk.X)

        # Floating log window reference
        self.floating_log_window = None

    def update_detection_display(self, cell=None):
        """Update the static detection status display"""
        if cell and cell != '?':
            self.detection_display_label.config(
                text=f"📍 Detection Status: Object in {cell}",
                foreground='green'
            )
        else:
            self.detection_display_label.config(
                text="📍 Detection Status: No object detected",
                foreground='gray'
            )

        # Mirror to floating window if open
        if hasattr(self, 'mirror_detection_callback') and self.mirror_detection_callback:
            self.root.after(0, lambda c=cell: self.mirror_detection_callback(c))

    def open_floating_log(self):
        """Open log in separate floating window"""
        if self.floating_log_window and self.floating_log_window.winfo_exists():
            # Already open, just bring to front
            self.floating_log_window.lift()
            self.floating_log_window.focus_force()
            return

        # Create floating window
        self.floating_log_window = tk.Toplevel(self.root)
        self.floating_log_window.title("📋 System Log - Floating")
        self.floating_log_window.geometry("800x400+100+100")
        self.floating_log_window.attributes('-topmost', False)  # Not always on top

        # Detection status display in floating window
        floating_detection_label = ttk.Label(self.floating_log_window,
                                              text="📍 Detection Status: No object detected",
                                              font=('Helvetica', 10, 'bold'), foreground='gray',
                                              relief=tk.SUNKEN, padding=5)
        floating_detection_label.pack(fill=tk.X, padx=5, pady=5)

        # Create log text in floating window
        floating_log_text = scrolledtext.ScrolledText(self.floating_log_window, height=20, width=100, state='disabled')
        floating_log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Copy existing logs
        current_logs = self.log_text.get('1.0', tk.END)
        floating_log_text.config(state='normal')
        floating_log_text.insert(tk.END, current_logs)
        floating_log_text.config(state='disabled')
        floating_log_text.see(tk.END)

        # Mirror new logs to floating window
        def mirror_log(message):
            try:
                if self.floating_log_window and self.floating_log_window.winfo_exists():
                    floating_log_text.config(state='normal')
                    floating_log_text.insert(tk.END, message + "\n")
                    floating_log_text.see(tk.END)
                    floating_log_text.config(state='disabled')
            except:
                pass

        # Mirror detection display updates
        def mirror_detection(cell):
            try:
                if self.floating_log_window and self.floating_log_window.winfo_exists():
                    if cell and cell != '?':
                        floating_detection_label.config(
                            text=f"📍 Detection Status: Object in {cell}",
                            foreground='green'
                        )
                    else:
                        floating_detection_label.config(
                            text="📍 Detection Status: No object detected",
                            foreground='gray'
                        )
            except:
                pass

        self.mirror_log_callback = mirror_log
        self.mirror_detection_callback = mirror_detection

        # Handle window close
        def on_close():
            self.floating_log_window = None
            self.mirror_log_callback = None
            self.mirror_detection_callback = None
            self.floating_log_window.destroy()

        self.floating_log_window.protocol("WM_DELETE_WINDOW", on_close)
        self.log("Floating log window opened")

    def log_user_event(self):
        """Log a user event for analysis"""
        # Create event log dialog
        dialog = tk.Toplevel(self.root)
        dialog.title("🔍 Log User Event")
        dialog.geometry("400x200")
        dialog.transient(self.root)
        dialog.grab_set()

        ttk.Label(dialog, text="What are you doing?", font=('Helvetica', 12, 'bold')).pack(pady=10)

        event_var = tk.StringVar()
        event_combo = ttk.Combobox(dialog, textvariable=event_var, width=40)
        event_combo['values'] = (
            'Testing servo movement',
            'Playing sequence',
            'Testing object detection',
            'Testing pickup',
            'Calibrating grid',
            'Other'
        )
        event_combo.pack(pady=5)
        event_combo.current(0)

        def log_it():
            event = event_var.get()
            self.log(f"🔍 USER EVENT: {event}")
            dialog.destroy()

        ttk.Button(dialog, text="Log Event", command=log_it).pack(pady=10)
    
    def log(self, message):
        """Add message to log"""
        try:
            timestamp = datetime.now().strftime("%H:%M:%S")
            formatted_msg = f"[{timestamp}] {message}"

            if hasattr(self, 'log_text') and self.log_text:
                self.log_text.config(state='normal')
                self.log_text.insert(tk.END, formatted_msg + "\n")
                self.log_text.see(tk.END)
                self.log_text.config(state='disabled')

            # Mirror to floating window if open
            if hasattr(self, 'mirror_log_callback') and self.mirror_log_callback:
                self.root.after(0, lambda msg=formatted_msg: self.mirror_log_callback(msg))

        except Exception as e:
            print(f"[{timestamp}] Log error: {e}")
    
    # Manual Control Methods
    def refresh_ports(self):
        """Refresh serial ports"""
        ports = serial.tools.list_ports.comports()
        port_list = [f"{p.device} - {p.description}" for p in ports]
        self.port_combo['values'] = port_list
        if port_list:
            # Auto-select Arduino
            for i, p in enumerate(ports):
                if ('Arduino' in p.description or 'CH340' in p.description or
                    'ttyACM' in p.device or 'ttyUSB' in p.device):
                    self.port_combo.current(i)
                    self.log(f"Found: {p.device}")
                    return
            self.port_combo.current(0)
    
    def toggle_connection(self):
        """Toggle serial connection"""
        if self.is_connected:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        """Connect to Arduino in background thread to prevent UI hang"""
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No Port", "Please select a port")
            return

        # Extract device path if format is "device - description"
        if ' - ' in port:
            port = port.split(' - ')[0]

        # Start connection in background thread
        thread = threading.Thread(target=self._connect_thread, args=(port,), daemon=True)
        thread.start()

    def _connect_thread(self, port):
        """Background thread for Arduino connection"""
        try:
            self.serial_conn = serial.Serial(port, BAUD_RATE, timeout=1)
            time.sleep(2)  # Wait for Arduino reset

            # Clear any pending data
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()

            # Update UI from main thread
            self.root.after(0, self._on_connected, port)
        except Exception as e:
            self.root.after(0, self._on_connect_error, e)

    def _on_connected(self, port):
        """Called when connection succeeds"""
        self.is_connected = True
        self.connect_btn.config(text="Disconnect")
        self.status_label.config(text="Status: Connected", foreground='green')
        self.log(f"✓ Connected to {port}")

        # ALWAYS go to rest position when (re)connected - prevents struggling
        self.log("Moving to rest position (prevents struggling)...")
        self.go_to_rest()

        # Start monitoring for communication errors
        self.root.after(5000, self._monitor_communication)

    def _monitor_communication(self):
        """Monitor serial communication for errors and auto-recover"""
        if self.is_connected and self.serial_conn:
            try:
                # Check if port is still open
                if not self.serial_conn.is_open:
                    self.log("⚠ Serial port closed! Attempting recovery...")
                    self.disconnect()
                    self.root.after(1000, self.connect)
                    return

                # Check for pending errors (you can add more checks here)
                # For now, just schedule next check
                self.root.after(5000, self._monitor_communication)
            except Exception as e:
                self.log(f"⚠ Communication error detected: {e}")
                self.log("Attempting to reconnect...")
                self.disconnect()
                self.root.after(1000, self.connect)

    def disconnect(self):
        """Disconnect from Arduino"""
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except:
                pass
        self.is_connected = False
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Status: Disconnected", foreground='red')
        self.log("Disconnected")

        # Start monitoring for reconnection
        self.root.after(1000, self._monitor_reconnection)

    def recover_connection(self):
        """Manual recovery - disconnect and reconnect"""
        self.log("🔄 Recovery initiated...")
        if self.is_connected:
            self.disconnect()
        self.root.after(1000, self.connect)
        self.log("Attempting reconnection...")

    def _monitor_reconnection(self):
        """Monitor for Arduino reconnection and auto-go to rest"""
        if not self.is_connected:
            port = self.port_var.get()
            if ' - ' in port:
                port = port.split(' - ')[0]

            # Try to reconnect
            try:
                test_conn = serial.Serial(port, BAUD_RATE, timeout=0.1)
                test_conn.close()
                # Arduino is back! Trigger reconnection
                self.log("🔌 Arduino reconnected! Going to rest position...")
                self.connect()
                return
            except:
                pass  # Not connected yet

            # Keep monitoring
            self.root.after(1000, self._monitor_reconnection)
    
    def adjust_servo(self, idx, delta):
        """Adjust servo angle with validation"""
        try:
            current = int(self.input_boxes[idx].get())
            new_val = max(MIN_ANGLE, min(MAX_ANGLE, current + delta))

            # Warn if trying to move base beyond safe limits
            if idx == 0:  # Base servo
                if new_val < 10 or new_val > 170:
                    self.log(f"⚠ Warning: Base at {new_val}° (safe range: 10-170°)")

                # CRITICAL: Warn about 110-120° dead zone
                if BASE_WARNING_MIN <= new_val <= BASE_WARNING_MAX:
                    self.log(f"🚨 DANGER: Base at {new_val}° - DEAD ZONE (110-120°) - May malfunction!")

            self.input_boxes[idx].set(str(new_val))
        except:
            pass
    
    def send_servo(self, idx):
        """Send single servo command in background thread"""
        if not self.is_connected:
            self.log("Not connected!")
            return

        # Send in background thread to prevent UI hang
        thread = threading.Thread(target=self._send_servo_thread, args=(idx,), daemon=True)
        thread.start()

    def _send_servo_thread(self, idx):
        """Background thread for sending servo command with validation"""
        try:
            angle = int(self.input_boxes[idx].get())

            # Validate angle
            if angle < MIN_ANGLE or angle > MAX_ANGLE:
                self.root.after(0, lambda: self.log(f"⚠ Invalid angle {angle}° for {JOINT_NAMES[idx]} (must be {MIN_ANGLE}-{MAX_ANGLE})"))
                return

            # Warn about base servo dead zone (but don't block)
            if idx == 0 and BASE_WARNING_MIN <= angle <= BASE_WARNING_MAX:
                self.root.after(0, lambda a=angle: self.log(f"⚠ WARNING: Base {a}° is in danger zone (110-120°) - May malfunction!"))

            command = f"{idx} {angle}\n"
            self.send_to_arduino(command)

            # Update UI from main thread
            self.root.after(0, lambda idx=idx, angle=angle: self.log(f"Sent: {JOINT_NAMES[idx]} -> {angle}°"))
        except ValueError:
            self.root.after(0, lambda: self.log(f"⚠ Invalid angle value"))
        except Exception as ex:
            error_msg = str(ex)
            self.root.after(0, lambda msg=error_msg: self.log(f"Error: {msg}"))

    def send_multi_move(self, angles):
        """Send multi-move command in background thread with validation"""
        if not self.is_connected:
            return

        # Validate all angles first
        for i, angle in enumerate(angles):
            if angle < MIN_ANGLE or angle > MAX_ANGLE:
                self.log(f"⚠ Invalid angle {angle}° for {JOINT_NAMES[i]} (must be {MIN_ANGLE}-{MAX_ANGLE})")
                return

        # Send in background thread to prevent UI hang
        thread = threading.Thread(target=self._send_multi_move_thread, args=(angles,), daemon=True)
        thread.start()

    def _send_multi_move_thread(self, angles):
        """Background thread for sending multi-move command"""
        try:
            command = f"M {angles[0]} {angles[1]} {angles[2]} {angles[3]} {angles[4]}\n"
            self.send_to_arduino(command)
            # Update UI from main thread
            self.root.after(0, lambda a=angles: self.log(f"Sent multi-move: {a} (simultaneous)"))
        except Exception as ex:
            error_msg = str(ex)
            self.root.after(0, lambda msg=error_msg: self.log(f"Error: {msg}"))

    def set_speed(self):
        """Set movement speed in background thread"""
        if not self.is_connected:
            return

        # Send in background thread
        thread = threading.Thread(target=self._set_speed_thread, daemon=True)
        thread.start()

    def _set_speed_thread(self):
        """Background thread for setting speed"""
        try:
            speed = int(self.speed_var.get())
            command = f"99 {speed}\n"
            self.send_to_arduino(command)
            # Update UI from main thread
            self.root.after(0, lambda s=speed: self.log(f"Speed set to {s}ms/deg"))
        except Exception as ex:
            error_msg = str(ex)
            self.root.after(0, lambda msg=error_msg: self.log(f"Error: {msg}"))

    def go_to_rest(self):
        """Go to rest position (simultaneous movement)"""
        rest_angles = LEGACY_PRESETS.get('Rest', DEFAULT_REST)
        for i, angle in enumerate(rest_angles):
            self.input_boxes[i].set(str(angle))
        if self.is_connected:
            self.send_multi_move(rest_angles)
        self.log("Moved to rest position (simultaneous)")

    def go_to_pickup(self):
        """Go to pickup position (simultaneous movement)"""
        pickup_angles = LEGACY_PRESETS.get('Pickup', DEFAULT_PICKUP)
        for i, angle in enumerate(pickup_angles):
            self.input_boxes[i].set(str(angle))
        if self.is_connected:
            self.send_multi_move(pickup_angles)
        self.log("Moved to pickup position (simultaneous)")

    def send_all_servos(self):
        """Send all servo angles simultaneously"""
        if not self.is_connected:
            messagebox.showwarning("Warning", "Not connected to Arduino!")
            return

        try:
            angles = [int(self.input_boxes[i].get()) for i in range(5)]

            # Validate all angles
            for i, angle in enumerate(angles):
                if angle < 0 or angle > 180:
                    messagebox.showwarning("Warning", f"{JOINT_NAMES[i]} angle must be 0-180°")
                    return

            # Send as multi-move for simultaneous movement
            self.log(f"⏱️ Sending all servos at {time.time():.3f}")
            self.send_multi_move(angles)
            self.log(f"📤 Sent all servos simultaneously: {angles}")
        except ValueError:
            messagebox.showerror("Error", "All angles must be valid numbers")

    def copy_current_angles(self):
        """Copy current manual control angles to clipboard"""
        angles = [int(self.input_boxes[i].get()) for i in range(5)]
        self.step_clipboard = [angles]
        self.log(f"📋 Copied current angles: {angles}")
        messagebox.showinfo("Copied!", f"Current angles copied to clipboard:\n{angles}\n\nYou can now paste these angles into sequence steps")

    def paste_to_manual(self):
        """Paste clipboard angles to manual control input boxes"""
        if not hasattr(self, 'step_clipboard') or not self.step_clipboard:
            messagebox.showinfo("Info", "Clipboard is empty\n\nCopy a step first from Sequence Editor (right-click → Copy Step)")
            return

        # Use first step from clipboard
        angles = self.step_clipboard[0]

        # Set input boxes
        for i in range(5):
            self.input_boxes[i].set(str(angles[i]))

        self.log(f"📌 Pasted to Manual Control: {angles}")
        messagebox.showinfo("Pasted!", f"Angles pasted to Manual Control:\n{angles}\n\nClick 'Send' on any servo to move to this position")

    def send_to_arduino(self, command):
        """Send command to Arduino directly (non-blocking) with debug timing"""
        if self.is_connected and self.serial_conn and self.serial_conn.is_open:
            try:
                start_time = time.time()
                with self.serial_lock:
                    self.serial_conn.write(command.encode())
                    self.serial_conn.flush()
                elapsed = time.time() - start_time
                # Log slow sends
                if elapsed > 0.1:
                    self.log(f"⏱️ Serial send took {elapsed:.3f}s: {command.strip()}")
                # Don't sleep inside lock - allows other commands to queue
            except Exception as e:
                self.log(f"Serial error: {e}")

    def start_camera_thread(self):
        """Start camera preview in separate thread"""
        def camera_loop():
            self.camera_running = True
            self.log(f"📷 Camera thread started at {time.time():.3f}")
            while self.camera_running:
                try:
                    loop_start = time.time()
                    self.update_calib_preview_once()
                    loop_elapsed = time.time() - loop_start
                    if loop_elapsed > 0.3:
                        self.log(f"📷 Camera frame took {loop_elapsed:.3f}s")
                    # Sleep to maintain ~0.5 FPS (1 frame every 2 seconds)
                    time.sleep(max(0, 2.0 - loop_elapsed))
                except Exception as e:
                    self.log(f"Camera error: {e}")
                    time.sleep(2)

        self.camera_thread = threading.Thread(target=camera_loop, daemon=True)
        self.camera_thread.start()
        self.log("Camera thread started")

    def update_calib_preview_once(self):
        """Update calibration preview once (called from camera thread) - OPTIMIZED"""
        try:
            if not self.cap or not self.cap.isOpened():
                return

            ret, frame = self.cap.read()
            if not ret:
                return

            # Use cached canvas size (updated periodically from main thread)
            canvas_width = self.canvas_width
            canvas_height = self.canvas_height

            if canvas_width < 2 or canvas_height < 2:
                return

            # Get frame dimensions - reduce resolution for faster processing
            frame_height, frame_width = frame.shape[:2]

            # Calculate scale to fit canvas (maintain aspect ratio)
            scale_x = canvas_width / frame_width
            scale_y = canvas_height / frame_height
            scale = min(scale_x, scale_y)

            new_width = int(frame_width * scale)
            new_height = int(frame_height * scale)

            if new_width < 1 or new_height < 1:
                return

            # Resize (smaller = faster)
            display_frame = cv2.resize(frame, (new_width, new_height))

            # Create black background
            bg = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)

            # Center image
            y_offset = (canvas_height - new_height) // 2
            x_offset = (canvas_width - new_width) // 2
            bg[y_offset:y_offset+new_height, x_offset:x_offset+new_width] = display_frame

            # Store scale and offset for marker drawing
            self.calib_scale = scale
            self.calib_x_offset = x_offset
            self.calib_y_offset = y_offset

            # Draw clicked corners (always visible) - scaled to display coordinates
            corner_colors = [(0, 0, 255), (255, 0, 0), (0, 255, 0), (0, 255, 255)]
            corner_names = ['1:TL', '2:TR', '3:BL', '4:BR']

            for i, (orig_x, orig_y) in enumerate(self.corners):
                if i < 4:
                    # Scale original click coordinates to display coordinates
                    disp_x = int(orig_x * scale) + x_offset
                    disp_y = int(orig_y * scale) + y_offset

                    cv2.circle(bg, (disp_x, disp_y), 15, corner_colors[i], -1)
                    cv2.circle(bg, (disp_x, disp_y), 20, corner_colors[i], 2)
                    cv2.putText(bg, corner_names[i], (disp_x+20, disp_y-20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, corner_colors[i], 2)

            # Draw grid lines if calibrated
            if self.is_calibrated and len(self.all_points) >= 25:
                for row in range(5):
                    idx1 = row * 5
                    idx2 = idx1 + 4
                    if idx2 < len(self.all_points):
                        pt1_x = int(self.all_points[idx1][0] * scale) + x_offset
                        pt1_y = int(self.all_points[idx1][1] * scale) + y_offset
                        pt2_x = int(self.all_points[idx2][0] * scale) + x_offset
                        pt2_y = int(self.all_points[idx2][1] * scale) + y_offset
                        cv2.line(bg, (pt1_x, pt1_y), (pt2_x, pt2_y), (0, 255, 0), 2)

                for col in range(5):
                    idx1 = col
                    idx2 = col + 20
                    if idx2 < len(self.all_points):
                        pt1_x = int(self.all_points[idx1][0] * scale) + x_offset
                        pt1_y = int(self.all_points[idx1][1] * scale) + y_offset
                        pt2_x = int(self.all_points[idx2][0] * scale) + x_offset
                        pt2_y = int(self.all_points[idx2][1] * scale) + y_offset
                        cv2.line(bg, (pt1_x, pt1_y), (pt2_x, pt2_y), (0, 255, 0), 2)

            # Convert BGR to RGB
            bg = cv2.cvtColor(bg, cv2.COLOR_BGR2RGB)

            # Convert to PhotoImage and update canvas (must be from main thread)
            img = Image.fromarray(bg)
            photo = ImageTk.PhotoImage(image=img)

            # Update canvas from main thread
            self.root.after(0, self._update_canvas_image, photo)

        except Exception as e:
            self.log(f"Preview error: {e}")

    def update_canvas_size_cache(self):
        """Update cached canvas size from main thread"""
        try:
            self.canvas_width = self.calib_canvas.winfo_width()
            self.canvas_height = self.calib_canvas.winfo_height()
        except:
            pass

    def _update_canvas_image(self, photo):
        """Update canvas with new image (called from main thread)"""
        try:
            self.calib_canvas.delete("all")
            self.calib_canvas.create_image(0, 0, anchor='nw', image=photo)
            self.calib_canvas.image = photo
        except:
            pass

    def refresh_seq_list(self):
        """Refresh sequence listbox"""
        self.seq_listbox.delete(0, tk.END)
        for name in self.sequences.keys():
            self.seq_listbox.insert(tk.END, name)
    
    def play_sequence(self):
        """Play selected sequence with simultaneous movement in background thread"""
        selection = self.seq_listbox.curselection()
        if not selection:
            self.log("⚠ No sequence selected")
            return

        seq_name = self.seq_listbox.get(selection[0])
        if seq_name not in self.sequences:
            self.log(f"⚠ Sequence '{seq_name}' not found")
            return

        if not self.is_connected:
            self.log("⚠ Not connected to Arduino!")
            return

        # Set stop flag for any currently playing sequence
        self._stop_sequence_flag = True
        self.log("⏹️ Stopping previous sequence...")
        time.sleep(0.1)  # Give thread time to stop

        # Start new sequence in background thread
        self.log(f"▶ Playing sequence: {seq_name}")
        self._stop_sequence_flag = False
        thread = threading.Thread(target=self._play_sequence_thread, args=(seq_name,), daemon=True)
        self._current_sequence_thread = thread
        thread.start()

    def _play_sequence_thread(self, seq_name):
        """Background thread for playing sequence with proper cleanup"""
        try:
            self.log(f"⏱️ Starting sequence {seq_name} at {time.time():.3f}")

            for i, step in enumerate(self.sequences[seq_name]):
                # Check if we should stop
                if getattr(self, '_stop_sequence_flag', False):
                    self.log("⏹️ Sequence stopped by user")
                    break

                if 'angles' in step:
                    # Send multi-move command
                    angles = step['angles']
                    delay = step.get('delay', 1000)
                    step_start = time.time()

                    command = f"M {angles[0]} {angles[1]} {angles[2]} {angles[3]} {angles[4]}\n"
                    self.send_to_arduino(command)
                    self.root.after(0, lambda a=angles: self.log(f"  → Sent: {a}"))

                    # Wait for servo movement (minimum 0.5 seconds)
                    # Check stop flag during wait
                    wait_start = time.time()
                    while time.time() - wait_start < max(0.5, delay / 1000.0):
                        if getattr(self, '_stop_sequence_flag', False):
                            break
                        time.sleep(0.1)

                    step_elapsed = time.time() - step_start
                    if step_elapsed > 0.6:
                        self.log(f"⏱️ Step {i+1} took {step_elapsed:.3f}s")

            self.root.after(0, lambda: self.log(f"✓ Sequence complete"))
        except Exception as ex:
            error_msg = str(ex)
            self.root.after(0, lambda msg=error_msg: self.log(f"✗ Error: {msg}"))
        finally:
            # Clean up thread reference
            if hasattr(self, '_current_sequence_thread'):
                self._current_sequence_thread = None

    def stop_sequence(self):
        """Stop sequence (placeholder)"""
        self._stop_sequence_flag = True
        self.log("⏹️ Sequence stopped")
    
    # Calibration Methods
    def start_calib_preview(self):
        """Start calibration preview - Z-Star USB camera (video3 or video4)"""
        self.log("Starting Z-Star camera (video3)...")

        # Close any existing camera
        if hasattr(self, 'cap') and self.cap:
            self.cap.release()

        # Try to open Z-Star camera (video3 - may change on replug)
        self.cap = cv2.VideoCapture(3)

        # If video3 fails, try video4 (Z-Star may move on replug)
        if not self.cap.isOpened():
            self.log("video3 not available, trying video4...")
            self.cap.release()
            self.cap = cv2.VideoCapture(4)

        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.log("✓ Z-Star camera opened")
            self.update_calib_preview()
        else:
            self.log("✗ ERROR: Z-Star camera not found on video3 or video4!")
            messagebox.showerror("Camera Error", "Could not open Z-Star camera!\n\nTried: /dev/video3, /dev/video4\n\nMake sure:\n1. Camera is plugged in\n2. Try a different USB port")
    
    def update_calib_preview(self):
        """Update calibration preview - scale to fit canvas and center"""
        try:
            if not self.cap or not self.cap.isOpened():
                self.calib_canvas.after(30, self.update_calib_preview)
                return

            ret, frame = self.cap.read()
            if not ret:
                self.calib_canvas.after(30, self.update_calib_preview)
                return

            # Get canvas size
            canvas_width = self.calib_canvas.winfo_width()
            canvas_height = self.calib_canvas.winfo_height()

            if canvas_width < 2 or canvas_height < 2:
                self.calib_canvas.after(30, self.update_calib_preview)
                return

            # Get frame dimensions
            frame_height, frame_width = frame.shape[:2]

            # Calculate scale to fit canvas (maintain aspect ratio)
            scale_x = canvas_width / frame_width
            scale_y = canvas_height / frame_height
            scale = min(scale_x, scale_y)

            # Calculate new dimensions
            new_width = int(frame_width * scale)
            new_height = int(frame_height * scale)

            # Resize frame
            display_frame = cv2.resize(frame, (new_width, new_height))

            # Create black background
            bg = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)

            # Center the image
            x_offset = (canvas_width - new_width) // 2
            y_offset = (canvas_height - new_height) // 2
            bg[y_offset:y_offset+new_height, x_offset:x_offset+new_width] = display_frame

            # Store scale and offset for marker drawing
            self.calib_scale = scale
            self.calib_x_offset = x_offset
            self.calib_y_offset = y_offset

            # Draw clicked corners (always visible) - scaled to display coordinates
            corner_colors = [(0, 0, 255), (255, 0, 0), (0, 255, 0), (0, 255, 255)]
            corner_names = ['1:TL', '2:TR', '3:BL', '4:BR']
            for i, (orig_x, orig_y) in enumerate(self.corners):
                if i < 4:
                    # Scale original click coordinates to display coordinates
                    disp_x = int(orig_x * scale) + x_offset
                    disp_y = int(orig_y * scale) + y_offset

                    cv2.circle(bg, (disp_x, disp_y), 15, corner_colors[i], -1)
                    cv2.circle(bg, (disp_x, disp_y), 20, corner_colors[i], 2)
                    cv2.putText(bg, corner_names[i], (disp_x+20, disp_y-20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, corner_colors[i], 2)

            # If calibrated, draw grid overlay on every frame - scaled
            if self.is_calibrated and len(self.all_points) >= 25:
                # Draw grid lines - scale all_points to display coordinates
                for row in range(5):
                    idx1 = row * 5
                    idx2 = idx1 + 4
                    if idx2 < len(self.all_points):
                        # Scale points to display coordinates
                        pt1_x = int(self.all_points[idx1][0] * scale) + x_offset
                        pt1_y = int(self.all_points[idx1][1] * scale) + y_offset
                        pt2_x = int(self.all_points[idx2][0] * scale) + x_offset
                        pt2_y = int(self.all_points[idx2][1] * scale) + y_offset
                        cv2.line(bg, (pt1_x, pt1_y), (pt2_x, pt2_y), (0, 255, 0), 2)

                for col in range(5):
                    idx1 = col
                    idx2 = col + 20
                    if idx2 < len(self.all_points):
                        # Scale points to display coordinates
                        pt1_x = int(self.all_points[idx1][0] * scale) + x_offset
                        pt1_y = int(self.all_points[idx1][1] * scale) + y_offset
                        pt2_x = int(self.all_points[idx2][0] * scale) + x_offset
                        pt2_y = int(self.all_points[idx2][1] * scale) + y_offset
                        cv2.line(bg, (pt1_x, pt1_y), (pt2_x, pt2_y), (0, 255, 0), 2)

                # Draw calculated corner points - scaled
                for i in range(4):
                    if i < len(self.all_points):
                        disp_x = int(self.all_points[i][0] * scale) + x_offset
                        disp_y = int(self.all_points[i][1] * scale) + y_offset
                        cv2.circle(bg, (disp_x, disp_y), 10, corner_colors[i], -1)

            # Object detection - draw detected objects (if enabled and empty grid is available)
            # Only detect every 5th frame (2 FPS instead of 10 FPS) to reduce CPU
            self.frame_count += 1
            if self.detect_in_calib_var.get() and hasattr(self, 'empty_grid') and self.empty_grid is not None:
                if self.frame_count % 5 == 0:  # Detect every 5th frame
                    objects = self.detect_objects(frame)
                    if objects:
                        self.last_detection_time = time.time()
                        self.last_detected_cell = max(objects, key=lambda o: o['area'])
                        self.last_detected_cell['cell'] = self.find_cell(self.last_detected_cell['cx'], self.last_detected_cell['cy'])
                else:
                    # Use last detection if within timeout (prevents flickering)
                    if time.time() - self.last_detection_time < self.detection_timeout and self.last_detected_cell:
                        objects = [self.last_detected_cell]
                    else:
                        objects = []
                        self.last_detected_cell = None

                # Track detected objects and enable pickup button
                self.detected_objects = objects

                if objects and self.last_detected_cell:
                    # Use the persisted detection
                    cell = self.last_detected_cell.get('cell', '?')

                    # Check if cell has a sequence and is A, B, or C (not D)
                    if cell and cell != '?' and cell in self.sequences and cell[0] in ['A', 'B', 'C']:
                        self.current_detection_cell = cell
                        self.pickup_btn.config(state='normal')
                        self.detection_status_label.config(
                            text=f"📍 Object in {cell} - Ready to pickup!",
                            foreground='green'
                        )
                        # Update static display at top of log
                        self.update_detection_display(cell)
                    else:
                        self.current_detection_cell = None
                        self.pickup_btn.config(state='disabled')
                        if cell and cell != '?':
                            if cell[0] in ['A', 'B', 'C']:
                                self.detection_status_label.config(
                                    text=f"Object in {cell} (no sequence)",
                                    foreground='orange'
                                )
                            else:
                                self.detection_status_label.config(
                                    text=f"Object in {cell} (cell D not supported)",
                                    foreground='orange'
                                )
                        else:
                            self.detection_status_label.config(
                                text="Object detected (outside grid)",
                                foreground='orange'
                            )
                        # Update static display
                        self.update_detection_display(cell if cell != '?' else None)
                else:
                    self.current_detection_cell = None
                    self.pickup_btn.config(state='disabled')
                    self.detection_status_label.config(
                        text="No object detected",
                        foreground='gray'
                    )
                    # Update static display
                    self.update_detection_display(None)

                for obj in objects:
                    # Scale detection coordinates to display coordinates
                    disp_x = int(obj['x'] * scale) + x_offset
                    disp_y = int(obj['y'] * scale) + y_offset
                    disp_w = int(obj['w'] * scale)
                    disp_h = int(obj['h'] * scale)

                    # Draw detection box
                    cv2.rectangle(bg, (disp_x, disp_y), (disp_x+disp_w, disp_y+disp_h), (0, 255, 255), 2)

                    # Get cell name
                    cell = self.find_cell(obj['cx'], obj['cy'])
                    cv2.putText(bg, f"Object: {cell}", (disp_x, disp_y-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # Convert BGR to RGB
            bg = cv2.cvtColor(bg, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(bg)
            photo = ImageTk.PhotoImage(image=img)

            self.calib_canvas.delete("all")
            self.calib_canvas.create_image(0, 0, anchor='nw', image=photo)
            self.calib_canvas.image = photo

            self.calib_canvas.after(500, self.update_calib_preview)  # 2 FPS for maximum performance
        except Exception as e:
            self.log(f"Preview error: {e}")
            self.calib_canvas.after(1000, self.update_calib_preview)
    
    def on_calib_click(self, event):
        """Handle calibration click - convert from display to original coordinates"""
        if len(self.corners) >= 4:
            return

        # Get click position (display coordinates)
        click_x = event.x
        click_y = event.y

        # Convert from display coordinates back to original image coordinates
        if hasattr(self, 'calib_scale') and self.calib_scale > 0:
            # Remove offset
            adj_x = click_x - self.calib_x_offset
            adj_y = click_y - self.calib_y_offset

            # Check if click is within image area
            if adj_x < 0 or adj_y < 0:
                return

            # Scale back to original image coordinates
            orig_x = int(adj_x / self.calib_scale)
            orig_y = int(adj_y / self.calib_scale)

            self.corners.append((orig_x, orig_y))
            self.log(f"Corner {len(self.corners)}: ({orig_x}, {orig_y})")
        else:
            # Fallback if scale not set yet
            self.corners.append((click_x, click_y))

        self.corner_label.config(text=f"Corners clicked: {len(self.corners)}/4")

        if len(self.corners) >= 4:
            self.calc_btn.config(state='normal')
            self.log("All 4 corners clicked - ready to calculate grid")
    
    def reset_calibration(self):
        """Reset calibration"""
        self.corners = []
        self.all_points = []
        self.is_calibrated = False
        self.corner_label.config(text="Corners clicked: 0/4")
        self.calc_btn.config(state='disabled')
        self.log("Calibration reset")
    
    def calculate_grid(self):
        """Calculate grid from corners"""
        if len(self.corners) < 4:
            return
        
        # Capture empty grid
        ret, frame = self.cap.read()
        if ret:
            self.empty_grid = frame.copy()
            cv2.imwrite(BG_FILE, self.empty_grid)
            self.log("Empty grid reference captured")
        
        # Calculate 25 points
        tl = np.array(self.corners[0], dtype=np.float32)
        tr = np.array(self.corners[1], dtype=np.float32)
        bl = np.array(self.corners[2], dtype=np.float32)
        br = np.array(self.corners[3], dtype=np.float32)
        
        self.all_points = []
        for row in range(5):
            for col in range(5):
                t = col / 4.0
                u = row / 4.0
                top = tl + (tr - tl) * t
                bottom = bl + (br - bl) * t
                point = top + (bottom - top) * u
                self.all_points.append((int(point[0]), int(point[1])))
        
        self.is_calibrated = True
        self.log(f"Grid calculated: {len(self.all_points)} points")

        # Grid overlay will now be drawn automatically by update_calib_preview

        messagebox.showinfo("Success", f"Grid calculated!\n\n{len(self.all_points)} points generated\n\nEmpty grid captured!\n\nGrid lines are now visible on the camera preview!\n\nGo to Auto Detection tab to test")

    def draw_grid_overlay(self):
        """Draw digital grid lines on calibration canvas"""
        try:
            if not self.is_calibrated:
                self.log("Cannot draw overlay: not calibrated")
                return

            if len(self.all_points) < 25:
                self.log(f"Cannot draw overlay: only {len(self.all_points)} points")
                return

            self.log(f"Drawing grid overlay with {len(self.all_points)} points...")

            # Get current frame from camera
            ret, frame = self.cap.read()
            if not ret:
                self.log("Cannot draw overlay: camera not available")
                return

            self.log("Camera frame obtained, drawing lines...")

            # Draw grid lines on frame
            for row in range(5):
                # Draw horizontal line
                idx1 = row * 5
                idx2 = idx1 + 4
                if idx2 < len(self.all_points):
                    pt1 = self.all_points[idx1]
                    pt2 = self.all_points[idx2]
                    self.log(f"  Horizontal {row}: {pt1} -> {pt2}")
                    cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            for col in range(5):
                # Draw vertical line
                idx1 = col
                idx2 = col + 20
                if idx2 < len(self.all_points):
                    pt1 = self.all_points[idx1]
                    pt2 = self.all_points[idx2]
                    self.log(f"  Vertical {col}: {pt1} -> {pt2}")
                    cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            # Draw corner points
            corner_colors = [(0, 0, 255), (255, 0, 0), (0, 255, 0), (0, 255, 255)]
            corner_names = ['TL', 'TR', 'BL', 'BR']
            for i in range(4):
                if i < len(self.all_points):
                    x, y = self.all_points[i]
                    cv2.circle(frame, (x, y), 10, corner_colors[i], -1)
                    cv2.putText(frame, corner_names[i], (x+15, y-15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, corner_colors[i], 2)

            # Convert and display
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            photo = ImageTk.PhotoImage(image=img)

            self.calib_canvas.delete("all")
            self.calib_canvas.create_image(0, 0, anchor='nw', image=photo)
            self.calib_canvas.image = photo

            self.log("✓ Grid overlay drawn successfully!")
        except Exception as e:
            self.log(f"✗ Error drawing overlay: {e}")
            import traceback
            self.log(traceback.format_exc())
    
    def capture_empty_grid(self):
        """Recapture empty grid"""
        ret, frame = self.cap.read()
        if ret:
            self.empty_grid = frame.copy()
            cv2.imwrite(BG_FILE, self.empty_grid)
            self.log("Empty grid recaptured")
            messagebox.showinfo("Success", "Empty grid reference updated!")

    def toggle_detection_in_calib(self):
        """Toggle object detection in calibration tab"""
        enabled = self.detect_in_calib_var.get()
        if enabled:
            self.log("Object detection enabled in calibration tab")
        else:
            self.log("Object detection disabled in calibration tab")
            # Clear detection state
            self.detected_objects = []
            self.current_detection_cell = None
            self.pickup_btn.config(state='disabled')
            self.detection_status_label.config(text="No object detected", foreground='gray')

    def pickup_detected_object(self):
        """Execute sequence for detected object"""
        if not self.current_detection_cell:
            messagebox.showwarning("Warning", "No object detected!")
            return

        cell = self.current_detection_cell

        if cell not in self.sequences:
            messagebox.showwarning("Warning", f"No sequence saved for {cell}!")
            return

        if not self.is_connected:
            messagebox.showerror("Error", "Not connected to Arduino!")
            return

        # Confirm pickup
        confirm = messagebox.askyesno(
            "Confirm Pickup",
            f"Execute sequence for {cell} to pickup object?\n\n"
            f"This will run {len(self.sequences[cell])} steps."
        )

        if not confirm:
            return

        # Execute sequence in background thread
        self.log(f"🤖 Starting pickup sequence for {cell}...")
        self.pickup_btn.config(state='disabled')

        # Set stop flag for any currently playing sequence
        self._stop_sequence_flag = True
        time.sleep(0.1)  # Give thread time to stop

        # Start new pickup sequence
        self._stop_sequence_flag = False
        thread = threading.Thread(target=self._execute_pickup_sequence, args=(cell,), daemon=True)
        self._current_pickup_thread = thread
        thread.start()

    def _execute_pickup_sequence(self, cell):
        """Execute pickup sequence in background thread with proper cleanup"""
        try:
            self.log(f"⏱️ Starting pickup for {cell} at {time.time():.3f}")

            for i, step in enumerate(self.sequences[cell]):
                # Check if we should stop
                if getattr(self, '_stop_sequence_flag', False):
                    self.log("⏹️ Pickup stopped")
                    break

                if 'angles' in step:
                    angles = step['angles']
                    delay = step.get('delay', 1000)
                    step_start = time.time()

                    # Send multi-move command directly
                    command = f"M {angles[0]} {angles[1]} {angles[2]} {angles[3]} {angles[4]}\n"
                    self.send_to_arduino(command)

                    self.root.after(0, lambda s=i+1, c=cell: self.log(f"  Step {s}/{len(self.sequences[c])}: {angles}"))

                    # Wait for servo movement (minimum 0.5 seconds)
                    wait_start = time.time()
                    while time.time() - wait_start < max(0.5, delay / 1000.0):
                        if getattr(self, '_stop_sequence_flag', False):
                            break
                        time.sleep(0.1)

                    step_elapsed = time.time() - step_start
                    if step_elapsed > 0.6:
                        self.log(f"⏱️ Step {i+1} took {step_elapsed:.3f}s")

            total_elapsed = time.time() - step_start
            self.root.after(0, lambda c=cell: self.log(f"✓ Pickup for {c} complete! (Total: {total_elapsed:.3f}s)"))
            self.root.after(0, lambda: self.log("🤖 Object picked up successfully!"))

            # Recapture empty grid after successful pickup
            self.root.after(1000, self._recapture_empty_grid_after_pickup)

        except Exception as e:
            self.root.after(0, lambda: self.log(f"✗ Pickup error: {e}"))
            self.root.after(0, lambda: self.pickup_btn.config(state='normal'))
        finally:
            # Clean up thread reference
            if hasattr(self, '_current_pickup_thread'):
                self._current_pickup_thread = None

    def _recapture_empty_grid_after_pickup(self):
        """Recapture empty grid reference after pickup"""
        self.log("📸 Recapturing empty grid reference...")

        ret, frame = self.cap.read()
        if ret:
            self.empty_grid = frame.copy()
            cv2.imwrite(BG_FILE, self.empty_grid)
            self.log("✓ Empty grid reference updated")
        else:
            self.log("⚠ Could not recapture empty grid")

        # Clear detection state
        self._clear_detection_after_pickup()

    def _clear_detection_after_pickup(self):
        """Clear detection state after successful pickup"""
        self.detected_objects = []
        self.current_detection_cell = None
        self.pickup_btn.config(state='disabled')
        self.detection_status_label.config(text="Pickup complete!", foreground='green')

        # Reset status after 3 seconds
        self.root.after(3000, lambda: self.detection_status_label.config(
            text="No object detected", foreground='gray'
        ))
    
    def update_sensitivity_labels(self):
        """Update sensitivity labels"""
        self.sens_label.config(
            text=f"Threshold: {self.threshold_var.get()}\n"
                 f"Area: {self.min_area_var.get()}\n"
                 f"Solidity: {self.solidity_var.get():.2f}"
        )
    
    # Sequence Methods
    def load_presets(self):
        """Load user-modified presets from file"""
        if os.path.exists(PRESETS_FILE):
            try:
                with open(PRESETS_FILE, 'r') as f:
                    saved_presets = json.load(f)

                # Update LEGACY_PRESETS with saved values
                for name, angles in saved_presets.items():
                    if name in LEGACY_PRESETS:
                        LEGACY_PRESETS[name] = angles
                        self.log(f"✓ Loaded preset: {name} = {angles}")

                # Update defaults
                if 'Rest' in saved_presets:
                    DEFAULT_REST[:] = saved_presets['Rest']
                if 'Pickup' in saved_presets:
                    DEFAULT_PICKUP[:] = saved_presets['Pickup']

            except Exception as e:
                self.log(f"⚠ Could not load presets: {e}")
        else:
            self.log("Using default presets")

    def save_presets(self):
        """Save current presets to file"""
        try:
            presets_to_save = {
                'Rest': LEGACY_PRESETS['Rest'],
                'Pickup': LEGACY_PRESETS['Pickup']
            }
            with open(PRESETS_FILE, 'w') as f:
                json.dump(presets_to_save, f, indent=2)
            self.log(f"✓ Presets saved to {PRESETS_FILE}")
        except Exception as e:
            self.log(f"✗ Could not save presets: {e}")

    def auto_go_to_rest_on_startup(self):
        """Auto-go to rest position on startup if connected"""
        if self.is_connected:
            self.log("Auto-going to rest position on startup...")
            self.go_to_rest()
        else:
            self.log("Not connected - skipping auto-rest")

    def load_sequences(self):
        # First load legacy sequences
        self.log("Loading legacy sequences...")
        for seq_name, steps in LEGACY_SEQUENCES.items():
            # Convert preset names to angles
            converted_steps = []
            for step in steps:
                if isinstance(step[0], str):
                    preset_name = step[0]
                    delay = step[1] if len(step) > 1 else 1000
                    # Get angles from legacy presets
                    if preset_name in LEGACY_PRESETS:
                        angles = LEGACY_PRESETS[preset_name]
                        converted_steps.append({
                            'angles': angles,
                            'delay': delay,
                            'name': preset_name
                        })
                        self.log(f"  {seq_name}: {preset_name} -> {angles}")
            self.sequences[seq_name] = converted_steps

        self.log(f"✓ Loaded {len(self.sequences)} legacy sequences")

        # Also try to load from file
        if os.path.exists(SEQUENCES_FILE):
            try:
                with open(SEQUENCES_FILE, 'r') as f:
                    file_sequences = json.load(f)
                for seq_name, steps in file_sequences.items():
                    if seq_name not in self.sequences:
                        self.sequences[seq_name] = steps
                        self.log(f"✓ Loaded additional sequence: {seq_name}")
            except Exception as e:
                self.log(f"Could not load sequences from file: {e}")
    
    def on_cell_select(self, event):
        """Handle cell selection"""
        selection = self.cell_listbox.curselection()
        if not selection:
            return

        cell = CELL_NAMES[selection[0]]
        self.current_cell = cell
        self.selected_cell_label.config(text=cell)

        # Load steps
        self.steps_listbox.delete(0, tk.END)
        if cell in self.sequences:
            for i, step in enumerate(self.sequences[cell]):
                angles = step.get('angles', [0,0,0,0,0])
                self.steps_listbox.insert(tk.END, f"{i+1}. {angles}")

            self.log(f"Loaded sequence for {cell} ({len(self.sequences[cell])} steps)")
        else:
            self.log(f"No sequence saved for {cell}")

    def load_selected_sequence_to_editor(self):
        """Load selected sequence - works with cell selection OR sequence listbox"""
        # First try cell selection (primary method)
        cell_selection = self.cell_listbox.curselection()
        if cell_selection:
            cell = CELL_NAMES[cell_selection[0]]
            if cell in self.sequences:
                self.on_cell_select(None)
                self.log(f"Loaded {cell} sequence for editing ({len(self.sequences[cell])} steps)")
                return
            else:
                messagebox.showinfo("Info", f"No sequence saved for {cell}\n\nCreate one by adding steps and clicking 'Save Sequence to Cell'")
                return

        # Fallback to sequence listbox
        selection = self.seq_listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Select a cell from the left list first")
            return

        seq_name = self.seq_listbox.get(selection[0])
        if seq_name not in self.sequences:
            return

        # Find which cell this sequence belongs to
        for cell, seq in self.sequences.items():
            if seq == self.sequences.get(seq_name):
                # Select that cell
                cell_index = CELL_NAMES.index(cell)
                self.cell_listbox.selection_clear(0, tk.END)
                self.cell_listbox.selection_set(cell_index)
                self.on_cell_select(None)
                self.log(f"Loaded '{seq_name}' for editing")
                return

    def delete_selected_sequence(self):
        """Delete selected sequence"""
        selection = self.seq_listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Select a sequence first")
            return

        seq_name = self.seq_listbox.get(selection[0])

        # Find and delete
        for cell in list(self.sequences.keys()):
            if self.sequences[cell] == self.sequences.get(seq_name):
                if messagebox.askyesno("Confirm Delete", f"Delete sequence for cell {cell}?"):
                    del self.sequences[cell]
                    self.log(f"Deleted sequence for {cell}")

                    # Save to file
                    with open(SEQUENCES_FILE, 'w') as f:
                        json.dump(self.sequences, f, indent=2)

                    # Refresh cell list
                    self.cell_listbox.delete(0, tk.END)
                    for c in CELL_NAMES:
                        has_seq = "✓" if c in self.sequences else ""
                        self.cell_listbox.insert(tk.END, f"{c} {has_seq}")

                    # Refresh sequence listbox
                    self.refresh_seq_list()

                    # Clear editor
                    self.steps_listbox.delete(0, tk.END)
                    self.selected_cell_label.config(text="None")
                    self.current_cell = None
                return

    def duplicate_sequence(self):
        """Duplicate selected sequence to another cell"""
        selection = self.seq_listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Select a sequence first")
            return

        seq_name = self.seq_listbox.get(selection[0])

        # Find source cell
        source_cell = None
        for cell, seq in self.sequences.items():
            if seq == self.sequences.get(seq_name):
                source_cell = cell
                break

        if not source_cell:
            return

        # Ask which cell to duplicate to
        dialog = tk.Toplevel(self.root)
        dialog.title("Duplicate Sequence")
        dialog.geometry("300x200")
        dialog.transient(self.root)
        dialog.grab_set()

        ttk.Label(dialog, text=f"Duplicate {source_cell} sequence to:", font=('Helvetica', 10, 'bold')).pack(pady=10)

        target_var = tk.StringVar(value=CELL_NAMES[0])
        target_combo = ttk.Combobox(dialog, textvariable=target_var, values=CELL_NAMES, state='readonly')
        target_combo.pack(pady=5)

        def do_duplicate():
            target = target_var.get()
            if target == source_cell:
                messagebox.showwarning("Warning", "Select a different cell!")
                return

            self.sequences[target] = self.sequences[source_cell].copy()

            # Save to file
            with open(SEQUENCES_FILE, 'w') as f:
                json.dump(self.sequences, f, indent=2)

            self.log(f"Duplicated {source_cell} sequence to {target}")

            # Refresh cell list
            self.cell_listbox.delete(0, tk.END)
            for c in CELL_NAMES:
                has_seq = "✓" if c in self.sequences else ""
                self.cell_listbox.insert(tk.END, f"{c} {has_seq}")

            # Refresh sequence listbox
            self.refresh_seq_list()

            dialog.destroy()
            messagebox.showinfo("Success", f"Sequence duplicated to {target}!")

        ttk.Button(dialog, text="Duplicate", command=do_duplicate).pack(pady=10)
    
    def add_current_step(self):
        """Add current servo positions as step"""
        if not self.current_cell:
            messagebox.showwarning("Warning", "Select a cell first")
            return
        
        angles = [int(self.input_boxes[i].get()) for i in range(5)]
        self.steps_listbox.insert(tk.END, f"{self.steps_listbox.size()+1}. {angles}")
        self.log(f"Added step to {self.current_cell}: {angles}")
    
    def add_rest_step(self):
        """Add rest position as step"""
        if not self.current_cell:
            return
        self.steps_listbox.insert(tk.END, f"{self.steps_listbox.size()+1}. {DEFAULT_REST}")

    def insert_step(self):
        """Insert step at selected position"""
        if not self.current_cell:
            messagebox.showwarning("Warning", "Select a cell first")
            return

        selection = self.steps_listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Select where to insert the step")
            return

        insert_pos = selection[0]

        # Create insert dialog
        dialog = tk.Toplevel(self.root)
        dialog.title(f"Insert Step at Position {insert_pos + 1}")
        dialog.geometry("450x350")
        dialog.transient(self.root)
        dialog.grab_set()

        ttk.Label(dialog, text=f"Insert Step at Position {insert_pos + 1}:",
                 font=('Helvetica', 12, 'bold')).pack(pady=10)

        # Paste from clipboard button
        paste_frame = ttk.Frame(dialog)
        paste_frame.pack(pady=5)

        ttk.Button(paste_frame, text="📌 Paste from Clipboard",
                  command=lambda: self.paste_to_insert_dialog(angle_vars, dialog, insert_pos)).pack()

        ttk.Label(dialog, text="— OR create new step —", foreground='gray').pack(pady=5)

        # Create angle inputs
        angle_vars = []
        angles_frame = ttk.Frame(dialog)
        angles_frame.pack(pady=10)

        for i, name in enumerate(JOINT_NAMES):
            frame = ttk.Frame(angles_frame)
            frame.grid(row=i, column=0, padx=10, pady=5)

            ttk.Label(frame, text=f"{name}:", width=10).pack(side=tk.LEFT)
            var = tk.StringVar(value="0")
            ttk.Entry(frame, textvariable=var, width=6, justify='center').pack(side=tk.LEFT)
            angle_vars.append(var)

        def do_insert():
            try:
                angles = [int(var.get()) for var in angle_vars]

                # Validate
                for i, angle in enumerate(angles):
                    if angle < 0 or angle > 180:
                        messagebox.showwarning("Warning", f"{JOINT_NAMES[i]} angle must be 0-180°")
                        return

                # Insert at selected position
                self.steps_listbox.insert(insert_pos, f"{insert_pos + 1}. {angles}")

                # Renumber all steps
                self.renumber_steps()

                self.log(f"Inserted step at {insert_pos + 1}: {angles}")
                dialog.destroy()
            except ValueError:
                messagebox.showerror("Error", "All angles must be numbers")

        ttk.Button(dialog, text="Insert Step", command=do_insert).pack(pady=10)
        ttk.Button(dialog, text="Cancel", command=dialog.destroy).pack()

    def paste_to_insert_dialog(self, angle_vars, dialog, insert_pos):
        """Paste clipboard angles to insert dialog"""
        if not hasattr(self, 'step_clipboard') or not self.step_clipboard:
            messagebox.showinfo("Info", "Clipboard is empty\n\nCopy a step first (right-click → Copy Step)")
            return

        # Use first step from clipboard
        angles = self.step_clipboard[0]

        # Set angle inputs
        for i, var in enumerate(angle_vars):
            var.set(str(angles[i]))

        self.log(f"📋 Pasted {angles} to insert dialog")
    
    def remove_step(self):
        """Remove selected step"""
        selection = self.steps_listbox.curselection()
        if selection:
            self.steps_listbox.delete(selection[0])
            self.renumber_steps()

    def clear_steps(self):
        """Clear all steps"""
        self.steps_listbox.delete(0, tk.END)

    def renumber_steps(self):
        """Renumber all steps in listbox"""
        items = []
        for i in range(self.steps_listbox.size()):
            item = self.steps_listbox.get(i)
            try:
                angles_str = item.split('. ')[1]
                items.append(angles_str)
            except:
                items.append(f"[0, 0, 0, 0, 0]")

        self.steps_listbox.delete(0, tk.END)
        for i, item in enumerate(items):
            self.steps_listbox.insert(tk.END, f"{i + 1}. {item}")

    def setup_step_context_menu(self):
        """Setup right-click context menu for steps listbox"""
        # Create context menu
        self.step_menu = tk.Menu(self.root, tearoff=0)
        self.step_menu.add_command(label="✏️ Edit Step", command=self.edit_selected_step)
        self.step_menu.add_separator()
        self.step_menu.add_command(label="📋 Copy Step", command=self.copy_selected_step)
        self.step_menu.add_command(label="📋 Copy All Steps", command=self.copy_all_steps)
        self.step_menu.add_separator()
        self.step_menu.add_command(label="✂️ Cut Step", command=self.cut_selected_step)
        self.step_menu.add_separator()
        self.step_menu.add_command(label="🗑️ Delete Step", command=self.remove_step)
        self.step_menu.add_separator()
        self.step_menu.add_command(label="📌 Paste Step", command=self.paste_step)

        # Bind right-click and double-click
        self.steps_listbox.bind('<Button-3>', self.show_step_menu)
        self.steps_listbox.bind('<Double-Button-1>', lambda e: self.edit_selected_step())

        # Clipboard for steps
        self.step_clipboard = []

    def show_step_menu(self, event):
        """Show context menu on right-click"""
        try:
            self.step_menu.tk_popup(event.x_root, event.y_root)
        finally:
            self.step_menu.grab_release()

    def edit_selected_step(self):
        """Edit selected step angles"""
        selection = self.steps_listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Select a step to edit")
            return

        step_idx = selection[0]
        item = self.steps_listbox.get(step_idx)

        # Parse current angles
        try:
            angles_str = item.split('. ')[1].strip('[]')
            current_angles = [int(x.strip()) for x in angles_str.split(',')]
        except:
            messagebox.showerror("Error", "Could not parse step")
            return

        # Create edit dialog
        dialog = tk.Toplevel(self.root)
        dialog.title(f"Edit Step {step_idx + 1}")
        dialog.geometry("400x300")
        dialog.transient(self.root)
        dialog.grab_set()

        ttk.Label(dialog, text=f"Edit Step {step_idx + 1} Angles:", font=('Helvetica', 12, 'bold')).pack(pady=10)

        # Create angle inputs
        angle_vars = []
        angles_frame = ttk.Frame(dialog)
        angles_frame.pack(pady=10)

        for i, name in enumerate(JOINT_NAMES):
            frame = ttk.Frame(angles_frame)
            frame.grid(row=i, column=0, padx=10, pady=5)

            ttk.Label(frame, text=f"{name}:", width=10).pack(side=tk.LEFT)
            var = tk.StringVar(value=str(current_angles[i]))
            ttk.Entry(frame, textvariable=var, width=6, justify='center').pack(side=tk.LEFT)
            angle_vars.append(var)

        def save_edit():
            try:
                new_angles = [int(var.get()) for var in angle_vars]

                # Validate
                for i, angle in enumerate(new_angles):
                    if angle < 0 or angle > 180:
                        messagebox.showwarning("Warning", f"{JOINT_NAMES[i]} angle must be 0-180°")
                        return

                # Update step
                self.steps_listbox.delete(step_idx)
                self.steps_listbox.insert(step_idx, f"{step_idx + 1}. {new_angles}")
                self.log(f"Edited step {step_idx + 1}: {new_angles}")
                dialog.destroy()
            except ValueError:
                messagebox.showerror("Error", "All angles must be numbers")

        ttk.Button(dialog, text="Save Changes", command=save_edit).pack(pady=10)
        ttk.Button(dialog, text="Cancel", command=dialog.destroy).pack()

    def copy_selected_step(self):
        """Copy selected step to clipboard"""
        selection = self.steps_listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Select a step to copy")
            return

        step_idx = selection[0]
        item = self.steps_listbox.get(step_idx)

        try:
            angles_str = item.split('. ')[1].strip('[]')
            angles = [int(x.strip()) for x in angles_str.split(',')]
            self.step_clipboard = [angles]
            self.log(f"Copied step {step_idx + 1}: {angles}")
        except:
            messagebox.showerror("Error", "Could not copy step")

    def copy_all_steps(self):
        """Copy all steps to clipboard"""
        if not self.current_cell or self.current_cell not in self.sequences:
            messagebox.showwarning("Warning", "No sequence loaded")
            return

        self.step_clipboard = [step['angles'] for step in self.sequences[self.current_cell]]
        self.log(f"Copied all {len(self.step_clipboard)} steps from {self.current_cell}")

    def cut_selected_step(self):
        """Cut selected step to clipboard"""
        self.copy_selected_step()
        self.remove_step()

    def paste_step(self):
        """Paste step from clipboard"""
        if not self.step_clipboard:
            messagebox.showinfo("Info", "Clipboard is empty\n\nCopy a step first")
            return

        if not self.current_cell:
            messagebox.showwarning("Warning", "Select a cell first")
            return

        # Get insertion position
        selection = self.steps_listbox.curselection()
        insert_pos = selection[0] if selection else tk.END

        # Paste all steps from clipboard
        for i, angles in enumerate(self.step_clipboard):
            self.steps_listbox.insert(insert_pos if insert_pos != tk.END else tk.END, f"{self.steps_listbox.size() + 1}. {angles}")

        self.log(f"Pasted {len(self.step_clipboard)} step(s)")

        # Renumber steps
        self.renumber_steps()
    
    def save_current_sequence(self):
        """Save sequence to cell"""
        if not self.current_cell:
            messagebox.showwarning("Warning", "Select a cell first")
            return

        steps = []
        for i in range(self.steps_listbox.size()):
            item = self.steps_listbox.get(i)
            # Parse angles from string
            try:
                angles_str = item.split('. ')[1].strip('[]')
                angles = [int(x.strip()) for x in angles_str.split(',')]
                steps.append({'angles': angles, 'delay': 1000})
                self.log(f"  Step {i+1}: {angles}")
            except Exception as e:
                self.log(f"⚠ Could not parse step {i+1}: {e}")

        if not steps:
            messagebox.showwarning("Warning", "No steps in sequence!")
            return

        self.sequences[self.current_cell] = steps

        # Save to file
        with open(SEQUENCES_FILE, 'w') as f:
            json.dump(self.sequences, f, indent=2)

        # Update cell list
        self.cell_listbox.delete(0, tk.END)
        for cell in CELL_NAMES:
            has_seq = "✓" if cell in self.sequences else ""
            self.cell_listbox.insert(tk.END, f"{cell} {has_seq}")

        # Refresh sequence listbox in Manual Control tab
        self.refresh_seq_list()

        self.log(f"✓ Saved sequence to {self.current_cell} ({len(steps)} steps)")
        messagebox.showinfo("Success", f"Sequence saved to {self.current_cell}!\n\n{len(steps)} steps")
    
    def load_rest_preset(self):
        """Load rest position from presets"""
        angles = LEGACY_PRESETS.get('Rest', DEFAULT_REST)
        for i, angle in enumerate(angles):
            self.input_boxes[i].set(str(angle))
        self.log(f"Loaded Rest position: {angles}")

    def load_pickup_preset(self):
        """Load pickup position from presets"""
        angles = LEGACY_PRESETS.get('Pickup', DEFAULT_PICKUP)
        for i, angle in enumerate(angles):
            self.input_boxes[i].set(str(angle))
        self.log(f"Loaded Pickup position: {angles}")

    def set_current_as_rest(self):
        """Set current servo positions as new Rest preset"""
        angles = [int(self.input_boxes[i].get()) for i in range(5)]
        LEGACY_PRESETS['Rest'] = angles
        DEFAULT_REST[:] = angles
        self.log(f"✓ Rest position updated to: {angles}")
        self.save_presets()
        messagebox.showinfo("Rest Updated", f"New Rest position saved:\n{angles}\n\nThis will persist after restart!")

    def set_current_as_pickup(self):
        """Set current servo positions as new Pickup preset"""
        angles = [int(self.input_boxes[i].get()) for i in range(5)]
        LEGACY_PRESETS['Pickup'] = angles
        DEFAULT_PICKUP[:] = angles
        self.log(f"✓ Pickup position updated to: {angles}")
        self.save_presets()
        messagebox.showinfo("Pickup Updated", f"New Pickup position saved:\n{angles}\n\nThis will persist after restart!")
    
    def test_sequence(self):
        """Test current sequence"""
        if not self.current_cell or self.current_cell not in self.sequences:
            messagebox.showwarning("Warning", "No sequence for this cell")
            return
        
        self.log(f"Testing sequence for {self.current_cell}")
        # Would execute sequence here

    def detect_objects(self, frame):
        """Detect objects in frame and log positions"""
        if self.empty_grid is None:
            return []

        # Background subtraction
        diff = cv2.absdiff(self.empty_grid, frame)
        diff_gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(diff_gray, self.threshold_var.get(), 255, cv2.THRESH_BINARY)

        # Morphological operations
        kernel = np.ones((5, 5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)

        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        objects = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.min_area_var.get():
                x, y, w, h = cv2.boundingRect(cnt)

                # Find cell
                cx, cy = x + w//2, y + h//2
                cell = self.find_cell(cx, cy)

                objects.append({
                    'x': x, 'y': y, 'w': w, 'h': h,
                    'cx': cx, 'cy': cy,
                    'area': area,
                    'cell': cell
                })

        # Log detected objects
        if objects:
            # Detection status shown in static display, no need to log
            pass

        return objects
    
    def find_cell(self, x, y):
        """Find which cell contains point (x, y)"""
        # First try calibrated grid points
        if self.is_calibrated and len(self.all_points) >= 25:
            for row in range(4):
                for col in range(4):
                    idx1 = row * 5 + col
                    idx2 = idx1 + 1
                    idx3 = idx1 + 5
                    idx4 = idx3 + 1

                    if idx4 < len(self.all_points):
                        x1, y1 = self.all_points[idx1]
                        x2, y2 = self.all_points[idx2]
                        x3, y3 = self.all_points[idx3]
                        x4, y4 = self.all_points[idx4]

                        min_x = min(x1, x2, x3, x4)
                        max_x = max(x1, x2, x3, x4)
                        min_y = min(y1, y2, y3, y4)
                        max_y = max(y1, y2, y3, y4)

                        if min_x <= x <= max_x and min_y <= y <= max_y:
                            return f"{chr(ord('A')+row)}{col+1}"

        # Fallback: Simple grid division (if calibration not available)
        # Assumes camera view is roughly aligned with grid
        if hasattr(self, 'empty_grid') and self.empty_grid is not None:
            h, w = self.empty_grid.shape[:2]

            # Divide into 4x4 grid
            cell_w = w / 4
            cell_h = h / 4

            col = int(x / cell_w)
            row = int(y / cell_h)

            if 0 <= row < 4 and 0 <= col < 4:
                cell = f"{chr(ord('A')+row)}{col+1}"
                return cell

        return "?"
    
    def load_calibration(self):
        """Load existing calibration"""
        if os.path.exists(CALIBRATION_FILE):
            try:
                with open(CALIBRATION_FILE, 'r') as f:
                    data = json.load(f)
                if 'grid_intersections' in data:
                    # Convert to list format
                    self.all_points = []
                    for i in range(5):
                        for j in range(5):
                            cell = f"{chr(ord('A')+i)}{j+1}"
                            if cell in data['grid_intersections']:
                                pt = data['grid_intersections'][cell]
                                self.all_points.append((pt['x'], pt['y']))
                    self.is_calibrated = True
                    self.log("Calibration loaded")
            except Exception as e:
                self.log(f"Could not load calibration: {e}")
        
        # Load empty grid
        if os.path.exists(BG_FILE):
            self.empty_grid = cv2.imread(BG_FILE)
            self.log("Empty grid reference loaded")


def main():
    root = tk.Tk()
    app = UnifiedControlSystem(root)
    root.mainloop()


if __name__ == "__main__":
    main()
