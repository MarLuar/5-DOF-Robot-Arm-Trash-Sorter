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
DEFAULT_SPEED = 5  # Default servo speed (ms/deg)
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
CELL_NAMES = [f"{chr(ord('A')+row)}{col+1}" for row in range(4) for col in range(4)]  # 4x4 = 16 cells

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
        self.min_area_var = tk.IntVar(value=1300)  # Default minimum area: 1300px²
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
        self.detection_timeout = 8.0  # Keep detection for 8 seconds (increased from 3s for stability)
        self.last_detected_cell = None
        self._prev_detection_cell = None  # Track previous cell for sample clearing

        # Cell hysteresis - prevents cell jumping due to detection noise
        self.cell_confirmation_count = 0
        self.cell_confirmed = None
        self.cell_hysteresis_threshold = 3  # Must see same cell 3 times before switching

        # Auto-pickup state tracking
        self.auto_pickup_enabled = False
        self.object_first_detected_time = 0
        self.auto_pickup_thread = None
        self.auto_pickup_running = False
        self.pending_pickup_cell = None  # Cell waiting for confirmation
        self.object_confirmation_delay = 4.0  # Object must be present for 4 seconds before auto pickup

        # Waste classification
        self.waste_classifier = None
        self.classifier_loaded = False
        self.classification_results = {}  # cell -> (class_name, confidence)

        # Classification smoothing (prevents flickering)
        self.classification_history = []  # List of (class_name, confidence) tuples
        self.classification_history_size = 10  # Keep last 10 classifications
        self.classification_confirmed = None  # Last confirmed classification
        self.classification_confirmation_threshold = 6  # Need 6/10 votes to confirm
        self.classification_last_update = 0  # Timestamp of last display update
        self.classification_update_interval = 1.0  # Update display at most every 1 second

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
        self._after_ids = []  # Track root.after() IDs to cancel them

        # Debug: Track thread activity
        self.main_thread_id = threading.current_thread().ident
        self.last_command_time = 0

        # Auto-offset ratio setting (compensates for trash between grids)
        self.offset_ratio_var = tk.DoubleVar(value=2.0)  # For every 2° error, adjust 1° (medium sensitivity)
        self.auto_offset_suggestions = []  # List of (offset, confidence, reason) tuples
        self.last_offset_analysis_time = 0
        self.offset_analysis_interval = 0.5  # Analyze every 0.5 seconds (2 Hz) for fast response
        self.auto_offset_enabled_var = tk.BooleanVar(value=True)  # Enable auto-analysis by default
        self._last_applied_offset = 0.0  # Track last applied offset for display

        # Load saved camera setting BEFORE UI setup (calibration tab needs it)
        self.load_camera_setting()  # Auto-load saved camera setting

        # Setup UI
        self.setup_ui()
        self.refresh_ports()
        self.load_calibration()
        self.load_presets()  # Load user-modified presets

        # Start logging
        self.log("System initialized - v1.00.02")
        self.log(f"Main thread: {threading.current_thread().name}")

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

        ttk.Button(top_bar, text="Open Floating Log Window",
                  command=self.open_floating_log).pack(side=tk.RIGHT, padx=5)

        # Analysis mode button (for debugging)
        ttk.Button(top_bar, text="Log Event",
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
                ttk.Label(frame, text="(110-120° risky)", foreground='orange', font=('Helvetica', 8)).pack(side=tk.LEFT, padx=3)

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
        ttk.Button(preset_frame, text="Send All (Simultaneous)", command=self.send_all_servos).pack(side=tk.LEFT, padx=5)

        # Camera Configuration (on left side)
        cam_frame = ttk.LabelFrame(left_frame, text="ZStar Camera Settings", padding="5")
        cam_frame.pack(fill=tk.X, pady=5)

        ttk.Label(cam_frame, text="Camera Device:", font=('Helvetica', 9, 'bold')).pack(anchor='w')

        cam_dev_frame = ttk.Frame(cam_frame)
        cam_dev_frame.pack(fill=tk.X, pady=5)

        self.camera_index_var = tk.StringVar(value="3")
        self.camera_combo = ttk.Combobox(cam_dev_frame, textvariable=self.camera_index_var, width=12, values=["0", "1", "2", "3", "4", "5", "6", "7"])
        self.camera_combo.pack(side=tk.LEFT, padx=5)

        ttk.Button(cam_dev_frame, text="Find", command=self.find_cameras).pack(side=tk.LEFT, padx=2)
        ttk.Button(cam_dev_frame, text="Test", command=self.test_camera).pack(side=tk.LEFT, padx=2)

        ttk.Separator(cam_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=5)

        ttk.Button(cam_frame, text="Save Camera Setting", command=self.save_camera_setting).pack(fill=tk.X, pady=2)
        ttk.Button(cam_frame, text="Load Saved Setting", command=self.load_camera_setting).pack(fill=tk.X, pady=2)

        ttk.Separator(cam_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=5)

        ttk.Button(cam_frame, text="Set Camera for Calibration", command=self.set_camera_for_calibration, style='Accent.TButton').pack(fill=tk.X, pady=2)

        self.camera_status_label = ttk.Label(cam_frame, text="Status: Not configured", foreground='gray', font=('Helvetica', 8))
        self.camera_status_label.pack(pady=5)

        # System Log (on left side)
        log_frame = ttk.LabelFrame(left_frame, text="System Log", padding="5")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        # Detection status display at top
        self.detection_display_label = ttk.Label(log_frame, text="Detection Status: No object detected",
                                                  font=('Helvetica', 9, 'bold'), foreground='gray',
                                                  relief=tk.SUNKEN, padding=5)
        self.detection_display_label.pack(fill=tk.X, pady=(0, 5))

        self.log_text = scrolledtext.ScrolledText(log_frame, height=10, width=50, state='disabled')
        self.log_text.pack(fill=tk.BOTH, expand=True)

        # Floating log window reference
        self.floating_log_window = None

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

        self.recover_btn = ttk.Button(port_frame, text="Recover", command=self.recover_connection)
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
        ttk.Button(preset_frame, text="Copy Current Angles", command=self.copy_current_angles).pack(fill=tk.X, pady=2)
        ttk.Button(preset_frame, text="Paste to Manual Control", command=self.paste_to_manual).pack(fill=tk.X, pady=2)

        # Speed
        speed_frame = ttk.LabelFrame(right_frame, text="Movement Speed", padding="5")
        speed_frame.pack(fill=tk.X, pady=5)

        self.speed_var = tk.StringVar(value=str(DEFAULT_SPEED))
        ttk.Label(speed_frame, text="Speed (ms/deg):").pack(anchor='w')
        ttk.Entry(speed_frame, textvariable=self.speed_var, width=8, justify='center').pack(pady=5)
        ttk.Button(speed_frame, text="Set Speed", command=self.set_speed).pack(pady=5)

        # Auto-Offset Ratio Setting (for grid alignment)
        offset_ratio_frame = ttk.LabelFrame(right_frame, text="Auto-Offset Ratio", padding="5")
        offset_ratio_frame.pack(fill=tk.X, pady=5)

        ttk.Label(offset_ratio_frame, text="Adjustment sensitivity:", font=('Helvetica', 9, 'bold')).pack(anchor='w')
        ttk.Label(offset_ratio_frame, text="For every X° detected error, adjust base by 1°",
                 font=('Helvetica', 8), foreground='gray').pack(anchor='w')

        ratio_control_frame = ttk.Frame(offset_ratio_frame)
        ratio_control_frame.pack(fill=tk.X, pady=5)

        ttk.Button(ratio_control_frame, text="-0.5", width=5,
                  command=lambda: self.adjust_offset_ratio(-0.5)).pack(side=tk.LEFT, padx=2)
        ttk.Button(ratio_control_frame, text="-0.1", width=5,
                  command=lambda: self.adjust_offset_ratio(-0.1)).pack(side=tk.LEFT, padx=2)

        self.offset_ratio_var = tk.DoubleVar(value=2.0)
        self.offset_ratio_entry = ttk.Entry(ratio_control_frame, textvariable=self.offset_ratio_var,
                                      width=8, justify='center')
        self.offset_ratio_entry.pack(side=tk.LEFT, padx=5)
        self.offset_ratio_entry.bind('<Return>', lambda e: self.save_offset_ratio())

        ttk.Button(ratio_control_frame, text="+0.1", width=5,
                  command=lambda: self.adjust_offset_ratio(0.1)).pack(side=tk.LEFT, padx=2)
        ttk.Button(ratio_control_frame, text="+0.5", width=5,
                  command=lambda: self.adjust_offset_ratio(0.5)).pack(side=tk.LEFT, padx=2)

        self.offset_ratio_status = ttk.Label(offset_ratio_frame, text="Ratio: 1:2.0 (Medium sensitivity)",
                                             foreground='blue', font=('Helvetica', 8))
        self.offset_ratio_status.pack(pady=3)

        ttk.Label(offset_ratio_frame, text="Lower = more sensitive, Higher = less sensitive",
                 font=('Helvetica', 8), foreground='gray').pack(anchor='w')

        # Sequence control
        seq_frame = ttk.LabelFrame(right_frame, text="Sequence Control", padding="5")
        seq_frame.pack(fill=tk.X, pady=5)

        self.seq_listbox = tk.Listbox(seq_frame, height=8, width=25)
        self.seq_listbox.pack(fill=tk.X, pady=5)
        self.refresh_seq_list()

        ttk.Button(seq_frame, text="Play Selected", command=self.play_sequence).pack(pady=2)
        ttk.Button(seq_frame, text="Stop", command=self.stop_sequence).pack(pady=2)

        # Auto-Offset Suggestion (Manual Control tab)
        manual_auto_offset_frame = ttk.LabelFrame(right_frame, text="Auto-Offset Suggestion", padding="10")
        manual_auto_offset_frame.pack(fill=tk.X, pady=10)

        self.manual_auto_offset_label = ttk.Label(manual_auto_offset_frame,
                                                   text="No suggestion available",
                                                   font=('Helvetica', 9), foreground='gray')
        self.manual_auto_offset_label.pack(anchor='w', pady=5)

        self.manual_auto_offset_conf_label = ttk.Label(manual_auto_offset_frame,
                                                      text="",
                                                      font=('Helvetica', 8), foreground='gray')
        self.manual_auto_offset_conf_label.pack(anchor='w', pady=3)

        manual_auto_btn_frame = ttk.Frame(manual_auto_offset_frame)
        manual_auto_btn_frame.pack(fill=tk.X, pady=5)

        ttk.Button(manual_auto_btn_frame, text="Apply Suggestion",
                  command=self.apply_suggested_offset, width=18).pack(side=tk.LEFT, padx=2)
        ttk.Button(manual_auto_btn_frame, text="Analyze Now",
                  command=self.analyze_offset_suggestion, width=12).pack(side=tk.LEFT, padx=2)

        ttk.Label(manual_auto_offset_frame, text="For auto-analysis, enable in Grid Calibration tab",
                 font=('Helvetica', 8), foreground='blue').pack(anchor='w', pady=(5,0))

    def setup_calibration_tab(self):
        """Setup calibration tab"""
        # Left: Camera preview (expands to fill available space)
        left_frame = ttk.LabelFrame(self.calib_tab, text="Camera Preview - Click 4 Corners", padding="10")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.calib_canvas = tk.Canvas(left_frame, bg='black', highlightthickness=0)
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

        # Waste classification toggle
        self.waste_classify_var = tk.BooleanVar(value=False)
        self.waste_classify_chk = ttk.Checkbutton(btn_frame, text="Enable Waste Classification",
                       variable=self.waste_classify_var,
                       command=self.toggle_waste_classification)
        self.waste_classify_chk.pack(side=tk.LEFT, padx=10)

        # Auto Pickup toggle
        self.auto_pickup_var = tk.BooleanVar(value=False)
        self.auto_pickup_chk = ttk.Checkbutton(btn_frame, text="Auto Pickup",
                       variable=self.auto_pickup_var,
                       command=self.toggle_auto_pickup)
        self.auto_pickup_chk.pack(side=tk.LEFT, padx=10)
        self.auto_pickup_chk.config(state='disabled')  # Disabled until calibrated

        # Auto-Offset toggle (moved from right panel)
        self.auto_offset_btn_frame = ttk.Frame(btn_frame)
        self.auto_offset_btn_frame.pack(side=tk.LEFT, padx=10)
        self.auto_offset_enabled_var = tk.BooleanVar(value=False)
        self.auto_offset_chk = ttk.Checkbutton(self.auto_offset_btn_frame, text="Auto Analysis",
                       variable=self.auto_offset_enabled_var,
                       command=self.toggle_auto_offset_suggestions)
        self.auto_offset_chk.pack(side=tk.LEFT)

        # Detection status label
        self.detection_status_label = ttk.Label(btn_frame, text="No object detected", foreground='gray')
        self.detection_status_label.pack(side=tk.LEFT, padx=10)

        # Right: Compact control panel with scrollbar
        right_container = ttk.Frame(self.calib_tab)
        right_container.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)

        # Create canvas with scrollbar for right panel
        right_canvas = tk.Canvas(right_container, width=440, highlightthickness=0)
        scrollbar = ttk.Scrollbar(right_container, orient="vertical", command=right_canvas.yview)
        self.right_panel = ttk.Frame(right_canvas)

        self.right_panel.bind(
            "<Configure>",
            lambda e: right_canvas.configure(scrollregion=right_canvas.bbox("all"))
        )

        right_canvas.create_window((0, 0), window=self.right_panel, anchor="nw")
        right_canvas.configure(yscrollcommand=scrollbar.set)

        right_container.pack(side=tk.RIGHT, fill=tk.Y)
        right_canvas.pack(side="left", fill="y", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Bind mouse wheel to scroll
        def _on_mousewheel(event):
            right_canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        right_canvas.bind_all("<MouseWheel>", _on_mousewheel)

        # Auto-Offset Suggestion (compact)
        auto_offset_frame = ttk.LabelFrame(self.right_panel, text="Auto-Offset Suggestion", padding="5")
        auto_offset_frame.pack(fill=tk.X, pady=5)

        ttk.Label(auto_offset_frame, text="Suggests base offset for trash between grids",
                 font=('Helvetica', 7), foreground='blue', wraplength=270).pack(anchor='w')

        self.auto_offset_suggest_label = ttk.Label(auto_offset_frame,
                                                   text="No suggestion",
                                                   font=('Helvetica', 9, 'bold'), foreground='gray')
        self.auto_offset_suggest_label.pack(anchor='w', pady=3)

        self.auto_offset_confidence_label = ttk.Label(auto_offset_frame,
                                                      text="",
                                                      font=('Helvetica', 7), foreground='gray')
        self.auto_offset_confidence_label.pack(anchor='w')

        auto_offset_btn_frame = ttk.Frame(auto_offset_frame)
        auto_offset_btn_frame.pack(fill=tk.X, pady=3)

        ttk.Button(auto_offset_btn_frame, text="Apply",
                  command=self.apply_suggested_offset, width=10).pack(side=tk.LEFT, padx=2)
        ttk.Button(auto_offset_btn_frame, text="Analyze",
                  command=self.analyze_offset_suggestion, width=10).pack(side=tk.LEFT, padx=2)

        # Offset Ratio Setting (compact)
        offset_ratio_calib_frame = ttk.LabelFrame(self.right_panel, text="Auto-Offset Ratio", padding="5")
        offset_ratio_calib_frame.pack(fill=tk.X, pady=5)

        ttk.Label(offset_ratio_calib_frame, text="For every X° error, adjust base by 1°",
                 font=('Helvetica', 7), foreground='gray').pack(anchor='w')

        ratio_calib_control = ttk.Frame(offset_ratio_calib_frame)
        ratio_calib_control.pack(fill=tk.X, pady=3)

        ttk.Button(ratio_calib_control, text="-0.5", width=4,
                  command=lambda: self.adjust_offset_ratio(-0.5)).pack(side=tk.LEFT, padx=1)
        ttk.Button(ratio_calib_control, text="-0.1", width=4,
                  command=lambda: self.adjust_offset_ratio(-0.1)).pack(side=tk.LEFT, padx=1)

        self.calib_offset_ratio_entry = ttk.Entry(ratio_calib_control, textvariable=self.offset_ratio_var,
                                            width=6, justify='center')
        self.calib_offset_ratio_entry.pack(side=tk.LEFT, padx=3)
        self.calib_offset_ratio_entry.bind('<Return>', lambda e: self.save_offset_ratio())

        ttk.Button(ratio_calib_control, text="+0.1", width=4,
                  command=lambda: self.adjust_offset_ratio(0.1)).pack(side=tk.LEFT, padx=1)
        ttk.Button(ratio_calib_control, text="+0.5", width=4,
                  command=lambda: self.adjust_offset_ratio(0.5)).pack(side=tk.LEFT, padx=1)

        self.calib_offset_ratio_status = ttk.Label(offset_ratio_calib_frame, text="Ratio: 1:2.0 (Medium sensitivity)",
                                             foreground='blue', font=('Helvetica', 7))
        self.calib_offset_ratio_status.pack(pady=2)

        # Sensitivity (compact)
        sens_frame = ttk.LabelFrame(self.right_panel, text="Detection Sensitivity", padding="5")
        sens_frame.pack(fill=tk.X, pady=5)

        ttk.Label(sens_frame, text="Color Threshold:", font=('Helvetica', 7)).pack(anchor='w')
        ttk.Scale(sens_frame, from_=10, to=100, variable=self.threshold_var,
                 orient=tk.HORIZONTAL, command=lambda v: self.update_sensitivity_labels()).pack(fill=tk.X)

        ttk.Label(sens_frame, text="Min Area:", font=('Helvetica', 7)).pack(anchor='w', pady=(5,0))
        ttk.Scale(sens_frame, from_=1000, to=10000, variable=self.min_area_var,
                 orient=tk.HORIZONTAL, command=lambda v: self.update_sensitivity_labels()).pack(fill=tk.X)

        ttk.Label(sens_frame, text="Min Solidity:", font=('Helvetica', 7)).pack(anchor='w', pady=(5,0))
        ttk.Scale(sens_frame, from_=0.1, to=0.9, variable=self.solidity_var,
                 orient=tk.HORIZONTAL, command=lambda v: self.update_sensitivity_labels()).pack(fill=tk.X)

        self.sens_label = ttk.Label(sens_frame, text="", font=('Helvetica', 7))
        self.sens_label.pack(pady=3)
        self.update_sensitivity_labels()

        # Corner status
        self.corner_label = ttk.Label(self.right_panel, text="Corners: 0/4 clicked", foreground='blue', font=('Helvetica', 8, 'bold'))
        self.corner_label.pack(pady=5)

        # Camera Settings (compact)
        cam_frame = ttk.LabelFrame(self.right_panel, text="Camera Exposure", padding="5")
        cam_frame.pack(fill=tk.X, pady=5)

        # Exposure mode
        ttk.Label(cam_frame, text="Mode:", font=('Helvetica', 7, 'bold')).pack(anchor='w')
        self.exposure_mode_var = tk.StringVar(value="auto")
        mode_frame = ttk.Frame(cam_frame)
        mode_frame.pack(fill=tk.X, pady=2)
        ttk.Radiobutton(mode_frame, text="Auto", variable=self.exposure_mode_var,
                       value="auto", command=self.update_camera_exposure).pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(mode_frame, text="Manual", variable=self.exposure_mode_var,
                       value="manual", command=self.update_camera_exposure).pack(side=tk.LEFT, padx=5)

        # Exposure slider
        ttk.Label(cam_frame, text="Exposure:", font=('Helvetica', 7)).pack(anchor='w', pady=(5,0))
        self.exposure_var = tk.IntVar(value=2400)
        self.exposure_scale = ttk.Scale(cam_frame, from_=156, to=5000,
                                        variable=self.exposure_var, orient=tk.HORIZONTAL,
                                        command=self.update_camera_exposure)
        self.exposure_scale.pack(fill=tk.X)
        self.exposure_value_label = ttk.Label(cam_frame, text="2400 (Auto)", foreground='gray', font=('Helvetica', 7))
        self.exposure_value_label.pack()

        # Gamma
        ttk.Label(cam_frame, text="Gamma:", font=('Helvetica', 7)).pack(anchor='w', pady=(5,0))
        self.gamma_var = tk.IntVar(value=140)
        self.gamma_scale = ttk.Scale(cam_frame, from_=100, to=200,
                                     variable=self.gamma_var, orient=tk.HORIZONTAL,
                                     command=self.update_camera_exposure)
        self.gamma_scale.pack(fill=tk.X)
        self.gamma_value_label = ttk.Label(cam_frame, text="140", foreground='gray', font=('Helvetica', 7))
        self.gamma_value_label.pack()

        # Saturation
        ttk.Label(cam_frame, text="Saturation:", font=('Helvetica', 7)).pack(anchor='w', pady=(5,0))
        self.saturation_var = tk.IntVar(value=17)
        self.saturation_scale = ttk.Scale(cam_frame, from_=0, to=32,
                                          variable=self.saturation_var, orient=tk.HORIZONTAL,
                                          command=self.update_camera_exposure)
        self.saturation_scale.pack(fill=tk.X)
        self.saturation_value_label = ttk.Label(cam_frame, text="17", foreground='gray', font=('Helvetica', 7))
        self.saturation_value_label.pack()

        # Sharpness
        ttk.Label(cam_frame, text="Sharpness:", font=('Helvetica', 7)).pack(anchor='w', pady=(5,0))
        self.sharpness_var = tk.IntVar(value=5)
        self.sharpness_scale = ttk.Scale(cam_frame, from_=0, to=10,
                                         variable=self.sharpness_var, orient=tk.HORIZONTAL,
                                         command=self.update_camera_exposure)
        self.sharpness_scale.pack(fill=tk.X)
        self.sharpness_value_label = ttk.Label(cam_frame, text="5", foreground='gray', font=('Helvetica', 7))
        self.sharpness_value_label.pack()

        # Reset button
        ttk.Button(cam_frame, text="Reset Defaults",
                  command=self.reset_camera_settings).pack(pady=5, fill=tk.X)

        # Step 3 & 4 Base Values Display
        step_values_frame = ttk.LabelFrame(self.right_panel, text="Step 3 & 4 Base Values", padding="5")
        step_values_frame.pack(fill=tk.X, pady=10)

        ttk.Label(step_values_frame, text="Base angles being sent to Arduino:",
                 font=('Helvetica', 7), foreground='gray').pack(anchor='w')

        self.step3_base_label = ttk.Label(step_values_frame, text="Step 3 (Pickup): --°",
                                          font=('Helvetica', 9, 'bold'), foreground='blue')
        self.step3_base_label.pack(anchor='w', pady=3)

        self.step4_base_label = ttk.Label(step_values_frame, text="Step 4 (Lift): --°",
                                          font=('Helvetica', 9, 'bold'), foreground='green')
        self.step4_base_label.pack(anchor='w', pady=3)

        ttk.Label(step_values_frame, text="Values include auto-offset if enabled",
                 font=('Helvetica', 6), foreground='gray').pack(anchor='w', pady=(3,0))

        # Apply initial settings
        self.root.after(1000, self.init_camera_settings)

    def setup_sequences_tab(self):
        """Setup sequences tab"""
        # Left: Cell list - show ALL sequences (original + _NON variants)
        left_frame = ttk.LabelFrame(self.seq_tab, text="Grid Cells", padding="10")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.cell_listbox = tk.Listbox(left_frame, height=16, width=18)
        self.cell_listbox.pack(fill=tk.BOTH, expand=True)
        self.cell_listbox.bind('<<ListboxSelect>>', self.on_cell_select)

        # First show original cells (A1-D4) - 4x4 = 16 cells
        ttk.Label(left_frame, text="Biodegradable (base=180°):", font=('Helvetica', 9, 'bold')).pack(anchor='w')
        for cell in CELL_NAMES:
            has_seq = "[OK]" if cell in self.sequences else ""
            self.cell_listbox.insert(tk.END, f"{cell} {has_seq}")

        # Then show NON variants (A1_NON-D4_NON)
        ttk.Separator(left_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=5)
        ttk.Label(left_frame, text="Non-biodegradable (base=0°):", font=('Helvetica', 9, 'bold')).pack(anchor='w')
        for cell in CELL_NAMES:
            non_cell = f"{cell}_NON"
            has_seq = "[OK]" if non_cell in self.sequences else ""
            self.cell_listbox.insert(tk.END, f"{non_cell} {has_seq}")
        
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

        ttk.Separator(edit_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=5)

        ttk.Label(edit_frame, text="Clipboard:", font=('Helvetica', 8)).pack(anchor='w')
        ttk.Button(edit_frame, text="Paste Step to Editor", command=self.paste_step_to_editor).pack(fill=tk.X, pady=2)

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

        # Global Base Adjustment
        base_adj_frame = ttk.LabelFrame(right_frame, text="Global Base Adjustment", padding="8")
        base_adj_frame.pack(fill=tk.X, pady=10)

        ttk.Label(base_adj_frame, text="Adjust ALL base angles in ALL sequences:", font=('Helvetica', 8, 'bold')).pack(anchor='w', pady=(0,5))

        base_adj_control = ttk.Frame(base_adj_frame)
        base_adj_control.pack(fill=tk.X, pady=3)

        ttk.Button(base_adj_control, text="-20°", width=5,
                  command=lambda: self.global_base_adjust(-20)).pack(side=tk.LEFT, padx=2)
        ttk.Button(base_adj_control, text="-10°", width=5,
                  command=lambda: self.global_base_adjust(-10)).pack(side=tk.LEFT, padx=2)
        ttk.Button(base_adj_control, text="-5°", width=5,
                  command=lambda: self.global_base_adjust(-5)).pack(side=tk.LEFT, padx=2)

        self.base_adjust_var = tk.StringVar(value="0")
        self.base_adjust_entry = ttk.Entry(base_adj_control, textvariable=self.base_adjust_var, width=6, justify='center')
        self.base_adjust_entry.pack(side=tk.LEFT, padx=5)
        self.base_adjust_entry.bind('<Return>', lambda e: self.global_base_adjust_custom())

        ttk.Button(base_adj_control, text="+5°", width=5,
                  command=lambda: self.global_base_adjust(5)).pack(side=tk.LEFT, padx=2)
        ttk.Button(base_adj_control, text="+10°", width=5,
                  command=lambda: self.global_base_adjust(10)).pack(side=tk.LEFT, padx=2)
        ttk.Button(base_adj_control, text="+20°", width=5,
                  command=lambda: self.global_base_adjust(20)).pack(side=tk.LEFT, padx=2)

        ttk.Button(base_adj_control, text="Apply", width=8,
                  command=self.global_base_adjust_custom, style='Accent.TButton').pack(side=tk.LEFT, padx=5)

        self.base_adjust_status = ttk.Label(base_adj_frame, text="No adjustment applied", foreground='gray', font=('Helvetica', 7))
        self.base_adjust_status.pack(anchor='w', pady=3)

        ttk.Label(base_adj_frame, text="⚠️ This modifies the sequences file", font=('Helvetica', 7), foreground='orange').pack(anchor='w')

    def update_detection_display(self, cell=None):
        """Update the static detection status display"""
        if cell and cell != '?':
            self.detection_display_label.config(
                text=f"Detection Status: Object in {cell}",
                foreground='green'
            )
        else:
            self.detection_display_label.config(
                text="Detection Status: No object detected",
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
        self.floating_log_window.title("System Log - Floating")
        self.floating_log_window.geometry("800x400+100+100")
        self.floating_log_window.attributes('-topmost', False)  # Not always on top

        # Detection status display in floating window
        floating_detection_label = ttk.Label(self.floating_log_window,
                                              text="Detection Status: No object detected",
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
                            text=f"Detection Status: Object in {cell}",
                            foreground='green'
                        )
                    else:
                        floating_detection_label.config(
                            text="Detection Status: No object detected",
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
        dialog.title("Log User Event")
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
            self.log(f"USER EVENT: {event}")
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

    def find_cameras(self):
        """Find all available camera devices"""
        self.log("Scanning for cameras...")
        available_cameras = []
        camera_details = []

        for i in range(8):
            try:
                cap = cv2.VideoCapture(i)
                if cap.isOpened():
                    ret, frame = cap.read()
                    if ret:
                        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        fps = cap.get(cv2.CAP_PROP_FPS)
                        fps_str = f"{fps:.1f}fps" if fps > 0 else "unknown fps"
                        available_cameras.append(i)
                        camera_details.append(f"• /dev/video{i}: {width}x{height} @ {fps_str}")
                        self.log(f"Camera found: /dev/video{i} - {width}x{height} @ {fps_str}")
                    cap.release()
                else:
                    cap.release()
            except Exception as e:
                self.log(f"Error checking /dev/video{i}: {e}")

        if available_cameras:
            self.camera_status_label.config(
                text=f"Found: {len(available_cameras)} camera(s)",
                foreground='green'
            )

            # Auto-select Z-Star (usually video3 or video4)
            zstar_found = False
            for i in available_cameras:
                if i in [3, 4]:
                    self.camera_combo.set(str(i))
                    self.log(f"ZStar camera detected at index {i}")
                    zstar_found = True
                    break

            # If no video3/4, select first available
            if not zstar_found and available_cameras:
                self.camera_combo.set(str(available_cameras[0]))

            # Show popup with camera list
            camera_list = "\n".join(camera_details)
            messagebox.showinfo(
                "Cameras Found",
                f"Found {len(available_cameras)} camera(s):\n\n"
                f"{camera_list}\n\n"
                f"Selected: /dev/video{self.camera_combo.get()}\n\n"
                f"Tip: ZStar camera is usually /dev/video3 or /dev/video4"
            )
        else:
            self.camera_status_label.config(
                text="No cameras found",
                foreground='red'
            )
            messagebox.showwarning(
                "No Cameras Found",
                "No camera devices were detected.\n\n"
                "Please check:\n"
                "1. Camera is properly connected\n"
                "2. Try a different USB port\n"
                "3. Check dmesg for device info"
            )

        self.log(f"Scan complete: {len(available_cameras)} camera(s) found")

    def test_camera(self):
        """Test the selected camera index"""
        try:
            index = int(self.camera_index_var.get())
            self.log(f"Testing camera at index {index}...")

            cap = cv2.VideoCapture(index)

            if not cap.isOpened():
                self.camera_status_label.config(
                    text=f"Failed to open /dev/video{index}",
                    foreground='red'
                )
                messagebox.showerror(
                    "Camera Test Failed",
                    f"Could not open camera at /dev/video{index}\n\n"
                    "Please try a different index or check camera connection."
                )
                cap.release()
                return

            # Try to get a frame
            ret, frame = cap.read()
            if not ret:
                self.camera_status_label.config(
                    text=f"Failed to capture frame",
                    foreground='red'
                )
                messagebox.showerror(
                    "Camera Test Failed",
                    f"Camera opened but failed to capture frames.\n\n"
                    "The camera may be in use by another application\n"
                    "or there may be a driver issue."
                )
                cap.release()
                return

            # Get camera properties
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = cap.get(cv2.CAP_PROP_FPS)

            cap.release()

            self.camera_status_label.config(
                text=f"Camera OK: {width}x{height} @ {fps:.1f}fps",
                foreground='green'
            )

            self.log(f"Camera test successful: /dev/video{index} - {width}x{height} @ {fps:.1f}fps")

            messagebox.showinfo(
                "Camera Test Successful",
                f"Camera at /dev/video{index} is working!\n\n"
                f"Resolution: {width}x{height}\n"
                f"FPS: {fps:.1f}\n\n"
                f"Click 'Save Camera Setting' to use this camera."
            )

        except ValueError:
            messagebox.showerror("Invalid Index", "Please enter a valid camera index (0-7)")
            self.log("Invalid camera index entered")
        except Exception as e:
            self.camera_status_label.config(
                text=f"Error: {str(e)}",
                foreground='red'
            )
            self.log(f"Camera test error: {e}")

    def save_camera_setting(self):
        """Save camera index to configuration file"""
        try:
            index = self.camera_index_var.get()
            config = {
                'camera_index': int(index),
                'timestamp': datetime.now().isoformat()
            }

            config_file = '/home/koogs/Documents/5DOF_Robotic_Arm_Vision/calibration/camera_config.json'
            with open(config_file, 'w') as f:
                json.dump(config, f, indent=2)

            self.camera_status_label.config(
                text=f"Saved: /dev/video{index}",
                foreground='green'
            )
            self.log(f"Camera setting saved: /dev/video{index}")
            messagebox.showinfo(
                "Setting Saved",
                f"Camera configuration saved!\n\n"
                f"Device: /dev/video{index}\n"
                f"File: camera_config.json\n\n"
                f"The camera will use this setting next time."
            )

        except Exception as e:
            self.log(f"Error saving camera config: {e}")
            messagebox.showerror("Save Error", f"Could not save camera setting:\n{e}")

    def load_camera_setting(self):
        """Load camera index from configuration file"""
        try:
            config_file = '/home/koogs/Documents/5DOF_Robotic_Arm_Vision/calibration/camera_config.json'

            if not os.path.exists(config_file):
                self.camera_status_label.config(
                    text="No saved config found",
                    foreground='orange'
                )
                messagebox.showinfo(
                    "No Saved Config",
                    "No camera configuration file found.\n\n"
                    "Use 'Find' to detect cameras,\n"
                    "then 'Save Camera Setting' to save."
                )
                return

            with open(config_file, 'r') as f:
                config = json.load(f)

            index = config.get('camera_index', 3)
            self.camera_index_var.set(str(index))
            self.camera_status_label.config(
                text=f"Loaded: /dev/video{index}",
                foreground='green'
            )
            self.log(f"Camera setting loaded: /dev/video{index}")

        except Exception as e:
            self.log(f"Error loading camera config: {e}")
            messagebox.showerror("Load Error", f"Could not load camera setting:\n{e}")

    def set_camera_for_calibration(self):
        """Set the selected camera and restart calibration preview"""
        try:
            index = int(self.camera_index_var.get())
            self.log(f"Setting camera for calibration: /dev/video{index}")

            # Confirm with user
            confirm = messagebox.askyesno(
                "Set Camera",
                f"Use /dev/video{index} for grid calibration?\n\n"
                f"This will restart the camera preview.\n\n"
                f"Make sure this is the correct camera for your ZStar."
            )

            if not confirm:
                return

            # Save the setting
            config = {
                'camera_index': index,
                'timestamp': datetime.now().isoformat()
            }
            config_file = '/home/koogs/Documents/5DOF_Robotic_Arm_Vision/calibration/camera_config.json'
            with open(config_file, 'w') as f:
                json.dump(config, f, indent=2)

            # Restart calibration preview with new camera
            self.camera_status_label.config(
                text=f"Using /dev/video{index}",
                foreground='green'
            )
            self.log(f"Restarting calibration preview with /dev/video{index}...")

            # Switch to calibration tab and restart preview
            self.notebook.select(1)  # Select calibration tab
            self.start_calib_preview()

            messagebox.showinfo(
                "Camera Set",
                f"Camera set to /dev/video{index}\n\n"
                f"Calibration preview restarted.\n"
                f"Check the camera preview to verify."
            )

        except ValueError:
            messagebox.showerror("Invalid Index", "Please select a valid camera index (0-7)")
            self.log("Invalid camera index for calibration")
        except Exception as e:
            self.log(f"Error setting camera: {e}")
            messagebox.showerror("Error", f"Could not set camera:\n{e}")


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
        self.log(f"Connected to {port}")

        # ALWAYS go to rest position when (re)connected - prevents struggling
        self.log("Moving to rest position (prevents struggling)...")
        self.go_to_rest()

        # Start monitoring for communication errors
        self.root.after(5000, self._monitor_communication)

    def _on_connect_error(self, e):
        """Called when connection fails"""
        self.is_connected = False
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Status: Disconnected", foreground='red')
        self.log(f"Connection failed: {e}")
        messagebox.showerror("Connection Error", f"Could not connect to serial port:\n{e}\n\nPlease check:\n1. Arduino is powered on\n2. Correct port is selected\n3. Cable is properly connected")

    def _monitor_communication(self):
        """Monitor serial communication for errors and auto-recover"""
        if self.is_connected and self.serial_conn:
            try:
                # Check if port is still open
                if not self.serial_conn.is_open:
                    self.log("Serial port closed! Attempting recovery...")
                    self.disconnect()
                    self.root.after(1000, self.connect)
                    return

                # Check for pending errors (you can add more checks here)
                # For now, just schedule next check
                self.root.after(5000, self._monitor_communication)
            except Exception as e:
                self.log(f"Communication error detected: {e}")
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
        self.log("Recovery initiated...")
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
                self.log("Arduino reconnected! Going to rest position...")
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
                    self.log(f"Warning: Base at {new_val}° (safe range: 10-170°)")

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
                self.root.after(0, lambda: self.log(f"Invalid angle {angle}° for {JOINT_NAMES[idx]} (must be {MIN_ANGLE}-{MAX_ANGLE})"))
                return

            # Warn about base servo dead zone (but don't block)
            if idx == 0 and BASE_WARNING_MIN <= angle <= BASE_WARNING_MAX:
                self.root.after(0, lambda a=angle: self.log(f"WARNING: Base {a}° is in danger zone (110-120°) - May malfunction!"))

            command = f"{idx} {angle}\n"
            self.send_to_arduino(command)

            # Update UI from main thread
            self.root.after(0, lambda idx=idx, angle=angle: self.log(f"Sent: {JOINT_NAMES[idx]} -> {angle}°"))
        except ValueError:
            self.root.after(0, lambda: self.log(f"Invalid angle value"))
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
                self.log(f"Invalid angle {angle}° for {JOINT_NAMES[i]} (must be {MIN_ANGLE}-{MAX_ANGLE})")
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
            self.log(f"[Timer]️ Sending all servos at {time.time():.3f}")
            self.send_multi_move(angles)
            self.log(f"Sent all servos simultaneously: {angles}")
        except ValueError:
            messagebox.showerror("Error", "All angles must be valid numbers")

    def copy_current_angles(self):
        """Copy current manual control angles to clipboard"""
        angles = [int(self.input_boxes[i].get()) for i in range(5)]
        self.step_clipboard = [angles]
        self.log(f"Copied current angles: {angles}")
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

        self.log(f"Pasted to Manual Control: {angles}")
        messagebox.showinfo("Pasted!", f"Angles pasted to Manual Control:\n{angles}\n\nClick 'Send' on any servo to move to this position")

    # Global Base Adjustment Methods
    def global_base_adjust(self, amount):
        """Adjust all base angles in all sequences by a fixed amount"""
        try:
            self.base_adjust_var.set(str(amount))
            self.global_base_adjust_custom()
        except Exception as e:
            self.log(f"Base adjustment error: {e}")

    def global_base_adjust_custom(self):
        """Apply global base adjustment to all sequences"""
        try:
            amount = int(self.base_adjust_var.get())

            # Confirm with user
            confirm = messagebox.askyesno(
                "Confirm Global Base Adjustment",
                f"Adjust ALL base angles in ALL sequences by {amount:+d}°?\n\n"
                f"This will modify the saved sequences file.\n"
                f"Base angles will be clamped to 0-180°."
            )

            if not confirm:
                return

            # Load sequences
            if not os.path.exists(SEQUENCES_FILE):
                messagebox.showerror("Error", "Sequences file not found")
                return

            with open(SEQUENCES_FILE, 'r') as f:
                sequences = json.load(f)

            # Count adjustments
            total_adjusted = 0
            modified_sequences = []

            for seq_name, steps in sequences.items():
                seq_modified = False
                for step in steps:
                    if 'angles' in step and len(step['angles']) >= 1:
                        old_base = step['angles'][0]
                        new_base = max(0, min(180, old_base + amount))  # Clamp to 0-180
                        if new_base != old_base:
                            step['angles'][0] = new_base
                            seq_modified = True
                            total_adjusted += 1

                if seq_modified:
                    modified_sequences.append(seq_name)

            # Save modified sequences
            with open(SEQUENCES_FILE, 'w') as f:
                json.dump(sequences, f, indent=2)

            # Update status
            if total_adjusted > 0:
                self.base_adjust_status.config(
                    text=f"Applied: {amount:+d}° to {total_adjusted} steps in {len(modified_sequences)} sequences",
                    foreground='green'
                )
                self.log(f"Global base adjustment: {amount:+d}° applied to {total_adjusted} steps")
                if modified_sequences:
                    self.log(f"Modified sequences: {', '.join(modified_sequences[:5])}{'...' if len(modified_sequences) > 5 else ''}")
                messagebox.showinfo(
                    "Base Adjustment Complete",
                    f"Successfully adjusted {total_adjusted} base angles by {amount:+d}°\n"
                    f"Modified sequences: {len(modified_sequences)}"
                )
            else:
                self.base_adjust_status.config(
                    text=f"No changes needed (amount: {amount:+d}°)",
                    foreground='gray'
                )
                self.log(f"Global base adjustment: No changes needed")

            # Reload sequences
            self.load_sequences()

        except ValueError:
            messagebox.showerror("Error", "Please enter a valid number")
        except Exception as e:
            self.log(f"Base adjustment error: {e}")
            messagebox.showerror("Error", f"Failed to adjust base angles: {e}")

    # Auto-Offset Ratio Methods (for grid alignment)
    def adjust_offset_ratio(self, amount):
        """Adjust offset ratio by amount"""
        current = self.offset_ratio_var.get()
        new_ratio = current + amount
        # Clamp to reasonable range (0.5 to 10.0)
        new_ratio = max(0.5, min(10.0, new_ratio))
        self.offset_ratio_var.set(new_ratio)
        self.update_offset_ratio_display()
        self.log(f"Offset ratio adjusted: 1:{new_ratio:.1f}")

    def save_offset_ratio(self):
        """Save offset ratio setting"""
        try:
            ratio = self.offset_ratio_var.get()
            ratio = max(0.5, min(10.0, ratio))
            self.offset_ratio_var.set(ratio)
            self.update_offset_ratio_display()
            self.log(f"Offset ratio set to 1:{ratio:.1f} (for every {ratio:.1f}° error, adjust 1°)")
        except Exception as e:
            self.log(f"Error setting ratio: {e}")

    def update_offset_ratio_display(self):
        """Update offset ratio status labels"""
        ratio = self.offset_ratio_var.get()
        if ratio < 0.5:
            sensitivity = "Very High"
        elif ratio < 1.0:
            sensitivity = "High"
        elif ratio < 2.0:
            sensitivity = "Normal"
        elif ratio < 3.0:
            sensitivity = "Medium"
        elif ratio < 5.0:
            sensitivity = "Low"
        else:
            sensitivity = "Very Low"

        status_text = f"Ratio: 1:{ratio:.1f} ({sensitivity} sensitivity)"
        if hasattr(self, 'offset_ratio_status'):
            self.offset_ratio_status.config(text=status_text, foreground='blue')
        if hasattr(self, 'calib_offset_ratio_status'):
            self.calib_offset_ratio_status.config(text=status_text, foreground='blue')

    def apply_offset_to_sequence_step(self, angles, step_index, total_steps):
        """Apply auto-offset to base angle only for steps 3 and 4 of sequence

        Args:
            angles: List of 5 servo angles [base, shoulder, elbow, wrist, gripper]
            step_index: Current step index (0-based)
            total_steps: Total number of steps in sequence

        Returns:
            Modified angles list with offset applied to base if step 3 or 4
        """
        # Only apply offset to steps 3 and 4 (indices 2 and 3)
        # These are typically the pickup and lift steps
        if step_index not in [2, 3]:
            return angles

        # Check if we have valid detection for auto-offset
        if not hasattr(self, 'last_detected_cell') or not self.last_detected_cell:
            return angles

        # Calculate offset based on detected position error
        obj = self.last_detected_cell
        obj_cell = obj.get('cell', '?')
        obj_cx = obj.get('cx', 0)

        if obj_cell == '?' or obj_cx == 0:
            return angles

        # Get expected center
        expected_center = self.get_cell_center(obj_cell)
        if not expected_center:
            return angles

        exp_cx, exp_cy = expected_center

        # Check if we have valid detection for auto-offset
        if not hasattr(self, 'last_detected_cell') or not self.last_detected_cell:
            return angles

        # Calculate offset based on detected position error
        obj = self.last_detected_cell
        obj_cell = obj.get('cell', '?')
        obj_cx = obj.get('cx', 0)

        if obj_cell == '?' or obj_cx == 0:
            return angles

        # Get expected center
        expected_center = self.get_cell_center(obj_cell)
        if not expected_center:
            return angles

        exp_cx, exp_cy = expected_center

        # Use the AVERAGED suggestion from auto_offset_suggestions for consistency
        # This ensures the applied offset matches what was shown to the user
        if hasattr(self, 'auto_offset_suggestions') and len(self.auto_offset_suggestions) > 0:
            # Calculate average offset from all samples (same as display)
            avg_offset = sum(s['offset'] for s in self.auto_offset_suggestions) / len(self.auto_offset_suggestions)
            angular_offset = avg_offset
            self.log(f"[DEBUG] Applied offset: {angular_offset:+.1f}° (avg of {len(self.auto_offset_suggestions)} samples: {[s['offset'] for s in self.auto_offset_suggestions]})")
        else:
            # Fallback: calculate from current detection
            # Calculate pixel offset
            dx = exp_cx - obj_cx  # Positive = object is to the right (need to decrease base angle to turn right)

            # Convert to angular offset using ratio setting
            ratio = self.offset_ratio_var.get()
            pixels_per_degree_x = 4.0  # Increased sensitivity (was 8.0) - smaller = more sensitive

            # Calculate angular adjustment: object right = decrease angle (negative offset)
            angular_offset = dx / (pixels_per_degree_x * ratio)

        # Clamp to reasonable range (increased from ±10° to ±20° for more sensitivity)
        angular_offset = max(-20.0, min(20.0, angular_offset))

        # Apply offset to base angle only (index 0)
        modified = angles.copy()
        new_base = int(modified[0] + angular_offset)
        # Clamp to valid range
        new_base = max(0, min(180, new_base))
        modified[0] = new_base

        # Store the offset for consistent display across steps 3 and 4
        self._last_applied_offset = angular_offset

        return modified

    # Auto-Offset Suggestion Methods
    def toggle_auto_offset_suggestions(self):
        """Enable or disable auto-offset suggestions"""
        enabled = self.auto_offset_enabled_var.get()
        if enabled:
            self.log("Auto-offset suggestions ENABLED")
            self.analyze_offset_suggestion()
        else:
            self.log("Auto-offset suggestions DISABLED")
            self.auto_offset_suggest_label.config(text="No suggestion available", foreground='gray')
            self.auto_offset_confidence_label.config(text="")
            if hasattr(self, 'manual_auto_offset_label'):
                self.manual_auto_offset_label.config(text="No suggestion available", foreground='gray')
                self.manual_auto_offset_conf_label.config(text="")

    def analyze_offset_suggestion(self):
        """Analyze detected object positions and suggest base offset"""
        if not self.auto_offset_enabled_var.get():
            # Still allow manual analysis from Manual tab
            pass

        if not self.is_calibrated or len(self.all_points) < 25:
            msg = "Grid not calibrated - cannot analyze"
            self.auto_offset_suggest_label.config(text=msg, foreground='orange')
            if hasattr(self, 'manual_auto_offset_label'):
                self.manual_auto_offset_label.config(text=msg, foreground='orange')
            return

        if not hasattr(self, 'last_detected_cell') or not self.last_detected_cell:
            msg = "No object detected - place trash on grid"
            self.auto_offset_suggest_label.config(text=msg, foreground='gray')
            self.auto_offset_confidence_label.config(text="")
            if hasattr(self, 'manual_auto_offset_label'):
                self.manual_auto_offset_label.config(text=msg, foreground='gray')
                self.manual_auto_offset_conf_label.config(text="")
            return

        # Get object info
        obj = self.last_detected_cell
        obj_cx, obj_cy = obj.get('cx', 0), obj.get('cy', 0)
        obj_cell = obj.get('cell', '?')
        obj_w, obj_h = obj.get('w', 0), obj.get('h', 0)

        if obj_cell == '?' or obj_cx == 0:
            msg = "Object position uncertain"
            self.auto_offset_suggest_label.config(text=msg, foreground='orange')
            if hasattr(self, 'manual_auto_offset_label'):
                self.manual_auto_offset_label.config(text=msg, foreground='orange')
            return

        # Calculate expected cell center from calibrated grid
        expected_center = self.get_cell_center(obj_cell)
        if not expected_center:
            self.auto_offset_suggest_label.config(
                text=f"Cannot calculate center for {obj_cell}",
                foreground='orange'
            )
            return

        exp_cx, exp_cy = expected_center

        # Calculate offset from expected center
        dx = exp_cx - obj_cx  # Positive = object is to the right (need negative offset to turn right)
        dy = obj_cy - exp_cy  # Positive = object is below

        # Convert pixel offset to angular offset (increased sensitivity)
        pixels_per_degree_x = 4.0  # Increased sensitivity (was 8.0) - smaller = more sensitive

        # Calculate suggested base offset - THIS IS THE FINAL APPLIED OFFSET (includes ratio)
        # If object is right of center (dx > 0), need to rotate base right (negative offset = lower angle)
        # If object is left of center (dx < 0), need to rotate base left (positive offset = higher angle)
        ratio = self.offset_ratio_var.get()
        suggested_offset = dx / (pixels_per_degree_x * ratio)

        # Clamp to reasonable range (increased from ±10° to ±20° for more sensitivity)
        suggested_offset = max(-20.0, min(20.0, suggested_offset))

        # Calculate confidence based on how centered the object is
        cell_width = self.get_cell_width(obj_cell)
        if cell_width and cell_width > 0:
            # How far from center as fraction of cell width
            normalized_offset = abs(dx) / (cell_width / 2)
            confidence = max(0, 1.0 - normalized_offset)
        else:
            confidence = 0.5

        # Store suggestion
        self.auto_offset_suggestions.append({
            'offset': suggested_offset,
            'confidence': confidence,
            'cell': obj_cell,
            'dx': dx,
            'dy': dy,
            'timestamp': time.time()
        })

        # Keep only last 10 samples for better accuracy
        if len(self.auto_offset_suggestions) > 10:
            self.auto_offset_suggestions = self.auto_offset_suggestions[-10:]

        # Calculate average suggestion
        avg_offset = sum(s['offset'] for s in self.auto_offset_suggestions) / len(self.auto_offset_suggestions)
        avg_confidence = sum(s['confidence'] for s in self.auto_offset_suggestions) / len(self.auto_offset_suggestions)

        # Update UI (both Calibration and Manual tabs)
        if abs(avg_offset) < 0.5:
            msg = f"Alignment looks good! (offset: {avg_offset:+.1f}°)"
            color = 'green'
        else:
            # For this servo: positive = higher angle = turns LEFT, negative = lower angle = turns RIGHT
            direction = "left" if avg_offset > 0 else "right"
            msg = f"Suggest: {avg_offset:+.1f}° base ({direction})"
            color = 'blue' if avg_confidence > 0.6 else 'orange'

        self.auto_offset_suggest_label.config(text=msg, foreground=color)
        if hasattr(self, 'manual_auto_offset_label'):
            self.manual_auto_offset_label.config(text=msg, foreground=color)

        # Update confidence display with sample count
        conf_text = f"Confidence: {avg_confidence*100:.0f}% ({len(self.auto_offset_suggestions)} samples)"
        if avg_confidence > 0.7:
            conf_color = 'green'
        elif avg_confidence > 0.4:
            conf_color = 'orange'
        else:
            conf_color = 'gray'
        self.auto_offset_confidence_label.config(text=conf_text, foreground=conf_color)
        if hasattr(self, 'manual_auto_offset_conf_label'):
            self.manual_auto_offset_conf_label.config(text=conf_text, foreground=conf_color)

        self.log(f"Auto-offset: {avg_offset:+.1f}° (confidence: {avg_confidence*100:.0f}%, {len(self.auto_offset_suggestions)} samples)")

    def get_cell_center(self, cell):
        """Get the expected center coordinates of a cell"""
        if not self.is_calibrated or len(self.all_points) < 25:
            return None

        try:
            row = ord(cell[0]) - ord('A')
            col = int(cell[1]) - 1

            if not (0 <= row < 4 and 0 <= col < 4):
                return None

            # Get 4 corners of the cell
            idx1 = row * 5 + col
            idx2 = idx1 + 1
            idx3 = idx1 + 5
            idx4 = idx3 + 1

            if idx4 >= len(self.all_points):
                return None

            # Calculate center as average of 4 corners
            x1, y1 = self.all_points[idx1]
            x2, y2 = self.all_points[idx2]
            x3, y3 = self.all_points[idx3]
            x4, y4 = self.all_points[idx4]

            center_x = (x1 + x2 + x3 + x4) / 4
            center_y = (y1 + y2 + y3 + y4) / 4

            return (center_x, center_y)
        except:
            return None

    def get_cell_width(self, cell):
        """Get the approximate width of a cell in pixels"""
        if not self.is_calibrated or len(self.all_points) < 25:
            return None

        try:
            row = ord(cell[0]) - ord('A')
            col = int(cell[1]) - 1

            if not (0 <= row < 4 and 0 <= col < 4):
                return None

            # Get left and right edge points
            idx_left = row * 5 + col
            idx_right = idx_left + 1

            if idx_right >= len(self.all_points):
                return None

            x_left = self.all_points[idx_left][0]
            x_right = self.all_points[idx_right][0]

            return abs(x_right - x_left)
        except:
            return None

    def apply_suggested_offset(self):
        """Apply the currently suggested offset"""
        if not self.auto_offset_suggestions:
            messagebox.showwarning("Warning", "No suggestion available to apply")
            return

        # Calculate average suggestion
        avg_offset = sum(s['offset'] for s in self.auto_offset_suggestions) / len(self.auto_offset_suggestions)
        avg_confidence = sum(s['confidence'] for s in self.auto_offset_suggestions) / len(self.auto_offset_suggestions)

        if avg_confidence < 0.3:
            confirm = messagebox.askyesno(
                "Low Confidence",
                f"The suggestion has low confidence ({avg_confidence*100:.0f}%).\n\n"
                f"Suggested offset: {avg_offset:+.1f}°\n\n"
                f"Apply anyway?"
            )
            if not confirm:
                return

        # Apply the offset
        self.base_offset_var.set(avg_offset)
        self.base_offset_enabled = True
        self.offset_enabled_var.set(True)
        self.update_offset_display()

        self.log(f"Applied suggested offset: {avg_offset:+.1f}° (confidence: {avg_confidence*100:.0f}%)")
        messagebox.showinfo(
            "Offset Applied",
            f"Applied offset: {avg_offset:+.1f}°\n"
            f"Confidence: {avg_confidence*100:.0f}%\n\n"
            f"The base offset is now enabled and will be applied to all movements."
        )

    def send_to_arduino(self, command):
        """Send command to Arduino with proper buffer management"""
        if self.is_connected and self.serial_conn and self.serial_conn.is_open:
            try:
                # Clear both buffers before sending
                if self.serial_conn.in_waiting > 0:
                    self.serial_conn.reset_input_buffer()
                if self.serial_conn.out_waiting > 0:
                    self.serial_conn.reset_output_buffer()

                # Send command
                self.serial_conn.write(command.encode())
                self.serial_conn.flush()

                # Wait for Arduino to process (critical for preventing buffer buildup)
                time.sleep(0.1)
            except Exception as e:
                self.log(f"Serial error: {e}")

    def start_camera_thread(self):
        """Start camera preview in separate thread"""
        def camera_loop():
            self.camera_running = True
            self.log(f"Camera thread started at {time.time():.3f}")
            while self.camera_running:
                try:
                    loop_start = time.time()
                    self.update_calib_preview_once()
                    loop_elapsed = time.time() - loop_start
                    if loop_elapsed > 0.08:
                        self.log(f"Camera frame took {loop_elapsed:.3f}s")
                    # Sleep to maintain ~10 FPS (1 frame every 0.1 seconds)
                    time.sleep(max(0, 0.1 - loop_elapsed))
                except Exception as e:
                    self.log(f"Camera error: {e}")
                    time.sleep(0.5)

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

            # Apply blur to reduce camera noise
            frame = cv2.GaussianBlur(frame, (5, 5), 0)

            # Use cached canvas size (updated periodically from main thread)
            canvas_width = self.canvas_width
            canvas_height = self.canvas_height

            if canvas_width < 2 or canvas_height < 2:
                return

            # Get frame dimensions
            frame_height, frame_width = frame.shape[:2]

            # Calculate scale to fit canvas (maintain aspect ratio)
            scale_x = canvas_width / frame_width
            scale_y = canvas_height / frame_height
            scale = min(scale_x, scale_y)

            new_width = int(frame_width * scale)
            new_height = int(frame_height * scale)

            if new_width < 1 or new_height < 1:
                return

            # Resize
            display_frame = cv2.resize(frame, (new_width, new_height))

            # Create black background
            bg = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)

            # Center the image
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

            # Object detection - run every 4th frame
            self.frame_count += 1
            if self.detect_in_calib_var.get() and hasattr(self, 'empty_grid') and self.empty_grid is not None:
                if self.frame_count % 4 == 0:
                    objects = self.detect_objects(frame)
                    if objects:
                        self.last_detection_time = time.time()
                        detected = max(objects, key=lambda o: o['area'])
                        detected_cell_name = self.find_cell(detected['cx'], detected['cy'])
                        detected['cell'] = detected_cell_name

                        if detected_cell_name == self.cell_confirmed:
                            self.cell_confirmation_count += 1
                        else:
                            if self.cell_confirmation_count >= self.cell_hysteresis_threshold:
                                self.cell_confirmed = detected_cell_name
                                self.cell_confirmation_count = 1
                                self.log(f"Object detected in {detected_cell_name}")
                                # Clear classification history for new object position
                                self.classification_history = []
                                self.classification_confirmed = None
                            else:
                                self.cell_confirmation_count += 1
                                detected['cell'] = self.cell_confirmed if self.cell_confirmed else detected_cell_name

                        self.last_detected_cell = detected
                    else:
                        # No objects detected - clear classification history
                        self.classification_history = []
                        self.classification_confirmed = None
                        self.last_detected_cell = None

                has_detection = hasattr(self, 'last_detected_cell') and self.last_detected_cell is not None
                if has_detection and time.time() - self.last_detection_time < self.detection_timeout:
                    obj = self.last_detected_cell
                    disp_x = int(obj['x'] * scale) + x_offset
                    disp_y = int(obj['y'] * scale) + y_offset
                    disp_w = int(obj['w'] * scale)
                    disp_h = int(obj['h'] * scale)

                    # Classify object if waste classification is enabled
                    class_name = None
                    confidence = 0.0
                    box_color = (0, 255, 255)  # Default yellow

                    if self.waste_classify_var.get() and self.classifier_loaded:
                        # Classify using original frame coordinates
                        class_name, confidence = self.classify_detected_object(frame, obj)
                        self.classification_results[obj.get('cell', '?')] = (class_name, confidence)

                        # Set color based on classification
                        if class_name == 'biodegradable':
                            box_color = (0, 255, 0)  # Green
                        elif class_name == 'non-biodegradable':
                            box_color = (0, 0, 255)  # Red
                        else:
                            box_color = (0, 255, 255)  # Yellow (unknown)
                    else:
                        box_color = (0, 255, 255)  # Yellow when classification disabled

                    cv2.rectangle(bg, (disp_x, disp_y), (disp_x+disp_w, disp_y+disp_h), box_color, 2)

                    cell = obj.get('cell', '?')
                    area = obj.get('area', 0)
                    solidity = obj.get('solidity', 0)
                    threshold = self.threshold_var.get()

                    # Build info text
                    info_text = f"{cell} | {area:.0f}px | S:{solidity:.2f}"
                    if class_name and confidence > 0:
                        info_text += f" | {class_name[:4]}:{confidence:.0%}"
                    info_text += f" | Th:{threshold}"

                    cv2.putText(bg, info_text, (disp_x, disp_y-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

                    # Update detection status label
                    if class_name and confidence > 0:
                        self.detection_status_label.config(
                            text=f"{cell}: {class_name} ({confidence:.0%})",
                            foreground='green' if class_name == 'biodegradable' else 'red'
                        )
                    else:
                        self.detection_status_label.config(
                            text=f"{cell} detected",
                            foreground='yellow'
                        )

            # Convert BGR to RGB
            bg = cv2.cvtColor(bg, cv2.COLOR_BGR2RGB)

            img = Image.fromarray(bg)
            photo = ImageTk.PhotoImage(image=img)

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
            self.log("No sequence selected")
            return

        seq_name = self.seq_listbox.get(selection[0])
        if seq_name not in self.sequences:
            self.log(f"Sequence '{seq_name}' not found")
            return

        if not self.is_connected:
            self.log("Not connected to Arduino!")
            return

        # CRITICAL: Cancel all pending after callbacks (prevents event queue buildup)
        self._cancel_pending_callbacks()

        # Set stop flag for any currently playing sequence
        self._stop_sequence_flag = True
        time.sleep(0.1)  # Give thread time to stop

        # Start new sequence in background thread
        self.log(f"▶ Playing sequence: {seq_name}")
        self._stop_sequence_flag = False
        thread = threading.Thread(target=self._play_sequence_thread, args=(seq_name,), daemon=True)
        self._current_sequence_thread = thread
        thread.start()

    def _cancel_pending_callbacks(self):
        """Cancel all pending root.after() callbacks"""
        for after_id in self._after_ids:
            try:
                self.root.after_cancel(after_id)
            except:
                pass
        self._after_ids = []

    def _safe_after(self, delay, callback):
        """Schedule callback and track it for cancellation"""
        after_id = self.root.after(delay, callback)
        self._after_ids.append(after_id)
        return after_id

    def _play_sequence_thread(self, seq_name):
        """Background thread for playing sequence with proper cleanup"""
        try:
            self.log(f"[Timer]️ Starting sequence {seq_name} at {time.time():.3f}")

            for i, step in enumerate(self.sequences[seq_name]):
                # Check if we should stop
                if getattr(self, '_stop_sequence_flag', False):
                    self.log("Sequence stopped by user")
                    break

                if 'angles' in step:
                    # Send multi-move command
                    angles = step['angles']
                    # Apply auto-offset to steps 3 and 4 only (indices 2 and 3)
                    angles_with_offset = self.apply_offset_to_sequence_step(angles, i, len(self.sequences[seq_name]))

                    # Use 15ms delay for fast transitions: step 2->3 (i=1) and step 4->5 (i=3)
                    if i in [1, 3]:
                        delay = 15
                    else:
                        delay = step.get('delay', 1000)
                    step_start = time.time()

                    # For steps 3 and 5 (indices 2 and 4), use 3x slower speed
                    current_speed = int(self.speed_var.get()) if hasattr(self, 'speed_var') else DEFAULT_SPEED
                    if i in [2, 4]:  # Step 3 or Step 5 - use 3x slower speed
                        slow_speed = current_speed * 3
                        speed_command = f"99 {slow_speed}\n"
                        self.send_to_arduino(speed_command)
                        self._safe_after(0, lambda s=slow_speed: self.log(f"  → Speed changed to {s}ms/deg (3x slower for step {i+1})"))

                    # Update step 3 & 4 base value display (show same offset for both)
                    if i in [2, 3]:  # Steps 3 or 4
                        offset = getattr(self, '_last_applied_offset', 0)
                        base_with_offset = angles_with_offset[0]
                        self._safe_after(0, lambda b=base_with_offset, o=offset: self.step3_base_label.config(
                            text=f"Step 3 (Pickup): {b:.0f}° (base: {angles[0]:.0f}° + offset: {o:+.0f}°)"))
                        self._safe_after(0, lambda b=base_with_offset, o=offset: self.step4_base_label.config(
                            text=f"Step 4 (Lift): {b:.0f}° (base: {angles[0]:.0f}° + offset: {o:+.0f}°)"))

                    # Debug: Log what's being sent
                    self.log(f"[DEBUG] Step {i+1}: base={angles[0]:.0f}° + offset={self._last_applied_offset:+.0f}° = {angles_with_offset[0]:.0f}° → Arduino")

                    command = f"M {angles_with_offset[0]} {angles_with_offset[1]} {angles_with_offset[2]} {angles_with_offset[3]} {angles_with_offset[4]}\n"
                    self.send_to_arduino(command)
                    self._safe_after(0, lambda a=angles_with_offset: self.log(f"  → Sent: {a}"))

                    # Wait for servo movement (minimum 1.0 seconds to let Arduino execute)
                    wait_start = time.time()
                    while time.time() - wait_start < max(1.0, delay / 1000.0):
                        if getattr(self, '_stop_sequence_flag', False):
                            break
                        time.sleep(0.1)

                    # Restore original speed after steps 3 and 5 (indices 2 and 4)
                    if i in [2, 4]:  # After Step 3 or Step 5 completes
                        speed_command = f"99 {current_speed}\n"
                        self.send_to_arduino(speed_command)
                        self._safe_after(0, lambda s=current_speed: self.log(f"  → Speed restored to {s}ms/deg"))

                    step_elapsed = time.time() - step_start
                    if step_elapsed > 0.6:
                        self._safe_after(0, lambda s=i+1, t=step_elapsed: self.log(f"[Timer]️ Step {s} took {t:.3f}s"))

            self._safe_after(0, lambda: self.log(f"Sequence complete"))
        except Exception as ex:
            error_msg = str(ex)
            self._safe_after(0, lambda msg=error_msg: self.log(f"Error: {msg}"))
        finally:
            # Clean up thread reference
            if hasattr(self, '_current_sequence_thread'):
                self._current_sequence_thread = None

    def stop_sequence(self):
        """Stop sequence (placeholder)"""
        self._stop_sequence_flag = True
        self.log("Sequence stopped")
    
    # Calibration Methods
    def start_calib_preview(self):
        """Start calibration preview - uses configured camera index"""
        # Get camera index from configuration
        try:
            camera_index = int(self.camera_index_var.get())
        except (ValueError, AttributeError):
            camera_index = 3  # Default fallback

        self.log(f"Starting camera (device index: {camera_index})...")

        # Close any existing camera
        if hasattr(self, 'cap') and self.cap:
            self.cap.release()

        # Try opening camera with multiple methods (index and direct path)
        camera_opened = False

        # Method 1: Try by index first
        self.cap = cv2.VideoCapture(camera_index)
        if self.cap.isOpened():
            camera_opened = True
            self.log(f"Camera opened by index {camera_index}")

        # Method 2: Try direct device path (more reliable after reboot)
        if not camera_opened:
            device_path = f"/dev/video{camera_index}"
            self.log(f"Index {camera_index} failed, trying direct path: {device_path}")
            self.cap.release()
            self.cap = cv2.VideoCapture(device_path)
            if self.cap.isOpened():
                camera_opened = True
                self.log(f"Camera opened by direct path {device_path}")

        # Method 3: Try adjacent indices (Z-Star may move on replug)
        if not camera_opened:
            self.log(f"Device {camera_index} not available, trying {camera_index+1}...")
            self.cap.release()
            self.cap = cv2.VideoCapture(camera_index + 1)
            if self.cap.isOpened():
                camera_opened = True
                self.log(f"Camera opened at index {camera_index+1}")

        # Method 4: Try common Z-Star indices as fallback
        if not camera_opened:
            for fallback_idx in [0, 3, 4]:
                if fallback_idx != camera_index and fallback_idx != camera_index + 1:
                    self.log(f"Trying fallback device {fallback_idx}...")
                    self.cap.release()
                    self.cap = cv2.VideoCapture(fallback_idx)
                    if self.cap.isOpened():
                        camera_opened = True
                        self.log(f"Fallback camera opened at {fallback_idx}")
                        # Update UI to reflect the working camera
                        self.camera_index_var.set(str(fallback_idx))
                        break

        # Method 5: Scan all /dev/video* devices
        if not camera_opened:
            import glob
            for dev in glob.glob("/dev/video[0-9]"):
                self.log(f"Scanning {dev}...")
                self.cap.release()
                self.cap = cv2.VideoCapture(dev)
                if self.cap.isOpened():
                    camera_opened = True
                    dev_idx = dev.replace("/dev/video", "")
                    self.log(f"Camera found at {dev}")
                    self.camera_index_var.set(dev_idx)
                    break

        if camera_opened and self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.log(f"Camera opened successfully")
            self.update_calib_preview()
        else:
            self.log(f"ERROR: Camera not found at configured index {camera_index} or any fallback!")
            messagebox.showerror(
                "Camera Error",
                f"Could not open camera!\n\n"
                f"Tried: index {camera_index}, /dev/video{camera_index}, /dev/video{camera_index+1}, /dev/video0, /dev/video3, /dev/video4\n\n"
                f"Make sure:\n"
                f"1. Camera is plugged in\n"
                f"2. Try a different USB port\n"
                f"3. Use 'Find' button to detect available cameras\n"
                f"4. Check if another application is using the camera"
            )
    
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

            # If calibrated, draw grid overlay on every frame - scaled (THICK lines)
            if self.is_calibrated and len(self.all_points) >= 25:
                # Draw grid lines - scale all_points to display coordinates (THICK: 2px)
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

            # Object detection state is managed by camera thread only
            # This function only reads shared state for UI updates (buttons, labels)
            has_detection = hasattr(self, 'last_detected_cell') and self.last_detected_cell is not None

            # Update UI based on detection state (buttons, labels only - no drawing)
            if has_detection and time.time() - self.last_detection_time < self.detection_timeout:
                cell = self.last_detected_cell.get('cell', '?')

                # Check if this is a NEW cell detection (different from previous)
                prev_cell = getattr(self, '_prev_detection_cell', None)
                if prev_cell != cell:
                    # New cell detected - clear old samples and reset hysteresis
                    self.auto_offset_suggestions = []
                    self._prev_detection_cell = cell
                    self.cell_confirmed = None
                    self.cell_confirmation_count = 0
                    self.log(f"[AUTO-OFFSET] New cell {cell} detected - cleared samples & reset hysteresis")

                # Auto-offset suggestion analysis (when enabled)
                if self.auto_offset_enabled_var.get():
                    current_time = time.time()
                    if current_time - self.last_offset_analysis_time > self.offset_analysis_interval:
                        self.root.after(0, self.analyze_offset_suggestion)
                        self.last_offset_analysis_time = current_time

                # Check if cell has a sequence and is A, B, or C (not D)
                if cell in self.sequences and cell[0] in ['A', 'B', 'C']:
                    self.current_detection_cell = cell
                    self.pickup_btn.config(state='normal')

                    # Auto-pickup logic - use current averaged offset (no additional sampling wait)
                    if self.auto_pickup_enabled and self.auto_pickup_running:
                        if self.pending_pickup_cell == cell:
                            elapsed = time.time() - self.object_first_detected_time
                            if elapsed >= self.object_confirmation_delay:
                                # Calculate current averaged offset for logging
                                avg_offset = 0
                                if hasattr(self, 'auto_offset_suggestions') and len(self.auto_offset_suggestions) > 0:
                                    avg_offset = sum(s['offset'] for s in self.auto_offset_suggestions) / len(self.auto_offset_suggestions)
                                self.log(f"[AUTO] Object confirmed in {cell} after {elapsed:.1f}s - Starting pickup!")
                                self.log(f"[AUTO] Using averaged offset: {avg_offset:+.1f}° from {len(self.auto_offset_suggestions)} samples")
                                self.auto_pickup_running = False
                                self.pending_pickup_cell = None
                                self.root.after(0, self.auto_trigger_pickup)
                            else:
                                remaining = self.object_confirmation_delay - elapsed
                                # Show averaged offset, not _last_applied_offset
                                avg_offset = 0
                                if hasattr(self, 'auto_offset_suggestions') and len(self.auto_offset_suggestions) > 0:
                                    avg_offset = sum(s['offset'] for s in self.auto_offset_suggestions) / len(self.auto_offset_suggestions)
                                samples = len(self.auto_offset_suggestions)
                                offset_display = f"{avg_offset:+.0f}°" if samples > 0 else "0°"
                                self.detection_status_label.config(
                                    text=f"Auto: {cell} ({remaining:.1f}s) offset:{offset_display}",
                                    foreground='orange'
                                )
                                self.update_detection_display(f"{cell} ({remaining:.1f}s)")
                        else:
                            self.pending_pickup_cell = cell
                            self.object_first_detected_time = time.time()
                            self.log(f"[AUTO] Object detected in {cell}, waiting {self.object_confirmation_delay:.0f}s confirmation...")
                    else:
                        self.detection_status_label.config(
                            text=f"Object in {cell} - Ready to pickup!",
                            foreground='green'
                        )
                        self.update_detection_display(cell)
                else:
                    self.current_detection_cell = None
                    self.pickup_btn.config(state='disabled')
                    if self.auto_pickup_enabled and self.auto_pickup_running:
                        self.pending_pickup_cell = None
                        self.object_first_detected_time = 0
                    if cell and len(cell) > 0 and cell[0] in ['A', 'B', 'C']:
                        self.detection_status_label.config(
                            text=f"Object in {cell} (no sequence)",
                            foreground='orange'
                        )
                    else:
                        self.detection_status_label.config(
                            text=f"Object in {cell} (cell D not supported)",
                            foreground='orange'
                        )
                    self.update_detection_display(cell)
            else:
                # No object detected or timeout expired
                self.current_detection_cell = None
                self.pickup_btn.config(state='disabled')
                if self.auto_pickup_enabled and self.auto_pickup_running:
                    self.pending_pickup_cell = None
                    self.object_first_detected_time = 0
                    # Keep offset samples - they're still valid for next detection
                # Reset cell hysteresis after timeout expires
                if self.cell_confirmed and time.time() - self.last_detection_time > self.detection_timeout:
                    self.cell_confirmed = None
                    self.cell_confirmation_count = 0
                self.detection_status_label.config(
                    text="No object detected",
                    foreground='gray'
                )
                self.update_detection_display(None)

            # Convert BGR to RGB
            bg = cv2.cvtColor(bg, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(bg)
            photo = ImageTk.PhotoImage(image=img)

            # DO NOT update canvas here - camera thread handles all canvas updates
            # This function only handles UI state (buttons, labels)
            # self.calib_canvas.delete("all")  # REMOVED - causes flickering!
            # self.calib_canvas.create_image(0, 0, anchor='nw', image=photo)
            # self.calib_canvas.image = photo

            self.calib_canvas.after(500, self.update_calib_preview)  # 2 FPS for UI updates
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
        # Reset auto pickup state
        self.auto_pickup_enabled = False
        self.auto_pickup_running = False
        self.pending_pickup_cell = None
        self.auto_pickup_var.set(False)
        self.auto_pickup_chk.config(state='disabled')
        # Reset detection hysteresis state
        self.cell_confirmed = None
        self.cell_confirmation_count = 0
        self.last_detected_cell = None
        self.last_detection_time = 0
        # Reset offset samples
        self.auto_offset_suggestions = []
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
        
        # Calculate 25 points for 4x4 grid (5 rows x 5 columns of points)
        tl = np.array(self.corners[0], dtype=np.float32)
        tr = np.array(self.corners[1], dtype=np.float32)
        bl = np.array(self.corners[2], dtype=np.float32)
        br = np.array(self.corners[3], dtype=np.float32)

        self.all_points = []
        for row in range(5):  # 5 rows of points
            for col in range(5):  # 5 columns of points
                t = col / 4.0  # 0, 0.25, 0.5, 0.75, 1.0
                u = row / 4.0
                top = tl + (tr - tl) * t
                bottom = bl + (br - bl) * t
                point = top + (bottom - top) * u
                self.all_points.append((int(point[0]), int(point[1])))

        self.is_calibrated = True
        self.log(f"Grid calculated: {len(self.all_points)} points (4x4 grid = 16 cells)")

        # Grid overlay will now be drawn automatically by update_calib_preview

        messagebox.showinfo("Success", f"Grid calculated!\n\n{len(self.all_points)} points generated\n\nEmpty grid captured!\n\nGrid lines are now visible on the camera preview!\n\nGo to Auto Detection tab to test")

        # Enable auto pickup checkbox after calibration
        self.auto_pickup_chk.config(state='normal')
        self.log("Auto pickup enabled")

    def draw_grid_overlay(self):
        """Draw digital grid lines on calibration canvas - 4x4 grid (16 cells)"""
        try:
            if not self.is_calibrated:
                self.log("Cannot draw overlay: not calibrated")
                return

            if len(self.all_points) < 25:
                self.log(f"Cannot draw overlay: only {len(self.all_points)} points, need 25")
                return

            self.log(f"Drawing grid overlay with {len(self.all_points)} points (4x4 grid)...")

            # Get current frame from camera
            ret, frame = self.cap.read()
            if not ret:
                self.log("Cannot draw overlay: camera not available")
                return

            self.log("Camera frame obtained, drawing lines...")

            # Draw grid lines on frame
            # 5 horizontal lines (creating 4 rows)
            for row in range(5):
                # Draw horizontal line across the row
                start_idx = row * 5  # First point in row (5 points per row)
                end_idx = start_idx + 4  # Last point in row
                if end_idx < len(self.all_points):
                    pt1 = self.all_points[start_idx]
                    pt2 = self.all_points[end_idx]
                    self.log(f"  Horizontal {row}: {pt1} -> {pt2}")
                    cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            # 5 vertical lines (creating 4 columns)
            for col in range(5):
                # Draw vertical line down the column
                top_idx = col  # Top point in column
                bottom_idx = 20 + col  # Bottom point (4 rows down = 20 points)
                if bottom_idx < len(self.all_points):
                    pt1 = self.all_points[top_idx]
                    pt2 = self.all_points[bottom_idx]
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

            self.log("Grid overlay drawn successfully!")
        except Exception as e:
            self.log(f"Error drawing overlay: {e}")
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
            self.auto_pickup_enabled = False
            self.auto_pickup_running = False
            self.pending_pickup_cell = None
            self.pickup_btn.config(state='disabled')
            self.auto_pickup_chk.config(state='disabled')
            self.detection_status_label.config(text="No object detected", foreground='gray')

    def toggle_waste_classification(self):
        """Toggle waste classification on/off"""
        enabled = self.waste_classify_var.get()
        if enabled:
            if not self.classifier_loaded:
                self.log("Loading waste classifier...")
                try:
                    import sys
                    sys.path.insert(0, '/home/koogs/Documents/5DOF_Robotic_Arm_Vision')
                    from waste_classifier import WasteClassifier
                    self.waste_classifier = WasteClassifier()
                    self.classifier_loaded = True
                    self.log("Waste classifier loaded successfully")
                except Exception as e:
                    self.log(f"Failed to load waste classifier: {e}")
                    self.waste_classify_var.set(False)
                    messagebox.showerror("Error", f"Could not load waste classifier:\n{e}")
                    return
            else:
                self.log("Waste classification enabled")
        else:
            self.classification_results = {}
            self.classification_history = []
            self.classification_confirmed = None
            self.log("Waste classification disabled")

    def classify_detected_object(self, frame, obj):
        """Classify a detected object using the waste classifier with temporal smoothing

        Args:
            frame: Original camera frame (BGR)
            obj: Detection dict with x, y, w, h keys

        Returns:
            tuple: (class_name, confidence) or ('Unknown', 0.0)
        """
        if not self.classifier_loaded or self.waste_classifier is None:
            return 'Unknown', 0.0

        try:
            bbox = (obj['x'], obj['y'], obj['w'], obj['h'])
            class_name, confidence = self.waste_classifier.classify(frame, bbox)

            # Add to history
            self.classification_history.append((class_name, confidence))

            # Keep only last N classifications
            if len(self.classification_history) > self.classification_history_size:
                self.classification_history = self.classification_history[-self.classification_history_size:]

            # Majority vote from history
            bio_count = sum(1 for c, _ in self.classification_history if c == 'biodegradable')
            nonbio_count = sum(1 for c, _ in self.classification_history if c == 'non-biodegradable')
            total = len(self.classification_history)

            # Calculate average confidence for each class
            bio_conf = sum(conf for c, conf in self.classification_history if c == 'biodegradable') / max(bio_count, 1)
            nonbio_conf = sum(conf for c, conf in self.classification_history if c == 'non-biodegradable') / max(nonbio_count, 1)

            # Determine confirmed classification
            if bio_count >= self.classification_confirmation_threshold:
                self.classification_confirmed = ('biodegradable', bio_conf)
                if len(self.classification_history) == self.classification_history_size:
                    self.log(f"[Class] BIO confirmed: {bio_count}/{total} votes, conf={bio_conf:.0%}")
            elif nonbio_count >= self.classification_confirmation_threshold:
                self.classification_confirmed = ('non-biodegradable', nonbio_conf)
                if len(self.classification_history) == self.classification_history_size:
                    self.log(f"[Class] NON-BIO confirmed: {nonbio_count}/{total} votes, conf={nonbio_conf:.0%}")
            # If no majority, keep previous confirmed result (prevents flickering)
            # Only show 'Unknown' if history is too short
            elif total < 3:
                self.classification_confirmed = ('Unknown', 0.0)

            return self.classification_confirmed

        except Exception as e:
            self.log(f"Classification error: {e}")
            return 'Unknown', 0.0

    def init_camera_settings(self):
        """Initialize camera settings on startup"""
        try:
            # Read current values from camera
            import subprocess
            result = subprocess.run(
                ['v4l2-ctl', '--device=/dev/video0', '--list-ctrls'],
                capture_output=True, text=True, timeout=2
            )
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if 'exposure_time_absolute' in line and 'value=' in line:
                        try:
                            val = int(line.split('value=')[1].split()[0])
                            self.exposure_var.set(val)
                            self.exposure_value_label.config(text=f"Value: {val} (Auto)")
                        except: pass
                    if 'gamma' in line and 'value=' in line:
                        try:
                            val = int(line.split('value=')[1].split()[0])
                            self.gamma_var.set(val)
                            self.gamma_value_label.config(text=f"Value: {val}")
                        except: pass
                    if 'saturation' in line and 'value=' in line:
                        try:
                            val = int(line.split('value=')[1].split()[0])
                            self.saturation_var.set(val)
                            self.saturation_value_label.config(text=f"Value: {val}")
                        except: pass
                    if 'sharpness' in line and 'value=' in line:
                        try:
                            val = int(line.split('value=')[1].split()[0])
                            self.sharpness_var.set(val)
                            self.sharpness_value_label.config(text=f"Value: {val}")
                        except: pass
        except Exception as e:
            pass  # Silently ignore if v4l2-ctl fails

    def update_camera_exposure(self, val=None):
        """Update camera exposure using v4l2-ctl"""
        try:
            mode = self.exposure_mode_var.get()
            exposure = self.exposure_var.get()
            gamma = self.gamma_var.get()
            saturation = self.saturation_var.get()
            sharpness = self.sharpness_var.get()

            # Update labels
            mode_text = "(Auto)" if mode == "auto" else f"(Manual: {exposure})"
            self.exposure_value_label.config(text=f"Value: {exposure} {mode_text}")
            self.gamma_value_label.config(text=f"Value: {gamma}")
            self.saturation_value_label.config(text=f"Value: {saturation}")
            self.sharpness_value_label.config(text=f"Value: {sharpness}")

            # Disable/enable exposure slider based on mode
            if mode == "auto":
                self.exposure_scale.config(state='disabled')
            else:
                self.exposure_scale.config(state='normal')

            # Apply settings via v4l2-ctl
            import subprocess

            # Set exposure mode: 3=Auto, 1=Manual
            if mode == "auto":
                subprocess.run(
                    ['v4l2-ctl', '--device=/dev/video0',
                     '-c', 'auto_exposure=3'],
                    capture_output=True, timeout=2
                )
                self.log("Camera exposure set to AUTO")
            else:
                # First set to manual mode, then set exposure value
                subprocess.run(
                    ['v4l2-ctl', '--device=/dev/video0',
                     '-c', 'auto_exposure=1',
                     '-c', f'exposure_time_absolute={exposure}'],
                    capture_output=True, timeout=2
                )
                self.log(f"Camera exposure set to MANUAL: {exposure}")

            # Set gamma, saturation, sharpness (always applied)
            subprocess.run(
                ['v4l2-ctl', '--device=/dev/video0',
                 '-c', f'gamma={gamma}',
                 '-c', f'saturation={saturation}',
                 '-c', f'sharpness={sharpness}'],
                capture_output=True, timeout=2
            )

        except Exception as e:
            self.log(f"Camera setting error: {e}")

    def reset_camera_settings(self):
        """Reset camera settings to defaults"""
        self.exposure_mode_var.set("auto")
        self.exposure_var.set(2400)
        self.gamma_var.set(140)
        self.saturation_var.set(17)  # Default saturation
        self.sharpness_var.set(5)    # Default sharpness
        self.update_camera_exposure()
        self.log("Camera settings reset to defaults")

    def toggle_auto_pickup(self):
        """Toggle auto pickup mode"""
        self.auto_pickup_enabled = self.auto_pickup_var.get()

        if self.auto_pickup_enabled:
            self.auto_pickup_running = True
            self.pending_pickup_cell = None
            self.object_first_detected_time = 0
            # Keep existing offset samples - don't reset!
            self.log(f"[AUTO] Auto pickup ENABLED - {self.object_confirmation_delay:.0f}s confirmation, using live offset")
            self.detection_status_label.config(
                text="Auto mode: Waiting for object...",
                foreground='blue'
            )
        else:
            self.auto_pickup_running = False
            self.pending_pickup_cell = None
            self.object_first_detected_time = 0
            # Keep offset samples for next enable
            self.log("[AUTO] Auto pickup DISABLED")
            self.detection_status_label.config(
                text="Manual mode",
                foreground='gray'
            )

    def auto_trigger_pickup(self):
        """Automatically trigger pickup without confirmation dialog.
        If waste classification is enabled, classify object and choose
        original (BIO) or _NON variant accordingly."""
        if not self.current_detection_cell:
            return

        cell = self.current_detection_cell

        # Determine which sequence to execute based on waste classification
        execute_cell = cell

        if self.waste_classify_var.get() and self.classifier_loaded:
            # Classify the detected object
            if hasattr(self, 'last_detected_cell') and self.last_detected_cell:
                obj = self.last_detected_cell

                # Grab current frame for classification
                if self.cap and self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if ret:
                        class_name, confidence = self.classify_detected_object(frame, obj)
                        self.log(f"[AUTO] Classification: {class_name} ({confidence:.0%})")

                        # Determine which sequence to use
                        base_cell = cell.replace('_NON', '')  # Get base cell (e.g., A1 from A1_NON)

                        if class_name == 'non-biodegradable':
                            # Use NON variant
                            non_cell = f"{base_cell}_NON"
                            if non_cell in self.sequences:
                                execute_cell = non_cell
                                self.log(f"[AUTO] NON-BIO detected -> using {execute_cell} sequence")
                            else:
                                self.log(f"[AUTO] NON-BIO detected but {non_cell} not found, falling back to {cell}")
                        else:
                            # BIO or unknown -> use original cell
                            execute_cell = base_cell
                            if base_cell in self.sequences:
                                self.log(f"[AUTO] BIO detected -> using {execute_cell} sequence")
                            else:
                                self.log(f"[AUTO] {base_cell} not found, falling back to {cell}")
                    else:
                        self.log("[AUTO] Could not grab frame for classification")
                else:
                    self.log("[AUTO] Camera not available for classification")
            else:
                self.log("[AUTO] No detected object info for classification")

        if execute_cell not in self.sequences:
            self.log(f"[AUTO] No sequence for {execute_cell}, skipping")
            return

        if not self.is_connected:
            self.log("[AUTO] Not connected to Arduino, cannot pickup")
            messagebox.showerror("Error", "Not connected to Arduino!")
            return

        # CRITICAL: Cancel all pending after callbacks
        self._cancel_pending_callbacks()

        # Execute sequence in background thread
        self.log(f"[AUTO] Starting automatic pickup for {execute_cell}...")
        self.pickup_btn.config(state='disabled')

        # Set stop flag for any currently playing sequence
        self._stop_sequence_flag = True
        time.sleep(0.1)

        # Start new pickup sequence
        self._stop_sequence_flag = False
        thread = threading.Thread(target=self._execute_pickup_sequence, args=(execute_cell,), daemon=True)
        thread.start()

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

        # CRITICAL: Cancel all pending after callbacks (prevents event queue buildup)
        self._cancel_pending_callbacks()

        # Execute sequence in background thread
        self.log(f"Starting pickup sequence for {cell}...")
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
            self.log(f"[Timer]️ Starting pickup for {cell} at {time.time():.3f}")

            for i, step in enumerate(self.sequences[cell]):
                # Check if we should stop
                if getattr(self, '_stop_sequence_flag', False):
                    self.log("Pickup stopped")
                    break

                if 'angles' in step:
                    angles_original = step['angles']
                    # Apply auto-offset to steps 3 and 4 only (indices 2 and 3)
                    angles = self.apply_offset_to_sequence_step(angles_original, i, len(self.sequences[cell]))

                    # Use 15ms delay for fast transitions: step 2->3 (i=1) and step 4->5 (i=3)
                    if i in [1, 3]:
                        delay = 15
                    else:
                        delay = step.get('delay', 1000)
                    step_start = time.time()

                    # For steps 3 and 5 (indices 2 and 4), use 3x slower speed
                    current_speed = int(self.speed_var.get()) if hasattr(self, 'speed_var') else DEFAULT_SPEED
                    if i in [2, 4]:  # Step 3 or Step 5 - use 3x slower speed
                        slow_speed = current_speed * 3
                        speed_command = f"99 {slow_speed}\n"
                        self.send_to_arduino(speed_command)
                        self._safe_after(0, lambda s=slow_speed: self.log(f"  → Speed changed to {s}ms/deg (3x slower for step {i+1})"))

                    # Update step 3 & 4 base value display (show same offset for both)
                    if i in [2, 3]:  # Steps 3 or 4
                        offset = getattr(self, '_last_applied_offset', 0)
                        base_with_offset = angles[0]
                        # Debug: Log what's being sent
                        self.log(f"[DEBUG] Step {i+1}: base={angles_original[0]:.0f}° + offset={offset:+.0f}° = {base_with_offset:.0f}° → Arduino")

                        self._safe_after(0, lambda b=base_with_offset, o=offset: self.step3_base_label.config(
                            text=f"Step 3 (Pickup): {b:.0f}° (base: {angles_original[0]:.0f}° + offset: {o:+.0f}°)"))
                        self._safe_after(0, lambda b=base_with_offset, o=offset: self.step4_base_label.config(
                            text=f"Step 4 (Lift): {b:.0f}° (base: {angles_original[0]:.0f}° + offset: {o:+.0f}°)"))

                    # Send multi-move command directly
                    command = f"M {angles[0]} {angles[1]} {angles[2]} {angles[3]} {angles[4]}\n"
                    self.send_to_arduino(command)

                    self._safe_after(0, lambda s=i+1, c=cell: self.log(f"  Step {s}/{len(self.sequences[c])}: {angles}"))

                    # Wait for servo movement (minimum 1.0 seconds to let Arduino execute)
                    wait_start = time.time()
                    while time.time() - wait_start < max(1.0, delay / 1000.0):
                        if getattr(self, '_stop_sequence_flag', False):
                            break
                        time.sleep(0.1)

                    # Restore original speed after steps 3 and 5 (indices 2 and 4)
                    if i in [2, 4]:  # After Step 3 or Step 5 completes
                        speed_command = f"99 {current_speed}\n"
                        self.send_to_arduino(speed_command)
                        self._safe_after(0, lambda s=current_speed: self.log(f"  → Speed restored to {s}ms/deg"))

                    step_elapsed = time.time() - step_start
                    if step_elapsed > 0.6:
                        self._safe_after(0, lambda s=i+1, t=step_elapsed: self.log(f"[Timer]️ Step {s} took {t:.3f}s"))

            total_elapsed = time.time() - step_start
            self._safe_after(0, lambda c=cell: self.log(f"Pickup for {c} complete! (Total: {total_elapsed:.3f}s)"))
            self._safe_after(0, lambda: self.log("Object picked up successfully!"))

            # Recapture empty grid after successful pickup
            self._safe_after(1000, self._recapture_empty_grid_after_pickup)

        except Exception as e:
            self._safe_after(0, lambda: self.log(f"Pickup error: {e}"))
            self._safe_after(0, lambda: self.pickup_btn.config(state='normal'))
        finally:
            # Clean up thread reference
            if hasattr(self, '_current_pickup_thread'):
                self._current_pickup_thread = None

    def _recapture_empty_grid_after_pickup(self):
        """Recapture empty grid reference after pickup"""
        self.log("Recapturing empty grid reference...")

        ret, frame = self.cap.read()
        if ret:
            self.empty_grid = frame.copy()
            cv2.imwrite(BG_FILE, self.empty_grid)
            self.log("Empty grid reference updated")
        else:
            self.log("Could not recapture empty grid")

        # Clear detection state
        self._clear_detection_after_pickup()

    def _clear_detection_after_pickup(self):
        """Clear detection state after successful pickup"""
        self.detected_objects = []
        self.current_detection_cell = None
        self.pickup_btn.config(state='disabled')
        # Reset cell hysteresis
        self.cell_confirmed = None
        self.cell_confirmation_count = 0
        self.last_detected_cell = None
        self.last_detection_time = 0

        # Re-enable auto pickup if it was enabled
        if self.auto_pickup_enabled:
            self.auto_pickup_running = True
            self.pending_pickup_cell = None
            self.object_first_detected_time = 0
            self.detection_status_label.config(
                text="Auto: Ready for next object...",
                foreground='blue'
            )
            self.log("[AUTO] Ready for next object...")
        else:
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
                        self.log(f"Loaded preset: {name} = {angles}")

                # Update defaults
                if 'Rest' in saved_presets:
                    DEFAULT_REST[:] = saved_presets['Rest']
                if 'Pickup' in saved_presets:
                    DEFAULT_PICKUP[:] = saved_presets['Pickup']

            except Exception as e:
                self.log(f"Could not load presets: {e}")
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
            self.log(f"Presets saved to {PRESETS_FILE}")
        except Exception as e:
            self.log(f"Could not save presets: {e}")

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

        self.log(f"Loaded {len(self.sequences)} legacy sequences")

        # Also try to load from file
        if os.path.exists(SEQUENCES_FILE):
            try:
                with open(SEQUENCES_FILE, 'r') as f:
                    file_sequences = json.load(f)
                for seq_name, steps in file_sequences.items():
                    if seq_name not in self.sequences:
                        self.sequences[seq_name] = steps
                        self.log(f"Loaded additional sequence: {seq_name}")
            except Exception as e:
                self.log(f"Could not load sequences from file: {e}")
    
    def on_cell_select(self, event):
        """Handle cell selection - works with both original and _NON sequences"""
        selection = self.cell_listbox.curselection()
        if not selection:
            return

        # Get cell name from listbox text (first word, before any [OK] marker)
        cell_text = self.cell_listbox.get(selection[0]).split()[0]
        self.current_cell = cell_text
        self.selected_cell_label.config(text=cell_text)

        # Load steps
        self.steps_listbox.delete(0, tk.END)
        if cell_text in self.sequences:
            for i, step in enumerate(self.sequences[cell_text]):
                angles = step.get('angles', [0,0,0,0,0])
                self.steps_listbox.insert(tk.END, f"{i+1}. {angles}")

            self.log(f"Loaded sequence for {cell_text} ({len(self.sequences[cell_text])} steps)")
        else:
            self.log(f"No sequence saved for {cell_text}")

    def load_selected_sequence_to_editor(self):
        """Load selected sequence - works with cell selection OR sequence listbox"""
        # First try cell selection (primary method)
        cell_selection = self.cell_listbox.curselection()
        if cell_selection:
            # Get cell name from listbox text
            cell = self.cell_listbox.get(cell_selection[0]).split()[0]
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

        # Find which cell this sequence belongs to and select it in listbox
        for i in range(self.cell_listbox.size()):
            cell_text = self.cell_listbox.get(i).split()[0]
            if cell_text == seq_name:
                self.cell_listbox.selection_clear(0, tk.END)
                self.cell_listbox.selection_set(i)
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
                        has_seq = "[OK]" if c in self.sequences else ""
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
                has_seq = "[OK]" if c in self.sequences else ""
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

        ttk.Button(paste_frame, text="Paste from Clipboard",
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

        self.log(f"Pasted {angles} to insert dialog")

    def paste_step_to_editor(self):
        """Paste clipboard step to sequence editor"""
        if not hasattr(self, 'step_clipboard') or not self.step_clipboard:
            messagebox.showinfo("Info", "Clipboard is empty\n\nCopy a step first (right-click on a step → Copy Step)")
            return

        # Use first step from clipboard
        angles = self.step_clipboard[0]

        # Add to steps listbox
        index = self.steps_listbox.size()
        self.steps_listbox.insert(tk.END, f"{index + 1}. {angles}")

        self.log(f"Pasted step {angles} to editor")
        messagebox.showinfo("Pasted!", f"Step pasted to sequence editor:\n{angles}")

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
        self.step_menu.add_command(label="Copy Step", command=self.copy_selected_step)
        self.step_menu.add_command(label="Copy All Steps", command=self.copy_all_steps)
        self.step_menu.add_separator()
        self.step_menu.add_command(label="✂️ Cut Step", command=self.cut_selected_step)
        self.step_menu.add_separator()
        self.step_menu.add_command(label="🗑️ Delete Step", command=self.remove_step)
        self.step_menu.add_separator()
        self.step_menu.add_command(label="Paste Step", command=self.paste_step)

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

        def paste_to_edit():
            if not hasattr(self, 'step_clipboard') or not self.step_clipboard:
                messagebox.showinfo("Info", "Clipboard is empty\n\nCopy a step first (right-click → Copy Step)")
                return
            angles = self.step_clipboard[0]
            for i, var in enumerate(angle_vars):
                var.set(str(angles[i]))
            self.log(f"Pasted {angles} to edit dialog")

        btn_frame = ttk.Frame(dialog)
        btn_frame.pack(pady=10)
        ttk.Button(btn_frame, text="Save Changes", command=save_edit).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Paste", command=paste_to_edit).pack(side=tk.LEFT, padx=5)
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
                self.log(f"Could not parse step {i+1}: {e}")

        if not steps:
            messagebox.showwarning("Warning", "No steps in sequence!")
            return

        self.sequences[self.current_cell] = steps

        # Save to file
        with open(SEQUENCES_FILE, 'w') as f:
            json.dump(self.sequences, f, indent=2)

        # Update cell list (show both original and _NON variants)
        self.cell_listbox.delete(0, tk.END)
        for cell in CELL_NAMES:
            has_seq = "[OK]" if cell in self.sequences else ""
            self.cell_listbox.insert(tk.END, f"{cell} {has_seq}")
        # Show NON variants
        for cell in CELL_NAMES:
            non_cell = f"{cell}_NON"
            has_seq = "[OK]" if non_cell in self.sequences else ""
            self.cell_listbox.insert(tk.END, f"{non_cell} {has_seq}")

        # Refresh sequence listbox in Manual Control tab
        self.refresh_seq_list()

        self.log(f"Saved sequence to {self.current_cell} ({len(steps)} steps)")
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
        self.log(f"Rest position updated to: {angles}")
        self.save_presets()
        messagebox.showinfo("Rest Updated", f"New Rest position saved:\n{angles}\n\nThis will persist after restart!")

    def set_current_as_pickup(self):
        """Set current servo positions as new Pickup preset"""
        angles = [int(self.input_boxes[i].get()) for i in range(5)]
        LEGACY_PRESETS['Pickup'] = angles
        DEFAULT_PICKUP[:] = angles
        self.log(f"Pickup position updated to: {angles}")
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
        """Detect objects in frame and log positions - only within grid area"""
        if self.empty_grid is None:
            return []

        # Apply blur to reduce camera noise
        frame = cv2.GaussianBlur(frame, (5, 5), 0)

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

            # Filter by area (min and max)
            if area > self.min_area_var.get() and area < 30000:  # Max 30,000px²
                x, y, w, h = cv2.boundingRect(cnt)

                # Calculate solidity (area / convex_hull_area)
                hull = cv2.convexHull(cnt)
                hull_area = cv2.contourArea(hull)
                solidity = float(area) / hull_area if hull_area > 0 else 0

                # Find cell
                cx, cy = x + w//2, y + h//2
                cell = self.find_cell(cx, cy)

                # Only include objects that are within a valid grid cell
                # Disregard objects outside the grid (cell returns '?' or invalid)
                if cell and cell != '?' and cell[0] in ['A', 'B', 'C', 'D']:
                    # Check if object spans more than 4 grid cells (only if calibrated)
                    if self.is_calibrated and len(self.all_points) >= 30:
                        # Get grid dimensions from calibrated points
                        # Use the full grid bounds to calculate cell size in the CURRENT frame
                        all_x = [p[0] for p in self.all_points[:30]]
                        all_y = [p[1] for p in self.all_points[:30]]
                        grid_width = max(all_x) - min(all_x)
                        grid_height = max(all_y) - min(all_y)

                        # Cell size in current frame coordinates (4 rows x 5 columns)
                        cell_width = grid_width / 5.0  # 5 columns
                        cell_height = grid_height / 4.0  # 4 rows

                        # Calculate how many cells the object spans
                        obj_cells_width = w / cell_width
                        obj_cells_height = h / cell_height
                        total_cells_spanned = obj_cells_width * obj_cells_height

                        # Debug logging for first contour
                        if len(objects) == 0:
                            self.log(f"Object: {w}x{h}px, area={area:.0f}px, solidity={solidity:.2f}, cell={cell}")

                        # Ignore objects that span more than 4 grid cells
                        if total_cells_spanned > 4:
                            self.log(f"Ignoring large object: spans {total_cells_spanned:.1f} cells")
                            continue

                    objects.append({
                        'x': x, 'y': y, 'w': w, 'h': h,
                        'cx': cx, 'cy': cy,
                        'area': area,
                        'solidity': solidity,
                        'cell': cell
                    })

        # Log detected objects
        if objects:
            self.log(f"Found {len(objects)} object(s): {[o['cell'] for o in objects]}")
        elif contours and len(contours) > 0:
            # Log why no objects were detected
            largest = max(contours, key=cv2.contourArea)
            largest_area = cv2.contourArea(largest)
            x, y, w, h = cv2.boundingRect(largest)
            cx, cy = x + w//2, y + h//2
            cell = self.find_cell(cx, cy)
            self.log(f"No valid objects: largest={largest_area:.0f}px² at {cell} ({cx},{cy})")

        return objects
    
    def find_cell(self, x, y):
        """Find which cell contains point (x, y) - 4x4 grid (16 cells)"""
        # First try calibrated grid points (25 points for 4x4 grid)
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
