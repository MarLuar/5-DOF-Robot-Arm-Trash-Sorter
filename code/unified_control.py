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
DEFAULT_REST = [150, 0, 70, 70, 140]
DEFAULT_PICKUP = [80, 100, 20, 0, 0]

# Legacy file paths from old system
LEGACY_SEQUENCES_FILE = os.path.expanduser("~/.robotic_arm_sequences.json")
LEGACY_PRESETS_FILE = os.path.expanduser("~/.robotic_arm_presets.json")

JOINT_NAMES = ["Base", "Shoulder", "Elbow", "Wrist", "Gripper"]
CELL_NAMES = [f"{chr(ord('A')+row)}{col+1}" for row in range(4) for col in range(4)]

CALIBRATION_FILE = '/home/koogs/Documents/5DOF_Robotic_Arm_Vision/calibration/vision_calibration.json'
SEQUENCES_FILE = '/home/koogs/Documents/5DOF_Robotic_Arm_Vision/sequences/cell_sequences.json'
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
        
        # Setup UI
        self.setup_ui()
        self.refresh_ports()
        self.load_calibration()

        # Start camera preview in auto tab (single instance shared between tabs)
        self.start_camera()

        # Start logging
        self.log("System initialized - v1.00.02")

        # Load sequences AFTER UI is setup (log_text must exist)
        self.load_sequences()
    
    def setup_ui(self):
        """Setup main UI with notebook tabs"""
        main_frame = ttk.Frame(self.root, padding="5")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
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
        
        # Tab 4: Auto Detection
        self.auto_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.auto_tab, text="Auto Detection")
        self.setup_auto_tab()
        
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
        
        # Right: Connection & Speed
        right_frame = ttk.LabelFrame(self.manual_tab, text="Connection & Speed", padding="10")
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
        
        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected", foreground='red')
        self.status_label.pack(pady=5)
        
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

        # Fixed size canvas matching camera aspect ratio (4:3)
        self.calib_canvas = tk.Canvas(left_frame, bg='black', width=640, height=480)
        self.calib_canvas.pack(fill=tk.BOTH, expand=True)
        self.calib_canvas.bind('<Button-1>', self.on_calib_click)

        # Buttons
        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(fill=tk.X, pady=10)
        
        ttk.Button(btn_frame, text="Reset", command=self.reset_calibration).pack(side=tk.LEFT, padx=5)
        self.calc_btn = ttk.Button(btn_frame, text="Calculate Grid", command=self.calculate_grid, state='disabled')
        self.calc_btn.pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Recapture Empty Grid", command=self.capture_empty_grid).pack(side=tk.LEFT, padx=5)
        
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

3. Verify grid overlay

4. Switch to Auto Detection tab
   to test object detection
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
        
        step_btn_frame = ttk.Frame(mid_frame)
        step_btn_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(step_btn_frame, text="Add Current", command=self.add_current_step).pack(side=tk.LEFT, padx=2)
        ttk.Button(step_btn_frame, text="Add Rest", command=self.add_rest_step).pack(side=tk.LEFT, padx=2)
        ttk.Button(step_btn_frame, text="Remove", command=self.remove_step).pack(side=tk.LEFT, padx=2)
        ttk.Button(step_btn_frame, text="Clear", command=self.clear_steps).pack(side=tk.LEFT, padx=2)
        
        ttk.Button(mid_frame, text="Save Sequence to Cell", command=self.save_current_sequence).pack(pady=5)
        
        # Right: Presets
        right_frame = ttk.LabelFrame(self.seq_tab, text="Quick Presets", padding="10")
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)
        
        ttk.Button(right_frame, text="Load Rest Position", command=self.load_rest_preset).pack(fill=tk.X, pady=2)
        ttk.Button(right_frame, text="Load Pickup Position", command=self.load_pickup_preset).pack(fill=tk.X, pady=2)
        
        ttk.Separator(right_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        ttk.Label(right_frame, text="Test:").pack(anchor='w')
        ttk.Button(right_frame, text="Play Sequence", command=self.test_sequence).pack(fill=tk.X, pady=2)
    
    def setup_auto_tab(self):
        """Setup auto detection tab"""
        # Left: Camera preview
        left_frame = ttk.LabelFrame(self.auto_tab, text="Live Detection", padding="10")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.auto_canvas = tk.Canvas(left_frame, bg='black', width=640, height=480)
        self.auto_canvas.pack(fill=tk.BOTH, expand=True)
        
        # Controls
        ctrl_frame = ttk.Frame(left_frame)
        ctrl_frame.pack(fill=tk.X, pady=10)
        
        self.auto_detect_var = tk.BooleanVar(value=False)
        self.auto_check = ttk.Checkbutton(ctrl_frame, text="Enable Auto-Detection",
                                         variable=self.auto_detect_var,
                                         command=self.toggle_auto_detect)
        self.auto_check.pack(side=tk.LEFT, padx=5)
        
        self.camera_btn = ttk.Button(ctrl_frame, text="Start Camera", command=self.toggle_camera)
        self.camera_btn.pack(side=tk.LEFT, padx=5)
        
        self.detect_btn = ttk.Button(ctrl_frame, text="Detect Now", command=self.detect_object_now)
        self.detect_btn.pack(side=tk.LEFT, padx=5)
        
        # Right: Detection results
        right_frame = ttk.LabelFrame(self.auto_tab, text="Detection Results", padding="10")
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.result_text = scrolledtext.ScrolledText(right_frame, height=15, width=45)
        self.result_text.pack(fill=tk.BOTH, expand=True)
        
        # Stats
        stats_frame = ttk.LabelFrame(right_frame, text="Statistics", padding="5")
        stats_frame.pack(fill=tk.X, pady=10)
        
        self.stats_label = ttk.Label(stats_frame, text="Objects detected: 0\nLast cell: N/A")
        self.stats_label.pack()
        
        # Action
        action_frame = ttk.LabelFrame(right_frame, text="Action on Detection", padding="5")
        action_frame.pack(fill=tk.X, pady=10)
        
        self.execute_on_detect_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(action_frame, text="Execute sequence on detection",
                       variable=self.execute_on_detect_var).pack(anchor='w')
        
        # Sensitivity (same as calib tab)
        sens_frame = ttk.LabelFrame(right_frame, text="Sensitivity", padding="5")
        sens_frame.pack(fill=tk.X, pady=10)
        
        ttk.Label(sens_frame, text="Threshold:").pack(anchor='w')
        ttk.Scale(sens_frame, from_=10, to=100, variable=self.threshold_var,
                 orient=tk.HORIZONTAL).pack(fill=tk.X)
        ttk.Label(sens_frame, text="Min Area:").pack(anchor='w', pady=(5,0))
        ttk.Scale(sens_frame, from_=1000, to=10000, variable=self.min_area_var,
                 orient=tk.HORIZONTAL).pack(fill=tk.X)
    
    def setup_log_panel(self, parent):
        """Setup shared log panel"""
        log_frame = ttk.LabelFrame(parent, text="System Log", padding="5")
        log_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=6, width=120, state='disabled')
        self.log_text.pack(fill=tk.X)
    
    def log(self, message):
        """Add message to log"""
        if not hasattr(self, 'log_text'):
            # UI not ready yet, print to console
            timestamp = datetime.now().strftime("%H:%M:%S")
            print(f"[{timestamp}] {message}")
            return

        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.config(state='normal')
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state='disabled')
    
    # Manual Control Methods
    def refresh_ports(self):
        """Refresh serial ports"""
        ports = serial.tools.list_ports.comports()
        port_list = [f"{p.device}" for p in ports]
        self.port_combo['values'] = port_list
        if port_list:
            self.port_combo.current(0)
    
    def toggle_connection(self):
        """Toggle serial connection"""
        if self.is_connected:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        """Connect to Arduino"""
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No Port", "Please select a port")
            return
        
        try:
            self.serial_conn = serial.Serial(port, BAUD_RATE, timeout=1)
            time.sleep(2)  # Wait for Arduino reset
            self.is_connected = True
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text="Status: Connected", foreground='green')
            self.log(f"Connected to {port}")
        except Exception as e:
            messagebox.showerror("Error", f"Could not connect: {e}")
            self.log(f"Connection failed: {e}")
    
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
    
    def adjust_servo(self, idx, delta):
        """Adjust servo angle"""
        try:
            current = int(self.input_boxes[idx].get())
            new_val = max(MIN_ANGLE, min(MAX_ANGLE, current + delta))
            self.input_boxes[idx].set(str(new_val))
        except:
            pass
    
    def send_servo(self, idx):
        """Send servo command"""
        if not self.is_connected:
            self.log("Not connected!")
            return
        
        try:
            angle = int(self.input_boxes[idx].get())
            command = f"{idx} {angle}\n"
            self.serial_conn.write(command.encode())
            self.log(f"Sent: {JOINT_NAMES[idx]} -> {angle}°")
        except Exception as e:
            self.log(f"Error: {e}")
    
    def set_speed(self):
        """Set movement speed"""
        if not self.is_connected:
            return
        
        try:
            speed = int(self.speed_var.get())
            command = f"99 {speed}\n"
            self.serial_conn.write(command.encode())
            self.log(f"Speed set to {speed}ms/deg")
        except:
            pass
    
    def go_to_rest(self):
        """Go to rest position"""
        for i, angle in enumerate(DEFAULT_REST):
            self.input_boxes[i].set(str(angle))
            if self.is_connected:
                self.send_servo(i)
        self.log("Moved to rest position")
    
    def go_to_pickup(self):
        """Go to pickup position"""
        for i, angle in enumerate(DEFAULT_PICKUP):
            self.input_boxes[i].set(str(angle))
            if self.is_connected:
                self.send_servo(i)
        self.log("Moved to pickup position")
    
    def refresh_seq_list(self):
        """Refresh sequence listbox"""
        self.seq_listbox.delete(0, tk.END)
        for name in self.sequences.keys():
            self.seq_listbox.insert(tk.END, name)
    
    def play_sequence(self):
        """Play selected sequence"""
        selection = self.seq_listbox.curselection()
        if not selection:
            return
        
        seq_name = self.seq_listbox.get(selection[0])
        if seq_name not in self.sequences:
            return
        
        self.log(f"Playing sequence: {seq_name}")
        for step in self.sequences[seq_name]:
            if self.is_connected and 'angles' in step:
                for i, angle in enumerate(step['angles']):
                    command = f"{i} {angle}\n"
                    self.serial_conn.write(command.encode())
                time.sleep(step.get('delay', 1000) / 1000.0)
        
        self.log("Sequence complete")
    
    def stop_sequence(self):
        """Stop sequence (placeholder)"""
        self.log("Sequence stopped")
    
    # Camera Methods (single instance shared between tabs)
    def start_camera(self):
        """Start single camera instance"""
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.log("Warning: Could not open camera on /dev/video2, trying /dev/video0...")
            self.cap = cv2.VideoCapture(0)

        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.is_running = True
            self.log("Camera started")
            # Start both previews
            self.update_calib_preview()
            self.update_auto_preview()
        else:
            self.log("ERROR: No camera available!")
            messagebox.showerror("Camera Error", "Could not open any camera!\n\nMake sure:\n1. Camera is plugged in\n2. Not used by another program\n3. Try: ls -la /dev/video*")

    def update_calib_preview(self):
        """Update calibration tab preview - copied from simple_grid_calib"""
        if not hasattr(self, 'cap') or self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            if hasattr(self, 'calib_canvas'):
                self.calib_canvas.after(30, self.update_calib_preview)
            return

        self.original_height, self.original_width = frame.shape[:2]

        # Get canvas size
        canvas_width = self.calib_canvas.winfo_width()
        canvas_height = self.calib_canvas.winfo_height()

        if canvas_width < 2 or canvas_height < 2:
            if hasattr(self, 'calib_canvas'):
                self.calib_canvas.after(30, self.update_calib_preview)
            return

        # Calculate scale (use MIN to fit, not MAX which crops)
        scale = min(canvas_width / self.original_width,
                   canvas_height / self.original_height)

        new_width = int(self.original_width * scale)
        new_height = int(self.original_height * scale)

        if new_width < 1 or new_height < 1:
            if hasattr(self, 'calib_canvas'):
                self.calib_canvas.after(30, self.update_calib_preview)
            return

        # Resize
        display_frame = cv2.resize(frame, (new_width, new_height))

        # Create black background
        bg = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)

        # Center image
        y_offset = (canvas_height - new_height) // 2
        x_offset = (canvas_width - new_width) // 2

        bg[y_offset:y_offset+new_height, x_offset:x_offset+new_width] = display_frame

        # Store for click calculation
        self.calib_x_offset = x_offset
        self.calib_y_offset = y_offset
        self.calib_img_width = new_width
        self.calib_img_height = new_height

        # Draw corners on background (not on camera frame)
        corner_colors = [(0, 0, 255), (255, 0, 0), (0, 255, 0), (0, 255, 255)]
        corner_names = ['1: TL', '2: TR', '3: BL', '4: BR']

        for i, (orig_x, orig_y) in enumerate(self.corners):
            # Convert original click coordinates to display coordinates
            disp_x = int(orig_x * (new_width / self.original_width)) + x_offset
            disp_y = int(orig_y * (new_height / self.original_height)) + y_offset

            cv2.circle(bg, (disp_x, disp_y), 15, corner_colors[i], -1)
            cv2.circle(bg, (disp_x, disp_y), 20, corner_colors[i], 2)
            cv2.putText(bg, corner_names[i], (disp_x+20, disp_y-20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, corner_colors[i], 2)

        # Convert BGR to RGB
        bg = cv2.cvtColor(bg, cv2.COLOR_BGR2RGB)

        # Convert to PhotoImage
        img = Image.fromarray(bg)
        photo = ImageTk.PhotoImage(image=img)

        # Update canvas
        self.calib_canvas.delete("all")
        self.calib_canvas.create_image(0, 0, anchor='nw', image=photo)
        self.calib_canvas.image = photo

        # Schedule next update
        if hasattr(self, 'calib_canvas'):
            self.calib_canvas.after(30, self.update_calib_preview)

    def update_auto_preview(self):
        """Update auto detection tab preview - copied from simple_grid_calib"""
        if not hasattr(self, 'cap') or self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            if hasattr(self, 'auto_canvas'):
                self.auto_canvas.after(30, self.update_auto_preview)
            return

        if self.auto_detect_enabled and self.empty_grid is not None:
            # Detect objects
            objects = self.detect_objects(frame)

            # Draw detections
            for obj in objects:
                x, y, w, h = obj['x'], obj['y'], obj['w'], obj['h']
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(frame, f"Cell: {obj.get('cell', '?')}", (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        self.original_height, self.original_width = frame.shape[:2]

        # Get canvas size
        canvas_width = self.auto_canvas.winfo_width()
        canvas_height = self.auto_canvas.winfo_height()

        if canvas_width < 2 or canvas_height < 2:
            if hasattr(self, 'auto_canvas'):
                self.auto_canvas.after(30, self.update_auto_preview)
            return

        # Calculate scale (use MIN to fit, not MAX which crops)
        scale = min(canvas_width / self.original_width,
                   canvas_height / self.original_height)

        new_width = int(self.original_width * scale)
        new_height = int(self.original_height * scale)

        if new_width < 1 or new_height < 1:
            if hasattr(self, 'auto_canvas'):
                self.auto_canvas.after(30, self.update_auto_preview)
            return

        # Resize
        display_frame = cv2.resize(frame, (new_width, new_height))

        # Create black background
        bg = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)

        # Center image
        y_offset = (canvas_height - new_height) // 2
        x_offset = (canvas_width - new_width) // 2

        bg[y_offset:y_offset+new_height, x_offset:x_offset+new_width] = display_frame

        # Store offsets for click calculation
        self.auto_x_offset = x_offset
        self.auto_y_offset = y_offset
        self.auto_img_width = new_width
        self.auto_img_height = new_height

        # Convert BGR to RGB
        bg = cv2.cvtColor(bg, cv2.COLOR_BGR2RGB)

        # Convert to PhotoImage
        img = Image.fromarray(bg)
        photo = ImageTk.PhotoImage(image=img)

        # Update canvas
        self.auto_canvas.delete("all")
        self.auto_canvas.create_image(0, 0, anchor='nw', image=photo)
        self.auto_canvas.image = photo

        # Schedule next update
        if hasattr(self, 'auto_canvas'):
            self.auto_canvas.after(30, self.update_auto_preview)
    
    def on_calib_click(self, event):
        """Handle calibration click - account for offset and scaling"""
        if len(self.corners) >= 4:
            return

        # Get click position
        click_x = event.x
        click_y = event.y

        # Convert from display coordinates back to original image coordinates
        if hasattr(self, 'calib_x_offset') and hasattr(self, 'calib_img_width'):
            # Remove offset
            adj_x = click_x - self.calib_x_offset
            adj_y = click_y - self.calib_y_offset

            # Check if click is within image area
            if adj_x < 0 or adj_x > self.calib_img_width:
                return
            if adj_y < 0 or adj_y > self.calib_img_height:
                return

            # Scale back to original image coordinates
            orig_x = int(adj_x * (self.original_width / self.calib_img_width))
            orig_y = int(adj_y * (self.original_height / self.calib_img_height))

            self.corners.append((orig_x, orig_y))
        else:
            # Fallback if offsets not set yet
            self.corners.append((click_x, click_y))

        self.corner_label.config(text=f"Corners clicked: {len(self.corners)}/4")

        if len(self.corners) >= 4:
            self.calc_btn.config(state='normal')
            self.log(f"All 4 corners clicked")
    
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

        # Calculate 25 points from 4 corners using bilinear interpolation
        # Corners are: TL, TR, BL, BR
        tl = np.array(self.corners[0], dtype=np.float32)  # Top-Left
        tr = np.array(self.corners[1], dtype=np.float32)  # Top-Right
        bl = np.array(self.corners[2], dtype=np.float32)  # Bottom-Left
        br = np.array(self.corners[3], dtype=np.float32)  # Bottom-Right

        self.all_points = []
        for row in range(5):  # 5 rows (0-4)
            for col in range(5):  # 5 columns (0-4)
                # Calculate position using bilinear interpolation
                u = col / 4.0  # Horizontal position (0.0 to 1.0)
                v = row / 4.0  # Vertical position (0.0 to 1.0)

                # Interpolate along top and bottom edges
                top_point = tl + (tr - tl) * u
                bottom_point = bl + (br - bl) * u

                # Interpolate between top and bottom
                point = top_point + (bottom_point - top_point) * v

                self.all_points.append((int(point[0]), int(point[1])))

        self.is_calibrated = True

        # Save calibration
        calib_data = {
            'grid_intersections': {},
            'timestamp': time.time()
        }
        for i, (x, y) in enumerate(self.all_points):
            row = i // 5
            col = i % 5
            cell_name = f"{chr(ord('A')+row)}{col+1}"
            calib_data['grid_intersections'][cell_name] = {'x': x, 'y': y}

        with open(CALIBRATION_FILE, 'w') as f:
            json.dump(calib_data, f, indent=2)

        self.log(f"Grid calculated: {len(self.all_points)} points")
        self.log(f"Calibration saved to {CALIBRATION_FILE}")

        messagebox.showinfo(
            "Success",
            f"Grid calculated!\n\n"
            f"25 intersection points generated\n\n"
            f"Empty grid captured!\n\n"
            f"Go to Auto Detection tab to test object detection"
        )
    
    def capture_empty_grid(self):
        """Recapture empty grid"""
        ret, frame = self.cap.read()
        if ret:
            self.empty_grid = frame.copy()
            cv2.imwrite(BG_FILE, self.empty_grid)
            self.log("Empty grid recaptured")
            messagebox.showinfo("Success", "Empty grid reference updated!")
    
    def update_sensitivity_labels(self):
        """Update sensitivity labels"""
        self.sens_label.config(
            text=f"Threshold: {self.threshold_var.get()}\n"
                 f"Area: {self.min_area_var.get()}\n"
                 f"Solidity: {self.solidity_var.get():.2f}"
        )
    
    # Sequence Methods
    def load_sequences(self):
        """Load sequences from file including legacy sequences"""
        # First try new location
        if os.path.exists(SEQUENCES_FILE):
            try:
                with open(SEQUENCES_FILE, 'r') as f:
                    self.sequences = json.load(f)
                self.log(f"Loaded {len(self.sequences)} sequences from new location")
                return
            except:
                pass

        # Try legacy location
        if os.path.exists(LEGACY_SEQUENCES_FILE):
            try:
                with open(LEGACY_SEQUENCES_FILE, 'r') as f:
                    legacy_seqs = json.load(f)

                # Convert legacy format to new format
                self.sequences = {}
                for name, steps in legacy_seqs.items():
                    self.sequences[name] = []
                    for step in steps:
                        if isinstance(step[0], str):
                            # It's a preset name, need to convert to angles
                            preset_name = step[0]
                            delay = step[1] if len(step) > 1 else 1000
                            # Load preset angles
                            angles = self.get_preset_angles(preset_name)
                            if angles:
                                self.sequences[name].append({'angles': angles, 'delay': delay, 'name': preset_name})
                        else:
                            # It's already angles
                            self.sequences[name].append({'angles': step[0], 'delay': step[1] if len(step) > 1 else 1000})

                self.log(f"Loaded {len(self.sequences)} sequences from legacy file")
                return
            except Exception as e:
                self.log(f"Could not load legacy sequences: {e}")

        self.sequences = {}
        self.log("No sequences found")

    def get_preset_angles(self, preset_name):
        """Get angles for a preset name from legacy presets"""
        if os.path.exists(LEGACY_PRESETS_FILE):
            try:
                with open(LEGACY_PRESETS_FILE, 'r') as f:
                    presets = json.load(f)
                if preset_name in presets:
                    return presets[preset_name]
            except:
                pass

        # Default presets
        defaults = {
            'Rest': DEFAULT_REST,
            'Pickup': DEFAULT_PICKUP
        }
        return defaults.get(preset_name)
    
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
    
    def remove_step(self):
        """Remove selected step"""
        selection = self.steps_listbox.curselection()
        if selection:
            self.steps_listbox.delete(selection[0])
    
    def clear_steps(self):
        """Clear all steps"""
        self.steps_listbox.delete(0, tk.END)
    
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
            except:
                pass
        
        self.sequences[self.current_cell] = steps
        
        # Save to file
        with open(SEQUENCES_FILE, 'w') as f:
            json.dump(self.sequences, f, indent=2)
        
        # Update cell list
        self.cell_listbox.delete(0, tk.END)
        for cell in CELL_NAMES:
            has_seq = "✓" if cell in self.sequences else ""
            self.cell_listbox.insert(tk.END, f"{cell} {has_seq}")
        
        self.log(f"Saved sequence to {self.current_cell} ({len(steps)} steps)")
        messagebox.showinfo("Success", f"Sequence saved to {self.current_cell}!\n\n{len(steps)} steps")
    
    def load_rest_preset(self):
        """Load rest preset"""
        for i, angle in enumerate(DEFAULT_REST):
            self.input_boxes[i].set(str(angle))
    
    def load_pickup_preset(self):
        """Load pickup preset"""
        for i, angle in enumerate(DEFAULT_PICKUP):
            self.input_boxes[i].set(str(angle))
    
    def test_sequence(self):
        """Test current sequence"""
        if not self.current_cell or self.current_cell not in self.sequences:
            messagebox.showwarning("Warning", "No sequence for this cell")
            return
        
        self.log(f"Testing sequence for {self.current_cell}")
        # Would execute sequence here
    
    # Auto Detection Methods
    def toggle_camera(self):
        """Toggle camera on/off"""
        if self.is_running:
            self.is_running = False
            self.camera_btn.config(text="Start Camera")
            self.log("Camera stopped")
        else:
            self.is_running = True
            self.camera_btn.config(text="Stop Camera")
            self.log("Camera started")
    
    def toggle_auto_detect(self):
        """Toggle auto detection"""
        self.auto_detect_enabled = self.auto_detect_var.get()
        self.log(f"Auto-detection {'enabled' if self.auto_detect_enabled else 'disabled'}")
    
    def detect_objects(self, frame):
        """Detect objects in frame"""
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
        
        return objects
    
    def find_cell(self, x, y):
        """Find which cell contains point (x, y)"""
        if not self.is_calibrated or len(self.all_points) < 25:
            return "?"

        # Simple bounding box check for each cell
        for row in range(4):
            for col in range(4):
                # Get 4 corners of this cell
                idx1 = row * 5 + col          # Top-Left
                idx2 = idx1 + 1               # Top-Right
                idx3 = idx1 + 5               # Bottom-Left
                idx4 = idx3 + 1               # Bottom-Right

                if idx4 < len(self.all_points):
                    x1, y1 = self.all_points[idx1]
                    x2, y2 = self.all_points[idx2]
                    x3, y3 = self.all_points[idx3]
                    x4, y4 = self.all_points[idx4]

                    # Get bounding box
                    min_x = min(x1, x2, x3, x4)
                    max_x = max(x1, x2, x3, x4)
                    min_y = min(y1, y2, y3, y4)
                    max_y = max(y1, y2, y3, y4)

                    # Check if point is inside
                    if min_x <= x <= max_x and min_y <= y <= max_y:
                        return f"{chr(ord('A')+row)}{col+1}"

        return "?"
    
    def detect_object_now(self):
        """Detect objects now"""
        ret, frame = self.auto_cap.read()
        if not ret:
            return
        
        objects = self.detect_objects(frame)
        
        self.result_text.delete('1.0', tk.END)
        self.result_text.insert(tk.END, f"Detection Results:\n")
        self.result_text.insert(tk.END, f"="*40 + "\n")
        
        if not objects:
            self.result_text.insert(tk.END, "No objects detected\n")
        else:
            for i, obj in enumerate(objects):
                self.result_text.insert(tk.END, f"\nObject {i+1}:\n")
                self.result_text.insert(tk.END, f"  Cell: {obj.get('cell', '?')}\n")
                self.result_text.insert(tk.END, f"  Position: ({obj['cx']}, {obj['cy']})\n")
                self.result_text.insert(tk.END, f"  Size: {obj['w']}x{obj['h']}\n")
                self.result_text.insert(tk.END, f"  Area: {obj['area']:.0f}\n")
                
                # Update stats
                self.detection_count += 1
                self.last_cell = obj.get('cell', '?')
                self.stats_label.config(text=f"Objects detected: {self.detection_count}\nLast cell: {self.last_cell}")
                
                # Execute sequence if enabled
                if self.execute_on_detect_var.get() and obj.get('cell') in self.sequences:
                    self.log(f"Object in {obj['cell']} - executing sequence!")
                    self.execute_cell_sequence(obj['cell'])
        
        self.log(f"Detection complete: {len(objects)} objects")
    
    def execute_cell_sequence(self, cell):
        """Execute sequence for cell"""
        if cell not in self.sequences:
            self.log(f"No sequence for {cell}")
            return
        
        if not self.is_connected:
            self.log("Not connected to Arduino!")
            return
        
        self.log(f"Executing sequence for {cell}")
        for step in self.sequences[cell]:
            if 'angles' in step:
                for i, angle in enumerate(step['angles']):
                    command = f"{i} {angle}\n"
                    self.serial_conn.write(command.encode())
                time.sleep(step.get('delay', 1000) / 1000.0)
        
        self.log(f"Sequence for {cell} complete")
    
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
