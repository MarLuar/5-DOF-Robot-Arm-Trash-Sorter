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

# Legacy preset positions from original robotic_arm_controller.py
LEGACY_PRESETS = {
    'Rest': [80, 180, 50, 80, 140],
    'Pickup': [80, 74, 50, 5, 140],
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

        # Start logging
        self.log("System initialized - v1.00.02")

        # Load sequences (after UI is ready)
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
                ttk.Label(frame, text="⚠ AVOID 110-120°!", foreground='red', font=('Helvetica', 8, 'bold')).pack(side=tk.LEFT, padx=3)

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

4. Place object on grid
   - See which cell it's detected in
   - Yellow boxes show detections
   - Check System Log for coordinates
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

        ttk.Label(right_frame, text="Test:").pack(anchor='w')
        ttk.Button(right_frame, text="Play Sequence", command=self.test_sequence).pack(fill=tk.X, pady=2)
    
    def setup_log_panel(self, parent):
        """Setup shared log panel"""
        log_frame = ttk.LabelFrame(parent, text="System Log", padding="5")
        log_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=6, width=120, state='disabled')
        self.log_text.pack(fill=tk.X)
    
    def log(self, message):
        """Add message to log"""
        try:
            timestamp = datetime.now().strftime("%H:%M:%S")
            if hasattr(self, 'log_text') and self.log_text:
                self.log_text.config(state='normal')
                self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
                self.log_text.see(tk.END)
                self.log_text.config(state='disabled')
            else:
                # Fallback: print to console
                print(f"[{timestamp}] {message}")
        except Exception as e:
            # Final fallback
            print(f"[ERROR] {message}")
    
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

        # Go to rest position automatically
        self.log("Moving to rest position...")
        self.go_to_rest()

    def _on_connect_error(self, error):
        """Called when connection fails"""
        messagebox.showerror("Error", f"Could not connect: {error}")
        self.log(f"Connection failed: {error}")

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

            # CRITICAL: Block base servo from dead zone
            if idx == 0 and BASE_WARNING_MIN <= angle <= BASE_WARNING_MAX:
                self.root.after(0, lambda a=angle: self.log(f"🚨 BLOCKED: Base {a}° is in DEAD ZONE (110-120°)!"))
                return

            command = f"{idx} {angle}\n"
            self.serial_conn.write(command.encode())
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
            self.serial_conn.write(command.encode())
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
            self.serial_conn.write(command.encode())
            # Update UI from main thread
            self.root.after(0, lambda s=speed: self.log(f"Speed set to {s}ms/deg"))
        except Exception as ex:
            error_msg = str(ex)
            self.root.after(0, lambda msg=error_msg: self.log(f"Error: {msg}"))

    def go_to_rest(self):
        """Go to rest position (simultaneous movement)"""
        rest_angles = LEGACY_PRESETS.get('Rest', DEFAULT_REST)

        # Check if rest position has base in danger zone
        if rest_angles[0] and BASE_WARNING_MIN <= rest_angles[0] <= BASE_WARNING_MAX:
            self.log(f"🚨 WARNING: Rest position has base at {rest_angles[0]}° (danger zone)!")
            self.log(f"  Consider changing Rest preset to avoid 110-120°")

        for i, angle in enumerate(rest_angles):
            self.input_boxes[i].set(str(angle))
        if self.is_connected:
            self.send_multi_move(rest_angles)
        self.log("Moved to rest position (simultaneous)")

    def go_to_pickup(self):
        """Go to pickup position (simultaneous movement)"""
        pickup_angles = LEGACY_PRESETS.get('Pickup', DEFAULT_PICKUP)

        # Check if pickup position has base in danger zone
        if pickup_angles[0] and BASE_WARNING_MIN <= pickup_angles[0] <= BASE_WARNING_MAX:
            self.log(f"🚨 WARNING: Pickup position has base at {pickup_angles[0]}° (danger zone)!")

        for i, angle in enumerate(pickup_angles):
            self.input_boxes[i].set(str(angle))
        if self.is_connected:
            self.send_multi_move(pickup_angles)
        self.log("Moved to pickup position (simultaneous)")

    def refresh_seq_list(self):
        """Refresh sequence listbox"""
        self.seq_listbox.delete(0, tk.END)
        for name in self.sequences.keys():
            self.seq_listbox.insert(tk.END, name)
    
    def play_sequence(self):
        """Play selected sequence with simultaneous movement"""
        selection = self.seq_listbox.curselection()
        if not selection:
            return

        seq_name = self.seq_listbox.get(selection[0])
        if seq_name not in self.sequences:
            return

        self.log(f"Playing sequence: {seq_name}")
        for step in self.sequences[seq_name]:
            if self.is_connected and 'angles' in step:
                # Use multi-move for simultaneous movement
                self.send_multi_move(step['angles'])
                time.sleep(step.get('delay', 1000) / 1000.0)

        self.log("Sequence complete")
    
    def stop_sequence(self):
        """Stop sequence (placeholder)"""
        self.log("Sequence stopped")
    
    # Calibration Methods
    def start_calib_preview(self):
        """Start calibration preview"""
        self.cap = cv2.VideoCapture(2)
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.update_calib_preview()
    
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
            if self.detect_in_calib_var.get() and hasattr(self, 'empty_grid') and self.empty_grid is not None:
                objects = self.detect_objects(frame)
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

            self.calib_canvas.after(30, self.update_calib_preview)
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
    
    def update_sensitivity_labels(self):
        """Update sensitivity labels"""
        self.sens_label.config(
            text=f"Threshold: {self.threshold_var.get()}\n"
                 f"Area: {self.min_area_var.get()}\n"
                 f"Solidity: {self.solidity_var.get():.2f}"
        )
    
    # Sequence Methods
    def load_sequences(self):
        """Load sequences including legacy sequences"""
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
        """Load rest position from legacy presets"""
        if 'Rest' in LEGACY_PRESETS:
            for i, angle in enumerate(LEGACY_PRESETS['Rest']):
                self.input_boxes[i].set(str(angle))
            self.log("Loaded legacy Rest position")
        else:
            for i, angle in enumerate(DEFAULT_REST):
                self.input_boxes[i].set(str(angle))

    def load_pickup_preset(self):
        """Load pickup position from legacy presets"""
        if 'Pickup' in LEGACY_PRESETS:
            for i, angle in enumerate(LEGACY_PRESETS['Pickup']):
                self.input_boxes[i].set(str(angle))
            self.log("Loaded legacy Pickup position")
        else:
            for i, angle in enumerate(DEFAULT_PICKUP):
                self.input_boxes[i].set(str(angle))
    
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
            for obj in objects:
                self.log(f"📍 Object detected at ({obj['cx']}, {obj['cy']}) in cell {obj['cell']}")

        return objects
    
    def find_cell(self, x, y):
        """Find which cell contains point (x, y)"""
        if not self.is_calibrated or len(self.all_points) < 25:
            return "?"
        
        # Simple bounding box check
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
