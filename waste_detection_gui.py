#!/usr/bin/env python3
"""
Waste Detection & Classification GUI
Integrates ZStar camera object detection with waste classification model
"""

import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from PIL import Image, ImageTk
import threading
import time
import os
import serial
import serial.tools.list_ports
from datetime import datetime

# Import waste classifier
import sys
sys.path.insert(0, '/home/koogs/Documents/5DOF_Robotic_Arm_Vision')
from waste_classifier import WasteClassifier

# Configuration
BG_FILE = '/home/koogs/empty_grid_reference.jpg'
CAMERA_INDEX = 0  # ZStar camera index
ARDUINO_PORT = '/dev/ttyUSB0'  # Default Arduino serial port
BAUD_RATE = 115200


class WasteDetectionGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Waste Detection & Classification System")
        self.root.geometry("1400x900")

        # Camera
        self.cap = None
        self.is_running = False
        self.empty_grid = None

        # Classifier
        self.classifier = None
        self.classifier_ready = False

        # Arduino serial connection
        self.arduino = None
        self.bio_capacity = 0
        self.nonbio_capacity = 0
        self.capacity_ready = False

        # Detection settings
        self.threshold_var = tk.IntVar(value=35)
        self.min_area_var = tk.IntVar(value=1300)
        self.solidity_var = tk.DoubleVar(value=0.5)

        # Detection state
        self.last_detected_objects = []
        self.classification_results = {}  # cell -> (class, confidence)

        # UI
        self.setup_ui()

        # Load empty grid reference
        self.load_empty_grid()

        # Load classifier
        self.load_classifier()

        # Connect to Arduino
        self.connect_arduino()

        # Start capacity monitoring
        self.update_capacity_display()

        # Start camera
        self.start_camera()

    def setup_ui(self):
        """Setup main UI"""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Top bar
        top_bar = ttk.Frame(main_frame)
        top_bar.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(top_bar, text="Waste Detection & Classification System",
                 font=('Helvetica', 14, 'bold')).pack(side=tk.LEFT)

        self.status_label = ttk.Label(top_bar, text="Initializing...", foreground='gray', font=('Helvetica', 10))
        self.status_label.pack(side=tk.RIGHT, padx=10)

        # Left: Camera preview
        left_frame = ttk.LabelFrame(main_frame, text="Camera Feed - Object Detection & Classification", padding="10")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.canvas = tk.Canvas(left_frame, bg='black', width=640, height=480)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Detection info overlay
        self.detection_info = ttk.Label(left_frame, text="No objects detected",
                                        font=('Helvetica', 10, 'bold'), foreground='gray')
        self.detection_info.pack(fill=tk.X, pady=(5, 0))

        # Right: Control panel
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)

        # Trash Capacity Bars (at the top of right panel)
        capacity_frame = ttk.LabelFrame(right_frame, text="Trash Bin Capacity", padding="10")
        capacity_frame.pack(fill=tk.X, pady=5)

        # Create canvas for capacity bars
        self.capacity_canvas = tk.Canvas(capacity_frame, width=280, height=180, bg='white', highlightthickness=1, highlightbackground='gray')
        self.capacity_canvas.pack(pady=5)

        # Arduino connection
        arduino_frame = ttk.Frame(capacity_frame)
        arduino_frame.pack(fill=tk.X, pady=5)

        ttk.Label(arduino_frame, text="Serial Port:").pack(side=tk.LEFT, padx=2)
        self.port_var = tk.StringVar(value=ARDUINO_PORT)
        self.port_combo = ttk.Combobox(arduino_frame, textvariable=self.port_var, width=12)
        self.port_combo.pack(side=tk.LEFT, padx=2)
        self.port_combo['values'] = self.get_serial_ports()

        self.connect_btn = ttk.Button(arduino_frame, text="Connect", command=self.connect_arduino)
        self.connect_btn.pack(side=tk.LEFT, padx=2)

        self.arduino_status = ttk.Label(capacity_frame, text="Arduino: Not Connected", foreground='gray', font=('Helvetica', 8))
        self.arduino_status.pack(pady=2)

        # Camera control
        cam_frame = ttk.LabelFrame(right_frame, text="Camera Control", padding="8")
        cam_frame.pack(fill=tk.X, pady=5)

        ttk.Label(cam_frame, text="Camera Index:").pack(anchor='w')
        self.camera_index_var = tk.StringVar(value="0")
        cam_control = ttk.Frame(cam_frame)
        cam_control.pack(fill=tk.X, pady=3)

        self.camera_combo = ttk.Combobox(cam_control, textvariable=self.camera_index_var, width=8, justify='center')
        self.camera_combo.pack(side=tk.LEFT, padx=2)
        self.camera_combo['values'] = [str(i) for i in range(8)]

        ttk.Button(cam_control, text="Find", command=self.find_cameras).pack(side=tk.LEFT, padx=2)
        ttk.Button(cam_control, text="Test", command=self.test_camera).pack(side=tk.LEFT, padx=2)

        ttk.Button(cam_frame, text="Start Camera", command=self.start_camera).pack(fill=tk.X, pady=3)
        ttk.Button(cam_frame, text="Capture Empty Grid", command=self.capture_empty_grid).pack(fill=tk.X, pady=3)

        self.camera_status_label = ttk.Label(cam_frame, text="Status: Not started", foreground='gray', font=('Helvetica', 8))
        self.camera_status_label.pack(pady=2)

        # Detection settings
        detect_frame = ttk.LabelFrame(right_frame, text="Detection Settings", padding="8")
        detect_frame.pack(fill=tk.X, pady=5)

        ttk.Label(detect_frame, text="Threshold:").pack(anchor='w')
        ttk.Scale(detect_frame, from_=10, to=100, variable=self.threshold_var,
                 orient=tk.HORIZONTAL, command=lambda v: self.update_labels()).pack(fill=tk.X, pady=2)

        ttk.Label(detect_frame, text="Min Area:").pack(anchor='w')
        ttk.Scale(detect_frame, from_=500, to=5000, variable=self.min_area_var,
                 orient=tk.HORIZONTAL, command=lambda v: self.update_labels()).pack(fill=tk.X, pady=2)

        ttk.Label(detect_frame, text="Min Solidity:").pack(anchor='w')
        ttk.Scale(detect_frame, from_=0.1, to=0.9, variable=self.solidity_var,
                 orient=tk.HORIZONTAL, command=lambda v: self.update_labels()).pack(fill=tk.X, pady=2)

        self.update_labels()

        # Classification results
        class_frame = ttk.LabelFrame(right_frame, text="Classification Results", padding="8")
        class_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.class_results_text = scrolledtext.ScrolledText(class_frame, height=12, width=30, state='disabled')
        self.class_results_text.pack(fill=tk.BOTH, expand=True)

        # Log
        log_frame = ttk.LabelFrame(right_frame, text="System Log", padding="8")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.log_text = scrolledtext.ScrolledText(log_frame, height=10, width=30, state='disabled')
        self.log_text.pack(fill=tk.BOTH, expand=True)

        # Start/Stop button
        btn_frame = ttk.Frame(right_frame)
        btn_frame.pack(fill=tk.X, pady=10)

        self.toggle_btn = ttk.Button(btn_frame, text="Stop Detection", command=self.toggle_detection)
        self.toggle_btn.pack(fill=tk.X)

        self.log("System initialized")

    def load_classifier(self):
        """Load waste classification model"""
        try:
            self.log("Loading waste classifier...")
            self.classifier = WasteClassifier()
            self.classifier_ready = True
            self.log("Classifier loaded successfully")
            self.status_label.config(text="Classifier Ready", foreground='green')
        except Exception as e:
            self.log(f"Classifier load failed: {e}")
            self.classifier_ready = False
            self.status_label.config(text="Classifier Not Loaded", foreground='red')
            messagebox.showwarning("Warning", f"Waste classifier could not be loaded:\n{e}\n\nDetection will work, but classification will be unavailable.")

    def load_empty_grid(self):
        """Load empty grid reference image"""
        if os.path.exists(BG_FILE):
            self.empty_grid = cv2.imread(BG_FILE)
            self.log(f"Empty grid loaded: {BG_FILE}")
        else:
            self.empty_grid = None
            self.log("No empty grid reference found. Click 'Capture Empty Grid' first.")

    def get_serial_ports(self):
        """Get list of available serial ports"""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def connect_arduino(self):
        """Connect to Arduino via serial"""
        try:
            if self.arduino and self.arduino.is_open:
                self.arduino.close()

            port = self.port_var.get()
            self.arduino = serial.Serial(port, BAUD_RATE, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset

            self.capacity_ready = True
            self.arduino_status.config(text=f"Arduino: Connected ({port})", foreground='green')
            self.log(f"Arduino connected on {port}")

            # Start monitoring capacity
            self.update_capacity_display()

        except Exception as e:
            self.log(f"Arduino connection failed: {e}")
            self.capacity_ready = False
            self.arduino_status.config(text=f"Arduino: Connection Failed", foreground='red')
            messagebox.showerror("Error", f"Could not connect to Arduino:\n{e}")

    def update_capacity_display(self):
        """Update capacity bars display"""
        if not self.capacity_ready or not self.arduino:
            # Draw empty bars
            self.draw_capacity_bar("BIO", 0, "N/A")
            self.draw_capacity_bar("NON-BIO", 0, "N/A")
            self.root.after(2000, self.update_capacity_display)
            return

        try:
            # Request capacity data from Arduino
            self.arduino.write(b'CAPACITY\n')
            time.sleep(0.1)

            if self.arduino.in_waiting > 0:
                line = self.arduino.readline().decode('utf-8').strip()

                if line.startswith('CAP:'):
                    # Parse: CAP:BIO:XX:NONBIO:XX
                    parts = line.split(':')
                    if len(parts) >= 5:
                        self.bio_capacity = int(parts[2])
                        self.nonbio_capacity = int(parts[4])

                        self.draw_capacity_bar("BIO", self.bio_capacity, f"{self.bio_capacity}%")
                        self.draw_capacity_bar("NON-BIO", self.nonbio_capacity, f"{self.nonbio_capacity}%")

        except Exception as e:
            self.log(f"Error reading capacity: {e}")

        # Update every 2 seconds
        self.root.after(2000, self.update_capacity_display)

    def draw_capacity_bar(self, label, percentage, text):
        """Draw a vertical capacity bar with green-to-red gradient"""
        canvas = self.capacity_canvas

        # Clear canvas
        canvas.delete("all")

        # Bar dimensions
        bar_width = 60
        bar_height = 120
        spacing = 20

        # Calculate positions for two bars
        if label == "BIO":
            x = 50
        else:
            x = 150

        y_start = 30
        y_end = y_start + bar_height

        # Draw bar background (empty)
        canvas.create_rectangle(
            x, y_start, x + bar_width, y_end,
            fill='#f0f0f0', outline='gray', width=2
        )

        # Calculate fill height
        fill_height = (percentage / 100.0) * bar_height
        fill_y = y_end - fill_height

        # Calculate color (green to red gradient)
        # Green at bottom (0%), Red at top (100%)
        if percentage <= 50:
            # Green to Yellow
            ratio = percentage / 50.0
            r = int(0 + (255 * ratio))
            g = 255
            b = 0
        else:
            # Yellow to Red
            ratio = (percentage - 50) / 50.0
            r = 255
            g = int(255 * (1 - ratio))
            b = 0

        color = f'#{r:02x}{g:02x}{b:02x}'

        # Draw filled portion (from bottom up)
        if percentage > 0:
            canvas.create_rectangle(
                x, fill_y, x + bar_width, y_end,
                fill=color, outline=''
            )

        # Draw border
        canvas.create_rectangle(
            x, y_start, x + bar_width, y_end,
            fill='', outline='black', width=2
        )

        # Draw label
        canvas.create_text(
            x + bar_width // 2, y_start - 15,
            text=label, font=('Helvetica', 10, 'bold')
        )

        # Draw percentage text
        canvas.create_text(
            x + bar_width // 2, y_end + 15,
            text=text, font=('Helvetica', 9, 'bold')
        )

        # Draw tick marks
        for i in range(0, 101, 25):
            tick_y = y_end - (i / 100.0) * bar_height
            canvas.create_line(
                x - 5, tick_y, x, tick_y,
                fill='black', width=1
            )
            canvas.create_text(
                x - 10, tick_y,
                text=f"{i}%", font=('Helvetica', 7),
                anchor='e'
            )

    def capture_empty_grid(self):
        """Capture current frame as empty grid reference"""
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self.empty_grid = frame.copy()
                cv2.imwrite(BG_FILE, self.empty_grid)
                self.log("Empty grid reference captured and saved")
                messagebox.showinfo("Success", "Empty grid reference captured!")
            else:
                messagebox.showerror("Error", "Failed to capture frame")
        else:
            messagebox.showerror("Error", "Camera not available")

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
                        camera_details.append(f"  /dev/video{i}: {width}x{height} @ {fps_str}")
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

            # Auto-select first available camera
            self.camera_combo.set(str(available_cameras[0]))

            # Show popup with camera list
            camera_list = "\n".join(camera_details)
            messagebox.showinfo(
                "Cameras Found",
                f"Found {len(available_cameras)} camera(s):\n\n"
                f"{camera_list}\n\n"
                f"Selected: /dev/video{self.camera_combo.get()}"
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
                self.log(f"Camera {index} could not be opened")
                self.camera_status_label.config(text=f"Camera {index}: Failed", foreground='red')
                messagebox.showerror("Error", f"Cannot open camera {index}")
                return

            ret, frame = cap.read()
            if ret:
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = cap.get(cv2.CAP_PROP_FPS)
                fps_str = f"{fps:.1f}fps" if fps > 0 else "unknown fps"

                self.log(f"Camera {index} OK: {width}x{height} @ {fps_str}")
                self.camera_status_label.config(
                    text=f"Camera {index}: {width}x{height} @ {fps_str}",
                    foreground='green'
                )
                messagebox.showinfo(
                    "Camera Test OK",
                    f"Camera {index} is working:\n\n"
                    f"Resolution: {width}x{height}\n"
                    f"FPS: {fps_str}"
                )
            else:
                self.log(f"Camera {index} opened but no frame")
                self.camera_status_label.config(text=f"Camera {index}: No frame", foreground='orange')
                messagebox.showwarning("Warning", f"Camera {index} opened but could not read frame")

            cap.release()
        except Exception as e:
            self.log(f"Camera test error: {e}")
            messagebox.showerror("Error", f"Camera test failed:\n{e}")

    def start_camera(self):
        """Start camera preview"""
        try:
            cam_idx = int(self.camera_index_var.get())
            self.cap = cv2.VideoCapture(cam_idx)

            if self.cap.isOpened():
                self.log(f"Camera started (index {cam_idx})")
                self.is_running = True
                self.status_label.config(text="Camera Running", foreground='green')
                self.update_frame()
            else:
                self.log(f"Failed to open camera {cam_idx}")
                self.status_label.config(text="Camera Error", foreground='red')
                messagebox.showerror("Error", f"Cannot open camera {cam_idx}")
        except Exception as e:
            self.log(f"Camera error: {e}")
            self.status_label.config(text="Camera Error", foreground='red')

    def restart_camera(self):
        """Restart camera with new index"""
        if self.cap:
            self.cap.release()

        self.start_camera()

    def update_frame(self):
        """Update camera frame preview"""
        if not self.is_running or not self.cap or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            self.root.after(30, self.update_frame)
            return

        # Detect objects
        display_frame = frame.copy()
        objects = self.detect_objects(frame)

        # Classify objects
        if objects and self.classifier_ready:
            self.classify_objects(frame, objects)

        # Draw detections
        for obj in objects:
            x, y, w, h = obj['x'], obj['y'], obj['w'], obj['h']
            cell = obj['cell']

            # Get classification
            class_name = "Unknown"
            confidence = 0.0
            color = (0, 255, 255)  # Yellow (default)

            if cell in self.classification_results:
                class_name, confidence = self.classification_results[cell]
                if class_name == 'biodegradable':
                    color = (0, 255, 0)  # Green
                elif class_name == 'non-biodegradable':
                    color = (0, 0, 255)  # Red

            # Draw bounding box
            cv2.rectangle(display_frame, (x, y), (x + w, y + h), color, 2)

            # Draw label
            label = f"{cell}: {class_name}"
            if confidence > 0:
                label += f" ({confidence:.1%})"

            cv2.putText(display_frame, label, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Convert to PhotoImage
        display_rgb = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
        display_resized = cv2.resize(display_rgb, (640, 480))
        photo = ImageTk.PhotoImage(image=Image.fromarray(display_resized))

        # Update canvas
        self.canvas.create_image(0, 0, anchor=tk.NW, image=photo)
        self.canvas.image = photo

        # Update detection info
        if objects:
            self.detection_info.config(
                text=f"Detected {len(objects)} object(s): {', '.join([o['cell'] for o in objects])}",
                foreground='green'
            )
        else:
            self.detection_info.config(text="No objects detected", foreground='gray')

        self.root.after(30, self.update_frame)

    def detect_objects(self, frame):
        """Detect objects in frame using background subtraction"""
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

            # Filter by area
            if area > self.min_area_var.get() and area < 30000:
                x, y, w, h = cv2.boundingRect(cnt)

                # Calculate solidity
                hull = cv2.convexHull(cnt)
                hull_area = cv2.contourArea(hull)
                solidity = float(area) / hull_area if hull_area > 0 else 0

                # Filter by solidity
                if solidity >= self.solidity_var.get():
                    cx, cy = x + w // 2, y + h // 2

                    objects.append({
                        'x': x, 'y': y, 'w': w, 'h': h,
                        'cx': cx, 'cy': cy,
                        'area': area,
                        'solidity': solidity,
                        'cell': f"Obj{len(objects) + 1}"
                    })

        self.last_detected_objects = objects
        return objects

    def classify_objects(self, frame, objects):
        """Classify detected objects using waste classifier"""
        self.classification_results = {}

        for obj in objects:
            x, y, w, h = obj['x'], obj['y'], obj['w'], obj['h']
            bbox = (x, y, w, h)

            class_name, confidence = self.classifier.classify(frame, bbox)
            cell = obj['cell']

            self.classification_results[cell] = (class_name, confidence)

        # Update results display
        self.update_classification_display()

    def update_classification_display(self):
        """Update classification results text"""
        self.class_results_text.config(state='normal')
        self.class_results_text.delete(1.0, tk.END)

        if not self.classification_results:
            self.class_results_text.insert(tk.END, "No classifications yet\n")
        else:
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.class_results_text.insert(tk.END, f"=== {timestamp} ===\n\n")

            for cell, (class_name, confidence) in self.classification_results.items():
                icon = "[BIO]" if class_name == 'biodegradable' else "[NON-BIO]"
                color = "GREEN" if class_name == 'biodegradable' else "RED"

                self.class_results_text.insert(tk.END, f"{cell}: {icon}\n")
                self.class_results_text.insert(tk.END, f"     Class: {class_name}\n")
                self.class_results_text.insert(tk.END, f"     Confidence: {confidence:.1%}\n\n")

        self.class_results_text.config(state='disabled')

    def update_labels(self):
        """Update sensitivity labels"""
        pass

    def toggle_detection(self):
        """Toggle detection on/off"""
        if self.is_running:
            self.is_running = False
            self.toggle_btn.config(text="Start Detection")
            self.log("Detection stopped")
        else:
            self.is_running = True
            self.toggle_btn.config(text="Stop Detection")
            self.log("Detection started")
            self.update_frame()

    def log(self, message):
        """Add message to log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        formatted_msg = f"[{timestamp}] {message}"

        self.log_text.config(state='normal')
        self.log_text.insert(tk.END, formatted_msg + "\n")
        self.log_text.see(tk.END)
        self.log_text.config(state='disabled')

    def on_closing(self):
        """Handle window close"""
        self.is_running = False
        if self.cap:
            self.cap.release()
        if self.arduino and self.arduino.is_open:
            self.arduino.close()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = WasteDetectionGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
