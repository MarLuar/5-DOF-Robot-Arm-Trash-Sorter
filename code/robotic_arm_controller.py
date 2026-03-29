#!/usr/bin/env python3
"""
5-DOF Robotic Arm Controller
A GUI application with preset management for servo positions.
"""

import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import serial
import serial.tools.list_ports
import threading
import json
import os
import time

# Configuration
BAUD_RATE = 115200
PRESETS_FILE = os.path.expanduser("~/.robotic_arm_presets.json")
SEQUENCES_FILE = os.path.expanduser("~/.robotic_arm_sequences.json")

MIN_ANGLE = 0
MAX_ANGLE = 180
DEFAULT_SPEED = 15  # ms/deg (lower = faster)
MIN_SPEED = 5
MAX_SPEED = 100

JOINT_NAMES = ["Base", "Shoulder", "Elbow", "Wrist", "Gripper"]

# Default presets
DEFAULT_PRESETS = {
    "Rest": [150, 0, 70, 70, 140],
    "Pickup": [80, 100, 20, 0, 140],
}

# Default sequences
DEFAULT_SEQUENCES = {
    "Basic Pick & Place": [
        ("Pickup", 1500),
        ("Rest", 1000),
    ],
}


class PresetManager:
    def __init__(self):
        self.presets = self.load_presets()

    def load_presets(self):
        """Load presets from file"""
        try:
            if os.path.exists(PRESETS_FILE):
                with open(PRESETS_FILE, 'r') as f:
                    presets = json.load(f)
                    # Ensure default presets exist
                    for name, positions in DEFAULT_PRESETS.items():
                        if name not in presets:
                            presets[name] = positions
                    return presets
        except Exception as e:
            print(f"Error loading presets: {e}")
        return DEFAULT_PRESETS.copy()

    def save_presets(self):
        """Save presets to file"""
        try:
            with open(PRESETS_FILE, 'w') as f:
                json.dump(self.presets, f, indent=2)
            return True
        except Exception as e:
            messagebox.showerror("Error", f"Could not save presets: {e}")
            return False

    def add_preset(self, name, positions):
        """Add or update a preset"""
        self.presets[name] = positions
        return self.save_presets()

    def delete_preset(self, name):
        """Delete a preset"""
        if name in DEFAULT_PRESETS:
            return False  # Can't delete defaults
        if name in self.presets:
            del self.presets[name]
            return self.save_presets()
        return False

    def get_preset(self, name):
        """Get preset positions"""
        return self.presets.get(name)


class SequenceManager:
    def __init__(self):
        self.sequences = self.load_sequences()

    def load_sequences(self):
        """Load sequences from file"""
        try:
            if os.path.exists(SEQUENCES_FILE):
                with open(SEQUENCES_FILE, 'r') as f:
                    sequences = json.load(f)
                    # Ensure default sequences exist
                    for name, seq in DEFAULT_SEQUENCES.items():
                        if name not in sequences:
                            sequences[name] = seq
                    return sequences
        except Exception as e:
            print(f"Error loading sequences: {e}")
        return DEFAULT_SEQUENCES.copy()

    def save_sequences(self):
        """Save sequences to file"""
        try:
            with open(SEQUENCES_FILE, 'w') as f:
                json.dump(self.sequences, f, indent=2)
            return True
        except Exception as e:
            messagebox.showerror("Error", f"Could not save sequences: {e}")
            return False

    def add_sequence(self, name, sequence):
        """Add or update a sequence"""
        self.sequences[name] = sequence
        return self.save_sequences()

    def delete_sequence(self, name):
        """Delete a sequence"""
        if name in self.sequences:
            del self.sequences[name]
            return self.save_sequences()
        return False

    def get_sequence(self, name):
        """Get sequence data"""
        return self.sequences.get(name)


class RoboticArmController:
    def __init__(self, root):
        self.root = root
        self.root.title("5-DOF Robotic Arm Controller")
        self.root.geometry("650x700")

        self.serial_conn = None
        self.is_connected = False
        self.input_boxes = []
        self.reading = False
        self.preset_manager = PresetManager()
        self.sequence_manager = SequenceManager()
        self.preset_buttons = []
        self.preset_vars = []  # For preset combo
        self.sequence = []  # Current sequence being built

        self.setup_ui()
        self.refresh_ports()
        self.refresh_preset_list()

    def setup_ui(self):
        """Setup the user interface"""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Title
        title_label = ttk.Label(main_frame, text="5-DOF Robotic Arm Controller",
                                font=('Helvetica', 16, 'bold'))
        title_label.pack(pady=(0, 10))

        # Connection frame
        conn_frame = ttk.LabelFrame(main_frame, text="Serial Connection", padding="10")
        conn_frame.pack(fill=tk.X, pady=(0, 10))

        port_frame = ttk.Frame(conn_frame)
        port_frame.pack(fill=tk.X)

        ttk.Label(port_frame, text="Port:").pack(side=tk.LEFT, padx=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(port_frame, textvariable=self.port_var, width=35)
        self.port_combo.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)

        self.connect_btn = ttk.Button(port_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=5)

        self.refresh_btn = ttk.Button(port_frame, text="Refresh", command=self.refresh_ports)
        self.refresh_btn.pack(side=tk.LEFT, padx=5)

        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected", foreground="red")
        self.status_label.pack(pady=(5, 0))

        # Servo control frame
        control_frame = ttk.LabelFrame(main_frame, text="Servo Control", padding="10")
        control_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

        for i in range(5):
            self.create_input_row(control_frame, i)

        # Preset buttons frame
        preset_btn_frame = ttk.LabelFrame(main_frame, text="Quick Presets", padding="10")
        preset_btn_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.preset_buttons_frame = ttk.Frame(preset_btn_frame)
        self.preset_buttons_frame.pack(fill=tk.X)

        # Speed control frame
        speed_frame = ttk.LabelFrame(main_frame, text="Movement Speed", padding="10")
        speed_frame.pack(fill=tk.X, pady=(0, 10))

        speed_inner = ttk.Frame(speed_frame)
        speed_inner.pack(fill=tk.X)

        ttk.Label(speed_inner, text="Speed:").pack(side=tk.LEFT, padx=5)

        self.speed_var = tk.StringVar(value=str(DEFAULT_SPEED))
        self.speed_entry = ttk.Entry(speed_inner, textvariable=self.speed_var, width=5,
                                     justify='center', font=('Helvetica', 11))
        self.speed_entry.pack(side=tk.LEFT, padx=5)
        self.speed_entry.bind('<Return>', lambda e: self.set_speed())

        ttk.Label(speed_inner, text="ms/deg").pack(side=tk.LEFT, padx=5)

        self.speed_slider = ttk.Scale(speed_inner, from_=MIN_SPEED, to=MAX_SPEED,
                                      orient=tk.HORIZONTAL, length=300)
        self.speed_slider.set(DEFAULT_SPEED)
        self.speed_slider.pack(side=tk.LEFT, padx=10)

        ttk.Button(speed_inner, text="Set Speed", command=self.set_speed).pack(side=tk.LEFT, padx=10)
        ttk.Label(speed_inner, text="(Lower = Faster)", foreground='gray').pack(side=tk.LEFT, padx=10)

        # Sequence Builder frame
        seq_frame = ttk.LabelFrame(main_frame, text="🎬 Sequence Builder", padding="10")
        seq_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

        # Top section: Available presets
        top_frame = ttk.Frame(seq_frame)
        top_frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(top_frame, text="Available Presets (drag to sequence):",
                 font=('Helvetica', 10, 'bold')).pack(anchor='w')

        avail_frame = ttk.Frame(top_frame)
        avail_frame.pack(fill=tk.X, pady=5)

        self.avail_listbox = tk.Listbox(avail_frame, height=4, font=('Helvetica', 11),
                                        selectmode=tk.SINGLE, exportselection=0)
        avail_listbar = ttk.Scrollbar(avail_frame, orient=tk.VERTICAL,
                                      command=self.avail_listbox.yview)
        self.avail_listbox.configure(yscrollcommand=avail_listbar.set)
        self.avail_listbox.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        avail_listbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.avail_listbox.bind('<Double-Button-1>', lambda e: self.add_to_sequence())
        self.refresh_avail_list()

        # Middle section: Sequence list
        mid_frame = ttk.Frame(seq_frame)
        mid_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

        ttk.Label(mid_frame, text="Sequence (double-click to edit delay):",
                 font=('Helvetica', 10, 'bold')).pack(anchor='w')

        seq_list_frame = ttk.Frame(mid_frame)
        seq_list_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.seq_listbox = tk.Listbox(seq_list_frame, height=6, font=('Helvetica', 11),
                                      selectmode=tk.SINGLE)
        seq_scrollbar = ttk.Scrollbar(seq_list_frame, orient=tk.VERTICAL,
                                      command=self.seq_listbox.yview)
        self.seq_listbox.configure(yscrollcommand=seq_scrollbar.set)
        self.seq_listbox.bind('<Double-Button-1>', lambda e: self.edit_seq_delay())
        self.seq_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        seq_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.sequence = []  # List of (preset_name, delay_ms) tuples

        # Sequence controls
        ctrl_frame = ttk.Frame(seq_frame)
        ctrl_frame.pack(fill=tk.X)

        ttk.Button(ctrl_frame, text="➕ Add Selected", width=12,
                  command=self.add_to_sequence).pack(side=tk.LEFT, padx=3)
        ttk.Button(ctrl_frame, text="⬆️ Up", width=8,
                  command=self.move_seq_up).pack(side=tk.LEFT, padx=3)
        ttk.Button(ctrl_frame, text="⬇️ Down", width=8,
                  command=self.move_seq_down).pack(side=tk.LEFT, padx=3)
        ttk.Button(ctrl_frame, text="❌ Remove", width=8,
                  command=self.remove_from_sequence).pack(side=tk.LEFT, padx=3)
        ttk.Button(ctrl_frame, text="🗑️ Clear All", width=8,
                  command=self.clear_sequence).pack(side=tk.LEFT, padx=3)
        ttk.Button(ctrl_frame, text="💾 Save Sequence", width=12,
                  command=self.save_sequence_dialog).pack(side=tk.LEFT, padx=3)
        ttk.Button(ctrl_frame, text="📂 Load Sequence", width=12,
                  command=self.load_sequence_dialog).pack(side=tk.LEFT, padx=3)

        # Play controls
        play_frame = ttk.Frame(seq_frame)
        play_frame.pack(fill=tk.X, pady=(10, 0))

        ttk.Label(play_frame, text="Delay after step (ms):").pack(side=tk.LEFT, padx=5)
        self.delay_var = tk.StringVar(value="1000")
        self.delay_entry = ttk.Entry(play_frame, textvariable=self.delay_var, width=6)
        self.delay_entry.pack(side=tk.LEFT, padx=5)

        self.is_playing = False
        self.play_btn = ttk.Button(play_frame, text="▶️ Play Sequence",
                                   command=self.toggle_sequence_play, width=15)
        self.play_btn.pack(side=tk.LEFT, padx=10)

        ttk.Button(play_frame, text="⏹️ Stop", command=self.stop_sequence,
                  width=8).pack(side=tk.LEFT, padx=5)

        self.seq_status_label = ttk.Label(play_frame, text="Ready", foreground='gray')
        self.seq_status_label.pack(side=tk.LEFT, padx=10)

        # Quick sequence buttons
        quick_seq_frame = ttk.LabelFrame(main_frame, text="⚡ Quick Sequences", padding="10")
        quick_seq_frame.pack(fill=tk.X, pady=(0, 10))

        self.quick_seq_buttons_frame = ttk.Frame(quick_seq_frame)
        self.quick_seq_buttons_frame.pack(fill=tk.X)
        self.quick_seq_buttons = []

        # Manage presets frame
        manage_frame = ttk.Frame(main_frame)
        manage_frame.pack(pady=(0, 10))

        ttk.Button(manage_frame, text="💾 Save Current as Preset", command=self.save_current_as_preset).pack(side=tk.LEFT, padx=5)
        ttk.Button(manage_frame, text="⚙️ Manage Presets", command=self.manage_presets).pack(side=tk.LEFT, padx=5)
        ttk.Button(manage_frame, text="🔄 Refresh Presets", command=self.refresh_preset_list).pack(side=tk.LEFT, padx=5)

        # Log frame
        log_frame = ttk.LabelFrame(main_frame, text="Log", padding="10")
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.log_text = tk.Text(log_frame, height=8, width=70, state='disabled')
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.configure(yscrollcommand=scrollbar.set)

    def create_input_row(self, parent, index):
        """Create an input row with +/- buttons for a servo"""
        servo_frame = ttk.Frame(parent)
        servo_frame.pack(fill=tk.X, pady=4)

        # Label
        ttk.Label(servo_frame, text=f"{JOINT_NAMES[index]}",
                  font=('Helvetica', 11, 'bold'), width=10).pack(side=tk.LEFT, padx=5)

        # Decrement buttons
        ttk.Button(servo_frame, text="-10", width=3,
                  command=lambda idx=index: self.decrement_angle(idx, 10)).pack(side=tk.LEFT, padx=2)

        ttk.Button(servo_frame, text="-1", width=3,
                  command=lambda idx=index: self.decrement_angle(idx, 1)).pack(side=tk.LEFT, padx=2)

        # Input box
        var = tk.StringVar(value="70")
        input_box = ttk.Entry(servo_frame, textvariable=var, width=5, 
                             font=('Helvetica', 11), justify='center')
        input_box.pack(side=tk.LEFT, padx=8)
        input_box.bind('<Return>', lambda e, idx=index: self.send_command(idx))
        self.input_boxes.append(var)

        # Increment buttons
        ttk.Button(servo_frame, text="+1", width=3,
                  command=lambda idx=index: self.increment_angle(idx, 1)).pack(side=tk.LEFT, padx=2)

        ttk.Button(servo_frame, text="+10", width=3,
                  command=lambda idx=index: self.increment_angle(idx, 10)).pack(side=tk.LEFT, padx=2)

        # Send button
        ttk.Button(servo_frame, text="Send", width=5,
                  command=lambda idx=index: self.send_command(idx)).pack(side=tk.LEFT, padx=5)

    def increment_angle(self, index, amount):
        """Increment angle by amount"""
        try:
            current = int(self.input_boxes[index].get())
            new_val = min(current + amount, MAX_ANGLE)
            self.input_boxes[index].set(str(new_val))
        except ValueError:
            self.input_boxes[index].set("70")

    def decrement_angle(self, index, amount):
        """Decrement angle by amount"""
        try:
            current = int(self.input_boxes[index].get())
            new_val = max(current - amount, MIN_ANGLE)
            self.input_boxes[index].set(str(new_val))
        except ValueError:
            self.input_boxes[index].set("70")

    def send_command(self, servo_num):
        """Send command to Arduino"""
        if not self.is_connected or not self.serial_conn:
            self.log("Not connected!")
            return

        try:
            angle = int(self.input_boxes[servo_num].get())
            angle = max(MIN_ANGLE, min(angle, MAX_ANGLE))
            self.input_boxes[servo_num].set(str(angle))
            
            command = f"{servo_num} {angle}\n"
            self.serial_conn.write(command.encode())
            self.log(f"Sent: '{servo_num} {angle}' → {JOINT_NAMES[servo_num]}")
        except ValueError:
            self.log(f"Invalid angle for {JOINT_NAMES[servo_num]}")
        except serial.SerialException as e:
            self.log(f"Error: {e}")
            self.disconnect()

    def send_preset(self, name):
        """Send preset positions to Arduino for simultaneous movement"""
        if not self.is_connected or not self.serial_conn:
            self.log("Not connected!")
            return

        positions = self.preset_manager.get_preset(name)
        if not positions:
            self.log(f"Preset '{name}' not found!")
            return

        # Update input boxes
        for i in range(5):
            self.input_boxes[i].set(str(positions[i]))

        # Send multi-move command for simultaneous movement: "M a1 a2 a3 a4 a5"
        command = f"M {positions[0]} {positions[1]} {positions[2]} {positions[3]} {positions[4]}\n"
        self.serial_conn.write(command.encode())

        self.log(f"✓ Preset '{name}': {positions} (simultaneous)")

    def go_to_rest(self):
        """Go to rest position"""
        self.send_preset("Rest")

    def go_to_pickup(self):
        """Go to pickup position"""
        self.send_preset("Pickup")

    def set_speed(self):
        """Set servo movement speed"""
        if not self.is_connected or not self.serial_conn:
            self.log("Not connected!")
            return

        try:
            speed = int(self.speed_var.get())
            speed = max(MIN_SPEED, min(speed, MAX_SPEED))
            self.speed_var.set(str(speed))
            self.speed_slider.set(speed)

            command = f"99 {speed}\n"
            self.serial_conn.write(command.encode())
            self.log(f"✓ Speed set to: {speed}ms/deg")
        except ValueError:
            messagebox.showwarning("Warning", "Speed must be a number")
            self.speed_var.set(str(DEFAULT_SPEED))

    def save_current_as_preset(self):
        """Save current positions as a new preset"""
        try:
            positions = [int(self.input_boxes[i].get()) for i in range(5)]
        except ValueError:
            messagebox.showwarning("Warning", "Invalid angle values in input boxes")
            return
        
        # Simple dialog for preset name
        dialog = tk.Toplevel(self.root)
        dialog.title("Save Preset")
        dialog.transient(self.root)
        dialog.grab_set()
        
        ttk.Label(dialog, text="Preset Name:").pack(pady=10)
        name_entry = ttk.Entry(dialog, width=30)
        name_entry.pack(padx=20)
        
        # Show current positions
        ttk.Label(dialog, text=f"Positions: {positions}").pack(pady=5)
        
        def on_save():
            name = name_entry.get().strip()
            if not name:
                messagebox.showwarning("Warning", "Please enter a name")
                return
            if self.preset_manager.add_preset(name, positions):
                self.log(f"✓ Saved preset '{name}'")
                self.refresh_preset_list()
                dialog.destroy()
        
        ttk.Button(dialog, text="Save", command=on_save).pack(pady=10)
        ttk.Button(dialog, text="Cancel", command=dialog.destroy).pack(pady=(0, 10))
        
        self.root.wait_window(dialog)

    def manage_presets(self):
        """Open preset management dialog"""
        self.log("Opening Manage Presets dialog...")

        # Debug: log current presets
        presets = self.preset_manager.presets
        self.log(f"DEBUG: Presets loaded: {presets}")

        # Create fullscreen dialog
        dialog = tk.Toplevel(self.root)
        dialog.title("Manage Presets")

        # Make dialog fullscreen
        dialog.attributes('-fullscreen', True)

        # Add close button with Esc key binding
        dialog.bind('<Escape>', lambda e: dialog.destroy())

        # Main frame with padding
        main_frame = ttk.Frame(dialog, padding=20)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Title
        ttk.Label(main_frame, text="Manage Presets",
                 font=('Helvetica', 18, 'bold')).pack(pady=10)

        # Instructions
        ttk.Label(main_frame, text="Click a preset to select, then Edit or Delete (Press ESC to close)",
                 font=('Helvetica', 11)).pack(pady=5)

        # Preset list with selection - use Text widget instead of Listbox
        list_frame = ttk.Frame(main_frame)
        list_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        # Create text widget with scrollbar
        text_widget = tk.Text(list_frame, font=('Helvetica', 13), height=20, width=60)
        text_widget.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=text_widget.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        text_widget.configure(yscrollcommand=scrollbar.set)
        text_widget.configure(state='disabled')  # Make read-only

        # Store preset data for later use
        dialog.preset_names = []
        dialog.preset_positions = []

        # Populate list
        for name in sorted(presets.keys()):
            positions = presets[name]
            dialog.preset_names.append(name)
            dialog.preset_positions.append(positions)
            text_widget.configure(state='normal')
            text_widget.insert(tk.END, f"  {name}: {positions}\n")
            text_widget.configure(state='disabled')

        self.log(f"DEBUG: Added {len(dialog.preset_names)} presets to list")

        # Selection tracking
        selected_idx = [-1]  # Use list to allow modification in closure

        def on_text_click(event):
            # Get line number
            line = text_widget.index(f"@{event.x},{event.y}").split('.')[0]
            idx = int(line) - 1
            if 0 <= idx < len(dialog.preset_names):
                selected_idx[0] = idx
                name = dialog.preset_names[idx]
                positions = dialog.preset_positions[idx]
                status_label.config(text=f"Selected: {name}: {positions}")
                # Highlight selection
                text_widget.tag_remove("selected", "1.0", tk.END)
                text_widget.tag_add("selected", f"{line}.0", f"{line}.end")
                text_widget.tag_configure("selected", background="lightblue")

        text_widget.bind("<Button-1>", on_text_click)

        # Status label
        status_label = ttk.Label(main_frame, text="", foreground='blue', font=('Helvetica', 12))
        status_label.pack(pady=5)

        # Buttons
        btn_frame = ttk.Frame(main_frame)
        btn_frame.pack(pady=15)

        def do_edit():
            if selected_idx[0] >= 0:
                self._edit_preset_by_idx(dialog, selected_idx[0], text_widget, status_label)
            else:
                messagebox.showwarning("Warning", "Please select a preset first")

        def do_delete():
            if selected_idx[0] >= 0:
                self._delete_preset_by_idx(dialog, selected_idx[0], text_widget, status_label)
            else:
                messagebox.showwarning("Warning", "Please select a preset first")

        ttk.Button(btn_frame, text="✏️ Edit", command=do_edit, width=15).pack(side=tk.LEFT, padx=15)
        ttk.Button(btn_frame, text="🗑️ Delete", command=do_delete, width=15).pack(side=tk.LEFT, padx=15)
        ttk.Button(btn_frame, text="Close (ESC)", command=dialog.destroy, width=15).pack(side=tk.LEFT, padx=15)

        self.log("Manage Presets dialog opened")

    def _edit_preset_by_idx(self, dialog, idx, text_widget, status_label):
        """Edit preset by index"""
        if idx < 0 or idx >= len(dialog.preset_names):
            return

        name = dialog.preset_names[idx]
        positions = dialog.preset_positions[idx]

        # Create edit dialog
        edit_dialog = tk.Toplevel(dialog)
        edit_dialog.title(f"Edit Preset: {name}")
        edit_dialog.geometry("450x420")
        edit_dialog.transient(dialog)
        edit_dialog.grab_set()

        ttk.Label(edit_dialog, text="Preset Name:", font=('Helvetica', 11, 'bold')).pack(pady=10)
        name_entry = ttk.Entry(edit_dialog, width=40, font=('Helvetica', 11))
        name_entry.insert(0, name)
        name_entry.pack(padx=20, pady=5)

        ttk.Label(edit_dialog, text="Angles:", font=('Helvetica', 11, 'bold')).pack(pady=(15, 5))

        # Create angle entries
        angle_vars = []
        angle_frame = ttk.Frame(edit_dialog)
        angle_frame.pack(pady=5)

        for i, joint in enumerate(JOINT_NAMES):
            row_frame = ttk.Frame(angle_frame)
            row_frame.pack(pady=2)
            ttk.Label(row_frame, text=f"{joint}:", width=10).pack(side=tk.LEFT, padx=10)
            var = tk.StringVar(value=str(positions[i]))
            entry = ttk.Entry(row_frame, textvariable=var, width=6, justify='center', font=('Helvetica', 11))
            entry.pack(side=tk.LEFT)
            angle_vars.append(var)

        def on_save():
            new_name = name_entry.get().strip()
            if not new_name:
                messagebox.showwarning("Warning", "Please enter a preset name")
                return

            try:
                new_positions = [int(var.get()) for var in angle_vars]
                for pos in new_positions:
                    if pos < MIN_ANGLE or pos > MAX_ANGLE:
                        messagebox.showwarning("Warning", f"Angles must be between {MIN_ANGLE}-{MAX_ANGLE}")
                        return
            except ValueError:
                messagebox.showwarning("Warning", "All angles must be valid numbers (0-180)")
                return

            # Delete old preset if name changed
            if new_name != name:
                self.preset_manager.delete_preset(name)

            if self.preset_manager.add_preset(new_name, new_positions):
                self.log(f"✓ Updated preset '{new_name}'")
                # Refresh the list
                self._refresh_preset_list_in_dialog(dialog, text_widget, status_label)
                edit_dialog.destroy()

        btn_frame = ttk.Frame(edit_dialog)
        btn_frame.pack(pady=15)
        ttk.Button(btn_frame, text="Save Changes", command=on_save).pack(side=tk.LEFT, padx=10)
        ttk.Button(btn_frame, text="Cancel", command=edit_dialog.destroy).pack(side=tk.LEFT, padx=10)

    def _delete_preset_by_idx(self, dialog, idx, text_widget, status_label):
        """Delete preset by index"""
        if idx < 0 or idx >= len(dialog.preset_names):
            return

        name = dialog.preset_names[idx]

        if name in ["Rest", "Pickup"]:
            messagebox.showwarning("Warning", "Cannot delete default presets (Rest, Pickup)")
            return

        if messagebox.askyesno("Confirm Delete", f"Are you sure you want to delete preset '{name}'?"):
            if self.preset_manager.delete_preset(name):
                self.log(f"✓ Deleted preset '{name}'")
                # Refresh the list
                self._refresh_preset_list_in_dialog(dialog, text_widget, status_label)
            else:
                self.log(f"Failed to delete preset '{name}'")

    def _refresh_preset_list_in_dialog(self, dialog, text_widget, status_label):
        """Refresh preset list in dialog"""
        presets = self.preset_manager.presets

        # Clear and repopulate
        dialog.preset_names = []
        dialog.preset_positions = []
        text_widget.configure(state='normal')
        text_widget.delete("1.0", tk.END)

        for name in sorted(presets.keys()):
            positions = presets[name]
            dialog.preset_names.append(name)
            dialog.preset_positions.append(positions)
            text_widget.insert(tk.END, f"  {name}: {positions}\n")

        text_widget.configure(state='disabled')
        status_label.config(text="")

        # Refresh main window preset buttons
        self.refresh_preset_list()

    def refresh_preset_list(self):
        """Refresh preset buttons in UI"""
        # Clear existing buttons
        for btn in self.preset_buttons:
            btn.destroy()
        self.preset_buttons = []

        # Create buttons for each preset
        for name in sorted(self.preset_manager.presets.keys()):
            btn = ttk.Button(self.preset_buttons_frame, text=name,
                           command=lambda n=name: self.send_preset(n))
            btn.pack(side=tk.LEFT, padx=3, pady=2)
            self.preset_buttons.append(btn)

        # Also refresh available presets list
        self.refresh_avail_list()

        # Refresh quick sequence buttons
        self.refresh_quick_sequences()

        self.log("✓ Presets refreshed")

    def refresh_quick_sequences(self):
        """Refresh quick sequence buttons"""
        # Clear existing buttons
        for btn in self.quick_seq_buttons:
            btn.destroy()
        self.quick_seq_buttons = []

        # Create buttons for each sequence
        for name in sorted(self.sequence_manager.sequences.keys()):
            btn = ttk.Button(self.quick_seq_buttons_frame, text=f"▶️ {name}",
                           command=lambda n=name: self.quick_play_sequence(n))
            btn.pack(side=tk.LEFT, padx=3, pady=2)
            self.quick_seq_buttons.append(btn)

        if not self.quick_seq_buttons:
            ttk.Label(self.quick_seq_buttons_frame, text="No saved sequences. Build and save one!",
                     foreground='gray').pack(side=tk.LEFT, padx=10)

    # ========== Sequence Builder Methods ==========

    def refresh_avail_list(self):
        """Refresh available presets listbox"""
        self.avail_listbox.delete(0, tk.END)
        for name in sorted(self.preset_manager.presets.keys()):
            self.avail_listbox.insert(tk.END, name)

    def add_to_sequence(self):
        """Add selected preset to sequence"""
        selection = self.avail_listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a preset to add")
            return

        preset_name = self.avail_listbox.get(selection[0])
        try:
            delay = int(self.delay_var.get())
            if delay < 0:
                delay = 0
        except ValueError:
            delay = 1000
            self.delay_var.set("1000")

        self.sequence.append((preset_name, delay))
        self.update_seq_listbox()
        self.log(f"Added '{preset_name}' to sequence (delay: {delay}ms)")

    def remove_from_sequence(self):
        """Remove selected item from sequence"""
        selection = self.seq_listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Please select an item to remove")
            return

        idx = selection[0]
        removed = self.sequence.pop(idx)
        self.update_seq_listbox()
        self.log(f"Removed '{removed[0]}' from sequence")

    def clear_sequence(self):
        """Clear entire sequence"""
        if self.sequence and messagebox.askyesno("Confirm", "Clear entire sequence?"):
            self.sequence = []
            self.update_seq_listbox()
            self.log("Sequence cleared")

    def move_seq_up(self):
        """Move selected sequence item up"""
        selection = self.seq_listbox.curselection()
        if not selection:
            return
        idx = selection[0]
        if idx > 0:
            self.sequence[idx], self.sequence[idx-1] = self.sequence[idx-1], self.sequence[idx]
            self.update_seq_listbox()
            self.seq_listbox.selection_clear(0, tk.END)
            self.seq_listbox.selection_set(idx - 1)

    def move_seq_down(self):
        """Move selected sequence item down"""
        selection = self.seq_listbox.curselection()
        if not selection:
            return
        idx = selection[0]
        if idx < len(self.sequence) - 1:
            self.sequence[idx], self.sequence[idx+1] = self.sequence[idx+1], self.sequence[idx]
            self.update_seq_listbox()
            self.seq_listbox.selection_clear(0, tk.END)
            self.seq_listbox.selection_set(idx + 1)

    def update_seq_listbox(self):
        """Update sequence listbox display"""
        self.seq_listbox.delete(0, tk.END)
        for i, (name, delay) in enumerate(self.sequence):
            self.seq_listbox.insert(tk.END, f"{i+1}. {name} (delay: {delay}ms)")

    def toggle_sequence_play(self):
        """Start or stop sequence playback"""
        if self.is_playing:
            self.stop_sequence()
        else:
            self.play_sequence()

    def play_sequence(self):
        """Play the sequence"""
        if not self.sequence:
            messagebox.showwarning("Warning", "Sequence is empty! Add some presets first.")
            return

        if not self.is_connected or not self.serial_conn:
            messagebox.showwarning("Warning", "Not connected to Arduino!")
            return

        self.is_playing = True
        self.play_btn.config(text="⏸️ Pause")
        self.seq_status_label.config(text="Playing...", foreground='green')
        self.log(f"▶️ Starting sequence ({len(self.sequence)} steps)")

        # Run sequence in background thread
        thread = threading.Thread(target=self._run_sequence, daemon=True)
        thread.start()

    def _run_sequence(self):
        """Run sequence in background thread"""
        for i, (preset_name, delay) in enumerate(self.sequence):
            if not self.is_playing:
                self.root.after(0, lambda: self.seq_status_label.config(text="Stopped", foreground='red'))
                return

            # Update status
            step_num = i + 1
            total = len(self.sequence)
            self.root.after(0, lambda n=step_num, t=total, p=preset_name:
                           self.seq_status_label.config(text=f"Step {n}/{t}: {p}", foreground='green'))
            self.root.after(0, lambda p=preset_name: self.log(f"→ Executing: {p}"))

            # Send preset
            positions = self.preset_manager.get_preset(preset_name)
            if positions:
                # Send multi-move command
                command = f"M {positions[0]} {positions[1]} {positions[2]} {positions[3]} {positions[4]}\n"
                self.serial_conn.write(command.encode())

                # Wait for delay
                time.sleep(delay / 1000.0)

        # Sequence complete
        if self.is_playing:
            self.root.after(0, lambda: self.seq_status_label.config(text="Complete!", foreground='blue'))
            self.root.after(0, lambda: self.log("✓ Sequence completed"))
            self.root.after(2000, lambda: self.seq_status_label.config(text="Ready", foreground='gray'))
        self.is_playing = False
        self.root.after(0, lambda: self.play_btn.config(text="▶️ Play Sequence"))

    def stop_sequence(self):
        """Stop sequence playback"""
        self.is_playing = False
        self.play_btn.config(text="▶️ Play Sequence")
        self.seq_status_label.config(text="Stopped", foreground='red')
        self.log("⏹️ Sequence stopped")

    def quick_play_sequence(self, sequence_name):
        """Quick play a saved sequence by name"""
        seq = self.sequence_manager.get_sequence(sequence_name)
        if not seq:
            self.log(f"Sequence '{sequence_name}' not found!")
            return

        # Load and play
        self.sequence = seq
        self.update_seq_listbox()
        self.log(f"✓ Loaded '{sequence_name}'")
        self.play_sequence()

    def save_sequence_dialog(self):
        """Open dialog to save current sequence"""
        if not self.sequence:
            messagebox.showwarning("Warning", "Sequence is empty! Add some presets first.")
            return

        dialog = tk.Toplevel(self.root)
        dialog.title("Save Sequence")
        dialog.geometry("400x200")
        dialog.transient(self.root)
        dialog.grab_set()

        ttk.Label(dialog, text="Save Current Sequence",
                 font=('Helvetica', 12, 'bold')).pack(pady=10)

        ttk.Label(dialog, text="Sequence Name:").pack(pady=5)
        name_entry = ttk.Entry(dialog, width=40, font=('Helvetica', 11))
        name_entry.pack(padx=20, pady=5)

        # Show sequence preview
        preview_text = "\n".join([f"  {i+1}. {name} ({delay}ms)" for i, (name, delay) in enumerate(self.sequence)])
        ttk.Label(dialog, text=f"Steps:\n{preview_text}",
                 justify=tk.LEFT, font=('Helvetica', 10)).pack(pady=10)

        def on_save():
            name = name_entry.get().strip()
            if not name:
                messagebox.showwarning("Warning", "Please enter a sequence name")
                return

            if self.sequence_manager.add_sequence(name, self.sequence):
                self.log(f"✓ Saved sequence '{name}'")
                messagebox.showinfo("Success", f"Sequence '{name}' saved!")
                dialog.destroy()
            else:
                messagebox.showerror("Error", "Failed to save sequence")

        btn_frame = ttk.Frame(dialog)
        btn_frame.pack(pady=15)
        ttk.Button(btn_frame, text="Save", command=on_save).pack(side=tk.LEFT, padx=10)
        ttk.Button(btn_frame, text="Cancel", command=dialog.destroy).pack(side=tk.LEFT, padx=10)

    def load_sequence_dialog(self):
        """Open dialog to load a saved sequence"""
        dialog = tk.Toplevel(self.root)
        dialog.title("Load Sequence")
        dialog.geometry("450x350")
        dialog.transient(self.root)
        dialog.grab_set()

        ttk.Label(dialog, text="Load Saved Sequence",
                 font=('Helvetica', 12, 'bold')).pack(pady=10)

        # Sequence list
        list_frame = ttk.Frame(dialog)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)

        listbox = tk.Listbox(list_frame, font=('Helvetica', 11), height=10)
        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=listbox.yview)
        listbox.configure(yscrollcommand=scrollbar.set)
        listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Populate list
        sequences = self.sequence_manager.sequences
        for name in sorted(sequences.keys()):
            seq = sequences[name]
            listbox.insert(tk.END, f"{name} ({len(seq)} steps)")

        selected_seq = [None]

        def on_select(event):
            selection = listbox.curselection()
            if selection:
                selected_seq[0] = listbox.get(selection[0]).split(' (')[0]

        listbox.bind('<<ListboxSelect>>', on_select)

        def on_load():
            if selected_seq[0]:
                name = selected_seq[0]
                seq = self.sequence_manager.get_sequence(name)
                if seq:
                    self.sequence = seq
                    self.update_seq_listbox()
                    self.log(f"✓ Loaded sequence '{name}'")
                    dialog.destroy()
            else:
                messagebox.showwarning("Warning", "Please select a sequence")

        def on_delete():
            if selected_seq[0]:
                name = selected_seq[0]
                if name in DEFAULT_SEQUENCES:
                    messagebox.showwarning("Warning", "Cannot delete default sequences")
                    return
                if messagebox.askyesno("Confirm", f"Delete sequence '{name}'?"):
                    if self.sequence_manager.delete_sequence(name):
                        self.log(f"✓ Deleted sequence '{name}'")
                        # Refresh list
                        listbox.delete(0, tk.END)
                        for seq_name in sorted(self.sequence_manager.sequences.keys()):
                            seq = self.sequence_manager.sequences[seq_name]
                            listbox.insert(tk.END, f"{seq_name} ({len(seq)} steps)")
                    else:
                        messagebox.showerror("Error", "Failed to delete sequence")
            else:
                messagebox.showwarning("Warning", "Please select a sequence")

        btn_frame = ttk.Frame(dialog)
        btn_frame.pack(pady=15)
        ttk.Button(btn_frame, text="Load", command=on_load).pack(side=tk.LEFT, padx=10)
        ttk.Button(btn_frame, text="Delete", command=on_delete).pack(side=tk.LEFT, padx=10)
        ttk.Button(btn_frame, text="Cancel", command=dialog.destroy).pack(side=tk.LEFT, padx=10)

    def edit_seq_delay(self):
        """Edit delay for selected sequence item"""
        selection = self.seq_listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Please select an item to edit")
            return

        idx = selection[0]
        if idx >= len(self.sequence):
            return

        preset_name, current_delay = self.sequence[idx]

        # Create edit dialog
        dialog = tk.Toplevel(self.root)
        dialog.title(f"Edit Delay: {preset_name}")
        dialog.geometry("300x150")
        dialog.transient(self.root)
        dialog.grab_set()

        ttk.Label(dialog, text=f"Preset: {preset_name}",
                 font=('Helvetica', 11, 'bold')).pack(pady=10)

        ttk.Label(dialog, text="Delay after this step (milliseconds):").pack(pady=5)

        delay_var = tk.StringVar(value=str(current_delay))
        delay_entry = ttk.Entry(dialog, textvariable=delay_var, width=10,
                               font=('Helvetica', 12), justify='center')
        delay_entry.pack(pady=5)

        def on_save():
            try:
                new_delay = int(delay_var.get())
                if new_delay < 0:
                    new_delay = 0
                self.sequence[idx] = (preset_name, new_delay)
                self.update_seq_listbox()
                self.log(f"Updated delay for '{preset_name}': {new_delay}ms")
                dialog.destroy()
            except ValueError:
                messagebox.showwarning("Warning", "Please enter a valid number")

        btn_frame = ttk.Frame(dialog)
        btn_frame.pack(pady=15)
        ttk.Button(btn_frame, text="Save", command=on_save).pack(side=tk.LEFT, padx=10)
        ttk.Button(btn_frame, text="Cancel", command=dialog.destroy).pack(side=tk.LEFT, padx=10)

    # ========== End Sequence Builder Methods ==========

    def update_input_box(self, servo_num, angle):
        """Update input box to match Arduino position"""
        if 0 <= servo_num < 5:
            self.input_boxes[servo_num].set(str(angle))

    def toggle_connection(self):
        """Toggle serial connection"""
        if self.is_connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        """Establish serial connection"""
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No Port", "Please select a serial port")
            return

        if ' - ' in port:
            port = port.split(' - ')[0]

        try:
            self.serial_conn = serial.Serial(port, BAUD_RATE, timeout=1)
            self.is_connected = True
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text="Status: Connected", foreground="green")
            self.port_combo.config(state='disabled')
            self.log(f"✓ Connected to {port}")

            self.reading = True
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()

        except serial.SerialException as e:
            messagebox.showerror("Connection Error", f"Could not connect: {e}")
            self.log(f"Connection failed: {e}")

    def disconnect(self):
        """Close serial connection"""
        self.reading = False
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except:
                pass
        self.is_connected = False
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Status: Disconnected", foreground="red")
        self.port_combo.config(state='readonly')
        self.log("Disconnected")

    def read_serial(self):
        """Read responses from Arduino in background thread"""
        while self.reading:
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.root.after(0, lambda l=line: self.handle_arduino_response(l))
            except:
                break

    def handle_arduino_response(self, line):
        """Parse Arduino response and update input boxes"""
        self.log(f"Arduino: {line}")
        
        if "moved to:" in line:
            parts = line.split()
            if len(parts) >= 5:
                try:
                    servo_num = int(parts[1])
                    angle = int(parts[4])
                    self.update_input_box(servo_num, angle)
                except (ValueError, IndexError):
                    pass
        
        for i, name in enumerate(JOINT_NAMES):
            if line.startswith(f"{name}:") and "deg" in line:
                try:
                    angle = int(line.split(':')[1].strip().replace(' deg', ''))
                    self.update_input_box(i, angle)
                except (ValueError, IndexError):
                    pass

    def refresh_ports(self):
        """Refresh available serial ports"""
        ports = serial.tools.list_ports.comports()
        port_list = [f"{p.device} - {p.description}" for p in ports]
        self.port_combo['values'] = port_list
        self.ports_list = ports

        if port_list:
            for i, p in enumerate(ports):
                if ('Arduino' in p.description or 'CH340' in p.description or
                    'ttyACM' in p.device or 'ttyUSB' in p.device):
                    self.port_combo.current(i)
                    self.log(f"Found: {p.device}")
                    return
            self.port_combo.current(0)
        else:
            self.log("No serial ports found")

    def log(self, message):
        """Add message to log"""
        self.log_text.config(state='normal')
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)
        self.log_text.config(state='disabled')


def main():
    root = tk.Tk()
    app = RoboticArmController(root)
    root.mainloop()


if __name__ == "__main__":
    main()
