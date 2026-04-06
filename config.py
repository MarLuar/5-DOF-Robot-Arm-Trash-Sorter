"""
Cross-platform path configuration module.
Automatically detects OS and provides correct paths.
"""
import os
import sys
from pathlib import Path

# Get the project root directory (where this config.py is located)
PROJECT_ROOT = Path(__file__).parent.resolve()

# Platform detection
IS_WINDOWS = sys.platform == 'win32'
IS_LINUX = sys.platform.startswith('linux')
IS_MACOS = sys.platform == 'darwin'

# Calibration files
CALIBRATION_DIR = PROJECT_ROOT / 'calibration'
CALIBRATION_FILE = CALIBRATION_DIR / 'vision_calibration.json'
PRESETS_FILE = CALIBRATION_DIR / 'servo_presets.json'
CAMERA_CONFIG_FILE = CALIBRATION_DIR / 'camera_config.json'

# Sequences
SEQUENCES_DIR = PROJECT_ROOT / 'sequences'
SEQUENCES_FILE = SEQUENCES_DIR / 'cell_sequences.json'

# Models
MODELS_DIR = PROJECT_ROOT / 'models'

# Dataset
DATASET_DIR = PROJECT_ROOT / 'dataset'
DATASET_CLASSIFICATION_DIR = PROJECT_ROOT / 'dataset_classification'

# Empty grid reference (in project root for cross-platform compatibility)
BG_FILE = PROJECT_ROOT / 'empty_grid_reference.jpg'

# Legacy support: paths that might be in user's home directory
# On first run, these will be created in the project directory
# but we check the old location for backwards compatibility
def get_legacy_path(filename):
    """Check legacy home directory path, fallback to project root."""
    legacy_path = Path.home() / filename
    if legacy_path.exists():
        return str(legacy_path)
    return str(PROJECT_ROOT / filename)

# For backwards compatibility, but prefer project root
BG_FILE_LEGACY = get_legacy_path('empty_grid_reference.jpg')


def get_camera_device_path(index):
    """
    Get camera device path based on platform.
    On Linux: /dev/video{index}
    On Windows: Just return the index (OpenCV handles it)
    On macOS: Just return the index (OpenCV handles it)
    """
    if IS_LINUX:
        return f'/dev/video{index}'
    else:
        # Windows and macOS use camera indices directly
        return index


def enumerate_camera_devices():
    """
    Enumerate available camera devices based on platform.
    Returns list of (index, description) tuples.
    """
    cameras = []
    
    if IS_LINUX:
        import glob
        devices = sorted(glob.glob('/dev/video[0-9]*'))
        for dev in devices:
            try:
                idx = int(dev.replace('/dev/video', ''))
                cameras.append((idx, dev))
            except ValueError:
                continue
    else:
        # Windows/macOS: test common camera indices
        # OpenCV will validate these when opening
        for idx in range(5):  # Test indices 0-4
            cameras.append((idx, f'Camera {idx}'))
    
    return cameras


def get_serial_ports():
    """
    Get list of available serial ports based on platform.
    """
    import serial.tools.list_ports
    
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]


def get_default_serial_port():
    """
    Get the most likely Arduino serial port based on platform.
    """
    if IS_WINDOWS:
        # Windows: COM ports (try to find Arduino)
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'Arduino' in port.description or 'CH340' in port.description or 'USB Serial' in port.description:
                return port.device
        # Fallback to first available COM port
        if ports:
            return ports[0].device
        return 'COM3'  # Common default
    elif IS_LINUX:
        # Linux: /dev/ttyUSB0 or /dev/ttyACM0
        import glob
        usb_ports = glob.glob('/dev/ttyUSB*')
        if usb_ports:
            return usb_ports[0]
        acm_ports = glob.glob('/dev/ttyACM*')
        if acm_ports:
            return acm_ports[0]
        return '/dev/ttyUSB0'
    elif IS_MACOS:
        # macOS: /dev/cu.usbmodem* or /dev/cu.usbserial*
        import glob
        ports = glob.glob('/dev/cu.usbmodem*') + glob.glob('/dev/cu.usbserial*')
        if ports:
            return ports[0]
        return '/dev/cu.usbmodem14101'
    
    return None


def ensure_directories():
    """Create necessary directories if they don't exist."""
    for dir_path in [CALIBRATION_DIR, SEQUENCES_DIR, MODELS_DIR]:
        dir_path.mkdir(parents=True, exist_ok=True)


# Auto-create directories on import
ensure_directories()
