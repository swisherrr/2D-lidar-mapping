DataCapture.py Tutorial
Overview

DataCapture.py is a Python script that interfaces with an RPLiDAR sensor to capture 360-degree environmental scans and save them to a CSV file. The script automatically detects the LiDAR device, manages the sensor's motor, and processes scan data into Cartesian coordinates.
Prerequisites
Hardware

    RPLiDAR sensor (A1, A2, or compatible model)
    USB connection to computer

Software

    Python 3.7 or higher
    Required packages (see Installation section)

Installation

    Clone the repository (if not already done):

bash

   git clone [<your-repo-url>](https://github.com/swisherrr/2D-lidar-mapping.git)
   cd <your-repo-directory>

    Install required dependencies:

bash

   pip install -r requirements.txt

Key dependencies for DataCapture.py:

    rplidar==0.9.2 - RPLiDAR SDK
    pyserial==3.5 - Serial port communication

Configuration

The script includes several configurable parameters at the top of the file:
Device Settings
python

DEVICE = None  # Set to None for auto-detection or specify port manually

    Auto-detection: Leave as None (recommended)
    Manual override: Set to specific port (e.g., "COM4", "/dev/cu.usbserial-0001")

Default Ports by Operating System

    Windows: COM4
    macOS: /dev/cu.usbserial-0001
    Linux: /dev/ttyUSB0

Scan Parameters

Parameter	    Default	    Description
TARGET_SCANS	    1	    Number of full rotations to capture
MIN_QUALITY	        0	    Minimum quality threshold (0-15)
MIN_COVERAGE_DEG	320.0	Minimum angular coverage required
MAX_ATTEMPTS	    20	    Maximum attempts per scan
WARMUP_SCANS	    3	    Initial scans to discard
SPINUP_S	        1.5	    Motor spin-up time in seconds

Output Settings
python

OUT = "scans.csv"  # Output filename

Usage
Basic Usage

    Connect your RPLiDAR sensor via USB
    Run the script:

bash

   python DataCapture.py

    Expected output:

   [info] Using device: COM4
   [info] Saved sweep #0 coverage=343.3° attempts=1
   [ok] Saved 1 scan(s) to C:\Users\...\scans.csv
   Process finished with exit code 0

Capturing Multiple Scans

Edit the TARGET_SCANS parameter:
python

TARGET_SCANS = 5  # Capture 5 full rotations

Adjusting Quality Filtering

To filter out low-quality readings:
python

MIN_QUALITY = 10  # Only keep readings with quality ≥ 10

Output Format

The script generates a CSV file (scans.csv) with the following columns:

Column	    Description	                    Units
timestamp	ISO 8601 timestamp of the scan	-
scan_idx	Scan number (0-indexed)	        -
meas_idx	Measurement index within scan	-
angle_deg	Angle from sensor	            degrees (0-360°)
distance_mm	Distance from sensor	        millimeters
quality	    Signal quality	                0-15
x_mm	    Cartesian X coordinate	        millimeters
y_mm	    Cartesian Y coordinate	        millimeters

Sample Output
csv

timestamp,scan_idx,meas_idx,angle_deg,distance_mm,quality,x_mm,y_mm
2025-11-23T17:03:45.123Z,0,0,0.125,2450.0,15,2450.0,5.3
2025-11-23T17:03:45.123Z,0,1,0.875,2448.5,15,2447.9,37.4

How It Works
1. Device Detection

The script automatically detects the LiDAR by:

    Scanning all available serial ports
    Matching against known USB identifiers (CP210x, Silicon Labs, etc.)
    Falling back to OS-specific defaults

2. Motor Control

    Starts the motor and waits for spin-up (SPINUP_S)
    Discards initial warm-up scans (WARMUP_SCANS)
    Stops motor gracefully on completion

3. Data Collection

For each scan:

    Attempts to capture a full 360° rotation
    Filters readings based on MIN_QUALITY
    Validates angular coverage (must meet MIN_COVERAGE_DEG)
    Retries up to MAX_ATTEMPTS if coverage is insufficient
    Keeps the best attempt (highest coverage)

4. Coordinate Conversion

Raw polar coordinates (angle, distance) are converted to Cartesian (x, y):
python

x = distance × cos(angle)
y = distance × sin(angle)

Troubleshooting
Device Not Found

Error: Script can't detect the LiDAR

Solutions:

    Check USB connection
    Verify device appears in Device Manager (Windows) or ls /dev/tty* (macOS/Linux)
    Manually set the DEVICE parameter:

python

   DEVICE = "COM5"  # Use your actual port

Low Coverage Warning

Issue: Coverage less than MIN_COVERAGE_DEG

Solutions:

    Ensure LiDAR has clear line of sight (360°)
    Lower MIN_COVERAGE_DEG threshold
    Increase MAX_ATTEMPTS
    Check for mechanical obstructions

Timeout Errors

Error: Timeout or no data received

Solutions:

    Increase TIMEOUT parameter
    Check USB cable quality
    Try a different USB port
    Verify correct BAUD rate (115200)

Permission Denied (Linux/macOS)

Error: Cannot access /dev/ttyUSB0 or /dev/cu.*

Solutions:
bash

# Linux: Add user to dialout group
sudo usermod -a -G dialout $USER

# macOS: Check system preferences for USB permissions

Advanced Configuration
Custom Output Location
python

OUT = "data/scan_2025-11-23.csv"

The script automatically creates parent directories if needed.
Platform-Specific Settings

If you're working across multiple operating systems, the script handles this automatically. To force a specific platform:
python

# Force Windows settings
WIN_DEVICE = "COM5"

# Force macOS settings  
MAC_DEVICE = "/dev/cu.usbserial-0002"

Integration with Other Scripts

The generated scans.csv can be used with:

    PlotScan.py - Visualize the captured data
    ScannerGUI.py - GUI-based scanning interface

Technical Notes

    Coordinate System: Origin (0,0) is at the LiDAR sensor position
    Angle Convention: 0° is directly in front of the sensor, increases counter-clockwise
    Quality Values: Range from 0 (poor) to 15 (excellent)
    Distance Range: Typically 150mm to 12,000mm depending on LiDAR model

Example Use Cases

    Room Mapping: Capture single scan with high coverage

python

   TARGET_SCANS = 1
   MIN_COVERAGE_DEG = 350.0

    Continuous Monitoring: Multiple scans over time

python

   TARGET_SCANS = 100
   OUT = "monitoring_scans.csv"

    High-Precision Scanning: Filter for quality readings

python

   MIN_QUALITY = 12
   MAX_ATTEMPTS = 50
