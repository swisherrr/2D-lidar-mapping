# ScannerGUI.py Tutorial

## Overview
`ScannerGUI.py` is a graphical user interface (GUI) application for real-time LiDAR scanning. It provides live visualization of scan data as it's being collected, with controls for starting/stopping scans and exporting data. The GUI displays a polar plot that updates in real-time, showing detected objects and their distances from the sensor.

## Prerequisites

### Hardware
- RPLiDAR sensor (A1, A2, or compatible model)
- USB connection to computer

### Software
- Python 3.7 or higher
- All dependencies from `DataCapture.py` and `PlotScan.py`

### Required Dependencies
- `rplidar==0.9.2` - RPLiDAR SDK
- `pyserial==3.5` - Serial port communication
- `matplotlib>=3.5.0` - Real-time plotting
- `numpy>=1.21.0` - Numerical operations
- `tkinter` - GUI framework (usually included with Python)

## Installation

1. **Ensure you're in the project directory**:
   ```bash
   cd <your-repo-directory>
   ```

2. **Install all required dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Verify tkinter is available** (usually pre-installed):
   ```bash
   python -c "import tkinter; print('tkinter available')"
   ```

## Usage

### Starting the GUI

**Windows**:
```bash
python ScannerGUI.py
```

**macOS/Linux**:
```bash
python3 ScannerGUI.py
```

The GUI window will open automatically.

## GUI Layout and Components

### Left Panel - Control Panel

#### 1. Device Port Selection
- **Port Dropdown**: Shows available COM/serial ports
- **Refresh Ports**: Updates the list of available ports
- **Auto-detect**: Automatically detects the LiDAR device
- **Test Connection**: Verifies the selected port is accessible

#### 2. Max Scan Radius
- **Spinbox Control**: Set maximum display radius (1000-50000mm)
- **Default**: 8000mm (8 meters)
- Adjusts the polar plot's radial axis

#### 3. Scan Controls
- **Start Scan** (Green): Begin real-time scanning
- **Stop Scan** (Red): Stop the current scan

#### 4. Export Options
- **Export CSV**: Save all collected scans to CSV format
- **Export SVG**: Save the polar plot as an SVG image

#### 5. Status Display
- **Status**: Current operation status
- **Scans**: Number of scans collected
- **Coverage**: Angular coverage of current scan (0-360°)

### Right Panel - Real-Time Visualization

**Polar Plot Display**:
- Live visualization of LiDAR scan data
- Blue dots represent detected objects/surfaces
- Concentric circles show distance markers
- Radial lines mark angles (0°, 45°, 90°, etc.)
- Interactive matplotlib toolbar for zoom/pan

## Step-by-Step Workflow

### 1. Connect the LiDAR

1. **Plug in** your RPLiDAR sensor via USB
2. Wait for the operating system to recognize it

### 2. Select the Device Port

**Option A: Auto-detect (Recommended)**
1. Click **"Auto-detect"** button
2. The GUI will automatically find and select the LiDAR port
3. Status will show: "Auto-detected: COM4" (or similar)

**Option B: Manual Selection**
1. Click **"Refresh Ports"** to update the port list
2. Select your device from the dropdown (e.g., COM4)
3. Click **"Test Connection"** to verify

**Option C: Manual Entry (if no ports detected)**
1. Type the port name directly into the dropdown box
2. Click **"Test Connection"** to verify

### 3. Configure Scan Settings

1. **Set Max Scan Radius**:
   - Adjust the spinbox to your environment size
   - For small rooms: 3000-5000mm
   - For large spaces: 8000-12000mm
   - This only affects visualization, not data collection

### 4. Start Scanning

1. Click the **"Start Scan"** button (green)
2. Watch the status change to "Starting scanner..."
3. The motor will spin up (takes ~1.5 seconds)
4. Real-time data appears on the polar plot
5. Status updates show:
   - "Scanning..." during operation
   - Scan count increases
   - Coverage percentage updates

### 5. Monitor the Scan

**What to look for**:
- **Blue dots**: Individual distance measurements
- **Dense clusters**: Walls, furniture, or obstacles
- **Gaps**: Open spaces or areas beyond sensor range
- **Coverage**: Should approach 360° for complete scans

**Plot Interactions**:
- **Zoom**: Click zoom icon, drag rectangle around area of interest
- **Pan**: Click pan icon, drag to move the view
- **Home**: Reset to original view
- **Save**: Export plot as PNG (separate from Export SVG)

### 6. Stop Scanning

1. Click the **"Stop Scan"** button (red)
2. The motor stops and data collection ends
3. All collected scans remain in memory

### 7. Export Data

**Export CSV**:
1. Click **"Export CSV"** button
2. Choose save location and filename
3. CSV contains all scans with timestamp, coordinates, and quality data
4. Format matches `DataCapture.py` output

**Export SVG**:
1. Click **"Export SVG"** button
2. Choose save location and filename
3. Saves a vector image of all collected scans
4. Scalable format suitable for reports/presentations

## Understanding the Display

### Polar Plot Interpretation

**Coordinate System**:
- **Center (0,0)**: LiDAR sensor position
- **0° (Right)**: East direction
- **90° (Top)**: North direction
- **180° (Left)**: West direction
- **270° (Bottom)**: South direction

**Distance Rings**:
- Concentric circles show fixed distances
- Default spacing: 1000mm (1 meter) increments
- Maximum radius adjustable via spinbox control

**Data Points**:
- Each blue dot = one distance measurement
- Brightness/density indicates scan quality
- Clustered points = solid surfaces
- Isolated points = small objects or noise

### Status Indicators

**Device Port Status**:
- "Auto-detected: COM4" - Successfully found device
- "Found X port(s)" - Multiple ports available
- "No ports detected" - Check USB connection
- "Port COM4 ready" - Connection verified

**Scanning Status**:
- "Ready" - Idle, ready to scan
- "Starting scanner..." - Initializing device
- "Scanner ready - collecting data..." - Warm-up phase
- "Scanning..." - Active data collection
- "Stopping scanner..." - Shutdown in progress
- "Scan stopped" - Collection complete

**Coverage Values**:
- 0-180°: Partial scan, objects blocking view
- 180-320°: Good coverage with some gaps
- 320-360°: Excellent full coverage

## Troubleshooting

### Port Connection Issues

**Problem**: "No ports detected - Enter COM port manually"

**Solutions**:
1. Check USB cable connection
2. Verify device appears in Device Manager (Windows) or `/dev/` (Linux/macOS)
3. Install/update USB drivers
4. Try different USB port
5. Manually enter port name (e.g., "COM4")

**Problem**: "Port 'COM4' not found"

**Solutions**:
1. Click "Refresh Ports" to update the list
2. Verify correct port number in Device Manager
3. Close other applications using the port
4. Restart the computer and reconnect device

**Problem**: Test Connection fails

**Solutions**:
1. Ensure no other program is using the LiDAR
2. Close DataCapture.py or other scanner applications
3. Check port permissions (Linux/macOS):
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```
4. Add user to dialout group (Linux):
   ```bash
   sudo usermod -a -G dialout $USER
   ```

### Scanning Issues

**Problem**: Start Scan button does nothing

**Solutions**:
- Ensure a port is selected in the dropdown
- Click "Test Connection" to verify port accessibility
- Check that RPLidar library is installed:
  ```bash
  pip install rplidar
  ```

**Problem**: "Connection failed" error when starting scan

**Solutions**:
1. Verify LiDAR is powered (should have spinning/flashing light)
2. Check USB cable is fully connected
3. Try unplugging and reconnecting the device
4. Restart the GUI application
5. Test with DataCapture.py to isolate the issue

**Problem**: Plot shows no data or very sparse data

**Solutions**:
- Check "Coverage" value - should be >320°
- Ensure LiDAR has clear 360° view (no obstructions)
- Increase Max Scan Radius if objects are far away
- Lower MIN_QUALITY in DataCapture.py if needed
- Clean LiDAR lens/window

**Problem**: Plot doesn't update in real-time

**Solutions**:
1. Check if "Scans" counter is increasing
2. Verify matplotlib is installed correctly
3. Computer may be too slow - reduce other programs
4. Try restarting the scan

### Export Issues

**Problem**: "No scan data to export"

**Solutions**:
- Must start and collect at least one scan before exporting
- Ensure scanning completed (saw data on plot)
- Restart scan and wait for data to appear

**Problem**: CSV export fails

**Solutions**:
1. Check disk space
2. Ensure write permissions for save location
3. Close the CSV file if it's open in Excel/other programs
4. Try saving to a different location

**Problem**: SVG export produces blank image

**Solutions**:
- Ensure matplotlib is installed: `pip install matplotlib`
- Verify scan data exists (check plot display)
- Try exporting after stopping the scan

## Advanced Features

### Adjusting Scan Parameters

The GUI uses parameters from `DataCapture.py`. To modify behavior:

1. **Edit DataCapture.py** parameters:
   ```python
   MIN_QUALITY = 10      # Increase for cleaner scans
   MIN_COVERAGE_DEG = 300.0  # Lower for faster scans
   ```

2. **Restart ScannerGUI.py** to apply changes

### Multiple Scan Sessions

The GUI accumulates all scans in memory:
- Each "Start Scan" → "Stop Scan" cycle adds to the collection
- "Scans" counter shows total accumulated
- Export includes ALL scans from current session
- Restart GUI to clear and begin fresh

### Real-Time Monitoring

Best practices for continuous monitoring:
1. Set appropriate Max Scan Radius for your space
2. Start scanning and let it run
3. Watch Coverage value - should stay >320°
4. Monitor for unexpected objects (new dots appearing)
5. Use matplotlib toolbar to zoom into areas of interest

### When to Use Each Tool

**Use DataCapture.py when**:
- Running automated/scheduled scans
- Deploying on headless systems
- Need minimal resource usage
- Scripting/batch processing

**Use PlotScan.py when**:
- Analyzing previously captured data
- Creating publication-quality plots
- Comparing multiple scans
- Don't need real-time feedback

**Use ScannerGUI.py when**:
- Need real-time visualization
- Interactive exploration and testing
- Adjusting sensor placement
- Demonstrating to others
- Quick data collection with visual feedback

## Technical Notes

### Threading
- Scanning runs in a separate thread to keep GUI responsive
- Main thread handles UI updates and plot rendering
- Auto-cleanup when closing window

### Memory Usage
- All scans stored in memory during session
- Large numbers of scans (100+) may use significant RAM
- Export and restart if memory becomes a concern

### Plot Update Rate
- Updates every scan rotation (~5-10 Hz depending on LiDAR model)
- Plot rendering may slow down with many points
- Performance depends on computer hardware

## Tips and Best Practices

1. **Always test connection** before starting a long scan
2. **Start with default radius** (8000mm), adjust as needed
3. **Watch coverage value** - aim for >320° for quality data
4. **Export frequently** for important scans (data not auto-saved)
5. **Position sensor carefully** - clear 360° view gives best results
6. **Use Stop button** before closing GUI (cleaner shutdown)
7. **Check status messages** for troubleshooting hints

## Limitations

- No undo function - stopping a scan cannot be resumed
- Cannot edit/filter data in GUI (use PlotScan.py for that)
- Export saves all scans, not individual scans
- Real-time plot performance limited by computer speed
- Must manually export - no auto-save functionality

## Next Steps

After using ScannerGUI.py:
- Export CSV for detailed analysis
- Use PlotScan.py to visualize specific scans
- Process data for mapping or obstacle detection
- Integrate with robotics or navigation systems
