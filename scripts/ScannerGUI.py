"""Real-time LiDAR Scanner Control GUI"""

from __future__ import annotations


import csv  
import math  
import threading  
import time  
import platform  
from pathlib import Path  
from typing import List, Tuple, Optional  


from tkinter import (
    Tk, Frame, Button, Label, StringVar, filedialog, messagebox, ttk
)
import tkinter as tk

# Import functions and constants from DataCapture.py
import sys
import os
scripts_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, scripts_dir)

try:
    import DataCapture as dc
    from DataCapture import (
        BAUD, TIMEOUT, MIN_QUALITY, SPINUP_S, WARMUP_SCANS,
        MIN_COVERAGE_DEG, MAX_ATTEMPTS, xy, time_ms, angular_span_deg,
        detect_port, check_device, USB_HINTS, WIN_DEVICE, MAC_DEVICE, LINUX_DEVICE
    )
    try:
        from serial.tools import list_ports
    except ImportError:
        list_ports = getattr(dc, 'list_ports', None)
except (ImportError, AttributeError):
    # Fallback if DataCapture.py is missing or imports fail
    BAUD = 115200
    TIMEOUT = 1
    MIN_QUALITY = 0
    SPINUP_S = 1.5
    WARMUP_SCANS = 3
    MIN_COVERAGE_DEG = 320.0
    MAX_ATTEMPTS = 20
    # convert polar (angle/dist) to cartesian (x/y)
    def xy(angle_deg, dist_mm):
        th = math.radians(angle_deg % 360.0)
        return dist_mm * math.cos(th), dist_mm * math.sin(th)
    # get current time string
    def time_ms():
        t = time.time()
        return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(t)) + f".{int((t % 1) * 1e3):03d}Z"
    # calculate angular coverage of a scan
    def angular_span_deg(angles):
        if not angles:
            return 0.0
        a = sorted((ang % 360.0) for ang in angles)
        gaps = []
        for i in range(len(a)):
            current = a[i]
            next_angle = a[(i + 1) % len(a)]
            diff = (next_angle - current) % 360.0
            gaps.append(diff)
        return 360.0 - max(gaps)
    def detect_port():
        return "COM4"
    def check_device():
        return "COM4"
    USB_HINTS = ()
    WIN_DEVICE = "COM4"
    MAC_DEVICE = "/dev/cu.usbserial-0001"
    LINUX_DEVICE = "/dev/ttyUSB0"
    list_ports = None


try:
    from rplidar import RPLidar 
except ImportError:
    RPLidar = None 

try:
    import matplotlib
    matplotlib.use('TkAgg')
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
    import numpy as np
except ImportError:
    Figure = None


class ScannerGUI:
    """Main GUI application for LiDAR scanner control."""
    
    def __init__(self, root: Tk):
        self.root = root
        self.root.title("LiDAR Scanner Control")
        self.root.geometry("1000x700")
        
        # Scanner state initialization
        self.lidar: Optional[RPLidar] = None
        self.scanning = False  # Flag to control scan loop
        self.scan_thread: Optional[threading.Thread] = None  # Background thread for scanning
        self.current_scan_data: List[Tuple[float, float, float]] = []  # Stores current frame: (quality, angle, distance)
        self.all_scans: List[List[Tuple[float, float, float]]] = []  # History of all captured scans
        self.scan_idx = 0  # Counter for number of scans taken
        self.device_port: Optional[str] = None  # Selected serial port
        self.scan_buffer: List[List[Tuple[float, float, float]]] = []  # Buffer for temporal averaging
        self.min_quality = 10  # Default quality threshold (hardware gives 0-63)
        self.min_distance = 100  # Default min distance in mm (filtering near-field noise)
        self.max_distance = 8000  # Default max distance in mm (8 meters)
        self.remove_outliers = True  # Flag to enable/disable Z-score filtering
        self.scan_averaging = False  # Flag to enable/disable temporal averaging
        self.avg_count = 3  # Number of frames to average together
        
        # Status variables for UI updates
        self.status_var = StringVar(value="Ready")
        self.scan_count_var = StringVar(value="Scans: 0")
        self.coverage_var = StringVar(value="Coverage: 0.0°")
        self.quality_stats_var = StringVar(value="Quality: N/A")
        

        self.setup_gui()
        
        # Initialize port list and auto-detect device on startup
        self.refresh_ports()
        self.auto_detect_device()
        self.device_var.trace('w', self.on_port_selected)
    
    def setup_gui(self):
        """Create and layout GUI components."""
        main_frame = Frame(self.root, padx=10, pady=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        control_frame = Frame(main_frame)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        
        device_frame = Frame(control_frame)
        device_frame.pack(fill=tk.X, pady=(0, 10))
        
        Label(device_frame, text="Device Port:", font=("Arial", 10, "bold")).pack(anchor=tk.W)
        self.device_var = StringVar()
        self.port_combo = ttk.Combobox(device_frame, textvariable=self.device_var, width=20)
        self.port_combo.pack(fill=tk.X, pady=(5, 5))
        
        help_label = Label(
            device_frame,
            text="If no ports detected, enter COM port manually",
            font=("Arial", 8),
            fg="gray",
            justify=tk.LEFT
        )
        help_label.pack(anchor=tk.W, pady=(0, 5))
        
        device_button_frame = Frame(device_frame)
        device_button_frame.pack(fill=tk.X)
        
        Button(device_button_frame, text="Refresh Ports", command=self.refresh_ports, padx=5, pady=2).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2))
        Button(device_button_frame, text="Auto-detect", command=self.auto_detect_device, padx=5, pady=2).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(2, 0))
        
        Button(device_frame, text="Test Connection", command=self.test_connection, padx=5, pady=2, bg="#2196F3", fg="white").pack(fill=tk.X, pady=(5, 0))
        
        radius_frame = Frame(control_frame)
        radius_frame.pack(fill=tk.X, pady=(10, 0))
        
        Label(radius_frame, text="Max Scan Radius:", font=("Arial", 10, "bold")).pack(anchor=tk.W)
        
        radius_control_frame = Frame(radius_frame)
        radius_control_frame.pack(fill=tk.X, pady=(5, 0))
        
        self.radius_var = StringVar(value="8000")
        radius_spinbox = ttk.Spinbox(
            radius_control_frame,
            from_=1000,
            to=50000,
            textvariable=self.radius_var,
            width=10,
            command=self.update_radius
        )
        radius_spinbox.pack(side=tk.LEFT, padx=(0, 5))
        
        Label(radius_control_frame, text="mm").pack(side=tk.LEFT)
        
        self.radius_var.trace('w', lambda *args: self.update_radius())
        
        # Quality threshold control section
        quality_frame = Frame(control_frame)
        quality_frame.pack(fill=tk.X, pady=(10, 0))
        
        Label(quality_frame, text="Min Quality:", font=("Arial", 10, "bold")).pack(anchor=tk.W)
        
        quality_control_frame = Frame(quality_frame)
        quality_control_frame.pack(fill=tk.X, pady=(5, 0))
        
        self.quality_var = StringVar(value=str(self.min_quality))
        quality_spinbox = ttk.Spinbox(
            quality_control_frame,
            from_=0,
            to=63,
            textvariable=self.quality_var,
            width=10
        )
        quality_spinbox.pack(side=tk.LEFT, padx=(0, 5))
        
        Label(quality_control_frame, text="(0-15)", font=("Arial", 8), fg="gray").pack(side=tk.LEFT)
        
        self.quality_var.trace('w', lambda *args: self.update_quality())
        
        button_frame = Frame(control_frame)
        button_frame.pack(fill=tk.X, pady=(10, 0)) 
        
        # Start Button - triggers the scan thread
        self.start_button = Button(
            button_frame, text="Start Scan", command=self.start_scan,
            bg="#4CAF50", fg="white", font=("Arial", 11, "bold"),
            padx=20, pady=10
        )
        self.start_button.pack(fill=tk.X, pady=(0, 5))
        
        # Stop Button - sets the flag to stop the thread
        self.stop_button = Button(
            button_frame, text="Stop Scan", command=self.stop_scan,
            bg="#f44336", fg="white", font=("Arial", 11, "bold"),
            padx=20, pady=10, state=tk.DISABLED 
        )
        self.stop_button.pack(fill=tk.X, pady=(0, 10))
        

        export_frame = Frame(control_frame)
        export_frame.pack(fill=tk.X, pady=(10, 0))
        
        Label(export_frame, text="Export:", font=("Arial", 10, "bold")).pack(anchor=tk.W)

        Button(
            export_frame, text="Export CSV", command=self.export_csv,
            padx=10, pady=5
        ).pack(fill=tk.X, pady=(5, 5))
        
        Button(
            export_frame, text="Export SVG", command=self.export_svg,
            padx=10, pady=5
        ).pack(fill=tk.X, pady=(0, 10))
        
        status_frame = Frame(control_frame)
        status_frame.pack(fill=tk.X, pady=(10, 0))
        
        Label(status_frame, text="Status:", font=("Arial", 10, "bold")).pack(anchor=tk.W)
        
        status_label = Label(
            status_frame, textvariable=self.status_var,
            relief=tk.SUNKEN, anchor=tk.W, padx=5, pady=2, bg="#e0e0e0"
        )
        status_label.pack(fill=tk.X, pady=(5, 5))
        

        Label(status_frame, textvariable=self.scan_count_var, anchor=tk.W).pack(fill=tk.X, pady=(0, 2))
        Label(status_frame, textvariable=self.coverage_var, anchor=tk.W).pack(fill=tk.X, pady=(0, 2))
        Label(status_frame, textvariable=self.quality_stats_var, anchor=tk.W, font=("Arial", 8), fg="gray").pack(fill=tk.X)
        

        plot_frame = Frame(main_frame)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        

        if Figure is not None:
            self.fig = Figure(figsize=(8, 8), dpi=100) 
            self.ax = self.fig.add_subplot(111, projection='polar') 
            self.ax.set_title("LiDAR Scan - Real-time")
            self.ax.grid(True, linestyle=":", linewidth=0.5)
            
            try:
                self.max_radius = float(self.radius_var.get())
            except (ValueError, AttributeError):
                self.max_radius = 8000  
            self.ax.set_ylim(0, self.max_radius)
            self.scatter = None  
            

            self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
            self.canvas.draw()

            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            

            toolbar = NavigationToolbar2Tk(self.canvas, plot_frame)
            toolbar.update()
        else:
            Label(plot_frame, text="Matplotlib not available", fg="red").pack()
    
    def get_available_ports(self) -> List[str]:
        """Get list of available serial ports for device connection."""
        ports = []
        if list_ports is None:
            return [] 
        
        try:
            available = list(list_ports.comports())
            ports = [p.device for p in available]
            
            if not ports and platform.system().lower().startswith("win"):
                try:
                    import winreg
                    key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
                    i = 0
                    while True:
                        try:
                            name, value, _ = winreg.EnumValue(key, i)
                            if value.startswith("COM") and value not in ports:
                                ports.append(value)
                            i += 1
                        except (WindowsError, OSError):
                            break
                    winreg.CloseKey(key)
                except Exception:
                    pass 
            
        except Exception:
            return [] 
        
        return sorted(ports)
    
    def refresh_ports(self):
        """Refresh the list of available ports in the dropdown."""
        ports = self.get_available_ports()
        self.port_combo['values'] = ports
        
        if ports:
            self.port_combo.config(state="readonly")  
            current = self.device_var.get()
            if current in ports:
                self.device_var.set(current)
            else:
                self.device_var.set(ports[0])
                self.device_port = ports[0]
            self.status_var.set(f"Found {len(ports)} port(s): {', '.join(ports)}")
        else:
            self.port_combo.config(state="normal")
            current = self.device_var.get()
            if not current:
                self.device_var.set("")
                self.device_port = None
            self.status_var.set("No ports detected - Enter COM port manually")
    
    def auto_detect_device(self):
        """Auto-detect and set the LiDAR device port based on HWID hints."""
        try:
            self.refresh_ports()
            
            available_ports = self.get_available_ports()
            
            if not available_ports:
                self.status_var.set("No ports available")
                return
            
            try:
                detected_port = detect_port()
                if detected_port in available_ports:
                    self.device_var.set(detected_port)
                    self.device_port = detected_port
                    self.status_var.set(f"Auto-detected: {detected_port}")
                elif available_ports:
                    self.device_var.set(available_ports[0])
                    self.device_port = available_ports[0]
                    self.status_var.set(f"Selected: {available_ports[0]}")
            except Exception:
                if available_ports:
                    self.device_var.set(available_ports[0])
                    self.device_port = available_ports[0]
                    self.status_var.set(f"Selected: {available_ports[0]}")
        except Exception:
            pass
    
    def on_port_selected(self, *args):
        """Handle event when user selects a different port."""
        selected = self.device_var.get()
        if selected:
            self.device_port = selected  # Update internal state
    
    def update_radius(self):
        """Update the maximum scan radius for the plot visualization."""
        try:
            new_radius = float(self.radius_var.get())
            if new_radius > 0:
                self.max_radius = new_radius
                # Update plot axis limits immediately if plot exists
                if hasattr(self, 'ax') and self.ax is not None:
                    self.ax.set_ylim(0, self.max_radius)
                    self.canvas.draw()
        except (ValueError, AttributeError):
            pass
    
    def update_quality(self):
        """Update the minimum quality threshold variable."""
        try:
            new_quality = int(self.quality_var.get())
            if 0 <= new_quality <= 63:
                self.min_quality = new_quality
        except (ValueError, AttributeError):
            pass
    
    def filter_scan_data(self, scan_data: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
        """
        Apply various filters to improve scan accuracy.
        Returns filtered scan data.
        Pipeline: Quality Check -> Range Check -> Statistical Outlier Removal
        """
        if not scan_data:
            return []
        
        try:
            # Create a copy to avoid modifying original iterator data
            filtered = scan_data.copy()
            
            # 1. Quality threshold filter
            # Reject points where signal quality < min_quality (weak return signal)
            original_count = len(filtered)
            filtered = [(q, ang, dist) for q, ang, dist in filtered if q >= self.min_quality]
            
            if original_count > 0:
                removed = original_count - len(filtered)
                if removed > 0:
                    print(f"Quality filter: removed {removed}/{original_count} points (threshold={self.min_quality})")
            
            # Warn if filtering removed all points (common issue if threshold set too high)
            if original_count > 0 and len(filtered) == 0:
                self.root.after(0, lambda: self.status_var.set(
                    f"Warning: Quality filter (min={self.min_quality}) removed all points!"
                ))
            
            # 2. Distance range filter
            # Reject points that are outside the specified valid range
            filtered = [(q, ang, dist) for q, ang, dist in filtered if self.min_distance <= dist <= self.max_distance]
            
            # 3. Outlier detection using statistical filtering (Z-Score)
            # Only run if we have enough points for valid statistics
            if self.remove_outliers and len(filtered) > 3:
                filtered = self.remove_outliers_statistical(filtered)
            
            return filtered
        except Exception as e:
            # Fail-safe: If filtering crashes, return original data instead of crashing app
            print(f"Filter error: {e}")
            import traceback
            traceback.print_exc()
            return scan_data
    
    def remove_outliers_statistical(self, scan_data: List[Tuple[float, float, float]], 
                       z_threshold: float = 2.5) -> List[Tuple[float, float, float]]:
        """
        Remove outliers using Z-score method on distances.
        Points with distances that deviate significantly from neighbors are removed.
        This effectively cleans 'salt-and-pepper' noise from the scan.
        """
        if len(scan_data) < 3:
            return scan_data
        
        # Sort by angle to find spatial neighbors (since it's a 2D polar scan)
        sorted_data = sorted(scan_data, key=lambda t: t[1])
        
        filtered = []
        # Adaptive window size based on point density (look at neighbors)
        window_size = min(5, len(sorted_data) // 4)
        
        for i in range(len(sorted_data)):
            # Define neighborhood window indices
            start_idx = max(0, i - window_size)
            end_idx = min(len(sorted_data), i + window_size + 1)
            neighbors = sorted_data[start_idx:end_idx]
            
            if len(neighbors) < 2:
                filtered.append(sorted_data[i])
                continue
            
            # Calculate mean and variance of neighbor distances
            neighbor_dists = [dist for _, _, dist in neighbors]
            mean_dist = sum(neighbor_dists) / len(neighbor_dists)
            variance = sum((d - mean_dist) ** 2 for d in neighbor_dists) / len(neighbor_dists)
            std_dist = math.sqrt(variance) if variance > 0 else 1.0
            
            # Check if current point is a statistical outlier (Z-score > threshold)
            current_dist = sorted_data[i][2]
            if std_dist > 0:
                z_score = abs(current_dist - mean_dist) / std_dist
                if z_score <= z_threshold:
                    filtered.append(sorted_data[i])  # Keep valid point
            else:
                filtered.append(sorted_data[i])  # Keep valid point? (fallback)
        
        return filtered
    
    def average_scans(self, scans: List[List[Tuple[float, float, float]]]) -> List[Tuple[float, float, float]]:
        """
        Average multiple scans by angle binning.
        This reduces noise and improves accuracy (Temporal Averaging).
        """
        if not scans:
            return []
        
        # Create angle bins (1 degree resolution) to group points from different frames
        angle_bins = {}
        for scan in scans:
            for q, ang, dist in scan:
                bin_angle = round(ang)  # Bin to nearest integer degree
                if bin_angle not in angle_bins:
                    angle_bins[bin_angle] = []
                angle_bins[bin_angle].append((q, dist))
        
        # Compute representative point for each bin
        averaged = []
        for angle, points in angle_bins.items():
            if points:
                # Use MEDIAN for distance 
                distances = [dist for _, dist in points]
                qualities = [q for q, _ in points]
                distances.sort()
                qualities.sort()
                
                median_dist = distances[len(distances) // 2]
                median_quality = qualities[len(qualities) // 2]
                
                averaged.append((median_quality, float(angle), median_dist))
        
        # Return sorted list for plotting
        return sorted(averaged, key=lambda t: t[1])
    
    def test_connection(self):
        """Test if the selected port can be opened."""
        port = self.device_var.get().strip()
        if not port:
            messagebox.showwarning("No Port", "Please enter a COM port (e.g., COM3)")
            return
        
        # Normalize port name (add COM if just number, e.g. "4" -> "COM4")
        if port.upper().startswith("COM"):
            test_port = port.upper()
        elif port.isdigit():
            test_port = f"COM{port}"
        else:
            test_port = port.upper()
        
        self.status_var.set(f"Testing connection to {test_port}...")
        
        # Try to open the port to verify availability
        try:
            try:
                import serial
            except ImportError:
                messagebox.showerror("Error", "pyserial not installed. Please install it with: pip install pyserial")
                return
            
            # Attempt a brief connection
            ser = serial.Serial(test_port, baudrate=BAUD, timeout=0.5)
            ser.close()  # Close immediately if successful
            messagebox.showinfo("Success", f"Port {test_port} is available")
            self.device_var.set(test_port)
            self.device_port = test_port
            self.status_var.set(f"Port {test_port} ready")
        except serial.SerialException:
            # Handle standard serial errors (port busy, not found, permission denied)
            messagebox.showerror(
                "Port Not Found",
                f"Port {test_port} not found.\n\n"
                "Please check:\n"
                "- Device is connected\n"
                "- Drivers are installed\n"
                "- Port number is correct"
            )
            self.status_var.set(f"Port {test_port} unavailable")
        except Exception:
            # Handle unexpected errors
            messagebox.showerror("Error", "Could not test port connection")
            self.status_var.set("Test failed")
    
    def start_scan(self):
        """Start scanning in a separate thread."""
        if self.scanning:
            return  # Function is idempotent
        
        # Get selected port from combobox
        selected_port = self.device_var.get()
        if not selected_port:
            messagebox.showerror("Error", "No device port selected. Please select a port from the dropdown.")
            return
        
        # Verify port is still available (it might have been unplugged)
        available_ports = self.get_available_ports()
        if selected_port not in available_ports:
            messagebox.showerror(
                "Port Not Available",
                f"Port '{selected_port}' is no longer available.\n\nPlease:\n1. Click 'Refresh Ports' to update the list\n2. Select an available port"
            )
            self.refresh_ports()
            return
        
        self.device_port = selected_port
        
        # Check if driver is loaded
        if RPLidar is None:
            messagebox.showerror("Error", "RPLidar library not available. Please install rplidar.")
            return
        
        self.scanning = True
        self.scan_buffer.clear()  # Clear averaging buffer on new scan session
        self.start_button.config(state=tk.DISABLED)  # Disable start button to prevent double-clicks
        self.stop_button.config(state=tk.NORMAL)  # Enable stop button
        self.status_var.set("Starting scanner...")
        
        # Start scanning in a separate background thread
        # daemon=True ensures thread dies if main app closes
        self.scan_thread = threading.Thread(target=self.scan_loop, daemon=True)
        self.scan_thread.start()
    
    def stop_scan(self):
        """Stop scanning."""
        self.scanning = False  # Set flag to False to break the loop in scan_thread
        self.scan_buffer.clear()  # Clear averaging buffer
        self.status_var.set("Stopping scanner...")
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
    
    def scan_loop(self):
        """Main scanning loop running in a separate thread."""
        try:
            # Verify port exists before trying to connect
            if not self.device_port:
                raise ValueError("No device port specified")
            
            # Initialize LiDAR connection
            # Use root.after to update UI from background thread safely
            self.root.after(0, lambda: self.status_var.set(f"Connecting to {self.device_port}..."))
            self.lidar = RPLidar(self.device_port, baudrate=BAUD, timeout=TIMEOUT)
            
            # Clean state - reset hardware buffers
            try:
                self.lidar.stop()
            except Exception:
                pass
            try:
                self.lidar.stop_motor()
            except Exception:
                pass
            try:
                if hasattr(self.lidar, 'clear_input'):
                    self.lidar.clear_input()
                elif hasattr(self.lidar, 'clean_input'): # Check for alternative name
                    self.lidar.clean_input()
            except Exception:
                pass
            
            # Spin up motor to operating speed
            self.lidar.start_motor()
            time.sleep(SPINUP_S)  # Wait for motor to stabilize
            
            self.root.after(0, lambda: self.status_var.set("Scanner ready - collecting data..."))
            
            # Get scan iterator from driver
            # max_buf_meas prevents internal buffer overflow
            it = self.lidar.iter_scans(max_buf_meas=5000)
            
            # Discard warm-up scans (often contain garbage data)
            for _ in range(WARMUP_SCANS):
                try:
                    next(it)
                except Exception:
                    # If warm-up fails, reset and retry
                    self.lidar.stop()
                    try:
                        if hasattr(self.lidar, 'clear_input'):
                            self.lidar.clear_input()
                    except:
                        pass
                    time.sleep(0.2)
                    it = self.lidar.iter_scans(max_buf_meas=5000)
            
            # Main scanning loop
            while self.scanning:
                try:
                    # Fetch next scan packet
                    scan = next(it)
                    
                    # Basic processing: normalize angles to 0-360 range
                    # Format: (quality, angle, distance)
                    raw_filtered = [
                        (q, (ang % 360.0), dist)
                        for (q, ang, dist) in scan
                    ]
                    # Sort by angle for correct plotting order
                    raw_filtered.sort(key=lambda t: t[1])
                    
                    # Apply user-defined filters (Quality, Range, Z-Score)
                    filtered = self.filter_scan_data(raw_filtered)
                    
                    if filtered:
                        # Handle temporal averaging (frame averaging) if enabled
                        if self.scan_averaging:
                            self.scan_buffer.append(filtered)
                            # Maintain buffer size
                            if len(self.scan_buffer) > self.avg_count:
                                self.scan_buffer.pop(0)  # Remove oldest scan
                            
                            # Average the scans in buffer
                            if len(self.scan_buffer) >= 2:
                                filtered = self.average_scans(self.scan_buffer)
                        
                        # Update current scan data storage
                        self.current_scan_data = filtered
                        
                        # Calculate angular coverage (field of view)
                        angles = [ang for _, ang, _ in filtered]
                        coverage = angular_span_deg(angles)
                        
                        # Calculate quality statistics (using filtered data)
                        if filtered:
                            qualities = [q for q, _, _ in filtered]
                            min_q = min(qualities) if qualities else 0
                            max_q = max(qualities) if qualities else 0
                            avg_q = sum(qualities) / len(qualities) if qualities else 0
                            quality_stats = f"Quality: min={int(min_q)}, max={int(max_q)}, avg={int(avg_q)} (filtered, threshold={self.min_quality})"
                        else:
                            quality_stats = "Quality: N/A"
                        
                        # Update UI elements in main thread (thread-safe)
                        # Pass data as default args to lambda to capture current value
                        self.root.after(0, lambda f=filtered: self.update_plot(f))
                        self.root.after(0, lambda c=coverage: self.coverage_var.set(f"Coverage: {c:.1f}°"))
                        self.root.after(0, lambda q=quality_stats: self.quality_stats_var.set(q))
                    else:
                        # No points passed the filters - show warning
                        if raw_filtered:
                            # Show stats of rejected points to help user debug
                            qualities = [q for q, _, _ in raw_filtered]
                            min_q = min(qualities) if qualities else 0
                            max_q = max(qualities) if qualities else 0
                            avg_q = sum(qualities) / len(qualities) if qualities else 0
                            quality_stats = f"Quality: min={int(min_q)}, max={int(max_q)}, avg={int(avg_q)} (FILTERED OUT!)"
                            
                            self.root.after(0, lambda q=quality_stats: self.quality_stats_var.set(q))
                            self.root.after(0, lambda: self.status_var.set(
                                "No points pass filters - lower quality threshold!"
                            ))
                        
                except StopIteration:
                    break  # End of data stream
                except Exception as e:
                    if self.scanning:
                        error_msg = f"Scan error: {str(e)}"
                        self.root.after(0, lambda msg=error_msg: self.status_var.set(msg))
                        
                        # Print detailed error to console
                        print(f"Scan error: {e}")
                        import traceback
                        traceback.print_exc()
                        
                        # Error Recovery Logic:
                        # Only break loop for critical errors (disconnect)
                        if "disconnect" in str(e).lower() or "port" in str(e).lower():
                            break
                        
                        # For transient errors, wait briefly and retry
                        time.sleep(0.1)
                        continue
                    break
            
            # Cleanup sequence when loop ends
            if self.lidar:
                try:
                    self.lidar.stop()
                    self.lidar.stop_motor()
                    self.lidar.disconnect()
                except Exception:
                    pass
                self.lidar = None
            
            # Reset UI state
            if self.scanning:
                self.root.after(0, lambda: self.status_var.set("Scan stopped"))
                self.root.after(0, lambda: self.start_button.config(state=tk.NORMAL))
                self.root.after(0, lambda: self.stop_button.config(state=tk.DISABLED))
            
        except FileNotFoundError:
            # Handle specific port not found error
            self.root.after(0, lambda: self.status_var.set(f"Port {self.device_port} not found"))
            self.root.after(0, lambda: messagebox.showerror(
                "Port Not Found",
                f"Port '{self.device_port}' not found.\n\nPlease check the connection and port selection."
            ))
            self.root.after(0, lambda: self.start_button.config(state=tk.NORMAL))
            self.root.after(0, lambda: self.stop_button.config(state=tk.DISABLED))
            self.scanning = False
        except Exception as e:
            # Handle generic initialization errors
            self.root.after(0, lambda: self.status_var.set("Connection failed"))
            self.root.after(0, lambda: messagebox.showerror(
                "Scan Error",
                f"Failed to start scanning:\n{str(e)}\n\nPlease check device connection and port selection."
            ))
            self.root.after(0, lambda: self.start_button.config(state=tk.NORMAL))
            self.root.after(0, lambda: self.stop_button.config(state=tk.DISABLED))
            self.scanning = False
    
    def update_plot(self, scan_data: List[Tuple[float, float, float]]):
        """Update the real-time plot with new scan data."""
        if Figure is None or not scan_data:
            return
        
        try:
            # Clear previous plot content
            self.ax.clear()
            
            # Extract angles and distances for plotting
            # Convert degrees to radians for polar plot
            angles = [math.radians(ang) for _, ang, _ in scan_data]
            distances = [dist for _, _, dist in scan_data]
            
            # Plot points as scatter
            self.ax.scatter(angles, distances, s=5, color="tab:blue", alpha=0.6)
            
            # Set fixed axis limits using the user-specified radius
            # Don't auto-expand - use the radius from the GUI control
            self.ax.set_ylim(0, self.max_radius)
            
            # Update plot labels and grid
            self.ax.set_title(f"LiDAR Scan - Real-time (Points: {len(scan_data)})")
            self.ax.set_rlabel_position(135)
            self.ax.set_ylabel("Radius (mm)")
            self.ax.grid(True, linestyle=":", linewidth=0.5)
            
            # Update scan counter
            self.scan_idx += 1
            self.scan_count_var.set(f"Scans: {self.scan_idx}")
            
            # Store scan in comprehensive history
            self.all_scans.append(scan_data.copy())
            
            # Refresh canvas widget to show changes
            self.canvas.draw()
            self.status_var.set("Scanning...")
            
        except Exception:
            self.status_var.set("Plot update error")
    
    def export_csv(self):
        """Export all collected scans to CSV file."""
        if not self.all_scans:
            messagebox.showwarning("No Data", "No scan data to export.")
            return
        
        # Open save dialog
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            title="Export Scan Data to CSV"
        )
        
        if not filename:
            return
        
        try:
            with open(filename, "w", newline="") as f:
                writer = csv.writer(f)
                # Write header row
                writer.writerow([
                    "timestamp", "scan_idx", "meas_idx",
                    "angle_deg", "distance_mm", "quality", "x_mm", "y_mm"
                ])
                
                # Write all data points
                for scan_idx, scan in enumerate(self.all_scans):
                    ts = time_ms()
                    for meas_idx, (q, ang, dist) in enumerate(scan):
                        # Convert polar to cartesian for convenience
                        x, y = xy(ang, dist)
                        writer.writerow([
                            ts, scan_idx, meas_idx,
                            f"{ang:.3f}", f"{dist:.1f}", int(q), f"{x:.1f}", f"{y:.1f}"
                        ])
            
            messagebox.showinfo("Success", f"Exported {len(self.all_scans)} scan(s) to {filename}")
            self.status_var.set(f"Exported to {Path(filename).name}")
            
        except Exception as e:
            messagebox.showerror("Export Error", f"Failed to export CSV: {str(e)}")
    
    def export_svg(self):
        """Export current plot to SVG file."""
        if Figure is None:
            messagebox.showerror("Error", "Matplotlib not available for SVG export.")
            return
        
        if not self.current_scan_data and not self.all_scans:
            messagebox.showwarning("No Data", "No scan data to export.")
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".svg",
            filetypes=[("SVG files", "*.svg"), ("All files", "*.*")],
            title="Export Scan Plot to SVG"
        )
        
        if not filename:
            return
        
        try:
            # Create a new separate figure for export (don't use the GUI one)
            fig_export = Figure(figsize=(8, 8), dpi=100)
            ax_export = fig_export.add_subplot(111, projection='polar')
            
            # Use all scans if available for a complete picture, otherwise current scan
            if self.all_scans:
                all_angles = []
                all_distances = []
                for scan in self.all_scans:
                    for _, ang, dist in scan:
                        all_angles.append(math.radians(ang))
                        all_distances.append(dist)
            else:
                all_angles = [math.radians(ang) for _, ang, _ in self.current_scan_data]
                all_distances = [dist for _, _, dist in self.current_scan_data]
            
            # Recreate plot on export figure
            ax_export.scatter(all_angles, all_distances, s=5, color="tab:blue", alpha=0.6)
            ax_export.set_title(f"LiDAR Scan Export ({len(self.all_scans) if self.all_scans else 1} scan(s))")
            ax_export.set_rlabel_position(135)
            ax_export.set_ylabel("Radius (mm)")
            ax_export.grid(True, linestyle=":", linewidth=0.5)
            
            # Save layout
            fig_export.tight_layout()
            fig_export.savefig(filename, format='svg')
            
            messagebox.showinfo("Success", f"Exported plot to {filename}")
            self.status_var.set(f"Exported SVG to {Path(filename).name}")
            
        except Exception as e:
            messagebox.showerror("Export Error", f"Failed to export SVG: {str(e)}")
    
    def on_closing(self):
        """Handle window closing event."""
        if self.scanning:
            self.stop_scan()
            # Wait a bit for thread to finish to clean up resources
            if self.scan_thread:
                self.scan_thread.join(timeout=2.0)
        
        # Ensure hardware is properly disconnected
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except Exception:
                pass
        
        self.root.destroy()


def main():
    """Entry point for the GUI application."""
    root = Tk()
    app = ScannerGUI(root)
    # properly handle window close button (X)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()

