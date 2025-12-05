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
    # Get list_ports - try to import directly, fallback to DataCapture's
    try:
        from serial.tools import list_ports
    except ImportError:
        list_ports = getattr(dc, 'list_ports', None)
except (ImportError, AttributeError):
    # Fallback if import fails
    BAUD = 115200
    TIMEOUT = 1
    MIN_QUALITY = 0
    SPINUP_S = 1.5
    WARMUP_SCANS = 3
    MIN_COVERAGE_DEG = 320.0
    MAX_ATTEMPTS = 20
    def xy(angle_deg, dist_mm):
        th = math.radians(angle_deg % 360.0)
        return dist_mm * math.cos(th), dist_mm * math.sin(th)
    def time_ms():
        t = time.time()
        return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(t)) + f".{int((t % 1) * 1e3):03d}Z"
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
        
        # Scanner state
        self.lidar: Optional[RPLidar] = None
        self.scanning = False
        self.scan_thread: Optional[threading.Thread] = None
        self.current_scan_data: List[Tuple[float, float, float]] = []  # (angle, distance, quality)
        self.all_scans: List[List[Tuple[float, float, float]]] = []  # Store all scans
        self.scan_idx = 0
        self.device_port: Optional[str] = None
        self.scan_buffer: List[List[Tuple[float, float, float]]] = []  # For averaging
        self.min_quality = 10  # Default quality threshold
        self.min_distance = 100  # Default min distance (mm)
        self.max_distance = 8000  # Default max distance (mm)
        self.remove_outliers = True  # Default: enable outlier removal
        self.scan_averaging = False  # Default: disable averaging
        self.avg_count = 3  # Default averaging count
        
        # Status variables
        self.status_var = StringVar(value="Ready")
        self.scan_count_var = StringVar(value="Scans: 0")
        self.coverage_var = StringVar(value="Coverage: 0.0°")
        self.quality_stats_var = StringVar(value="Quality: N/A")
        
        # Setup GUI
        self.setup_gui()
        
        # Initialize port list and auto-detect device on startup
        self.refresh_ports()
        self.auto_detect_device()
        
        # Update device_port when combobox changes
        self.device_var.trace('w', self.on_port_selected)
    
    def setup_gui(self):
        """Create and layout GUI components."""
        # Main container
        main_frame = Frame(self.root, padx=10, pady=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Control panel (left side)
        control_frame = Frame(main_frame)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        
        # Device selection
        device_frame = Frame(control_frame)
        device_frame.pack(fill=tk.X, pady=(0, 10))
        
        Label(device_frame, text="Device Port:", font=("Arial", 10, "bold")).pack(anchor=tk.W)
        
        # Port selection combobox (allow manual entry if ports not detected)
        self.device_var = StringVar()
        self.port_combo = ttk.Combobox(device_frame, textvariable=self.device_var, width=20)
        self.port_combo.pack(fill=tk.X, pady=(5, 5))
        
        # Help text
        help_label = Label(
            device_frame,
            text="If no ports detected, enter COM port manually",
            font=("Arial", 8),
            fg="gray",
            justify=tk.LEFT
        )
        help_label.pack(anchor=tk.W, pady=(0, 5))
        
        # Buttons for device management
        device_button_frame = Frame(device_frame)
        device_button_frame.pack(fill=tk.X)
        
        Button(device_button_frame, text="Refresh Ports", command=self.refresh_ports, padx=5, pady=2).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2))
        Button(device_button_frame, text="Auto-detect", command=self.auto_detect_device, padx=5, pady=2).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(2, 0))
        
        # Test connection button
        Button(device_frame, text="Test Connection", command=self.test_connection, padx=5, pady=2, bg="#2196F3", fg="white").pack(fill=tk.X, pady=(5, 0))
        
        # Scan radius control
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
        
        # Bind to update when value is changed manually
        self.radius_var.trace('w', lambda *args: self.update_radius())
        
        # Quality threshold control
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
        
        # Update min_quality when spinbox changes
        self.quality_var.trace('w', lambda *args: self.update_quality())
        
        # Control buttons
        button_frame = Frame(control_frame)
        button_frame.pack(fill=tk.X, pady=(10, 0))
        
        self.start_button = Button(
            button_frame, text="Start Scan", command=self.start_scan,
            bg="#4CAF50", fg="white", font=("Arial", 11, "bold"),
            padx=20, pady=10
        )
        self.start_button.pack(fill=tk.X, pady=(0, 5))
        
        self.stop_button = Button(
            button_frame, text="Stop Scan", command=self.stop_scan,
            bg="#f44336", fg="white", font=("Arial", 11, "bold"),
            padx=20, pady=10, state=tk.DISABLED
        )
        self.stop_button.pack(fill=tk.X, pady=(0, 10))
        
        # Export buttons
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
        
        # Status display
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
        
        # Plot area (right side)
        plot_frame = Frame(main_frame)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        if Figure is not None:
            self.fig = Figure(figsize=(8, 8), dpi=100)
            self.ax = self.fig.add_subplot(111, projection='polar')
            self.ax.set_title("LiDAR Scan - Real-time")
            self.ax.grid(True, linestyle=":", linewidth=0.5)
            # Set initial radius from GUI control (defaults to 8000mm)
            try:
                self.max_radius = float(self.radius_var.get())
            except (ValueError, AttributeError):
                self.max_radius = 8000  # Fallback default
            self.ax.set_ylim(0, self.max_radius)
            self.scatter = None
            
            self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
            self.canvas.draw()
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            
            # Add toolbar
            toolbar = NavigationToolbar2Tk(self.canvas, plot_frame)
            toolbar.update()
        else:
            Label(plot_frame, text="Matplotlib not available", fg="red").pack()
    
    def get_available_ports(self) -> List[str]:
        """Get list of available serial ports."""
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
        """Refresh the list of available ports."""
        ports = self.get_available_ports()
        self.port_combo['values'] = ports
        
        # Allow manual entry if no ports detected
        if ports:
            self.port_combo.config(state="readonly")
            # If current selection is still valid, keep it; otherwise select first
            current = self.device_var.get()
            if current in ports:
                self.device_var.set(current)
            else:
                self.device_var.set(ports[0])
                self.device_port = ports[0]
            self.status_var.set(f"Found {len(ports)} port(s): {', '.join(ports)}")
        else:
            # Allow manual entry when no ports detected
            self.port_combo.config(state="normal")
            current = self.device_var.get()
            # Keep current value if user has typed something
            if not current:
                self.device_var.set("")
                self.device_port = None
            self.status_var.set("No ports detected - Enter COM port manually")
    
    def auto_detect_device(self):
        """Auto-detect and set the LiDAR device port."""
        try:
            # First refresh ports to get current list
            self.refresh_ports()
            
            # Get available ports
            available_ports = self.get_available_ports()
            
            if not available_ports:
                self.status_var.set("No ports available")
                return
            
            # Use detect_port from DataCapture.py
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
        """Handle port selection change."""
        selected = self.device_var.get()
        if selected:
            self.device_port = selected
    
    def update_radius(self):
        """Update the maximum scan radius for the plot."""
        try:
            new_radius = float(self.radius_var.get())
            if new_radius > 0:
                self.max_radius = new_radius
                # Update plot if it exists
                if hasattr(self, 'ax') and self.ax is not None:
                    self.ax.set_ylim(0, self.max_radius)
                    self.canvas.draw()
        except (ValueError, AttributeError):
            # Invalid value, ignore
            pass
    
    def update_quality(self):
        """Update the minimum quality threshold."""
        try:
            new_quality = int(self.quality_var.get())
            if 0 <= new_quality <= 63:
                self.min_quality = new_quality
        except (ValueError, AttributeError):
            # Invalid value, ignore
            pass
    
    def filter_scan_data(self, scan_data: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
        """
        Apply various filters to improve scan accuracy.
        Returns filtered scan data.
        """
        if not scan_data:
            return []
        
        try:
            filtered = scan_data.copy()
            
            # 1. Quality threshold filter
            original_count = len(filtered)
            # Apply quality filter - only keep points with quality >= min_quality
            filtered = [(q, ang, dist) for q, ang, dist in filtered if q >= self.min_quality]
            
            # Debug: print filter stats (can be removed later)
            if original_count > 0:
                removed = original_count - len(filtered)
                if removed > 0:
                    print(f"Quality filter: removed {removed}/{original_count} points (threshold={self.min_quality})")
            
            # Warn if filtering removed all points
            if original_count > 0 and len(filtered) == 0:
                # Update status to warn user
                self.root.after(0, lambda: self.status_var.set(
                    f"Warning: Quality filter (min={self.min_quality}) removed all points!"
                ))
            
            # 2. Distance range filter
            filtered = [(q, ang, dist) for q, ang, dist in filtered if self.min_distance <= dist <= self.max_distance]
            
            # 3. Outlier detection using statistical filtering
            if self.remove_outliers and len(filtered) > 3:
                filtered = self.remove_outliers_statistical(filtered)
            
            return filtered
        except Exception as e:
            # If filtering fails, return original data to prevent scan from stopping
            print(f"Filter error: {e}")
            import traceback
            traceback.print_exc()
            return scan_data
    
    def remove_outliers_statistical(self, scan_data: List[Tuple[float, float, float]], 
                       z_threshold: float = 2.5) -> List[Tuple[float, float, float]]:
        """
        Remove outliers using Z-score method on distances.
        Points with distances that deviate significantly from neighbors are removed.
        """
        if len(scan_data) < 3:
            return scan_data
        
        # Sort by angle for neighbor analysis
        sorted_data = sorted(scan_data, key=lambda t: t[1])
        
        # Calculate local distance statistics for each point
        filtered = []
        window_size = min(5, len(sorted_data) // 4)  # Adaptive window size
        
        for i in range(len(sorted_data)):
            # Get neighbors within window
            start_idx = max(0, i - window_size)
            end_idx = min(len(sorted_data), i + window_size + 1)
            neighbors = sorted_data[start_idx:end_idx]
            
            if len(neighbors) < 2:
                filtered.append(sorted_data[i])
                continue
            
            # Calculate mean and std of neighbor distances
            neighbor_dists = [dist for _, _, dist in neighbors]
            mean_dist = sum(neighbor_dists) / len(neighbor_dists)
            variance = sum((d - mean_dist) ** 2 for d in neighbor_dists) / len(neighbor_dists)
            std_dist = math.sqrt(variance) if variance > 0 else 1.0
            
            # Check if current point is within acceptable range
            current_dist = sorted_data[i][2]
            if std_dist > 0:
                z_score = abs(current_dist - mean_dist) / std_dist
                if z_score <= z_threshold:
                    filtered.append(sorted_data[i])
            else:
                filtered.append(sorted_data[i])
        
        return filtered
    
    def average_scans(self, scans: List[List[Tuple[float, float, float]]]) -> List[Tuple[float, float, float]]:
        """
        Average multiple scans by angle binning.
        This reduces noise and improves accuracy.
        """
        if not scans:
            return []
        
        # Create angle bins (1 degree resolution)
        angle_bins = {}
        for scan in scans:
            for q, ang, dist in scan:
                bin_angle = round(ang)
                if bin_angle not in angle_bins:
                    angle_bins[bin_angle] = []
                angle_bins[bin_angle].append((q, dist))
        
        # Average points in each bin
        averaged = []
        for angle, points in angle_bins.items():
            if points:
                # Use median for distance (more robust than mean)
                distances = [dist for _, dist in points]
                qualities = [q for q, _ in points]
                distances.sort()
                qualities.sort()
                
                median_dist = distances[len(distances) // 2]
                median_quality = qualities[len(qualities) // 2]
                
                averaged.append((median_quality, float(angle), median_dist))
        
        return sorted(averaged, key=lambda t: t[1])
    
    def test_connection(self):
        """Test if the selected port can be opened."""
        port = self.device_var.get().strip()
        if not port:
            messagebox.showwarning("No Port", "Please enter a COM port (e.g., COM3)")
            return
        
        # Normalize port name (add COM if just number)
        if port.upper().startswith("COM"):
            test_port = port.upper()
        elif port.isdigit():
            test_port = f"COM{port}"
        else:
            test_port = port.upper()
        
        self.status_var.set(f"Testing connection to {test_port}...")
        
        # Try to open the port
        try:
            try:
                import serial
            except ImportError:
                messagebox.showerror("Error", "pyserial not installed. Please install it with: pip install pyserial")
                return
            
            ser = serial.Serial(test_port, baudrate=BAUD, timeout=0.5)
            ser.close()
            messagebox.showinfo("Success", f"Port {test_port} is available")
            self.device_var.set(test_port)
            self.device_port = test_port
            self.status_var.set(f"Port {test_port} ready")
        except serial.SerialException:
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
            messagebox.showerror("Error", "Could not test port connection")
            self.status_var.set("Test failed")
    
    def start_scan(self):
        """Start scanning in a separate thread."""
        if self.scanning:
            return
        
        # Get selected port from combobox
        selected_port = self.device_var.get()
        if not selected_port:
            messagebox.showerror("Error", "No device port selected. Please select a port from the dropdown.")
            return
        
        # Verify port is still available
        available_ports = self.get_available_ports()
        if selected_port not in available_ports:
            messagebox.showerror(
                "Port Not Available",
                f"Port '{selected_port}' is no longer available.\n\nPlease:\n1. Click 'Refresh Ports' to update the list\n2. Select an available port"
            )
            self.refresh_ports()
            return
        
        self.device_port = selected_port
        
        if RPLidar is None:
            messagebox.showerror("Error", "RPLidar library not available. Please install rplidar.")
            return
        
        self.scanning = True
        self.scan_buffer.clear()  # Clear averaging buffer on new scan session
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.status_var.set("Starting scanner...")
        
        # Start scanning in a separate thread
        self.scan_thread = threading.Thread(target=self.scan_loop, daemon=True)
        self.scan_thread.start()
    
    def stop_scan(self):
        """Stop scanning."""
        self.scanning = False
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
            
            # Initialize LiDAR
            self.root.after(0, lambda: self.status_var.set(f"Connecting to {self.device_port}..."))
            self.lidar = RPLidar(self.device_port, baudrate=BAUD, timeout=TIMEOUT)
            
            # Clean state
            for fn in (self.lidar.stop, self.lidar.clear_input, self.lidar.stop_motor):
                try:
                    fn()
                except Exception:
                    pass
            
            # Spin up motor
            self.lidar.start_motor()
            time.sleep(SPINUP_S)
            
            self.root.after(0, lambda: self.status_var.set("Scanner ready - collecting data..."))
            
            # Get scan iterator
            it = self.lidar.iter_scans(max_buf_meas=5000)
            
            # Discard warm-up scans
            for _ in range(WARMUP_SCANS):
                try:
                    next(it)
                except Exception:
                    self.lidar.stop()
                    self.lidar.clear_input()
                    time.sleep(0.2)
                    it = self.lidar.iter_scans(max_buf_meas=5000)
            
            # Main scanning loop
            while self.scanning:
                try:
                    scan = next(it)
                    # Basic filtering: normalize angles
                    raw_filtered = [
                        (q, (ang % 360.0), dist)
                        for (q, ang, dist) in scan
                    ]
                    raw_filtered.sort(key=lambda t: t[1])
                    
                    # Apply user-defined filters
                    filtered = self.filter_scan_data(raw_filtered)
                    
                    if filtered:
                        # Handle scan averaging if enabled
                        if self.scan_averaging:
                            self.scan_buffer.append(filtered)
                            if len(self.scan_buffer) > self.avg_count:
                                self.scan_buffer.pop(0)  # Keep only last N scans
                            
                            # Average the scans in buffer
                            if len(self.scan_buffer) >= 2:
                                filtered = self.average_scans(self.scan_buffer)
                        
                        # Update current scan data
                        self.current_scan_data = filtered
                        
                        # Calculate coverage
                        angles = [ang for _, ang, _ in filtered]
                        coverage = angular_span_deg(angles)
                        
                        # Calculate quality statistics (show filtered data, not raw)
                        if filtered:
                            qualities = [q for q, _, _ in filtered]
                            min_q = min(qualities) if qualities else 0
                            max_q = max(qualities) if qualities else 0
                            avg_q = sum(qualities) / len(qualities) if qualities else 0
                            quality_stats = f"Quality: min={int(min_q)}, max={int(max_q)}, avg={int(avg_q)} (filtered, threshold={self.min_quality})"
                        else:
                            quality_stats = "Quality: N/A"
                        
                        # Update UI in main thread
                        self.root.after(0, lambda f=filtered: self.update_plot(f))
                        self.root.after(0, lambda c=coverage: self.coverage_var.set(f"Coverage: {c:.1f}°"))
                        self.root.after(0, lambda q=quality_stats: self.quality_stats_var.set(q))
                    else:
                        # No points after filtering - show warning
                        if raw_filtered:
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
                    break
                except Exception as e:
                    if self.scanning:
                        error_msg = f"Scan error: {str(e)}"
                        self.root.after(0, lambda msg=error_msg: self.status_var.set(msg))
                        # Print to console for debugging
                        print(f"Scan error: {e}")
                        import traceback
                        traceback.print_exc()
                        # Try to continue scanning instead of breaking immediately
                        # Only break if it's a critical error
                        if "disconnect" in str(e).lower() or "port" in str(e).lower():
                            break
                        # Otherwise, continue to next iteration
                        time.sleep(0.1)
                        continue
                    break
            
            # Cleanup
            if self.lidar:
                try:
                    self.lidar.stop()
                    self.lidar.stop_motor()
                    self.lidar.disconnect()
                except Exception:
                    pass
                self.lidar = None
            
            if self.scanning:
                self.root.after(0, lambda: self.status_var.set("Scan stopped"))
                self.root.after(0, lambda: self.start_button.config(state=tk.NORMAL))
                self.root.after(0, lambda: self.stop_button.config(state=tk.DISABLED))
            
        except FileNotFoundError:
            self.root.after(0, lambda: self.status_var.set(f"Port {self.device_port} not found"))
            self.root.after(0, lambda: messagebox.showerror(
                "Port Not Found",
                f"Port '{self.device_port}' not found.\n\nPlease check the connection and port selection."
            ))
            self.root.after(0, lambda: self.start_button.config(state=tk.NORMAL))
            self.root.after(0, lambda: self.stop_button.config(state=tk.DISABLED))
            self.scanning = False
        except Exception as e:
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
            # Clear previous plot
            self.ax.clear()
            
            # Extract angles and distances
            angles = [math.radians(ang) for _, ang, _ in scan_data]
            distances = [dist for _, _, dist in scan_data]
            
            # Plot points
            self.ax.scatter(angles, distances, s=5, color="tab:blue", alpha=0.6)
            
            # Set fixed axis limits using the user-specified radius
            # Don't auto-expand - use the radius from the GUI control
            self.ax.set_ylim(0, self.max_radius)
            
            # Update plot settings
            self.ax.set_title(f"LiDAR Scan - Real-time (Points: {len(scan_data)})")
            self.ax.set_rlabel_position(135)
            self.ax.set_ylabel("Radius (mm)")
            self.ax.grid(True, linestyle=":", linewidth=0.5)
            
            # Update scan count
            self.scan_idx += 1
            self.scan_count_var.set(f"Scans: {self.scan_idx}")
            
            # Store scan
            self.all_scans.append(scan_data.copy())
            
            # Refresh canvas
            self.canvas.draw()
            self.status_var.set("Scanning...")
            
        except Exception:
            self.status_var.set("Plot update error")
    
    def export_csv(self):
        """Export all collected scans to CSV file."""
        if not self.all_scans:
            messagebox.showwarning("No Data", "No scan data to export.")
            return
        
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
                writer.writerow([
                    "timestamp", "scan_idx", "meas_idx",
                    "angle_deg", "distance_mm", "quality", "x_mm", "y_mm"
                ])
                
                for scan_idx, scan in enumerate(self.all_scans):
                    ts = time_ms()
                    for meas_idx, (q, ang, dist) in enumerate(scan):
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
            # Create a new figure for export
            fig_export = Figure(figsize=(8, 8), dpi=100)
            ax_export = fig_export.add_subplot(111, projection='polar')
            
            # Use all scans if available, otherwise current scan
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
            
            ax_export.scatter(all_angles, all_distances, s=5, color="tab:blue", alpha=0.6)
            ax_export.set_title(f"LiDAR Scan Export ({len(self.all_scans) if self.all_scans else 1} scan(s))")
            ax_export.set_rlabel_position(135)
            ax_export.set_ylabel("Radius (mm)")
            ax_export.grid(True, linestyle=":", linewidth=0.5)
            
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
            # Wait a bit for thread to finish
            if self.scan_thread:
                self.scan_thread.join(timeout=2.0)
        
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
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()

