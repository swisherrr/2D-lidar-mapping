"""Real-time LiDAR Scanner Control GUI

A GUI application for controlling the LiDAR scanner in real-time with features:
- Start/Stop scanning
- Real-time visualization
- Export scan to CSV
- Export scan plot to SVG
"""

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
        
        # Status variables
        self.status_var = StringVar(value="Ready")
        self.scan_count_var = StringVar(value="Scans: 0")
        self.coverage_var = StringVar(value="Coverage: 0.0°")
        
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
        Label(status_frame, textvariable=self.coverage_var, anchor=tk.W).pack(fill=tk.X)
        
        # Plot area (right side)
        plot_frame = Frame(main_frame)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        if Figure is not None:
            self.fig = Figure(figsize=(8, 8), dpi=100)
            self.ax = self.fig.add_subplot(111, projection='polar')
            self.ax.set_title("LiDAR Scan - Real-time")
            self.ax.grid(True, linestyle=":", linewidth=0.5)
            # Set a reasonable default radial limit (in mm) - can be adjusted
            self.max_radius = 8000  # 8 meters default max range
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
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.status_var.set("Starting scanner...")
        
        # Start scanning in a separate thread
        self.scan_thread = threading.Thread(target=self.scan_loop, daemon=True)
        self.scan_thread.start()
    
    def stop_scan(self):
        """Stop scanning."""
        self.scanning = False
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
                    # Filter and process scan
                    filtered = [
                        (q, (ang % 360.0), dist)
                        for (q, ang, dist) in scan
                        if q >= MIN_QUALITY
                    ]
                    filtered.sort(key=lambda t: t[1])
                    
                    if filtered:
                        # Update current scan data
                        self.current_scan_data = filtered
                        
                        # Calculate coverage
                        angles = [ang for _, ang, _ in filtered]
                        coverage = angular_span_deg(angles)
                        
                        # Update UI in main thread
                        self.root.after(0, lambda: self.update_plot(filtered))
                        self.root.after(0, lambda: self.coverage_var.set(f"Coverage: {coverage:.1f}°"))
                        
                except StopIteration:
                    break
                except Exception:
                    if self.scanning:
                        self.root.after(0, lambda: self.status_var.set("Scan error occurred"))
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
            self.ax.scatter(angles, distances, s=10, color="tab:blue", alpha=0.6)
            
            # Set fixed axis limits to prevent auto-zooming
            # Use the maximum distance from current scan or keep existing max
            if distances:
                current_max = max(distances)
                # Update max_radius if current scan exceeds it, but don't shrink it
                if current_max > self.max_radius:
                    self.max_radius = current_max * 1.1  # Add 10% padding
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
            
            ax_export.scatter(all_angles, all_distances, s=10, color="tab:blue", alpha=0.6)
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

