"""
Lightweight Fast-API Web Server for Real-time LiDAR Scanning.
Runs on the Raspberry Pi and securely serves the web dashboard over the local network.
"""
import sys
import os
import time
import math
import json
import asyncio
import threading
from pathlib import Path
from typing import List, Tuple, Set

try:
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.staticfiles import StaticFiles
    import uvicorn
except ImportError:
    print("Missing dependencies. Please install with: pip install fastapi uvicorn websockets")
    sys.exit(1)

# Import shared constants from DataCapture (simulating setup from ScannerGUI)
scripts_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, scripts_dir)

try:
    import DataCapture as dc
    from DataCapture import (
        BAUD, TIMEOUT, SPINUP_S, WARMUP_SCANS,
        angular_span_deg, list_ports
    )
except (ImportError, AttributeError):
    BAUD = 115200
    TIMEOUT = 1
    SPINUP_S = 1.5
    WARMUP_SCANS = 3
    def angular_span_deg(angles): return 0.0
    list_ports = None

try:
    from rplidar import RPLidar
except ImportError:
    RPLidar = None

# Initialize FastAPI App
app = FastAPI(title="LiDAR Web Dashboard")

# Thread-safe State Manager
class LidarStateManager:
    def __init__(self):
        self.lidar = None
        self.scanning = False
        self.scan_thread = None
        self.device_port = None
        
        # UI Config
        self.min_quality = 10
        self.max_radius = 8000
        self.min_distance = 100
        self.remove_outliers = True
        
        # Telemetry State
        self.latest_msg = {
            "type": "scan_data",
            "points": [],
            "coverage": 0.0,
            "quality_str": "N/A"
        }
        self.status_text = "Ready"
        
        # Connected WebSocket Clients
        self.active_connections: Set[WebSocket] = set()

    def get_available_ports(self):
        if not list_ports: return ["/dev/ttyUSB0", "COM4"]
        try:
            return sorted([p.device for p in list_ports.comports()])
        except Exception:
            return ["/dev/ttyUSB0"]

manager = LidarStateManager()

# Broadcast background task
async def telemetry_broadcaster():
    """Continuously broadcast the latest scan frame to all connected browsers at 15Hz"""
    while True:
        if manager.scanning and manager.active_connections:
            # Create a string payload to avoid JSON serializing for every single client
            payload = json.dumps(manager.latest_msg)
            dead_connections = set()
            
            for connection in list(manager.active_connections):
                try:
                    await connection.send_text(payload)
                except Exception:
                    dead_connections.add(connection)
            
            # Cleanup dead connections
            manager.active_connections -= dead_connections
            
        await asyncio.sleep(0.06)  # ~16 FPS

# Async app startup
main_loop = None

@app.on_event("startup")
async def startup_event():
    global main_loop
    main_loop = asyncio.get_running_loop()
    asyncio.create_task(telemetry_broadcaster())

# Optional: Z-score filter implementation from GUI
def remove_outliers_statistical(scan_data, z_threshold=2.5):
    if len(scan_data) < 3: return scan_data
    sorted_data = sorted(scan_data, key=lambda t: t[1])
    filtered = []
    window_size = min(5, len(sorted_data) // 4)
    
    for i in range(len(sorted_data)):
        start_idx = max(0, i - window_size)
        end_idx = min(len(sorted_data), i + window_size + 1)
        neighbors = sorted_data[start_idx:end_idx]
        if len(neighbors) < 2:
            filtered.append(sorted_data[i])
            continue
            
        neighbor_dists = [dist for _, _, dist in neighbors]
        mean_dist = sum(neighbor_dists) / len(neighbor_dists)
        variance = sum((d - mean_dist) ** 2 for d in neighbor_dists) / len(neighbor_dists)
        std_dist = math.sqrt(variance) if variance > 0 else 1.0
        
        current_dist = sorted_data[i][2]
        if std_dist > 0 and abs(current_dist - mean_dist) / std_dist <= z_threshold:
            filtered.append(sorted_data[i])
            
    return filtered

def filter_scan_data(scan_data):
    if not scan_data: return []
    filtered = [(q, ang, dist) for q, ang, dist in scan_data if q >= manager.min_quality]
    filtered = [(q, ang, dist) for q, ang, dist in filtered if manager.min_distance <= dist <= manager.max_radius]
    
    if manager.remove_outliers and len(filtered) > 3:
        filtered = remove_outliers_statistical(filtered)
    return filtered

def set_status(msg):
    manager.status_text = msg
    print(f"[LiDAR] {msg}")
    broadcast_status()

def broadcast_status():
    if not main_loop: return
    payload = json.dumps({
        "type": "status",
        "is_scanning": manager.scanning,
        "status_text": manager.status_text
    })
    # Cannot await inside thread, so we fire and forget in main loop
    for connection in list(manager.active_connections):
        asyncio.run_coroutine_threadsafe(connection.send_text(payload), main_loop)

# Blocking scan thread
def hardware_scan_loop():
    try:
        set_status(f"Connecting to {manager.device_port}...")
        manager.lidar = RPLidar(manager.device_port, baudrate=BAUD, timeout=TIMEOUT)
        
        try: manager.lidar.stop()
        except: pass
        try: manager.lidar.stop_motor()
        except: pass
        try: manager.lidar.clean_input()
        except: pass
        
        manager.lidar.start_motor()
        time.sleep(SPINUP_S)
        set_status("Scanner ready - collecting data...")
        
        it = manager.lidar.iter_scans(max_buf_meas=5000)
        
        for _ in range(WARMUP_SCANS):
            try: next(it)
            except: time.sleep(0.1)
            
        while manager.scanning:
            try:
                scan = next(it)
                raw_filtered = [(q, (ang % 360.0), dist) for (q, ang, dist) in scan]
                raw_filtered.sort(key=lambda t: t[1])
                
                filtered = filter_scan_data(raw_filtered)
                
                if filtered:
                    angles = [ang for _, ang, _ in filtered]
                    qualities = [q for q, _, _ in filtered]
                    
                    manager.latest_msg = {
                        "type": "scan_data",
                        "points": filtered, # Only sending essential float tuples
                        "coverage": angular_span_deg(angles),
                        "quality_str": f"Avg={int(sum(qualities)/len(qualities))} (min={int(min(qualities))})"
                    }
                
            except StopIteration:
                break
            except Exception as e:
                set_status(f"Transient Error: {e}")
                time.sleep(0.1)
                
    except Exception as e:
        set_status(f"Fatal Error: {e}")
        manager.scanning = False
        broadcast_status()
        
    finally:
        manager.scanning = False
        if manager.lidar:
            try: manager.lidar.stop()
            except: pass
            try: manager.lidar.stop_motor()
            except: pass
            try: manager.lidar.disconnect()
            except: pass
            manager.lidar = None
            
        set_status("Scan stopped.")

# WebSockets Endpoint
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    manager.active_connections.add(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            cmd = json.loads(data)
            action = cmd.get("action")
            
            if action == "get_ports":
                await websocket.send_json({"type": "ports", "data": manager.get_available_ports()})
                
            elif action == "get_status":
                await websocket.send_json({"type": "status", "is_scanning": manager.scanning, "status_text": manager.status_text})
                
            elif action == "update_config":
                cfg = cmd.get("config", {})
                if "min_quality" in cfg: manager.min_quality = cfg["min_quality"]
                if "max_radius" in cfg: manager.max_radius = cfg["max_radius"]
                
            elif action == "start_scan":
                if not manager.scanning:
                    manager.device_port = cmd.get("port", manager.get_available_ports()[0])
                    cfg = cmd.get("config", {})
                    if "min_quality" in cfg: manager.min_quality = cfg["min_quality"]
                    if "max_radius" in cfg: manager.max_radius = cfg["max_radius"]
                    
                    manager.scanning = True
                    broadcast_status()
                    manager.scan_thread = threading.Thread(target=hardware_scan_loop, daemon=True)
                    manager.scan_thread.start()
                    
            elif action == "stop_scan":
                manager.scanning = False
                set_status("Stopping scanner...")
                
            elif action == "export_csv":
                csv_lines = ["Quality,Angle(deg),Distance(mm)"]
                for p in manager.latest_msg["points"]:
                    csv_lines.append(f"{p[0]},{p[1]},{p[2]}")
                csv_string = "\n".join(csv_lines)
                await websocket.send_json({"type": "export_ready", "csv_data": csv_string})
                
    except WebSocketDisconnect:
        manager.active_connections.remove(websocket)

# Mount Static Files (Frontend) AT THE END
web_dir = os.path.join(scripts_dir, "web")
app.mount("/", StaticFiles(directory=web_dir, html=True), name="web")

if __name__ == "__main__":
    print("="*50)
    print("LiDAR Web Dashboard Starting")
    print("To view, open a browser on any device in your network to: http://192.168.50.149:5000")
    print("="*50)
    
    # Run the uvicorn ASGI server
    uvicorn.run(app, host="0.0.0.0", port=5000, log_level="error")
