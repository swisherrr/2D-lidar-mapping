"""
Lightweight Fast-API Web Server for Real-time 3D point cloud LiDAR Scanning.
Runs on the Raspberry Pi and sweeps adafruit_servokit while broadcasting tracking data.
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
    print("Missing web dependencies. pip install fastapi uvicorn websockets")
    sys.exit(1)

scripts_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, scripts_dir)

try:
    import DataCapture as dc
    from DataCapture import (BAUD, TIMEOUT, SPINUP_S, WARMUP_SCANS, list_ports)
except (ImportError, AttributeError):
    BAUD = 115200; TIMEOUT = 1; SPINUP_S = 1.5; WARMUP_SCANS = 3
    list_ports = None

try:
    from rplidar import RPLidar
except ImportError:
    RPLidar = None

# Initialize FastAPI App
app = FastAPI(title="3D LiDAR Backend")
main_loop = None

@app.on_event("startup")
async def startup_event():
    global main_loop
    main_loop = asyncio.get_running_loop()
    asyncio.create_task(telemetry_broadcaster())

# Thread-safe State Manager
class LidarStateManager:
    def __init__(self):
        self.lidar = None
        self.scanning = False
        self.scan_thread = None
        self.servo_thread = None
        self.device_port = None
        
        # UI Config & Constraints
        self.min_quality = 10
        self.max_radius = 8000
        self.min_distance = 100
        
        # 3D Tilt Parameters
        self.tilt_min = 85.0
        self.tilt_max = 95.0
        self.sweep_speed = 1.0  # degrees shifted per ~50ms
        self.current_tilt_angle = 90.0
        self.kit = None
        
        # Telemetry State
        self.latest_msg = { "type": "scan_data", "points": [] }
        self.status_text = "Ready"
        self.active_connections: Set[WebSocket] = set()

    def get_available_ports(self):
        if not list_ports: return ["/dev/ttyUSB0", "COM4"]
        try: return sorted([p.device for p in list_ports.comports()])
        except: return ["/dev/ttyUSB0"]

manager = LidarStateManager()

async def telemetry_broadcaster():
    """Continuously broadcast the latest scan frame to all connected browsers at 15Hz"""
    while True:
        if manager.scanning and manager.active_connections and getattr(manager, 'payload', None):
            dead = set()
            for connection in list(manager.active_connections):
                try: await connection.send_text(manager.payload)
                except Exception: dead.add(connection)
            manager.active_connections -= dead
        await asyncio.sleep(0.06)

def set_status(msg):
    manager.status_text = msg
    print(f"[3D SCAN] {msg}")
    broadcast_status()

def broadcast_status():
    if not main_loop: return
    payload = json.dumps({
        "type": "status",
        "is_scanning": manager.scanning,
        "status_text": manager.status_text
    })
    for connection in list(manager.active_connections):
        asyncio.run_coroutine_threadsafe(connection.send_text(payload), main_loop)

# ---------------------------------------------
# HARDWARE LOOPS
# ---------------------------------------------

def servo_sweep_loop():
    """Background thread to continuously tilt the platform while scanning"""
    try:
        from adafruit_servokit import ServoKit
        manager.kit = ServoKit(channels=16)
        TILT_CHANNEL = 0
        PAN_CHANNEL = 1
        
        # Expand ranges based on tilt_test.py
        manager.kit.servo[PAN_CHANNEL].set_pulse_width_range(500, 2500)
        manager.kit.servo[TILT_CHANNEL].set_pulse_width_range(500, 2500)
        
        # We will deliberately NOT write any angle to the PAN servo. 
        # Leaving it limp prevents the assembly from rotating sideways into the frame constraints.
    except Exception as e:
        set_status(f"Servo I2C Error: {e}")
        return # Exit loop if we can't find I2C servos
        
    set_status("Servo interface connected...")
    
    sweep_dir = 1
    manager.current_tilt_angle = manager.tilt_min
    
    while manager.scanning:
        try:
            # Advance angle
            manager.current_tilt_angle += sweep_dir * manager.sweep_speed
            
            # Boundary checks
            if manager.current_tilt_angle >= manager.tilt_max:
                manager.current_tilt_angle = manager.tilt_max
                sweep_dir = -1
            elif manager.current_tilt_angle <= manager.tilt_min:
                manager.current_tilt_angle = manager.tilt_min
                sweep_dir = 1
                
            manager.kit.servo[TILT_CHANNEL].angle = int(manager.current_tilt_angle)
            time.sleep(0.05)
            # 50ms update delay (~20 Hz refresh rate for smooth sweep)
            time.sleep(0.05)
            
        except Exception as e:
            print(f"Servo sweep error: {e}")
            time.sleep(1)
            
    # Relax servos on shutdown so they don't stay stiff
    try:
        if manager.kit:
            manager.kit.servo[0].angle = None
            manager.kit.servo[1].angle = None
    except:
        pass


def hardware_scan_loop():
    """Main LiDAR Iterator, tags incoming points with current servo tilt"""
    
    while manager.scanning:
        try:
            set_status(f"Connecting to {manager.device_port}...")
            manager.lidar = RPLidar(manager.device_port, baudrate=BAUD, timeout=TIMEOUT)
            
            try: manager.lidar.stopMotor()
            except: pass
            try: manager.lidar.stop()
            except: pass
            time.sleep(0.5)
            
            try: manager.lidar.clean_input()
            except: pass
            
            manager.lidar.start_motor()
            time.sleep(SPINUP_S)
            set_status("Scanner ready - collecting 3D data...")
            
            it = manager.lidar.iter_scans(max_buf_meas=5000)
            
            for _ in range(WARMUP_SCANS):
                next(it) # Let exceptions bubble up to trigger the outer reconnect loop
                
            while manager.scanning:
                try:
                    scan = next(it) 
                    
                    pitch = manager.current_tilt_angle
                    
                    filtered_points = []
                    for q, ang, dist in scan:
                        if q >= manager.min_quality and manager.min_distance <= dist <= manager.max_radius:
                            filtered_points.append([float(q), float(ang % 360.0), float(dist), float(pitch)])
                    
                    if filtered_points:
                        msg = { "type": "scan_data", "points": filtered_points }
                        manager.payload = json.dumps(msg) 
                    
                except StopIteration:
                    set_status("LiDAR stopped yielding data. Rebuilding...")
                    break # Break inner loop to trigger outer reconnect
                except Exception as e:
                    set_status(f"Transient Error: {e}")
                    break # Break inner loop to trigger outer reconnect
                    
        except Exception as e:
            if manager.scanning:
                set_status(f"Hardware Error: {e}")
                time.sleep(1)
            
        finally:
            if manager.lidar:
                try: manager.lidar.stop()
                except: pass
                try: manager.lidar.stop_motor()
                except: pass
                try: manager.lidar.disconnect()
                except: pass
                manager.lidar = None
                
            if manager.scanning:
                time.sleep(1.2)
                
    set_status("Scan fully stopped.")

# ---------------------------------------------
# WEBSOCKET API
# ---------------------------------------------
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
                if "tilt_min" in cfg: manager.tilt_min = cfg["tilt_min"]
                if "tilt_max" in cfg: manager.tilt_max = cfg["tilt_max"]
                
            elif action == "start_scan":
                if not manager.scanning:
                    manager.device_port = cmd.get("port", manager.get_available_ports()[0])
                    cfg = cmd.get("config", {})
                    if "min_quality" in cfg: manager.min_quality = cfg["min_quality"]
                    if "max_radius" in cfg: manager.max_radius = cfg["max_radius"]
                    if "tilt_min" in cfg: manager.tilt_min = cfg["tilt_min"]
                    if "tilt_max" in cfg: manager.tilt_max = cfg["tilt_max"]
                    
                    manager.scanning = True
                    broadcast_status()
                    
                    # Fire both threads!
                    manager.scan_thread = threading.Thread(target=hardware_scan_loop, daemon=True)
                    manager.servo_thread = threading.Thread(target=servo_sweep_loop, daemon=True)
                    manager.scan_thread.start()
                    manager.servo_thread.start()
                    
            elif action == "stop_scan":
                manager.scanning = False
                set_status("Stopping scanner...")
                
    except WebSocketDisconnect:
        manager.active_connections.remove(websocket)

# Mount Frontend
web_dir = os.path.join(scripts_dir, "web")
app.mount("/", StaticFiles(directory=web_dir, html=True), name="web")

if __name__ == "__main__":
    print("="*50)
    print("3D LiDAR Server Active")
    print("="*50)
    uvicorn.run(app, host="0.0.0.0", port=5000, log_level="error")
