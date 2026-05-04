# 3D LiDAR Mapping System

> Real-time 3D point cloud scanning streamed live to any browser on your local network — built on a Raspberry Pi 5, an RPLidar A-series sensor, and a servo-driven pan/tilt platform.

**Team:** Zac Swisher · Sneha Patel · Jamil Velez · Bryton Montgomery

---

## Overview

This project turns a commodity RPLidar sensor + a pair of hobby servos into a full 3D room scanner. The Raspberry Pi runs a **FastAPI/WebSocket backend** (`ScannerWeb.py`) that orchestrates two concurrent threads:

- **`hardware_scan_loop`** — reads polar scan packets from the LiDAR over serial USB
- **`servo_sweep_loop`** — incrementally tilts the sensor through a configurable angular sweep

Every incoming point is tagged with the sensor's current tilt angle and broadcast at ~15 Hz to connected browsers. The browser renders the accumulating point cloud in an interactive **Three.js WebGL** scene with a height-based HSL heatmap.

Any device on the same LAN — phone, tablet, laptop — can view the live scan with no software installation.

---

## Hardware Components

| Component | Details |
|---|---|
| **Compute** | Raspberry Pi 5 |
| **LiDAR** | RPLidar A1 / A2 / A3 (USB serial) |
| **Servo driver** | Adafruit PCA9685 16-channel PWM (I²C `0x40`) |
| **Tilt servo** | Yahboom 25 kg or equivalent; wired to **Channel 0** |
| **Motor power** | Dedicated AA-battery pack → `V+/GND` block on PCA9685  |



---

## Repository Structure

```

| scripts/
│   ├── ScannerWeb.py       # Backend (FastAPI + WebSocket + threads)
│   ├── DataCapture.py      # Shared library: port detection, coord math, CSV capture
│   ├── ScannerGUI.py       # Offline desktop GUI (Tkinter + Matplotlib, 2D polar)
│   ├── PlotScan.py         # Offline viewer — re-renders a saved CSV scan
│   ├── requirements.txt    # Python dependencies
│   └── web/
│       ├── index.html      # Browser UI shell
│       ├── app.js          # Three.js point cloud renderer + WebSocket client
│       └── styles.css      # UI theme

```

---

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/swisherrr/2D-lidar-mapping.git
cd 2D-lidar-mapping
```

### 2. Install Python dependencies on the Raspberry Pi

```bash
pip install fastapi uvicorn websockets rplidar-roboticia adafruit-circuitpython-servokit pyserial
```

> The `requirements.txt` in `scripts/` lists all pinned versions used during development.

### 3. Enable I²C on the Raspberry Pi

```bash
sudo raspi-config
# Interface Options → I2C → Enable
```

Verify the PCA9685 is visible on the bus:

```bash
i2cdetect -y 1
# Should show 0x40
```

### 4. (If using software I²C on alternate pins)

Add to `/boot/config.txt`:

```
dtoverlay=i2c-gpio,bus=4,i2c_gpio_sda=23,i2c_gpio_scl=24
```

### 5. Physically calibrate the servo (critical — do this before first run)

See the step-by-step procedure in [DEVELOPMENT.md § 1](DEVELOPMENT.md#1-hardware-assembly--motor-limitations).  
**TL;DR:** Mount the bracket so that servo angle `90°` points the LiDAR straight up. This maps `45°` to forward pitch and `135°` to backward pitch with no frame collisions.

---

## Running the Scanner

### On the Raspberry Pi

```bash
cd ~/aLiDAR/scripts
python ScannerWeb.py
```


Open any browser on the same network and navigate to:

```
http://<Pi-IP-address>:5000
```

### Controls

| Control | Description |
|---|---|
| **Port selector** | Auto-detected serial port for the LiDAR (e.g. `/dev/ttyUSB0`) |
| **Start Scan** | Connects to the LiDAR, starts both hardware threads, begins streaming |
| **Stop Scan** | Gracefully stops scanning and relaxes the servos |
| **Reset Point Cloud** | Clears the accumulated 3D points from the browser view |
| **Min Quality** | Minimum LiDAR measurement quality threshold (0–15) |
| **Max Radius (mm)** | Maximum scan range in millimeters |
| **Tilt Min / Max (°)** | Angular sweep bounds for the tilt servo |
| **OrbitControls** | Click-drag to orbit, scroll to zoom, right-drag to pan |

---

## Configuration Reference

These parameters can be adjusted from the web UI or hardcoded in `ScannerWeb.py`:

| Parameter | Default | Description |
|---|---|---|
| `min_quality` | `10` | Discard LiDAR readings below this quality score |
| `min_distance` | `100 mm` | Ignore readings closer than this (prevents self-detection) |
| `max_radius` | `8000 mm` | Maximum scan range (8 m) |
| `tilt_min` | `85°` | Lower bound of servo tilt sweep |
| `tilt_max` | `95°` | Upper bound of servo tilt sweep |
| `sweep_speed` | `1.0°` | Degrees stepped per ~50 ms tick |
| Broadcast rate | `~15 Hz` | WebSocket push interval (`0.06 s` sleep) |
| Max points (GPU) | `1,000,000` | Pre-allocated `Float32Array` in `app.js` |

---


## Limitations

- **No authentication.** Anyone on the same WiFi network can view the live scan feed and send start/stop commands. Do not expose port 5000 externally.
- **No HTTPS/WSS.** Communication is unencrypted. Suitable for trusted LAN environments only.

