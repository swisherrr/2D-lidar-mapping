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
| **Motor power** | Dedicated AA-battery pack → `V+/GND` block on PCA9685 |
| **Motor controller** | Arduino Mega 2560 |
| **Bluetooth module** | HC-05 (connected to Arduino `Serial1`, 9600 baud) |
| **H-bridge driver** | L298N dual H-bridge; ENA/ENB on pins 5/6, direction on pins 50–53 |
| **Drive motors** | Two DC gear motors wired to L298N output channels A & B |



---

## Repository Structure

```

Arduino/
│   └── Bluetooth_Code.ino  # Arduino sketch: HC-05 Bluetooth → L298N motor control
|
scripts/
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

## Arduino — Bluetooth Motor Controller

`Arduino/Bluetooth_Code.ino` turns any Arduino Mega (or compatible board) into a wireless motor controller for a two-wheel differential drive base using an **HC-05 Bluetooth module** and an **L298N H-bridge driver**.

### Pin Mapping

| Signal | Arduino Pin |
|---|---|
| Motor A PWM (ENA) | 5 |
| Motor A dir IN1 | 50 |
| Motor A dir IN2 | 51 |
| Motor B PWM (ENB) | 6 |
| Motor B dir IN3 | 52 |
| Motor B dir IN4 | 53 |
| HC-05 TX → RX | Serial1 (pins 18/19) |

### Command Protocol

Send single ASCII characters over Bluetooth (9600 baud) to control the robot:

| Character | Action |
|---|---|
| `w` | Forward |
| `s` | Reverse |
| `a` | Pivot left |
| `d` | Pivot right |
| `x` | Stop |
| `q` | Emergency stop (locks until reset) |
| `1`–`9`, `0` | Set speed to 10%–90%, 100% |

### Uploading the Sketch

1. Open `Arduino/Bluetooth_Code.ino` in the Arduino IDE.
2. Select **Board → Arduino Mega 2560** and the correct COM port.
3. Upload. Open the Serial Monitor at **115,200 baud** to verify the `Bluetooth Ready` message.
4. Pair your phone or laptop with the HC-05 module (default PIN: `1234`) and use any Bluetooth terminal app to send the commands above.

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

