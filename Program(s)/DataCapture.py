# LiDAR Data Capture
import csv, math, time, sys, platform
from pathlib import Path
from rplidar import RPLidar
# Auto-detect using pyserial package
try:
    from serial.tools import list_ports
except Exception:
    list_ports = None  # If it fails, disables auto-detect

# Defaults for devices, change Windows dependent on Device Manager > Ports
MAC_DEVICE = "/dev/cu.usbserial-0001"
LINUX_DEVICE = "/dev/ttyUSB0"
WIN_DEVICE = "COM4"


BAUD = 115200               # Communication speed
TIMEOUT = 1                 # Seconds to timeout if no data is collecting, 1-3
OUT = "scans.csv"           # CSV filename
TARGET_SCANS = 1            # Full rotation
MIN_QUALITY = 0             # Filter readings
SPINUP_S = 1.5              # Seconds to reach full RPM
WARMUP_SCANS = 3            # Discard first scans
MIN_COVERAGE_DEG = 320.0    # At least these many have to be able to be reached to save to CSV
MAX_ATTEMPTS = 20           # Prevents inf loops

# Set to None to auto-detect; set to a string to force (e.g., "COM4" or "/dev/cu.usbserial-0002")
DEVICE = None

# Clues to check which port belongs to the LiDAR
USB_HINTS = ("CP210", "Silicon Labs", "SLAB", "usbserial", "CH340", "USB-to-UART", "USB2.0-Serial")

def CheckDevice():
    sysname = platform.system().lower()
    if "windows" in sysname: return WIN_DEVICE
    if "darwin" in sysname:  return MAC_DEVICE
    return LINUX_DEVICE

def DetectPort():
    if list_ports is None:    # Auto-detect fails, check system
        return CheckDevice()
    candidates = list(list_ports.comports())     # Check all serial devices, add them to list
    if not candidates:        # Check again if it fails
        return CheckDevice()
    # Prefer “usb”/“CP210x” style ports
    scored = []               # Array for determining which is most likely the LiDAR
    for p in candidates:      # Builds a blob containing description, hardware ID
        desc = f"{p.description or ''} {p.hwid or ''} {p.device or ''}".lower()
        score = 0
        for hint in USB_HINTS:
            if hint.lower() in desc:
                score += 1
        # Heuristic: prefer 'COMx' on Windows, '/dev/cu.' on macOS
        if platform.system().lower().startswith("win") and str(p.device).upper().startswith("COM"):
            score += 1
        if platform.system().lower() == "darwin" and ("/dev/cu." in p.device.lower()):
            score += 1
        scored.append((score, p.device))
    scored.sort(reverse=True)
    best = scored[0][1] if scored else CheckDevice()
    return best

# Converts from polar to Cartesian in millimeters
def xy(angle_deg, dist_mm):
    th = math.radians(angle_deg % 360.0)                    # Angle wraps between 0°-360°
    return dist_mm*math.cos(th), dist_mm*math.sin(th)       # x = r * cos(theta); y = r * sin (theta)

# Timestamp function
def time_ms():
    t = time.time()
    # Example return time: 2025-10-31T14:14:14.350Z
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(t)) + f".{int((t%1)*1e3):03d}Z"

# Sorts angle captures and checks for gaps
def angular_span_deg(angles):
    if not angles:
        return 0.0
    a = sorted((ang % 360.0) for ang in angles)
    # Compute angular gaps between each consecutive angle (including wrap-around)
    gaps = []
    for i in range(len(a)):
        current = a[i]
        next_angle = a[(i + 1) % len(a)]        # wraps to start when at last element
        diff = (next_angle - current) % 360.0   # ensures positive result
        gaps.append(diff)

    # Coverage = 360° minus the largest uncovered gap
    return 360.0 - max(gaps)

def main():
    dev = DEVICE or DetectPort()
    print(f"[info] Using device: {dev}")

    out = Path(OUT); out.parent.mkdir(parents=True, exist_ok=True)
    lidar = None
    try:
        lidar = RPLidar(dev, baudrate=BAUD, timeout=TIMEOUT)

        # Clean state
        for fn in (lidar.stop, lidar.clear_input, lidar.stop_motor):
            try: fn()
            except Exception: pass

        # Spin up
        lidar.start_motor()
        time.sleep(SPINUP_S)

        it = lidar.iter_scans(max_buf_meas=5000)

        # Discard warm-up sweeps
        for _ in range(WARMUP_SCANS):
            try: next(it)
            except Exception:
                lidar.stop(); lidar.clear_input(); time.sleep(0.2)
                it = lidar.iter_scans(max_buf_meas=5000)

        saved = 0
        with out.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp","scan_idx","meas_idx",
                        "angle_deg","distance_mm","quality","x_mm","y_mm"])

            while saved < TARGET_SCANS:
                attempts, best = 0, None  # (coverage, filtered)

                while attempts < MAX_ATTEMPTS:
                    attempts += 1
                    scan = next(it)       # one sweep
                    filtered = [(q, (ang % 360.0), dist) for (q, ang, dist) in scan if q >= MIN_QUALITY]
                    filtered.sort(key=lambda t: t[1])
                    cov = angular_span_deg([ang for _, ang, _ in filtered])

                    if best is None or cov > best[0]:
                        best = (cov, filtered)

                    if cov >= MIN_COVERAGE_DEG:
                        break

                cov, filtered = best if best else (0.0, [])
                ts = time_ms()
                for meas_idx, (q, ang, dist) in enumerate(filtered):
                    x, y = xy(ang, dist)
                    w.writerow([ts, saved, meas_idx, f"{ang:.3f}", f"{dist:.1f}", int(q), f"{x:.1f}", f"{y:.1f}"])
                print(f"[info] Saved sweep #{saved} coverage={cov:.1f}° attempts={attempts}")
                saved += 1

        print(f"[ok] Saved {saved} scan(s) to {out.resolve()}")

#   This doesn't work currently. Trying to make it stop when connected to a device but here just in case.
    finally:
        if lidar:
            try: lidar.stop()
            except Exception: pass
            try: lidar.stop_motor()
            except Exception: pass
            time.sleep(0.3)
            # Best-effort force DTR low if driver exposes it (works on many Windows drivers too)
            try:
                lidar._serial.setDTR(False)
            except Exception:
                pass
            try: lidar.disconnect()
            except Exception: pass

if __name__ == "__main__":
    main()
