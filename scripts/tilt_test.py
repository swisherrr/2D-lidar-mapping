import time
from adafruit_servokit import ServoKit

# Initialize PCA9685
try:
    # Most Yahboom kits use 16 channels
    kit = ServoKit(channels=16)
except Exception as e:
    print(f"I2C Error: {e}. Is I2C enabled in raspi-config?")
    exit(1)

# --- CALIBRATION SECTION ---
# Yahboom servos (like MG90S) often perform best with these pulse widths:
# Default is usually 750 min and 2250 max.
# Try 500 and 2500 for a wider range.
PAN_CHANNEL = 0
TILT_CHANNEL = 1

def calibrate_servos():
    print("Applying calibration...")
    # Adjust these numbers if the movement is still restricted
    kit.servo[PAN_CHANNEL].set_pulse_width_range(500, 2500)
    kit.servo[TILT_CHANNEL].set_pulse_width_range(500, 2500)

def move_servo(channel, angle):
    print(f"Moving channel {channel} -> {angle} degrees")
    kit.servo[channel].angle = angle

def test_full_range():
    print("Testing full 0 to 180 degree range...")
    calibrate_servos()
    
    # Test Sweep
    for angle in [0, 90, 180, 90]:
        move_servo(PAN_CHANNEL, angle)
        move_servo(TILT_CHANNEL, angle)
        time.sleep(2)

if __name__ == "__main__":
    try:
        test_full_range()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as e:
        print(f"Error during execution: {e}")
        print("\nTIP: If movement is 'weak', check your 5V power source.")
