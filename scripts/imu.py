from mpu6050 import mpu6050
import time
import math

# Initialize Sensor
sensor = mpu6050.mpu6050(0x68)
G_CONSTANT = 9.80665

# Filter Constants
ALPHA = 0.96  # Weight for Gyro (blending factor)
DT = 0.05     # Loop interval (20Hz)

def get_calibration_offsets():
    print("Calibrating... Keep car LEVEL and STILL.")
    samples = 50
    off = {'ax': 0, 'ay': 0, 'az': 0, 'gx': 0, 'gy': 0, 'gz': 0}
    
    for _ in range(samples):
        a = sensor.get_accel_data()
        g = sensor.get_gyro_data()
        off['ax'] += a['x']; off['ay'] += a['y']; off['az'] += a['z']
        off['gx'] += g['x']; off['gy'] += g['y']; off['gz'] += g['z']
        time.sleep(0.01)
    
    # Average and remove gravity from Z offset
    for key in off: off[key] /= samples
    off['az'] -= G_CONSTANT
    return off

def monitor_sensor():
    offsets = get_calibration_offsets()
    roll = 0.0
    pitch = 0.0
    
    print("System Live. Press Ctrl+C to stop.")
    try:
        while True:
            start_loop = time.time()
            
            # 1. Get Data
            a = sensor.get_accel_data()
            g = sensor.get_gyro_data()

            # 2. Process 6-Axis (Centered at 0.0)
            ax_net = a['x'] - offsets['ax']
            ay_net = a['y'] - offsets['ay']
            az_net = a['z'] - offsets['az'] - G_CONSTANT # Vertical oscillation
            
            gx_net = g['x'] - offsets['gx']
            gy_net = g['y'] - offsets['gy']
            gz_net = g['z'] - offsets['gz'] # True turning rate

            # 3. Calculate Roll & Pitch (Degrees)
            # We use raw 'a' here because atan2 needs the gravity vector
            accel_roll = math.degrees(math.atan2(a['y'], a['z']))
            accel_pitch = math.degrees(math.atan2(-a['x'], math.sqrt(a['y']**2 + a['z']**2)))

            # Complementary Filter: combines gyro rate with accel stability
            roll = ALPHA * (roll + gx_net * DT) + (1 - ALPHA) * accel_roll
            pitch = ALPHA * (pitch + gy_net * DT) + (1 - ALPHA) * accel_pitch

            # 4. Display Dashboard
            print("\033c", end="") # Clear terminal
            print(f"{'--- RASPI-KOMA TELEMETRY ---':^40}")
            print(f"{'ORIENTATION':<15} | Roll: {roll:6.1f}°  Pitch: {pitch:6.1f}°")
            print("-" * 45)
            print(f"{'LINEAR (m/s²)':<15} | X: {ax_net:5.2f}  Y: {ay_net:5.2f}  Z: {az_net:5.2f}")
            print(f"{'GYRO (°/s)':<15} | X: {gx_net:5.2f}  Y: {gy_net:5.2f}  Z: {gz_net:5.2f}")
            print("-" * 45)
            
            # Simple Alerts
            if abs(az_net) > 4.0: print("!! VIBRATION ALERT !!")
            if abs(roll) > 25.0:  print("!! TILT WARNING !!")

            # Maintain DT
            elapsed = time.time() - start_loop
            if DT > elapsed:
                time.sleep(DT - elapsed)

    except KeyboardInterrupt:
        print("\nTelemetry Stopped.")

if __name__ == "__main__":
    monitor_sensor()