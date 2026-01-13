import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from .mpu6050 import mpu6050
import math

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # --- Parameters ---
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('frequency', 20.0)
        
        i2c_addr = self.get_parameter('i2c_address').value
        frequency = self.get_parameter('frequency').value
        self.dt = 1.0 / frequency
        
        # --- IMU Hardware Setup ---
        try:
            self.sensor = mpu6050(i2c_addr, frequency)
            self.get_logger().info(f"Connected to MPU6050 at {hex(i2c_addr)}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MPU6050: {e}")
            # Do not use exit(). Just return, node will sit idle or user can kill it.
            return

        # --- Calibration ---
        self.offsets = {'ax': 0.0, 'ay': 0.0, 'az': 0.0, 'gx': 0.0, 'gy': 0.0, 'gz': 0.0}
        self.calibrate_sensor()

        # --- ROS 2 Publishers ---
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        
        # --- Timer ---
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(f"IMU Node publishing at {frequency}Hz")

    def calibrate_sensor(self):
        self.get_logger().info("Calibrating... Keep car LEVEL and STILL.")
        samples = 50
        
        try:
            for _ in range(samples):
                a = self.sensor.get_accel_data()
                g = self.sensor.get_gyro_data()
                self.offsets['ax'] += a['x']; self.offsets['ay'] += a['y']; self.offsets['az'] += a['z']
                self.offsets['gx'] += g['x']; self.offsets['gy'] += g['y']; self.offsets['gz'] += g['z']
                # Sleep briefly to not spam I2C
                # time.sleep(0.01) # removing time import relies on timer spacing, but here we block.
                # simpler just to run tight loop for calibration
            
            for key in self.offsets: self.offsets[key] /= samples
            self.offsets['az'] -= self.sensor.GRAVITIY_MS2 # Remove gravity
            self.get_logger().info("Calibration complete.")
        except Exception as e:
            self.get_logger().warn(f"Calibration failed: {e}")

    def timer_callback(self):
        try:
            # 1. Read Sensor
            a = self.sensor.get_accel_data()
            g = self.sensor.get_gyro_data()

            # 2. Apply Offsets (Net movements)
            ax_net = a['x'] - self.offsets['ax']
            ay_net = a['y'] - self.offsets['ay']
            az_net = a['z'] - self.offsets['az'] # Gravity is included in raw data usually? 
            # Note: For Madgwick/EKF, they often expect gravity vector in accel.
            # However, if we calibrate 'az' to be 0 when flat, we are removing gravity. 
            # Standard IMU msg expects accel to include gravity (approx 9.8 up).
            # The previous code removed G_CONSTANT during calibration from the offset, meaning:
            # offset_az = average_z - 9.8.
            # az_net = measured_z - (average_z - 9.8)
            # If car is flat: measured_z ~ 9.8. offset_az ~ 0.
            # az_net ~ 9.8. This seems correct for ROS.

            gx_net = g['x'] - self.offsets['gx']
            gy_net = g['y'] - self.offsets['gy']
            gz_net = g['z'] - self.offsets['gz']

            # 3. Construct Imu Message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            # ROS 2 expects values in m/s^2 and rad/s
            imu_msg.linear_acceleration.x = ax_net
            imu_msg.linear_acceleration.y = ay_net
            imu_msg.linear_acceleration.z = az_net
            
            imu_msg.angular_velocity.x = math.radians(gx_net)
            imu_msg.angular_velocity.y = math.radians(gy_net)
            imu_msg.angular_velocity.z = math.radians(gz_net)

            # No orientation estimation here. Leaving it to madgwick/ekf.
            # Covariance (unknown)
            imu_msg.orientation_covariance[0] = -1.0

            self.imu_pub.publish(imu_msg)

        except Exception as e:
            self.get_logger().warn(f"IMU Read Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()