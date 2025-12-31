import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from mpu6050 import mpu6050
import math
import time

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        # --- IMU Hardware Setup ---
        try:
            self.sensor = mpu6050.mpu6050(0x68)
            self.G_CONSTANT = self.sensor.GRAVITIY_MS2
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MPU6050: {e}")
            exit()

        # --- Parameters & Constants ---
        self.alpha = 0.96
        self.dt = 0.05  # 20Hz
        self.roll = 0.0
        self.pitch = 0.0
        
        # --- Calibration ---
        self.offsets = self.get_calibration_offsets()

        # --- ROS 2 Publishers ---
        # Standard IMU message
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        # Orientation for easy visualization in Rviz
        self.pose_pub = self.create_publisher(PoseStamped, 'imu/orientation', 10)
        
        # --- Timer ---
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info("IMU Node started and publishing at 20Hz")

    def get_calibration_offsets(self):
        self.get_logger().info("Calibrating... Keep car LEVEL and STILL.")
        samples = 50
        off = {'ax': 0.0, 'ay': 0.0, 'az': 0.0, 'gx': 0.0, 'gy': 0.0, 'gz': 0.0}
        
        for _ in range(samples):
            a = self.sensor.get_accel_data()
            g = self.sensor.get_gyro_data()
            off['ax'] += a['x']; off['ay'] += a['y']; off['az'] += a['z']
            off['gx'] += g['x']; off['gy'] += g['y']; off['gz'] += g['z']
            time.sleep(0.01)
        
        for key in off: off[key] /= samples
        off['az'] -= self.G_CONSTANT
        self.get_logger().info("Calibration complete.")
        return off

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Helper to convert Euler angles to Quaternion for ROS msgs"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def timer_callback(self):
        # 1. Read Sensor
        a = self.sensor.get_accel_data()
        g = self.sensor.get_gyro_data()

        # 2. Apply Offsets (Net movements)
        ax_net = a['x'] - self.offsets['ax']
        ay_net = a['y'] - self.offsets['ay']
        az_net = a['z'] - self.offsets['az'] - self.G_CONSTANT
        
        gx_net = g['x'] - self.offsets['gx']
        gy_net = g['y'] - self.offsets['gy']
        gz_net = g['z'] - self.offsets['gz']

        # 3. Calculate Roll/Pitch (Complementary Filter)
        accel_roll = math.degrees(math.atan2(a['y'], a['z']))
        accel_pitch = math.degrees(math.atan2(-a['x'], math.sqrt(a['y']**2 + a['z']**2)))
        
        self.roll = self.alpha * (self.roll + gx_net * self.dt) + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + gy_net * self.dt) + (1 - self.alpha) * accel_pitch

        # 4. Construct Imu Message
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

        # Convert orientation to Quaternion
        q = self.euler_to_quaternion(math.radians(self.roll), math.radians(self.pitch), 0.0)
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        self.imu_pub.publish(imu_msg)

        # 5. Publish Pose (for Rviz)
        pose_msg = PoseStamped()
        pose_msg.header = imu_msg.header
        pose_msg.pose.orientation = imu_msg.orientation
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()